'use strict';
// test_load_lump_binary.js — Integration tests for ChurchSimulator.loadLumpBinary()
// Task #1266
//
// Exercises the full "word array → loadLumpBinary → NS-slot-3 wiring → step()"
// pipeline without any DOM dependencies.
//
// Run:  node simulator/test_load_lump_binary.js
//
// Coverage (synthetic LUMP tests):
//   LLB-01  Synthetic LUMP (cw=10, cc=2): NS slot 3 word0/word1 and CR14 wired
//            correctly after loadLumpBinary.
//   LLB-02  step() fetches the first code word at 0x0401 without fault; a HALT
//            word (0) produces a clean halt rather than a fault entry.
//   LLB-03  Synthetic NS slot 3 parameters (cw=17, cc=1) round-trip through
//            loadLumpBinary: NS slot 3 word1 encodes limit=17, clistCount=1.
//            (These are the same header parameters used by the real LED flash
//            lump — token 00000300 — which occupies NS slot 3 at runtime.)
//   LLB-04  loadLumpBinary with cc=0: NS slot 3 clistCount=0, CR6 zeroed.
//   LLB-05  Memory layout: code word at 0x0401 survives the load unchanged.
//   LLB-06  PC is reset to 0 by loadLumpBinary.
//   LLB-07  CR14.word1 is updated to EXTENDED_BASE (0x0400) after the call.
//   LLB-08  makeVersionSeals is re-run for 0x0400 so NS word2 seal is consistent.
//
// Coverage (real fixture + app-path tests):
// Fixture: server/lumps/00000300.lump — "LED flash" abstraction (per sidecar
// JSON), loaded into NS slot 3 (BOOT_ABSTR_NS_SLOT) at runtime.  cw=17, cc=1.
// Note: the original task description referenced token "00000003"; the actual
// Boot.Abstr slot fixture on disk is "00000300" (canonical per sidecar JSON).
//
//   LLB-RBA Load the real LED flash binary (00000300.lump, big-endian as served
//            by Flask /api/lump/00000300/words).  Verifies the file decodes as
//            cw=17 cc=1 and that loadLumpBinary installs it correctly.
//   LLB-APP Mimic the _loadLumpBinaryIntoSim() app-path (fetch → rawWords →
//            loadLumpBinary → parseLumpHeader → wordCount).  Uses the fixture
//            file in place of the network fetch and asserts the "header row"
//            values (wordCount=17, hdr.cc=1) that the browser console line
//            displays after a successful load.
//   LLB-STP step() on real LED flash (NS slot 3) code: physicalPC = 0x0401
//            and _instrHistory contains rawWords[1] after step(), proving the
//            fetch phase reached the right address with the right word.

const fs   = require('fs');
const path = require('path');
const ChurchSimulator = require('./simulator.js');

let pass = 0;
let fail = 0;

function check(label, cond, detail) {
    if (cond) {
        console.log('PASS ' + label);
        pass++;
    } else {
        console.log('FAIL ' + label + (detail ? ' — ' + detail : ''));
        fail++;
    }
}

// ── Constants (must match simulator.js) ───────────────────────────────────────
const EXTENDED_BASE      = 0x0400;
const BOOT_ABSTR_NS_SLOT = 3;

// ── Helper: build a valid LUMP header word ────────────────────────────────────
// magic=0x1F fixed; n_minus_6 defaults to 0 (lumpSize=64); typ defaults to 0.
function makeHdr(cw, cc, n_minus_6 = 0, typ = 0) {
    return (
        (0x1F          << 27) |
        ((n_minus_6 & 0xF)  << 23) |
        ((cw  & 0x1FFF)     << 10) |
        ((typ & 0x3)        <<  8) |
        ( cc  & 0xFF)
    ) >>> 0;
}

// ── Helper: build a LUMP word array and load it into a fresh simulator ─────────
//   cw       : code words advertised in the header
//   cc       : c-list words advertised in the header
//   codeWord : the word placed at position 1 (the first code word)
// Returns { sim, nsBase, hdrWord } ready for assertions.
function setupAndLoad({ cw, cc, codeWord = 0, n_minus_6 = 0 }) {
    const sim = new ChurchSimulator();
    sim.bootComplete = true;

    const lumpSize = 1 << (n_minus_6 + 6);   // 64 for n_minus_6=0
    const hdrWord  = makeHdr(cw, cc, n_minus_6);

    // Build the full LUMP word array (lumpSize words):
    //   [0]          = header
    //   [1]          = first code word (codeWord)
    //   [2..lumpSize-cc-1] = zeros (padding)
    //   [lumpSize-cc..lumpSize-1] = c-list (zeros by default)
    const words = new Array(lumpSize).fill(0);
    words[0] = hdrWord;
    words[1] = codeWord;

    // Pre-seed NS slot 3 with an initial entry so loadLumpBinary has existing
    // flags (b, g, gtType) and a gt_seq to preserve.
    const GT_SEQ   = 5;
    const INIT_BASE = 0x80;
    const INIT_CW   = 64;
    const nsBase = sim.NS_TABLE_BASE + BOOT_ABSTR_NS_SLOT * sim.NS_ENTRY_WORDS;
    sim.memory[nsBase + 0] = INIT_BASE;
    sim.memory[nsBase + 1] = sim.packNSWord1(INIT_CW, 0, 0, 0, 0);
    sim.memory[nsBase + 2] = sim.makeVersionSeals(GT_SEQ, INIT_BASE, INIT_CW);

    // Pre-seed CR14 with a valid GT so loadLumpBinary can update word1/2/3.
    // (loadLumpBinary only updates word1/2/3; word0 must be non-null beforehand.)
    sim.cr[14] = {
        word0: sim.createGT(GT_SEQ, BOOT_ABSTR_NS_SLOT, {R:1,W:0,X:1,L:0,S:0,E:0}, 1),
        word1: INIT_BASE,
        word2: sim.memory[nsBase + 1],
        word3: sim.memory[nsBase + 2],
        m: 0,
    };

    // Also pre-seed CR12 to prevent internal crashes in call-stack helpers.
    sim.cr[12] = { word0: 0, word1: 0, word2: 0, word3: 0, m: 0 };

    const ok = sim.loadLumpBinary(words);
    return { sim, nsBase, hdrWord, ok, GT_SEQ };
}

// ── Helper: read a .lump binary as big-endian uint32 words ────────────────────
// Matches the format served by Flask's /api/lump/<token>/words endpoint:
//   words = struct.unpack(f'>{num_words}I', data[:num_words * 4])
function readLumpFile(filePath) {
    const buf      = fs.readFileSync(filePath);
    const numWords = buf.length >> 2;
    const words    = [];
    for (let i = 0; i < numWords; i++) {
        words.push(buf.readUInt32BE(i * 4));
    }
    return words;
}

// ── Helper: seed a fresh sim for a binary-lump load and call loadLumpBinary ───
// Mimics the preconditions that the browser sets up before calling loadLumpBinary:
//   instantBoot() → bootComplete = true, NS table populated, CR14/CR12 seeded.
// Here we call reset() (already done in constructor) and set the minimum needed:
//   bootComplete, CR14 (non-null), CR12 (non-null).
function setupSimForBinary() {
    const sim = new ChurchSimulator();
    sim.bootComplete = true;

    const GT_SEQ    = 1;
    const INIT_BASE = 0x80;
    const INIT_CW   = 64;
    const nsBase = sim.NS_TABLE_BASE + BOOT_ABSTR_NS_SLOT * sim.NS_ENTRY_WORDS;
    sim.memory[nsBase + 0] = INIT_BASE;
    sim.memory[nsBase + 1] = sim.packNSWord1(INIT_CW, 0, 0, 0, 0);
    sim.memory[nsBase + 2] = sim.makeVersionSeals(GT_SEQ, INIT_BASE, INIT_CW);
    sim.cr[14] = {
        word0: sim.createGT(GT_SEQ, BOOT_ABSTR_NS_SLOT, {R:1,W:0,X:1,L:0,S:0,E:0}, 1),
        word1: INIT_BASE,
        word2: sim.memory[nsBase + 1],
        word3: sim.memory[nsBase + 2],
        m: 0,
    };
    sim.cr[12] = { word0: 0, word1: 0, word2: 0, word3: 0, m: 0 };
    return { sim, nsBase, GT_SEQ };
}

// ── LLB-01: NS slot 3 wiring after loadLumpBinary (cw=10, cc=2) ───────────────
console.log('\n--- LLB-01: NS slot 3 word0/word1 and CR14 wired correctly ---');
{
    const { sim, nsBase, ok } = setupAndLoad({ cw: 10, cc: 2 });

    check('LLB-01a: loadLumpBinary returns true', ok === true);

    // NS slot 3 word0 must point to EXTENDED_BASE (0x0400)
    const nsW0 = sim.memory[nsBase + 0];
    check('LLB-01b: NS[3].word0 = 0x0400',
        nsW0 === EXTENDED_BASE,
        `got 0x${nsW0.toString(16)}`);

    // NS slot 3 word1: limit field must encode cw=10, clistCount must encode cc=2
    const nsW1 = sim.memory[nsBase + 1];
    const parsed1 = sim.parseNSWord1(nsW1);
    check('LLB-01c: NS[3].word1 limit = cw (10)',
        parsed1.limit === 10,
        `got limit=${parsed1.limit}`);
    check('LLB-01d: NS[3].word1 clistCount = cc (2)',
        parsed1.clistCount === 2,
        `got clistCount=${parsed1.clistCount}`);

    // CR14 must mirror NS slot 3 word1/word2, and word1 must be EXTENDED_BASE
    const cr14 = sim.cr[14];
    check('LLB-01e: CR14.word1 = EXTENDED_BASE (0x0400)',
        cr14.word1 === EXTENDED_BASE,
        `got 0x${cr14.word1.toString(16)}`);
    check('LLB-01f: CR14.word2 = NS[3].word1',
        cr14.word2 === sim.memory[nsBase + 1],
        `cr14.word2=0x${cr14.word2.toString(16)} ns=0x${sim.memory[nsBase+1].toString(16)}`);
    check('LLB-01g: CR14.word3 = NS[3].word2',
        cr14.word3 === sim.memory[nsBase + 2],
        `cr14.word3=0x${cr14.word3.toString(16)} ns=0x${sim.memory[nsBase+2].toString(16)}`);
}

// ── LLB-02: step() fetches first code word at 0x0401 without fault ────────────
// A zero code word (HALT instruction) produces a clean halt — no fault entry.
console.log('\n--- LLB-02: step() fetches first code word without fault ---');
{
    const SENTINEL = 0;   // word=0 → HALT (clean, no fault)
    const { sim, nsBase, GT_SEQ } = setupAndLoad({ cw: 4, cc: 1, codeWord: SENTINEL });

    // Rebuild CR14.word0 with the gt_seq that loadLumpBinary preserved in NS[3].word2
    const gtSeqAfter = (sim.memory[nsBase + 2] >>> 25) & 0x7F;
    sim.cr[14] = {
        word0: sim.createGT(gtSeqAfter, BOOT_ABSTR_NS_SLOT, {R:1,W:0,X:1,L:0,S:0,E:0}, 1),
        word1: sim.cr[14].word1,
        word2: sim.cr[14].word2,
        word3: sim.cr[14].word3,
        m: 0,
    };

    // Verify _fetchInstruction() agrees with the expected physical address
    const fetch = sim._fetchInstruction();
    check('LLB-02a: _fetchInstruction returns ok=true', fetch.ok === true, fetch.message);
    const expectedAddr = EXTENDED_BASE + 1 + sim.pc;   // 0x0400 + 1 + 0 = 0x0401
    check('LLB-02b: fetch.addr = EXTENDED_BASE+1+PC = 0x0401',
        fetch.ok && fetch.addr === expectedAddr,
        `got 0x${fetch.ok ? fetch.addr.toString(16) : 'N/A'}`);
    check('LLB-02c: fetch.word = SENTINEL (0)',
        fetch.ok && fetch.word === SENTINEL,
        `got 0x${fetch.ok ? fetch.word.toString(16) : 'N/A'}`);

    // Now call step(): should HALT cleanly (no fault entry)
    const result = sim.step();
    check('LLB-02d: step() returns a result (not null)',
        result !== null,
        'step() returned null (halted before calling?)');
    check('LLB-02e: faultLog is empty after step() — no fault occurred',
        sim.faultLog.length === 0,
        `faultLog has ${sim.faultLog.length} entry/entries`);
    check('LLB-02f: sim.halted = true after HALT word',
        sim.halted === true,
        'sim.halted is false');
    check('LLB-02g: step() result describes HALT (not a fault)',
        result !== null && typeof result.desc === 'string' && result.desc.includes('HALT'),
        `desc: "${result ? result.desc : ''}"` );
}

// ── LLB-03: NS slot 3 parameters (cw=17, cc=1) round-trip ────────────────────
// Synthetic test: constructs a LUMP with the header parameters that match the
// real LED flash abstraction (token 00000300, NS slot 3): cw=17, cc=1.
console.log('\n--- LLB-03: Synthetic NS slot 3 cw=17 cc=1 round-trip (LED flash params) ---');
{
    const { sim, nsBase, ok } = setupAndLoad({ cw: 17, cc: 1 });

    check('LLB-03a: loadLumpBinary returns true', ok === true);

    const nsW1   = sim.memory[nsBase + 1];
    const parsed = sim.parseNSWord1(nsW1);
    check('LLB-03b: NS[3].word1 limit = 17 (cw matching LED flash / NS slot 3)',
        parsed.limit === 17,
        `got limit=${parsed.limit}`);
    check('LLB-03c: NS[3].word1 clistCount = 1 (cc matching LED flash / NS slot 3)',
        parsed.clistCount === 1,
        `got clistCount=${parsed.clistCount}`);

    // Verify the LUMP header in memory at EXTENDED_BASE decodes identically
    const hdrInMem  = sim.memory[EXTENDED_BASE] >>> 0;
    const hdrParsed = sim.parseLumpHeader(hdrInMem);
    check('LLB-03d: header at 0x0400 is valid (magic=0x1F)',
        hdrParsed.valid,
        `magic=0x${hdrParsed.magic.toString(16)}`);
    check('LLB-03e: header.cw = 17',
        hdrParsed.cw === 17,
        `got cw=${hdrParsed.cw}`);
    check('LLB-03f: header.cc = 1',
        hdrParsed.cc === 1,
        `got cc=${hdrParsed.cc}`);
}

// ── LLB-04: cc=0 — CR6 zeroed, clistCount=0 ──────────────────────────────────
console.log('\n--- LLB-04: cc=0 leaves CR6 zeroed ---');
{
    const { sim, nsBase, ok } = setupAndLoad({ cw: 8, cc: 0 });

    check('LLB-04a: loadLumpBinary returns true', ok === true);

    const parsed = sim.parseNSWord1(sim.memory[nsBase + 1]);
    check('LLB-04b: NS[3].word1 clistCount = 0',
        parsed.clistCount === 0,
        `got clistCount=${parsed.clistCount}`);

    const cr6 = sim.cr[6];
    const cr6IsZero = cr6 &&
        cr6.word0 === 0 && cr6.word1 === 0 &&
        cr6.word2 === 0 && cr6.word3 === 0;
    check('LLB-04c: CR6 is zeroed when cc=0', cr6IsZero === true,
        cr6 ? `word0=${cr6.word0} word1=${cr6.word1}` : 'cr6 is null');
}

// ── LLB-05: Memory layout — code word at 0x0401 survives the load ─────────────
console.log('\n--- LLB-05: Code word at 0x0401 survives load unchanged ---');
{
    const CODE_WORD = 0xDEADBEEF >>> 0;
    const { sim, ok } = setupAndLoad({ cw: 5, cc: 1, codeWord: CODE_WORD });

    check('LLB-05a: loadLumpBinary returns true', ok === true);
    check('LLB-05b: memory[0x0401] = CODE_WORD after load',
        (sim.memory[EXTENDED_BASE + 1] >>> 0) === CODE_WORD,
        `got 0x${(sim.memory[EXTENDED_BASE + 1] >>> 0).toString(16)}`);
}

// ── LLB-06: PC is reset to 0 by loadLumpBinary ───────────────────────────────
console.log('\n--- LLB-06: PC is reset to 0 after loadLumpBinary ---');
{
    const { sim } = setupAndLoad({ cw: 4, cc: 0 });
    check('LLB-06: sim.pc = 0 after loadLumpBinary',
        sim.pc === 0,
        `got pc=${sim.pc}`);
}

// ── LLB-07: CR14.word1 updated to EXTENDED_BASE ───────────────────────────────
console.log('\n--- LLB-07: CR14.word1 = EXTENDED_BASE (0x0400) ---');
{
    const { sim } = setupAndLoad({ cw: 6, cc: 1 });
    check('LLB-07: CR14.word1 = 0x0400',
        sim.cr[14].word1 === EXTENDED_BASE,
        `got 0x${sim.cr[14].word1.toString(16)}`);
}

// ── LLB-08: NS word2 seal is consistent with new base and cw ──────────────────
// makeVersionSeals(gt_seq, 0x0400, cw) should equal what is stored in NS[3].word2.
console.log('\n--- LLB-08: NS[3].word2 seal consistent with EXTENDED_BASE and cw ---');
{
    const CW = 12;
    const { sim, nsBase, GT_SEQ } = setupAndLoad({ cw: CW, cc: 1 });

    const storedWord2  = sim.memory[nsBase + 2];
    const gtSeqStored  = (storedWord2 >>> 25) & 0x7F;
    // Recompute seal from scratch: should match storedWord2
    const expected     = sim.makeVersionSeals(gtSeqStored, EXTENDED_BASE, CW);
    check('LLB-08: NS[3].word2 seal matches makeVersionSeals(gt_seq, 0x0400, cw)',
        storedWord2 === expected,
        `stored=0x${storedWord2.toString(16)} expected=0x${expected.toString(16)}`);
}

// ── LLB-RBA: Real LED flash binary (00000300.lump, NS slot 3) ────────────────
// Fixture: server/lumps/00000300.lump — canonical abstraction name "LED flash"
// per sidecar JSON (server/lumps/00000300.json), loaded into NS slot 3
// (BOOT_ABSTR_NS_SLOT) at runtime.  cw=17, cc=1.
// Note: the original task description referenced token "00000003"; the actual
// fixture file on disk is "00000300" (confirmed by sidecar JSON "token" field).
// Reads the file as big-endian uint32 words — the same format served by the
// Flask /api/lump/00000300/words endpoint.
console.log('\n--- LLB-RBA: Real LED flash binary (00000300.lump, token 00000300, NS slot 3) ---');
{
    const lumpPath = path.join(__dirname, '..', 'server', 'lumps', '00000300.lump');
    const lumpExists = fs.existsSync(lumpPath);
    check('LLB-RBA-0: 00000300.lump fixture file exists on disk', lumpExists,
        lumpPath);

    if (lumpExists) {
        // Read as big-endian uint32 — matches Flask: struct.unpack(f'>{n}I', data)
        const rawWords = readLumpFile(lumpPath);

        check('LLB-RBA-1: rawWords has 64 entries (256-byte file)',
            rawWords.length === 64,
            `got ${rawWords.length}`);

        // Parse the header word from the fixture — must decode as cw=17, cc=1
        const sim = new ChurchSimulator();
        const hdr0 = sim.parseLumpHeader(rawWords[0] >>> 0);
        check('LLB-RBA-2: fixture header magic = 0x1F (valid LUMP)',
            hdr0.valid,
            `magic=0x${hdr0.magic.toString(16)} word[0]=0x${(rawWords[0]>>>0).toString(16)}`);
        check('LLB-RBA-3: fixture header cw = 17 (LED flash, NS slot 3)',
            hdr0.cw === 17,
            `got cw=${hdr0.cw}`);
        check('LLB-RBA-4: fixture header cc = 1 (LED flash, NS slot 3)',
            hdr0.cc === 1,
            `got cc=${hdr0.cc}`);
        check('LLB-RBA-5: fixture header lumpSize = 64',
            hdr0.lumpSize === 64,
            `got lumpSize=${hdr0.lumpSize}`);

        // Now load the real binary into a fresh simulator
        const { sim: sim2, nsBase } = setupSimForBinary();
        const loaded = sim2.loadLumpBinary(rawWords);
        check('LLB-RBA-6: loadLumpBinary returns true for real fixture',
            loaded === true);
        check('LLB-RBA-7: NS[3].word0 = EXTENDED_BASE (0x0400) after real load',
            sim2.memory[nsBase + 0] === EXTENDED_BASE,
            `got 0x${sim2.memory[nsBase+0].toString(16)}`);

        const p = sim2.parseNSWord1(sim2.memory[nsBase + 1]);
        check('LLB-RBA-8: NS[3].word1 limit = 17 (cw from real fixture)',
            p.limit === 17,
            `got limit=${p.limit}`);
        check('LLB-RBA-9: NS[3].word1 clistCount = 1 (cc from real fixture)',
            p.clistCount === 1,
            `got clistCount=${p.clistCount}`);
        check('LLB-RBA-10: CR14.word1 = EXTENDED_BASE after real load',
            sim2.cr[14].word1 === EXTENDED_BASE,
            `got 0x${sim2.cr[14].word1.toString(16)}`);

        // LLB-RBA-11: Sidecar JSON metadata — prevents future token/name drift.
        // The sidecar file must identify the abstraction as "LED flash" on NS slot 3.
        const sidecarPath = path.join(__dirname, '..', 'server', 'lumps', '00000300.json');
        if (fs.existsSync(sidecarPath)) {
            const sidecar = JSON.parse(fs.readFileSync(sidecarPath, 'utf8'));
            check('LLB-RBA-11: sidecar ns_slot = 3 (canonical NS slot for this lump)',
                sidecar.ns_slot === 3,
                `got ns_slot=${sidecar.ns_slot}`);
        } else {
            console.log('SKIP LLB-RBA-11 (sidecar 00000300.json not found)');
        }
    }
}

// ── LLB-APP: Mimic _loadLumpBinaryIntoSim app path with real fixture ──────────
// The browser function _loadLumpBinaryIntoSim (simulator/app-lumps.js ~L3439)
// does:
//   1. fetch(`/api/lump/${token}/words`) → data.words (big-endian uint32 array)
//   2. sim.loadLumpBinary(rawWords)
//   3. const hdr = sim.parseLumpHeader(rawWords[0] >>> 0)
//   4. const wordCount = (hdr && hdr.valid) ? hdr.cw : rawWords.length
//   5. Displays: `Loaded LUMP "${name}" — cw=${wordCount} cc=${hdr.cc}`
// Here we substitute the network fetch with a file read (same byte format) and
// verify the "header row" values that the browser console would display.
console.log('\n--- LLB-APP: Mimic _loadLumpBinaryIntoSim app path with real fixture ---');
{
    const lumpPath = path.join(__dirname, '..', 'server', 'lumps', '00000300.lump');
    if (fs.existsSync(lumpPath)) {
        // Step 1: Read words from disk (replaces fetch → data.words)
        const rawWords = readLumpFile(lumpPath);
        check('LLB-APP-1: rawWords non-empty (simulates non-empty fetch response)',
            rawWords.length > 0,
            `got ${rawWords.length} words`);

        // Step 2: Load into sim
        const { sim, nsBase } = setupSimForBinary();
        const loaded = sim.loadLumpBinary(rawWords);
        check('LLB-APP-2: sim.loadLumpBinary returns true (app-path step 2)',
            loaded === true);

        // Step 3+4: Parse header and compute wordCount — mirrors app-lumps.js logic
        const hdr       = rawWords.length ? sim.parseLumpHeader(rawWords[0] >>> 0) : null;
        const wordCount = (hdr && hdr.valid) ? hdr.cw : rawWords.length;

        // Step 5 assertions: verify "header row" values shown in the browser console
        check('LLB-APP-3: hdr.valid (header row is well-formed)',
            hdr !== null && hdr.valid,
            hdr ? `magic=0x${hdr.magic.toString(16)}` : 'hdr is null');
        check('LLB-APP-4: wordCount = 17 (cw shown in browser console line)',
            wordCount === 17,
            `got wordCount=${wordCount}`);
        check('LLB-APP-5: hdr.cc = 1 (cc shown in browser console line)',
            hdr !== null && hdr.cc === 1,
            hdr ? `got cc=${hdr.cc}` : 'hdr is null');

        // Verify simulator state matches the header-row values
        const p = sim.parseNSWord1(sim.memory[nsBase + 1]);
        check('LLB-APP-6: NS[3] limit agrees with header-row wordCount (17)',
            p.limit === wordCount,
            `ns limit=${p.limit} wordCount=${wordCount}`);
        check('LLB-APP-7: NS[3] clistCount agrees with header-row cc (1)',
            p.clistCount === (hdr ? hdr.cc : -1),
            `ns clistCount=${p.clistCount}`);
    } else {
        console.log('SKIP LLB-APP-* (fixture file not found)');
    }
}

// ── LLB-STP: step() on real LED flash (NS slot 3) code word ──────────────────
// Calls step() once with the real LED flash lump (token 00000300) loaded into
// NS slot 3 and verifies that:
//   (a) the fetch phase succeeded (physicalPC = 0x0401, instrHistory populated),
//   (b) the right raw word was fetched (rawWords[1]).
// We do NOT call _fetchInstruction() separately first — mLoad has side-effects
// that would cause the second call (inside step()) to fail.  Instead we let
// step() do the single fetch and inspect sim state afterwards.
// The instruction itself may fault in this minimal harness because the NS table
// is not fully populated; that is an execution fault, not a fetch fault, so we
// only assert on the fetch evidence in _instrHistory.
console.log('\n--- LLB-STP: step() on real LED flash (NS slot 3, token 00000300) code word ---');
{
    const lumpPath = path.join(__dirname, '..', 'server', 'lumps', '00000300.lump');
    if (fs.existsSync(lumpPath)) {
        const rawWords = readLumpFile(lumpPath);
        const { sim, nsBase } = setupSimForBinary();
        sim.loadLumpBinary(rawWords);

        // Rebuild CR14.word0 with an RX GT matching the preserved gt_seq
        const gtSeqAfter = (sim.memory[nsBase + 2] >>> 25) & 0x7F;
        sim.cr[14] = {
            word0: sim.createGT(gtSeqAfter, BOOT_ABSTR_NS_SLOT, {R:1,W:0,X:1,L:0,S:0,E:0}, 1),
            word1: sim.cr[14].word1,
            word2: sim.cr[14].word2,
            word3: sim.cr[14].word3,
            m: 0,
        };

        // Single step() call — let it do the fetch internally
        sim.step();

        // The fetch succeeded if physicalPC was set to 0x0401 by _fetchInstruction
        check('LLB-STP-1: sim.physicalPC = 0x0401 after step() (fetch reached the right address)',
            sim.physicalPC === EXTENDED_BASE + 1,
            `got physicalPC=0x${sim.physicalPC.toString(16)}`);

        // _instrHistory is populated on a successful fetch+decode (step() pushes
        // before dispatching the instruction)
        const hist = sim._instrHistory;
        check('LLB-STP-2: _instrHistory has an entry (fetch + decode succeeded)',
            hist.length > 0,
            `_instrHistory is empty — step() returned before decode`);

        if (hist.length > 0) {
            const entry = hist[hist.length - 1];
            check('LLB-STP-3: instrHistory physicalPC = 0x0401',
                entry.physicalPC === EXTENDED_BASE + 1,
                `got 0x${entry.physicalPC.toString(16)}`);

            const expectedWord1 = rawWords[1] >>> 0;
            check('LLB-STP-4: instrHistory raw word = rawWords[1] (actual first code word)',
                (entry.raw >>> 0) === expectedWord1,
                `got 0x${(entry.raw>>>0).toString(16)} expected 0x${expectedWord1.toString(16)}`);
        }
    } else {
        console.log('SKIP LLB-STP-* (fixture file not found)');
    }
}

// ── Summary ───────────────────────────────────────────────────────────────────
console.log('\n══════════════════════════════════════');
console.log(`Results: ${pass} passed, ${fail} failed`);
if (fail > 0) {
    console.error('SOME TESTS FAILED');
    process.exit(1);
} else {
    console.log('ALL TESTS PASSED');
}
