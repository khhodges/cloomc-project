'use strict';
// Headless harness used by tests/gates/test_outform_call_lazy.py.
//
// Verifies the Mode 2 (Outform) lazy-load path in _execCall:
//
//   1. initLazyManifest writes gtType=2 (Outform) into NS entry word1 for
//      every cold-priority slot.
//
//   2. CALL on a CR holding an Outform GT (type=0b10) triggers the lazy
//      loader (Mode 2), installs the lump, promotes the CR's GT from
//      Outform→Inform, and falls through to the normal Inform CALL path.
//
//   3. lazyLoad flips the NS entry's gtType from 2→1 (Inform) after a
//      successful install, so subsequent GT derivations from the NS entry
//      carry type=1.
//
// Exits with code 0 on success, 1 on failure (errors written to stderr).

global.window = { bootConfig: {} };

const ChurchSimulator = require('../../simulator/simulator.js');

// boot_uploads.js is a browser global script (no module.exports).
// Load it by evaluating it in the current context so BOOT_UPLOADS is visible.
const fs   = require('fs');
const path = require('path');
const vm   = require('vm');
const bootUploadsCode = fs.readFileSync(
    path.join(__dirname, '..', '..', 'simulator', 'boot_uploads.js'), 'utf8');
vm.runInThisContext(bootUploadsCode);

const ERRORS = [];
function fail(msg) { ERRORS.push(msg); }

// ─── Helper: fully boot the simulator ────────────────────────────────────────

function bootSim() {
    const sim = new ChurchSimulator();
    let steps = 0;
    while (!sim.bootComplete && !sim.halted && steps < 300) {
        sim._bootStep();
        steps++;
    }
    return sim;
}

// ─── Test 1: initLazyManifest marks cold NS entries as Outform (gtType=2) ───

(function testColdEntryMarkedOutform() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete for cold-entry marking test');
        return;
    }

    const slideRule = BOOT_UPLOADS.find(b => b.index === 16);
    if (!slideRule) {
        fail('SlideRule (index=16) not found in BOOT_UPLOADS');
        return;
    }

    // Before initLazyManifest: NS[16] should have gtType=1 (Inform) from boot.
    const nsBase = sim.NS_TABLE_BASE + 16 * sim.NS_ENTRY_WORDS;
    const w1Before = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1Before.gtType !== 1) {
        fail(`Expected NS[16] gtType=1 (Inform) before manifest init, got ${w1Before.gtType}`);
        return;
    }

    // Register slot 16 as cold and call initLazyManifest.
    sim.initLazyManifest({
        16: {
            priority: 'cold',
            label:    'SlideRule',
            source:   'boot_upload',
            bootUpload: slideRule,
        }
    });

    // After initLazyManifest: NS[16] word1 gtType must be 2 (Outform).
    const w1After = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1After.gtType !== 2) {
        fail(`Expected NS[16] gtType=2 (Outform) after manifest init, got ${w1After.gtType}`);
        return;
    }

    console.log('[PASS] initLazyManifest: cold NS[16] marked Outform (gtType=2)');
})();

// ─── Test 2: lazyLoad promotes NS entry gtType 2→1 after install ─────────────

(function testLazyLoadPromotion() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete for lazyLoad promotion test');
        return;
    }

    const slideRule = BOOT_UPLOADS.find(b => b.index === 16);
    if (!slideRule) {
        fail('SlideRule (index=16) not found in BOOT_UPLOADS');
        return;
    }

    // Set up cold manifest and call initLazyManifest (writes gtType=2).
    sim.initLazyManifest({
        16: {
            priority: 'cold',
            label:    'SlideRule',
            source:   'boot_upload',
            bootUpload: slideRule,
        }
    });

    const nsBase = sim.NS_TABLE_BASE + 16 * sim.NS_ENTRY_WORDS;
    const w1Cold = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1Cold.gtType !== 2) {
        fail(`Pre-condition failed: NS[16] gtType should be 2 before lazyLoad, got ${w1Cold.gtType}`);
        return;
    }

    // Invoke lazyLoad directly.
    const ok = sim.lazyLoad(16);
    if (!ok) {
        fail('lazyLoad(16) returned false — install failed');
        return;
    }

    // NS[16] word1 gtType must now be 1 (Inform).
    const w1After = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1After.gtType !== 1) {
        fail(`Expected NS[16] gtType=1 (Inform) after lazyLoad, got ${w1After.gtType}`);
        return;
    }

    // Confirm the loader log line fired.
    if (!sim.output.includes('[LOADER] NS[16] Outform→Inform promotion')) {
        fail('Outform→Inform promotion log line not found in simulator output');
        return;
    }

    console.log('[PASS] lazyLoad: NS[16] Outform→Inform promotion in word1 after install');
})();

// ─── Test 3: CALL on Outform GT triggers Mode 2 lazy load ────────────────────
//
// Also verifies the complete cycle: cold entry → CALL on Outform GT →
// loader fires → CALL completes → method return value is correct.
//
// A mock abstractionRegistry is wired in so that SlideRule.Multiply is
// dispatched with DR1=7, DR2=4 and the result (7*4=28) is written to DR1
// before the CALL instruction returns.

(function testCallOutformLazyLoad() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete for CALL Outform test');
        return;
    }

    const slideRule = BOOT_UPLOADS.find(b => b.index === 16);
    if (!slideRule) {
        fail('SlideRule (index=16) not found in BOOT_UPLOADS');
        return;
    }

    // Register slot 16 as cold and call initLazyManifest.
    sim.initLazyManifest({
        16: {
            priority: 'cold',
            label:    'SlideRule',
            source:   'boot_upload',
            bootUpload: slideRule,
        }
    });

    // Confirm NS[16] is now Outform (gtType=2).
    const nsBase = sim.NS_TABLE_BASE + 16 * sim.NS_ENTRY_WORDS;
    const w1 = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1.gtType !== 2) {
        fail(`Pre-condition failed: NS[16] gtType should be 2, got ${w1.gtType}`);
        return;
    }

    // Install a mock abstractionRegistry so _dispatchAbstraction fires after
    // Mode 2 load.  The mock handles SlideRule (index=16) with a Multiply
    // method: result = dr1 * dr2.  We set DR1=7, DR2=4 so the expected
    // return value is 28.  _selectNavanaMethod(d) returns null for DR1=7
    // (not a Navana selector), so method selection falls through to
    // abstraction.methods[0] = 'Multiply'.
    const SLIDE_RULE_METHODS = slideRule.methods.map(m => m.name);
    sim.abstractionRegistry = {
        getAbstraction: (idx) => idx === 16
            ? { name: 'SlideRule', methods: SLIDE_RULE_METHODS }
            : null,
        activate:       () => {},
        dispatchMethod: (idx, methodName, _sim, args) => ({
            ok:     true,
            result: args.dr1 * args.dr2,   // Multiply: 7 * 4 = 28
            message: `SlideRule.${methodName}(${args.dr1}, ${args.dr2}) = ${args.dr1 * args.dr2}`,
        }),
    };

    // Set the multiply operands in DR1 and DR2.
    sim._writeDR(1, 7);
    sim._writeDR(2, 4);

    // Reset the M-window (CR15.m) so _mwinWriteback() in _execCall skips the
    // writeback gate and the CALL proceeds cleanly.  After a full boot the M
    // elevation flag is 1 on CR15; clearing it here mirrors what would happen
    // in a real program that had already executed its first CALL/RETURN pair.
    if (sim.cr[15]) sim.cr[15].m = 0;

    // Build an Outform GT (type=2) for NS slot 16 using the current gt_seq.
    const nsW2 = sim.memory[nsBase + 2];
    const gt_seq = (nsW2 >>> 25) & 0x7F;
    const outformGT = sim.createGT(gt_seq, 16, { E: 1 }, 2);

    // Verify the GT really has type=2 before the test.
    const parsedBefore = sim.parseGT(outformGT);
    if (parsedBefore.type !== 2) {
        fail(`Outform GT type should be 2, got ${parsedBefore.type} (${parsedBefore.typeName})`);
        return;
    }

    // Put the Outform GT in CR1 (update word0 in-place to preserve word2/3/m).
    sim.cr[1].word0 = outformGT;

    const CALL_OPCODE = 2;   // index in the opcode names array
    const codeBase = sim.cr[14].word1;
    const callInstr = sim.encodeInstruction(CALL_OPCODE, 0xE /* AL */, 1, 0, 0);
    sim.memory[codeBase + 1] = callInstr >>> 0;

    sim.pc = 0;
    sim.halted = false;
    const outputBefore = sim.output;
    const faultsBefore = sim.faultLog.length;

    sim.step();

    const newOutput = sim.output.slice(outputBefore.length);
    const newFaults = sim.faultLog.slice(faultsBefore);

    // (a) Loader log line must have fired.
    if (!newOutput.includes('[LOADER] CALL: CR1 is Outform GT (NS[16])')) {
        fail('Mode 2 loader log line not found in output after CALL CR1 (Outform GT)\n' +
             'Output delta:\n' + newOutput);
        return;
    }

    // (b) No TYPE fault about Outform (the old guard that rejected type=2).
    const typeFault = newFaults.find(f => f.type === 'TYPE' &&
        f.message && f.message.includes('Outform'));
    if (typeFault) {
        fail('TYPE fault fired for Outform GT — Mode 2 intercept did not suppress it: ' +
             typeFault.message);
        return;
    }

    // No faults at all — the full CALL cycle succeeded.
    if (newFaults.length > 0) {
        fail('Unexpected fault after CALL on Outform GT: ' +
             newFaults.map(f => `${f.type}: ${f.message}`).join('; '));
        return;
    }

    // (c) Entry must be marked loaded after the lazy load completed.
    if (!sim.lazyManifest[16] || !sim.lazyManifest[16].loaded) {
        fail('lazyManifest[16].loaded is not true after CALL on Outform GT');
        return;
    }

    // (d) NS[16] must now carry gtType=1 (Inform) in its word1.
    const w1After = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1After.gtType !== 1) {
        fail(`Expected NS[16] gtType=1 after Mode 2 load, got ${w1After.gtType}`);
        return;
    }

    // (e) CR1 must now hold an Inform GT (type=1) — promoted from Outform.
    const cr1After = sim.parseGT(sim.cr[1].word0);
    if (cr1After.type !== 1) {
        fail(`Expected CR1 to hold Inform GT (type=1) after Mode 2 promotion, got type=${cr1After.type} (${cr1After.typeName})`);
        return;
    }

    // (f) Return value correct: DR1 = 7 * 4 = 28 (Multiply dispatched by mock registry).
    const expectedResult = 28;
    if (sim.dr[1] !== expectedResult) {
        fail(`Expected DR1=${expectedResult} (SlideRule.Multiply(7,4)) after CALL, got DR1=${sim.dr[1]}`);
        return;
    }

    console.log('[PASS] CALL on Outform GT: Mode 2 lazy load fired, lump installed, CR1 promoted to Inform');
    console.log(`[PASS] SlideRule.Multiply(7,4) = DR1=${sim.dr[1]} (expected ${expectedResult})`);
    console.log('[PASS] lazyManifest[16].loaded = true, NS[16] word1 gtType = 1 (Inform)');
})();

// ─── Test 4: warm entries are NOT marked Outform by initLazyManifest ──────────

(function testWarmEntryNotMarkedOutform() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete for warm-entry test');
        return;
    }

    const slideRule = BOOT_UPLOADS.find(b => b.index === 16);
    if (!slideRule) {
        fail('SlideRule (index=16) not found in BOOT_UPLOADS');
        return;
    }

    // Register slot 16 as WARM (not cold).
    sim.initLazyManifest({
        16: {
            priority: 'warm',
            label:    'SlideRule',
            source:   'boot_upload',
            bootUpload: slideRule,
        }
    });

    // Warm entries must NOT have their gtType changed to 2.
    const nsBase = sim.NS_TABLE_BASE + 16 * sim.NS_ENTRY_WORDS;
    const w1After = sim.parseNSWord1(sim.memory[nsBase + 1]);
    if (w1After.gtType === 2) {
        fail('Warm NS[16] was incorrectly marked Outform (gtType=2) — must remain Inform (gtType=1)');
        return;
    }

    console.log(`[PASS] Warm NS[16] gtType unchanged (${w1After.gtType}) — Outform marking only for cold slots`);
})();

// ─── Report ──────────────────────────────────────────────────────────────────

if (ERRORS.length > 0) {
    for (const e of ERRORS) process.stderr.write('[FAIL] ' + e + '\n');
    process.exit(1);
}
process.exit(0);
