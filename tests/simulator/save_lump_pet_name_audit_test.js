// save_lump_pet_name_audit_test.js
//
// Unit tests for petNameAudit() from simulator/pet_name_audit.js.
//
// Covered cases:
//   T1:  cc=0 — trivial pass, no c-list to audit
//   T2:  Abstract LED GT, name in nsSymbols — ✓ pass
//   T3:  Abstract LED GT, no nsSymbols — ✓ pass (derived name sufficient)
//   T4:  Inform GT, no nsLabels entry — ⚠ warn (unnamed, not referenced)
//   T5:  Inform GT, no nsLabels, LOAD references it — ⚠ warn (unnamed, referenced)
//   T6:  Null GT in slot 0, LOAD references it — ✗ fail (ok=false)
//   T7:  Null GT in slot 0, not referenced — silent (no issue)
//   T8:  SAVE opcode via CR6 — caught by audit (named slot → pass)
//   T9:  ELOADCALL via CR6 — caught by audit (named slot → pass)
//   T10: XLOADLAMBDA via CR6 — caught by audit (named slot → pass)
//   T11: CALL opcode (not a Church c-list op) — ignored by audit
//   T12: LOAD via CR3 (crSrc ≠ 6) — ignored by audit
//   T13: cc=2, slot 0 named (Abstract), slot 1 unnamed Inform GT — mixed result
//   T14: Registered name differs from derived name — both shown
//   T15: M-Elevation Abstract GT — derived name = 'M-Elevation'
//   T16: Unknown Abstract GT subtype — derived name = 'Abs3'
//   T17: Multiple Church instructions referencing same slot — one slot report
//   T18: slot refs for a slot beyond cc (out of bounds) — ignored
//
// Run with:  node tests/simulator/save_lump_pet_name_audit_test.js
// Exits 0 on full success, non-zero on any failure.

'use strict';

const path = require('path');
const { petNameAudit } = require(path.join(__dirname, '../../simulator/pet_name_audit.js'));

let passed = 0;
let failed = 0;

function assert(label, cond, detail) {
    if (cond) {
        console.log('PASS ' + label);
        passed++;
    } else {
        console.error('FAIL ' + label + (detail ? '\n     ' + detail : ''));
        failed++;
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

const LUMP_SIZE = 64;
const ABSTRACT_LED_GT = 0x07800100;  // LED0: gtType=3, abType=0, devCls=1, devDat=0

// Encode a Church instruction that loads from c-list slot via CR6.
// op: 0=LOAD, 1=SAVE, 8=ELOADCALL, 9=XLOADLAMBDA; crSrc defaults to 6.
function churchWord(op, slot, crSrc) {
    crSrc = (crSrc === undefined) ? 6 : crSrc;
    return ((op & 0x1F) << 27) | ((crSrc & 0xF) << 15) | (slot & 0x7FFF);
}

// Build a flat 64-word lump array.
//   codeWords   — words to place at indices 1..cw  (cw = codeWords.length)
//   clistSlots  — GT values to place at tail (cc = clistSlots.length)
function makeLump(codeWords, clistSlots) {
    const cc = clistSlots.length;
    const cw = codeWords.length;
    const words = new Array(LUMP_SIZE).fill(0);
    for (let i = 0; i < cw; i++)       words[1 + i]                      = codeWords[i];
    for (let i = 0; i < cc; i++)       words[LUMP_SIZE - cc + i]          = clistSlots[i];
    return words;
}

function hdr(codeWords, clistSlots) {
    return { cw: codeWords.length, cc: clistSlots.length, lumpSize: LUMP_SIZE };
}

// ── T1: cc=0 → trivial pass ──────────────────────────────────────────────────
{
    const words = new Array(LUMP_SIZE).fill(0);
    const r = petNameAudit(words, { cw: 1, cc: 0, lumpSize: LUMP_SIZE }, {}, {});
    assert('T1 cc=0: ok=true',       r.ok);
    assert('T1 cc=0: warnCount=0',   r.warnCount === 0);
    assert('T1 cc=0: lines has cc=0 note',
        r.lines.some(l => l.includes('cc=0')));
}

// ── T2: Abstract LED GT in slot 0, name in nsSymbols → ✓ pass ───────────────
{
    const code = [churchWord(0, 0)];          // LOAD CRx, CR6, #0
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, { LED0: 0 });
    assert('T2 named via nsSymbols: ok=true',      r.ok);
    assert('T2 named via nsSymbols: warnCount=0',  r.warnCount === 0);
    assert('T2 named via nsSymbols: ✓ in summary', r.lines[0].includes('\u2713'));
    assert('T2 named via nsSymbols: slot named LED0',
        r.lines.some(l => l.includes('LED0') && l.includes('\u2713')));
}

// ── T3: Abstract LED GT, no nsSymbols → ✓ pass (derived name sufficient) ─────
{
    const code = [churchWord(0, 0)];
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T3 derived name only: ok=true',     r.ok);
    assert('T3 derived name only: warnCount=0', r.warnCount === 0);
    assert('T3 derived name only: LED0 shown',
        r.lines.some(l => l.includes('LED0')));
}

// ── T4: Inform GT (unknown nsLabels), not referenced by code → ⚠ warn ────────
{
    const INFORM_GT = 0x00000001;              // gtType=0, slotId=1, nsLabels={} → no name
    const code = [];
    const w = makeLump(code, [INFORM_GT]);
    const r = petNameAudit(w, hdr(code, [INFORM_GT]), {}, {});
    assert('T4 unnamed not referenced: ok=true',    r.ok);
    assert('T4 unnamed not referenced: warnCount=1', r.warnCount === 1);
    assert('T4 unnamed not referenced: ⚠ summary',
        r.lines[0].includes('\u26A0'));
    assert('T4 unnamed not referenced: "not referenced"',
        r.lines.some(l => l.includes('not referenced')));
}

// ── T5: Inform GT, no nsLabels, LOAD references it → ⚠ warn (named reference)
{
    const INFORM_GT = 0x00000001;
    const code = [churchWord(0, 0)];          // LOAD CRx, CR6, #0
    const w = makeLump(code, [INFORM_GT]);
    const r = petNameAudit(w, hdr(code, [INFORM_GT]), {}, {});
    assert('T5 unnamed referenced: ok=true',    r.ok);
    assert('T5 unnamed referenced: warnCount=1', r.warnCount === 1);
    assert('T5 unnamed referenced: "used by"',
        r.lines.some(l => l.includes('used by') && l.includes('LOAD')));
}

// ── T6: Null GT in slot 0, LOAD references it → ✗ fail (ok=false) ────────────
{
    const code = [churchWord(0, 0)];          // LOAD CRx, CR6, #0
    const w = makeLump(code, [0]);            // slot 0 = null GT
    const r = petNameAudit(w, hdr(code, [0]), {}, {});
    assert('T6 null GT referenced: ok=false',    !r.ok);
    assert('T6 null GT referenced: ✗ summary',   r.lines[0].includes('\u2717'));
    assert('T6 null GT referenced: "FAULT"',
        r.lines.some(l => l.includes('FAULT')));
    assert('T6 null GT referenced: "slot [0]"',
        r.lines.some(l => l.includes('slot [0]') && l.includes('\u2717')));
}

// ── T7: Null GT in slot 0, not referenced by code → silent ───────────────────
{
    const code = [];
    const w = makeLump(code, [0]);
    const r = petNameAudit(w, hdr(code, [0]), {}, {});
    assert('T7 null GT not referenced: ok=true',    r.ok);
    assert('T7 null GT not referenced: warnCount=0', r.warnCount === 0);
    assert('T7 null GT not referenced: no ✗ line',
        !r.lines.some(l => l.includes('\u2717')));
}

// ── T8: SAVE opcode via CR6 to a named slot → ✓ pass ────────────────────────
{
    const code = [churchWord(1, 0)];          // SAVE CR6, #0 (opcode 1)
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T8 SAVE named: ok=true',     r.ok);
    assert('T8 SAVE named: warnCount=0', r.warnCount === 0);
}

// ── T9: ELOADCALL via CR6 to a named slot → ✓ pass ───────────────────────────
{
    const code = [churchWord(8, 0)];          // ELOADCALL CR6, #0 (opcode 8)
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T9 ELOADCALL named: ok=true',     r.ok);
    assert('T9 ELOADCALL named: warnCount=0', r.warnCount === 0);
}

// ── T10: XLOADLAMBDA via CR6 to a named slot → ✓ pass ────────────────────────
{
    const code = [churchWord(9, 0)];          // XLOADLAMBDA CR6, #0 (opcode 9)
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T10 XLOADLAMBDA named: ok=true',     r.ok);
    assert('T10 XLOADLAMBDA named: warnCount=0', r.warnCount === 0);
}

// ── T11: CALL opcode (op=2, not a c-list op) via CR6 → ignored ───────────────
{
    const CALL_WORD = (2 << 27) | (6 << 15) | 0;  // CALL, crSrc=6, slot=0 ignored
    const code = [CALL_WORD];
    const w = makeLump(code, [0]);            // slot 0 null — would fail if scanned
    const r = petNameAudit(w, hdr(code, [0]), {}, {});
    assert('T11 CALL ignored: ok=true',    r.ok);   // CALL not in _CHURCH_OPS
    assert('T11 CALL ignored: no fault',   !r.lines.some(l => l.includes('FAULT')));
}

// ── T12: LOAD via CR3 (crSrc≠6) → ignored ────────────────────────────────────
{
    const LOAD_CR3 = churchWord(0, 0, 3);    // LOAD CRx, CR3, #0 (crSrc=3)
    const code = [LOAD_CR3];
    const w = makeLump(code, [0]);            // slot 0 null — irrelevant if not scanned
    const r = petNameAudit(w, hdr(code, [0]), {}, {});
    assert('T12 LOAD via CR3 ignored: ok=true', r.ok);
    assert('T12 LOAD via CR3: no fault line',
        !r.lines.some(l => l.includes('FAULT')));
}

// ── T13: cc=2, slot 0 named Abstract, slot 1 unnamed Inform → mixed ──────────
{
    const INFORM_GT = 0x00000001;
    const code = [
        churchWord(0, 0),                    // LOAD via slot 0 (named)
        churchWord(0, 1),                    // LOAD via slot 1 (unnamed)
    ];
    const w = makeLump(code, [ABSTRACT_LED_GT, INFORM_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT, INFORM_GT]), {}, {});
    assert('T13 mixed: ok=true (no null GT)',   r.ok);
    assert('T13 mixed: warnCount=1',            r.warnCount === 1);
    assert('T13 mixed: slot 0 named ✓',
        r.lines.some(l => l.includes('slot [0]') && l.includes('\u2713')));
    assert('T13 mixed: slot 1 unnamed ⚠',
        r.lines.some(l => l.includes('slot [1]') && l.includes('\u26A0')));
}

// ── T14: Registered name differs from derived name → both shown ──────────────
{
    const code = [churchWord(0, 0)];
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    // nsSymbols maps 'MyLED' → 0, but derived name is 'LED0'
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, { MyLED: 0 });
    assert('T14 both names shown: ok=true',    r.ok);
    assert('T14 both names shown: line has both',
        r.lines.some(l => l.includes('MyLED') && l.includes('LED0')));
    assert('T14 both names shown: "(registered)" label',
        r.lines.some(l => l.includes('registered')));
}

// ── T15: M-Elevation Abstract GT → derived name = 'M-Elevation' ─────────────
{
    // M-Elevation: gtType=3 (bits[24:23]=11), abType=1 (bits[31:27]=00001)
    // Build: (1 << 27) | (3 << 23) = 0x08000000 | 0x01800000 = 0x09800000
    // But we need gtType=(wVal>>>23)&3=3 → bits[24:23]=11
    // And abType=(wVal>>>27)&0x1F=1 → bits[31:27]=00001
    // Combined: (1<<27)|(3<<23) = 0x08000000|0x01800000 = 0x09800000
    const M_ELEV_GT = 0x09800000;
    const code = [churchWord(0, 0)];
    const w = makeLump(code, [M_ELEV_GT]);
    const r = petNameAudit(w, hdr(code, [M_ELEV_GT]), {}, {});
    assert('T15 M-Elevation: ok=true',      r.ok);
    assert('T15 M-Elevation: name derived',
        r.lines.some(l => l.includes('M-Elevation')));
}

// ── T16: Unknown Abstract GT subtype → derived name = 'Abs3' ─────────────────
{
    // abType=3 (bits[31:27]=00011), gtType=3 (bits[24:23]=11)
    // (3<<27)|(3<<23) = 0x18000000|0x01800000 = 0x19800000
    const ABS3_GT = 0x19800000;
    const code = [churchWord(0, 0)];
    const w = makeLump(code, [ABS3_GT]);
    const r = petNameAudit(w, hdr(code, [ABS3_GT]), {}, {});
    assert('T16 Abs3: ok=true',   r.ok);
    assert('T16 Abs3: name=Abs3', r.lines.some(l => l.includes('Abs3')));
}

// ── T17: Multiple Church instructions referencing same slot → one slot report ─
{
    const code = [
        churchWord(0, 0),   // LOAD via slot 0
        churchWord(8, 0),   // ELOADCALL via slot 0
        churchWord(9, 0),   // XLOADLAMBDA via slot 0
    ];
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T17 multi-ref same slot: ok=true',     r.ok);
    assert('T17 multi-ref same slot: warnCount=0', r.warnCount === 0);
    // Exactly one slot report line (✓ slot [0])
    const slotLines = r.lines.filter(l => l.includes('slot [0]'));
    assert('T17 multi-ref same slot: one slot line', slotLines.length === 1);
}

// ── T18: Slot ref index >= cc → ignored (caught by check 6 before audit) ──────
{
    const code = [churchWord(0, 5)];          // LOAD via slot 5, but cc=1
    const w = makeLump(code, [ABSTRACT_LED_GT]);
    const r = petNameAudit(w, hdr(code, [ABSTRACT_LED_GT]), {}, {});
    assert('T18 out-of-bounds slot ref ignored: ok=true', r.ok);
    assert('T18 out-of-bounds slot ref: no issue for slot 5',
        !r.lines.some(l => l.includes('slot [5]')));
}

// ── Summary ───────────────────────────────────────────────────────────────────
console.log('\n' + passed + ' passed, ' + failed + ' failed');
if (failed > 0) process.exit(1);
