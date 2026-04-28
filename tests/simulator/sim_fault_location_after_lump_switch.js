'use strict';
// Headless harness used by tests/simulator/test_fault_location_after_lump_switch.py.
//
// Regression test for the bug fixed in Task #649: after a CALL E-GT triggers
// a lump switch, any subsequent fault was attributed to "Boot.NS +NNN" instead
// of the new lump's label and relative offset.
//
// The fix records CR14 in crSnapshot at fault time.  app-run.js then reads
// crSnapshot[14].word0 & 0xFFFF as the NS index and crSnapshot[14].word1 as
// the lump base — the raw crSnapshot path, not getFormattedCR.
//
// Test plan:
//   1. Boot the simulator.
//   2. Load the SlideRule lump (NS slot 16) so it is resident in memory.
//   3. Execute a CALL E-GT instruction that switches CR14 from the boot-entry
//      lump to SlideRule.
//   4. Trigger a NULL_CAP fault inside SlideRule's code area (CALL on a null
//      CR0 written at SlideRule's first instruction slot).
//   5. Assert that the fault entry's crSnapshot[14].word0 & 0xFFFF == 16
//      (SlideRule's NS index, not the boot-entry slot).
//   6. Assert that nsLabels[16] is 'SlideRule' (not Boot.NS or any boot label).
//   7. Assert that (physicalPC - crSnapshot[14].word1) gives a non-negative
//      offset that falls within the SlideRule lump, confirming the offset is
//      computed relative to SlideRule's base, not the boot lump's base.
//
// Exits with code 0 on success, 1 on failure (errors written to stderr).

global.window = { bootConfig: {} };

const ChurchSimulator = require('../../simulator/simulator.js');
const fs   = require('fs');
const path = require('path');
const vm   = require('vm');

// Load boot_uploads.js so the BOOT_UPLOADS global is available.
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

// ─── Test: fault inside a switched-to lump is attributed to that lump ────────

(function testFaultLocationAfterLumpSwitch() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete: ' +
             (sim.faultLog && sim.faultLog.length
                 ? sim.faultLog[sim.faultLog.length - 1].message
                 : '(no fault)'));
        return;
    }

    // ── Step 1: Install SlideRule lump (NS slot 16) into memory ──────────────
    const slideRule = BOOT_UPLOADS.find(b => b.index === 16);
    if (!slideRule) {
        fail('SlideRule (index=16) not found in BOOT_UPLOADS');
        return;
    }

    // Register slot 16 as warm so initLazyManifest does not flip gtType to 2
    // (we want a plain Inform CALL, not an Outform lazy-load intercept).
    sim.initLazyManifest({
        16: {
            priority: 'warm',
            label:    'SlideRule',
            source:   'boot_upload',
            bootUpload: slideRule,
        }
    });

    // Explicitly install the lump so its header and code are in memory.
    const loadOk = sim.lazyLoad(16);
    if (!loadOk) {
        fail('lazyLoad(16) failed — SlideRule lump could not be installed');
        return;
    }

    // ── Step 2: Build an E-GT for NS[16] and put it in CR1 ───────────────────
    const nsW2    = sim.memory[sim.NS_TABLE_BASE + 16 * sim.NS_ENTRY_WORDS + 2];
    const gt_seq  = (nsW2 >>> 25) & 0x7F;
    const eGT16   = sim.createGT(gt_seq, 16, { E: 1 }, 1);
    sim.cr[1].word0 = eGT16;

    // ── Step 3: Write CALL CR1 at the boot-entry lump's first instruction slot
    // and execute it to switch CR14 to SlideRule.
    // The M-window writeback gate fires on the first Inform CALL after boot if
    // CR15.m is still set; clear it to let the CALL proceed cleanly.
    if (sim.cr[15]) sim.cr[15].m = 0;

    const CALL_OPCODE = 2;   // index 2 in the opcode names array
    const bootCodeBase = sim.cr[14].word1;

    // Write CALL AL, CR1, CR1 at PC-slot 0 of the current (boot-entry) lump.
    sim.memory[bootCodeBase + 1] = sim.encodeInstruction(CALL_OPCODE, 0xE, 1, 0, 0);

    sim.pc     = 0;
    sim.halted = false;

    const faultsBefore = sim.faultLog.length;
    sim.step();   // Execute the CALL CR1 — switches CR14 to SlideRule

    if (sim.faultLog.length > faultsBefore) {
        const f = sim.faultLog[faultsBefore];
        fail('CALL to SlideRule (NS[16]) faulted unexpectedly: [' + f.type + '] ' + f.message);
        return;
    }

    // ── Step 4: Verify CR14 now points to SlideRule (NS slot 16) ─────────────
    const cr14After = sim.cr[14];
    const cr14NsIdx = cr14After.word0 & 0xFFFF;   // raw crSnapshot path
    if (cr14NsIdx !== 16) {
        fail('After CALL, CR14 should point to NS[16] (SlideRule), but word0 & 0xFFFF = ' + cr14NsIdx);
        return;
    }

    const slideRuleBase = cr14After.word1 >>> 0;

    // ── Step 5: Trigger a fault inside SlideRule's code area ─────────────────
    // Null out CR0, then write CALL CR0 at SlideRule's first instruction slot.
    // This produces a NULL_CAP fault with physicalPC = slideRuleBase + 1.
    sim.cr[0] = { word0: 0, word1: 0, word2: 0, word3: 0, m: 0 };
    sim.memory[slideRuleBase + 1] = sim.encodeInstruction(CALL_OPCODE, 0xE, 0, 0, 0);

    sim.pc     = 0;
    sim.halted = false;

    const faultsBefore2 = sim.faultLog.length;
    sim.step();   // Fetch from SlideRule slot 0; CALL CR0 (null) → NULL_CAP
    const newFaults = sim.faultLog.slice(faultsBefore2);

    if (newFaults.length === 0) {
        fail('Expected a NULL_CAP fault inside SlideRule lump, but no fault was recorded');
        return;
    }

    const f = newFaults[0];

    // ── Step 6: Core assertion — crSnapshot[14] names SlideRule, not the boot lump
    if (!f.crSnapshot || !f.crSnapshot[14]) {
        fail('Fault entry is missing crSnapshot[14]');
        return;
    }

    // Use the raw crSnapshot path (matches app-run.js lines ~1609–1615):
    //   _ni  = crSnapshot[14].word0 & 0xFFFF   → NS slot index
    //   _lbl = nsLabels[_ni]                   → lump label
    //   _base = crSnapshot[14].word1            → lump base address
    const faultCR14  = f.crSnapshot[14];
    const faultNsIdx = faultCR14.word0 & 0xFFFF;

    if (faultNsIdx !== 16) {
        const bootEntryNsIdx = sim.cr[14].word0 & 0xFFFF;
        fail(
            'crSnapshot[14] reports NS[' + faultNsIdx + '] ' +
            '(nsLabel="' + (sim.nsLabels[faultNsIdx] || '?') + '") — ' +
            'expected NS[16] (SlideRule).  ' +
            'Before the bug fix this would have reported NS[' + bootEntryNsIdx + '] ' +
            '("' + (sim.nsLabels[bootEntryNsIdx] || '?') + '") or derived via _nsOwnerOf.'
        );
        return;
    }

    // Confirm the label is SlideRule (not Boot.NS, Boot.Abstr, or LED flash).
    const faultLabel    = (sim.nsLabels && sim.nsLabels[faultNsIdx]) || ('NS[' + faultNsIdx + ']');
    const bootEntryLabel = (sim.nsLabels && sim.nsLabels[sim.bootEntrySlot])
        || ('NS[' + sim.bootEntrySlot + ']');

    if (!faultLabel.includes('SlideRule')) {
        fail('Fault label is "' + faultLabel + '" — expected "SlideRule" for NS[16]');
        return;
    }

    // Negative assertion: the fault label must NOT equal the boot-entry lump's
    // label.  Before the Task #649 fix, _nsOwnerOf(physicalPC) could return the
    // boot-entry label even after a lump switch — this guard would have caught it.
    if (faultLabel === bootEntryLabel) {
        fail(
            'Fault label "' + faultLabel + '" equals the boot-entry label "' + bootEntryLabel +
            '" (NS[' + sim.bootEntrySlot + ']) — the fault is wrongly attributed to the ' +
            'pre-switch lump (regression from Task #649)'
        );
        return;
    }

    // ── Step 7: Offset must be relative to SlideRule's base ──────────────────
    const faultBase = faultCR14.word1 >>> 0;
    const faultPC   = (f.physicalPC !== undefined && f.physicalPC !== null)
        ? f.physicalPC >>> 0
        : f.pc;
    const offset = faultPC - faultBase;

    if (offset < 0) {
        fail('Fault offset is negative (' + offset + '): physicalPC=0x' +
             faultPC.toString(16) + ' base=0x' + faultBase.toString(16) +
             ' — offset is not relative to SlideRule lump');
        return;
    }

    // The offset should be 1 (instruction slot 0 of the lump = base+1+pc=base+1+0).
    // We allow any small positive value to remain robust to minor layout changes,
    // but it must be strictly less than the lump size.
    const nsEntry = sim.readNSEntry(16);
    const lumpSize = nsEntry ? (nsEntry.word1_limit & 0x1FFFF) : 256;
    if (offset >= lumpSize) {
        fail('Fault offset ' + offset + ' exceeds SlideRule lump size ' + lumpSize +
             ' — offset is not relative to SlideRule base');
        return;
    }

    console.log('[PASS] Fault correctly attributed to SlideRule (NS[16]) after CALL E-GT lump switch');
    console.log('[PASS] crSnapshot[14].word0 & 0xFFFF = ' + faultNsIdx + ' (SlideRule NS index)');
    console.log('[PASS] label = "' + faultLabel + '", offset = ' + offset +
                ', physicalPC=0x' + faultPC.toString(16) +
                ', base=0x' + faultBase.toString(16));
})();

// ── Second assertion: boot-entry slot label is NOT what the fault reports ─────
//
// This test documents the regression: before the fix, _nsOwnerOf(physicalPC)
// could return the boot-entry lump's label if its memory range happened to
// contain the physical PC.  We confirm that nsLabels for the boot-entry slot
// is different from nsLabels[16] so the assertions above have discriminating
// power.

(function testBootSlotAndSlideRuleHaveDifferentLabels() {
    const sim = bootSim();
    if (!sim.bootComplete) {
        fail('Boot did not complete for label-discrimination test');
        return;
    }

    // bootEntrySlot is 3 by default (LED flash); nsLabels[16] is SlideRule.
    const bootLabel    = sim.nsLabels[sim.bootEntrySlot] || ('NS[' + sim.bootEntrySlot + ']');
    const slideLabel   = sim.nsLabels[16] || 'NS[16]';

    if (bootLabel === slideLabel) {
        fail('Boot-entry label ("' + bootLabel + '") equals SlideRule label — ' +
             'test has no discriminating power against the old bug');
        return;
    }

    console.log('[PASS] Boot-entry label "' + bootLabel + '" differs from SlideRule label "' + slideLabel + '"');
    console.log('[PASS] Fault-location assertions in the primary test have discriminating power');
})();

// ─── Report ──────────────────────────────────────────────────────────────────

if (ERRORS.length > 0) {
    for (const e of ERRORS) process.stderr.write('[FAIL] ' + e + '\n');
    process.exit(1);
}
process.exit(0);
