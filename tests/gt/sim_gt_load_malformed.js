'use strict';
// Headless harness for tests/gt/test_gt_load_malformed_perm.py.
//
// Tests that a GT word with malformed permissions written directly into a
// C-List slot in memory (bypassing createGT()) produces a DOMAIN_PURITY fault
// when a LOAD instruction reads that slot and tries to place it into a CR.
//
// This validates the defence-in-depth chain introduced by Task #953:
//   parseGT() sets the 'malformed' flag for any GT whose permission bits
//   violate isDomainPure or isSinglePerm, and _execLoad() surfaces that as a
//   DOMAIN_PURITY fault before the GT reaches any CR.
//
// GT bit layout (simulator parseGT):
//   bits [15: 0]  namespace slot index
//   bits [22:16]  gt_seq
//   bits [24:23]  type  (0b00=NULL 0b01=Inform 0b10=Outform 0b11=Abstract)
//   bits [31:25]  permBits: B=bit6 E=bit5 S=bit4 L=bit3 X=bit2 W=bit1 R=bit0
//
// LOAD opcode = 0 (simulator.js step(), case 0 → _execLoad).
//
// Stdin:  (none — scenarios are hardcoded)
// Stdout: JSON array of result objects

global.window = { bootConfig: {} };

const { bootSim, setupCR6 } = require('../gates/sim_helpers');

// Craft GT words with malformed permission bits:
//   X+E  — domain-impure (Turing X mixed with Church E)
//           permBits = X(bit2) | E(bit5) = 0b0100100 = 0x24
//   L+E  — multi-Church (two Church bits set, violates isSinglePerm)
//           permBits = L(bit3) | E(bit5) = 0b0101000 = 0x28
// Both use Inform type (0b01 at bits[24:23]) and index=1 (valid NS slot).
const MALFORMED_XE = ((0x24 << 25) | (0x01 << 23) | 1) >>> 0;
const MALFORMED_LE = ((0x28 << 25) | (0x01 << 23) | 1) >>> 0;

function runLoadFromClistSlot(scenarioName, slotGTWord) {
    const sim = bootSim();
    if (!sim.bootComplete) {
        return { name: scenarioName, error: 'boot did not complete' };
    }

    // Wire CR6 to a 2-slot scratch c-list at address 500.
    // setupCR6 installs a valid E-GT for the boot entry slot, so mLoad on the
    // c-list pointer itself (CR6) succeeds.  The malformed GT is in the slot
    // content — what LOAD reads and attempts to place into a CR.
    setupCR6(sim);

    // Write the malformed GT word directly into c-list slot 0 (address 500).
    // This simulates a STORE instruction or a test harness writing a GT with
    // bad permission bits into memory, bypassing the createGT() guards.
    sim.memory[500] = slotGTWord >>> 0;

    // Find the code-lump base from CR14.
    const cr14 = sim.cr[14];
    const codeBase = cr14 ? cr14.word1 : null;
    if (codeBase == null) return { name: scenarioName, error: 'CR14.word1 is null' };

    // Encode: LOAD CR1, [CR6 + 0]
    // opcode=0 (LOAD), cond=0xE (AL=Always), crDst=1, crSrc=6, imm=0
    const instr = sim.encodeInstruction(0, 0xE, 1, 6, 0);
    sim.memory[codeBase + 1] = instr >>> 0;

    sim.pc = 0;
    sim.halted = false;

    const faultsBefore = sim.faultLog ? sim.faultLog.length : 0;
    sim.step();
    const newFaults = sim.faultLog ? sim.faultLog.slice(faultsBefore) : [];

    return {
        name:      scenarioName,
        faulted:   newFaults.length > 0,
        faultCode: newFaults.length ? newFaults[0].type    : null,
        faultMsg:  newFaults.length ? newFaults[0].message : null,
    };
}

const results = [
    // Scenario 1: X+E GT in c-list slot — domain-impure → DOMAIN_PURITY fault
    runLoadFromClistSlot('malformed_xe_gt_faults_domain_purity', MALFORMED_XE),
    // Scenario 2: L+E GT in c-list slot — multi-Church → DOMAIN_PURITY fault
    runLoadFromClistSlot('malformed_le_gt_faults_domain_purity', MALFORMED_LE),
];

process.stdout.write(JSON.stringify(results, null, 2) + '\n');
