// Boot.Abstr 3-instruction step-through harness (Task #655).
//
// After the boot ROM completes (bootComplete=true) the simulator is parked at
// PC=0 inside Boot.Abstr (NS Slot 3) with:
//   CR12 -> Boot.Thread (NS Slot 1)   [set by boot ROM]
//   CR14 -> Boot.Abstr (NS Slot 3)    [code register]
//   CR0  -> NULL (boot ROM did not populate the caps zone)
//
// Boot.Abstr code (Task #651, 3 instructions):
//   PC=0  CHANGE AL, CR12, CR12, #1   -- reload CR12=Boot.Thread, RESTORE_CALL loads CR0-CR11
//   PC=1  TPERM  AL, CR0,  #E        -- restrict CR0 to E-permission only
//   PC=2  CALL   AL, CR0,  CR0       -- call through CR0 (Entry E-GT)
//
// The fix (Task #655): _execChange CR12 path must restore CR0-CR11 from the
// incoming thread's caps zone (THREAD_CAPS_OFFSET) when mElevation=true or
// when the thread has no prior saved context (first-activation).
//
// Without the fix CR0 remains NULL and TPERM/CALL fault.
//
// Output (JSON to stdout):
//   {
//     "bootComplete":    true,           // boot ROM completed
//     "faultLog":        [],             // faults during boot
//     "cr0AfterChange":  { word0, word1 },  // CR0 after step 1 (CHANGE)
//     "step1":           { desc, pc, faulted },
//     "step2":           { desc, pc, faulted },
//     "step3":           { desc, pc, faulted },
//     "anyFault":        false,
//     "allStepsFaultFree": true,
//   }

global.window = {
    bootConfig: {
        step1: {
            totalNamespaceWords:  16384,
            namespaceLumpWords:      64,
            threadLumpWords:        256,
        }
    }
};

const ChurchSimulator     = require('../../simulator/simulator.js');
const AbstractionRegistry = require('../../simulator/abstractions.js');
const SystemAbstractions  = require('../../simulator/system_abstractions.js');

const sim      = new ChurchSimulator();
const registry = new AbstractionRegistry();
const sys      = new SystemAbstractions(registry);
sim.initAbstractions(registry, sys, null);

const bootFaultLog = [];
const origFault = sim.fault.bind(sim);
sim.fault = (type, msg) => {
    bootFaultLog.push({ type, message: msg });
    origFault(type, msg);
};

// --- Drive boot state machine (B:00–B:05) ---
const MAX_BOOT = 32;
let bootIters = 0;
while (bootIters < MAX_BOOT && !sim.bootComplete && !sim.halted) {
    const advanced = sim._bootStep();
    bootIters++;
    if (!advanced) break;
}

// --- Step through the 3 Boot.Abstr instructions ---
const stepResults = [];

function doStep(label) {
    const faultsBefore = (sim.faultLog || []).length;
    const pcBefore     = sim.pc;
    let result = null;
    try {
        result = sim.step();
    } catch (e) {
        result = null;
    }
    const faultsAfter = (sim.faultLog || []).length;
    const faulted     = faultsAfter > faultsBefore || sim.halted;
    stepResults.push({
        label,
        pc:      pcBefore,
        desc:    result ? result.desc : null,
        faulted,
    });
}

doStep('CHANGE CR12');
const cr0AfterChange = { word0: sim.cr[0].word0 >>> 0, word1: sim.cr[0].word1 >>> 0 };

doStep('TPERM CR0,#E');
doStep('CALL CR0,CR0');

const anyFault = stepResults.some(s => s.faulted);

const out = {
    bootComplete:        sim.bootComplete === true,
    bootFaultLog:        bootFaultLog.map(f => ({ type: f.type, message: f.message })),
    cr0AfterChange,
    step1:               stepResults[0],
    step2:               stepResults[1],
    step3:               stepResults[2],
    anyFault,
    allStepsFaultFree:   !anyFault,
};

process.stdout.write(JSON.stringify(out) + '\n');
