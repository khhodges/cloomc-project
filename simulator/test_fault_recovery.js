'use strict';
// test_fault_recovery.js — Unit tests for Task #1077 three-tier fault recovery
// Run:  node simulator/test_fault_recovery.js
//
// Coverage:
//   T001 — Tier 1: .catch method invoked when present on the faulting abstraction
//   T002 — Tier 2: .catch absent, Scheduler.IRQ dispatched with registered handler
//   T003 — Tier 3: double-fault (fault while irqActive=true) calls _returnToBoot
//   T004 — pause timer fire: Scheduler.pause arms timer; step to deadline fires IRQ
//   T005 — Wait flag-set wake: pendingWakeFlags signals sleeping thread via IRQ sweep
//   T006 — Structured fault record: all new fields populated on an unhandled fault

const ChurchSimulator   = require('./simulator.js');
const AbstractionRegistry = require('./abstractions.js');
const SystemAbstractions  = require('./system_abstractions.js');

let pass = 0;
let fail = 0;

function check(label, cond) {
    if (cond) {
        console.log(`PASS ${label}`);
        pass++;
    } else {
        console.log(`FAIL ${label}`);
        fail++;
    }
}

// Build a minimal simulator wired to a fresh registry + system abstractions.
function makeTestSim() {
    const sim = new ChurchSimulator();
    const registry = new AbstractionRegistry();
    // SystemAbstractions binds all methods (including Scheduler.pause and Scheduler.IRQ)
    const sysAbs = new SystemAbstractions(registry);
    sim.abstractionRegistry = registry;
    sim.bootComplete = true;
    return { sim, registry, sysAbs };
}

// ── T001: Tier 1 — .catch method invoked when present ─────────────────────────
console.log('\n--- T001: Tier 1 (.catch) ---');
{
    const { sim, registry } = makeTestSim();
    // Point CR14 at NS slot 10 (DijkstraFlag) as the "currently executing lump"
    if (!sim.cr) sim.cr = new Array(16).fill(null);
    sim.cr[14] = { word0: 10, word1: 0, word2: 0, word3: 0 };

    // Bind a .catch handler on slot 10 that signals recovery
    let catchCalled = false;
    registry.bindMethod(10, '.catch', (s, args) => {
        catchCalled = true;
        return { handled: true };
    });

    const pcBefore = sim.pc;
    sim.fault('PERM_R', 'Test Tier 1 catch');

    check('T001a: machine NOT halted after Tier 1 catch', !sim.halted);
    check('T001b: faultLog has exactly one entry', sim.faultLog.length === 1);
    check('T001c: fault entry tier === 1', sim.faultLog[0].tier === 1);
    check('T001d: fault entry catchInvoked === true', sim.faultLog[0].catchInvoked === true);
    check('T001e: .catch function was actually called', catchCalled === true);
    check('T001f: faultCode matches PERM_R', sim.faultLog[0].faultCode === ChurchSimulator.FAULT_CODES.PERM_R);
    check('T001g: faultingAbstractionSlot === 10', sim.faultLog[0].faultingAbstractionSlot === 10);
    check('T001h: PC advanced past faulting instruction', sim.pc === pcBefore + 1);
    check('T001i: irqActive is false (not entered IRQ frame)', !sim.irqState.irqActive);
}

// ── T002: Tier 2 — .catch absent, Scheduler.IRQ succeeds ──────────────────────
console.log('\n--- T002: Tier 2 (Scheduler.IRQ) ---');
{
    const { sim, registry, sysAbs } = makeTestSim();
    // CR14 points at NS slot 10 but no .catch bound → Tier 1 skipped
    if (!sim.cr) sim.cr = new Array(16).fill(null);
    sim.cr[14] = { word0: 10, word1: 0, word2: 0, word3: 0 };

    // Register a fault recovery handler so Tier 2 can succeed
    let handlerCalled = false;
    sysAbs._schedulerState.faultRecoveryHandler = (faultRecord) => {
        handlerCalled = true;
        return true;
    };

    const pcBefore = sim.pc;
    sim.fault('PERM_W', 'Test Tier 2 IRQ escalation');

    check('T002a: machine NOT halted after Tier 2 recovery', !sim.halted);
    check('T002b: faultLog has exactly one entry', sim.faultLog.length === 1);
    check('T002c: fault entry tier === 2', sim.faultLog[0].tier === 2);
    check('T002d: fault entry irqInvoked === true', sim.faultLog[0].irqInvoked === true);
    check('T002e: fault entry catchInvoked === false (Tier 1 was skipped)', !sim.faultLog[0].catchInvoked);
    check('T002f: faultRecoveryHandler was called', handlerCalled === true);
    check('T002g: irqActive reset to false after Tier 2', !sim.irqState.irqActive);
    check('T002h: PC advanced past faulting instruction', sim.pc === pcBefore + 1);
}

// ── T003: Tier 3 — double-fault while irqActive=true ─────────────────────────
console.log('\n--- T003: Tier 3 (double-fault) ---');
{
    const { sim } = makeTestSim();
    // Simulate: fault fires while Scheduler.IRQ is already executing
    sim.irqState.irqActive = true;

    // Spy on _returnToBoot so we can verify it's called without running the full boot
    let tier3Called = false;
    const origReturn = sim._returnToBoot ? sim._returnToBoot.bind(sim) : null;
    sim._returnToBoot = () => {
        tier3Called = true;
        if (origReturn) origReturn();
    };

    sim.fault('NULL_CAP', 'Test Tier 3 double-fault');

    check('T003a: faultLog has exactly one entry', sim.faultLog.length === 1);
    check('T003b: fault entry tier === 3', sim.faultLog[0].tier === 3);
    check('T003c: fault entry tier3Recovery === true', sim.faultLog[0].tier3Recovery === true);
    check('T003d: _returnToBoot was called', tier3Called === true);
    check('T003e: irqActive cleared after Tier 3 recovery', !sim.irqState.irqActive);
    check('T003f: machine is NOT permanently halted (Tier 3 resets to boot)', !sim.halted);
}

// ── T004: pause timer fire ────────────────────────────────────────────────────
console.log('\n--- T004: Scheduler.pause timer fire ---');
{
    const { sim, registry, sysAbs } = makeTestSim();
    const DURATION = 5;
    const stepAtCall = sim.stepCount;

    // Invoke Scheduler.pause(duration=5) via the registry
    const pauseResult = registry.dispatchMethod(8, 'pause', sim, { duration: DURATION });

    check('T004a: Scheduler.pause — ok=true', pauseResult && pauseResult.ok === true);
    check('T004b: Scheduler.pause — irqState.timerArmed=true', sim.irqState.timerArmed === true);
    check('T004c: Scheduler.pause — timerDeadline = stepAtCall + DURATION',
        sim.irqState.timerDeadline === stepAtCall + DURATION);
    check('T004d: Scheduler.pause — timerRegs[4] = 1 (CTL armed)', sim.timerRegs[4] === 1);
    check('T004e: Scheduler.pause — timerRegs[3] = deadline (ALARM_CMP)',
        sim.timerRegs[3] === stepAtCall + DURATION);

    // Advance stepCount to the deadline so the timer fires
    sim.stepCount = sim.irqState.timerDeadline;

    // _fireSchedulerIRQ('TIMER') should now succeed and clear the alarm
    const swept = sysAbs._schedulerState._irqSweepCount;
    const fired = sim._fireSchedulerIRQ('TIMER', null);

    check('T004f: _fireSchedulerIRQ(TIMER) returned true', fired === true);
    check('T004g: timerArmed cleared to false after IRQ fires', !sim.irqState.timerArmed);
    check('T004h: irqSweepCount incremented', sysAbs._schedulerState._irqSweepCount === swept + 1);
    check('T004i: irqActive is false after TIMER IRQ completes', !sim.irqState.irqActive);
}

// ── T005: Wait(flag) + signal + IRQ sweep wake ────────────────────────────────
// Uses the production Scheduler.Wait() path to suspend a thread on a flag,
// then signals the flag via pendingWakeFlags and fires Scheduler.IRQ to sweep.
console.log('\n--- T005: Scheduler.Wait(flag) / signal / wake ---');
{
    const { sim, registry, sysAbs } = makeTestSim();

    // Add a second thread that will call Wait('startFlag')
    sysAbs._schedulerState.threads.push({ id: 1, state: 'running', name: 'waiter' });
    sysAbs._schedulerState.currentThread = 1;

    // Call Scheduler.Wait('startFlag') via the production registry binding
    const waitResult = registry.dispatchMethod(8, 'Wait', sim, { flag: 'startFlag' });

    check('T005a: Scheduler.Wait — ok=true', waitResult && waitResult.ok === true);
    const waiterBefore = sysAbs._schedulerState.threads.find(t => t.id === 1);
    check('T005b: Scheduler.Wait — thread state = sleeping', waiterBefore && waiterBefore.state === 'sleeping');
    check('T005c: Scheduler.Wait — thread.waitFlag = startFlag', waiterBefore && waiterBefore.waitFlag === 'startFlag');
    check('T005d: Scheduler.Wait — irqState.waitingOnFlags[1] = startFlag',
        sim.irqState.waitingOnFlags['1'] === 'startFlag');

    // An external event signals 'startFlag': enqueue it in pendingWakeFlags
    sim.irqState.pendingWakeFlags = ['startFlag'];

    // Fire Scheduler.IRQ with reason=TIMER — sweep should wake the waiting thread
    const irqResult = registry.dispatchMethod(8, 'IRQ', sim, {
        reason: 'TIMER', faultRecord: null, savedContext: null
    });

    check('T005e: IRQ dispatch ok=true', irqResult && irqResult.ok === true);
    const waiterAfter = sysAbs._schedulerState.threads.find(t => t.id === 1);
    check('T005f: thread woken to ready after flag signal', waiterAfter && waiterAfter.state === 'ready');
    check('T005g: waitFlag cleared from thread', waiterAfter && !waiterAfter.waitFlag);
    check('T005h: irqState.waitingOnFlags[1] cleared after wake', !sim.irqState.waitingOnFlags['1']);
    check('T005i: pendingWakeFlags cleared (startFlag consumed)',
        !(sim.irqState.pendingWakeFlags || []).includes('startFlag'));
    check('T005j: irqSweepCount incremented', sysAbs._schedulerState._irqSweepCount >= 1);
}

// ── T006: Structured fault record fields (baseline — unhandled fault) ─────────
console.log('\n--- T006: Structured fault record (unhandled) ---');
{
    const { sim } = makeTestSim();
    sim.fault('BOUNDS', 'Test structured record fields (no recovery)');

    const entry = sim.faultLog[0];
    check('T006a: faultLog has one entry', sim.faultLog.length === 1);
    check('T006b: faultCode = BOUNDS code', entry.faultCode === ChurchSimulator.FAULT_CODES.BOUNDS);
    check('T006c: tier is null (all tiers failed — halt)', entry.tier === null);
    check('T006d: catchInvoked=false', entry.catchInvoked === false);
    check('T006e: irqInvoked=true (Scheduler.IRQ always dispatched before halt)', entry.irqInvoked === true);
    check('T006f: tier3Recovery=false', entry.tier3Recovery === false);
    check('T006g: machine halted', sim.halted === true);
    check('T006h: ChurchSimulator.SCHEDULER_NS_SLOT === 8',
        ChurchSimulator.SCHEDULER_NS_SLOT === 8);
    check('T006i: ChurchSimulator.SCHEDULER_IRQ_NS_SLOT === 50',
        ChurchSimulator.SCHEDULER_IRQ_NS_SLOT === 50);
    check('T006j: FAULT_CODES object has PERM_R key',
        ChurchSimulator.FAULT_CODES.PERM_R === 0x01);
    check('T006k: FAULT_CODES object has NULL_CAP key',
        ChurchSimulator.FAULT_CODES.NULL_CAP === 0x07);
}

// ── T007: .catch throws — catchInvoked=true, escalation continues ────────────
// Verifies that catchInvoked is true even when the .catch handler throws,
// and that the fault escalates to Tier 2/halt rather than being silently dropped.
console.log('\n--- T007: .catch throws → catchInvoked=true, escalation ---');
{
    const { sim, registry } = makeTestSim();

    // Bind a .catch method on NS slot 10 that throws
    registry.abstractions[10] = registry.abstractions[10] || { dispatch: {} };
    registry.abstractions[10].dispatch['.CATCH'] = function(_sim, _args) {
        throw new Error('deliberate .catch failure for T007');
    };

    // Simulate CR14 pointing to NS slot 10 so fault() resolves faultingAbstractionSlot=10
    sim.cr[14] = { word0: 10 };
    // No faultRecoveryHandler → Tier 2 also fails → halt
    sim.fault('PERM_R', 'T007 test — .catch throws');

    const entry = sim.faultLog[0];
    check('T007a: faultLog has one entry', sim.faultLog.length === 1);
    check('T007b: catchInvoked=true (handler was called even though it threw)',
        entry.catchInvoked === true);
    check('T007c: irqInvoked=true (escalated to Scheduler.IRQ after .catch threw)',
        entry.irqInvoked === true);
    check('T007d: tier is null (all tiers exhausted — halt)', entry.tier === null);
    check('T007e: machine halted', sim.halted === true);
    check('T007f: output mentions .catch threw', sim.output.includes('threw'));
}

// ── Summary ───────────────────────────────────────────────────────────────────
console.log(`\n${pass} passed, ${fail} failed`);
if (fail > 0) process.exit(1);
