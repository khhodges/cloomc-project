// Headless harness for testing Abstract GT I/O dispatch (UART, Button, Timer).
// Used by tests/test_uart_button_timer_dispatch.py.
//
// Reads a JSON request from stdin:
//   { "ops": [ { "op": "DREAD"|"DWRITE", "device_class": N, "device_data": N,
//               "dr_value": N, "dr_idx": N, "cr_idx": N, "expect_fault": false } ] }
//
// Returns a JSON array of results to stdout:
//   [ { "ok": true, "dr1": N, "fault": null } | { "ok": false, "fault": "PERM_R"|... } ]
//
// Usage:
//   echo '<json>' | node tests/sim_dispatch_abstract_io.js

'use strict';

let raw = '';
process.stdin.setEncoding('utf8');
process.stdin.on('data', (c) => { raw += c; });
process.stdin.on('end', () => {
    const req = JSON.parse(raw);
    global.window = { bootConfig: {} };

    const ChurchSimulator = require('../simulator/simulator.js');
    const sim = new ChurchSimulator();

    const AB_TYPE_IO = ChurchSimulator.AB_TYPE_IO;

    const results = [];

    for (const op of req.ops) {
        // Fresh device state per op so state doesn't bleed between tests.
        sim.uartRegs    = [0, 0, 0];
        sim.buttonState = 0;
        sim.timerRegs   = [0, 0, 0, 0, 0];
        sim.halted      = false;
        sim.faultLog    = [];

        // Build Abstract GT word
        const ab_data = ((op.device_class & 0xFF) << 8) | (op.device_data & 0xFF);
        const ab_type = AB_TYPE_IO;
        const rBit = op.r_perm !== undefined ? op.r_perm : 1;
        const wBit = op.w_perm !== undefined ? op.w_perm : 1;
        const gt32 = sim.createAbstractGT(ab_type, { R: rBit, W: wBit }, 0, ab_data);

        // Load GT into CR at cr_idx (as a 3-word capability register).
        const crIdx = op.cr_idx !== undefined ? op.cr_idx : 5;
        const drIdx = op.dr_idx !== undefined ? op.dr_idx : 1;

        // Directly set the capability register's GT word (word0).
        sim.cr[crIdx] = { word0: gt32, word1: 0, word2: 0 };

        // For DWRITE: set DR[drIdx] to the value to write.
        if (op.op === 'DWRITE' && op.dr_value !== undefined) {
            sim.dr[drIdx] = op.dr_value >>> 0;
        }
        // For seeding device state before DREAD:
        if (op.seed_uart)    sim.uartRegs    = op.seed_uart.slice();
        if (op.seed_button !== undefined) sim.buttonState = op.seed_button;
        if (op.seed_timer)   sim.timerRegs   = op.seed_timer.slice();

        let result;
        try {
            if (op.op === 'DREAD') {
                result = sim._dispatchAbstractDread(crIdx, drIdx, 0);
            } else {
                result = sim._dispatchAbstractDwrite(crIdx, drIdx, 0);
            }
        } catch (e) {
            results.push({ ok: false, error: e.message });
            continue;
        }

        if (sim.halted) {
            const lastFault = sim.faultLog && sim.faultLog.length > 0
                ? sim.faultLog[sim.faultLog.length - 1] : null;
            results.push({ ok: false, fault: lastFault ? lastFault.type : 'UNKNOWN' });
        } else {
            const dr1 = sim.dr[drIdx] >>> 0;
            results.push({
                ok: true,
                dr_value: dr1,
                uart_regs:    sim.uartRegs.slice(),
                button_state: sim.buttonState,
                timer_regs:   sim.timerRegs.slice(),
            });
        }
    }

    process.stdout.write(JSON.stringify(results) + '\n');
});
