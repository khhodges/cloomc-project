// Headless harness for GT-validation tests.
//
// Reads a JSON envelope from stdin:
//   {
//     "tests": [
//       { "name": "...", "action": "createGT", "args": [gt_seq, slotId, perms, type] },
//       { "name": "...", "action": "boot" }
//     ]
//   }
//
// For each test:
//   - action "createGT": calls sim.createGT(...args); records whether it threw
//     and the error message.
//   - action "boot": instantiates a fresh sim, runs _bootStep() until complete
//     or halted; reports bootComplete and any faultLog entries.
//
// Emits a JSON array of result objects on stdout.

const ChurchSimulator = require('../../simulator/simulator.js');

let raw = '';
process.stdin.setEncoding('utf8');
process.stdin.on('data', (c) => { raw += c; });
process.stdin.on('end', () => {
    const env = JSON.parse(raw);
    const results = [];

    for (const t of env.tests) {
        if (t.action === 'createGT') {
            const sim = new ChurchSimulator();
            let threw = false;
            let errorMessage = null;
            let value = null;
            try {
                const [gt_seq, slotId, perms, type] = t.args;
                value = sim.createGT(gt_seq, slotId, perms, type);
            } catch (e) {
                threw = true;
                errorMessage = e.message;
            }
            results.push({ name: t.name, threw, errorMessage, value });

        } else if (t.action === 'boot') {
            global.window = { bootConfig: {} };
            const sim = new ChurchSimulator();
            const MAX_STEPS = 64;
            let iters = 0;
            while (iters < MAX_STEPS && !sim.bootComplete && !sim.halted) {
                const advanced = sim._bootStep();
                iters++;
                if (!advanced) break;
            }
            // Collect any createGT-related faults (thrown errors show as halted
            // because the simulator catches them into faultLog).
            const gtErrors = (sim.faultLog || []).filter(f =>
                f.message && (f.message.includes('domain-impure') || f.message.includes('single-Church-perm'))
            );
            results.push({
                name: t.name,
                bootComplete: sim.bootComplete === true,
                halted: sim.halted === true,
                faultLog: (sim.faultLog || []).map(f => ({ type: f.type, message: f.message })),
                gtErrors,
            });
        }
    }

    process.stdout.write(JSON.stringify(results));
});
