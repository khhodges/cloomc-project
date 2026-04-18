// Headless harness used by tests/test_boot_image_matches_simulator.py.
//
// Reads a boot-config JSON from stdin, instantiates ChurchSimulator (which
// runs reset() → _initNamespaceTable()), and writes the resulting memory[]
// as a raw little-endian Uint32Array binary to stdout.
//
// Usage:  echo '{"step1":{...},"step2":{...},"step3":{...}}' | node tests/sim_init_dump.js
//
// The harness sets `global.window = { bootConfig }` before requiring the
// simulator so the same window.bootConfig branches the simulator uses in
// the IDE are exercised here.

let cfgRaw = '';
process.stdin.setEncoding('utf8');
process.stdin.on('data', (c) => { cfgRaw += c; });
process.stdin.on('end', () => {
    const cfg = cfgRaw.trim() ? JSON.parse(cfgRaw) : {};
    global.window = { bootConfig: cfg };

    const ChurchSimulator = require('../simulator/simulator.js');
    const sim = new ChurchSimulator();    // reset() → _initNamespaceTable()

    // memory is a Uint32Array; emit the raw underlying bytes.
    const buf = Buffer.from(sim.memory.buffer, sim.memory.byteOffset,
                            sim.memory.byteLength);
    process.stdout.write(buf);
});
