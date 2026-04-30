// Full Hello-Mum flow harness — with live HTTP bridge.
//
// Reads a JSON envelope from stdin:
//   {
//     "imageBase64":  "<base64-encoded 32-bit LE boot image>",
//     "config":       { ... bootConfig object ... },
//     "identityWord": <32-bit unsigned integer, optional>,
//     "bridgeUrl":    "http://host:port"   (optional; enables live HTTP bridge)
//   }
//
// When "bridgeUrl" is supplied the harness rebinds Tunnel.Call (NS[31].Call)
// in the AbstractionRegistry to make a real synchronous HTTP POST to
// <bridgeUrl>/mum/hello before Keystone.Hello() is dispatched.  This means
// the result that Keystone.Hello() returns is derived from the live server
// response — not from an in-process constant.  The full causal chain is:
//
//   boot image
//     → ChurchSimulator (real AbstractionRegistry + SystemAbstractions)
//     → boot state machine (_bootStep loop)
//     → Navana.Init → Keystone.Init wires Tunnel E-GT into c-list slot 0
//     → Keystone.Connect(identityWord) → MumGT in c-list slot 1
//     → Tunnel.Call rebound → curl -X POST <bridgeUrl>/mum/hello  ← live call
//     → Keystone.Hello() → dispatches Tunnel.Call(mumGT)
//     → response.result (0x48454C4C) returned to caller
//
// Without "bridgeUrl" the harness uses the in-process SystemAbstractions
// Tunnel.Call implementation (useful for smoke tests that don't need a server).
//
// Emits a single JSON line on stdout:
//   {
//     ok, greetResult, greetHex,
//     bridgeHit,        // true iff the live HTTP bridge was invoked
//     bridgeStatus,     // HTTP status code returned by /mum/hello (or null)
//     bridgeResult,     // "result" field from the bridge JSON response (or null)
//     slot0, slot1, tunnelNS, eBitSet,
//     bootComplete, faultCount, loaded,
//     navanaOk, connectOk,
//     message
//   }
//
// Exit codes:
//   0 — Hello() returned GREET_RESPONSE (0x48454C4C)
//   1 — any step failed; see JSON output for details
//
// Used by test_hello_mum_e2e.py::test_hello_mum_full_chain_on_real_boot_image.

'use strict';

const spawnSync = require('child_process').spawnSync;

const KEYSTONE_NS    = 32;
const TUNNEL_NS      = 31;
const NAVANA_NS      = 5;
const GREET_RESPONSE = 0x48454C4C;  // 'HELL' in big-endian ASCII

const DEFAULT_IDENTITY = (0x1 << 28) | 0x0CEEFFE;

// ---------------------------------------------------------------------------
// Synchronous HTTP POST via curl (no external npm dependencies).
// Returns { status, data } where data is the parsed JSON body, or throws.
// ---------------------------------------------------------------------------
function curlPost(url, body) {
    const payload = JSON.stringify(body || {});
    const res = spawnSync(
        'curl',
        ['-s', '-w', '\n%{http_code}', '-X', 'POST',
         '-H', 'Content-Type: application/json',
         '-d', payload, url],
        { encoding: 'utf8', timeout: 10000 }
    );
    if (res.error) throw new Error(`curl error: ${res.error.message}`);
    const out   = (res.stdout || '').trimEnd();
    const nl    = out.lastIndexOf('\n');
    const body2 = out.substring(0, nl);
    const code  = parseInt(out.substring(nl + 1), 10);
    let data = {};
    try { data = JSON.parse(body2); } catch (_) { /* non-JSON response */ }
    return { status: code, data };
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

let raw = '';
process.stdin.setEncoding('utf8');
process.stdin.on('data', (c) => { raw += c; });
process.stdin.on('end', () => {
    let env;
    try {
        env = JSON.parse(raw);
    } catch (e) {
        process.stdout.write(JSON.stringify({
            ok: false,
            message: `stdin parse error: ${e.message}`
        }) + '\n');
        process.exit(1);
    }

    const cfg          = env.config       || {};
    const imgBuf       = Buffer.from(env.imageBase64 || '', 'base64');
    const identityWord = (env.identityWord !== undefined)
        ? (env.identityWord >>> 0)
        : DEFAULT_IDENTITY;
    const bridgeUrl    = env.bridgeUrl    || null;   // e.g. "http://127.0.0.1:5001"

    global.window = { bootConfig: cfg };

    const AbstractionRegistry = require('../../simulator/abstractions.js');
    const SystemAbstractions  = require('../../simulator/system_abstractions.js');
    const ChurchSimulator     = require('../../simulator/simulator.js');

    const registry = new AbstractionRegistry();
    new SystemAbstractions(registry);

    const sim = new ChurchSimulator();
    sim.initAbstractions(registry, null, null);

    // --- Step 1: load boot image ---
    sim.memory.fill(0);
    const ab     = imgBuf.buffer.slice(imgBuf.byteOffset, imgBuf.byteOffset + imgBuf.byteLength);
    const loaded = sim.loadBootImage(ab);

    // --- Step 2: boot state machine ---
    const MAX_BOOT_STEPS = 64;
    let iters = 0;
    while (iters < MAX_BOOT_STEPS && !sim.bootComplete && !sim.halted) {
        const advanced = sim._bootStep();
        iters++;
        if (!advanced) break;
    }

    const bootComplete = sim.bootComplete === true;
    const faultCount   = (sim.faultLog || []).length;

    if (!bootComplete) {
        process.stdout.write(JSON.stringify({
            ok: false, loaded: loaded === true, bootComplete, faultCount,
            message: 'Boot state machine did not complete — aborting Hello-Mum flow'
        }) + '\n');
        process.exit(1);
    }

    // --- Step 3: Navana.Init → wires Keystone c-list slot 0 with Tunnel E-GT ---
    const navanaResult = sim.abstractionRegistry.dispatchMethod(NAVANA_NS, 'Init', sim, {});
    const navanaOk     = !!(navanaResult && navanaResult.ok);

    if (!navanaOk) {
        process.stdout.write(JSON.stringify({
            ok: false, loaded: loaded === true, bootComplete, faultCount, navanaOk,
            message: `Navana.Init failed: ${navanaResult ? navanaResult.message : 'null result'}`
        }) + '\n');
        process.exit(1);
    }

    // --- Step 4: Keystone.Connect(identityWord) → MumGT placed in c-list slot 1 ---
    const connectResult = sim.abstractionRegistry.dispatchMethod(
        KEYSTONE_NS, 'Connect', sim, [identityWord]
    );
    const connectOk = !!(connectResult && connectResult.ok && connectResult.result === 1);

    if (!connectOk) {
        process.stdout.write(JSON.stringify({
            ok: false, loaded: loaded === true, bootComplete, faultCount, navanaOk, connectOk,
            connectResult: connectResult ? connectResult.result : null,
            message: `Keystone.Connect failed: ${connectResult ? connectResult.message : 'null result'}`
        }) + '\n');
        process.exit(1);
    }

    // --- Step 4b: Rebind Tunnel.Call to use the live HTTP bridge ---
    // The synchronous rebinding must happen *before* Keystone.Hello() is
    // dispatched.  AbstractionRegistry.bindMethod stores the function at
    // abstractions[ns].dispatch['CALL']; dispatchMethod resolves and calls
    // it synchronously.  We use spawnSync(curl) to keep the call synchronous
    // so that Keystone.Hello() — which is also synchronous — can receive the
    // result normally.

    let bridgeHit    = false;
    let bridgeStatus = null;
    let bridgeResult = null;
    let bridgeErr    = null;

    if (bridgeUrl) {
        sim.abstractionRegistry.bindMethod(TUNNEL_NS, 'Call', function(s, args) {
            const mumGT = (args && args.cr2 !== undefined) ? (args.cr2 >>> 0) : 0;
            if (!mumGT) {
                return { ok: false, result: 0xDEAD0001, fault: 'NO_CONTACT',
                         message: 'Tunnel.Call(bridge): cr2 is NULL GT' };
            }
            let resp;
            try {
                resp = curlPost(`${bridgeUrl}/mum/hello`, {});
            } catch (err) {
                bridgeErr = err.message;
                return { ok: false, result: 0xDEAD0002, fault: 'TUNNEL_OFFLINE',
                         message: `Tunnel.Call(bridge): HTTP error: ${err.message}` };
            }
            bridgeHit    = true;
            bridgeStatus = resp.status;
            bridgeResult = (resp.data.result !== undefined)
                ? (resp.data.result >>> 0)
                : null;
            if (resp.status !== 200 || !resp.data.ok) {
                return { ok: false, result: 0xDEAD0002, fault: 'TUNNEL_OFFLINE',
                         message: `Tunnel.Call(bridge): /mum/hello returned HTTP ${resp.status}` };
            }
            return {
                ok:      true,
                result:  bridgeResult,
                message: `Tunnel.Call(bridge): /mum/hello \u2192 0x${(bridgeResult || 0).toString(16).toUpperCase().padStart(8, '0')}`
            };
        });
    }

    // --- Step 5: Keystone.Hello() — primary assertion ---
    const helloResult  = sim.abstractionRegistry.dispatchMethod(KEYSTONE_NS, 'Hello', sim, []);
    const greetResult  = helloResult ? (helloResult.result >>> 0) : 0;
    const greetHex     = `0x${greetResult.toString(16).toUpperCase().padStart(8, '0')}`;
    const ok           = greetResult === GREET_RESPONSE;

    // Diagnostics: read c-list state.
    let slot0 = 0; let slot1 = 0; let tunnelNS = 0; let eBitSet = false;
    const entry = sim.readNSEntry(KEYSTONE_NS);
    if (entry) {
        const hdr       = sim.parseLumpHeader(sim.memory[entry.word0_location]);
        const clistBase = entry.word0_location + hdr.lumpSize - hdr.cc;
        slot0    = sim.memory[clistBase + 0] >>> 0;
        slot1    = sim.memory[clistBase + 1] >>> 0;
        tunnelNS = slot0 & 0xFFFF;
        eBitSet  = ((slot0 >>> 30) & 1) === 1;
    }

    process.stdout.write(JSON.stringify({
        ok,
        greetResult,
        greetHex,
        bridgeHit,
        bridgeStatus,
        bridgeResult,
        bridgeErr,
        slot0, slot1, tunnelNS, eBitSet,
        bootComplete,
        faultCount,
        loaded:     loaded === true,
        navanaOk,
        connectOk,
        message: ok
            ? `Keystone.Hello() \u2192 ${greetHex} ('HELL') \u2014 Hello-Mum flow complete${bridgeHit ? ' (live bridge)' : ' (in-process)'}`
            : `Keystone.Hello() returned ${greetHex}, expected 0x${GREET_RESPONSE.toString(16).toUpperCase().padStart(8,'0')} \u2014 ${helloResult ? helloResult.message : 'null result'}${bridgeErr ? ` [bridgeErr: ${bridgeErr}]` : ''}`
    }) + '\n');
    process.exit(ok ? 0 : 1);
});
