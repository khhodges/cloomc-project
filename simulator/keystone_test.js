// keystone_test.js — unit tests for Keystone identity validation and FAULT_NO_CONTACT
//
// Covers the _initKeystone() logic in system_abstractions.js:
//   - Connect(0):           zero word rejected (result=0, "AM rejected")
//   - Connect(0x20001234):  unknown version tag 0x2 rejected (result=0)
//   - Hello() (no Connect): no contact established → FAULT_NO_CONTACT (0xDEAD0001)
//   - Connect(0x10001234) then Hello(): valid identity → GREET_RESPONSE (0x48454C4C)
//
// Run with: node simulator/keystone_test.js

'use strict';

const SystemAbstractions = require('./system_abstractions.js');

// ── Minimal test harness ──────────────────────────────────────────────────────

let passed = 0;
let failed = 0;

function assert(label, condition, detail) {
    if (condition) {
        console.log('PASS ' + label);
        passed++;
    } else {
        console.log('FAIL ' + label + (detail !== undefined ? ' — ' + detail : ''));
        failed++;
    }
}

// ── Minimal registry ──────────────────────────────────────────────────────────
//
// SystemAbstractions only calls registry.bindMethod(nsIndex, methodName, fn)
// during construction.  We store each bound function and expose a call() helper.

function makeRegistry() {
    const methods = {};  // { nsIndex: { METHODNAME: fn } }
    return {
        bindMethod(index, methodName, fn) {
            if (!methods[index]) methods[index] = {};
            methods[index][methodName.toUpperCase()] = fn;
        },
        call(index, methodName, sim, args) {
            const ns = methods[index] || {};
            const fn = ns[methodName.toUpperCase()];
            if (!fn) return { ok: false, fault: 'METHOD', message: `${methodName} not bound at NS[${index}]` };
            return fn(sim, args);
        }
    };
}

// ── Minimal sim for zero/version-rejection tests ──────────────────────────────
//
// Connect returns early before touching sim when identityWord is 0 or has a
// bad version tag — no sim state is accessed.

const nullSim = {};

// ── Minimal sim for Hello() / full round-trip tests ───────────────────────────
//
// We allocate a small memory array and set up a fake NS entry for slot 32
// (KEYSTONE_NS) so that Connect can store the minted GT and Hello can read it.

function makeKeystoneSim() {
    const WORD0_LOC  = 1000;   // arbitrary lump base address in memory[]
    const LUMP_SIZE  = 64;     // must match parseLumpHeader return below
    const CC         = 4;      // c-list capacity; clistBase = 1000 + 64 - 4 = 1060
    const CLIST_BASE = WORD0_LOC + LUMP_SIZE - CC;  // 1060

    const memory = new Uint32Array(4096);
    const nsClistMap = {};

    return {
        memory,
        nsClistMap,

        // Return a fake NS entry for slot 32; null for any other slot.
        readNSEntry(idx) {
            if (idx !== 32) return null;
            return { word0_location: WORD0_LOC };
        },

        // Return a fixed header so clistBase is predictable.
        parseLumpHeader(_word) {
            return { lumpSize: LUMP_SIZE, cc: CC, valid: true };
        },

        // Produce a distinctive non-zero GT for Keystone.
        createGT(_token, _index, _perms, _type) {
            return 0xAB000020;  // non-zero sentinel
        }
    };
}

// ── Instantiate SystemAbstractions (binds all methods into our registry) ──────

const registry = makeRegistry();
const _sa = new SystemAbstractions(registry);   // side-effect: binds Keystone methods

// ── Test cases ────────────────────────────────────────────────────────────────

// T1: Connect(0) — zero identity word must be rejected.
{
    const r = registry.call(32, 'Connect', nullSim, [0]);
    assert('T1 Connect(0) returns result=0',
        r.result === 0,
        'got result=' + r.result);
    assert('T1 Connect(0) message mentions AM rejected',
        typeof r.message === 'string' && r.message.includes('AM rejected'),
        'got message: ' + r.message);
}

// T2: Connect(0x20001234) — version tag 0x2 is unknown; must be rejected.
{
    const r = registry.call(32, 'Connect', nullSim, [0x20001234]);
    assert('T2 Connect(0x20001234) returns result=0',
        r.result === 0,
        'got result=' + r.result);
    assert('T2 Connect(0x20001234) message mentions AM rejected',
        typeof r.message === 'string' && r.message.includes('AM rejected'),
        'got message: ' + r.message);
}

// T3: Hello() before Connect() — c-list slot 1 is NULL GT → FAULT_NO_CONTACT.
{
    const sim = makeKeystoneSim();
    // No Connect called; memory[clistBase+1] is 0 (Uint32Array default).
    const r = registry.call(32, 'Hello', sim, []);
    assert('T3 Hello() (no Connect) returns result=0xDEAD0001',
        (r.result >>> 0) === 0xDEAD0001,
        'got result=0x' + (r.result >>> 0).toString(16));
    assert('T3 Hello() (no Connect) fault is NO_CONTACT',
        r.fault === 'NO_CONTACT',
        'got fault=' + r.fault);
}

// T4: Connect(0x10001234) then Hello() — valid tag 0x1; Hello must return GREET_RESPONSE.
{
    const sim = makeKeystoneSim();

    const connectResult = registry.call(32, 'Connect', sim, [0x10001234]);
    assert('T4 Connect(0x10001234) succeeds (result=1)',
        connectResult.result === 1,
        'got result=' + connectResult.result);

    const helloResult = registry.call(32, 'Hello', sim, []);
    assert('T4 Hello() after Connect returns result=0x48454C4C',
        (helloResult.result >>> 0) === 0x48454C4C,
        'got result=0x' + (helloResult.result >>> 0).toString(16));
    assert('T4 Hello() after Connect has ok=true',
        helloResult.ok === true,
        'got ok=' + helloResult.ok);
}

// ── Summary ───────────────────────────────────────────────────────────────────

console.log('');
console.log(`${passed} passed, ${failed} failed`);
if (failed > 0) process.exit(1);
