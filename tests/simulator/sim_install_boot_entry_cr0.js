// Headless harness used by tests/simulator/test_install_boot_entry_cr0.py.
//
// Loads _installBootEntryGTIntoCR0() from the production file
// simulator/app-memory.js via Node's vm module, supplying minimal stubs for
// the browser globals the file references.  This ensures the harness exercises
// the real implementation — not a copy — so any regression in app-memory.js
// will be caught by the Python test suite.
//
// Prints a single JSON object to stdout with keys:
//   returned       bool   — return value of _installBootEntryGTIntoCR0()
//   simOutputLine  string — the full log line extracted from sim.output
//   consoleText    string — editorConsole.textContent after the call
//   slot           number — slot number parsed from the log line
//   gtHexInLog     string — hex GT word parsed from the log line
//   gtHexExpected  string — hex GT word computed independently for verification
//   bootSlot       number — sim.bootEntrySlot at call time
//   fnSource       string — first 60 chars of the function source (proves it
//                           originates from the loaded file, not a local copy)

'use strict';

const fs   = require('fs');
const path = require('path');
const vm   = require('vm');

// ── Bootstrap the simulator ──────────────────────────────────────────────────
global.window = {
    bootConfig: {
        step1: {
            totalNamespaceWords: 16384,
            namespaceLumpWords:     64,
            threadLumpWords:       256,
        }
    }
};

const ChurchSimulator = require('../../simulator/simulator.js');
const sim = new ChurchSimulator();

if (sim.bootEntrySlot === null || sim.bootEntrySlot === undefined) {
    process.stderr.write('ERROR: sim.bootEntrySlot is not set after ChurchSimulator construction\n');
    process.exit(1);
}
const bootSlot = sim.bootEntrySlot;

// ── Mock editorConsole DOM element ───────────────────────────────────────────
const _consoleMock = {
    textContent: '',
    scrollTop: 0,
    scrollHeight: 0,
};

// ── Build a VM sandbox that provides the browser globals app-memory.js needs ─
// We wire sim and nsExpandedSlot directly into the sandbox so the loaded
// _installBootEntryGTIntoCR0() operates on the real ChurchSimulator instance.
const sandbox = {
    // Core globals needed by app-memory.js at load time / function bodies
    window: global.window,
    localStorage: { getItem: () => null, setItem: () => {}, removeItem: () => {} },
    document: {
        getElementById(id) {
            if (id === 'editorConsole') return _consoleMock;
            return null;
        },
        createElement: (t) => ({
            style: {},
            addEventListener: () => {},
            innerHTML: '',
            textContent: '',
            value: '',
        }),
        body: { appendChild: () => {} },
        querySelectorAll: () => [],
    },
    console,
    setTimeout:    () => {},
    clearTimeout:  () => {},
    setInterval:   () => {},
    clearInterval: () => {},
    fetch:         () => Promise.resolve({}),
    alert:         () => {},
    confirm:       () => false,

    // Simulator state — the production function reads/writes these
    sim,
    nsExpandedSlot: -1,   // -1 = no secondary thread slot in view

    // Optional UI callbacks referenced by the function (must be null/undefined
    // or a function — not missing, to avoid ReferenceError)
    updateCRDetail: null,
    updateNamespace: null,

    // Other globals referenced by functions elsewhere in app-memory.js
    selectedCR: null,
    selectedNS: null,
    editorMode: null,
    showPatchModal: null,
    renderLumps: null,
};

// ── Load app-memory.js into the sandbox ──────────────────────────────────────
const appMemoryPath = path.resolve(__dirname, '../../simulator/app-memory.js');
const appMemoryCode = fs.readFileSync(appMemoryPath, 'utf8');

vm.createContext(sandbox);
vm.runInContext(appMemoryCode, sandbox);

// Verify we got the real function from the loaded file.
if (typeof sandbox._installBootEntryGTIntoCR0 !== 'function') {
    process.stderr.write('ERROR: _installBootEntryGTIntoCR0 not found in app-memory.js sandbox\n');
    process.exit(1);
}

// Capture the start of the function source as evidence it came from the file.
const fnSource = sandbox._installBootEntryGTIntoCR0.toString().slice(0, 60);

// ── Invoke the real function ──────────────────────────────────────────────────
sim.output = '';   // clear so the log line is easy to extract
const returned = sandbox._installBootEntryGTIntoCR0();

// ── Extract results ───────────────────────────────────────────────────────────
const outputLines  = sim.output.split('\n').filter(l => l.length > 0);
const logLine      = outputLines.find(l => l.startsWith('[IDE] CR0')) || '';

const slotMatch    = logLine.match(/E-GT\(Slot (\d+)\)/);
const parsedSlot   = slotMatch ? parseInt(slotMatch[1], 10) : -1;

const hexMatch     = logLine.match(/0x[0-9A-Fa-f]{8}/);
const gtHexInLog   = hexMatch ? hexMatch[0] : '';

// Compute the expected GT word independently so the test can cross-check the hex.
const expectedGtWord = sim.createGT(0, bootSlot, {E:1}, 1);
const expectedGtHex  = '0x' + (expectedGtWord >>> 0).toString(16).toUpperCase().padStart(8, '0');

const out = {
    returned,
    simOutputLine: logLine,
    consoleText:   _consoleMock.textContent,
    slot:          parsedSlot,
    gtHexInLog,
    gtHexExpected: expectedGtHex,
    bootSlot,
    fnSource,
};

process.stdout.write(JSON.stringify(out) + '\n');
