// Headless harness used by tests/simulator/test_tab_correction.py.
//
// Verifies correctCRDetailTab() resets crDetailTab to the correct value for
// every invalid (tab, CR-capability) combination that the old inline block
// silently broke.
//
// Coverage:
//   1. 'api' tab on a non-code CR  → 'clist', 'register', or 'lump'
//   2. 'code' tab on a non-code CR → 'clist', 'register', or 'lump'
//   3. 'register' tab on code-only CR (showData=false) → 'code', 'clist', or 'lump'
//   4. 'binary' tab on code-only CR (showData=false)   → 'code', 'clist', or 'lump'
//   5. 'clist' tab on CR without C-List → 'code', 'register', or 'lump'
//   6. Already-valid tabs stay unchanged
//
// Exits 0 on success, 1 on failure (failures written to stderr).

'use strict';

const { correctCRDetailTab } = require('../../simulator/tab-correction.js');

const ERRORS = [];
function fail(label, msg) {
    ERRORS.push(`[FAIL] ${label}: ${msg}`);
    process.stderr.write(`[FAIL] ${label}: ${msg}\n`);
}
function pass(label) {
    process.stdout.write(`[PASS] ${label}\n`);
}
function check(label, got, expected) {
    if (got !== expected) {
        fail(label, `got '${got}', expected '${expected}'`);
    } else {
        pass(label);
    }
}

// ─── Rule 1 & 2: 'api' / 'code' on a non-code CR ─────────────────────────────

(function testApiOnNonCodeWithCList() {
    // showCode=false, showCList=true, showData=false → 'clist'
    check("'api' tab, non-code CR with C-List → 'clist'",
        correctCRDetailTab('api', false, true, false), 'clist');
})();

(function testApiOnNonCodeWithData() {
    // showCode=false, showCList=false, showData=true → 'register'
    check("'api' tab, non-code CR with data (no clist) → 'register'",
        correctCRDetailTab('api', false, false, true), 'register');
})();

(function testApiOnNonCodeNoCapabilities() {
    // showCode=false, showCList=false, showData=false → 'lump'
    check("'api' tab, non-code CR with no capabilities → 'lump'",
        correctCRDetailTab('api', false, false, false), 'lump');
})();

(function testCodeOnNonCodeWithCList() {
    check("'code' tab, non-code CR with C-List → 'clist'",
        correctCRDetailTab('code', false, true, false), 'clist');
})();

(function testCodeOnNonCodeWithData() {
    check("'code' tab, non-code CR with data (no clist) → 'register'",
        correctCRDetailTab('code', false, false, true), 'register');
})();

(function testCodeOnNonCodeNoCapabilities() {
    check("'code' tab, non-code CR with no capabilities → 'lump'",
        correctCRDetailTab('code', false, false, false), 'lump');
})();

// ─── Rule 3 & 4: 'register' / 'binary' on a code-only CR ─────────────────────

(function testRegisterOnCodeOnlyCR() {
    // showCode=true, showCList=false, showData=false → 'code'
    check("'register' tab, code-only CR → 'code'",
        correctCRDetailTab('register', true, false, false), 'code');
})();

(function testRegisterOnNoDataWithCList() {
    // showCode=false, showCList=true, showData=false → 'clist'
    check("'register' tab, no-data CR with C-List → 'clist'",
        correctCRDetailTab('register', false, true, false), 'clist');
})();

(function testRegisterOnNoDataNoCapabilities() {
    // showCode=false, showCList=false, showData=false → 'lump'
    check("'register' tab, CR with no capabilities → 'lump'",
        correctCRDetailTab('register', false, false, false), 'lump');
})();

(function testBinaryOnCodeOnlyCR() {
    check("'binary' tab, code-only CR → 'code'",
        correctCRDetailTab('binary', true, false, false), 'code');
})();

(function testBinaryOnNoDataWithCList() {
    check("'binary' tab, no-data CR with C-List → 'clist'",
        correctCRDetailTab('binary', false, true, false), 'clist');
})();

(function testBinaryOnNoDataNoCapabilities() {
    check("'binary' tab, CR with no capabilities → 'lump'",
        correctCRDetailTab('binary', false, false, false), 'lump');
})();

// ─── Rule 5: 'clist' on a CR without a C-List ─────────────────────────────────

(function testClistOnNoClistWithCode() {
    // showCode=true, showCList=false → 'code'
    check("'clist' tab, CR without C-List (has code) → 'code'",
        correctCRDetailTab('clist', true, false, false), 'code');
})();

(function testClistOnNoClistWithData() {
    // showCode=false, showCList=false, showData=true → 'register'
    check("'clist' tab, CR without C-List (has data) → 'register'",
        correctCRDetailTab('clist', false, false, true), 'register');
})();

(function testClistOnNoClistNoCapabilities() {
    // showCode=false, showCList=false, showData=false → 'lump'
    check("'clist' tab, CR without C-List (no other capabilities) → 'lump'",
        correctCRDetailTab('clist', false, false, false), 'lump');
})();

// ─── Rule 6: Valid tabs remain unchanged ──────────────────────────────────────

(function testCodeTabOnCodeCR() {
    check("'code' tab, code CR (showCode=true) → unchanged",
        correctCRDetailTab('code', true, false, false), 'code');
})();

(function testApiTabOnCodeCR() {
    check("'api' tab, code CR (showCode=true) → unchanged",
        correctCRDetailTab('api', true, false, false), 'api');
})();

(function testRegisterTabOnDataCR() {
    check("'register' tab, data CR (showData=true) → unchanged",
        correctCRDetailTab('register', false, false, true), 'register');
})();

(function testBinaryTabOnDataCR() {
    check("'binary' tab, data CR (showData=true) → unchanged",
        correctCRDetailTab('binary', false, false, true), 'binary');
})();

(function testClistTabOnClistCR() {
    check("'clist' tab, clist CR (showCList=true) → unchanged",
        correctCRDetailTab('clist', false, true, false), 'clist');
})();

(function testLumpTabAlwaysUnchanged() {
    check("'lump' tab always unchanged (code+clist+data)",
        correctCRDetailTab('lump', true, true, true), 'lump');
    check("'lump' tab always unchanged (no capabilities)",
        correctCRDetailTab('lump', false, false, false), 'lump');
})();

// ─── Report ───────────────────────────────────────────────────────────────────

if (ERRORS.length > 0) {
    process.stderr.write(`\n${ERRORS.length} test(s) failed.\n`);
    process.exit(1);
}
process.exit(0);
