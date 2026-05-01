'use strict';
// ── correctCRDetailTab ────────────────────────────────────────────────────────
// Pure helper: given the current tab name and three boolean capability flags
// for the CR being displayed, return the corrected tab name.
//
// Rules (mirror the block in app-memory.js updateCRDetail):
//   1. 'code' or 'api' on a non-code CR  → 'clist', 'register', or 'lump'
//   2. 'register' or 'binary' on a code-only CR → 'code', 'clist', or 'lump'
//   3. 'clist' on a CR without a C-List  → 'code', 'register', or 'lump'
//
// Exported via module.exports when running under Node.js (test harnesses).
// Loaded as a plain browser global when included via <script> in index.html.

function correctCRDetailTab(crDetailTab, showCode, showCList, showData) {
    if ((crDetailTab === 'code' || crDetailTab === 'api') && !showCode) {
        return showCList ? 'clist' : showData ? 'register' : 'lump';
    } else if ((crDetailTab === 'register' || crDetailTab === 'binary') && !showData) {
        return showCode ? 'code' : showCList ? 'clist' : 'lump';
    } else if (crDetailTab === 'clist' && !showCList) {
        return showCode ? 'code' : showData ? 'register' : 'lump';
    }
    return crDetailTab;
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { correctCRDetailTab };
}
