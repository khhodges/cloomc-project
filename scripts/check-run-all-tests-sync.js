#!/usr/bin/env node
/**
 * check-run-all-tests-sync.js
 *
 * Diffs the set of test workflows in .replit against the run_suite entries in
 * scripts/run-all-tests.sh.  Exits non-zero with a clear message if any test
 * workflow is missing from the script, or if the script lists a suite name
 * that has no matching workflow.
 *
 * "Test workflow" is defined as any named workflow that is NOT in the
 * INFRASTRUCTURE_WORKFLOWS exclusion set below.  Add entries to that set only
 * when you introduce a new non-test workflow (e.g. a new app server).
 *
 * Usage:
 *   node scripts/check-run-all-tests-sync.js
 *
 * Wired into:
 *   - scripts/run-all-tests.sh  (self-check before running any suite)
 *   - check-api-reference-stale workflow  (CI gate)
 */

'use strict';

const fs   = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');

// ---------------------------------------------------------------------------
// Infrastructure workflows that are NOT test suites.
// Extend this list only when adding a new non-test workflow (app servers,
// orchestrators, design previews, etc.).
// ---------------------------------------------------------------------------
const INFRASTRUCTURE_WORKFLOWS = new Set([
    'Project',
    'Church Machine IDE',
    'all-tests',
    'artifacts/mockup-sandbox: Component Preview Server',
]);

// ---------------------------------------------------------------------------
// Parse .replit — extract names of all [[workflows.workflow]] entries
// ---------------------------------------------------------------------------
function parseAllWorkflowNames(replitPath) {
    const text  = fs.readFileSync(replitPath, 'utf8');
    const names = new Set();
    const re    = /^\[\[workflows\.workflow\]\]\s*\n(?:.*\n)*?name\s*=\s*"([^"]+)"/gm;
    let m;
    while ((m = re.exec(text)) !== null) {
        names.add(m[1]);
    }
    return names;
}

// ---------------------------------------------------------------------------
// Parse run-all-tests.sh — extract the first arg of every launch_suite call
// (previously run_suite; supports both for backwards compatibility)
// ---------------------------------------------------------------------------
function parseRunAllSuites(scriptPath) {
    const text  = fs.readFileSync(scriptPath, 'utf8');
    const names = new Set();
    const re    = /^\s*(?:launch_suite|run_suite)\s+"([^"]+)"/gm;
    let m;
    while ((m = re.exec(text)) !== null) {
        names.add(m[1]);
    }
    return names;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
const replitPath = path.join(ROOT, '.replit');
const scriptPath = path.join(ROOT, 'scripts', 'run-all-tests.sh');

if (!fs.existsSync(replitPath)) {
    console.error('ERROR: .replit not found at', replitPath);
    process.exit(1);
}
if (!fs.existsSync(scriptPath)) {
    console.error('ERROR: scripts/run-all-tests.sh not found at', scriptPath);
    process.exit(1);
}

const allWorkflowNames = parseAllWorkflowNames(replitPath);
const testWorkflowNames = new Set(
    [...allWorkflowNames].filter(n => !INFRASTRUCTURE_WORKFLOWS.has(n))
);
const suiteNames = parseRunAllSuites(scriptPath);

// Test workflows that are missing from run-all-tests.sh
const missingFromScript = [...testWorkflowNames].filter(n => !suiteNames.has(n)).sort();

// Suite names in run-all-tests.sh that have no matching workflow in .replit
const orphanInScript = [...suiteNames].filter(n => !testWorkflowNames.has(n)).sort();

let ok = true;

if (missingFromScript.length > 0) {
    ok = false;
    console.error('');
    console.error('SYNC ERROR — the following workflows exist in .replit but are');
    console.error('missing from scripts/run-all-tests.sh:');
    for (const name of missingFromScript) {
        console.error(`  • ${name}`);
    }
    console.error('');
    console.error('Add a run_suite entry for each missing workflow, then re-run.');
    console.error('If the workflow is infrastructure (not a test), add it to');
    console.error('INFRASTRUCTURE_WORKFLOWS in scripts/check-run-all-tests-sync.js.');
}

if (orphanInScript.length > 0) {
    ok = false;
    console.error('');
    console.error('SYNC ERROR — the following run_suite names in run-all-tests.sh');
    console.error('have no matching workflow in .replit:');
    for (const name of orphanInScript) {
        console.error(`  • ${name}`);
    }
    console.error('');
    console.error('Either add a matching workflow to .replit or remove the stale');
    console.error('run_suite entry from scripts/run-all-tests.sh.');
}

if (ok) {
    const n = testWorkflowNames.size;
    console.log(`OK — all ${n} test workflow${n === 1 ? '' : 's'} are present in run-all-tests.sh.`);
    process.exit(0);
} else {
    process.exit(1);
}
