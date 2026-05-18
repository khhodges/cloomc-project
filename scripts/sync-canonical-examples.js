#!/usr/bin/env node
// scripts/sync-canonical-examples.js
//
// Reads simulator/app-run.js, extracts every inline example string, and
// writes each one to simulator/examples/<key>.cloomc.
//
// Usage:
//   node scripts/sync-canonical-examples.js          # update — write files
//   node scripts/sync-canonical-examples.js --check  # CI guard — exit 1 if any file would change
//
// The --check flag is designed for use in CI / pre-commit hooks.  It makes no
// writes; it only reports which files are out of date and exits non-zero.
//
// Skipped keys
// ────────────
// led_dr_test is deliberately excluded.  In app-run.js it is a variable
// reference, not a backtick literal:
//   'led_dr_test': _TURING_DR_TEST_SOURCE,
// The extraction regex (which matches 'key': `...`) cannot capture a variable
// reference.  The led_dr_test source (_TURING_DR_TEST_SOURCE) is a separate
// top-level backtick literal and is already verified by the EX-LED test suite
// in assembler_test.js (EX14–EX15).

'use strict';

const fs   = require('fs');
const path = require('path');

const CHECK_MODE  = process.argv.includes('--check');
const ROOT        = path.resolve(__dirname, '..');
const APP_RUN     = path.join(ROOT, 'simulator', 'app-run.js');
const EXAMPLES_DIR = path.join(ROOT, 'simulator', 'examples');

// Keys that cannot be extracted with the simple backtick regex.
const SKIP_KEYS = new Set(['led_dr_test']);

// ── helpers ──────────────────────────────────────────────────────────────────

function extractInline(src, key) {
    // Matches:  'key'  :  `...content...`  ,
    // The lazy [\s\S]*? is safe because CLOOMC source never contains backticks.
    const re = new RegExp("'" + key + "'\\s*:\\s*`([\\s\\S]*?)`\\s*,");
    const m  = src.match(re);
    return m ? m[1] : null;
}

function canonicalPath(key) {
    return path.join(EXAMPLES_DIR, key + '.cloomc');
}

// ── main ─────────────────────────────────────────────────────────────────────

const appRunSrc = fs.readFileSync(APP_RUN, 'utf8');

// Collect every 'key': ` pattern present in app-run.js.
const keyRe   = /'([a-z_0-9]+)'\s*:\s*`/g;
let   m;
const allKeys = [];
while ((m = keyRe.exec(appRunSrc)) !== null) {
    if (!allKeys.includes(m[1])) allKeys.push(m[1]);
}

let changed  = 0;
let skipped  = 0;
let created  = 0;
let upToDate = 0;
let errors   = 0;

for (const key of allKeys) {
    if (SKIP_KEYS.has(key)) {
        console.log(`  skip   ${key}  (variable reference — see comment at top of script)`);
        skipped++;
        continue;
    }

    const inline = extractInline(appRunSrc, key);
    if (inline === null) {
        console.error(`  ERROR  ${key}  — could not extract inline source`);
        errors++;
        continue;
    }

    const dest    = canonicalPath(key);
    const isNew   = !fs.existsSync(dest);
    const current = isNew ? null : fs.readFileSync(dest, 'utf8');

    if (!isNew && current === inline) {
        console.log(`  ok     ${key}`);
        upToDate++;
        continue;
    }

    if (CHECK_MODE) {
        if (isNew) {
            console.error(`  DRIFT  ${key}  — canonical file does not exist yet`);
        } else {
            console.error(`  DRIFT  ${key}  — canonical file differs from inline source`);
        }
        changed++;
    } else {
        fs.writeFileSync(dest, inline, 'utf8');
        if (isNew) {
            console.log(`  create ${key}`);
            created++;
        } else {
            console.log(`  update ${key}`);
            changed++;
        }
    }
}

// ── summary ──────────────────────────────────────────────────────────────────

console.log('');
if (CHECK_MODE) {
    const drifted = changed;
    console.log(`sync-canonical-examples (check mode): ${drifted} drifted, ${upToDate} up-to-date, ${skipped} skipped, ${errors} errors`);
    if (drifted > 0 || errors > 0) {
        console.error('');
        console.error('Run  node scripts/sync-canonical-examples.js  to regenerate the canonical files.');
        process.exit(1);
    }
} else {
    console.log(`sync-canonical-examples: ${changed} updated, ${created} created, ${upToDate} up-to-date, ${skipped} skipped, ${errors} errors`);
    if (errors > 0) process.exit(1);
}
