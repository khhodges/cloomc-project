#!/usr/bin/env node
// scripts/test_check_selftest_lump_stale.js
//
// Tests for check_selftest_lump_stale.js — specifically the manifest-token
// cross-check branch.
//
// Suites:
//   1. Stale-token (failure path): patches the PostFlashSelftest token to a
//      bogus value → expects exit code 1 and the expected FAIL message.
//   2. Happy path: passes the real manifest → expects exit code 0.
//
// Run:
//   node scripts/test_check_selftest_lump_stale.js

'use strict';

const fs            = require('fs');
const path          = require('path');
const os            = require('os');
const { spawnSync } = require('child_process');

const ROOT          = path.resolve(__dirname, '..');
const GUARD         = path.join(__dirname, 'check_selftest_lump_stale.js');
const REAL_MANIFEST = path.join(ROOT, 'server', 'lumps', 'manifest.json');

let passed = 0;
let failed = 0;

function assert(condition, message) {
    if (condition) {
        console.log(`  PASS: ${message}`);
        passed++;
    } else {
        console.error(`  FAIL: ${message}`);
        failed++;
    }
}

function runGuard(manifestPath) {
    const result = spawnSync(
        process.execPath,
        [GUARD, '--manifest', manifestPath],
        { encoding: 'utf8' }
    );
    return {
        code:   result.status,
        stdout: result.stdout || '',
        stderr: result.stderr || '',
    };
}

function writeTempManifest(content) {
    const tmp = path.join(os.tmpdir(), `manifest_test_${Date.now()}_${Math.random().toString(36).slice(2)}.json`);
    fs.writeFileSync(tmp, JSON.stringify(content, null, 4), 'utf8');
    return tmp;
}

// ── Read and parse the real manifest once ────────────────────────────────────
let realManifest;
try {
    realManifest = JSON.parse(fs.readFileSync(REAL_MANIFEST, 'utf8'));
} catch (e) {
    console.error(`Cannot read real manifest: ${e.message}`);
    process.exit(1);
}

const selfTestEntry = realManifest.find(e => e.abstraction === 'PostFlashSelftest');
if (!selfTestEntry) {
    console.error('No PostFlashSelftest entry found in manifest.json — cannot run tests.');
    process.exit(1);
}
const realToken = selfTestEntry.token;

// ── Suite 1: stale / wrong token ─────────────────────────────────────────────
console.log('\nSuite 1: stale manifest token → guard must exit 1');

let tmp1;
try {
    const bogusToken  = 'deadbeef';
    const patched     = realManifest.map(e =>
        e.abstraction === 'PostFlashSelftest'
            ? Object.assign({}, e, { token: bogusToken })
            : e
    );
    tmp1 = writeTempManifest(patched);

    const r = runGuard(tmp1);

    assert(r.code === 1,
        'guard exits with code 1 when token is wrong');

    const expectedFragment = `FAIL: manifest.json PostFlashSelftest entry has token "${bogusToken}" but expected "${realToken}"`;
    const combined = r.stdout + r.stderr;
    assert(combined.includes(expectedFragment),
        `stderr/stdout contains expected FAIL message (looking for: ${expectedFragment})`);
} finally {
    if (tmp1) try { fs.unlinkSync(tmp1); } catch (_) {}
}

// ── Suite 2: missing PostFlashSelftest entry ──────────────────────────────────
console.log('\nSuite 2: no PostFlashSelftest entry → guard must exit 1');

let tmp2;
try {
    const stripped = realManifest.filter(e => e.abstraction !== 'PostFlashSelftest');
    tmp2 = writeTempManifest(stripped);

    const r = runGuard(tmp2);

    assert(r.code === 1,
        'guard exits with code 1 when PostFlashSelftest entry is absent');

    const combined = r.stdout + r.stderr;
    assert(combined.includes('FAIL: manifest.json has no PostFlashSelftest entry'),
        'stderr/stdout contains "FAIL: manifest.json has no PostFlashSelftest entry"');
} finally {
    if (tmp2) try { fs.unlinkSync(tmp2); } catch (_) {}
}

// ── Suite 3: duplicate PostFlashSelftest entries ──────────────────────────────
console.log('\nSuite 3: duplicate PostFlashSelftest entries → guard must exit 1');

let tmp3;
try {
    const duplicated = [...realManifest,
        Object.assign({}, selfTestEntry, { token: 'aaaabbbb' })];
    tmp3 = writeTempManifest(duplicated);

    const r = runGuard(tmp3);

    assert(r.code === 1,
        'guard exits with code 1 when there are duplicate PostFlashSelftest entries');

    const combined = r.stdout + r.stderr;
    assert(combined.includes('FAIL: manifest.json has') && combined.includes('PostFlashSelftest entries'),
        'stderr/stdout reports the duplicate-entry FAIL');
} finally {
    if (tmp3) try { fs.unlinkSync(tmp3); } catch (_) {}
}

// ── Suite 4: malformed JSON manifest ─────────────────────────────────────────
console.log('\nSuite 4: malformed manifest JSON → guard must exit 1');

let tmp4;
try {
    tmp4 = path.join(os.tmpdir(), `manifest_bad_${Date.now()}.json`);
    fs.writeFileSync(tmp4, '{ this is not valid json', 'utf8');

    const r = runGuard(tmp4);

    assert(r.code === 1,
        'guard exits with code 1 when manifest JSON is malformed');

    const combined = r.stdout + r.stderr;
    assert(combined.includes('FAIL: server/lumps/manifest.json could not be parsed') ||
           combined.includes('could not be parsed'),
        'stderr/stdout reports parse failure');
} finally {
    if (tmp4) try { fs.unlinkSync(tmp4); } catch (_) {}
}

// ── Suite 5: happy path (real manifest) ──────────────────────────────────────
console.log('\nSuite 5: happy path — real manifest → guard must exit 0');
{
    const r = runGuard(REAL_MANIFEST);

    assert(r.code === 0,
        'guard exits with code 0 when real manifest is used');

    const combined = r.stdout + r.stderr;
    assert(combined.includes('OK:   manifest.json PostFlashSelftest entry — token matches'),
        'stdout confirms manifest token matches');
}

// ── Summary ───────────────────────────────────────────────────────────────────
console.log(`\n${'─'.repeat(60)}`);
console.log(`Results: ${passed} passed, ${failed} failed`);

if (failed > 0) {
    process.exit(1);
}
console.log('All tests passed.');
process.exit(0);
