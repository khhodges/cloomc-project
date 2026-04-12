#!/usr/bin/env node
// compile_sliderule.js — compile sliderule.cloomc and emit updated SlideRule.json
// Usage: node tools/compile_sliderule.js

const fs   = require('fs');
const path = require('path');
const vm   = require('vm');

const ROOT        = path.join(__dirname, '..');
const COMPILER_JS = path.join(ROOT, 'simulator', 'cloomc_compiler.js');
const SOURCE_FILE = path.join(ROOT, 'simulator', 'cloomc', 'sliderule.cloomc');
const OUTPUT_JSON = path.join(ROOT, 'simulator', 'cloomc', 'SlideRule.json');

// ── Load the compiler class via vm sandbox ───────────────────────────────────
const compilerSrc = fs.readFileSync(COMPILER_JS, 'utf8');
const sandbox = { module: { exports: {} }, exports: {} };
sandbox.global = sandbox;
vm.runInNewContext(compilerSrc + '\nmodule.exports = CLOOMCCompiler;', sandbox);
const CLOOMCCompiler = sandbox.module.exports;

if (typeof CLOOMCCompiler !== 'function') {
    console.error('Failed to load CLOOMCCompiler — got:', typeof CLOOMCCompiler);
    process.exit(1);
}

const compiler = new CLOOMCCompiler();

// ── Compile ──────────────────────────────────────────────────────────────────
const source = fs.readFileSync(SOURCE_FILE, 'utf8');
console.log(`Compiling ${SOURCE_FILE} …`);
const result = compiler.compile(source, []);

if (result.errors && result.errors.length > 0) {
    console.error('Compilation errors:');
    for (const e of result.errors) {
        console.error(`  line ${e.line}: ${e.message}`);
    }
    process.exit(1);
}

console.log(`OK — ${result.methods.length} methods compiled from "${result.abstractionName}"`);

// ── Build updated JSON ───────────────────────────────────────────────────────
const methods = result.methods.map(m => ({
    name: m.name,
    code: m.code.map(w => '0x' + ((w >>> 0).toString(16).padStart(8, '0'))),
}));

const existing = JSON.parse(fs.readFileSync(OUTPUT_JSON, 'utf8'));

const updated = {
    abstraction  : existing.abstraction,
    token        : existing.token,
    ns_slot      : existing.ns_slot,
    type         : existing.type,
    grants       : existing.grants,
    capabilities : existing.capabilities,
    methods,
};

fs.writeFileSync(OUTPUT_JSON, JSON.stringify(updated, null, 2) + '\n', 'utf8');
console.log(`Wrote ${OUTPUT_JSON}`);

console.log('\nMethod summary:');
let total = 0;
for (const m of methods) {
    console.log(`  ${m.name.padEnd(14)} ${String(m.code.length).padStart(4)} words`);
    total += m.code.length;
}
console.log(`  ${'TOTAL'.padEnd(14)} ${String(total).padStart(4)} words`);
