// assembler_test.js — regression tests for ChurchAssembler
// Run with: node simulator/assembler_test.js
'use strict';

const ChurchAssembler = require('./assembler.js');

let passed = 0;
let failed = 0;

function assert(label, condition, detail) {
    if (condition) {
        console.log('PASS ' + label);
        passed++;
    } else {
        console.log('FAIL ' + label + (detail ? ' — ' + detail : ''));
        failed++;
    }
}

const CONVENTIONS = {
    'SlideRule': {
        'Multiply': { index: 0 },
        'Divide':   { index: 1 },
        'Sqrt':     { index: 2 },
    }
};

const NS_SYMBOLS = { 'SlideRule': 3 };

// ── CALL CR<n>, MethodName  (task-478) ──────────────────────────────────────

// T1: CALL CR11, Multiply succeeds when CR11 is bound to SlideRule via a prior LOAD.
{
    const a = new ChurchAssembler(CONVENTIONS);
    a.setNamespace(NS_SYMBOLS);
    const result = a.assemble('LOAD CR11, SlideRule\nCALL CR11, Multiply');
    const errors = a.errors;
    const word   = result.words[1];
    const opcode = (word >>> 27) & 0x1F;
    const crDst  = (word >>> 19) & 0xF;
    const crSrc  = (word >>> 15) & 0xF;
    assert('T1 CALL CR11, Multiply assembles with no errors',
        errors.length === 0, errors.map(e => e.message).join('; '));
    assert('T1 opcode=2 (CALL)', opcode === 2, 'got ' + opcode);
    assert('T1 crDst=11', crDst === 11, 'got ' + crDst);
    assert('T1 crSrc=0 (Multiply index)', crSrc === 0, 'got ' + crSrc);
}

// T2: CALL CR11, Multiply produces a clear, targeted error when no binding exists.
{
    const a = new ChurchAssembler(CONVENTIONS);
    const result = a.assemble('CALL CR11, Multiply');
    const errors = a.errors;
    assert('T2 CALL CR11,Multiply (unbound) produces an error',
        errors.length > 0, 'expected at least one error');
    assert('T2 error message mentions CR11 and no-binding',
        errors.length > 0 && errors[0].message.includes('CR11') && errors[0].message.includes('no known'),
        errors.length > 0 ? errors[0].message : '(no error)');
}

// T3: CALL CR11, 0 (numeric selector) is unaffected by the change.
{
    const a = new ChurchAssembler(CONVENTIONS);
    a.setNamespace(NS_SYMBOLS);
    const result = a.assemble('LOAD CR11, SlideRule\nCALL CR11, 0');
    const errors = a.errors;
    const word   = result.words[1];
    const crSrc  = (word >>> 15) & 0xF;
    assert('T3 CALL CR11, 0 (numeric) assembles without errors',
        errors.length === 0, errors.map(e => e.message).join('; '));
    assert('T3 crSrc=0', crSrc === 0, 'got ' + crSrc);
}

// T4: Bound abstraction with no registered conventions produces a clear error.
{
    const a = new ChurchAssembler({});   // empty conventions
    a.setNamespace(NS_SYMBOLS);
    const result = a.assemble('LOAD CR11, SlideRule\nCALL CR11, Multiply');
    const errors = a.errors;
    assert('T4 CALL CR11, Multiply (no conventions) produces an error',
        errors.length > 0, 'expected at least one error');
    assert('T4 error message mentions no conventions',
        errors.length > 0 && errors[0].message.includes('No method conventions'),
        errors.length > 0 ? errors[0].message : '(no error)');
}

// T5: Unknown method name produces a clear error listing known methods.
{
    const a = new ChurchAssembler(CONVENTIONS);
    a.setNamespace(NS_SYMBOLS);
    const result = a.assemble('LOAD CR11, SlideRule\nCALL CR11, UnknownMethod');
    const errors = a.errors;
    assert('T5 CALL CR11, UnknownMethod produces an error',
        errors.length > 0, 'expected at least one error');
    assert('T5 error message says "not a known method" and lists known methods',
        errors.length > 0 &&
        errors[0].message.includes('not a known method') &&
        errors[0].message.includes('Multiply'),
        errors.length > 0 ? errors[0].message : '(no error)');
}

// ── Summary ──────────────────────────────────────────────────────────────────
console.log('\n' + passed + ' passed, ' + failed + ' failed');
if (failed > 0) process.exit(1);
