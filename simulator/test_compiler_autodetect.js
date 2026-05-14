'use strict';
// test_compiler_autodetect.js — Unit tests for Task #1143: CLOOMC multi-language auto-detection
// Run:  node simulator/test_compiler_autodetect.js
//
// Coverage:
//   T1 — English source is detected as English
//   T2 — Lambda Calculus source (-- LAMBDA CALCULUS header) is detected as lambda
//   T3 — Lambda Calculus source (Unicode λ) is detected as lambda
//   T4 — Symbolic Math (Ada) source is detected as symbolic
//   T5 — Haskell source is detected as haskell
//   T6 — JavaScript source falls through to javascript (default)
//   T7 — PetName (calculator expression) source is detected as petname
//   T8 — Haskell-style source is NOT misidentified as javascript
//   T9 — Symbolic-math source is NOT misidentified as english
//   T10 — Empty source falls back to javascript (sensible default)
//   T11 — Unknown/comment-only source falls back to javascript
//   T12 — compile() returns correct .language field for each front-end

const fs  = require('fs');
const path = require('path');
const vm  = require('vm');

// ── Counters ───────────────────────────────────────────────────────────────────
let pass = 0;
let fail = 0;

function check(label, cond) {
    if (cond) {
        console.log('PASS ' + label);
        pass++;
    } else {
        console.log('FAIL ' + label);
        fail++;
    }
}

// ── Global stubs required by cloomc_compiler.js ───────────────────────────────
// METHOD_REGISTER_CONVENTIONS and BOOT_UPLOADS are used by _buildPetNameTables;
// provide empty stubs so the compiler loads cleanly in Node.
global.METHOD_REGISTER_CONVENTIONS = {};
global.BOOT_UPLOADS = [];

// ── Load the compiler class ───────────────────────────────────────────────────
const compilerSrc = fs.readFileSync(path.join(__dirname, 'cloomc_compiler.js'), 'utf8');
vm.runInThisContext(compilerSrc);

// CLOOMCCompiler is now available as a global (declared with `class`, which
// vm.runInThisContext promotes to a global binding in this context).
const compiler = new CLOOMCCompiler();

// ── Representative source snippets ────────────────────────────────────────────

// English: score-based detection; "create an abstraction called" alone scores 3.
const ENGLISH_SRC = `
create an abstraction called Counter
  add a method called increment
    set result to result plus 1
    return the result
`.trim();

// Lambda Calculus: -- LAMBDA CALCULUS header triggers immediate true.
const LAMBDA_HEADER_SRC = `
-- LAMBDA CALCULUS
abstraction Church {
  method zero() = \\f.\\x.x
}
`.trim();

// Lambda Calculus: Unicode λ character triggers detection.
const LAMBDA_UNICODE_SRC = `
abstraction Church {
  method succ(n) = \u03BBf.\u03BBx.(f (n f x))
}
`.trim();

// Symbolic (Ada): two V-numbered variable assignments ≥ 2 → adaVars threshold.
const SYMBOLIC_SRC = `
abstraction Bernoulli {
  method compute() {
    V0 = 6
    V1 = V0 + 2
    V2 = V1
  }
}
`.trim();

// Symbolic (Ada): step keyword triggers opKeywords counter.
const SYMBOLIC_STEP_SRC = `
step 1: V0 = 10
step 2: V1 = V0 * 2
`.trim();

// Haskell: "method name() = expr" with no body braces.
const HASKELL_SRC = `
abstraction Math {
  method double(x) = x + x
  method square(x) = x * x
}
`.trim();

// Haskell: backslash-arrow lambda style.
const HASKELL_ARROW_SRC = `
abstraction FuncStyle {
  method apply(f) = \\x -> f x
}
`.trim();

// JavaScript: canonical curly-brace abstraction with method body.
const JAVASCRIPT_SRC = `
abstraction Counter {
  method increment() {
    let x = 0;
    return x + 1;
  }
}
`.trim();

// PetName: simple calculator expression — operatorPattern matches "x = y + z".
const PETNAME_SRC = `
x = 3 + 4
y = x * 2
`.trim();

// PetName via built-in function call (fallback regex uses Sqrt/GCD/etc.).
const PETNAME_FUNC_SRC = `
result = Sqrt(9)
`.trim();

// ── T1: English detection ─────────────────────────────────────────────────────
console.log('\n--- T1: English detection ---');
check('T1a: _detectEnglish returns true for English source',
    compiler._detectEnglish(ENGLISH_SRC));
check('T1b: _detectEnglish returns false for JS source',
    !compiler._detectEnglish(JAVASCRIPT_SRC));

// ── T2: Lambda detection via header comment ───────────────────────────────────
console.log('\n--- T2: Lambda detection (header) ---');
check('T2a: _detectLambda returns true for -- LAMBDA CALCULUS header',
    compiler._detectLambda(LAMBDA_HEADER_SRC));
check('T2b: _detectLambda returns false for English source',
    !compiler._detectLambda(ENGLISH_SRC));

// ── T3: Lambda detection via Unicode λ ───────────────────────────────────────
console.log('\n--- T3: Lambda detection (Unicode) ---');
check('T3a: _detectLambda returns true for Unicode \u03BB source',
    compiler._detectLambda(LAMBDA_UNICODE_SRC));
check('T3b: _detectLambda returns false for Haskell backslash source',
    !compiler._detectLambda(HASKELL_SRC));

// ── T4: Symbolic Math detection ───────────────────────────────────────────────
console.log('\n--- T4: Symbolic Math (Ada) detection ---');
check('T4a: _detectSymbolic returns true for V0/V1 assignment source',
    compiler._detectSymbolic(SYMBOLIC_SRC));
check('T4b: _detectSymbolic returns true for step-keyword source',
    compiler._detectSymbolic(SYMBOLIC_STEP_SRC));
check('T4c: _detectSymbolic returns false for plain JS source',
    !compiler._detectSymbolic(JAVASCRIPT_SRC));

// ── T5: Haskell detection ─────────────────────────────────────────────────────
console.log('\n--- T5: Haskell detection ---');
check('T5a: _detectHaskell returns true for method-equals style',
    compiler._detectHaskell(HASKELL_SRC));
check('T5b: _detectHaskell returns true for backslash-arrow style',
    compiler._detectHaskell(HASKELL_ARROW_SRC));
check('T5c: _detectHaskell returns false for JS source',
    !compiler._detectHaskell(JAVASCRIPT_SRC));
check('T5d: _detectHaskell returns false for English source',
    !compiler._detectHaskell(ENGLISH_SRC));

// ── T6: JavaScript falls through as default ───────────────────────────────────
console.log('\n--- T6: JavaScript default ---');
check('T6a: _detectEnglish false for JS source',  !compiler._detectEnglish(JAVASCRIPT_SRC));
check('T6b: _detectLambda  false for JS source',  !compiler._detectLambda(JAVASCRIPT_SRC));
check('T6c: _detectSymbolic false for JS source', !compiler._detectSymbolic(JAVASCRIPT_SRC));
check('T6d: _detectHaskell false for JS source',  !compiler._detectHaskell(JAVASCRIPT_SRC));
check('T6e: _detectPetName false for JS source',  !compiler._detectPetName(JAVASCRIPT_SRC));

// ── T7: PetName detection ─────────────────────────────────────────────────────
console.log('\n--- T7: PetName detection ---');
check('T7a: _detectPetName returns true for operator-expression source',
    compiler._detectPetName(PETNAME_SRC));
check('T7b: _detectPetName returns true for built-in-function-call source',
    compiler._detectPetName(PETNAME_FUNC_SRC));
check('T7c: _detectPetName returns false for JS source with braces',
    !compiler._detectPetName(JAVASCRIPT_SRC));
check('T7d: _detectPetName returns false for English source',
    !compiler._detectPetName(ENGLISH_SRC));

// ── T8: Haskell is NOT misidentified as JavaScript ────────────────────────────
console.log('\n--- T8: Haskell not misidentified as JS ---');
{
    const r8a = compiler.compile(HASKELL_SRC, []);
    check('T8a: compile(HASKELL_SRC).language is "haskell"',
        r8a.language === 'haskell');
    check('T8b: compile(HASKELL_SRC).language is NOT "javascript"',
        r8a.language !== 'javascript');

    const r8b = compiler.compile(HASKELL_ARROW_SRC, []);
    check('T8c: compile(HASKELL_ARROW_SRC).language is "haskell"',
        r8b.language === 'haskell');
}

// ── T9: Symbolic Math is NOT misidentified as English ─────────────────────────
console.log('\n--- T9: Symbolic not misidentified as English ---');
{
    check('T9a: _detectEnglish returns false for V0/V1 symbolic source',
        !compiler._detectEnglish(SYMBOLIC_SRC));
    check('T9b: _detectEnglish returns false for step-keyword source',
        !compiler._detectEnglish(SYMBOLIC_STEP_SRC));

    const r9 = compiler.compile(SYMBOLIC_STEP_SRC, []);
    check('T9c: compile(SYMBOLIC_STEP_SRC).language is "symbolic"',
        r9.language === 'symbolic');
    check('T9d: compile(SYMBOLIC_STEP_SRC).language is NOT "english"',
        r9.language !== 'english');
}

// ── T10: Empty source falls back to javascript ────────────────────────────────
console.log('\n--- T10: Empty source fallback ---');
{
    const r10 = compiler.compile('', []);
    check('T10a: compile("").language is "javascript"',
        r10.language === 'javascript');
    check('T10b: _detectEnglish returns false for empty source',
        !compiler._detectEnglish(''));
    check('T10c: _detectLambda returns false for empty source',
        !compiler._detectLambda(''));
    check('T10d: _detectSymbolic returns false for empty source',
        !compiler._detectSymbolic(''));
    check('T10e: _detectHaskell returns false for empty source',
        !compiler._detectHaskell(''));
    check('T10f: _detectPetName returns false for empty source',
        !compiler._detectPetName(''));
}

// ── T11: Comment-only source falls back to javascript ─────────────────────────
console.log('\n--- T11: Comment-only source fallback ---');
{
    const commentOnly = '// just a comment\n// nothing here\n';
    const r11 = compiler.compile(commentOnly, []);
    check('T11a: compile(comment-only).language is "javascript"',
        r11.language === 'javascript');
}

// ── T12: compile() .language field correctness for each front-end ─────────────
console.log('\n--- T12: compile().language field for each front-end ---');
{
    const r_english  = compiler.compile(ENGLISH_SRC, []);
    check('T12a: English source → language "english"',
        r_english.language === 'english');

    const r_lambda   = compiler.compile(LAMBDA_HEADER_SRC, []);
    check('T12b: Lambda header source → language "lambda"',
        r_lambda.language === 'lambda');

    const r_lambdaU  = compiler.compile(LAMBDA_UNICODE_SRC, []);
    check('T12c: Lambda Unicode source → language "lambda"',
        r_lambdaU.language === 'lambda');

    const r_symbolic = compiler.compile(SYMBOLIC_SRC, []);
    check('T12d: Symbolic source → language "symbolic"',
        r_symbolic.language === 'symbolic');

    const r_haskell  = compiler.compile(HASKELL_SRC, []);
    check('T12e: Haskell source → language "haskell"',
        r_haskell.language === 'haskell');

    const r_js       = compiler.compile(JAVASCRIPT_SRC, []);
    check('T12f: JavaScript source → language "javascript"',
        r_js.language === 'javascript');
}

// ── Summary ────────────────────────────────────────────────────────────────────
console.log('\n\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550');
console.log('Results: ' + pass + ' passed, ' + fail + ' failed');
if (fail > 0) {
    console.log('SOME TESTS FAILED');
    process.exit(1);
} else {
    console.log('ALL TESTS PASSED');
}
