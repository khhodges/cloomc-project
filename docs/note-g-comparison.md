# Ada Lovelace's Note G vs. Church Computer

## Context

Ada Lovelace published Note G in August 1843 — a 25-operation program for Babbage's Analytical Engine that computed Bernoulli numbers. It is widely regarded as the first published computer algorithm. She used only four arithmetic operations (+, -, x, ÷) and variable columns (V1-V24), with each step performing one operation and tracking variable state changes.

Our `bernoulli.church` program computes the sum of squares 1² + 2² + 3² + 4² = 30 using the same step-by-step style — but on a Pure Church Lambda Machine where every operation passes through a 7-step capability-checked security pipeline. We use only six Church-domain instructions. No Turing instructions exist.

Both programs share the same philosophy: one operation per line, each result named, each step auditable.

**A note on indexing**: Ada labeled her result B₇, following a convention where B₁ was the first non-trivial Bernoulli number. In modern notation, her result corresponds to B₈ = -1/30. This document uses Ada's original labeling.

---

## Ada's Note G: Summary (1843)

Computing B₇ on the Analytical Engine. Reconstructed from Ada's published table in Taylor's Scientific Memoirs, August 1843. The original table uses Babbage's notation where superscript-prefixed variables (e.g., ²V₄) track the number of times each variable column has been assigned a new value.

**Structure**: 25 operations organized into four blocks:

| Block | Operations | Purpose | Runs |
|-------|-----------|---------|------|
| Initial setup | 1-7 | Compute A₀, set loop counter | Once per Bernoulli number |
| First term | 8-12 | Compute A₁ × B₁, accumulate | Once per Bernoulli number |
| Inner loop | 13-23 | Compute Aₖ × Bₖ terms, accumulate | (n-2) times |
| Finalize | 24-25 | Divide to get result, advance n | Once per Bernoulli number |

**Key characteristics**:
- 25 operations using only +, -, ×, ÷
- Variables V1-V3 hold constants and the parameter n
- Working variables hold intermediate computations
- Separate variables hold previously computed Bernoulli numbers
- Operations 13-23 form a loop body (conditional backward jump if counter ≠ 0)
- Superscript notation (²V₄) tracks each variable's assignment history — an innovation by Lovelace for documenting program state
- One known bug: a division in the published table has its operands swapped (possibly a typesetting error, as the Engine was never built to test it)

**The algorithm**: Uses a recurrence relation where each new Bernoulli number is computed from all previous ones, via weighted sums with binomial-derived coefficients that are recalculated at each step.

---

## Our bernoulli.church (2026)

Computing 1² + 2² + 3² + 4² = 30 on the Pure Church Lambda Machine.

| Line | Symbolic Math | Church Translation | Purpose |
|------|-------------|-------------------|---------|
| 1 | `let n = succ(3)` | Call(Lambda.SUCC, 3) | Establish n = 4 |
| 2 | `let two = succ(1)` | Call(Lambda.SUCC, 1) | Establish constant 2 |
| 3 | `let n_plus_1 = succ(n)` | Call(Lambda.SUCC, 4) | Compute n + 1 = 5 |
| 4 | `let two_n = two * n` | Call(Lambda.MUL, 2, 4) | Compute 2n = 8 |
| 5 | `let two_n_plus_1 = succ(two_n)` | Call(Lambda.SUCC, 8) | Compute 2n + 1 = 9 |
| 6 | `let prod1 = n * n_plus_1` | Call(Lambda.MUL, 4, 5) | Compute n(n+1) = 20 |
| 7 | `let product = prod1 * two_n_plus_1` | Call(Lambda.MUL, 20, 9) | Compute n(n+1)(2n+1) = 180 |
| 8 | `let six = two * 3` | Call(Lambda.MUL, 2, 3) | Establish constant 6 |
| 9 | `let sum_of_squares = product / six` | Call(SlideRule.DIV, 180, 6) | n(n+1)(2n+1)/6 = 30 |
| 10 | `let sq1 = 1 ^ two` | Call(Lambda.POW, 1, 2) | Compute 1² = 1 |
| 11 | `let sq2 = two ^ two` | Call(Lambda.POW, 2, 2) | Compute 2² = 4 |
| 12 | `let sq3 = 3 ^ two` | Call(Lambda.POW, 3, 2) | Compute 3² = 9 |
| 13 | `let sq4 = n ^ two` | Call(Lambda.POW, 4, 2) | Compute 4² = 16 |
| 14 | `let partial1 = sq1 + sq2` | Call(SlideRule.ADD, 1, 4) | 1 + 4 = 5 |
| 15 | `let partial2 = partial1 + sq3` | Call(SlideRule.ADD, 5, 9) | 5 + 9 = 14 |
| 16 | `let verify = partial2 + sq4` | Call(SlideRule.ADD, 14, 16) | 14 + 16 = 30 |

**Totals**: 16 operations. Six Church-domain instructions only (LOAD, TPERM, CALL, LAMBDA, RETURN + method dispatch). Zero Turing instructions.

---

## Thematic Comparison

These are different programs solving different problems on different machines 183 years apart. The comparison is thematic — what connects them is the programming model, not the specific computation.

| Aspect | Ada's Note G (1843) | bernoulli.church (2026) |
|--------|-------------------|----------------------|
| **Date** | August 1843 | February 2026 |
| **Machine** | Analytical Engine (never built) | Pure Church Lambda Machine |
| **Operations** | 25 | 16 |
| **Operation types** | 4 (add, subtract, multiply, divide) | 6 Church instructions per pipeline |
| **Variables** | V-numbered columns (gear columns) | Named let bindings |
| **Variable style** | Numbered with state tracking (²V₄) | Named, immutable (n, prod1...) |
| **Control flow** | Loops (ops 13-23 repeat) | Straight-line (no loops) |
| **Computes** | B₇ (Bernoulli number) | Sum of squares 1²+2²+3²+4² = 30 |
| **Verification** | Single computation path | Two independent paths compared |
| **Security** | None | 7-step capability pipeline per op |
| **Security gates per op** | 0 | 7 (LOAD→TPERM→CALL→LOAD→TPERM→LAMBDA→RETURN) |
| **Total security gates** | 0 | 112 (16 ops × 7 gates) |

---

## What They Share

1. **One operation per line** — Each step does exactly one arithmetic/logical operation and stores the result. Ada wrote each operation on one row of her table; we write each on one line of `.church` code.

2. **Named intermediates** — Ada used V-numbered columns; we use descriptive names like `n_plus_1` and `prod1`. Both approaches make every intermediate result inspectable.

3. **Step-by-step auditability** — Ada invented a notation (superscript state tracking) specifically to document how variables change through the program. Our REPL shows the Church-domain translation alongside each result, making the security pipeline visible.

4. **No shortcuts** — Neither program uses compound expressions. Ada could not — the Analytical Engine's mill processed one operation at a time. We choose not to — each line maps to exactly one 7-step security pipeline traversal, making the capability checking auditable.

5. **Constants established first** — Ada sets V1=1, V2=2, V3=n before computing. We establish `n`, `two`, and other base values before deriving results.

6. **Mathematical proof structure** — Both programs read like mathematical derivations: establish premises, derive intermediate results, arrive at conclusion.

---

## What Differs

1. **Security model** — Ada's Analytical Engine had no access control. Every gear column was accessible to every operation. The Church Computer enforces Golden Token capability checking on every single operation — 112 security gate traversals for 16 operations.

2. **Domain purity** — Ada used Turing-domain operations (add, subtract, multiply, divide acting directly on bare numerical data). The Church Computer uses zero Turing instructions — all computation happens through Church-encoded lambda calculus, with data registers (DRn) and context registers (CRn) on physically separate hardware paths.

3. **Immutability** — Ada's variables mutate (²V₄ overwrites ¹V₄ — the same gear column holds a new value). Church Computer `let` bindings are immutable — each creates a new named value. This eliminates an entire class of state-related bugs.

4. **Loops vs. straight-line** — Ada needed loops (ops 13-23) because her algorithm computes each Bernoulli number from all previous ones, requiring iteration. Our program is a straight-line derivation — no loops, no branches, no conditional execution.

5. **Abstraction access** — Ada's operations go directly to the mill (the Engine's arithmetic unit). Our operations go through named abstractions (Lambda, SlideRule) with separate C-Lists, each requiring a Golden Token with the correct permissions.

---

## The Connection

Ada Lovelace and Charles Babbage exchanged symbolic mathematics in their correspondence. She wrote that the Analytical Engine "weaves algebraical patterns just as the Jacquard loom weaves flowers and leaves." Our symbolic math notation — `let product = n * n_plus_1` — continues that vision: a programming model that reads like mathematical correspondence, where each named step builds on the last.

The difference is that 183 years later, every one of those algebraic steps now passes through a capability-checked security pipeline. Ada's insight was that machines could compute symbolically. Ours is that symbolic computation can be made inherently secure — not by adding security on top, but by building it into the instruction set itself.

Ada's program had a bug (a division with swapped operands). In a capability-secured architecture, accessing the wrong resource would require presenting the correct Golden Token — the permission system catches classes of errors that are invisible in unprotected architectures.

---

*Sources:*
*Ada Lovelace, "Notes by the Translator" (Note G), in L.F. Menabrea, "Sketch of The Analytical Engine," Taylor's Scientific Memoirs, Vol. III, August 1843.*
*R. Glaschick, "Ada Lovelace's Calculation of Bernoulli's Numbers," rclab.de, 2016.*
*Church Computer, bernoulli.church, CTMM Simulator Project, February 2026.*
