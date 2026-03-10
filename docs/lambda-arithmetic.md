# Beyond Integer Division — Arithmetic on the Church Machine

The Church Machine operates on 32-bit integers. There are no floating-point registers or instructions. Every value in a data register (DR0–DR7) is a whole number, and the division instruction (`/`) performs integer division via repeated subtraction, truncating any fractional part.

This means `7 / 3 = 2`, not `2.333...`.

So how do you compute precise results on integer-only hardware? This document covers three techniques — **fixed-point arithmetic**, **remainder/modulo**, and **rational numbers** — each implemented as a Lambda Calculus abstraction in the Church Machine IDE.

---

## 1. Fixed-Point Arithmetic

### The Idea

Instead of storing `2.33`, store `233` and remember that the last two digits are after the decimal point. You pick a **scale factor** (we use 100 for two decimal places) and multiply all values by it before doing arithmetic.

This is exactly how slide rules work — the S scale reads `sin(θ) × 10` to avoid sub-unit markings. The Church Machine's `FixedPointMath` abstraction applies the same principle to general computation.

### How It Works

| Real value | Fixed-point (×100) |
|---|---|
| 3.14 | 314 |
| 0.50 | 50 |
| 1.00 | 100 |
| 7.25 | 725 |

To compute `7 / 3` with two decimal places:

1. Scale up: `7 × 100 = 700`
2. Divide: `700 / 3 = 233` (integer division)
3. Interpret: `233` means `2.33`

### The Abstraction: `FixedPointMath`

Load the **LC: Fixed-Pt** example tab in the Code view.

```
abstraction FixedPointMath {
    capabilities { Constants }

    method toFixed(n)         = n * 100         -- integer → fixed-point
    method fromFixed(f)       = f / 100         -- fixed-point → integer (truncates)
    method addFixed(a, b)     = a + b           -- addition (scale preserved)
    method subFixed(a, b)     = a - b           -- subtraction (scale preserved)
    method mulFixed(a, b)     = (a * b) / 100   -- multiply then rescale
    method divFixed(a, b)     = (a * 100) / b   -- prescale then divide
    method percent(whole, pct) = (whole * pct) / 100
    method roundFixed(f)      = (f + 50) / 100  -- round to nearest integer
}
```

### Usage Examples

**Adding prices:**
- £3.50 + £2.75 → `350 + 275 = 625` → £6.25

**Multiplying with rescale:**
- 1.5 × 2.4 → `150 × 240 = 36000`, then `36000 / 100 = 360` → 3.60

**Division with prescale:**
- 7 / 3 → `toFixed(7) = 700`, then `divFixed(700, 300) = (700 × 100) / 300 = 233` → 2.33

**Percentage:**
- 15% of 200 → `percent(200, 15) = (200 × 15) / 100 = 30`

**Rounding:**
- `roundFixed(233) = (233 + 50) / 100 = 2` (rounds 2.33 down)
- `roundFixed(267) = (267 + 50) / 100 = 3` (rounds 2.67 up)

### Limitations

- **Overflow risk**: Multiplying two scaled values produces very large intermediate results. `999 × 999 = 998001` is fine in 32 bits, but `99999 × 99999` can overflow. Keep values reasonable for your scale factor.
- **Truncation**: Integer division always rounds toward zero. `roundFixed` compensates for this in the final step.
- **Scale factor is fixed**: The abstraction uses 100 (two decimal places). For more precision, change the scale to 1000 or 10000 and adjust the methods accordingly.

---

## 2. Remainder / Modulo

### The Idea

The Church Machine's division instruction works by repeated subtraction:

```
quotient = 0
remainder = dividend
while remainder >= divisor:
    remainder = remainder - divisor
    quotient = quotient + 1
```

When the loop exits, the **remainder** is still sitting in a register. Integer division discards it — but a `mod` method can return it instead.

### How It Works

`17 / 5`:
- Start: quotient=0, remainder=17
- Step 1: remainder=12, quotient=1
- Step 2: remainder=7, quotient=2
- Step 3: remainder=2, quotient=3
- Exit (2 < 5): **quotient = 3**, **remainder = 2**

So `17 / 5 = 3` with remainder `2`, meaning `17 = 5 × 3 + 2`.

### Practical Uses

- **Clock arithmetic**: `minutes mod 60` gives minutes past the hour
- **Even/odd test**: `n mod 2` is 0 for even, 1 for odd
- **Digit extraction**: `n mod 10` gives the last digit
- **Cyclic patterns**: `step mod 4` cycles through 0, 1, 2, 3, 0, 1, ...

### Connection to Fixed-Point

You can combine division and modulo to reconstruct a decimal result:

```
7 / 3 = 2       (integer part)
7 mod 3 = 1     (remainder)
```

The remainder `1` over divisor `3` gives the fractional part `0.333...`. Or using fixed-point: `(1 × 100) / 3 = 33`, so `7 / 3 = 2.33`.

---

## 3. Rational Numbers

### The Idea

Instead of approximating with fixed-point, represent every number as an exact fraction: a pair of integers `(numerator, denominator)`. The fraction `1/3` stays as `(1, 3)` — no precision is ever lost.

All arithmetic follows the standard fraction rules you learned in school:

| Operation | Rule | Example |
|---|---|---|
| a/b + c/d | (a×d + c×b) / (b×d) | 1/3 + 1/6 = (6+3)/18 = 9/18 |
| a/b − c/d | (a×d − c×b) / (b×d) | 3/4 − 1/4 = (12−4)/16 = 8/16 |
| a/b × c/d | (a×c) / (b×d) | 2/3 × 3/5 = 6/15 |
| a/b ÷ c/d | (a×d) / (b×c) | 2/3 ÷ 4/5 = 10/12 |
| a/b = c/d? | a×d = c×b? | 9/18 = 1/2? → 18 = 18 ✓ |

### The Abstraction: `RationalArith`

Load the **LC: Rational** example tab in the Code view.

```
abstraction RationalArith {
    capabilities { }

    method numerator(n, d)            = n
    method denominator(n, d)          = d

    method addNum(n1, d1, n2, d2)     = (n1 * d2) + (n2 * d1)
    method addDen(d1, d2)             = d1 * d2

    method subNum(n1, d1, n2, d2)     = (n1 * d2) - (n2 * d1)

    method mulNum(n1, n2)             = n1 * n2
    method mulDen(d1, d2)             = d1 * d2

    method divNum(n1, d2)             = n1 * d2
    method divDen(d1, n2)             = d1 * n2

    method isEqual(n1, d1, n2, d2)    = if (n1 * d2) == (n2 * d1) then 1 else 0

    method gcd(a, b)                  = if b == 0 then a
                                        else if a == b then a
                                        else if a > b then a - b
                                        else b - a
}
```

### Usage Examples

**Add 1/3 + 1/4:**
```
numerator   = addNum(1, 3, 1, 4)  = (1×4) + (1×3) = 7
denominator = addDen(3, 4)        = 3 × 4         = 12
Result: 7/12
```

**Multiply 2/3 × 3/5:**
```
numerator   = mulNum(2, 3)  = 6
denominator = mulDen(3, 5)  = 15
Result: 6/15 (simplifies to 2/5)
```

**Simplify using GCD:**
```
gcd(6, 15):
  6 < 15 → 15 - 6 = 9
  6 < 9  → 9 - 6  = 3
  6 > 3  → 6 - 3  = 3
  3 == 3 → GCD = 3

6/15 → (6/3) / (15/3) = 2/5
```

**Test equality: is 9/18 equal to 1/2?**
```
isEqual(9, 18, 1, 2) = if (9×2) == (1×18) then 1 else 0
                      = if 18 == 18 then 1 else 0
                      = 1 (yes, they are equal)
```

### Why Separate Num/Den Methods?

The Church Machine passes and returns single integer values — there are no tuple return types at the hardware level. So each fraction operation is split into two method calls: one for the numerator, one for the denominator. The caller tracks both values.

This is how real hardware fraction libraries work: you store the numerator and denominator in separate registers and call the appropriate method for each component.

### Limitations

- **Denominator growth**: Without simplification, denominators grow with every operation. After several additions, `d1 × d2 × d3 × ...` can overflow 32 bits. Use `gcd` to simplify periodically.
- **The GCD method is iterative, not recursive**: The implementation uses subtraction (`a - b`) rather than modulo (`a mod b`), so it takes more steps for values that are far apart. This is correct but slower than the Euclidean algorithm with modulo.
- **No automatic simplification**: The abstraction returns unsimplified fractions. The caller must use `gcd` and divide both components to reduce.

---

## Choosing the Right Technique

| Technique | Precision | Speed | Best for |
|---|---|---|---|
| **Fixed-point** | Approximate (2-4 decimal places) | Fast (one extra multiply or divide) | Prices, measurements, percentages, sensor data |
| **Remainder** | Exact (integer remainder) | Free (already computed by division) | Clock math, even/odd, digit extraction, cycles |
| **Rational** | Exact (no loss ever) | Slower (multiple method calls per operation) | Scientific computation, exact comparisons, proofs |

All three techniques compile to the same 20 Church Machine instructions. The hardware doesn't change — only the abstraction layer above it.

---

## Further Reading

- The **LC: Slide Rule** example demonstrates fixed-point thinking — its `SineApprox` method returns `sin(θ) × 10` to preserve one decimal place on integer hardware.
- The **LC: Church** example's `divide` method shows the basic integer division that all three techniques build upon.
- The **LC: Pairs** example shows how Church pairs `(a, b)` work — the same encoding used by rational numbers to represent fractions.
