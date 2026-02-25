---
title: "[SoC-03] Computer Arithmetic: How Computers Calculate"
date: 2026-02-25
description: "A detailed exploration of the binary number system and how addition, subtraction, multiplication, and division work at the hardware level — including two's complement, overflow detection, and IEEE 754 floating-point representation."
categories: ["SoC Design"]
tags: ["SoC", "Binary Arithmetic", "Two's Complement", "IEEE 754", "Floating Point", "ALU", "Computer Architecture"]
series: ["SoC Design Course"]
series_order: 3
draft: false
---

{{< katex >}}

## Introduction

In the previous post, we reviewed digital system fundamentals — number systems, logic gates, and basic circuits. Now it's time to get our hands dirty with the question that matters most for building a processor:

> **How does a computer actually perform arithmetic?**

This is not just an academic question. The way numbers are represented and manipulated in binary directly affects:
- **ALU design** (how many gates, how fast)
- **Instruction set architecture** (what operations to support)
- **Correctness** (overflow, rounding, precision errors)
- **Performance** (carry propagation, multiplier latency)

Let's start from the basics and build up to the full picture.

---

## 1. Unsigned Binary Integers

### 1.1 Representation

An $n$-bit unsigned integer represents values from $0$ to $2^n - 1$:

$$
V = \sum_{i=0}^{n-1} b_i \cdot 2^i
$$

Where $b_i$ is the bit at position $i$ (0 = LSB, $n-1$ = MSB).

| Bits (n) | Range | Max Value |
|----------|-------|-----------|
| 8 | 0 to 255 | $2^8 - 1$ |
| 16 | 0 to 65,535 | $2^{16} - 1$ |
| 32 | 0 to 4,294,967,295 | $2^{32} - 1$ |

### 1.2 Unsigned Binary Addition

Binary addition follows the same column-by-column approach as decimal addition, but with simpler rules:

| A | B | Cin | Sum | Cout |
|---|---|-----|-----|------|
| 0 | 0 | 0 | 0 | 0 |
| 0 | 1 | 0 | 1 | 0 |
| 1 | 0 | 0 | 1 | 0 |
| 1 | 1 | 0 | 0 | 1 |
| 0 | 0 | 1 | 1 | 0 |
| 0 | 1 | 1 | 0 | 1 |
| 1 | 0 | 1 | 0 | 1 |
| 1 | 1 | 1 | 1 | 1 |

**Example:** Add $13 + 11$ in 8-bit unsigned binary:

```
  Carry:  0 1 1 0 0 0
          0 0 0 0 1 1 0 1   (13)
        + 0 0 0 0 1 0 1 1   (11)
        ─────────────────
          0 0 0 1 1 0 0 0   (24) ✓
```

**Overflow detection (unsigned):** If there is a carry out of the MSB position, the result doesn't fit in $n$ bits. For example, in 8-bit unsigned: $200 + 100 = 300 > 255$ → overflow!

---

## 2. Signed Binary Integers: Two's Complement

Real programs need negative numbers. Several representations have been tried historically:

| Method | Representation of -5 (8-bit) | Problems |
|--------|------------------------------|----------|
| **Sign-magnitude** | 10000101 | Two zeros (+0, -0), complex addition |
| **One's complement** | 11111010 | Two zeros, end-around carry |
| **Two's complement** | 11111011 | **One zero, simple addition** ✓ |

Modern computers universally use **two's complement** because it makes the adder hardware simple — the same circuit handles both signed and unsigned addition.

### 2.1 Two's Complement Definition

For an $n$-bit signed integer:

$$
V = -b_{n-1} \cdot 2^{n-1} + \sum_{i=0}^{n-2} b_i \cdot 2^i
$$

The MSB ($b_{n-1}$) has a **negative weight**. This is the key insight.

| Bits (n) | Range | Min | Max |
|----------|-------|-----|-----|
| 8 | $-128$ to $+127$ | $-2^7$ | $2^7 - 1$ |
| 16 | $-32{,}768$ to $+32{,}767$ | $-2^{15}$ | $2^{15} - 1$ |
| 32 | $-2{,}147{,}483{,}648$ to $+2{,}147{,}483{,}647$ | $-2^{31}$ | $2^{31} - 1$ |

Notice the **asymmetry**: there is one more negative number than positive. For 8-bit: you can represent $-128$ but not $+128$.

### 2.2 How to Negate (Find -X)

To compute the two's complement (negation) of a number:

**Method 1: Invert and add 1**

$$
-X = \overline{X} + 1
$$

**Example:** Find the representation of $-6$ in 8-bit two's complement:

```
Step 1: Start with +6        →  00000110
Step 2: Invert all bits       →  11111001
Step 3: Add 1                 →  11111010  ← This is -6
```

**Verification:** $-128 + 64 + 32 + 16 + 8 + 0 + 2 + 0 = -128 + 122 = -6$ ✓

**Method 2: Subtract from $2^n$**

$$
-X = 2^n - X
$$

For 8-bit: $-6 = 256 - 6 = 250 = 11111010_2$ ✓

### 2.3 Sign Extension

When you need to represent a smaller number in more bits (e.g., loading an 8-bit value into a 32-bit register), you **extend the sign bit** to the left:

```
8-bit:   11111010          (-6)
16-bit:  11111111 11111010  (-6)  ← sign bit (1) copied to all new positions
32-bit:  11111111 11111111 11111111 11111010  (-6)

8-bit:   00000110          (+6)
16-bit:  00000000 00000110  (+6)  ← sign bit (0) copied
```

This preserves the numeric value. In RISC-V, the `LB` (Load Byte) instruction does sign extension, while `LBU` (Load Byte Unsigned) does zero extension.

---

## 3. Signed Addition and Subtraction

### 3.1 Addition with Two's Complement

The beauty of two's complement: **the same adder circuit works for both signed and unsigned addition.** You just ignore the final carry out.

**Example 1:** $(-3) + 5 = 2$

```
  11111101   (-3)
+ 00000101   (+5)
──────────
 100000010   → discard carry → 00000010 = +2 ✓
```

**Example 2:** $(-3) + (-5) = -8$

```
  11111101   (-3)
+ 11111011   (-5)
──────────
 111111000   → discard carry → 11111000 = -8 ✓
```

### 3.2 Subtraction

Subtraction is implemented as **addition of the negated value**:

$$
A - B = A + (-B) = A + \overline{B} + 1
$$

In hardware, this is trivially implemented:

```
                  A         B
                  │         │
                  │    ┌────┴────┐
                  │    │ XOR w/  │
                  │    │ Sub ctrl│
                  │    └────┬────┘
                  │         │
                  ▼         ▼
              ┌─────────────────┐
 Sub ────────►│  Carry-in       │
              │     ADDER       │
              │                 │
              └────────┬────────┘
                       │
                       ▼
                    Result
```

When the `Sub` control signal is 1:
- Each bit of B is XOR'd with 1 (inverting it → $\overline{B}$)
- The carry-in is set to 1 (adding the +1)
- Result: $A + \overline{B} + 1 = A - B$

This dual-purpose adder/subtractor is what the ALU uses — **one circuit, two operations**.

### 3.3 Overflow Detection (Signed)

Signed overflow occurs when the result is too large (or too small) to fit in the number of bits available. It happens when adding two numbers of the **same sign** and getting a result of the **opposite sign**.

$$
\text{Overflow} = C_{n-1} \oplus C_{n-2}
$$

(XOR of the carry into and carry out of the MSB position)

| Operation | Operands | Overflow? |
|-----------|----------|-----------|
| $(+A) + (+B)$ | Both positive | Yes, if result is negative |
| $(-A) + (-B)$ | Both negative | Yes, if result is positive |
| $(+A) + (-B)$ | Mixed signs | **Never overflows** |
| $(-A) + (+B)$ | Mixed signs | **Never overflows** |

**Example:** 8-bit signed: $100 + 50 = 150$, but $150 > 127$ (max for 8-bit signed) → overflow!

```
  01100100   (+100)
+ 00110010   (+50)
──────────
  10010110   → interpreted as -106 (wrong!)
              → sign changed from 0 to 1 → OVERFLOW detected
```

---

## 4. Binary Multiplication

### 4.1 Pencil-and-Paper Method

Binary multiplication works just like decimal long multiplication, but simpler — each partial product is either 0 (multiply by 0) or a shifted copy of the multiplicand (multiply by 1).

**Example:** $13 \times 11 = 143$

```
          1 1 0 1   (13 = multiplicand)
        × 1 0 1 1   (11 = multiplier)
        ─────────
          1 1 0 1   (13 × 1, shift 0)
        1 1 0 1     (13 × 1, shift 1)
      0 0 0 0       (13 × 0, shift 2)
    1 1 0 1         (13 × 1, shift 3)
    ─────────────
  1 0 0 0 1 1 1 1   (143) ✓
```

**Key observation:** Multiplying two $n$-bit numbers produces a result up to $2n$ bits wide. This is why the RISC-V `MUL` instruction stores only the lower 32 bits, while `MULH` stores the upper 32 bits.

### 4.2 Hardware Multiplier Architectures

#### Sequential Multiplier

The simplest approach: examine one bit of the multiplier per clock cycle, conditionally add and shift.

```
Cycle 0: Check multiplier bit 0 → if 1, add multiplicand to accumulator
         Shift multiplicand left (or accumulator right)
Cycle 1: Check multiplier bit 1 → if 1, add
         Shift
...
Cycle N-1: Check multiplier bit N-1 → if 1, add
```

- **N cycles** for N-bit multiplication
- **Small area** (just one adder + shift register)
- **Slow** for large N

#### Array Multiplier

Generates all partial products simultaneously and adds them using an array of adders:

```
         b3  b2  b1  b0
    ×    a3  a2  a1  a0
    ─────────────────────
         a0b3 a0b2 a0b1 a0b0   ← row 0 (AND gates)
    a1b3 a1b2 a1b1 a1b0        ← row 1
a2b3 a2b2 a2b1 a2b0            ← row 2
...
```

Each partial product bit is simply $a_i \cdot b_j$ (an AND gate). The rows are summed using carry-save adders.

- **1 cycle** (purely combinational)
- **Large area** ($O(N^2)$ AND gates and adders)
- **Fast** but expensive

#### Booth's Algorithm

An optimization for signed multiplication that reduces the number of additions by encoding runs of 1s in the multiplier:

- A run of 1s like `0111110` is replaced by `+1000000 - 0000010` (one addition and one subtraction instead of five additions).

Booth's encoding is used in most modern high-performance multipliers.

### 4.3 Multiplication Summary

| Architecture | Cycles | Area | Use Case |
|-------------|--------|------|----------|
| Sequential | N | Small | Low-power embedded |
| Array | 1 | Large ($O(N^2)$) | High-performance |
| Wallace Tree | 1 | Large | Fastest combinational |
| Booth-encoded | ~N/2 | Medium | Signed, general purpose |

---

## 5. Binary Division

### 5.1 Restoring Division

Binary division follows a similar approach to long division in decimal. At each step, we try to subtract the divisor from the current partial remainder:

1. **Shift** the partial remainder left by 1 bit, bringing in the next dividend bit
2. **Subtract** the divisor
3. If the result is **non-negative**: the quotient bit is 1 (keep the result)
4. If the result is **negative**: the quotient bit is 0 (**restore** the previous value)

**Example:** $7 \div 2$ (4-bit: 0111 ÷ 0010)

```
Step 0: Remainder = 0000, Dividend = 0111

Step 1: Shift left → 00001 (bring in bit 3 of dividend: '0')
        Subtract 0010 → 00001 - 00010 = negative
        Quotient bit = 0, Restore → 00001

Step 2: Shift left → 00010 (bring in bit 2: '1')
        Subtract 0010 → 00010 - 00010 = 00000
        Quotient bit = 1, Keep → 00000

Step 3: Shift left → 00001 (bring in bit 1: '1')
        Subtract 0010 → 00001 - 00010 = negative
        Quotient bit = 0, Restore → 00001

Step 4: Shift left → 00011 (bring in bit 0: '1')
        Subtract 0010 → 00011 - 00010 = 00001
        Quotient bit = 1, Keep → 00001

Result: Quotient = 0011 (3), Remainder = 0001 (1)
Check: 2 × 3 + 1 = 7 ✓
```

### 5.2 Non-Restoring Division

An optimization: instead of restoring when the subtraction gives a negative result, we **add** the divisor in the next step instead of subtracting. This saves one addition operation per step.

### 5.3 Division in Processors

Division is the **slowest** basic arithmetic operation:
- Takes ~30–40 cycles for 32-bit division (compared to 1 cycle for addition, 3–5 cycles for multiplication)
- Some embedded processors (like simple RISC-V cores) don't include a hardware divider at all
- Compilers often replace division by constants with multiplication by the reciprocal (a much faster operation)

---

## 6. IEEE 754 Floating-Point Representation

Integers alone are not enough. Scientific computing, graphics, and AI all need to represent very large numbers (like $3.0 \times 10^{38}$) and very small numbers (like $1.0 \times 10^{-45}$) with fractional precision. This is what **floating-point** numbers are for.

### 6.1 The Idea: Scientific Notation in Binary

Just like decimal scientific notation:

$$
-6.022 \times 10^{23} \quad \text{(decimal)}
$$

We can write binary numbers as:

$$
(-1)^s \times 1.f \times 2^{e} \quad \text{(binary)}
$$

Where:
- $s$ = sign bit (0 = positive, 1 = negative)
- $1.f$ = significand (also called mantissa), with an implicit leading 1
- $e$ = exponent

### 6.2 IEEE 754 Formats

The IEEE 754 standard defines two common formats:

| Format | Total Bits | Sign | Exponent | Fraction (Mantissa) | Bias |
|--------|-----------|------|----------|---------------------|------|
| **Single (float)** | 32 | 1 | 8 | 23 | 127 |
| **Double (double)** | 64 | 1 | 11 | 52 | 1023 |

```
Single Precision (32 bits):
┌──┬──────────┬───────────────────────────────┐
│S │ Exponent │          Fraction              │
│1 │  8 bits  │          23 bits               │
└──┴──────────┴───────────────────────────────┘
31  30      23 22                             0
```

### 6.3 Value Interpretation

$$
\text{Value} = (-1)^s \times (1 + \text{Fraction}) \times 2^{(\text{Exponent} - \text{Bias})}
$$

The **bias** converts the unsigned exponent field to a signed effective exponent. For single precision (bias = 127):
- Exponent field = 0000 0001 (1) → effective exponent = $1 - 127 = -126$ (smallest normal)
- Exponent field = 0111 1111 (127) → effective exponent = $127 - 127 = 0$
- Exponent field = 1111 1110 (254) → effective exponent = $254 - 127 = +127$ (largest normal)

### 6.4 Worked Example

**Represent $-12.625$ in IEEE 754 single precision:**

**Step 1: Convert to binary**

$12 = 1100_2$

$0.625 = 0.101_2$ (because $0.5 + 0.125 = 0.625$)

$12.625 = 1100.101_2$

**Step 2: Normalize**

$1100.101 = 1.100101 \times 2^3$

**Step 3: Extract fields**

- Sign: 1 (negative)
- Exponent: $3 + 127 = 130 = 10000010_2$
- Fraction: $10010100000000000000000$ (23 bits, drop the leading 1)

**Result:**

```
1  10000010  10010100000000000000000
S  Exponent  Fraction
```

In hex: `0xC14A0000`

### 6.5 Special Values

| Exponent | Fraction | Meaning |
|----------|----------|---------|
| 0 | 0 | **Zero** ($+0$ or $-0$) |
| 0 | ≠ 0 | **Denormalized** (subnormal) — very small numbers near zero |
| 255 (all 1s) | 0 | **Infinity** ($+\infty$ or $-\infty$) |
| 255 (all 1s) | ≠ 0 | **NaN** (Not a Number — e.g., $0/0$, $\sqrt{-1}$) |

### 6.6 Floating-Point Range and Precision

**Single precision:**

| Property | Value |
|----------|-------|
| Smallest positive normal | $\approx 1.18 \times 10^{-38}$ |
| Largest finite | $\approx 3.40 \times 10^{38}$ |
| Decimal digits of precision | ~7.2 |
| Machine epsilon | $2^{-23} \approx 1.19 \times 10^{-7}$ |

**Double precision:**

| Property | Value |
|----------|-------|
| Smallest positive normal | $\approx 2.23 \times 10^{-308}$ |
| Largest finite | $\approx 1.80 \times 10^{308}$ |
| Decimal digits of precision | ~15.9 |
| Machine epsilon | $2^{-52} \approx 2.22 \times 10^{-16}$ |

### 6.7 Floating-Point Arithmetic

#### Addition / Subtraction

Adding two floating-point numbers requires several steps:

```
Step 1: Align exponents (shift smaller number's mantissa right)
Step 2: Add/subtract mantissas
Step 3: Normalize the result
Step 4: Round to fit the available precision
Step 5: Check for overflow/underflow
```

**Example:** $1.0 \times 2^3 + 1.0 \times 2^1$

```
Step 1: Align → 1.000 × 2³ + 0.010 × 2³  (shifted right by 2)
Step 2: Add  → 1.010 × 2³
Step 3: Already normalized
Result: 1.010 × 2³ = 10.10₂ = 10.5₁₀

Check: 8 + 2 = 10... Wait: 1.0 × 2³ = 8, 1.0 × 2¹ = 2, sum = 10
But 1.010 × 2³ = 1010₂ = 10₁₀ ✓
```

#### Multiplication

Simpler than addition:

```
Step 1: Multiply mantissas (integer multiplication)
Step 2: Add exponents (and subtract bias once)
Step 3: Determine sign (XOR of sign bits)
Step 4: Normalize and round
```

$$
(M_1 \times 2^{E_1}) \times (M_2 \times 2^{E_2}) = (M_1 \times M_2) \times 2^{E_1 + E_2}
$$

#### Rounding Modes

IEEE 754 defines four rounding modes:

| Mode | Rule | Example (to integer) |
|------|------|---------------------|
| Round to Nearest Even | Default; round to nearest, tie to even | 2.5 → 2, 3.5 → 4 |
| Round toward Zero | Truncate | 2.7 → 2, -2.7 → -2 |
| Round toward +∞ | Ceiling | 2.1 → 3, -2.9 → -2 |
| Round toward -∞ | Floor | 2.9 → 2, -2.1 → -3 |

"Round to Nearest Even" (also called "banker's rounding") is the default because it minimizes statistical bias over many operations.

---

## 7. Floating-Point Pitfalls

Every engineer should be aware of these common issues:

### 7.1 Precision Loss

Not all decimal fractions can be exactly represented in binary floating-point:

$$
0.1_{10} = 0.0\overline{0011}_{2} \quad \text{(repeating!)}
$$

This is why `0.1 + 0.2 ≠ 0.3` in most programming languages — it's not a bug, it's a fundamental limitation of binary representation.

### 7.2 Catastrophic Cancellation

When subtracting two nearly equal numbers, most significant digits cancel, leaving only the noisy low-order digits:

$$
1.000000 \times 10^7 - 9.999999 \times 10^6 = 1.000000
$$

The result has only 1 significant digit, even though the inputs had 7 each. This is critical in numerical algorithms and must be handled carefully (e.g., using the numerically stable form of the quadratic formula).

### 7.3 Associativity Failure

Floating-point addition is **not associative**:

$$
(a + b) + c \neq a + (b + c) \quad \text{in general}
$$

This matters for parallel computing: if you split a sum across multiple cores, the order of operations affects the result. Reproducible numerical computing requires careful attention to this.

---

## 8. Relevance to SoC and AI

Why does all of this matter for the SoC designer?

### 8.1 ALU Design

The arithmetic circuits we discussed — adders, multipliers, dividers — are the core of the ALU. Design choices (ripple carry vs. CLA, sequential vs. array multiplier) directly impact:

- **Clock frequency** (shorter critical path → higher $f$)
- **Area** (fewer gates → smaller die → lower cost)
- **Power** (fewer switching transistors → less energy)

### 8.2 AI and Reduced Precision

As we discussed in [SoC-01], AI workloads are tolerant of reduced precision. The arithmetic hardware for INT8 multiplication is **dramatically** smaller and more efficient than FP32:

| Operation | Relative Area | Relative Energy |
|-----------|:------------:|:---------------:|
| FP32 multiply | 16× | 19× |
| FP16 multiply | 4× | 4× |
| INT8 multiply | 1× | 1× |

This is why AI accelerators (NPUs) in modern SoCs use INT8 or even INT4 arithmetic for inference — it's the key to achieving high TOPS (Tera Operations Per Second) within a tight power budget.

### 8.3 Floating-Point Units (FPU)

High-performance CPUs include dedicated FPU hardware that handles IEEE 754 operations in a pipelined fashion. In RISC-V, the **F extension** adds single-precision FP instructions, and the **D extension** adds double-precision.

---

## 9. Summary

| Topic | Key Takeaway |
|-------|-------------|
| **Unsigned integers** | $n$ bits → range $[0, 2^n - 1]$; overflow when carry out of MSB |
| **Two's complement** | Universal signed representation; negate by inverting + adding 1 |
| **Addition/Subtraction** | Same adder circuit for signed and unsigned; subtraction = add negated |
| **Overflow** | Signed: same-sign inputs produce different-sign result |
| **Multiplication** | Partial products + addition; $n$-bit × $n$-bit = $2n$-bit result |
| **Division** | Slowest operation; restoring/non-restoring algorithms |
| **IEEE 754** | $(-1)^s \times 1.f \times 2^{e-\text{bias}}$; special values for 0, ∞, NaN |
| **FP pitfalls** | Precision loss, cancellation, non-associativity |
| **AI relevance** | INT8 arithmetic is 16× cheaper than FP32 — key for NPU design |

In the **next post ([SoC-04])**, we will begin exploring **Instruction Set Architecture (ISA)** — the contract between software and hardware that defines what a CPU can actually do.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
