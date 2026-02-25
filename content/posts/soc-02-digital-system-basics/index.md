---
title: "[SoC-02] Digital System Basics: The Foundation of Every Computer"
date: 2026-02-25
description: "A thorough review of the background knowledge needed to study digital computer systems — covering number systems, logic gates, Boolean algebra, combinational and sequential circuits."
categories: ["SoC Design"]
tags: ["SoC", "Digital Logic", "Boolean Algebra", "Logic Gates", "Combinational Circuits", "Sequential Circuits"]
series: ["SoC Design Course"]
series_order: 2
draft: false
---

{{< katex >}}

## Introduction

Before we can design a CPU, build a pipeline, or write firmware for an SoC, we need to speak the language that all digital hardware speaks: **binary logic**. This post is a comprehensive review of the foundational concepts you will need throughout the rest of this course.

Even if you have seen this material before, I encourage you to read through it carefully — a solid foundation here will make everything else much easier to understand.

---

## 1. Number Systems

Computers do not think in decimal. They think in **binary** — because at the physical level, a transistor is either ON or OFF, a voltage is either HIGH or LOW. But humans find binary cumbersome, so we also use hexadecimal and octal as convenient shorthands.

### 1.1 Decimal (Base-10)

The system we use every day. Each digit position represents a power of 10.

$$
(347)_{10} = 3 \times 10^2 + 4 \times 10^1 + 7 \times 10^0 = 300 + 40 + 7
$$

### 1.2 Binary (Base-2)

Each digit (called a **bit**) is either 0 or 1. Each position represents a power of 2.

$$
(1011)_2 = 1 \times 2^3 + 0 \times 2^2 + 1 \times 2^1 + 1 \times 2^0 = 8 + 0 + 2 + 1 = (11)_{10}
$$

**Common terminology:**

| Term | Meaning |
|------|---------|
| **Bit** | A single binary digit (0 or 1) |
| **Nibble** | 4 bits |
| **Byte** | 8 bits |
| **Word** | Typically 32 or 64 bits (architecture-dependent) |

### 1.3 Hexadecimal (Base-16)

Uses digits 0–9 and letters A–F. Each hex digit represents exactly **4 binary bits**, making it a compact way to write binary values.

| Hex | Binary | Decimal |
|-----|--------|---------|
| 0 | 0000 | 0 |
| 1 | 0001 | 1 |
| 2 | 0010 | 2 |
| 3 | 0011 | 3 |
| 4 | 0100 | 4 |
| 5 | 0101 | 5 |
| 6 | 0110 | 6 |
| 7 | 0111 | 7 |
| 8 | 1000 | 8 |
| 9 | 1001 | 9 |
| A | 1010 | 10 |
| B | 1011 | 11 |
| C | 1100 | 12 |
| D | 1101 | 13 |
| E | 1110 | 14 |
| F | 1111 | 15 |

**Example:**

$$
(2F3)_{16} = 2 \times 16^2 + 15 \times 16^1 + 3 \times 16^0 = 512 + 240 + 3 = (755)_{10}
$$

In binary: $2F3_{16} = 0010\ 1111\ 0011_2$

### 1.4 Octal (Base-8)

Uses digits 0–7. Each octal digit represents exactly **3 binary bits**. Less common today but still seen in Unix file permissions.

$$
(752)_8 = 7 \times 8^2 + 5 \times 8^1 + 2 \times 8^0 = 448 + 40 + 2 = (490)_{10}
$$

### 1.5 Conversion Summary

```
Binary ←──────→ Hexadecimal
  │     (group by 4 bits)
  │
  ├──────→ Octal
  │  (group by 3 bits)
  │
  └──────→ Decimal
     (positional weight sum)
```

**Decimal → Binary conversion** (repeated division by 2):

```
Example: Convert 25 to binary

25 ÷ 2 = 12  remainder 1  ← LSB
12 ÷ 2 = 6   remainder 0
 6 ÷ 2 = 3   remainder 0
 3 ÷ 2 = 1   remainder 1
 1 ÷ 2 = 0   remainder 1  ← MSB

Result: (11001)₂  →  Read remainders bottom-to-top
```

---

## 2. Logic Gates: The Building Blocks

Every digital circuit — from a simple LED controller to a billion-transistor SoC — is built from a small set of **logic gates**. Each gate takes one or more binary inputs and produces a binary output according to a fixed rule.

### 2.1 Basic Gates

#### NOT Gate (Inverter)

Flips the input: 0 becomes 1, 1 becomes 0.

$$
Y = \overline{A}
$$

| A | Y |
|---|---|
| 0 | 1 |
| 1 | 0 |

```
A ──►[▷○]──► Y
```

#### AND Gate

Output is 1 **only if all inputs are 1**.

$$
Y = A \cdot B
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 0 |
| 0 | 1 | 0 |
| 1 | 0 | 0 |
| 1 | 1 | 1 |

```
A ──┐
    ├──[&]──► Y
B ──┘
```

#### OR Gate

Output is 1 **if at least one input is 1**.

$$
Y = A + B
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 0 |
| 0 | 1 | 1 |
| 1 | 0 | 1 |
| 1 | 1 | 1 |

```
A ──┐
    ├──[≥1]──► Y
B ──┘
```

### 2.2 Universal Gates

#### NAND Gate

AND followed by NOT. This single gate is **universal** — any logic function can be built using only NAND gates.

$$
Y = \overline{A \cdot B}
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 1 |
| 0 | 1 | 1 |
| 1 | 0 | 1 |
| 1 | 1 | 0 |

#### NOR Gate

OR followed by NOT. Also a **universal** gate.

$$
Y = \overline{A + B}
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 1 |
| 0 | 1 | 0 |
| 1 | 0 | 0 |
| 1 | 1 | 0 |

> **Why are NAND and NOR called "universal"?** Because you can construct AND, OR, NOT, and any other gate using only NAND gates (or only NOR gates). In real chip manufacturing, CMOS NAND and NOR gates are the most natural structures to build from transistors.

### 2.3 XOR and XNOR

#### XOR (Exclusive OR)

Output is 1 when the inputs **differ**.

$$
Y = A \oplus B = A\overline{B} + \overline{A}B
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 0 |
| 0 | 1 | 1 |
| 1 | 0 | 1 |
| 1 | 1 | 0 |

XOR is extremely important for:
- **Arithmetic** (addition, parity checking)
- **Error detection** (CRC, parity bits)
- **Comparators** (checking if two values differ)

#### XNOR (Exclusive NOR)

Output is 1 when the inputs are **the same**.

$$
Y = \overline{A \oplus B} = AB + \overline{A}\,\overline{B}
$$

| A | B | Y |
|---|---|---|
| 0 | 0 | 1 |
| 0 | 1 | 0 |
| 1 | 0 | 0 |
| 1 | 1 | 1 |

### 2.4 Gate Summary

| Gate | Expression | Output = 1 when... |
|------|-----------|---------------------|
| NOT | $\overline{A}$ | Input is 0 |
| AND | $A \cdot B$ | All inputs are 1 |
| OR | $A + B$ | At least one input is 1 |
| NAND | $\overline{A \cdot B}$ | At least one input is 0 |
| NOR | $\overline{A + B}$ | All inputs are 0 |
| XOR | $A \oplus B$ | Inputs differ |
| XNOR | $\overline{A \oplus B}$ | Inputs are the same |

---

## 3. Boolean Algebra

Boolean algebra provides the mathematical framework for analyzing and simplifying digital logic circuits. Mastering these rules lets you reduce complex circuits to simpler, cheaper, faster equivalents.

### 3.1 Fundamental Laws

| Law | AND Form | OR Form |
|-----|----------|---------|
| **Identity** | $A \cdot 1 = A$ | $A + 0 = A$ |
| **Null** | $A \cdot 0 = 0$ | $A + 1 = 1$ |
| **Idempotent** | $A \cdot A = A$ | $A + A = A$ |
| **Complement** | $A \cdot \overline{A} = 0$ | $A + \overline{A} = 1$ |
| **Involution** | $\overline{\overline{A}} = A$ | — |
| **Commutative** | $A \cdot B = B \cdot A$ | $A + B = B + A$ |
| **Associative** | $(AB)C = A(BC)$ | $(A+B)+C = A+(B+C)$ |
| **Distributive** | $A(B+C) = AB+AC$ | $A+BC = (A+B)(A+C)$ |

### 3.2 De Morgan's Theorems

These two theorems are arguably the most important rules in digital design:

$$
\overline{A \cdot B} = \overline{A} + \overline{B}
$$

$$
\overline{A + B} = \overline{A} \cdot \overline{B}
$$

**In words:**
- "The complement of AND is OR of complements"
- "The complement of OR is AND of complements"

**Practical significance:** De Morgan's theorems let you convert between AND/OR representations, which is essential for implementing logic using only NAND or only NOR gates.

### 3.3 Simplification Example

Let's simplify the expression $Y = A\overline{B}C + A\overline{B}\,\overline{C} + AB\overline{C}$:

$$
Y = A\overline{B}(C + \overline{C}) + AB\overline{C}
$$

$$
Y = A\overline{B}(1) + AB\overline{C}
$$

$$
Y = A\overline{B} + AB\overline{C}
$$

$$
Y = A(\overline{B} + B\overline{C})
$$

$$
Y = A(\overline{B} + \overline{C})
$$

We reduced a 3-term expression to 2 terms — which means fewer gates, less area, less power, and shorter delay in hardware.

### 3.4 Karnaugh Maps (K-Maps)

For functions with up to 4–5 variables, **Karnaugh maps** provide a visual method for simplification. The key idea: arrange truth table entries in a grid where **adjacent cells differ by exactly one variable**, then group adjacent 1s into rectangles of power-of-2 size.

**Example:** Simplify $F(A, B, C, D) = \sum m(0, 1, 2, 5, 8, 9, 10)$

```
        CD
AB    00  01  11  10
  ┌────┬────┬────┬────┐
00│ 1  │ 1  │ 0  │ 1  │
  ├────┼────┼────┼────┤
01│ 0  │ 1  │ 0  │ 0  │
  ├────┼────┼────┼────┤
11│ 0  │ 0  │ 0  │ 0  │
  ├────┼────┼────┼────┤
10│ 1  │ 1  │ 0  │ 1  │
  └────┴────┴────┴────┘
```

Groupings:
- Group 1: cells (0,0), (0,1), (10,0), (10,1) → $\overline{B}\,\overline{D} + \overline{B}\,\overline{C}$ → simplifies to $\overline{B}\,\overline{D}$...

Let me walk through it more carefully:

- **Group of 4** — corners: m(0), m(2), m(8), m(10) → $\overline{B}\,\overline{D}$
- **Group of 2** — m(0), m(1) and m(8), m(9): → $\overline{B}\,\overline{C}$
- **Single** — m(5): $\overline{A}B\overline{C}D$

$$
F = \overline{B}\,\overline{D} + \overline{B}\,\overline{C} + \overline{A}B\overline{C}D
$$

The K-map gives us a **minimal sum-of-products** form. In real designs, EDA (Electronic Design Automation) tools do this optimization automatically for much larger circuits.

---

## 4. Combinational Circuits

**Combinational circuits** produce outputs that depend **only** on the current inputs — they have no memory. They are the "pure functions" of digital hardware.

### 4.1 Multiplexer (MUX)

A multiplexer selects **one of several inputs** and forwards it to the output, based on a select signal. Think of it as a digitally controlled switch.

**2-to-1 MUX:**

$$
Y = \overline{S} \cdot I_0 + S \cdot I_1
$$

```
I0 ──┐
     ├──[MUX]──► Y
I1 ──┘
       ↑
       S (select)
```

| S | Y |
|---|---|
| 0 | I₀ |
| 1 | I₁ |

**4-to-1 MUX** uses 2 select lines ($S_1 S_0$) to choose from 4 inputs.

**Where MUXes are used in CPUs:**
- Selecting between register data and immediate values for the ALU
- Choosing the next PC value (PC+4 vs. branch target)
- Data forwarding paths in pipelined processors

### 4.2 Decoder

A decoder takes an $n$-bit input and activates **exactly one** of $2^n$ output lines.

**2-to-4 Decoder:**

```
       ┌──────────┐
A ────►│          │──► Y0 = Ā·B̄
B ────►│ 2-to-4   │──► Y1 = Ā·B
       │ Decoder  │──► Y2 = A·B̄
       │          │──► Y3 = A·B
       └──────────┘
```

| A | B | Y3 | Y2 | Y1 | Y0 |
|---|---|----|----|----|----|
| 0 | 0 | 0 | 0 | 0 | 1 |
| 0 | 1 | 0 | 0 | 1 | 0 |
| 1 | 0 | 0 | 1 | 0 | 0 |
| 1 | 1 | 1 | 0 | 0 | 0 |

**Where decoders are used in CPUs:**
- Instruction decoding (opcode → control signals)
- Memory address decoding (selecting a memory bank)
- Register file access (selecting which register to read/write)

### 4.3 Encoder

The reverse of a decoder: takes $2^n$ input lines and produces an $n$-bit binary code indicating which input is active.

**Priority Encoder:** When multiple inputs are active simultaneously, the priority encoder outputs the code for the **highest-priority** input. This is essential for interrupt handling in SoCs.

### 4.4 Adder Circuits

Adders are the most fundamental arithmetic circuits and form the core of the ALU.

#### Half Adder

Adds two single bits:

$$
\text{Sum} = A \oplus B
$$

$$
\text{Carry} = A \cdot B
$$

| A | B | Sum | Carry |
|---|---|-----|-------|
| 0 | 0 | 0 | 0 |
| 0 | 1 | 1 | 0 |
| 1 | 0 | 1 | 0 |
| 1 | 1 | 0 | 1 |

#### Full Adder

Adds two bits **plus a carry-in** from the previous position:

$$
\text{Sum} = A \oplus B \oplus C_{in}
$$

$$
\text{Carry}_{out} = AB + C_{in}(A \oplus B)
$$

| A | B | Cin | Sum | Cout |
|---|---|-----|-----|------|
| 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 1 | 1 | 0 |
| 0 | 1 | 0 | 1 | 0 |
| 0 | 1 | 1 | 0 | 1 |
| 1 | 0 | 0 | 1 | 0 |
| 1 | 0 | 1 | 0 | 1 |
| 1 | 1 | 0 | 0 | 1 |
| 1 | 1 | 1 | 1 | 1 |

#### Ripple Carry Adder (RCA)

Chain N full adders together to add N-bit numbers:

```
A3 B3    A2 B2    A1 B1    A0 B0
 │  │     │  │     │  │     │  │
 ▼  ▼     ▼  ▼     ▼  ▼     ▼  ▼
┌────┐   ┌────┐   ┌────┐   ┌────┐
│ FA │◄──│ FA │◄──│ FA │◄──│ FA │◄── Cin=0
└──┬─┘   └──┬─┘   └──┬─┘   └──┬─┘
Cout S3      S2       S1       S0
```

**Problem:** The carry must **ripple** through all N stages. For a 32-bit adder, the worst-case delay is proportional to 32 gate delays — this is slow!

#### Carry Lookahead Adder (CLA)

Solves the ripple delay problem by computing carries **in parallel** using generate ($G$) and propagate ($P$) signals:

$$
G_i = A_i \cdot B_i \quad \text{(generate: this bit always produces a carry)}
$$

$$
P_i = A_i \oplus B_i \quad \text{(propagate: this bit passes an incoming carry)}
$$

$$
C_{i+1} = G_i + P_i \cdot C_i
$$

By expanding this recursion, all carries can be computed simultaneously in $O(\log N)$ gate delays instead of $O(N)$.

### 4.5 Arithmetic Logic Unit (ALU)

The ALU combines multiple functional units (adder, AND, OR, XOR, shift, comparator) with a MUX that selects the desired operation:

```
       A          B
       │          │
       ▼          ▼
   ┌───────────────────┐
   │       ALU         │
   │  ┌─────┐ ┌─────┐ │
   │  │Adder│ │ AND  │ │
   │  └──┬──┘ └──┬──┘ │
   │  ┌──┴──┐ ┌──┴──┐ │
   │  │ OR  │ │ XOR │ │
   │  └──┬──┘ └──┬──┘ │
   │     └───┬───┘     │
   │      [MUX]        │
   │        ↑           │
   │     ALU_Op         │
   └────────┬───────────┘
            │
         Result
     (+ Zero, Overflow flags)
```

---

## 5. Sequential Circuits

Unlike combinational circuits, **sequential circuits** have **memory** — their output depends not only on current inputs but also on the **history** of past inputs. This memory is what allows computers to store data, maintain state, and execute programs step by step.

### 5.1 Latches

#### SR Latch

The simplest memory element, built from two cross-coupled NOR or NAND gates:

```
S ──┐        ┌──► Q
    ├──[NOR]─┤
  ┌─┘        │
  │  ┌───────┘
  │  │
  └──┤
    ├──[NOR]─┤
R ──┘        └──► Q̄
```

| S | R | Q (next) | Meaning |
|---|---|----------|---------|
| 0 | 0 | Q (hold) | No change |
| 0 | 1 | 0 | Reset |
| 1 | 0 | 1 | Set |
| 1 | 1 | Undefined | **Forbidden!** |

**Problem:** The SR latch is **level-sensitive** — it responds to input changes at any time, which makes it hard to control in synchronous systems.

#### D Latch

Eliminates the forbidden state by using a single data input D and an enable signal:

$$
\text{When Enable = 1:} \quad Q = D
$$

$$
\text{When Enable = 0:} \quad Q = Q_{prev} \quad \text{(hold)}
$$

### 5.2 Flip-Flops

Flip-flops are **edge-triggered** — they only capture the input at the precise moment of a clock edge (usually the rising edge). This is essential for building reliable synchronous circuits.

#### D Flip-Flop

$$
Q_{next} = D \quad \text{(captured at rising edge of CLK)}
$$

```
      ┌─────────┐
D ───►│  D   Q  │───► Q
      │         │
CLK ─►│>        │
      │  D   Q̄  │───► Q̄
      └─────────┘
```

**Timing parameters:**

| Parameter | Symbol | Meaning |
|-----------|--------|---------|
| Setup time | $t_{setup}$ | D must be stable *before* clock edge |
| Hold time | $t_{hold}$ | D must remain stable *after* clock edge |
| Clock-to-Q delay | $t_{CQ}$ | Time from clock edge to valid Q output |

These parameters determine the **maximum clock frequency** of a synchronous circuit:

$$
T_{clk} \geq t_{CQ} + t_{comb} + t_{setup}
$$

Where $t_{comb}$ is the delay of the combinational logic between two flip-flops.

### 5.3 Registers

A **register** is a group of flip-flops that store a multi-bit value. For example, a 32-bit register consists of 32 D flip-flops sharing the same clock signal.

```
              32-bit Register
CLK ─────────────────────────────────►
         │     │     │          │
D[31] ─►[FF] [FF] [FF]  ...  [FF]◄─ D[0]
         │     │     │          │
       Q[31] Q[30] Q[29]     Q[0]
```

In a CPU:
- **Program Counter (PC)**: register holding the address of the current instruction
- **Register File**: array of 32 registers (in RISC-V), each 32 or 64 bits wide
- **Pipeline registers**: hold intermediate results between pipeline stages

### 5.4 Finite State Machines (FSMs)

An FSM is the general model for any sequential circuit. It consists of:

```
                 ┌─────────────────────────┐
                 │                          │
Input ──────────►│   Combinational Logic    │──────► Output
                 │   (Next State + Output)  │
                 └────────┬────────────────┘
                          │
                          ▼
                    ┌───────────┐
                    │  State    │
                    │ Register  │◄─── CLK
                    └─────┬─────┘
                          │
                          └─────────────► (fed back to combinational logic)
```

**Two types:**

| Type | Output Depends On | Example |
|------|-------------------|---------|
| **Moore** | Current state only | Traffic light controller |
| **Mealy** | Current state AND current input | Vending machine |

FSMs are used extensively in CPU control units, communication protocol handlers, and bus arbiters inside SoCs.

---

## 6. From Gates to Processors: The Big Picture

Now you can see how these building blocks stack up:

```
Level 0:  Transistors (NMOS, PMOS)
              ↓
Level 1:  Logic Gates (NAND, NOR, XOR, ...)
              ↓
Level 2:  Combinational Blocks (MUX, Decoder, Adder, ALU)
              ↓
Level 3:  Sequential Blocks (Flip-Flops, Registers, FSMs)
              ↓
Level 4:  Functional Units (Register File, Control Unit, Memory)
              ↓
Level 5:  Processor (CPU core with datapath + control)
              ↓
Level 6:  System-on-Chip (CPU + GPU + Accelerators + I/O)
```

In the upcoming posts, we will climb this ladder step by step — from computer arithmetic (Level 2) all the way up to a complete pipelined processor (Level 5) and embedded SoC software (Level 6).

---

## 7. Summary

Here is what we covered in this post:

| Topic | Key Takeaway |
|-------|-------------|
| **Number Systems** | Binary is the native language of hardware; hex is its human-friendly shorthand |
| **Logic Gates** | 7 fundamental gates (NOT, AND, OR, NAND, NOR, XOR, XNOR) build all digital circuits |
| **Boolean Algebra** | De Morgan's theorems and simplification rules minimize hardware cost |
| **Combinational Circuits** | MUX, decoder, encoder, and adders are the workhorses of datapaths |
| **Sequential Circuits** | Flip-flops and registers add memory; FSMs add control |
| **Timing** | Setup time, hold time, and propagation delay determine maximum clock speed |

In the **next post ([SoC-03])**, we will dive deep into **computer arithmetic** — how binary addition, subtraction, multiplication, and division actually work inside a processor, and how floating-point numbers are represented.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
