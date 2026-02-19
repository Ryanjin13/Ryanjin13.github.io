---
title: "Latches and Flip-Flops: The Fundamentals of Digital Memory"
date: 2026-02-19
description: "A step-by-step guide to understanding latches and flip-flops — from basic SR latches built with NAND gates to D flip-flops and edge triggering, with clear diagrams at every step"
categories: ["Digital Design"]
tags: ["Latch", "Flip-Flop", "Digital Logic", "Sequential Circuit", "Hardware Basics"]
draft: false
---

{{< katex >}}

## Overview

In digital circuits, we need a way to **store** a single bit of information — a 0 or a 1. This is the most fundamental building block of all digital memory, from a single register bit in a CPU to gigabytes of SRAM in your cache.

The two basic storage elements are:
- **Latch**: A level-sensitive storage element (transparent when enabled)
- **Flip-Flop**: An edge-sensitive storage element (captures data only at the clock edge)

This post builds up from the simplest storage circuit to the D flip-flop used in every modern processor, one step at a time.

---

## 1. Before We Start: Logic Gates Review

Everything in this post is built from just two gates. Let's make sure we're comfortable with them.

### NAND Gate

A NAND gate outputs 0 **only** when both inputs are 1. Otherwise, it outputs 1.

```
  A ──┐
      │ NAND ──→ Y
  B ──┘

  A  B  │  Y
  ──────┼────
  0  0  │  1
  0  1  │  1
  1  0  │  1
  1  1  │  0     ← Only case where output is 0
```

### NOR Gate

A NOR gate outputs 1 **only** when both inputs are 0. Otherwise, it outputs 0.

```
  A ──┐
      │ NOR ──→ Y
  B ──┘

  A  B  │  Y
  ──────┼────
  0  0  │  1     ← Only case where output is 1
  0  1  │  0
  1  0  │  0
  1  1  │  0
```

---

## 2. SR Latch: The Simplest Memory

### 2.1 Building It with NOR Gates

Take two NOR gates and **cross-couple** them — feed each gate's output back into the other gate's input:

```
        ┌───────────┐
  S ───→│           │
        │   NOR 1   ├───→ Q ──────┐
   ┌───→│           │              │
   │    └───────────┘              │
   │                               │
   │    ┌───────────┐              │
   │    │           │              │
   └────┤   NOR 2   │←── R        │
        │           ├───→ Q̄ ──────┘
   ┌───→│           │      (fed back to NOR 1)
   │    └───────────┘
   │         │
   └─────────┘ (Q fed back to NOR 2)
```

- **S** = Set: Makes Q = 1
- **R** = Reset: Makes Q = 0

### 2.2 How It Works — Step by Step

Let's trace through each input combination carefully.

**Case 1: S=0, R=0 (Hold / Memory)**

Neither Set nor Reset is active. The latch **holds** its previous value.

```
If Q was 1 before:
  NOR 1: inputs are S=0, Q̄=0 → output Q = 1  ✓ (stays 1)
  NOR 2: inputs are R=0, Q=1  → output Q̄ = 0  ✓ (stays 0)

If Q was 0 before:
  NOR 1: inputs are S=0, Q̄=1 → output Q = 0  ✓ (stays 0)
  NOR 2: inputs are R=0, Q=0  → output Q̄ = 1  ✓ (stays 1)

→ The circuit remembers! This is memory.
```

**Case 2: S=1, R=0 (Set)**

We want to store a 1.

```
  NOR 1: inputs are S=1, Q̄=?  → output Q = 0... wait.

  Actually, let's trace carefully:
  NOR 1: S=1, anything → Q = 0  (NOR with a 1 input always outputs 0)

  Hmm, but we wanted Q=1. Let me re-examine the wiring...
```

Actually, the standard NOR-based SR latch has **active-high** S and R. Let me redraw more carefully:

```
  NOR-based SR Latch (corrected wiring):

        ┌───────────┐
  R ───→│   NOR 1   ├───→ Q ──────┐
   ┌───→│           │              │
   │    └───────────┘              │
   │                               │
   │    ┌───────────┐              │
   └────┤   NOR 2   │←───── S     │
   ┌───→│           ├───→ Q̄       │
   │    └───────────┘              │
   └───────────────────────────────┘
        (Q output fed back to NOR 2 input)
```

Now let's trace again:

**S=1, R=0 (Set → Q becomes 1):**

```
Step 1: NOR 2 has inputs S=1, Q=? → Q̄ = 0  (any 1 input → NOR outputs 0)
Step 2: NOR 1 has inputs R=0, Q̄=0 → Q = 1  (both inputs 0 → NOR outputs 1)
Step 3: Stable! Q=1, Q̄=0 ✓
```

**S=0, R=1 (Reset → Q becomes 0):**

```
Step 1: NOR 1 has inputs R=1, Q̄=? → Q = 0  (any 1 input → NOR outputs 0)
Step 2: NOR 2 has inputs S=0, Q=0 → Q̄ = 1  (both inputs 0 → NOR outputs 1)
Step 3: Stable! Q=0, Q̄=1 ✓
```

**S=1, R=1 (Forbidden!)**

```
NOR 1: R=1 → Q = 0
NOR 2: S=1 → Q̄ = 0
Both outputs are 0 → Q and Q̄ are no longer complementary!
When both S and R return to 0 simultaneously, the output is unpredictable.
→ This combination is FORBIDDEN.
```

### 2.3 SR Latch Truth Table

| S | R | Q (next) | Meaning |
|---|---|----------|---------|
| 0 | 0 | Q (no change) | **Hold** — memory state |
| 1 | 0 | 1 | **Set** — store 1 |
| 0 | 1 | 0 | **Reset** — store 0 |
| 1 | 1 | ??? | **Forbidden** — undefined behavior |

---

## 3. Gated SR Latch: Adding Control

The basic SR latch responds to S and R **immediately** — there is no control over *when* changes happen. We fix this by adding an **enable** signal:

```
Gated SR Latch:

  S ──→[AND]──→ S' ──┐
  EN ─→[   ]         │    ┌───────────┐
                      └───→│           │
                           │  SR Latch ├──→ Q
                      ┌───→│           │
  R ──→[AND]──→ R' ──┘    └───────────┘
  EN ─→[   ]

  When EN=0: S'=0, R'=0 → Latch holds (no change)
  When EN=1: S'=S, R'=R → Latch responds to S, R
```

Now the latch only changes state when **EN is high**. This is the concept of **level-sensitive** control — the latch is "transparent" while EN=1 and "opaque" while EN=0.

---

## 4. D Latch: Eliminating the Forbidden State

The SR latch has a forbidden state (S=R=1). The **D latch** eliminates this problem by using a single data input:

```
D Latch:

  D ──────────→[AND]──→ S' ──┐
       │       [   ]←── EN    │    ┌───────────┐
       │                      └───→│           │
       │                           │  SR Latch ├──→ Q
       │                      ┌───→│           │
       └──[NOT]──→[AND]──→ R'┘    └───────────┘
                  [   ]←── EN

  S' = D AND EN
  R' = (NOT D) AND EN

  When D=1: S'=EN, R'=0  → Sets the latch (Q=1)
  When D=0: S'=0, R'=EN  → Resets the latch (Q=0)

  S' and R' can NEVER both be 1 simultaneously!
  → Forbidden state is structurally impossible.
```

### D Latch Truth Table

| EN | D | Q (next) | Behavior |
|----|---|----------|----------|
| 0 | X | Q (no change) | **Latch is opaque** — holds value |
| 1 | 0 | 0 | **Transparent** — Q follows D |
| 1 | 1 | 1 | **Transparent** — Q follows D |

When EN=1, Q simply **follows** D (transparent). When EN=0, Q **holds** its last value.

### Timing Diagram

```
EN:    ┌──────────┐          ┌──────────┐
  ─────┘          └──────────┘          └─────

D:  ───┐  ┌──┐  ┌───────────┐  ┌──┐
       └──┘  └──┘           └──┘  └───────

Q:  ───┐  ┌──┐  ┌──────────────────┐
       └──┘  └──┘                  └──────
       ↑              ↑            ↑
    Transparent    Holds last    Transparent
    (Q follows D)  value (D=1)  (Q follows D)
```

---

## 5. The Problem with Latches: Why We Need Flip-Flops

Latches are **transparent** while enabled. This causes a critical problem in synchronous circuits:

```
The Problem:

  CLK ──→ [D Latch A] ──→ [D Latch B] ──→ ...
          (EN = CLK)       (EN = CLK)

  When CLK=1: BOTH latches are transparent!
  Data "races" through A and into B in the same clock phase.
  B should wait for A to finish, but it doesn't.

  → Data may propagate through multiple stages
    in a single clock cycle = RACE CONDITION
```

The solution: make the storage element respond only to the **edge** of the clock, not the **level**. This is a **flip-flop**.

---

## 6. D Flip-Flop: Edge-Triggered Storage

### 6.1 Master-Slave Construction

A D flip-flop is built from **two D latches in series**, with inverted enable signals:

```
D Flip-Flop (Master-Slave):

             CLK        CLK (inverted)
              │              │
              ▼              ▼
  D ──→ [D Latch] ──→ [D Latch] ──→ Q
         (Master)       (Slave)
         EN = !CLK      EN = CLK

  When CLK=0: Master is transparent (captures D)
              Slave is opaque (holds output)

  When CLK=1: Master is opaque (holds captured value)
              Slave is transparent (passes master's value to Q)
```

### 6.2 Step-by-Step Operation

**Phase 1: CLK = 0 (Setup Phase)**

```
  D ──→ [Master: OPEN] ──→ Qm ──→ [Slave: CLOSED] ──→ Q (unchanged)

  Master captures whatever D is.
  Slave holds its previous value — output Q does NOT change.
```

**Phase 2: CLK transitions 0 → 1 (The Critical Moment)**

```
  D ──→ [Master: CLOSING] ──→ Qm ──→ [Slave: OPENING] ──→ Q = Qm

  Master closes and locks in the value of D.
  Slave opens and passes Qm to the output.
  Q takes on the value that D had at the rising edge.
```

**Phase 3: CLK = 1 (Hold Phase)**

```
  D ──→ [Master: CLOSED] ──→ Qm ──→ [Slave: OPEN] ──→ Q (stable)

  Master is closed — D can change freely, master ignores it.
  Slave passes the locked master value — Q is stable.
```

### 6.3 Key Insight

The flip-flop **samples D at the rising edge of CLK** and holds that value until the next rising edge. Changes to D at any other time are ignored.

```
CLK:  ─────┐     ┌─────┐     ┌─────┐     ┌─────
           └─────┘     └─────┘     └─────┘

D:    ═══1═══════0═══════1═══1═══════0════════

Q:    ────────┐1┌─────────0──────────┐1┌──────
              └─┘                    └─┘
              ↑           ↑           ↑
           D was 1     D was 0     D was 1
           at edge     at edge     at edge
```

---

## 7. D Flip-Flop Variants

### 7.1 With Asynchronous Reset

```
always @(posedge clk or posedge reset) begin
    if (reset)
        q <= 0;     // Immediately reset, don't wait for clock
    else
        q <= d;
end

Behavior:
  reset=1 → Q=0 immediately (regardless of clock)
  reset=0 → Q captures D on rising clock edge
```

### 7.2 With Synchronous Reset

```
always @(posedge clk) begin
    if (reset)
        q <= 0;     // Reset only on clock edge
    else
        q <= d;
end

Behavior:
  reset=1 + clock edge → Q=0
  reset=1 + no clock edge → Q unchanged (waits for clock!)
```

### 7.3 With Enable

```
always @(posedge clk) begin
    if (enable)
        q <= d;     // Capture D only when enabled
    // else: Q retains its value
end

Behavior:
  enable=1 + clock edge → Q captures D
  enable=0 + clock edge → Q unchanged (holds)
```

---

## 8. Latch vs Flip-Flop: Summary

```
D Latch:                          D Flip-Flop:

EN ──────┐     ┌──────            CLK ──┐  ┌──┐  ┌──┐  ┌──
         └─────┘                        └──┘  └──┘  └──┘

D:  ──┐ ┌─┐ ┌────────            D:  ──┐ ┌─┐ ┌────────
      └─┘ └─┘                          └─┘ └─┘

Q:  ──┐ ┌─┐ ┌───── ──            Q:  ────┐   ┌──────────
      └─┘ └─┘                          └───┘
   ↑                               ↑
   Q follows D while               Q changes ONLY at
   EN is high (transparent)        clock rising edges
```

| Property | Latch | Flip-Flop |
|----------|-------|-----------|
| **Trigger** | Level-sensitive | Edge-sensitive |
| **When transparent** | Entire time EN=1 | Only at clock edge |
| **Construction** | 1 stage | 2 latches (master-slave) |
| **Gate count** | Fewer | More (~2x a latch) |
| **Timing analysis** | Complex (time borrowing) | Simple (clear boundaries) |
| **In modern design** | Rarely used intentionally | The standard storage element |

---

## 9. Why Flip-Flops Matter

Every register in a CPU, every bit of SRAM, every pipeline stage in a GPU — they all rely on flip-flops (or latch-based variants). A modern processor contains **billions** of flip-flops.

```
A Single CPU Register (8-bit):

  D[7] ──→ [D-FF] ──→ Q[7]
  D[6] ──→ [D-FF] ──→ Q[6]
  D[5] ──→ [D-FF] ──→ Q[5]
  D[4] ──→ [D-FF] ──→ Q[4]
  D[3] ──→ [D-FF] ──→ Q[3]
  D[2] ──→ [D-FF] ──→ Q[2]
  D[1] ──→ [D-FF] ──→ Q[1]
  D[0] ──→ [D-FF] ──→ Q[0]
               ↑
              CLK (shared by all bits)

  8 D flip-flops working together = one 8-bit register.
  A 64-bit CPU register = 64 D flip-flops.
  A processor with 32 architectural registers = 32 × 64 = 2,048 flip-flops.
  (And that's just the programmer-visible registers —
   the actual count is millions more for pipeline stages,
   caches, and control logic.)
```

Understanding latches and flip-flops is understanding the **heartbeat of all digital systems**. Every computation happens between clock edges, every result is captured by a flip-flop, and every pipeline stage is defined by the registers at its boundaries.
