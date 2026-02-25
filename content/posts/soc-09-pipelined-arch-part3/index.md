---
title: "[SoC-09] Pipelined Architecture Part 3: Hazards and How to Overcome Them"
date: 2026-02-25
description: "A comprehensive guide to the three types of pipeline hazards — data, control, and structural — and the hardware and software techniques used to resolve them, including forwarding, stalling, and branch prediction."
categories: ["SoC Design"]
tags: ["SoC", "Pipeline Hazards", "Data Hazard", "Control Hazard", "Forwarding", "Branch Prediction", "RISC-V"]
series: ["SoC Design Course"]
series_order: 9
draft: false
---

{{< katex >}}

## Introduction

In [SoC-08], we built a pipelined RISC-V processor that can ideally complete one instruction per cycle. But in reality, certain instruction sequences create **hazards** — situations where the pipeline cannot continue at full speed because the next instruction depends on something that isn't ready yet.

Understanding hazards and their solutions is essential for any SoC designer. Let's dive deep into each type.

---

## 1. Three Types of Hazards

| Hazard Type | Cause | Example |
|-------------|-------|---------|
| **Structural** | Two instructions need the same hardware resource | Two instructions both need memory in the same cycle |
| **Data** | An instruction depends on the result of a previous instruction | `add x1, x2, x3` followed by `sub x4, x1, x5` |
| **Control** | The next instruction depends on the outcome of a branch | `beq x1, x2, L` — should we fetch from PC+4 or L? |

---

## 2. Structural Hazards

### 2.1 The Problem

A structural hazard occurs when the hardware cannot support the combination of instructions that the pipeline wants to execute in the same cycle.

**Classic example:** A processor with a **single unified memory** for both instructions and data:

```
Cycle 5:
I1: ──────────────────────── [WB]
I2: ───────────────── [MEM]  ◄── Needs to access D-Mem
I3: ──────────── [EX]
I4: ─────── [ID]
I5: ── [IF]                  ◄── Needs to access I-Mem (same memory!)
```

Both I2 (MEM stage) and I5 (IF stage) need memory access in the same cycle — but there's only one memory port!

### 2.2 Solutions

| Solution | How It Works |
|----------|-------------|
| **Separate memories** | Use separate I-Mem and D-Mem (Harvard architecture) — most common |
| **Stall** | Pause the pipeline for one cycle (insert a "bubble") |
| **Resource duplication** | Add more ports or duplicate the resource |

Our RISC-V design already uses **separate instruction and data memories**, so this particular structural hazard doesn't arise. However, structural hazards can still occur with shared resources like floating-point units or cache ports.

---

## 3. Data Hazards

Data hazards are the most common and most impactful. They occur when an instruction needs data that a previous instruction hasn't finished computing yet.

### 3.1 Types of Data Dependencies

| Type | Notation | Example | Occurs In Our Pipeline? |
|------|----------|---------|:-----------------------:|
| **RAW** (Read After Write) | I2 reads what I1 writes | `add x1,...` then `sub ...,x1,...` | Yes — the main problem |
| **WAR** (Write After Read) | I2 writes what I1 reads | `add ...,x1,...` then `sub x1,...` | No (in-order pipeline) |
| **WAW** (Write After Write) | I2 writes what I1 writes | `add x1,...` then `sub x1,...` | No (in-order pipeline) |

WAR and WAW hazards only occur in **out-of-order** or **superscalar** processors. For our simple in-order pipeline, we only need to worry about **RAW hazards**.

### 3.2 RAW Hazard Example

```asm
add  x1, x2, x3     # I1: writes x1
sub  x4, x1, x5     # I2: reads x1 — BUT x1 isn't written yet!
```

```
Cycle:    1     2     3     4     5     6     7
I1(add): [IF]  [ID]  [EX]  [MEM] [WB]
I2(sub):       [IF]  [ID]  [EX]  [MEM] [WB]
                       ↑                 ↑
                  I2 reads x1      I1 writes x1
                  (old value!)     (new value)
```

I2 reads x1 in cycle 3 (ID stage), but I1 doesn't write x1 until cycle 5 (WB stage). I2 gets the **stale** value — this is a bug!

### 3.3 Solution 1: Stalling (Bubbling)

The simplest solution: **stop the pipeline** until the data is ready.

```
Cycle:    1     2     3     4     5     6     7     8     9
I1(add): [IF]  [ID]  [EX]  [MEM] [WB]
                            stall stall
I2(sub):       [IF]  [ID]  [~~]  [~~]  [ID]  [EX]  [MEM] [WB]
```

The pipeline control unit detects the hazard and:
1. **Freezes** the IF/ID register (keeps fetching the same instruction)
2. Inserts a **bubble** (NOP) into the ID/EX register
3. Waits until the data is available

**Cost:** 2 stall cycles per hazard. This significantly degrades performance.

### 3.4 Solution 2: Forwarding (Bypassing)

**Key insight:** The result of `add x1, x2, x3` is actually computed at the end of the **EX stage** (cycle 3). Why wait until WB (cycle 5) to use it?

**Forwarding** adds extra paths that route results from later pipeline stages directly to where they are needed:

```
                      ┌──── Forward from EX/MEM ────┐
                      │                              │
Cycle:    1     2     3     4     5     6     7      │
I1(add): [IF]  [ID]  [EX]  [MEM] [WB]               │
I2(sub):       [IF]  [ID]  [EX]  [MEM] [WB]         │
                             ↑                        │
                        ALU input A ← Forwarded value─┘
```

**Forwarding hardware:**

```
                 ┌────────────────────────────────────┐
                 │           Forward from EX/MEM       │
                 │                                     │
RegFile[rs1] ──┐ │                                     │
               ├─┴──[MUX]──► ALU input A               │
EX/MEM.Result ─┤      ↑                                │
MEM/WB.Result ─┘   ForwardA                            │
                                                        │
RegFile[rs2] ──┐                                        │
               ├────[MUX]──► ALU input B                │
EX/MEM.Result ─┤      ↑                                │
MEM/WB.Result ─┘   ForwardB                            │
                                                        │
                Forwarding Unit                         │
                ┌──────────────────┐                    │
                │ if (EX/MEM.rd    │                    │
                │   == ID/EX.rs1)  │──► ForwardA = 10  │
                │                  │                    │
                │ if (MEM/WB.rd    │                    │
                │   == ID/EX.rs1)  │──► ForwardA = 01  │
                │                  │                    │
                │ else             │──► ForwardA = 00  │
                └──────────────────┘                    │
```

**Forwarding conditions:**

| Forward From | Condition | Priority |
|-------------|-----------|----------|
| EX/MEM | EX/MEM.RegWrite AND EX/MEM.rd == ID/EX.rs1 | High (most recent) |
| MEM/WB | MEM/WB.RegWrite AND MEM/WB.rd == ID/EX.rs1 | Low |
| None | No match | Use register file value |

**Note:** Never forward from x0 (rd = 0 should be ignored since x0 is hardwired to zero).

### 3.5 Load-Use Hazard: When Forwarding Isn't Enough

There is one case where forwarding alone cannot solve the problem:

```asm
lw   x1, 0(x2)      # I1: loads x1 from memory
add  x3, x1, x4     # I2: uses x1 immediately
```

```
Cycle:    1     2     3     4     5     6
I1(lw):  [IF]  [ID]  [EX]  [MEM] [WB]
I2(add):       [IF]  [ID]  [EX]  [MEM] [WB]
                             ↑     ↑
                        Need x1   x1 available
                        here!     here (too late!)
```

The load result isn't available until the **end of MEM** (cycle 4), but the `add` needs it at the **beginning of EX** (cycle 4) — they happen in the same cycle, but the data arrives too late!

**Solution: Stall + Forward**

We must insert **one bubble** (one cycle stall), then forward:

```
Cycle:    1     2     3     4     5     6     7
I1(lw):  [IF]  [ID]  [EX]  [MEM] [WB]
                            stall
I2(add):       [IF]  [ID]  [~~]  [EX]  [MEM] [WB]
                                   ↑
                            Forward from MEM/WB
```

**Hazard detection unit:**

```
if (ID/EX.MemRead == 1)        // Previous instruction is a load
  AND (ID/EX.rd == IF/ID.rs1   // AND the load destination matches
       OR ID/EX.rd == IF/ID.rs2)  // a source register of current instruction
then STALL for one cycle
```

### 3.6 Software Solution: Instruction Reordering

Compilers can often **reorder instructions** to avoid load-use hazards:

```c
// Original C code:
a = b + c;
d = e + f;
```

**Naive compilation (has load-use hazard):**

```asm
lw   x1, 0(x10)     # load b
lw   x2, 4(x10)     # load c
add  x3, x1, x2     # a = b + c  ← STALL (x2 not ready)
lw   x4, 8(x10)     # load e
lw   x5, 12(x10)    # load f
add  x6, x4, x5     # d = e + f  ← STALL (x5 not ready)
```

**Reordered (no stalls!):**

```asm
lw   x1, 0(x10)     # load b
lw   x2, 4(x10)     # load c
lw   x4, 8(x10)     # load e      ← moved here (fills the gap)
add  x3, x1, x2     # a = b + c   ← x2 now ready (2 cycles since load)
lw   x5, 12(x10)    # load f
add  x6, x4, x5     # d = e + f   ← x5... still has 1 gap, might need more reordering
```

Good compilers are remarkably effective at this kind of **instruction scheduling**.

---

## 4. Control Hazards

### 4.1 The Problem

When the processor encounters a **branch instruction**, it doesn't know which instruction to fetch next until the branch condition is evaluated:

```asm
beq  x1, x2, L      # Branch: should we go to L or PC+4?
add  x3, x4, x5     # ← Fetched speculatively (might be wrong!)
...
L: sub  x6, x7, x8
```

```
Cycle:    1     2     3     4     5
beq:     [IF]  [ID]  [EX]  [MEM] [WB]
                             ↑
                      Branch decision
                      known here

add:           [IF]  [ID]  [EX]  [MEM] [WB]
                ↑
          Fetched before we know
          if branch is taken!
```

By the time we know the branch outcome (cycle 4 in MEM stage), we've already fetched and started executing 3 wrong instructions!

### 4.2 Solution 1: Always Stall

The simplest (but slowest) approach: stall the pipeline for 3 cycles on every branch until the outcome is known.

**Cost:** If 20% of instructions are branches → 0.2 × 3 = 0.6 extra CPI. That's a 60% performance loss!

### 4.3 Solution 2: Early Branch Resolution

Move the branch comparison from MEM to the **ID stage** by adding a dedicated comparator:

```
ID Stage (enhanced):
┌──────────────────────────────────┐
│  RegFile[rs1] ──┐               │
│                  ├── [== ?] ──► Branch decision (1 cycle earlier!)
│  RegFile[rs2] ──┘               │
│                                  │
│  PC + Imm ──────────► Branch target                              │
└──────────────────────────────────┘
```

This reduces the branch penalty from 3 cycles to **1 cycle** (only one instruction fetched before the branch decision is known).

### 4.4 Solution 3: Branch Prediction

Instead of stalling, **predict** the branch outcome and continue fetching. If the prediction is correct, no penalty. If wrong, flush the incorrectly fetched instructions.

#### Static Prediction

Simple rules, fixed at design time:

| Strategy | How It Works | Accuracy |
|----------|-------------|:--------:|
| **Predict Not Taken** | Always fetch PC+4; flush if taken | ~50–60% |
| **Predict Taken** | Always fetch branch target; flush if not taken | ~60–70% |
| **Backward Taken, Forward Not Taken (BTFNT)** | Backward branches (loops) predicted taken, forward branches not taken | ~65–75% |

**Predict Not Taken** is the simplest to implement — just keep fetching the next sequential instruction. If the branch turns out to be taken, flush the incorrectly fetched instruction(s) and redirect to the branch target.

```
Cycle:    1     2     3     4     5
beq:     [IF]  [ID]  [EX]  [MEM] [WB]
                 ↑
           Branch resolved:
           NOT TAKEN → continue normally (no penalty!)
           TAKEN → flush next instruction, redirect PC (1 cycle penalty)
```

#### Dynamic Prediction

Uses **runtime history** to predict branches:

**1-bit Predictor:**

Each branch has a 1-bit counter: 0 = predict Not Taken, 1 = predict Taken. Updated after each branch execution.

Problem: A loop that runs 100 times will mispredict **twice** — on the first iteration (entering) and the last iteration (exiting).

**2-bit Predictor (Saturating Counter):**

```
            Taken
Strongly  ─────────►  Weakly    ─────────►  Weakly    ─────────►  Strongly
Not Taken              Not Taken              Taken                 Taken
  (00)    ◄─────────    (01)    ◄─────────    (10)    ◄─────────    (11)
           Not Taken             Not Taken             Not Taken
```

The prediction changes only after **two consecutive mispredictions**. This handles the loop case much better — only mispredicts once at the exit.

| States 00, 01 → Predict **Not Taken** |
| States 10, 11 → Predict **Taken** |

**Branch Target Buffer (BTB):**

A small cache that stores the **target address** of recently taken branches. When a branch instruction is fetched:

```
PC ──► [BTB lookup] ──► Hit? ──► Predicted target address
                         │
                      Miss? ──► Predict not taken (use PC+4)
```

#### Modern Branch Predictors

Modern processors use sophisticated predictors:

| Technique | Description | Accuracy |
|-----------|-------------|:--------:|
| 2-bit counter | Saturating counter per branch | ~85% |
| Correlating | Uses history of recent branches | ~90% |
| Tournament | Combines local + global predictors | ~95% |
| Neural (perceptron) | ML-based prediction | ~97% |
| TAGE | Tagged geometric history length | ~97%+ |

### 4.5 Branch Penalty Calculation

$$
\text{CPI}_{branch} = 1 + (\text{mispredict rate}) \times (\text{penalty cycles})
$$

**Example:** With 2-bit predictor (90% accuracy) and 1-cycle penalty (early resolution):

$$
\text{Branch CPI} = 1 + 0.10 \times 1 = 1.1
$$

If 20% of instructions are branches:

$$
\text{Overall CPI} = 1 + 0.20 \times 0.10 \times 1 = 1.02
$$

Only 2% overhead — excellent!

---

## 5. Putting It All Together: The Forwarding Unit

Here is the complete forwarding logic:

```
// Forward A (ALU input for rs1)
if (EX/MEM.RegWrite && EX/MEM.rd != 0 && EX/MEM.rd == ID/EX.rs1)
    ForwardA = 10   // Forward from EX/MEM (most recent)
else if (MEM/WB.RegWrite && MEM/WB.rd != 0 && MEM/WB.rd == ID/EX.rs1)
    ForwardA = 01   // Forward from MEM/WB
else
    ForwardA = 00   // No forwarding (use register file)

// Forward B (same logic for rs2)
if (EX/MEM.RegWrite && EX/MEM.rd != 0 && EX/MEM.rd == ID/EX.rs2)
    ForwardB = 10
else if (MEM/WB.RegWrite && MEM/WB.rd != 0 && MEM/WB.rd == ID/EX.rs2)
    ForwardB = 01
else
    ForwardB = 00
```

### 5.1 Complete Hazard Resolution for a Code Sequence

```asm
lw   x1, 0(x10)     # I1
add  x2, x1, x3     # I2: load-use hazard with I1 (1 stall + forward)
sub  x4, x2, x5     # I3: data hazard with I2 (forward from EX/MEM)
and  x6, x4, x7     # I4: data hazard with I3 (forward from EX/MEM)
or   x8, x6, x9     # I5: data hazard with I4 (forward from EX/MEM)
```

```
Cycle:  1    2    3    4    5    6    7    8    9
I1(lw): [IF] [ID] [EX] [MEM][WB]
I2(add):     [IF] [ID] [**] [EX] [MEM][WB]        ** = stall (load-use)
I3(sub):          [IF] [**] [ID] [EX] [MEM][WB]
I4(and):               [**] [IF] [ID] [EX] [MEM][WB]
I5(or):                     [IF] [ID] [EX] [MEM][WB]

Forwarding:
- I2 gets x1 from MEM/WB (after stall)
- I3 gets x2 from EX/MEM (forwarded)
- I4 gets x4 from EX/MEM (forwarded)
- I5 gets x6 from EX/MEM (forwarded)
```

**Total stalls: 1** (only for the load-use hazard). All other dependencies resolved by forwarding.

---

## 6. Pipeline Flush for Branches

When a branch is mispredicted, the incorrectly fetched instructions must be **flushed** (discarded):

```
Flush operation:
1. Set IF/ID pipeline register to NOP (zero out instruction)
2. Redirect PC to the correct target
3. Pipeline continues from the correct path

Cycle:    1     2     3     4     5     6
beq:     [IF]  [ID]
                 ↑ Branch taken!
wrong:         [IF] → FLUSHED (replaced with bubble)
correct:              [IF]  [ID]  [EX]  [MEM] [WB]
```

---

## 7. Exception Handling in the Pipeline

What happens when an instruction causes an **exception** (illegal instruction, overflow, page fault)?

The processor must:
1. **Complete** all instructions before the faulting instruction
2. **Flush** the faulting instruction and all later instructions
3. **Save** the PC of the faulting instruction (to return later)
4. **Jump** to the exception handler

This is called achieving **precise exceptions** — the processor state looks as if instructions executed one at a time in order, even though they were actually in a pipeline.

```
Cycle:    1     2     3     4     5     6
I1:      [IF]  [ID]  [EX]  [MEM] [WB]         ← completes normally
I2:            [IF]  [ID]  [EX]  💥EXCEPTION
I3:                  [IF]  [ID]  → FLUSHED
I4:                        [IF]  → FLUSHED
Handler:                         [IF]  [ID]  [EX]  ...
```

---

## 8. Performance Impact Summary

| Hazard | Without Solution | With Solution | Typical CPI Impact |
|--------|:----------------:|:-------------:|:------------------:|
| Structural | Stall every conflict | Separate I-Mem/D-Mem | ~0 |
| Data (RAW) | Stall 1–2 cycles | Forwarding | ~0.05 |
| Load-Use | Stall 1 cycle | Stall + Forward + Scheduling | ~0.1 |
| Control (Branch) | Stall 1–3 cycles | Prediction (95%+) | ~0.02 |
| **Total** | | | **CPI ≈ 1.1–1.3** |

---

## 9. Summary

| Hazard Type | Cause | Main Solution |
|-------------|-------|--------------|
| **Structural** | Resource conflict | Separate memories, resource duplication |
| **Data (RAW)** | Read-before-write dependency | Forwarding / Bypassing |
| **Load-Use** | Load result not ready for next instruction | 1-cycle stall + forwarding |
| **Control** | Branch outcome unknown | Prediction + early resolution |

| Technique | What It Does | Hardware Cost |
|-----------|-------------|---------------|
| **Forwarding** | Routes results directly to where needed | MUXes + forwarding unit |
| **Stalling** | Freezes pipeline, inserts bubbles | Hazard detection unit |
| **Branch Prediction** | Guesses branch outcome to avoid stalls | BTB + predictor tables |
| **Instruction Reordering** | Compiler schedules to avoid hazards | No hardware cost (compiler) |

In the **next post ([SoC-10])**, we move beyond the processor core to study **memory hierarchy** — the cache systems that bridge the enormous speed gap between the CPU and main memory.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
