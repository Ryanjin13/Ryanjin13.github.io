---
title: "[SoC-08] Pipelined Architecture Part 2: Turning a Single-Cycle CPU into a Pipeline"
date: 2026-02-25
description: "Understanding the pipeline concept — how breaking instruction execution into stages dramatically improves processor throughput, and how to apply this technique to the RISC-V single-cycle architecture."
categories: ["SoC Design"]
tags: ["SoC", "Pipeline", "CPU Design", "RISC-V", "Throughput", "Latency"]
series: ["SoC Design Course"]
series_order: 8
draft: false
---

{{< katex >}}

## Introduction

In [SoC-07], we built a single-cycle RISC-V processor. It works, but it is slow — every instruction takes 850 ps because the clock must accommodate the slowest instruction (load). Most instructions finish much sooner and waste the remaining time.

The solution is **pipelining** — the single most important technique in computer architecture for improving throughput.

---

## 1. The Pipeline Concept

### 1.1 The Laundry Analogy

Imagine doing four loads of laundry. Each load requires:
1. **Wash** (30 min)
2. **Dry** (30 min)
3. **Fold** (30 min)

**Without pipelining** (sequential):

```
Time:   0    30   60   90   120  150  180  210  240  270  300  330  360
Load 1: [WASH][DRY ][FOLD]
Load 2:                    [WASH][DRY ][FOLD]
Load 3:                                      [WASH][DRY ][FOLD]
Load 4:                                                        [WASH][DRY ][FOLD]

Total: 360 minutes
```

**With pipelining** (overlap stages):

```
Time:   0    30   60   90   120  150  180
Load 1: [WASH][DRY ][FOLD]
Load 2:       [WASH][DRY ][FOLD]
Load 3:             [WASH][DRY ][FOLD]
Load 4:                   [WASH][DRY ][FOLD]

Total: 180 minutes  (2× speedup!)
```

**Key insight:** Pipelining doesn't make any single load faster (each still takes 90 min). It improves **throughput** — loads are completed more frequently.

### 1.2 Pipeline Terminology

| Term | Definition |
|------|-----------|
| **Throughput** | Number of instructions completed per unit time |
| **Latency** | Time for one instruction from start to finish |
| **Pipeline stage** | One step of the pipeline |
| **Pipeline depth** | Number of stages |
| **Pipeline register** | Storage between stages to hold intermediate results |

---

## 2. Five-Stage RISC-V Pipeline

We divide instruction execution into **five stages**, each taking one clock cycle:

| Stage | Abbreviation | Work Done |
|-------|-------------|-----------|
| 1. Instruction Fetch | **IF** | Read instruction from I-Mem, increment PC |
| 2. Instruction Decode | **ID** | Read registers, decode instruction, generate control signals |
| 3. Execute | **EX** | ALU operation, compute branch target |
| 4. Memory Access | **MEM** | Read/write data memory |
| 5. Write Back | **WB** | Write result to register file |

```
┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐
│ IF  │─►│ ID  │─►│ EX  │─►│ MEM │─►│ WB  │
└─────┘  └─────┘  └─────┘  └─────┘  └─────┘
```

### 2.1 Stage Details

**IF (Instruction Fetch):**
```
PC → I-Mem → Instruction
PC ← PC + 4
Store {Instruction, PC+4} in IF/ID register
```

**ID (Instruction Decode):**
```
Read IF/ID register
Decode opcode, extract rs1, rs2, rd, immediate
Read RegFile[rs1] and RegFile[rs2]
Generate control signals
Store {control, reg_data1, reg_data2, imm, rd} in ID/EX register
```

**EX (Execute):**
```
Read ID/EX register
ALU performs operation (add, sub, etc.)
Compute branch target = PC + offset
Store {control, ALU_result, reg_data2, rd} in EX/MEM register
```

**MEM (Memory Access):**
```
Read EX/MEM register
If load: ReadData = D-Mem[ALU_result]
If store: D-Mem[ALU_result] = reg_data2
Store {control, ALU_result, ReadData, rd} in MEM/WB register
```

**WB (Write Back):**
```
Read MEM/WB register
If RegWrite: RegFile[rd] = ALU_result or ReadData
```

---

## 3. Pipeline Registers

Between each pair of stages, we insert a **pipeline register** that captures all the data and control signals needed by the next stage:

```
        IF/ID        ID/EX        EX/MEM       MEM/WB
          │            │            │            │
[IF] ──►  ║  ──► [ID] ──►  ║  ──► [EX] ──►  ║  ──► [MEM] ──►  ║  ──► [WB]
          │            │            │            │
       Stores:      Stores:      Stores:      Stores:
       - Instr      - Control    - Control    - Control
       - PC+4       - RegData1   - ALU result - ALU result
                    - RegData2   - RegData2   - MemData
                    - Imm        - rd         - rd
                    - rd
                    - rs1, rs2
```

**Why pipeline registers?**
- They **isolate** each stage so it can work independently
- They **save** the current instruction's intermediate data while the next stage processes the previous instruction's data
- They ensure each stage takes exactly one clock cycle

---

## 4. Pipeline Execution Example

Let's trace five instructions through the pipeline:

```asm
I1: add  x1, x2, x3
I2: sub  x4, x5, x6
I3: and  x7, x8, x9
I4: or   x10, x11, x12
I5: slt  x13, x14, x15
```

```
Cycle:    1     2     3     4     5     6     7     8     9
I1:      [IF]  [ID]  [EX]  [MEM] [WB]
I2:             [IF]  [ID]  [EX]  [MEM] [WB]
I3:                   [IF]  [ID]  [EX]  [MEM] [WB]
I4:                         [IF]  [ID]  [EX]  [MEM] [WB]
I5:                               [IF]  [ID]  [EX]  [MEM] [WB]
```

**Observations:**
- **Cycle 5**: All five stages are active simultaneously, each working on a different instruction. This is the steady state.
- **Throughput**: After the pipeline fills (cycle 5), one instruction completes every cycle.
- **Latency**: Each instruction still takes 5 cycles from start to finish.

### 4.1 Pipeline Speedup

$$
\text{Speedup}_{ideal} = \frac{T_{single-cycle}}{T_{pipelined}} = \frac{N \times T_{stage} \times k}{(N + k - 1) \times T_{stage}} \approx k \quad \text{(for large } N\text{)}
$$

Where:
- $N$ = number of instructions
- $k$ = number of pipeline stages
- $T_{stage}$ = time for one pipeline stage

For our 5-stage pipeline: **ideal speedup = 5×**

**In practice**, the speedup is less than ideal due to:
1. Pipeline stages may not be perfectly balanced (some stages take longer)
2. Pipeline fill and drain time (at program start and end)
3. **Hazards** — situations that prevent the next instruction from executing in the next clock cycle

---

## 5. Clock Period in a Pipelined Processor

### 5.1 Single-Cycle vs. Pipelined Clock

**Single-cycle:**

$$
T_{cycle} = T_{IF} + T_{ID} + T_{EX} + T_{MEM} + T_{WB} = 200 + 100 + 200 + 200 + 100 = 800\ \text{ps}
$$

**Pipelined:**

$$
T_{cycle} = \max(T_{IF}, T_{ID}, T_{EX}, T_{MEM}, T_{WB}) + T_{reg}
$$

$$
T_{cycle} = 200 + 20 = 220\ \text{ps}
$$

(Where $T_{reg} = 20$ ps is the overhead of the pipeline register)

**Speedup:**

$$
\text{Speedup} = \frac{800}{220} \approx 3.6\times
$$

Not quite 5× because the stages aren't perfectly balanced (ID and WB are faster than IF, EX, MEM).

### 5.2 Impact of Imbalanced Stages

```
Stage durations:
IF:  200 ps  ████████████████████
ID:  100 ps  ██████████
EX:  200 ps  ████████████████████
MEM: 200 ps  ████████████████████
WB:  100 ps  ██████████

Pipeline clock = 200 ps (+ register overhead)

ID and WB waste:  100 ps each per cycle (idle time)
```

The clock is determined by the **slowest** stage. Faster stages simply finish early and wait. This is why pipeline designers try to **balance** the stages (make them take roughly equal time).

---

## 6. Pipelined Datapath Diagram

The pipelined datapath is the single-cycle datapath with pipeline registers inserted:

```
  ┌─────────── IF ──────────┐  ┌────── ID ──────┐  ┌────── EX ──────┐  ┌───── MEM ─────┐  ┌──── WB ────┐
  │                          │  │                 │  │                 │  │                │  │             │
  │  ┌────┐    ┌──────┐     │  │  ┌──────────┐  │  │   ┌──────┐    │  │  ┌──────┐     │  │             │
  │  │ PC │──►│I-Mem │─────║──║─►│ RegFile  │──║──║──►│ ALU  │────║──║──►│D-Mem │─────║──║──►[MUX]──┐ │
  │  └─┬──┘    └──────┘     ║  ║  │ + Decode │  ║  ║   └──────┘    ║  ║  └──────┘     ║  ║           │ │
  │    │                     ║  ║  └──────────┘  ║  ║               ║  ║               ║  ║           │ │
  │  [+4]                   ║  ║  [Imm Gen]     ║  ║  [MUX]        ║  ║               ║  ║           ▼ │
  │    │                     ║  ║               ║  ║               ║  ║               ║  ║    RegFile  │
  │    └─────────────────────╝  ╚───────────────╝  ╚───────────────╝  ╚───────────────╝  ╚────Write───┘
  │                         IF/ID             ID/EX             EX/MEM             MEM/WB              │
  │                                                                                                     │
  └──────────────────◄─────────────────────── Write-back path ──────────────────────────────────────────┘
```

**Key detail:** The write-back path goes from WB all the way back to the register file in the ID stage. This creates a potential **hazard** — what if a later instruction reads a register that an earlier instruction hasn't written back yet? We'll tackle this in [SoC-09].

---

## 7. Control Signal Propagation

In the single-cycle design, control signals are generated once and used immediately. In the pipelined design, control signals must travel with the instruction through the pipeline registers:

```
                   Generated     Used in
                   in ID stage   later stages
                   ─────────────────────────
RegWrite     ──────────────────────────────► WB
MemToReg     ──────────────────────────────► WB
Branch       ─────────────────────► MEM
MemRead      ─────────────────────► MEM
MemWrite     ─────────────────────► MEM
ALUOp        ──────────► EX
ALUSrc       ──────────► EX
```

Control signals are split into groups and stored in pipeline registers:

```
ID/EX register stores: ALL control signals
EX/MEM register stores: MEM + WB signals (EX signals consumed)
MEM/WB register stores: WB signals only (MEM signals consumed)
```

At each stage, the relevant signals are "peeled off" and used, while the remaining signals pass through to the next stage.

---

## 8. Pipeline Performance Analysis

### 8.1 CPI in a Pipelined Processor

In an ideal pipeline with no hazards:

$$
\text{CPI}_{ideal} = 1
$$

One instruction completes per clock cycle (after the pipeline fills).

**Effective CPI with hazards:**

$$
\text{CPI}_{actual} = 1 + \text{stall cycles per instruction}
$$

### 8.2 Pipeline Throughput

$$
\text{Throughput} = \frac{1}{\text{CPI} \times T_{cycle}} \quad \text{(instructions per second)}
$$

**Example comparison:**

| Design | CPI | T_cycle | Throughput |
|--------|:---:|:-------:|:----------:|
| Single-cycle | 1 | 800 ps | 1.25 GHz |
| 5-stage pipeline (ideal) | 1 | 220 ps | 4.55 GHz |
| 5-stage pipeline (realistic) | 1.2 | 220 ps | 3.79 GHz |

Even with some stalls (CPI = 1.2), the pipeline is **3× faster** than single-cycle.

### 8.3 Deeper Pipelines

Some processors use much deeper pipelines:

| Processor | Pipeline Depth | Year |
|-----------|:--------------:|:----:|
| MIPS R2000 | 5 | 1985 |
| Intel Pentium | 5 | 1993 |
| ARM Cortex-A9 | 8 | 2007 |
| Intel Core i7 (Skylake) | 14–19 | 2015 |
| Intel Pentium 4 (Prescott) | 31 | 2004 |

Deeper pipelines allow shorter clock periods but increase hazard penalties and power consumption. The Pentium 4's 31-stage pipeline was widely considered "too deep" — it had high branch misprediction penalties and consumed too much power.

---

## 9. Why Pipelining Works So Well

| Advantage | Explanation |
|-----------|-------------|
| **Higher throughput** | Multiple instructions in-flight simultaneously |
| **Better hardware utilization** | Every stage is busy every cycle (ideally) |
| **Same ISA** | Software doesn't need to change — pipelining is invisible to the programmer |
| **Scalable** | Can add more stages for higher clock frequency |

| Limitation | Explanation |
|------------|-------------|
| **Hazards** | Dependencies between instructions cause stalls |
| **Latency unchanged** | Each instruction still takes $k$ cycles |
| **Diminishing returns** | Deeper pipelines have higher hazard penalties |
| **Power overhead** | Pipeline registers consume energy |

---

## 10. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Pipelining** | Overlap instruction execution stages to increase throughput |
| **5-stage pipeline** | IF → ID → EX → MEM → WB |
| **Pipeline registers** | Store intermediate data between stages |
| **Ideal speedup** | Equal to pipeline depth (5× for 5-stage) |
| **Actual speedup** | Less than ideal due to imbalanced stages and hazards |
| **CPI** | Ideal = 1; actual = 1 + stall rate |
| **Clock period** | Determined by the slowest pipeline stage + register overhead |
| **Control propagation** | Control signals flow through pipeline registers alongside data |

In the **next post ([SoC-09])**, we will tackle the biggest challenge of pipelining: **hazards** — the situations that prevent the pipeline from running at full speed, and the clever techniques (forwarding, stalling, branch prediction) used to overcome them.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
