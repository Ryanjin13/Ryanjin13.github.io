---
title: "CPU Design Fundamentals"
date: 2024-06-25
description: "Understanding the basics of processor design and datapath"
categories: ["Computer Science"]
tags: ["Computer Structure", "CPU Design", "Datapath"]
draft: false
---

{{< katex >}}

## Overview

CPU design involves creating the hardware that fetches, decodes, and executes instructions. This post covers fundamental concepts in processor datapath design.

## CPU Components

### Datapath Elements

```
┌─────────────────────────────────────────────────────────┐
│                        CPU                               │
│  ┌──────┐   ┌────┐   ┌─────┐   ┌─────┐   ┌──────┐      │
│  │  PC  │──→│I-Mem│──→│ Regs│──→│ ALU │──→│D-Mem │      │
│  └──────┘   └────┘   └─────┘   └─────┘   └──────┘      │
│      ↑                  ↑          ↓         ↓          │
│      └──────────────────┴──────────┴─────────┘          │
└─────────────────────────────────────────────────────────┘
```

### Key Components

| Component | Function |
|-----------|----------|
| PC | Program Counter - holds current instruction address |
| I-Mem | Instruction Memory |
| Registers | Fast storage (32 registers typical) |
| ALU | Arithmetic Logic Unit |
| D-Mem | Data Memory |

## Single-Cycle Datapath

### Instruction Fetch

```
PC ──→ [I-Mem] ──→ Instruction
 ↑
PC + 4
```

### R-Type Execution

```
Instruction
    ↓
[Decode: rs1, rs2, rd]
    ↓
[Read Registers]
    ↓
[ALU Operation]
    ↓
[Write to rd]
```

### Load Instruction

```
Instruction
    ↓
[Decode: rs1, imm, rd]
    ↓
[Read rs1] + imm ──→ Address
    ↓
[Read D-Mem at Address]
    ↓
[Write to rd]
```

## Control Signals

### ALU Control

| ALU Op | Function |
|--------|----------|
| 0000 | AND |
| 0001 | OR |
| 0010 | ADD |
| 0110 | SUB |
| 0111 | SLT |

### Main Control

| Signal | Meaning |
|--------|---------|
| RegWrite | Write to register file |
| MemRead | Read from data memory |
| MemWrite | Write to data memory |
| Branch | Conditional branch |
| ALUSrc | ALU second operand source |

## Pipelined Datapath

### Five-Stage Pipeline

```
IF → ID → EX → MEM → WB
│    │    │     │     │
↓    ↓    ↓     ↓     ↓
Fetch Decode Execute Memory Writeback
```

### Pipeline Registers

```
       IF/ID    ID/EX    EX/MEM   MEM/WB
         │        │        │        │
[IF] ──→ ║ ──→ [ID] ──→ ║ ──→ [EX] ──→ ║ ──→ [MEM] ──→ ║ ──→ [WB]
```

## Hazard Handling

### Data Hazards

**Forwarding (Bypassing):**

```
ADD R1, R2, R3   ; R1 = R2 + R3
SUB R4, R1, R5   ; R1 needed immediately
                   ↑
            Forward from EX/MEM
```

**Stalling:**

When forwarding isn't possible (load-use):

```
LW  R1, 0(R2)    ; Load R1
ADD R3, R1, R4   ; Need R1 - must stall
```

Insert bubble (NOP) for one cycle.

### Control Hazards

**Branch Prediction:**

| Strategy | Description |
|----------|-------------|
| Static | Always/never taken |
| Dynamic | Based on history |
| BTB | Branch Target Buffer |

$$
\text{CPI}_{branch} = 1 + p_{wrong} \times \text{penalty}
$$

## Performance Analysis

### CPI Calculation

$$
\text{CPI} = 1 + \text{stall cycles per instruction}
$$

### Pipeline Speedup

Ideal speedup = number of stages

$$
\text{Speedup} = \frac{n}{1 + \text{stall rate} \times \text{stall cycles}}
$$

## Advanced Techniques

### Superscalar

Execute multiple instructions per cycle:

```
Cycle 1: [IF IF] [ID ID] [EX EX] [MEM MEM] [WB WB]
```

Issue width = 2, 4, or more.

### Out-of-Order Execution

1. Fetch in order
2. Decode and rename registers
3. Execute when operands ready (out of order)
4. Commit in order

### Branch Prediction

$$
\text{Accuracy} = \frac{\text{Correct predictions}}{\text{Total branches}}
$$

Modern predictors achieve >95% accuracy.

## Design Trade-offs

| Approach | Pros | Cons |
|----------|------|------|
| Single-cycle | Simple | Long cycle time |
| Multi-cycle | Shorter cycle | Complex control |
| Pipelined | High throughput | Hazards |
| Superscalar | Higher IPC | Complex, power hungry |

## Critical Path

The longest path determines cycle time:

$$
T_{cycle} = \max(\text{all paths through combinational logic})
$$

Common critical paths:
- Memory access
- ALU operations
- Register file access
