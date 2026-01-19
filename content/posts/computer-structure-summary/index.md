---
title: "Computer Structure Summary"
date: 2024-06-25
description: "Overview of computer architecture fundamentals"
categories: ["Computer Science"]
tags: ["Computer Structure", "CPU", "Architecture"]
draft: false
---

## Overview

Computer structure describes how hardware components are organized to execute programs. Understanding computer architecture is fundamental for system programming and optimization.

## Von Neumann Architecture

```
┌─────────────────────────────────────────┐
│              Memory                      │
│     (Instructions and Data)              │
└───────────────┬─────────────────────────┘
                │ Bus
┌───────────────┴─────────────────────────┐
│                 CPU                      │
│  ┌──────────┐    ┌──────────────────┐   │
│  │ Control  │    │    Datapath      │   │
│  │  Unit    │    │  ┌────┐ ┌────┐   │   │
│  └──────────┘    │  │ALU │ │Regs│   │   │
│                  │  └────┘ └────┘   │   │
│                  └──────────────────┘   │
└─────────────────────────────────────────┘
                │
┌───────────────┴─────────────────────────┐
│           I/O Devices                    │
└─────────────────────────────────────────┘
```

### Key Principles

1. **Stored program:** Instructions in memory
2. **Sequential execution:** Fetch-decode-execute
3. **Single memory:** Data and instructions shared

## CPU Components

### Control Unit

- Fetches instructions
- Decodes opcodes
- Generates control signals
- Manages program counter

### Datapath

- **ALU:** Arithmetic Logic Unit
- **Registers:** Fast storage
- **Multiplexers:** Data routing
- **Buses:** Data transfer

### Registers

| Register | Purpose |
|----------|---------|
| PC | Program Counter |
| IR | Instruction Register |
| MAR | Memory Address Register |
| MDR | Memory Data Register |
| Accumulator | Result storage |

## Instruction Cycle

```
┌────────┐
│ Fetch  │ ← Get instruction from memory
└───┬────┘
    ↓
┌───┴────┐
│ Decode │ ← Interpret instruction
└───┬────┘
    ↓
┌───┴────┐
│Execute │ ← Perform operation
└───┬────┘
    ↓
┌───┴────┐
│ Store  │ ← Write results
└────────┘
```

## Memory Hierarchy

```
        ┌─────────┐
        │Registers│  ← Fastest, smallest
        ├─────────┤
        │ L1 Cache│
        ├─────────┤
        │ L2 Cache│
        ├─────────┤
        │ L3 Cache│
        ├─────────┤
        │  DRAM   │  ← Main memory
        ├─────────┤
        │  SSD    │
        ├─────────┤
        │  HDD    │  ← Slowest, largest
        └─────────┘
```

### Memory Characteristics

| Level | Size | Latency |
|-------|------|---------|
| Registers | ~KB | <1 ns |
| L1 Cache | 32-64 KB | ~1 ns |
| L2 Cache | 256 KB - 1 MB | ~4 ns |
| L3 Cache | 2-32 MB | ~12 ns |
| DRAM | 8-64 GB | ~100 ns |
| SSD | 256 GB - 4 TB | ~100 μs |
| HDD | 1-10 TB | ~10 ms |

## Instruction Set Architecture (ISA)

### CISC vs RISC

| Aspect | CISC | RISC |
|--------|------|------|
| Instructions | Complex, variable length | Simple, fixed length |
| Addressing modes | Many | Few |
| Execution | Multi-cycle | Single cycle (pipelined) |
| Examples | x86 | ARM, RISC-V |

### Common Instructions

| Type | Examples |
|------|----------|
| Data transfer | LOAD, STORE, MOV |
| Arithmetic | ADD, SUB, MUL, DIV |
| Logic | AND, OR, XOR, NOT |
| Control | JMP, CALL, RET |
| Comparison | CMP, TEST |

## Pipelining

```
Time:    1   2   3   4   5   6   7
Inst 1: [IF][ID][EX][MEM][WB]
Inst 2:     [IF][ID][EX][MEM][WB]
Inst 3:         [IF][ID][EX][MEM][WB]
Inst 4:             [IF][ID][EX][MEM][WB]
```

### Pipeline Stages

1. **IF:** Instruction Fetch
2. **ID:** Instruction Decode
3. **EX:** Execute
4. **MEM:** Memory access
5. **WB:** Write Back

### Hazards

| Type | Cause | Solution |
|------|-------|----------|
| Structural | Resource conflict | More hardware |
| Data | RAW dependency | Forwarding, stall |
| Control | Branch | Prediction, delay slot |

## Parallelism

### Instruction Level Parallelism (ILP)

- Superscalar: Multiple instructions per cycle
- Out-of-order execution
- Branch prediction

### Thread Level Parallelism (TLP)

- Simultaneous multithreading (SMT)
- Multi-core processors

### Data Level Parallelism (DLP)

- SIMD: Single Instruction Multiple Data
- Vector processing
- GPU computing

## Performance Equation

$$
\text{CPU Time} = \text{Instructions} \times \text{CPI} \times \text{Clock Period}
$$

Where:
- CPI: Cycles Per Instruction
- Clock Period = 1 / Clock Frequency

### Improving Performance

| Method | Reduces |
|--------|---------|
| Better algorithms | Instruction count |
| Better ISA | CPI |
| Better implementation | CPI, clock period |
| Better circuits | Clock period |
