---
title: "RISC-V R-Type Instructions"
date: 2024-06-25
description: "Understanding R-type instruction format in RISC-V architecture"
categories: ["Computer Science"]
tags: ["RISC-V", "Computer Structure", "ISA"]
draft: false
---

## Overview

RISC-V is an open-source instruction set architecture. R-type instructions perform register-to-register operations, the most common instruction type.

## R-Type Instruction Format

### Bit Fields

```
31       25 24    20 19    15 14  12 11     7 6      0
┌──────────┬────────┬────────┬──────┬────────┬────────┐
│  funct7  │   rs2  │   rs1  │funct3│   rd   │ opcode │
│  (7 bit) │ (5 bit)│ (5 bit)│(3bit)│ (5 bit)│ (7 bit)│
└──────────┴────────┴────────┴──────┴────────┴────────┘
```

### Field Descriptions

| Field | Bits | Purpose |
|-------|------|---------|
| opcode | 6:0 | Operation class (0110011 for R-type) |
| rd | 11:7 | Destination register |
| funct3 | 14:12 | Operation type |
| rs1 | 19:15 | Source register 1 |
| rs2 | 24:20 | Source register 2 |
| funct7 | 31:25 | Operation variant |

## Basic R-Type Instructions

### Arithmetic Operations

| Instruction | funct7 | funct3 | Operation |
|-------------|--------|--------|-----------|
| ADD | 0000000 | 000 | rd = rs1 + rs2 |
| SUB | 0100000 | 000 | rd = rs1 - rs2 |
| SLL | 0000000 | 001 | rd = rs1 << rs2 |
| SLT | 0000000 | 010 | rd = (rs1 < rs2) ? 1 : 0 |
| SLTU | 0000000 | 011 | rd = (rs1 < rs2) unsigned |
| XOR | 0000000 | 100 | rd = rs1 ^ rs2 |
| SRL | 0000000 | 101 | rd = rs1 >> rs2 (logical) |
| SRA | 0100000 | 101 | rd = rs1 >> rs2 (arithmetic) |
| OR | 0000000 | 110 | rd = rs1 | rs2 |
| AND | 0000000 | 111 | rd = rs1 & rs2 |

## Logical Instructions

### Bitwise Operations

```
AND:  rd = rs1 & rs2
      1010 & 1100 = 1000

OR:   rd = rs1 | rs2
      1010 | 1100 = 1110

XOR:  rd = rs1 ^ rs2
      1010 ^ 1100 = 0110
```

### Shift Operations

```
SLL (Shift Left Logical):
      rs1 = 0001_0100
      rs2 = 2
      rd  = 0101_0000

SRL (Shift Right Logical):
      rs1 = 1000_0100
      rs2 = 2
      rd  = 0010_0001

SRA (Shift Right Arithmetic):
      rs1 = 1000_0100 (negative)
      rs2 = 2
      rd  = 1110_0001 (sign-extended)
```

## Comparison Instructions

### SLT (Set Less Than)

```assembly
SLT rd, rs1, rs2    ; rd = (rs1 < rs2) ? 1 : 0
```

Signed comparison:
- If rs1 < rs2 (signed), rd = 1
- Otherwise, rd = 0

### SLTU (Set Less Than Unsigned)

```assembly
SLTU rd, rs1, rs2   ; rd = (rs1 < rs2) ? 1 : 0
```

Unsigned comparison.

## Encoding Example

### ADD x5, x6, x7

```
rs2 = x7  = 00111
rs1 = x6  = 00110
rd  = x5  = 00101
funct7    = 0000000
funct3    = 000
opcode    = 0110011

Binary: 0000000_00111_00110_000_00101_0110011
Hex:    0x007302B3
```

### SUB x5, x6, x7

```
funct7    = 0100000  (different from ADD)
funct3    = 000      (same as ADD)

Binary: 0100000_00111_00110_000_00101_0110011
Hex:    0x407302B3
```

## Register Conventions

| Register | ABI Name | Purpose |
|----------|----------|---------|
| x0 | zero | Hardwired zero |
| x1 | ra | Return address |
| x2 | sp | Stack pointer |
| x5-x7 | t0-t2 | Temporaries |
| x10-x11 | a0-a1 | Arguments/Return |
| x12-x17 | a2-a7 | Arguments |
| x28-x31 | t3-t6 | Temporaries |

## M Extension (Multiply/Divide)

Additional R-type instructions:

| Instruction | funct7 | funct3 | Operation |
|-------------|--------|--------|-----------|
| MUL | 0000001 | 000 | rd = (rs1 × rs2)[31:0] |
| MULH | 0000001 | 001 | rd = (rs1 × rs2)[63:32] signed |
| MULHSU | 0000001 | 010 | rd = (rs1 × rs2)[63:32] signed×unsigned |
| MULHU | 0000001 | 011 | rd = (rs1 × rs2)[63:32] unsigned |
| DIV | 0000001 | 100 | rd = rs1 / rs2 signed |
| DIVU | 0000001 | 101 | rd = rs1 / rs2 unsigned |
| REM | 0000001 | 110 | rd = rs1 % rs2 signed |
| REMU | 0000001 | 111 | rd = rs1 % rs2 unsigned |

## Decoding Logic

### Opcode Check

```
if (opcode == 0110011)
    // R-type instruction
```

### Operation Selection

```
switch (funct3) {
    case 000:
        if (funct7 == 0000000) ADD
        if (funct7 == 0100000) SUB
    case 001: SLL
    case 010: SLT
    ...
}
```

## Why This Design?

### Regularity

- Fixed field positions
- Easy decoding
- Simple hardware

### Flexibility

- funct7 allows instruction variants
- Extensible for custom instructions

### Efficiency

- 32 registers addressable (5 bits)
- All operations in one cycle
