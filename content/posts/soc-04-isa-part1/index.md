---
title: "[SoC-04] Instruction Set Architecture Part 1: The CPU's Contract with Software"
date: 2026-02-25
description: "Understanding the fundamental concepts of Instruction Set Architecture (ISA) — the critical interface between software and hardware that defines what a CPU can do."
categories: ["SoC Design"]
tags: ["SoC", "ISA", "CPU", "Instruction Format", "Computer Architecture", "RISC-V"]
series: ["SoC Design Course"]
series_order: 4
draft: false
---

{{< katex >}}

## Introduction

In the previous posts, we covered digital logic fundamentals and computer arithmetic. Now we arrive at one of the most important concepts in computer architecture:

> **What exactly can a CPU do?**

The answer is defined by the **Instruction Set Architecture (ISA)** — the complete specification of every instruction the processor understands. Think of it as a **contract** between software and hardware:

- **Software** (compilers, operating systems, applications) promises to express all computation using only the instructions defined in the ISA.
- **Hardware** (the processor) promises to execute every instruction correctly and predictably.

This separation is powerful because it allows software and hardware to evolve independently, as long as both sides honor the contract.

---

## 1. What Is an ISA?

### 1.1 Definition

An **Instruction Set Architecture** specifies:

1. **Instructions**: The operations the CPU can perform (add, subtract, load, store, branch, etc.)
2. **Data types**: What kinds of data the CPU can operate on (integers, floating-point, vectors)
3. **Registers**: How many registers are available and their purpose
4. **Memory model**: How the CPU accesses memory (addressing modes, alignment, endianness)
5. **Encoding**: How instructions are represented as binary bit patterns

### 1.2 The ISA as an Abstraction Layer

```
┌─────────────────────────┐
│     Application          │  (Python, Java, C++)
├─────────────────────────┤
│     Operating System     │  (Linux, Windows, RTOS)
├─────────────────────────┤
│     Compiler             │  (GCC, LLVM, Clang)
├═════════════════════════╡
│     ISA                  │  ◄── THE CONTRACT
├═════════════════════════╡
│     Microarchitecture    │  (Pipeline, Cache, OoO)
├─────────────────────────┤
│     Logic / RTL          │  (Gates, Flip-flops)
├─────────────────────────┤
│     Physics / Silicon    │  (Transistors, Metal layers)
└─────────────────────────┘
```

Everything **above** the ISA is software. Everything **below** is hardware implementation. The ISA is the boundary.

**Key insight:** Multiple different microarchitectures can implement the **same** ISA. For example:
- Intel's Alder Lake and AMD's Zen 4 both implement the x86-64 ISA, but with completely different internal designs
- ARM's Cortex-A78 and Cortex-A55 both implement ARMv8-A, but one is high-performance while the other is energy-efficient

### 1.3 Why ISA Matters for SoC Design

When designing an SoC, the choice of ISA determines:

| Aspect | Impact |
|--------|--------|
| **Software ecosystem** | What compilers, OS, and libraries are available |
| **Hardware complexity** | How many gates are needed to implement the decoder |
| **Performance** | How efficiently the ISA maps to the microarchitecture |
| **Power efficiency** | Simpler ISAs generally lead to simpler, lower-power designs |
| **Licensing cost** | Proprietary ISAs (ARM, x86) require licensing; open ISAs (RISC-V) are free |

---

## 2. Anatomy of an Instruction

Every instruction tells the CPU three things:

1. **What to do** (the operation) → encoded in the **opcode**
2. **What to do it to** (the data) → specified by **operands**
3. **Where to put the result** → specified by the **destination operand**

### 2.1 A Simple Example

Consider this high-level operation:

```c
c = a + b;
```

In assembly (RISC-V):

```asm
add  x3, x1, x2    # x3 = x1 + x2
```

The instruction has four fields:

| Field | Value | Meaning |
|-------|-------|---------|
| Operation | `add` | Addition |
| Destination | `x3` | Where to store the result |
| Source 1 | `x1` | First operand |
| Source 2 | `x2` | Second operand |

### 2.2 Instruction Fields

In general, instructions contain these types of fields:

```
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│  Opcode  │   Dest   │  Source1 │  Source2 │  Other   │
│  (what)  │ (where)  │  (from)  │  (from)  │ (extra)  │
└──────────┴──────────┴──────────┴──────────┴──────────┘
```

| Field | Purpose |
|-------|---------|
| **Opcode** | Identifies the operation (add, sub, load, branch, etc.) |
| **rd** (destination register) | The register that receives the result |
| **rs1, rs2** (source registers) | Registers providing input operands |
| **Immediate** | A constant value embedded directly in the instruction |
| **funct** | Additional opcode bits for distinguishing similar operations |

---

## 3. Types of Instructions

A typical ISA provides four main categories of instructions:

### 3.1 Arithmetic and Logic Instructions

Perform computation on register values:

| Operation | Example (RISC-V) | Meaning |
|-----------|-------------------|---------|
| Add | `add x3, x1, x2` | x3 = x1 + x2 |
| Subtract | `sub x3, x1, x2` | x3 = x1 - x2 |
| AND | `and x3, x1, x2` | x3 = x1 & x2 |
| OR | `or x3, x1, x2` | x3 = x1 \| x2 |
| XOR | `xor x3, x1, x2` | x3 = x1 ^ x2 |
| Shift Left | `sll x3, x1, x2` | x3 = x1 << x2 |
| Set Less Than | `slt x3, x1, x2` | x3 = (x1 < x2) ? 1 : 0 |

**With immediate values** (constant operands):

| Operation | Example | Meaning |
|-----------|---------|---------|
| Add Immediate | `addi x3, x1, 10` | x3 = x1 + 10 |
| AND Immediate | `andi x3, x1, 0xFF` | x3 = x1 & 0xFF |

### 3.2 Memory Access Instructions (Load/Store)

Transfer data between **registers** and **memory**:

```
   Registers                    Memory
  ┌────────┐                 ┌────────────┐
  │   x1   │ ──── Store ──► │ Address A  │
  │   x2   │ ◄─── Load ──── │ Address B  │
  │   ...  │                 │    ...     │
  └────────┘                 └────────────┘
```

| Operation | Example | Meaning |
|-----------|---------|---------|
| Load Word | `lw x3, 0(x1)` | x3 = Memory[x1 + 0] |
| Store Word | `sw x3, 8(x1)` | Memory[x1 + 8] = x3 |
| Load Byte | `lb x3, 0(x1)` | x3 = sign-extend(Memory[x1]) |
| Load Byte Unsigned | `lbu x3, 0(x1)` | x3 = zero-extend(Memory[x1]) |

The syntax `offset(base)` means: compute the memory address as `base register + offset`.

### 3.3 Control Flow Instructions (Branch/Jump)

Change the order of instruction execution:

**Conditional branches** (decide based on comparison):

| Operation | Example | Meaning |
|-----------|---------|---------|
| Branch if Equal | `beq x1, x2, label` | if (x1 == x2) goto label |
| Branch if Not Equal | `bne x1, x2, label` | if (x1 != x2) goto label |
| Branch if Less Than | `blt x1, x2, label` | if (x1 < x2) goto label |
| Branch if ≥ | `bge x1, x2, label` | if (x1 >= x2) goto label |

**Unconditional jumps:**

| Operation | Example | Meaning |
|-----------|---------|---------|
| Jump and Link | `jal x1, label` | x1 = PC+4; goto label |
| Jump and Link Register | `jalr x1, 0(x2)` | x1 = PC+4; goto (x2+0) |

`jal` is used for **function calls** — it saves the return address in the destination register before jumping.

### 3.4 System Instructions

Special operations for OS interaction and hardware control:

| Operation | Example | Purpose |
|-----------|---------|---------|
| ECALL | `ecall` | System call (request OS service) |
| EBREAK | `ebreak` | Debugger breakpoint |
| FENCE | `fence` | Memory ordering barrier |
| CSR Read/Write | `csrrw x1, csr, x2` | Access control/status registers |

---

## 4. Instruction Encoding

### 4.1 Why Encoding Matters

Every instruction must be stored in memory as a sequence of bits. The **encoding format** determines:

- How the CPU decodes (interprets) instructions
- How much memory instructions consume
- How complex the decoder hardware needs to be

### 4.2 Fixed-Length vs. Variable-Length

| Approach | Example ISA | Pros | Cons |
|----------|-------------|------|------|
| **Fixed-length** | RISC-V (32-bit) | Simple decoding, easy pipelining | May waste bits |
| **Variable-length** | x86 (1–15 bytes) | Compact code | Complex decoder |

RISC-V uses **fixed 32-bit instructions** (with an optional 16-bit compressed extension). This means every instruction is exactly 4 bytes, which makes the hardware decoder much simpler.

### 4.3 RISC-V Base Instruction Formats

RISC-V defines six instruction formats, all exactly 32 bits wide:

```
R-type:  [  funct7  |  rs2  |  rs1  | funct3 |   rd   | opcode ]
         [  31:25   | 24:20 | 19:15 | 14:12  |  11:7  |  6:0   ]

I-type:  [     imm[11:0]    |  rs1  | funct3 |   rd   | opcode ]
         [      31:20       | 19:15 | 14:12  |  11:7  |  6:0   ]

S-type:  [ imm[11:5] |  rs2  |  rs1  | funct3 |imm[4:0]| opcode ]
         [  31:25    | 24:20 | 19:15 | 14:12  |  11:7  |  6:0   ]

B-type:  [imm[12|10:5]| rs2  |  rs1  | funct3 |imm[4:1|11]|opcode]
         [   31:25    | 24:20| 19:15 | 14:12  |  11:7  |  6:0   ]

U-type:  [          imm[31:12]          |   rd   | opcode ]
         [            31:12             |  11:7  |  6:0   ]

J-type:  [  imm[20|10:1|11|19:12]       |   rd   | opcode ]
         [            31:12             |  11:7  |  6:0   ]
```

**Design principle:** Notice that `rs1`, `rs2`, and `rd` are always in the **same bit positions** across all formats. This allows the register file to be read **before** the instruction is fully decoded — a critical optimization for pipelined processors.

### 4.4 Format Usage

| Format | Used For | Example |
|--------|----------|---------|
| R-type | Register-register ALU ops | `add x3, x1, x2` |
| I-type | Immediate ALU ops, loads | `addi x3, x1, 10` / `lw x3, 0(x1)` |
| S-type | Stores | `sw x3, 8(x1)` |
| B-type | Conditional branches | `beq x1, x2, label` |
| U-type | Upper immediate | `lui x3, 0x12345` |
| J-type | Unconditional jumps | `jal x1, label` |

---

## 5. Registers

### 5.1 Why Registers?

Registers are the **fastest** storage in a computer — they are built directly into the CPU and can be accessed in a single clock cycle (or even less). Memory access, by contrast, takes many cycles.

```
Speed Hierarchy:
  Registers    ──→  ~0.5 ns   (within CPU)
  L1 Cache     ──→  ~1–2 ns
  L2 Cache     ──→  ~5–10 ns
  Main Memory  ──→  ~50–100 ns  (100× slower than registers!)
  SSD          ──→  ~100 μs
```

### 5.2 RISC-V Register File

RISC-V has **32 general-purpose registers**, each 32 bits wide (in RV32I) or 64 bits (in RV64I):

| Register | ABI Name | Purpose |
|----------|----------|---------|
| x0 | zero | Hardwired to 0 (always reads as 0) |
| x1 | ra | Return address |
| x2 | sp | Stack pointer |
| x3 | gp | Global pointer |
| x4 | tp | Thread pointer |
| x5–x7 | t0–t2 | Temporaries |
| x8 | s0/fp | Saved register / Frame pointer |
| x9 | s1 | Saved register |
| x10–x11 | a0–a1 | Function arguments / return values |
| x12–x17 | a2–a7 | Function arguments |
| x18–x27 | s2–s11 | Saved registers |
| x28–x31 | t3–t6 | Temporaries |

**Why is x0 hardwired to 0?** It simplifies many operations:
- `add x3, x1, x0` → move (copy x1 to x3)
- `addi x0, x0, 0` → nop (no operation)
- `slt x3, x0, x1` → test if x1 > 0

### 5.3 Register Design Trade-offs

| More Registers | Fewer Registers |
|---------------|-----------------|
| Fewer memory accesses (faster) | Simpler hardware |
| More bits needed per instruction | Shorter instructions |
| Larger register file (more area/power) | Less context switch overhead |

RISC-V's choice of 32 registers is a well-established sweet spot — enough to keep most operands in registers, but not so many that instruction encoding becomes bloated (5 bits per register specifier × 3 registers = 15 bits, leaving room for opcode and immediates in 32-bit instructions).

---

## 6. The Program Counter (PC)

### 6.1 What Is the PC?

The **Program Counter** is a special register that holds the memory address of the **current instruction** being executed. After each instruction, the PC is typically updated to point to the next instruction:

$$
PC_{next} = PC + 4 \quad \text{(for 32-bit fixed-length instructions)}
$$

Unless a branch or jump instruction redirects execution elsewhere.

### 6.2 Program Execution Flow

```
Memory:
┌──────────┬──────────────────┐
│ Address  │ Instruction      │
├──────────┼──────────────────┤
│ 0x0000   │ addi x1, x0, 5  │ ◄── PC starts here
│ 0x0004   │ addi x2, x0, 3  │
│ 0x0008   │ add  x3, x1, x2 │
│ 0x000C   │ sw   x3, 0(x4)  │
│ 0x0010   │ beq  x3, x5, L  │ ── Branch: if taken, PC jumps to L
│ 0x0014   │ addi x1, x1, 1  │
│ 0x0018   │ ...              │ ◄── L (branch target)
└──────────┴──────────────────┘
```

The CPU repeats this cycle endlessly:

```
┌────────────────────────────────┐
│   1. FETCH instruction at PC   │
│   2. DECODE the instruction    │
│   3. EXECUTE the operation     │
│   4. UPDATE the PC             │
│              │                 │
│              ▼                 │
│       (repeat forever)         │
└────────────────────────────────┘
```

This is the **fetch-decode-execute cycle** — the fundamental heartbeat of every processor.

---

## 7. Operand Types

Instructions can get their data from three sources:

### 7.1 Register Operands

Data comes from the register file. This is the **fastest** option.

```asm
add  x3, x1, x2   # All operands are registers
```

### 7.2 Immediate Operands

A **constant value** is encoded directly in the instruction bits. No memory or register lookup needed.

```asm
addi x3, x1, 42   # 42 is the immediate value
```

Immediates have limited range because they must fit within the instruction:
- I-type: 12 bits → range $[-2048, +2047]$
- U-type: 20 bits → for loading upper bits of large constants

**Loading a full 32-bit constant** requires two instructions:

```asm
lui  x3, 0x12345    # Load upper 20 bits: x3 = 0x12345000
addi x3, x3, 0x678  # Add lower 12 bits:  x3 = 0x12345678
```

### 7.3 Memory Operands

Data is loaded from or stored to memory at a computed address:

```asm
lw  x3, 8(x1)     # x3 = Memory[x1 + 8]
sw  x3, 8(x1)     # Memory[x1 + 8] = x3
```

In RISC architectures like RISC-V, **only load and store instructions access memory**. All computation happens on registers. This is called a **load-store architecture**.

---

## 8. Instruction Execution: Putting It All Together

Let's trace through a complete example — computing `a[3] = a[1] + a[2]`:

Given: base address of array `a` is in `x10`, each element is 4 bytes (word).

```asm
# Step 1: Load a[1] into x5
lw   x5, 4(x10)     # x5 = Memory[x10 + 4] = a[1]

# Step 2: Load a[2] into x6
lw   x6, 8(x10)     # x6 = Memory[x10 + 8] = a[2]

# Step 3: Add them
add  x7, x5, x6     # x7 = x5 + x6 = a[1] + a[2]

# Step 4: Store result into a[3]
sw   x7, 12(x10)    # Memory[x10 + 12] = x7 → a[3] = a[1] + a[2]
```

Execution trace:

```
Step  PC      Instruction         Registers Changed
────  ──────  ──────────────────  ─────────────────────
  1   0x0000  lw  x5, 4(x10)     x5 ← Memory[x10+4]
  2   0x0004  lw  x6, 8(x10)     x6 ← Memory[x10+8]
  3   0x0008  add x7, x5, x6     x7 ← x5 + x6
  4   0x000C  sw  x7, 12(x10)    Memory[x10+12] ← x7
```

---

## 9. Design Principles Behind ISA

Several guiding principles shape good ISA design:

### Principle 1: Simplicity Favors Regularity

All RISC-V arithmetic instructions have the same format: `op rd, rs1, rs2`. This regularity makes the hardware decoder simple and fast.

### Principle 2: Smaller Is Faster

RISC-V has 32 registers — not 64 or 128. A smaller register file is faster to access, consumes less power, and requires fewer bits in each instruction to specify.

### Principle 3: Good Design Demands Compromise

The ISA must balance competing goals:
- Large immediates (more flexibility) vs. short instructions (less memory)
- Many instruction types (more expressiveness) vs. simple decoder (less hardware)

### Principle 4: Make the Common Case Fast

The most frequently used instructions should be the simplest and fastest. RISC-V's base integer ISA (RV32I) contains only 47 instructions — just enough for a complete computer, but no more.

---

## 10. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **ISA** | The contract between software and hardware; defines what the CPU can do |
| **Instruction types** | Arithmetic/logic, memory access, control flow, system |
| **Encoding** | How instructions are represented in binary; RISC-V uses fixed 32-bit formats |
| **Registers** | 32 fast storage locations (x0–x31) inside the CPU |
| **Program Counter** | Tracks the address of the current instruction |
| **Operands** | Can come from registers, immediates, or memory |
| **Load-store architecture** | Only load/store instructions access memory; all computation uses registers |
| **Design principles** | Simplicity, regularity, and making the common case fast |

In the **next post ([SoC-05])**, we will dive deeper into **memory addressing modes**, compare **CISC vs. RISC** architectures, and explore the **design philosophy of RISC-V**.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
