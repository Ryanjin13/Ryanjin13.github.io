---
title: "[SoC-05] Instruction Set Architecture Part 2: Addressing, CISC vs RISC, and the RISC-V Philosophy"
date: 2026-02-25
description: "A deep dive into memory addressing modes, the fundamental differences between CISC and RISC architectures, and why RISC-V was designed the way it was."
categories: ["SoC Design"]
tags: ["SoC", "ISA", "RISC-V", "CISC", "RISC", "Addressing Modes", "Computer Architecture"]
series: ["SoC Design Course"]
series_order: 5
draft: false
---

{{< katex >}}

## Introduction

In [SoC-04], we introduced the concept of ISA and learned about instruction types, encoding formats, and registers. Now let's go deeper into three critical topics:

1. **How does the CPU find data in memory?** (Addressing modes)
2. **What are the fundamental ISA design philosophies?** (CISC vs. RISC)
3. **Why was RISC-V designed the way it was?** (Design philosophy)

Understanding these topics will give you the conceptual framework to appreciate the elegance of modern processor design.

---

## 1. Memory Addressing

### 1.1 The Memory Model

Computer memory is organized as a large **one-dimensional array of bytes**, each with a unique address:

```
Address    Content
┌──────┬──────────┐
│ 0x00 │ byte 0   │
│ 0x01 │ byte 1   │
│ 0x02 │ byte 2   │
│ 0x03 │ byte 3   │
│ 0x04 │ byte 4   │
│ ...  │ ...      │
└──────┴──────────┘
```

But most data types are larger than one byte:

| Data Type | Size | Bytes |
|-----------|------|-------|
| Byte | 8 bits | 1 |
| Half-word | 16 bits | 2 |
| Word | 32 bits | 4 |
| Double-word | 64 bits | 8 |

This raises the question: when we store a 32-bit word at an address, **which byte goes where?**

### 1.2 Endianness

**Big-Endian:** Most significant byte at the lowest address.

**Little-Endian:** Least significant byte at the lowest address.

Example: storing the 32-bit value `0x12345678` at address `0x100`:

```
         Big-Endian          Little-Endian
Addr   Byte                 Byte
0x100  0x12 (MSB)           0x78 (LSB)
0x101  0x34                 0x56
0x102  0x56                 0x34
0x103  0x78 (LSB)           0x12 (MSB)
```

| ISA | Endianness |
|-----|-----------|
| x86, RISC-V | Little-Endian |
| ARM | Bi-Endian (configurable, usually Little) |
| MIPS | Bi-Endian |
| Network protocols (TCP/IP) | Big-Endian ("network byte order") |

RISC-V chose **little-endian** because it simplifies some hardware operations (e.g., sign extension is just about the MSB position, which is always at the highest address).

### 1.3 Alignment

**Alignment** means that a data item of size $N$ bytes should be stored at an address that is a multiple of $N$:

| Data Type | Aligned Addresses |
|-----------|-------------------|
| Byte | Any address |
| Half-word (2 bytes) | 0, 2, 4, 6, ... |
| Word (4 bytes) | 0, 4, 8, 12, ... |
| Double-word (8 bytes) | 0, 8, 16, 24, ... |

```
Aligned (good):                 Misaligned (bad):
┌────┬────┬────┬────┐          ┌────┬────┬────┬────┐
│ W0 │ W0 │ W0 │ W0 │ addr 0  │    │ W0 │ W0 │ W0 │ addr 0
├────┼────┼────┼────┤          ├────┼────┼────┼────┤
│ W1 │ W1 │ W1 │ W1 │ addr 4  │ W0 │    │    │    │ addr 4
└────┴────┴────┴────┘          └────┴────┴────┴────┘
  One memory access              Two memory accesses needed!
```

Misaligned accesses are either:
- **Slow** (requires two memory reads + merge, as on x86)
- **Illegal** (causes a hardware exception, as on many RISC processors)

RISC-V requires **natural alignment** for loads and stores — the hardware is simpler and faster as a result.

---

## 2. Addressing Modes

An **addressing mode** specifies how the operand's effective address is calculated. Different ISAs support different sets of addressing modes.

### 2.1 Common Addressing Modes

| Mode | How Address Is Computed | Example | Effective Address |
|------|------------------------|---------|-------------------|
| **Immediate** | Operand is in the instruction | `addi x3, x1, 5` | Value = 5 |
| **Register** | Operand is in a register | `add x3, x1, x2` | Value = R[x2] |
| **Base + Offset** | Register + constant | `lw x3, 8(x1)` | Addr = R[x1] + 8 |
| **PC-relative** | PC + constant | `beq x1, x2, L` | Addr = PC + offset |
| **Indexed** | Base + index register | `lw x3, x1(x2)` | Addr = R[x1] + R[x2] |
| **Indirect** | Address in register points to address | `lw x3, (x1); lw x3, (x3)` | Addr = Mem[R[x1]] |
| **Auto-increment** | Use register, then increment it | `lw x3, (x1)+` | Addr = R[x1]; x1 += 4 |
| **Scaled** | Base + (index × scale) | `lw x3, x1(x2, 4)` | Addr = R[x1] + R[x2]×4 |

### 2.2 RISC-V Addressing Modes

RISC-V deliberately supports only a **small, simple set** of addressing modes:

| Mode | Used In | Example |
|------|---------|---------|
| **Register** | R-type instructions | `add x3, x1, x2` |
| **Immediate** | I-type instructions | `addi x3, x1, 100` |
| **Base + displacement** | Loads and stores | `lw x3, 12(x1)` |
| **PC-relative** | Branches and `jal` | `beq x1, x2, offset` |

That's it — **only four modes**. Compare this to x86, which has over a dozen, including complex modes like `[base + index*scale + displacement]`.

**Why so few?** Because:
1. Simple addressing modes → simple hardware → faster clock, less power
2. A compiler can synthesize complex addresses from simple ones with a few extra instructions
3. Fewer modes → simpler decoder → easier to pipeline

### 2.3 Building Complex Addresses from Simple Ones

Need `array[i]` where each element is 4 bytes?

```asm
# x10 = base address of array
# x11 = index i

slli  x12, x11, 2      # x12 = i * 4 (shift left by 2 = multiply by 4)
add   x12, x10, x12    # x12 = base + i*4
lw    x13, 0(x12)      # x13 = array[i]
```

Three simple instructions replace one complex addressing mode — and each instruction is fast and easy to pipeline.

---

## 3. CISC vs. RISC

### 3.1 The Two Philosophies

The history of computer architecture is largely the story of two competing design philosophies:

| | CISC | RISC |
|---|------|------|
| **Full name** | Complex Instruction Set Computer | Reduced Instruction Set Computer |
| **Philosophy** | "Make each instruction powerful" | "Make each instruction simple and fast" |
| **Examples** | x86, VAX, IBM System/360 | RISC-V, ARM, MIPS, SPARC, PowerPC |

### 3.2 CISC: Complex Instructions

The CISC philosophy emerged in the 1960s–70s when:
- Memory was expensive and slow
- Compilers were primitive
- Programmers often wrote assembly by hand

The solution: pack as much work as possible into each instruction to **reduce the total number of instructions** (and therefore reduce memory usage and the number of slow instruction fetches).

**x86 example — string copy:**

```asm
rep movsb    # Copy CX bytes from DS:SI to ES:DI
             # This SINGLE instruction:
             # 1. Reads a byte from memory
             # 2. Writes it to another memory location
             # 3. Increments/decrements pointers
             # 4. Decrements counter
             # 5. Loops until counter = 0
```

One instruction does the work of an entire loop!

**Characteristics of CISC:**

| Feature | Description |
|---------|-------------|
| Variable-length instructions | 1–15 bytes (x86) |
| Many addressing modes | 10+ modes |
| Memory-to-memory operations | ALU can operate directly on memory |
| Complex instructions | Single instruction can do multiply-and-accumulate, string operations, etc. |
| Microcode | Complex instructions are implemented as sequences of simpler micro-operations |

### 3.3 RISC: Simple Instructions

The RISC philosophy emerged in the 1980s from research at Berkeley (RISC-I, led by David Patterson) and Stanford (MIPS, led by John Hennessy). Their key insight:

> **Simple instructions executing quickly in a pipeline beat complex instructions that take many cycles.**

The 80/20 rule applies: about 80% of executed instructions are simple operations (add, load, store, branch). Making these simple operations blazingly fast matters more than having fancy complex instructions.

**RISC approach to string copy:**

```asm
loop:
    lb   x5, 0(x10)     # Load byte from source
    sb   x5, 0(x11)     # Store byte to destination
    addi x10, x10, 1    # Increment source pointer
    addi x11, x11, 1    # Increment destination pointer
    addi x12, x12, -1   # Decrement counter
    bnez x12, loop       # Branch if counter ≠ 0
```

Six simple instructions in a loop — but each one completes in one clock cycle in a pipeline.

**Characteristics of RISC:**

| Feature | Description |
|---------|-------------|
| Fixed-length instructions | 32 bits (typically) |
| Few addressing modes | 3–4 modes |
| Load-store architecture | Only load/store access memory; ALU works only on registers |
| Simple instructions | Each does one thing, completes in ~1 cycle |
| Hardwired control | No microcode needed |

### 3.4 Detailed Comparison

| Aspect | CISC (x86) | RISC (RISC-V) |
|--------|-----------|---------------|
| Instruction count per program | Lower | Higher |
| Cycles per instruction (CPI) | Higher (variable) | Lower (~1 in pipeline) |
| Clock frequency | Often lower | Often higher (simpler logic) |
| Code size | Smaller | Larger |
| Hardware complexity | Complex decoder | Simple decoder |
| Power consumption | Higher | Lower |
| Pipeline friendliness | Difficult | Natural fit |
| Compiler complexity | Lower | Higher (compiler does more work) |

### 3.5 The Performance Equation Revisited

$$
\text{Time} = \text{Instructions} \times \text{CPI} \times T_{cycle}
$$

| Factor | CISC | RISC |
|--------|------|------|
| Instruction count | Fewer ✓ | More |
| CPI | Higher | Lower (~1) ✓ |
| Cycle time | Longer | Shorter ✓ |

CISC wins on instruction count but loses on CPI and cycle time. In practice, modern high-performance CISC processors (like Intel/AMD x86) actually **translate CISC instructions into RISC-like micro-operations internally** — the best of both worlds, but at the cost of a very complex front-end decoder.

### 3.6 Modern Reality: CISC Outside, RISC Inside

Modern x86 processors internally:

```
┌─────────────────────────────────────────────┐
│              x86 Front-End                   │
│  ┌─────────┐    ┌──────────────────────┐    │
│  │ Complex  │───►│ Micro-op Translator  │    │
│  │ x86 Inst │    │ (CISC → RISC-like)   │    │
│  └─────────┘    └──────────┬───────────┘    │
│                            │                 │
│              ┌─────────────▼──────────────┐  │
│              │    RISC-like Execution      │  │
│              │    Engine (Pipeline,        │  │
│              │    Out-of-Order, etc.)      │  │
│              └────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

This is why x86 processors have billions of transistors — a large fraction is devoted to the complex decode/translation stage.

---

## 4. The RISC-V Design Philosophy

### 4.1 What Is RISC-V?

RISC-V (pronounced "risk-five") is an **open-source ISA** created at UC Berkeley in 2010 by a team led by Krste Asanović and David Patterson (a co-inventor of the original RISC concept).

Key attributes:

| Feature | Detail |
|---------|--------|
| **Open standard** | Free to implement, no licensing fees |
| **Modular** | Base ISA + optional extensions |
| **Clean design** | No legacy baggage (unlike x86 or ARM) |
| **Academic origin** | Designed for teaching and research |
| **Industry adoption** | Used by SiFive, Alibaba, Google, NVIDIA, and many others |

### 4.2 Modular ISA Design

Unlike monolithic ISAs, RISC-V is built in layers:

```
┌─────────────────────────────────────────────────────┐
│  Custom Extensions (application-specific)            │
├─────────────────────────────────────────────────────┤
│  V: Vector Operations    │  B: Bit Manipulation     │
├──────────────────────────┼──────────────────────────┤
│  M: Multiply/Divide      │  A: Atomic Operations    │
├──────────────────────────┼──────────────────────────┤
│  F: Single-Precision FP  │  D: Double-Precision FP  │
├──────────────────────────┴──────────────────────────┤
│  C: Compressed Instructions (16-bit)                │
├─────────────────────────────────────────────────────┤
│  I: Base Integer Instructions (REQUIRED)            │
│  (RV32I or RV64I)                                   │
└─────────────────────────────────────────────────────┘
```

| Extension | Letter | Description | Instruction Count |
|-----------|--------|-------------|:-----------------:|
| Base Integer | I | Core arithmetic, load/store, branches | 47 |
| Multiply/Divide | M | Hardware multiplication and division | 8 |
| Atomic | A | Atomic memory operations for multi-core | 11 |
| Single-Precision Float | F | IEEE 754 single-precision operations | 26 |
| Double-Precision Float | D | IEEE 754 double-precision operations | 26 |
| Compressed | C | 16-bit short instructions for code density | 46 |
| Vector | V | SIMD-like vector operations for data parallelism | 300+ |

**"RV32IMAC"** means: 32-bit RISC-V with Multiply, Atomic, and Compressed extensions. This is a common configuration for embedded microcontrollers.

**"RV64GC"** means: 64-bit RISC-V with the "General" set (IMAFD) plus Compressed. This is suitable for application processors running Linux.

### 4.3 Design Principles of RISC-V

#### Principle 1: Cost Reduction through Simplicity

The RV32I base ISA has only **47 instructions**. Compare:

| ISA | Approximate Instruction Count |
|-----|:-----------------------------:|
| x86-64 | ~1,500+ (and growing) |
| ARMv8-A | ~1,000+ |
| RISC-V (RV32I base) | 47 |
| RISC-V (RV32GC) | ~200 |

Fewer instructions → smaller decoder → less silicon area → lower power → lower cost.

#### Principle 2: No Legacy Burden

x86 still carries instructions from the 8086 (1978). ARM carries legacy from ARM1 (1985). RISC-V started with a **clean slate** in 2010, incorporating 30 years of lessons learned.

Examples of "lessons learned" embedded in RISC-V:
- **No condition codes / flags register**: Avoids complex flag handling and simplifies out-of-order execution
- **No branch delay slots**: Earlier RISC ISAs (MIPS) had this and it became a permanent burden
- **No predicated instructions**: Adds complexity with marginal benefit
- **Fixed register positions in encoding**: Enables register file access before decode completes

#### Principle 3: Extensibility Without Fragmentation

The modular design means:
- A tiny embedded core only needs RV32I (very small, very low power)
- A Linux-capable core uses RV64GC
- An AI accelerator can add custom instructions for MAC operations
- All share the same base ISA and can run the same base software

#### Principle 4: Practical Openness

RISC-V is not just academically open — it is governed by **RISC-V International**, a non-profit organization. Companies can implement RISC-V without paying royalties, modify it freely, and add proprietary extensions without licensing headaches.

---

## 5. Memory Layout of a Program

Understanding how a program is organized in memory is essential for ISA-level programming:

```
High Address
┌─────────────────────┐
│       Stack          │ ↓ Grows downward
│   (local variables,  │
│    return addresses)  │
├─────────────────────┤
│         ↕            │ (free space)
├─────────────────────┤
│       Heap           │ ↑ Grows upward
│  (dynamically        │
│   allocated memory)  │
├─────────────────────┤
│   Static Data        │ Global variables
│  (.data, .bss)       │
├─────────────────────┤
│   Text (Code)        │ Program instructions
│  (.text)             │
├─────────────────────┤
│   Reserved           │ OS/interrupt vectors
└─────────────────────┘
Low Address (0x00000000)
```

| Section | Content | RISC-V Register |
|---------|---------|:---------------:|
| Text | Machine instructions | PC points here |
| Static Data | Global/static variables | gp (x3) points here |
| Heap | malloc/free memory | — |
| Stack | Local variables, saved registers | sp (x2) points to top |

---

## 6. Byte Ordering in Instructions

Let's see how a real RISC-V instruction is stored in memory. Consider:

```asm
addi x5, x0, 42    # x5 = 0 + 42 = 42
```

Encoding (I-type):
- imm[11:0] = 42 = 000000101010
- rs1 = x0 = 00000
- funct3 = 000
- rd = x5 = 00101
- opcode = 0010011

Binary: `0000 0010 1010 | 00000 | 000 | 00101 | 0010011`

Rearranged into 32 bits:
```
00000010101000000000001010010011
= 0x02A00293
```

In little-endian memory:
```
Address   Byte
0x0000    0x93   (LSB)
0x0001    0x02
0x0002    0xA0
0x0003    0x02   (MSB)
```

---

## 7. Comparing Major ISAs

| Feature | x86-64 | ARMv8-A | RISC-V |
|---------|--------|---------|--------|
| Type | CISC | RISC | RISC |
| Inst. Length | 1–15 bytes | 32 bits (fixed) | 32 bits (16 with C ext.) |
| Registers | 16 GPR | 31 GPR | 31 GPR (x0 = 0) |
| Endianness | Little | Bi (usually Little) | Little |
| Addressing Modes | 10+ | ~6 | 4 |
| License | Proprietary (Intel/AMD) | Proprietary (ARM Ltd.) | Open (free) |
| Condition Flags | Yes (EFLAGS) | Yes (NZCV) | No |
| Branch Delay Slot | No | No | No |
| Predication | Limited (CMOVcc) | Full (ARMv7); limited (v8) | No |
| First Year | 1978 (8086) | 1985 (ARM1) | 2010 |

---

## 8. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Endianness** | RISC-V is little-endian; byte order matters for multi-byte data |
| **Alignment** | Natural alignment simplifies hardware; RISC-V requires it |
| **Addressing modes** | RISC-V uses only 4 simple modes; complex addresses built from simple instructions |
| **CISC** | Many complex instructions, variable-length, many addressing modes |
| **RISC** | Few simple instructions, fixed-length, load-store only, pipeline-friendly |
| **Modern x86** | CISC outside, RISC inside (translates to micro-ops) |
| **RISC-V** | Open, modular, clean-slate RISC ISA with no legacy burden |
| **Modularity** | Base I + optional M, A, F, D, C, V extensions |

In the **next post ([SoC-06])**, we will study the **RISC-V instruction set in detail** and learn how C code is translated into assembly instructions.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
