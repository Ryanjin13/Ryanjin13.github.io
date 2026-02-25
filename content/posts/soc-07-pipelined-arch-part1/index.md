---
title: "[SoC-07] Pipelined Architecture Part 1: Building Blocks and the Single-Cycle RISC-V Processor"
date: 2026-02-25
description: "Understanding the fundamental hardware building blocks of a CPU and how they connect to form a complete single-cycle RISC-V processor that executes one instruction per clock cycle."
categories: ["SoC Design"]
tags: ["SoC", "CPU Design", "Single-Cycle", "RISC-V", "Datapath", "Control Unit"]
series: ["SoC Design Course"]
series_order: 7
draft: false
---

{{< katex >}}

## Introduction

In the previous three posts, we studied the RISC-V ISA — the **what** of a processor. Now we begin studying the **how**: the actual hardware that fetches, decodes, and executes instructions.

We start with the simplest possible implementation: a **single-cycle processor** where every instruction completes in exactly one clock cycle. While not practical for high performance, it provides the clearest view of how hardware implements an ISA.

---

## 1. The Building Blocks

Every processor is built from a small set of fundamental hardware components. Let's understand each one.

### 1.1 Combinational Elements

These produce outputs that depend **only** on current inputs (no memory):

**Adder:**
$$
\text{Result} = A + B
$$

```
A ──┐
    ├──[+]──► Result
B ──┘
```

**ALU (Arithmetic Logic Unit):**

Performs multiple operations, selected by a control signal:

```
A ──┐
    ├──[ALU]──► Result
B ──┘     ↑       │
      ALU_Op    Zero flag
```

| ALU_Op | Operation |
|--------|-----------|
| 0000 | AND |
| 0001 | OR |
| 0010 | ADD |
| 0110 | SUB |
| 0111 | SLT (Set Less Than) |

**Multiplexer (MUX):**

Selects one of several inputs:

```
A ──┐
    ├──[MUX]──► Y
B ──┘
      ↑
     Sel
```

$$
Y = \begin{cases} A & \text{if Sel = 0} \\ B & \text{if Sel = 1} \end{cases}
$$

**Immediate Generator:**

Extracts and sign-extends the immediate value from different instruction formats:

```
Instruction[31:0] ──► [Imm Gen] ──► 32-bit sign-extended immediate
```

### 1.2 Sequential Elements

These have **memory** — they capture and hold values on a clock edge:

**Register (D Flip-Flop Array):**

```
      ┌─────────────┐
D ───►│   Register  │──► Q
      │             │
CLK ─►│>            │
      └─────────────┘
```

Captures D at the rising clock edge. Used for PC, pipeline registers, etc.

**Register File:**

The most important storage in the CPU — an array of 32 registers with two read ports and one write port:

```
        ┌──────────────────────┐
Read1 ──►│                      │──► Data1
Read2 ──►│   32 × 32-bit       │──► Data2
         │   Register File     │
Write  ──►│                      │
WData  ──►│                      │
WrEn   ──►│                      │
CLK    ──►│>                     │
        └──────────────────────┘
```

- **Two read ports**: Can read two registers simultaneously (needed for R-type: read rs1 and rs2 at the same time)
- **One write port**: Can write one register per cycle (write rd)
- **Read is combinational** (instant), **write is sequential** (happens at clock edge)

**Memories:**

```
Instruction Memory (I-Mem):            Data Memory (D-Mem):
┌───────────────────┐                 ┌───────────────────┐
│   Read-only       │                 │   Read/Write      │
│                   │                 │                   │
Addr ──►│              │──► Inst     Addr ──►│              │──► ReadData
        └───────────────────┘        WData──►│              │
                                     MemRd──►│              │
                                     MemWr──►│              │
                                     CLK  ──►│>             │
                                             └───────────────────┘
```

---

## 2. Single-Cycle Datapath

Now let's connect these building blocks to execute RISC-V instructions. We build the datapath incrementally, instruction type by instruction type.

### 2.1 Instruction Fetch

Every instruction begins the same way: read the instruction at the address stored in PC, then advance PC to the next instruction.

```
              ┌─────┐        ┌──────────┐
              │     │        │          │
     ┌───────►│ PC  │───────►│  I-Mem   │───────► Instruction
     │        │     │        │          │
     │        └─────┘        └──────────┘
     │           │
     │        ┌──┴──┐
     │        │     │
     └────────│ +4  │
              │     │
              └─────┘
```

$$
\text{Instruction} = \text{I-Mem}[PC]
$$
$$
PC_{next} = PC + 4
$$

### 2.2 R-Type Datapath (e.g., `add x3, x1, x2`)

```
Instruction
    │
    ├── [rs1 field] ──► RegFile Read1 ──► A ──┐
    │                                         ├──[ALU]──► Result ──► RegFile WriteData
    ├── [rs2 field] ──► RegFile Read2 ──► B ──┘              │
    │                                                     ALU_Op
    └── [rd field]  ──► RegFile WriteReg
                         RegWrite = 1
```

Steps:
1. **Fetch**: Read instruction from I-Mem[PC]
2. **Decode**: Extract rs1, rs2, rd, funct3, funct7
3. **Read registers**: RegFile provides values of rs1 and rs2
4. **ALU**: Perform the operation (add, sub, and, etc.)
5. **Write back**: Store ALU result into rd

### 2.3 I-Type ALU Datapath (e.g., `addi x3, x1, 10`)

The second ALU input comes from the **immediate** instead of rs2:

```
RegFile[rs1] ──► A ──┐
                      ├──[ALU]──► Result ──► RegFile[rd]
Imm Gen ─────► B ──┘
                  ↑
              [MUX] ← ALUSrc
```

A MUX selects between the register value (for R-type) and the immediate (for I-type), controlled by the **ALUSrc** signal.

### 2.4 Load Datapath (e.g., `lw x3, 8(x1)`)

```
RegFile[rs1] ──► A ──┐
                      ├──[ALU]──► Address ──► D-Mem ──► ReadData ──► RegFile[rd]
Imm Gen ─────► B ──┘                          │
                                           MemRead=1
```

Steps:
1. Read base register (rs1)
2. Add immediate offset in ALU → memory address
3. Read data memory at that address
4. Write the loaded data to rd

A MUX is needed to select whether RegFile write data comes from the ALU result (R-type) or from memory (load):

```
ALU Result ──┐
              ├──[MUX]──► RegFile WriteData
D-Mem Data ──┘     ↑
              MemToReg
```

### 2.5 Store Datapath (e.g., `sw x3, 8(x1)`)

```
RegFile[rs1] ──► A ──┐
                      ├──[ALU]──► Address ──► D-Mem
Imm Gen ─────► B ──┘                    ↑
                                    WriteData = RegFile[rs2]
                                    MemWrite = 1
```

Note: For stores, there is **no register write** (RegWrite = 0).

### 2.6 Branch Datapath (e.g., `beq x1, x2, offset`)

```
RegFile[rs1] ──► A ──┐
                      ├──[ALU]──► Zero flag
RegFile[rs2] ──► B ──┘

                         Branch Target:
PC ──────┐              PC + (Imm << 1)
          ├──[+]──┐
Imm Gen ─┘        │
                   ▼
PC+4 ──┐        Branch
        ├──[MUX]──► Next PC
Target ─┘    ↑
         Branch & Zero
```

The branch is **taken** if both:
- The `Branch` control signal is active, AND
- The `Zero` flag from the ALU is set (meaning rs1 == rs2 for `beq`)

$$
PC_{next} = \begin{cases} PC + 4 & \text{if branch not taken} \\ PC + \text{offset} & \text{if branch taken} \end{cases}
$$

---

## 3. Complete Single-Cycle Datapath

Combining all the above, the complete single-cycle datapath looks like this:

```
                                                  ┌─────────────┐
                                                  │  Control    │
                                         Inst ───►│  Unit       │──► RegWrite
                                                  │             │──► ALUSrc
                                                  │             │──► MemToReg
                                                  │             │──► MemRead
                                                  │             │──► MemWrite
                                                  │             │──► Branch
                                                  │             │──► ALUOp
                                                  └─────────────┘

┌──────┐    ┌────────┐   ┌─────────────┐    ┌──────┐    ┌────────┐    ┌─────┐
│      │    │        │   │             │    │      │    │        │    │     │
│  PC  │──►│ I-Mem  │──►│  Register   │──►│ ALU  │──►│ D-Mem  │──►│ MUX │──┐
│      │    │        │   │  File       │    │      │    │        │    │     │  │
└──┬───┘    └────────┘   │             │    └──────┘    └────────┘    └─────┘  │
   │                     │  [rs1]──►A  │       ↑                       ↑      │
   │                     │  [rs2]──►B  │    ALU_Op                  MemToReg  │
   │                     │             │       ↑                              │
   │  ┌───┐              │  [rd]◄──────┼───────┼──────────────────────────────┘
   └─►│+4 │              │  WrData     │   ┌───┴───┐
      └─┬─┘              └─────────────┘   │ALU    │
        │                      ↑           │Control│
        ▼                   ALUSrc         └───────┘
   ┌────┴────┐                 ↑
   │  MUX    │           ┌─────┴─────┐
   │ (PCSrc) │           │  Imm Gen  │
   └────┬────┘           └───────────┘
        │
        └──► Next PC
```

---

## 4. The Control Unit

The control unit takes the **opcode** (and funct3/funct7 fields) from the instruction and generates all the control signals that configure the datapath.

### 4.1 Main Control Signals

| Signal | Meaning When = 1 | Meaning When = 0 |
|--------|-------------------|-------------------|
| **RegWrite** | Write result to register file | Don't write |
| **ALUSrc** | ALU input B = immediate | ALU input B = register |
| **MemToReg** | Register write data = memory | Register write data = ALU |
| **MemRead** | Read from data memory | Don't read |
| **MemWrite** | Write to data memory | Don't write |
| **Branch** | Instruction is a branch | Not a branch |

### 4.2 Control Signal Truth Table

| Instruction | opcode | RegWrite | ALUSrc | MemToReg | MemRead | MemWrite | Branch | ALUOp |
|-------------|--------|:--------:|:------:|:--------:|:-------:|:--------:|:------:|:-----:|
| R-type | 0110011 | 1 | 0 | 0 | 0 | 0 | 0 | 10 |
| I-type ALU | 0010011 | 1 | 1 | 0 | 0 | 0 | 0 | 10 |
| Load (lw) | 0000011 | 1 | 1 | 1 | 1 | 0 | 0 | 00 |
| Store (sw) | 0100011 | 0 | 1 | X | 0 | 1 | 0 | 00 |
| Branch (beq) | 1100011 | 0 | 0 | X | 0 | 0 | 1 | 01 |

### 4.3 ALU Control

The ALU operation is determined by a two-level decode:

**Level 1** (Main Control → ALUOp):

| ALUOp | Meaning |
|-------|---------|
| 00 | Load/Store: always ADD (compute address) |
| 01 | Branch: always SUB (compare operands) |
| 10 | R-type/I-type: depends on funct3/funct7 |

**Level 2** (ALU Control unit uses ALUOp + funct3 + funct7):

| ALUOp | funct7 | funct3 | ALU Operation |
|-------|--------|--------|---------------|
| 00 | X | X | ADD |
| 01 | X | X | SUB |
| 10 | 0000000 | 000 | ADD |
| 10 | 0100000 | 000 | SUB |
| 10 | 0000000 | 111 | AND |
| 10 | 0000000 | 110 | OR |
| 10 | 0000000 | 010 | SLT |

---

## 5. Instruction Execution Walkthrough

Let's trace through three different instructions to see the datapath in action:

### 5.1 R-Type: `add x9, x20, x21`

```
1. FETCH:     PC → I-Mem → Instruction = 0x015A04B3
2. DECODE:    opcode=0110011, rd=9, rs1=20, rs2=21, funct7=0, funct3=0
              Control: RegWrite=1, ALUSrc=0, MemToReg=0, Branch=0
3. READ REGS: RegFile[20] → A, RegFile[21] → B
4. ALU:       Result = A + B (ALU Op = ADD)
5. MEM:       (no memory access)
6. WRITEBACK: RegFile[9] ← ALU Result
7. PC:        PC ← PC + 4
```

### 5.2 Load: `lw x9, 40(x20)`

```
1. FETCH:     PC → I-Mem → Instruction
2. DECODE:    opcode=0000011, rd=9, rs1=20, imm=40
              Control: RegWrite=1, ALUSrc=1, MemToReg=1, MemRead=1
3. READ REGS: RegFile[20] → A
4. ALU:       Address = A + 40 (ALU Op = ADD, B = immediate)
5. MEM:       ReadData = D-Mem[Address]
6. WRITEBACK: RegFile[9] ← ReadData (from memory, not ALU)
7. PC:        PC ← PC + 4
```

### 5.3 Branch: `beq x1, x2, offset`

```
1. FETCH:     PC → I-Mem → Instruction
2. DECODE:    opcode=1100011, rs1=1, rs2=2, imm=offset
              Control: RegWrite=0, ALUSrc=0, Branch=1
3. READ REGS: RegFile[1] → A, RegFile[2] → B
4. ALU:       Result = A - B (ALU Op = SUB)
              Zero flag = (Result == 0) = (A == B)
5. MEM:       (no memory access)
6. WRITEBACK: (no register write)
7. PC:        if (Branch AND Zero)
                PC ← PC + offset
              else
                PC ← PC + 4
```

---

## 6. Critical Path and Performance

### 6.1 The Problem with Single-Cycle Design

In a single-cycle processor, every instruction must complete within **one clock cycle**. The clock period must be long enough for the **slowest instruction** — which is the load instruction:

```
Critical Path (load instruction):
I-Mem → RegFile Read → MUX → ALU → D-Mem → MUX → RegFile Write

 200ps    100ps      25ps  200ps  200ps   25ps    100ps
 ─────────────────────────────────────────────────────────
                    Total: 850 ps
```

$$
T_{cycle} = 850\ \text{ps} \quad \Rightarrow \quad f_{max} = \frac{1}{850 \times 10^{-12}} \approx 1.18\ \text{GHz}
$$

But most instructions (like `add`) don't need memory access and could complete faster:

```
R-type path:
I-Mem → RegFile Read → MUX → ALU → MUX → RegFile Write
 200ps    100ps      25ps  200ps  25ps    100ps
 ─────────────────────────────────────────────
                Total: 650 ps (wasted 200ps!)
```

**The single-cycle design wastes time on every instruction that isn't a load.** This is why we need pipelining — the topic of the next post.

### 6.2 Performance Metric

$$
\text{CPU Time} = \text{Instructions} \times \text{CPI} \times T_{cycle}
$$

For single-cycle: CPI = 1 (every instruction takes exactly one cycle), but $T_{cycle}$ is long.

---

## 7. Adding Jump Support

To complete our processor, we need to handle `jal` (Jump and Link) instructions:

```
jal x1, offset    # x1 = PC + 4; PC = PC + offset
```

This requires:
1. A path to write **PC + 4** into the register file (as the return address)
2. A path to compute **PC + offset** as the next PC value

```
               PC+4 ──┐
                       ├──[MUX]──► RegFile WriteData
ALU Result ───┘  ↑
MemData ──────┘  │
            WriteDataSrc (00=ALU, 01=Mem, 10=PC+4)
```

The PC MUX also needs a third input:

```
PC+4 ─────────┐
               ├──[MUX]──► Next PC
Branch Target ─┤     ↑
Jump Target ───┘  PCSrc (00=PC+4, 01=Branch, 10=Jump)
```

---

## 8. Summary

| Component | Role in Single-Cycle CPU |
|-----------|-------------------------|
| **PC** | Holds address of current instruction |
| **I-Mem** | Stores program instructions (read-only) |
| **Register File** | 32 registers with 2 read, 1 write port |
| **Imm Gen** | Extracts/sign-extends immediates from instruction |
| **ALU** | Performs arithmetic/logic/comparison operations |
| **D-Mem** | Stores program data (read/write) |
| **MUXes** | Select between data sources based on instruction type |
| **Control Unit** | Decodes opcode → generates control signals |

**Key takeaway:** The single-cycle design is **correct** (it implements the ISA) but **inefficient** (clock period is limited by the slowest instruction). The solution is **pipelining**, which we explore in [SoC-08].

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
