---
title: "[SoC-12] Software for SoC Part 1: Embedded SoC Architecture and the ARM Cortex-M0+"
date: 2026-02-25
description: "Analyzing the structure of common embedded SoC environments and understanding the internal architecture and features of the ARM Cortex-M0+ processor core."
categories: ["SoC Design"]
tags: ["SoC", "ARM", "Cortex-M0+", "Embedded Systems", "Microcontroller", "ARMv6-M"]
series: ["SoC Design Course"]
series_order: 12
draft: false
---

{{< katex >}}

## Introduction

In posts [SoC-01] through [SoC-11], we studied computer architecture from the ground up — digital logic, ISA, pipelining, and memory hierarchy. We used RISC-V as our primary example because of its clean, open design.

Now we shift to the **practical world of embedded SoC engineering**. Most real embedded products use **ARM Cortex-M** cores, which dominate the microcontroller market. In this post, we'll explore the typical embedded SoC architecture and dive into the internals of the **ARM Cortex-M0+** — one of the smallest, most power-efficient ARM cores available.

---

## 1. Embedded SoC: The Big Picture

### 1.1 What Is an Embedded SoC?

An embedded SoC is a single chip designed for a specific application, integrating:
- A **processor core** (ARM Cortex-M, RISC-V, etc.)
- **Memory** (Flash for code, SRAM for data)
- **Peripherals** (GPIO, UART, SPI, I2C, ADC, Timer, etc.)
- **Bus interconnect** (AHB, APB)
- **Clock and power management**

```
┌─────────────────────────────────────────────────────────────┐
│                     Embedded SoC                             │
│                                                              │
│  ┌──────────┐  ┌────────┐  ┌────────┐                      │
│  │ Cortex-  │  │ Flash  │  │  SRAM  │                      │
│  │   M0+    │  │(64-256 │  │ (8-32  │                      │
│  │  Core    │  │  KB)   │  │  KB)   │                      │
│  └────┬─────┘  └───┬────┘  └───┬────┘                      │
│       │            │           │                             │
│  ┌────┴────────────┴───────────┴────────────────────┐       │
│  │              AHB-Lite Bus (High Speed)            │       │
│  └────────────────────────┬─────────────────────────┘       │
│                           │                                  │
│  ┌────────────────────────┴─────────────────────────┐       │
│  │           AHB-APB Bridge                          │       │
│  └────────────────────────┬─────────────────────────┘       │
│                           │                                  │
│  ┌────────────────────────┴─────────────────────────┐       │
│  │              APB Bus (Low Speed Peripherals)      │       │
│  └──┬──────┬──────┬──────┬──────┬──────┬───────────┘       │
│     │      │      │      │      │      │                     │
│  ┌──┴──┐┌──┴──┐┌──┴──┐┌──┴──┐┌──┴──┐┌──┴──┐               │
│  │GPIO ││UART ││SPI  ││I2C  ││Timer││ ADC │               │
│  └─────┘└─────┘└─────┘└─────┘└─────┘└─────┘               │
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │  NVIC    │  │  Clock   │  │  Power   │                  │
│  │(Interrupt│  │ Generator│  │Management│                  │
│  │Controller│  │  + PLL   │  │          │                  │
│  └──────────┘  └──────────┘  └──────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Bus Architecture

Embedded SoCs use a hierarchical bus to connect components:

| Bus | Speed | Connected To |
|-----|:-----:|-------------|
| **AHB-Lite** | High (CPU clock) | CPU, Flash, SRAM, DMA |
| **APB** | Low (divided clock) | GPIO, UART, SPI, I2C, Timer, ADC |
| **AHB-APB Bridge** | — | Converts between AHB and APB protocols |

**AHB (Advanced High-performance Bus):**
- Single-cycle pipelined transfers
- Burst transfers supported
- Used for high-bandwidth components

**APB (Advanced Peripheral Bus):**
- Two-cycle minimum transfer (setup + access)
- Simple, low-power
- Used for slow peripherals that don't need high bandwidth

### 1.3 Memory Map

Embedded SoCs use **memory-mapped I/O** — peripherals are accessed at specific memory addresses, just like regular memory:

```
ARM Cortex-M Memory Map (32-bit address space):
┌──────────────────────┐ 0xFFFFFFFF
│  System (SCS, NVIC)  │ 0xE0000000 - 0xFFFFFFFF
├──────────────────────┤
│  Private Peripheral  │ 0xE0000000 - 0xE00FFFFF
├──────────────────────┤
│  External Device     │ 0xA0000000 - 0xDFFFFFFF
├──────────────────────┤
│  External RAM        │ 0x60000000 - 0x9FFFFFFF
├──────────────────────┤
│  Peripheral          │ 0x40000000 - 0x5FFFFFFF
│  (GPIO, UART, etc.)  │
├──────────────────────┤
│  SRAM                │ 0x20000000 - 0x3FFFFFFF
├──────────────────────┤
│  Code (Flash)        │ 0x00000000 - 0x1FFFFFFF
└──────────────────────┘ 0x00000000
```

---

## 2. ARM Cortex-M0+ Overview

### 2.1 Design Philosophy

The Cortex-M0+ is designed for:
- **Minimum gate count** (~12,000 gates) — smallest ARM core
- **Ultra-low power** — suitable for battery-operated and energy-harvesting devices
- **Deterministic behavior** — predictable execution timing for real-time applications
- **Easy programmability** — full C/C++ support, no need for assembly

### 2.2 Key Specifications

| Feature | Cortex-M0+ |
|---------|-----------|
| Architecture | ARMv6-M |
| Pipeline | 2-stage (Fetch + Execute) |
| Instruction set | Thumb (16-bit) + subset of Thumb-2 (32-bit) |
| Registers | 16 (R0–R15) |
| Interrupts | Up to 32 external + NMI |
| Bus interface | AHB-Lite (von Neumann or Harvard) |
| Gate count | ~12,000 |
| Power | ~12 μW/MHz (at 90nm) |
| Clock speed | Up to 48 MHz (typical) |

### 2.3 Comparison with Other Cortex-M Cores

| Feature | M0+ | M0 | M3 | M4 | M7 |
|---------|:---:|:--:|:--:|:--:|:--:|
| Pipeline stages | 2 | 3 | 3 | 3 | 6 |
| Gate count | 12K | 12K | 40K | 50K | 100K+ |
| Hardware multiply | 1 or 32 cycle | 1 or 32 cycle | 1 cycle | 1 cycle | 1 cycle |
| Hardware divide | No | No | Yes | Yes | Yes |
| DSP extensions | No | No | No | Yes | Yes |
| FPU | No | No | No | Optional | Yes |
| Max clock | ~48 MHz | ~48 MHz | ~120 MHz | ~180 MHz | ~400+ MHz |
| Typical use | IoT sensors | Simple control | General embedded | Audio/motor | High-perf embedded |

---

## 3. Cortex-M0+ Registers

### 3.1 Register Set

```
General Purpose:           Special Registers:
┌────┬───────────┐        ┌────┬──────────────────┐
│ R0 │ Argument  │        │ R13│ SP (Stack Pointer)│
│ R1 │ Argument  │        │    │  MSP (Main SP)    │
│ R2 │ Argument  │        │    │  PSP (Process SP) │
│ R3 │ Argument  │        ├────┼──────────────────┤
│ R4 │ Callee-   │        │ R14│ LR (Link Register)│
│ R5 │ saved     │        ├────┼──────────────────┤
│ R6 │           │        │ R15│ PC (Program Ctr)  │
│ R7 │           │        └────┴──────────────────┘
├────┤           │
│ R8 │ High regs │        Special Purpose:
│ R9 │ (limited  │        ┌──────────────────────┐
│R10 │  access)  │        │ xPSR (Program Status)│
│R11 │           │        │  ├─ APSR (flags)     │
│R12 │           │        │  ├─ IPSR (exception) │
└────┴───────────┘        │  └─ EPSR (execution) │
                           ├──────────────────────┤
                           │ PRIMASK (int mask)   │
                           │ CONTROL (priv/stack) │
                           └──────────────────────┘
```

### 3.2 Important Registers

**Stack Pointer (R13 / SP):**
- Two stack pointers: **MSP** (Main Stack Pointer) for handler/OS mode, **PSP** (Process Stack Pointer) for user/thread mode
- Used for function calls, local variables, interrupt handling
- Stack grows **downward** (from high to low addresses)

**Link Register (R14 / LR):**
- Stores the **return address** when a function is called (via `BL` instruction)
- On exception entry, stores a special **EXC_RETURN** value

**Program Counter (R15 / PC):**
- Points to the current instruction + 4 (due to pipeline)
- Bit 0 must always be 1 (indicates Thumb mode)

**Program Status Register (xPSR):**

```
31 30 29 28 27 26 ........... 8  7  6  5  4  3  2  1  0
┌──┬──┬──┬──┬──┬─────────────┬───────────────────────────┐
│N │Z │C │V │  │             │     Exception Number      │
└──┴──┴──┴──┴──┴─────────────┴───────────────────────────┘
 APSR flags                    IPSR (which interrupt is active)
```

| Flag | Meaning |
|------|---------|
| N | Negative (result bit 31 = 1) |
| Z | Zero (result = 0) |
| C | Carry (unsigned overflow) |
| V | Overflow (signed overflow) |

---

## 4. The Two-Stage Pipeline

### 4.1 Pipeline Structure

The Cortex-M0+ uses a simple **2-stage pipeline**:

```
┌────────────────┐    ┌────────────────┐
│    FETCH       │───►│    EXECUTE     │
│ Read inst from │    │ Decode + ALU   │
│ memory         │    │ + Register     │
│                │    │   access       │
└────────────────┘    └────────────────┘
```

**Why only 2 stages?** (vs. 5 in our RISC-V study)
- Simpler hardware → fewer gates → lower power
- Shorter pipeline → lower branch penalty (just 1 cycle)
- Deterministic timing → easier to predict execution time for real-time systems

### 4.2 Branch Penalty

With a 2-stage pipeline, a taken branch wastes only **1 fetch cycle**:

```
Cycle:  1      2      3      4
BEQ:   [FETCH][EXEC]
wrong:        [FETCH] → FLUSHED
target:               [FETCH][EXEC]
```

Compare this to the 3-cycle penalty we saw with the 5-stage RISC-V pipeline — the M0+'s shorter pipeline is more forgiving.

---

## 5. Thumb Instruction Set

### 5.1 Why 16-bit Instructions?

The Cortex-M0+ uses the **Thumb** instruction set — predominantly 16-bit instructions:

| Advantage | Explanation |
|-----------|-------------|
| **Smaller code** | 16-bit instructions use half the memory of 32-bit instructions |
| **Lower cost** | Less Flash memory needed → cheaper chips |
| **Better I-cache** | More instructions fit per cache line |
| **Lower power** | Fewer bits to fetch from memory per instruction |

**Trade-off:** 16-bit encoding limits the number of registers and immediate values that can be specified. Thumb solves this by:
- Only accessing R0–R7 for most operations (3-bit register specifier)
- Using R8–R12 only with special MOV/ADD instructions
- Providing a subset of ARM's full functionality

### 5.2 Key Thumb Instructions

| Category | Instruction | Operation |
|----------|------------|-----------|
| Arithmetic | `ADDS Rd, Rn, Rm` | Rd = Rn + Rm |
| | `SUBS Rd, Rn, Rm` | Rd = Rn - Rm |
| | `ADDS Rd, Rn, #imm3` | Rd = Rn + imm (3-bit immediate) |
| | `MULS Rd, Rn, Rd` | Rd = Rd × Rn |
| Logic | `ANDS Rd, Rd, Rm` | Rd = Rd & Rm |
| | `ORRS Rd, Rd, Rm` | Rd = Rd \| Rm |
| | `EORS Rd, Rd, Rm` | Rd = Rd ^ Rm |
| | `MVNS Rd, Rm` | Rd = ~Rm |
| Shift | `LSLS Rd, Rm, #imm5` | Rd = Rm << imm |
| | `LSRS Rd, Rm, #imm5` | Rd = Rm >> imm (logical) |
| | `ASRS Rd, Rm, #imm5` | Rd = Rm >> imm (arithmetic) |
| Load/Store | `LDR Rd, [Rn, #imm5]` | Rd = Mem[Rn + imm×4] |
| | `STR Rd, [Rn, #imm5]` | Mem[Rn + imm×4] = Rd |
| | `LDR Rd, [SP, #imm8]` | Rd = Mem[SP + imm×4] |
| Branch | `B label` | Unconditional branch |
| | `BEQ label` | Branch if Z == 1 |
| | `BL label` | Branch with link (function call) |
| Stack | `PUSH {reglist}` | Push registers to stack |
| | `POP {reglist}` | Pop registers from stack |

**Note:** Most Thumb instructions automatically update the condition flags (the "S" suffix is implied).

---

## 6. Processor Modes and Privilege Levels

### 6.1 Two Modes

| Mode | When Active | Stack Used | Privilege |
|------|-------------|:----------:|:---------:|
| **Thread Mode** | Normal code execution | MSP or PSP | Privileged or Unprivileged |
| **Handler Mode** | Exception/interrupt handling | MSP (always) | Privileged (always) |

```
                  ┌──────────────────┐
                  │   Thread Mode    │
                  │ (normal program) │
                  └───────┬──────────┘
                          │
              Exception / │ \ Exception
              Entry      │   \ Return
                          ▼
                  ┌──────────────────┐
                  │  Handler Mode    │
                  │ (ISR execution)  │
                  └──────────────────┘
```

### 6.2 Privilege Levels

- **Privileged:** Full access to all resources and instructions
- **Unprivileged:** Cannot access certain system registers or execute system instructions

This separation enables simple OS/RTOS implementations where application tasks run unprivileged and the OS runs privileged.

---

## 7. Nested Vectored Interrupt Controller (NVIC)

The NVIC is a key component of the Cortex-M0+, tightly integrated with the processor:

### 7.1 Features

| Feature | Cortex-M0+ |
|---------|-----------|
| External interrupts | Up to 32 |
| Priority levels | 4 (2-bit priority) |
| Priority grouping | Not supported |
| Nested interrupts | Yes |
| Tail-chaining | Yes |
| Late-arriving | Yes |

### 7.2 Exception Types

| Number | Type | Priority | Description |
|:------:|------|:--------:|-------------|
| 1 | Reset | -3 (highest) | System reset |
| 2 | NMI | -2 | Non-Maskable Interrupt |
| 3 | HardFault | -1 | All fault conditions |
| 11 | SVCall | Configurable | Supervisor call (SVC instruction) |
| 14 | PendSV | Configurable | Pendable service request (context switching) |
| 15 | SysTick | Configurable | System timer tick |
| 16+ | IRQ0–IRQ31 | Configurable | External peripheral interrupts |

### 7.3 Interrupt Latency

The Cortex-M0+ has a **deterministic** interrupt latency of **15 cycles** from interrupt request to first ISR instruction execution. This includes:

```
1. Finish current instruction (1-3 cycles)
2. Stack push (8 registers × 1 cycle each in some implementations)
3. Vector fetch (fetch ISR address from vector table)
4. Pipeline refill
─────────────────────────────────────
Total: ~15 cycles (worst case)
```

---

## 8. Boot Process

When the Cortex-M0+ comes out of reset:

```
Step 1: Read address 0x00000000 → Load into MSP (initial stack pointer)
Step 2: Read address 0x00000004 → Load into PC (Reset_Handler address)
Step 3: Begin executing from Reset_Handler in Thread Mode, Privileged

Vector Table (at 0x00000000):
┌──────────────┬──────────────────────────┐
│ Address      │ Content                  │
├──────────────┼──────────────────────────┤
│ 0x00000000   │ Initial MSP value        │
│ 0x00000004   │ Reset Handler address    │
│ 0x00000008   │ NMI Handler address      │
│ 0x0000000C   │ HardFault Handler addr   │
│ ...          │ ...                      │
│ 0x00000040   │ IRQ0 Handler address     │
│ 0x00000044   │ IRQ1 Handler address     │
│ ...          │ ...                      │
└──────────────┴──────────────────────────┘
```

The vector table is simply an array of function pointers, stored at the beginning of Flash memory.

---

## 9. Summary

| Feature | Detail |
|---------|--------|
| **Embedded SoC** | CPU + Memory + Peripherals + Bus on one chip |
| **Bus hierarchy** | AHB (fast) → Bridge → APB (slow peripherals) |
| **Memory-mapped I/O** | Peripherals accessed via specific memory addresses |
| **Cortex-M0+** | 2-stage pipeline, 12K gates, ultra-low power, ARMv6-M |
| **Thumb ISA** | Mostly 16-bit instructions for code density |
| **16 registers** | R0–R12 (GP), SP, LR, PC |
| **NVIC** | Up to 32 interrupts, 4 priority levels, 15-cycle latency |
| **Boot** | Loads MSP from 0x0, then jumps to Reset_Handler at 0x4 |

In the **next post ([SoC-13])**, we will learn how C code is compiled into Cortex-M0+ assembly and trace through key code constructs step by step.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
