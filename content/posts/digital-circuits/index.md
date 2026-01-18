---
title: "Digital Circuits"
date: 2024-08-25
description: "Fundamentals of digital circuit design and Verilog RTL"
categories: ["Circuits"]
tags: ["Digital Circuits", "Verilog", "RTL", "FPGA"]
draft: false
---

## Overview

Digital circuits form the foundation of modern computing, from simple logic gates to complex processors and memory systems.

## Hierarchical Design Structure

```
Physical Layer (Silicon)
    ↓
Transistor Level (NMOS/PMOS)
    ↓
Logic Gates (AND, OR, NOT)
    ↓
Functional Blocks (ALU, Registers)
    ↓
Processor / Memory Architecture
```

## Programming Languages

| Level | Language | Use Case |
|-------|----------|----------|
| High-level | C, Python | Software, algorithms |
| Low-level | Verilog, VHDL | Hardware description (RTL) |

## Transistor Basics

### NMOS and PMOS

In digital circuits, transistors function as **discrete switches**, not amplifiers.

| Type | Conducting When | Symbol |
|------|-----------------|--------|
| NMOS | Gate = HIGH (1) | n-channel |
| PMOS | Gate = LOW (0) | p-channel |

### CMOS Inverter

```
        VDD
         |
       [PMOS]
         |
Input ---+--- Output
         |
       [NMOS]
         |
        GND
```

## RTL Design Process

**RTL = Register Transfer Level**

1. **Behavioral Description** - High-level functionality
2. **Synthesis** - Convert to gate-level
3. **Placement & Routing** - Physical layout
4. **Timing Analysis** - Verify timing constraints

## Verilog Basics

### Module Definition

```verilog
module my_module (
    input  wire clk,
    input  wire reset,
    input  wire [7:0] data_in,
    output reg  [7:0] data_out
);
    // Module logic here
endmodule
```

### Port Connections

**Positional:**
```verilog
my_module inst1 (clk, reset, din, dout);
```

**Named (Recommended):**
```verilog
my_module inst1 (
    .clk(system_clk),
    .reset(sys_reset),
    .data_in(input_data),
    .data_out(output_data)
);
```

### Blocking vs Non-Blocking

| Assignment | Symbol | Use Case |
|------------|--------|----------|
| Blocking | `=` | Combinational logic |
| Non-blocking | `<=` | Sequential logic |

**Combinational Logic:**
```verilog
always @(*) begin
    y = a & b;      // Blocking
    z = y | c;      // Executes after y
end
```

**Sequential Logic:**
```verilog
always @(posedge clk) begin
    q <= d;         // Non-blocking
    q2 <= q;        // Both execute simultaneously
end
```

## Timing Concepts

### Propagation Delay

Time for signal to travel through a gate:
- Rise time (t_r)
- Fall time (t_f)
- Propagation delay (t_pd)

### Clock Skew

**Problem:** Clock arrives at different times due to:
- Wire length differences
- External noise
- Temperature variations

**Solution:** Phase-Locked Loop (PLL)
- Synchronizes clock distribution
- Compensates for skew
- Generates clean clock edges

## Design Tips

1. **Use non-blocking for flip-flops** - Prevents race conditions
2. **Synchronize inputs** - Use double-flip-flop for async signals
3. **Reset all registers** - Ensure known initial state
4. **Avoid latches** - Use complete if-else or case statements
