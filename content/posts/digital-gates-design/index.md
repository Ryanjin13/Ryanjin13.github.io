---
title: "Digital Gates Design Fundamentals"
date: 2024-08-12
description: "CMOS gate design principles including sizing, capacitance, and optimization"
categories: ["Circuits"]
tags: ["Digital Design", "CMOS", "Logic Gates", "VLSI"]
draft: false
---

{{< katex >}}

## Overview

Digital gate design requires careful consideration of transistor sizing, capacitance effects, and timing optimization. This guide covers the fundamental principles of CMOS logic gate design.

## NMOS vs PMOS Mobility

### The Mobility Problem

NMOS transistors have approximately 2× higher mobility than PMOS:

$$
\mu_n \approx 2\mu_p
$$

This means for the same dimensions, NMOS can carry more current:

$$
I_D = \frac{1}{2}\mu C_{ox}\frac{W}{L}(V_{GS} - V_{th})^2
$$

### Solution: Width Compensation

To achieve equal rise and fall times, PMOS width is doubled:

$$
W_p = 2W_n
$$

**Symmetric Inverter:**

```
        VDD
         │
      ┌──┴──┐
      │ PMOS│  W = 2W_n
      └──┬──┘
         │
In ──────┼────── Out
         │
      ┌──┴──┐
      │ NMOS│  W = W_n
      └──┬──┘
         │
        GND
```

**Result:**
- \\(t_{rise} \approx t_{fall}\\)
- Consistent switching behavior
- Predictable timing

## Oxide Capacitance

### Definition

Gate oxide capacitance per unit area:

$$
C_{ox} = \frac{\varepsilon_{ox}}{t_{ox}}
$$

Where:
- \\(\varepsilon_{ox}\\): Oxide permittivity (\\(\approx 3.9\varepsilon_0\\) for SiO₂)
- \\(t_{ox}\\): Oxide thickness

### Impact on Performance

**Larger \\(C_{ox}\\):**
- Better gate control over channel
- Higher drive current
- Improved \\(g_m\\)

**Trade-off:**
- Increased gate capacitance
- Higher leakage current (thin oxide)

### Scaling Trends

| Technology | \\(t_{ox}\\) (nm) | \\(C_{ox}\\) (fF/μm²) |
|------------|-------------------|----------------------|
| 180nm | 4.0 | 8.6 |
| 90nm | 2.0 | 17.2 |
| 45nm | 1.2 | 28.7 |
| 22nm | 0.9 | 38.3 |

## Depletion Capacitance Modulation

### Body Effect Factor

The factor \\(m\\) captures short-channel effects:

$$
m = 1 + \frac{C_{dm}}{C_{ox}}
$$

Where \\(C_{dm}\\) is the depletion capacitance modulation.

**Interpretation:**
- \\(m \approx 1\\): Long channel behavior
- \\(m > 1\\): Short channel effects present
- Higher \\(m\\) indicates stronger substrate influence

### Impact on Threshold Voltage

$$
V_{th} = V_{th,long} - \Delta V_{th}
$$

Short-channel effects reduce threshold voltage.

## Series Transistor Sizing

### NMOS in Series

For NAND gates, series NMOS requires width increase:

```
        Out
         │
      ┌──┴──┐
 A ───│NMOS1│  W = 2W_n
      └──┬──┘
         │
      ┌──┴──┐
 B ───│NMOS2│  W = 2W_n
      └──┬──┘
         │
        GND
```

**Reasoning:**
- Series resistance doubles
- Double width to maintain current

### PMOS in Series

For NOR gates, series PMOS needs 4× width:

```
        VDD
         │
      ┌──┴──┐
 A ───│PMOS1│  W = 4W_n
      └──┬──┘
         │
      ┌──┴──┐
 B ───│PMOS2│  W = 4W_n
      └──┬──┘
         │
        Out
```

**Calculation:**
- Base PMOS: 2× (mobility compensation)
- Series: 2× (resistance compensation)
- Total: 2 × 2 = 4×

### General Sizing Rule

For \\(n\\) transistors in series:

$$
W_{series} = n \times W_{single}
$$

## Gate Delay Optimization

### Propagation Delay

$$
t_p = \frac{C_L \cdot V_{DD}}{2 \cdot I_{avg}}
$$

Where:
- \\(C_L\\): Load capacitance
- \\(V_{DD}\\): Supply voltage
- \\(I_{avg}\\): Average switching current

### Tapered Buffer Chain

For driving large capacitive loads, use progressively sized buffers:

```
                 ┌───┐    ┌───┐    ┌───┐
In ──▶│ 1 │──▶│ f │──▶│f² │──▶ Out
                 └───┘    └───┘    └───┘
                  W        fW       f²W
```

**Optimal Tapering Factor:**

$$
f_{opt} = e \approx 2.7
$$

**Number of Stages:**

$$
N = \log_f\left(\frac{C_{out}}{C_{in}}\right)
$$

**Minimum Delay:**

$$
t_{total} = N \cdot t_{unit} \cdot f
$$

### Comparison: Single Buffer vs. Tapered Chain

| Approach | Delay | Area |
|----------|-------|------|
| Single large buffer | High (large input cap) | Large |
| Tapered chain | Lower (distributed) | Similar total |

## Logic Gate Sizing

### NAND Gate

```
        VDD
         │
      ┌──┴──┐   ┌──┴──┐
 A ───│ Pp  │───│ Pp  │─── B
      └──┬──┘   └──┬──┘
         └────┬────┘
              │
             Out
              │
           ┌──┴──┐
      A ───│ 2Wn │
           └──┬──┘
           ┌──┴──┐
      B ───│ 2Wn │
           └──┬──┘
              │
             GND
```

**Sizing:**
- PMOS: \\(W_p\\) (parallel, no increase needed)
- NMOS: \\(2W_n\\) (series, doubled)

### NOR Gate

```
        VDD
         │
      ┌──┴──┐
 A ───│ 4Wp │
      └──┬──┘
      ┌──┴──┐
 B ───│ 4Wp │
      └──┬──┘
         │
        Out
         │
      ┌──┴──┐   ┌──┴──┐
 A ───│ Wn  │───│ Wn  │─── B
      └──┬──┘   └──┬──┘
         └────┬────┘
             GND
```

**Sizing:**
- PMOS: \\(4W_p\\) (series, ×2 for mobility, ×2 for series)
- NMOS: \\(W_n\\) (parallel, no increase)

## Capacitance Components

### Total Gate Capacitance

$$
C_{total} = C_g + C_{gd,overlap} + C_{gs,overlap}
$$

Where:
- \\(C_g = C_{ox} \cdot W \cdot L\\): Gate capacitance
- \\(C_{gd,overlap}\\): Gate-drain overlap
- \\(C_{gs,overlap}\\): Gate-source overlap

### Load Capacitance

$$
C_L = C_{self} + C_{wire} + C_{fanout}
$$

## Summary

Key principles in digital gate design:
1. **PMOS sizing**: 2× NMOS width for equal mobility
2. **Series transistors**: Multiply width by series count
3. **Tapered buffers**: Optimal factor \\(f \approx e\\)
4. **NAND**: Efficient (NMOS in series smaller area)
5. **NOR**: Less efficient (PMOS in series requires large area)
6. **Trade-offs**: Speed vs. area vs. power

