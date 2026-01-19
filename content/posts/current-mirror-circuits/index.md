---
title: "Current Mirror Circuits in Analog Design"
date: 2024-08-12
description: "Understanding current mirror operation, design, and variations"
categories: ["Circuits"]
tags: ["Current Mirror", "Analog Design", "MOSFET", "Biasing"]
draft: false
---

{{< katex >}}

## Overview

Current mirrors are fundamental building blocks in analog circuit design, providing stable current sources and enabling precise current copying. This guide covers basic operation, design considerations, and advanced variations.

## Basic Current Mirror

### Circuit Structure

```
        VDD
         │
    ┌────┴────┐
    │         │
┌───┴───┐ ┌───┴───┐
│  M1   │ │  M2   │
│(diode)│ │(output)│
└───┬───┘ └───┬───┘
    │         │
    ├─────────┤
    │         │
   Iref     Iout
    │         │
   GND       Load
```

### Operating Principle

1. **Reference Side (M1):**
   - Diode-connected (gate tied to drain)
   - Sets \\(V_{GS}\\) based on \\(I_{ref}\\)
   - Always in saturation (\\(V_{DS} = V_{GS}\\))

2. **Output Side (M2):**
   - Shares \\(V_{GS}\\) with M1
   - Mirrors the current
   - Must be kept in saturation

### Current Relationship

For matched transistors (same \\(W/L\\)):

$$
I_{out} = I_{ref} \cdot \frac{(W/L)_2}{(W/L)_1}
$$

If \\((W/L)_1 = (W/L)_2\\):

$$
I_{out} = I_{ref}
$$

### Saturation Requirement

For accurate mirroring, M2 must be in saturation:

$$
V_{DS2} \geq V_{GS} - V_{th} = V_{ov}
$$

Where \\(V_{ov}\\) is the overdrive voltage.

## Non-Ideal Effects

### Channel Length Modulation

Real current includes \\(\lambda\\) effect:

$$
I_D = \frac{1}{2}\mu_n C_{ox}\frac{W}{L}(V_{GS} - V_{th})^2(1 + \lambda V_{DS})
$$

**Impact on Mirror:**

$$
\frac{I_{out}}{I_{ref}} = \frac{(W/L)_2}{(W/L)_1} \cdot \frac{1 + \lambda V_{DS2}}{1 + \lambda V_{DS1}}
$$

Since \\(V_{DS1} = V_{GS}\\) and \\(V_{DS2}\\) varies with load:

$$
\Delta I_{out} \propto \lambda(V_{DS2} - V_{DS1})
$$

### Output Impedance

The output impedance limits accuracy:

$$
r_{out} = \frac{1}{\lambda I_{out}} = r_o
$$

Higher \\(r_{out}\\) means better current stability.

## Transistor vs. Resistor Trade-offs

### Using Resistor Instead of Current Source

**Advantages:**
- Simplicity
- Inherent linearity
- Reduced noise
- Temperature stability
- Lower cost

**Disadvantages:**
- Reduced flexibility
- Lost current control
- Decreased gain
- Impedance matching difficulties
- Increased power consumption
- Limited frequency response

### Comparison

| Aspect | Transistor Current Source | Resistor |
|--------|--------------------------|----------|
| Output Impedance | High (\\(r_o\\)) | Fixed (\\(R\\)) |
| Current Control | Programmable | Fixed |
| Area | Small | Large (for high R) |
| Power | Low | Higher (\\(I^2R\\)) |

## Cascode Current Mirror

### Circuit

```
        VDD
         │
    ┌────┴────┐
    │         │
┌───┴───┐ ┌───┴───┐
│  M3   │ │  M4   │
│(cascode)│(cascode)│
└───┬───┘ └───┬───┘
    │         │
┌───┴───┐ ┌───┴───┐
│  M1   │ │  M2   │
│(diode)│ │(mirror)│
└───┬───┘ └───┬───┘
    │         │
   Iref     Iout
```

### Benefits

**Increased Output Impedance:**

$$
r_{out,cascode} = g_{m4} r_{o4} r_{o2}
$$

Compared to basic mirror (\\(r_o\\)), this is much higher.

**Better Current Matching:**
- Less sensitivity to \\(V_{DS}\\) variations
- Improved PSRR

### Trade-off

**Reduced Voltage Swing:**

$$
V_{out,min} = 2V_{ov} = 2(V_{GS} - V_{th})
$$

## Wide-Swing Current Mirror

### Purpose

Achieve high output impedance while maintaining voltage headroom.

### Circuit

```
        VDD
         │
    ┌────┴────┐
    │         │
┌───┴───┐ ┌───┴───┐
│  M3   │ │  M4   │
│       │ │       │
└───┬───┘ └───┬───┘
    │         │
   Vbias     Iout
    │         │
┌───┴───┐ ┌───┴───┐
│  M1   │ │  M2   │
│       │ │       │
└───┬───┘ └───┬───┘
    │         │
   Iin       GND
```

### Operation

1. **Input Stage (M1):** Detects incoming current \\(I_{in}\\)
2. **Bias Generation:** Creates appropriate gate voltage
3. **Output Stage (M2):** Mirrors current with high impedance
4. **Feedback:** Maintains \\(V_{DS}\\) near \\(V_{ov}\\)

### Minimum Output Voltage

$$
V_{out,min} = V_{ov2} + V_{ov4} = 2V_{ov}
$$

With proper biasing, both transistors operate just at the edge of saturation.

## Wilson Current Mirror

### Circuit

```
        VDD
         │
    ┌────┴────┐
    │         │
┌───┴───┐     │
│  M3   │─────┤
└───┬───┘     │
    │         │
┌───┴───┐ ┌───┴───┐
│  M1   │ │  M2   │
└───┬───┘ └───┬───┘
    │         │
   Iref     Iout
```

### Advantages

- High output impedance (\\(\approx g_m r_o^2\\))
- Self-biasing
- Negative feedback improves matching

## Design Considerations

### Sizing for Accuracy

| Parameter | Impact |
|-----------|--------|
| Matching | Use common-centroid layout |
| Length | Longer L reduces \\(\lambda\\) |
| \\(V_{ov}\\) | Lower gives higher \\(r_o\\) |

### Current Scaling

For \\(I_{out} = n \cdot I_{ref}\\):

$$
\frac{(W/L)_2}{(W/L)_1} = n
$$

**Methods:**
1. Increase W₂: \\(W_2 = n \cdot W_1\\)
2. Decrease L₂: \\(L_2 = L_1/n\\)
3. Parallel transistors: \\(n\\) copies of M2

### Temperature Compensation

Current mirrors are sensitive to temperature:

$$
I_D \propto \mu(T) \propto T^{-1.5}
$$

Use bandgap references for stable \\(I_{ref}\\).

## Summary

Key concepts in current mirror design:
1. **Basic mirror**: Simple, limited output impedance
2. **Channel length modulation**: Main error source
3. **Cascode**: High impedance, reduced headroom
4. **Wide-swing**: Balanced impedance and headroom
5. **Wilson**: Self-biasing, very high impedance
6. **Trade-offs**: Accuracy vs. voltage headroom vs. complexity

