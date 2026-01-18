---
title: "Current Mirror"
date: 2024-08-10
description: "Current mirror circuits for analog IC design"
categories: ["Circuits"]
tags: ["Analog Circuits", "Current Mirror", "IC Design"]
draft: false
---

{{< katex >}}

## Overview

A current mirror is a fundamental analog circuit that replicates (mirrors) a reference current to create a controlled output current.

## Basic Operation

```
    VDD                    VDD
     |                      |
   [M1]                   [M2]
     |                      |
     +------- Vg -----------+
     |                      |
   I_ref                  I_out
     |                      |
    GND                   Load
```

**Principle:**
- Left side (M1): Reference - establishes gate voltage through biasing
- Right side (M2): Output - mirrors the reference current

## Current Relationship

For matched transistors:

$$
I_{out} = I_{ref} \times \frac{(W/L)_2}{(W/L)_1}
$$

If \((W/L)_1 = (W/L)_2\):

$$
I_{out} = I_{ref}
$$

## Cascode Current Mirror

**Problem:** Basic mirror has finite output impedance.

**Solution:** Cascode configuration using M1-M2 to absorb drain voltage variations.

```
      VDD
       |
     [M3] ← Cascode device
       |
     [M1] ← Main mirror
       |
     I_ref
```

**Benefits:**
- Higher output impedance: \(R_{out} = g_m \cdot r_{o1} \cdot r_{o2}\)
- Better current accuracy
- Reduced channel length modulation effects

## Wide-Swing Current Mirror

Detects \(I_{in}\) and replicates to \(I_{out}\) through self-adjusting gate voltage feedback.

**Features:**
- Maintains saturation operation
- Precise current matching
- Extended output voltage range

## Transistor vs Resistor Bias

| Aspect | Transistor | Resistor |
|--------|------------|----------|
| Flexibility | High | Low |
| Noise | Higher | Lower |
| Temperature | Variable | Stable |
| Area | Smaller | Larger |
| Gain | Higher | Lower |
| Power (low-V) | Lower | Higher |

### When to Use Resistors

**Advantages:**
- Simplified design
- Linear V-I relationship
- Better temperature stability
- Lower noise
- Lower cost

**Disadvantages:**
- Less flexible bias adjustment
- Reduced current control
- Impedance matching difficulties

## Design Considerations

1. **Matching** - Use common-centroid layout
2. **Output impedance** - Consider cascode for high \(R_{out}\)
3. **Headroom** - Wide-swing for low VDD
4. **Noise** - Larger transistors for lower noise
5. **Mismatch** - Increase W×L product for better matching

## Applications

- Bias current generation
- Active loads in amplifiers
- Current DACs
- Reference current distribution
- Differential pair biasing
