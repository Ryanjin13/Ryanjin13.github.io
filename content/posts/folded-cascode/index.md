---
title: "Folded Cascode Structure"
date: 2024-08-21
description: "Folded Cascode amplifier architecture for high-frequency analog design"
categories: ["Circuits"]
tags: ["Analog Circuits", "Op-Amp", "Cascode"]
draft: false
---

{{< katex >}}

## Overview

The Folded Cascode amplifier is an advanced analog circuit topology that improves upon traditional cascode designs, offering better high-frequency performance and voltage headroom.

## Problem with Standard Cascode

**Miller Effect Issue:**

At high transconductance (\(G_m\)), the Miller Effect causes:

$$
C_{miller} = C_{gd} \times (1 + A_v)
$$

This increased capacitance reduces high-frequency gain.

## Folded Cascode Solution

### Key Advantages

| Characteristic | Standard Cascode | Folded Cascode |
|----------------|------------------|----------------|
| Voltage Headroom | Limited | Improved |
| Output Swing | Restricted | Extended |
| High-Freq Performance | Miller limited | Better |
| Complexity | Simple | More complex |

### Circuit Operation

**Structure:**
```
        VDD
         |
    [PMOS Cascode] ← IREF2 (bias)
         |
    +----+----+
    |         |
  [Input]   [Output]
    |         |
    [NMOS Cascode]
         |
        VSS
```

## Design Features

### 1. Lower On-Resistance

Folded structure reduces transistor \(R_{on}\) compared to stacked cascodes.

### 2. Improved Headroom

- Near VDD: PMOS cascode provides margin
- Near VSS: Extended output swing range

### 3. High Output Impedance

$$
R_{out} = g_{m} \cdot r_{o1} \cdot r_{o2}
$$

Cascode tail current source (M9-M10) enhances output impedance over single-transistor designs.

### 4. Power Supply Rejection

PMOS transistors connected to bias circuit IREF2 improve supply rejection.

### 5. Negative Feedback

Connecting Vout to Vin1 implements negative feedback for stability.

## Transistor Functions

| Transistors | Function |
|-------------|----------|
| M1-M2 | Input differential pair |
| M3-M4 | PMOS current mirror |
| M5-M6 | Cascode devices |
| M7-M8 | Output stage |
| M9-M10 | Cascode tail current source |
| M11 | Bias generation |

## Trade-offs

| Advantage | Disadvantage |
|-----------|--------------|
| Better high-freq response | Increased complexity |
| Improved headroom | Higher power consumption |
| Higher gain | More transistors |
| Better linearity | Larger area |

## Applications

- High-speed operational amplifiers
- Low-voltage analog design
- ADC/DAC front-ends
- Sensor interfaces
- RF circuits
