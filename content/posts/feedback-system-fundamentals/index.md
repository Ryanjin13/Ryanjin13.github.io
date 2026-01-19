---
title: "Feedback System Fundamentals"
date: 2024-08-12
description: "Understanding feedback in amplifiers, stability, and applications like LDO and PLL"
categories: ["Circuits"]
tags: ["Feedback", "Amplifiers", "Stability", "LDO", "PLL"]
draft: false
---

{{< katex >}}

## Overview

Feedback is a fundamental concept in analog circuit design, enabling stable amplification, precise voltage regulation, and frequency synthesis. Understanding feedback principles is essential for designing reliable electronic systems.

## Basic Feedback Concept

### Feedback Loop Structure

```
              ┌──────────────────────┐
              │                      │
    ─────────(+)───▶ Amplifier A ────┼────▶ Output
              ▲                      │
              │                      │
              └──── Feedback β ◀─────┘
```

### Closed-Loop Gain

With negative feedback:

$$
A_{CL} = \frac{A}{1 + A\beta}
$$

Where:
- \\(A\\): Open-loop gain
- \\(\beta\\): Feedback factor
- \\(A\beta\\): Loop gain

For large \\(A\beta\\):

$$
A_{CL} \approx \frac{1}{\beta}
$$

The closed-loop gain becomes independent of the amplifier gain!

## Negative vs Positive Feedback

### Negative Feedback

Output opposes input change → System converges to stable value.

**Benefits:**
- Reduced sensitivity to component variations
- Improved linearity
- Extended bandwidth
- Predictable gain (\\(1/\beta\\))

### Positive Feedback

Output reinforces input change → System diverges or oscillates.

**Applications:**
- Oscillators
- Comparators with hysteresis
- Latches

**Condition for Oscillation (Barkhausen):**

$$
|A\beta| = 1 \quad \text{and} \quad \angle A\beta = 0° \text{ or } 360°
$$

## Frequency Response

### Parasitic Capacitance Effect

At high frequencies, parasitic capacitance absorbs high-frequency components:

```
Gain (dB)
    │
    │ ────┐
    │     │ -20 dB/decade
    │     └────┐
    │          │ -40 dB/decade
    │          └────┐
    │               │ -60 dB/decade
    └─────────────────────── Frequency (log)
         f₁    f₂    f₃
        (poles)
```

### Gain Reduction per Pole

Each pole contributes:
- -20 dB/decade magnitude roll-off
- -90° phase shift (asymptotically)

| Number of Poles | Roll-off | Phase Shift |
|-----------------|----------|-------------|
| 1 | -20 dB/dec | -90° |
| 2 | -40 dB/dec | -180° |
| 3 | -60 dB/dec | -270° |

### Three-Pole Threshold

With three poles, phase shift can reach -270°, making stability critical:

$$
\text{If } |A\beta| > 1 \text{ when phase} = -180° \rightarrow \text{Unstable}
$$

## Stability Analysis

### Phase Margin

The phase margin measures stability:

$$
PM = 180° + \angle A\beta \bigg|_{|A\beta|=1}
$$

**Acceptable Range:** 45° - 60°

```
|Aβ| (dB)
    │
    │  \
    │   \
    │    \  ← Unity gain (0 dB)
    │─────\────────────────────
    │      \
    │       \
    └──────────────────────── f
                ↑
           Phase Margin
           measured here
```

### Gain Margin

$$
GM = \frac{1}{|A\beta|} \bigg|_{\angle A\beta = -180°}
$$

Expressed in dB:

$$
GM_{dB} = -20\log|A\beta| \bigg|_{\angle A\beta = -180°}
$$

**Acceptable Range:** > 10 dB

### Bandwidth

Measured at -3 dB points:

$$
BW = f_{-3dB}
$$

For first-order systems:
$$
f_{-3dB} = \frac{1}{2\pi RC}
$$

## Applications

### LDO (Low Drop-out) Regulator

Provides stable voltage output with minimal dropout voltage.

```
        VIN
         │
      ┌──┴──┐
      │Pass │
      │Trans│
      └──┬──┘
         │
         ├──────────── VOUT
         │
        ┌┴┐
        │R1│
        └┬┘
         │
         ├───▶ Error Amp ◀─── VREF
         │
        ┌┴┐
        │R2│
        └┬┘
         │
        GND
```

**Output Voltage:**

$$
V_{OUT} = V_{REF}\left(1 + \frac{R_1}{R_2}\right)
$$

**Key Specifications:**
- Dropout voltage: \\(V_{IN} - V_{OUT,min}\\)
- Line regulation: \\(\Delta V_{OUT}/\Delta V_{IN}\\)
- Load regulation: \\(\Delta V_{OUT}/\Delta I_{LOAD}\\)
- PSRR: Power supply rejection ratio

### PLL (Phase-Locked Loop)

Maintains constant output frequency locked to a reference.

```
    ┌─────────────────────────────────────────┐
    │                                         │
    │    ┌───────┐   ┌──────┐   ┌─────┐      │
FREF──▶│Phase  │──▶│Charge│──▶│Loop │──▶VCO──┼──▶FOUT
    │   │Detect │   │Pump  │   │Filter│      │
    │   └───────┘   └──────┘   └─────┘      │
    │       ▲                                │
    │       │       ┌────────┐               │
    │       └───────│Divider │◀──────────────┘
    │               │  ÷N    │
    │               └────────┘
    └─────────────────────────────────────────┘
```

**Locked Condition:**

$$
F_{OUT} = N \cdot F_{REF}
$$

**Components:**

| Block | Function |
|-------|----------|
| Phase Detector | Compares phases of FREF and divided output |
| Charge Pump | Converts phase error to current |
| Loop Filter | Smooths control voltage |
| VCO | Voltage-controlled oscillator |
| Divider | Divides output frequency by N |

**Frequency Adjustment:**

$$
N = 2^n \quad \text{for integer-N PLLs}
$$

Fractional-N PLLs allow finer frequency steps.

## Design Guidelines

### Compensation Techniques

| Issue | Solution |
|-------|----------|
| Low phase margin | Dominant pole compensation |
| Slow response | Increase bandwidth |
| Peaking | Reduce Q factor |
| Oscillation | Add compensation capacitor |

### Dominant Pole Compensation

Add a low-frequency pole to ensure stability:

$$
f_d \ll f_1, f_2, f_3
$$

This ensures 20 dB/decade roll-off at unity gain.

## Summary

Key concepts in feedback systems:
1. **Negative feedback**: Stabilizes gain to \\(1/\beta\\)
2. **Phase margin**: 45°-60° for stability
3. **Poles**: Each adds -90° phase shift
4. **Three poles**: Critical stability threshold
5. **LDO**: Voltage regulation via feedback
6. **PLL**: Frequency synthesis via phase feedback

