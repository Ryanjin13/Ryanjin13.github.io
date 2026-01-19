---
title: "Single Stage Amplifier Fundamentals"
date: 2024-08-12
description: "Understanding transconductance, Miller effect, and common amplifier configurations"
categories: ["Circuits"]
tags: ["Amplifier", "MOSFET", "Analog Design", "Transconductance"]
draft: false
---

{{< katex >}}

## Overview

Single-stage amplifiers form the building blocks of analog circuit design. Understanding transconductance, the Miller effect, and common configurations is essential for designing high-performance analog systems.

## Transconductance

### Definition

Transconductance (\\(g_m\\)) describes how output current changes with input voltage:

$$
g_m = \frac{\partial I_D}{\partial V_{GS}} \bigg|_{V_{DS}=\text{const}}
$$

### Calculation Methods

**1. From Drain Current Equation (Saturation):**

$$
I_D = \frac{1}{2}\mu_n C_{ox} \frac{W}{L}(V_{GS} - V_{th})^2
$$

Taking the derivative:

$$
g_m = \mu_n C_{ox} \frac{W}{L}(V_{GS} - V_{th})
$$

**2. In Terms of Drain Current:**

$$
g_m = \sqrt{2\mu_n C_{ox} \frac{W}{L} I_D}
$$

**3. In Terms of Overdrive Voltage:**

$$
g_m = \frac{2I_D}{V_{GS} - V_{th}} = \frac{2I_D}{V_{ov}}
$$

**4. Small-Signal Parameter:**

$$
g_m = \frac{i_d}{v_{gs}}
$$

## MOSFET Operating Regions

### I-V Characteristics

```
I_D
 │                    ___________  Saturation
 │                 __/
 │              __/
 │           __/
 │        __/
 │     __/  Triode (Linear)
 │  __/
 │_/
 └────────────────────────────────── V_DS
           V_DSAT
```

### Region Boundaries

| Region | Condition | I_D Expression |
|--------|-----------|----------------|
| Cutoff | \\(V_{GS} < V_{th}\\) | \\(\approx 0\\) |
| Triode | \\(V_{DS} < V_{GS} - V_{th}\\) | \\(\mu_n C_{ox} \frac{W}{L}[(V_{GS}-V_{th})V_{DS} - \frac{V_{DS}^2}{2}]\\) |
| Saturation | \\(V_{DS} \geq V_{GS} - V_{th}\\) | \\(\frac{1}{2}\mu_n C_{ox} \frac{W}{L}(V_{GS} - V_{th})^2(1+\lambda V_{DS})\\) |

## Body Effect

When the source-to-body voltage is non-zero:

$$
V_{th} = V_{th0} + \gamma(\sqrt{2\phi_F + V_{SB}} - \sqrt{2\phi_F})
$$

Where:
- \\(V_{th0}\\): Zero-bias threshold voltage
- \\(\gamma\\): Body effect coefficient
- \\(\phi_F\\): Fermi potential
- \\(V_{SB}\\): Source-body voltage

**Impact:**
- Positive \\(V_{SB}\\) increases threshold voltage
- Reduces drain current for fixed \\(V_{GS}\\)
- Impedes channel formation

## Miller Effect

### Concept

The Miller effect describes how feedback capacitance appears larger due to voltage gain:

$$
C_{Miller} = C_{gd}(1 + |A_v|)
$$

Where \\(A_v\\) is the voltage gain of the stage.

### Impact on Frequency Response

```
Gain (dB)
    │
    │   Low frequency            High frequency
    │   (improved by             (degraded by
    │    Miller effect)          increased capacitance)
    │_____
    │     \_
    │       \__
    │          \___
    │              \____
    └────────────────────────── Frequency (log)
                f_3dB
```

**3dB Bandwidth:**

$$
f_{3dB} = \frac{1}{2\pi R_{in}(C_{in} + C_{Miller})}
$$

## Common Amplifier Configurations

### Common-Source Amplifier

```
        VDD
         │
        [RD]
         │
         ├───── Vout
         │
      ┌──┴──┐
Vin ──│ M1  │
      └──┬──┘
         │
        GND
```

**Characteristics:**

| Parameter | Expression |
|-----------|------------|
| Voltage Gain | \\(A_v = -g_m R_D\\) |
| Input Impedance | \\(R_{in} \approx \infty\\) |
| Output Impedance | \\(R_{out} = R_D \parallel r_o\\) |

**Operation:**
1. Input voltage rise increases \\(V_{GS}\\)
2. Drain current increases proportionally (\\(g_m\\))
3. Voltage drop across \\(R_D\\) increases
4. Output voltage decreases (inverted)

### Common-Source with Current Source

```
        VDD
         │
      ┌──┴──┐
      │ M2  │ (current source)
      └──┬──┘
         │
         ├───── Vout
         │
      ┌──┴──┐
Vin ──│ M1  │
      └──┬──┘
         │
        GND
```

**Benefits:**
- Higher output impedance: \\(R_{out} = r_{o1} \parallel r_{o2}\\)
- Higher gain: \\(A_v = -g_{m1}(r_{o1} \parallel r_{o2})\\)
- Better power supply rejection

### Source Follower (Common-Drain)

```
        VDD
         │
      ┌──┴──┐
Vin ──│ M1  │
      └──┬──┘
         │
         ├───── Vout
         │
        [RS]
         │
        GND
```

**Characteristics:**

| Parameter | Expression |
|-----------|------------|
| Voltage Gain | \\(A_v \approx \frac{g_m R_S}{1 + g_m R_S} \approx 1\\) |
| Input Impedance | \\(R_{in} \approx \infty\\) |
| Output Impedance | \\(R_{out} \approx \frac{1}{g_m}\\) |

**Operation:**
- Output follows input with unity gain
- Low output impedance (buffer function)
- No phase inversion

### Common-Gate

```
        VDD
         │
        [RD]
         │
         ├───── Vout
         │
      ┌──┴──┐
      │ M1  ├───── Vbias
      └──┬──┘
         │
Vin ─────┤
         │
        [RS]
         │
        GND
```

**Characteristics:**

| Parameter | Expression |
|-----------|------------|
| Voltage Gain | \\(A_v = g_m R_D\\) |
| Input Impedance | \\(R_{in} \approx \frac{1}{g_m}\\) |
| Output Impedance | \\(R_{out} = R_D\\) |

**Applications:**
- High-frequency circuits (no Miller effect on input)
- Current sensing
- Cascode stage

## Cascode Configuration

Combines common-source and common-gate for high gain:

```
        VDD
         │
        [RD]
         │
         ├───── Vout
         │
      ┌──┴──┐
Vbias─│ M2  │ (CG)
      └──┬──┘
         │
      ┌──┴──┐
Vin ──│ M1  │ (CS)
      └──┬──┘
         │
        GND
```

**Gain:**
$$
A_v = -g_{m1}(g_{m2}r_{o2}r_{o1} \parallel R_D)
$$

**Benefits:**
- Very high output impedance
- Reduced Miller effect
- Higher gain

## Summary

Key concepts in single-stage amplifiers:
1. **Transconductance**: Links input voltage to output current
2. **Miller effect**: Capacitance multiplication impacts bandwidth
3. **Common-source**: Inverting, high gain
4. **Source follower**: Unity gain, low output impedance
5. **Common-gate**: Non-inverting, low input impedance
6. **Cascode**: Combines CS and CG for optimal performance

