---
title: "Voltage Holding Ratio in LCD Pixels"
date: 2024-06-28
description: "Understanding voltage retention in LCD pixel circuits"
categories: ["Circuits"]
tags: ["LCD", "Display", "Pixel Circuit"]
draft: false
---

{{< katex >}}

## Overview

The Voltage Holding Ratio (VHR) is a critical parameter in LCD displays, measuring how well a pixel maintains its voltage between refresh cycles. Higher VHR means better image quality and reduced flicker.

## Definition

### Voltage Holding Ratio

$$
\text{VHR} = \frac{V_{end}}{V_{initial}} \times 100\%
$$

Where:
- \\(V_{initial}\\): Voltage at start of frame
- \\(V_{end}\\): Voltage at end of frame

### Ideal vs Reality

| Condition | VHR |
|-----------|-----|
| Ideal (no leakage) | 100% |
| Typical TFT-LCD | 95-99% |
| Minimum acceptable | ~90% |

## Voltage Decay Mechanism

### During Frame Period

```
V(t)
  │▓▓▓▓▓▓▓▓▓
  │         ╲
  │          ╲
  │           ╲▓▓▓▓▓
  └─────────────────→ t
    Write    Frame period
```

### Decay Equation

$$
V(t) = V_0 \cdot e^{-t/\tau}
$$

Where:

$$
\tau = R_{off} \cdot C_{total}
$$

- \\(R_{off}\\): TFT off-resistance
- \\(C_{total}\\): Pixel capacitance (Clc + Cst)

## Leakage Sources

### 1. TFT Leakage

$$
I_{TFT} = I_0 \cdot e^{(V_{gs} - V_{th})/nV_T}
$$

Even in "off" state, small current flows.

### 2. Liquid Crystal Leakage

$$
I_{LC} = \frac{V_{pixel}}{R_{LC}}
$$

LC has finite resistivity.

### 3. Gate Dielectric Leakage

Through gate insulator.

### 4. Parasitic Paths

Surface and bulk leakage currents.

## Impact of Low VHR

### Image Quality Issues

| VHR | Effect |
|-----|--------|
| >98% | Excellent |
| 95-98% | Good |
| 90-95% | Visible gray level shift |
| <90% | Flicker, poor image |

### Gray Level Accuracy

If voltage drops during frame:
- Brightness changes
- Wrong gray level displayed
- Worse at low gray levels

## Factors Affecting VHR

### Temperature

$$
I_{leak} \propto e^{-E_a/kT}
$$

Higher temperature → more leakage → lower VHR.

| Temperature | VHR Change |
|-------------|------------|
| 25°C | Reference |
| 50°C | -5% typical |
| 70°C | -10% typical |

### Frame Rate

Longer frame time → more decay:

$$
\text{VHR} = e^{-t_{frame}/\tau}
$$

| Refresh Rate | Frame Time | VHR Impact |
|--------------|------------|------------|
| 120 Hz | 8.3 ms | Highest |
| 60 Hz | 16.7 ms | Standard |
| 30 Hz | 33.3 ms | Lowest |

### Pixel Capacitance

$$
\Delta V = \frac{I_{leak} \cdot t}{C_{total}}
$$

Larger capacitance → less voltage drop → better VHR.

## Improving VHR

### Design Strategies

| Strategy | Effect |
|----------|--------|
| Larger Cst | More charge storage |
| Better TFT | Lower off-current |
| Higher refresh | Less time for decay |
| Low-ion LC | Reduces LC leakage |

### Material Selection

- High-resistivity LC materials
- Low-leakage TFT technology (IGZO vs a-Si)
- Quality dielectrics

## Measurement Method

### Test Setup

1. Apply known voltage to pixel
2. Wait one frame period
3. Measure remaining voltage

### Typical Results

```
Applied: 5.0V
After 16.7ms: 4.9V
VHR = 4.9/5.0 = 98%
```

## VHR vs TFT Technology

| TFT Type | Typical Off-Current | VHR |
|----------|--------------------|----|
| a-Si | ~1 pA | 95-98% |
| LTPS | ~0.1 pA | 97-99% |
| IGZO | ~0.01 pA | 99%+ |

IGZO's extremely low leakage enables:
- Lower refresh rates (power saving)
- Higher resolution (more time per line)

## Design Trade-offs

### Capacitor Size

| Larger Cst | Smaller Cst |
|------------|-------------|
| Better VHR | Lower VHR |
| Lower aperture | Higher aperture |
| Slower charging | Faster charging |

### Refresh Rate

| Higher Rate | Lower Rate |
|-------------|------------|
| Better VHR | Lower VHR |
| More power | Less power |
| Less motion blur | More motion blur |
