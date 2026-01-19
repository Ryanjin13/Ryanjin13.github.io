---
title: "I-V Characteristics of a-Si TFT"
date: 2024-06-25
description: "Understanding current-voltage characteristics of amorphous silicon thin-film transistors"
categories: ["Circuits"]
tags: ["TFT", "Display", "a-Si", "Electronics"]
draft: false
---

{{< katex >}}

## Overview

Amorphous silicon thin-film transistors (a-Si TFT) are the backbone of LCD display technology. Understanding their I-V characteristics is essential for display circuit design.

## Basic Structure

```
      Gate
       ↓
┌─────────────────┐
│    Gate Metal   │
├─────────────────┤
│   Gate Insulator│
├─────────────────┤
│     a-Si:H      │ ← Active layer
├───┬─────────┬───┤
│ n+│         │n+ │ ← Contact layer
└───┴─────────┴───┘
Source        Drain
```

## Operating Voltage Ranges

| Parameter | Range |
|-----------|-------|
| Gate voltage (Vgs) | -20V to +20V |
| Drain-source voltage (Vds) | 0 to 10V |
| Threshold voltage (Vth) | 1-3V typical |

## I-V Characteristics

### Transfer Characteristics (Id vs Vgs)

```
Id (log)
    │     ╱────── On region
    │    ╱
    │   ╱
    │  ╱
    │ ╱
    │╱_____________ Vgs
   -5V    0    Vth  20V
```

### Output Characteristics (Id vs Vds)

```
Id
    │         _____ Vgs = 20V
    │     ___╱_____ Vgs = 15V
    │   _╱_________ Vgs = 10V
    │ _╱___________ Vgs = 5V
    │╱_____________
    └───────────────── Vds
         Saturation
```

## Operating Regions

### Linear Region

When \\(V_{ds} < V_{gs} - V_{th}\\):

$$
I_d = \mu C_{ox} \frac{W}{L} \left[(V_{gs} - V_{th})V_{ds} - \frac{V_{ds}^2}{2}\right]
$$

### Saturation Region

When \\(V_{ds} \geq V_{gs} - V_{th}\\):

$$
I_d = \frac{1}{2} \mu C_{ox} \frac{W}{L} (V_{gs} - V_{th})^2
$$

Above ~20V, current plateaus in full saturation.

## Critical Design Fact: Incomplete Switching

**Important:** TFTs cannot completely close!

### Off-State Behavior

At \\(V_{gs} = -5V\\):
- TFT is "off" but leakage current exists
- Leakage is in picoampere range
- Complete off-state is impossible

### Leakage Current Equation

$$
I_{off} = I_0 \cdot e^{(V_{gs} - V_{th})/nkT/q}
$$

### Factors Increasing Leakage

| Factor | Effect |
|--------|--------|
| Shorter channel (ΔL) | Higher leakage |
| Higher Vds | Higher leakage |
| Higher temperature | Higher leakage |

## On/Off Current Ratio

$$
\frac{I_{on}}{I_{off}} > 10^6
$$

This ratio must be high enough for display operation:
- Ion: Charges pixel capacitor quickly
- Ioff: Must hold charge for frame period

## Charging Speed

The charging time constant:

$$
\tau = R_{on} \cdot C_{pixel}
$$

Where:
- \\(R_{on}\\): TFT on-resistance
- \\(C_{pixel}\\): Total pixel capacitance

### On-Resistance

$$
R_{on} = \frac{L}{\mu C_{ox} W (V_{gs} - V_{th})}
$$

Lower Ron → faster charging → higher gate voltage needed.

## Design Considerations

### Operating Window

$$
V_{gate,on} = 15-20V
$$
$$
V_{gate,off} = -5 \text{ to } -10V
$$

This ensures:
- Complete charging during on-time
- Minimal leakage during off-time

### Leakage Budget

For 60 Hz (16.7 ms frame):

$$
\Delta V = \frac{I_{leak} \cdot t_{frame}}{C_{st} + C_{lc}}
$$

Acceptable \\(\Delta V < 50mV\\) for imperceptible brightness change.

## a-Si TFT Limitations

| Limitation | Impact |
|------------|--------|
| Low mobility (~0.5 cm²/Vs) | Slow switching |
| Threshold shift | Long-term stability |
| Light sensitivity | Gate leakage in bright conditions |
| Temperature sensitivity | Performance variation |

## Comparison with Other TFT Types

| Property | a-Si | LTPS | IGZO |
|----------|------|------|------|
| Mobility (cm²/Vs) | 0.5-1 | 50-100 | 10-30 |
| Uniformity | Excellent | Moderate | Good |
| Cost | Low | High | Medium |
| Application | LCD TV | Mobile OLED | High-res LCD |

## Summary

Key points for a-Si TFT design:
1. TFTs don't completely turn off
2. Leakage current must be budgeted
3. On/off ratio > 10⁶ required
4. Charging time limits by Ron × C
5. Operating voltage: -5V to +20V typical
