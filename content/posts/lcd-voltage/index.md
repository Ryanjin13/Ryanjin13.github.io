---
title: "Liquid Crystal Response to Voltage"
date: 2024-06-22
description: "How voltage controls light transmission in LCD displays"
categories: ["Circuits"]
tags: ["LCD", "Display", "Liquid Crystal"]
draft: false
---

{{< katex >}}

## Overview

LCD displays control light transmission by applying voltage to liquid crystal molecules. The molecular alignment changes with voltage, modulating polarized light passage.

## Basic Structure

```
            Light Source
                 ↓
         ┌─────────────┐
         │  Polarizer  │ (0°)
         ├─────────────┤
         │ Glass + ITO │
         ├─────────────┤
         │   Liquid    │ ← Twist angle: 90°
         │  Crystals   │
         ├─────────────┤
         │ Glass + ITO │
         ├─────────────┤
         │  Analyzer   │ (90°)
         └─────────────┘
                 ↓
            Viewer
```

## Operating Principle

### No Voltage Applied (Bright State)

1. Light enters through polarizer (horizontal)
2. LC molecules twist light 90°
3. Light passes through analyzer (vertical)
4. **Result:** Light transmits → Bright pixel

### Voltage Applied (Dark State)

1. Electric field aligns LC molecules vertically
2. No twist occurs
3. Light blocked by analyzer
4. **Result:** Light blocked → Dark pixel

## Molecular Alignment

### Twisted Nematic (TN) Mode

Without voltage:
```
Top surface:     ─ ─ ─
                 ╲
                  ╲
                   ╲
Bottom surface:   │ │ │
```

With voltage:
```
                  │ │ │
                  │ │ │
                  │ │ │
                  │ │ │
```

## Voltage-Transmittance Relationship

The transmission follows:

$$
T = T_0 \sin^2\left(\frac{\pi}{2}\sqrt{1 + \left(\frac{V}{V_{th}}\right)^2}\right)
$$

For typical TN-LCD:

| Voltage | Transmission |
|---------|--------------|
| 0V | 100% (bright) |
| \\(V_{th}\\) | ~90% |
| \\(2V_{th}\\) | ~10% |
| \\(V_{sat}\\) | ~0% (dark) |

## Threshold Voltage

The voltage at which molecules begin to reorient:

$$
V_{th} = \pi \sqrt{\frac{K_{11}}{\epsilon_0 \Delta\epsilon}}
$$

Where:
- \\(K_{11}\\): Splay elastic constant
- \\(\Delta\epsilon\\): Dielectric anisotropy

## Gray Scale Control

Intermediate voltages create partial alignment:

| Voltage Level | Alignment | Brightness |
|---------------|-----------|------------|
| Low | Twisted | High |
| Medium | Partially aligned | Medium |
| High | Fully aligned | Low |

Modern LCDs use 8-bit control (256 levels per color).

## Response Time

### Rise Time (\\(\tau_{on}\\))

Voltage applied → molecules align:

$$
\tau_{on} = \frac{\gamma_1 d^2}{K(\pi^2 + V^2/V_{th}^2)}
$$

### Decay Time (\\(\tau_{off}\\))

Voltage removed → molecules relax:

$$
\tau_{off} = \frac{\gamma_1 d^2}{\pi^2 K}
$$

Where:
- \\(\gamma_1\\): Rotational viscosity
- \\(d\\): Cell gap
- \\(K\\): Elastic constant

## Key Design Considerations

### Cell Gap

- Smaller gap → Faster response
- Trade-off with manufacturing difficulty

### Alignment Layers

- Rubbed polyimide
- Determines pre-tilt angle
- Must only contact upper/lower plates

### Standard Twist Angle

- **90°:** Standard TN mode
- **180-270°:** Super-twisted nematic (STN)
- Adjusted by cell gap and material properties

## LC Contact Requirements

Liquid crystals must only touch metal surfaces (ITO) on upper and lower plates. Contact with side walls causes:
- Irregular twisting
- Light leakage
- Non-uniform display

## Viewing Angle

TN-LCDs have limited viewing angle:
- Brightness varies with angle
- Color shift at extreme angles

Solutions:
- IPS (In-Plane Switching)
- VA (Vertical Alignment)
- Optical compensation films
