---
title: "Pixel Structure and Circuit Design"
date: 2024-06-24
description: "Understanding pixel circuit configurations and storage capacitor designs in LCD"
categories: ["Circuits"]
tags: ["LCD", "TFT", "Display", "Pixel Circuit"]
draft: false
---

{{< katex >}}

## Overview

LCD pixel circuits must maintain voltage between refresh cycles. The storage capacitor configuration significantly impacts display performance.

## Basic Pixel Circuit

```
Gate Line (Gi) ──┬──[TFT]──┬── Data Line (Dj)
                 │         │
                ═╪═       ═╪═
                ═╪═ Cst   ═╪═ Clc
                ═╪═       ═╪═
                 │         │
              Common ──────┘
```

### Components

| Component | Function |
|-----------|----------|
| TFT | Switch (on/off control) |
| Clc | Liquid crystal capacitance |
| Cst | Storage capacitor |

## Storage Capacitor Configurations

### Configuration 1: Storage on Common (Cs on Com)

```
Gate (Gi) ────[TFT]───┬──── Data (Dj)
                      │
                     ═╪═ Clc
                     ═╪═
                      │
                     ═╪═ Cst
                     ═╪═
                      │
                  Common (Vcom)
```

**Characteristics:**
- Capacitor between pixel electrode and common line
- Simpler structure
- Independent of gate timing

### Configuration 2: Storage on Gate (Cs on Gate)

```
Gate (Gi) ────[TFT]───┬──── Data (Dj)
                      │
                     ═╪═ Clc
                     ═╪═
                      │
                     ═╪═ Cst
                     ═╪═
                      │
               Gate (Gi-1) ← Previous row
```

**Characteristics:**
- Capacitor connected to previous row's gate line
- More compact design
- Potential coupling effects

## Voltage Coupling Issue

### Problem with Cs on Gate

When data is written to row i:
1. Gate line Gi is high (TFT on)
2. Data voltage applied to pixel
3. Storage capacitor couples to Gi-1

This can cause voltage fluctuations:

$$
\Delta V_{pixel} = \frac{C_{st}}{C_{st} + C_{lc}} \cdot \Delta V_{gate}
$$

### Impact on Previous Row

The coupling may cause:
- Slight gate voltage change on row i-1
- Minimal TFT conduction if \\(V_{gs}\\) approaches threshold
- Potential charge leakage

### Why It's Usually Acceptable

1. **Timing window is short**
   - Gate pulse duration: ~15 μs
   - Coupling effect brief

2. **Voltage change is small**
   - Capacitive divider reduces effect
   - Typically < 0.1V change

3. **TFT threshold margin**
   - Gate off voltage is well below threshold
   - Small perturbation doesn't turn on TFT

## TFT Leakage Considerations

### Off-State Leakage

TFT gates cannot achieve perfect closure when \\(V_{ds}\\) exists:

$$
I_{off} = I_0 \cdot e^{(V_{gs} - V_{th})/S}
$$

Where S is subthreshold slope.

### Permissible Leakage

The acceptable leakage current relates to perceptible luminance changes:

$$
\Delta V = \frac{I_{leak} \cdot t_{frame}}{C_{total}}
$$

If \\(\Delta V\\) causes < 1% brightness change, it's imperceptible.

### Design Margins

| Parameter | Typical Value |
|-----------|---------------|
| Off-state leakage | < 1 pA |
| Frame time | 16.7 ms (60 Hz) |
| Storage capacitance | 0.3-0.5 pF |
| Acceptable ΔV | < 50 mV |

## Capacitance Requirements

### Total Pixel Capacitance

$$
C_{total} = C_{lc} + C_{st} + C_{parasitic}
$$

### Sizing Guidelines

$$
C_{st} \approx (2-3) \times C_{lc}
$$

Larger storage capacitor:
- Better voltage retention
- Reduced aperture ratio

### Trade-offs

| Larger Cst | Smaller Cst |
|------------|-------------|
| Better holding | More droop |
| Slower charging | Faster charging |
| Lower aperture | Higher aperture |

## Feedthrough Voltage

### Kickback Effect

When gate turns off:

$$
\Delta V_{pixel} = \frac{C_{gs}}{C_{gs} + C_{lc} + C_{st}} \cdot \Delta V_{gate}
$$

This shifts pixel voltage, requiring compensation.

### Compensation Methods

1. **Vcom adjustment:** Shift common voltage
2. **Data adjustment:** Pre-compensate data voltage
3. **Layout optimization:** Minimize gate-source overlap

## Advanced Pixel Designs

### Dual TFT

```
Gate ────[TFT1]──┬──[TFT2]──── Data
                 │
                 Pixel
```

Reduces kickback and leakage.

### Compensation Capacitor

Additional capacitor for feedthrough correction:

```
         ┌─── Cst ───┐
Gate ─[TFT]─┤         ├─ Data
         └─── Clc ───┘
         └─── Cc ────┘ Compensation
```

## Summary

| Configuration | Pros | Cons |
|---------------|------|------|
| Cs on Com | No coupling, simpler | More space needed |
| Cs on Gate | Compact, higher Cst | Potential coupling |

Design choice depends on:
- Display size and resolution
- Manufacturing process
- Performance requirements
