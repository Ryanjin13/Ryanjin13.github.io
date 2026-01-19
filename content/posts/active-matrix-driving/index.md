---
title: "Active Matrix Driving"
date: 2024-06-22
description: "How active matrix technology enables modern high-resolution displays"
categories: ["Circuits"]
tags: ["LCD", "Display", "TFT", "Matrix Driving"]
draft: false
---

{{< katex >}}

## Overview

Active matrix driving uses thin-film transistors (TFTs) at each pixel to maintain voltage between refresh cycles. This enables higher resolution, faster response, and better image quality than passive matrix.

## Architecture

```
              Data Lines
         D1   D2   D3   D4
          │    │    │    │
    ┌─────┼────┼────┼────┼─────┐
G1 ─┤    ⊏●   ⊏●   ⊏●   ⊏●    │
    ├─────┼────┼────┼────┼─────┤
G2 ─┤    ⊏●   ⊏●   ⊏●   ⊏●    │  Gate Lines
    ├─────┼────┼────┼────┼─────┤
G3 ─┤    ⊏●   ⊏●   ⊏●   ⊏●    │
    ├─────┼────┼────┼────┼─────┤
G4 ─┤    ⊏●   ⊏●   ⊏●   ⊏●    │
    └─────┴────┴────┴────┴─────┘
         ⊏ = TFT, ● = Pixel
```

## Pixel Circuit

### Basic TFT-LCD Pixel

```
Gate Line ──┬──[TFT]──┬── Data Line
            │         │
           ═╪═       ═╪═
           ═╪═ Cst   ═╪═ Clc
           ═╪═       ═╪═
            │         │
         Common ──────┘
```

Components:
- **TFT:** Thin-film transistor (switch)
- **Clc:** Liquid crystal capacitance
- **Cst:** Storage capacitor

## Operating Principle

### Write Phase

1. Gate line activates TFT
2. Data voltage charges pixel capacitor
3. TFT turns off, moves to next row

### Hold Phase

When scan line closes and moves to next row:
- Storage capacitor maintains voltage
- Electric field persists across liquid crystal
- Image remains stable until next refresh

### Voltage Retention

$$
V_{pixel}(t) = V_{data} \cdot e^{-t/\tau}
$$

Where \\(\tau = R_{TFT(off)} \cdot C_{total}\\)

High TFT off-resistance ensures minimal voltage decay.

## Why This Matters

### Human Perception

Since data updates occur discretely row by row:
- Without storage: flickering image
- With storage: stable, static appearance

The capacitor bridges the gap between discrete updates and continuous perception.

### Resolution Scaling

As displays increase in resolution:
- More rows to scan
- Less time per row
- Storage becomes critical

## TFT Types

| Type | Material | Mobility | Application |
|------|----------|----------|-------------|
| a-Si | Amorphous Si | Low | Standard LCD |
| LTPS | Low-temp poly-Si | High | Mobile, OLED |
| IGZO | Oxide | Medium-High | High-res, large |

## Comparison with Passive Matrix

| Aspect | Passive | Active |
|--------|---------|--------|
| Voltage holding | None | Capacitor |
| Crosstalk | Significant | Minimal |
| Resolution limit | ~256 rows | Unlimited |
| Contrast ratio | 10:1 | 1000:1+ |
| Response time | Slow | Fast |

## Timing Parameters

### Frame Period

For 60 Hz display:
$$
T_{frame} = \frac{1}{60} = 16.67 \text{ ms}
$$

### Line Time

For 1080 rows:
$$
T_{line} = \frac{T_{frame}}{1080} \approx 15.4 \text{ μs}
$$

## Storage Capacitor Design

### Purpose

1. Increase total capacitance
2. Reduce voltage droop
3. Stabilize pixel voltage

### Sizing

$$
C_{st} \approx C_{lc} \times (2 \sim 3)
$$

Trade-off:
- Larger Cst → Better holding, slower charging
- Smaller Cst → Faster charging, more droop

## Connection to DRAM

The same principle applies to Dynamic RAM:
- TFT ≈ Access transistor
- Storage capacitor ≈ Memory cell
- Refresh needed ≈ Periodic data refresh

```
DRAM Cell:          TFT-LCD Pixel:
    │                    │
  ──┼── Word Line     ──┼── Gate Line
    │                    │
  [Tr]                 [TFT]
    │                    │
  ═╪═ Capacitor       ═╪═ Cst + Clc
  ═╪═                 ═╪═
    │                    │
    ─ Bit Line          ─ Data Line
```

## Advanced Pixel Circuits

### OLED Active Matrix (AMOLED)

Additional transistors for current control:

```
2T1C Structure:
- T1: Switching TFT
- T2: Driving TFT
- C1: Storage capacitor
```

### Compensation Circuits

Address TFT variation:
- Current sensing
- Voltage compensation
- Multiple TFTs per pixel (4T, 6T, etc.)
