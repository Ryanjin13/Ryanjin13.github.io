---
title: "Structure of TFT-LCD"
date: 2024-06-24
description: "Understanding the layered architecture of TFT-LCD displays"
categories: ["Circuits"]
tags: ["LCD", "TFT", "Display", "Electronics"]
draft: false
---

## Overview

TFT-LCD (Thin-Film Transistor Liquid Crystal Display) is the dominant display technology. Understanding its layered architecture is essential for display engineering.

## Display Driver IC (DDI)

The DDI controls the entire display:

```
┌─────────────────────────────────────┐
│          Display Driver IC          │
├──────────┬──────────┬───────────────┤
│  Timing  │   Data   │    Power      │
│ Controller│ Driver  │  Management   │
└──────────┴──────────┴───────────────┘
        ↓         ↓           ↓
    Gate Lines  Data Lines   Voltages
```

### Key Functions

1. **Timing Controller:** Generates sync signals
2. **Data Driver:** Converts digital to analog voltages
3. **Gate Driver:** Sequential row activation
4. **Power Management:** Voltage regulation

## Layered Architecture

### Physical Stack

```
        ↓ Light from backlight
┌─────────────────────────────────────┐
│         Rear Polarizer              │
├─────────────────────────────────────┤
│      TFT Glass Substrate            │
│  ┌─────────────────────────────┐    │
│  │  TFT Array + Storage Cap    │    │ ← Dense circuits
│  └─────────────────────────────┘    │
├─────────────────────────────────────┤
│         Liquid Crystal              │
├─────────────────────────────────────┤
│      Color Filter Substrate         │
│  ┌─────────────────────────────┐    │
│  │    Common Electrode (GND)   │    │ ← Constant voltage
│  └─────────────────────────────┘    │
├─────────────────────────────────────┤
│         Front Polarizer             │
└─────────────────────────────────────┘
        ↓ Light to viewer
```

### Lower Section (TFT Array)

Dense circuit components:
- Thin-film transistors
- Storage capacitors
- Data and gate lines
- Pixel electrodes

### Upper Section

Common electrode:
- Applies constant ground voltage
- Uniform across display
- Simpler structure

## Pixel Structure

### Basic Pixel Circuit

```
Gate Line ──┬──[TFT]──┬── Data Line
            │         │
           ═╪═       ═╪═
           ═╪═ Cst   ═╪═ Clc
           ═╪═       ═╪═
            │         │
         Common ──────┘
```

### Aperture Ratio

The aperture ratio significantly depends on capacitor area:

$$
\text{Aperture Ratio} = \frac{\text{Light-transmitting area}}{\text{Total pixel area}}
$$

| Component | Effect on Aperture |
|-----------|-------------------|
| TFT | Reduces (opaque) |
| Storage capacitor | Reduces (opaque) |
| Bus lines | Reduces (metal) |
| Pixel electrode | Increases (transparent) |

### Trade-off

Larger capacitor:
- Better voltage holding
- Reduced aperture ratio
- Lower brightness

Design optimization balances these factors.

## Storage Capacitor Configurations

### Type 1: Storage on Common (Cs on Com)

Capacitor formed between:
- Pixel electrode
- Common line

Simple structure, good aperture ratio.

### Type 2: Storage on Gate (Cs on Gate)

Capacitor formed between:
- Pixel electrode
- Previous row's gate line

Higher capacitance possible, more compact.

### Circuit Variations

Different manufacturers use various configurations:
- Single capacitor
- Dual capacitor
- Hybrid designs

Each optimizes for different priorities.

## Layer Details

### TFT Glass Substrate

- Thin-film transistor fabrication
- a-Si, LTPS, or IGZO technology
- Multiple metal and insulator layers

### Liquid Crystal Layer

- Aligned by rubbed polyimide
- Gap controlled by spacers
- Determines response time

### Color Filter Substrate

- RGB sub-pixel filters
- Black matrix for contrast
- Common electrode layer

## Manufacturing Considerations

### Process Steps

1. TFT array fabrication
2. Color filter fabrication
3. Cell assembly
4. Liquid crystal filling
5. Module assembly

### Yield Factors

| Factor | Impact |
|--------|--------|
| Particle defects | Dead pixels |
| Pattern alignment | Mura defects |
| Rubbing uniformity | Color shift |
| Gap uniformity | Brightness variation |

## Performance Metrics

| Metric | Typical Value |
|--------|---------------|
| Resolution | 1920×1080 to 4K+ |
| Pixel pitch | 50-300 μm |
| Aperture ratio | 40-60% |
| Response time | 5-15 ms |
| Contrast ratio | 1000:1+ |
