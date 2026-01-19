---
title: "Structure of LCD Backlight"
date: 2024-06-24
description: "Understanding backlight unit components and design in LCD displays"
categories: ["Circuits"]
tags: ["LCD", "Display", "Backlight", "LED"]
draft: false
---

## Overview

The backlight unit (BLU) provides illumination for LCD panels, which cannot emit light themselves. Modern displays primarily use LED backlights.

## Backlight Types

### Edge-lit LED

```
        LCD Panel
┌─────────────────────────┐
│                         │
│    Light Guide Plate    │ ← Light spreads across
│                         │
└─────────────────────────┘
LED ▶ ▶ ▶ ▶ ▶ ▶ ▶ ▶ ▶ ▶ LED
      Edge-mounted LEDs
```

**Advantages:**
- Thin profile
- Lower cost
- Good for small-medium displays

**Disadvantages:**
- Limited local dimming
- Potential edge hotspots

### Direct-lit LED

```
        LCD Panel
┌─────────────────────────┐
│    Diffuser Sheets      │
├─────────────────────────┤
│  ●  ●  ●  ●  ●  ●  ●   │ ← LED array
│  ●  ●  ●  ●  ●  ●  ●   │
│  ●  ●  ●  ●  ●  ●  ●   │
└─────────────────────────┘
```

**Advantages:**
- Better uniformity
- Local dimming possible (Full-array)
- Higher brightness

**Disadvantages:**
- Thicker design
- Higher power consumption
- More expensive

## Component Stack

### Edge-lit Assembly

```
From bottom to top:
┌─────────────────────────┐
│      Reflector          │ ← Recycles light
├─────────────────────────┤
│   Light Guide Plate     │ ← Distributes light
├─────────────────────────┤
│    Diffuser Sheet       │ ← Scatters light
├─────────────────────────┤
│   Prism Sheets (2x)     │ ← Redirects light
├─────────────────────────┤
│   Brightness Film       │ ← Enhances brightness
└─────────────────────────┘
        ↓ To LCD Panel
```

## Key Components

### Light Guide Plate (LGP)

Distributes edge light across panel area.

**Design features:**
- Micro-patterns or dots
- Density varies with distance from LEDs
- PMMA or PC material

```
LED → [Dense dots | Medium | Sparse dots] ← LED
      Near edge         Far from edge
```

### Diffuser Sheet

- Homogenizes light distribution
- Reduces hotspots
- Multiple sheets may be used

### Prism Sheets (BEF)

Brightness Enhancement Film redirects light:

```
     Viewing angle
         ↑
        /│\
       / │ \
      /  │  \  Prism redirects
     ────────── light forward
```

Crossed prisms for 2D enhancement.

### Reflector

- Recycles backward-scattered light
- White or silver surface
- Improves efficiency

## LED Light Sources

### White LED Types

| Type | Method | Color Quality |
|------|--------|---------------|
| Blue + YAG phosphor | Blue LED + yellow phosphor | Standard |
| Blue + RG phosphor | Blue LED + red/green | Better gamut |
| RGB LED | Separate R, G, B LEDs | Best gamut |

### Quantum Dot Enhancement

```
Blue LED → [QD Film] → White light (enhanced R/G)
```

Benefits:
- Wider color gamut
- Better efficiency than RGB LED
- Used in "QLED" displays

## Local Dimming

### Full-array Local Dimming (FALD)

```
┌───┬───┬───┬───┐
│ ● │ ● │ ● │ ● │  Zone 1-4
├───┼───┼───┼───┤
│ ● │ ● │ ● │ ● │  Zone 5-8
├───┼───┼───┼───┤
│ ● │ ● │ ● │ ● │  Zone 9-12
└───┴───┴───┴───┘
Each zone independently dimmable
```

**Benefits:**
- Improved contrast ratio
- Deeper blacks
- HDR capability

**Challenges:**
- Blooming around bright objects
- More complex control
- Higher cost

### Edge-lit Dimming

Limited zones along edges:
- 8-16 zones typical
- Less effective than FALD
- Visible artifacts possible

## Performance Metrics

| Metric | Description | Typical Value |
|--------|-------------|---------------|
| Luminance | Brightness | 300-1000+ nits |
| Uniformity | Evenness | >80% |
| Efficiency | Light output/power | 80-150 lm/W |
| Color temp | White point | 6500K (D65) |

## Power Consumption

Backlight is major power consumer:

| Display State | Backlight Power |
|---------------|-----------------|
| Maximum brightness | 100% |
| Typical use | 40-60% |
| Dark scene (with local dimming) | 10-30% |

## Mini-LED Technology

Next generation backlighting:

| Feature | Standard LED | Mini-LED |
|---------|-------------|----------|
| LED size | 300+ μm | 100-300 μm |
| Zone count | 10-500 | 500-2000+ |
| Contrast | Good | Excellent |
| Blooming | Noticeable | Minimal |
| Thickness | Standard | Can be thin |

## Future: Micro-LED

Direct emission display:
- No backlight needed
- Each pixel is an LED
- Ultimate contrast and efficiency
