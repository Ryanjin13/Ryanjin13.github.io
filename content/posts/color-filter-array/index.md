---
title: "Color Filter Array in LCD Displays"
date: 2024-06-24
description: "Understanding color filter array technology in LCD displays"
categories: ["Circuits"]
tags: ["LCD", "Display", "Color Filter"]
draft: false
---

## Overview

Color filter arrays enable LCD displays to produce full-color images. Each pixel is divided into sub-pixels with red, green, and blue filters.

## Basic Structure

### Sub-pixel Arrangement

```
┌───┬───┬───┐ ┌───┬───┬───┐ ┌───┬───┬───┐
│ R │ G │ B │ │ R │ G │ B │ │ R │ G │ B │
└───┴───┴───┘ └───┴───┴───┘ └───┴───┴───┘
   Pixel 1       Pixel 2       Pixel 3
```

### Color Filter Layer

```
┌─────────────────────────────────────┐
│     RGB Color Filters + Black Matrix│
├─────────────────────────────────────┤
│         Overcoat Layer              │
├─────────────────────────────────────┤
│       Common Electrode (ITO)        │
├─────────────────────────────────────┤
│          Glass Substrate            │
└─────────────────────────────────────┘
```

## Filter Patterns

### RGB Stripe

Most common arrangement:

```
R G B R G B R G B
R G B R G B R G B
R G B R G B R G B
```

Good for text, vertical lines.

### RGB Delta (Triangle)

```
 R G B R G B R
G B R G B R G B
 R G B R G B R
```

Better for curved lines, photographic images.

### PenTile

Samsung AMOLED pattern:

```
R G R G R G
 B G B G B
R G R G R G
```

Fewer sub-pixels, reduced power.

## Color Filter Properties

### Spectral Characteristics

Each filter passes specific wavelengths:

| Filter | Peak Wavelength | Bandwidth |
|--------|-----------------|-----------|
| Red | ~620 nm | 580-700 nm |
| Green | ~530 nm | 490-570 nm |
| Blue | ~460 nm | 430-500 nm |

### Color Gamut

Filter selection affects color coverage:

| Standard | Coverage |
|----------|----------|
| sRGB | Standard monitors |
| Adobe RGB | Professional photo |
| DCI-P3 | HDR, wide gamut |
| Rec. 2020 | Future standard |

## Black Matrix

### Purpose

- Separates sub-pixels
- Blocks light leakage
- Improves contrast ratio

### Materials

| Material | Properties |
|----------|------------|
| Chromium | High opacity, reflective |
| Carbon-based | Low reflectivity |
| Resin + pigment | Cost-effective |

### Design

```
┌─────┬─────┬─────┐
│  R  │  G  │  B  │
├─────┼─────┼─────┤ ← Black matrix
│  R  │  G  │  B  │
└─────┴─────┴─────┘
  ↑     ↑     ↑
  Black matrix columns
```

## Manufacturing Process

### Photolithography Method

1. Deposit photoresist with pigment
2. Expose through mask
3. Develop to pattern
4. Repeat for each color
5. Add overcoat

### Inkjet Printing

1. Print color filter directly
2. Pattern defined by bank structure
3. Lower cost potential
4. Resolution limitations

## Alignment with TFT

The color filter must align precisely with TFT pixels:

```
        Color Filter Glass
┌───────────────────────────┐
│    R    │    G    │   B   │
└───────────────────────────┘
         ↕ Gap (3-5 μm)
┌───────────────────────────┐
│ Pixel 1 │ Pixel 2 │Pixel 3│
└───────────────────────────┘
         TFT Glass
```

Misalignment causes:
- Color mixing
- Reduced aperture ratio
- Mura defects

## Performance Factors

### Transmittance

$$
T_{total} = T_{polarizer} \times T_{LC} \times T_{filter}
$$

Color filters reduce brightness by ~30%.

### Contrast Ratio

$$
CR = \frac{L_{white}}{L_{black}}
$$

Black matrix quality directly affects contrast.

## Advanced Configurations

### Quantum Dot Enhancement

- Blue LED backlight
- QD film converts to R/G
- Wider color gamut

### RGBW Patterns

Add white sub-pixel:
- Better power efficiency
- Brighter highlights
- Used in some LG panels

## Quality Considerations

| Defect | Cause | Impact |
|--------|-------|--------|
| Color variation | Thickness non-uniformity | Color shift |
| Pinholes | Particle contamination | Light leakage |
| Pattern shift | Alignment error | Color mixing |
| Black matrix gaps | Process variation | Reduced contrast |
