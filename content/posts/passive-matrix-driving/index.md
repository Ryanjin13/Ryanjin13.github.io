---
title: "Passive Matrix Driving"
date: 2024-06-22
description: "Understanding passive matrix display driving technology"
categories: ["Circuits"]
tags: ["LCD", "Display", "Matrix Driving"]
draft: false
---

## Overview

Passive matrix driving is the most intuitive form of display circuitry, using simple row-column addressing without active switching elements at each pixel.

## Architecture

```
         Column Lines (Data)
         C1   C2   C3   C4
          │    │    │    │
    ┌─────┼────┼────┼────┼─────┐
R1 ─┤     ●    ●    ●    ●     │
    ├─────┼────┼────┼────┼─────┤
R2 ─┤     ●    ●    ●    ●     │  Row Lines
    ├─────┼────┼────┼────┼─────┤  (Scan)
R3 ─┤     ●    ●    ●    ●     │
    ├─────┼────┼────┼────┼─────┤
R4 ─┤     ●    ●    ●    ●     │
    └─────┴────┴────┴────┴─────┘
          Pixels at intersections
```

## Operating Principle

### Sequential Scanning

1. **Row Selection:** Activate one row at a time
2. **Column Data:** Apply voltage to all columns simultaneously
3. **Pixel Response:** Only pixels at selected row respond
4. **Repeat:** Move to next row, continue cycling

### Timing Diagram

```
Row 1:  ████____________________████
Row 2:  ____████________________████
Row 3:  ________████____________████
Row 4:  ____________████________████
        ← One Frame Period →
```

## Key Characteristics

### No Storage Capacitor

- Voltage not maintained between scans
- Light emission only during row selection
- Relies on persistence of vision

### PWM for Brightness

Pulse Width Modulation controls gray levels:

| Duty Cycle | Brightness |
|------------|------------|
| 100% | Maximum |
| 50% | Medium |
| 25% | Low |
| 0% | Off |

## Advantages

| Advantage | Description |
|-----------|-------------|
| Simple design | No transistors per pixel |
| Low cost | Fewer manufacturing steps |
| High aperture ratio | More light through pixel |
| Easy to manufacture | Simpler process |

## Disadvantages

| Disadvantage | Description |
|--------------|-------------|
| Limited resolution | Cross-talk increases with size |
| Slow response | Sequential nature |
| Low contrast | Voltage averaging |
| Flickering | At low refresh rates |

## Crosstalk Problem

When one pixel is addressed, neighboring pixels receive partial voltage:

```
        Selected Column
             ↓
Row OFF ─── ◐ ─── Partial voltage
Row ON  ─── ● ─── Full voltage
Row OFF ─── ◐ ─── Partial voltage
```

This limits practical display size.

## Persistence of Vision

The eye perceives continuous image if:
- Refresh rate > 60 Hz
- Frame time < 16.7 ms

Human vision integrates rapid sequential images into perceived static display.

## Applications

### LCD Displays

- Simple calculators
- Basic watches
- Small character displays
- Low-resolution graphics

### OLED/MicroLED

Passive matrix principles extended to:
- Small OLED displays
- Wearable devices
- Indicator panels

## Comparison with Active Matrix

| Aspect | Passive Matrix | Active Matrix |
|--------|----------------|---------------|
| Transistors/pixel | 0 | 1-2+ |
| Cost | Low | Higher |
| Resolution | Limited | High |
| Response time | Slow | Fast |
| Contrast | Low | High |
| Power | Can be high | Efficient |

## Circuit Implementation

### Row Driver

Sequentially activates each row with scan pulse.

### Column Driver

Applies data voltage pattern to all columns during row selection.

### Timing Control

Synchronizes row selection with column data.

## Evolution

```
Passive Matrix
      ↓
Super Twisted Nematic (STN)
      ↓
Dual Scan STN
      ↓
Active Matrix (TFT)
```

The need for higher resolution and faster response led to active matrix development.
