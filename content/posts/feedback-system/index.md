---
title: "Feedback System"
date: 2024-08-15
description: "Feedback principles, stability analysis, LDO, and PLL"
categories: ["Circuits"]
tags: ["Analog Circuits", "Feedback", "PLL", "LDO"]
draft: false
---

{{< katex >}}

## Overview

Feedback is a fundamental concept in analog circuit design, enabling stable amplification, voltage regulation, and frequency synthesis.

## Negative Feedback Principle

```
Input (+) ────┐
              ├──→ [Amplifier] ──→ Output
Feedback (-) ─┘
       ↑                           |
       └───────── β ←──────────────┘
```

**Transfer Function:**

$$
A_{closed} = \frac{A_{open}}{1 + A_{open} \cdot \beta}
$$

Where:
- \(A_{open}\) = Open-loop gain
- \(\beta\) = Feedback factor

## Stability Analysis

### Oscillation Condition

If input differences oscillate between positive and negative values based on output, the system becomes an **oscillator**.

**Barkhausen Criteria:**
$$
|A \cdot \beta| = 1 \quad \text{and} \quad \angle(A \cdot \beta) = 0°
$$

### Frequency Response

**Gain Degradation:**
- Parasitic capacitance attenuates high frequencies
- Gain decreases by **20 dB/decade** per pole

### Stability Margins

| Parameter | Stable Range |
|-----------|--------------|
| Pole count | < 3 poles at unity gain |
| Phase margin | 45° - 60° |
| Gain margin | > 10 dB |

### Bandwidth

- **1st-order:** -3dB at first pole
- **2nd-order:** -3dB after first pole

## LDO (Low Drop-Out) Regulator

Linear voltage regulator using negative feedback.

```
    VIN ──→ [Pass Transistor] ──→ VOUT
                    ↑                |
              [Error Amp]            |
                    ↑                |
              VREF ─┴───── R1 ───────┤
                           |         |
                          R2         |
                           |         |
                          GND ←──────┘
```

**Output Voltage:**

$$
V_{OUT} = V_{REF} \times \left(1 + \frac{R_1}{R_2}\right)
$$

**Features:**
- Stable power supply
- Low dropout voltage
- Feedback maintains consistent output across load variations

## PLL (Phase-Locked Loop)

Essential for high-speed I/O and clock generation.

```
REF ──→ [Phase Detector] ──→ [Charge Pump] ──→ [Loop Filter] ──→ [VCO] ──→ OUT
              ↑                                                      |
              └────────────────── [Divider ÷N] ←─────────────────────┘
```

### Components

| Block | Function |
|-------|----------|
| Phase Detector | Compares REF and feedback phases |
| Charge Pump | Converts phase error to current |
| Loop Filter | Smooths control voltage |
| VCO | Voltage-controlled oscillator |
| Divider | Frequency division (÷N) |

### Output Frequency

$$
f_{OUT} = N \times f_{REF}
$$

**Note:** N must be integer multiples (2^n divisions of reference).

### Lock Process

1. Phase detector compares REF vs divided output
2. Charge pump adjusts VCO control voltage
3. Loop filter stabilizes control signal
4. VCO frequency adjusts until phase lock

## Design Considerations

1. **Loop stability** - Adequate phase margin
2. **Bandwidth** - Trade-off: speed vs noise
3. **Settling time** - Time to reach lock
4. **Jitter** - Minimize for clock applications
