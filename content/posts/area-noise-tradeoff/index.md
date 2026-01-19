---
title: "Area vs Noise Trade-off Analysis"
date: 2024-08-14
description: "Understanding the trade-off between circuit area and noise performance"
categories: ["Circuits"]
tags: ["Noise", "Area", "Trade-offs", "Analog Design"]
draft: false
---

{{< katex >}}

## Overview

In analog circuit design, achieving low noise typically requires larger device sizes and more area. This analysis explores the fundamental relationship between circuit area and noise performance.

## Noise Fundamentals

### Thermal Noise

Random motion of charge carriers:

$$
\overline{v_n^2} = 4kTR\Delta f
$$

Or in spectral density:

$$
S_v(f) = 4kTR \quad [V^2/Hz]
$$

Where:
- \\(k\\): Boltzmann constant (\\(1.38 \times 10^{-23}\\) J/K)
- \\(T\\): Temperature (K)
- \\(R\\): Resistance (Ω)
- \\(\Delta f\\): Bandwidth (Hz)

### MOSFET Thermal Noise

Channel thermal noise:

$$
\overline{i_n^2} = 4kT\gamma g_m \Delta f
$$

Where \\(\gamma \approx 2/3\\) for long-channel devices.

Input-referred:

$$
\overline{v_{n,in}^2} = \frac{4kT\gamma}{g_m}\Delta f
$$

### Flicker (1/f) Noise

$$
\overline{v_n^2} = \frac{K_f}{C_{ox}WL} \cdot \frac{1}{f} \Delta f
$$

Where \\(K_f\\) is a process-dependent constant.

## The Area-Noise Trade-off

### Why Larger Area = Lower Noise

**For Thermal Noise:**

$$
g_m = \mu C_{ox}\frac{W}{L}(V_{GS} - V_{th})
$$

Larger \\(W\\) → larger \\(g_m\\) → lower input-referred noise:

$$
\overline{v_{n,in}^2} \propto \frac{1}{g_m} \propto \frac{L}{W}
$$

**For Flicker Noise:**

$$
\overline{v_n^2} \propto \frac{1}{WL}
$$

Larger area directly reduces 1/f noise.

### Quantitative Relationship

| Parameter | 2× Area | 4× Area |
|-----------|---------|---------|
| Thermal noise power | 0.5× | 0.25× |
| Flicker noise power | 0.5× | 0.25× |
| Noise voltage | 0.71× | 0.5× |

To halve noise voltage, quadruple the area.

## Noise Sources in Circuits

### Resistor Noise

```
    ┌───[R]───┐
    │    ~    │  ← Thermal noise source
    └─────────┘
```

$$
\overline{v_n^2} = 4kTR
$$

**Trade-off**: Lower R = less noise but more power or different gain.

### MOSFET Noise Model

```
         Drain
           │
    ┌──────┼──────┐
    │      │      │
    │   ┌──┴──┐   │
    │   │ i_n │   │  Channel noise
    │   └──┬──┘   │
    │      │      │
Gate──────[gm·vgs]─────Source
    │             │
    │    ┌───┐    │
    │    │v_n│    │  Gate noise (1/f + thermal)
    │    └─┬─┘    │
    └──────┴──────┘
```

### Input-Referred Noise

For an amplifier with multiple noise sources:

$$
\overline{v_{n,total}^2} = \overline{v_{n1}^2} + \frac{\overline{v_{n2}^2}}{A_1^2} + \frac{\overline{v_{n3}^2}}{(A_1 A_2)^2} + ...
$$

**Key insight**: First stage dominates → make it large and low-noise.

## Design Strategies

### Sizing for Low Noise

**Input Transistor:**

$$
W_{opt} = \sqrt{\frac{K_f}{4kT\gamma} \cdot \frac{1}{f_{corner}}}
$$

Where \\(f_{corner}\\) is the 1/f corner frequency.

**Practical Rule:**
- Large W for low thermal noise
- Large WL for low 1/f noise

### Current Density Optimization

For minimum noise figure:

$$
g_m = \sqrt{\omega^2 C_{gs}^2 + \omega^2 C_{gd}^2}
$$

Optimal bias current:

$$
I_{D,opt} \propto \sqrt{f}
$$

### Noise-Efficient Design

Noise efficiency factor (NEF):

$$
NEF = V_{n,rms}\sqrt{\frac{2I_{total}}{\pi \cdot V_T \cdot 4kT \cdot BW}}
$$

Lower NEF = more noise-efficient use of power/area.

## Topology Considerations

### Single-Ended vs Differential

| Aspect | Single-Ended | Differential |
|--------|--------------|--------------|
| Area | 1× | 2× |
| Noise | Baseline | \\(\sqrt{2}\\)× worse |
| CMRR | Poor | Excellent |
| PSRR | Poor | Good |

Differential adds noise but improves other metrics.

### Cascaded Stages

```
       ┌──────┐     ┌──────┐     ┌──────┐
Vin ──▶│ A1   │────▶│ A2   │────▶│ A3   │──▶ Vout
       │ (big)│     │      │     │      │
       └──────┘     └──────┘     └──────┘
         ↑
    Make this large for low noise
```

**Noise contribution of stage n:**

$$
\text{Contribution}_n = \frac{\overline{v_n^2}}{\prod_{i=1}^{n-1} A_i^2}
$$

## Area Optimization Techniques

### 1. Use Minimum Size Where Noise Isn't Critical

| Location | Size Strategy |
|----------|---------------|
| Input stage | Large (noise-critical) |
| Current mirrors | Medium |
| Load devices | Minimum |
| Digital circuits | Minimum |

### 2. Chopper Stabilization

Modulate signal above 1/f corner:

```
     ┌───────┐      ┌───────┐      ┌───────┐
Vin─▶│ Chop  │─────▶│ Amp   │─────▶│ Chop  │─▶Vout
     │ (fc)  │      │       │      │ (fc)  │
     └───────┘      └───────┘      └───────┘
```

**Benefit**: Eliminates 1/f noise without larger area.

### 3. Correlated Double Sampling (CDS)

For discrete-time systems:

$$
V_{out} = (V_{sig} + V_n(t_2)) - V_n(t_1)
$$

If \\(t_2 - t_1\\) is small, noise cancels.

### 4. Averaging

Parallel devices reduce uncorrelated noise:

$$
\overline{v_n^2}_{parallel} = \frac{\overline{v_n^2}}{N}
$$

**Cost**: N× area, N× power.

## Practical Design Flow

### Step 1: Determine Noise Budget

$$
SNR_{required} = \frac{V_{signal,rms}}{V_{noise,rms}}
$$

### Step 2: Allocate Noise to Stages

$$
V_{n,total}^2 = V_{n,stage1}^2 + V_{n,stage2}^2 + ...
$$

Typically: Stage 1 gets 70% of noise budget.

### Step 3: Size for Noise Target

$$
W = \frac{4kT\gamma}{g_m \cdot \overline{v_{n,target}^2} / \Delta f}
$$

### Step 4: Verify Area Constraints

If area too large:
- Reduce bandwidth
- Use chopping/CDS
- Relax noise specification

## Summary

Key insights on area-noise trade-off:
1. **Thermal noise**: \\(\propto 1/\sqrt{W}\\)
2. **Flicker noise**: \\(\propto 1/\sqrt{WL}\\)
3. **First stage**: Dominates total noise
4. **Chopping**: Eliminates 1/f without area increase
5. **Averaging**: \\(N\\)× area for \\(N\\)× noise power reduction
6. **Trade-off**: To halve noise voltage, quadruple area

