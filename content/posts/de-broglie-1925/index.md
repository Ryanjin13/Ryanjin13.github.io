---
title: "1925 de Broglie - Matter Waves"
date: 2024-06-22
description: "Louis de Broglie's hypothesis of wave-particle duality for matter"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Physics History", "de Broglie", "Wave-Particle Duality"]
draft: false
---

{{< katex >}}

## Overview

In 1924, Louis de Broglie proposed in his PhD thesis that all matter exhibits wave-like properties. This revolutionary idea unified the wave-particle duality of light with the behavior of material particles.

## The Hypothesis

If light (waves) can behave like particles (photons), then particles might behave like waves!

### de Broglie Wavelength

$$
\lambda = \frac{h}{p} = \frac{h}{mv}
$$

Where:
- \\(\lambda\\): de Broglie wavelength
- \\(h\\): Planck's constant
- \\(p\\): Momentum
- \\(m\\): Mass
- \\(v\\): Velocity

### de Broglie Frequency

$$
\nu = \frac{E}{h}
$$

## Reasoning

### From Photons

For photons:
- Energy: \\(E = h\nu\\)
- Momentum: \\(p = E/c = h\nu/c = h/\lambda\\)

### Extended to Matter

de Broglie proposed the same relation holds for particles:

$$
p = \frac{h}{\lambda} \implies \lambda = \frac{h}{p}
$$

## Wave-Particle Relations

| Particle Property | Wave Property | Relation |
|-------------------|---------------|----------|
| Energy \\(E\\) | Frequency \\(\nu\\) | \\(E = h\nu\\) |
| Momentum \\(p\\) | Wavelength \\(\lambda\\) | \\(p = h/\lambda\\) |

## Example Calculations

### Electron at 100 eV

$$
v = \sqrt{\frac{2E}{m}} = \sqrt{\frac{2 \times 100 \times 1.6 \times 10^{-19}}{9.11 \times 10^{-31}}} \approx 5.9 \times 10^6 \text{ m/s}
$$

$$
\lambda = \frac{h}{mv} = \frac{6.63 \times 10^{-34}}{9.11 \times 10^{-31} \times 5.9 \times 10^6} \approx 0.12 \text{ nm}
$$

Comparable to X-ray wavelengths!

### Baseball (0.15 kg at 40 m/s)

$$
\lambda = \frac{6.63 \times 10^{-34}}{0.15 \times 40} \approx 10^{-34} \text{ m}
$$

Far too small to detect—explains why we don't see quantum effects in everyday objects.

## Bohr Model Connection

de Broglie waves explain Bohr's quantization condition:

### Standing Wave Requirement

Electron wave must form standing wave around orbit:

$$
2\pi r = n\lambda
$$

### Substituting de Broglie Wavelength

$$
2\pi r = n \frac{h}{mv}
$$

$$
mvr = n\frac{h}{2\pi} = n\hbar
$$

This is exactly Bohr's quantization condition!

```
n = 3:  ●───●───●  (3 wavelengths around orbit)
         \     /
          ●───●
```

## Experimental Confirmation

### Davisson-Germer Experiment (1927)

- Electrons scattered from nickel crystal
- Diffraction pattern observed
- Confirmed wave nature of electrons

Measured wavelength matched de Broglie prediction.

### Thomson Electron Diffraction (1927)

- Electrons through thin metal foil
- Ring diffraction pattern
- Like X-ray diffraction

G.P. Thomson (son of J.J. Thomson who discovered the electron particle!) showed its wave nature.

## Wave Properties of Matter

### Phase Velocity

$$
v_p = \frac{\omega}{k} = \frac{E}{p} = \frac{c^2}{v}
$$

Greater than \\(c\\) for massive particles! (Not physical velocity)

### Group Velocity

$$
v_g = \frac{d\omega}{dk} = \frac{dE}{dp} = v
$$

Equals particle velocity—carries energy and information.

### Wave Packet

Particle localized by superposition of waves:

$$
\Psi(x, t) = \int A(k) e^{i(kx - \omega t)} dk
$$

## Implications

### 1. Electron Microscopy

- Electron wavelength < visible light
- Higher resolution possible
- TEM, SEM, STEM

### 2. Quantum Tunneling

- Wave can penetrate barriers
- Essential for many phenomena

### 3. Semiconductor Devices

- Electron wave effects in small structures
- Quantum wells, wires, dots

### 4. Uncertainty Principle

Wave packets have:
$$
\Delta x \cdot \Delta k \geq \frac{1}{2}
$$

Since \\(p = \hbar k\\):
$$
\Delta x \cdot \Delta p \geq \frac{\hbar}{2}
$$

## Historical Context

de Broglie's thesis was initially met with skepticism. Einstein supported it enthusiastically, saying:

> "He has lifted a corner of the great veil."

## Nobel Prize

Louis de Broglie received the Nobel Prize in Physics in 1929:

> "For his discovery of the wave nature of electrons"

His work laid the foundation for wave mechanics and Schrödinger's equation.
