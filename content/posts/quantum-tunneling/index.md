---
title: "Quantum Tunneling"
date: 2024-06-21
description: "Quantum mechanical phenomenon of barrier penetration"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Tunneling", "Barrier Penetration"]
draft: false
---

{{< katex >}}

## Overview

Quantum tunneling is a phenomenon where particles can pass through potential barriers that would be classically forbidden. This has no classical analog and is purely quantum mechanical.

## Classical vs Quantum

### Classical Mechanics

A particle with energy \\(E\\) encountering barrier \\(V_0 > E\\):
- **Result:** Complete reflection
- **Probability of transmission:** 0

### Quantum Mechanics

The wave function can penetrate the barrier:
- **Result:** Finite probability of transmission
- **Probability:** Non-zero (depends on barrier properties)

## Mathematical Description

### Setup

Consider a rectangular barrier:

$$
V(x) = \begin{cases}
0 & x < 0 \\
V_0 & 0 \leq x \leq a \\
0 & x > a
\end{cases}
$$

### Wave Functions

**Region I (x < 0):**
$$
\psi_I = Ae^{ikx} + Be^{-ikx}
$$

**Region II (0 ≤ x ≤ a):**
$$
\psi_{II} = Ce^{\kappa x} + De^{-\kappa x}
$$

**Region III (x > a):**
$$
\psi_{III} = Fe^{ikx}
$$

Where:
$$
k = \frac{\sqrt{2mE}}{\hbar}, \quad \kappa = \frac{\sqrt{2m(V_0 - E)}}{\hbar}
$$

### Transmission Coefficient

For thick barriers (\\(\kappa a \gg 1\\)):

$$
T \approx 16\frac{E}{V_0}\left(1 - \frac{E}{V_0}\right)e^{-2\kappa a}
$$

General form:

$$
T = \frac{1}{1 + \frac{V_0^2 \sinh^2(\kappa a)}{4E(V_0 - E)}}
$$

### Key Observations

| Factor | Effect on Tunneling |
|--------|---------------------|
| Barrier width ↑ | Transmission ↓ exponentially |
| Barrier height ↑ | Transmission ↓ |
| Particle mass ↑ | Transmission ↓ |
| Particle energy ↑ | Transmission ↑ |

## Decay Length

The wave function decays inside barrier:

$$
|\psi|^2 \propto e^{-2\kappa x}
$$

Decay length:

$$
\delta = \frac{1}{2\kappa} = \frac{\hbar}{2\sqrt{2m(V_0-E)}}
$$

## WKB Approximation

For arbitrary barrier shapes:

$$
T \approx e^{-2\gamma}
$$

Where:

$$
\gamma = \int_{x_1}^{x_2} \frac{\sqrt{2m(V(x) - E)}}{\hbar} dx
$$

Integration is over the classically forbidden region.

## Applications

### 1. Scanning Tunneling Microscope (STM)

Electrons tunnel between tip and surface:

$$
I \propto e^{-2\kappa d}
$$

- Atomic resolution imaging
- Surface structure analysis

### 2. Alpha Decay

Alpha particle tunnels out of nucleus:

$$
\lambda = f \cdot T
$$

Where \\(f\\) is attempt frequency and \\(T\\) is tunneling probability.

### 3. Tunnel Diodes

Electrons tunnel through thin barrier:
- Negative resistance region
- High-speed switching
- Microwave applications

### 4. Josephson Junction

Cooper pairs tunnel between superconductors:

$$
I = I_c \sin(\phi)
$$

- SQUID magnetometers
- Quantum computing (qubits)

### 5. Nuclear Fusion

Protons overcome Coulomb barrier:
- Powers stars
- Enables fusion reactors

## Resonant Tunneling

For double barriers, resonance occurs at specific energies:

$$
T = 1 \text{ when } E = E_n \text{ (resonance)}
$$

Used in:
- Resonant tunneling diodes (RTDs)
- Quantum cascade lasers

## Time Aspects

### Tunneling Time

How long does tunneling take?

Various definitions:
- **Phase time:** \\(\tau_\phi = \hbar \frac{\partial \phi}{\partial E}\\)
- **Dwell time:** Time spent in barrier
- **Büttiker-Landauer time**

Still debated in physics community.
