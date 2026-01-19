---
title: "1901 Planck - Birth of Quantum Theory"
date: 2024-06-22
description: "Max Planck's revolutionary solution to blackbody radiation"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Physics History", "Planck"]
draft: false
---

{{< katex >}}

## Overview

In 1900-1901, Max Planck introduced the concept of energy quantization to solve the blackbody radiation problem. This marked the birth of quantum theory.

## The Problem: Blackbody Radiation

A blackbody is an idealized object that absorbs all electromagnetic radiation. When heated, it emits radiation with a characteristic spectrum.

### Classical Prediction

The Rayleigh-Jeans law predicted:

$$
u(\nu, T) = \frac{8\pi\nu^2}{c^3} k_B T
$$

This leads to the **ultraviolet catastrophe**: infinite energy at high frequencies.

### Experimental Observation

Real blackbody spectrum:
- Rises with frequency at low \\(\nu\\)
- Peaks at intermediate frequency
- Decreases to zero at high \\(\nu\\)

## Planck's Revolutionary Solution

### The Quantum Hypothesis

Planck proposed that oscillators in the cavity walls can only have discrete energies:

$$
E_n = nh\nu
$$

Where:
- \\(n = 0, 1, 2, 3, ...\\) (integer)
- \\(h\\): Planck's constant
- \\(\nu\\): Frequency

### Planck's Constant

$$
h = 6.626 \times 10^{-34} \text{ J·s}
$$

This fundamental constant relates energy to frequency.

### Planck's Radiation Law

$$
u(\nu, T) = \frac{8\pi h\nu^3}{c^3} \cdot \frac{1}{e^{h\nu/k_B T} - 1}
$$

This formula perfectly matches experimental observations.

## Derivation Outline

### Average Energy per Mode

Classical (Boltzmann):
$$
\langle E \rangle = k_B T
$$

Quantum (Planck):
$$
\langle E \rangle = \frac{h\nu}{e^{h\nu/k_B T} - 1}
$$

### Limiting Cases

**Low frequency** (\\(h\nu \ll k_B T\\)):
$$
\langle E \rangle \approx k_B T
$$
Recovers classical result.

**High frequency** (\\(h\nu \gg k_B T\\)):
$$
\langle E \rangle \approx h\nu \cdot e^{-h\nu/k_B T} \rightarrow 0
$$
Prevents ultraviolet catastrophe.

## Wien's Displacement Law

From Planck's law, the peak wavelength:

$$
\lambda_{max} T = b = 2.898 \times 10^{-3} \text{ m·K}
$$

## Stefan-Boltzmann Law

Total radiated power:

$$
P = \sigma T^4
$$

Where:
$$
\sigma = \frac{2\pi^5 k_B^4}{15 c^2 h^3} = 5.67 \times 10^{-8} \text{ W/(m²·K⁴)}
$$

## Key Concepts Introduced

| Concept | Significance |
|---------|--------------|
| Energy quantization | Energy comes in discrete packets |
| Planck's constant | Fundamental quantum of action |
| Quantum of energy | \\(E = h\nu\\) |

## Why Planck's Work Was Revolutionary

1. **Broke continuous energy assumption**
   - Classical physics: Any energy value allowed
   - Quantum: Only specific values permitted

2. **Introduced fundamental constant**
   - \\(h\\) appears in all quantum phenomena
   - Links wave (frequency) to particle (energy)

3. **Solved real problem**
   - Matched experimental data precisely
   - Avoided infinity in theory

## Historical Context

Planck initially viewed quantization as a mathematical trick, not physical reality. He spent years trying to derive his formula classically.

Einstein (1905) took the quantum seriously with the photoelectric effect, showing light itself is quantized.

## Legacy

Planck's quantum hypothesis led to:
- Quantum mechanics
- Atomic structure understanding
- Modern physics and chemistry
- Semiconductors and lasers

Max Planck received the Nobel Prize in Physics in 1918 for his discovery of energy quanta.
