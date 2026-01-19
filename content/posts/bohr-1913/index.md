---
title: "1913 Bohr - Atomic Model"
date: 2024-06-22
description: "Niels Bohr's quantized atomic model and hydrogen spectrum explanation"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Physics History", "Bohr", "Atomic Model"]
draft: false
---

{{< katex >}}

## Overview

In 1913, Niels Bohr proposed a revolutionary model of the atom that explained the discrete spectral lines of hydrogen. He introduced the concept of quantized electron orbits.

## The Problem

### Classical Atomic Model Failure

Rutherford's nuclear model:
- Electrons orbit nucleus
- Classical physics: accelerating charges radiate
- Predicted: electron spirals into nucleus

This didn't match reality—atoms are stable!

### Hydrogen Spectrum Mystery

Hydrogen emits light at specific wavelengths:

$$
\frac{1}{\lambda} = R_H \left(\frac{1}{n_1^2} - \frac{1}{n_2^2}\right)
$$

Where \\(R_H = 1.097 \times 10^7\\) m⁻¹ (Rydberg constant).

Why only these wavelengths?

## Bohr's Postulates

### 1. Quantized Orbits

Electrons can only occupy specific orbits where angular momentum is quantized:

$$
L = mvr = n\hbar = n\frac{h}{2\pi}
$$

Where \\(n = 1, 2, 3, ...\\) (principal quantum number).

### 2. Stationary States

In allowed orbits:
- Electrons don't radiate energy
- Atoms are stable
- Classical electromagnetism doesn't apply

### 3. Quantum Jumps

Energy is emitted/absorbed only during transitions:

$$
\Delta E = E_{n_2} - E_{n_1} = h\nu
$$

## Derivation of Hydrogen Energy Levels

### Force Balance

Coulomb force = Centripetal force:

$$
\frac{ke^2}{r^2} = \frac{mv^2}{r}
$$

### Quantization Condition

$$
mvr = n\hbar
$$

### Solving for Radius

$$
r_n = \frac{n^2\hbar^2}{mke^2} = n^2 a_0
$$

Where Bohr radius:
$$
a_0 = \frac{\hbar^2}{mke^2} = 0.529 \text{ Å}
$$

### Energy Levels

$$
E_n = -\frac{mk^2e^4}{2\hbar^2} \cdot \frac{1}{n^2} = -\frac{13.6 \text{ eV}}{n^2}
$$

## Energy Level Diagram

```
n = ∞  ────────────── 0 eV (ionization)
n = 4  ────────────── -0.85 eV
n = 3  ────────────── -1.51 eV
n = 2  ────────────── -3.40 eV



n = 1  ────────────── -13.6 eV (ground state)
```

## Spectral Series

### Lyman Series (UV)

Transitions to \\(n = 1\\):

$$
\frac{1}{\lambda} = R_H\left(1 - \frac{1}{n^2}\right), \quad n = 2, 3, 4, ...
$$

### Balmer Series (Visible)

Transitions to \\(n = 2\\):

$$
\frac{1}{\lambda} = R_H\left(\frac{1}{4} - \frac{1}{n^2}\right), \quad n = 3, 4, 5, ...
$$

### Paschen Series (IR)

Transitions to \\(n = 3\\):

$$
\frac{1}{\lambda} = R_H\left(\frac{1}{9} - \frac{1}{n^2}\right), \quad n = 4, 5, 6, ...
$$

## Predictions Confirmed

| Prediction | Experimental Value | Bohr Value |
|------------|-------------------|------------|
| Rydberg constant | 1.097 × 10⁷ m⁻¹ | 1.097 × 10⁷ m⁻¹ |
| Bohr radius | 0.529 Å | 0.529 Å |
| H-alpha wavelength | 656.3 nm | 656.3 nm |

Remarkable agreement!

## Limitations

| Limitation | Description |
|------------|-------------|
| Only works for H | Multi-electron atoms fail |
| No fine structure | Misses spectral line splitting |
| Arbitrary quantization | Why is L quantized? |
| No chemical bonding | Can't explain molecules |
| Incorrect angular momentum | Ground state has L=0, not ℏ |

## Correspondence Principle

At large quantum numbers, quantum results approach classical:

$$
\lim_{n \to \infty} (\text{quantum}) = \text{classical}
$$

Bohr used this to develop his theory.

## Legacy

### What Bohr Got Right

1. Energy quantization
2. Discrete spectral lines
3. Stability of atoms
4. Photon emission/absorption

### Foundation for

- Wave mechanics (Schrödinger)
- Matrix mechanics (Heisenberg)
- Quantum numbers
- Atomic structure

Bohr received the Nobel Prize in Physics in 1922.
