---
title: "Quantum Wave Function"
date: 2024-06-21
description: "Understanding the wave function in quantum mechanics"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Wave Function", "Schrödinger Equation"]
draft: false
---

{{< katex >}}

## Overview

The wave function \\(\Psi\\) is the fundamental mathematical description of quantum systems. It contains all information about a particle's quantum state.

## Definition

The wave function \\(\Psi(x, t)\\) is a complex-valued function:

$$
\Psi(x, t) = A e^{i(kx - \omega t)}
$$

Where:
- \\(k = \frac{2\pi}{\lambda}\\): Wave number
- \\(\omega = 2\pi f\\): Angular frequency
- \\(A\\): Amplitude

## Physical Interpretation

### Born's Probability Interpretation

The probability of finding a particle between \\(x\\) and \\(x + dx\\):

$$
P(x) dx = |\Psi(x)|^2 dx = \Psi^* \Psi \, dx
$$

### Normalization Condition

Total probability must equal 1:

$$
\int_{-\infty}^{\infty} |\Psi(x)|^2 dx = 1
$$

## The Schrödinger Equation

### Time-Dependent

$$
i\hbar \frac{\partial \Psi}{\partial t} = -\frac{\hbar^2}{2m}\frac{\partial^2 \Psi}{\partial x^2} + V(x)\Psi
$$

Or in operator form:

$$
i\hbar \frac{\partial \Psi}{\partial t} = \hat{H}\Psi
$$

### Time-Independent

For stationary states \\(\Psi(x,t) = \psi(x)e^{-iEt/\hbar}\\):

$$
-\frac{\hbar^2}{2m}\frac{d^2\psi}{dx^2} + V(x)\psi = E\psi
$$

## Important Examples

### Free Particle

\\(V(x) = 0\\):

$$
\Psi(x, t) = Ae^{i(kx - \omega t)}
$$

Energy relation:

$$
E = \frac{\hbar^2 k^2}{2m} = \frac{p^2}{2m}
$$

### Infinite Square Well

\\(V = 0\\) for \\(0 < x < L\\), \\(V = \infty\\) otherwise:

$$
\psi_n(x) = \sqrt{\frac{2}{L}}\sin\left(\frac{n\pi x}{L}\right)
$$

Energy levels:

$$
E_n = \frac{n^2 \pi^2 \hbar^2}{2mL^2}
$$

### Harmonic Oscillator

\\(V(x) = \frac{1}{2}m\omega^2 x^2\\):

$$
\psi_n(x) = \left(\frac{m\omega}{\pi\hbar}\right)^{1/4} \frac{1}{\sqrt{2^n n!}} H_n(\xi) e^{-\xi^2/2}
$$

Where \\(\xi = \sqrt{\frac{m\omega}{\hbar}}x\\) and \\(H_n\\) are Hermite polynomials.

$$
E_n = \hbar\omega\left(n + \frac{1}{2}\right)
$$

## Properties of Wave Functions

### Superposition

If \\(\Psi_1\\) and \\(\Psi_2\\) are solutions, so is:

$$
\Psi = c_1\Psi_1 + c_2\Psi_2
$$

### Expectation Values

Position:
$$
\langle x \rangle = \int_{-\infty}^{\infty} \Psi^* x \Psi \, dx
$$

Momentum:
$$
\langle p \rangle = \int_{-\infty}^{\infty} \Psi^* \left(-i\hbar\frac{\partial}{\partial x}\right) \Psi \, dx
$$

### Uncertainty

$$
\Delta x = \sqrt{\langle x^2 \rangle - \langle x \rangle^2}
$$

$$
\Delta p = \sqrt{\langle p^2 \rangle - \langle p \rangle^2}
$$

Heisenberg uncertainty principle:

$$
\Delta x \cdot \Delta p \geq \frac{\hbar}{2}
$$

## Wave Function Collapse

Upon measurement:
- Wave function "collapses" to eigenstate
- Probability becomes certainty
- Copenhagen interpretation

## Dirac Notation

| Notation | Meaning |
|----------|---------|
| \\(\ket{\psi}\\) | State vector (ket) |
| \\(\bra{\phi}\\) | Dual vector (bra) |
| \\(\braket{\phi\|\psi}\\) | Inner product |
| \\(\ket{\psi}\bra{\phi}\\) | Outer product |
