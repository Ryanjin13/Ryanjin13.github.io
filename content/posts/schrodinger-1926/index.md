---
title: "1926 Schrödinger - Wave Equation"
date: 2024-06-22
description: "Erwin Schrödinger's formulation of wave mechanics"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Physics History", "Schrödinger", "Wave Equation"]
draft: false
---

{{< katex >}}

## Overview

In 1926, Erwin Schrödinger developed wave mechanics, providing a complete mathematical framework for quantum mechanics. His wave equation describes how quantum systems evolve in time.

## The Schrödinger Equation

### Time-Dependent Form

$$
i\hbar \frac{\partial \Psi}{\partial t} = \hat{H}\Psi
$$

For a particle in potential \\(V(x)\\):

$$
i\hbar \frac{\partial \Psi}{\partial t} = -\frac{\hbar^2}{2m}\frac{\partial^2 \Psi}{\partial x^2} + V(x)\Psi
$$

### Time-Independent Form

For stationary states \\(\Psi(x,t) = \psi(x)e^{-iEt/\hbar}\\):

$$
\hat{H}\psi = E\psi
$$

$$
-\frac{\hbar^2}{2m}\frac{d^2\psi}{dx^2} + V(x)\psi = E\psi
$$

## Derivation Motivation

### From de Broglie Waves

For a free particle wave:

$$
\Psi = Ae^{i(kx - \omega t)}
$$

Taking derivatives:
- \\(\frac{\partial \Psi}{\partial t} = -i\omega\Psi\\) → \\(E = \hbar\omega\\)
- \\(\frac{\partial^2 \Psi}{\partial x^2} = -k^2\Psi\\) → \\(p = \hbar k\\)

### Energy Relation

Kinetic energy:
$$
E = \frac{p^2}{2m} = \frac{\hbar^2 k^2}{2m}
$$

This leads naturally to the Schrödinger equation.

## Key Concepts

### Wave Function \\(\Psi\\)

- Complex-valued function
- Contains all information about the system
- \\(|\Psi|^2\\) gives probability density

### Hamiltonian Operator

$$
\hat{H} = -\frac{\hbar^2}{2m}\nabla^2 + V(\mathbf{r})
$$

Total energy = Kinetic + Potential

### Operators and Observables

| Observable | Operator |
|------------|----------|
| Position | \\(\hat{x} = x\\) |
| Momentum | \\(\hat{p} = -i\hbar\frac{\partial}{\partial x}\\) |
| Energy | \\(\hat{H}\\) |

## Important Solutions

### Free Particle

\\(V = 0\\):

$$
\psi_k(x) = Ae^{ikx}
$$

Continuous energy spectrum.

### Infinite Square Well

$$
E_n = \frac{n^2\pi^2\hbar^2}{2mL^2}
$$

$$
\psi_n(x) = \sqrt{\frac{2}{L}}\sin\left(\frac{n\pi x}{L}\right)
$$

### Harmonic Oscillator

\\(V = \frac{1}{2}m\omega^2x^2\\):

$$
E_n = \hbar\omega\left(n + \frac{1}{2}\right)
$$

Ground state has zero-point energy!

### Hydrogen Atom

$$
E_n = -\frac{13.6 \text{ eV}}{n^2}
$$

Reproduces Bohr model results, plus angular momentum states.

## Properties of Solutions

### Normalization

$$
\int_{-\infty}^{\infty} |\Psi|^2 dx = 1
$$

### Orthogonality

$$
\int \psi_m^* \psi_n dx = \delta_{mn}
$$

### Completeness

Any wave function can be expanded:

$$
\Psi = \sum_n c_n \psi_n
$$

## Matrix Mechanics Equivalence

Schrödinger proved his wave mechanics is equivalent to Heisenberg's matrix mechanics (1925):

| Wave Mechanics | Matrix Mechanics |
|----------------|------------------|
| Wave functions | State vectors |
| Operators | Matrices |
| Differential equations | Matrix equations |

Both give identical predictions.

## Interpretations

### Born Interpretation

\\(|\Psi(x)|^2\\) is probability density.

Max Born received Nobel Prize (1954) for this interpretation.

### Copenhagen Interpretation

- Wave function is complete description
- Measurement causes collapse
- No underlying deterministic reality

### Schrödinger's Cat

Famous thought experiment highlighting measurement paradox:
- Cat in superposition until observed
- Illustrates interpretation difficulties

## Three-Dimensional Form

$$
i\hbar\frac{\partial\Psi}{\partial t} = -\frac{\hbar^2}{2m}\nabla^2\Psi + V(\mathbf{r})\Psi
$$

Where:
$$
\nabla^2 = \frac{\partial^2}{\partial x^2} + \frac{\partial^2}{\partial y^2} + \frac{\partial^2}{\partial z^2}
$$

## Applications

1. **Atomic structure** - Electron orbitals
2. **Molecular chemistry** - Chemical bonds
3. **Solid state physics** - Band theory
4. **Quantum computing** - Qubit evolution
5. **Quantum field theory** - Foundation

## Nobel Prize

Erwin Schrödinger shared the Nobel Prize in Physics (1933) with Paul Dirac:

> "For the discovery of new productive forms of atomic theory"

## Legacy

The Schrödinger equation is the fundamental equation of non-relativistic quantum mechanics, as central to quantum physics as Newton's laws to classical mechanics.
