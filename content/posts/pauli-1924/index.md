---
title: "1924 Pauli - Exclusion Principle"
date: 2024-06-22
description: "Wolfgang Pauli's exclusion principle and its role in atomic structure"
categories: ["Quantum"]
tags: ["Quantum Mechanics", "Physics History", "Pauli", "Exclusion Principle"]
draft: false
---

{{< katex >}}

## Overview

In 1925, Wolfgang Pauli formulated the exclusion principle, which states that no two identical fermions can occupy the same quantum state. This principle explains atomic shell structure and much of chemistry.

## The Problem

### Anomalous Zeeman Effect

In a magnetic field, spectral lines split in unexpected ways:
- Expected: 3 lines (normal Zeeman)
- Observed: More complex patterns

### Shell Structure Mystery

Why do atoms have specific electron configurations?
- Why 2 electrons in first shell?
- Why 8 in second?
- Why does the periodic table work?

## Pauli's Solution

### The Fourth Quantum Number

Pauli proposed electrons have a fourth quantum number (later identified as spin):

$$
m_s = +\frac{1}{2} \text{ or } -\frac{1}{2}
$$

### The Exclusion Principle

**No two electrons in an atom can have the same set of all four quantum numbers:**

$$
(n, l, m_l, m_s)_1 \neq (n, l, m_l, m_s)_2
$$

## Quantum Numbers

| Number | Symbol | Values | Meaning |
|--------|--------|--------|---------|
| Principal | \\(n\\) | 1, 2, 3, ... | Energy level, shell |
| Angular | \\(l\\) | 0 to n-1 | Orbital shape |
| Magnetic | \\(m_l\\) | -l to +l | Orbital orientation |
| Spin | \\(m_s\\) | ±1/2 | Spin orientation |

## Shell Filling

### Maximum Electrons per Subshell

For given \\(l\\):
- \\(2l + 1\\) values of \\(m_l\\)
- 2 values of \\(m_s\\) each
- Total: \\(2(2l + 1)\\) electrons

| Subshell | l | Orbitals | Max Electrons |
|----------|---|----------|---------------|
| s | 0 | 1 | 2 |
| p | 1 | 3 | 6 |
| d | 2 | 5 | 10 |
| f | 3 | 7 | 14 |

### Maximum per Shell

$$
\text{Max electrons in shell } n = 2n^2
$$

| Shell | n | Max Electrons |
|-------|---|---------------|
| K | 1 | 2 |
| L | 2 | 8 |
| M | 3 | 18 |
| N | 4 | 32 |

## Mathematical Formulation

### Antisymmetric Wave Functions

For fermions, the total wave function must be antisymmetric:

$$
\Psi(x_1, x_2) = -\Psi(x_2, x_1)
$$

### Slater Determinant

For N fermions:

$$
\Psi = \frac{1}{\sqrt{N!}} \begin{vmatrix}
\phi_1(1) & \phi_2(1) & \cdots & \phi_N(1) \\
\phi_1(2) & \phi_2(2) & \cdots & \phi_N(2) \\
\vdots & \vdots & \ddots & \vdots \\
\phi_1(N) & \phi_2(N) & \cdots & \phi_N(N)
\end{vmatrix}
$$

If two electrons have same state, two rows are identical → determinant = 0.

## Fermions vs Bosons

| Property | Fermions | Bosons |
|----------|----------|--------|
| Spin | Half-integer (1/2, 3/2, ...) | Integer (0, 1, 2, ...) |
| Statistics | Fermi-Dirac | Bose-Einstein |
| Exclusion | Yes | No |
| Examples | Electrons, protons, neutrons | Photons, gluons, Higgs |

## Consequences

### 1. Periodic Table Structure

Electron configurations follow exclusion principle:
- H: 1s¹
- He: 1s²
- Li: 1s² 2s¹
- Ne: 1s² 2s² 2p⁶

### 2. Chemical Properties

- Noble gases: Filled shells → stable
- Alkali metals: One electron beyond filled shell → reactive
- Halogens: One electron short of filled shell → reactive

### 3. Solid State Physics

- Band theory of metals
- Fermi energy and Fermi surface
- Conductors vs insulators

### 4. White Dwarf Stars

Electron degeneracy pressure:
- Pauli exclusion prevents collapse
- Supports star against gravity

### 5. Neutron Stars

Neutron degeneracy pressure:
- Same principle with neutrons
- Even denser than white dwarfs

## Spin-Statistics Theorem

Deep connection between spin and statistics:

> Particles with half-integer spin must obey Fermi-Dirac statistics (exclusion principle).

Proven by Pauli (1940) using relativistic quantum field theory.

## Nobel Prize

Wolfgang Pauli received the Nobel Prize in Physics in 1945:

> "For the discovery of the Exclusion Principle, also called the Pauli Principle"
