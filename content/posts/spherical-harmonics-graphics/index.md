---
title: "Spherical Harmonics in 3D Graphics"
date: 2024-07-13
description: "Understanding spherical harmonics for view-dependent appearance in Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Spherical Harmonics", "Gaussian Splatting", "Computer Graphics"]
draft: false
---

{{< katex >}}

## Overview

Spherical harmonics (SH) provide a compact representation for view-dependent appearance in 3D graphics. They're essential in Gaussian Splatting for representing color that changes with viewing angle.

## Mathematical Foundation

### Spherical Harmonics Definition

$$
Y_l^m(\theta, \phi) = N_{lm} \cdot e^{im\phi} \cdot P_l^m(\cos\theta)
$$

Where:
- \\(l\\): Degree (band)
- \\(m\\): Order (\\(-l \leq m \leq l\\))
- \\(N_{lm}\\): Normalization constant
- \\(P_l^m\\): Associated Legendre polynomial
- \\(\theta, \phi\\): Spherical coordinates

### Function Approximation

Any function on the sphere can be represented as:

$$
L(\theta, \phi) = \sum_{l=0}^{\infty} \sum_{m=-l}^{l} c_{lm} Y_l^m(\theta, \phi)
$$

Where \\(c_{lm}\\) are the SH coefficients.

## SH Bands

### Band 0 (l=0): Constant

$$
Y_0^0 = \frac{1}{2}\sqrt{\frac{1}{\pi}}
$$

- Single coefficient
- Ambient/isotropic light
- Same in all directions

### Band 1 (l=1): Linear

$$
Y_1^{-1} = \sqrt{\frac{3}{4\pi}} \cdot y
$$

$$
Y_1^0 = \sqrt{\frac{3}{4\pi}} \cdot z
$$

$$
Y_1^1 = \sqrt{\frac{3}{4\pi}} \cdot x
$$

- Three coefficients
- Directional light component
- Linear variation

### Band 2 (l=2): Quadratic

- Five coefficients
- More complex lighting
- Soft shadows

## Application in Gaussian Splatting

### Typical Configuration

Gaussian Splatting commonly uses bands 0 and 1:

$$
L(\theta, \phi) \approx c_0^0 Y_0^0 + c_1^{-1} Y_1^{-1} + c_1^0 Y_1^0 + c_1^1 Y_1^1
$$

Total: 4 coefficients per color channel = 12 values (RGB).

### Per-Gaussian Storage

| Component | Coefficients | Purpose |
|-----------|--------------|---------|
| SH Band 0 | 1 × 3 (RGB) | Base color |
| SH Band 1 | 3 × 3 (RGB) | View-dependence |
| Total | 12 | Complete appearance |

## Analogy to Fourier Transform

Like Fourier series for periodic functions:

| Fourier | Spherical Harmonics |
|---------|-------------------|
| 1D functions | Functions on sphere |
| Sine/cosine basis | SH basis functions |
| Frequency components | Angular components |

### Key Insight

Diverse frequency summations create directionality:
- Low frequencies → smooth variation
- High frequencies → sharp details

## Computing SH Coefficients

### From Environment Map

```python
def compute_sh_coefficients(envmap, bands=2):
    """
    Compute SH coefficients from environment map
    """
    coeffs = np.zeros((bands**2, 3))

    for l in range(bands):
        for m in range(-l, l+1):
            idx = l*l + l + m
            # Integrate envmap * Y_lm over sphere
            coeffs[idx] = integrate_sh(envmap, l, m)

    return coeffs
```

### Evaluating SH

```python
def evaluate_sh(coeffs, direction, bands=2):
    """
    Evaluate SH at given direction
    """
    color = np.zeros(3)

    # Band 0
    color += coeffs[0] * 0.282095  # Y_0^0

    if bands > 1:
        x, y, z = direction
        # Band 1
        color += coeffs[1] * 0.488603 * y   # Y_1^-1
        color += coeffs[2] * 0.488603 * z   # Y_1^0
        color += coeffs[3] * 0.488603 * x   # Y_1^1

    return color
```

## Visual Representation

### Band 0: Constant

```
     ●
    ●●●    All same color
     ●
```

### Band 1: Directional

```
     ●
    ○●●    Varies with direction
     ○
```

### Higher Bands: Complex

More bands = more view-dependent detail.

## Trade-offs

| More Bands | Fewer Bands |
|------------|-------------|
| Better quality | Faster |
| More memory | Less storage |
| Slower evaluation | Real-time friendly |
| Captures specular | Diffuse only |

## In Practice

### Gaussian Splatting Implementation

```python
class Gaussian:
    def __init__(self):
        self.position = np.zeros(3)
        self.covariance = np.eye(3)
        self.opacity = 1.0

        # SH coefficients: 4 per channel (bands 0-1)
        self.sh_r = np.zeros(4)
        self.sh_g = np.zeros(4)
        self.sh_b = np.zeros(4)

    def get_color(self, view_direction):
        r = evaluate_sh(self.sh_r, view_direction)
        g = evaluate_sh(self.sh_g, view_direction)
        b = evaluate_sh(self.sh_b, view_direction)
        return np.array([r, g, b])
```

### Training

SH coefficients are optimized during training to match ground truth appearance from all viewing angles.

## Summary

Spherical harmonics in 3D graphics:
1. Compact representation of view-dependent color
2. Bands 0-1 commonly used (4 coefficients)
3. Analogous to Fourier transform on sphere
4. Enable realistic specular effects
5. Essential for neural rendering quality
