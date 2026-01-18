---
title: "Spherical Harmonics on 3D Graphics"
date: 2024-07-15
description: "Mathematical framework for directional light representation in 3D graphics"
categories: ["3D Vision"]
tags: ["Spherical Harmonics", "Gaussian Splatting", "3D Graphics", "Rendering"]
draft: false
---

{{< katex >}}

## Overview

Spherical Harmonics (SH) provide a mathematical framework for representing directional functions on a sphere, commonly used for lighting in 3D graphics and Gaussian Splatting.

## Mathematical Foundation

### General Form

Directional light distribution:

$$
L(\theta, \phi) = \sum_{l=0}^{\infty} \sum_{m=-l}^{l} c_l^m Y_l^m(\theta, \phi)
$$

Where:
- \(L(\theta, \phi)\): Light intensity at direction \((\theta, \phi)\)
- \(c_l^m\): Spherical harmonic coefficients
- \(Y_l^m\): Basis functions

### Basis Functions

$$
Y_l^m(\theta, \phi) = N \cdot e^{im\phi} \cdot P_l^m(\cos\theta)
$$

Where:
- \(N\): Normalization constant
- \(P_l^m\): Associated Legendre polynomials
- \(l\): Degree (band)
- \(m\): Order (-l to +l)

## Practical Implementation

### Band 0 (l=0): Isotropic

Single coefficient representing uniform omnidirectional emission:

$$
Y_0^0 = \frac{1}{2}\sqrt{\frac{1}{\pi}}
$$

**Result:** Constant light in all directions (ambient).

### Band 1 (l=1): Directional

Three basis functions (m = -1, 0, 1) control directional factors:

$$
Y_1^{-1} = \sqrt{\frac{3}{4\pi}} \cdot y
$$
$$
Y_1^{0} = \sqrt{\frac{3}{4\pi}} \cdot z
$$
$$
Y_1^{1} = \sqrt{\frac{3}{4\pi}} \cdot x
$$

**Result:** Linear directional variation across x, y, z axes.

### Practical Approximation (Bands 0-1)

$$
L(\theta, \phi) \approx c_0^0 Y_0^0 + c_1^{-1} Y_1^{-1} + c_1^0 Y_1^0 + c_1^1 Y_1^1
$$

**4 coefficients** capture ambient + basic directionality.

## Application in Gaussian Splatting

In 3D Gaussian Splatting, SH coefficients encode view-dependent color:

```
Gaussian Parameters:
- Position (x, y, z)
- Covariance (scale, rotation)
- Opacity (α)
- SH Coefficients (c_l^m)  ← View-dependent color
```

**Typical Configuration:**
- Use only l=0,1 orders (4 coefficients per color channel)
- Total: 4 × 3 (RGB) = 12 coefficients
- Balance between quality and computation

## Connection to Fourier Transform

Spherical harmonics are analogous to Fourier transforms on a sphere:

| Fourier | Spherical Harmonics |
|---------|---------------------|
| 1D signal | Spherical function |
| Frequency | Band (l) |
| Sine/Cosine | Y_l^m basis |
| Coefficients | c_l^m coefficients |

Higher bands capture higher frequency directional variations.

## Coefficient Count by Band

| Max Band | Coefficients | Use Case |
|----------|--------------|----------|
| l=0 | 1 | Ambient only |
| l=1 | 4 | Basic directional |
| l=2 | 9 | Glossy surfaces |
| l=3 | 16 | Detailed lighting |

## Benefits

1. **Compact representation** - Few coefficients for smooth lighting
2. **Rotation invariant** - Easy to rotate light environment
3. **Efficient evaluation** - Simple polynomial computation
4. **Natural for diffuse** - Perfect for Lambertian surfaces
