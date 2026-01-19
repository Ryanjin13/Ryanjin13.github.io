---
title: "SIFT Algorithm"
date: 2024-06-21
description: "Scale-Invariant Feature Transform for robust feature detection"
categories: ["2D Vision"]
tags: ["SIFT", "Feature Detection", "Computer Vision"]
draft: false
---

{{< katex >}}

## Overview

SIFT (Scale-Invariant Feature Transform) detects and describes local features in images that are invariant to scale, rotation, and partially invariant to illumination changes.

## Pipeline

```
Image → Scale Space → DoG → Keypoint Detection → Orientation → Descriptor
```

## 1. Scale Space Construction

Build Gaussian pyramid by repeatedly blurring and downsampling:

$$
L(x, y, \sigma) = G(x, y, \sigma) * I(x, y)
$$

Where Gaussian kernel:

$$
G(x, y, \sigma) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2 + y^2}{2\sigma^2}}
$$

### Octaves and Scales

```
Octave 1: σ, kσ, k²σ, k³σ, k⁴σ
Octave 2: 2σ, 2kσ, 2k²σ, ... (half resolution)
Octave 3: 4σ, ... (quarter resolution)
```

Typically k = √2, 5 scales per octave.

## 2. Difference of Gaussian (DoG)

Approximate Laplacian of Gaussian:

$$
D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)
$$

DoG approximates scale-normalized LoG:

$$
\sigma^2 \nabla^2 G \approx \frac{G(k\sigma) - G(\sigma)}{k - 1}
$$

## 3. Keypoint Detection

### Extrema Detection

Compare each pixel with 26 neighbors (8 in same scale + 9 above + 9 below).

### Keypoint Refinement

Taylor expansion for sub-pixel accuracy:

$$
D(\mathbf{x}) = D + \frac{\partial D^T}{\partial \mathbf{x}}\mathbf{x} + \frac{1}{2}\mathbf{x}^T \frac{\partial^2 D}{\partial \mathbf{x}^2}\mathbf{x}
$$

Extremum location:

$$
\hat{\mathbf{x}} = -\frac{\partial^2 D^{-1}}{\partial \mathbf{x}^2} \frac{\partial D}{\partial \mathbf{x}}
$$

### Edge Response Elimination

Using Hessian matrix eigenvalue ratio:

$$
\frac{Tr(H)^2}{Det(H)} < \frac{(r+1)^2}{r}
$$

Where r = 10 (threshold for edge ratio).

## 4. Orientation Assignment

Compute gradient magnitude and orientation:

$$
m(x,y) = \sqrt{(L_{x+1} - L_{x-1})^2 + (L_{y+1} - L_{y-1})^2}
$$

$$
\theta(x,y) = \tan^{-1}\left(\frac{L_{y+1} - L_{y-1}}{L_{x+1} - L_{x-1}}\right)
$$

Build 36-bin orientation histogram, assign dominant orientation(s).

## 5. Descriptor Generation

### 128-dimensional descriptor:

1. Take 16×16 window around keypoint
2. Divide into 4×4 grid of cells
3. Compute 8-bin orientation histogram per cell
4. Result: 4×4×8 = 128 dimensions
5. Normalize to unit length

```
┌───┬───┬───┬───┐
│ 8 │ 8 │ 8 │ 8 │  ← 8-bin histogram per cell
├───┼───┼───┼───┤
│ 8 │ 8 │ 8 │ 8 │
├───┼───┼───┼───┤     4×4 = 16 cells
│ 8 │ 8 │ 8 │ 8 │     16 × 8 = 128 dimensions
├───┼───┼───┼───┤
│ 8 │ 8 │ 8 │ 8 │
└───┴───┴───┴───┘
```

## Matching

Use Euclidean distance with ratio test:

$$
\frac{d_1}{d_2} < 0.8
$$

Where d₁ = nearest neighbor, d₂ = second nearest.

## Properties

| Property | SIFT |
|----------|------|
| Scale invariant | Yes |
| Rotation invariant | Yes |
| Illumination | Partially |
| Affine | No (use ASIFT) |
| Speed | Slow |
| Descriptor size | 128-D |

## Applications

- Image stitching
- Object recognition
- 3D reconstruction
- Robot navigation
- Augmented reality
