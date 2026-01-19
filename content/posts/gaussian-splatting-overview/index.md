---
title: "3D Gaussian Splatting Overview"
date: 2024-06-22
description: "Comprehensive overview of the 3D Gaussian Splatting pipeline for novel view synthesis"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Novel View Synthesis", "3D Reconstruction", "Real-time Rendering"]
draft: false
---

{{< katex >}}

## Overview

3D Gaussian Splatting achieves high-quality novel view synthesis with real-time rendering speeds. This post provides a comprehensive overview of the complete pipeline.

## Pipeline Summary

```
Input Images → SFM → Point Cloud → 3D Gaussians → Optimization → Rendering
                ↓                       ↓              ↓
          Camera Poses           Parameters    Adaptive Density
```

## Stage 1: Structure from Motion

### Input

- Multiple photographs of a scene
- Various viewpoints

### Output

- Sparse point cloud
- Camera poses (position + orientation)
- Intrinsic parameters

### Tools

- COLMAP (commonly used)
- VisualSFM
- OpenMVG

## Stage 2: 3D Gaussian Initialization

Each point becomes a 3D Gaussian with parameters:

### Position (Mean) \\(\mu\\)

$$
\mu \in \mathbb{R}^3
$$

Center of the Gaussian in world coordinates.

### Covariance \\(\Sigma\\)

$$
\Sigma = RSS^TR^T \in \mathbb{R}^{3 \times 3}
$$

Where:
- \\(R\\): Rotation matrix (from quaternion)
- \\(S\\): Scale matrix (diagonal)

This ensures positive semi-definiteness.

### Opacity \\(\alpha\\)

$$
\alpha \in [0, 1]
$$

Controls transparency.

### Color (Spherical Harmonics)

$$
c = \text{SH}(\mathbf{d}) \in \mathbb{R}^3
$$

View-dependent color via spherical harmonics coefficients.

## Stage 3: Spherical Harmonics

### Why SH?

- Compact representation of view-dependent effects
- Differentiable
- Captures specular highlights

### Representation

Color as function of viewing direction:

$$
c(\mathbf{d}) = \sum_{l=0}^{L} \sum_{m=-l}^{l} c_{lm} Y_l^m(\mathbf{d})
$$

Typically \\(L = 3\\) (16 coefficients per color channel).

## Stage 4: Rendering (Tile-based Rasterizer)

### Screen Division

Split screen into 16×16 pixel tiles.

### Per-Tile Processing

1. **Culling:** Identify Gaussians overlapping tile
2. **Sorting:** Order by depth
3. **Blending:** Alpha composite front-to-back

### Alpha Blending

$$
C = \sum_{i=1}^{N} c_i \alpha_i \prod_{j=1}^{i-1}(1 - \alpha_j)
$$

### Why Tile-based?

| Benefit | Description |
|---------|-------------|
| Parallelism | Tiles processed independently |
| Cache efficiency | Spatial locality |
| GPU-friendly | Maps to GPU architecture |

## Stage 5: Optimization

### Loss Function

$$
L = (1 - \lambda)L_1 + \lambda L_{D-SSIM}
$$

- \\(L_1\\): Pixel-wise L1 loss
- \\(L_{D-SSIM}\\): Structural similarity
- \\(\lambda\\): Typically 0.2

### Gradient Flow

Loss → Rendered Image → Alpha Blending → Gaussian Parameters

Fast backpropagation compared to NeRF.

### Optimized Parameters

- Position \\(\mu\\)
- Covariance (via quaternion + scale)
- Opacity \\(\alpha\\)
- SH coefficients

## Stage 6: Adaptive Density Control

### Why Needed?

Initial SFM points are sparse and may not capture all details.

### Operations

**Clone:** Copy small Gaussians in high-gradient regions

$$
\text{If } \|\nabla L\| > \tau_{grad} \text{ and } \|S\| < \tau_{size}
$$

**Split:** Divide large Gaussians

$$
\text{If } \|\nabla L\| > \tau_{grad} \text{ and } \|S\| > \tau_{size}
$$

**Prune:** Remove unnecessary Gaussians

$$
\text{If } \alpha < \tau_{opacity} \text{ or } \|S\| > \tau_{large}
$$

### Densification Schedule

- Densify every N iterations
- Prune periodically
- Reset opacity occasionally

## Comparison with NeRF

| Aspect | NeRF | 3D Gaussian Splatting |
|--------|------|----------------------|
| Representation | Implicit (MLP) | Explicit (Gaussians) |
| Rendering | Ray marching | Rasterization |
| Training time | Hours | Minutes |
| Rendering speed | Seconds/frame | Real-time |
| Editability | Difficult | Easy |
| Memory | Network weights | Point cloud |

## Advantages

### Speed

- Real-time rendering (100+ FPS)
- Fast training (~30 minutes)
- Fast backpropagation

### Quality

- High-quality novel views
- View-dependent effects via SH
- Sharp details

### Flexibility

- Explicit representation allows editing
- Easy to manipulate individual Gaussians
- Compatible with graphics pipelines

## Challenges

### Normal Estimation

Gaussians don't explicitly store normals:

> "Estimating normals is very difficult"

Addressed through covariance orientation or auxiliary methods.

### Memory

Large scenes require many Gaussians:
- Adaptive pruning helps
- Level-of-detail strategies

### Thin Structures

May require many small Gaussians:
- Densification addresses this
- But increases memory

## Applications

1. **Virtual Reality:** Real-time scene exploration
2. **Game Development:** Fast asset creation
3. **E-commerce:** Product visualization
4. **Cultural Heritage:** Digital preservation
5. **Film/VFX:** Previsualization

## Summary

3D Gaussian Splatting provides:
- High-quality novel view synthesis
- Real-time rendering capability
- Fast training compared to NeRF
- Explicit, editable representation

The key innovation is representing scenes as 3D Gaussians rendered via differentiable tile-based rasterization.
