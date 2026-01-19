---
title: "Anisotropic Learning in Gaussian Splatting"
date: 2024-07-10
description: "Limitations and solutions for directional representation in 3D Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Anisotropy", "3D Reconstruction"]
draft: false
---

{{< katex >}}

## Overview

Standard 3D Gaussian Splatting has limitations in representing anisotropic (directional) structures. This post explores these limitations and proposes solutions.

## Current Limitations

### 1. Diagonal-Only Covariance

The standard approach projects 3D Gaussians onto 2D using only diagonal matrices:

$$
\Sigma_{diag} = \begin{pmatrix} \sigma_x^2 & 0 & 0 \\ 0 & \sigma_y^2 & 0 \\ 0 & 0 & \sigma_z^2 \end{pmatrix}
$$

This restricts representational capacity.

### 2. Incomplete Covariance Learning

Off-diagonal elements express directional relationships:

$$
\sigma_{xy}, \sigma_{xz}, \sigma_{yz}
$$

Without these, anisotropic surfaces are difficult to represent.

### 3. Axis-Aligned Splitting

When Gaussians subdivide, they primarily grow along x and y axes:

```
Original:      After Split:
    ●          ● ●

```

Non-axis-aligned surfaces require many more Gaussians.

## Consequences

| Issue | Effect |
|-------|--------|
| More Gaussians needed | Memory and compute increase |
| Limited detail | Complex geometry poorly represented |
| View-dependent artifacts | Visible at certain angles |

### Visual Example

For a tilted surface:

```
Axis-aligned (many Gaussians):    Anisotropic (fewer needed):
  ● ● ● ● ●                           ⬤
  ● ● ● ● ●                              ⬤
  ● ● ● ● ●                                 ⬤
```

## Proposed Solutions

### 1. Full Covariance Matrices

Enable complete 3×3 covariance learning:

$$
\Sigma = \begin{pmatrix}
\sigma_x^2 & \sigma_{xy} & \sigma_{xz} \\
\sigma_{xy} & \sigma_y^2 & \sigma_{yz} \\
\sigma_{xz} & \sigma_{yz} & \sigma_z^2
\end{pmatrix}
$$

**Advantage:** Complete directional expression
**Challenge:** Ensure positive semi-definiteness

### 2. Quaternion-Based Rotation

Represent orientation with quaternions:

$$
q = (q_w, q_x, q_y, q_z)
$$

**Advantages:**
- No gimbal lock
- Stable optimization
- Smooth interpolation

### 3. Separated Scale and Rotation

Learn independently:

$$
\Sigma = R \cdot S \cdot S^T \cdot R^T
$$

Where:
- \\(R\\): Rotation matrix (from quaternion)
- \\(S\\): Diagonal scale matrix

### 4. Adaptive Splitting

Split based on local surface curvature:

```
High curvature → More splits
Low curvature → Fewer splits
```

Algorithm:
1. Estimate local surface normal
2. Calculate curvature
3. Split direction follows surface tangent

### 5. Directional Loss Functions

Incorporate surface normals in loss:

$$
L = L_{color} + \lambda_n L_{normal}
$$

Where:

$$
L_{normal} = \|n_{predicted} - n_{target}\|^2
$$

### 6. Multi-Lobe Gaussians

Combine multiple smaller distributions:

$$
G_{multi} = \sum_{i=1}^{k} w_i \cdot G_i
$$

**Use case:** Complex textures and fine details.

### 7. Hierarchical Structures

Two-level representation:

| Level | Purpose |
|-------|---------|
| Coarse | Large-scale anisotropy |
| Fine | Small details |

## Trade-offs

### Computational Cost

| Approach | Complexity Increase |
|----------|---------------------|
| Full covariance | ~2× |
| Multi-lobe | ~k× per Gaussian |
| Hierarchical | ~2× memory |

### Quality Improvement

- Better thin structure representation
- Fewer Gaussians for same quality
- Reduced view-dependent artifacts

## Implementation Considerations

### Positive Semi-Definiteness

Ensure valid covariance via Cholesky decomposition:

$$
\Sigma = LL^T
$$

Learn \\(L\\) (lower triangular) instead of \\(\Sigma\\).

### Gradient Stability

Quaternion normalization:

$$
q_{norm} = \frac{q}{\|q\|}
$$

Apply after each optimization step.

### Memory Management

For large scenes:
- Level-of-detail (LOD) based on distance
- Streaming for distant regions
- Compression for inactive areas

## Conclusion

Addressing anisotropic learning limitations:
1. Increases representational efficiency
2. Improves rendering quality
3. Reduces Gaussian count
4. At cost of computational complexity

The trade-off is worthwhile for high-quality reconstruction.
