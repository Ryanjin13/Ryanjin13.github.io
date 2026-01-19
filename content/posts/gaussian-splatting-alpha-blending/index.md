---
title: "Alpha Blending in Gaussian Splatting"
date: 2024-06-22
description: "How final colors are computed through alpha blending in 3D Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Alpha Blending", "Neural Rendering", "NeRF"]
draft: false
---

{{< katex >}}

## Overview

3D Gaussian Splatting uses alpha blending to compute final pixel colors by accumulating contributions from multiple overlapping Gaussians along a ray. This post explains the rendering equation and how it differs from NeRF.

## Transmittance and Opacity

### Key Relationship

$$
T + \alpha = 1
$$

Where:
- \\(T\\): Transmittance (light passing through)
- \\(\alpha\\): Opacity (light absorbed/scattered)

### Through Multiple Media

For light passing through N Gaussians:

$$
T_{total} = \prod_{i=1}^{N} T_i = \prod_{i=1}^{N} (1 - \alpha_i)
$$

## Alpha Blending Equation

### Color Accumulation

Final pixel color computed front-to-back:

$$
C = \sum_{i=1}^{N} c_i \alpha_i \prod_{j=1}^{i-1}(1 - \alpha_j)
$$

Equivalently:

$$
C = \sum_{i=1}^{N} c_i \alpha_i T_i
$$

Where \\(T_i = \prod_{j=1}^{i-1}(1 - \alpha_j)\\) is accumulated transmittance.

### Iterative Formulation

$$
C_i = C_{i-1} + T_{i-1} \cdot \alpha_i \cdot c_i
$$

$$
T_i = T_{i-1} \cdot (1 - \alpha_i)
$$

Starting with \\(C_0 = 0\\), \\(T_0 = 1\\).

## Ray Traversal

```
Camera ──→ G1 ──→ G2 ──→ G3 ──→ Background
            ↓      ↓      ↓
         α₁,c₁  α₂,c₂  α₃,c₃
```

### Processing Order

1. Sort Gaussians by depth (front to back)
2. Initialize: \\(C = 0\\), \\(T = 1\\)
3. For each Gaussian:
   - Add contribution: \\(C += T \cdot \alpha_i \cdot c_i\\)
   - Update transmittance: \\(T *= (1 - \alpha_i)\\)
4. Early termination when \\(T < \epsilon\\)

### Termination Condition

When accumulated transmittance falls below threshold:

$$
T < \epsilon \text{ (e.g., } \epsilon = 0.01\text{)}
$$

No significant contribution from further Gaussians.

## Gradient Computation

### Loss Function

$$
L = \|C_{predicted} - C_{target}\|^2
$$

### Backpropagation

Using chain rule, propagate from deepest to shallowest:

$$
\frac{\partial L}{\partial c_i} = \frac{\partial L}{\partial C} \cdot T_i \cdot \alpha_i
$$

$$
\frac{\partial L}{\partial \alpha_i} = \frac{\partial L}{\partial C} \cdot T_i \cdot c_i + \text{(terms from subsequent Gaussians)}
$$

### Backward Pass

Process Gaussians back-to-front:

$$
\frac{\partial L}{\partial \alpha_i} = T_i \cdot c_i \cdot \frac{\partial L}{\partial C} - \sum_{j>i} \frac{T_j}{1-\alpha_i} \cdot \alpha_j \cdot c_j \cdot \frac{\partial L}{\partial C}
$$

## Memory Optimization

### Challenge

Storing all intermediate \\(T_i\\) values for backprop is expensive.

### Solution

Reconstruct from accumulated values:

$$
T_i = \frac{T_{final}}{\prod_{j=i}^{N}(1-\alpha_j)}
$$

Store only:
- Final transmittance \\(T_{final}\\)
- Per-Gaussian \\(\alpha_i\\) values

Reconstruct \\(T_i\\) during backward pass.

## Comparison with NeRF

### NeRF Rendering

Volume rendering integral:

$$
C = \int_{t_n}^{t_f} T(t) \sigma(\mathbf{r}(t)) c(\mathbf{r}(t), \mathbf{d}) dt
$$

$$
T(t) = \exp\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s)) ds\right)
$$

### Key Differences

| Aspect | NeRF | Gaussian Splatting |
|--------|------|-------------------|
| Representation | Continuous MLP | Discrete Gaussians |
| Density | Continuous \\(\sigma(x)\\) | Discrete \\(\alpha_i\\) |
| Transparency | True gradual | Learns toward opaque |
| Integration | Numerical (slow) | Analytic (fast) |
| Memory | High (ray samples) | Lower (sorted Gaussians) |

### NeRF as Linear Model

NeRF models radiance as continuous function via MLP. Density is learned to be:
- High where surfaces exist
- Low in empty space

### Gaussian Splatting

Alpha blending treats each Gaussian as discrete element. Optimization tends to push:
- \\(\alpha \to 1\\) at surface locations
- Clear separation between Gaussians

## Practical Considerations

### Sorting

Efficient sorting is critical:
- Per-tile sorting
- GPU-friendly algorithms
- Approximate sorting acceptable

### Numerical Stability

Avoid:
- \\(\log(0)\\) in gradients
- Division by small \\((1-\alpha)\\)

Use clamping: \\(\alpha \in [\epsilon, 1-\epsilon]\\)

### Background Handling

Add background color when \\(T_{final} > 0\\):

$$
C_{final} = C + T_{final} \cdot C_{background}
$$

## Summary

Alpha blending in Gaussian Splatting:
1. Processes Gaussians front-to-back
2. Accumulates color weighted by transmittance
3. Enables efficient forward and backward passes
4. Provides real-time rendering capability
