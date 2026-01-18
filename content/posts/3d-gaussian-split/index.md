---
title: "3D Gaussian Split Conditions"
date: 2024-07-01
description: "Adaptive density control in 3D Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "3D Reconstruction", "Optimization"]
draft: false
---

{{< katex >}}

## Overview

3D Gaussian Splatting uses adaptive density control to refine scene representation. Gaussians are split or cloned based on specific conditions during optimization.

## Split Conditions

### Two Primary Criteria

**1. Gradient Magnitude Threshold**

When positional gradient exceeds threshold:

$$
\|\nabla_p L\| > \tau_{grad}
$$

Indicates the Gaussian needs refinement to better represent the scene.

**2. Size Threshold**

When Gaussian scale exceeds limit:

$$
\max(s_x, s_y, s_z) > \tau_{size}
$$

Large Gaussians should be split into smaller ones.

## Adaptive Control Operations

### Split (Over-reconstruction)

When Gaussian is **too large** with high gradient:

```
Parent Gaussian → 2 Child Gaussians
- Positions: Along principal axes
- Scale: 1/2 to 2/3 of original
- Opacity: Inherited, then adjusted
- Color: NOT inherited (re-learned)
```

### Clone (Under-reconstruction)

When Gaussian is **too small** with high gradient:

```
Parent Gaussian → Parent + 1 Clone
- Clone positioned along gradient direction
- Same scale as parent
```

### Prune

Remove Gaussians with:
- Very low opacity: \(\alpha < \tau_{opacity}\)
- Very large scale in world space

## Algorithm

```python
for gaussian in gaussians:
    grad = compute_gradient(gaussian)

    if norm(grad) > tau_grad:
        if gaussian.scale > tau_size:
            # Split: Too large
            split_gaussian(gaussian)
        else:
            # Clone: Too small
            clone_gaussian(gaussian)

    if gaussian.opacity < tau_opacity:
        prune_gaussian(gaussian)
```

## Axis-Aligned Benefits

| Advantage | Description |
|-----------|-------------|
| **Fast computation** | Simple matrix operations |
| **GPU friendly** | Efficient parallel processing |
| **Easy sampling** | Simplified ray-casting |
| **Hierarchical** | Natural octree integration |
| **Fast convergence** | Quick early training |

## Trade-offs

| Pros | Cons |
|------|------|
| Computational efficiency | Less expressive for diagonal surfaces |
| Real-time rendering | May need more Gaussians |
| Simple implementation | Memory overhead for complex scenes |

## Training Schedule

Typical adaptive control:

| Iteration | Operation |
|-----------|-----------|
| 0 - 500 | Densification disabled |
| 500 - 15000 | Active split/clone |
| 15000+ | Densification disabled |

Opacity reset at iteration ~3000 to remove floaters.

## Parameters

| Parameter | Typical Value | Description |
|-----------|---------------|-------------|
| \(\tau_{grad}\) | 0.0002 | Gradient threshold |
| \(\tau_{size}\) | World-dependent | Size threshold |
| \(\tau_{opacity}\) | 0.005 | Prune threshold |
| Densify interval | 100 iters | Check frequency |
