---
title: "Fast Differentiable Rasterizer for Gaussians"
date: 2024-06-30
description: "GPU-based tile rasterization for efficient Gaussian Splatting rendering"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Rasterization", "GPU", "Real-time Rendering"]
draft: false
---

{{< katex >}}

## Overview

The fast differentiable rasterizer is a key component enabling real-time 3D Gaussian Splatting. This GPU-based approach achieves high performance through tile-based processing and efficient sorting.

## Design Goals

1. **Fast overall rendering**
2. **Fast sorting for approximate α-blending**
3. **No limit on splats receiving gradients**
4. **Constant memory overhead per pixel**

## Tile-Based Architecture

### Screen Division

Divide screen into 16×16 pixel tiles:

```
┌────┬────┬────┬────┐
│Tile│Tile│Tile│Tile│
│ 0  │ 1  │ 2  │ 3  │
├────┼────┼────┼────┤
│Tile│Tile│Tile│Tile│
│ 4  │ 5  │ 6  │ 7  │
├────┼────┼────┼────┤
│ ...                │
```

### Why Tiles?

| Benefit | Description |
|---------|-------------|
| Parallelism | Each tile processed independently |
| Cache efficiency | Nearby pixels share data |
| GPU-friendly | Maps to thread blocks |

## Culling and Preprocessing

### Frustum Culling

Retain only Gaussians visible in view:

$$
\text{Keep if: } \mu_{projected} + 3\sigma \text{ intersects view}
$$

99% confidence interval ensures no visible Gaussians are missed.

### Guard Band

Reject primitives near camera plane:
- Avoids numerical instabilities
- Handles edge cases

## Sorting Strategy

### Key Construction

Each Gaussian creates key combining:

```
Key = [Tile ID (upper bits)] | [Depth (lower bits)]
```

### Instance Creation

One instance per overlapping tile:

```
Gaussian G overlaps tiles 5, 6, 9, 10
  → Create 4 instances with keys:
     (5, depth_G), (6, depth_G), (9, depth_G), (10, depth_G)
```

### GPU Radix Sort

Single parallel sort organizes all instances:
- O(n) complexity with radix sort
- No per-pixel sorting needed
- Approximate but fast

## Forward Pass

### Thread Block Processing

Each tile handled by one thread block (256 threads for 16×16):

```cpp
// Pseudocode
for each Gaussian in tile (front-to-back):
    load to shared memory
    for each pixel in tile:
        compute Gaussian contribution
        accumulate color: C += α_i * T_i * c_i
        update transmittance: T *= (1 - α_i)
        if (T < threshold) mark saturated
    if (all pixels saturated) break
```

### Early Termination

When all pixels in tile reach α saturation:
- Stop processing remaining Gaussians
- Significant speedup for dense scenes

### Memory Efficiency

No per-pixel storage of blend lists:
- Only final color accumulated
- Constant memory per pixel

## Backward Pass

### Challenge

Need gradients for all Gaussians, but didn't store intermediate values.

### Solution: Recover from Final Values

Traverse Gaussians back-to-front:

$$
T_i = \frac{T_{final}}{\prod_{j=i}^{N}(1 - \alpha_j)}
$$

### Gradient Computation

For each Gaussian (reverse order):

$$
\frac{\partial L}{\partial c_i} = T_i \cdot \alpha_i \cdot \frac{\partial L}{\partial C}
$$

$$
\frac{\partial L}{\partial \alpha_i} = T_i \cdot c_i \cdot \frac{\partial L}{\partial C} + \text{(accumulated terms)}
$$

### Memory Trade-off

| Approach | Memory | Speed |
|----------|--------|-------|
| Store all | O(N×P) | Fast backward |
| Recompute | O(P) | Slower backward |
| This paper | O(P) | Fast backward |

Where N = Gaussians, P = Pixels.

## Key Advantages

### No Hard Limits

All blended primitives receive gradients:
- No arbitrary cutoff
- Better optimization

### Scene Agnostic

No hyperparameter tuning needed:
- Works for dense and sparse scenes
- No tile size adjustment
- Automatic load balancing

### Differentiability

Every operation is differentiable:
- Sorting (discrete but gradients flow through values)
- Blending
- Projection

## Performance

### Typical Numbers

| Scene | Gaussians | FPS |
|-------|-----------|-----|
| Indoor | 500K | 150+ |
| Outdoor | 1M | 100+ |
| Complex | 2M+ | 60+ |

### Bottlenecks

1. **Sorting:** O(n log n) or O(n) with radix
2. **Rasterization:** Depends on overlap
3. **Backward pass:** Similar to forward

## Implementation Details

### CUDA Considerations

- Thread block = 16×16 = 256 threads
- Shared memory for Gaussian data
- Warp-level primitives for efficiency

### Numerical Stability

- Clamp α to avoid division by zero
- Log-space for very small transmittance
- Guard band for near-plane issues

## Comparison

| Method | Speed | Quality | Memory |
|--------|-------|---------|--------|
| Ray marching | Slow | High | High |
| Point splatting | Fast | Lower | Low |
| This rasterizer | Fast | High | Low |
