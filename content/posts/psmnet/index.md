---
title: "PSMNet: Pyramid Stereo Matching Network"
date: 2024-06-21
description: "Deep learning approach for stereo depth estimation"
categories: ["3D Vision"]
tags: ["Stereo Matching", "Depth Estimation", "Deep Learning"]
draft: false
---

{{< katex >}}

## Overview

PSMNet (Pyramid Stereo Matching Network) is a deep learning architecture for stereo matching that uses spatial pyramid pooling and 3D convolutions to estimate disparity maps from stereo image pairs.

## Problem: Stereo Matching

Given left and right images, find corresponding pixels to estimate depth:

$$
Z = \frac{f \cdot B}{d}
$$

Where:
- \\(Z\\): Depth
- \\(f\\): Focal length
- \\(B\\): Baseline (camera separation)
- \\(d\\): Disparity

## Architecture

```
Left Image  ──┬──→ Feature Extraction ──┬──→ Cost Volume ──→ 3D CNN ──→ Disparity
              │     (ResNet + SPP)      │    Construction    Stacked    Regression
Right Image ──┘                         └────────────────────────────────────────→
```

### 1. Feature Extraction

**Backbone:** Modified ResNet with dilated convolutions

**Spatial Pyramid Pooling (SPP):**

Pool at multiple scales to capture global context:

```
Input Feature Map
       ↓
┌──────┼──────┬──────┬──────┐
│ 1×1  │ 2×2  │ 3×3  │ 6×6  │  Pooling sizes
└──────┴──────┴──────┴──────┘
       ↓ Upsample & Concatenate
  Multi-scale Features
```

### 2. Cost Volume Construction

Create 4D cost volume by concatenating features at different disparities:

$$
C(d, h, w) = \text{concat}(F_L(h, w), F_R(h, w-d))
$$

Dimensions: \\(D_{max} \times H \times W \times 2C\\)

Where \\(D_{max}\\) is maximum disparity.

### 3. 3D CNN Aggregation

**Stacked Hourglass Architecture:**

```
Cost Volume
     ↓
┌─────────────────────┐
│  3D Conv Encoder    │
│    (Downsample)     │
├─────────────────────┤
│  3D Conv Decoder    │    × 3 stacks
│    (Upsample)       │
├─────────────────────┤
│  Skip Connections   │
└─────────────────────┘
     ↓
Regularized Cost Volume
```

### 4. Disparity Regression

Soft argmax for sub-pixel accuracy:

$$
\hat{d} = \sum_{d=0}^{D_{max}} d \cdot \sigma(-c_d)
$$

Where \\(\sigma\\) is softmax over cost values.

## Loss Function

Smooth L1 loss with multi-scale supervision:

$$
L = \sum_{s} \lambda_s \cdot \text{SmoothL1}(\hat{d}_s - d_{gt})
$$

$$
\text{SmoothL1}(x) = \begin{cases}
0.5x^2 & \text{if } |x| < 1 \\
|x| - 0.5 & \text{otherwise}
\end{cases}
$$

## Key Innovations

| Component | Benefit |
|-----------|---------|
| Spatial Pyramid Pooling | Global context awareness |
| 4D Cost Volume | Explicit disparity modeling |
| Stacked Hourglass | Multi-scale regularization |
| Soft Argmax | Sub-pixel accuracy |

## Performance

On KITTI 2015 benchmark:

| Metric | PSMNet |
|--------|--------|
| D1-all | 2.32% |
| Runtime | ~0.4s |

## Comparison with Other Methods

| Method | Approach | Accuracy |
|--------|----------|----------|
| MC-CNN | Patch matching | Lower |
| GC-Net | 3D CNN | Good |
| PSMNet | SPP + 3D CNN | Better |
| GA-Net | Guided aggregation | Best |

## Implementation Details

- Input resolution: 256 × 512
- Max disparity: 192
- Batch size: 12
- Optimizer: Adam (lr = 0.001)
- Training epochs: 10 (SceneFlow) + 300 (KITTI)

## Applications

1. Autonomous driving depth perception
2. Robot navigation
3. 3D reconstruction
4. Augmented reality
