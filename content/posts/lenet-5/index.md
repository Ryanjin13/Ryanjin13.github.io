---
title: "LeNet-5 (1998)"
date: 2025-04-01
description: "Architecture of LeNet-5, a foundational CNN"
categories: ["2D Vision"]
tags: ["CNN", "Deep Learning History", "Object Detection"]
draft: false
---

{{< katex >}}

## Overview

LeNet-5, proposed by Yann LeCun in 1998, is one of the foundational convolutional neural networks that pioneered modern deep learning architectures.

## Architecture

### Input Layer
- Input: 32×32 grayscale image

### C1: Convolutional Layer 1
- 6 kernels (5×5)
- Output: 6 feature maps of 28×28
- Parameters: \(6 \times (5 \times 5 + 1) = 156\)

### S2: Subsampling (Average Pooling)
Unlike modern max pooling, LeNet-5 uses **average pooling with learnable parameters**:

$$
y = \sigma(w \cdot avg(x) + b)
$$

- Output: 6 feature maps of 14×14

### C3: Convolutional Layer 2
- 16 kernels (5×5)
- Output: 16 feature maps of 10×10

### S4: Subsampling
- Output: 16 feature maps of 5×5

### C5: Convolutional Layer 3
- 120 kernels (5×5)
- Output: 120 units (fully connected)

### F6: Fully Connected
- 84 units

### Output: Gaussian Connections (RBF)

Unlike modern softmax, LeNet-5 uses **Radial Basis Function (RBF)**:

$$
y_i = \sum_j (x_j - w_{ij})^2
$$

The class with **minimum L2 distance** is the predicted output.

## Architecture Summary

| Layer | Type | Output Size | Parameters |
|-------|------|-------------|------------|
| Input | - | 32×32×1 | - |
| C1 | Conv 5×5 | 28×28×6 | 156 |
| S2 | Avg Pool | 14×14×6 | 12 |
| C3 | Conv 5×5 | 10×10×16 | 1,516 |
| S4 | Avg Pool | 5×5×16 | 32 |
| C5 | Conv 5×5 | 1×1×120 | 48,120 |
| F6 | FC | 84 | 10,164 |
| Output | RBF | 10 | 850 |

**Total Parameters: ~60,000**

## Historical Significance

LeNet-5 introduced several concepts still used today:
- Convolutional layers for feature extraction
- Pooling for spatial reduction
- Hierarchical feature learning

However, some design choices were later replaced:
- RBF output → Softmax
- Average pooling → Max pooling
- Sigmoid activation → ReLU
