---
title: "Depthwise Convolution"
date: 2025-04-07
description: "Depthwise Separable Convolution for efficient neural networks"
categories: ["2D Vision"]
tags: ["CNN", "MobileNet", "Model Compression", "Edge AI"]
draft: false
---

{{< katex >}}

## Overview

Depthwise Convolution is a key technique used in efficient neural network architectures like MobileNet-V2.

## Standard Convolution vs Depthwise Separable Convolution

### Standard Convolution

Traditional convolution processes all input channels together with a single kernel.

- Input: \(H \times W \times C_{in}\)
- Kernel: \(K \times K \times C_{in} \times C_{out}\)
- Operations: \(H \times W \times K^2 \times C_{in} \times C_{out}\)

### Depthwise Separable Convolution

Splits convolution into two steps:

**1. Depthwise Convolution**
- Applies separate kernel to each input channel individually
- Kernel: \(K \times K \times 1\) per channel
- Operations: \(H \times W \times K^2 \times C_{in}\)

**2. Pointwise Convolution (1×1 Conv)**
- Combines channel information
- Kernel: \(1 \times 1 \times C_{in} \times C_{out}\)
- Operations: \(H \times W \times C_{in} \times C_{out}\)

## Computational Cost Comparison

$$
\text{Reduction Ratio} = \frac{1}{C_{out}} + \frac{1}{K^2}
$$

For typical values (\(K=3\), \(C_{out}=256\)):

$$
\frac{1}{256} + \frac{1}{9} \approx 0.115
$$

**~8-9x fewer operations** compared to standard convolution.

## Key Benefits

| Benefit | Description |
|---------|-------------|
| **Reduced Computation** | Significantly fewer multiply-add operations |
| **Smaller Model Size** | Fewer parameters to store |
| **Edge Deployment** | Enables deployment on embedded systems |
| **Mobile Optimization** | Core technique in MobileNet series |

## Applications

- MobileNet-V1, V2, V3
- EfficientNet
- Edge AI / IoT devices
- Real-time mobile applications
