---
title: "Layer Normalization"
date: 2025-06-15
description: "Concept and formula of Layer Normalization"
categories: ["2D Vision"]
tags: ["CNN", "Normalization", "Deep Learning"]
draft: false
---

{{< katex >}}

## What is Layer Normalization?

Layer Normalization is a normalization technique that adjusts **mean to 0 and variance to 1** for all values in a specific layer.

## Formula

$$
\hat{x}_i = \frac{x_i - \mu}{\sqrt{\sigma^2 + \epsilon}} \tag{1}
$$

Where:
- \(\mu\): Mean of all values in the layer
- \(\sigma^2\): Variance of all values in the layer
- \(\epsilon\): Small value for numerical stability

$$
\mu = \frac{1}{H} \sum_{i=1}^{H} x_i \tag{2}
$$

$$
\sigma^2 = \frac{1}{H} \sum_{i=1}^{H} (x_i - \mu)^2 \tag{3}
$$

## Learnable Parameters

After normalization, apply scale (\(\gamma\)) and shift (\(\beta\)) parameters:

$$
y_i = \gamma \hat{x}_i + \beta \tag{4}
$$

## Key Benefits

| Benefit | Description |
|---------|-------------|
| **Activation Stabilization** | Prevents values from becoming too large or small during forward propagation |
| **Gradient Protection** | Mitigates gradient explosion/vanishing problems |

## Batch Norm vs Layer Norm

| | Batch Norm | Layer Norm |
|---|------------|------------|
| Normalization axis | Batch direction | Feature direction |
| Batch size dependency | Yes | No |
| RNN/Transformer | Not suitable | Suitable |
