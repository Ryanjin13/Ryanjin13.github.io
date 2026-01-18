---
title: "Deep Learning Point"
date: 2025-03-25
description: "Key concepts in deep learning: R² score and input normalization"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Normalization"]
draft: false
---

{{< katex >}}

## R² Score (Coefficient of Determination)

R² measures how well the model explains variance in data.

$$
R^2 = 1 - \frac{SS_{res}}{SS_{tot}} = 1 - \frac{\sum(y_i - \hat{y}_i)^2}{\sum(y_i - \bar{y})^2}
$$

### Interpretation

| R² Value | Quality |
|----------|---------|
| > 0.8 | Very good model |
| 0.6 - 0.8 | Acceptable model |
| 0.4 - 0.6 | Needs improvement |
| < 0.4 | Requires significant enhancement |

## Why Normalize Input Data?

### 1. Prevents Gradient Explosion

Large input values cause unstable gradients during backpropagation.

**Example with Chain Rule:**

For a simple layer: \(y = wx + b\), the gradient is:

$$
\frac{\partial L}{\partial w} = \frac{\partial L}{\partial y} \cdot x
$$

| Input x | Gradient |
|---------|----------|
| x = 2 | -0.4 (stable) |
| x = 1000 | -200 (unstable) |

Large inputs amplify gradients across layers, causing explosion.

### 2. Avoids Gradient Saturation

Sigmoid function:

$$
\sigma(x) = \frac{1}{1 + e^{-x}}
$$

**Problem:** At extreme values, sigmoid saturates:
- \(\sigma(10) \approx 0.99995\)
- \(\sigma(-10) \approx 0.00005\)

**Derivative approaches zero:**

$$
\sigma'(x) = \sigma(x)(1 - \sigma(x))
$$

When \(\sigma(x) \approx 0\) or \(\sigma(x) \approx 1\):

$$
\sigma'(x) \approx 0 \quad \text{(vanishing gradient)}
$$

### 3. Faster Convergence

Normalized inputs:
- Keep values in active region of activation functions
- Enable effective weight updates
- Reduce training time

## Normalization Methods

| Method | Formula | Use Case |
|--------|---------|----------|
| **Min-Max** | \(\frac{x - x_{min}}{x_{max} - x_{min}}\) | Bounded range [0, 1] |
| **Z-Score** | \(\frac{x - \mu}{\sigma}\) | Gaussian distribution |
| **Batch Norm** | Per-batch normalization | During training |
| **Layer Norm** | Per-layer normalization | RNN/Transformers |

## Summary

Input normalization is critical for:
1. **Stable training** - prevents gradient explosion
2. **Effective learning** - avoids saturation zones
3. **Faster convergence** - efficient weight updates
