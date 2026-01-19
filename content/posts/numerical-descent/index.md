---
title: "Numerical Descent"
date: 2024-06-21
description: "Gradient descent optimization methods for neural network training"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Optimization", "Gradient Descent"]
draft: false
---

{{< katex >}}

## Overview

Numerical descent methods are iterative optimization algorithms used to find local minima of differentiable functions. In deep learning, these methods minimize loss functions to train neural networks.

## Gradient Descent

The fundamental update rule:

$$
\theta_{t+1} = \theta_t - \eta \nabla_\theta L(\theta_t)
$$

Where:
- \\(\theta\\): Model parameters
- \\(\eta\\): Learning rate
- \\(L\\): Loss function
- \\(\nabla_\theta L\\): Gradient of loss with respect to parameters

## Types of Gradient Descent

### Batch Gradient Descent

Uses entire dataset for each update:

$$
\theta = \theta - \eta \cdot \nabla_\theta L(\theta; X, Y)
$$

| Pros | Cons |
|------|------|
| Stable convergence | Slow for large datasets |
| Guaranteed descent | High memory usage |

### Stochastic Gradient Descent (SGD)

Updates using single sample:

$$
\theta = \theta - \eta \cdot \nabla_\theta L(\theta; x_i, y_i)
$$

| Pros | Cons |
|------|------|
| Fast updates | Noisy gradients |
| Can escape local minima | Unstable convergence |

### Mini-batch Gradient Descent

Uses subset of data:

$$
\theta = \theta - \eta \cdot \nabla_\theta L(\theta; X_{batch}, Y_{batch})
$$

Typical batch sizes: 32, 64, 128, 256

## Advanced Optimizers

### Momentum

Accumulates velocity in consistent gradient directions:

$$
v_t = \gamma v_{t-1} + \eta \nabla_\theta L(\theta_t)
$$

$$
\theta_{t+1} = \theta_t - v_t
$$

Where \\(\gamma\\) is momentum coefficient (typically 0.9).

### RMSprop

Adapts learning rate per parameter:

$$
E[g^2]_t = \gamma E[g^2]_{t-1} + (1-\gamma) g_t^2
$$

$$
\theta_{t+1} = \theta_t - \frac{\eta}{\sqrt{E[g^2]_t + \epsilon}} g_t
$$

### Adam

Combines momentum and adaptive learning rates:

$$
m_t = \beta_1 m_{t-1} + (1-\beta_1) g_t
$$

$$
v_t = \beta_2 v_{t-1} + (1-\beta_2) g_t^2
$$

Bias-corrected estimates:

$$
\hat{m}_t = \frac{m_t}{1-\beta_1^t}, \quad \hat{v}_t = \frac{v_t}{1-\beta_2^t}
$$

Update:

$$
\theta_{t+1} = \theta_t - \frac{\eta}{\sqrt{\hat{v}_t} + \epsilon} \hat{m}_t
$$

Default values: \\(\beta_1 = 0.9\\), \\(\beta_2 = 0.999\\), \\(\epsilon = 10^{-8}\\)

## Comparison

| Optimizer | Adaptive LR | Momentum | Use Case |
|-----------|-------------|----------|----------|
| SGD | No | Optional | Simple, well-tuned |
| RMSprop | Yes | No | RNNs, non-stationary |
| Adam | Yes | Yes | Default choice |
| AdamW | Yes | Yes | With weight decay |

## Learning Rate Schedules

### Step Decay

$$
\eta_t = \eta_0 \cdot \gamma^{\lfloor t/s \rfloor}
$$

### Exponential Decay

$$
\eta_t = \eta_0 \cdot e^{-kt}
$$

### Cosine Annealing

$$
\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})(1 + \cos(\frac{t}{T}\pi))
$$

## Convergence Considerations

1. **Learning rate too high:** Divergence or oscillation
2. **Learning rate too low:** Slow convergence, stuck in local minima
3. **Gradient clipping:** Prevents exploding gradients

$$
g = \min\left(1, \frac{\text{threshold}}{\|g\|}\right) \cdot g
$$
