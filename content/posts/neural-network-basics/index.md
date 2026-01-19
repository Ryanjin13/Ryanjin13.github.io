---
title: "Neural Network Training Fundamentals"
date: 2024-08-01
description: "Understanding how neural networks learn through loss minimization"
categories: ["AI"]
tags: ["Neural Networks", "Deep Learning", "Machine Learning", "Gradient Descent"]
draft: false
---

{{< katex >}}

## Overview

Neural networks learn by minimizing the gap between their predictions and the correct answers. This fundamental principle drives all deep learning training.

## The Core Principle

> As the numerical gap between neural network output and correct answers narrows, accuracy improves.

$$
\text{Learning Goal: } \min_{\theta} \mathcal{L}(f_\theta(x), y)
$$

Where:
- \\(\theta\\): Network parameters (weights and biases)
- \\(f_\theta(x)\\): Network output given input \\(x\\)
- \\(y\\): Correct answer (ground truth)
- \\(\mathcal{L}\\): Loss function

## Learning Mechanism

### Step 1: Forward Pass

Input flows through the network to produce output:

```
Input (x) ──▶ Hidden Layers ──▶ Output (ŷ)
                   │
              Weights (θ)
```

$$
\hat{y} = f_\theta(x) = \sigma(W_n \cdot \sigma(W_{n-1} \cdot ... \sigma(W_1 \cdot x + b_1) ... + b_{n-1}) + b_n)
$$

### Step 2: Compute Loss

Measure the difference between prediction and truth:

**Common Loss Functions:**

| Task | Loss Function |
|------|--------------|
| Regression | MSE: \\(\frac{1}{n}\sum(y - \hat{y})^2\\) |
| Classification | Cross-Entropy: \\(-\sum y \log(\hat{y})\\) |

### Step 3: Backpropagation

Calculate how each weight affects the loss:

$$
\frac{\partial \mathcal{L}}{\partial w_{ij}} = \frac{\partial \mathcal{L}}{\partial \hat{y}} \cdot \frac{\partial \hat{y}}{\partial w_{ij}}
$$

### Step 4: Update Weights

Move weights in the direction that reduces loss:

$$
\theta_{new} = \theta_{old} - \eta \cdot \nabla_\theta \mathcal{L}
$$

Where \\(\eta\\) is the learning rate.

## The Gradient Direction

### Key Insight

The gradient tells us which direction to adjust weights:

- **Positive gradient**: Increasing the weight increases loss → Decrease the weight
- **Negative gradient**: Increasing the weight decreases loss → Increase the weight

```
Loss
  │
  │    ╲        ╱
  │     ╲      ╱
  │      ╲    ╱
  │       ╲  ╱
  │        ╲╱
  │         ●  ← Goal: Find minimum
  └────────────────── Weight

  Gradient descent follows the slope downward
```

## Iterative Refinement

Neural network training is an iterative process:

```
┌──────────────────────────────────────────────┐
│                                              │
│  ┌─────────┐    ┌──────────┐    ┌─────────┐ │
│  │ Forward │───▶│ Compute  │───▶│Backward │ │
│  │  Pass   │    │   Loss   │    │  Pass   │ │
│  └─────────┘    └──────────┘    └────┬────┘ │
│       ▲                              │      │
│       │         ┌──────────┐         │      │
│       └─────────│  Update  │◀────────┘      │
│                 │ Weights  │                │
│                 └──────────┘                │
│                                              │
│            Repeat until converged            │
└──────────────────────────────────────────────┘
```

### Training Progress

| Epoch | Loss | Accuracy |
|-------|------|----------|
| 1 | 2.45 | 15% |
| 10 | 1.23 | 45% |
| 50 | 0.42 | 78% |
| 100 | 0.15 | 92% |
| 200 | 0.08 | 97% |

## Gradient Descent Variants

### Batch Gradient Descent

Use entire dataset for each update:

$$
\theta = \theta - \eta \cdot \frac{1}{N}\sum_{i=1}^{N} \nabla_\theta \mathcal{L}(x_i, y_i)
$$

**Pros**: Stable convergence
**Cons**: Slow for large datasets

### Stochastic Gradient Descent (SGD)

Update after each sample:

$$
\theta = \theta - \eta \cdot \nabla_\theta \mathcal{L}(x_i, y_i)
$$

**Pros**: Fast updates
**Cons**: Noisy gradients

### Mini-Batch Gradient Descent

Update after small batches (best of both):

$$
\theta = \theta - \eta \cdot \frac{1}{B}\sum_{i=1}^{B} \nabla_\theta \mathcal{L}(x_i, y_i)
$$

**Typical batch sizes**: 32, 64, 128, 256

## Advanced Optimizers

### Momentum

Add velocity to smooth updates:

$$
v_t = \beta v_{t-1} + \nabla_\theta \mathcal{L}
$$
$$
\theta = \theta - \eta \cdot v_t
$$

### Adam (Adaptive Moment Estimation)

Combine momentum with adaptive learning rates:

$$
m_t = \beta_1 m_{t-1} + (1-\beta_1) \nabla_\theta \mathcal{L}
$$
$$
v_t = \beta_2 v_{t-1} + (1-\beta_2) (\nabla_\theta \mathcal{L})^2
$$
$$
\theta = \theta - \eta \cdot \frac{\hat{m}_t}{\sqrt{\hat{v}_t} + \epsilon}
$$

## Hyperparameters

Key settings that affect learning:

| Hyperparameter | Typical Range | Effect |
|----------------|---------------|--------|
| Learning rate | 1e-4 to 1e-1 | Step size |
| Batch size | 16 to 512 | Gradient noise |
| Epochs | 10 to 1000 | Training duration |
| Momentum | 0.9 to 0.99 | Update smoothing |

## Avoiding Common Problems

### Underfitting

Model too simple to capture patterns:
- Increase model capacity
- Train longer
- Add features

### Overfitting

Model memorizes training data:
- Add regularization (L2, dropout)
- Increase training data
- Early stopping

### Vanishing Gradients

Gradients become too small in deep networks:
- Use ReLU activation
- Batch normalization
- Residual connections

## Summary

Neural network training fundamentals:
1. **Forward pass**: Compute predictions
2. **Loss calculation**: Measure error
3. **Backward pass**: Compute gradients
4. **Weight update**: Adjust parameters
5. **Repeat**: Until loss is minimized

The key insight: Networks learn by iteratively moving weights in directions that reduce the loss function.

