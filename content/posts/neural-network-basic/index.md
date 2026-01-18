---
title: "Neural Network Basic"
date: 2024-08-01
description: "Fundamental concepts of neural network training"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Neural Network", "Gradient Descent"]
draft: false
---

{{< katex >}}

## Core Principle

Neural network training operates on a simple principle: **minimize the gap between network output and ground truth**.

$$
\text{Goal: } \min_{\theta} L(f_\theta(x), y)
$$

The narrower this quantified gap (loss), the closer to accuracy.

## Network Structure

```
Input Layer → Hidden Layers → Output Layer
    x      →    h₁, h₂...  →      ŷ
```

### Single Neuron

$$
y = \sigma(w \cdot x + b)
$$

Where:
- \(w\): weights
- \(b\): bias
- \(\sigma\): activation function

## Learning Mechanism

### 1. Forward Pass

Input flows through network to produce output:

$$
\hat{y} = f_\theta(x)
$$

### 2. Loss Calculation

Measure error between prediction and target:

$$
L = \frac{1}{n}\sum_{i=1}^{n}(y_i - \hat{y}_i)^2 \quad \text{(MSE)}
$$

### 3. Backward Pass (Backpropagation)

Calculate gradients using chain rule:

$$
\frac{\partial L}{\partial w} = \frac{\partial L}{\partial \hat{y}} \cdot \frac{\partial \hat{y}}{\partial w}
$$

### 4. Weight Update

Adjust weights to reduce loss:

$$
w_{new} = w_{old} - \eta \cdot \frac{\partial L}{\partial w}
$$

Where \(\eta\) is the learning rate.

## Gradient Descent

The optimization algorithm that drives learning:

```
Repeat until convergence:
    1. Compute gradient of loss
    2. Update weights in opposite direction
    3. Check if loss decreased
```

### Variants

| Type | Description | Batch Size |
|------|-------------|------------|
| Batch GD | All samples | N |
| Stochastic GD | One sample | 1 |
| Mini-batch GD | Subset | 32-256 |

## Activation Functions

| Function | Formula | Use Case |
|----------|---------|----------|
| Sigmoid | \(\frac{1}{1+e^{-x}}\) | Binary output |
| Tanh | \(\frac{e^x - e^{-x}}{e^x + e^{-x}}\) | Hidden layers |
| ReLU | \(\max(0, x)\) | Deep networks |
| Softmax | \(\frac{e^{x_i}}{\sum e^{x_j}}\) | Multi-class |

## Training Loop

```python
for epoch in range(epochs):
    for batch in data_loader:
        # Forward
        output = model(batch.x)
        loss = criterion(output, batch.y)

        # Backward
        optimizer.zero_grad()
        loss.backward()

        # Update
        optimizer.step()
```

## Key Concepts

1. **Loss Function** - Quantifies prediction error
2. **Gradient** - Direction of steepest increase
3. **Learning Rate** - Step size for updates
4. **Epoch** - One pass through entire dataset
5. **Batch** - Subset of data for one update
