---
title: "Cross-Entropy and MSE Principles"
date: 2025-03-30
description: "Geometric principles comparison between Cross-Entropy and MSE loss functions"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic"]
draft: false
---

{{< katex >}}

## Loss Functions

**Mean Square Error (MSE):**

$$
L_{MSE} = \frac{1}{n} \sum_{i} (y_i - q_i)^2 \tag{1}
$$

**Cross-Entropy (CE):**

$$
L_{CE} = -\sum_{i} y_i \log(q_i) \tag{2}
$$

## Key Differences

### Geometric Foundation

- **MSE**: Based on Euclidean distance (L2 norm)
- **Cross-Entropy**: Based on "Information Geometry"

### Computational Approach

| MSE | Cross-Entropy |
|-----|---------------|
| Distance calculation on flat 2D plane | Directional calculation on curved probability manifold |
| Additive error | Multiplicative probability |

### One-Hot Encoding Impact

With one-hot encoding:

- **MSE**: Computes errors for all classes (including unnecessary calculations)
$$
(0 - q_{incorrect})^2
$$

- **Cross-Entropy**: Only computes probability of correct class
$$
-\log(q_{correct})
$$

### Convergence Behavior

As training progresses and correct probability approaches 1:

$$
\lim_{q_{correct} \to 1} -\log(q_{correct}) = -\log(1) = 0
$$

Cross-Entropy loss naturally converges to 0.
