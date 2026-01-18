---
title: "Why MSE"
date: 2025-03-30
description: "Geometric interpretation of Mean Square Error"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Loss Function"]
draft: false
---

{{< katex >}}

## Mean Square Error

$$
MSE = \frac{1}{n} \sum_{i=1}^{n} (y_i - \hat{y}_i)^2
$$

## Geometric Interpretation

MSE is fundamentally a **distance measurement** in high-dimensional vector space.

### Vector Space Perspective

The number of classifications builds the dimensions (Vector) of our space.

- **Prediction vector**: \(\hat{y} = (\hat{y}_1, \hat{y}_2, ..., \hat{y}_n)\)
- **Label vector**: \(y = (y_1, y_2, ..., y_n)\)

### Euclidean Distance

MSE calculates the **squared Euclidean distance** between prediction and label vectors:

$$
d^2 = \sum_{i=1}^{n} (y_i - \hat{y}_i)^2
$$

This is an extension of the **Pythagorean theorem** to n-dimensions:

**2D case:**
$$
d = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2}
$$

**n-D case:**
$$
d = \sqrt{\sum_{i=1}^{n} (y_i - \hat{y}_i)^2}
$$

## Why Squared?

| Reason | Explanation |
|--------|-------------|
| **Differentiable** | Smooth gradient for optimization |
| **Penalizes large errors** | Outliers have greater impact |
| **Always positive** | No sign issues |
| **Geometric meaning** | Euclidean distance in vector space |

## Summary

MSE is not just a statistical metric—it represents the geometric distance between predictions and ground truth in high-dimensional space, connecting deep learning to classical Euclidean geometry.
