---
title: "Zero-Shot and Few-Shot Learning"
date: 2024-08-02
description: "Understanding learning paradigms that minimize data requirements"
categories: ["AI"]
tags: ["Zero-Shot Learning", "Few-Shot Learning", "Transfer Learning", "Foundation Models"]
draft: false
---

{{< katex >}}

## Overview

Zero-shot and few-shot learning represent paradigm shifts in machine learning, enabling models to classify new categories with minimal or no training examples.

## Zero-Shot Learning (ZSL)

### Definition

Zero-shot learning enables classification of entirely new categories without any training examples, using knowledge from pre-trained foundation models.

$$
P(y_{new}|x) = f(x; \theta_{foundation}, \text{semantic\_info})
$$

### Training Methodologies

#### 1. Embedding Space Learning

Maps images and semantic information into a shared conceptual space:

$$
\text{similarity}(x, c) = \cos(f_{image}(x), f_{semantic}(c))
$$

#### 2. Attribute-Based Learning

Uses detailed semantic properties to describe classes:

| Class | Furry | Has Wings | Four Legs |
|-------|-------|-----------|-----------|
| Cat | Yes | No | Yes |
| Bird | No | Yes | No |
| Horse | Yes | No | Yes |

#### 3. Text-Image Linking

CLIP-style contrastive training:

$$
\mathcal{L} = -\frac{1}{N}\sum_{i=1}^{N}\log\frac{\exp(sim(I_i, T_i)/\tau)}{\sum_{j=1}^{N}\exp(sim(I_i, T_j)/\tau)}
$$

## Few-Shot Learning (FSL)

### Definition

Few-shot learning enables learning new categories from only 1-5 examples.

### Approaches

#### Meta-Learning (Learning to Learn)

$$
\theta^* = \arg\min_\theta \sum_{\mathcal{T}_i} \mathcal{L}(\mathcal{T}_i; \theta)
$$

#### Metric Learning

Prototypical Networks:

$$
P(y=k|x) = \frac{\exp(-d(f(x), c_k))}{\sum_{k'}\exp(-d(f(x), c_{k'}))}
$$

## Comparison

| Aspect | Zero-Shot | Few-Shot |
|--------|-----------|----------|
| Training Examples | 0 | 1-5 |
| Auxiliary Info | Required | Optional |
| Flexibility | High | Medium |
| Accuracy | Lower | Higher |

## Summary

Key takeaways:
1. Zero-shot: No examples needed, relies on semantic knowledge
2. Few-shot: 1-5 examples enable rapid adaptation
3. Foundation models enable both paradigms through transfer
4. Applications reduce data annotation burden significantly
