---
title: "Zero-shot Learning / Few-shot Learning"
date: 2024-08-01
description: "Learning new classes with minimal or no training examples"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Transfer Learning", "Foundation Models"]
draft: false
---

## Overview

Zero-shot and Few-shot learning enable models to recognize new categories with minimal training data, mimicking how humans rapidly acquire knowledge.

## Zero-shot Learning

**Definition:** Using pre-trained Foundation Models to classify entirely new categories **without any direct training examples**.

```
Foundation Model → Inference on Unseen Classes → Prediction
     (pre-trained)       (no examples)
```

### How It Works

The model leverages learned semantic relationships to infer about completely new objects or concepts.

**Example:**
- Training: Dog, Cat, Horse images
- Inference: Classify "Zebra" (never seen)
- Method: Model knows "striped horse-like animal" → Zebra

### Three Training Approaches

#### 1. Embedding Space Learning

Connect objects and meanings in abstract space to infer relationships to unseen classes.

```
Visual Features ──┐
                  ├──→ Shared Embedding Space ──→ Classification
Semantic Features ─┘
```

#### 2. Attribute-based Learning

Focus on detailed properties and semantic attributes rather than general features.

| Animal | Stripes | Four-legged | Domestic |
|--------|---------|-------------|----------|
| Cat | No | Yes | Yes |
| Zebra | Yes | Yes | No |

Model learns: "Striped + Four-legged + Wild" → Zebra

#### 3. Text-Image Linking (CLIP-style)

Combine image data with textual descriptions.

```
Image Encoder ──────┐
                    ├──→ Similarity Matching ──→ Classification
Text Encoder ───────┘
    "A photo of a zebra"
```

## Few-shot Learning

**Definition:** Using transfer learning with **minimal labeled data** (typically 1-5 examples per class) to adapt to new classes.

```
Foundation Model → Fine-tune with K examples → New Classifier
                        (K = 1~5)
```

### Types

| Type | Examples per Class | Description |
|------|-------------------|-------------|
| 1-shot | 1 | Single example per class |
| 5-shot | 5 | Five examples per class |
| N-shot | N | N examples per class |

### Advantages

- **Reduced annotation cost** - Minimal labeling required
- **Faster training** - Quick adaptation
- **Practical** - Real-world scenarios often have limited data

### Common Approaches

#### 1. Metric Learning

Learn a distance function to compare query images with support examples.

```
Support Set (K examples) ──┐
                           ├──→ Distance Metric ──→ Classification
Query Image ───────────────┘
```

#### 2. Meta-Learning (Learning to Learn)

Train model to quickly adapt to new tasks.

```
Task 1: Learn cat vs dog
Task 2: Learn car vs bike
   ...
New Task: Learn zebra vs giraffe (with few examples)
```

#### 3. Prototypical Networks

Compute class prototypes from support examples.

```python
# Prototype = mean of support embeddings
prototype_c = mean(embeddings[class_c])

# Classify by nearest prototype
prediction = argmin(distance(query, prototypes))
```

## Comparison

| Aspect | Zero-shot | Few-shot |
|--------|-----------|----------|
| Training examples | 0 | 1-5 per class |
| Relies on | Semantic knowledge | Example similarity |
| Flexibility | High | Medium |
| Accuracy | Lower | Higher |
| Data requirements | None | Minimal |

## Applications

- **Image classification** with new categories
- **Natural language processing** tasks
- **Medical imaging** with rare conditions
- **Robotics** for new object manipulation
- **Recommendation systems** for new items
