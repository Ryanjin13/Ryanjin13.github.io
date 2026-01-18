---
title: "AlexNet Architecture (2012)"
date: 2025-05-20
description: "6 key innovations of AlexNet"
categories: ["2D Vision"]
tags: ["CNN", "Object Detection", "Deep Learning History"]
draft: false
---

## AlexNet (2012)

AlexNet won the ImageNet competition in 2012 and ushered in the deep learning era. Here are its key innovations.

## 6 Key Innovations

### 1. Deep Architecture

```
5 Convolutional Layers + 3 Fully Connected Layers
Total: 60 million parameters
```

A groundbreaking network depth for its time.

### 2. ReLU Activation

**First major CNN to use ReLU** (replacing tanh/sigmoid)

| Activation | Problem |
|------------|---------|
| Sigmoid/Tanh | Vanishing gradient |
| **ReLU** | Fast training, gradient preservation |

### 3. Dropout Regularization

```
50% Dropout applied to FC layers
```

A novel regularization technique for preventing overfitting.

### 4. GPU Training

```
Trained on 2x GTX 580 GPUs in parallel
```

Hardware acceleration enabling large-scale network training.

### 5. Data Augmentation

- Image translations
- Horizontal reflections
- PCA color augmentation

Artificially expanding training data for better generalization.

### 6. Local Response Normalization (LRN)

Normalization technique applied after ReLU.

> Later replaced by **Batch Normalization**.

## Architecture Summary

| Layer | Output Size | Details |
|-------|-------------|---------|
| Input | 224×224×3 | RGB Image |
| Conv1 | 55×55×96 | 11×11, stride 4 |
| Conv2 | 27×27×256 | 5×5 |
| Conv3 | 13×13×384 | 3×3 |
| Conv4 | 13×13×384 | 3×3 |
| Conv5 | 13×13×256 | 3×3 |
| FC6 | 4096 | Dropout 50% |
| FC7 | 4096 | Dropout 50% |
| FC8 | 1000 | Softmax |
