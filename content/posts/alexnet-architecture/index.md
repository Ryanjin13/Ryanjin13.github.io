---
title: "AlexNet Architecture (2012)"
date: 2025-05-20
description: "AlexNet의 6가지 핵심 혁신"
categories: ["2D Vision"]
tags: ["CNN", "Object Detection", "Deep Learning History"]
draft: false
---

## AlexNet (2012)

2012년 ImageNet 대회에서 우승하며 딥러닝 시대를 연 AlexNet의 핵심 혁신을 정리합니다.

## 6가지 핵심 혁신

### 1. Deep Architecture

```
5 Convolutional Layers + 3 Fully Connected Layers
총 60 million 파라미터
```

당시로서는 획기적으로 깊은 네트워크 구조.

### 2. ReLU Activation

**최초로 ReLU를 주요 CNN에 적용** (기존: tanh/sigmoid)

| Activation | 문제점 |
|------------|--------|
| Sigmoid/Tanh | Vanishing gradient |
| **ReLU** | 빠른 학습, gradient 유지 |

### 3. Dropout Regularization

```
FC layers에 50% Dropout 적용
```

과적합 방지를 위한 당시 혁신적인 정규화 기법.

### 4. GPU Training

```
2x GTX 580 GPU로 병렬 학습
```

대규모 네트워크 학습을 가능하게 한 하드웨어 활용.

### 5. Data Augmentation

- Image translations (이동)
- Horizontal reflections (좌우 반전)
- PCA color augmentation (색상 변환)

학습 데이터를 인위적으로 확장하여 일반화 성능 향상.

### 6. Local Response Normalization (LRN)

ReLU 이후 적용된 정규화 기법.

> 이후 **Batch Normalization**으로 대체됨.

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
