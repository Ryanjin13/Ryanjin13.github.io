---
title: "Layer Normalization"
date: 2025-06-15
description: "Layer Normalization의 개념과 수식"
categories: ["2D Vision"]
tags: ["CNN", "Normalization", "Deep Learning"]
draft: false
---

{{< katex >}}

## Layer Normalization이란?

Layer Normalization은 특정 레이어의 모든 값에 대해 **평균을 0으로, 분산을 1로** 조정하는 정규화 기법입니다.

## 수식

$$
\hat{x}_i = \frac{x_i - \mu}{\sqrt{\sigma^2 + \epsilon}} \tag{1}
$$

여기서:
- \(\mu\): 레이어 내 모든 값의 평균
- \(\sigma^2\): 레이어 내 모든 값의 분산
- \(\epsilon\): 수치 안정성을 위한 작은 값

$$
\mu = \frac{1}{H} \sum_{i=1}^{H} x_i \tag{2}
$$

$$
\sigma^2 = \frac{1}{H} \sum_{i=1}^{H} (x_i - \mu)^2 \tag{3}
$$

## 학습 가능한 파라미터

정규화 후 scale(\(\gamma\))과 shift(\(\beta\)) 파라미터 적용:

$$
y_i = \gamma \hat{x}_i + \beta \tag{4}
$$

## 주요 효과

| 효과 | 설명 |
|------|------|
| **Activation 안정화** | Forward propagation에서 값이 너무 크거나 작아지는 것 방지 |
| **Gradient 보호** | Gradient explosion/vanishing 문제 완화 |

## Batch Norm vs Layer Norm

| | Batch Norm | Layer Norm |
|---|------------|------------|
| 정규화 축 | Batch 방향 | Feature 방향 |
| 배치 크기 의존 | O | X |
| RNN/Transformer | 부적합 | 적합 |
