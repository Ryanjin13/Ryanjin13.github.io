---
title: "Cross-Entropy and MSE Principles"
date: 2025-03-30
description: "Cross-Entropy와 MSE 손실 함수의 기하학적 원리 비교"
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

- **MSE**: Euclidean distance (L2 norm) 기반
- **Cross-Entropy**: "Information Geometry" 기반

### Computational Approach

| MSE | Cross-Entropy |
|-----|---------------|
| 평면(2D plane) 위에서 거리 계산 | 곡면(probability manifold) 위에서 방향성 계산 |
| Additive error | Multiplicative probability |

### One-Hot Encoding Impact

One-hot encoding 상황에서:

- **MSE**: 모든 클래스에 대해 에러 계산 (불필요한 연산 포함)
$$
(0 - q_{incorrect})^2
$$

- **Cross-Entropy**: 정답 클래스의 확률만 계산
$$
-\log(q_{correct})
$$

### Convergence Behavior

학습이 진행되어 정답 확률이 1에 가까워지면:

$$
\lim_{q_{correct} \to 1} -\log(q_{correct}) = -\log(1) = 0
$$

Cross-Entropy 손실이 자연스럽게 0으로 수렴합니다.
