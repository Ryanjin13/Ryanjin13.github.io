---
title: "RNN - LSTM - LLM Summary"
date: 2024-06-21
description: "Evolution from Recurrent Neural Networks to Large Language Models"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "RNN", "LSTM", "LLM", "Transformer"]
draft: false
---

{{< katex >}}

## Overview

The evolution of sequence modeling: RNN → LSTM → Transformer → LLM

## RNN (Recurrent Neural Network)

### Architecture

```
x_t → [Hidden State h_t] → y_t
           ↑ ↓
         h_{t-1}
```

### Equations

$$
h_t = \tanh(W_{hh} h_{t-1} + W_{xh} x_t + b_h)
$$

$$
y_t = W_{hy} h_t + b_y
$$

### Problems

| Issue | Description |
|-------|-------------|
| **Vanishing Gradient** | Gradients shrink exponentially over time |
| **Exploding Gradient** | Gradients grow exponentially |
| **Short Memory** | Difficulty with long sequences |

## LSTM (Long Short-Term Memory)

### Key Innovation: Gates

```
         ┌──────────────────────────────┐
x_t ────→│  Forget │ Input │ Output    │────→ h_t
h_{t-1} →│   Gate  │ Gate  │  Gate     │
         └──────────────────────────────┘
                      ↕
                  Cell State c_t
```

### Equations

**Forget Gate:**
$$
f_t = \sigma(W_f \cdot [h_{t-1}, x_t] + b_f)
$$

**Input Gate:**
$$
i_t = \sigma(W_i \cdot [h_{t-1}, x_t] + b_i)
$$

**Cell Update:**
$$
\tilde{c}_t = \tanh(W_c \cdot [h_{t-1}, x_t] + b_c)
$$

$$
c_t = f_t \odot c_{t-1} + i_t \odot \tilde{c}_t
$$

**Output Gate:**
$$
o_t = \sigma(W_o \cdot [h_{t-1}, x_t] + b_o)
$$

$$
h_t = o_t \odot \tanh(c_t)
$$

### Benefits

- Solves vanishing gradient via cell state highway
- Selective memory through gates
- Better long-term dependencies

## Transformer

### Key Innovation: Self-Attention

No recurrence - parallel processing of entire sequence.

**Attention:**
$$
\text{Attention}(Q, K, V) = \text{softmax}\left(\frac{QK^T}{\sqrt{d_k}}\right)V
$$

### Architecture

```
Input → Embedding → [Multi-Head Attention + FFN] × N → Output
                           ↓
                    Position Encoding
```

### Advantages over RNN/LSTM

| Aspect | RNN/LSTM | Transformer |
|--------|----------|-------------|
| Parallelization | Sequential | Fully parallel |
| Long-range | Difficult | Easy (direct attention) |
| Training speed | Slow | Fast |
| Scalability | Limited | Scales well |

## LLM (Large Language Model)

### Definition

Transformer-based models with billions of parameters trained on massive text corpora.

### Key Examples

| Model | Parameters | Organization |
|-------|------------|--------------|
| GPT-3 | 175B | OpenAI |
| GPT-4 | ~1.7T (est.) | OpenAI |
| LLaMA | 7B-70B | Meta |
| Claude | Unknown | Anthropic |
| PaLM | 540B | Google |

### Capabilities

- Text generation
- Question answering
- Summarization
- Translation
- Code generation
- Reasoning

### Training

1. **Pre-training:** Self-supervised on large corpus
2. **Fine-tuning:** Task-specific or instruction tuning
3. **RLHF:** Reinforcement Learning from Human Feedback

## Evolution Summary

```
RNN (1986)
  ↓ (vanishing gradient)
LSTM (1997)
  ↓ (sequential bottleneck)
Transformer (2017)
  ↓ (scale up)
LLM (2020+)
```
