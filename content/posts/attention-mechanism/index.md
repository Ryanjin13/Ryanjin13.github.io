---
title: "Attention Mechanism"
date: 2024-06-23
description: "Understanding the attention mechanism in deep learning"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic", "Attention", "Transformer", "NLP"]
draft: false
---

{{< katex >}}

## Overview

The attention mechanism allows neural networks to focus on relevant parts of the input when producing outputs. It revolutionized sequence-to-sequence models and led to the Transformer architecture.

## Encoder-Decoder Architecture

```
Input Sequence → [Encoder] → Context → [Decoder] → Output Sequence
```

### The Bottleneck Problem

Traditional seq2seq:
- Encoder compresses entire input to fixed-size context
- Long sequences lose information
- Decoder has limited access to input details

### Attention Solution

- Decoder can "look back" at all encoder states
- Weighted combination based on relevance
- Dynamic focus at each decoding step

## Attention Mechanism

### Components

**Query (Q):** What we're looking for
**Key (K):** What we're matching against
**Value (V):** What we retrieve

### Attention Function

$$
\text{Attention}(Q, K, V) = \text{softmax}\left(\frac{QK^T}{\sqrt{d_k}}\right)V
$$

Where \\(d_k\\) is the dimension of keys.

### Step by Step

1. **Score:** Compute similarity between query and keys
   $$
   e_{ij} = Q_i \cdot K_j
   $$

2. **Scale:** Divide by \\(\sqrt{d_k}\\) for stable gradients

3. **Normalize:** Apply softmax to get weights
   $$
   \alpha_{ij} = \frac{\exp(e_{ij})}{\sum_k \exp(e_{ik})}
   $$

4. **Aggregate:** Weighted sum of values
   $$
   \text{output}_i = \sum_j \alpha_{ij} V_j
   $$

## Types of Attention

### Self-Attention

Query, Key, Value all from same sequence:

$$
Q = XW^Q, \quad K = XW^K, \quad V = XW^V
$$

Each position attends to all positions in the sequence.

### Cross-Attention

Query from decoder, Key/Value from encoder:

$$
Q = X_{dec}W^Q, \quad K = X_{enc}W^K, \quad V = X_{enc}W^V
$$

### Multi-Head Attention

Run multiple attention operations in parallel:

$$
\text{MultiHead}(Q, K, V) = \text{Concat}(\text{head}_1, ..., \text{head}_h)W^O
$$

Where:
$$
\text{head}_i = \text{Attention}(QW_i^Q, KW_i^K, VW_i^V)
$$

## Attention Scores

### Dot-Product Attention

$$
e_{ij} = Q_i \cdot K_j
$$

Fast, efficient, requires same dimensions.

### Additive Attention (Bahdanau)

$$
e_{ij} = v^T \tanh(W_1 Q_i + W_2 K_j)
$$

More flexible, additional parameters.

### Scaled Dot-Product

$$
e_{ij} = \frac{Q_i \cdot K_j}{\sqrt{d_k}}
$$

Prevents large values that saturate softmax.

## Visualization

```
Query: "The cat sat on the ___"

Attention weights to fill in "mat":
  The  → 0.05
  cat  → 0.15
  sat  → 0.20
  on   → 0.10
  the  → 0.50  ← High attention to context
```

## In Transformers

### Architecture Role

```
Input
  ↓
[Multi-Head Self-Attention]
  ↓
[Feed Forward Network]
  ↓
(Repeat N times)
  ↓
Output
```

### Encoder

Self-attention over input sequence.

### Decoder

1. Masked self-attention (prevent looking ahead)
2. Cross-attention to encoder outputs

## Complexity Analysis

| Operation | Time | Space |
|-----------|------|-------|
| Attention | \\(O(n^2 d)\\) | \\(O(n^2)\\) |
| Per position | \\(O(nd)\\) | \\(O(n)\\) |

Quadratic in sequence length!

## Efficient Attention Variants

| Method | Approach | Complexity |
|--------|----------|------------|
| Sparse | Attend to subset | \\(O(n\sqrt{n})\\) |
| Linear | Kernel approximation | \\(O(n)\\) |
| Longformer | Local + global | \\(O(n)\\) |
| Flash Attention | Memory efficient | \\(O(n^2)\\) time, less memory |

## Applications

1. **Machine Translation:** Align source and target
2. **Text Summarization:** Focus on key sentences
3. **Question Answering:** Find relevant passages
4. **Image Captioning:** Attend to image regions
5. **Speech Recognition:** Align audio and text

## Key Insights

### Why Attention Works

- Direct connection between positions
- No information bottleneck
- Parallelizable computation
- Interpretable weights

### Limitations

- Quadratic complexity
- Position information lost (needs encoding)
- May overfit on small data
