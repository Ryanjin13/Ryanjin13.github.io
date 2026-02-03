---
title: "Cross Entropy & Softmax Derivation"
date: 2026-01-18
description: "Derivation of Cross Entropy Loss and Softmax gradients"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic"]
draft: false
---

{{< katex >}}

## 1. Forward Pass: Neural Network Output Pipeline

In the final stage of a classification neural network, the output flows through three sequential transformations:

$$
\underbrace{\vphantom{\frac{A}{B}}\text{Neural Network}}_{\text{feature extraction}} \longrightarrow
\underbrace{\vphantom{\frac{A}{B}}z_i}_{\text{logits}} \longrightarrow
\underbrace{\vphantom{\frac{A}{B}}p_i}_{\text{softmax}} \longrightarrow
\underbrace{\vphantom{\frac{A}{B}}L}_{\text{cross entropy}}
$$

Each stage has a specific role:
- **Logits ($z_i$)**: Raw, unnormalized scores from the final linear layer
- **Softmax ($p_i$)**: Converts logits into a valid probability distribution
- **Cross Entropy ($L$)**: Measures the difference between predicted and true distributions

---

## 2. Mathematical Definitions

### 2.1 Logits to Probability: Softmax Function

The softmax function transforms raw logits into probabilities that sum to 1:

$$
\underbrace{{\color{blue}p_i}}_{\text{probability}} = 
\underbrace{\frac{e^{z_i}}{\sum_j e^{z_j}}}_{\text{softmax function}} \tag{1}
$$

where:
- $z_i$ is the logit (raw score) for class $i$
- $e^{z_i}$ ensures all values are positive
- $\sum_j e^{z_j}$ normalizes so that $\sum_i p_i = 1$

### 2.2 Probability to Loss: Cross Entropy

Cross entropy measures how well the predicted distribution $p$ matches the true distribution $y$:

$$
L = -
\underbrace{\vphantom{\sum_i^n}\sum_i}_{\text{all classes}}
\overbrace{\vphantom{\sum_i^n}y_i}^{\text{true label}}
\underbrace{\vphantom{\sum_i^n}\log({\color{blue}p_i})}_{\text{log probability}} \tag{2}
$$

where:
- $y_i = 1$ for the correct class, $y_i = 0$ otherwise (one-hot encoding)
- Since $0 < p_i \leq 1$, we have $\log(p_i) \leq 0$. The negative sign flips this to ensure $L \geq 0$

---

## 3. Backpropagation: Chain Rule Setup

To update network weights, we need $\frac{\partial L}{\partial z_i}$ (gradient w.r.t. logits).

By the chain rule, we decompose this through intermediate variables:

$$
\underbrace{\vphantom{\frac{\partial L}{\partial p}}\frac{\partial L}{\partial z_i}}_{\text{what we want}} =
{\color{red}\underbrace{\vphantom{\frac{\partial L}{\partial p}}\frac{\partial L}{\partial p}}_{\text{CE gradient}}} \cdot
{\color{blue}\underbrace{\vphantom{\frac{\partial L}{\partial p}}\frac{\partial p}{\partial z}}_{\text{softmax gradient}}} \tag{3}
$$

We will derive each term separately:
1. ${\color{red}\frac{\partial L}{\partial p}}$ **(CE gradient)**: How loss changes with probability
2. ${\color{blue}\frac{\partial p}{\partial z}}$ **(Softmax gradient)**: How probability changes with logit

Starting from the **output layer** and working **backwards**.

---

## 4. Derivation Part 1: Cross Entropy Gradient

**Goal:** Find ${\color{red}\frac{\partial L}{\partial p_i}}$

Starting from the cross entropy definition:

$$
L = -\sum_i y_i \log(p_i)
$$

Taking the partial derivative with respect to $p_i$:

$$
{\color{red}\underbrace{\frac{\partial L}{\partial p_i}}_{\text{CE gradient}}} =
\frac{\partial}{\partial p_i}\left[
\underbrace{-y_i}_{\text{label}}
\underbrace{\log(p_i)}_{\text{log prob}}
\right] \tag{4}
$$

Using the derivative of natural log: $\frac{d}{dx}\log(x) = \frac{1}{x}$

$$
\underbrace{\frac{\partial}{\partial p_i}\log_e(p_i)}_{\text{natural log}} =
\underbrace{\frac{1}{p_i}}_{\text{derivative of log}} \tag{5}
$$

Therefore:

$$
\boxed{{\color{red}\frac{\partial L}{\partial p_i} = -\frac{y_i}{p_i}}} \tag{6}
$$

---

## 5. Derivation Part 2: Softmax Gradient

**Goal:** Find ${\color{blue}\frac{\partial p_j}{\partial z_i}}$

Recall the softmax function:

$$
p_i = \frac{e^{z_i}}{\sum_k e^{z_k}} \tag{7}
$$

For cleaner notation, let's define the **normalization constant**:

$$
S \equiv \sum_k e^{z_k} = e^{z_1} + e^{z_2} + \cdots + e^{z_n} \tag{8}
$$

Now softmax becomes:

$$
\underbrace{p_i}_{\text{probability}} = 
\frac{\overbrace{e^{z_i}}^{\text{numerator}}}
     {\underbrace{S}_{\text{normalizer}}} \tag{9}
$$

**Key observation:** $S$ contains ALL $z_k$ terms. Therefore:
- Changing $z_i$ affects the **numerator** $e^{z_i}$
- Changing $z_i$ also affects the **denominator** $S$ (since $z_i$ is one of the terms in the sum)

This is why we need the **quotient rule** for differentiation.

We must consider two cases due to the summation in the denominator.

### Case 1: Same Index ($i = j$)

When differentiating $p_i$ with respect to $z_i$:

$$
{\color{blue}\underbrace{\frac{\partial p_i}{\partial z_i}}_{\text{diagonal term}}} =
\frac{\partial}{\partial z_i}\left(
\frac{\overbrace{e^{z_i}}^{f}}
     {\underbrace{S}_{g}}
\right) \tag{10}
$$

Applying the quotient rule: $\frac{d}{dx}\left[\frac{f}{g}\right] = \frac{f'g - fg'}{g^2}$

where:
- $f = e^{z_i}$, so $f' = e^{z_i}$
- $g = S = \sum_k e^{z_k}$, so $g' = \frac{\partial S}{\partial z_i} = e^{z_i}$

$$
= \frac{
\overbrace{e^{z_i}}^{f'} \cdot 
\overbrace{S}^{g} -
\overbrace{e^{z_i}}^{f} \cdot 
\overbrace{e^{z_i}}^{g'}}
{S^2} \tag{11}
$$

Factoring out $e^{z_i}$:

$$
= \frac{
\overbrace{e^{z_i}}^{\text{common}} \cdot
\overbrace{(S-e^{z_i})}^{\text{remaining}}}
{S^2} \tag{12}
$$

Separating the fractions:

$$
= \underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot
  \underbrace{\frac{S-e^{z_i}}{S}}_{1-p_i} \tag{13}
$$

Recognizing $p_i = \frac{e^{z_i}}{S}$:

$$
= \underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot
\left(1 - \underbrace{\frac{e^{z_i}}{S}}_{p_i}\right) \tag{14}
$$

$$
\boxed{{\color{blue}\frac{\partial p_i}{\partial z_i} = p_i(1-p_i)}} \tag{15}
$$

### Case 2: Different Index ($i \neq j$)

When differentiating $p_i$ with respect to $z_j$ (where $j \neq i$):

$$
{\color{blue}\underbrace{\frac{\partial p_i}{\partial z_j}}_{\text{off-diagonal}}} =
\frac{\partial}{\partial z_j}\left(
\frac{\overbrace{e^{z_i}}^{\text{const w.r.t. }z_j}}
     {\underbrace{S}_{\text{contains }z_j}}
\right) \tag{16}
$$

Here:
- $f = e^{z_i}$ is constant w.r.t. $z_j$, so $f' = 0$
- $g = S$, so $g' = \frac{\partial S}{\partial z_j} = e^{z_j}$

$$
= \frac{
\overbrace{0}^{f'} \cdot S -
\overbrace{e^{z_i}}^{f} \cdot
\overbrace{e^{z_j}}^{g'}}
{S^2} \tag{17}
$$

$$
= -\frac{
\overbrace{e^{z_i}}^{\text{from }p_i} \cdot
\overbrace{e^{z_j}}^{\text{from }p_j}}
{S^2} \tag{18}
$$

$$
= -\underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot
   \underbrace{\frac{e^{z_j}}{S}}_{p_j} \tag{19}
$$

$$
\boxed{{\color{blue}\frac{\partial p_i}{\partial z_j} = -p_i \cdot p_j \quad (i \neq j)}} \tag{20}
$$

### Summary: Softmax Jacobian

$$
{\color{blue}\frac{\partial p_i}{\partial z_j}} = 
\begin{cases}
p_i(1 - p_i) & \text{if } i = j \\
-p_i \cdot p_j & \text{if } i \neq j
\end{cases}
= p_i(\delta_{ij} - p_j) \tag{21}
$$

where $\delta_{ij}$ is the Kronecker delta.

---

## 6. Combining: Full Gradient Derivation

Now we combine both gradients using the chain rule.

**Key observation:** In softmax, changing $z_i$ affects **ALL** probabilities $p_j$ (not just $p_i$), because $z_i$ appears in the denominator $S = \sum_k e^{z_k}$.

Therefore, we must sum over all paths:

$$
\frac{\partial L}{\partial z_i} =
\sum_{j}
{\color{red}\frac{\partial L}{\partial p_j}} \cdot
{\color{blue}\frac{\partial p_j}{\partial z_i}} \tag{22}
$$

This splits into two cases based on our softmax derivative:

$$
\frac{\partial L}{\partial z_i} =
\underbrace{{\color{red}\frac{\partial L}{\partial p_i}} \cdot {\color{blue}\frac{\partial p_i}{\partial z_i}}}_{\text{when }j=i} +
\underbrace{\sum_{j \neq i}{\color{red}\frac{\partial L}{\partial p_j}} \cdot {\color{blue}\frac{\partial p_j}{\partial z_i}}}_{\text{when }j \neq i} \tag{23}
$$

Substituting our derived values from equations (6), (15), and (20):

$$
= \underbrace{{\color{red}\left(-\frac{y_i}{p_i}\right)} \cdot {\color{blue}p_i(1-p_i)}}_{\text{diagonal term}} + 
\underbrace{\sum_{j \neq i}{\color{red}\left(-\frac{y_j}{p_j}\right)} \cdot {\color{blue}(-p_j \cdot p_i)}}_{\text{off-diagonal terms}} \tag{24}
$$

Simplifying the diagonal term:

$$
{\color{red}\left(-\frac{y_i}{p_i}\right)} \cdot {\color{blue}p_i(1-p_i)} = -y_i(1-p_i) = -y_i + y_i p_i \tag{25}
$$

Simplifying the off-diagonal terms:

$$
{\color{red}\left(-\frac{y_j}{p_j}\right)} \cdot {\color{blue}(-p_j \cdot p_i)} = y_j \cdot p_i \tag{26}
$$

Combining:

$$
= \underbrace{-y_i + y_i p_i}_{\text{from diagonal}} + 
\underbrace{\sum_{j \neq i} y_j \cdot p_i}_{\text{from off-diagonal}} \tag{27}
$$

$$
= -y_i + y_i p_i + p_i \sum_{j \neq i} y_j \tag{28}
$$

Since $\sum_j y_j = 1$ (one-hot encoding), we have $\sum_{j \neq i} y_j = 1 - y_i$:

$$
= -y_i + y_i p_i + p_i(1 - y_i) \tag{29}
$$

$$
= -y_i + y_i p_i + p_i - y_i p_i \tag{30}
$$

$$
= p_i - y_i \tag{31}
$$

---

## 7. Final Result

$$
\boxed{
\underbrace{\frac{\partial L}{\partial z_i}}_{\text{gradient}} = 
\underbrace{p_i}_{\text{predicted}} - 
\underbrace{y_i}_{\text{true}}
} \tag{32}
$$

**Interpretation:**
- The gradient is simply the **difference between predicted probability and true label**
- If $y_i = 1$ (correct class): gradient $= p_i - 1 < 0$ (negative, pushing logit **up**)
- If $y_i = 0$ (wrong class): gradient $= p_i > 0$ (positive, pushing logit **down**)

This elegant result is why **Softmax + Cross Entropy** is the standard choice for classification tasks.

---

## Summary Table

| Step | Forward | Backward |
|------|---------|----------|
| Softmax | $z_i \xrightarrow{\text{softmax}} p_i$ | ${\color{blue}\frac{\partial p_j}{\partial z_i} = p_i(\delta_{ij} - p_j)}$ |
| Cross Entropy | $p_i \xrightarrow{\text{CE}} L$ | ${\color{red}\frac{\partial L}{\partial p_i} = -\frac{y_i}{p_i}}$ |
| **Combined** | $z_i \rightarrow L$ | $\frac{\partial L}{\partial z_i} = p_i - y_i$ |