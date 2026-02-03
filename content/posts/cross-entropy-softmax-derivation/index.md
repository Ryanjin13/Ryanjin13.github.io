---
title: "Cross Entropy & Softmax Derivation"
date: 2026-01-18
description: "Derivation of Cross Entropy Loss and Softmax gradients"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic"]
draft: false
---

{{< katex >}}

## Cross Entropy Loss

$$
\underbrace{L}_{\text{loss}}=-\sum_i \underbrace{y_i}_{\text{true label}} \underbrace{\log(p_i)}_{\text{log probability}} \tag{1}
$$

## Softmax

$$
\underbrace{p_i}_{\text{probability}}=\frac{\overbrace{e^{z_i}}^{\text{exponential of logit}}}{\underbrace{\sum_j e^{z_j}}_{\text{normalization factor}}} \tag{2}
$$

## Chain Rule

$$
\underbrace{\frac{\partial L}{\partial z_i}}_{\text{gradient w.r.t. logit}}=\underbrace{p_i}_{\text{predicted prob}}-\underbrace{y_i}_{\text{true label}} \tag{3}
$$

## Softmax + CE

$$
\underbrace{\frac{\partial CE}{\partial z_i}}_{\text{CE gradient}} = \underbrace{\text{softmax}(z_i)}_{\text{predicted prob}}-\underbrace{y_i}_{\text{true label}} = \underbrace{p_i-y_i}_{\text{error}} \tag{4}
$$

## Derivate Cross Entropy

$$
\underbrace{\frac{\partial L}{\partial p_i}}_{\text{gradient w.r.t. prob}}=\frac{\partial}{\partial p_i}\left[\underbrace{-y_i}_{\text{label}}\underbrace{\log(p_i)}_{\text{log prob}}\right] \tag{5}
$$

$$
\frac{\partial}{\partial p_i} \underbrace{\log_e(p_i)}_{\text{natural log}} =\underbrace{\frac{1}{p_i}}_{\text{derivative of log}} \tag{6}
$$

$$
=\underbrace{-y_i}_{\text{label}} \cdot \underbrace{\frac{1}{p_i}}_{\text{inverse prob}} \tag{7}
$$

## Derivate Softmax

$$
\underbrace{p_i}_{\text{softmax output}}=\frac{\overbrace{e^{z_i}}^{\text{numerator}}}{\underbrace{\sum_k e^{z_k}}_{\text{sum (S)}}} \tag{8}
$$

### Case 1: \(\frac{\partial p_i}{\partial z_i}\) (same index)

$$
\underbrace{\frac{\partial p_i}{\partial z_i}}_{\text{diagonal term}}=\frac{\partial}{\partial z_i}\left[\frac{\overbrace{e^{z_i}}^{f}}{\underbrace{\sum_k e^{z_k}}_{g}}\right] \tag{9}
$$

Quotient rule: \(\frac{\partial}{\partial x} \left[\frac{f}{g}\right] = \frac{f'g-fg'}{g^2}\)

$$
\frac{\partial p_i}{\partial z_i} = \frac{\partial}{\partial z_i} \left(\frac{\overbrace{e^{z_i}}^{f}}{\underbrace{S}_{g}}\right) \tag{10}
$$

$$
= \frac{\overbrace{e^{z_i}}^{f'} \cdot \overbrace{S}^{g}-\overbrace{e^{z_i}}^{f} \cdot \overbrace{e^{z_i}}^{g'}}{\underbrace{S^2}_{g^2}} \tag{11}
$$

$$
= \frac{\overbrace{e^{z_i}}^{\text{common factor}}(\overbrace{S-e^{z_i}}^{\text{remaining}})}{\underbrace{S^2}_{\text{denominator}}} \tag{12}
$$

$$
= \underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot \underbrace{\frac{(S-e^{z_i})}{S}}_{1-p_i} \tag{13}
$$

$$
= \underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot \left(1-\underbrace{\frac{e^{z_i}}{S}}_{p_i}\right) \tag{14}
$$

$$
= \underbrace{p_i(1-p_i)}_{\text{sigmoid-like derivative}} \tag{15}
$$

### Case 2: \(\frac{\partial p_i}{\partial z_j}, i \neq j\) (different index)

$$
\underbrace{\frac{\partial p_i}{\partial z_j}}_{\text{off-diagonal term}}=\frac{\partial}{\partial z_j}\left(\frac{\overbrace{e^{z_i}}^{\text{const w.r.t. } z_j}}{S}\right) \tag{16}
$$

$$
= \frac{\overbrace{0}^{f'} \cdot S-\overbrace{e^{z_i}}^{f} \cdot \overbrace{e^{z_j}}^{g'}}{S^2} \tag{17}
$$

$$
= -\frac{\overbrace{e^{z_i}}^{\text{from } p_i} \cdot \overbrace{e^{z_j}}^{\text{from } p_j}}{\underbrace{S^2}_{\text{denominator}}} \tag{18}
$$

$$
= -\underbrace{\frac{e^{z_i}}{S}}_{p_i} \cdot \underbrace{\frac{e^{z_j}}{S}}_{p_j} \tag{19}
$$

$$
= \underbrace{-p_i \cdot p_j}_{\text{negative product}} \tag{20}
$$

## Chain Rule

$$
\underbrace{\frac{\partial L}{\partial z_i}}_{\text{final gradient}} = \sum_{j=1}^n \underbrace{\frac{\partial L}{\partial p_j}}_{\text{CE gradient}} \cdot \underbrace{\frac{\partial p_j}{\partial z_i}}_{\text{softmax gradient}} \tag{21}
$$

$$
\frac{\partial L}{\partial z_i}=\underbrace{\frac{\partial L}{\partial p_i} \cdot \frac{\partial p_i}{\partial z_i}}_{\text{when } j=i}+\underbrace{\sum_{j \neq i}\frac{\partial L}{\partial p_j} \cdot \frac{\partial p_j}{\partial z_i}}_{\text{when } j \neq i} \tag{22}
$$

$$
\underbrace{\frac{\partial L}{\partial p_j}}_{\text{CE gradient}}=\underbrace{-\frac{y_j}{p_j}}_{\text{from equation 7}} \tag{23}
$$
