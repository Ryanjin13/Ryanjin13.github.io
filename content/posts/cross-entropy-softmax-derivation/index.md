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
L = \underbrace{\vphantom{\sum_{i}}-}_{\text{negative}}
    \underbrace{\sum_i}_{\text{sum over classes}}
    \underbrace{\vphantom{\sum_{i}}y_i}_{\text{true label}}
    \underbrace{\vphantom{\sum_{i}}\log(p_i)}_{\text{log probability}} \tag{1}
$$

## Softmax

$$
\underbrace{\vphantom{\frac{e^{z_i}}{\sum_j}}p_i}_{\text{probability}} =
\frac{\overbrace{\vphantom{\sum_j}e^{z_i}}^{\text{exp of logit}}}
     {\underbrace{\sum_j e^{z_j}}_{\text{normalization}}} \tag{2}
$$

## Chain Rule

$$
\underbrace{\vphantom{\frac{\partial}{\partial}}\frac{\partial L}{\partial z_i}}_{\text{gradient w.r.t. logit}} =
\underbrace{\vphantom{\frac{\partial}{\partial}}p_i}_{\text{predicted}} -
\underbrace{\vphantom{\frac{\partial}{\partial}}y_i}_{\text{true label}} \tag{3}
$$

## Softmax + CE

$$
\underbrace{\vphantom{\frac{\partial}{\partial}}\frac{\partial CE}{\partial z_i}}_{\text{CE gradient}} =
\underbrace{\vphantom{\frac{\partial}{\partial}}\text{softmax}(z_i)}_{\text{predicted}} -
\underbrace{\vphantom{\frac{\partial}{\partial}}y_i}_{\text{true label}} =
\underbrace{\vphantom{\frac{\partial}{\partial}}p_i - y_i}_{\text{error}} \tag{4}
$$

## Derivate Cross Entropy

$$
\underbrace{\vphantom{\frac{\partial}{\partial}}\frac{\partial L}{\partial p_i}}_{\text{gradient w.r.t. prob}} =
\frac{\partial}{\partial p_i}\left[
\underbrace{\vphantom{\frac{\partial}{\partial}}-y_i}_{\text{label}}
\underbrace{\vphantom{\frac{\partial}{\partial}}\log(p_i)}_{\text{log prob}}
\right] \tag{5}
$$

$$
\frac{\partial}{\partial p_i}
\underbrace{\vphantom{\frac{1}{p_i}}\log_e(p_i)}_{\text{natural log}} =
\underbrace{\vphantom{\frac{1}{p_i}}\frac{1}{p_i}}_{\text{derivative of log}} \tag{6}
$$

$$
= \underbrace{\vphantom{\frac{1}{p_i}}-y_i}_{\text{label}} \cdot
  \underbrace{\vphantom{\frac{1}{p_i}}\frac{1}{p_i}}_{\text{inverse prob}} \tag{7}
$$

## Derivate Softmax

$$
\underbrace{\vphantom{\frac{e^{z_i}}{\sum_k}}p_i}_{\text{softmax output}} =
\frac{\overbrace{\vphantom{\sum_k}e^{z_i}}^{\text{numerator}}}
     {\underbrace{\sum_k e^{z_k}}_{\text{sum (S)}}} \tag{8}
$$

### Case 1: \(\frac{\partial p_i}{\partial z_i}\) (same index)

$$
\underbrace{\vphantom{\frac{\partial}{\partial}}\frac{\partial p_i}{\partial z_i}}_{\text{diagonal term}} =
\frac{\partial}{\partial z_i}\left[
\frac{\overbrace{\vphantom{\sum_k}e^{z_i}}^{f}}
     {\underbrace{\sum_k e^{z_k}}_{g}}
\right] \tag{9}
$$

Quotient rule: \(\frac{\partial}{\partial x} \left[\frac{f}{g}\right] = \frac{f'g-fg'}{g^2}\)

$$
\frac{\partial p_i}{\partial z_i} =
\frac{\partial}{\partial z_i} \left(
\frac{\overbrace{\vphantom{S}e^{z_i}}^{f}}
     {\underbrace{\vphantom{e^{z_i}}S}_{g}}
\right) \tag{10}
$$

$$
= \frac{
\overbrace{\vphantom{S^2}e^{z_i}}^{f'} \cdot \overbrace{\vphantom{S^2}S}^{g} -
\overbrace{\vphantom{S^2}e^{z_i}}^{f} \cdot \overbrace{\vphantom{S^2}e^{z_i}}^{g'}}
{\underbrace{\vphantom{e^{z_i}}S^2}_{g^2}} \tag{11}
$$

$$
= \frac{
\overbrace{\vphantom{S^2}e^{z_i}}^{\text{common}} \cdot
\overbrace{\vphantom{S^2}(S-e^{z_i})}^{\text{remaining}}}
{\underbrace{\vphantom{e^{z_i}}S^2}_{\text{denom}}} \tag{12}
$$

$$
= \underbrace{\vphantom{\frac{S}{S}}\frac{e^{z_i}}{S}}_{p_i} \cdot
  \underbrace{\vphantom{\frac{S}{S}}\frac{S-e^{z_i}}{S}}_{1-p_i} \tag{13}
$$

$$
= \underbrace{\vphantom{\frac{S}{S}}\frac{e^{z_i}}{S}}_{p_i} \cdot
\left(1 - \underbrace{\vphantom{\frac{S}{S}}\frac{e^{z_i}}{S}}_{p_i}\right) \tag{14}
$$

$$
= \underbrace{\vphantom{\frac{S}{S}}p_i(1-p_i)}_{\text{sigmoid-like deriv}} \tag{15}
$$

### Case 2: \(\frac{\partial p_i}{\partial z_j}, i \neq j\) (different index)

$$
\underbrace{\vphantom{\frac{\partial}{\partial}}\frac{\partial p_i}{\partial z_j}}_{\text{off-diagonal}} =
\frac{\partial}{\partial z_j}\left(
\frac{\overbrace{\vphantom{S}e^{z_i}}^{\text{const w.r.t. }z_j}}
     {\underbrace{\vphantom{e^{z_i}}S}_{\text{sum}}}
\right) \tag{16}
$$

$$
= \frac{
\overbrace{\vphantom{S^2}0}^{f'} \cdot S -
\overbrace{\vphantom{S^2}e^{z_i}}^{f} \cdot
\overbrace{\vphantom{S^2}e^{z_j}}^{g'}}
{S^2} \tag{17}
$$

$$
= -\frac{
\overbrace{\vphantom{S^2}e^{z_i}}^{\text{from }p_i} \cdot
\overbrace{\vphantom{S^2}e^{z_j}}^{\text{from }p_j}}
{\underbrace{\vphantom{e^{z_i}}S^2}_{\text{denom}}} \tag{18}
$$

$$
= -\underbrace{\vphantom{\frac{S}{S}}\frac{e^{z_i}}{S}}_{p_i} \cdot
   \underbrace{\vphantom{\frac{S}{S}}\frac{e^{z_j}}{S}}_{p_j} \tag{19}
$$

$$
= \underbrace{\vphantom{\frac{S}{S}}-p_i \cdot p_j}_{\text{negative product}} \tag{20}
$$

## Chain Rule

$$
\underbrace{\vphantom{\sum_{j=1}^n}\frac{\partial L}{\partial z_i}}_{\text{final gradient}} =
\underbrace{\sum_{j=1}^n}_{\text{sum over j}}
\underbrace{\vphantom{\sum_{j=1}^n}\frac{\partial L}{\partial p_j}}_{\text{CE grad}} \cdot
\underbrace{\vphantom{\sum_{j=1}^n}\frac{\partial p_j}{\partial z_i}}_{\text{softmax grad}} \tag{21}
$$

$$
\frac{\partial L}{\partial z_i} =
\underbrace{\vphantom{\sum_{j \neq i}}\frac{\partial L}{\partial p_i} \cdot \frac{\partial p_i}{\partial z_i}}_{\text{when }j=i} +
\underbrace{\sum_{j \neq i}\frac{\partial L}{\partial p_j} \cdot \frac{\partial p_j}{\partial z_i}}_{\text{when }j \neq i} \tag{22}
$$

$$
\underbrace{\vphantom{\frac{y_j}{p_j}}\frac{\partial L}{\partial p_j}}_{\text{CE gradient}} =
\underbrace{\vphantom{\frac{y_j}{p_j}}-\frac{y_j}{p_j}}_{\text{from eq. 7}} \tag{23}
$$
