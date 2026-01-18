---
title: "Cross Entropy & Softmax Derivation"
date: 2026-01-18
description: "Cross Entropy Loss와 Softmax의 미분 유도 과정"
categories: ["Artificial Intelligence"]
tags: ["Deep Learning Basic"]
math: true
draft: false
---

## Cross Entropy Loss

$$
L=-\sum_i y_i \log(p_i) \tag{1}
$$

## Softmax

$$
p_i=\frac{e^{z_i}}{\sum_j e^{z_j}} \tag{2}
$$

## Chain Rule

$$
\frac{\partial L}{\partial z_i}=p_i-y_i \tag{3}
$$

## Softmax + CE

$$
\frac{\partial CE}{\partial z_i} = \text{softmax}(z_i)-y_i = p_i-y_i \tag{4}
$$

## Derivate Cross Entropy

$$
\frac{\partial L}{\partial p_i}=\frac{\partial}{\partial p_i}[-y_i\log(p_i)] \tag{5}
$$

$$
\frac{\partial}{\partial p_i} \log_e(p_i) =\frac{1}{p_i} \tag{6}
$$

$$
=-y_i \cdot \frac{1}{p_i} \tag{7}
$$

## Derivate Softmax

$$
p_i=\frac{e^{z_i}}{\sum_k e^{z_k}} \tag{8}
$$

### Case 1: $\frac{\partial p_i}{\partial z_i}$

$$
\frac{\partial p_i}{\partial z_i}=\frac{\partial}{\partial z_i}\left[\frac{e^{z_i}}{\sum_k e^{z_k}}\right] \tag{9}
$$

fractional differentiation: $\frac{\partial}{\partial x} \left[\frac{f}{g}\right] = \frac{f'g-fg'}{g^2}$

$$
\frac{\partial p_i}{\partial z_i} = \frac{\partial}{\partial z_i} \left(\frac{e^{z_i}}{S}\right) \tag{10}
$$

$$
= \frac{e^{z_i} \cdot S-e^{z_i} \cdot e^{z_i}}{S^2} \tag{11}
$$

$$
= \frac{e^{z_i}(S-e^{z_i})}{S^2} \tag{12}
$$

$$
= \frac{e^{z_i}}{S} \cdot \frac{(S-e^{z_i})}{S} \tag{13}
$$

$$
= \frac{e^{z_i}}{S} \cdot \left(1-\frac{e^{z_i}}{S}\right) \tag{14}
$$

$$
= p_i(1-p_i) \tag{15}
$$

### Case 2: $\frac{\partial p_i}{\partial z_j}, i \neq j$

$$
\frac{\partial p_i}{\partial z_j}=\frac{\partial}{\partial z_j}\left(\frac{e^{z_i}}{S}\right) \tag{16}
$$

$$
= \frac{0 \cdot S-e^{z_i} \cdot e^{z_j}}{S^2} \tag{17}
$$

$$
= -\frac{e^{z_i} \cdot e^{z_j}}{S^2} \tag{18}
$$

$$
= -\frac{e^{z_i}}{S} \cdot \frac{e^{z_j}}{S} \tag{19}
$$

$$
= -p_i \cdot p_j \tag{20}
$$

## Chain Rule

$$
\frac{\partial L}{\partial z_i} = \sum_{j=1}^n \frac{\partial L}{\partial p_j} \cdot \frac{\partial p_j}{\partial z_i} \tag{21}
$$

$$
\frac{\partial L}{\partial z_i}=\frac{\partial L}{\partial p_i} \cdot \frac{\partial p_i}{\partial z_i}+\sum_{j \neq i}\frac{\partial L}{\partial p_j} \cdot \frac{\partial p_j}{\partial z_i} \tag{22}
$$

$$
\frac{\partial L}{\partial p_j}=-\frac{y_j}{p_j} \tag{23}
$$
