---
title: "Eigenvalue Decomposition: General vs Symmetric Matrices"
date: 2024-06-24
description: "Comparing eigenvalue decomposition for general and symmetric matrices"
categories: ["Math"]
tags: ["Linear Algebra", "Eigenvalues", "Matrix Decomposition", "PCA"]
draft: false
---

{{< katex >}}

## Overview

Eigenvalue decomposition takes different forms depending on matrix properties. This post compares the general case with the special case of symmetric matrices.

## General Eigenvalue Decomposition

### Form

For any diagonalizable square matrix \\(A\\):

$$
A = S\Lambda S^{-1}
$$

Where:
- \\(S\\): Matrix of eigenvectors (columns)
- \\(\Lambda\\): Diagonal matrix of eigenvalues
- \\(S^{-1}\\): Inverse of eigenvector matrix

### Eigenvalue Equation

$$
Av = \lambda v
$$

Each column of \\(S\\) satisfies this equation.

### Properties

| Property | General Case |
|----------|--------------|
| Eigenvalues | May be complex |
| Eigenvectors | Not necessarily orthogonal |
| \\(S\\) | May not be orthonormal |
| \\(S^{-1}\\) | Must be computed explicitly |

### Example

$$
A = \begin{pmatrix} 4 & 2 \\ 1 & 3 \end{pmatrix}
$$

Eigenvalues: \\(\lambda_1 = 5, \lambda_2 = 2\\)

Eigenvectors form \\(S\\), but they're not orthogonal.

## Symmetric Matrix Decomposition

### Form

For symmetric matrix \\(B = B^T\\):

$$
B = Q\Lambda Q^T = Q\Lambda Q^{-1}
$$

Where:
- \\(Q\\): Orthonormal eigenvector matrix
- \\(\Lambda\\): Diagonal matrix of (real) eigenvalues
- \\(Q^T = Q^{-1}\\)

### Special Properties

| Property | Symmetric Case |
|----------|----------------|
| Eigenvalues | Always real |
| Eigenvectors | Orthogonal |
| \\(Q\\) | Orthonormal (\\(Q^TQ = I\\)) |
| \\(Q^{-1}\\) | Simply \\(Q^T\\) |

### Spectral Theorem

Every symmetric matrix can be diagonalized by an orthonormal matrix:

$$
B = Q\Lambda Q^T = \sum_{i=1}^{n} \lambda_i \mathbf{q}_i \mathbf{q}_i^T
$$

Outer product form shows each eigenvalue-eigenvector pair's contribution.

## Comparison

| Aspect | \\(A = S\Lambda S^{-1}\\) | \\(B = Q\Lambda Q^T\\) |
|--------|---------------------------|------------------------|
| Matrix type | General square | Symmetric |
| Eigenvalues | \\(\lambda \in \mathbb{C}\\) | \\(\lambda \in \mathbb{R}\\) |
| Eigenvectors | Non-orthogonal | Orthonormal |
| Inverse | Compute \\(S^{-1}\\) | Just transpose |
| Reconstruction | \\(S\Lambda S^{-1}\\) | \\(Q\Lambda Q^T\\) |
| Numerical stability | Less stable | Very stable |

## Application: Covariance Matrices

Covariance matrices are symmetric!

$$
\Sigma = Q\Lambda Q^T
$$

### Interpretation

- \\(\mathbf{q}_i\\): Principal directions (eigenvectors)
- \\(\lambda_i\\): Variances along each direction

### Largest Eigenvalue

Corresponds to direction of maximum variance—the principal component.

### Smallest Eigenvalue

In point cloud analysis:
- Direction of minimum variance
- Often represents surface normal
- Perpendicular to the surface

## Rotation Matrices

Interesting case: Rotation matrices are orthogonal.

$$
R^TR = I
$$

Some rotation matrices are also symmetric (e.g., reflections):

$$
R = R^T
$$

These have eigenvalues \\(\pm 1\\).

## PCA Connection

### Covariance Decomposition

$$
\Sigma = \frac{1}{n-1}X^TX = Q\Lambda Q^T
$$

### Principal Components

1. Columns of \\(Q\\) are principal directions
2. \\(\lambda_i\\) are variances explained
3. Project data: \\(X_{proj} = XQ\\)

### Dimensionality Reduction

Keep top \\(k\\) eigenvalues:

$$
\Sigma_k = Q_k \Lambda_k Q_k^T
$$

## Point Cloud Normal Estimation

For local point neighborhood with covariance \\(\Sigma\\):

```
Eigenvalue analysis:
λ₁ > λ₂ > λ₃

q₁: Direction of max spread (along surface)
q₂: Second spread direction (along surface)
q₃: Normal vector (perpendicular to surface)
```

The eigenvector corresponding to the smallest eigenvalue gives the surface normal.

## Numerical Computation

### General Matrix

```python
eigenvalues, eigenvectors = np.linalg.eig(A)
```

### Symmetric Matrix

```python
eigenvalues, eigenvectors = np.linalg.eigh(B)  # More stable
```

Use `eigh` for symmetric matrices—faster and more numerically stable.

## Summary

| Use Case | Form | Why |
|----------|------|-----|
| General analysis | \\(S\Lambda S^{-1}\\) | Any square matrix |
| Symmetric/PCA | \\(Q\Lambda Q^T\\) | Guaranteed real, orthogonal |
| Covariance | \\(Q\Lambda Q^T\\) | Find principal directions |
| Point clouds | \\(Q\Lambda Q^T\\) | Normal estimation |
