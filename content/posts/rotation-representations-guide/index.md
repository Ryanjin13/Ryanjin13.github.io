---
title: "Complete Guide to Rotation Representations"
date: 2024-08-15
description: "From basic math to geometric intuition and optimization examples - Euler angles, rotation matrices, quaternions, and Lie algebra"
categories: ["3D Vision"]
tags: ["Rotation", "Euler Angles", "Quaternion", "Lie Algebra", "Optimization", "3D Graphics", "Robotics"]
draft: false
---

{{< katex >}}

## Overview

This guide covers rotation representations from fundamentals to optimization. We explore Euler angles, rotation matrices, quaternions, and Lie algebra - comparing their geometric meanings and practical applications.

---

## Part 0: Prerequisite Math Review

### Matrix Multiplication

A matrix transforms a vector - moving, rotating, or scaling it:

$$\begin{bmatrix} a & b \\ c & d \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} ax + by \\ cx + dy \end{bmatrix}$$

**Intuition:** First row dot product with vector → first element of result.

### Trigonometry Review

Sine and cosine describe coordinates on a unit circle:

$$\text{Point coordinates} = (\cos\theta, \sin\theta)$$

| Angle (θ) | cos θ | sin θ | Meaning |
|-----------|-------|-------|---------|
| 0° | 1 | 0 | Right (start) |
| 45° | 0.707 | 0.707 | Diagonal |
| 90° | 0 | 1 | Up |
| 180° | -1 | 0 | Left |

**Geometric meaning:**
- $\cos\theta$ = "How much remains in original direction (X)"
- $\sin\theta$ = "How much moved to new direction (Y)"

### Radians

One full circle = $2\pi$ radians = 360°

$$1 \text{ radian} = \frac{180°}{\pi} \approx 57.3°$$

### 2D Rotation Matrix

$$R_{2D}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

**Example:** Rotating point $(1, 0)$ by 90°:

$$R_{2D}(90°) \cdot \begin{bmatrix} 1 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

The point moves from right to up - counterclockwise 90° rotation confirmed!

### Cross Product

The cross product of two 3D vectors produces a vector perpendicular to both:

$$\mathbf{a} \times \mathbf{b} = \begin{bmatrix} a_2 b_3 - a_3 b_2 \\ a_3 b_1 - a_1 b_3 \\ a_1 b_2 - a_2 b_1 \end{bmatrix}$$

**Example:** $(1,0,0) \times (0,1,0) = (0, 0, 1)$ — X-axis × Y-axis = Z-axis (right-hand rule)

---

## Part 1: Euler Angles & Rotation Matrix

### 3D Rotation Matrices

**X-axis rotation (Roll):** Rotation in Y-Z plane

$$R_x(\alpha) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\alpha & -\sin\alpha \\ 0 & \sin\alpha & \cos\alpha \end{bmatrix}$$

**Y-axis rotation (Pitch):** Rotation in X-Z plane

$$R_y(\beta) = \begin{bmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{bmatrix}$$

**Z-axis rotation (Yaw):** Rotation in X-Y plane

$$R_z(\gamma) = \begin{bmatrix} \cos\gamma & -\sin\gamma & 0 \\ \sin\gamma & \cos\gamma & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

### What Are Euler Angles?

Euler angles say: "Do three separate rotations in sequence."

$$R_{total} = R_z(\gamma) \cdot R_y(\beta) \cdot R_x(\alpha)$$

**Key relationship:**
- **Euler angles = Input (recipe)** → 3 numbers
- **Rotation matrix = Output (result)** → 3×3 = 9 numbers

Same rotation, different representations.

### Gimbal Lock Problem

When the middle rotation (Pitch/Y) reaches 90°, you lose one degree of freedom:

At $\beta = 90°$:
$$R_z(\gamma) \cdot R_y(90°) \cdot R_x(\alpha) = \begin{bmatrix} 0 & \sin(\alpha - \gamma) & \cos(\alpha - \gamma) \\ 0 & \cos(\alpha - \gamma) & -\sin(\alpha - \gamma) \\ -1 & 0 & 0 \end{bmatrix}$$

**Problem:** $\alpha$ and $\gamma$ always appear as $(\alpha - \gamma)$ — two independent variables collapsed into one: $3 \text{ DOF} \to 2 \text{ DOF}$

---

## Part 2: Geometric Meaning of Each Representation

### Euler Angles: "Three Separate Turns"

```
Rotate γ around Z → Rotate β around Y → Rotate α around X

Analogy: Parking
  Step 1: Turn steering wheel (Yaw)
  Step 2: Go up slope (Pitch)
  Step 3: Tilt body (Roll)
```

### Rotation Matrix: "New Coordinate Frame"

$$R = \begin{bmatrix} | & | & | \\ \mathbf{x}' & \mathbf{y}' & \mathbf{z}' \\ | & | & | \end{bmatrix}$$

Each **column** represents where the original axis points after rotation:
- Column 1 = Where X-axis now points
- Column 2 = Where Y-axis now points
- Column 3 = Where Z-axis now points

### Quaternion: "Axis + Angle in 4 Numbers"

$$q = \left(\underbrace{\cos\frac{\theta}{2}}_{w}, \underbrace{\sin\frac{\theta}{2} \cdot u_x}_{x}, \underbrace{\sin\frac{\theta}{2} \cdot u_y}_{y}, \underbrace{\sin\frac{\theta}{2} \cdot u_z}_{z}\right)$$

- $(u_x, u_y, u_z)$ = Rotation axis (unit vector)
- $\theta$ = Rotation angle
- $w$ = $\cos(\theta/2)$ → Angle information
- $(x, y, z)$ = $\sin(\theta/2) \cdot$ axis → Axis information

**Why half angle?** From quaternion rotation formula $\mathbf{p}' = q \cdot \mathbf{p} \cdot q^{-1}$, the quaternion acts on both sides, so angle is applied twice.

**Example:** Z-axis 90° rotation
$$q = (\cos 45°, 0, 0, \sin 45°) = (0.707, 0, 0, 0.707)$$

### Lie Algebra: "Axis + Angle in 3 Numbers"

$$\boldsymbol{\omega} = \theta \cdot \hat{\mathbf{u}} = \theta \cdot (u_x, u_y, u_z)$$

- **Direction** $\hat{\boldsymbol{\omega}}$ = Rotation axis
- **Magnitude** $\|\boldsymbol{\omega}\|$ = Rotation angle (radians)

One vector contains both axis and angle!

**Example:** Z-axis 90° rotation
$$\boldsymbol{\omega} = \frac{\pi}{2} \cdot (0, 0, 1) = (0, 0, 1.571)$$

| Quaternion | Lie Algebra |
|------------|-------------|
| $(0.707, 0, 0, 0.707)$ — 4 numbers | $(0, 0, 1.571)$ — 3 numbers |
| w has angle, xyz has axis (separate) | Direction=axis, magnitude=angle (combined) |

### Exp and Log: Geometric Meaning

$$R = \exp([\boldsymbol{\omega}]_\times) \leftarrow \text{Vector to rotation matrix}$$
$$\boldsymbol{\omega} = \log(R) \leftarrow \text{Rotation matrix to vector}$$

**Analogy:**
- **exp** = Unfolding a path from flat map onto a globe
- **log** = Flattening a path on globe to a flat map

#### Rodrigues' Formula

$$R = I + \frac{\sin\theta}{\theta}[\boldsymbol{\omega}]_\times + \frac{1 - \cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times^2$$

Where $[\boldsymbol{\omega}]_\times$ is the **skew-symmetric matrix**:

$$[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

This matrix multiplication equals cross product: $[\boldsymbol{\omega}]_\times \cdot \mathbf{v} = \boldsymbol{\omega} \times \mathbf{v}$

---

## Part 3: What is Optimization?

### Intuition

Optimization is like finding the lowest point in a valley while blindfolded. You can only feel the slope under your feet and take steps downhill.

1. Check which direction is downhill → **Gradient**
2. Take one step that direction → **Update**
3. Repeat until convergence

### Mathematical Formulation

$$x_{new} = x_{old} - \eta \cdot \frac{d\mathcal{L}}{dx}$$

| Symbol | Meaning | Analogy |
|--------|---------|---------|
| $x$ | Current position (parameter) | My position on mountain |
| $\mathcal{L}$ | Cost function (value to minimize) | Altitude |
| $\frac{d\mathcal{L}}{dx}$ | Gradient (slope) | Slope under feet |
| $\eta$ | Learning rate (step size) | Step length |
| $-$ | Downhill direction | Opposite to slope |

### What's Special About Rotation Optimization?

Rotations live on a curved surface (manifold), not flat space:

- Regular numbers: $3 + 0.5 = 3.5$ → Still valid ✓
- Rotation matrix: $R + \Delta R$ → **May not be valid rotation!** ✗
- Quaternion: $q + \Delta q$ → **May not have magnitude 1!** ✗
- Lie algebra: $\boldsymbol{\omega} + \Delta\boldsymbol{\omega}$ → **Always valid!** ✓

---

## Part 4: Solving the Same Problem 4 Ways

### Problem Definition

**Goal:** Rotate point $\mathbf{p} = (1, 0, 0)$ to be close to $\mathbf{p}^* = (0, 1, 0)$

**Answer:** 90° rotation around Z-axis

$$\mathcal{L} = \frac{1}{2}\|R \cdot \mathbf{p} - \mathbf{p}^*\|^2$$

### Method 1: Euler Angle Optimization

Simplified: Only optimize $\gamma$ (Z-axis rotation)

$$R_z(\gamma) \cdot \mathbf{p} = \begin{bmatrix} \cos\gamma \\ \sin\gamma \\ 0 \end{bmatrix}$$

$$\mathcal{L}(\gamma) = 1 - \sin\gamma$$

$$\frac{d\mathcal{L}}{d\gamma} = -\cos\gamma$$

**Gradient descent** (η = 0.5): $0° \to 28.6° \to 53.8° \to 70.7° \to 80.1° \to \cdots \to 90°$ ✓

**Problem:** Works here with single axis, but with all 3 axes, **gimbal lock** occurs at $\beta \to 90°$.

### Method 2: Direct Rotation Matrix Optimization

Optimize all 9 elements of R:

$$R_{new} = I - 0.5 \times \frac{\partial \mathcal{L}}{\partial R} = \begin{bmatrix} 0.5 & 0 & 0 \\ 0.5 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Problem:**
$$R^T R \neq I \text{ ❌ Not orthogonal! Not a valid rotation!}$$

**Solution:** Must project back onto SO(3) using SVD every iteration — computationally expensive.

### Method 3: Quaternion Optimization

Optimize 4 numbers $q = (w, x, y, z)$:

$$q_{new} = (1, 0, 0, 0) - 0.25 \times (0, 0, 0, -2) = (1, 0, 0, 0.5)$$

**Problem:**
$$\|q_{new}\| = 1.118 \neq 1 \text{ ❌ Not unit quaternion!}$$

**Solution:** Normalize every iteration:
$$q_{normalized} = \frac{(1, 0, 0, 0.5)}{1.118} = (0.894, 0, 0, 0.447)$$

### Method 4: Lie Algebra Optimization ★

Optimize 3 numbers $\boldsymbol{\omega} = (\omega_1, \omega_2, \omega_3)$, **no constraints!**

**Jacobian calculation** using cross products:

$$J = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix}$$

Each column tells "if I rotate slightly around this axis, where does the point go":
- Column 1 (X-axis): $(0, 0, 0)$ → No effect (point is on X-axis)
- Column 2 (Y-axis): $(0, 0, -1)$ → Moves to -Z
- Column 3 (Z-axis): $(0, 1, 0)$ → Moves to +Y ← This is what we need!

**Update:**
$$\boldsymbol{\omega}_{new} = (0, 0, 0) - 0.5 \times (0, 0, -1) = (0, 0, 0.5)$$

**Done! No additional work needed!**
- Normalization? ❌ Not needed
- SVD reprojection? ❌ Not needed
- Just vector addition! ✅

**Verification:** $\exp([(0, 0, 0.5)]_\times)$ automatically produces valid rotation matrix with $R^TR = I$ ✓

| Iteration | $\boldsymbol{\omega}$ | Angle | $\mathcal{L}$ | Post-processing |
|-----------|----------------------|-------|---------------|-----------------|
| 0 | $(0, 0, 0)$ | 0° | 1.000 | None |
| 1 | $(0, 0, 0.500)$ | 28.6° | 0.521 | None |
| 2 | $(0, 0, 0.939)$ | 53.8° | 0.191 | None |
| ... | $(0, 0, 1.571)$ | 90° | 0.000 | None ✓ |

---

## Part 5: Final Comparison

### Side-by-Side Comparison

| Item | Euler Angles | Rotation Matrix | Quaternion | Lie Algebra |
|------|--------------|-----------------|------------|-------------|
| Parameters | 3 | 9 | 4 | **3** |
| Degrees of Freedom | 3 | 3 | 3 | **3** |
| Params = DOF? | ✅ | ❌ 6 wasted | ❌ 1 wasted | **✅ Exact** |
| Constraints | None | $R^TR = I$, det=1 | $\|q\| = 1$ | **None** |
| Valid after update? | ✅ (but gimbal lock) | ❌ SVD needed | ❌ Normalization needed | **✅ Always** |
| Singularities | ❌ Gimbal lock | ✅ None | ✅ None | **✅ None** |
| Gradient computation | Complex (chain rule) | Simple but 9D | 4D + constraint | **3D, cross products only** |

### One Step Comparison

**Rotation Matrix — After update:**
$$R_{new} = \begin{bmatrix} 0.5 & 0 & 0 \\ 0.5 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
$R^TR \neq I$ ❌ → SVD reprojection needed

**Quaternion — After update:**
$$q_{new} = (1, 0, 0, 0.5)$$
$\|q\| = 1.118 \neq 1$ ❌ → Normalization needed

**Lie Algebra — After update:**
$$\boldsymbol{\omega}_{new} = (0, 0, 0.5)$$
Just a vector. Always valid. No post-processing. ✅

---

## Conclusion

| Representation | Best Use Case |
|----------------|---------------|
| **Euler Angles** | Human-understandable, UI. Not for computation (gimbal lock). |
| **Rotation Matrix** | Applying coordinate transforms. Not for optimization (constraints). |
| **Quaternion** | Real-time computation, interpolation. Slight overhead for optimization (normalization). |
| **Lie Algebra** | Optimization/differentiation. Minimal parameters, no constraints, no singularities. |

### Practical Workflow

- User input → **Euler angles**
- Optimization computation → **Lie algebra**
- Real-time composition/interpolation → **Quaternion**
- Coordinate transform application → **Rotation matrix**

Use the right representation for each stage and convert between them as needed.
