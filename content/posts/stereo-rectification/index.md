---
title: "Stereo Rectification: The Math Behind Stereo Image Alignment"
date: 2026-02-19
description: "A mathematically detailed walkthrough of Stereo Rectification from Epipolar Geometry fundamentals to Homography computation using Bouguet and Hartley methods"
categories: ["3D Vision"]
tags: ["Stereo Vision", "Rectification", "Epipolar Geometry", "Camera Calibration", "Disparity"]
draft: false
---

{{< katex >}}

## Overview

In stereo vision, estimating depth requires finding **correspondences** between two images. In a general stereo camera setup, this correspondence search spans the entire 2D image — making it computationally expensive.

**Rectification** transforms both images so that **all epipolar lines become horizontal**. This reduces the correspondence search to a **1D horizontal scanline**, dramatically improving efficiency.

```
Before Rectification:                After Rectification:
┌──────────┐  ┌──────────┐          ┌──────────┐  ┌──────────┐
│    ╲     │  │     ╱    │          │──────────│  │──────────│
│     ╲    │  │    ╱     │          │──●───────│  │──────●──│
│  ●   ╲   │  │   ╱  ●  │          │──────────│  │──────────│
│       ╲  │  │  ╱       │          │──────────│  │──────────│
│        ╲ │  │ ╱        │          └──────────┘  └──────────┘
└──────────┘  └──────────┘           Search only along horizontal scanlines
 Epipolar lines are non-horizontal
```

---

## 1. Epipolar Geometry Fundamentals

### 1.1 Basic Components

Let \(C_1\), \(C_2\) be the optical centers of two cameras, and let a 3D point \(\mathbf{X}\) project to \(\mathbf{x}_1\) in the left image and \(\mathbf{x}_2\) in the right image.

- **Epipole** \(e_1\): Projection of right camera center \(C_2\) onto the left image
- **Epipole** \(e_2\): Projection of left camera center \(C_1\) onto the right image
- **Epipolar Plane**: The plane containing \(\mathbf{X}\), \(C_1\), and \(C_2\)
- **Epipolar Line**: Intersection of the epipolar plane with each image plane
- **Baseline**: The line connecting \(C_1\) and \(C_2\)

### 1.2 Essential Matrix

Setting the left camera as the world coordinate origin, the relationship to the right camera is expressed by rotation \(\mathbf{R}\) and translation \(\mathbf{t}\):

$$
\mathbf{x}_2 = \mathbf{R}\mathbf{x}_1 + \mathbf{t}
$$

where \(\mathbf{x}_1\), \(\mathbf{x}_2\) are **normalized image coordinates**.

Deriving the epipolar constraint:

$$
\mathbf{x}_2^\top [\mathbf{t}]_\times \mathbf{R} \, \mathbf{x}_1 = 0
$$

where \([\mathbf{t}]_\times\) is the **skew-symmetric matrix** of \(\mathbf{t}\):

$$
[\mathbf{t}]_\times = \begin{bmatrix} 0 & -t_z & t_y \\\\ t_z & 0 & -t_x \\\\ -t_y & t_x & 0 \end{bmatrix}
$$

We define the **Essential Matrix** as:

$$
\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}
$$

Thus the epipolar constraint becomes:

$$
\boxed{\mathbf{x}_2^\top \mathbf{E} \, \mathbf{x}_1 = 0}
$$

### 1.3 Fundamental Matrix

In practice, we work with **pixel coordinates** \(\mathbf{p}_1\), \(\mathbf{p}_2\) rather than normalized coordinates. The intrinsic matrices \(\mathbf{K}_1\), \(\mathbf{K}_2\) relate them:

$$
\mathbf{p}_1 = \mathbf{K}_1 \mathbf{x}_1, \quad \mathbf{p}_2 = \mathbf{K}_2 \mathbf{x}_2
$$

Substituting into the Essential Matrix relation:

$$
(\mathbf{K}_2^{-1}\mathbf{p}_2)^\top \mathbf{E} (\mathbf{K}_1^{-1}\mathbf{p}_1) = 0
$$

$$
\mathbf{p}_2^\top \underbrace{\mathbf{K}_2^{-\top} \mathbf{E} \, \mathbf{K}_1^{-1}}_{\mathbf{F}} \mathbf{p}_1 = 0
$$

The **Fundamental Matrix**:

$$
\boxed{\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \, \mathbf{K}_1^{-1}}
$$

$$
\mathbf{p}_2^\top \mathbf{F} \, \mathbf{p}_1 = 0
$$

**Epipolar lines** are obtained as:
- Right epipolar line for left point \(\mathbf{p}_1\): \(\mathbf{l}_2 = \mathbf{F}\mathbf{p}_1\)
- Left epipolar line for right point \(\mathbf{p}_2\): \(\mathbf{l}_1 = \mathbf{F}^\top\mathbf{p}_2\)

---

## 2. Mathematical Goal of Rectification

Rectification applies **Homographies** \(\mathbf{H}_1\), \(\mathbf{H}_2\) to each image such that:

1. All epipolar lines become **horizontal** (same \(y\) coordinate)
2. Epipoles move to the **point at infinity** \((f, 0, 0)^\top\)
3. Corresponding points lie on the **same scanline**

Mathematically, after rectification the Fundamental Matrix \(\mathbf{F}'\) takes the form:

$$
\mathbf{F}' = [\mathbf{e}'_2]_\times = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}
$$

This means \(\mathbf{p}'\_{2y} = \mathbf{p}'\_{1y}\) — corresponding points have identical \(y\) coordinates.

The transformed coordinates are:

$$
\mathbf{p}'_1 = \mathbf{H}_1 \mathbf{p}_1, \quad \mathbf{p}'_2 = \mathbf{H}_2 \mathbf{p}_2
$$

Computing \(\mathbf{H}_1\) and \(\mathbf{H}_2\) is the core of any rectification algorithm.

---

## 3. Calibrated Rectification (Bouguet Method)

The standard approach when both intrinsic and extrinsic parameters are known (calibrated stereo).

### 3.1 Input

- Intrinsic parameters: \(\mathbf{K}_1\), \(\mathbf{K}_2\)
- Extrinsic parameters: Rotation \(\mathbf{R}\), Translation \(\mathbf{t}\) (Camera 1 → Camera 2)

### 3.2 Step 1: Compute a New Common Rotation

Both cameras must be aligned to the **same orientation**. Bouguet's method **splits the rotation equally** between the two cameras.

For rotation \(\mathbf{R}\), compute the rotation vector \(\mathbf{r}\) via Rodrigues:

$$
\mathbf{R} = \exp([\mathbf{r}]_\times)
$$

Apply half-rotation to each camera:

$$
\mathbf{r}_{1} = -\frac{\mathbf{r}}{2}, \quad \mathbf{r}_{2} = +\frac{\mathbf{r}}{2}
$$

$$
\mathbf{R}_{rect1} = \exp([\mathbf{r}_1]_\times), \quad \mathbf{R}_{rect2} = \exp([\mathbf{r}_2]_\times)
$$

After this rotation, both cameras' optical axes point in the **same direction**, but the baseline may not yet be horizontal.

### 3.3 Step 2: Align Baseline to the Horizontal Axis

The baseline direction after half-rotation:

$$
\mathbf{t}' = \mathbf{R}_{rect1} \cdot \mathbf{t}
$$

Construct an additional rotation \(\mathbf{R}_{align}\) so that this baseline becomes the \(x\)-axis of the new coordinate system.

New \(x\)-axis (baseline direction):

$$
\mathbf{e}_1 = \frac{\mathbf{t}'}{||\mathbf{t}'||}
$$

New \(y\)-axis (cross product of \(z\)-axis and \(x\)-axis):

$$
\mathbf{e}_2 = \frac{(-t'_y, t'_x, 0)^\top}{||(-t'_y, t'_x, 0)||}
$$

New \(z\)-axis:

$$
\mathbf{e}_3 = \mathbf{e}_1 \times \mathbf{e}_2
$$

Alignment rotation:

$$
\mathbf{R}_{align} = \begin{bmatrix} \mathbf{e}_1^\top \\\\ \mathbf{e}_2^\top \\\\ \mathbf{e}_3^\top \end{bmatrix}
$$

### 3.4 Step 3: Final Rectification Homography

The final Homography applied to each camera:

$$
\boxed{\mathbf{H}_1 = \mathbf{K}_{new} \cdot \mathbf{R}_{align} \cdot \mathbf{R}_{rect1} \cdot \mathbf{K}_1^{-1}}
$$

$$
\boxed{\mathbf{H}_2 = \mathbf{K}_{new} \cdot \mathbf{R}_{align} \cdot \mathbf{R}_{rect2} \cdot \mathbf{K}_2^{-1}}
$$

where \(\mathbf{K}_{new}\) is the new intrinsic matrix applied after rectification, typically set as:

$$
\mathbf{K}_{new} = \frac{\mathbf{K}_1 + \mathbf{K}_2}{2}
$$

### 3.5 Geometric Interpretation

Breaking down each Homography component:

$$
\underbrace{\mathbf{K}_{new}}_{\text{new projection}} \cdot \underbrace{\mathbf{R}_{align}}_{\text{baseline alignment}} \cdot \underbrace{\mathbf{R}_{rect}}_{\text{half rotation}} \cdot \underbrace{\mathbf{K}^{-1}}_{\text{back-projection}}
$$

1. \(\mathbf{K}^{-1}\): Back-project pixel coordinates to normalized coordinates
2. \(\mathbf{R}_{rect}\): Half rotation to make both optical axes parallel
3. \(\mathbf{R}_{align}\): Additional rotation to align baseline with the \(x\)-axis
4. \(\mathbf{K}_{new}\): Re-project normalized coordinates back to pixel coordinates

---

## 4. Uncalibrated Rectification (Hartley Method)

When intrinsic parameters are unknown, rectification can be performed using **only the Fundamental Matrix \(\mathbf{F}\)**.

### 4.1 Step 1: Compute Epipoles

Find epipoles from the null space of \(\mathbf{F}\):

$$
\mathbf{F} \mathbf{e}_1 = \mathbf{0}, \quad \mathbf{F}^\top \mathbf{e}_2 = \mathbf{0}
$$

Via SVD: from \(\mathbf{F} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top\), \(\mathbf{e}_1\) is the last column of \(\mathbf{V}\), and \(\mathbf{e}_2\) is the last column of \(\mathbf{U}\).

### 4.2 Step 2: Homography to Send Epipole to Infinity

Map epipole \(\mathbf{e}_2 = (e_x, e_y, e_w)^\top\) to the point at infinity \((f, 0, 0)^\top\).

First, translate the epipole to the origin:

$$
\mathbf{T} = \begin{bmatrix} 1 & 0 & -c_x \\\\ 0 & 1 & -c_y \\\\ 0 & 0 & 1 \end{bmatrix}
$$

where \((c_x, c_y)\) is the image center.

Rotate the translated epipole \(\mathbf{e}' = \mathbf{T}\mathbf{e}_2 = (e'_x, e'_y, e'_w)^\top\) onto the \(x\)-axis:

$$
\mathbf{R}_\theta = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1 \end{bmatrix}
$$

where:

$$
\cos\theta = \frac{e'_x}{\sqrt{e'^2_x + e'^2_y}}, \quad \sin\theta = \frac{e'_y}{\sqrt{e'^2_x + e'^2_y}}
$$

Finally, a projective transformation sends the epipole to infinity:

$$
\mathbf{G} = \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & 1 & 0 \\\\ -1/f & 0 & 1 \end{bmatrix}
$$

where \(f\) is the \(x\)-component of \(\mathbf{R}_\theta \mathbf{T} \mathbf{e}_2\).

The rectification Homography for the right image:

$$
\mathbf{H}_2 = \mathbf{G} \cdot \mathbf{R}_\theta \cdot \mathbf{T}
$$

### 4.3 Step 3: Left Image Homography

\(\mathbf{H}_1\) must satisfy the condition that, after rectification, corresponding points lie on the same horizontal scanline:

$$
\mathbf{H}_2 \mathbf{p}_2 - \mathbf{H}_1 \mathbf{p}_1 = (d, 0, 0)^\top
$$

where \(d\) is the disparity. This is achieved by:

$$
\mathbf{H}_1 = \mathbf{H}_A \cdot \mathbf{H}_2 \cdot \mathbf{M}
$$

where \(\mathbf{M} = [\mathbf{e}_2]_\times \mathbf{F} + \mathbf{e}_2 \mathbf{v}^\top\) (for arbitrary \(\mathbf{v}\))

\(\mathbf{H}_A\) is an affine correction that minimizes the vertical distance between corresponding points:

$$
\mathbf{H}_A = \begin{bmatrix} a & b & c \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}
$$

The parameters \(a, b, c\) are found by solving:

$$
\min_{a,b,c} \sum_i \left( (a \hat{p}^i_{1x} + b \hat{p}^i_{1y} + c) - \hat{p}^i_{2x} \right)^2
$$

where \(\hat{\mathbf{p}}_1 = \mathbf{H}_2 \mathbf{M} \mathbf{p}_1\), \(\hat{\mathbf{p}}_2 = \mathbf{H}_2 \mathbf{p}_2\). This is solvable via **Least Squares**.

---

## 5. After Rectification: Disparity and Depth

Once rectification is complete, corresponding points lie on the same row, and **disparity** can be directly computed:

$$
d = x_1 - x_2
$$

By triangulation, depth is:

$$
\boxed{Z = \frac{f \cdot B}{d}}
$$

where:
- \(f\): Focal length (in pixels)
- \(B\): Baseline length (distance between cameras)
- \(d\): Disparity (in pixels)

### Derivation

```
         f          f
    ◄────────►◄────────►

  ──●────────C₁───────C₂────────●──  ← Image plane
    x₁        │   B    │        x₂
              │         │
              │         │
              │    X    │         ← 3D point
              │   /|\   │
              │  / | \  │
              │ /  |Z \ │
              │/   |   \│
              ●────●────●
                   X
```

From similar triangles:

$$
\frac{x_1}{f} = \frac{X}{Z}, \quad \frac{x_2}{f} = \frac{X - B}{Z}
$$

$$
x_1 - x_2 = d = \frac{f \cdot B}{Z}
$$

$$
\therefore Z = \frac{f \cdot B}{d}
$$

Key observations:
- Disparity and depth are **inversely proportional**
- When \(d = 0\), depth is infinite (very distant point)
- Precise depth estimation requires **sub-pixel disparity accuracy**

---

## 6. Practical Considerations

### 6.1 Lens Distortion Correction

In real systems, **lens undistortion** must be performed before rectification. In OpenCV, `stereoRectify()` and `initUndistortRectifyMap()` combine distortion correction and rectification into a single remapping:

$$
\mathbf{map}(u, v) = \text{undistort}\left(\mathbf{K}^{-1}_{new} \cdot \mathbf{R}_{rect}^{-1} \cdot \begin{pmatrix} u \\\\ v \\\\ 1 \end{pmatrix}\right)
$$

### 6.2 Valid Region (ROI)

Rectification geometrically transforms the images, creating **invalid black regions** in the output. OpenCV's `stereoRectify()` controls this with the `alpha` parameter:

- \(\alpha = 0\): Only valid pixels included (more cropping)
- \(\alpha = 1\): All original pixels included (black borders present)

### 6.3 Quality Verification

To verify rectification was performed correctly:

$$
\text{Rectification Error} = \frac{1}{N}\sum_{i=1}^{N} |y^i_1 - y^i_2|
$$

In ideal rectification, the \(y\)-coordinate difference between corresponding points should be 0. In practice, **less than 1 pixel** is considered good rectification.

---

## 7. Summary

The full Stereo Rectification pipeline:

```
Camera Calibration
       │
       ▼
┌──────────────────┐     ┌──────────────────┐
│  K₁, dist₁, R, t │     │  K₂, dist₂       │
└────────┬─────────┘     └────────┬─────────┘
         │                        │
         ▼                        ▼
   ┌─────────────────────────────────┐
   │    Compute Rectification H₁, H₂ │
   │    (Bouguet or Hartley method)   │
   └──────────────┬──────────────────┘
                  │
         ┌────────┴────────┐
         ▼                 ▼
   ┌───────────┐    ┌───────────┐
   │ Remap L   │    │ Remap R   │
   │ (undist + │    │ (undist + │
   │  rectify) │    │  rectify) │
   └─────┬─────┘    └─────┬─────┘
         │                 │
         ▼                 ▼
   ┌───────────────────────────┐
   │  Stereo Matching          │
   │  (horizontal scanline)    │
   │  → Disparity Map          │
   └─────────────┬─────────────┘
                 │
                 ▼
   ┌───────────────────────────┐
   │  Depth = f·B / disparity  │
   │  → 3D Reconstruction      │
   └───────────────────────────┘
```

Key equations at a glance:

| Item | Equation |
|------|----------|
| Epipolar Constraint | \(\mathbf{p}_2^\top \mathbf{F} \, \mathbf{p}_1 = 0\) |
| Essential Matrix | \(\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}\) |
| Fundamental Matrix | \(\mathbf{F} = \mathbf{K}_2^{-\top}\mathbf{E}\,\mathbf{K}_1^{-1}\) |
| Rectification Homography | \(\mathbf{H} = \mathbf{K}_{new} \mathbf{R}_{align} \mathbf{R}_{rect} \mathbf{K}^{-1}\) |
| Depth from Disparity | \(Z = f \cdot B / d\) |
