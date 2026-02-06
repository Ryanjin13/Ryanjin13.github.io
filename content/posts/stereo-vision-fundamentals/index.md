---
title: "Stereo Vision Fundamentals: Complete Mathematical Guide"
date: 2024-07-15
description: "Comprehensive guide to stereo vision mathematics including camera models, epipolar geometry, rectification, and depth estimation"
categories: ["3D Vision"]
tags: ["Stereo Vision", "Epipolar Geometry", "Depth Estimation", "Camera Calibration", "Computer Vision"]
draft: false
---

{{< katex >}}

## Overview

Stereo vision is a fundamental technique in computer vision that enables depth perception by analyzing images from two or more cameras. This comprehensive guide covers the complete mathematical framework from camera models to depth estimation.

---

## 1. Camera Projection Model

### 1.1 The Pinhole Camera Model

The pinhole camera model describes how 3D world points are projected onto a 2D image plane.

```
                    World Point P(X, Y, Z)
                           *
                          /|
                         / |
                        /  |
                       /   |
                      /    |
        Image Plane  /     |
            ┌───────*──────┼─────────────────┐
            │      p(u,v)  |                 │
            │       │      |                 │
            │       │      |                 │
            └───────┼──────┼─────────────────┘
                    │      |
                    │      | Z (depth)
                    │      |
                    O──────┴──────────────→ X
                   /  Camera Center
                  /
                 ↓ Y
```

### 1.2 Projection Equation

The fundamental projection equation in homogeneous coordinates:

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K [R | t] \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$

This can be written more compactly as:

$$
\lambda \mathbf{p} = K [R | t] \mathbf{P}
$$

Where:
- $\mathbf{p} = (u, v, 1)^T$: Image coordinates (homogeneous)
- $\mathbf{P} = (X, Y, Z, 1)^T$: World coordinates (homogeneous)
- $\lambda$: Scale factor (depth)
- $K$: Intrinsic matrix
- $[R|t]$: Extrinsic matrix

### 1.3 Intrinsic Matrix

The intrinsic matrix $K$ encodes the internal camera parameters:

$$
K = \begin{bmatrix} f_x & s & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

| Parameter | Description | Unit |
|-----------|-------------|------|
| $f_x, f_y$ | Focal length | pixels |
| $c_x, c_y$ | Principal point (image center) | pixels |
| $s$ | Skew coefficient (usually 0) | - |

**Physical Interpretation:**

```
                    Image Plane
        ┌─────────────────────────────────┐
        │                                 │
        │                                 │
        │            (cx, cy)             │
        │               *─────────────────┼──→ u
        │               │                 │
        │               │                 │
        │               │                 │
        │               ↓                 │
        └───────────────v─────────────────┘

        Focal length f determines magnification:
        larger f → more zoom (narrower FOV)
```

### 1.4 Extrinsic Matrix

The extrinsic matrix $[R|t]$ represents the camera pose relative to the world:

$$
[R | t] = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix}
$$

- **R**: 3×3 rotation matrix (world → camera)
- **t**: 3×1 translation vector

**Transformation Pipeline:**

```
World Coordinates     Camera Coordinates     Image Coordinates
     (X, Y, Z)    ──────────────────────>    (x, y, z)   ─────────>    (u, v)
                         [R|t]                              K

      P_cam = R · P_world + t              p = K · P_cam / Z
```

---

## 2. Lens Distortion

Real lenses introduce geometric distortions that must be corrected for accurate 3D reconstruction.

### 2.1 Radial Distortion

Radial distortion causes straight lines to appear curved. It's most pronounced near image edges.

**Barrel Distortion** ($k_1 < 0$):
```
    Undistorted          Distorted
    ┌─────────┐         ╭─────────╮
    │         │         │         │
    │    +    │   →     │    +    │
    │         │         │         │
    └─────────┘         ╰─────────╯
```

**Pincushion Distortion** ($k_1 > 0$):
```
    Undistorted          Distorted
    ┌─────────┐         ╭─────────╮
    │         │         ╱         ╲
    │    +    │   →     │    +    │
    │         │         ╲         ╱
    └─────────┘         ╰─────────╯
```

**Mathematical Model:**

$$
\begin{aligned}
x_{distorted} &= x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) \\
y_{distorted} &= y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)
\end{aligned}
$$

Where $r^2 = x^2 + y^2$ is the squared distance from the principal point.

### 2.2 Tangential Distortion

Tangential distortion occurs when the lens is not perfectly parallel to the image sensor.

$$
\begin{aligned}
x_{distorted} &= x + [2p_1 xy + p_2(r^2 + 2x^2)] \\
y_{distorted} &= y + [p_1(r^2 + 2y^2) + 2p_2 xy]
\end{aligned}
$$

**Visualization:**

```
    Ideal Alignment           Tilted Lens
         ┌───┐                   ┌───┐
         │   │                   │   │ ← Lens
         └───┘                   └───┘
           │                       ╱
           │                      ╱ ← Misalignment
         ┌───┐                   ┌───┐
         │   │                   │   │ ← Sensor
         └───┘                   └───┘
```

### 2.3 Complete Distortion Coefficients

$$
\text{distCoeffs} = (k_1, k_2, p_1, p_2, k_3)
$$

In OpenCV, extended models may include $k_4, k_5, k_6$ for fisheye lenses.

---

## 3. Epipolar Geometry

Epipolar geometry describes the geometric relationship between two camera views observing the same 3D scene.

### 3.1 Basic Concept

When a 3D point $P$ is observed by two cameras, the corresponding image points $p$ and $p'$ are constrained to lie on specific lines called **epipolar lines**.

```
                        P (3D Point)
                       /╲
                      / | ╲
                     /  |  ╲
                    /   |   ╲
                   /    |    ╲
                  /     |     ╲
                 /      |      ╲
                /       |       ╲
           ────*────────┼────────*────
              p│        |        │p'
               │        |        │
    Left      e│      Baseline   │e'     Right
    Camera     O_L──────────────O_R      Camera
               │                 │
          Epipole           Epipole
```

**Key Elements:**
- **Baseline**: Line connecting the two camera centers $O_L$ and $O_R$
- **Epipole** ($e$, $e'$): Intersection of baseline with image planes
- **Epipolar Plane**: Plane containing $P$, $O_L$, and $O_R$
- **Epipolar Line**: Intersection of epipolar plane with image plane

### 3.2 Fundamental Matrix

The Fundamental Matrix $F$ is a 3×3 matrix that encodes the epipolar constraint:

$$
\mathbf{p'}^T F \mathbf{p} = 0
$$

**Properties of F:**
- Rank 2 (determinant = 0)
- 7 degrees of freedom
- Maps points to epipolar lines

**Computing Epipolar Lines:**

For a point $p$ in the left image, the corresponding epipolar line $l'$ in the right image is:

$$
l' = F \mathbf{p}
$$

For a point $p'$ in the right image, the epipolar line $l$ in the left image is:

$$
l = F^T \mathbf{p'}
$$

### 3.3 Essential Matrix

The Essential Matrix $E$ is related to the Fundamental Matrix but works with normalized (calibrated) coordinates:

$$
E = [t]_\times R
$$

Where $[t]_\times$ is the skew-symmetric matrix of translation:

$$
[t]_\times = \begin{bmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{bmatrix}
$$

**Properties of E:**
- Rank 2
- Two equal non-zero singular values
- 5 degrees of freedom

### 3.4 Relationship Between F and E

$$
F = K'^{-T} E K^{-1}
$$

$$
E = K'^T F K
$$

Where $K$ and $K'$ are the intrinsic matrices of the left and right cameras respectively.

---

## 4. Stereo Rectification

Stereo rectification transforms the images so that corresponding epipolar lines become horizontal and aligned.

### 4.1 Purpose of Rectification

```
Before Rectification:                After Rectification:
    Left           Right                Left           Right
  ┌───────┐     ┌───────┐            ┌───────┐     ┌───────┐
  │   ╱   │     │   ╲   │            │ ───── │     │ ───── │
  │  ╱    │     │    ╲  │     →      │ ───── │     │ ───── │
  │ ╱     │     │     ╲ │            │ ───── │     │ ───── │
  │╱      │     │      ╲│            │ ───── │     │ ───── │
  └───────┘     └───────┘            └───────┘     └───────┘

  Epipolar lines are          Epipolar lines are horizontal
  at arbitrary angles         → Search along same row only!
```

**Benefits:**
1. Reduces 2D search to 1D (same row)
2. Simplifies stereo matching algorithms
3. Enables efficient hardware implementations

### 4.2 Rectification Transformation

For each camera, we compute a rectification homography $H_L$ and $H_R$:

$$
p_L^{rect} = H_L \cdot p_L
$$

$$
p_R^{rect} = H_R \cdot p_R
$$

### 4.3 New Camera Matrices After Rectification

After rectification, the new projection matrices have a special form:

$$
P_L = K_{rect} [I | 0]
$$

$$
P_R = K_{rect} [I | (-B, 0, 0)^T]
$$

Where:
- $K_{rect}$: Common intrinsic matrix for both rectified images
- $B$: Baseline (horizontal separation between cameras)
- $I$: 3×3 identity matrix

**Rectified Camera Configuration:**

```
        Left Camera              Right Camera
             │                        │
             │                        │
             ▼ Z                      ▼ Z
    ─────────*────────────────────────*─────────
              O_L ◄────── B ──────► O_R

    Both cameras now have:
    - Parallel optical axes
    - Coplanar image planes
    - Horizontal baseline
```

---

## 5. Disparity and Depth

### 5.1 Definition of Disparity

Disparity $d$ is the horizontal difference in image coordinates between corresponding points:

$$
d = u_L - u_R
$$

```
    Left Image                    Right Image
    ┌──────────────┐              ┌──────────────┐
    │              │              │              │
    │      *       │              │          *   │
    │     u_L      │              │         u_R  │
    │              │              │              │
    └──────────────┘              └──────────────┘

    Disparity d = u_L - u_R

    Closer objects → larger disparity
    Farther objects → smaller disparity
```

### 5.2 Depth-Disparity Relationship

The fundamental relationship between depth and disparity:

$$
\boxed{Z = \frac{f \cdot B}{d}}
$$

Where:
- $Z$: Depth (distance from camera)
- $f$: Focal length (pixels)
- $B$: Baseline (meters)
- $d$: Disparity (pixels)

### 5.3 Derivation

Consider a 3D point $P(X, Y, Z)$ observed by two cameras separated by baseline $B$:

```
                     P(X, Y, Z)
                        *
                       /|\
                      / | \
                     /  |  \
                    /   |   \
                   /    |Z   \
                  /     |     \
                 /      |      \
                /       |       \
        ───────*────────┼────────*───────
              p_L       |       p_R
              u_L       |       u_R
               │                 │
               O_L ◄──── B ────► O_R
```

From similar triangles:

**Left camera projection:**
$$
\frac{X}{Z} = \frac{u_L - c_x}{f}
$$

**Right camera projection:**
$$
\frac{X - B}{Z} = \frac{u_R - c_x}{f}
$$

Subtracting the second equation from the first:

$$
\frac{B}{Z} = \frac{(u_L - c_x) - (u_R - c_x)}{f} = \frac{u_L - u_R}{f} = \frac{d}{f}
$$

Therefore:

$$
Z = \frac{f \cdot B}{d}
$$

### 5.4 Depth Resolution

The depth error $\delta Z$ for a small disparity error $\delta d$:

$$
\delta Z = -\frac{f \cdot B}{d^2} \delta d = -\frac{Z^2}{f \cdot B} \delta d
$$

**Key Insight**: Depth error grows quadratically with depth!

```
    Depth Error vs. Distance

    δZ │
       │                              *
       │                           *
       │                        *
       │                     *
       │                 *
       │             *
       │        *
       │    *
       │*
       └──────────────────────────────→ Z

    At 1m depth: small error
    At 10m depth: 100× larger error!
```

---

## 6. Stereo Matching

### 6.1 Problem Definition

For each pixel $(u, v)$ in the rectified left image, find the corresponding pixel $(u-d, v)$ in the right image at the same row.

```
    Left Image (reference)        Right Image (target)
    ┌────────────────────┐        ┌────────────────────┐
    │                    │        │                    │
    │   [*]◄───────────────────────────────────[*]     │
    │    ↑               │        │             ↑      │
    │   (u,v)            │        │          (u-d,v)   │
    │                    │        │                    │
    └────────────────────┘        └────────────────────┘

    Search along the same row (scanline)
    within disparity range [d_min, d_max]
```

### 6.2 Matching Cost Functions

#### Absolute Difference (AD)

$$
C_{AD}(u, v, d) = |I_L(u, v) - I_R(u-d, v)|
$$

Simple pixel-wise comparison.

#### Sum of Absolute Differences (SAD)

$$
C_{SAD}(u, v, d) = \sum_{(i,j) \in W} |I_L(u+i, v+j) - I_R(u+i-d, v+j)|
$$

Uses a window $W$ for more robust matching:

```
    Matching Window
    ┌───────────────┐
    │ . . . . . . . │
    │ . . . . . . . │
    │ . . . * . . . │  ← Center pixel
    │ . . . . . . . │
    │ . . . . . . . │
    └───────────────┘
```

#### Sum of Squared Differences (SSD)

$$
C_{SSD}(u, v, d) = \sum_{(i,j) \in W} [I_L(u+i, v+j) - I_R(u+i-d, v+j)]^2
$$

More sensitive to outliers than SAD.

#### Normalized Cross-Correlation (NCC)

$$
C_{NCC}(u, v, d) = \frac{\sum_{W} (I_L - \bar{I}_L)(I_R - \bar{I}_R)}{\sqrt{\sum_{W}(I_L - \bar{I}_L)^2 \sum_{W}(I_R - \bar{I}_R)^2}}
$$

Invariant to linear intensity changes (gain and bias).

#### Census Transform

The Census transform encodes local pixel relationships as a binary string:

$$
\text{Census}(u, v) = \bigotimes_{(i,j) \in W} \xi(I(u,v), I(u+i, v+j))
$$

Where $\xi(a, b) = 1$ if $a < b$, else $0$.

```
    Original Patch          Census Bit String
    ┌───────────┐
    │ 45  50  40│           For center 52:
    │ 55  52  48│     →     10101010 (binary)
    │ 60  54  47│
    └───────────┘

    Compare: Hamming distance between bit strings
```

**Advantages of Census:**
- Robust to illumination changes
- Handles radiometric differences between cameras
- Preserves edge structure

### 6.3 Semi-Global Matching (SGM)

SGM aggregates costs from multiple directions to enforce smoothness:

$$
S(p, d) = \sum_{r} L_r(p, d)
$$

Path cost for direction $r$:

$$
L_r(p, d) = C(p, d) + \min \begin{cases}
L_r(p-r, d) & \text{same disparity} \\
L_r(p-r, d-1) + P_1 & \text{small change} \\
L_r(p-r, d+1) + P_1 & \text{small change} \\
\min_i L_r(p-r, i) + P_2 & \text{large change}
\end{cases} - \min_k L_r(p-r, k)
$$

**SGM Path Directions (8 or 16 paths):**

```
         ↖  ↑  ↗
          ╲ │ ╱
       ← ──[p]── →
          ╱ │ ╲
         ↙  ↓  ↘

    8 scanline directions
    Aggregate costs from all paths
```

**Parameters:**
- $P_1$: Penalty for small disparity changes (1 pixel)
- $P_2$: Penalty for large disparity changes (> 1 pixel), typically $P_2 > P_1$

---

## 7. Sub-pixel Disparity Estimation

### 7.1 Motivation

Integer pixel disparity limits depth resolution. Sub-pixel interpolation improves accuracy.

### 7.2 Parabola Fitting

Fit a parabola to the cost function around the minimum:

```
    Cost
    │
    │     *
    │    * *
    │   *   *
    │  *     *
    │ *   *   *    ← Parabola fit
    └──────────────→ Disparity
         d-1 d d+1
              ↑
           d_sub (sub-pixel minimum)
```

Given costs $C(d-1)$, $C(d)$, $C(d+1)$ around integer minimum $d$:

$$
d_{sub} = d - \frac{C(d+1) - C(d-1)}{2(C(d+1) - 2C(d) + C(d-1))}
$$

### 7.3 Equiangular Fitting

An alternative formula that handles asymmetric minima:

$$
d_{sub} = d + \frac{C(d-1) - C(d+1)}{2 \max(C(d-1) - C(d), C(d+1) - C(d))}
$$

---

## 8. 3D Reconstruction

### 8.1 From Disparity to 3D

Once we have the disparity map, we can compute 3D coordinates:

$$
Z = \frac{f \cdot B}{d}
$$

$$
X = \frac{(u - c_x) \cdot Z}{f} = \frac{(u - c_x) \cdot B}{d}
$$

$$
Y = \frac{(v - c_y) \cdot Z}{f} = \frac{(v - c_y) \cdot B}{d}
$$

### 8.2 Disparity to Point Cloud Pipeline

```
    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
    │  Disparity  │     │ Reprojection│     │ Point Cloud │
    │    Map      │ ──→ │   Matrix Q  │ ──→ │  (X, Y, Z)  │
    │   d(u,v)    │     │             │     │             │
    └─────────────┘     └─────────────┘     └─────────────┘
```

The reprojection matrix $Q$ (from `cv2.stereoRectify`):

$$
Q = \begin{bmatrix}
1 & 0 & 0 & -c_x \\
0 & 1 & 0 & -c_y \\
0 & 0 & 0 & f \\
0 & 0 & -1/B & 0
\end{bmatrix}
$$

3D point computation:

$$
\begin{bmatrix} X \\ Y \\ Z \\ W \end{bmatrix} = Q \begin{bmatrix} u \\ v \\ d \\ 1 \end{bmatrix}
$$

Then normalize: $(X/W, Y/W, Z/W)$

---

## 9. Coordinate Systems

### 9.1 Image Coordinate System

```
    (0,0)─────────────────────→ u (column)
        │
        │
        │
        │
        │
        ↓
        v (row)
```

- Origin: Top-left corner
- u: Horizontal (column index)
- v: Vertical (row index)

### 9.2 Camera Coordinate System (OpenCV Convention)

```
              Z (forward, optical axis)
              ↑
              │
              │
              │
              O────────→ X (right)
             ╱
            ╱
           ↓
          Y (down)
```

- Origin: Camera optical center
- X: Right
- Y: Down
- Z: Forward (looking direction)

### 9.3 Normalized Coordinates

Normalized coordinates remove the effect of intrinsic parameters:

$$
\hat{x} = \frac{u - c_x}{f_x}, \quad \hat{y} = \frac{v - c_y}{f_y}
$$

These represent the point on the normalized image plane at $Z = 1$.

---

## 10. Summary: Key Formulas

| Concept | Formula |
|---------|---------|
| Projection | $\lambda \mathbf{p} = K[R \mid t]\mathbf{P}$ |
| Epipolar Constraint | $\mathbf{p'}^T F \mathbf{p} = 0$ |
| Essential Matrix | $E = [t]_\times R$ |
| F and E Relationship | $F = K'^{-T} E K^{-1}$ |
| Epipolar Line | $l' = F\mathbf{p}$ |
| Depth from Disparity | $Z = \frac{f \cdot B}{d}$ |
| Depth Error | $\delta Z = -\frac{Z^2}{fB}\delta d$ |

---

## 11. OpenCV Function Reference

| Concept | OpenCV Function |
|---------|-----------------|
| Camera Calibration | `cv2.calibrateCamera()` |
| Stereo Calibration | `cv2.stereoCalibrate()` |
| Rectification Parameters | `cv2.stereoRectify()` |
| Rectification Maps | `cv2.initUndistortRectifyMap()` |
| Apply Rectification | `cv2.remap()` |
| Fundamental Matrix | `cv2.findFundamentalMat()` |
| Essential Matrix | `cv2.findEssentialMat()` |
| Stereo Matching (BM) | `cv2.StereoBM_create()` |
| Stereo Matching (SGBM) | `cv2.StereoSGBM_create()` |
| Disparity to 3D | `cv2.reprojectImageTo3D()` |

---

## 12. Practical Considerations

### 12.1 Choosing Baseline

```
    Small Baseline                    Large Baseline
    ┌───┐     ┌───┐                  ┌───┐         ┌───┐
    │ L │     │ R │                  │ L │         │ R │
    └───┘     └───┘                  └───┘         └───┘
      │◄─ B ─►│                        │◄─── B ────►│

    + Better for close objects       + Better depth resolution
    + Fewer occlusions               + Works for far objects
    - Poor depth at distance         - More occlusions
```

### 12.2 Disparity Range Selection

- **$d_{min}$**: Based on maximum expected depth: $d_{min} = fB/Z_{max}$
- **$d_{max}$**: Based on minimum expected depth: $d_{max} = fB/Z_{min}$

### 12.3 Common Issues

1. **Textureless regions**: Add structured light or use segment-based methods
2. **Occlusions**: Left-right consistency check
3. **Repetitive patterns**: Use larger matching windows
4. **Specular reflections**: Multi-view fusion or polarization filtering

---

## References

1. Hartley, R., & Zisserman, A. (2003). *Multiple View Geometry in Computer Vision*. Cambridge University Press.

2. Szeliski, R. (2010). *Computer Vision: Algorithms and Applications*. Springer.

3. Hirschmüller, H. (2005). "Accurate and Efficient Stereo Processing by Semi-Global Matching and Mutual Information." *CVPR*.

4. Fusiello, A., Trucco, E., & Verri, A. (2000). "A Compact Algorithm for Rectification of Stereo Pairs." *Machine Vision and Applications*.

5. Bouguet, J. Y. (2008). "Camera Calibration Toolbox for Matlab."
