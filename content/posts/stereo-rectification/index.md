---
title: "Stereo Rectification: The Math Behind Stereo Image Alignment"
date: 2026-02-19
description: "A mathematically detailed yet intuitive walkthrough of Stereo Rectification — from Epipolar Geometry fundamentals to Homography computation, with every equation explained step by step"
categories: ["3D Vision"]
tags: ["Stereo Vision", "Rectification", "Epipolar Geometry", "Camera Calibration", "Disparity"]
draft: false
---

{{< katex >}}

## Overview

In stereo vision, estimating depth requires finding **correspondences** between two images — "this pixel in the left image matches that pixel in the right image." In a general stereo camera setup, this correspondence search spans the entire 2D image, making it computationally expensive.

**Rectification** transforms both images so that **all epipolar lines become horizontal**. This reduces the correspondence search from a 2D area to a **1D horizontal scanline**, dramatically improving efficiency.

```
Before Rectification:                After Rectification:
┌──────────┐  ┌──────────┐          ┌──────────┐  ┌──────────┐
│    ╲     │  │     ╱    │          │──────────│  │──────────│
│     ╲    │  │    ╱     │          │──●───────│  │──────●──│
│  ●   ╲   │  │   ╱  ●  │          │──────────│  │──────────│
│       ╲  │  │  ╱       │          │──────────│  │──────────│
│        ╲ │  │ ╱        │          └──────────┘  └──────────┘
└──────────┘  └──────────┘           Search only along horizontal scanlines
 Epipolar lines are tilted
```

After rectification, if a point appears at row 200 in the left image, its match **must** also be at row 200 in the right image. You only need to search along that one row.

---

## 1. Epipolar Geometry: The Foundation

### 1.1 The Physical Setup

Imagine two cameras looking at the same 3D scene. A point \(\mathbf{X}\) in the real world is seen by both cameras:

```
                        X (3D point in the world)
                       /|\
                      / | \
                     /  |  \
                    /   |   \
                   /    |    \
                  /     |     \
        ────────/──────────────\────────
        C₁ (left camera)    C₂ (right camera)
         \     |              |     /
          \    |              |    /
           \   |              |   /
            \  |              |  /
        ┌────\─┤──────┐ ┌────┤─/────┐
        │  x₁  │      │ │    │ x₂  │
        │ (left │      │ │    │(right│
        │  image)      │ │    │image)│
        └──────────────┘ └──────────┘

  C₁, C₂ = camera optical centers
  x₁, x₂ = where the 3D point X appears in each image
  Baseline = the line connecting C₁ and C₂
```

Key terminology:

- **Baseline**: The line connecting the two camera centers \(C_1\) and \(C_2\)
- **Epipolar Plane**: The plane defined by the 3D point \(\mathbf{X}\) and both camera centers \(C_1\), \(C_2\). Every 3D point defines a different epipolar plane, but they all share the baseline.
- **Epipolar Line**: The intersection of the epipolar plane with each image plane. This is where the matching point *must* lie.
- **Epipole** \(e_1\): Where the right camera center \(C_2\) would appear if projected onto the left image (it's where the baseline "pierces" the left image plane)
- **Epipole** \(e_2\): Where the left camera center \(C_1\) would appear if projected onto the right image

**The key property**: If you know a point \(\mathbf{x}_1\) in the left image, its corresponding point \(\mathbf{x}_2\) in the right image must lie somewhere on the epipolar line \(\mathbf{l}_2\). This reduces matching from a 2D search to a 1D search — even before rectification.

### 1.2 The Epipolar Constraint (Informal)

Here is the central geometric fact:

> The three points — the 3D world point \(\mathbf{X}\), and its two projections \(\mathbf{x}_1\) and \(\mathbf{x}_2\) — all lie on the same epipolar plane. This plane also contains both camera centers.

Since three points on a plane are coplanar, we can express this as a mathematical constraint. That constraint turns out to be a single elegant equation — the **epipolar constraint**.

### 1.3 Essential Matrix — Deriving the Constraint

Let's set up coordinates. We place the **left camera at the world origin**, so its coordinate system is the reference frame. The right camera is rotated by \(\mathbf{R}\) and translated by \(\mathbf{t}\) relative to the left camera.

A 3D point \(\mathbf{X}\) has coordinates \(\mathbf{x}_1\) in the left camera frame and \(\mathbf{x}_2\) in the right camera frame. These are **normalized image coordinates** (not pixel coordinates — we'll get to pixels later). They are related by:

$$
\mathbf{x}_2 = \mathbf{R}\mathbf{x}_1 + \mathbf{t}
$$

**What this says**: To express a point from the left camera's viewpoint in the right camera's viewpoint, you first rotate it (\(\mathbf{R}\mathbf{x}_1\)) and then translate it (\(+ \mathbf{t}\)).

Now, the coplanarity condition says that \(\mathbf{x}_2\), \(\mathbf{t}\), and \(\mathbf{R}\mathbf{x}_1\) all lie in the same plane. Three vectors are coplanar when the **scalar triple product** is zero:

$$
\mathbf{x}_2 \cdot (\mathbf{t} \times \mathbf{R}\mathbf{x}_1) = 0
$$

**What this says**: The vector from the right camera center to the 3D point (\(\mathbf{x}_2\)) is perpendicular to the normal of the epipolar plane (\(\mathbf{t} \times \mathbf{R}\mathbf{x}_1\)). The cross product gives the normal to the plane formed by the baseline direction \(\mathbf{t}\) and the ray direction \(\mathbf{R}\mathbf{x}_1\).

We can rewrite the cross product using the **skew-symmetric matrix** notation. For any vector \(\mathbf{t} = (t_x, t_y, t_z)^\top\), the cross product \(\mathbf{t} \times \mathbf{v}\) can be written as the matrix-vector product \([\mathbf{t}]_\times \mathbf{v}\), where:

$$
[\mathbf{t}]_\times = \begin{bmatrix} 0 & -t_z & t_y \\\\ t_z & 0 & -t_x \\\\ -t_y & t_x & 0 \end{bmatrix}
$$

**Why this matrix?** If you multiply it out: \([\mathbf{t}]_\times \mathbf{v}\) gives exactly \(\mathbf{t} \times \mathbf{v}\). This is just a convenient way to express a cross product as a matrix multiplication.

So the coplanarity condition becomes:

$$
\mathbf{x}_2^\top [\mathbf{t}]_\times \mathbf{R} \, \mathbf{x}_1 = 0
$$

We define the **Essential Matrix** as:

$$
\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}
$$

And the epipolar constraint is:

$$
\boxed{\mathbf{x}_2^\top \mathbf{E} \, \mathbf{x}_1 = 0}
$$

**What this says in plain English**: For any pair of corresponding points (the same 3D point seen by both cameras), when you put their normalized coordinates into this equation with the Essential Matrix, the result is always zero. It encodes all the geometric information about the relative pose between the two cameras.

### 1.4 Fundamental Matrix — Moving to Pixel Coordinates

The Essential Matrix works with **normalized image coordinates** (where the camera intrinsics have been "removed"). But in practice, we measure points in **pixel coordinates**. The intrinsic matrix \(\mathbf{K}\) converts between them:

$$
\mathbf{p} = \mathbf{K} \mathbf{x}
$$

where \(\mathbf{p}\) is the pixel coordinate and \(\mathbf{x}\) is the normalized coordinate. The intrinsic matrix looks like:

$$
\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\\\ 0 & f_y & c_y \\\\ 0 & 0 & 1 \end{bmatrix}
$$

**What each element means**:
- \(f_x, f_y\): Focal length in pixels (how many pixels correspond to one unit of distance at depth Z=1)
- \(c_x, c_y\): Principal point — where the optical axis hits the image sensor (usually near image center)

To go from pixels back to normalized coordinates: \(\mathbf{x} = \mathbf{K}^{-1}\mathbf{p}\).

Now substitute into the Essential Matrix equation. For the left camera, \(\mathbf{x}_1 = \mathbf{K}_1^{-1}\mathbf{p}_1\), and for the right camera, \(\mathbf{x}_2 = \mathbf{K}_2^{-1}\mathbf{p}_2\):

$$
(\mathbf{K}_2^{-1}\mathbf{p}_2)^\top \mathbf{E} (\mathbf{K}_1^{-1}\mathbf{p}_1) = 0
$$

Using the transpose property \((\mathbf{A}\mathbf{b})^\top = \mathbf{b}^\top \mathbf{A}^\top\):

$$
\mathbf{p}_2^\top \underbrace{\mathbf{K}_2^{-\top} \mathbf{E} \, \mathbf{K}_1^{-1}}_{\mathbf{F}} \mathbf{p}_1 = 0
$$

The **Fundamental Matrix** is:

$$
\boxed{\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \, \mathbf{K}_1^{-1}}
$$

**What this says**: \(\mathbf{F}\) is just the Essential Matrix "wrapped" with the camera intrinsics, so it works directly with pixel coordinates. The epipolar constraint in pixel space is:

$$
\mathbf{p}_2^\top \mathbf{F} \, \mathbf{p}_1 = 0
$$

**How to find epipolar lines from \(\mathbf{F}\)**:
- Given a point \(\mathbf{p}_1\) in the left image, the epipolar line in the right image is: \(\mathbf{l}_2 = \mathbf{F}\mathbf{p}_1\)
- Given a point \(\mathbf{p}_2\) in the right image, the epipolar line in the left image is: \(\mathbf{l}_1 = \mathbf{F}^\top\mathbf{p}_2\)

A line \(\mathbf{l} = (a, b, c)^\top\) represents the equation \(ax + by + c = 0\). A point \(\mathbf{p}\) lies on line \(\mathbf{l}\) if \(\mathbf{l}^\top \mathbf{p} = 0\).

---

## 2. What Rectification Must Achieve

Now we know the problem: epipolar lines can be tilted at arbitrary angles, making correspondence search expensive. Rectification applies a **warping transformation** (Homography) to each image so that:

1. **All epipolar lines become horizontal** — they are parallel to the image x-axis
2. **Corresponding epipolar lines have the same y-coordinate** — the left and right epipolar lines align vertically
3. **Epipoles move to infinity** — this is what makes the lines parallel (if the epipole is at a finite point, the lines converge toward it)

Mathematically, we want to find two \(3 \times 3\) matrices \(\mathbf{H}_1\) and \(\mathbf{H}_2\) (Homographies) such that the transformed coordinates:

$$
\mathbf{p}'_1 = \mathbf{H}_1 \mathbf{p}_1, \quad \mathbf{p}'_2 = \mathbf{H}_2 \mathbf{p}_2
$$

produce images where corresponding points have **identical y-coordinates**:

$$
p'_{1y} = p'_{2y} \quad \text{for every pair of corresponding points}
$$

After rectification, the new Fundamental Matrix \(\mathbf{F}'\) takes a special form:

$$
\mathbf{F}' = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}
$$

**Why this particular matrix?** Plug any pair of corresponding points into \(\mathbf{p}'_2{}^\top \mathbf{F}' \mathbf{p}'_1 = 0\) and you get:

$$
\begin{bmatrix} x'_2 & y'_2 & 1 \end{bmatrix} \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} x'_1 \\\\ y'_1 \\\\ 1 \end{bmatrix} = -y'_2 + y'_1 = 0
$$

Which gives us exactly \(y'_1 = y'_2\) — the corresponding points are on the same row.

---

## 3. Calibrated Rectification (Bouguet Method)

This is the standard method when you have **calibrated cameras** — meaning you know the intrinsic parameters \(\mathbf{K}_1\), \(\mathbf{K}_2\) and the extrinsic parameters \(\mathbf{R}\), \(\mathbf{t}\) (rotation and translation from camera 1 to camera 2).

The idea is intuitive: **virtually rotate both cameras** so that:
- Their image planes become **coplanar** (same plane)
- Their x-axes are **parallel to the baseline**
- They point in the **same direction**

### Step 1: Split the Rotation Equally

The two cameras are rotated relative to each other by \(\mathbf{R}\). To make them parallel, we need to "undo" this rotation. Bouguet's clever idea: instead of rotating one camera all the way to match the other, **rotate each camera halfway** toward the other. This minimizes the distortion in both images.

First, convert the rotation matrix to a **rotation vector** \(\mathbf{r}\) using Rodrigues' formula:

$$
\mathbf{R} = \exp([\mathbf{r}]_\times)
$$

**What this means**: Any 3D rotation can be represented as a single rotation by angle \(||\mathbf{r}||\) around axis \(\mathbf{r}/||\mathbf{r}||\). The exponential map converts this compact representation into a \(3 \times 3\) rotation matrix.

Now apply half the rotation to each camera, in opposite directions:

$$
\mathbf{r}_{1} = -\frac{\mathbf{r}}{2}, \quad \mathbf{r}_{2} = +\frac{\mathbf{r}}{2}
$$

$$
\mathbf{R}_{rect1} = \exp([\mathbf{r}_1]_\times), \quad \mathbf{R}_{rect2} = \exp([\mathbf{r}_2]_\times)
$$

**What this achieves**: After applying \(\mathbf{R}_{rect1}\) to the left camera and \(\mathbf{R}_{rect2}\) to the right camera, both optical axes point in the **same direction**. The cameras are now parallel — but the baseline might not yet be horizontal.

### Step 2: Make the Baseline Horizontal

After the half-rotation, the baseline direction (in the new rotated frame) is:

$$
\mathbf{t}' = \mathbf{R}_{rect1} \cdot \mathbf{t}
$$

**Why multiply by \(\mathbf{R}_{rect1}\)?** The original translation vector \(\mathbf{t}\) was expressed in the old left-camera frame. After rotating the left camera by \(\mathbf{R}_{rect1}\), we need to express the baseline in the new frame.

Now we build a new coordinate system where this baseline becomes the x-axis. We construct three orthonormal basis vectors:

**New x-axis** (along the baseline):

$$
\mathbf{e}_1 = \frac{\mathbf{t}'}{||\mathbf{t}'||}
$$

This is simply the baseline direction, normalized to unit length.

**New y-axis** (perpendicular to baseline and roughly vertical):

$$
\mathbf{e}_2 = \frac{(-t'_y, t'_x, 0)^\top}{||(-t'_y, t'_x, 0)||}
$$

**How was this chosen?** We want a vector perpendicular to the baseline that has no z-component (so it stays roughly "vertical" in the image). The vector \((-t'_y, t'_x, 0)\) is perpendicular to \((t'_x, t'_y, \cdot)\) in the xy-plane — you can verify: \(\mathbf{e}_1 \cdot \mathbf{e}_2 = t'_x(-t'_y) + t'_y(t'_x) = 0\).

**New z-axis** (completes the right-handed system):

$$
\mathbf{e}_3 = \mathbf{e}_1 \times \mathbf{e}_2
$$

The alignment rotation matrix places these basis vectors as rows:

$$
\mathbf{R}_{align} = \begin{bmatrix} \mathbf{e}_1^\top \\\\ \mathbf{e}_2^\top \\\\ \mathbf{e}_3^\top \end{bmatrix}
$$

**What this matrix does**: When you multiply a vector by \(\mathbf{R}_{align}\), it expresses that vector in the new coordinate system where the baseline is the x-axis.

### Step 3: Assemble the Final Homography

Now we chain everything together. For a pixel \(\mathbf{p}_1\) in the original left image, the rectified pixel is:

$$
\boxed{\mathbf{H}_1 = \mathbf{K}_{new} \cdot \mathbf{R}_{align} \cdot \mathbf{R}_{rect1} \cdot \mathbf{K}_1^{-1}}
$$

$$
\boxed{\mathbf{H}_2 = \mathbf{K}_{new} \cdot \mathbf{R}_{align} \cdot \mathbf{R}_{rect2} \cdot \mathbf{K}_2^{-1}}
$$

**Reading right-to-left, here is what each piece does**:

1. \(\mathbf{K}^{-1}\): **Back-project** — Convert pixel coordinates to normalized camera coordinates (undo the camera intrinsics). This takes us from "pixel space" to "ray direction space."

2. \(\mathbf{R}_{rect}\): **Half-rotate** — Rotate the camera so both cameras' optical axes become parallel. The left camera rotates by \(-\mathbf{r}/2\), the right by \(+\mathbf{r}/2\).

3. \(\mathbf{R}_{align}\): **Align baseline** — Rotate the coordinate system so the baseline (line connecting camera centers) becomes the x-axis. This ensures epipolar lines are horizontal.

4. \(\mathbf{K}_{new}\): **Re-project** — Convert back from normalized coordinates to pixel coordinates using a new intrinsic matrix. This is typically the average of both cameras' intrinsics:

$$
\mathbf{K}_{new} = \frac{\mathbf{K}_1 + \mathbf{K}_2}{2}
$$

**Why average?** To keep the rectified images as similar as possible to the originals, minimizing distortion in both.

---

## 4. Uncalibrated Rectification (Hartley Method)

When camera intrinsics are **unknown**, we can still rectify using only the **Fundamental Matrix** \(\mathbf{F}\), which can be estimated from point correspondences alone (no calibration needed).

### Step 1: Find the Epipoles

The epipoles satisfy:

$$
\mathbf{F} \mathbf{e}_1 = \mathbf{0}, \quad \mathbf{F}^\top \mathbf{e}_2 = \mathbf{0}
$$

**What this means**: The epipole \(\mathbf{e}_1\) is the point where ALL epipolar lines in the left image converge. Mathematically, it's the **null space** of \(\mathbf{F}\) — the vector that \(\mathbf{F}\) maps to zero.

To find it, compute the SVD: \(\mathbf{F} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top\). The last column of \(\mathbf{V}\) gives \(\mathbf{e}_1\), and the last column of \(\mathbf{U}\) gives \(\mathbf{e}_2\).

### Step 2: Send the Epipole to Infinity (Right Image)

If the epipole is at a finite point, the epipolar lines all converge toward it — they fan out like spokes of a wheel. To make them parallel, we need to send the epipole to **infinity** (a "point at infinity" in projective geometry means all lines through it are parallel).

We build this transformation in three stages:

**Stage A — Translate** the image so the epipole is at the origin:

$$
\mathbf{T} = \begin{bmatrix} 1 & 0 & -c_x \\\\ 0 & 1 & -c_y \\\\ 0 & 0 & 1 \end{bmatrix}
$$

\((c_x, c_y)\) is the image center. After this, the epipole is near the origin.

**Stage B — Rotate** so the (translated) epipole lies on the x-axis:

$$
\mathbf{R}_\theta = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1 \end{bmatrix}
$$

where the angle \(\theta\) is chosen so that the rotated epipole \((e'_x, e'_y)\) has \(e'_y = 0\):

$$
\cos\theta = \frac{e'_x}{\sqrt{e'^2_x + e'^2_y}}, \quad \sin\theta = \frac{e'_y}{\sqrt{e'^2_x + e'^2_y}}
$$

**What this does**: After rotation, the epipole sits on the positive x-axis at some distance \(f\) from the origin.

**Stage C — Projective map** that sends the epipole to infinity:

$$
\mathbf{G} = \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & 1 & 0 \\\\ -1/f & 0 & 1 \end{bmatrix}
$$

**How does this work?** In homogeneous coordinates, a point \((x, y, 1)\) maps to \((x, y, 1 - x/f)\). When \(x = f\) (the epipole), the third coordinate becomes 0 — and in projective geometry, a point with zero third coordinate is at infinity. All epipolar lines now become parallel (horizontal).

The right image Homography:

$$
\mathbf{H}_2 = \mathbf{G} \cdot \mathbf{R}_\theta \cdot \mathbf{T}
$$

### Step 3: Compute the Left Image Homography

We need \(\mathbf{H}_1\) such that corresponding points end up on the same horizontal scanline:

$$
\mathbf{H}_2 \mathbf{p}_2 - \mathbf{H}_1 \mathbf{p}_1 = (d, 0, 0)^\top
$$

**What this says**: After transformation, corresponding points differ only in their x-coordinate (by the disparity \(d\)). The y-coordinates and homogeneous coordinates are identical.

The solution involves:

$$
\mathbf{H}_1 = \mathbf{H}_A \cdot \mathbf{H}_2 \cdot \mathbf{M}
$$

where \(\mathbf{M} = [\mathbf{e}_2]_\times \mathbf{F} + \mathbf{e}_2 \mathbf{v}^\top\) maps corresponding points between images, and \(\mathbf{H}_A\) is a small affine correction:

$$
\mathbf{H}_A = \begin{bmatrix} a & b & c \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}
$$

The parameters \(a, b, c\) are found by **least squares**, minimizing the sum of squared vertical differences between transformed corresponding points:

$$
\min_{a,b,c} \sum_i \left( (a \hat{p}^i_{1x} + b \hat{p}^i_{1y} + c) - \hat{p}^i_{2x} \right)^2
$$

**What this optimization does**: It fine-tunes the horizontal alignment so that corresponding points match as closely as possible in the x-direction. The y-direction is already constrained to match; this step handles the remaining x-direction discrepancy.

---

## 5. After Rectification: Disparity and Depth

Once rectification is complete, corresponding points lie on the **same row**. The horizontal difference between them is called **disparity**:

$$
d = x_1 - x_2
$$

**What disparity means physically**: A nearby object appears at very different horizontal positions in the two images (large disparity). A distant object appears at nearly the same position (small disparity). This is exactly like how your two eyes see slightly different views of close objects but nearly identical views of distant mountains.

### Deriving the Depth Equation

Consider the rectified stereo setup:

```
                        X (3D point at depth Z)
                       /|
                      / |
                     /  |
                    /   |
                   /    | Z (depth we want to find)
                  /     |
         ───────/───────────────────
         C₁    |   B    |        C₂      ← Two cameras, separated by baseline B
          \    |        |       /
           \   |        |      /
        ────\──┤────────┤────/──────
            x₁ |        |  x₂            ← Image positions (in pixels)
               |        |
           ◄──f──►  ◄──f──►              ← Focal length f
```

By similar triangles from the left camera:

$$
\frac{x_1}{f} = \frac{X_{world}}{Z}
$$

**What this says**: The ratio of the image position to the focal length equals the ratio of the 3D lateral position to the depth. This is basic perspective projection.

By similar triangles from the right camera (which is shifted by baseline \(B\)):

$$
\frac{x_2}{f} = \frac{X_{world} - B}{Z}
$$

**Why \(X_{world} - B\)?** The right camera is shifted by \(B\) along the baseline, so it sees the 3D point at a different lateral position.

Subtracting the second equation from the first:

$$
\frac{x_1 - x_2}{f} = \frac{X_{world} - (X_{world} - B)}{Z} = \frac{B}{Z}
$$

Since \(d = x_1 - x_2\):

$$
\frac{d}{f} = \frac{B}{Z}
$$

Solving for depth:

$$
\boxed{Z = \frac{f \cdot B}{d}}
$$

**What each variable means**:
- \(Z\): Depth (distance from camera to the 3D point, in meters)
- \(f\): Focal length (in pixels — how many pixels correspond to one unit of angular size)
- \(B\): Baseline (physical distance between cameras, in meters)
- \(d\): Disparity (horizontal pixel difference between corresponding points)

**Key observations**:
- Disparity and depth are **inversely proportional**: close objects have large disparity, far objects have small disparity
- When \(d = 0\), depth is infinite — the point is so far away that both cameras see it at the same position
- When \(d = 1\) pixel, any sub-pixel error causes a large depth error — this is why **sub-pixel disparity accuracy** is critical for distant objects
- Doubling the baseline \(B\) doubles the depth resolution (but also increases the minimum range where both cameras can see the same point)

---

## 6. Practical Considerations

### 6.1 Combining Undistortion and Rectification

Real lenses introduce distortion (barrel, pincushion). In practice, **undistortion** and **rectification** are combined into a single remapping operation for efficiency. OpenCV's `stereoRectify()` computes the rectification transforms, and `initUndistortRectifyMap()` creates the combined remapping:

$$
\mathbf{map}(u, v) = \text{undistort}\left(\mathbf{K}^{-1}_{new} \cdot \mathbf{R}_{rect}^{-1} \cdot \begin{pmatrix} u \\\\ v \\\\ 1 \end{pmatrix}\right)
$$

**What this does**: For each pixel \((u, v)\) in the rectified output, it computes where to sample from the original distorted input image. This is a **backward mapping** — the same principle used in lens undistortion.

### 6.2 Valid Image Region

Rectification warps the images geometrically, which can create black borders where no original pixel data exists. OpenCV's `alpha` parameter controls the trade-off:

- \(\alpha = 0\): Crop aggressively — only show pixels that have valid data in both images
- \(\alpha = 1\): Show everything — keep all original pixels, accept black borders

### 6.3 Verifying Rectification Quality

A simple and effective quality check: measure the average y-coordinate difference between known corresponding points:

$$
\text{Rectification Error} = \frac{1}{N}\sum_{i=1}^{N} |y^i_1 - y^i_2|
$$

If rectification is perfect, every corresponding pair has the same y-coordinate, so this error is zero. In practice, **less than 1 pixel** is considered good, and **less than 0.5 pixel** is excellent.

You can also visually verify by drawing horizontal lines across both rectified images — corresponding features should align along the same line.

---

## 7. Summary

The full pipeline from calibration to 3D reconstruction:

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

| Item | Equation | What It Means |
|------|----------|---------------|
| Epipolar Constraint | \(\mathbf{p}_2^\top \mathbf{F} \mathbf{p}_1 = 0\) | Corresponding points satisfy this geometric relationship |
| Essential Matrix | \(\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}\) | Encodes camera rotation and translation (normalized coords) |
| Fundamental Matrix | \(\mathbf{F} = \mathbf{K}_2^{-\top}\mathbf{E}\mathbf{K}_1^{-1}\) | Same as E but works with pixel coordinates |
| Rectification Homography | \(\mathbf{H} = \mathbf{K}_{new} \mathbf{R}_{align} \mathbf{R}_{rect} \mathbf{K}^{-1}\) | Warps image so epipolar lines become horizontal |
| Depth from Disparity | \(Z = f \cdot B / d\) | Converts pixel displacement to metric depth |
