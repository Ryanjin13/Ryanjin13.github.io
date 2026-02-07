---
title: "Lens Undistortion: Backward Mapping and Bilinear Interpolation"
date: 2026-02-06
description: "How lens distortion correction works using backward mapping, sub-pixel coordinate calculation, and bilinear interpolation"
categories: ["3D Vision"]
tags: ["Lens Distortion", "Undistortion", "Camera Calibration", "OpenCV", "Image Processing"]
draft: false
---

{{< katex >}}

## Overview

Lens distortion correction (undistortion) is a fundamental preprocessing step in computer vision. This post explains the **backward mapping** approach used in practice, why it's preferred over forward mapping, and how **bilinear interpolation** enables sub-pixel accuracy.

---

## 1. The Problem: Lens Distortion

Real camera lenses introduce geometric distortions that bend straight lines:

```
    Ideal (Pinhole)              Barrel Distortion           Pincushion Distortion
    ┌─────────────┐              ╭─────────────╮              ╭─────────────╮
    │ ┌─────────┐ │              │ ╭─────────╮ │              │ ╱─────────╲ │
    │ │         │ │              │ │         │ │              │╱           ╲│
    │ │    +    │ │              │ │    +    │ │              ││     +     ││
    │ │         │ │              │ │         │ │              │╲           ╱│
    │ └─────────┘ │              │ ╰─────────╯ │              │ ╲─────────╱ │
    └─────────────┘              ╰─────────────╯              ╰─────────────╯
```

To perform accurate 3D reconstruction, we need to **undistort** images to match the ideal pinhole camera model.

---

## 2. Forward vs Backward Mapping

There are two approaches to image transformation:

### 2.1 Forward Mapping (Not Used)

Map each source pixel to its destination location.

```
    Source (Distorted)              Destination (Undistorted)
    ┌───────────────┐               ┌───────────────┐
    │               │               │   ?   ?   ?   │
    │   [A] [B]     │    ────→      │     [A]       │
    │   [C] [D]     │               │ [B]   ?  [C]  │
    │               │               │        [D]    │
    └───────────────┘               └───────────────┘

    Problem: Pixels land at non-integer positions
             → Holes appear between mapped pixels
             → Some destination pixels receive no data
```

**Problems with forward mapping:**
- Destination pixels may be left empty (holes)
- Multiple source pixels may map to the same destination
- Requires expensive hole-filling algorithms

### 2.2 Backward Mapping (Standard Approach)

For each destination pixel, compute which source pixel it came from.

```
    Source (Distorted)              Destination (Undistorted)
    ┌───────────────┐               ┌───────────────┐
    │               │               │               │
    │   ●───────────┼───────────────┼───[A]         │
    │       ●───────┼───────────────┼───────[B]     │
    │           ●───┼───────────────┼───────────[C] │
    │               │    ←────      │               │
    └───────────────┘               └───────────────┘

    For EACH destination pixel:
    "Where in the source image did this pixel come from?"
```

**Advantages of backward mapping:**
- Every destination pixel gets a value (no holes)
- Clean, predictable output
- Easily parallelizable (each output pixel is independent)

---

## 3. The Backward Mapping Process

### 3.1 Step-by-Step Algorithm

For each pixel $(u_{dst}, v_{dst})$ in the undistorted output image:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Step 1: Normalize destination coordinates                                  │
│  ─────────────────────────────────────────                                  │
│                                                                             │
│       x_n = (u_dst - c_x) / f_x                                             │
│       y_n = (v_dst - c_y) / f_y                                             │
│                                                                             │
│  These are coordinates on the normalized image plane (Z = 1)                │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Step 2: Apply distortion model (inverse direction)                         │
│  ──────────────────────────────────────────────────                         │
│                                                                             │
│       r² = x_n² + y_n²                                                      │
│                                                                             │
│       Radial distortion factor:                                             │
│       k_radial = 1 + k₁r² + k₂r⁴ + k₃r⁶                                     │
│                                                                             │
│       x_d = x_n · k_radial + [2p₁x_ny_n + p₂(r² + 2x_n²)]                   │
│       y_d = y_n · k_radial + [p₁(r² + 2y_n²) + 2p₂x_ny_n]                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Step 3: Convert back to pixel coordinates                                  │
│  ──────────────────────────────────────────                                 │
│                                                                             │
│       u_src = f_x · x_d + c_x                                               │
│       v_src = f_y · y_d + c_y                                               │
│                                                                             │
│  ⚠️  These are typically NON-INTEGER values!                                │
│      Example: u_src = 142.37, v_src = 89.72                                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Step 4: Bilinear interpolation                                             │
│  ──────────────────────────────────                                         │
│                                                                             │
│       Sample the 4 neighboring pixels and blend by distance                 │
│       (Explained in detail in Section 4)                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Visual Example

```
    Undistorted Output                    Distorted Source
    (What we want)                        (Original image)

    ┌─────────────────┐                   ┌─────────────────┐
    │                 │                   │                 │
    │    [u,v]=       │                   │   (142.37,      │
    │    [200,150]    │  ──── maps to ──→ │    89.72)       │
    │        ●        │                   │       ○         │
    │                 │                   │                 │
    └─────────────────┘                   └─────────────────┘

    Integer coordinates                   Sub-pixel coordinates!
    in destination                        in source
```

---

## 4. Bilinear Interpolation

Since backward mapping produces **sub-pixel coordinates**, we need to interpolate between neighboring pixels.

### 4.1 The Sub-Pixel Problem

```
    Computed source coordinate: (142.37, 89.72)

    This point falls BETWEEN four pixels:

         col 142      col 143
           │            │
    row 89 ─┼────────────┼─
           │  ●         │      ← Pixel (142, 89)
           │      ○     │      ← Our point (142.37, 89.72)
           │            │
    row 90 ─┼────────────┼─
           │  ●         │      ← Pixel (142, 90)
           │            │

    ● = Actual pixel centers (integer coordinates)
    ○ = Computed sub-pixel location
```

### 4.2 The Four Neighbors

```
    P₀₀ ────────────────── P₁₀
     │                      │
     │    α                 │
     │◄──────►              │
     │        ○ (u,v)       │      α = u - floor(u) = 0.37
     │        │             │      β = v - floor(v) = 0.72
     │        │ β           │
     │        ▼             │
    P₀₁ ────────────────── P₁₁

    P₀₀ = I(142, 89)    P₁₀ = I(143, 89)
    P₀₁ = I(142, 90)    P₁₁ = I(143, 90)
```

### 4.3 Bilinear Interpolation Formula

The interpolated value is computed as a weighted average:

$$
I_{out} = (1-\alpha)(1-\beta) \cdot P_{00} + \alpha(1-\beta) \cdot P_{10} + (1-\alpha)\beta \cdot P_{01} + \alpha\beta \cdot P_{11}
$$

Where:
- $\alpha = u_{src} - \lfloor u_{src} \rfloor$ (horizontal fractional part)
- $\beta = v_{src} - \lfloor v_{src} \rfloor$ (vertical fractional part)

### 4.4 Weight Visualization

```
    The weights are based on OPPOSITE corner distances:

    P₀₀ ─────────────────── P₁₀
     │  weight:              │  weight:
     │  (1-α)(1-β)           │  α(1-β)
     │  = 0.63 × 0.28        │  = 0.37 × 0.28
     │  = 0.176              │  = 0.104
     │         ○             │
     │                       │
    P₀₁ ─────────────────── P₁₁
     │  weight:              │  weight:
     │  (1-α)β               │  αβ
     │  = 0.63 × 0.72        │  = 0.37 × 0.72
     │  = 0.454              │  = 0.266

    Sum of weights = 0.176 + 0.104 + 0.454 + 0.266 = 1.0 ✓
```

### 4.5 Intuition

- Points **closer** to a pixel contribute **more** to the result
- Points **farther** from a pixel contribute **less**
- If the computed point lands exactly on a pixel center, that pixel gets weight 1.0

```
    Example: Point at (142.0, 89.0) exactly on P₀₀

    α = 0.0, β = 0.0

    Weight of P₀₀ = (1-0)(1-0) = 1.0
    Weight of P₁₀ = (0)(1-0) = 0.0
    Weight of P₀₁ = (1-0)(0) = 0.0
    Weight of P₁₁ = (0)(0) = 0.0

    Result = 100% of P₀₀ ✓
```

---

## 5. OpenCV Implementation

### 5.1 The Two-Step Approach

OpenCV separates undistortion into two phases for efficiency:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  INITIALIZATION (Once)                                                      │
│  ─────────────────────                                                      │
│                                                                             │
│  map_x, map_y = cv2.initUndistortRectifyMap(                               │
│      cameraMatrix,    # K (intrinsic matrix)                               │
│      distCoeffs,      # (k₁, k₂, p₁, p₂, k₃)                               │
│      R,               # Rectification rotation (optional)                   │
│      newCameraMatrix, # Output camera matrix                                │
│      size,            # Output image size                                   │
│      m1type           # Map type (CV_32FC1 or CV_16SC2)                    │
│  )                                                                          │
│                                                                             │
│  This computes the source coordinates for EVERY destination pixel          │
│  and stores them in map_x and map_y arrays.                                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │  map_x[v,u] = source x-coordinate
                                    │  map_y[v,u] = source y-coordinate
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  RUNTIME (Every Frame)                                                      │
│  ─────────────────────                                                      │
│                                                                             │
│  undistorted = cv2.remap(                                                   │
│      distorted_image,  # Input image                                        │
│      map_x, map_y,     # Pre-computed coordinate maps                       │
│      interpolation     # cv2.INTER_LINEAR (bilinear)                       │
│  )                                                                          │
│                                                                             │
│  Simply looks up source coordinates and interpolates.                       │
│  Very fast! No distortion math at runtime.                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 Why This is Efficient

```
    Without pre-computation:              With pre-computation:

    For each frame:                       Once at startup:
    ┌─────────────────────┐               ┌─────────────────────┐
    │ For each pixel:     │               │ For each pixel:     │
    │   - Normalize       │               │   - Normalize       │
    │   - Apply distortion│               │   - Apply distortion│
    │   - Denormalize     │               │   - Store in map    │
    │   - Interpolate     │               └─────────────────────┘
    └─────────────────────┘
           │                              For each frame:
           │ 30 FPS                       ┌─────────────────────┐
           ▼                              │ For each pixel:     │
    Very slow!                            │   - Lookup map      │
    ~50ms per frame                       │   - Interpolate     │
                                          └─────────────────────┘
                                                 │
                                                 │ 30+ FPS
                                                 ▼
                                          Very fast!
                                          ~2ms per frame
```

### 5.3 Complete Code Example

```python
import cv2
import numpy as np

# Camera parameters (from calibration)
K = np.array([[800, 0, 320],
              [0, 800, 240],
              [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([-0.2, 0.1, 0.001, -0.001, 0.05])

# Image size
width, height = 640, 480

# ============================================
# STEP 1: Pre-compute maps (ONCE)
# ============================================
map_x, map_y = cv2.initUndistortRectifyMap(
    cameraMatrix=K,
    distCoeffs=dist_coeffs,
    R=None,                    # No rotation
    newCameraMatrix=K,         # Keep same intrinsics
    size=(width, height),
    m1type=cv2.CV_32FC1        # Float maps
)

# ============================================
# STEP 2: Apply to each frame (FAST)
# ============================================
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort using pre-computed maps
    undistorted = cv2.remap(
        frame,
        map_x, map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT
    )

    cv2.imshow('Undistorted', undistorted)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 6. What the Maps Look Like

### 6.1 Map Structure

```
    map_x (same size as output image):
    ┌─────────────────────────────────┐
    │ 0.12   1.15   2.18   3.21  ...  │  ← For row 0, where to sample x
    │ 0.14   1.17   2.20   3.23  ...  │  ← For row 1
    │ 0.16   1.19   2.22   3.25  ...  │
    │  ...    ...    ...    ...       │
    └─────────────────────────────────┘

    map_y (same size as output image):
    ┌─────────────────────────────────┐
    │ 0.08   0.09   0.10   0.11  ...  │  ← For row 0, where to sample y
    │ 1.10   1.11   1.12   1.13  ...  │  ← For row 1
    │ 2.12   2.13   2.14   2.15  ...  │
    │  ...    ...    ...    ...       │
    └─────────────────────────────────┘
```

### 6.2 Visualization

```python
# Visualize the distortion maps
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# map_x shows horizontal displacement
axes[0].imshow(map_x, cmap='jet')
axes[0].set_title('map_x (source x-coordinates)')

# map_y shows vertical displacement
axes[1].imshow(map_y, cmap='jet')
axes[1].set_title('map_y (source y-coordinates)')

plt.show()
```

```
    map_x visualization:              map_y visualization:

    ┌───────────────────┐             ┌───────────────────┐
    │░░░▒▒▒▓▓▓███▓▓▓▒▒▒░│             │░░░░░░░░░░░░░░░░░░░│
    │░░▒▒▒▓▓▓████▓▓▓▒▒░░│             │▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒│
    │░▒▒▒▓▓▓█████▓▓▓▒▒░░│             │▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
    │░▒▒▓▓▓██████▓▓▓▒▒░░│             │████████████████████│
    │░▒▒▓▓▓██████▓▓▓▒▒░░│             │████████████████████│
    │░▒▒▒▓▓▓█████▓▓▓▒▒░░│             │▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
    │░░▒▒▒▓▓▓████▓▓▓▒▒░░│             │▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒│
    │░░░▒▒▒▓▓▓███▓▓▓▒▒▒░│             │░░░░░░░░░░░░░░░░░░░│
    └───────────────────┘             └───────────────────┘

    Gradient left→right               Gradient top→bottom
    (x increases)                     (y increases)
```

---

## 7. Interpolation Methods Comparison

OpenCV's `remap()` supports multiple interpolation methods:

| Method | Speed | Quality | Use Case |
|--------|-------|---------|----------|
| `INTER_NEAREST` | Fastest | Lowest | Masks, labels |
| `INTER_LINEAR` | Fast | Good | **Real-time video** |
| `INTER_CUBIC` | Slow | Better | High-quality stills |
| `INTER_LANCZOS4` | Slowest | Best | Maximum quality |

### 7.1 Nearest Neighbor (Not Recommended)

```
    Uses the single closest pixel:

    P₀₀ ──────────────── P₁₀
     │                    │
     │        ○           │    → Result = P₁₀
     │                    │      (nearest to the point)
    P₀₁ ──────────────── P₁₁

    Problem: Creates blocky artifacts
```

### 7.2 Bilinear (Recommended for Video)

```
    Blends 4 neighbors (as explained above):

    P₀₀ ──────────────── P₁₀
     │    ╲      ╱       │
     │      ╲  ╱         │
     │        ○          │    → Result = weighted blend
     │      ╱  ╲         │
     │    ╱      ╲       │
    P₀₁ ──────────────── P₁₁

    Good balance of speed and quality
```

### 7.3 Bicubic (For High Quality)

```
    Uses 16 neighbors (4×4 grid):

    ● ─── ● ─── ● ─── ●
    │     │     │     │
    ● ─── ● ─── ● ─── ●
    │     │  ○  │     │
    ● ─── ● ─── ● ─── ●
    │     │     │     │
    ● ─── ● ─── ● ─── ●

    Smoother results, but slower
```

---

## 8. Summary

### Key Concepts

1. **Backward Mapping**: For each output pixel, find where it came from in the input
2. **Sub-pixel Coordinates**: Computed source locations are usually non-integer
3. **Bilinear Interpolation**: Blend 4 neighbors based on distance weights
4. **Pre-computed Maps**: Calculate coordinate mappings once, apply quickly per frame

### The Pipeline

```
┌──────────────┐     ┌───────────────────┐     ┌──────────────────┐
│   Distorted  │     │  initUndistort-   │     │   Pre-computed   │
│    Image     │     │   RectifyMap()    │     │   map_x, map_y   │
└──────────────┘     └───────────────────┘     └──────────────────┘
                              │                         │
                              │ (once)                  │ (every frame)
                              ▼                         ▼
                     ┌───────────────────┐     ┌──────────────────┐
                     │  Compute source   │     │     remap()      │
                     │  coordinates for  │     │  + bilinear      │
                     │  each dst pixel   │     │  interpolation   │
                     └───────────────────┘     └──────────────────┘
                                                        │
                                                        ▼
                                               ┌──────────────────┐
                                               │   Undistorted    │
                                               │     Image        │
                                               └──────────────────┘
```

---

## References

1. OpenCV Documentation: [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

2. Bradski, G., & Kaehler, A. (2008). *Learning OpenCV*. O'Reilly Media.

3. Hartley, R., & Zisserman, A. (2003). *Multiple View Geometry in Computer Vision*. Cambridge University Press.
