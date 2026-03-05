---
title: "Day 11 — Camera Geometry and Calibration"
date: 2026-03-05T11:00:00
description: "Pinhole camera model, intrinsic and extrinsic parameters, lens distortion, Zhang's calibration method, homography, and Bird's Eye View transform"
categories: ["Autonomous Driving"]
tags: ["Camera Calibration", "Pinhole Model", "Homography", "Bird's Eye View", "OpenCV"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 11
draft: false
---

{{< katex >}}

## What You'll Learn

In Day 10 we added depth cameras to our autonomous car, capturing both color and depth images. But raw images are **geometrically distorted** by the lens and contain no information about how the camera relates to the physical world. Before we can measure anything from an image — lane width in meters, obstacle position in 3D — we must **calibrate** the camera.

By the end of this post you will be able to:

1. Write down the full pinhole camera projection equation from world coordinates to pixel coordinates.
2. Explain every element of the intrinsic matrix \(K\) and the extrinsic matrix \([R \mid t]\).
3. Derive how radial and tangential distortion warp an image and how to correct it.
4. Understand Zhang's calibration method at a conceptual and mathematical level.
5. Perform camera calibration using OpenCV in Python.
6. Compute a **homography** and apply a **Bird's Eye View (BEV)** transform.
7. Save calibration data in ROS2-compatible YAML format.

---

## 1. The Pinhole Camera Model

### 1.1 Geometry

The pinhole camera is the simplest model of perspective projection. A 3D point \(\mathbf{P}_w = (X, Y, Z)\) in world coordinates is projected onto a 2D image point \(\mathbf{p} = (u, v)\) by passing all light rays through a single point — the **optical center** (or camera center).

```
  World point P = (X, Y, Z)
         *
          \
           \     light ray
            \
             \
              \
    ───────────O───────────── optical axis (Z_c)
               │\             (optical center)
               │ \
               │  * image point p = (u, v)
               │
          Image Plane (at focal length f from O)
```

By similar triangles, the projection equations in the camera coordinate frame are:

$$
u' = f \cdot \frac{X_c}{Z_c}, \qquad v' = f \cdot \frac{Y_c}{Z_c}
$$

where \((u', v')\) are coordinates in the image plane (in metric units, e.g., millimeters), \(f\) is the focal length, and \((X_c, Y_c, Z_c)\) is the 3D point expressed in camera coordinates.

### 1.2 From Metric to Pixel Coordinates

Real cameras have discrete pixels, not continuous coordinates. The conversion involves:

- **Focal length in pixels**: \(f_x = f / s_x\) and \(f_y = f / s_y\), where \(s_x, s_y\) are the physical pixel sizes (mm/pixel).
- **Principal point**: \((c_x, c_y)\), the pixel where the optical axis intersects the image plane. Ideally at the image center, but not exactly due to manufacturing tolerances.

The full projection in pixel coordinates:

$$
u = f_x \cdot \frac{X_c}{Z_c} + c_x, \qquad v = f_y \cdot \frac{Y_c}{Z_c} + c_y
$$

### 1.3 The Full Projection Equation (Matrix Form)

We express this elegantly using **homogeneous coordinates**. A 3D point becomes \(\tilde{\mathbf{P}}_w = [X, Y, Z, 1]^T\) and a 2D point becomes \(\tilde{\mathbf{p}} = [u, v, 1]^T\) (up to a scale factor):

$$
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K}
\underbrace{\begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix}}_{[R \mid t]}
\begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$

Or compactly:

$$
\boxed{s\,\tilde{\mathbf{p}} = K \, [R \mid t] \, \tilde{\mathbf{P}}_w}
$$

where:

- \(s\) is an arbitrary scale factor (equal to the depth \(Z_c\) of the point in the camera frame),
- \(K\) is the \(3 \times 3\) **intrinsic matrix** (camera internal parameters),
- \([R \mid t]\) is the \(3 \times 4\) **extrinsic matrix** (camera pose in the world),
- The product \(P = K[R \mid t]\) is the \(3 \times 4\) **projection matrix**.

This single equation is the foundation of all camera-based measurement. Every formula in computer vision — stereo, SfM, SLAM — starts here.

---

## 2. Intrinsic Matrix K — What's Inside the Camera

$$
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

| Parameter | Physical meaning | Typical value (640x480 camera) |
|-----------|-----------------|-------------------------------|
| \(f_x\) | Focal length in pixels (horizontal) | 400 -- 800 |
| \(f_y\) | Focal length in pixels (vertical) | 400 -- 800 |
| \(c_x\) | Principal point x-coordinate | ~320 (half of width) |
| \(c_y\) | Principal point y-coordinate | ~240 (half of height) |

**Notes on each parameter**:

- \(f_x \neq f_y\) in general, because pixels may not be perfectly square. In practice, the difference is often less than 1%.
- Some formulations include a **skew** parameter \(\gamma\) in position \(K[0,1]\), but for modern cameras this is essentially zero.
- \(K\) is fixed once the camera lens and sensor are manufactured. It does not change with camera position or orientation. Zoom lenses change \(f_x, f_y\) — fixed-focus cameras have constant \(K\).

### 2.1 Field of View

The horizontal field of view (FOV) relates to \(f_x\) and the image width \(W\):

$$
\text{FOV}_x = 2 \arctan\!\left(\frac{W}{2 f_x}\right)
$$

Similarly for vertical:

$$
\text{FOV}_y = 2 \arctan\!\left(\frac{H}{2 f_y}\right)
$$

For \(f_x = 600\) and \(W = 640\):

$$
\text{FOV}_x = 2 \arctan\!\left(\frac{640}{1200}\right) \approx 2 \times 28.1° = 56.1°
$$

A wider FOV (shorter focal length) sees more of the scene but with more distortion and lower angular resolution. A narrower FOV (longer focal length) provides higher angular resolution but a smaller viewing area.

### 2.2 Example: Raspberry Pi Camera v2

```
Resolution:   3280 x 2464
Sensor size:  3.68 x 2.76 mm
Focal length: 3.04 mm
Pixel size:   3.68 / 3280 = 0.00112 mm/px

f_x = 3.04 / 0.00112 = 2714 px
f_y = 3.04 / 0.00112 = 2714 px
c_x = 3280 / 2 = 1640 px
c_y = 2464 / 2 = 1232 px

K = [2714    0   1640]
    [   0  2714  1232]
    [   0     0     1]

FOV_x = 2 * arctan(3280 / (2*2714)) = 62.2 degrees
```

---

## 3. Extrinsic Matrix [R | t] — Where Is the Camera?

The extrinsic parameters define the rigid-body transformation from the **world coordinate frame** to the **camera coordinate frame**:

$$
\mathbf{P}_c = R \, \mathbf{P}_w + t
$$

where:

- \(R\) is a \(3 \times 3\) **rotation matrix** (\(R^T R = I\), \(\det(R) = 1\)),
- \(t\) is a \(3 \times 1\) **translation vector**.

### 3.1 Understanding R and t

```
  World Frame (W)              Camera Frame (C)

  Z_w (up)                     Z_c (forward / optical axis)
   |                            |
   |                            |
   |______ X_w (east)           |______ X_c (right in image)
  /                            /
 Y_w (north)                  Y_c (down in image)
```

The camera's \(Z_c\) axis points along the optical axis (into the scene). The \(X_c\) axis points right in the image, and \(Y_c\) points down. This is the standard computer vision convention.

**Important**: \(t\) is NOT the camera's position in the world. \(t\) is the position of the **world origin expressed in camera coordinates**. The camera's position in world coordinates is:

$$
\mathbf{C}_w = -R^T t
$$

This is a common source of confusion. Always ask: "in which coordinate frame is this vector expressed?"

### 3.2 Rotation Representations

The rotation matrix \(R\) has 9 elements but only 3 degrees of freedom (because of the orthogonality constraints \(R^T R = I\)). Common representations:

| Representation | Parameters | Pros | Cons |
|---------------|-----------|------|------|
| Rotation matrix | 9 (constrained to 3) | Compose by multiplication | Redundant, numerical drift |
| Euler angles (roll, pitch, yaw) | 3 | Intuitive | Gimbal lock |
| Rodrigues vector | 3 | Compact, OpenCV uses this | Less intuitive |
| Quaternion | 4 | No gimbal lock, smooth interpolation | 4 params for 3 DOF |

OpenCV's `calibrateCamera` returns rotation as **Rodrigues vectors** (3x1). Convert to rotation matrix with `cv2.Rodrigues()`.

### 3.3 Degrees of Freedom Summary

| Component | Parameters | DOF |
|-----------|-----------|-----|
| Intrinsic \(K\) | \(f_x, f_y, c_x, c_y\) | 4 (or 5 with skew) |
| Distortion \(D\) | \(k_1, k_2, p_1, p_2, k_3\) | 5 |
| Extrinsic per view | \(R, t\) | 6 (3 rotation + 3 translation) |
| Total for \(n\) views | | \(9 + 6n\) |

---

## 4. Lens Distortion

Real lenses are not ideal pinholes. They introduce geometric distortion that must be corrected before the pinhole model applies.

### 4.1 Normalized Image Coordinates

Before applying distortion, convert pixel coordinates to **normalized** image coordinates by removing the intrinsic matrix:

$$
x_n = \frac{u - c_x}{f_x}, \qquad y_n = \frac{v - c_y}{f_y}
$$

and define:

$$
r^2 = x_n^2 + y_n^2
$$

### 4.2 Radial Distortion

Caused by the spherical shape of lens elements. Points farther from the image center are displaced radially:

$$
x_{\text{radial}} = x_n(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)
$$
$$
y_{\text{radial}} = y_n(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)
$$

**Barrel distortion** (\(k_1 < 0\)): straight lines bow outward. Common in wide-angle and fisheye lenses.

**Pincushion distortion** (\(k_1 > 0\)): straight lines bow inward. Common in telephoto lenses.

```
  Barrel                No distortion          Pincushion
  (k1 < 0)                                    (k1 > 0)

  ┌──────────┐          ┌──────────┐          ┌──────────┐
  │ (      ) │          │ |      | │          │ )      ( │
  │(        )│          │ |      | │          │)        (│
  │(        )│          │ |      | │          │)        (│
  │ (      ) │          │ |      | │          │ )      ( │
  └──────────┘          └──────────┘          └──────────┘
  Straight lines        Straight lines        Straight lines
  bow outward           remain straight       bow inward
```

### 4.3 Tangential Distortion

Caused by lens elements not being perfectly centered on the optical axis (decentering). This introduces asymmetric distortion:

$$
x_{\text{tangential}} = 2p_1 x_n y_n + p_2(r^2 + 2x_n^2)
$$
$$
y_{\text{tangential}} = p_1(r^2 + 2y_n^2) + 2p_2 x_n y_n
$$

Tangential distortion is usually much smaller than radial distortion, but ignoring it can introduce sub-pixel errors that matter for precision applications.

### 4.4 Complete Distortion Model

Combining both radial and tangential distortion:

$$
x' = x_n(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 x_n y_n + p_2(r^2 + 2x_n^2)
$$
$$
y' = y_n(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y_n^2) + 2p_2 x_n y_n
$$

Then convert back to pixel coordinates:

$$
u_{\text{distorted}} = f_x \cdot x' + c_x, \qquad v_{\text{distorted}} = f_y \cdot y' + c_y
$$

The distortion coefficients vector in OpenCV convention:

$$
D = [k_1, \; k_2, \; p_1, \; p_2, \; k_3]
$$

Five coefficients are typically sufficient for standard lenses. Fisheye lenses need a different model (equidistant / Kannala-Brandt) with its own set of coefficients.

### 4.5 Why Distortion Matters for Our Car

If we do not correct distortion:

- **Lane lines** appear curved when they are actually straight — the BEV transform produces a warped top-down view.
- **Distance measurements** from pixel positions are wrong — a 3.7 m lane appears wider or narrower depending on position in the image.
- **Feature matching** in SLAM (Day 12) degrades — the same 3D point maps to different pixel locations depending on where it appears in the frame.

---

## 5. Zhang's Calibration Method

Zhengyou Zhang's 1999 paper introduced the standard approach for camera calibration that is used by virtually every robotics project today. It requires only a planar calibration pattern (checkerboard) photographed from multiple angles.

### 5.1 The Key Insight

For a planar calibration pattern, we can set \(Z = 0\) without loss of generality. The projection equation simplifies:

$$
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= K \begin{bmatrix} r_1 & r_2 & r_3 & t \end{bmatrix}
\begin{bmatrix} X \\ Y \\ 0 \\ 1 \end{bmatrix}
= K \begin{bmatrix} r_1 & r_2 & t \end{bmatrix}
\begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}
$$

The third column of \(R\) drops out because \(Z = 0\). The remaining \(3 \times 3\) matrix:

$$
H = K \begin{bmatrix} r_1 & r_2 & t \end{bmatrix}
$$

is a **homography** — a projective mapping from the 2D calibration plane to the 2D image.

### 5.2 Extracting Constraints from Orthogonality

Since \(r_1\) and \(r_2\) are columns of a rotation matrix, they must satisfy:

$$
r_1^T r_2 = 0 \qquad \text{(orthogonality)}
$$
$$
\|r_1\| = \|r_2\| \qquad \text{(unit norm)}
$$

Substituting \(r_1 = K^{-1} h_1\) and \(r_2 = K^{-1} h_2\) (where \(h_1, h_2\) are the first two columns of \(H\)):

$$
h_1^T K^{-T} K^{-1} h_2 = 0
$$
$$
h_1^T K^{-T} K^{-1} h_1 = h_2^T K^{-T} K^{-1} h_2
$$

Define \(B = K^{-T} K^{-1}\), a symmetric \(3 \times 3\) matrix with 6 unique entries. Each image gives 2 equations in the entries of \(B\). With \(n \geq 3\) images, we get enough equations to solve for \(B\) (and hence \(K\)).

### 5.3 The Full Algorithm

1. **Detect corners**: find the checkerboard inner corners in each image to sub-pixel accuracy using `cv2.findChessboardCorners` + `cv2.cornerSubPix`.

2. **Estimate homography**: for each image \(i\), compute \(H_i\) from the corner correspondences using DLT (Direct Linear Transform).

3. **Extract intrinsics**: solve for \(B\) using the orthogonality constraints from all images, then factor \(B = K^{-T} K^{-1}\) to recover \(K\) via Cholesky decomposition.

4. **Extract extrinsics**: for each image, compute \([R_i \mid t_i]\) from \(H_i\) and \(K\):
   $$r_1 = \lambda K^{-1} h_1, \quad r_2 = \lambda K^{-1} h_2, \quad r_3 = r_1 \times r_2, \quad t = \lambda K^{-1} h_3$$
   where \(\lambda = 1 / \|K^{-1} h_1\|\).

5. **Refine with bundle adjustment**: minimize the total reprojection error over all parameters simultaneously using Levenberg-Marquardt optimization:

$$
\min_{K, D, \{R_i, t_i\}} \sum_{i=1}^{n} \sum_{j=1}^{m} \left\|\mathbf{p}_{ij} - \hat{\mathbf{p}}(K, D, R_i, t_i, \mathbf{P}_j)\right\|^2
$$

where \(\mathbf{p}_{ij}\) is the detected corner position and \(\hat{\mathbf{p}}\) is the reprojected position using the current parameter estimates.

### 5.4 Practical Requirements

| Requirement | Recommendation |
|-------------|---------------|
| Number of images | 10 -- 30 |
| Pattern size | 7x5 or 9x6 inner corners |
| Pattern variety | Vary position, angle, distance |
| Tilt range | Include images tilted 30-45 degrees |
| Corner detection | Sub-pixel refinement mandatory |
| Expected reprojection error | < 0.5 px (good), < 0.3 px (excellent) |
| Pattern flatness | Glue to rigid board; warped patterns ruin calibration |

### 5.5 Reprojection Error — The Quality Metric

For each detected corner, compute:

$$
\text{RMS Reprojection Error} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left[(u_i - \hat{u}_i)^2 + (v_i - \hat{v}_i)^2\right]}
$$

This measures the average pixel distance between where the model predicts each corner should appear and where it was actually detected.

| Error level | Quality |
|-------------|---------|
| < 0.3 px | Excellent |
| 0.3 -- 0.5 px | Good |
| 0.5 -- 1.0 px | Acceptable |
| > 1.0 px | Poor — check pattern flatness, detection quality |

---

## 6. Homography

### 6.1 Definition

A **homography** (or projective transformation) is a \(3 \times 3\) invertible matrix \(H\) that maps points on one plane to points on another plane, expressed in homogeneous coordinates:

$$
s \begin{bmatrix} u' \\ v' \\ 1 \end{bmatrix}
= H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix}
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

\(H\) has 8 degrees of freedom (9 entries minus 1 for overall scale). Each point correspondence provides 2 equations, so **4 non-collinear point correspondences** determine \(H\) uniquely.

### 6.2 Computing H: Direct Linear Transform (DLT)

Given \(n \geq 4\) correspondences \((u_i, v_i) \leftrightarrow (u_i', v_i')\), each pair gives two equations:

$$
\begin{bmatrix}
-u_i & -v_i & -1 & 0 & 0 & 0 & u_i u_i' & v_i u_i' & u_i' \\
0 & 0 & 0 & -u_i & -v_i & -1 & u_i v_i' & v_i v_i' & v_i'
\end{bmatrix}
\begin{bmatrix} h_{11} \\ h_{12} \\ \vdots \\ h_{33} \end{bmatrix} = 0
$$

Stack all equations into \(A\mathbf{h} = 0\) and solve via SVD: \(\mathbf{h}\) is the last column of \(V\) in the SVD of \(A\).

### 6.3 Applications in Autonomous Driving

| Application | Source plane | Destination plane |
|-------------|-------------|-------------------|
| Camera calibration | Checkerboard plane (3D) | Image plane (2D) |
| Bird's Eye View | Road surface (camera view) | Top-down view |
| Image stitching | Image 1 pixels | Image 2 pixels (panorama) |
| Augmented reality | Real-world plane | Screen overlay |
| Lane detection | Camera perspective | Rectified lane view |

---

## 7. Bird's Eye View (BEV) Transform

### 7.1 Why BEV?

In a front-facing camera image, parallel lane lines converge toward a vanishing point due to perspective projection. This makes it difficult to measure lane width, curvature, or the car's lateral offset. A BEV transform removes perspective, giving a top-down view where parallel lines remain parallel.

```
  Front camera view:              Bird's Eye View:

      ╲          ╱                 |              |
       ╲        ╱                  |              |
        ╲      ╱                   |              |
         ╲    ╱                    |              |
          ╲  ╱                     |              |
           ╲╱   <-- vanishing     |              |
           ╱╲       point          |              |
          ╱  ╲                     |              |
         ╱    ╲                    |              |
        ╱      ╲                   |              |
       ╱________╲                  |______________|

  Lanes converge                  Lanes are parallel
  (hard to measure width)        (easy to measure width)
```

### 7.2 Computing the BEV Homography

We define 4 source points forming a trapezoid on the road in the camera image and 4 destination points forming a rectangle in the BEV image.

```
  Camera Image:                    BEV Image:

  (src[0])──────────(src[1])       (dst[0])────────(dst[1])
      \                /            |                    |
       \              /             |                    |
        \            /              |                    |
         \          /               |                    |
  (src[3])──(src[2])               (dst[3])────────(dst[2])
```

The homography is:

$$
H_{\text{BEV}} = \texttt{cv2.getPerspectiveTransform}(\text{src}, \text{dst})
$$

And the inverse (for projecting BEV coordinates back to camera):

$$
H_{\text{BEV}}^{-1} = \texttt{cv2.getPerspectiveTransform}(\text{dst}, \text{src})
$$

### 7.3 Metric Scaling

If you know the real-world distances between the source points (e.g., lane width = 3.7 m, dashed line spacing = 3 m), you can set the destination points such that each pixel in the BEV image corresponds to a known physical distance:

$$
\text{pixels\_per\_meter} = \frac{\text{BEV image width [px]}}{\text{real-world width [m]}}
$$

For example, with a 640-pixel-wide BEV image covering 6 m of road width:

$$
\text{pixels\_per\_meter} = \frac{640}{6} \approx 107 \text{ px/m}
$$

This means you can directly measure distances in the BEV image by counting pixels.

### 7.4 Limitations of BEV

- **Only valid on the road plane**: objects above the road (cars, pedestrians) appear stretched and distorted.
- **Sensitive to camera mounting**: small changes in camera pitch angle significantly affect the BEV mapping.
- **Far regions are low resolution**: pixels near the horizon map to large areas in BEV, producing blurry results.

---

## 8. Depth Camera Calibration Notes

### 8.1 Depth Scale

Depth cameras report depth values as integers (typically uint16). The **depth scale** converts raw values to meters:

$$
d_{\text{meters}} = d_{\text{raw}} \times \text{depth\_scale}
$$

| Camera | depth_scale | Raw value 1000 = |
|--------|-------------|------------------|
| Intel RealSense D435 | 0.001 | 1.0 m |
| Microsoft Kinect v2 | 0.001 | 1.0 m |
| Orbbec Astra | 0.001 | 1.0 m |

### 8.2 RGB-Depth Alignment (Recap from Day 10)

The RGB camera and depth sensor are physically offset by a baseline distance. To fuse color and depth, we must project them into a common frame. Two options:

1. **Align depth to color** (most common): reproject each depth pixel into the color camera frame using the known extrinsic transform between sensors.
2. **Align color to depth**: reproject each color pixel into the depth camera frame.

The RealSense SDK provides `rs.align(rs.stream.color)` for option 1. After alignment, pixel \((u, v)\) in the color image has the correct depth value from the aligned depth image.

### 8.3 3D Reconstruction from Depth (Backprojection)

Given a calibrated camera with intrinsic matrix \(K\) and a depth value \(d\) at pixel \((u, v)\), the 3D point in camera coordinates is obtained by inverting the projection:

$$
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
= d \cdot K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= d \begin{bmatrix} (u - c_x) / f_x \\ (v - c_y) / f_y \\ 1 \end{bmatrix}
$$

This is the **inverse projection** (also called backprojection or deprojection). It is the foundation of:

- **Point cloud generation**: backproject every pixel to get a dense 3D point cloud.
- **SLAM** (Day 12): 3D-to-3D registration between frames.
- **Obstacle detection**: determine the 3D position of objects relative to the car.

---

## 9. Hands-On Lab: Camera Calibration with OpenCV

### 9.1 Capturing Calibration Images

```python
"""
capture_calibration.py
Capture checkerboard images for camera calibration.
Press 's' to save when pattern is detected, 'q' to quit.
"""

import cv2
import os
import time


def capture_calibration_images(camera_index=0, save_dir='calibration_images',
                                pattern_size=(9, 6)):
    """
    Interactive tool for capturing calibration images.

    Shows live preview with checkerboard detection overlay.
    Only allows saving when the pattern is successfully found.
    """
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Error: cannot open camera")
        return

    count = 0
    last_save = 0

    print("=" * 60)
    print("Camera Calibration Image Capture")
    print("=" * 60)
    print(f"Pattern size: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print(f"Save directory: {save_dir}")
    print()
    print("Instructions:")
    print("  1. Hold checkerboard in front of camera")
    print("  2. When green overlay appears, press 's' to save")
    print("  3. Move board to different angle/distance, repeat")
    print("  4. Aim for 15-25 images from varied poses")
    print("  5. Press 'q' to quit")
    print()
    print("Tips for good calibration:")
    print("  - Cover all regions of the image (corners too!)")
    print("  - Include tilted views (30-45 degree angles)")
    print("  - Vary the distance (close, medium, far)")
    print("  - Keep the board flat and still when saving")
    print("=" * 60)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()

        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            cv2.drawChessboardCorners(display, pattern_size,
                                      corners_refined, found)
            cv2.putText(display, "PATTERN FOUND - press 's' to save",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)
        else:
            cv2.putText(display, "Pattern not detected...",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)

        cv2.putText(display, f"Images saved: {count}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 0), 2)
        cv2.imshow('Calibration Capture', display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') and found and (time.time() - last_save > 0.5):
            filename = os.path.join(save_dir, f'calib_{count:03d}.png')
            cv2.imwrite(filename, frame)
            print(f"  Saved {filename}")
            count += 1
            last_save = time.time()
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nTotal images saved: {count}")
    if count < 10:
        print("WARNING: At least 10 images recommended for good calibration.")


if __name__ == "__main__":
    capture_calibration_images()
```

### 9.2 Performing Calibration

```python
"""
calibrate_camera.py
Calibrate camera from checkerboard images using Zhang's method (via OpenCV).
Reports per-image errors and overall quality metrics.
"""

import cv2
import numpy as np
import glob
import os
import yaml


def calibrate(image_dir='calibration_images',
              pattern_size=(9, 6),
              square_size_mm=25.0):
    """
    Calibrate camera from checkerboard images.

    Args:
        image_dir:     directory containing calibration PNG images
        pattern_size:  (columns, rows) of inner checkerboard corners
        square_size_mm: physical size of each square in mm

    Returns:
        ret:    RMS reprojection error
        K:      3x3 intrinsic matrix
        D:      1x5 distortion coefficients
        rvecs:  list of rotation vectors (one per image)
        tvecs:  list of translation vectors (one per image)
    """
    # --- Prepare 3D object points ---
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0],
                            0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_mm

    obj_points = []
    img_points = []
    img_shape = None

    # --- Process each image ---
    images = sorted(glob.glob(os.path.join(image_dir, '*.png')))
    if not images:
        images = sorted(glob.glob(os.path.join(image_dir, '*.jpg')))
    print(f"Found {len(images)} images in {image_dir}/")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if img_shape is None:
            img_shape = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            print(f"  [OK]   {os.path.basename(fname)}: "
                  f"{len(corners_refined)} corners detected")
        else:
            print(f"  [SKIP] {os.path.basename(fname)}: pattern not found")

    if len(obj_points) < 3:
        raise ValueError(
            f"Need at least 3 valid images, found only {len(obj_points)}"
        )

    print(f"\nCalibrating with {len(obj_points)} valid images...")

    # --- Run calibration ---
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None
    )

    # --- Report results ---
    print(f"\n{'='*60}")
    print(f"CALIBRATION RESULTS")
    print(f"{'='*60}")
    print(f"\nRMS Reprojection Error: {ret:.4f} pixels")

    if ret < 0.3:
        quality = "EXCELLENT"
    elif ret < 0.5:
        quality = "GOOD"
    elif ret < 1.0:
        quality = "ACCEPTABLE"
    else:
        quality = "POOR - check pattern and images"
    print(f"Quality: {quality}")

    print(f"\nIntrinsic Matrix K:")
    print(f"  fx = {K[0, 0]:.2f} px")
    print(f"  fy = {K[1, 1]:.2f} px")
    print(f"  cx = {K[0, 2]:.2f} px")
    print(f"  cy = {K[1, 2]:.2f} px")

    w, h = img_shape
    fov_x = 2 * np.arctan(w / (2 * K[0, 0])) * 180 / np.pi
    fov_y = 2 * np.arctan(h / (2 * K[1, 1])) * 180 / np.pi
    print(f"\nField of View:")
    print(f"  Horizontal: {fov_x:.1f} degrees")
    print(f"  Vertical:   {fov_y:.1f} degrees")

    print(f"\nDistortion Coefficients D:")
    print(f"  k1 = {D[0, 0]:+.6f}  (radial)")
    print(f"  k2 = {D[0, 1]:+.6f}  (radial)")
    print(f"  p1 = {D[0, 2]:+.6f}  (tangential)")
    print(f"  p2 = {D[0, 3]:+.6f}  (tangential)")
    print(f"  k3 = {D[0, 4]:+.6f}  (radial)")

    # --- Per-image reprojection errors ---
    print(f"\nPer-image reprojection errors:")
    errors = []
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        reproj, _ = cv2.projectPoints(obj_points[i], rvec, tvec, K, D)
        error = cv2.norm(img_points[i], reproj, cv2.NORM_L2) / len(reproj)
        errors.append(error)
        marker = " ***" if error > 1.0 else ""
        print(f"  Image {i:3d}: {error:.4f} px{marker}")

    print(f"\n  Mean:   {np.mean(errors):.4f} px")
    print(f"  Median: {np.median(errors):.4f} px")
    print(f"  Max:    {np.max(errors):.4f} px")

    return ret, K, D, rvecs, tvecs, img_shape


if __name__ == "__main__":
    ret, K, D, rvecs, tvecs, img_shape = calibrate()
```

### 9.3 Distortion Correction: Before and After

```python
"""
undistort_demo.py
Apply distortion correction and compare different alpha values.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt


def undistort_comparison(image_path, K, D, img_shape):
    """
    Show original vs undistorted with different alpha values.

    alpha controls the tradeoff:
      alpha=0: all output pixels are valid (some FOV is lost)
      alpha=1: all source pixels retained (black borders appear)
    """
    img = cv2.imread(image_path)
    h, w = img.shape[:2]

    # Simple undistort
    undistorted_simple = cv2.undistort(img, K, D)

    # Optimal alpha=0 (no black borders)
    new_K_0, roi_0 = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
    undistorted_a0 = cv2.undistort(img, K, D, None, new_K_0)

    # Optimal alpha=0.5 (balanced)
    new_K_05, roi_05 = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0.5)
    undistorted_a05 = cv2.undistort(img, K, D, None, new_K_05)

    # Optimal alpha=1 (keep all pixels)
    new_K_1, roi_1 = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=1)
    undistorted_a1 = cv2.undistort(img, K, D, None, new_K_1)

    # Using remap (faster for video — compute maps once, apply many times)
    map1, map2 = cv2.initUndistortRectifyMap(
        K, D, None, new_K_05, (w, h), cv2.CV_32FC1
    )
    undistorted_remap = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

    # --- Display ---
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    images_list = [
        (img, 'Original (distorted)'),
        (undistorted_simple, 'Simple undistort'),
        (undistorted_a0, 'alpha=0 (no black borders)'),
        (undistorted_a05, 'alpha=0.5 (balanced)'),
        (undistorted_a1, 'alpha=1 (all source pixels)'),
        (undistorted_remap, 'Remap (same as alpha=0.5)'),
    ]

    for ax, (im, title) in zip(axes.flat, images_list):
        ax.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
        ax.set_title(title)
        ax.axis('off')

    plt.suptitle('Distortion Correction Comparison', fontsize=14)
    plt.tight_layout()
    plt.savefig('undistortion_comparison.png', dpi=150)
    plt.show()

    print("\nPerformance note:")
    print("  cv2.undistort():  recomputes mapping every call")
    print("  cv2.remap():      uses precomputed maps (10x faster for video)")
    print("  For real-time: compute map1/map2 once, then remap every frame")
```

### 9.4 Bird's Eye View Transform

```python
"""
bev_transform.py
Compute and apply Bird's Eye View (BEV) perspective transform.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt


def compute_bev_transform(img_shape=(480, 640)):
    """Define source/destination points for BEV homography."""
    h, w = img_shape

    # Source: trapezoid on road (tune for your camera mount!)
    src = np.float32([
        [w * 0.40, h * 0.65],   # top-left
        [w * 0.60, h * 0.65],   # top-right
        [w * 0.85, h * 0.95],   # bottom-right
        [w * 0.15, h * 0.95],   # bottom-left
    ])

    # Destination: rectangle in BEV
    margin = w * 0.25
    dst = np.float32([
        [margin, 0],
        [w - margin, 0],
        [w - margin, h],
        [margin, h],
    ])

    H_bev = cv2.getPerspectiveTransform(src, dst)
    H_bev_inv = cv2.getPerspectiveTransform(dst, src)

    return H_bev, H_bev_inv, src, dst


def demo_bev():
    """Create synthetic road image and demonstrate BEV transform."""
    h, w = 480, 640

    # Create synthetic road
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = [80, 80, 80]
    img[:int(h * 0.55)] = [200, 180, 150]  # sky

    vanish_x, vanish_y = w // 2, int(h * 0.55)

    # Lane lines
    cv2.line(img, (int(w * 0.15), h), (vanish_x - 10, vanish_y),
             (0, 255, 255), 3)
    cv2.line(img, (int(w * 0.85), h), (vanish_x + 10, vanish_y),
             (0, 255, 255), 3)

    # Dashed center line
    for i in range(8):
        frac1 = 0.55 + i * 0.055
        frac2 = frac1 + 0.025
        if frac2 > 1.0:
            break
        y1, y2 = int(h * frac1), int(h * frac2)
        x1 = int(vanish_x + (w * 0.5 - vanish_x) * (frac1 - 0.55) / 0.45)
        x2 = int(vanish_x + (w * 0.5 - vanish_x) * (frac2 - 0.55) / 0.45)
        cv2.line(img, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # BEV transform
    H_bev, H_bev_inv, src, dst = compute_bev_transform((h, w))
    bev = cv2.warpPerspective(img, H_bev, (w, h))

    # Annotate
    img_annotated = img.copy()
    cv2.polylines(img_annotated, [src.astype(np.int32)], True, (0, 0, 255), 2)

    # Display
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].imshow(cv2.cvtColor(img_annotated, cv2.COLOR_BGR2RGB))
    axes[0].set_title('Camera View (source trapezoid in red)')
    axes[1].imshow(cv2.cvtColor(bev, cv2.COLOR_BGR2RGB))
    axes[1].set_title("Bird's Eye View (lanes are now parallel)")
    for ax in axes:
        ax.axis('off')
    plt.tight_layout()
    plt.savefig('bev_transform.png', dpi=150)
    plt.show()

    print("BEV Homography Matrix H:")
    np.set_printoptions(precision=4, suppress=True)
    print(H_bev)


demo_bev()
```

### 9.5 Save Calibration as ROS2 YAML

```python
"""
save_calibration_ros2.py
Save camera calibration in ROS2 camera_info YAML format.
"""

import numpy as np
import yaml
import os


def save_calibration_yaml(filename, K, D, img_shape,
                          camera_name='autonomous_car_camera',
                          distortion_model='plumb_bob'):
    """
    Save calibration in ROS2 camera_calibration format.

    This YAML file can be loaded by:
    - image_proc nodes for rectification
    - camera_info_manager for publishing CameraInfo
    - Any ROS2 node subscribing to sensor_msgs/CameraInfo
    """
    w, h = img_shape

    R = np.eye(3)
    P = np.zeros((3, 4))
    P[:3, :3] = K

    calibration = {
        'image_width': int(w),
        'image_height': int(h),
        'camera_name': camera_name,
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': [float(x) for x in K.flatten()]
        },
        'distortion_model': distortion_model,
        'distortion_coefficients': {
            'rows': 1, 'cols': 5,
            'data': [float(x) for x in D.flatten()]
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3,
            'data': [float(x) for x in R.flatten()]
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': [float(x) for x in P.flatten()]
        }
    }

    with open(filename, 'w') as f:
        yaml.dump(calibration, f, default_flow_style=False)

    print(f"Calibration saved to {filename}")
    print(f"\nTo use in ROS2 launch file:")
    print(f"  camera_info_url: 'file://{os.path.abspath(filename)}'")


# Example
if __name__ == "__main__":
    K = np.array([[615.0, 0, 320.0],
                  [0, 615.0, 240.0],
                  [0, 0, 1.0]])
    D = np.array([[-0.05, 0.12, 0.001, -0.002, -0.08]])
    save_calibration_yaml('camera_calibration.yaml', K, D, (640, 480))
```

The resulting YAML follows standard ROS2 format:

```yaml
image_width: 640
image_height: 480
camera_name: autonomous_car_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.05, 0.12, 0.001, -0.002, -0.08]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [615.0, 0.0, 320.0, 0.0, 0.0, 615.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

---

## Review

Today we built the mathematical foundation for using cameras as precision measurement devices.

| Topic | Key equation / concept |
|-------|----------------------|
| Pinhole projection | \(s\,\tilde{\mathbf{p}} = K[R \mid t]\tilde{\mathbf{P}}_w\) |
| Intrinsic \(K\) | \(f_x, f_y, c_x, c_y\) — focal length in pixels and principal point |
| Extrinsic \([R \mid t]\) | Camera pose: rotation + translation (6 DOF per view) |
| Camera position in world | \(\mathbf{C}_w = -R^T t\) (not \(t\) itself!) |
| Radial distortion | \(k_1, k_2, k_3\) — barrel (\(k_1 < 0\)) or pincushion (\(k_1 > 0\)) |
| Tangential distortion | \(p_1, p_2\) — lens decentering |
| Zhang's method | Planar homography constraints \(\to\) solve for \(K\), then bundle adjust |
| Reprojection error | < 0.5 px is good, < 0.3 px is excellent |
| Homography | 8-DOF plane-to-plane mapping, 4 correspondences needed |
| BEV transform | Removes perspective for lane analysis; only valid on road plane |
| Backprojection | \(\mathbf{P}_c = d \cdot K^{-1}\tilde{\mathbf{p}}\) — from pixel + depth to 3D |

### Connection to Previous Days

- **Day 10** (Depth Camera): we now understand the intrinsic matrix needed to convert depth pixels to 3D points via backprojection, and why RGB-depth alignment requires knowing both cameras' extrinsics.
- **Day 9** (PID Control): the BEV transform enables us to measure cross-track error (CTE) in meters, which feeds directly into the steering PID controller.
- **Day 7** (IMU): the rotation representations for extrinsic parameters (Euler, Rodrigues, quaternion) connect directly to IMU orientation output.

### What Comes Next

In **Day 12**, we tackle **SLAM** — Simultaneous Localization and Mapping. We will use the calibrated camera, depth data, and odometry to build a map of the environment while simultaneously tracking the car's position within it. The calibration parameters \(K\) and \(D\) computed today are a critical prerequisite: every visual odometry and feature projection calculation in SLAM starts with the intrinsic matrix.
