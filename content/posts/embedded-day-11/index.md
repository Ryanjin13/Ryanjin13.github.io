---
title: "Day 11 — Camera Geometry and Calibration"
date: 2026-03-06
description: "Pinhole camera model, intrinsic and extrinsic parameters, lens distortion, Zhang's calibration method, homography, and Bird's Eye View transform"
categories: ["Autonomous Driving"]
tags: ["Camera Calibration", "Pinhole Model", "Homography", "OpenCV", "BEV"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 11
draft: false
---

{{< katex >}}

## What You'll Learn

Yesterday we measured distances with LiDAR and depth cameras. But raw pixel coordinates from a camera are meaningless unless we know the **mathematical relationship** between a 3D point in the real world and its 2D pixel location. That relationship is **camera calibration**.

By the end of this post you will:

1. Derive the full pinhole camera projection from 3D world to 2D pixel coordinates.
2. Understand the **intrinsic matrix K** and what each element physically means.
3. Understand the **extrinsic parameters** (rotation R + translation t) that define where the camera sits in the world.
4. Model **lens distortion** (radial and tangential) and correct it.
5. Perform calibration using **Zhang's method** with a checkerboard.
6. Compute a **homography** and use it for Bird's Eye View (BEV) transforms.
7. Align depth camera data with RGB using the calibration results.

---

## 1. The Pinhole Camera Model

### From 3D World to 2D Image

Imagine poking a tiny hole in a box and letting light through. Each 3D point in the world projects onto the back wall of the box along a straight line through the hole. This is the **pinhole model** — the foundation of all camera geometry.

```
        3D World Point P = (X, Y, Z)
                    │
                    │  light ray
                    ▼
    ────────────────●──────────────── Pinhole (Optical Center)
                    │
                    │  focal length f
                    ▼
              ┌─────●─────┐
              │  p = (u,v) │  Image Plane
              └───────────┘
```

By similar triangles, a point \(P = (X, Y, Z)\) in camera coordinates projects to:

$$x = f \frac{X}{Z}, \quad y = f \frac{Y}{Z}$$

This is **perspective projection** — objects farther away appear smaller, proportional to \(1/Z\).

### Homogeneous Coordinates

To express projection as a **matrix multiplication** (which is computationally efficient), we use homogeneous coordinates. A 2D point \((u, v)\) becomes \((u, v, 1)\), and a 3D point \((X, Y, Z)\) becomes \((X, Y, Z, 1)\).

The projection equation in homogeneous coordinates:

$$s \begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{K} \begin{pmatrix} \mathbf{R} & \mathbf{t} \end{pmatrix} \begin{pmatrix} X \\ Y \\ Z \\ 1 \end{pmatrix}$$

Where:
- \(s\) = scale factor (the depth \(Z\))
- \(\mathbf{K}\) = intrinsic matrix (camera internal properties)
- \(\mathbf{R}\) = 3×3 rotation matrix (camera orientation)
- \(\mathbf{t}\) = 3×1 translation vector (camera position)

Let's break this apart.

---

## 2. Intrinsic Parameters — The Camera Matrix K

The intrinsic matrix captures everything about the camera's internal optics:

$$\mathbf{K} = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$$

### What Each Element Means

| Parameter | Physical Meaning |
|-----------|-----------------|
| \(f_x\) | Focal length in pixels (horizontal) |
| \(f_y\) | Focal length in pixels (vertical) |
| \(c_x\) | Principal point x (where optical axis hits the sensor) |
| \(c_y\) | Principal point y |

**Focal length in pixels** relates to the physical focal length \(f\) (in mm) and pixel size \(s\) (in mm/pixel):

$$f_x = \frac{f}{s_x}, \quad f_y = \frac{f}{s_y}$$

For most modern cameras, pixels are square so \(f_x \approx f_y\). The principal point \((c_x, c_y)\) is ideally at the image center but in practice is slightly offset due to manufacturing.

### Example: Typical Raspberry Pi Camera v2

```
Resolution: 3280 × 2464
Sensor size: 3.68 × 2.76 mm
Focal length: 3.04 mm

Pixel size: 3.68/3280 = 0.00112 mm/px

f_x = 3.04 / 0.00112 ≈ 2714 pixels
f_y ≈ 2714 pixels
c_x ≈ 3280/2 = 1640
c_y ≈ 2464/2 = 1232

K = [2714    0   1640]
    [   0  2714  1232]
    [   0     0     1]
```

### The Full Projection (Intrinsic Only)

For a point already in **camera coordinates** \((X_c, Y_c, Z_c)\):

$$\begin{pmatrix} u \\ v \end{pmatrix} = \begin{pmatrix} f_x \frac{X_c}{Z_c} + c_x \\ f_y \frac{Y_c}{Z_c} + c_y \end{pmatrix}$$

This maps a 3D point in front of the camera to a pixel location on the image.

---

## 3. Extrinsic Parameters — Where Is the Camera?

The extrinsic parameters transform points from **world coordinates** to **camera coordinates**:

$$\begin{pmatrix} X_c \\ Y_c \\ Z_c \end{pmatrix} = \mathbf{R} \begin{pmatrix} X_w \\ Y_w \\ Z_w \end{pmatrix} + \mathbf{t}$$

```
  World Frame (W)              Camera Frame (C)
     Z_w ▲                        Z_c ──► (into scene)
         │                        │
         │                        │ Y_c
         │                        ▼ (down in image)
    ─────┼────► X_w          X_c ◄─── (right in image)
         │
         │
    ─────┘
    Y_w
```

### Rotation Matrix R (3×3)

\(\mathbf{R}\) is a 3×3 orthogonal matrix (\(\mathbf{R}^T \mathbf{R} = \mathbf{I}\), \(\det(\mathbf{R}) = 1\)) that encodes the camera's orientation relative to the world. It has 3 degrees of freedom (roll, pitch, yaw), often represented as:

- **Euler angles**: intuitive but suffer from gimbal lock (Day 7)
- **Rodrigues vector**: 3-element axis-angle representation (OpenCV uses this)
- **Quaternion**: 4 elements, no gimbal lock (ROS2 standard)

### Translation Vector t (3×1)

\(\mathbf{t}\) is the position of the world origin expressed in camera coordinates. **Not** the position of the camera in the world! The camera position in world coordinates is:

$$\mathbf{C}_{world} = -\mathbf{R}^T \mathbf{t}$$

### Combined: The 3×4 Projection Matrix P

$$\mathbf{P} = \mathbf{K} [\mathbf{R} \mid \mathbf{t}]$$

This single 3×4 matrix takes a 3D world point (in homogeneous coordinates) directly to a 2D pixel:

$$s \begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{P} \begin{pmatrix} X_w \\ Y_w \\ Z_w \\ 1 \end{pmatrix}$$

---

## 4. Lens Distortion

Real lenses are not perfect pinholes. They introduce geometric distortions that must be corrected before measurements are meaningful.

### Radial Distortion

Straight lines in the world appear curved in the image, especially near the edges:

```
  Barrel Distortion          Pincushion Distortion
  (k1 < 0, wide-angle)      (k1 > 0, telephoto)

  ┌──╮    ╭──┐              ┌──╲    ╱──┐
  │   ╲  ╱   │              │   ╱  ╲   │
  │    ╲╱    │              │  ╱    ╲  │
  │    ╱╲    │              │  ╲    ╱  │
  │   ╱  ╲   │              │   ╲  ╱   │
  └──╯    ╰──┘              └──╱    ╲──┘
```

The radial distortion model uses normalized coordinates \((x_n, y_n) = (X_c/Z_c, \; Y_c/Z_c)\) and radius \(r^2 = x_n^2 + y_n^2\):

$$x_{distorted} = x_n (1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$
$$y_{distorted} = y_n (1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$

### Tangential Distortion

Caused by the lens not being perfectly parallel to the image sensor:

$$x_{distorted} = x_n + [2 p_1 x_n y_n + p_2 (r^2 + 2 x_n^2)]$$
$$y_{distorted} = y_n + [p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n]$$

### The 5-Parameter Distortion Vector

OpenCV uses the distortion coefficient vector:

$$\mathbf{d} = (k_1, k_2, p_1, p_2, k_3)$$

For the RPi Camera v2, typical values might be:

```
k1 ≈ 0.1,  k2 ≈ -0.25,  k3 ≈ 0.1   (radial)
p1 ≈ 0.001, p2 ≈ -0.001              (tangential)
```

### Why This Matters for Our Car

If we don't correct distortion:
- Lane lines appear curved when they're actually straight
- Distance measurements from pixel positions are wrong
- BEV (Bird's Eye View) transforms produce warped maps

---

## 5. Zhang's Calibration Method

How do we actually find \(\mathbf{K}\) and \(\mathbf{d}\)? We use a **known pattern** — typically a checkerboard — photographed from multiple angles.

### Why a Checkerboard?

The corners of a checkerboard are precisely located (known 3D positions on a flat plane) and can be detected automatically with sub-pixel accuracy.

### The Process

```
Step 1: Print checkerboard, place it on a flat surface

Step 2: Capture N images (N ≥ 10) from different angles

  ┌─────┐    ┌─────┐    ┌─────┐    ┌─────┐
  │ ▦▦▦ │    │  ▦▦ │    │▦▦   │    │ ▦▦▦ │
  │ ▦▦▦ │    │ ▦▦  │    │ ▦▦  │    │▦▦▦  │
  │ ▦▦▦ │    │▦▦   │    │  ▦▦ │    │ ▦▦▦ │
  └─────┘    └─────┘    └─────┘    └─────┘
  Frontal     Tilted     Rotated   Close-up

Step 3: Detect corners in each image

Step 4: Zhang's algorithm solves for K and d by minimizing
        the reprojection error across all images
```

### Reprojection Error

The key quality metric. For each detected corner, compute:

1. Project the known 3D point using estimated K, R, t, d → predicted pixel \((\hat{u}, \hat{v})\)
2. Compare with detected pixel \((u, v)\)

$$\text{RMS Reprojection Error} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left[(u_i - \hat{u}_i)^2 + (v_i - \hat{v}_i)^2\right]}$$

A good calibration achieves **< 0.5 pixels** reprojection error.

### Zhang's Algorithm — Mathematical Core

For a planar calibration target (Z = 0), the projection simplifies to a **homography**:

$$s \begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{K} \begin{pmatrix} r_1 & r_2 & t \end{pmatrix} \begin{pmatrix} X \\ Y \\ 1 \end{pmatrix} = \mathbf{H} \begin{pmatrix} X \\ Y \\ 1 \end{pmatrix}$$

Where \(\mathbf{H} = \mathbf{K}[r_1 \; r_2 \; t]\) is a 3×3 homography matrix. The key insight of Zhang's method:

1. Estimate H for each image (requires ≥ 4 point correspondences per image)
2. From H, extract constraints on K using the orthogonality of rotation columns: \(r_1^T r_2 = 0\) and \(\|r_1\| = \|r_2\|\)
3. Each image provides 2 constraints; K has 5 unknowns → need ≥ 3 images
4. Solve for K, then extract R and t per image
5. Refine everything (K, d, all R's and t's) jointly via Levenberg-Marquardt optimization

---

## 6. Homography and Bird's Eye View

### What Is a Homography?

A homography is a 3×3 matrix H that maps points on one plane to points on another plane:

$$s \begin{pmatrix} u' \\ v' \\ 1 \end{pmatrix} = \mathbf{H} \begin{pmatrix} u \\ v \\ 1 \end{pmatrix}$$

It has 8 degrees of freedom (9 elements minus 1 for scale), so we need at least 4 point correspondences to compute it.

### Bird's Eye View (BEV) Transform

For lane detection (Day 17), we need a **top-down view** of the road. This is an **Inverse Perspective Mapping (IPM)**:

```
  Camera View (Perspective)          Bird's Eye View (BEV)
  ┌───────────────────────┐          ┌───────────────────────┐
  │         sky            │          │                       │
  │    ╱─────────╲        │          │   │           │       │
  │   ╱           ╲       │          │   │           │       │
  │  ╱    road     ╲      │    H     │   │   road    │       │
  │ ╱               ╲     │  ────►   │   │           │       │
  │╱                 ╲    │          │   │           │       │
  │───────────────────│   │          │   │           │       │
  └───────────────────────┘          └───────────────────────┘
  Lanes converge at horizon          Lanes are parallel
```

The BEV transform is a homography from the road plane in the camera image to a top-down coordinate system. We define 4 source points (a trapezoid on the road) and 4 destination points (a rectangle):

```python
# Source: trapezoid in camera image
src = np.float32([
    [200, 720],   # bottom-left
    [580, 460],   # top-left
    [700, 460],   # top-right
    [1080, 720],  # bottom-right
])

# Destination: rectangle in BEV
dst = np.float32([
    [300, 720],
    [300, 0],
    [980, 0],
    [980, 720],
])

H = cv2.getPerspectiveTransform(src, dst)
bev = cv2.warpPerspective(image, H, (1280, 720))
```

---

## 7. Depth Camera Alignment

When using an RGB + Depth camera (like the Intel RealSense from Day 10), the RGB sensor and depth sensor are at slightly different positions. We need to **align** them.

### The Alignment Problem

```
  RGB Camera            Depth Camera
  ┌──────┐              ┌──────┐
  │ RGB  │◄── b ────►  │Depth │
  │sensor│  baseline    │sensor│
  └──────┘              └──────┘
       ↓                     ↓
  ┌─────────┐          ┌─────────┐
  │ Color   │          │ Depth   │
  │ image   │          │ map     │
  └─────────┘          └─────────┘

  Pixel (u,v) in RGB ≠ Pixel (u,v) in Depth
  → Need alignment transform
```

Each sensor has its own intrinsic matrix (\(\mathbf{K}_{rgb}\), \(\mathbf{K}_{depth}\)) and they are related by an extrinsic transform (\(\mathbf{R}_{rel}\), \(\mathbf{t}_{rel}\)):

$$P_{rgb} = \mathbf{R}_{rel} P_{depth} + \mathbf{t}_{rel}$$

### Depth Scale Factor

Depth cameras return integer values (e.g., uint16). The **depth scale** converts to meters:

$$Z_{meters} = \text{raw\_value} \times \text{depth\_scale}$$

For Intel RealSense: `depth_scale ≈ 0.001` (raw value 1000 = 1.0 m).

### 3D Reconstruction from Depth

Given a depth value \(Z\) at pixel \((u, v)\) and the intrinsic matrix \(\mathbf{K}\):

$$X = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y = \frac{(v - c_y) \cdot Z}{f_y}$$

This recovers the full 3D point cloud from a depth image — essential for obstacle detection and SLAM (Day 12).

---

## 8. Hands-On Lab

### Lab 1: Camera Calibration with OpenCV

```python
#!/usr/bin/env python3
"""Camera calibration using a checkerboard pattern."""

import numpy as np
import cv2
import glob
import yaml

# Checkerboard dimensions (inner corners)
BOARD_SIZE = (9, 6)  # 9 columns, 6 rows of inner corners
SQUARE_SIZE = 25.0   # mm

# Prepare 3D object points (Z = 0 for all points on the board)
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # scale to real-world mm

# Storage for calibration data
obj_points = []  # 3D points in world
img_points = []  # 2D points in image

# Load calibration images
images = glob.glob('calibration_images/*.jpg')
print(f"Found {len(images)} calibration images")

gray_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_shape = gray.shape[::-1]  # (width, height)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

    if ret:
        # Refine to sub-pixel accuracy
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        obj_points.append(objp)
        img_points.append(corners_refined)

        # Draw and display corners
        cv2.drawChessboardCorners(img, BOARD_SIZE, corners_refined, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

if len(obj_points) < 3:
    print("Need at least 3 valid images for calibration!")
    exit()

print(f"\nCalibrating with {len(obj_points)} images...")

# Calibrate!
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray_shape, None, None
)

print(f"\nRMS Reprojection Error: {ret:.4f} pixels")
print(f"\nIntrinsic Matrix K:\n{K}")
print(f"\nDistortion Coefficients: {dist.ravel()}")

# Compute per-image reprojection error
print("\nPer-image reprojection error:")
for i in range(len(obj_points)):
    projected, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
    error = cv2.norm(img_points[i], projected, cv2.NORM_L2) / len(projected)
    print(f"  Image {i}: {error:.4f} px")

# Save calibration result
calib_data = {
    'camera_matrix': K.tolist(),
    'dist_coeffs': dist.tolist(),
    'rms_error': float(ret),
    'image_size': list(gray_shape),
}
with open('calibration.yaml', 'w') as f:
    yaml.dump(calib_data, f)
print("\nCalibration saved to calibration.yaml")
```

### Lab 2: Undistortion — Before vs After

```python
#!/usr/bin/env python3
"""Visualize distortion correction."""

import numpy as np
import cv2
import yaml

# Load calibration
with open('calibration.yaml', 'r') as f:
    calib = yaml.safe_load(f)

K = np.array(calib['camera_matrix'])
dist = np.array(calib['dist_coeffs'])
w, h = calib['image_size']

# Compute optimal new camera matrix
# alpha=0: crop all black pixels, alpha=1: keep all pixels
new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=1)

# Method 1: cv2.undistort (simple)
img = cv2.imread('test_image.jpg')
undistorted = cv2.undistort(img, K, dist, None, new_K)

# Method 2: Remap (faster for video — compute maps once, apply many times)
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, new_K, (w, h), cv2.CV_16SC2)
undistorted_remap = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

# Side by side comparison
comparison = np.hstack([img, undistorted])
cv2.putText(comparison, 'Original', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
cv2.putText(comparison, 'Undistorted', (w + 10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
cv2.imshow('Distortion Correction', comparison)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

### Lab 3: Bird's Eye View Transform

```python
#!/usr/bin/env python3
"""Perspective transform to Bird's Eye View."""

import numpy as np
import cv2

img = cv2.imread('road_image.jpg')
h, w = img.shape[:2]

# Define source trapezoid (road region in perspective view)
# These points depend on your camera mounting and road geometry
src = np.float32([
    [w * 0.15, h],         # bottom-left
    [w * 0.45, h * 0.63],  # top-left
    [w * 0.55, h * 0.63],  # top-right
    [w * 0.85, h],         # bottom-right
])

# Define destination rectangle (BEV output)
dst = np.float32([
    [w * 0.2, h],
    [w * 0.2, 0],
    [w * 0.8, 0],
    [w * 0.8, h],
])

# Compute homography (forward and inverse)
H = cv2.getPerspectiveTransform(src, dst)
H_inv = cv2.getPerspectiveTransform(dst, src)

# Apply BEV transform
bev = cv2.warpPerspective(img, H, (w, h))

# Draw source trapezoid on original
img_annotated = img.copy()
pts = src.astype(np.int32).reshape((-1, 1, 2))
cv2.polylines(img_annotated, [pts], True, (0, 255, 0), 2)

# Show results
combined = np.hstack([img_annotated, bev])
cv2.putText(combined, 'Camera View', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
cv2.putText(combined, "Bird's Eye View", (w + 10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
cv2.imshow('Perspective Transform', combined)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save homography for lane detection (Day 17)
np.savez('bev_transform.npz', H=H, H_inv=H_inv, src=src, dst=dst)
print("BEV transform saved to bev_transform.npz")
```

### Lab 4: Save Calibration as ROS2 camera_info YAML

```python
#!/usr/bin/env python3
"""Convert OpenCV calibration to ROS2 camera_info format."""

import numpy as np
import yaml

# Load OpenCV calibration
with open('calibration.yaml', 'r') as f:
    calib = yaml.safe_load(f)

K = np.array(calib['camera_matrix'])
dist = np.array(calib['dist_coeffs']).ravel()
w, h = calib['image_size']

# ROS2 camera_info format
# D: distortion coefficients [k1, k2, p1, p2, k3]
# K: 3x3 intrinsic matrix (row-major)
# R: 3x3 rectification matrix (identity for monocular)
# P: 3x4 projection matrix

R = np.eye(3)
P = np.zeros((3, 4))
P[:3, :3] = K  # For monocular: P = [K | 0]

camera_info = {
    'image_width': w,
    'image_height': h,
    'camera_name': 'rpi_camera_v2',
    'camera_model': 'plumb_bob',
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': {
        'rows': 1,
        'cols': 5,
        'data': dist.tolist(),
    },
    'camera_matrix': {
        'rows': 3,
        'cols': 3,
        'data': K.ravel().tolist(),
    },
    'rectification_matrix': {
        'rows': 3,
        'cols': 3,
        'data': R.ravel().tolist(),
    },
    'projection_matrix': {
        'rows': 3,
        'cols': 4,
        'data': P.ravel().tolist(),
    },
}

with open('camera_info.yaml', 'w') as f:
    yaml.dump(camera_info, f, default_flow_style=False)

print("ROS2 camera_info saved to camera_info.yaml")
print(f"\nTo use in ROS2 launch file:")
print(f"  camera_info_url: 'file:///path/to/camera_info.yaml'")
```

---

## 9. Review

### Key Takeaways

1. **Pinhole model**: \(s[u,v,1]^T = \mathbf{K}[\mathbf{R}|\mathbf{t}][X,Y,Z,1]^T\)
2. **Intrinsic matrix K**: focal lengths (\(f_x, f_y\)) and principal point (\(c_x, c_y\)) — fixed for a given camera
3. **Extrinsic parameters**: R (rotation) + t (translation) — changes when the camera moves
4. **Distortion**: radial (\(k_1, k_2, k_3\)) warps straight lines, tangential (\(p_1, p_2\)) from lens misalignment
5. **Zhang's method**: multiple checkerboard images → solve K and d via homography constraints
6. **Reprojection error** < 0.5 px = good calibration
7. **Homography**: planar mapping, 4 point pairs → H matrix → BEV transform
8. **Depth alignment**: different sensors need extrinsic calibration + depth scale factor

### Connection Map

```
Day 7 (IMU)  ──► Extrinsic R uses rotation representations
Day 10 (Depth) ──► Depth camera alignment, 3D reconstruction
Day 12 (SLAM) ──► Calibration feeds into Visual Odometry
Day 17 (Lane) ──► BEV transform for lane detection pipeline
```

### Looking Ahead

Tomorrow (Day 12), we tackle **SLAM** — Simultaneous Localization and Mapping. The calibration data we computed today (K, d, and the extrinsic transforms) feeds directly into visual odometry and RTAB-Map, letting our car build a map of its environment while figuring out where it is in that map.
