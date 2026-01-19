---
title: "Camera Calibration with OpenCV"
date: 2024-07-11
description: "Complete guide to camera calibration using checkerboard patterns"
categories: ["2D Vision"]
tags: ["Camera Calibration", "OpenCV", "Computer Vision"]
draft: false
---

{{< katex >}}

## Overview

Camera calibration determines the intrinsic parameters and lens distortion coefficients necessary for accurate 3D reconstruction and computer vision applications.

## What We're Finding

### Intrinsic Matrix

$$
K = \begin{pmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}
$$

Where:
- \\(f_x, f_y\\): Focal lengths (pixels)
- \\(c_x, c_y\\): Principal point

### Distortion Coefficients

$$
(k_1, k_2, p_1, p_2, k_3)
$$

- \\(k_1, k_2, k_3\\): Radial distortion
- \\(p_1, p_2\\): Tangential distortion

## Calibration Setup

### Checkerboard Pattern

- Pattern: 7×10 internal corners
- Square size: Measure actual size (e.g., 25mm)
- Print on flat surface

### Requirements

- 10-20 images
- Various angles and positions
- Cover entire image area

## Python Implementation

```python
import numpy as np
import cv2
import glob

# Checkerboard dimensions
CHECKERBOARD = (7, 10)
square_size = 0.025  # 25mm in meters

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS +
           cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3),
               np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0],
                       0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store points
objpoints = []  # 3D points in world
imgpoints = []  # 2D points in image

# Load calibration images
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)

        # Refine corner positions
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display corners
        cv2.drawChessboardCorners(img, CHECKERBOARD,
                                 corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# Print results
print("Camera Matrix:")
print(mtx)
print("\nDistortion Coefficients:")
print(dist)
print(f"\nRMS Error: {ret}")
```

## Understanding the Code

### Termination Criteria

```python
criteria = (cv2.TERM_CRITERIA_EPS +
           cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Type | EPS + MAX_ITER | Stop on precision or iterations |
| Max iterations | 30 | Maximum refinement steps |
| Epsilon | 0.001 | Precision threshold |

### Object Points Setup

```python
objp[:, :2] = np.mgrid[0:7, 0:10].T.reshape(-1, 2)
```

Creates a grid of 3D points:
```
(0,0,0), (1,0,0), (2,0,0), ...
(0,1,0), (1,1,0), (2,1,0), ...
...
```

### Corner Refinement

```python
corners2 = cv2.cornerSubPix(gray, corners,
                           (11, 11), (-1, -1), criteria)
```

- Window size: 11×11 pixels
- Search for sub-pixel accuracy
- Improves calibration precision

## Calibration Output

### Example Results

```
Camera Matrix:
[[844.123   0.    319.56]
 [  0.    843.89  239.78]
 [  0.      0.      1.  ]]

Distortion Coefficients:
[[-0.2145  0.1234  0.0012 -0.0008  0.0456]]

RMS Error: 0.234
```

### Interpreting Results

| Metric | Good Value |
|--------|------------|
| RMS Error | < 0.5 pixels |
| fx ≈ fy | Should be similar |
| cx, cy | Near image center |

## Using Calibration

### Undistort Images

```python
# Load calibration
mtx = np.load('camera_matrix.npy')
dist = np.load('distortion.npy')

# Undistort
img = cv2.imread('image.jpg')
h, w = img.shape[:2]

# Get optimal camera matrix
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
    mtx, dist, (w, h), 1, (w, h))

# Undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# Crop
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
```

### Save Calibration

```python
np.save('camera_matrix.npy', mtx)
np.save('distortion.npy', dist)
```

## Tips for Good Calibration

1. **Even lighting** - Avoid shadows
2. **Sharp images** - No motion blur
3. **Full coverage** - Fill entire frame
4. **Multiple angles** - Tilt pattern 30-45°
5. **Steady pattern** - Use rigid backing

## ROS Integration

Using `camera_calibration` package:

```bash
rosrun camera_calibration cameracalibrator.py \
    --size 7x10 \
    --square 0.025 \
    image:=/camera/image_raw \
    camera:=/camera
```
