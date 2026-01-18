---
title: "Camera Calibration"
date: 2024-07-10
description: "Extracting intrinsic and extrinsic camera parameters using OpenCV"
categories: ["2D Vision"]
tags: ["Camera Calibration", "OpenCV", "Computer Vision"]
draft: false
---

{{< katex >}}

## Overview

Camera calibration determines the intrinsic and extrinsic parameters needed to accurately map 3D world coordinates to 2D image coordinates.

## Camera Model

### Projection Equation

$$
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K [R | t] \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$

Where:
- \((u, v)\): Image coordinates (pixels)
- \((X, Y, Z)\): World coordinates
- \(K\): Intrinsic matrix
- \([R|t]\): Extrinsic matrix (rotation + translation)

### Intrinsic Matrix

$$
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

| Parameter | Description |
|-----------|-------------|
| \(f_x, f_y\) | Focal length (pixels) |
| \(c_x, c_y\) | Principal point (image center) |

### Distortion Coefficients

Radial: \(k_1, k_2, k_3\)
Tangential: \(p_1, p_2\)

$$
dist = [k_1, k_2, p_1, p_2, k_3]
$$

## Implementation

### Setup

```python
import numpy as np
import cv2
import glob

# Checkerboard dimensions (inner corners)
CHECKERBOARD = (7, 10)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
```

### Prepare Object Points

```python
# 3D points in real world space (z=0 for flat checkerboard)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store points from all images
objpoints = []  # 3D points
imgpoints = []  # 2D points
```

### Detect Corners

```python
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)

        # Refine corner positions
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Visualize
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()
```

### Calibrate

```python
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Camera Matrix:\n", mtx)
print("Distortion Coefficients:\n", dist)
```

## Output Parameters

### Camera Matrix (mtx)

```
[[fx  0  cx]
 [ 0 fy  cy]
 [ 0  0   1]]
```

### Distortion Coefficients (dist)

```
[k1, k2, p1, p2, k3]
```

### Rotation & Translation Vectors

- **rvecs**: Object orientation relative to camera (per image)
- **tvecs**: Object position relative to camera (per image)

## Undistortion

```python
# Get optimal new camera matrix
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# Crop to valid region
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
```

## Tips

1. **Use 10-20 images** from different angles
2. **Cover entire frame** with checkerboard
3. **Vary orientation** - tilted views improve accuracy
4. **Print checkerboard on flat surface**
5. **Good lighting** - avoid reflections and shadows
