---
title: "Day 17 — OpenCV Fundamentals and Lane Detection Pipeline"
date: 2026-03-22
description: "Color space conversion, thresholding, morphological operations, Canny edge detection, Hough transform, BEV perspective transform, and sliding window lane detection"
categories: ["Autonomous Driving"]
tags: ["OpenCV", "Lane Detection", "Canny Edge", "Hough Transform", "Bird's Eye View"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 17
draft: false
---

{{< katex >}}

## What You'll Learn

Today marks the beginning of the **perception** phase of our autonomous car project. Over the past sixteen days, you have built up hardware, firmware, communication protocols, motor control, and SLAM mapping. Now it is time to give your car **eyes** — and more importantly, teach it to **understand** what it sees.

By the end of this post you will be able to:

1. Convert images between color spaces (BGR, HSV, Grayscale) and understand **when** to use each.
2. Apply thresholding techniques including Otsu's method and adaptive thresholding.
3. Use morphological operations (erosion, dilation, opening, closing) to clean up binary masks.
4. Implement the full **Canny edge detection** pipeline and explain every mathematical step.
5. Detect straight lane boundaries using the **Hough Line Transform**.
6. Create a **Bird's Eye View (BEV)** perspective transform using the calibration from Day 11.
7. Apply the **sliding window** method to detect curved lanes.
8. Fit a second-order polynomial to lane pixels and compute a **cross-track error** for steering.

This is a long, hands-on day. Every section includes the underlying math, intuition, and working Python code.

---

## 1. Color Space Conversion

### 1.1 Why Color Spaces Matter

A digital camera records light as three channels — **Blue, Green, Red** (BGR in OpenCV, note the reversed order from the more familiar RGB). While BGR faithfully represents what the sensor captured, it is *terrible* for isolating colors programmatically. The reason is that **brightness** is entangled with **hue** in the BGR representation. A yellow lane marking in bright sunlight and the same marking in shadow will have wildly different B, G, R values even though a human would call both "yellow."

### 1.2 BGR to Grayscale

Grayscale conversion collapses three channels into one using a weighted sum that models human luminance perception:

$$
Y = 0.299 \, R + 0.587 \, G + 0.114 \, B
$$

Green gets the largest weight because the human eye is most sensitive to green light. OpenCV uses this exact formula internally.

```python
import cv2

bgr_image = cv2.imread("road.jpg")
gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
```

**When to use grayscale:** Edge detection (Canny, Sobel), feature detection (corners, ORB), and any algorithm that operates on intensity gradients. Grayscale is also faster to process — one channel instead of three.

### 1.3 BGR to HSV

HSV stands for **Hue, Saturation, Value**:

| Channel | Meaning | OpenCV Range |
|---------|---------|-------------|
| **H** (Hue) | The "pure color" — position on the color wheel | 0 – 179 |
| **S** (Saturation) | How vivid the color is (0 = gray, 255 = pure color) | 0 – 255 |
| **V** (Value) | Brightness (0 = black, 255 = brightest) | 0 – 255 |

> **Why 0–179 instead of 0–360?** OpenCV stores H in a `uint8` (max 255). To fit the full 360-degree hue wheel into one byte, they halve it: \(H_{\text{OpenCV}} = H_{\text{degrees}} / 2\).

The conversion from BGR to HSV decouples **color identity** (H) from **illumination** (V). This is why HSV is the go-to color space for lane color segmentation: you can threshold on H alone and be robust to shadows and lighting changes.

```python
hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
```

**Common HSV ranges for lane detection:**

| Color | H Low | H High | S Low | S High | V Low | V High |
|-------|-------|--------|-------|--------|-------|--------|
| Yellow | 15 | 35 | 80 | 255 | 80 | 255 |
| White | 0 | 179 | 0 | 40 | 200 | 255 |

White lanes have **low saturation** (near-gray) and **high value** (bright). Yellow lanes have a specific **hue range** with moderate-to-high saturation.

```python
import numpy as np

# Yellow mask
lower_yellow = np.array([15, 80, 80])
upper_yellow = np.array([35, 255, 255])
yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

# White mask
lower_white = np.array([0, 0, 200])
upper_white = np.array([179, 40, 255])
white_mask = cv2.inRange(hsv, lower_white, upper_white)

# Combined
lane_mask = cv2.bitwise_or(yellow_mask, white_mask)
```

### 1.4 HLS and LAB (Brief Mention)

Two other color spaces occasionally appear in lane detection:

- **HLS** (Hue, Lightness, Saturation): The L channel isolates lightness even better than V in HSV. Some pipelines threshold on the S channel in HLS because saturated colors (like yellow paint) have high S regardless of lighting.
- **LAB** (Lightness, a, b): The B channel (yellow-blue axis) is excellent for isolating yellow lanes in a single threshold.

For our project, HSV is sufficient and well understood. Know that alternatives exist if HSV struggles in extreme lighting.

---

## 2. Thresholding

Thresholding converts a grayscale (or single-channel) image into a **binary mask** — every pixel becomes either 0 (black) or 255 (white). This is the foundation of all subsequent processing.

### 2.1 Simple (Global) Thresholding

Given a threshold \(T\):

$$
\text{dst}(x, y) =
\begin{cases}
255 & \text{if } \text{src}(x, y) > T \\
0   & \text{otherwise}
\end{cases}
$$

```python
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
```

The problem: how do you choose \(T\)? A manually chosen value fails when lighting changes.

### 2.2 Otsu's Method — Automatic Threshold Selection

Otsu's method finds the threshold that **minimizes the weighted intra-class variance** of the two classes (foreground and background). Equivalently, it **maximizes the inter-class variance**.

#### The Math

Let the image have \(L\) gray levels (0 to \(L-1\)). Let \(p_i\) be the normalized histogram (probability of gray level \(i\)).

For a threshold \(t\), define:

- **Class 0** (background): pixels with intensity \(\leq t\)
- **Class 1** (foreground): pixels with intensity \(> t\)

Class weights:

$$
w_0(t) = \sum_{i=0}^{t} p_i, \qquad w_1(t) = \sum_{i=t+1}^{L-1} p_i = 1 - w_0(t)
$$

Class means:

$$
\mu_0(t) = \frac{1}{w_0(t)} \sum_{i=0}^{t} i \, p_i, \qquad \mu_1(t) = \frac{1}{w_1(t)} \sum_{i=t+1}^{L-1} i \, p_i
$$

Class variances:

$$
\sigma_0^2(t) = \frac{1}{w_0(t)} \sum_{i=0}^{t} (i - \mu_0)^2 \, p_i, \qquad \sigma_1^2(t) = \frac{1}{w_1(t)} \sum_{i=t+1}^{L-1} (i - \mu_1)^2 \, p_i
$$

**Intra-class (within-class) variance:**

$$
\sigma_w^2(t) = w_0(t) \, \sigma_0^2(t) + w_1(t) \, \sigma_1^2(t)
$$

Otsu's optimal threshold:

$$
t^* = \arg\min_{t} \; \sigma_w^2(t)
$$

In practice, it is computationally cheaper to **maximize the between-class variance**:

$$
\sigma_b^2(t) = w_0(t) \, w_1(t) \, \big(\mu_0(t) - \mu_1(t)\big)^2
$$

Since \(\sigma_{\text{total}}^2 = \sigma_w^2 + \sigma_b^2\) and the total variance is constant, maximizing \(\sigma_b^2\) is equivalent to minimizing \(\sigma_w^2\).

**Intuition:** Otsu finds the gray level that best separates the histogram into two "clumps." It works beautifully when the histogram is **bimodal** — which is exactly the case for a road image where dark asphalt and bright lane markings form two peaks.

```python
otsu_thresh, binary_otsu = cv2.threshold(
    gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
)
print(f"Otsu selected threshold: {otsu_thresh}")
```

> **Limitation:** Otsu assumes a bimodal histogram. If lighting is uneven across the image (common on real roads), the histogram may be multimodal and Otsu will fail. That is where adaptive thresholding comes in.

### 2.3 Adaptive Thresholding

Instead of one global threshold, adaptive thresholding computes a **local threshold** for each pixel based on the mean (or Gaussian-weighted mean) of its neighborhood:

$$
T(x, y) = \text{mean}_{\text{local}}(x, y) - C
$$

where \(C\) is a user-defined constant (typically 2–10) that controls sensitivity.

```python
adaptive = cv2.adaptiveThreshold(
    gray,
    255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  # Gaussian-weighted local mean
    cv2.THRESH_BINARY,
    blockSize=15,   # neighborhood size (must be odd)
    C=5             # constant subtracted from mean
)
```

**When to use adaptive:** Uneven illumination, shadows across the lane, images with both dark and bright regions. The tradeoff is more noise and a need to tune `blockSize` and `C`.

---

## 3. Morphological Operations

After thresholding, the binary mask often contains noise — small white specks in the background and small black holes in the foreground. Morphological operations clean this up using a **structuring element** (kernel).

### 3.1 Structuring Element

A structuring element is a small binary matrix that defines the neighborhood shape. Common choices:

```python
# Rectangular
kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
# array([[1, 1, 1, 1, 1],
#        [1, 1, 1, 1, 1],
#        [1, 1, 1, 1, 1],
#        [1, 1, 1, 1, 1],
#        [1, 1, 1, 1, 1]])

# Elliptical
kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# Cross
kernel_cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
```

The kernel size controls how aggressively the operation modifies the image. Larger kernels = stronger effect.

### 3.2 Erosion

Erosion **shrinks** white regions. For each pixel, if **any** pixel in the kernel neighborhood is black, the output pixel becomes black.

$$
(\mathbf{A} \ominus \mathbf{B})(x,y) = \min_{(i,j) \in \mathbf{B}} A(x+i, y+j)
$$

**Effect:** Removes small white noise, separates touching objects, shrinks foreground.

```python
eroded = cv2.erode(binary, kernel_rect, iterations=1)
```

### 3.3 Dilation

Dilation **expands** white regions. For each pixel, if **any** pixel in the kernel neighborhood is white, the output pixel becomes white.

$$
(\mathbf{A} \oplus \mathbf{B})(x,y) = \max_{(i,j) \in \mathbf{B}} A(x+i, y+j)
$$

**Effect:** Fills small black holes, connects nearby white regions, expands foreground.

```python
dilated = cv2.dilate(binary, kernel_rect, iterations=1)
```

### 3.4 Opening (Erosion then Dilation)

Opening removes **small white noise** without significantly shrinking large white regions. Erosion kills the noise; dilation restores the surviving objects to roughly their original size.

$$
\mathbf{A} \circ \mathbf{B} = (\mathbf{A} \ominus \mathbf{B}) \oplus \mathbf{B}
$$

```python
opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_rect)
```

### 3.5 Closing (Dilation then Erosion)

Closing fills **small black holes** inside white regions. Dilation fills the holes; erosion restores the boundary to roughly its original position.

$$
\mathbf{A} \bullet \mathbf{B} = (\mathbf{A} \oplus \mathbf{B}) \ominus \mathbf{B}
$$

```python
closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_rect)
```

### 3.6 Practical Strategy for Lane Detection

A typical morphological cleanup pipeline for lane masks:

```python
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# Step 1: Opening to remove small noise specks
clean = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel, iterations=1)

# Step 2: Closing to fill small gaps in lane lines
clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel, iterations=2)

# Step 3: Optional dilation to thicken thin lane lines
clean = cv2.dilate(clean, kernel, iterations=1)
```

---

## 4. Canny Edge Detection — The Full Pipeline

The Canny edge detector is arguably the most important classical vision algorithm. It produces thin, well-localized edges with minimal false detections. Understanding it deeply is essential because it forms the front end of the Hough transform lane detector.

### 4.1 Overview

The Canny pipeline has four stages:

```
Input Image → [1] Gaussian Smoothing → [2] Gradient Computation
           → [3] Non-Maximum Suppression → [4] Hysteresis Thresholding → Edges
```

### 4.2 Stage 1: Gaussian Smoothing

Real images contain noise. Taking gradients amplifies noise. So we first smooth the image with a Gaussian kernel:

$$
G(x, y) = \frac{1}{2\pi\sigma^2} \exp\left(-\frac{x^2 + y^2}{2\sigma^2}\right)
$$

A common choice is a \(5 \times 5\) kernel with \(\sigma = 1.4\). The smoothed image is:

$$
I_s = G * I
$$

where \(*\) denotes 2D convolution. Larger \(\sigma\) means more smoothing — fewer spurious edges, but also blurrier true edges.

### 4.3 Stage 2: Gradient Magnitude and Direction (Sobel)

We compute the image gradient using **Sobel operators**:

$$
\mathbf{S}_x = \begin{pmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{pmatrix}, \qquad
\mathbf{S}_y = \begin{pmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{pmatrix}
$$

These produce the horizontal and vertical gradient components:

$$
G_x = \mathbf{S}_x * I_s, \qquad G_y = \mathbf{S}_y * I_s
$$

From these we compute:

**Gradient magnitude:**

$$
G = \sqrt{G_x^2 + G_y^2}
$$

(Some implementations approximate with \(G \approx |G_x| + |G_y|\) for speed.)

**Gradient direction:**

$$
\theta = \arctan\left(\frac{G_y}{G_x}\right)
$$

The direction \(\theta\) tells us which way the edge "points" — perpendicular to the edge boundary. It is quantized to four directions: 0, 45, 90, 135 degrees.

### 4.4 Stage 3: Non-Maximum Suppression (NMS)

The gradient magnitude image has **thick** edges — every pixel near an edge has a high gradient. NMS thins these to **one-pixel-wide** edges.

**Algorithm:** For each pixel, look at its two neighbors along the gradient direction \(\theta\). If the pixel's gradient magnitude is not the **local maximum** among these three pixels, suppress it (set to 0).

```
Example: gradient direction = horizontal (0°)
Compare pixel (x, y) with neighbors (x-1, y) and (x+1, y)
Keep (x, y) only if G(x,y) >= G(x-1,y) AND G(x,y) >= G(x+1,y)
```

This is what gives Canny edges their characteristic thin, crisp appearance.

### 4.5 Stage 4: Hysteresis Thresholding

After NMS, we have thin edges, but some are strong (true edges) and some are weak (noise). Hysteresis uses **two thresholds**:

- **High threshold** \(T_H\): Pixels above this are **definitely edges** (strong edges).
- **Low threshold** \(T_L\): Pixels below this are **definitely not edges**.
- **Between** \(T_L\) and \(T_H\): These are edges **only if connected to a strong edge**.

$$
\text{edge}(x, y) =
\begin{cases}
\text{strong} & \text{if } G(x,y) > T_H \\
\text{weak}   & \text{if } T_L \leq G(x,y) \leq T_H \\
\text{suppressed} & \text{if } G(x,y) < T_L
\end{cases}
$$

Weak pixels connected (8-connectivity) to strong pixels are promoted to strong. Everything else is discarded.

**Intuition:** Strong edges "pull in" nearby weak edges, forming continuous contours. Isolated weak pixels (noise) get removed.

**Rule of thumb:** \(T_H : T_L = 2:1\) or \(3:1\). For lane detection, typical values: \(T_L = 50\), \(T_H = 150\).

### 4.6 OpenCV Implementation

```python
# All four stages in one call
edges = cv2.Canny(gray, threshold1=50, threshold2=150, apertureSize=3)
```

The `apertureSize` controls the Sobel kernel size (3, 5, or 7). Larger = smoother gradients but more computation.

**Applying Canny to a masked image:**

```python
# First isolate lane colors, then detect edges within that mask
masked_gray = cv2.bitwise_and(gray, gray, mask=lane_mask)
edges = cv2.Canny(masked_gray, 50, 150)
```

### 4.7 Region of Interest (ROI) Masking

The sky, buildings, and oncoming traffic are irrelevant for lane detection. We define a trapezoidal ROI covering just the road ahead:

```python
def region_of_interest(edges, vertices):
    """Apply a polygon mask to keep only the ROI."""
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(edges, mask)

h, w = edges.shape
# Trapezoid: bottom-left, top-left, top-right, bottom-right
roi_vertices = np.array([[
    (int(0.05 * w), h),         # bottom-left
    (int(0.40 * w), int(0.6 * h)),  # top-left
    (int(0.60 * w), int(0.6 * h)),  # top-right
    (int(0.95 * w), h)          # bottom-right
]], dtype=np.int32)

roi_edges = region_of_interest(edges, roi_vertices)
```

---

## 5. Hough Line Transform

### 5.1 The Problem

Canny gives us edge pixels. But which edge pixels belong to **lines**? We need to go from a collection of points to parametric line equations.

### 5.2 Hough Space Parameterization

A line in Cartesian space can be parameterized as:

$$
y = mx + b
$$

But this fails for vertical lines (\(m = \infty\)). Instead, the Hough transform uses **polar parameters**:

$$
\rho = x \cos\theta + y \sin\theta
$$

where:
- \(\rho\) = perpendicular distance from the origin to the line
- \(\theta\) = angle of the perpendicular with respect to the x-axis

Every line in image space corresponds to a **single point** \((\rho, \theta)\) in Hough space. Conversely, every **point** \((x_0, y_0)\) in image space corresponds to a **sinusoidal curve** in Hough space:

$$
\rho = x_0 \cos\theta + y_0 \sin\theta
$$

### 5.3 The Voting Mechanism

The Hough transform works by **voting**:

1. Create an **accumulator array** \(A[\rho][\theta]\), initialized to zero.
2. For each edge pixel \((x_i, y_i)\):
   - For each discrete \(\theta\) value (e.g., 0 to 180 in 1-degree steps):
     - Compute \(\rho = x_i \cos\theta + y_i \sin\theta\)
     - Increment \(A[\rho][\theta]\)
3. Find **peaks** in the accumulator — these correspond to lines that many edge pixels "voted" for.

**Intuition:** If many edge pixels lie on the same line, their sinusoidal curves in Hough space all pass through the same point. That point accumulates many votes.

The accumulator resolution determines accuracy:
- \(\Delta\rho = 1\) pixel
- \(\Delta\theta = 1°\) = \(\pi/180\) radians

### 5.4 Probabilistic Hough Transform

The standard Hough transform is computationally expensive. OpenCV's `HoughLinesP` uses a **probabilistic** variant that:

1. Randomly samples edge pixels (not all of them).
2. Returns **line segments** \((x_1, y_1, x_2, y_2)\) instead of infinite lines.
3. Uses `minLineLength` and `maxLineGap` to filter results.

```python
lines = cv2.HoughLinesP(
    roi_edges,
    rho=1,              # accumulator resolution: 1 pixel
    theta=np.pi / 180,  # accumulator resolution: 1 degree
    threshold=50,        # minimum votes to consider a line
    minLineLength=40,    # discard lines shorter than this
    maxLineGap=100       # merge lines with gaps up to this
)
```

### 5.5 Separating Left and Right Lanes

Lane lines have distinct slopes:

- **Left lane:** negative slope (line goes up-left to down-right in image coords where y increases downward)
- **Right lane:** positive slope

```python
left_lines = []
right_lines = []

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            continue  # skip vertical lines
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < 0.3:
            continue  # skip near-horizontal lines
        if slope < 0:
            left_lines.append(line[0])
        else:
            right_lines.append(line[0])
```

### 5.6 Averaging and Extrapolating

Multiple line segments per lane boundary should be merged into one representative line:

```python
def average_lines(lines, h):
    """Average multiple line segments into one extrapolated line."""
    if len(lines) == 0:
        return None

    # Collect all slopes and intercepts
    slopes = []
    intercepts = []
    for x1, y1, x2, y2 in lines:
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        slopes.append(slope)
        intercepts.append(intercept)

    avg_slope = np.mean(slopes)
    avg_intercept = np.mean(intercepts)

    # Extrapolate from bottom of image to 60% height
    y_bottom = h
    y_top = int(0.6 * h)
    x_bottom = int((y_bottom - avg_intercept) / avg_slope)
    x_top = int((y_top - avg_intercept) / avg_slope)

    return [x_bottom, y_bottom, x_top, y_top]
```

---

## 6. Perspective Transform — Bird's Eye View (BEV)

### 6.1 Why BEV?

From the camera's perspective, parallel lane lines **converge** toward a vanishing point. This projective distortion makes it impossible to measure lane curvature accurately. A **Bird's Eye View** transform "undoes" the perspective projection, making parallel lanes appear parallel and enabling accurate polynomial fitting.

### 6.2 The Math — Homography

A perspective transform is a **3x3 homography matrix** \(\mathbf{H}\) that maps source points to destination points:

$$
\begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix}
\sim
\mathbf{H}
\begin{pmatrix} x \\ y \\ 1 \end{pmatrix}
= \begin{pmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{pmatrix}
\begin{pmatrix} x \\ y \\ 1 \end{pmatrix}
$$

The \(\sim\) means equality up to a scale factor. The matrix has 8 degrees of freedom (9 entries minus 1 for scale), so we need **4 point correspondences** (each gives 2 equations) to solve for \(\mathbf{H}\).

### 6.3 Choosing Source and Destination Points

The source points form a **trapezoid** on the original image (the road region where lanes are visible). The destination points form a **rectangle** in the warped image.

```python
h, w = image.shape[:2]

# Source: trapezoid on original image
src_points = np.float32([
    [int(0.43 * w), int(0.65 * h)],  # top-left
    [int(0.57 * w), int(0.65 * h)],  # top-right
    [int(0.90 * w), int(0.95 * h)],  # bottom-right
    [int(0.10 * w), int(0.95 * h)],  # bottom-left
])

# Destination: rectangle
dst_points = np.float32([
    [int(0.20 * w), 0],              # top-left
    [int(0.80 * w), 0],              # top-right
    [int(0.80 * w), h],              # bottom-right
    [int(0.20 * w), h],              # bottom-left
])
```

### 6.4 Connection to Day 11 Calibration

In Day 11 you calibrated your camera and obtained the intrinsic matrix \(\mathbf{K}\) and distortion coefficients. **Undistort the image first**, then apply the perspective transform:

```python
import pickle

# Load calibration from Day 11
with open("calibration.pkl", "rb") as f:
    calib = pickle.load(f)

K = calib["camera_matrix"]
dist = calib["dist_coeffs"]

# Step 1: Undistort
undistorted = cv2.undistort(bgr_image, K, dist)

# Step 2: Perspective transform
M = cv2.getPerspectiveTransform(src_points, dst_points)
M_inv = cv2.getPerspectiveTransform(dst_points, src_points)  # for unwarping later

bev = cv2.warpPerspective(undistorted, M, (w, h))
```

The inverse matrix `M_inv` is essential — it lets you project detected lane points back onto the original camera view for visualization and steering.

### 6.5 Verifying the Transform

A good BEV transform should make straight lane lines appear **vertical and parallel** in the warped image. If the lines converge or diverge, adjust the source points.

```python
# Draw source trapezoid on original
vis_src = undistorted.copy()
cv2.polylines(vis_src, [src_points.astype(int)], True, (0, 0, 255), 3)

# Draw destination rectangle on BEV
vis_dst = bev.copy()
cv2.polylines(vis_dst, [dst_points.astype(int)], True, (0, 255, 0), 3)

cv2.imshow("Source", vis_src)
cv2.imshow("BEV", vis_dst)
cv2.waitKey(0)
```

---

## 7. Sliding Window Lane Detection

### 7.1 Why Sliding Window?

The Hough transform detects **straight** lines. Real road lanes **curve**. The sliding window method finds lane pixels in a BEV binary image by searching column by column from bottom to top, following the lane wherever it goes.

### 7.2 Algorithm Step by Step

**Step 1: Histogram peak detection**

Take the bottom half of the BEV binary image and compute a column-wise histogram (sum of white pixels per column). The two highest peaks indicate the **starting x-positions** of the left and right lanes.

```python
def find_lane_starts(binary_bev):
    """Find left and right lane starting x-positions using histogram."""
    bottom_half = binary_bev[binary_bev.shape[0] // 2:, :]
    histogram = np.sum(bottom_half, axis=0)

    midpoint = histogram.shape[0] // 2
    left_x = np.argmax(histogram[:midpoint])
    right_x = np.argmax(histogram[midpoint:]) + midpoint

    return left_x, right_x, histogram
```

**Step 2: Sliding windows**

Divide the image vertically into \(N\) horizontal bands (windows). Start at the bottom with windows centered on the histogram peaks. For each window:

1. Identify all white pixels within the window boundaries.
2. If enough pixels found (> `minpix`), **recenter** the next window on the mean x-position of those pixels.
3. Collect all identified lane pixels.

```python
def sliding_window_search(binary_bev, left_x_start, right_x_start,
                          n_windows=9, margin=80, minpix=50):
    """
    Sliding window lane pixel search.

    Parameters:
        binary_bev: BEV binary image (single channel, 0 or 255)
        left_x_start: starting x for left lane
        right_x_start: starting x for right lane
        n_windows: number of sliding windows
        margin: half-width of each window
        minpix: minimum pixels to recenter window

    Returns:
        left_lane_pixels: (y_coords, x_coords) of left lane pixels
        right_lane_pixels: (y_coords, x_coords) of right lane pixels
        visualization: image showing the windows
    """
    h, w = binary_bev.shape
    window_height = h // n_windows

    # Identify all nonzero pixel positions
    nonzero_y, nonzero_x = binary_bev.nonzero()

    # Current window centers
    left_x_current = left_x_start
    right_x_current = right_x_start

    # Collect pixel indices for each lane
    left_lane_inds = []
    right_lane_inds = []

    # Visualization
    vis = np.dstack([binary_bev, binary_bev, binary_bev])

    for win in range(n_windows):
        # Window vertical boundaries (top to bottom)
        y_low = h - (win + 1) * window_height
        y_high = h - win * window_height

        # Left window horizontal boundaries
        left_x_low = left_x_current - margin
        left_x_high = left_x_current + margin

        # Right window horizontal boundaries
        right_x_low = right_x_current - margin
        right_x_high = right_x_current + margin

        # Draw windows on visualization
        cv2.rectangle(vis, (left_x_low, y_low), (left_x_high, y_high), (0, 255, 0), 2)
        cv2.rectangle(vis, (right_x_low, y_low), (right_x_high, y_high), (0, 255, 0), 2)

        # Find pixels within left window
        good_left = (
            (nonzero_y >= y_low) & (nonzero_y < y_high) &
            (nonzero_x >= left_x_low) & (nonzero_x < left_x_high)
        ).nonzero()[0]

        # Find pixels within right window
        good_right = (
            (nonzero_y >= y_low) & (nonzero_y < y_high) &
            (nonzero_x >= right_x_low) & (nonzero_x < right_x_high)
        ).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        # Recenter if enough pixels found
        if len(good_left) > minpix:
            left_x_current = int(np.mean(nonzero_x[good_left]))
        if len(good_right) > minpix:
            right_x_current = int(np.mean(nonzero_x[good_right]))

    # Concatenate all window results
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract pixel coordinates
    left_y = nonzero_y[left_lane_inds]
    left_x = nonzero_x[left_lane_inds]
    right_y = nonzero_y[right_lane_inds]
    right_x = nonzero_x[right_lane_inds]

    return (left_y, left_x), (right_y, right_x), vis
```

### 7.3 Key Parameters

| Parameter | Typical Value | Effect |
|-----------|--------------|--------|
| `n_windows` | 9 | More windows = finer vertical resolution |
| `margin` | 80 px | Wider = captures more curved lanes, but more noise |
| `minpix` | 50 | Higher = more confident recentering, but may "lose" thin lanes |

---

## 8. Polynomial Fitting

### 8.1 Why a Polynomial?

Lane lines are not straight — they curve. In the BEV image, we model each lane boundary as a **second-order polynomial** (parabola):

$$
x = f(y) = A y^2 + B y + C
$$

Note: we fit \(x\) as a function of \(y\) (not \(y\) as a function of \(x\)) because lanes are nearly vertical in the BEV image. A function of \(y\) avoids the problem of multiple \(y\) values for a single \(x\).

### 8.2 Least Squares Fitting

Given \(N\) detected lane pixels \(\{(y_i, x_i)\}_{i=1}^{N}\), we minimize:

$$
\min_{A, B, C} \sum_{i=1}^{N} \left(x_i - A y_i^2 - B y_i - C\right)^2
$$

This is a standard linear least squares problem. In matrix form:

$$
\underbrace{\begin{pmatrix} y_1^2 & y_1 & 1 \\ y_2^2 & y_2 & 1 \\ \vdots & \vdots & \vdots \\ y_N^2 & y_N & 1 \end{pmatrix}}_{\mathbf{Y}}
\underbrace{\begin{pmatrix} A \\ B \\ C \end{pmatrix}}_{\mathbf{p}}
= \underbrace{\begin{pmatrix} x_1 \\ x_2 \\ \vdots \\ x_N \end{pmatrix}}_{\mathbf{x}}
$$

The solution is:

$$
\mathbf{p} = (\mathbf{Y}^T \mathbf{Y})^{-1} \mathbf{Y}^T \mathbf{x}
$$

NumPy's `polyfit` handles this:

```python
# Fit left lane
left_fit = np.polyfit(left_y, left_x, deg=2)  # returns [A, B, C]

# Fit right lane
right_fit = np.polyfit(right_y, right_x, deg=2)

# Generate smooth curve for plotting
plot_y = np.linspace(0, binary_bev.shape[0] - 1, binary_bev.shape[0])
left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
```

### 8.3 Lane Center and Cross-Track Error

The **cross-track error (CTE)** is the lateral offset between the vehicle and the lane center. It is the input to the PID steering controller from Day 16.

```python
def compute_cte(left_fit, right_fit, image_height, image_width):
    """
    Compute cross-track error: how far the car is from lane center.

    Positive CTE = car is right of center → steer left
    Negative CTE = car is left of center → steer right
    """
    # Evaluate lane positions at the bottom of the image (closest to car)
    y_eval = image_height - 1

    left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
    right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]

    lane_center = (left_x + right_x) / 2.0
    image_center = image_width / 2.0

    cte = lane_center - image_center  # in pixels

    return cte
```

To convert CTE from **pixels to meters**, you need a calibration factor. For a typical BEV with known road width:

$$
\text{CTE}_{\text{meters}} = \text{CTE}_{\text{pixels}} \times \frac{\text{lane\_width\_meters}}{\text{lane\_width\_pixels}}
$$

For US roads, standard lane width is 3.7 m. Measure the pixel distance between detected lanes in the BEV to get the conversion factor.

### 8.4 Radius of Curvature

The radius of curvature at a point on the fitted curve is:

$$
R = \frac{\left(1 + \left(\frac{dx}{dy}\right)^2\right)^{3/2}}{\left|\frac{d^2x}{dy^2}\right|}
$$

For our polynomial \(x = Ay^2 + By + C\):

$$
\frac{dx}{dy} = 2Ay + B, \qquad \frac{d^2x}{dy^2} = 2A
$$

Therefore:

$$
R = \frac{(1 + (2Ay + B)^2)^{3/2}}{|2A|}
$$

```python
def radius_of_curvature(fit_coeffs, y_eval):
    """Compute radius of curvature in pixels."""
    A, B, C = fit_coeffs
    R = ((1 + (2 * A * y_eval + B)**2)**1.5) / abs(2 * A)
    return R
```

---

## 9. Hands-On Lab: Complete Lane Detection Pipeline

Now let's put everything together into one coherent pipeline.

### 9.1 Full Pipeline Code

```python
"""
Lane Detection Pipeline
Day 17 — Embedded Basics for Autonomous Car

Complete pipeline: Camera → Undistort → HSV Mask → Canny → BEV →
                   Sliding Window → Polynomial Fit → CTE
"""

import cv2
import numpy as np
import pickle


# ─────────────────────────────────────────────
# 1. Configuration
# ─────────────────────────────────────────────

class LaneConfig:
    """All tunable parameters in one place."""

    # HSV thresholds for yellow
    YELLOW_LOW = np.array([15, 80, 80])
    YELLOW_HIGH = np.array([35, 255, 255])

    # HSV thresholds for white
    WHITE_LOW = np.array([0, 0, 200])
    WHITE_HIGH = np.array([179, 40, 255])

    # Canny thresholds
    CANNY_LOW = 50
    CANNY_HIGH = 150

    # Morphological kernel size
    MORPH_KERNEL_SIZE = (5, 5)

    # Sliding window
    N_WINDOWS = 9
    WINDOW_MARGIN = 80
    WINDOW_MINPIX = 50

    # Lane width in meters (for CTE conversion)
    LANE_WIDTH_METERS = 0.30  # 30 cm for a model car track


# ─────────────────────────────────────────────
# 2. Calibration Loader
# ─────────────────────────────────────────────

def load_calibration(path="calibration.pkl"):
    """Load camera matrix and distortion coefficients from Day 11."""
    try:
        with open(path, "rb") as f:
            calib = pickle.load(f)
        return calib["camera_matrix"], calib["dist_coeffs"]
    except FileNotFoundError:
        print("[WARN] No calibration file found. Skipping undistortion.")
        return None, None


# ─────────────────────────────────────────────
# 3. Preprocessing
# ─────────────────────────────────────────────

def undistort(frame, K, dist):
    """Remove lens distortion."""
    if K is None:
        return frame
    return cv2.undistort(frame, K, dist)


def color_mask(frame, config):
    """Create binary mask for lane colors using HSV."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow = cv2.inRange(hsv, config.YELLOW_LOW, config.YELLOW_HIGH)
    white = cv2.inRange(hsv, config.WHITE_LOW, config.WHITE_HIGH)

    combined = cv2.bitwise_or(yellow, white)

    # Morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, config.MORPH_KERNEL_SIZE)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel, iterations=1)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)

    return combined


# ─────────────────────────────────────────────
# 4. BEV Transform
# ─────────────────────────────────────────────

def get_bev_transform(h, w):
    """Compute perspective transform matrices."""
    src = np.float32([
        [int(0.43 * w), int(0.65 * h)],
        [int(0.57 * w), int(0.65 * h)],
        [int(0.90 * w), int(0.95 * h)],
        [int(0.10 * w), int(0.95 * h)],
    ])
    dst = np.float32([
        [int(0.20 * w), 0],
        [int(0.80 * w), 0],
        [int(0.80 * w), h],
        [int(0.20 * w), h],
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv, src, dst


# ─────────────────────────────────────────────
# 5. Sliding Window Search
# ─────────────────────────────────────────────

def histogram_peaks(binary_bev):
    """Find lane start positions from histogram of bottom half."""
    bottom_half = binary_bev[binary_bev.shape[0] // 2:, :]
    histogram = np.sum(bottom_half, axis=0)
    mid = histogram.shape[0] // 2
    left_x = np.argmax(histogram[:mid])
    right_x = np.argmax(histogram[mid:]) + mid
    return left_x, right_x


def sliding_window(binary_bev, config):
    """Full sliding window search returning polynomial fits."""
    h, w = binary_bev.shape
    left_start, right_start = histogram_peaks(binary_bev)

    window_h = h // config.N_WINDOWS
    nonzero_y, nonzero_x = binary_bev.nonzero()

    left_current = left_start
    right_current = right_start

    left_inds = []
    right_inds = []

    vis = np.dstack([binary_bev, binary_bev, binary_bev])

    for win_idx in range(config.N_WINDOWS):
        y_low = h - (win_idx + 1) * window_h
        y_high = h - win_idx * window_h

        # Left window
        xl_low = left_current - config.WINDOW_MARGIN
        xl_high = left_current + config.WINDOW_MARGIN

        # Right window
        xr_low = right_current - config.WINDOW_MARGIN
        xr_high = right_current + config.WINDOW_MARGIN

        cv2.rectangle(vis, (xl_low, y_low), (xl_high, y_high), (0, 255, 0), 2)
        cv2.rectangle(vis, (xr_low, y_low), (xr_high, y_high), (0, 255, 0), 2)

        good_left = (
            (nonzero_y >= y_low) & (nonzero_y < y_high) &
            (nonzero_x >= xl_low) & (nonzero_x < xl_high)
        ).nonzero()[0]

        good_right = (
            (nonzero_y >= y_low) & (nonzero_y < y_high) &
            (nonzero_x >= xr_low) & (nonzero_x < xr_high)
        ).nonzero()[0]

        left_inds.append(good_left)
        right_inds.append(good_right)

        if len(good_left) > config.WINDOW_MINPIX:
            left_current = int(np.mean(nonzero_x[good_left]))
        if len(good_right) > config.WINDOW_MINPIX:
            right_current = int(np.mean(nonzero_x[good_right]))

    left_inds = np.concatenate(left_inds)
    right_inds = np.concatenate(right_inds)

    left_y, left_x = nonzero_y[left_inds], nonzero_x[left_inds]
    right_y, right_x = nonzero_y[right_inds], nonzero_x[right_inds]

    # Polynomial fit
    left_fit = np.polyfit(left_y, left_x, 2) if len(left_y) > 0 else None
    right_fit = np.polyfit(right_y, right_x, 2) if len(right_y) > 0 else None

    return left_fit, right_fit, vis


# ─────────────────────────────────────────────
# 6. CTE Computation
# ─────────────────────────────────────────────

def compute_cte(left_fit, right_fit, h, w, lane_width_m):
    """Compute cross-track error in meters."""
    if left_fit is None or right_fit is None:
        return None

    y_eval = h - 1
    left_x = np.polyval(left_fit, y_eval)
    right_x = np.polyval(right_fit, y_eval)

    lane_center_px = (left_x + right_x) / 2.0
    image_center_px = w / 2.0
    cte_px = lane_center_px - image_center_px

    lane_width_px = right_x - left_x
    if lane_width_px > 0:
        meters_per_pixel = lane_width_m / lane_width_px
    else:
        meters_per_pixel = 1.0  # fallback

    cte_m = cte_px * meters_per_pixel
    return cte_m


# ─────────────────────────────────────────────
# 7. Visualization
# ─────────────────────────────────────────────

def draw_lane_overlay(original, binary_bev, left_fit, right_fit, M_inv):
    """Draw detected lane area back on original image."""
    if left_fit is None or right_fit is None:
        return original

    h, w = binary_bev.shape
    plot_y = np.linspace(0, h - 1, h)
    left_x = np.polyval(left_fit, plot_y)
    right_x = np.polyval(right_fit, plot_y)

    # Create overlay in BEV space
    overlay = np.zeros((h, w, 3), dtype=np.uint8)

    pts_left = np.array([np.flipud(np.column_stack([left_x, plot_y]))], dtype=np.int32)
    pts_right = np.array([np.column_stack([right_x, plot_y])], dtype=np.int32)
    pts = np.hstack((pts_left, pts_right))

    cv2.fillPoly(overlay, pts, (0, 255, 0))

    # Warp back to original perspective
    overlay_unwarped = cv2.warpPerspective(overlay, M_inv, (w, h))

    # Blend with original
    result = cv2.addWeighted(original, 0.8, overlay_unwarped, 0.3, 0)
    return result


# ─────────────────────────────────────────────
# 8. Main Pipeline
# ─────────────────────────────────────────────

def main():
    config = LaneConfig()
    K, dist = load_calibration()

    cap = cv2.VideoCapture(0)  # or video file path
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    ret, frame = cap.read()
    if not ret:
        return

    h, w = frame.shape[:2]
    M, M_inv, src_pts, dst_pts = get_bev_transform(h, w)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Pipeline steps
        undist = undistort(frame, K, dist)
        mask = color_mask(undist, config)
        bev_mask = cv2.warpPerspective(mask, M, (w, h))
        left_fit, right_fit, win_vis = sliding_window(bev_mask, config)
        cte = compute_cte(left_fit, right_fit, h, w, config.LANE_WIDTH_METERS)
        result = draw_lane_overlay(undist, bev_mask, left_fit, right_fit, M_inv)

        # Display CTE
        if cte is not None:
            direction = "RIGHT" if cte > 0 else "LEFT"
            cv2.putText(result, f"CTE: {cte:.3f} m ({direction})",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            cv2.putText(result, "NO LANE DETECTED",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("Lane Detection", result)
        cv2.imshow("BEV + Windows", win_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

### 9.2 Alternative: Canny + Hough Pipeline (for Straight Roads)

If your track has mostly straight lanes, the simpler Hough-based pipeline may be sufficient:

```python
def hough_lane_pipeline(frame, config):
    """Simpler pipeline using Canny + Hough for straight lanes."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, config.CANNY_LOW, config.CANNY_HIGH)

    # ROI mask
    h, w = edges.shape
    roi_vertices = np.array([[
        (int(0.05 * w), h),
        (int(0.40 * w), int(0.6 * h)),
        (int(0.60 * w), int(0.6 * h)),
        (int(0.95 * w), h)
    ]], dtype=np.int32)

    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, roi_vertices, 255)
    roi_edges = cv2.bitwise_and(edges, mask)

    # Hough lines
    lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 50,
                            minLineLength=40, maxLineGap=100)

    # Separate and average
    left_lines, right_lines = [], []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:
                continue
            if slope < 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

    # Draw
    vis = frame.copy()
    for x1, y1, x2, y2 in left_lines:
        cv2.line(vis, (x1, y1), (x2, y2), (255, 0, 0), 3)
    for x1, y1, x2, y2 in right_lines:
        cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 255), 3)

    return vis, left_lines, right_lines
```

### 9.3 Testing Tips

1. **Start with a still image** before testing on video. Capture one frame and tune all parameters.
2. **Print the histogram** to verify that lane starts are detected correctly.
3. **Visualize every stage** — mask, BEV, sliding windows, polynomial overlay — to see where failures occur.
4. **Adjust HSV thresholds** for your specific track lighting. The values above are starting points.

---

## 10. Review and Summary

### What We Covered

| Topic | Key Takeaway |
|-------|-------------|
| Color Spaces | HSV separates color from brightness — essential for robust lane color detection |
| Thresholding | Otsu is automatic for bimodal histograms; use adaptive for uneven lighting |
| Morphology | Opening removes noise; Closing fills gaps. Always apply after thresholding. |
| Canny | Four stages: Smooth → Gradient → NMS → Hysteresis. Two thresholds, 2:1 ratio. |
| Hough | \(\rho = x\cos\theta + y\sin\theta\). Voting in accumulator detects lines. |
| BEV | Perspective transform makes parallel lanes parallel. Needs 4 point pairs. |
| Sliding Window | Follows curved lanes from bottom to top. Histogram initializes search. |
| Polynomial Fit | \(x = Ay^2 + By + C\). CTE = lane center minus image center. |

### Connection to Other Days

- **Day 11 (Camera Calibration):** We load the calibration file to undistort images before processing.
- **Day 9 (PID Control):** The CTE computed today becomes the **error signal** fed to the PID controller.
- **Day 18 (Tomorrow):** We will wrap this entire pipeline into a **ROS2 node**, add sensor fusion with LiDAR, and design safety fallbacks for when lane detection fails.

### Key Formulas to Remember

$$
\text{Canny gradient: } G = \sqrt{G_x^2 + G_y^2}, \quad \theta = \arctan\left(\frac{G_y}{G_x}\right)
$$

$$
\text{Hough line: } \rho = x \cos\theta + y \sin\theta
$$

$$
\text{Lane polynomial: } x = Ay^2 + By + C
$$

$$
\text{CTE} = \frac{x_{\text{left}} + x_{\text{right}}}{2} - \frac{W_{\text{image}}}{2}
$$

$$
\text{Curvature radius: } R = \frac{(1 + (2Ay + B)^2)^{3/2}}{|2A|}
$$

---

**Next up — Day 18:** We integrate this lane detection pipeline into ROS2, add LiDAR-based obstacle detection, and design a fail-safe state machine so the car degrades gracefully when perception fails.
