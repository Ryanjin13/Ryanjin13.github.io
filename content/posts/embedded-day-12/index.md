---
title: "Day 12 — SLAM Fundamentals and RTAB-Map Architecture"
date: 2026-03-05T12:00:00
description: "SLAM problem definition, front-end feature extraction, back-end pose graph optimization, occupancy grid maps, and RTAB-Map architecture for RPi 5"
categories: ["Autonomous Driving"]
tags: ["SLAM", "RTAB-Map", "Visual Odometry", "Loop Closure", "Occupancy Grid"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 12
draft: false
---

{{< katex >}}

## What You'll Learn

We can now measure distances (Day 10), calibrate cameras (Day 11), filter noise (Day 8), and control motors (Day 9). But our car still does not know **where it is** or **what the environment looks like**. That is the SLAM problem — and it is one of the most important problems in all of robotics.

By the end of this post you will be able to:

1. Explain the chicken-and-egg nature of SLAM and write the joint probability equation.
2. Describe the **front-end** (feature extraction, visual odometry, wheel odometry) and **back-end** (pose graph optimization, loop closure) architecture.
3. Understand **occupancy grid maps** and derive the log-odds update formula.
4. Explain the **RTAB-Map** architecture in detail: Working Memory, Long-Term Memory, and loop closure detection.
5. Configure RTAB-Map parameters for Raspberry Pi 5.
6. Run SLAM with depth camera, IMU, and wheel odometry using ROS2.
7. Record and replay data with `ros2 bag` for offline experimentation.

---

## 1. The SLAM Problem

### 1.1 The Chicken-and-Egg Dilemma

Imagine you wake up blindfolded in an unknown building. You can feel walls, hear echoes, and count your steps. You need to simultaneously:

- Figure out where you are (**Localization**) — but this requires a map.
- Build a mental map of the building (**Mapping**) — but this requires knowing your position.

Neither can be solved independently. This circular dependency is the core challenge of SLAM.

```
  ┌──────────────────────────────────────────────┐
  │              The SLAM Chicken-and-Egg         │
  │                                               │
  │   "Where am I?"  ◄───── needs ────► "What    │
  │   (Localization)                    does the  │
  │        │                            world     │
  │        │                            look      │
  │        └──── needs ────►            like?"    │
  │                                    (Mapping)  │
  │                                               │
  │   You need a map to localize,                 │
  │   but you need your location to build a map.  │
  └──────────────────────────────────────────────┘
```

### 1.2 Formal Problem Statement

SLAM seeks the joint posterior distribution of the robot trajectory \(\mathbf{x}_{0:t}\) and the map \(\mathbf{m}\), given all sensor observations \(\mathbf{z}_{1:t}\) and control inputs \(\mathbf{u}_{1:t}\):

$$
p(\mathbf{x}_{0:t}, \mathbf{m} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})
$$

This is a high-dimensional estimation problem. Direct computation is intractable, so practical SLAM systems decompose it into manageable pieces.

Using the chain rule and Markov assumptions (the current state depends only on the previous state and current action):

$$
p(\mathbf{x}_{0:t}, \mathbf{m} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) \propto p(\mathbf{z}_t \mid \mathbf{x}_t, \mathbf{m}) \cdot p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \cdot p(\mathbf{x}_{0:t-1}, \mathbf{m} \mid \mathbf{z}_{1:t-1}, \mathbf{u}_{1:t-1})
$$

where:
- \(p(\mathbf{z}_t \mid \mathbf{x}_t, \mathbf{m})\) is the **observation model** — how likely is this sensor reading given this pose and map?
- \(p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t)\) is the **motion model** — where do we expect to be given the previous pose and control input?

### 1.3 The Two Key Decompositions

**Decomposition 1: Filtering vs Smoothing**

| Approach | Estimates | Example |
|----------|----------|---------|
| **Filtering** | Current pose + map only: \(p(\mathbf{x}_t, \mathbf{m} \mid \mathbf{z}_{1:t})\) | EKF-SLAM, particle filter SLAM |
| **Smoothing** | Full trajectory + map: \(p(\mathbf{x}_{0:t}, \mathbf{m} \mid \mathbf{z}_{1:t})\) | Graph-based SLAM (used by RTAB-Map) |

Smoothing is more accurate because it can retroactively correct past estimates when new information (like loop closure) arrives. This is why modern SLAM systems use graph-based approaches.

**Decomposition 2: Front-End + Back-End**

| Component | Role | Speed requirement |
|-----------|------|-------------------|
| **Front-end** | Extract information from raw sensors | Real-time (every frame) |
| **Back-end** | Optimize global consistency | Can be slower (triggered by events) |

```
  ┌──────────────────────────────────────────┐
  │              The SLAM Loop               │
  │                                          │
  │   Sensors ──► Feature          Map       │
  │               Extraction  ◄──  Update    │
  │                   │              ▲       │
  │                   ▼              │       │
  │               Data         Pose Graph    │
  │             Association    Optimization  │
  │                   │              ▲       │
  │                   ▼              │       │
  │               Motion ──► Pose   │       │
  │               Model     Estimate ──┘    │
  │                                          │
  └──────────────────────────────────────────┘
```

---

## 2. Front-End: Feature Extraction

### 2.1 Why Features?

To recognize places and track motion, we need to identify distinctive visual landmarks in images — **features** or **keypoints**. A good feature is:

- **Repeatable**: detected in the same location across different viewpoints.
- **Distinctive**: its descriptor uniquely identifies it among thousands of candidates.
- **Fast to compute**: we need hundreds per frame at 30+ fps.

### 2.2 FAST Corner Detection

FAST (Features from Accelerated Segment Test) is the first step in most real-time feature detectors. It examines a ring of 16 pixels around each candidate pixel:

```
    . . 1 2 3 . .
    . 16        4 .
    15            5
    14      P     6     P = candidate pixel
    13            7     1-16 = Bresenham circle pixels
    . 12       8 .
    . . 11 10 9 . .
```

A pixel \(P\) with intensity \(I_P\) is a corner if there exists a contiguous arc of \(N\) pixels on the circle (typically \(N = 9\) or \(N = 12\)) that are all brighter than \(I_P + \tau\) or all darker than \(I_P - \tau\), where \(\tau\) is a threshold.

**Speed trick**: check pixels 1, 5, 9, 13 first. If fewer than 3 of these 4 pass the brightness/darkness test, the pixel cannot be a corner — reject immediately. This eliminates the vast majority of pixels with just 4 comparisons.

### 2.3 ORB Features (Oriented FAST and Rotated BRIEF)

ORB combines FAST detection with BRIEF binary descriptors, adding rotation and scale invariance:

1. **Multi-scale FAST**: build an image pyramid (typically 8 levels, scale factor 1.2) and detect FAST corners at each level for scale invariance.

2. **Orientation assignment**: compute the **intensity centroid** of the patch around each keypoint:

$$
\theta = \arctan\!\left(\frac{m_{01}}{m_{10}}\right)
$$

where \(m_{pq} = \sum_{x,y} x^p y^q I(x, y)\) are image moments computed over a patch around the keypoint. This angle provides rotation invariance.

3. **rBRIEF descriptor**: compute a 256-bit binary descriptor by comparing pixel pairs in a learned pattern, rotated by \(\theta\):

$$
\tau(p; x_a, x_b) = \begin{cases} 1 & \text{if } I(x_a) < I(x_b) \\ 0 & \text{otherwise} \end{cases}
$$

The 256 binary comparisons produce a 256-bit (32-byte) descriptor.

4. **Matching**: use **Hamming distance** (XOR + popcount), which is extremely fast on modern CPUs — a single instruction on most architectures:

$$
d_{\text{Hamming}}(a, b) = \texttt{popcount}(a \oplus b)
$$

**ORB extracts ~1000 features per frame in < 10 ms on RPi 5**, making it the standard choice for embedded SLAM.

### 2.4 Feature Matching and Outlier Rejection

Given features from frame \(A\) and frame \(B\), find correspondences:

$$
\text{match}(f_A) = \arg\min_{f_B} \; d_{\text{Hamming}}(f_A, f_B)
$$

Apply **Lowe's ratio test** to reject ambiguous matches:

$$
\frac{d_{\text{best}}}{d_{\text{second}}} < 0.75
$$

If the best match is not significantly better than the second best, the match is likely wrong. This simple test eliminates most false matches.

After the ratio test, apply **RANSAC** (Random Sample Consensus) with the fundamental or essential matrix to reject geometrically inconsistent matches. RANSAC randomly selects minimal subsets of matches, fits a geometric model, and counts inliers. After many iterations, the model with the most inliers wins.

---

## 3. Front-End: Odometry

### 3.1 Visual Odometry

Visual odometry (VO) estimates camera motion between consecutive frames using matched features:

```
  Frame k                          Frame k+1
  ┌──────────────────┐            ┌──────────────────┐
  │  *1   *2         │            │     *1'  *2'      │
  │        *3        │   motion   │          *3'      │
  │  *4        *5    │   ────►   │  *4'        *5'   │
  └──────────────────┘            └──────────────────┘

  Match features: *i <-> *i'
  With depth: solve R, t using 3D-3D correspondences
  Without depth: estimate essential matrix, decompose into R, t
```

For an RGB-D camera (like our RealSense D435), we have depth at each feature point. Given \(N\) matched 3D-3D point pairs, solve:

$$
\mathbf{P}_{k+1}^{(i)} = R \, \mathbf{P}_k^{(i)} + t, \qquad i = 1, \ldots, N
$$

This is the **rigid body registration** problem, solvable by:

- **SVD method**: compute centroids \(\bar{p}\) and \(\bar{q}\), form the cross-covariance matrix \(H = \sum (p_i - \bar{p})(q_i - \bar{q})^T\), and decompose via SVD: \(H = U \Sigma V^T\), then \(R = V U^T\).
- **ICP** (Iterative Closest Point): iteratively refine R, t.
- **PnP** (Perspective-n-Point): for 2D-3D correspondences.

### 3.2 Frame-to-Map vs Frame-to-Frame

| Strategy | Description | Accuracy | Speed |
|----------|-------------|----------|-------|
| **Frame-to-Frame** | Match current frame to previous frame | Lower (drift) | Faster |
| **Frame-to-Map** | Match current frame to a local map of recent features | Higher (less drift) | Slower |

RTAB-Map's `Odom/Strategy: 0` uses Frame-to-Map, building a local feature map from recent keyframes and matching the current frame against it. This significantly reduces drift compared to Frame-to-Frame.

### 3.3 Wheel Odometry

Hall sensor encoders from Day 6 provide a complementary motion estimate. For a differential-drive robot:

$$
\Delta s = \frac{\Delta s_L + \Delta s_R}{2}, \qquad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{b}
$$

$$
\Delta x = \Delta s \cos\theta, \qquad \Delta y = \Delta s \sin\theta
$$

where \(\Delta s_L, \Delta s_R\) are left/right wheel arc lengths and \(b\) is the wheel base.

### 3.4 Odometry Drift Comparison

| Odometry type | Typical drift | Primary failure mode |
|---------------|--------------|---------------------|
| Visual only | 0.5 -- 2% of distance | Textureless walls, darkness, blur |
| Wheel only | 1 -- 5% of distance | Wheel slip, uneven surfaces |
| Fused (visual + wheel) | 0.3 -- 1% of distance | Simultaneous failure of both |

Both types accumulate errors over time. This is why we need **loop closure**.

---

## 4. Back-End: Pose Graph Optimization

### 4.1 The Pose Graph

As the robot moves, we build a **graph** where:
- **Nodes** represent robot poses \(x_i = (x, y, \theta)\) at keyframe times.
- **Edges** represent relative transform constraints between poses.

```
  Pose Graph:

  x0 ──── x1 ──── x2 ──── x3 ──── x4
                                     │
                              loop   │ loop closure
                            closure  │ edge
                                     │
  x9 ──── x8 ──── x7 ──── x6 ──── x5

  Sequential edges: small uncertainty (from odometry)
  Loop closure edges: connects distant poses (from place recognition)
```

Each edge stores:
- A **measured relative transform** \(\mathbf{z}_{ij}\) (e.g., "node \(j\) is 0.5 m forward and 0.1 rad right of node \(i\)").
- An **information matrix** \(\Omega_{ij} = \Sigma_{ij}^{-1}\) expressing measurement certainty. Higher values mean more certain measurements, which get more weight in optimization.

### 4.2 Graph Optimization: The Math

The back-end minimizes the total weighted error across all edges:

$$
\mathbf{x}^* = \arg\min_{\mathbf{x}} \sum_{(i,j) \in \mathcal{E}} \mathbf{e}_{ij}^T \, \Omega_{ij} \, \mathbf{e}_{ij}
$$

where the residual for each edge is:

$$
\mathbf{e}_{ij} = \mathbf{z}_{ij} \ominus (x_j \ominus x_i)
$$

The \(\ominus\) operator computes the relative transform between two poses. For 2D poses:

$$
x_j \ominus x_i = \begin{bmatrix} \cos\theta_i & \sin\theta_i & 0 \\ -\sin\theta_i & \cos\theta_i & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_j - x_i \\ y_j - y_i \\ \theta_j - \theta_i \end{bmatrix}
$$

This is a **nonlinear least-squares problem** solved iteratively using Gauss-Newton or Levenberg-Marquardt. Libraries like **g2o** and **GTSAM** handle the sparse matrix structure efficiently.

### 4.3 Why Graph Optimization Works

When a loop closure edge is added, it introduces a strong constraint connecting two nodes that are far apart in the graph but close in physical space. The optimizer must adjust all intermediate poses to satisfy both the sequential odometry edges and the loop closure edge simultaneously.

```
  Before Loop Closure               After Loop Closure

  Start *───*                       Start *───*
         \   \                            │    \
          *   *                           *     *
           \   \                           \   │
            *   *                           * ─*
             \   \ ← accumulated            │  │
              *   *   drift                 *  *
               \   \                         \ │
                *───* ← gap!                 *─*  ← closed!
```

The key insight: the error from drift is distributed across many edges (each absorbing a small correction), rather than being concentrated at one point.

### 4.4 Loop Closure Detection

Loop closure is the "magic ingredient" that bounds drift. In RTAB-Map:

1. **Extract ORB features** from the current frame.
2. **Build bag-of-words** (BoW): quantize each descriptor to the nearest visual word in a pre-trained vocabulary, then create a histogram of word frequencies.
3. **TF-IDF search**: compare the BoW vector against all WM nodes:

$$
\text{sim}(q, d) = \sum_{w \in \text{vocab}} \text{tf}(w, q) \cdot \text{idf}(w) \cdot \text{tf}(w, d) \cdot \text{idf}(w)
$$

4. **Threshold**: if \(\text{sim} > \texttt{LoopThr}\), flag as a candidate.
5. **Geometric verification**: match features between the two frames, compute the relative pose using RANSAC, verify sufficient inliers.
6. **Add edge**: create a loop closure constraint in the pose graph.
7. **Re-optimize**: run graph optimization to correct the full trajectory.

---

## 5. Occupancy Grid Maps

### 5.1 The Map Representation

An occupancy grid divides the environment into a uniform grid of cells. Each cell stores the **probability of being occupied**:

```
  Occupancy Grid (top-down view):

  ############            # = Occupied (wall)     p > 0.7
  #          #            . = Free (empty space)  p < 0.3
  #  ........#            ? = Unknown             p ~ 0.5
  #  ..##..?.#
  #  ..##..?.#
  #  ......?.#            * = Robot
  #  ..*...?.#
  ############

  Cell size: 5 cm x 5 cm typical
```

### 5.2 Log-Odds Representation

Working with probabilities directly causes numerical issues. Instead, we use **log-odds**:

$$
l(m_i) = \log \frac{p(m_i)}{1 - p(m_i)}
$$

The inverse transform recovers probability:

$$
p(m_i) = \frac{1}{1 + e^{-l(m_i)}}
$$

| Probability | Log-odds | Interpretation |
|-------------|----------|----------------|
| 0.0 | \(-\infty\) | Certainly free |
| 0.1 | -2.2 | Likely free |
| 0.5 | 0.0 | Unknown (prior) |
| 0.9 | +2.2 | Likely occupied |
| 1.0 | \(+\infty\) | Certainly occupied |

### 5.3 The Log-Odds Update Rule

The Bayesian update in log-odds form becomes a simple addition:

$$
\boxed{l_t(m_i) = l_{t-1}(m_i) + l_{\text{sensor}}(m_i) - l_0}
$$

With prior \(p_0 = 0.5\), the prior log-odds \(l_0 = 0\), so:

$$
l_t(m_i) = l_{t-1}(m_i) + l_{\text{sensor}}(m_i)
$$

**Why this is elegant**:
- No multiplication of small probabilities (numerically stable).
- Update is a simple **addition** — one operation per cell.
- Easy to clamp: set \(l_{\max}\) and \(l_{\min}\) to prevent over-confidence.
- Multiple observations accumulate evidence naturally.

### 5.4 Sensor Model for Ray Casting

Given a depth measurement, cast a ray from the robot to the measured endpoint:

```
  Robot * ─ ─ ─ ─ ─ ─ ─ ─ # Wall
         free  free  free  occupied

  Free cells along ray:     l += l_free  (e.g., -0.4)
  Endpoint (occupied cell): l += l_occ   (e.g., +0.85)
```

The log-odds values come from the sensor model:

$$
l_{\text{occ}} = \log\frac{p_{\text{occ}}}{1 - p_{\text{occ}}}, \qquad l_{\text{free}} = \log\frac{p_{\text{free}}}{1 - p_{\text{free}}}
$$

For a RealSense D435 at close range (\(p_{\text{occ}} = 0.7\), \(p_{\text{free}} = 0.3\)):

$$
l_{\text{occ}} = \log\frac{0.7}{0.3} \approx 0.85, \qquad l_{\text{free}} = \log\frac{0.3}{0.7} \approx -0.85
$$

### 5.5 From Depth Image to Grid

The complete pipeline:

1. For each depth pixel \((u, v)\) with depth \(d\):
2. **Backproject** to 3D camera frame using \(K\) from Day 11:
   \(\mathbf{P}_c = d \cdot K^{-1}[u, v, 1]^T\)
3. **Transform** to world frame using current pose: \(\mathbf{P}_w = R \mathbf{P}_c + t\)
4. **Discretize** to grid cell: \(g_x = \lfloor P_{w,x} / \text{cell\_size} \rfloor\)
5. **Ray-cast** (Bresenham's algorithm) from robot cell to endpoint cell.
6. **Update** all cells along the ray.

---

## 6. RTAB-Map Architecture

### 6.1 Why RTAB-Map?

**RTAB-Map** (Real-Time Appearance-Based Mapping) is our SLAM system of choice because:
- Designed for **RGB-D cameras** (exactly our setup).
- Robust **loop closure** detection using visual bag-of-words.
- Unique **memory management** enables unlimited operation time on constrained hardware.
- Full **ROS2 integration** with standard message types.
- Outputs both **2D occupancy grids** (for Nav2 navigation) and **3D point clouds**.

### 6.2 Core Architecture

```
                    ┌──────────────────────────────────┐
                    │          RTAB-Map Core             │
                    │                                    │
  RGB Image ──────►│  ┌──────────┐   ┌──────────────┐  │
                    │  │  Visual  │   │    Loop      │  │
  Depth Image ────►│  │ Odometry │   │   Closure    │  │
                    │  │ (VO)     │   │  Detection   │  │
  Wheel Odom ─────►│  └────┬─────┘   └──────┬───────┘  │
                    │       │                │           │
  IMU ────────────►│       ▼                ▼           │
                    │  ┌──────────────────────────────┐  │
                    │  │   Pose Graph Optimization    │  │──► Occupancy Grid
                    │  │   (g2o / GTSAM backend)      │  │──► 3D Point Cloud
                    │  └──────────────────────────────┘  │──► TF: map → odom
                    │                                    │
                    │  ┌──────────────────────────────┐  │
                    │  │    Memory Management         │  │
                    │  │    STM ◄─► WM ◄─► LTM        │  │
                    │  └──────────────────────────────┘  │
                    └──────────────────────────────────┘
```

### 6.3 Memory Management: STM, WM, LTM

This is RTAB-Map's key innovation for embedded systems.

| Tier | Name | Storage | Size | Role |
|------|------|---------|------|------|
| **STM** | Short-Term Memory | RAM | Fixed (e.g., 10 nodes) | Most recent keyframes |
| **WM** | Working Memory | RAM | Variable (bounded by time) | Actively searched for loop closure |
| **LTM** | Long-Term Memory | SQLite on disk | Unlimited | Archived nodes, retrieved on demand |

```
  New keyframe ──► STM (ring buffer, last 10 nodes)
                    │
                    ▼ (oldest STM node moves to WM)
                   WM (all nodes actively compared for loop closure)
                    │
                    ▼ (when iteration exceeds TimeThr budget)
                   LTM (on disk — retrieved ONLY if loop closure detected)
                    │
                    ▲ (if BoW similarity with current frame is high,
                       retrieve from LTM back to WM for verification)
```

**The `Rtabmap/TimeThr` parameter** controls this flow. If the current iteration's processing time exceeds `TimeThr` (e.g., 700 ms), the least-recently-accessed WM nodes are transferred to LTM. This keeps each iteration within the time budget.

**Result**: RTAB-Map can run indefinitely without exhausting RAM. The RPi 5 with 4-8 GB RAM can map for hours because old data lives on disk.

### 6.4 Sensor Roles in Our Car

| Sensor | RTAB-Map input | Role | Day reference |
|--------|---------------|------|---------------|
| Depth Camera (D435) | RGB + Depth images | Primary visual features + 3D mapping | Day 10 |
| IMU | `sensor_msgs/Imu` | Gravity alignment, rotation prediction | Day 7 |
| Wheel Odometry | `nav_msgs/Odometry` | Motion prior between frames | Day 6 |
| 1D LiDAR | Costmap obstacle layer | Forward obstacle detection | Day 10 |

---

## 7. RTAB-Map Tuning for RPi 5

### 7.1 Critical Parameters

```yaml
# RTAB-Map parameters optimized for Raspberry Pi 5 (4-8 GB RAM)
rtabmap:
  ros__parameters:
    # === Memory Management ===
    Mem/STMSize: "10"              # Short-term memory: last 10 keyframes
    Rtabmap/TimeThr: "700"         # Max processing time per iteration (ms)

    # === Feature Detection ===
    Kp/MaxFeatures: "200"          # Keypoints for loop closure matching
    Vis/FeatureType: "6"           # ORB (fast, patent-free)
    Vis/MaxFeatures: "200"         # Keypoints for visual odometry

    # === Visual Odometry ===
    Odom/Strategy: "0"             # 0=Frame-to-Map, 1=Frame-to-Frame
    RGBD/LinearUpdate: "0.1"       # 10cm minimum movement per keyframe
    RGBD/AngularUpdate: "0.1"      # 0.1 rad minimum rotation per keyframe

    # === Loop Closure ===
    Rtabmap/LoopThr: "0.11"        # BoW similarity threshold
    RGBD/OptimizeFromGraphEnd: "false"

    # === Occupancy Grid ===
    Grid/CellSize: "0.05"          # 5cm resolution
    Grid/RangeMax: "3.0"           # 3m max depth for grid updates
    Grid/FromDepth: "true"
    Grid/MaxGroundHeight: "0.02"   # Below 2cm = ground plane
    Grid/MaxObstacleHeight: "0.5"  # Above 50cm = ceiling (ignore)

    # === Database ===
    Db/Sqlite3InMemory: "false"    # Save to disk (essential for RPi 5)
```

### 7.2 Performance on RPi 5

| Metric | Value |
|--------|-------|
| Processing time per frame | 400 -- 700 ms |
| Effective map update rate | 1.5 -- 3 Hz |
| Features tracked per frame | ~200 |
| Occupancy grid resolution | 5 cm |
| Map database (10 min run) | 50 -- 100 MB |
| RAM usage | 1 -- 2 GB |
| Loop closure detection | 50 -- 200 ms |

### 7.3 Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "Not enough features" | Blank walls, low light | Add texture, improve lighting |
| Excessive drift | Long corridors without loops | Rely more on wheel odometry |
| High CPU usage | Too many features | Reduce `Kp/MaxFeatures` to 150 |
| Map gaps | Moving too fast | Slow down or reduce `LinearUpdate` |
| False loop closures | Repeated patterns | Increase `LoopThr` to 0.15 |
| Out of memory | WM too large | Reduce `TimeThr` to 500 |
| VO failure | Fast rotation | Add IMU for rotation estimation |

---

## 8. Hands-On Lab

### 8.1 ORB Feature Detection and Matching

```python
"""
orb_matching.py
Detect ORB features and match between two frames.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt


def demo_orb_matching():
    """Demonstrate ORB feature detection and matching."""
    h, w = 480, 640

    # Create synthetic textured images
    img1 = np.random.randint(0, 255, (h, w), dtype=np.uint8)
    img1 = cv2.GaussianBlur(img1, (5, 5), 3)
    cv2.rectangle(img1, (100, 100), (300, 300), 200, 2)
    cv2.circle(img1, (450, 250), 80, 180, 2)
    cv2.line(img1, (50, 400), (600, 350), 220, 2)

    # Second image: shifted (simulating camera motion)
    M = np.float32([[1, 0, 15], [0, 1, 8]])
    img2 = cv2.warpAffine(img1, M, (w, h))
    noise = np.random.normal(0, 10, img2.shape).astype(np.int16)
    img2 = np.clip(img2.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # Detect ORB features
    orb = cv2.ORB_create(nfeatures=500)
    kp1, desc1 = orb.detectAndCompute(img1, None)
    kp2, desc2 = orb.detectAndCompute(img2, None)
    print(f"Features: img1={len(kp1)}, img2={len(kp2)}")

    # Match with BFMatcher + Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(desc1, desc2, k=2)

    # Lowe's ratio test
    good = [m for m, n in matches if m.distance < 0.75 * n.distance]
    print(f"Good matches: {len(good)} / {len(matches)}")

    # RANSAC geometric verification
    if len(good) >= 4:
        src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        _, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        inliers = [m for m, f in zip(good, mask.ravel()) if f]
        print(f"Inliers after RANSAC: {len(inliers)}")
    else:
        inliers = good

    # Visualize
    result = cv2.drawMatches(img1, kp1, img2, kp2, inliers[:50], None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.figure(figsize=(14, 6))
    plt.imshow(result, cmap='gray')
    plt.title(f'ORB Matching: {len(inliers)} inlier matches')
    plt.axis('off')
    plt.savefig('orb_matching.png', dpi=150)
    plt.show()


demo_orb_matching()
```

### 8.2 Occupancy Grid from Depth

```python
"""
occupancy_grid.py
Build an occupancy grid from simulated depth data with log-odds updates.
"""

import numpy as np
import matplotlib.pyplot as plt

# Camera intrinsics
fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
img_w, img_h = 640, 480

# Grid config
GRID_SIZE = 200
CELL_SIZE = 0.05
L_OCC, L_FREE = 0.85, -0.40
L_MAX, L_MIN = 5.0, -5.0

grid = np.zeros((GRID_SIZE, GRID_SIZE))
robot_gx, robot_gy = GRID_SIZE // 2, GRID_SIZE // 4


def bresenham(x0, y0, x1, y1):
    """Bresenham line algorithm for ray casting."""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


def update_from_depth(depth, robot_pos, angle=0.0):
    """Project depth to occupancy grid with ray casting."""
    global grid
    cos_a, sin_a = np.cos(angle), np.sin(angle)

    for v in range(0, img_h, 4):
        for u in range(0, img_w, 4):
            z = depth[v, u]
            if z <= 0.1 or z > 4.0:
                continue

            x_cam = (u - cx) * z / fx
            x_w = cos_a * z - sin_a * x_cam
            y_w = sin_a * z + cos_a * x_cam

            hx = robot_pos[0] + int(x_w / CELL_SIZE)
            hy = robot_pos[1] + int(y_w / CELL_SIZE)

            if not (0 <= hx < GRID_SIZE and 0 <= hy < GRID_SIZE):
                continue

            # Ray cast
            for gx, gy in bresenham(robot_pos[0], robot_pos[1], hx, hy)[:-1]:
                if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                    grid[gy, gx] = np.clip(grid[gy, gx] + L_FREE, L_MIN, L_MAX)

            grid[hy, hx] = np.clip(grid[hy, hx] + L_OCC, L_MIN, L_MAX)


# Simulate depth with obstacles
depth = np.full((img_h, img_w), 2.0, dtype=np.float32)
depth[180:300, 250:390] = 0.8
depth[100:150, 450:550] = 1.2

update_from_depth(depth, (robot_gx, robot_gy))

prob = 1.0 / (1.0 + np.exp(-grid))

# Plot
fig, axes = plt.subplots(1, 2, figsize=(14, 6))
axes[0].imshow(depth, cmap='jet', vmin=0, vmax=4)
axes[0].set_title('Depth Image')
plt.colorbar(axes[0].images[0], ax=axes[0])

im = axes[1].imshow(prob, cmap='RdYlGn_r', vmin=0, vmax=1, origin='lower')
axes[1].plot(robot_gx, robot_gy, 'g^', markersize=15, label='Robot')
axes[1].set_title('Occupancy Grid (log-odds update)')
axes[1].legend()
plt.colorbar(im, ax=axes[1], label='P(occupied)')

plt.tight_layout()
plt.savefig('occupancy_grid.png', dpi=150)
plt.show()
```

### 8.3 RTAB-Map ROS2 Launch File

```python
"""
rtabmap_launch.py
ROS2 launch file for RTAB-Map SLAM on autonomous car.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path',
                              default_value='~/.ros/rtabmap.db'),

        # Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'Odom/Strategy': '0',
                'Vis/FeatureType': '6',
                'Vis/MaxFeatures': '200',
                'OdomF2M/MaxSize': '1000',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'database_path': LaunchConfiguration('database_path'),
                'Db/Sqlite3InMemory': 'false',
                'Mem/STMSize': '10',
                'Rtabmap/TimeThr': '700',
                'Kp/MaxFeatures': '200',
                'Vis/FeatureType': '6',
                'Vis/MaxFeatures': '200',
                'Odom/Strategy': '0',
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.1',
                'Rtabmap/LoopThr': '0.11',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '3.0',
                'Grid/FromDepth': 'true',
                'Grid/MaxGroundHeight': '0.02',
                'Grid/MaxObstacleHeight': '0.5',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/odom'),
                ('imu', '/imu/data'),
            ],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'rtabmap_config.rviz'],
        ),
    ])
```

### 8.4 ros2 bag Recording and Replay

```bash
# === Record sensor data for offline experimentation ===
ros2 bag record \
    /camera/color/image_raw \
    /camera/aligned_depth_to_color/image_raw \
    /camera/color/camera_info \
    /imu/data \
    /wheel_odom \
    --output slam_recording \
    --max-bag-duration 300

# Inspect the recording
ros2 bag info slam_recording

# === Replay for offline SLAM tuning ===

# Terminal 1: play bag with simulated clock
ros2 bag play slam_recording --clock --rate 0.5

# Terminal 2: launch RTAB-Map
ros2 launch my_car_slam rtabmap_launch.py use_sim_time:=true

# === Explore the SLAM database ===

# Open the database viewer (GUI)
rtabmap-databaseViewer ~/.ros/rtabmap.db

# In the viewer, examine:
# - Graph tab: nodes (green=WM, gray=LTM), edges (blue=odom, red=loop closure)
# - Click loop closure edges to see feature matches
# - Map tab: occupancy grid and 3D cloud
# - Statistics: processing time, WM/LTM sizes

# Export occupancy grid for Nav2
rtabmap-export --scan --scan_voxel 0.05 ~/.ros/rtabmap.db

# Export 3D point cloud
rtabmap-export --cloud --cloud_voxel 0.01 ~/.ros/rtabmap.db
```

### 8.5 Loop Closure Visualization

```python
"""
loop_closure_demo.py
Visualize the effect of loop closure on trajectory accuracy.
"""

import numpy as np
import matplotlib.pyplot as plt


def simulate_loop(n_poses=100, side=4.0, drift_std=0.02):
    """Simulate square path with odometry drift."""
    np.random.seed(42)
    true, per_side = [], n_poses // 4

    for i in range(n_poses):
        s = i // per_side
        p = (i % per_side) / per_side * side
        if s == 0:   true.append([p, 0])
        elif s == 1: true.append([side, p])
        elif s == 2: true.append([side - p, side])
        else:        true.append([0, side - p])

    true = np.array(true)
    drift = np.cumsum(np.random.randn(n_poses, 2) * drift_std, axis=0)
    noisy = true + drift
    return true, noisy


true_path, noisy_path = simulate_loop()
gap = noisy_path[-1] - noisy_path[0]
correction = np.outer(np.linspace(0, 1, len(noisy_path)), gap)
corrected = noisy_path - correction

fig, axes = plt.subplots(1, 3, figsize=(18, 6))
for ax, path, title, c in [
    (axes[0], true_path, 'Ground Truth', 'green'),
    (axes[1], noisy_path, f'Drift (gap: {np.linalg.norm(gap):.2f}m)', 'red'),
    (axes[2], corrected, 'After Loop Closure', 'blue'),
]:
    ax.plot(path[:, 0], path[:, 1], '-', color=c, linewidth=2)
    ax.plot(*path[0], 'ko', ms=10, label='Start')
    ax.plot(*path[-1], 'k^', ms=10, label='End')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

rmse_before = np.sqrt(np.mean((noisy_path - true_path)**2))
rmse_after = np.sqrt(np.mean((corrected - true_path)**2))
plt.suptitle(f'Loop Closure: RMSE {rmse_before:.3f}m -> {rmse_after:.3f}m '
             f'({(1-rmse_after/rmse_before)*100:.0f}% improvement)', fontsize=14)
plt.tight_layout()
plt.savefig('loop_closure_effect.png', dpi=150)
plt.show()
```

---

## Review

Today we covered the full SLAM pipeline from theory to practice.

| Topic | Key equation / concept |
|-------|----------------------|
| SLAM problem | \(p(\mathbf{x}_{0:t}, \mathbf{m} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})\) |
| ORB features | FAST corners + rBRIEF descriptors, Hamming matching |
| Lowe's ratio test | \(d_{\text{best}} / d_{\text{second}} < 0.75\) |
| Visual odometry | 3D-3D registration via SVD or ICP |
| Wheel odometry | \(\Delta s, \Delta\theta\) from encoder counts |
| Pose graph | Nodes = poses, edges = transforms + information matrices |
| Graph optimization | \(\min \sum e_{ij}^T \Omega_{ij} e_{ij}\) |
| Loop closure | BoW similarity + geometric verification |
| Occupancy grid | Log-odds update: \(l_t = l_{t-1} + l_{\text{sensor}}\) |
| RTAB-Map memory | STM → WM → LTM (disk), bounded by TimeThr |
| RPi 5 tuning | TimeThr=700, MaxFeatures=200, STMSize=10 |

### Connection to Previous Days

- **Day 6** (Hall Encoders): wheel odometry feeds SLAM as a motion prior.
- **Day 7** (IMU): gravity alignment and rotation prediction improve visual odometry.
- **Day 8** (Kalman Filter): sensor fusion principles apply to odometry combination.
- **Day 10** (Depth Camera): RGB-D images are the primary SLAM input.
- **Day 11** (Calibration): intrinsic matrix \(K\) is required for backprojection and feature matching.

### What Comes Next

In the next section of the course, we enter **ROS2 territory** (Days 13-16). The SLAM map we build today becomes the foundation for autonomous navigation: Nav2 uses the occupancy grid for costmaps, path planning, and obstacle avoidance. But first, we need to understand the ROS2 middleware that connects all these components together.
