---
title: "Day 12 — SLAM: Simultaneous Localization and Mapping"
date: 2026-03-05
description: "The SLAM problem, visual odometry, feature extraction, pose graph optimization, loop closure, occupancy grids, and RTAB-Map on Raspberry Pi 5"
categories: ["Autonomous Driving"]
tags: ["SLAM", "RTAB-Map", "Visual Odometry", "Loop Closure", "Occupancy Grid"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 12
draft: false
---

{{< katex >}}

## What You'll Learn

We can now measure distances (Day 10), calibrate cameras (Day 11), filter noise (Day 8), and control motors (Day 9). But our car still doesn't know **where it is** or **what the environment looks like**. That's the SLAM problem.

By the end of this post you will:

1. Understand the chicken-and-egg nature of SLAM: you need a map to localize, but you need your location to build a map.
2. Know the **front-end** (feature extraction, odometry) and **back-end** (optimization, loop closure) architecture.
3. Understand **occupancy grid maps** and how cells store probabilistic obstacle information.
4. Learn the **RTAB-Map** architecture and why it works well on resource-constrained devices.
5. Configure RTAB-Map for Raspberry Pi 5 with practical tuning parameters.
6. Run SLAM with depth camera, IMU, and wheel odometry.

---

## 1. The SLAM Problem

### Why Is This Hard?

Imagine you wake up blindfolded in an unknown building. You can feel walls, hear echoes, and count your steps. You need to simultaneously:

- Figure out where you are (**Localization**)
- Build a mental map of the building (**Mapping**)

But localization requires a map, and mapping requires knowing your location. This circular dependency is the core challenge of SLAM.

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

### SLAM Architecture: Front-End + Back-End

| Component | Role | Examples |
|-----------|------|---------|
| **Front-end** | Extracts information from raw sensor data | Feature detection, odometry estimation |
| **Back-end** | Optimizes the global map consistency | Pose graph optimization, loop closure |

---

## 2. Front-End: Sensing and Odometry

### 2.1 Feature Extraction

To recognize places and track motion, we extract **visual features** — distinctive points in images that can be reliably detected and matched across frames.

#### ORB Features (Oriented FAST and Rotated BRIEF)

ORB is the go-to feature detector for real-time SLAM because it's fast and free (no patents):

```
Step 1: FAST corner detection
  Find pixels where the surrounding ring of 16 pixels
  has a contiguous arc of N pixels all brighter or darker
  than the center pixel + threshold

    · · ● ● ● · ·
    · ●         ● ·
    ●             ●
    ●      P      ●    P = center pixel
    ●             ●    ● = ring pixels to check
    · ●         ● ·
    · · ● ● ● · ·

Step 2: Orientation assignment (Harris corner measure + intensity centroid)
  → Rotation invariant

Step 3: BRIEF descriptor (256-bit binary string)
  Compare pixel pairs in a learned pattern around the keypoint
  → Fast matching via Hamming distance (XOR + popcount)
```

**ORB extracts ~1000 features per frame in < 10ms on RPi 5.**

#### Feature Matching

Given features from frame A and frame B, find correspondences:

$$\text{match}(f_A, f_B) = \arg\min_{f_B} \; d_{Hamming}(f_A, f_B)$$

With **Lowe's ratio test**: accept a match only if the best match is significantly better than the second-best:

$$\frac{d_{best}}{d_{second}} < 0.75$$

This eliminates most false matches.

### 2.2 Visual Odometry

Visual odometry estimates camera motion between consecutive frames using matched features:

```
  Frame k                          Frame k+1
  ┌──────────────────┐            ┌──────────────────┐
  │  ●₁   ●₂        │            │     ●₁'  ●₂'     │
  │        ●₃        │   ──►     │          ●₃'     │
  │  ●₄        ●₅    │  motion   │  ●₄'        ●₅'  │
  └──────────────────┘            └──────────────────┘

  Match features: ●ᵢ ↔ ●ᵢ'
  Solve for R, t between frames
```

For a calibrated camera, given N matched point pairs, solve:

$$s_i \begin{pmatrix} u_i' \\ v_i' \\ 1 \end{pmatrix} \times \left[ \mathbf{K}^{-T} \mathbf{E} \mathbf{K}^{-1} \begin{pmatrix} u_i \\ v_i \\ 1 \end{pmatrix} \right] = 0$$

Where \(\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}\) is the **essential matrix**. With a depth camera, we can directly solve for R and t using 3D-to-3D point correspondences (PnP or ICP), which is more robust.

### 2.3 Wheel Odometry

Hall sensor encoders from Day 6 provide another motion estimate:

$$\Delta x = \frac{\Delta s_L + \Delta s_R}{2} \cos\theta, \quad \Delta y = \frac{\Delta s_L + \Delta s_R}{2} \sin\theta$$

$$\Delta \theta = \frac{\Delta s_R - \Delta s_L}{b}$$

Where \(\Delta s_L, \Delta s_R\) are left/right wheel distances and \(b\) is the wheel base.

**Wheel odometry drifts** due to wheel slip, uneven surfaces, and measurement noise. Visual odometry drifts too. Combining them (sensor fusion) reduces total drift — but drift still accumulates over time. That's why we need **loop closure**.

---

## 3. Back-End: Optimization

### 3.1 Pose Graph

As the robot moves, we build a **graph** of poses (positions + orientations):

```
  Pose Graph:

  x₀ ──── x₁ ──── x₂ ──── x₃ ──── x₄
                                      │
                                      │ loop closure!
                                      │
  x₉ ──── x₈ ──── x₇ ──── x₆ ──── x₅

  Nodes: Robot poses (x, y, θ) at each time step
  Edges: Relative transforms from odometry
  Loop closure edge: Connects x₄ to x₅ when
    the robot recognizes it's back at the same place
```

Each edge encodes a **relative transform** \(\mathbf{z}_{ij}\) between poses \(x_i\) and \(x_j\), with associated uncertainty \(\Omega_{ij}\) (information matrix).

### 3.2 Graph Optimization

The back-end minimizes the total error across all edges:

$$\mathbf{x}^* = \arg\min_{\mathbf{x}} \sum_{(i,j) \in \mathcal{E}} \mathbf{e}_{ij}^T \, \Omega_{ij} \, \mathbf{e}_{ij}$$

Where the error for each edge is:

$$\mathbf{e}_{ij} = \mathbf{z}_{ij} - (x_j \ominus x_i)$$

The \(\ominus\) operator computes the relative transform between two poses. This is a **nonlinear least-squares problem** solved by libraries like **g2o** or **GTSAM** using Gauss-Newton or Levenberg-Marquardt.

### 3.3 Loop Closure Detection

The magic of SLAM: when the robot returns to a previously visited place, the loop closure **corrects accumulated drift** in one shot.

```
  Before Loop Closure          After Loop Closure

  Start ●───●                  Start ●───●
         \   \                        │    \
          ●   ●                       ●     ●
           \   \                       \   │
            ●   ●                       ● ─●
             \   \   ← drift!           │  │
              ●   ●                     ●  ●
               \   \                     \ │
                ●───● ← gap!             ●─●  ← closed!
```

**How it works in RTAB-Map:**

1. Extract visual features (ORB) from current frame
2. Convert to a **bag-of-words** (BoW) representation
3. Compare BoW vector with all previously stored frames
4. If similarity score exceeds threshold → potential loop closure
5. Verify with geometric consistency (feature matching + RANSAC)
6. Add loop closure edge to pose graph
7. Re-optimize the entire graph

---

## 4. Occupancy Grid Maps

### The Map Representation

An occupancy grid divides the environment into a grid of cells. Each cell stores the **probability of being occupied**:

$$P(\text{occupied} \mid z_{1:t})$$

```
  Occupancy Grid (top-down view):

  ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■    ■ = Occupied (wall)
  ■ □ □ □ □ □ □ □ □ □ □ ■    □ = Free
  ■ □ □ □ □ □ □ □ □ □ □ ■    ░ = Unknown
  ■ □ □ ■ ■ □ □ □ □ □ □ ■
  ■ □ □ ■ ■ □ □ ░ ░ ░ ░ ■
  ■ □ □ □ □ □ □ ░ ░ ░ ░ ■    Robot ★ is here
  ■ □ □ □ ★ □ □ ░ ░ ░ ░ ■    Right side not yet explored
  ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■ ■

  Cell resolution: 5 cm × 5 cm typical
```

### Log-Odds Update

Instead of working with probabilities directly (which requires normalization), we use **log-odds**:

$$l = \log \frac{P(\text{occupied})}{1 - P(\text{occupied})}$$

The update rule becomes a simple addition:

$$l_{t} = l_{t-1} + l_{sensor} - l_0$$

Where:
- \(l_{t-1}\) = previous log-odds for this cell
- \(l_{sensor}\) = log-odds from the current sensor reading
- \(l_0 = \log \frac{P_0}{1 - P_0}\) = prior (usually \(P_0 = 0.5\), so \(l_0 = 0\))

**Advantages of log-odds:**
- No multiplication of small probabilities (numerically stable)
- Update is a simple addition
- Easy to clamp to prevent over-confidence

### From Depth to Occupancy

Given a depth camera measurement:

1. Project each depth pixel to a 3D point (using K from Day 11)
2. Transform to world coordinates (using the current pose estimate)
3. Ray-cast from the robot to the point:
   - All cells along the ray → **free** (increase free log-odds)
   - The cell at the endpoint → **occupied** (increase occupied log-odds)

```
  Robot ★ ─ ─ ─ ─ ─ ─ ─ ─ ● Wall
         free  free  free  occupied

  Cells along ray get P(free) increased
  Cell at endpoint gets P(occupied) increased
```

---

## 5. RTAB-Map Architecture

**RTAB-Map** (Real-Time Appearance-Based Mapping) is our SLAM system of choice because:
- Works with depth cameras (RGB-D SLAM)
- Handles loop closure with visual bag-of-words
- Manages memory to run on limited hardware (RPi 5)
- ROS2 integration out of the box

### Core Architecture

```
                    ┌─────────────────────────────┐
                    │         RTAB-Map Core        │
                    │                              │
  RGB Image ──────►│  ┌────────┐   ┌──────────┐  │
                    │  │ Visual │   │  Loop    │  │
  Depth Image ────►│  │Odometry│   │ Closure  │  │
                    │  └───┬────┘   └────┬─────┘  │
  Wheel Odom ─────►│      │             │         │
                    │      ▼             ▼         │
  IMU ────────────►│  ┌────────────────────────┐  │──► Occupancy Grid
                    │  │   Pose Graph + Map     │  │──► 3D Point Cloud
                    │  │   Optimization         │  │──► TF (odom→map)
                    │  └────────────────────────┘  │
                    │                              │
                    │  ┌────────────────────────┐  │
                    │  │   Memory Management    │  │
                    │  │   WM ↔ STM ↔ LTM      │  │
                    │  └────────────────────────┘  │
                    └─────────────────────────────┘
```

### Memory Management: WM, STM, LTM

RTAB-Map's key innovation for embedded systems: **three-tier memory management**.

| Memory Tier | Name | Role |
|-------------|------|------|
| **STM** | Short-Term Memory | Recent nodes, always in RAM |
| **WM** | Working Memory | Active nodes for loop closure search |
| **LTM** | Long-Term Memory | Archived nodes on disk (SQLite DB) |

```
  New node ──► STM (last N nodes)
                │
                ▼ (when STM full)
               WM (actively searched for loop closure)
                │
                ▼ (when WM exceeds time budget)
               LTM (on disk, retrieved only if needed)
```

The **TimeThr** parameter controls how much time RTAB-Map spends per iteration. If processing takes too long, less-recently-used nodes move from WM to LTM. When a potential loop closure is detected with an LTM node, it's retrieved back to WM.

This is why RTAB-Map can run indefinitely without running out of RAM — crucial for Raspberry Pi 5 with only 4-8 GB.

### Sensor Roles in Our Car

| Sensor | RTAB-Map Role | Why |
|--------|---------------|-----|
| Depth Camera | Primary visual odometry + 3D mapping | Rich visual features + depth |
| IMU | Gravity alignment + rotation prediction | Fast updates between frames (Day 7) |
| Wheel Odometry | Motion prior between frames | Helps when visual features are poor |
| 1D LiDAR | Obstacle layer in costmap | Simple, reliable forward distance |

---

## 6. RTAB-Map Tuning for RPi 5

### Critical Parameters

```yaml
# RTAB-Map parameters for Raspberry Pi 5
rtabmap:
  ros__parameters:
    # Memory management
    Mem/STMSize: "10"              # Short-term memory size (number of nodes)
    Rtabmap/TimeThr: "700"         # Max processing time per iteration (ms)
                                    # RPi 5: 500-700ms is realistic

    # Feature detection
    Kp/MaxFeatures: "200"          # Max keypoints per frame
                                    # Lower = faster, but less accurate matching
                                    # Desktop: 500, RPi 5: 200
    Vis/FeatureType: "6"           # 6 = ORB (fast, free)
    Vis/MaxFeatures: "200"         # Visual odometry features

    # Visual odometry
    Odom/Strategy: "0"             # 0 = Frame-to-Map (more accurate)
                                    # 1 = Frame-to-Frame (faster)
    RGBD/LinearUpdate: "0.1"       # Min distance (m) to create new node
    RGBD/AngularUpdate: "0.1"      # Min rotation (rad) to create new node

    # Loop closure
    Rtabmap/LoopThr: "0.11"        # Loop closure detection threshold
    RGBD/OptimizeFromGraphEnd: "false"  # Optimize from first node (more stable)

    # Map
    Grid/CellSize: "0.05"          # 5cm grid cells
    Grid/RangeMax: "3.0"           # Max range for grid updates (m)
    Grid/FromDepth: "true"         # Build occupancy grid from depth camera
```

### Performance Expectations on RPi 5

| Metric | Value |
|--------|-------|
| Processing time per frame | 400-700 ms |
| Map update rate | ~2-3 Hz |
| Max features tracked | ~200 per frame |
| Occupancy grid resolution | 5 cm |
| Typical map size (10 min run) | 50-100 MB (LTM on disk) |
| RAM usage | 1-2 GB |

### Common Issues and Fixes

| Problem | Cause | Solution |
|---------|-------|----------|
| "Not enough features" | Textureless walls | Add more texture, reduce Kp/MaxFeatures threshold |
| Drift in long corridors | Few loop closure opportunities | Use wheel odometry as prior |
| High CPU usage | Too many features | Reduce Kp/MaxFeatures, increase RGBD/LinearUpdate |
| Map gaps | Moving too fast | Slow down, reduce RGBD/LinearUpdate |
| Wrong loop closure | Similar-looking places | Increase Rtabmap/LoopThr |

---

## 7. Hands-On Lab

### Lab 1: ORB Feature Detection and Matching

```python
#!/usr/bin/env python3
"""Detect and match ORB features between two images."""

import cv2
import numpy as np

# Load two images (consecutive frames or overlapping views)
img1 = cv2.imread('frame_001.jpg', cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('frame_002.jpg', cv2.IMREAD_GRAYSCALE)

# Create ORB detector
orb = cv2.ORB_create(nfeatures=500)

# Detect keypoints and compute descriptors
kp1, desc1 = orb.detectAndCompute(img1, None)
kp2, desc2 = orb.detectAndCompute(img2, None)
print(f"Features: img1={len(kp1)}, img2={len(kp2)}")

# Match using BFMatcher with Hamming distance
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
matches = bf.knnMatch(desc1, desc2, k=2)

# Lowe's ratio test
good_matches = []
for m, n in matches:
    if m.distance < 0.75 * n.distance:
        good_matches.append(m)

print(f"Good matches: {len(good_matches)}")

# Draw matches
result = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None,
                          flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

cv2.imshow('ORB Matches', result)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

### Lab 2: Simple Occupancy Grid from Depth

```python
#!/usr/bin/env python3
"""Build a simple occupancy grid from depth camera data."""

import numpy as np
import matplotlib.pyplot as plt

# Simulated depth camera parameters
fx, fy = 600, 600  # focal length in pixels
cx, cy = 320, 240  # principal point
width, height = 640, 480

# Grid parameters
grid_size = 200      # 200x200 cells
cell_size = 0.05     # 5 cm per cell
grid = np.zeros((grid_size, grid_size))  # log-odds
robot_x, robot_y = grid_size // 2, grid_size // 4  # robot position in grid

# Simulated depth image (replace with real depth camera data)
np.random.seed(42)
depth = np.full((height, width), 2.0)  # 2m flat wall
# Add a closer obstacle in the center
depth[200:280, 280:360] = 0.8  # 80cm obstacle

def depth_to_grid(depth_image, robot_pos, robot_angle=0):
    """Project depth pixels to occupancy grid."""
    global grid

    for v in range(0, height, 4):  # subsample for speed
        for u in range(0, width, 4):
            z = depth_image[v, u]
            if z <= 0 or z > 4.0:
                continue

            # Back-project to 3D (camera frame)
            x_cam = (u - cx) * z / fx
            y_cam = (v - cy) * z / fy

            # Transform to world frame (simplified: camera looking forward)
            # x_cam → lateral, y_cam → vertical (ignore), z → forward
            x_world = x_cam
            z_world = z

            # Rotate by robot angle
            cos_a, sin_a = np.cos(robot_angle), np.sin(robot_angle)
            gx = robot_pos[0] + int((cos_a * z_world - sin_a * x_world) / cell_size)
            gy = robot_pos[1] + int((sin_a * z_world + cos_a * x_world) / cell_size)

            # Bresenham ray-cast (simplified: just mark endpoint)
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] += 0.4  # occupied log-odds

            # Mark cells along ray as free (simplified)
            steps = int(z / cell_size)
            for s in range(1, steps):
                frac = s / steps
                fx_r = robot_pos[0] + int(frac * (gx - robot_pos[0]))
                fy_r = robot_pos[1] + int(frac * (gy - robot_pos[1]))
                if 0 <= fx_r < grid_size and 0 <= fy_r < grid_size:
                    grid[fy_r, fx_r] -= 0.1  # free log-odds

# Build grid
depth_to_grid(depth, (robot_x, robot_y))

# Convert log-odds to probability for visualization
prob_grid = 1.0 / (1.0 + np.exp(-grid))

# Plot
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

axes[0].imshow(depth, cmap='jet', vmin=0, vmax=4)
axes[0].set_title('Depth Image')
axes[0].set_xlabel('u (pixels)')
axes[0].set_ylabel('v (pixels)')

im = axes[1].imshow(prob_grid, cmap='RdYlGn_r', vmin=0, vmax=1, origin='lower')
axes[1].plot(robot_x, robot_y, 'b^', markersize=15, label='Robot')
axes[1].set_title('Occupancy Grid')
axes[1].set_xlabel('x (cells, 5cm each)')
axes[1].set_ylabel('y (cells, 5cm each)')
axes[1].legend()
plt.colorbar(im, ax=axes[1], label='P(occupied)')

plt.tight_layout()
plt.savefig('occupancy_grid.png', dpi=150)
plt.show()
```

### Lab 3: RTAB-Map ROS2 Launch Analysis

```python
#!/usr/bin/env python3
"""
RTAB-Map ROS2 launch file template for HiWonder car.
Run with: ros2 launch <package_name> rtabmap_launch.py
"""

# This is a Python-based ROS2 launch file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),

        # RTAB-Map node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Input topics
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom': True,

                # Frame IDs
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',

                # Database
                'database_path': LaunchConfiguration('database_path'),
                'Db/Sqlite3InMemory': 'false',  # Save to disk (important for RPi 5)

                # Memory management (RPi 5 optimized)
                'Mem/STMSize': '10',
                'Rtabmap/TimeThr': '700',

                # Features
                'Kp/MaxFeatures': '200',
                'Vis/FeatureType': '6',  # ORB
                'Vis/MaxFeatures': '200',

                # Visual odometry
                'Odom/Strategy': '0',  # Frame-to-Map
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.1',

                # Grid map
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '3.0',
                'Grid/FromDepth': 'true',
                'Grid/MaxGroundHeight': '0.02',
                'Grid/MaxObstacleHeight': '0.5',

                # Loop closure
                'Rtabmap/LoopThr': '0.11',
                'RGBD/OptimizeFromGraphEnd': 'false',

                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/wheel_odom'),
                ('imu', '/imu/data'),
            ],
        ),

        # Visual Odometry node (separate for better control)
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
    ])
```

### Lab 4: Visualize RTAB-Map Database

```bash
# After running RTAB-Map, explore the generated database:

# 1. View the database (GUI tool)
rtabmap-databaseViewer ~/.ros/rtabmap.db

# 2. Export the occupancy grid as an image
rtabmap-export --images ~/.ros/rtabmap.db

# 3. Key things to observe in the viewer:
#    - Working Memory (WM) nodes: highlighted in the graph
#    - Long-Term Memory (LTM) nodes: grayed out
#    - Loop closure links: shown as colored edges
#    - Feature matches: visible when clicking on a link
#    - Occupancy grid: viewable in the map tab

# 4. ROS2 bag recording for offline analysis
ros2 bag record /camera/color/image_raw \
                 /camera/aligned_depth_to_color/image_raw \
                 /camera/color/camera_info \
                 /imu/data \
                 /wheel_odom \
                 -o slam_recording

# 5. Replay and re-run SLAM with different parameters
ros2 bag play slam_recording --clock
# In another terminal:
ros2 launch <package> rtabmap_launch.py use_sim_time:=true
```

### Lab 5: Loop Closure Impact Visualization

```python
#!/usr/bin/env python3
"""Visualize the effect of loop closure on pose graph."""

import numpy as np
import matplotlib.pyplot as plt

# Simulate a square path with drift (no loop closure)
n_poses = 100
true_path = []
noisy_path = []

# True path: 4m × 4m square
side = 4.0
poses_per_side = n_poses // 4

for i in range(n_poses):
    side_idx = i // poses_per_side
    progress = (i % poses_per_side) / poses_per_side * side

    if side_idx == 0:    # right
        true_path.append((progress, 0))
    elif side_idx == 1:  # up
        true_path.append((side, progress))
    elif side_idx == 2:  # left
        true_path.append((side - progress, side))
    else:                # down
        true_path.append((0, side - progress))

true_path = np.array(true_path)

# Add cumulative drift (grows over time)
np.random.seed(42)
drift = np.cumsum(np.random.randn(n_poses, 2) * 0.02, axis=0)
noisy_path = true_path + drift

# Simulate loop closure correction
# When robot returns to start, detect the gap and distribute correction
gap = noisy_path[-1] - noisy_path[0]  # should be zero for closed loop
correction = np.outer(np.linspace(0, 1, n_poses), gap)
corrected_path = noisy_path - correction

# Plot
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

for ax, path, title, color in [
    (axes[0], true_path, 'Ground Truth', 'green'),
    (axes[1], noisy_path, 'With Drift (No Loop Closure)', 'red'),
    (axes[2], corrected_path, 'After Loop Closure', 'blue'),
]:
    ax.plot(path[:, 0], path[:, 1], color=color, linestyle='-', linewidth=2)
    ax.plot(path[0, 0], path[0, 1], 'ko', markersize=10, label='Start')
    ax.plot(path[-1, 0], path[-1, 1], 'k^', markersize=10, label='End')
    ax.set_title(title, fontsize=14)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

# Annotate drift
axes[1].annotate(f'Gap: ({gap[0]:.2f}, {gap[1]:.2f})m',
                  xy=(noisy_path[-1, 0], noisy_path[-1, 1]),
                  fontsize=12, color='red',
                  xytext=(1, -1), textcoords='offset fontsize',
                  arrowprops=dict(arrowstyle='->', color='red'))

plt.suptitle('Loop Closure Corrects Accumulated Drift', fontsize=16)
plt.tight_layout()
plt.savefig('loop_closure_effect.png', dpi=150)
plt.show()
```

---

## 8. Review

### Key Takeaways

1. **SLAM** solves the chicken-and-egg problem: localize and map simultaneously.
2. **Front-end**: ORB features + visual/wheel odometry estimate frame-to-frame motion.
3. **Back-end**: pose graph optimization + loop closure correct accumulated drift.
4. **Occupancy grid**: probabilistic map where each cell stores P(occupied) via log-odds updates.
5. **RTAB-Map**: memory-managed visual SLAM that runs on RPi 5 by moving old nodes to disk (LTM).
6. **Key tuning**: `Kp/MaxFeatures=200`, `TimeThr=700ms`, `STMSize=10` for RPi 5.

### Connection Map

```
Day 6 (Encoders) ──► Wheel odometry feeds into SLAM
Day 7 (IMU)      ──► Gravity alignment, rotation prediction
Day 8 (Kalman)   ──► Sensor fusion for odometry
Day 11 (Calib)   ──► Camera matrix K for feature projection
Day 15 (Nav2)    ──► SLAM map feeds into Nav2 costmap for path planning
```

### Looking Ahead

Next week we enter **ROS2 territory** (Days 13-16). The SLAM map we build here becomes the foundation for autonomous navigation — Nav2 uses the occupancy grid for path planning and obstacle avoidance. But first, we need to understand the ROS2 middleware that connects all these components together.
