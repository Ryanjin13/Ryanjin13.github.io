---
title: "Day 18 — Lane Detection ROS2 Integration, Sensor Fusion, and Safety Design"
date: 2026-03-06
description: "Packaging vision pipeline as ROS2 node, multi-sensor fusion architecture, watchdog timers, emergency stop logic, and fail-safe state machines"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "ROS2", "Safety", "Watchdog", "State Machine", "Lane Detection"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 18
draft: false
---

{{< katex >}}

## What You'll Learn

Yesterday you built a complete lane detection pipeline — from raw pixels to a cross-track error in meters. Today you take that algorithm and make it **production-grade** by addressing three critical engineering challenges:

1. **ROS2 Integration** — Package the vision pipeline as a proper ROS2 node that subscribes to camera images and publishes steering commands.
2. **Sensor Fusion** — Combine camera-based lane detection with 1D LiDAR obstacle distance to make unified driving decisions.
3. **Safety Design** — Build watchdog timers, emergency stop logic, and a fail-safe state machine so the car degrades gracefully when things go wrong.

By the end of today you will be able to:

- Write a ROS2 Python node that processes `sensor_msgs/Image` and publishes `Float32` steering error.
- Implement a confidence-weighted fusion of camera and LiDAR data.
- Design and code a state machine with NORMAL, DEGRADED, EMERGENCY_STOP, and SAFE states.
- Record and replay `ros2 bag` data for post-run failure analysis.
- Explain why every autonomous system needs a watchdog and how to implement one.

This is the day where your project stops being a demo and starts being a **system**.

---

## 1. Lane Detection Failure — What Could Go Wrong?

Before we write any ROS2 code, we need to think about **failure modes**. An autonomous car that works 99% of the time and crashes 1% of the time is not a product — it is a liability.

### 1.1 Common Lane Detection Failures

| Failure Mode | Cause | Symptom |
|-------------|-------|---------|
| No lanes detected | Worn paint, shadows, glare, rain | `left_fit` or `right_fit` is `None` |
| False lane detection | Tire marks, road patches, guard rails | CTE jumps wildly between frames |
| Partial detection | Only one lane visible (merge, intersection) | One fit valid, one `None` |
| Latency spike | CPU overloaded, garbage collection | Frame processing > 100 ms |
| Camera failure | USB disconnect, lens obstruction | No frames received |

### 1.2 Fallback Strategy Design

A robust system needs a **hierarchy of fallbacks**:

```
Level 0: Both lanes detected, confidence high
         → Use computed CTE for steering

Level 1: Only one lane detected
         → Estimate other lane (assume fixed lane width)
         → Reduce speed by 30%

Level 2: No lanes detected, but previous fit available (< 500 ms old)
         → Use previous CTE (stale data)
         → Reduce speed by 50%

Level 3: No lanes for > 500 ms
         → Slow to crawl speed
         → Activate emergency search mode (widen HSV thresholds)

Level 4: No lanes for > 2000 ms OR camera failure
         → Emergency stop
```

The key insight: **never make a binary "works / doesn't work" decision**. Always provide graceful degradation with increasing conservatism.

### 1.3 Detection Confidence Score

We can quantify detection quality with a simple confidence metric:

$$
\text{confidence} = \min\left(\frac{N_{\text{pixels}}}{N_{\text{threshold}}}, 1.0\right) \times \left(1 - \frac{|\text{CTE}_t - \text{CTE}_{t-1}|}{\Delta_{\text{max}}}\right)
$$

where:
- \(N_{\text{pixels}}\) = number of lane pixels found by sliding window
- \(N_{\text{threshold}}\) = expected minimum pixel count (e.g., 1000)
- \(\text{CTE}_t - \text{CTE}_{t-1}\) = CTE change between frames
- \(\Delta_{\text{max}}\) = maximum plausible CTE change per frame (e.g., 0.05 m)

The first term rewards having enough lane pixels. The second term penalizes sudden jumps (which indicate false detections). Confidence ranges from 0 to 1.

---

## 2. ROS2 Lane Detection Node

### 2.1 Node Architecture

```
┌──────────────┐    sensor_msgs/Image     ┌────────────────────┐
│  USB Camera   │ ──────────────────────► │  lane_detection_node │
│  (v4l2_camera)│                          │                      │
└──────────────┘                          │  - undistort          │
                                           │  - color mask         │
                                           │  - BEV transform      │
                                           │  - sliding window     │
                                           │  - polynomial fit     │
                                           │  - CTE computation    │
                                           └───────┬──────────────┘
                                                   │
                                    ┌──────────────┼───────────────┐
                                    │              │               │
                              std_msgs/      sensor_msgs/    std_msgs/
                              Float32        Image            Float32
                              /lane/cte      /lane/debug     /lane/confidence
```

### 2.2 The cv_bridge Problem

ROS2 uses `sensor_msgs/Image` messages. OpenCV uses NumPy arrays. The `cv_bridge` package converts between them:

```python
from cv_bridge import CvBridge

bridge = CvBridge()

# ROS Image → OpenCV
cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

# OpenCV → ROS Image
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
```

### 2.3 Complete ROS2 Lane Detection Node

```python
#!/usr/bin/env python3
"""
lane_detection_node.py
Day 18 — Lane Detection ROS2 Node

Subscribes to: /camera/image_raw (sensor_msgs/Image)
Publishes:
  /lane/cte        (std_msgs/Float32) — cross-track error in meters
  /lane/confidence (std_msgs/Float32) — detection confidence [0, 1]
  /lane/debug      (sensor_msgs/Image) — annotated debug image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import numpy as np
import pickle
import time


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node")

        # ── Parameters ──────────────────────────────────
        self.declare_parameter("calibration_file", "calibration.pkl")
        self.declare_parameter("yellow_h_low", 15)
        self.declare_parameter("yellow_h_high", 35)
        self.declare_parameter("yellow_s_low", 80)
        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("n_windows", 9)
        self.declare_parameter("window_margin", 80)
        self.declare_parameter("window_minpix", 50)
        self.declare_parameter("lane_width_meters", 0.30)
        self.declare_parameter("confidence_pixel_threshold", 1000)
        self.declare_parameter("max_cte_jump", 0.05)

        # ── Load calibration ────────────────────────────
        calib_path = self.get_parameter("calibration_file").value
        self.K, self.dist = self._load_calibration(calib_path)

        # ── State ───────────────────────────────────────
        self.bridge = CvBridge()
        self.prev_cte = 0.0
        self.prev_left_fit = None
        self.prev_right_fit = None
        self.last_detection_time = time.time()
        self.M = None
        self.M_inv = None
        self.frame_count = 0

        # ── Publishers ──────────────────────────────────
        self.pub_cte = self.create_publisher(Float32, "/lane/cte", 10)
        self.pub_conf = self.create_publisher(Float32, "/lane/confidence", 10)
        self.pub_debug = self.create_publisher(Image, "/lane/debug", 1)

        # ── Subscriber ──────────────────────────────────
        self.sub_image = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.get_logger().info("Lane detection node started.")

    def _load_calibration(self, path):
        try:
            with open(path, "rb") as f:
                calib = pickle.load(f)
            self.get_logger().info(f"Loaded calibration from {path}")
            return calib["camera_matrix"], calib["dist_coeffs"]
        except FileNotFoundError:
            self.get_logger().warn("No calibration file. Skipping undistortion.")
            return None, None

    def _init_bev(self, h, w):
        """Initialize BEV transform matrices on first frame."""
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
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.M_inv = cv2.getPerspectiveTransform(dst, src)

    def _color_mask(self, frame):
        """HSV-based lane color mask."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        yh_low = self.get_parameter("yellow_h_low").value
        yh_high = self.get_parameter("yellow_h_high").value
        ys_low = self.get_parameter("yellow_s_low").value

        yellow = cv2.inRange(hsv,
                             np.array([yh_low, ys_low, 80]),
                             np.array([yh_high, 255, 255]))
        white = cv2.inRange(hsv,
                            np.array([0, 0, 200]),
                            np.array([179, 40, 255]))

        combined = cv2.bitwise_or(yellow, white)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)

        return combined

    def _sliding_window(self, binary_bev):
        """Sliding window lane search. Returns fits and pixel counts."""
        h, w = binary_bev.shape
        n_win = self.get_parameter("n_windows").value
        margin = self.get_parameter("window_margin").value
        minpix = self.get_parameter("window_minpix").value

        # Histogram
        bottom_half = binary_bev[h // 2:, :]
        histogram = np.sum(bottom_half, axis=0)
        mid = w // 2
        left_start = np.argmax(histogram[:mid])
        right_start = np.argmax(histogram[mid:]) + mid

        window_h = h // n_win
        nonzero_y, nonzero_x = binary_bev.nonzero()

        lx = left_start
        rx = right_start
        left_inds, right_inds = [], []

        for i in range(n_win):
            y_lo = h - (i + 1) * window_h
            y_hi = h - i * window_h

            good_l = (
                (nonzero_y >= y_lo) & (nonzero_y < y_hi) &
                (nonzero_x >= lx - margin) & (nonzero_x < lx + margin)
            ).nonzero()[0]

            good_r = (
                (nonzero_y >= y_lo) & (nonzero_y < y_hi) &
                (nonzero_x >= rx - margin) & (nonzero_x < rx + margin)
            ).nonzero()[0]

            left_inds.append(good_l)
            right_inds.append(good_r)

            if len(good_l) > minpix:
                lx = int(np.mean(nonzero_x[good_l]))
            if len(good_r) > minpix:
                rx = int(np.mean(nonzero_x[good_r]))

        left_inds = np.concatenate(left_inds)
        right_inds = np.concatenate(right_inds)

        n_left = len(left_inds)
        n_right = len(right_inds)

        left_fit = None
        right_fit = None

        if n_left > 0:
            left_fit = np.polyfit(nonzero_y[left_inds], nonzero_x[left_inds], 2)
        if n_right > 0:
            right_fit = np.polyfit(nonzero_y[right_inds], nonzero_x[right_inds], 2)

        return left_fit, right_fit, n_left, n_right

    def _compute_cte(self, left_fit, right_fit, h, w):
        """Compute CTE in meters."""
        lane_w = self.get_parameter("lane_width_meters").value
        y_eval = h - 1

        lx = np.polyval(left_fit, y_eval)
        rx = np.polyval(right_fit, y_eval)

        center_px = (lx + rx) / 2.0
        image_cx = w / 2.0
        cte_px = center_px - image_cx

        lane_px = rx - lx
        m_per_px = lane_w / lane_px if lane_px > 0 else 1.0
        return cte_px * m_per_px

    def _compute_confidence(self, n_left, n_right, cte):
        """Compute detection confidence [0, 1]."""
        pix_thresh = self.get_parameter("confidence_pixel_threshold").value
        max_jump = self.get_parameter("max_cte_jump").value

        pixel_score = min((n_left + n_right) / (2 * pix_thresh), 1.0)
        jump = abs(cte - self.prev_cte)
        stability_score = max(1.0 - jump / max_jump, 0.0)

        return pixel_score * stability_score

    def image_callback(self, msg):
        """Main processing callback."""
        t_start = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]

        # Init BEV on first frame
        if self.M is None:
            self._init_bev(h, w)

        # Undistort
        if self.K is not None:
            frame = cv2.undistort(frame, self.K, self.dist)

        # Pipeline
        mask = self._color_mask(frame)
        bev = cv2.warpPerspective(mask, self.M, (w, h))
        left_fit, right_fit, n_left, n_right = self._sliding_window(bev)

        # ── Decision logic with fallbacks ───────────────
        cte = None
        confidence = 0.0

        if left_fit is not None and right_fit is not None:
            # Level 0: Both lanes detected
            cte = self._compute_cte(left_fit, right_fit, h, w)
            confidence = self._compute_confidence(n_left, n_right, cte)
            self.prev_left_fit = left_fit
            self.prev_right_fit = right_fit
            self.last_detection_time = time.time()

        elif left_fit is not None or right_fit is not None:
            # Level 1: One lane detected — estimate other
            # Use previous fit for missing lane if available
            if left_fit is None and self.prev_left_fit is not None:
                left_fit = self.prev_left_fit
            if right_fit is None and self.prev_right_fit is not None:
                right_fit = self.prev_right_fit

            if left_fit is not None and right_fit is not None:
                cte = self._compute_cte(left_fit, right_fit, h, w)
                confidence = 0.5 * self._compute_confidence(
                    n_left, n_right, cte
                )
                self.last_detection_time = time.time()

        if cte is None:
            # Level 2: Use previous CTE if recent enough
            elapsed = time.time() - self.last_detection_time
            if elapsed < 0.5:
                cte = self.prev_cte
                confidence = max(0.0, 0.3 - 0.6 * elapsed)
            else:
                # Level 3/4: No valid detection
                cte = 0.0  # go straight
                confidence = 0.0

        # ── Publish ─────────────────────────────────────
        cte_msg = Float32()
        cte_msg.data = float(cte)
        self.pub_cte.publish(cte_msg)

        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self.pub_conf.publish(conf_msg)

        self.prev_cte = cte

        # ── Debug image (every 3rd frame to save bandwidth) ──
        self.frame_count += 1
        if self.frame_count % 3 == 0:
            debug = frame.copy()
            t_ms = (time.time() - t_start) * 1000
            cv2.putText(debug, f"CTE: {cte:.3f}m  Conf: {confidence:.2f}  "
                        f"dt: {t_ms:.0f}ms",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 2.4 Launch and Testing

```bash
# Terminal 1: Start camera
ros2 run v4l2_camera v4l2_camera_node

# Terminal 2: Start lane detection
ros2 run my_lane_pkg lane_detection_node \
    --ros-args -p lane_width_meters:=0.30

# Terminal 3: Monitor output
ros2 topic echo /lane/cte
ros2 topic echo /lane/confidence

# Terminal 4: View debug image
ros2 run rqt_image_view rqt_image_view /lane/debug
```

---

## 3. Sensor Fusion

### 3.1 Why Fuse Sensors?

No single sensor is sufficient for autonomous driving:

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| Camera | Rich semantic info (lanes, signs, colors) | No direct depth, affected by lighting |
| 1D LiDAR | Accurate distance, works in dark | Only one point, no semantic info |
| Depth Camera | Per-pixel depth + RGB | Short range, fails in sunlight |
| IMU | High rate motion data | Drifts over time |

**Sensor fusion** combines multiple sensors to compensate for individual weaknesses. The result is more reliable and robust than any single sensor alone.

### 3.2 Our Fusion Architecture

For the model car, we fuse two primary sensors:

```
┌──────────────┐         ┌──────────────────┐
│   Camera      │──CTE──►│                    │
│ (lane detect) │──conf─►│  Fusion Node       │──► /cmd_vel
│               │         │                    │    (steering + speed)
└──────────────┘         │  decision_maker    │
                          │                    │
┌──────────────┐         │                    │
│  1D LiDAR     │──dist─►│                    │
│ (TF-Luna)     │         └──────────────────┘
└──────────────┘
```

### 3.3 Camera + LiDAR: Decision Matrix

The fusion node makes decisions based on both lane CTE and obstacle distance:

| Lane Confidence | Obstacle Distance | Action |
|----------------|-------------------|--------|
| High (> 0.7) | Far (> 0.5 m) | Normal driving: use CTE for steering |
| High (> 0.7) | Near (0.2–0.5 m) | Slow down, keep steering |
| High (> 0.7) | Very near (< 0.2 m) | Emergency stop |
| Medium (0.3–0.7) | Far (> 0.5 m) | Reduced speed, use CTE |
| Medium (0.3–0.7) | Near (< 0.5 m) | Very slow, prepare to stop |
| Low (< 0.3) | Any | Stop — cannot see lane |
| Any | Very near (< 0.2 m) | Emergency stop — obstacle priority |

The critical design principle: **obstacle avoidance always has higher priority than lane following.** A car that veers out of lane is inconvenient. A car that hits an obstacle is dangerous.

### 3.4 Depth Camera + Bounding Box: 3D Position Estimation

If you have a depth camera (e.g., Intel RealSense D435), you can combine the depth map with a 2D bounding box from object detection (Day 19) to estimate the 3D position of detected objects:

Given:
- Bounding box center in pixels: \((u, v)\)
- Depth at that pixel: \(Z\) (from depth map)
- Camera intrinsics: \(f_x, f_y, c_x, c_y\)

The 3D position in the camera frame is:

$$
X = \frac{(u - c_x) \cdot Z}{f_x}, \qquad Y = \frac{(v - c_y) \cdot Z}{f_y}, \qquad Z = Z
$$

```python
def pixel_to_3d(u, v, depth, K):
    """Convert pixel + depth to 3D point in camera frame."""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    Z = depth
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z])
```

### 3.5 Weighted Fusion: Confidence-Based Decision Making

When multiple sensors provide conflicting information, we weight each by its **confidence**:

$$
\text{decision} = \frac{\sum_{i} w_i \cdot \text{sensor}_i}{\sum_{i} w_i}
$$

For our two-sensor system, the steering command is:

$$
\text{steer} = w_{\text{lane}} \cdot \text{CTE}_{\text{pid}} + w_{\text{obstacle}} \cdot \text{avoidance\_cmd}
$$

where \(w_{\text{lane}}\) is the lane detection confidence and \(w_{\text{obstacle}}\) increases as obstacle distance decreases. In practice, for a simple model car, a **decision matrix** (table above) is clearer and more debuggable than continuous weighting.

---

## 4. Safety Design

Safety is not a feature — it is a **constraint** that every design decision must satisfy. In automotive engineering, the standard is ISO 26262 (Functional Safety). While our model car does not need full ISO compliance, the principles still apply.

### 4.1 Watchdog Timer

A **watchdog timer** is a hardware or software mechanism that detects system hangs. The principle is simple:

1. The watchdog starts a countdown timer.
2. The main program must periodically **kick** (reset) the watchdog before it expires.
3. If the program hangs and fails to kick the watchdog, the timer expires and triggers a **safe action** (usually reset or emergency stop).

```
Normal operation:
  ┌──────┐    kick    ┌──────┐    kick    ┌──────┐
  │ Task │ ─────────► │ Task │ ─────────► │ Task │ ...
  └──────┘   200ms    └──────┘   200ms    └──────┘
              ▼                    ▼
         Timer reset          Timer reset

Hung program:
  ┌──────┐    kick    ┌──────────────────────────┐
  │ Task │ ─────────► │ HUNG (no kick)            │
  └──────┘   200ms    └──────────────────────────┘
              ▼                              ▼
         Timer reset                   TIMEOUT → SAFE STATE
```

#### Software Watchdog in ROS2

```python
class WatchdogTimer:
    """Software watchdog that triggers callback on timeout."""

    def __init__(self, timeout_sec, on_timeout):
        self.timeout = timeout_sec
        self.on_timeout = on_timeout
        self.last_kick = time.time()
        self._triggered = False

    def kick(self):
        """Reset the watchdog. Call this periodically from main loop."""
        self.last_kick = time.time()
        self._triggered = False

    def check(self):
        """Check if watchdog has expired. Call from a timer callback."""
        if not self._triggered and (time.time() - self.last_kick > self.timeout):
            self._triggered = True
            self.on_timeout()
            return True
        return False
```

For our car, we create watchdogs for each critical data source:

```python
# Watchdog for camera: timeout if no image for 1 second
camera_wd = WatchdogTimer(1.0, lambda: self.get_logger().error("CAMERA TIMEOUT"))

# Watchdog for LiDAR: timeout if no distance for 500 ms
lidar_wd = WatchdogTimer(0.5, lambda: self.get_logger().error("LIDAR TIMEOUT"))

# In camera callback:
def image_callback(self, msg):
    self.camera_wd.kick()
    # ... processing ...

# In lidar callback:
def lidar_callback(self, msg):
    self.lidar_wd.kick()
    # ... processing ...
```

### 4.2 Emergency Stop Logic

Emergency stop must be **instantaneous** and **unconditional**. It is triggered by:

- Obstacle closer than safety threshold
- Any watchdog timeout
- Manual E-stop button
- State machine entering EMERGENCY_STOP state

The E-stop command must have **highest priority** — no other node should be able to override it.

```python
class EmergencyStop:
    """Priority-based emergency stop manager."""

    def __init__(self):
        self.estop_active = False
        self.triggers = {}  # source → bool

    def set_trigger(self, source, active):
        """Set or clear a trigger source."""
        self.triggers[source] = active
        self.estop_active = any(self.triggers.values())

    def is_active(self):
        return self.estop_active

    def get_active_triggers(self):
        return [src for src, active in self.triggers.items() if active]
```

```python
estop = EmergencyStop()

# In lidar callback
if distance < 0.15:  # 15 cm
    estop.set_trigger("lidar_proximity", True)
else:
    estop.set_trigger("lidar_proximity", False)

# In watchdog check
if camera_wd.check():
    estop.set_trigger("camera_timeout", True)

# In motor command publisher
if estop.is_active():
    publish_zero_velocity()
    log(f"E-STOP active: {estop.get_active_triggers()}")
```

### 4.3 Fail-Safe State Machine

A state machine provides **structured** behavior transitions. Our car has four states:

```
                ┌──────────────────────────────────────────────┐
                │                                              │
                ▼                                              │
         ┌───────────┐                                         │
   ┌────►│  NORMAL    │                                         │
   │     │            │──── sensor degradation ────►┌───────────┴──┐
   │     └───────────┘                             │  DEGRADED     │
   │          │                                     │              │
   │          │ obstacle < 0.15m                   │              │
   │          │ OR camera timeout                  │              │
   │          │ OR manual E-stop                   │  obstacle or  │
   │          ▼                                     │  timeout      │
   │     ┌───────────────┐◄────────────────────────┘              │
   │     │ EMERGENCY_STOP │                                        │
   │     └───────┬───────┘                                        │
   │             │                                                 │
   │             │ motors confirmed stopped                        │
   │             │ AND timeout elapsed                             │
   │             ▼                                                 │
   │     ┌───────────┐                                            │
   │     │   SAFE     │                                            │
   │     │ (waiting)  │                                            │
   │     └───────┬───┘                                            │
   │             │                                                 │
   │             │ manual reset command                            │
   └─────────────┘                                                 │
                                                                    │
   Recovery from DEGRADED when sensors restored ───────────────────┘
```

#### State Definitions

| State | Behavior | Entry Condition |
|-------|----------|----------------|
| **NORMAL** | Full speed, lane following + obstacle detection | All sensors healthy, confidence > 0.7 |
| **DEGRADED** | Reduced speed (50%), widened safety margins | Confidence 0.3–0.7, or one sensor degraded |
| **EMERGENCY_STOP** | Zero velocity, all motors stopped | Obstacle < 15 cm, any timeout, E-stop button |
| **SAFE** | Motors locked, waiting for manual reset | Motors confirmed stopped after E-stop |

#### Implementation

```python
from enum import Enum, auto


class CarState(Enum):
    NORMAL = auto()
    DEGRADED = auto()
    EMERGENCY_STOP = auto()
    SAFE = auto()


class SafetyStateMachine:
    """Fail-safe state machine for autonomous car."""

    def __init__(self, logger):
        self.state = CarState.SAFE  # start in SAFE, require manual start
        self.logger = logger
        self.estop_time = None
        self.ESTOP_HOLD_SEC = 2.0  # hold E-stop for 2 seconds before SAFE

    def update(self, confidence, obstacle_dist, camera_alive, lidar_alive,
               manual_estop, manual_reset):
        """
        Evaluate conditions and transition state.
        Call this every control cycle (~50 Hz).
        """
        prev = self.state

        if self.state == CarState.NORMAL:
            if manual_estop or obstacle_dist < 0.15 or not camera_alive:
                self.state = CarState.EMERGENCY_STOP
                self.estop_time = time.time()
            elif confidence < 0.3 or not lidar_alive:
                self.state = CarState.DEGRADED

        elif self.state == CarState.DEGRADED:
            if manual_estop or obstacle_dist < 0.15 or not camera_alive:
                self.state = CarState.EMERGENCY_STOP
                self.estop_time = time.time()
            elif confidence > 0.7 and lidar_alive:
                self.state = CarState.NORMAL

        elif self.state == CarState.EMERGENCY_STOP:
            if self.estop_time and (time.time() - self.estop_time > self.ESTOP_HOLD_SEC):
                self.state = CarState.SAFE

        elif self.state == CarState.SAFE:
            if manual_reset and confidence > 0.5 and camera_alive and lidar_alive:
                self.state = CarState.NORMAL

        if self.state != prev:
            self.logger.info(f"State transition: {prev.name} → {self.state.name}")

        return self.state

    def get_speed_factor(self):
        """Return speed multiplier for current state."""
        factors = {
            CarState.NORMAL: 1.0,
            CarState.DEGRADED: 0.5,
            CarState.EMERGENCY_STOP: 0.0,
            CarState.SAFE: 0.0,
        }
        return factors[self.state]
```

---

## 5. Complete Fusion + Safety Node

### 5.1 The Decision Maker Node

This is the central node that fuses all sensor data and outputs motor commands:

```python
#!/usr/bin/env python3
"""
decision_maker_node.py
Day 18 — Sensor Fusion + Safety Decision Node

Subscribes to:
  /lane/cte         (Float32) — lane cross-track error
  /lane/confidence  (Float32) — lane detection confidence
  /lidar/distance   (Float32) — obstacle distance in meters
  /estop/button     (Bool)    — manual emergency stop

Publishes:
  /cmd_vel          (Twist)   — steering + speed command
  /car/state        (String)  — current state machine state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

import time


class DecisionMakerNode(Node):
    def __init__(self):
        super().__init__("decision_maker_node")

        # ── Parameters ──────────────────────────────────
        self.declare_parameter("base_speed", 0.3)         # m/s
        self.declare_parameter("kp_steering", 2.0)        # PID P gain
        self.declare_parameter("ki_steering", 0.0)
        self.declare_parameter("kd_steering", 0.5)
        self.declare_parameter("obstacle_slow_dist", 0.5)  # meters
        self.declare_parameter("obstacle_stop_dist", 0.15)
        self.declare_parameter("control_rate", 50.0)       # Hz

        # ── State ───────────────────────────────────────
        self.cte = 0.0
        self.confidence = 0.0
        self.obstacle_dist = 999.0
        self.manual_estop = False
        self.manual_reset = False

        self.last_camera_time = time.time()
        self.last_lidar_time = time.time()

        self.camera_wd = WatchdogTimer(1.0, self._on_camera_timeout)
        self.lidar_wd = WatchdogTimer(0.5, self._on_lidar_timeout)
        self.camera_alive = True
        self.lidar_alive = True

        self.state_machine = SafetyStateMachine(self.get_logger())

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # ── Publishers ──────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/car/state", 10)

        # ── Subscribers ─────────────────────────────────
        self.create_subscription(Float32, "/lane/cte", self.cte_cb, 10)
        self.create_subscription(Float32, "/lane/confidence", self.conf_cb, 10)
        self.create_subscription(Float32, "/lidar/distance", self.lidar_cb, 10)
        self.create_subscription(Bool, "/estop/button", self.estop_cb, 10)

        # ── Control loop timer ──────────────────────────
        rate = self.get_parameter("control_rate").value
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("Decision maker node started.")

    def _on_camera_timeout(self):
        self.camera_alive = False
        self.get_logger().error("Camera watchdog timeout!")

    def _on_lidar_timeout(self):
        self.lidar_alive = False
        self.get_logger().warn("LiDAR watchdog timeout!")

    def cte_cb(self, msg):
        self.cte = msg.data
        self.camera_wd.kick()
        self.camera_alive = True

    def conf_cb(self, msg):
        self.confidence = msg.data

    def lidar_cb(self, msg):
        self.obstacle_dist = msg.data
        self.lidar_wd.kick()
        self.lidar_alive = True

    def estop_cb(self, msg):
        self.manual_estop = msg.data
        if not msg.data:
            self.manual_reset = True  # rising edge = reset request

    def _pid_steering(self, error, dt):
        """PID controller for steering (from Day 9)."""
        kp = self.get_parameter("kp_steering").value
        ki = self.get_parameter("ki_steering").value
        kd = self.get_parameter("kd_steering").value

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # anti-windup

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = kp * error + ki * self.integral + kd * derivative
        return max(-1.0, min(1.0, output))  # clamp to [-1, 1]

    def control_loop(self):
        """Main control loop at fixed rate."""
        dt = 1.0 / self.get_parameter("control_rate").value

        # Check watchdogs
        self.camera_wd.check()
        self.lidar_wd.check()

        # Update state machine
        state = self.state_machine.update(
            confidence=self.confidence,
            obstacle_dist=self.obstacle_dist,
            camera_alive=self.camera_alive,
            lidar_alive=self.lidar_alive,
            manual_estop=self.manual_estop,
            manual_reset=self.manual_reset,
        )
        self.manual_reset = False  # consume reset

        # Compute commands based on state
        speed_factor = self.state_machine.get_speed_factor()
        base_speed = self.get_parameter("base_speed").value

        # Obstacle-based speed reduction (even in NORMAL state)
        slow_dist = self.get_parameter("obstacle_slow_dist").value
        stop_dist = self.get_parameter("obstacle_stop_dist").value

        if self.obstacle_dist < stop_dist:
            obstacle_factor = 0.0
        elif self.obstacle_dist < slow_dist:
            # Linear ramp: 0 at stop_dist, 1 at slow_dist
            obstacle_factor = (self.obstacle_dist - stop_dist) / (slow_dist - stop_dist)
        else:
            obstacle_factor = 1.0

        # Final speed
        speed = base_speed * speed_factor * obstacle_factor

        # Steering (only if moving)
        steering = 0.0
        if speed > 0.01:
            steering = self._pid_steering(self.cte, dt)

        # Publish Twist
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.pub_cmd.publish(cmd)

        # Publish state
        state_msg = String()
        state_msg.data = state.name
        self.pub_state.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 6. ROS2 Bag Recording and Failure Analysis

### 6.1 Why Record?

You cannot debug a real-time system by staring at it in real time. **ros2 bag** records all messages on selected topics to disk. You can replay them later at any speed, enabling frame-by-frame failure analysis.

### 6.2 Recording

```bash
# Record all relevant topics
ros2 bag record \
    /camera/image_raw \
    /lane/cte \
    /lane/confidence \
    /lidar/distance \
    /cmd_vel \
    /car/state \
    -o test_run_001

# Record with compression (saves disk space for images)
ros2 bag record \
    /camera/image_raw \
    /lane/cte \
    /lane/confidence \
    /lidar/distance \
    /cmd_vel \
    /car/state \
    --compression-mode message \
    --compression-format zstd \
    -o test_run_001
```

### 6.3 Replaying for Analysis

```bash
# Replay at normal speed
ros2 bag play test_run_001

# Replay at half speed (slow motion for debugging)
ros2 bag play test_run_001 --rate 0.5

# Replay specific topics only
ros2 bag play test_run_001 --topics /camera/image_raw /lane/cte

# Check bag info
ros2 bag info test_run_001
```

### 6.4 Failure Analysis Workflow

```
1. Run the car on the track → record ros2 bag
2. Note timestamps where failures occurred (veered, stopped unexpectedly, etc.)
3. Replay the bag at 0.5x speed
4. Run lane_detection_node on replayed data → observe /lane/debug images
5. Identify root cause:
   - Was the mask missing lane pixels? → tune HSV thresholds
   - Was CTE jumping wildly? → tighten confidence thresholds
   - Was there a latency spike? → check processing time
   - Did the state machine transition incorrectly? → review conditions
6. Fix parameters, replay bag again to verify fix
7. Test on real track
```

### 6.5 Python Script for Offline Analysis

```python
#!/usr/bin/env python3
"""
analyze_bag.py — Offline analysis of recorded test run.
Extracts CTE, confidence, and state over time for plotting.
"""

import sqlite3
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32, String
from rosidl_runtime_py.utilities import get_message


def read_bag_messages(bag_path, topic):
    """Read all messages from a topic in a ros2 bag (sqlite3 format)."""
    db_path = f"{bag_path}/{bag_path.split('/')[-1]}_0.db3"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get topic ID
    cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic,))
    row = cursor.fetchone()
    if row is None:
        print(f"Topic {topic} not found in bag.")
        return [], []

    topic_id, msg_type = row
    msg_class = get_message(msg_type)

    # Read messages
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,)
    )

    timestamps = []
    messages = []
    for ts, data in cursor.fetchall():
        msg = deserialize_message(data, msg_class)
        timestamps.append(ts * 1e-9)  # nanoseconds → seconds
        messages.append(msg)

    conn.close()
    return timestamps, messages


def analyze_run(bag_path):
    """Plot CTE, confidence, and state over time."""
    # Read topics
    t_cte, msgs_cte = read_bag_messages(bag_path, "/lane/cte")
    t_conf, msgs_conf = read_bag_messages(bag_path, "/lane/confidence")
    t_state, msgs_state = read_bag_messages(bag_path, "/car/state")

    if not t_cte:
        print("No data found.")
        return

    # Normalize time to start at 0
    t0 = min(t_cte[0], t_conf[0]) if t_conf else t_cte[0]
    t_cte = [t - t0 for t in t_cte]
    t_conf = [t - t0 for t in t_conf]
    t_state = [t - t0 for t in t_state]

    cte_vals = [m.data for m in msgs_cte]
    conf_vals = [m.data for m in msgs_conf]
    state_vals = [m.data for m in msgs_state]

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(t_cte, cte_vals, 'b-', linewidth=0.8)
    axes[0].set_ylabel("CTE (m)")
    axes[0].axhline(0, color='gray', linestyle='--', alpha=0.5)
    axes[0].set_title("Cross-Track Error Over Time")

    axes[1].plot(t_conf, conf_vals, 'g-', linewidth=0.8)
    axes[1].set_ylabel("Confidence")
    axes[1].axhline(0.7, color='orange', linestyle='--', label="Normal threshold")
    axes[1].axhline(0.3, color='red', linestyle='--', label="Degraded threshold")
    axes[1].legend()
    axes[1].set_title("Detection Confidence Over Time")

    # State as colored background
    state_colors = {
        "NORMAL": "green", "DEGRADED": "orange",
        "EMERGENCY_STOP": "red", "SAFE": "gray"
    }
    for i in range(len(t_state) - 1):
        color = state_colors.get(state_vals[i], "white")
        axes[2].axvspan(t_state[i], t_state[i + 1], alpha=0.3, color=color)
    axes[2].set_ylabel("State")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("State Machine State Over Time")

    plt.tight_layout()
    plt.savefig("run_analysis.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    import sys
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "test_run_001"
    analyze_run(bag_path)
```

---

## 7. Real Track Test Protocol

### 7.1 Test Procedure

Follow this structured procedure for repeatable results:

```
1. SETUP
   - Place car at starting position
   - Verify camera feed: ros2 topic hz /camera/image_raw
   - Verify LiDAR feed: ros2 topic echo /lidar/distance
   - Start ros2 bag recording

2. TEST 1: Lane following only (no obstacles)
   - Start car (manual_reset = True)
   - Run 3 full laps
   - Record: number of lane departures, max CTE

3. TEST 2: Lane following + obstacle
   - Place static obstacle on track
   - Run 3 laps
   - Record: stop distance from obstacle, resumption time

4. TEST 3: Failure injection
   - Cover camera lens mid-run → verify E-stop triggers
   - Unplug LiDAR → verify DEGRADED state
   - Press manual E-stop → verify immediate stop

5. ANALYSIS
   - Stop recording
   - Run analyze_bag.py
   - Document results
```

### 7.2 Performance Metrics

| Metric | Target | How to Measure |
|--------|--------|---------------|
| Lane keeping | CTE < 5 cm | Average CTE from bag data |
| Obstacle stop distance | > 10 cm | LiDAR reading when stopped |
| E-stop latency | < 200 ms | Timestamp difference: trigger → zero velocity |
| Frame rate | > 15 FPS | `ros2 topic hz /lane/cte` |
| State transition correctness | No false E-stops in normal driving | Review state log from bag |

---

## 8. Review and Summary

### What We Covered

| Topic | Key Takeaway |
|-------|-------------|
| Fallback Strategy | Five levels from "both lanes" to "emergency stop" — never binary |
| ROS2 Node | `sensor_msgs/Image` → `cv_bridge` → pipeline → `Float32` CTE |
| Sensor Fusion | Camera provides steering, LiDAR provides braking — obstacle always wins |
| Watchdog | Periodic heartbeat; timeout → safe state. Simple, effective, essential. |
| State Machine | NORMAL → DEGRADED → EMERGENCY_STOP → SAFE with explicit conditions |
| ros2 bag | Record everything, replay at 0.5x, analyze offline — the only sane way to debug |

### Key Formulas

$$
\text{confidence} = \min\left(\frac{N_{\text{pixels}}}{N_{\text{threshold}}}, 1\right) \times \left(1 - \frac{|\Delta\text{CTE}|}{\Delta_{\max}}\right)
$$

$$
\text{3D position: } X = \frac{(u - c_x) Z}{f_x}, \quad Y = \frac{(v - c_y) Z}{f_y}
$$

### Connection to Other Days

- **Day 9 (PID Control):** The PID steering controller is embedded in the decision maker node.
- **Day 17 (Lane Detection):** The vision pipeline is now a ROS2 node feeding the fusion system.
- **Day 19 (Tomorrow):** We add YOLOv5 object detection — the car will not just detect obstacles as "something is close" but identify **what** the obstacle is (traffic sign, pedestrian, other car).

---

**Next up — Day 19:** YOLOv5 object detection, transfer learning on a custom dataset, and INT8 quantization to prepare models for edge deployment on the Hailo NPU.
