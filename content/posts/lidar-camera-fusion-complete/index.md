---
title: "Complete LiDAR-Camera Fusion for TurtleBot3"
date: 2024-07-11
description: "Full implementation of LiDAR and camera sensor fusion with ROS"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "LiDAR", "Camera", "ROS", "TurtleBot3"]
draft: false
---

{{< katex >}}

## Overview

This guide provides a complete implementation for fusing LiDAR and camera data on TurtleBot3, including the critical LiDAR coordinate quirks and optimized visualization.

## System Architecture

```
┌─────────────┐     ┌─────────────┐
│ USB Camera  │     │   LiDAR     │
│ /image_raw  │     │   /scan     │
└──────┬──────┘     └──────┬──────┘
       │                   │
       └─────────┬─────────┘
                 │
         ┌───────┴───────┐
         │  Time Sync    │
         │ Fusion Node   │
         └───────┬───────┘
                 │
         ┌───────┴───────┐
         │ /fusion_image │
         └───────────────┘
```

## LiDAR Coordinate System

### Critical Discovery

The LiDAR rotates clockwise, but the angle mapping is counterintuitive:

```
           0°
     45° ─┼─ 315°
          │
    90° ──●── 270°
          │
   135° ─┼─ 225°
         180°

Front-facing angles:
- Left side: 0° to 45°
- Right side: 315° to 360°
```

This requires coordinate transformation in the fusion code.

## Camera Parameters

```python
fx = 844   # Focal length x (pixels)
fy = 844   # Focal length y (pixels)
cx = 320   # Principal point x
cy = 250   # Principal point y
```

## Complete Fusion Node

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2

class CameraLidarFusion:
    def __init__(self):
        rospy.init_node('camera_lidar_fusion')
        self.bridge = CvBridge()

        # Camera parameters
        self.fx = 844
        self.fy = 844
        self.cx = 320
        self.cy = 250

        # Precompute color LUT
        self.color_lut = self._create_color_lut()

        # Subscribers with time sync
        image_sub = message_filters.Subscriber(
            '/usb_cam/image_raw', Image)
        scan_sub = message_filters.Subscriber(
            '/scan', LaserScan)

        # Approximate time synchronization (100ms tolerance)
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, scan_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        # Publisher
        self.pub = rospy.Publisher(
            '/fusion_image', Image, queue_size=10)

        rospy.spin()

    def _create_color_lut(self):
        """Precompute HSV to BGR color lookup table"""
        lut = np.zeros((256, 3), dtype=np.uint8)
        for i in range(256):
            # Orange spectrum (hue 20-30)
            h = 25
            s = 200 + int(55 * (1 - i/255))
            v = 100 + int(155 * i/255)
            hsv = np.uint8([[[h, s, v]]])
            bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            lut[i] = bgr[0, 0]
        return lut

    def callback(self, image_msg, scan_msg):
        # Convert image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                image_msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # Process LiDAR data
        angles = np.arange(scan_msg.angle_min,
                          scan_msg.angle_max,
                          scan_msg.angle_increment)
        angles_deg = np.degrees(angles) % 360
        ranges = np.array(scan_msg.ranges)

        # Filter front-facing angles (0-15° and 345-360°)
        left_mask = (angles_deg >= 0) & (angles_deg <= 15)
        right_mask = (angles_deg >= 345) & (angles_deg <= 360)
        valid_mask = (left_mask | right_mask) & \
                    (ranges > scan_msg.range_min) & \
                    (ranges < scan_msg.range_max)

        valid_angles = angles_deg[valid_mask]
        valid_ranges = ranges[valid_mask]

        # Project to image
        h, w = cv_image.shape[:2]
        for angle, distance in zip(valid_angles, valid_ranges):
            # Convert LiDAR angle to image x coordinate
            if angle <= 15:
                x = int(w/2 - (angle/15) * (w/2))
            else:
                x = int(w/2 + ((360-angle)/15) * (w/2))

            # Height based on distance
            y = int(cy)

            # Bounds check
            if 0 <= x < w and 0 <= y < h:
                # Distance-based color
                color_idx = int(np.clip(
                    distance/scan_msg.range_max * 255, 0, 255))
                color = tuple(map(int, self.color_lut[color_idx]))

                # Draw point
                cv2.circle(cv_image, (x, y), 5, color, -1)

        # Publish
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Publish error: {e}")

if __name__ == '__main__':
    CameraLidarFusion()
```

## Projection Mathematics

### LiDAR to Camera Coordinates

For a LiDAR point at \\((r, \theta)\\):

$$
x_{cam} = r \cdot \sin(\theta)
$$

$$
z_{cam} = r \cdot \cos(\theta)
$$

### Camera to Image

$$
u = f_x \cdot \frac{x_{cam}}{z_{cam}} + c_x
$$

$$
v = f_y \cdot \frac{y_{cam}}{z_{cam}} + c_y
$$

## Launch Sequence

### Terminal 1: roscore

```bash
roscore
```

### Terminal 2: TurtleBot

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Terminal 3: USB Camera

```bash
roslaunch usb_cam usb_cam.launch
```

### Terminal 4: Fusion

```bash
rosrun fusion_package fusion_node.py
```

### Terminal 5: View

```bash
rosrun image_view image_view image:=/fusion_image
```

## Optimization Tips

| Technique | Benefit |
|-----------|---------|
| Color LUT | Avoid HSV conversion per point |
| NumPy vectorization | Faster than Python loops |
| Reduced FOV | Less computation |
| Approximate sync | More robust timing |

## Verification

```bash
rostopic list | grep fusion
rostopic hz /fusion_image
```

Expected: ~15-30 Hz depending on hardware.
