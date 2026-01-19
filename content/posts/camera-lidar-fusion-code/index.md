---
title: "Camera-LiDAR Fusion Implementation"
date: 2024-07-08
description: "Real-time camera and LiDAR sensor fusion with ROS"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "ROS", "Camera", "LiDAR", "Python"]
draft: false
---

{{< katex >}}

## Overview

This guide covers implementing real-time camera and LiDAR fusion in ROS, projecting 3D points onto 2D images with depth-based coloring.

## System Setup

### Build Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Create Package

```bash
cd ~/catkin_ws/src
catkin_create_pkg fusion_package rospy sensor_msgs cv_bridge image_transport
```

## Fusion Node Implementation

### Complete Python Code

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class CameraLidarFusion:
    def __init__(self):
        # Initialize node with anonymous mode
        rospy.init_node('camera_lidar_fusion', anonymous=True)

        # CvBridge for ROS-OpenCV conversion
        self.bridge = CvBridge()

        # Data storage
        self.current_image = None
        self.current_points = None

        # Camera intrinsic parameters
        self.fx = 615.0  # Focal length x
        self.fy = 615.0  # Focal length y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y

        # Subscribers
        rospy.Subscriber('/camera/image_raw', Image,
                        self.image_callback)
        rospy.Subscriber('/camera/depth/points', PointCloud2,
                        self.pointcloud_callback)

        # Publisher for fused image
        self.pub = rospy.Publisher('/fusion_image', Image,
                                   queue_size=10)

        rospy.spin()

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            # For YUYV format cameras
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Convert YUYV to BGR if needed
            if msg.encoding == 'yuyv':
                cv_image = cv2.cvtColor(cv_image,
                                       cv2.COLOR_YUV2BGR_YUYV)

            self.current_image = cv_image
            self.process_fusion()

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def pointcloud_callback(self, msg):
        # Extract points from PointCloud2
        points = []
        for p in pc2.read_points(msg,
                                 field_names=('x','y','z'),
                                 skip_nans=True):
            points.append([p[0], p[1], p[2]])

        self.current_points = np.array(points)

    def process_fusion(self):
        if self.current_image is None or self.current_points is None:
            return

        # Copy image for visualization
        fusion_image = self.current_image.copy()

        # Project 3D points to 2D
        for point in self.current_points:
            x, y, z = point

            # Skip points behind camera
            if z <= 0:
                continue

            # Project to image plane
            u = int(self.fx * x / z + self.cx)
            v = int(self.fy * y / z + self.cy)

            # Check if within image bounds
            h, w = fusion_image.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                # Distance-based coloring
                distance = np.sqrt(x**2 + y**2 + z**2)
                color = self.distance_to_color(distance)

                # Draw point on image
                cv2.circle(fusion_image, (u, v), 2, color, -1)

        # Publish fused image
        try:
            msg = self.bridge.cv2_to_imgmsg(fusion_image, 'bgr8')
            self.pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def distance_to_color(self, distance, min_dist=0.5, max_dist=10.0):
        # Normalize distance
        normalized = (distance - min_dist) / (max_dist - min_dist)
        normalized = np.clip(normalized, 0, 1)

        # Blue (close) to Red (far)
        r = int(normalized * 255)
        b = int((1 - normalized) * 255)
        g = 0

        return (b, g, r)  # BGR format

if __name__ == '__main__':
    try:
        CameraLidarFusion()
    except rospy.ROSInterruptException:
        pass
```

## Launch File

Create `fusion.launch`:

```xml
<launch>
  <node pkg="fusion_package" type="fusion_node.py"
        name="fusion" output="screen"/>
</launch>
```

## Key Concepts

### 3D to 2D Projection

$$
u = f_x \cdot \frac{x}{z} + c_x
$$

$$
v = f_y \cdot \frac{y}{z} + c_y
$$

### Distance Calculation

$$
d = \sqrt{x^2 + y^2 + z^2}
$$

### Color Mapping

| Distance | Color |
|----------|-------|
| Close | Blue |
| Medium | Green |
| Far | Red |

## YUYV Color Format

Many USB cameras use YUYV format:

```python
# Convert YUYV to BGR
cv_image = cv2.cvtColor(raw_image, cv2.COLOR_YUV2BGR_YUYV)
```

## Running the System

```bash
# Terminal 1: roscore
roscore

# Terminal 2: Camera node
roslaunch usb_cam usb_cam.launch

# Terminal 3: LiDAR/depth sensor
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# Terminal 4: Fusion node
roslaunch fusion_package fusion.launch

# Terminal 5: View result
rosrun image_view image_view image:=/fusion_image
```

## Calibration

For accurate projection, calibrate camera:

```bash
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.025 \
    image:=/camera/image_raw
```
