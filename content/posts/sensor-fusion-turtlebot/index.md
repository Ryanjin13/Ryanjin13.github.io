---
title: "Sensor Fusion on TurtleBot3"
date: 2024-07-03
description: "Implementing camera and LIDAR sensor fusion for robotics"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "Turtlebot3", "ROS", "SLAM"]
draft: false
---

{{< katex >}}

## Overview

Sensor fusion combines data from multiple sensors to create a more accurate and robust perception system. This guide covers integrating camera and LIDAR on TurtleBot3.

## Required Components

### Hardware

- TurtleBot3 robot platform
- Camera (Intel RealSense or USB camera)
- LIDAR (LDS-01/02 included with TurtleBot3)

### Software

- ROS (Noetic)
- SLAM package (gmapping or cartographer)
- Image processing libraries (OpenCV, cv_bridge)
- RealSense driver (if using RealSense)

## Installation

### RealSense Driver

```bash
sudo apt install ros-noetic-realsense2-camera
```

### OpenCV Bridge

```bash
sudo apt install ros-noetic-cv-bridge
sudo apt install ros-noetic-image-transport
```

### SLAM Package

```bash
sudo apt install ros-noetic-slam-gmapping
# OR
sudo apt install ros-noetic-cartographer-ros
```

## Data Collection

### Launch Camera

RealSense:
```bash
roslaunch realsense2_camera rs_camera.launch
```

USB Camera:
```bash
roslaunch usb_cam usb_cam-test.launch
```

### Launch LIDAR

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Start SLAM

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

## Sensor Fusion Implementation

### Python Node

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2

class SensorFusion:
    def __init__(self):
        rospy.init_node('sensor_fusion_node')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None

        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image,
                        self.image_callback)
        rospy.Subscriber('/scan', LaserScan,
                        self.lidar_callback)

        rospy.spin()

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, 'bgr8')
            self.process_fusion()
        except Exception as e:
            rospy.logerr(f"Image error: {e}")

    def lidar_callback(self, msg):
        self.latest_scan = msg
        # Process LIDAR data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

    def process_fusion(self):
        if self.latest_image is None or self.latest_scan is None:
            return

        # Fusion logic here
        # Example: Overlay LIDAR on image
        pass

if __name__ == '__main__':
    try:
        SensorFusion()
    except rospy.ROSInterruptException:
        pass
```

## Visualization with RViz

### Launch RViz

```bash
rosrun rviz rviz
```

### Add Displays

1. **Add Camera:**
   - Click "Add"
   - Select "Image"
   - Set topic: `/camera/color/image_raw`

2. **Add LIDAR:**
   - Click "Add"
   - Select "LaserScan"
   - Set topic: `/scan`

3. **Add Map (if SLAM running):**
   - Click "Add"
   - Select "Map"
   - Set topic: `/map`

### Save Configuration

File → Save Config As → `sensor_fusion.rviz`

## Fusion Strategies

### Early Fusion

Combine raw sensor data:

$$
\text{Fused} = \alpha \cdot \text{Camera} + (1-\alpha) \cdot \text{LIDAR}
$$

### Late Fusion

Combine processed results:

$$
\text{Detection} = f(\text{Camera Detection}, \text{LIDAR Detection})
$$

### Kalman Filter Fusion

Optimal state estimation:

$$
\hat{x}_k = \hat{x}_{k-1} + K_k(z_k - H\hat{x}_{k-1})
$$

## Calibration

### Camera-LIDAR Alignment

1. Collect calibration data
2. Find transformation matrix
3. Project LIDAR points to image

### Extrinsic Calibration

Transform between sensor frames:

```bash
rosrun tf static_transform_publisher x y z yaw pitch roll parent_frame child_frame period_ms
```

## Applications

| Application | Camera Role | LIDAR Role |
|-------------|-------------|------------|
| Navigation | Visual odometry | Obstacle detection |
| SLAM | Feature extraction | Range measurement |
| Object Detection | Classification | Localization |
| Collision Avoidance | Visual awareness | Precise distance |

## Performance Tips

1. **Synchronize timestamps** between sensors
2. **Reduce resolution** if processing too slow
3. **Use hardware acceleration** if available
4. **Filter noise** before fusion
