---
title: "ROS Sensor Fusion Packages"
date: 2024-07-08
description: "Essential ROS packages for sensor fusion in robotics"
categories: ["Autonomous Driving"]
tags: ["ROS", "Sensor Fusion", "Point Cloud", "Computer Vision"]
draft: false
---

## Overview

ROS provides numerous packages for sensor fusion. This guide covers essential packages for point cloud processing, image processing, and multi-sensor integration.

## Point Cloud Packages

### pcl_ros

Point Cloud Library's ROS wrapper:

```bash
sudo apt install ros-noetic-pcl-ros
```

**Capabilities:**
- Point cloud filtering
- Segmentation
- Surface reconstruction
- Feature extraction

### laser_geometry

Converts 2D laser scans to 3D point clouds:

```bash
sudo apt install ros-noetic-laser-geometry
```

**Use case:** Transform LaserScan to PointCloud2.

### pointcloud_to_laserscan

Converts 3D point cloud to 2D laser scan:

```bash
sudo apt install ros-noetic-pointcloud-to-laserscan
```

**Use case:** Use 3D sensor with 2D navigation.

### depth_image_proc

Depth image processing:

```bash
sudo apt install ros-noetic-depth-image-proc
```

**Operations:**
- Depth to point cloud
- Register depth to RGB
- Convert formats

### octomap_ros

3D occupancy grid mapping:

```bash
sudo apt install ros-noetic-octomap-ros
```

**Features:**
- Efficient 3D representation
- Probabilistic updates
- Dynamic environments

### rtabmap_ros

Real-time appearance-based mapping:

```bash
sudo apt install ros-noetic-rtabmap-ros
```

**Capabilities:**
- Visual SLAM
- RGB-D SLAM
- Multi-session mapping
- Loop closure

## Image Processing Packages

### vision_opencv

OpenCV integration with ROS:

```bash
sudo apt install ros-noetic-vision-opencv
```

Includes:
- cv_bridge: ROS ↔ OpenCV conversion
- image_geometry: Camera models

### darknet_ros

YOLO object detection:

```bash
# Clone to workspace
cd ~/catkin_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
catkin_make
```

**Features:**
- Real-time detection
- Multiple classes
- GPU acceleration

### find_object_2d

2D object detection with pose estimation:

```bash
sudo apt install ros-noetic-find-object-2d
```

## Example: Point Cloud Visualizer

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2

class PointCloudVisualizer:
    def __init__(self):
        rospy.init_node('pointcloud_visualizer')
        self.bridge = CvBridge()

        # Subscriber
        rospy.Subscriber('/camera/depth/points',
                        PointCloud2, self.callback)

        # Publisher
        self.pub = rospy.Publisher('/visualization',
                                   Image, queue_size=10)

    def callback(self, msg):
        # Extract points
        points = []
        for p in pc2.read_points(msg, field_names=('x','y','z')):
            points.append([p[0], p[1], p[2]])

        points = np.array(points)

        # Calculate distances
        distances = np.sqrt(np.sum(points**2, axis=1))

        # Normalize for visualization
        normalized = (distances - distances.min()) / \
                    (distances.max() - distances.min())

        # Apply colormap
        colors = cv2.applyColorMap(
            (normalized * 255).astype(np.uint8),
            cv2.COLORMAP_JET
        )

        # Publish visualization
        msg = self.bridge.cv2_to_imgmsg(colors, 'bgr8')
        self.pub.publish(msg)

if __name__ == '__main__':
    PointCloudVisualizer()
    rospy.spin()
```

## Package Comparison

| Package | Input | Output | Use Case |
|---------|-------|--------|----------|
| pcl_ros | PointCloud2 | Filtered/Segmented | General processing |
| laser_geometry | LaserScan | PointCloud2 | 2D to 3D |
| octomap | PointCloud2 | OctoMap | 3D mapping |
| rtabmap | RGB-D | Map + Pose | Visual SLAM |

## Installation Summary

```bash
# Core packages
sudo apt install ros-noetic-pcl-ros
sudo apt install ros-noetic-laser-geometry
sudo apt install ros-noetic-depth-image-proc
sudo apt install ros-noetic-vision-opencv

# Mapping
sudo apt install ros-noetic-octomap-ros
sudo apt install ros-noetic-rtabmap-ros

# Detection
sudo apt install ros-noetic-find-object-2d
```
