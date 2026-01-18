---
title: "Sensor Fusion Summary"
date: 2024-07-25
description: "Camera and LiDAR sensor fusion on TurtleBot3 with ROS"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "ROS", "LiDAR", "Camera", "Turtlebot3"]
draft: false
---

## Overview

This post documents a sensor fusion project integrating camera and LiDAR data from TurtleBot3 to a PC using ROS (Robot Operating System).

## System Architecture

```
TurtleBot3 (Raspberry Pi)
    ├── Camera → /camera/image_raw
    └── LiDAR  → /scan
          ↓
    [ROS TOPIC Layer]
          ↓
    PC (Fusion Processing)
    └── fusion.py
```

## Implementation

### ROS Package Setup

Custom ROS package required for systematic data interconnection:

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Create package
catkin_create_pkg sensor_fusion rospy std_msgs sensor_msgs cv_bridge

# Build
cd ~/catkin_ws
catkin_make
```

### LiDAR-Camera Fusion Challenge

**Problem:** 2D LiDAR provides only 1D point data in static conditions.

**Solution:** Mask and distribute sensor information across a specific image row aligned with camera's field of view.

```python
import rospy
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np

class SensorFusion:
    def __init__(self):
        self.bridge = CvBridge()
        self.lidar_data = None
        self.camera_fov = 30  # degrees

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)

    def lidar_callback(self, msg):
        # Extract valid range: ±45° from front
        self.lidar_data = msg.ranges

    def camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.lidar_data is not None:
            self.fuse_data(image, self.lidar_data)

    def fuse_data(self, image, lidar):
        # Map LiDAR points to image coordinates
        # Based on camera FOV alignment
        pass
```

### Field of View Calibration

| Sensor | FOV | Valid Range |
|--------|-----|-------------|
| Camera | 30° | Full image width |
| LiDAR | 360° | ±15° from center (adjusted) |

Initial setting: ±45° → Refined to ±15° to match camera FOV.

## Network Configuration

### Raspberry Pi WiFi Setup

Challenge: Connecting RPi via laptop hotspot.

**Solution:** Install NetworkManager and configure YAML:

```yaml
# /etc/netplan/01-network-manager-all.yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "HotspotName":
          password: "password"
```

```bash
sudo netplan apply
```

### ROS Network Configuration

```bash
# On TurtleBot3 (RPi)
export ROS_MASTER_URI=http://PC_IP:11311
export ROS_HOSTNAME=RPI_IP

# On PC
export ROS_MASTER_URI=http://PC_IP:11311
export ROS_HOSTNAME=PC_IP
```

## Results

- Successful camera-LiDAR data synchronization
- Real-time fusion pipeline execution
- Depth information overlay on camera image

## Technical Stack

- ROS Noetic
- Python 3
- OpenCV
- Raspberry Pi 4
- TurtleBot3 Burger
