---
title: "Turtlebot3 Puppy Mode"
date: 2024-10-27
description: "Autonomous mobile robot system with SLAM, object detection, and human-robot interaction"
categories: ["Autonomous Driving"]
tags: ["Turtlebot3", "ROS", "SLAM", "YOLO", "Sensor Fusion"]
draft: false
---

## Overview

This project implements a comprehensive autonomous mobile robot system built on the Turtlebot3 platform, integrating multiple sensors for navigation, object detection, and human-robot interaction.

## System Architecture

### Hardware Configuration

| Component | Specification |
|-----------|---------------|
| **Computing** | Raspberry Pi 3 |
| **Controller** | OpenCR |
| **Motors** | Dynamixel servos |
| **LiDAR** | 360° scanning |
| **Camera** | USB camera |
| **OS** | Ubuntu 20.04 + ROS |

### Sensor Setup

**LiDAR Configuration:**
- 360-degree environmental scanning
- 30-degree depth field gradation
- Primary sensor for SLAM

**Camera Positioning:**
- Positioned 50 pixels above LiDAR
- 15-degree angular coverage per side
- Optimized for sensor fusion with LiDAR data

## Key Capabilities

### 1. Perception & Detection

**YOLO Object Detection:**
```python
# Real-time object detection
def detect_objects(frame):
    results = yolo_model(frame)
    return results.xyxy[0]  # Bounding boxes
```

**Person Detection:**
- Specialized algorithms for human tracking
- Synchronized callbacks for temporal data alignment

### 2. SLAM Navigation

Simultaneous Localization and Mapping:

```bash
# Launch SLAM node
roslaunch turtlebot3_slam turtlebot3_slam.launch

# Launch navigation
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

**Features:**
- Real-time map building
- Obstacle avoidance
- Path planning
- Autonomous navigation

### 3. Distance Calculation

```python
def calculate_distance(lidar_data, angle):
    # Get range at specific angle
    index = int(angle * len(lidar_data) / 360)
    distance = lidar_data[index]
    return distance
```

### 4. Sensor Fusion

Combining LiDAR and camera data:

```python
def sensor_fusion_callback(lidar_msg, camera_msg):
    # Synchronize timestamps
    # Fuse spatial data from LiDAR with visual data from camera
    # Generate unified perception output
    pass
```

## Visualization

**RVIZ Monitoring:**
- 3D visualization of robot state
- Real-time sensor data display
- Environmental map rendering
- Path visualization

```bash
# Launch RVIZ
roslaunch turtlebot3_bringup turtlebot3_remote.launch
rviz
```

## ROS Node Structure

```
/turtlebot3/
├── /scan (LiDAR data)
├── /camera/image_raw (Camera feed)
├── /cmd_vel (Velocity commands)
├── /odom (Odometry)
├── /map (SLAM map)
└── /detection (YOLO results)
```

## Applications

- Autonomous navigation in indoor environments
- Human following ("Puppy" mode)
- Object detection and tracking
- Security patrol
- Research platform
