---
title: "Depth Image Processing in ROS"
date: 2024-07-08
description: "Converting point clouds to depth images for sensor fusion"
categories: ["Autonomous Driving"]
tags: ["ROS", "Depth", "Point Cloud", "Sensor Fusion"]
draft: false
---

## Overview

Depth image processing bridges 3D point cloud data with 2D image processing. This guide covers using the `depth_image_proc` package in ROS.

## Sensor Fusion Pipeline

```
Point Cloud ────→ Depth Image ────→ Fusion with RGB
    ↑                                      ↓
  SLAM/LiDAR                         Combined Data
```

## Data Sources

### Point Cloud Data

From SLAM systems or 3D sensors:
- Topic: `/points` or `/cloud`
- Message type: `sensor_msgs/PointCloud2`

### Camera Images

From USB or RGB-D cameras:
- Topic: `/camera/image_raw`
- Message type: `sensor_msgs/Image`

## Installation

### Install depth_image_proc

```bash
sudo apt-get install ros-noetic-depth-image-proc
```

### Verify Installation

```bash
rospack find depth_image_proc
```

## Available Nodes

### pointcloud_to_depth_image

Converts 3D point cloud to 2D depth image:

```bash
rosrun depth_image_proc pointcloud_to_depth_image \
    input:=/points \
    output:=/depth_image
```

### depth_image_to_pointcloud

Converts depth image back to point cloud:

```bash
rosrun nodelet nodelet standalone depth_image_proc/point_cloud_xyz \
    image_rect:=/camera/depth/image_raw
```

### register_depth

Registers depth image to color camera frame.

## Launch File Example

```xml
<launch>
  <!-- Point cloud to depth -->
  <node pkg="depth_image_proc" type="pointcloud_to_depth_image"
        name="cloud_to_depth">
    <remap from="input" to="/points"/>
    <remap from="output" to="/depth/image"/>
    <param name="range_max" value="10.0"/>
  </node>

  <!-- Combine depth with RGB -->
  <node pkg="depth_image_proc" type="register"
        name="register_depth_to_rgb">
    <remap from="rgb/image_rect" to="/camera/image_raw"/>
    <remap from="depth/image_rect" to="/depth/image"/>
    <remap from="rgb/camera_info" to="/camera/camera_info"/>
    <remap from="depth/camera_info" to="/depth/camera_info"/>
  </node>
</launch>
```

## Fusion Algorithm Concepts

### Depth from Point Cloud

For each point \\((x, y, z)\\) in camera frame:

$$
u = f_x \cdot \frac{x}{z} + c_x
$$

$$
v = f_y \cdot \frac{y}{z} + c_y
$$

$$
\text{depth}(u, v) = z
$$

### Camera Model

| Parameter | Description |
|-----------|-------------|
| \\(f_x, f_y\\) | Focal lengths |
| \\(c_x, c_y\\) | Principal point |
| \\(z\\) | Depth value |

## Visualization

### In RViz

1. Add "DepthCloud" display
2. Set depth topic: `/depth/image`
3. Set color topic: `/camera/image_raw`

### View Depth Image

```bash
rosrun image_view image_view image:=/depth/image
```

## Troubleshooting

### No Output

Check topic connections:

```bash
rostopic info /depth/image
```

Verify input topics exist:

```bash
rostopic list | grep points
```

### Frame Mismatch

Ensure point cloud and camera share common frame:

```bash
rosrun tf tf_echo camera_frame lidar_frame
```

### Range Issues

Adjust maximum depth range in parameters:

```xml
<param name="range_max" value="20.0"/>
```

## Common Message Types

| Type | Description |
|------|-------------|
| sensor_msgs/Image | 2D depth image |
| sensor_msgs/PointCloud2 | 3D point cloud |
| sensor_msgs/CameraInfo | Camera calibration |

## Applications

1. **Obstacle detection** - 2D depth analysis
2. **RGBD reconstruction** - Colored point clouds
3. **Navigation** - Depth-based planning
4. **Object detection** - Combined RGB-D inference
