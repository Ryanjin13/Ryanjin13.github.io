---
title: "LiDAR Coordinate System and TF in ROS"
date: 2024-07-08
description: "Understanding LiDAR coordinate frames and transforms in ROS"
categories: ["Autonomous Driving"]
tags: ["LiDAR", "ROS", "TF", "Coordinate System"]
draft: false
---

## Overview

Understanding coordinate systems is essential for sensor fusion in robotics. This guide covers LiDAR coordinate conventions and ROS Transform (TF) system.

## Standard Coordinate Convention

### RGB Axis System

```
         Z (Blue)
         ↑
         │
         │
         └────────→ X (Red)
        ╱
       ╱
      Y (Green)
```

| Axis | Color | Direction |
|------|-------|-----------|
| X | Red | Forward |
| Y | Green | Left |
| Z | Blue | Upward |

This follows the right-hand rule.

## TF (Transform) System

### What is TF?

TF manages relationships between multiple coordinate systems:
- Robot base to sensor frames
- Map to robot frame
- World to local frames

### TF Tree Structure

```
       map
        │
        ↓
      odom
        │
        ↓
    base_link
    ↙       ↘
laser       camera
```

## Common Commands

### View TF Tree

```bash
rosrun tf view_frames
```

Generates `frames.pdf` showing transform tree.

### Alternative (Noetic)

For Python 3 compatibility:

```bash
rosrun tf2_tools view_frames.py
```

Or use GUI:

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

### Echo Transform

```bash
rosrun tf tf_echo base_link laser
```

Output:
```
At time t
- Translation: [x, y, z]
- Rotation: [qx, qy, qz, qw]
```

### Static Transform Publisher

For fixed sensor positions:

```bash
rosrun tf static_transform_publisher x y z yaw pitch roll parent_frame child_frame period_ms
```

Example:

```bash
rosrun tf static_transform_publisher 0.1 0 0.05 0 0 0 base_link laser 100
```

## RViz Visualization

### Launch RViz

```bash
rosrun rviz rviz
```

### Add TF Display

1. Click "Add"
2. Select "TF"
3. Configure:
   - Show Axes: Check
   - Frame Timeout: 15
   - Frames: Select relevant ones

## URDF Definition

### Sensor Position in URDF

Located in: `~/catkin_ws/src/your_robot_package/urdf/robot.urdf`

```xml
<robot name="my_robot">
  <link name="base_link"/>

  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.03"/>
      </geometry>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| xyz | Position offset |
| rpy | Roll, Pitch, Yaw (radians) |
| parent | Reference frame |
| child | This sensor's frame |

## Transform Types

### Static Transform

Fixed relationship (sensor to robot):

```xml
<node pkg="tf" type="static_transform_publisher" name="laser_tf"
      args="0.1 0 0.05 0 0 0 base_link laser 100"/>
```

### Dynamic Transform

Changing relationship (robot to map):
- Published by odometry
- Updated by localization

## Sensor Fusion Application

### Why Transforms Matter

To combine LiDAR with other sensors:
1. Know each sensor's position
2. Transform data to common frame
3. Fuse in unified coordinate system

### Example: LiDAR + Camera

```python
import tf

listener = tf.TransformListener()

# Get transform from camera to laser
(trans, rot) = listener.lookupTransform(
    '/camera_link',
    '/laser_link',
    rospy.Time(0)
)

# Transform point from laser to camera frame
# Apply translation and rotation
```

## Troubleshooting

### "Could not find transform"

```bash
# Check if frames exist
rostopic echo /tf | grep frame_id
```

### Old Python Error (Noetic)

If `view_frames` fails:

```bash
# Use tf2 instead
rosrun tf2_tools view_frames.py
```

### Verify Transform Chain

```bash
rosrun tf tf_monitor
```

Shows all transforms and their rates.
