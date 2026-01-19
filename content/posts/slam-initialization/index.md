---
title: "SLAM Initialization Commands for TurtleBot3"
date: 2024-07-04
description: "Step-by-step guide to launching SLAM on TurtleBot3"
categories: ["Autonomous Driving"]
tags: ["SLAM", "Turtlebot3", "ROS", "Mapping"]
draft: false
---

## Overview

SLAM (Simultaneous Localization and Mapping) allows a robot to build a map while tracking its position. This guide covers the initialization commands for TurtleBot3.

## System Architecture

```
┌─────────────────────────────────┐
│              PC                  │
│  ┌─────────┐  ┌──────────────┐  │
│  │ roscore │  │ SLAM + RViz  │  │
│  └────┬────┘  └──────┬───────┘  │
│       │              │          │
└───────┼──────────────┼──────────┘
        │    WiFi      │
┌───────┼──────────────┼──────────┐
│       │              │          │
│  ┌────┴────────┐     │          │
│  │  Bringup   │     │          │
│  │ (sensors)  │←────┘          │
│  └────────────┘                 │
│         TurtleBot3              │
└─────────────────────────────────┘
```

## Step 1: Start ROS Master (PC)

```bash
roscore
```

Expected output:

```
... logging to /home/user/.ros/log/...
started roslaunch server http://192.168.0.3:xxxxx/
ros_comm version 1.16.0

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES

auto-starting new master
process[master]: started with pid [xxxx]
ROS_MASTER_URI=http://192.168.0.3:11311
...
```

## Step 2: Launch Robot (TurtleBot3)

SSH into TurtleBot3:

```bash
ssh ubuntu@<TURTLEBOT_IP>
```

Set model and launch:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Expected output:

```
SUMMARY
========

PARAMETERS
...

NODES
  /
    turtlebot3_core (rosserial_python/serial_node.py)
    turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
    turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

...
[INFO] Calibration End
```

Key nodes:
- **turtlebot3_core**: Serial communication with OpenCR
- **turtlebot3_lds**: Laser scanner driver
- **turtlebot3_diagnostics**: System health monitoring

## Step 3: Launch SLAM (PC)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

This launches:
- SLAM algorithm (gmapping by default)
- RViz for visualization

### Alternative SLAM Methods

```bash
# Gmapping (default)
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

# Cartographer
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer

# Hector SLAM
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
```

## Step 4: Launch Teleop (PC)

In a new terminal:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Control keys:

```
        w
   a    s    d
        x

w/x: Increase/decrease linear velocity
a/d: Increase/decrease angular velocity
s: Stop
CTRL+C: Quit
```

## Verification

### Check Topics

```bash
rostopic list
```

Important topics:
- `/scan`: Laser data
- `/odom`: Odometry
- `/map`: Generated map
- `/cmd_vel`: Velocity commands

### Monitor Laser

```bash
rostopic echo /scan
```

### View TF Tree

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

## Saving the Map

After exploring:

```bash
rosrun map_server map_saver -f ~/map
```

Creates:
- `map.pgm`: Image file
- `map.yaml`: Metadata

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No laser data | Check LDS connection |
| Robot not moving | Verify OpenCR power |
| Map drifting | Move slower, better loop closure |
| RViz not showing | Check Fixed Frame = "map" |

## Complete Command Summary

| Terminal | Machine | Command |
|----------|---------|---------|
| 1 | PC | `roscore` |
| 2 | TB3 (SSH) | `roslaunch turtlebot3_bringup turtlebot3_robot.launch` |
| 3 | PC | `roslaunch turtlebot3_slam turtlebot3_slam.launch` |
| 4 | PC | `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` |
