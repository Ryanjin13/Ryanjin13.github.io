---
title: "ROS1 Setup and Version Compatibility"
date: 2024-07-01
description: "Understanding ROS1 installation and Ubuntu version requirements"
categories: ["Autonomous Driving"]
tags: ["ROS", "Turtlebot3", "Robotics", "Ubuntu"]
draft: false
---

## Overview

Robot Operating System (ROS) requires specific Ubuntu versions for each release. Understanding these compatibility requirements is essential for successful robotics development.

## Version Compatibility

### ROS1 Distributions

Each ROS version requires a compatible Ubuntu version:

| ROS Version | Ubuntu Version | End of Life |
|-------------|----------------|-------------|
| Noetic | 20.04 (Focal) | May 2025 |
| Melodic | 18.04 (Bionic) | May 2023 |
| Kinetic | 16.04 (Xenial) | April 2021 |
| Indigo | 14.04 (Trusty) | April 2019 |

## Two-Device Setup

### Desktop PC

Primary development machine:
- Full Ubuntu installation
- ROS development tools
- Simulation (Gazebo, RViz)
- Larger storage and memory

### Embedded System (TurtleBot3)

Robot's onboard computer:
- Raspberry Pi or similar
- Lightweight Ubuntu version
- ROS communication nodes
- Hardware drivers

## Hardware Considerations

### Embedded System Limitations

| Factor | Constraint |
|--------|------------|
| ARM vs x86 | Different packages |
| RAM (1-4 GB) | Limited simultaneous nodes |
| Storage | Minimal installation |
| Compute power | Offload heavy processing |

### Version Selection Strategy

1. Check embedded system's Ubuntu support
2. Match ROS version to that Ubuntu
3. Install same ROS on desktop
4. Ensure network compatibility

## Installation Overview

### Desktop Installation

```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install
sudo apt update
sudo apt install ros-noetic-desktop-full

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Embedded System Installation

Minimal installation for Raspberry Pi:

```bash
# Install ROS base (no GUI)
sudo apt install ros-noetic-ros-base

# Install TurtleBot3 packages
sudo apt install ros-noetic-turtlebot3
```

## Network Configuration

### ROS Master Setup

On desktop (master):
```bash
export ROS_MASTER_URI=http://<desktop_ip>:11311
export ROS_HOSTNAME=<desktop_ip>
```

On robot:
```bash
export ROS_MASTER_URI=http://<desktop_ip>:11311
export ROS_HOSTNAME=<robot_ip>
```

### Communication Check

```bash
# On desktop
roscore

# On robot
rostopic list
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Version mismatch | Use same ROS on both |
| Network unreachable | Check firewall, IPs |
| Package not found | Check architecture (ARM/x86) |
| Permission denied | Add user to dialout group |

## TurtleBot3 Specific

### Supported Configurations

| Robot Model | Recommended ROS |
|-------------|-----------------|
| Burger | Noetic (20.04) or Melodic (18.04) |
| Waffle | Noetic (20.04) or Melodic (18.04) |
| Waffle Pi | Noetic (20.04) or Melodic (18.04) |

### Quick Start

```bash
# Set model
export TURTLEBOT3_MODEL=burger

# Launch robot
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# On desktop - teleop
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
