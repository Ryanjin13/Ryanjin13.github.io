---
title: "ROS Noetic Installation Guide"
date: 2024-07-01
description: "Complete ROS Noetic installation and TurtleBot3 setup on Ubuntu 20.04"
categories: ["Autonomous Driving"]
tags: ["ROS", "Noetic", "Turtlebot3", "Installation"]
draft: false
---

## Overview

This guide provides a complete ROS Noetic installation procedure for Ubuntu 20.04, including TurtleBot3 packages and workspace setup.

## Clean Previous Installation

If ROS was previously installed:

```bash
sudo apt-get remove ros-*
sudo apt-get autoremove
sudo rm -rf /etc/ros
```

## ROS Noetic Installation

### Configure Repository

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set Up Keys

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### Update Package Index

```bash
sudo apt update
```

### Install ROS Desktop Full

```bash
sudo apt install ros-noetic-desktop-full
```

This includes:
- ROS core
- rqt tools
- RViz
- Gazebo
- Robot-generic libraries

### Install Additional Dependencies

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

## Environment Setup

### Add to Bashrc

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
rosversion ros
```

Should output: `noetic`

## TurtleBot3 Packages

### Install Core Packages

```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```

### Install Simulation (Optional)

```bash
sudo apt install ros-noetic-turtlebot3-simulations
```

### Set Robot Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

Options: `burger`, `waffle`, `waffle_pi`

## Create Catkin Workspace

### Initialize Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### Source Workspace

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verification

### Check ROS Version

```bash
rosversion -d
```

Output: `noetic`

### Test with Simulation

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

This should open Gazebo with TurtleBot3 in a simulation world.

### Test Teleop

In another terminal:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Use WASD keys to control robot.

## Complete Bashrc Configuration

```bash
# ROS Noetic
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# ROS Network (adjust IPs)
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# TurtleBot3
export TURTLEBOT3_MODEL=burger

# Gazebo (optional)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```

## Common Issues

### Package Not Found

```bash
sudo apt update
sudo apt install ros-noetic-<package-name>
```

### Gazebo Slow/Crash

Lower physics update rate or use simpler world.

### rosdep Errors

```bash
sudo rosdep fix-permissions
rosdep update
```

## Summary

Installation completed successfully! The system is ready for:
- Real robot control
- Simulation testing
- ROS development
