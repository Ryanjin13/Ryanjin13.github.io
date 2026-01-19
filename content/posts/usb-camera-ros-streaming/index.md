---
title: "USB Camera Streaming with ROS"
date: 2024-07-08
description: "Streaming USB camera from Raspberry Pi to PC with RViz visualization"
categories: ["Autonomous Driving"]
tags: ["ROS", "Camera", "Raspberry Pi", "RViz"]
draft: false
---

## Overview

This guide covers setting up a USB camera on Raspberry Pi and streaming the video to a PC for visualization in RViz.

## System Architecture

```
┌─────────────────────┐         ┌─────────────────────┐
│   Raspberry Pi      │         │        PC           │
│  ┌───────────────┐  │         │  ┌───────────────┐  │
│  │  USB Camera   │  │  WiFi   │  │     RViz      │  │
│  └───────┬───────┘  │ ──────► │  │  Visualization│  │
│  ┌───────┴───────┐  │         │  └───────────────┘  │
│  │   usb_cam     │  │         │                     │
│  │   ros node    │  │         │                     │
│  └───────────────┘  │         │                     │
└─────────────────────┘         └─────────────────────┘
```

## Raspberry Pi Setup

### Install ROS Noetic

```bash
# Add repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Add key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install
sudo apt update
sudo apt install ros-noetic-ros-base
```

### Install USB Camera Package

```bash
sudo apt install ros-noetic-usb-cam
```

### Create Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Create Launch File

Create `~/catkin_ws/src/usb_cam.launch`:

```xml
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0"/>
    <param name="image_width" value="640"/>
    <param name="image_height" value="480"/>
    <param name="pixel_format" value="yuyv"/>
    <param name="camera_frame_id" value="usb_cam"/>
    <param name="io_method" value="mmap"/>
  </node>
</launch>
```

### Configure Environment

Add to `~/.bashrc`:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PI_IP>
```

Apply:

```bash
source ~/.bashrc
```

## PC Setup

### Install ROS Noetic

```bash
sudo apt install ros-noetic-desktop-full
```

### Configure Environment

Add to `~/.bashrc`:

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>
```

## Running the System

### Terminal 1: PC - roscore

```bash
roscore
```

### Terminal 2: Pi (SSH) - Camera

```bash
roslaunch usb_cam usb_cam.launch
```

Or use the custom launch file:

```bash
roslaunch ~/catkin_ws/src/usb_cam.launch
```

### Terminal 3: PC - RViz

```bash
rosrun rviz rviz
```

### Add Camera Display in RViz

1. Click "Add" button
2. Select "Image"
3. Set Image Topic: `/usb_cam/image_raw`
4. Image should appear in RViz

## Verify Image Stream

### Check Topic

```bash
rostopic list | grep image
```

Should show:
```
/usb_cam/image_raw
```

### Check Bandwidth

```bash
rostopic bw /usb_cam/image_raw
```

### View with image_view

```bash
rosrun image_view image_view image:=/usb_cam/image_raw
```

## Troubleshooting

### Camera Not Found

```bash
# Check device
ls /dev/video*

# Test camera
v4l2-ctl --list-devices
```

### Permission Denied

```bash
sudo chmod 666 /dev/video0
# Or add user to video group
sudo usermod -a -G video $USER
```

### Low Frame Rate

Reduce resolution or use compressed topic:

```bash
rostopic echo /usb_cam/image_raw/compressed
```

## Advanced Options

### Camera Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| video_device | Camera device | /dev/video0 |
| image_width | Frame width | 640 |
| image_height | Frame height | 480 |
| framerate | Frames per second | 30 |
| pixel_format | Color format | yuyv |
