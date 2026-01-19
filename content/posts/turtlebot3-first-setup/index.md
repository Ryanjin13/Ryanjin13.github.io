---
title: "TurtleBot3 First Setup Procedure"
date: 2024-07-03
description: "Complete first-time setup guide for TurtleBot3 with ROS Noetic"
categories: ["Autonomous Driving"]
tags: ["Turtlebot3", "ROS", "Noetic", "Setup"]
draft: false
---

## Overview

This guide covers the complete first-time setup procedure for TurtleBot3, from WiFi configuration to keyboard control testing.

## Prerequisites

- PC with Ubuntu 20.04
- Raspberry Pi with Ubuntu 20.04 Server
- TurtleBot3 robot (Burger/Waffle)
- Both connected to same WiFi network

## Step 1: WiFi Configuration (Raspberry Pi)

Edit netplan:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Add WiFi configuration (use spaces, not tabs):

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YOUR_WIFI":
          password: "YOUR_PASSWORD"
```

Apply and verify:

```bash
sudo netplan apply
ifconfig
```

## Step 2: ROS Noetic Installation (PC)

### Add Repository

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### Install ROS

```bash
sudo apt install ros-noetic-desktop-full
```

### Environment Setup

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Create Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Step 3: TurtleBot3 Packages

### Install on PC and Pi

```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```

### Simulation (PC only)

```bash
sudo apt install ros-noetic-turtlebot3-simulations
```

### Set Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: LDS-02 LIDAR Setup (Pi)

If using LDS-02 laser:

```bash
sudo apt install ros-noetic-hls-lfcd-lds-driver
```

Update dependencies:

```bash
cd ~/catkin_ws/src
git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/catkin_ws
catkin_make
```

If directory error occurs:

```bash
mkdir -p ~/catkin_ws/src
```

## Step 5: OpenCR Setup (Pi)

### Add Architecture Support

```bash
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf
```

### Flash Firmware

```bash
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

## Step 6: Bringup Test

### Terminal 1: PC (Master)

```bash
roscore
```

### Terminal 2: Pi (SSH)

```bash
ssh ubuntu@<PI_IP>
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Expected output:

```
[turtlebot3_core-1] process has finished
SUMMARY
========

PARAMETERS
...
Calibration End
```

### Terminal 3: PC (Teleop)

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Control keys:
- W: Forward
- X: Backward
- A: Left turn
- D: Right turn
- S: Stop

## Known Issues

### Backward Motion

Some units have issues with backward movement. Check motor configuration if occurs.

### LDS Not Spinning

Check power connection to LIDAR.

### Connection Timeout

Verify ROS_MASTER_URI and ROS_HOSTNAME on both machines.

## Verification Checklist

- [ ] WiFi connected on Pi
- [ ] roscore running on PC
- [ ] Robot launch successful on Pi
- [ ] Teleop controlling robot
- [ ] LIDAR data visible (rostopic echo /scan)
