---
title: "ROS Master-Slave Connection Setup"
date: 2024-07-01
description: "Setting up ROS communication between Raspberry Pi and PC"
categories: ["Autonomous Driving"]
tags: ["ROS", "Turtlebot3", "Networking"]
draft: false
---

## Overview

ROS uses a master-slave architecture where one machine runs the ROS Master (roscore) and other machines connect to it. This guide covers setting up communication between a PC and Raspberry Pi.

## Network Architecture

```
        WiFi Router
        ↙        ↘
   Desktop PC    Raspberry Pi
  (ROS Master)    (Robot)
  192.168.0.3    192.168.0.4
```

## Prerequisites

- Both devices connected to same WiFi network
- Ubuntu/ROS installed on both machines
- SSH access to Raspberry Pi

## Raspberry Pi Setup

### Check Network Configuration

```bash
ifconfig
```

Note the IP address (e.g., 192.168.0.4).

### SSH into Raspberry Pi

From PC:
```bash
ssh pi@192.168.0.4
```

Or for Ubuntu:
```bash
ssh ubuntu@192.168.0.4
```

### Time Synchronization

Important for ROS communication:

```bash
sudo apt-get install ntpdate
sudo ntpdate ntp.ubuntu.com
```

### Configure ROS Environment

Edit bashrc:

```bash
nano ~/.bashrc
```

Add at the end:

```bash
export ROS_MASTER_URI=http://192.168.0.3:11311
export ROS_HOSTNAME=192.168.0.4
```

Apply changes:

```bash
source ~/.bashrc
```

## PC (Master) Setup

### Configure ROS Environment

Edit bashrc:

```bash
nano ~/.bashrc
```

Add:

```bash
export ROS_MASTER_URI=http://192.168.0.3:11311
export ROS_HOSTNAME=192.168.0.3
```

Apply changes:

```bash
source ~/.bashrc
```

## Environment Variables Explained

| Variable | Purpose | Value |
|----------|---------|-------|
| ROS_MASTER_URI | Location of ROS Master | Master's IP:11311 |
| ROS_HOSTNAME | This machine's IP | Own IP address |

### Common Mistake

```bash
# WRONG - hostname doesn't match machine
export ROS_HOSTNAME=192.168.0.3  # On Pi with IP .4
```

Each machine's ROS_HOSTNAME must be its own IP!

## Testing Connection

### On PC (Master)

Start ROS Master:

```bash
roscore
```

### On Raspberry Pi

List topics to verify connection:

```bash
rostopic list
```

Should show at least:
```
/rosout
/rosout_agg
```

### Publish Test

On Pi:
```bash
rostopic pub /test std_msgs/String "Hello from Pi"
```

On PC:
```bash
rostopic echo /test
```

Should see: `data: "Hello from Pi"`

## Troubleshooting

### "Unable to contact my own server"

```
Unable to contact my own server at [http://192.168.0.4:42767/]
```

**Solution:** Check ROS_HOSTNAME is set correctly on each machine.

### "Cannot reach master"

**Solutions:**
1. Verify roscore is running on master
2. Check firewall settings
3. Ping between machines
4. Verify same network

### Ping Test

```bash
# From PC
ping 192.168.0.4

# From Pi
ping 192.168.0.3
```

### Firewall

If using UFW:

```bash
sudo ufw allow 11311
sudo ufw allow 11311/tcp
```

## Permanent Configuration

### Bashrc vs Environment

For permanent setup, bashrc is recommended:

```bash
# ~/.bashrc additions
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.3:11311
export ROS_HOSTNAME=192.168.0.4  # Change per machine
export TURTLEBOT3_MODEL=burger
```

### Dynamic IP Handling

If IPs change (DHCP), update bashrc or use:

```bash
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')
```
