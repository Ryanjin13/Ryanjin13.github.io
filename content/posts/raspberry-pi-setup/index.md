---
title: "Raspberry Pi 3B+ OS Setup"
date: 2024-07-01
description: "Setting up operating system on Raspberry Pi for robotics"
categories: ["Autonomous Driving"]
tags: ["Raspberry Pi", "Turtlebot3", "Linux", "Setup"]
draft: false
---

## Overview

Setting up a Raspberry Pi for robotics projects requires proper OS installation. This guide covers the basic setup process using Raspberry Pi Imager.

## Requirements

### Hardware

- Raspberry Pi 3B+ (or compatible model)
- microSD card (16GB+ recommended)
- USB card reader
- HDMI monitor
- USB keyboard and mouse
- Power supply (5V 2.5A)

### Software

- Raspberry Pi Imager (download from raspberrypi.org)
- Computer for image writing

## Installation Steps

### Step 1: Download Raspberry Pi Imager

Download from official website:
- Windows, macOS, or Linux versions available
- Simple installer process

### Step 2: Prepare SD Card

1. Insert microSD card into reader
2. Connect reader to computer
3. Launch Raspberry Pi Imager

### Step 3: Select Operating System

For TurtleBot3/ROS:
- **Recommended:** Ubuntu Server 20.04 (64-bit)
- Alternative: Raspberry Pi OS (for testing)

```
Raspberry Pi Imager
┌─────────────────────────────────────┐
│  CHOOSE OS                          │
│  ┌─────────────────────────────┐    │
│  │ Ubuntu Server 20.04 (64-bit)│    │
│  │ Other Ubuntu versions...    │    │
│  │ Raspberry Pi OS (32-bit)    │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

### Step 4: Select Storage

Choose the microSD card:
- Verify correct drive selected
- All data will be erased

### Step 5: Configure Settings (Optional)

Click gear icon for advanced options:
- Set hostname
- Enable SSH
- Configure WiFi
- Set username/password

```
Advanced Options
├── Hostname: turtlebot
├── Enable SSH: Yes
├── WiFi SSID: your_network
├── WiFi Password: ********
└── Username: ubuntu
```

### Step 6: Write Image

1. Click "Write"
2. Confirm data erasure
3. Wait for completion
4. Verify write

## First Boot

### Connect Hardware

1. Insert SD card into Pi
2. Connect monitor via HDMI
3. Connect keyboard
4. Connect power (boot starts)

### Initial Login

Default credentials (Ubuntu):
- Username: ubuntu
- Password: ubuntu

You'll be prompted to change password on first login.

### Network Setup

If WiFi wasn't configured:

```bash
# Edit netplan configuration
sudo nano /etc/netplan/50-cloud-init.yaml
```

Add WiFi configuration:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "your_network":
          password: "your_password"
```

Apply changes:

```bash
sudo netplan apply
```

### Enable SSH

```bash
sudo systemctl enable ssh
sudo systemctl start ssh
```

## Post-Installation

### System Update

```bash
sudo apt update
sudo apt upgrade -y
```

### Install Essential Tools

```bash
sudo apt install -y vim git curl wget
```

### Check IP Address

```bash
ip addr show wlan0
```

Note the IP for remote connection.

## Remote Access

### SSH from Desktop

```bash
ssh ubuntu@<raspberry_pi_ip>
```

### Headless Operation

After initial setup, no monitor needed:
- Power on
- Auto-connects to WiFi
- Access via SSH

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No display | Check HDMI connection, try different cable |
| No WiFi | Verify credentials, check signal strength |
| Boot loop | Re-flash SD card, check power supply |
| Slow boot | Normal for first boot, patience needed |

## Next Steps

1. Install ROS (ros-noetic-ros-base)
2. Install TurtleBot3 packages
3. Configure ROS networking
4. Test communication with desktop
