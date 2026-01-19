---
title: "Solving ROS Networking Errors"
date: 2024-07-01
description: "Troubleshooting roscore connection errors in multi-machine ROS setup"
categories: ["Autonomous Driving"]
tags: ["ROS", "Networking", "Troubleshooting"]
draft: false
---

## Overview

ROS networking issues are common when setting up multi-machine systems. This guide addresses the "Unable to contact my own server" error and related problems.

## The Error

```
Unable to contact my own server at [http://192.168.0.4:42767/].
This usually means that the network is not configured properly.
```

## Root Cause

The ROS environment variables are incorrectly configured. Each machine needs:
- `ROS_MASTER_URI`: Points to the ROS Master
- `ROS_HOSTNAME`: This machine's own IP address

## Network Setup Example

### Topology

```
Router (192.168.0.1)
    ├── PC (192.168.0.3) ← ROS Master
    └── Raspberry Pi (192.168.0.4) ← Robot
```

### Correct Configuration

**On PC (Master):**

```bash
export ROS_MASTER_URI=http://192.168.0.3:11311
export ROS_HOSTNAME=192.168.0.3
```

**On Raspberry Pi:**

```bash
export ROS_MASTER_URI=http://192.168.0.3:11311
export ROS_HOSTNAME=192.168.0.4
```

### Common Mistakes

| Mistake | Problem |
|---------|---------|
| Same hostname on both | Nodes can't distinguish machines |
| Wrong master URI | Can't find roscore |
| Localhost on robot | Can't reach across network |
| Wrong IP address | Network unreachable |

## Verification Steps

### Step 1: Check IP Addresses

On each machine:

```bash
hostname -I
```

### Step 2: Verify Connectivity

From PC:
```bash
ping 192.168.0.4
```

From Pi:
```bash
ping 192.168.0.3
```

### Step 3: Check Environment

```bash
echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
```

### Step 4: Test roscore

On master:
```bash
roscore
```

On slave:
```bash
rostopic list
```

## Debugging Commands

### ROS Network Debug

```bash
roswtf
```

Checks for common issues.

### Port Connectivity

```bash
nc -zv 192.168.0.3 11311
```

Should report success.

### Firewall Check

```bash
sudo ufw status
```

If active, allow ROS:

```bash
sudo ufw allow 11311
```

## Error Messages Explained

### "Unable to contact my own server"

ROS_HOSTNAME is wrong for this machine.

### "Unable to connect to master"

- roscore not running
- Wrong ROS_MASTER_URI
- Network/firewall issue

### "ERROR: Unable to communicate with master"

Same as above, check master status.

### "Could not contact ROS master"

Master unreachable from this machine.

## Multi-Machine Checklist

- [ ] Both machines on same network
- [ ] Can ping between machines
- [ ] ROS_MASTER_URI same on both (master's IP)
- [ ] ROS_HOSTNAME different (each machine's own IP)
- [ ] roscore running on master
- [ ] No firewall blocking port 11311
- [ ] Time synchronized (ntpdate)

## Headless Operation

For Raspberry Pi without GUI:

### Verify WiFi Connection

```bash
iwconfig wlan0
```

### Check DHCP

```bash
cat /var/lib/dhcp/dhclient.leases
```

### Static IP (Optional)

Edit netplan:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.0.4/24
      gateway4: 192.168.0.1
```

## Recovery Steps

If everything breaks:

1. Reset to localhost on both machines
2. Test roscore locally
3. Gradually add network configuration
4. Test after each change

```bash
# Temporary reset
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roscore
```
