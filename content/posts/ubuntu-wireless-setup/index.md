---
title: "Ubuntu Server WiFi Configuration"
date: 2024-07-02
description: "Setting up wireless network on Ubuntu 20.04 Server using netplan"
categories: ["Autonomous Driving"]
tags: ["Ubuntu", "Networking", "Turtlebot3", "WiFi"]
draft: false
---

## Overview

Ubuntu Server doesn't include a GUI for network configuration. This guide covers setting up WiFi using netplan on Ubuntu 20.04 Server.

## Netplan Configuration

### Edit Configuration File

```bash
sudo vi /etc/netplan/50-cloud-init.yaml
```

Or use nano:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

### Configuration Structure

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "YOUR_WIFI_NAME":
          password: "YOUR_WIFI_PASSWORD"
```

## Critical Formatting Rules

### Indentation

| Rule | Requirement |
|------|-------------|
| Use spaces | Never tabs |
| Indent level | 2 spaces per level |
| Alignment | `wifis` aligns with `ethernets` |

### YAML Syntax

- Colon after keys
- Quotes around SSID and password
- No trailing spaces

### Example with Proper Indentation

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "MyNetwork":
          password: "MyPassword123"
```

## Apply Configuration

### Apply Changes

```bash
sudo netplan apply
```

### Verify Connection

```bash
ifconfig
```

Or:

```bash
ip addr show wlan0
```

Look for an IP address assigned to wlan0.

## Troubleshooting

### Common Errors

**YAML syntax error:**
```
Error in network definition: unknown key 'wlan0'
```

Solution: Check indentation is correct.

**Network not found:**

```bash
# Scan for networks
sudo iwlist wlan0 scan | grep ESSID
```

Verify SSID matches exactly.

### Debug Mode

```bash
sudo netplan --debug apply
```

Shows detailed error messages.

### Check Interface Status

```bash
ip link show wlan0
```

Should show `UP` state.

## Static IP Configuration

For fixed IP instead of DHCP:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      addresses:
        - 192.168.0.4/24
      gateway4: 192.168.0.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
      access-points:
        "MyNetwork":
          password: "MyPassword123"
```

## Multiple Networks

Configure backup networks:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "HomeNetwork":
          password: "home123"
        "LabNetwork":
          password: "lab456"
```

Connects to first available.

## Security Considerations

### File Permissions

The configuration contains passwords:

```bash
sudo chmod 600 /etc/netplan/50-cloud-init.yaml
```

### Hidden Networks

```yaml
access-points:
  "HiddenNetwork":
    hidden: true
    password: "secret"
```

## After Connection

### Verify Internet

```bash
ping -c 3 google.com
```

### Check DNS

```bash
nslookup google.com
```

### Get IP Info

```bash
hostname -I
```
