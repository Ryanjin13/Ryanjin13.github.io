---
title: "Day 2 — Linux Fundamentals and Boot Sequence"
date: 2026-03-07
description: "RPi 5 boot sequence from EEPROM to systemd, filesystem hierarchy, process model, and shell scripting essentials"
categories: ["Autonomous Driving"]
tags: ["Linux", "Boot Sequence", "systemd", "Process Management", "Shell Scripting"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 2
draft: false
---

{{< katex >}}

## What You'll Learn

Yesterday we explored the hardware: BCM2712, RP1, ARM Cortex-A76. Today we cross into the **software** layer. Every autonomous car running on Linux needs a rock-solid understanding of what happens from the moment power is applied to the moment your perception stack starts running.

By the end of this post, you will:

- Trace the complete RPi 5 boot sequence: EEPROM -> bootloader -> kernel -> systemd
- Navigate the Linux filesystem hierarchy with confidence
- Understand the process model: fork/exec, PID, parent-child, zombies
- Write udev rules for automatic device configuration
- Create systemd services to auto-start your autonomous car software
- Write practical shell scripts for automation

---

## 1. RPi 5 Boot Sequence — From Power-On to Login Prompt

### 1.1 Overview

When you plug in the USB-C cable and power reaches the BCM2712, a carefully orchestrated sequence begins. Let's trace every stage.

```
Power On
    |
    v
+------------------+
| Stage 1: EEPROM  |  BCM2712 internal ROM loads EEPROM bootloader
| Bootloader       |  Initializes LPDDR4X RAM, finds boot media
+--------+---------+
         |
         v
+------------------+
| Stage 2: start4  |  VideoCore VII firmware (GPU boots first!)
| (GPU firmware)   |  Reads config.txt, loads kernel + DTB
+--------+---------+
         |
         v
+------------------+
| Stage 3: Linux   |  Kernel initializes hardware, mounts rootfs
| Kernel           |  Launches PID 1 (systemd)
+--------+---------+
         |
         v
+------------------+
| Stage 4: systemd |  Starts services in dependency order
| (PID 1)          |  Network, SSH, your custom services
+--------+---------+
         |
         v
    Login prompt / SSH ready
```

### 1.2 Stage 1: EEPROM Bootloader

The BCM2712 has a small **boot ROM** burned into the silicon. When power is applied:

1. The boot ROM executes and reads the **SPI EEPROM** on the Pi 5 board
2. The EEPROM contains the **first-stage bootloader** — a small program that:
   - Initializes the LPDDR4X memory controller and trains the RAM
   - Scans for boot media: SD card, USB, NVMe, network (PXE)
   - Reads the boot partition (FAT32) from the selected media

**Key difference from Pi 4:** Pi 5 stores its bootloader in a **dedicated SPI EEPROM** chip, separate from the SD card. This means:
- The bootloader can be updated independently (`sudo rpi-eeprom-update`)
- Boot configuration persists even if you swap SD cards
- USB and NVMe boot work without any SD card hacks

```bash
# Check current EEPROM version
sudo rpi-eeprom-update

# View EEPROM configuration
sudo rpi-eeprom-config

# Key settings:
# BOOT_ORDER=0xf416    (try NVMe, then USB, then SD)
# BOOT_UART=1          (enable UART debug during boot)
# POWER_OFF_ON_HALT=1  (actually cut power on shutdown)
```

The `BOOT_ORDER` value is read right-to-left as a sequence of nibbles:
- `0xf416` means: try SD card (1), then NVMe (6), then USB (4), then stop (f)
- Each nibble represents a boot device: 1=SD, 2=Network, 4=USB, 5=BCM-USB, 6=NVMe, f=stop

To change boot order (for example, to boot from NVMe first):

```bash
sudo rpi-eeprom-config --edit
# Change to NVMe-first: BOOT_ORDER=0xf146
# Read right-to-left: 6=NVMe, 4=USB, 1=SD, f=stop
```

### 1.3 Stage 2: GPU Firmware (start4.elf)

Here is something surprising: **the GPU boots before the CPU**.

The VideoCore VII GPU loads and executes `start4.elf` from the boot partition. This firmware:

1. Reads `config.txt` — the Pi's "BIOS settings" file
2. Applies hardware configuration: memory split, clock speeds, display settings, overlays
3. Loads the **Device Tree Blob (DTB)** — `bcm2712-rpi-5-b.dtb` — which describes all hardware to the kernel
4. Applies any Device Tree Overlays specified in `config.txt`
5. Loads the Linux kernel (`kernel_2712.img`) into RAM
6. Optionally loads an initramfs
7. **Releases the ARM cores** from reset, pointing them at the kernel entry point

```
Boot Partition (/boot/firmware/):
|
|-- start4.elf           GPU firmware (loads and runs on VideoCore VII)
|-- fixup4.dat           GPU firmware fixup data
|-- config.txt           Hardware configuration ("BIOS settings")
|-- cmdline.txt          Kernel command line parameters
|-- kernel_2712.img      Linux kernel for BCM2712
|-- bcm2712-rpi-5-b.dtb  Device Tree Blob
|-- overlays/            Device Tree Overlays (enable specific hardware)
|   |-- imx219.dtbo      Camera Module v2
|   |-- imx708.dtbo      Camera Module v3
|   |-- i2c-sensor.dtbo  I2C sensor overlays
|   |-- spi0-1cs.dtbo    SPI configuration
|   |-- pwm-2chan.dtbo    Hardware PWM
```

**Important `config.txt` settings for autonomous driving:**

```ini
# /boot/firmware/config.txt

# --- Serial/Debug ---
enable_uart=1              # Enable hardware UART for debug console

# --- Sensor Interfaces ---
dtparam=i2c_arm=on         # Enable I2C bus 1 (for IMU, sensors)
dtparam=spi=on             # Enable SPI bus 0 (for additional peripherals)

# --- Camera ---
# Uncomment one based on your camera module:
# dtoverlay=imx219         # Camera Module v2 (8MP)
# dtoverlay=imx708         # Camera Module v3 (12MP)

# --- PCIe (for Hailo AI accelerator) ---
# dtoverlay=pciex1-compat-pi5,no-mip

# --- GPU Memory (headless = minimize GPU allocation) ---
gpu_mem=128

# --- Performance ---
# arm_freq=2600            # Overclock (requires good cooling!)
# over_voltage=4           # Increase voltage for overclock stability
```

### 1.4 Stage 3: Linux Kernel Boot

Once the ARM Cortex-A76 cores are released from reset, the kernel takes over:

1. **Early init**: Sets up the MMU (memory management unit), page tables, exception vectors
2. **Hardware probing**: Uses the Device Tree to discover and initialize hardware:
   - Memory controller configuration
   - Interrupt controller (GIC-400)
   - PCIe controller -> RP1 enumeration
   - Timer, watchdog, RNG
3. **Driver initialization**: Loads built-in drivers for storage, filesystem, network
4. **Root filesystem mount**: Finds and mounts the ext4 partition as `/`
5. **PID 1 launch**: Executes `/sbin/init`, which on modern systems is a symlink to `systemd`

The kernel command line (`cmdline.txt`) tells the kernel critical information:

```
console=serial0,115200 console=tty1 root=PARTUUID=xxxx-02 rootfstype=ext4
fsck.repair=yes rootwait quiet splash
```

Breaking this down:

| Parameter | Meaning |
|-----------|---------|
| `console=serial0,115200` | Send boot messages to UART at 115200 baud |
| `console=tty1` | Also display on HDMI |
| `root=PARTUUID=xxxx-02` | Root filesystem partition (by UUID) |
| `rootfstype=ext4` | Filesystem type |
| `fsck.repair=yes` | Auto-repair filesystem errors |
| `rootwait` | Wait for root device to appear (important for slow SD cards) |
| `quiet` | Suppress most boot messages |
| `splash` | Show splash screen instead of text |

**For debugging, remove `quiet splash`** to see all boot messages. This is invaluable when something goes wrong.

### 1.5 Stage 4: systemd — The Service Manager

`systemd` is PID 1 — the **first userspace process** and the ancestor of all other processes.

```
systemd (PID 1)
    |
    |-- systemd-journald     (centralized logging)
    |-- systemd-udevd        (device manager)
    |-- systemd-networkd     (networking)
    |-- sshd                 (SSH server)
    |-- getty@tty1           (console login)
    |-- your-autocar.service (YOUR custom service!)
    |-- ...
```

systemd starts services based on a **dependency graph**, not a simple linear sequence. Services declare what they need (e.g., "start after network is up") and systemd resolves the optimal parallel startup order. This is much faster than the old SysVinit sequential approach.

**Key systemd concepts:**

| Concept | Description |
|---------|-------------|
| **Unit** | A thing systemd manages (service, mount, timer, socket, device) |
| **Service** | A daemon/program to run (`Type=simple`, `Type=forking`, `Type=oneshot`) |
| **Target** | A grouping of units (like a "runlevel"): `multi-user.target`, `graphical.target` |
| **Dependency** | `Requires=`, `After=`, `Wants=`, `Before=` — ordering and hard/soft requirements |
| **Journal** | Centralized binary logging via `journalctl` |

**Target hierarchy (boot progression):**

```
sysinit.target          (early system initialization)
    |
basic.target            (basic system ready)
    |
network.target          (network interfaces configured)
    |
network-online.target   (network actually connected)
    |
multi-user.target       (full multi-user, no GUI -- this is our target)
    |
graphical.target        (desktop environment -- not used on headless car)
```

**Essential systemd commands:**

```bash
# See overall system state
systemctl status

# List all active services
systemctl list-units --type=service

# Check a specific service
systemctl status sshd

# Start/stop/restart a service
sudo systemctl start myservice
sudo systemctl stop myservice
sudo systemctl restart myservice

# Enable/disable auto-start at boot
sudo systemctl enable myservice
sudo systemctl disable myservice

# View logs
journalctl -b                          # Current boot
journalctl -b -1                       # Previous boot
journalctl -u sshd                     # Specific service
journalctl -u autocar-monitor -f       # Follow live (like tail -f)
journalctl --since "10 minutes ago"    # Time-based filter
journalctl -p err                      # Only errors

# Boot timing analysis
systemd-analyze                        # Total boot time
systemd-analyze blame | head -20       # Slowest services
systemd-analyze critical-chain         # Critical path
systemd-analyze plot > boot.svg        # Visual boot chart

# Dependency inspection
systemctl list-dependencies multi-user.target
systemctl list-dependencies --reverse sshd  # What depends on sshd?
```

**Boot time analysis is critical for autonomous cars.** If your car takes 30 seconds to boot, that is 30 seconds of blindness after power cycling. Let's identify and eliminate slow services:

```bash
# See what's slow
systemd-analyze blame | head -15

# Common culprits on Pi 5 (and how to fix them):
# dhcpcd.service (10s) -- waiting for DHCP lease
#   Fix: use static IP, or NetworkManager with quick-connect
# apt-daily.service (5s) -- package update check
#   Fix: sudo systemctl disable apt-daily.timer
# man-db.service (3s) -- rebuild man page cache
#   Fix: sudo systemctl disable man-db.timer
# bluetooth.service (2s) -- Bluetooth stack
#   Fix: sudo systemctl disable bluetooth (if not needed)
```

---

## 2. Filesystem Hierarchy

### 2.1 The Standard Directory Structure

Linux follows the **Filesystem Hierarchy Standard (FHS)**. Here is what each directory does and why it matters for embedded development:

```
/
|-- bin/           Essential user binaries (ls, cp, mv, cat, bash)
|-- boot/          Boot files
|   |-- firmware/  FAT32 boot partition (kernel, DTB, config.txt)
|-- dev/           Device files (hardware as files)
|-- etc/           System-wide configuration files
|   |-- systemd/   systemd configuration
|   |-- udev/      udev rules
|   |-- ssh/       SSH server config
|-- home/          User home directories
|   |-- pi/        Your working directory
|-- lib/           Shared libraries and kernel modules
|-- mnt/           Temporary mount points
|-- opt/           Optional add-on software
|-- proc/          Virtual FS: process and kernel info (generated live)
|-- root/          Root user home directory
|-- run/           Runtime data (PIDs, sockets, cleared each boot)
|-- sbin/          System binaries (systemctl, fdisk, ip, reboot)
|-- sys/           Virtual FS: hardware/driver info (generated live)
|-- tmp/           Temporary files (may be tmpfs in RAM)
|-- usr/           User programs and libraries
|   |-- bin/       Most user commands
|   |-- lib/       Libraries
|   |-- local/     Locally installed software
|-- var/           Variable data
    |-- log/       System logs
    |-- cache/     Application caches
```

### 2.2 /dev — Device Files

In Linux, **everything is a file** — including hardware. The `/dev` directory contains special files that represent devices:

```bash
# Block devices (storage)
ls -la /dev/mmcblk0*
# mmcblk0    -- the entire SD card
# mmcblk0p1  -- boot partition (FAT32)
# mmcblk0p2  -- root partition (ext4)

# Character devices (serial, GPIO, sensors)
ls -la /dev/ttyAMA*      # UART ports (via RP1)
ls -la /dev/gpiochip*    # GPIO controllers (gpiochip4 = RP1)
ls -la /dev/i2c-*        # I2C buses
ls -la /dev/spidev*      # SPI devices
ls -la /dev/video*       # Camera (V4L2 interface)

# Special devices
ls -la /dev/null          # Black hole: discards anything written
ls -la /dev/zero          # Infinite source of zero bytes
ls -la /dev/urandom       # Random bytes (uses hardware RNG on Pi 5)
ls -la /dev/mem           # Physical memory access (dangerous!)
```

**Key device files for autonomous driving:**

| Device | Path | Purpose |
|--------|------|---------|
| Camera | `/dev/video0` | V4L2 camera interface |
| UART | `/dev/ttyAMA0` | Serial debug console / sensor comms |
| I2C | `/dev/i2c-1` | Sensor bus (IMU, magnetometer, etc.) |
| SPI | `/dev/spidev0.0` | High-speed peripheral interface |
| GPIO | `/dev/gpiochip4` | RP1 GPIO (used by libgpiod) |
| NVMe | `/dev/nvme0n1` | NVMe SSD (if attached via PCIe) |

### 2.3 /proc — Process and Kernel Virtual Filesystem

`/proc` is not a real filesystem on disk — the kernel generates its contents dynamically. It is a window into the running kernel and all processes.

```bash
# System-wide information
cat /proc/cpuinfo        # CPU details per core
cat /proc/meminfo        # Detailed memory statistics
cat /proc/version        # Kernel version string
cat /proc/cmdline        # Kernel command line (from cmdline.txt)
cat /proc/uptime         # Uptime in seconds (and idle time)
cat /proc/loadavg        # CPU load averages: 1, 5, 15 minutes

# Interrupt information (crucial for real-time debugging!)
cat /proc/interrupts
# Shows interrupt counts per CPU core per device
# If one core has way more interrupts, you have an IRQ affinity issue

# I/O memory map (where hardware registers are in physical address space)
sudo cat /proc/iomem | head -40
# Look for "pcie" and "rp1" entries to see RP1's address space

# Per-process information (PID 1 = systemd)
ls /proc/1/
cat /proc/1/cmdline      # Command line that started the process
cat /proc/1/status       # Process status (state, memory, threads)
cat /proc/1/maps         # Virtual memory mapping
cat /proc/1/fd/          # Open file descriptors (ls -la)
```

**Practical exploration of /proc/interrupts:**

```bash
# Watch interrupt counts change in real time
watch -n 1 'cat /proc/interrupts | head -30'
```

This shows:
- Which IRQ number is assigned to which device
- How many interrupts have fired on each CPU core
- Whether interrupt load is balanced across cores

If your camera interrupt is only hitting Core 0, and Core 0 is also running your perception stack, you will get frame drops. This can be fixed with IRQ affinity tuning — we will cover this in later days.

### 2.4 /sys — Hardware and Driver Virtual Filesystem

`/sys` (sysfs) provides a structured, writable view of the kernel's device model:

```bash
# CPU frequency scaling
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq    # Current freq (kHz)
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq    # Max freq
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor     # Current governor

# Set performance governor (max speed always -- good for real-time)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Temperature (millidegrees Celsius)
cat /sys/class/thermal/thermal_zone0/temp
# 45000 means 45.0 degrees C

# LED control
ls /sys/class/leds/
# led0 = green activity LED, led1 = red power LED
echo none | sudo tee /sys/class/leds/led0/trigger    # Disable activity LED
echo 1 | sudo tee /sys/class/leds/led0/brightness    # Turn on
echo 0 | sudo tee /sys/class/leds/led0/brightness    # Turn off

# Network interface details
cat /sys/class/net/eth0/speed              # Link speed in Mbps
cat /sys/class/net/eth0/statistics/rx_bytes # Total bytes received
cat /sys/class/net/eth0/statistics/tx_bytes # Total bytes transmitted
```

### 2.5 /boot/firmware — The Boot Partition

```bash
ls /boot/firmware/
# bcm2712-rpi-5-b.dtb     Device Tree Blob
# cmdline.txt              Kernel command line
# config.txt               Hardware configuration
# kernel_2712.img          Linux kernel
# overlays/                Device Tree Overlays
# start4.elf               GPU firmware
```

**Device Tree Overlays** are fragments that modify the base DTB to enable specific hardware:

```bash
ls /boot/firmware/overlays/ | grep -E "i2c|spi|uart|imx|pwm" | head -15
```

To enable an overlay, add it to `config.txt`:

```ini
# Enable Camera Module v3
dtoverlay=imx708

# Enable additional I2C bus on specific pins
dtoverlay=i2c3,pins_4_5

# Enable hardware PWM (2 channels)
dtoverlay=pwm-2chan

# Enable hardware UART on specific pins
dtoverlay=uart2,pins_0_1
```

After editing `config.txt`, reboot for changes to take effect.

---

## 3. Process Model

### 3.1 What Is a Process?

A **process** is a running instance of a program. Each process has:

- **PID**: Process ID (unique integer, assigned sequentially)
- **PPID**: Parent Process ID (who created this process)
- **UID/GID**: User and Group ownership
- **Virtual address space**: Isolated memory (other processes cannot see it)
- **File descriptors**: Open files (stdin=0, stdout=1, stderr=2, plus any opened files/devices/sockets)
- **State**: Running (R), Sleeping (S), Stopped (T), Zombie (Z), Dead (X)
- **Priority/Nice value**: Scheduling priority (-20 to +19)

### 3.2 fork() and exec() — How Processes Are Born

In Linux, new processes are created by **forking** an existing process:

```
Parent Process (PID 100)
         |
      fork()
         |
    +----+----+
    |         |
Parent      Child (exact copy)
(PID 100)  (PID 101)
    |         |
continues   exec("python3 camera.py")
    |         |
    |      Camera process
    |      (PID 101, now running Python)
    |         |
  wait()    exit(0)
    |         |
  reap      [removed from process table]
```

1. **`fork()`**: Creates an exact copy of the parent process. The child gets a new PID but inherits everything else (memory pages via copy-on-write, file descriptors, environment variables).
2. **`exec()`**: Replaces the child's program image with a new one. The PID stays the same, but the code, data, and stack are completely replaced.

This two-step model is fundamental to how Linux works. Every process on the system (except PID 1) was created by fork+exec.

```python
#!/usr/bin/env python3
"""
fork_demo.py -- Demonstrates the fork/exec process model
Run on the Pi to see process creation in action.
"""

import os
import sys
import time

print(f"=== Parent process started ===")
print(f"  PID  = {os.getpid()}")
print(f"  PPID = {os.getppid()}")
print()

# Create a child process
pid = os.fork()

if pid == 0:
    # This code runs in the CHILD process
    print(f"  [Child] I am the child!")
    print(f"  [Child] My PID  = {os.getpid()}")
    print(f"  [Child] My PPID = {os.getppid()} (that's the parent)")
    print(f"  [Child] Sleeping 2 seconds then exiting...")
    time.sleep(2)
    print(f"  [Child] Goodbye!")
    os._exit(42)  # Exit with status 42
else:
    # This code runs in the PARENT process
    print(f"  [Parent] I created child with PID = {pid}")
    print(f"  [Parent] Waiting for child to finish...")

    child_pid, raw_status = os.waitpid(pid, 0)
    exit_code = os.WEXITSTATUS(raw_status)

    print(f"  [Parent] Child {child_pid} exited with code {exit_code}")
    print(f"=== Parent process done ===")
```

### 3.3 Process States

```
                +----------+
    fork()      |          |
 +----------->  | CREATED  |
                |   (new)  |
                +----+-----+
                     |
                scheduler picks it
                     |
                +----v-----+
           +--->|          |
           |    | RUNNING  |----+
           |    |   (R)    |    |
           |    +----+-----+    |
           |         |         I/O wait / sleep()
      CPU quantum    |          |
      or wakeup     exit()     |
           |         |    +----v-----+
           |         |    |          |
           |         |    | SLEEPING |
           |         |    |  (S/D)   |
           |         |    +----+-----+
           |         |         |
           |         |    event arrives (I/O complete, signal, timer)
           |         |         |
           |         +---------+
           |         |
           |    +----v-----+
           |    |          |
           +----+  ZOMBIE  |  Parent has not called wait() yet
                |   (Z)    |  Process is dead but PID still occupied
                +----+-----+
                     |
                parent calls wait()
                     |
                +----v-----+
                |  REMOVED |  Fully cleaned up
                +----------+
```

**State meanings:**
- **R (Running/Runnable)**: Currently executing or ready to execute
- **S (Interruptible Sleep)**: Waiting for an event (I/O, timer, signal). Can be interrupted.
- **D (Uninterruptible Sleep)**: Waiting for I/O. Cannot be killed (even with `kill -9`). Usually brief.
- **Z (Zombie)**: Process exited but parent has not called `wait()`. PID is occupied.
- **T (Stopped)**: Paused by SIGSTOP or SIGTSTP (Ctrl+Z). Can be resumed with SIGCONT.

### 3.4 Zombie Processes — A Real Problem in Robotics

A **zombie process** occurs when a child exits but its parent has not called `wait()` to collect the exit status. The process occupies a PID slot and a kernel process table entry, even though it is not running.

**Why this matters for autonomous driving:** If your camera node spawns subprocesses for image processing and does not properly collect their exit status, you will accumulate zombies. Eventually you run out of PIDs (default max ~32768 on Linux) and the system cannot create new processes. Your car stops processing.

```bash
# Find zombie processes
ps aux | grep ' Z'

# Example output:
# pi    1234  0.0  0.0  0  0 ?  Z  10:15  0:00 [camera_worker] <defunct>

# Count zombies
ps aux | awk '$8 ~ /Z/ {count++} END {print count+0, "zombie processes"}'
```

**How to prevent zombies in Python:**

```python
#!/usr/bin/env python3
"""
Three ways to prevent zombie processes
"""

import subprocess
import signal
import os

# Method 1: Use subprocess module (RECOMMENDED)
# subprocess.run() automatically calls wait()
result = subprocess.run(
    ["python3", "process_frame.py"],
    capture_output=True,
    timeout=10  # Kill if it takes more than 10 seconds
)
print(f"Exit code: {result.returncode}")

# Method 2: If using os.fork(), ALWAYS call waitpid()
pid = os.fork()
if pid == 0:
    # Child does work
    os._exit(0)
else:
    # Parent MUST wait for child
    os.waitpid(pid, 0)

# Method 3: Ignore SIGCHLD (kernel auto-reaps children)
# Use this when you don't care about child exit status
signal.signal(signal.SIGCHLD, signal.SIG_IGN)
# Now any child that exits is automatically cleaned up

# Method 4: Double-fork (daemon pattern)
# The child forks again and the middle process exits immediately
# The grandchild is adopted by PID 1 (systemd), which always reaps
pid = os.fork()
if pid == 0:
    # First child
    pid2 = os.fork()
    if pid2 == 0:
        # Grandchild -- this is the real worker
        # systemd (PID 1) will adopt and reap this process
        os._exit(0)
    else:
        # First child exits immediately
        os._exit(0)
else:
    # Parent waits for first child (instant -- it exits right away)
    os.waitpid(pid, 0)
```

### 3.5 Process Monitoring Commands

```bash
# Real-time interactive process monitor (much better than top)
htop
# Install if not present: sudo apt install htop

# Snapshot of all processes (two styles)
ps aux                  # BSD-style: shows all processes with details
ps -ef                  # POSIX-style: shows full command lines

# Process tree (shows parent-child hierarchy)
pstree -p              # With PIDs
pstree -p 1            # From systemd down

# Find processes by name
pgrep -a python        # All Python processes with full command line
pgrep -af camera       # Processes with "camera" in the command

# Signal management
kill 1234              # Send SIGTERM (request graceful shutdown)
kill -9 1234           # Send SIGKILL (force kill -- last resort)
kill -STOP 1234        # Pause a process (SIGSTOP)
kill -CONT 1234        # Resume a paused process (SIGCONT)
kill -USR1 1234        # Send custom signal (for log rotation, etc.)

# System resource monitoring
vmstat 1 5             # Virtual memory, CPU, I/O stats (every 1s, 5 times)
iostat 1 5             # Disk I/O stats (need: sudo apt install sysstat)
free -h                # Memory usage
uptime                 # Load averages
```

---

## 4. File Permissions and udev Rules

### 4.1 Linux File Permissions

Every file and directory has three permission sets: **owner**, **group**, **others**.

```
-rwxr-xr-- 1 pi gpio 4096 Jan 15 10:00 camera.py
 |||  |||  |||
 |||  |||  ||+-- others: read only          (r--)
 |||  |||  |+--- separator
 |||  ||+--+---- group (gpio): read+execute (r-x)
 |||  |+-------- separator
 ||+--+--------- owner (pi): read+write+exec(rwx)
 |+------------- file type: - = regular, d = directory, l = symlink
```

**Permission values (octal):**

| Symbol | Octal | Meaning |
|--------|-------|---------|
| `r` | 4 | Read |
| `w` | 2 | Write |
| `x` | 1 | Execute |

So `rwxr-xr--` = 754:
- Owner: 7 = 4+2+1 = read+write+execute
- Group: 5 = 4+0+1 = read+execute
- Others: 4 = 4+0+0 = read only

```bash
# Change permissions
chmod 755 camera.py        # rwxr-xr-x (owner full, others read+exec)
chmod +x start.sh          # Add execute permission for all
chmod 600 ~/.ssh/id_ed25519  # Owner read/write only (REQUIRED for SSH keys)

# Change ownership
sudo chown pi:gpio camera.py    # Owner=pi, Group=gpio

# Add user to a group (for device access)
sudo usermod -aG gpio pi        # Add pi to gpio group
sudo usermod -aG i2c pi         # Add pi to i2c group
sudo usermod -aG spi pi         # Add pi to spi group
sudo usermod -aG dialout pi     # Add pi to dialout group (serial ports)
# Log out and back in for group changes to take effect!
```

### 4.2 udev Rules — Automatic Device Configuration

`udev` is the Linux device manager. When hardware appears (boot or hotplug), udev:
1. Receives a kernel event (uevent)
2. Matches the device against rules in `/etc/udev/rules.d/`
3. Creates the device file in `/dev/` with proper name, permissions, and ownership
4. Optionally creates symlinks and runs scripts

**Why this matters for autonomous cars:** You might have a USB camera, a USB-to-CAN adapter, and a USB GPS receiver. When the car boots, you need these devices to **always** appear at the same `/dev/` path, regardless of which USB port they are plugged into or the order they are detected.

**Step 1: Identify device attributes**

```bash
# Plug in the USB device and find it
dmesg | tail -20
# Look for: "usb 1-1: new full-speed USB device"
# And: "ttyUSB0" or "video0"

# Get detailed device attributes for rule matching
udevadm info -a -n /dev/ttyUSB0

# Key attributes to look for:
# ATTRS{idVendor}=="1a86"      -- USB Vendor ID
# ATTRS{idProduct}=="7523"     -- USB Product ID
# ATTRS{serial}=="AB12CD34"    -- Serial number (most unique)
# ATTRS{manufacturer}=="QinHeng"

# You can also use:
udevadm info --query=all --name=/dev/ttyUSB0
```

**Step 2: Write udev rules**

```bash
# /etc/udev/rules.d/99-autocar.rules
# Rules are processed in filename order; 99- runs last (highest priority)

# USB-to-Serial adapter -> always /dev/can_adapter
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="can_adapter", MODE="0666"

# USB Camera (Logitech C920) -> always /dev/autocar_camera
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
  ATTR{index}=="0", SYMLINK+="autocar_camera", MODE="0666"

# USB GPS receiver -> always /dev/gps
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a7", \
  SYMLINK+="gps", MODE="0666", GROUP="dialout"

# IMU over USB-serial -> always /dev/imu
SUBSYSTEM=="tty", ATTRS{serial}=="IMU_UNIT_001", \
  SYMLINK+="imu", MODE="0666", GROUP="dialout"
```

**Step 3: Reload and test**

```bash
# Reload rules (no reboot needed)
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify the symlink was created
ls -la /dev/can_adapter
# lrwxrwxrwx 1 root root 7 Jan 15 10:00 /dev/can_adapter -> ttyUSB0

# Test: unplug and replug the device
# The symlink should reappear at the same name
```

Now your code can always use `/dev/can_adapter` regardless of which physical USB port the adapter is in. This is essential for reliable autonomous car operation.

**Advanced: Run a script when a device appears**

```bash
# In /etc/udev/rules.d/99-autocar.rules:
ACTION=="add", SUBSYSTEM=="video4linux", ATTR{index}=="0", \
  RUN+="/home/pi/on_camera_connect.sh %k"

ACTION=="remove", SUBSYSTEM=="video4linux", \
  RUN+="/home/pi/on_camera_disconnect.sh"
```

```bash
#!/bin/bash
# /home/pi/on_camera_connect.sh
DEVICE=$1
echo "$(date): Camera connected as /dev/${DEVICE}" >> /home/pi/device_events.log
# Optionally restart camera service:
# systemctl restart autocar-camera
```

### 4.3 cron and systemd Timers — Scheduled Tasks

**cron** (traditional approach):

```bash
crontab -e
# Format: minute hour day month weekday command

# Log CPU temperature every 5 minutes
*/5 * * * * vcgencmd measure_temp >> /home/pi/temp_log.txt

# Clean up old camera frames at midnight
0 0 * * * find /home/pi/frames/ -mtime +7 -delete

# Restart perception stack daily at 3 AM (safety reset)
0 3 * * * sudo systemctl restart autocar-perception
```

**systemd timer** (modern approach — preferred):

```ini
# /etc/systemd/system/cleanup-frames.timer
[Unit]
Description=Clean up old camera frames daily

[Timer]
OnCalendar=daily
Persistent=true

[Install]
WantedBy=timers.target
```

```ini
# /etc/systemd/system/cleanup-frames.service
[Unit]
Description=Remove camera frames older than 7 days

[Service]
Type=oneshot
ExecStart=/usr/bin/find /home/pi/frames/ -mtime +7 -delete
```

```bash
sudo systemctl enable cleanup-frames.timer
sudo systemctl start cleanup-frames.timer
systemctl list-timers  # Verify it's scheduled
```

---

## 5. Shell Scripting Essentials

### 5.1 Variables and Basic Syntax

```bash
#!/bin/bash
# Shell scripting basics for autonomous car automation

# Variables (NO spaces around the = sign!)
CAR_NAME="autocar-01"
CAMERA_DEV="/dev/video0"
LOG_DIR="/home/pi/logs"
FRAME_RATE=30

# Using variables (always quote to handle spaces safely)
echo "Starting ${CAR_NAME} with camera ${CAMERA_DEV}"

# Command substitution -- capture command output
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
KERNEL_VER=$(uname -r)
CPU_TEMP=$(vcgencmd measure_temp | cut -d= -f2 | cut -d\' -f1)

echo "Boot at ${TIMESTAMP}, kernel ${KERNEL_VER}, CPU ${CPU_TEMP}C"

# Arithmetic
WIDTH=640
HEIGHT=480
PIXELS=$((WIDTH * HEIGHT))
FRAME_SIZE=$((PIXELS * 3))  # 3 bytes per pixel (RGB)
BITRATE=$((FRAME_SIZE * FRAME_RATE * 8))  # bits per second

echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "Frame size: ${FRAME_SIZE} bytes"
echo "Raw bitrate: $((BITRATE / 1000000)) Mbps"
```

### 5.2 Conditionals

```bash
#!/bin/bash
# preflight_check.sh -- Pre-flight check for autonomous car system

set -e  # Exit on error

echo "==========================================="
echo "   AutoCar Pre-Flight Check"
echo "   $(date)"
echo "==========================================="

PASS=0
FAIL=0
WARN=0

check_pass() { echo "[PASS] $1"; PASS=$((PASS + 1)); }
check_fail() { echo "[FAIL] $1"; FAIL=$((FAIL + 1)); }
check_warn() { echo "[WARN] $1"; WARN=$((WARN + 1)); }

# Check hardware platform
if grep -q "Cortex-A76" /proc/cpuinfo; then
    check_pass "Running on Cortex-A76 (RPi 5)"
else
    check_fail "Not running on RPi 5!"
fi

# Check CPU temperature
TEMP_RAW=$(cat /sys/class/thermal/thermal_zone0/temp)
TEMP_C=$((TEMP_RAW / 1000))

if [ "${TEMP_C}" -lt 60 ]; then
    check_pass "CPU temperature: ${TEMP_C}C"
elif [ "${TEMP_C}" -lt 80 ]; then
    check_warn "CPU temperature: ${TEMP_C}C (consider better cooling)"
else
    check_fail "CPU temperature: ${TEMP_C}C (OVERHEATING!)"
fi

# Check throttling
THROTTLE=$(vcgencmd get_throttled | cut -d= -f2)
if [ "${THROTTLE}" = "0x0" ]; then
    check_pass "No throttling detected"
else
    check_fail "Throttling active: ${THROTTLE}"
fi

# Check camera
if [ -e /dev/video0 ]; then
    check_pass "Camera device: /dev/video0"
else
    check_fail "No camera detected at /dev/video0"
fi

# Check I2C bus
if [ -e /dev/i2c-1 ]; then
    check_pass "I2C bus: /dev/i2c-1"
else
    check_warn "I2C bus not available (enable in config.txt)"
fi

# Check available disk space (need at least 1 GB free)
AVAIL_KB=$(df / | awk 'NR==2 {print $4}')
AVAIL_MB=$((AVAIL_KB / 1024))

if [ "${AVAIL_MB}" -gt 1024 ]; then
    check_pass "Disk space: ${AVAIL_MB} MB available"
elif [ "${AVAIL_MB}" -gt 256 ]; then
    check_warn "Low disk space: ${AVAIL_MB} MB"
else
    check_fail "Critical disk space: ${AVAIL_MB} MB"
fi

# Check available memory
MEM_AVAIL=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
if [ "${MEM_AVAIL}" -gt 512 ]; then
    check_pass "Available memory: ${MEM_AVAIL} MB"
else
    check_warn "Low memory: ${MEM_AVAIL} MB"
fi

# Check network
if ping -c 1 -W 2 8.8.8.8 > /dev/null 2>&1; then
    check_pass "Network connectivity (internet)"
else
    check_warn "No internet (offline mode)"
fi

# Summary
echo ""
echo "==========================================="
echo "   Results: ${PASS} passed, ${WARN} warnings, ${FAIL} failures"
echo "==========================================="

if [ "${FAIL}" -gt 0 ]; then
    echo "   STATUS: NOT READY -- fix failures before driving"
    exit 1
else
    echo "   STATUS: READY"
    exit 0
fi
```

### 5.3 Loops

```bash
#!/bin/bash
# system_monitor.sh -- Monitor system vitals continuously

echo "System monitor started (Ctrl+C to stop)..."
echo "Time       | Temp | Freq     | Load | Memory"
echo "-----------|------|----------|------|--------"

while true; do
    TEMP=$(($(cat /sys/class/thermal/thermal_zone0/temp) / 1000))
    FREQ=$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) / 1000))
    LOAD=$(cut -d' ' -f1 /proc/loadavg)
    MEM_USED=$(free -m | awk 'NR==2 {print $3}')
    MEM_TOTAL=$(free -m | awk 'NR==2 {print $2}')

    printf "%s | %3dC | %4d MHz | %s | %d/%d MB\n" \
        "$(date +%H:%M:%S)" "${TEMP}" "${FREQ}" "${LOAD}" \
        "${MEM_USED}" "${MEM_TOTAL}"

    sleep 1
done
```

```bash
#!/bin/bash
# process_images.sh -- Batch process images using a for loop

INPUT_DIR="/home/pi/raw_frames"
OUTPUT_DIR="/home/pi/processed"
COUNTER=0

mkdir -p "${OUTPUT_DIR}"

for img in "${INPUT_DIR}"/*.jpg; do
    [ -f "${img}" ] || continue  # Skip if no matches

    BASENAME=$(basename "${img}")
    echo "Processing [${COUNTER}]: ${BASENAME}"

    # Example: resize to 320x240 using Python
    python3 -c "
import cv2, sys
img = cv2.imread('${img}')
if img is not None:
    resized = cv2.resize(img, (320, 240))
    cv2.imwrite('${OUTPUT_DIR}/small_${BASENAME}', resized)
"
    COUNTER=$((COUNTER + 1))
done

echo "Processed ${COUNTER} images"
```

### 5.4 Functions

```bash
#!/bin/bash
# autocar_utils.sh -- Reusable functions for car management
# Source this file: source autocar_utils.sh

LOG_FILE="/home/pi/autocar.log"

# Log a message with timestamp and level
log_msg() {
    local LEVEL="${1}"
    local MSG="${2}"
    local TIMESTAMP
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[${TIMESTAMP}] [${LEVEL}] ${MSG}" | tee -a "${LOG_FILE}"
}

# Check if a device exists
check_device() {
    local DEVICE="${1}"
    local NAME="${2}"
    if [ -e "${DEVICE}" ]; then
        log_msg "INFO" "${NAME} found at ${DEVICE}"
        return 0
    else
        log_msg "ERROR" "${NAME} NOT found at ${DEVICE}"
        return 1
    fi
}

# Get CPU temperature as integer (Celsius)
get_temp() {
    echo $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))
}

# Wait for a device to appear (with timeout)
wait_for_device() {
    local DEVICE="${1}"
    local TIMEOUT="${2:-30}"  # Default 30 seconds
    local ELAPSED=0

    log_msg "INFO" "Waiting for ${DEVICE} (timeout: ${TIMEOUT}s)..."
    while [ ! -e "${DEVICE}" ] && [ "${ELAPSED}" -lt "${TIMEOUT}" ]; do
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done

    if [ -e "${DEVICE}" ]; then
        log_msg "INFO" "${DEVICE} appeared after ${ELAPSED}s"
        return 0
    else
        log_msg "ERROR" "${DEVICE} did not appear within ${TIMEOUT}s"
        return 1
    fi
}

# Usage example:
# source autocar_utils.sh
# log_msg "INFO" "System starting"
# check_device "/dev/video0" "Camera"
# wait_for_device "/dev/can_adapter" 10
# TEMP=$(get_temp)
# log_msg "INFO" "Temperature: ${TEMP}C"
```

---

## 6. Hands-On Lab

### 6.1 Lab 1: Writing a systemd Service

Let's create a systemd service that auto-starts a Python monitoring script at boot.

**Step 1: Create the Python script**

```python
#!/usr/bin/env python3
"""
/home/pi/autocar_monitor.py
System health monitor for autonomous car platform.
Designed to run as a systemd service.
"""

import time
import os
import json
from datetime import datetime

LOG_FILE = "/home/pi/autocar_health.jsonl"
INTERVAL = 10  # seconds between readings

def get_cpu_temp():
    """Read CPU temperature in Celsius."""
    with open("/sys/class/thermal/thermal_zone0/temp") as f:
        return int(f.read().strip()) / 1000.0

def get_cpu_freq():
    """Read current CPU frequency in MHz."""
    with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
        return int(f.read().strip()) / 1000

def get_memory_usage():
    """Return (total_mb, used_mb)."""
    with open("/proc/meminfo") as f:
        lines = f.readlines()
    total = int(lines[0].split()[1]) / 1024  # MemTotal
    available = int(lines[2].split()[1]) / 1024  # MemAvailable
    return round(total, 1), round(total - available, 1)

def get_load_average():
    """Read 1-minute load average."""
    with open("/proc/loadavg") as f:
        return float(f.read().split()[0])

def get_throttled():
    """Read throttle status from vcgencmd."""
    try:
        import subprocess
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout.strip().split("=")[1]
    except Exception:
        return "unknown"

def main():
    print(f"AutoCar Health Monitor started (PID: {os.getpid()})")
    print(f"Logging to: {LOG_FILE}")
    print(f"Interval: {INTERVAL}s")

    while True:
        try:
            temp = get_cpu_temp()
            freq = get_cpu_freq()
            mem_total, mem_used = get_memory_usage()
            load = get_load_average()
            throttled = get_throttled()

            record = {
                "ts": datetime.now().isoformat(),
                "temp_c": round(temp, 1),
                "freq_mhz": int(freq),
                "mem_used_mb": mem_used,
                "mem_total_mb": mem_total,
                "load_1m": round(load, 2),
                "throttled": throttled,
            }

            # Append as JSON Lines (one JSON object per line)
            with open(LOG_FILE, "a") as f:
                f.write(json.dumps(record) + "\n")

            # Print to stdout (captured by journalctl)
            status = (f"T:{temp:.1f}C F:{freq:.0f}MHz "
                      f"M:{mem_used:.0f}/{mem_total:.0f}MB "
                      f"L:{load:.2f}")
            print(status)

            if temp > 80:
                print(f"WARNING: CPU temperature {temp:.1f}C exceeds 80C!")

        except Exception as e:
            print(f"Error: {e}")

        time.sleep(INTERVAL)


if __name__ == "__main__":
    main()
```

Save and make executable:

```bash
nano /home/pi/autocar_monitor.py   # Paste the code
chmod +x /home/pi/autocar_monitor.py
python3 /home/pi/autocar_monitor.py  # Test it manually first (Ctrl+C to stop)
```

**Step 2: Create the systemd service unit file**

```bash
sudo nano /etc/systemd/system/autocar-monitor.service
```

```ini
[Unit]
Description=AutoCar System Health Monitor
Documentation=man:autocar-monitor
After=multi-user.target
Wants=network-online.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/autocar_monitor.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=autocar-monitor

# Resource limits (prevent runaway resource usage)
MemoryMax=128M
CPUQuota=10%

# Security hardening
NoNewPrivileges=yes
ProtectSystem=strict
ReadWritePaths=/home/pi

[Install]
WantedBy=multi-user.target
```

**Understanding each directive:**

| Directive | Purpose |
|-----------|---------|
| `After=multi-user.target` | Start after basic system is ready |
| `Type=simple` | The ExecStart process IS the service |
| `Restart=always` | Auto-restart if it crashes |
| `RestartSec=5` | Wait 5s before restart (prevents restart storms) |
| `StandardOutput=journal` | Stdout goes to journald |
| `MemoryMax=128M` | OOM-kill if memory exceeds 128 MB |
| `CPUQuota=10%` | Limit to 10% of one core |
| `NoNewPrivileges=yes` | Cannot escalate privileges |
| `ProtectSystem=strict` | Filesystem is read-only except ReadWritePaths |

**Step 3: Enable and manage**

```bash
# Reload systemd (picks up new/changed unit files)
sudo systemctl daemon-reload

# Start the service
sudo systemctl start autocar-monitor

# Check status
systemctl status autocar-monitor

# View live logs
journalctl -u autocar-monitor -f

# Enable auto-start at boot
sudo systemctl enable autocar-monitor

# Stop the service
sudo systemctl stop autocar-monitor

# View historical logs
journalctl -u autocar-monitor --since "1 hour ago"

# Parse the JSON log file
cat /home/pi/autocar_health.jsonl | python3 -c "
import sys, json
for line in sys.stdin:
    r = json.loads(line)
    print(f\"{r['ts']}: {r['temp_c']}C, {r['freq_mhz']}MHz, Load:{r['load_1m']}\")
"
```

### 6.2 Lab 2: Boot Time Analysis and Optimization

```bash
# Measure current boot time
systemd-analyze

# Find the slowest services
systemd-analyze blame | head -20

# See the critical chain (longest dependency path)
systemd-analyze critical-chain

# Generate visual boot chart
systemd-analyze plot > /tmp/boot_chart.svg
# Transfer to host: scp pi@autocar.local:/tmp/boot_chart.svg .

# Optimize: disable unnecessary services
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer
sudo systemctl disable man-db.timer

# If Bluetooth is not needed:
sudo systemctl disable bluetooth.service
sudo systemctl disable hciuart.service

# If ModemManager is not needed:
sudo systemctl disable ModemManager.service

# Reboot and measure again
sudo reboot
# After reboot:
systemd-analyze
# Compare before and after!
```

### 6.3 Lab 3: Exploring /proc and System Internals

```bash
# See interrupt distribution across CPU cores
cat /proc/interrupts | head -20
# Each column is a CPU core
# Watch for unbalanced interrupt counts

# I/O memory map
sudo cat /proc/iomem | grep -i "pcie\|rp1\|ram"
# Shows where PCIe and RP1 are mapped in physical memory

# Loaded kernel modules
lsmod | head -20

# Device tree as the kernel sees it
cat /proc/device-tree/model
# "Raspberry Pi 5 Model B Rev 1.0"

ls /proc/device-tree/soc/
# Shows all SoC peripherals known to the kernel

# Memory allocation details
cat /proc/buddyinfo     # Memory fragmentation
cat /proc/slabinfo | head -20  # Kernel slab allocator stats
```

### 6.4 Lab 4: udev Rules for Persistent Device Naming

```bash
# Step 1: Plug in a USB device and identify it
dmesg | tail -10

# Step 2: Get its attributes
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial|manufacturer"

# Step 3: Create a rule
sudo nano /etc/udev/rules.d/99-autocar.rules

# Step 4: Add this rule (substitute your device's Vendor/Product IDs):
# SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
#   SYMLINK+="can_adapter", MODE="0666"

# Step 5: Reload and test
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/can_adapter

# Step 6: Verify persistence -- unplug and replug the device
# The symlink should reappear
```

### 6.5 Lab 5: Complete Setup Automation Script

```bash
#!/bin/bash
# /home/pi/setup_autocar.sh
# One-shot setup for a fresh RPi 5 autonomous car platform

set -euo pipefail

echo "==========================================="
echo "   AutoCar RPi 5 Setup Script"
echo "   $(date)"
echo "==========================================="

# --- System Updates ---
echo ""
echo "[1/7] Updating system packages..."
sudo apt update && sudo apt upgrade -y

# --- Essential Packages ---
echo ""
echo "[2/7] Installing required packages..."
sudo apt install -y \
    python3-pip python3-venv \
    python3-gpiozero python3-lgpio python3-libgpiod \
    python3-smbus python3-spidev \
    i2c-tools spi-tools \
    git vim htop tmux \
    minicom screen \
    jq \
    libcamera-apps \
    python3-opencv

# --- Enable Hardware Interfaces ---
echo ""
echo "[3/7] Enabling hardware interfaces in config.txt..."

CONFIG="/boot/firmware/config.txt"

add_config() {
    local LINE="${1}"
    if ! grep -q "^${LINE}" "${CONFIG}"; then
        echo "${LINE}" | sudo tee -a "${CONFIG}" > /dev/null
        echo "  Added: ${LINE}"
    else
        echo "  Already present: ${LINE}"
    fi
}

add_config "dtparam=i2c_arm=on"
add_config "dtparam=spi=on"
add_config "enable_uart=1"
add_config "gpu_mem=128"

# --- Project Directory Structure ---
echo ""
echo "[4/7] Creating project directories..."
mkdir -p ~/autocar/{src,logs,data,config,scripts}
mkdir -p ~/autocar/data/{camera,lidar,imu,can}

# --- Performance Tuning ---
echo ""
echo "[5/7] Performance tuning..."
# Set CPU governor to performance
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance | sudo tee "$cpu" > /dev/null
done
echo "  CPU governor set to performance"

# --- Disable Unnecessary Services ---
echo ""
echo "[6/7] Disabling unnecessary services..."
SERVICES_TO_DISABLE="apt-daily.timer apt-daily-upgrade.timer man-db.timer"
for svc in ${SERVICES_TO_DISABLE}; do
    if systemctl is-enabled "${svc}" 2>/dev/null | grep -q enabled; then
        sudo systemctl disable "${svc}"
        echo "  Disabled: ${svc}"
    fi
done

# --- Final Report ---
echo ""
echo "[7/7] Setup complete!"
echo ""
echo "  Platform:    $(cat /proc/device-tree/model 2>/dev/null || echo 'Unknown')"
echo "  Kernel:      $(uname -r)"
echo "  CPU:         $(lscpu | grep 'Model name' | awk -F: '{print $2}' | xargs)"
echo "  Memory:      $(free -h | awk 'NR==2 {print $2}')"
echo "  Disk free:   $(df -h / | awk 'NR==2 {print $4}')"
echo "  Temperature: $(vcgencmd measure_temp | cut -d= -f2)"
echo ""
echo "  REBOOT REQUIRED to activate I2C, SPI, UART, and config changes."
echo "  Run: sudo reboot"
```

---

## 7. Review

### Key Concepts Checklist

1. **Boot sequence**: EEPROM bootloader -> GPU firmware (start4.elf reads config.txt) -> Linux kernel -> systemd (PID 1). The GPU boots before the CPU on RPi 5.

2. **config.txt** is the Pi's equivalent of BIOS settings. Device Tree Overlays enable specific hardware (camera, I2C, SPI, PWM).

3. **Filesystem hierarchy**: `/dev` (devices as files), `/proc` (live process/kernel data), `/sys` (hardware/driver attributes), `/boot/firmware` (boot partition).

4. **Process model**: `fork()` creates a child process, `exec()` replaces its program image. Always `wait()` for children to prevent zombie accumulation.

5. **systemd**: Dependency-based parallel service management. `systemctl` controls services, `journalctl` reads logs, `systemd-analyze` profiles boot time.

6. **udev rules**: Create persistent device symlinks and auto-configure permissions. Critical for reliable USB device management in robotics.

7. **Shell scripting**: Variables, conditionals, loops, functions. The automation glue for embedded systems.

### Self-Test Questions

**Q1:** In what order do the four boot stages execute? Where and when is `config.txt` read?

**Answer:** (1) EEPROM bootloader initializes RAM and finds boot media, (2) GPU firmware (start4.elf) reads config.txt and loads the kernel, (3) Linux kernel initializes hardware and mounts rootfs, (4) systemd starts services. `config.txt` is read by the GPU firmware in stage 2, before the ARM CPU even starts.

**Q2:** Your camera service needs `/dev/video0` to exist before it starts. What systemd directives ensure this?

**Answer:** Use `After=dev-video0.device` and optionally `Requires=dev-video0.device`. For USB cameras that may appear late, a more robust approach is to have a udev rule that triggers `systemctl start autocar-camera` when the camera appears, rather than blocking boot.

**Q3:** After running your autonomous car for 6 hours, `ps aux` shows 5000 zombie processes. The system is sluggish. What happened and how do you fix it?

**Answer:** The parent process is spawning child workers (probably via `os.fork()` or `subprocess.Popen()`) but never calling `wait()` / `.communicate()` to collect their exit status. Fix: use `subprocess.run()` (which waits automatically), or add `signal.signal(signal.SIGCHLD, signal.SIG_IGN)` to auto-reap children, or explicitly call `os.waitpid()` after each fork. Clean up existing zombies by killing the parent process (zombies disappear when their parent dies, as systemd adopts and reaps orphans).

---

## Next: Day 3

Tomorrow we add the **physical electronics layer**: Ohm's law, voltage dividers, pull-up/pull-down resistors, and the RPi 5 power design. Most importantly, we will connect a UART debug cable and watch the entire boot sequence scroll by in real time on a terminal. This is the most powerful debugging technique for embedded systems.

See you in [Day 3 -- Electronics Basics, UART Debug Console, and GPIO](/posts/embedded-day-03/).
