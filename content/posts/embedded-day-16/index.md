---
title: "Day 16 — Team Code Review and Architecture Presentation"
date: 2026-03-05T16:00:00
description: "Full-day team presentations analyzing the Hawonder vehicle codebase — motor driver, camera pipeline, sensor nodes, and launch file architecture"
categories: ["Autonomous Driving"]
tags: ["Code Review", "ROS2", "System Architecture", "Team Presentation"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 16
draft: false
---

{{< katex >}}

## What You'll Learn

Today is different from the previous 15 days. There is no new theory lecture. Instead, this is a **full-day team presentation and code review session** where each team dives deep into one subsystem of the Hawonder autonomous vehicle.

The goal is to bridge the gap between **understanding concepts in isolation** (Days 1-15) and **understanding how they all work together in a real system**. By the end of today, every student should have a clear mental picture of how the entire vehicle software stack operates — from boot to autonomous navigation.

In this post, you will find:

1. **The presentation format** and schedule
2. **Detailed guides for each team's module** — what to look for, what to present, what questions to investigate
3. **A code review checklist** applicable to any ROS2 robotics project
4. **How this connects Week 1-3 knowledge to Week 4 integration work**

Use this post as your **preparation guide** before the presentation and as a **reference** during it.

---

## 1. Presentation Format and Schedule

### 1.1 Schedule

```
09:00 - 09:30   Setup and final preparation (30 min)
09:30 - 10:30   Team A: Motor Driver + ros2_control + Hall Odometry (60 min)
10:30 - 10:45   Break
10:45 - 11:45   Team B: Camera Node + Depth Stream Publishing (60 min)
11:45 - 13:00   Lunch
13:00 - 14:00   Team C: IMU + 1D LiDAR Nodes + TF2 Frame Configuration (60 min)
14:00 - 14:15   Break
14:15 - 15:15   Team D: Launch Files + Parameter Management + RTAB-Map (60 min)
15:15 - 15:30   Break
15:30 - 16:30   Cross-team discussion + Integration architecture diagram (60 min)
16:30 - 17:00   Wrap-up: Week 4 preview and individual assignments (30 min)
```

### 1.2 Presentation Structure (60 minutes per team)

Each team presentation follows this structure:

| Time | Section | What to Show |
|------|---------|-------------|
| 0-10 min | **Architecture Overview** | High-level diagram of your module. Which nodes, which topics, which services. Show rqt_graph screenshot. |
| 10-25 min | **Code Walkthrough** | Walk through the key source files. Explain the main loop, callbacks, and data flow. Highlight interesting patterns. |
| 25-35 min | **Live Demo** | Run your module on the actual vehicle. Show topic data, TF frames, or control behavior in real time. |
| 35-45 min | **Analysis and Findings** | QoS choices, threading model, error handling, performance measurements. What's good? What could be improved? |
| 45-55 min | **Q&A from Other Teams** | Other teams ask questions. The presenting team must be able to answer or investigate on the spot. |
| 55-60 min | **Improvement Proposals** | At least 2-3 concrete suggestions for improving the module. |

### 1.3 Evaluation Criteria

Each presentation is evaluated on:

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Technical Depth | 30% | Did the team go beyond surface-level explanation? |
| Accuracy | 20% | Are the technical claims correct? |
| Live Demo | 20% | Did the demo work? Was it informative? |
| Improvements | 15% | Are the proposals realistic and valuable? |
| Communication | 15% | Was the presentation clear and well-structured? |

---

## 2. Team A: Motor Driver + ros2_control + Hall Odometry

### 2.1 Architecture Overview

Team A is responsible for the **actuation and odometry** subsystem — the lowest layer of the autonomous driving stack.

```
                                   ros2_control
┌──────────┐    /cmd_vel    ┌──────────────────────────────┐
│ Nav2 /   │ ──────────────►│  diff_drive_controller       │
│ Teleop   │                │  ┌────────────────────────┐  │
└──────────┘                │  │ Inverse Kinematics:    │  │
                            │  │ v_L = v - ωL/2         │  │
                            │  │ v_R = v + ωL/2         │  │
                            │  └──────────┬─────────────┘  │
                            │             │                 │
                            │  ┌──────────▼─────────────┐  │
                            │  │ Hardware Interface      │  │
                            │  │ (HawonderSystemHW)      │  │
                            │  │ read() ↑     ↓ write()  │  │
                            │  └────────┼─────┼──────────┘  │
                            └───────────┼─────┼─────────────┘
                                        │     │
                              ┌─────────┘     └─────────┐
                              │                         │
                       ┌──────┴──────┐           ┌──────┴──────┐
                       │ Left Motor  │           │ Right Motor │
                       │ + Encoder   │           │ + Encoder   │
                       └─────────────┘           └─────────────┘
                              │                         │
                              ▼                         ▼
                    /odom (Odometry)            /joint_states
                    /tf (odom → base_link)
```

### 2.2 Key Source Files to Examine

```
src/hawonder_hardware/
├── include/hawonder_hardware/
│   └── hawonder_system.hpp          ← Hardware interface class definition
├── src/
│   └── hawonder_system.cpp          ← Hardware interface implementation
├── config/
│   └── diff_drive_controller.yaml   ← Controller configuration
├── urdf/
│   └── ros2_control.xacro           ← Hardware interface URDF tags
└── CMakeLists.txt
```

### 2.3 What the Code Does

The hardware interface implements five lifecycle callbacks:

```cpp
// Pseudocode of the hardware interface lifecycle
class HawonderSystemHardware {

  on_init():
    // Parse URDF parameters (serial port, baud rate)
    // Initialize data structures

  on_configure():
    // Open serial connection to motor controller board
    // Verify communication
    // Reset encoder counters

  on_activate():
    // Enable motor drivers
    // Start encoder reading

  read(time, period):
    // Read encoder tick counts from serial
    // Convert ticks to radians (position)
    // Compute velocity from position change / period
    // Store in state interface buffers

  write(time, period):
    // Read velocity commands from command interface buffers
    // Convert rad/s to motor driver format (RPM, PWM, etc.)
    // Send command over serial

  on_deactivate():
    // Send zero velocity to motors
    // Disable motor drivers

  on_cleanup():
    // Close serial connection
}
```

### 2.4 Questions to Investigate

These are the questions Team A should answer during their investigation:

**Hardware Communication:**
1. What serial protocol does the motor controller use? (UART? I2C? Custom?)
2. What is the command format? (ASCII? Binary? Protobuf?)
3. What is the communication baud rate? Is it fast enough for the control loop?
4. What happens if a serial message is corrupted? Is there error detection (checksum, CRC)?

**Control Loop:**
5. What is the `update_rate` in the controller YAML? Is it sufficient for smooth control?
6. Does the hardware interface's `read()` block until data arrives, or does it use non-blocking I/O?
7. What is the measured latency from receiving a `cmd_vel` to the wheels actually moving?

**Odometry:**
8. What is the encoder CPR (counts per revolution)? At maximum speed, how many ticks per control cycle?
9. Are the wheel radius and separation parameters accurate? Measure them physically.
10. Drive the robot in a 1m square. What is the odometry drift? Is it within 10%?

**Error Handling:**
11. What happens if the serial connection drops mid-drive?
12. What happens if the encoder returns garbage data?
13. Is there a watchdog timer that stops the motors if no command is received?

### 2.5 Improvement Ideas to Discuss

- **Velocity smoothing**: Does the diff_drive_controller apply acceleration limits? If not, sudden cmd_vel changes cause wheel slip.
- **PID tuning**: Is the motor-level PID well-tuned? Measure step response.
- **Encoder filtering**: Are encoder values filtered to reduce noise? A simple moving average can help.
- **Timeout safety**: If the control loop hangs, motors should stop. Check if `cmd_vel_timeout` is configured.
- **Covariance estimation**: Does the odometry message include realistic covariance values? Nav2 needs these for localization.

### 2.6 Relevant Day References

| Concept | Day | How It Connects |
|---------|-----|-----------------|
| PWM motor control | Day 6 | The write() function ultimately sets PWM duty cycle |
| PID control | Day 6 | Motor driver runs PID to track velocity setpoint |
| Encoder reading | Day 9 | read() function reads Hall sensor encoder ticks |
| Serial communication | Day 7 | UART protocol to motor controller board |
| QoS for cmd_vel | Day 13 | Should use RELIABLE QoS |
| Lifecycle management | Day 13 | Hardware interface follows lifecycle pattern |
| TF2 odom broadcast | Day 14 | diff_drive_controller publishes odom → base_link |
| Differential kinematics | Day 15 | Inverse/forward kinematics equations |

---

## 3. Team B: Camera Node + Depth Stream Publishing

### 3.1 Architecture Overview

Team B covers the **visual perception pipeline** — from raw sensor data to ROS2 image topics.

```
┌────────────────┐        ┌───────────────────────┐
│  RGB Camera    │  USB   │  v4l2_camera_node     │
│  (hardware)    │────────│                        │
└────────────────┘        │  /camera/image_raw     │──────► Perception
                          │  /camera/camera_info   │        nodes
                          └───────────────────────┘

┌────────────────┐        ┌───────────────────────┐
│  Depth Camera  │  USB   │  depth_camera_node    │
│  (RealSense /  │────────│                        │
│   OAK-D)       │        │  /depth/image_raw     │──────► RTAB-Map
│                │        │  /depth/camera_info   │
│                │        │  /depth/color/image   │
└────────────────┘        └───────────────────────┘
```

### 3.2 Key Source Files to Examine

```
src/hawonder_camera/
├── hawonder_camera/
│   ├── __init__.py
│   ├── camera_node.py              ← RGB camera publisher
│   └── depth_camera_node.py        ← Depth camera publisher
├── config/
│   ├── camera_params.yaml          ← Resolution, FPS, device ID
│   └── camera_calibration.yaml     ← Intrinsic matrix, distortion
├── launch/
│   └── camera.launch.py
└── package.xml
```

### 3.3 Questions to Investigate

**Image Pipeline:**
1. What resolution and frame rate is configured? Is it the maximum the camera supports?
2. Is the image compressed before publishing? (`compressed` transport vs raw)
3. What is the actual measured frame rate? (Use `ros2 topic hz`)
4. What is the bandwidth? (Use `ros2 topic bw`)

For example, a 640x480 BGR8 image at 30fps:

$$
\text{bandwidth} = 640 \times 480 \times 3 \times 30 = 27.65 \text{ MB/s (raw)}
$$

With JPEG compression (10:1 ratio): ~2.8 MB/s.

**QoS Analysis:**
5. What QoS profile is used for image topics? (From Day 13)
6. Is it BEST_EFFORT or RELIABLE? Why?
7. What `depth` (history) is configured? Why that value?
8. If a subscriber is slow, does it get the latest frame or a stale one?

**Camera Calibration:**
9. Is there a camera calibration file? What distortion model does it use?
10. Is the camera_info topic published alongside the image? (Critical for 3D reconstruction)
11. Are the intrinsic parameters \(f_x, f_y, c_x, c_y\) correct? (From Day 9 camera calibration)

**Depth Camera Specifics:**
12. What depth range is configured? (min/max depth)
13. Is the depth image aligned (registered) with the RGB image?
14. What is the depth encoding? (16UC1 = millimeters? 32FC1 = meters?)
15. How are invalid depth pixels represented? (0? NaN?)

### 3.4 Improvement Ideas to Discuss

- **Compressed transport**: If using raw transport, switching to compressed can save 90% bandwidth without significant quality loss for detection tasks.
- **Region of interest (ROI)**: If only the center of the image matters for lane detection, crop before publishing.
- **Frame synchronization**: Are RGB and depth images timestamp-synchronized? If not, fused data will be misaligned.
- **Dynamic reconfigure**: Can resolution and frame rate be changed at runtime without restarting the node? Parameters should support this.
- **Error recovery**: What happens if the USB camera disconnects? Does the node attempt reconnection?

### 3.5 Live Demo Suggestions

```bash
# Show live camera feed
ros2 run rqt_image_view rqt_image_view

# Measure actual FPS
ros2 topic hz /camera/image_raw

# Measure bandwidth
ros2 topic bw /camera/image_raw

# Check QoS
ros2 topic info /camera/image_raw --verbose

# View camera intrinsics
ros2 topic echo /camera/camera_info --once
```

---

## 4. Team C: IMU + 1D LiDAR Nodes + TF2 Frame Configuration

### 4.1 Architecture Overview

Team C covers the **spatial awareness** subsystem — the sensors that tell the robot where it is and what's around it, plus the coordinate frame system that ties everything together.

```
┌──────────┐       ┌────────────────────┐
│   IMU    │ I2C/  │   imu_driver_node  │    /imu/data
│ (MPU6050 │ SPI   │                    │───────────────► Sensor Fusion
│  / BNO055)│──────│    Accel + Gyro    │                 (EKF / UKF)
└──────────┘       │    + Mag (opt)     │
                   └────────────────────┘

┌──────────┐       ┌────────────────────┐
│  LiDAR   │ UART  │  lidar_driver_node │    /scan
│ (LD06 /  │───────│                    │───────────────► Nav2 Costmap
│  LD19)   │       │  360° laser scan   │
└──────────┘       └────────────────────┘

┌────────────────────────────────────────────────────┐
│              TF2 Transform Tree                     │
│                                                     │
│  map → odom → base_link → camera_link              │
│                          → lidar_link               │
│                          → imu_link                 │
│                          → depth_camera_link        │
│                                                     │
│  Published by:                                      │
│    robot_state_publisher (static transforms)        │
│    diff_drive_controller (odom → base_link)         │
│    SLAM / localization  (map → odom)                │
└────────────────────────────────────────────────────┘
```

### 4.2 Questions to Investigate

**IMU:**
1. What IMU chip is used? What are its specifications (range, noise, bias)?
2. What is the publish rate? Is it configured in parameters or hardcoded?
3. What coordinate convention does the driver use? ROS uses **ENU** (East-North-Up). Some IMUs default to **NED** (North-East-Down). A mismatch causes incorrect heading.

$$
\text{ENU to NED}: \quad x_{\text{NED}} = y_{\text{ENU}}, \quad y_{\text{NED}} = x_{\text{ENU}}, \quad z_{\text{NED}} = -z_{\text{ENU}}
$$

4. Does the IMU driver publish orientation (quaternion) or only raw gyro/accel?
5. Is there a magnetometer? If so, is it calibrated for the vehicle's magnetic environment?
6. What is the covariance matrix in the IMU message? Is it realistic or just identity?

**LiDAR:**
7. What is the scan frequency? (e.g., LD06 = 10Hz)
8. What is the angular resolution? (e.g., 1 degree = 360 points per scan)
9. What are the min/max range values? Points beyond max range are typically reported as `inf`.
10. What is the coordinate convention? Is 0 degrees forward? Clockwise or counterclockwise?
11. Are there known blind spots (e.g., the LiDAR mount blocks a 10-degree sector)?

**TF2:**
12. Run `ros2 run tf2_tools view_frames` and verify the complete tree. Are there any disconnected frames?
13. Measure the physical sensor positions with a ruler. Do they match the URDF values?
14. Is the LiDAR mounted level? A 2-degree tilt can cause the floor to appear as an obstacle.
15. What is the transform latency? Run `ros2 run tf2_ros tf2_echo base_link lidar_link` — is the timestamp current?

### 4.3 Key Measurement Exercise

Verify the TF2 transforms are correct by performing a physical measurement:

```
1. Place an obstacle exactly 1.000m directly in front of the LiDAR
2. Read the LiDAR scan data:
   ros2 topic echo /scan --field ranges --once
3. The range at index corresponding to 0 degrees should read ~1.000m
4. Transform this point to base_link frame using TF2
5. The x coordinate in base_link should equal:
   1.000 + lidar_to_base_link_x_offset

If not, the TF2 transform is wrong!
```

### 4.4 Improvement Ideas to Discuss

- **IMU bias estimation**: IMU gyroscopes have a slowly drifting bias. Is there a calibration procedure at startup (keep the robot still for 5 seconds and estimate bias)?
- **LiDAR filtering**: Raw scans often contain noise at very short range (internal reflections). A min-range filter removes these.
- **TF2 transform accuracy**: The static transforms should be measured to millimeter precision. Even 1cm error at the sensor can mean 10cm error at 10m distance.
- **Time synchronization**: If the IMU and LiDAR have different clocks, their timestamps won't align. Is there a time sync mechanism (PTP, chrony)?
- **Sensor health monitoring**: Is there a node that monitors sensor data rates and warns if a sensor stops publishing?

### 4.5 Live Demo Suggestions

```bash
# Visualize the TF tree
ros2 run tf2_tools view_frames

# Show LiDAR scan in rviz2
rviz2
# Add LaserScan display, set topic to /scan, fixed frame to base_link

# Show IMU orientation
ros2 topic echo /imu/data --field orientation

# Verify TF chain
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## 5. Team D: Launch Files + Parameter Management + RTAB-Map Integration

### 5.1 Architecture Overview

Team D covers the **system orchestration** layer — how all the individual nodes are brought together into a functioning system.

```
┌─────────────────────────────────────────────────────────────┐
│                  hawonder_bringup.launch.py                  │
│                                                              │
│   ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│   │ robot_state_ │  │ ros2_control │  │ sensor drivers  │  │
│   │ publisher    │  │ + controllers│  │ (camera, lidar, │  │
│   └─────────────┘  └──────────────┘  │  IMU)            │  │
│                                       └─────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  navigation.launch.py                        │
│                                                              │
│   ┌──────────┐  ┌──────────────┐  ┌───────────────────┐   │
│   │ Nav2     │  │ RTAB-Map     │  │ AMCL / map_server │   │
│   │ stack    │  │ (SLAM)       │  │ (localization)     │   │
│   └──────────┘  └──────────────┘  └───────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  Parameter Files (YAML)                       │
│                                                              │
│   diff_drive_controller.yaml                                │
│   nav2_params.yaml                                          │
│   rtabmap_params.yaml                                       │
│   camera_params.yaml                                        │
│   lidar_params.yaml                                         │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Key Source Files to Examine

```
src/hawonder_bringup/
├── launch/
│   ├── hawonder_bringup.launch.py    ← Hardware bring-up
│   ├── navigation.launch.py          ← Nav2 + SLAM
│   ├── slam.launch.py                ← SLAM-only mode
│   └── localization.launch.py        ← Localization with known map
├── config/
│   ├── diff_drive_controller.yaml
│   ├── nav2_params.yaml
│   ├── rtabmap.yaml
│   └── rviz_config.rviz
├── maps/
│   ├── classroom.yaml                ← Saved map metadata
│   └── classroom.pgm                 ← Saved map image
├── urdf/
│   └── hawonder.urdf.xacro
└── package.xml
```

### 5.3 Questions to Investigate

**Launch Files:**
1. What is the startup order? Do hardware drivers start before controllers?
2. Are there launch arguments for switching between simulation and real hardware?
3. What happens if a node fails to start? Is there a `on_exit` action?
4. Are node names and namespaces consistent? (e.g., `/hawonder/camera` vs `/camera`)
5. Is the launch file composable? Can you launch just the hardware or just the navigation?

**Parameter Management:**
6. Are all tunable parameters in YAML files, or are some hardcoded in source?
7. Can parameters be changed at runtime? Which ones require a restart?
8. Is there a clear separation between hardware parameters (serial port) and algorithm parameters (PID gains)?
9. Are default parameter values sensible? Would a new developer know what to change?

**RTAB-Map Integration:**
10. What inputs does RTAB-Map receive? (RGB image, depth image, odometry, LiDAR?)
11. What outputs does it produce? (map → odom transform, occupancy grid, 3D point cloud?)
12. How does it integrate with the TF tree? Does it publish `map → odom`?
13. Can you switch between SLAM mode (building a map) and localization mode (using a saved map)?
14. What is the RTAB-Map loop closure detection rate? How often does it correct drift?

### 5.4 RTAB-Map Deep Dive

RTAB-Map (Real-Time Appearance-Based Mapping) is a visual SLAM system that combines:

```
Inputs:                         RTAB-Map                        Outputs:
                          ┌──────────────────┐
/camera/image_raw    ────►│                  │────► /map (OccupancyGrid)
/depth/image_raw     ────►│  Visual Odometry │────► /rtabmap/cloud_map
/odom               ────►│  Loop Closure    │────► map → odom TF
/scan                ────►│  Graph Optimize  │────► /rtabmap/info
                          └──────────────────┘
```

The key concept is **loop closure**: when the robot revisits a previously seen location, RTAB-Map detects the visual similarity and corrects the accumulated drift in one step.

$$
\text{Corrected pose} = \text{Odometry pose} + \text{Loop closure correction}
$$

This correction is applied by adjusting the `map → odom` transform, which is why the `odom → base_link` transform remains smooth (Day 14 concept).

### 5.5 Improvement Ideas to Discuss

- **Parameterized launch arguments**: Can you pass `robot_name:=hawonder_01` to namespace all nodes for multi-robot support?
- **Health monitoring**: Is there a system health node that monitors all sensor rates and raises alarms?
- **Configuration validation**: Is there a check that prevents launching with incompatible parameters (e.g., LiDAR topic name mismatch between driver and costmap)?
- **Logging configuration**: Are log levels configurable? Can you enable debug logging for specific nodes?
- **Map management**: Is there a clean way to save and load maps? Can you switch maps without restarting?

### 5.6 Live Demo Suggestions

```bash
# Show the complete launch process
ros2 launch hawonder_bringup hawonder_bringup.launch.py

# Show all running nodes
ros2 node list

# Show the full topic graph
ros2 run rqt_graph rqt_graph

# Show all parameters
ros2 param list

# Demonstrate parameter change at runtime
ros2 param set /diff_drive_controller wheel_separation 0.31

# Show RTAB-Map building a map in real time (in rviz2)
ros2 launch hawonder_bringup slam.launch.py
```

---

## 6. Cross-Team Integration: The Full Picture

### 6.1 Complete Data Flow Diagram

After all four presentations, the class should collaboratively build the complete data flow diagram:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FULL SYSTEM ARCHITECTURE                      │
│                                                                      │
│  ┌──────────┐                                                       │
│  │ Camera   │──/camera/image──►┌──────────┐                        │
│  │ (Team B) │                  │ RTAB-Map │──map→odom TF           │
│  │          │──/depth/image───►│ (Team D) │──/map (occupancy grid) │
│  └──────────┘                  └──────────┘                        │
│                                     │                               │
│  ┌──────────┐                       │                               │
│  │ LiDAR    │──/scan──────────►┌────▼─────┐                        │
│  │ (Team C) │                  │  Nav2    │──/cmd_vel              │
│  └──────────┘                  │  (Day 15)│     │                   │
│                                └──────────┘     │                   │
│  ┌──────────┐                                   │                   │
│  │ IMU      │──/imu/data──►┌──────────┐        │                   │
│  │ (Team C) │              │ EKF      │        │                   │
│  └──────────┘              │ (fusion) │        │                   │
│                            └──────────┘        │                   │
│  ┌──────────┐                                   │                   │
│  │ Encoders │──►┌───────────────────┐           │                   │
│  │ (Team A) │   │ diff_drive_ctrl   │◄──────────┘                   │
│  └──────────┘   │ (ros2_control)    │──/odom                       │
│                 │ (Team A)          │──odom→base_link TF           │
│                 └───────┬───────────┘                               │
│                         │                                           │
│                    ┌────▼────┐                                      │
│                    │ Motors  │                                      │
│                    │ (Team A)│                                      │
│                    └─────────┘                                      │
│                                                                      │
│  TF Tree: map → odom → base_link → {camera, lidar, imu, depth}    │
│           (Team D)  (Team A)  (URDF / Team C)                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.2 Data Flow Through the Stack

Trace a complete navigation command from start to finish:

```
1. User sends goal: "Navigate to (3.0, 2.0)"
   → Nav2 BT Navigator receives goal (Action)

2. Global planner queries /map (from RTAB-Map / map_server)
   → Computes global path using A* on costmap

3. Local planner (DWB) runs at 20Hz:
   a. Reads /scan (from LiDAR, Team C) → updates local costmap
   b. Reads /odom (from diff_drive_controller, Team A) → current position
   c. Looks up TF: map → base_link (Team D SLAM + Team A odom)
   d. Generates candidate velocities in (v, ω) space
   e. Evaluates each against critics
   f. Publishes best (v, ω) to /cmd_vel

4. diff_drive_controller (Team A) receives /cmd_vel:
   a. Inverse kinematics: compute (v_L, v_R)
   b. Hardware interface write(): send to motor controller
   c. Hardware interface read(): read encoder ticks
   d. Forward kinematics: compute new odometry
   e. Publish /odom and odom → base_link TF

5. RTAB-Map (Team D) runs loop closure:
   a. Reads /camera/image (Team B) + /depth/image (Team B) + /odom (Team A)
   b. Detects if current view matches a previous view
   c. If match: corrects drift by adjusting map → odom TF
   d. Publishes updated /map

6. Cycle repeats at 20Hz until goal is reached or navigation fails
```

---

## 7. Code Review Checklist

Use this checklist when reviewing any ROS2 robotics codebase. Each item maps to specific concepts from the course.

### 7.1 Naming Conventions

```
[ ] Node names are descriptive and lowercase_with_underscores
    Good: /camera_driver, /diff_drive_controller
    Bad:  /CamDrv, /node1

[ ] Topic names follow ROS conventions
    Good: /camera/image_raw, /scan, /cmd_vel, /odom
    Bad:  /Camera_Image, /lidar_data_topic

[ ] Frame IDs match REP 105 convention
    Good: base_link, odom, map, camera_link
    Bad:  robot_base, world, cam
```

### 7.2 QoS Choices (Day 13)

```
[ ] Sensor topics (camera, LiDAR) use BEST_EFFORT for low latency
[ ] Control topics (cmd_vel) use RELIABLE for guaranteed delivery
[ ] Map topics use TRANSIENT_LOCAL for late-joining subscribers
[ ] History depth is appropriate (1 for latest-only, N for buffering)
[ ] Deadline is set for critical topics (camera should deliver within 50ms)
[ ] QoS profiles are documented in comments explaining the rationale
```

### 7.3 Threading and Callback Groups (Day 14)

```
[ ] If using MultiThreadedExecutor, callback groups are properly assigned
[ ] Callbacks that share state use MutuallyExclusiveCallbackGroup
[ ] Independent callbacks use ReentrantCallbackGroup
[ ] No raw threading (threading.Thread) without proper synchronization
[ ] Heavy processing doesn't block time-critical callbacks
```

### 7.4 Error Handling

```
[ ] Hardware initialization failures are caught and reported
[ ] Serial/network disconnections are detected and handled
[ ] Timeout mechanisms exist for blocking operations
[ ] Node can recover from transient errors without restart
[ ] Error states are logged with appropriate severity (WARN, ERROR, FATAL)
[ ] Safety-critical operations (motor commands) have watchdog timeouts
```

### 7.5 TF2 Configuration (Day 14)

```
[ ] All sensor frames are defined relative to base_link
[ ] Static transforms match physical measurements (verified with ruler)
[ ] No TF2 tree breaks (all frames connected)
[ ] Transform timestamps are correct (not stale)
[ ] Coordinate conventions are consistent (REP 103: x=forward, y=left, z=up)
```

### 7.6 Performance

```
[ ] Topic publish rates match expected frequencies
[ ] No unnecessary data copies (use intra-process when possible)
[ ] Large messages (images) use appropriate compression
[ ] Control loops have bounded latency (measured, not assumed)
[ ] Memory usage is stable (no leaks over long-running operation)
```

### 7.7 Code Quality

```
[ ] Functions are short and focused (single responsibility)
[ ] Magic numbers are replaced with named constants or parameters
[ ] Dependencies are declared in package.xml
[ ] Entry points are defined in setup.py
[ ] Logging is used instead of print statements
[ ] Comments explain "why", not "what"
```

---

## 8. Connecting Weeks 1-3 to Week 4

### 8.1 What We've Covered

```
Week 1 (Days 1-4): Hardware Foundations
  ├── Digital circuits, logic gates
  ├── Microcontroller architecture (ARM Cortex)
  ├── Memory hierarchy (Flash, SRAM, registers)
  └── Bare-metal programming (GPIO, interrupts)

Week 2 (Days 5-8): System Software
  ├── OS concepts (threads, scheduling, mutexes)
  ├── PWM, motor control, PID
  ├── Communication protocols (UART, SPI, I2C, CAN)
  └── Embedded Linux, device trees, kernel modules

Week 3 (Days 9-12): Sensors and Integration
  ├── Cameras, LiDAR, IMU, encoders
  ├── Sensor fusion concepts
  ├── ROS2 architecture (DDS, QoS, lifecycle)
  └── Executor model, TF2, Nav2, ros2_control

Week 3-4 Bridge (Days 13-16): THIS WEEK
  ├── ROS2 communication deep dive
  ├── Concurrency and performance
  ├── Vehicle bring-up and first drive
  └── Code review and architecture understanding ← TODAY
```

### 8.2 What's Coming in Week 4

Week 4 focuses on **integration and autonomy**:

```
Week 4 (Days 17-20): Perception and Integration
  ├── Day 17: OpenCV fundamentals and lane detection pipeline
  ├── Day 18: Lane detection ROS2 integration, sensor fusion, and safety
  ├── Day 19: YOLOv5 object detection, transfer learning, and quantization
  └── Day 20: Hailo-10 NPU deployment and final integration demo
```

Everything from today's code review feeds directly into Week 4. You need to understand the existing codebase deeply before you can modify it for autonomous behavior.

### 8.3 Individual Pre-Work for Week 4

Before Day 17, each student should:

1. **Verify** that the vehicle boots and all sensors publish data
2. **Measure** the odometry accuracy (drive 1m forward, check /odom)
3. **Test** teleop driving to confirm motor control works
4. **Identify** one issue from today's code review and propose a fix

---

## 9. Review

### Key Takeaways from Today

1. **Code review is a skill** — understanding someone else's code requires systematic investigation, not just reading top to bottom. Use the checklist.

2. **The vehicle software stack has four clear layers**: hardware drivers (Team A), sensor drivers (Teams B & C), system orchestration (Team D), and autonomy (Nav2, coming in Week 4).

3. **Every concept from Days 1-15 appears in the real codebase**: PWM in the motor driver, PID in the controller, UART in the serial interface, threading in the executor, QoS in the topic configuration, TF2 in the coordinate frames.

4. **No subsystem works in isolation** — the motor driver needs odometry from encoders (Team A), the planner needs LiDAR data (Team C) on the costmap, SLAM needs camera images (Team B) and odometry (Team A), and the launch system (Team D) starts everything in the right order.

5. **Real code has rough edges** — the purpose of code review is not to criticize but to understand and improve. Every improvement proposal should be specific, actionable, and justified.

### The Biggest Lesson

The value of this course is not any single day's content. It's the ability to trace a signal from a navigation goal:

$$
\text{Goal} \xrightarrow{\text{Nav2}} \text{cmd\_vel} \xrightarrow{\text{ros2\_control}} \text{PWM} \xrightarrow{\text{H-bridge}} \text{Motor} \xrightarrow{\text{Encoder}} \text{Odometry} \xrightarrow{\text{TF2}} \text{Map Position}
$$

...understanding what happens at every stage, why it's designed that way, and how to debug when something goes wrong.

If you can do that after today, you're ready for Week 4.

---

*Next up: Day 17 — OpenCV Fundamentals and Lane Detection Pipeline, where we build a complete vision pipeline from color space conversion through Canny edge detection to Bird's Eye View lane tracking.*
