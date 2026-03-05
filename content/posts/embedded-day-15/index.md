---
title: "Day 15 — ros2_control, Nav2, and First Vehicle Setup"
date: 2026-03-05
description: "ros2_control framework with diff_drive_controller, Nav2 navigation stack architecture, URDF/XACRO basics, and first vehicle kit setup"
categories: ["Autonomous Driving"]
tags: ["ros2_control", "Nav2", "URDF", "Navigation", "Costmap"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 15
draft: false
---

{{< katex >}}

## What You'll Learn

Over the past two days we built a solid understanding of ROS2's communication architecture ([Day 13](/posts/embedded-day-13/)) and callback execution model ([Day 14](/posts/embedded-day-14/)). Today we cross the boundary from **software concepts** to **physical hardware** — connecting ROS2 topics and controllers to actual motors, sensors, and wheels.

In this post, you will learn:

1. **ros2_control** — the framework that abstracts hardware behind a standard interface
2. **diff_drive_controller** — translating `cmd_vel` to wheel speeds (connecting to Day 6 PWM/encoders and Day 9 PID control)
3. **Nav2** — the complete autonomous navigation stack
4. **URDF/XACRO** — describing your robot's geometry for visualization and planning
5. **Vehicle bring-up** — running your first ROS2 commands on a real robot

By the end of today, you will have a mental model of how every piece fits together — from a high-level navigation goal down to individual wheel PWM signals.

---

## 1. ros2_control: Bridging ROS2 and Real Hardware

### 1.1 The Problem

Consider the path from a navigation goal to wheel motion:

```
"Go to position (10, 5)"
    → Nav2 plans a path
    → Local planner generates velocity commands: cmd_vel = (0.5 m/s, 0.1 rad/s)
    → ??? something converts this to left/right wheel speeds ???
    → ??? something sends PWM to motor drivers ???
    → ??? something reads encoder feedback ???
    → Wheels turn, robot moves
```

The "???" is where **ros2_control** lives. It provides a standardized framework for:

1. **Hardware Abstraction**: A plugin interface for different motor controllers, sensors, and actuators
2. **Controller Management**: Loading, configuring, and switching controllers at runtime
3. **Real-Time Loop**: A deterministic control loop that reads sensors and writes actuators

### 1.2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    ros2_control Framework                     │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Controller Manager                       │   │
│  │   ┌─────────────────┐  ┌─────────────────────────┐  │   │
│  │   │ diff_drive_      │  │ joint_state_            │  │   │
│  │   │ controller       │  │ broadcaster             │  │   │
│  │   │                  │  │                          │  │   │
│  │   │ cmd_vel →        │  │ joint_states →           │  │   │
│  │   │ wheel velocities │  │ TF2 odom broadcast      │  │   │
│  │   └────────┬─────────┘  └──────────┬──────────────┘  │   │
│  │            │ command interfaces      │ state interfaces│   │
│  └────────────┼─────────────────────────┼────────────────┘   │
│               ▼                         ▲                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            Resource Manager                           │   │
│  └──────────────────────────────────────────────────────┘   │
│               │                         ▲                     │
│               ▼ write()                 │ read()              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            Hardware Interface (Plugin)                 │   │
│  │   ┌──────────────┐  ┌──────────────┐                 │   │
│  │   │  Left Motor   │  │  Right Motor  │                │   │
│  │   │  velocity_cmd │  │  velocity_cmd │  ← write()    │   │
│  │   │  position_fb  │  │  position_fb  │  → read()     │   │
│  │   └──────┬───────┘  └──────┬───────┘                 │   │
│  └──────────┼──────────────────┼────────────────────────┘   │
└─────────────┼──────────────────┼────────────────────────────┘
              │                  │
              ▼                  ▼
         ┌─────────┐       ┌─────────┐
         │  Motor   │       │  Motor   │
         │  Driver  │       │  Driver  │
         │  (PWM)   │       │  (PWM)   │
         └────┬────┘       └────┬────┘
              │                  │
              ▼                  ▼
         ┌─────────┐       ┌─────────┐
         │  Left    │       │  Right   │
         │  Wheel   │       │  Wheel   │
         └─────────┘       └─────────┘
```

### 1.3 Key Components

**Controller Manager**: The orchestrator. It loads controller plugins, manages their lifecycle, and coordinates the real-time update loop.

**Controllers**: Plugins that implement specific control algorithms. Examples:
- `diff_drive_controller`: converts `cmd_vel` to wheel velocities
- `joint_trajectory_controller`: follows trajectories (for arms)
- `joint_state_broadcaster`: publishes joint states to topics

**Hardware Interface**: A plugin that talks to your specific hardware. It implements two key methods:
- `read()`: reads sensor values (encoder counts, IMU data)
- `write()`: sends commands to actuators (PWM duty cycles, velocities)

**Resource Manager**: Manages the lifecycle of hardware interfaces and maps them to controllers via **command interfaces** (write) and **state interfaces** (read).

### 1.4 The Control Loop

The ros2_control update loop runs at a fixed frequency (typically 50-1000 Hz):

```
┌──────────────────────────────────────┐
│         ros2_control loop            │
│                                      │
│   1. hardware.read()                 │  ← Read encoder positions
│   2. controller.update()             │  ← Compute new commands
│   3. hardware.write()                │  ← Send PWM to motors
│                                      │
│   Repeat at fixed rate (e.g., 50Hz)  │
└──────────────────────────────────────┘
```

This fixed-rate loop is critical for control stability. From Day 9 (PID control), we know that the control loop frequency affects the derivative and integral terms:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

In discrete time with period \(T\):

$$
u[k] = K_p e[k] + K_i T \sum_{i=0}^{k} e[i] + K_d \frac{e[k] - e[k-1]}{T}
$$

If \(T\) varies (jitter), the integral accumulates incorrectly and the derivative becomes noisy. This is why ros2_control uses a **dedicated real-time thread** separate from the ROS2 executor.

---

## 2. Differential Drive Controller

### 2.1 Differential Drive Kinematics Review

A differential drive robot has two independently driven wheels. By varying their speeds, the robot can move forward, backward, and rotate.

```
              Front
               ↑
     ┌─────────────────────┐
     │                     │
   ──┤  Left   ● center   ├──    ← wheel baseline L
   ──┤  Wheel             ├──
     │          Right      │
     │          Wheel      │
     └─────────────────────┘
```

The relationship between wheel velocities and robot motion (from Day 6):

**Forward kinematics** (wheel speeds to robot velocity):

$$
v = \frac{v_R + v_L}{2}
$$

$$
\omega = \frac{v_R - v_L}{L}
$$

where:
- \(v\) = linear velocity of the robot center (m/s)
- \(\omega\) = angular velocity (rad/s)
- \(v_R\) = right wheel linear velocity (m/s)
- \(v_L\) = left wheel linear velocity (m/s)
- \(L\) = wheel baseline (distance between wheels) (m)

**Inverse kinematics** (robot velocity to wheel speeds):

$$
v_L = v - \frac{\omega L}{2}
$$

$$
v_R = v + \frac{\omega L}{2}
$$

### 2.2 From cmd_vel to Wheel PWM

The complete data flow:

```
/cmd_vel (Twist)                     diff_drive_controller
  linear.x = 0.5 m/s    ──────►    Inverse kinematics:
  angular.z = 0.3 rad/s             v_L = 0.5 - 0.3×0.15 = 0.455 m/s
                                     v_R = 0.5 + 0.3×0.15 = 0.545 m/s
                                              │
                                              ▼
                                     Convert to wheel angular velocity:
                                     ω_L = v_L / r = 0.455 / 0.033 = 13.8 rad/s
                                     ω_R = v_R / r = 0.545 / 0.033 = 16.5 rad/s
                                              │
                                              ▼
                                     Hardware Interface write():
                                     left_wheel.velocity_command = 13.8
                                     right_wheel.velocity_command = 16.5
                                              │
                                              ▼
                                     Motor driver (e.g., L298N):
                                     left_PWM = PID(target=13.8, actual=ω_L_measured)
                                     right_PWM = PID(target=16.5, actual=ω_R_measured)
```

Here \(r\) is the wheel radius. For the Hawonder vehicle with 33mm wheels and 300mm baseline:

$$
r = 0.033 \text{ m}, \quad L = 0.30 \text{ m}
$$

### 2.3 Odometry from Wheel Encoders

The reverse direction: reading wheel encoder ticks to estimate the robot's position.

Given encoder counts over a time step \(\Delta t\):

$$
\Delta \theta_L = \frac{2\pi \cdot \Delta \text{ticks}_L}{\text{CPR}}
$$

$$
\Delta \theta_R = \frac{2\pi \cdot \Delta \text{ticks}_R}{\text{CPR}}
$$

where CPR is the counts per revolution of the encoder.

$$
\Delta s_L = r \cdot \Delta \theta_L, \quad \Delta s_R = r \cdot \Delta \theta_R
$$

$$
\Delta s = \frac{\Delta s_L + \Delta s_R}{2}, \quad \Delta \phi = \frac{\Delta s_R - \Delta s_L}{L}
$$

Update the pose:

$$
x_{k+1} = x_k + \Delta s \cdot \cos\left(\phi_k + \frac{\Delta \phi}{2}\right)
$$

$$
y_{k+1} = y_k + \Delta s \cdot \sin\left(\phi_k + \frac{\Delta \phi}{2}\right)
$$

$$
\phi_{k+1} = \phi_k + \Delta \phi
$$

The `diff_drive_controller` does all of this automatically and publishes the result on `/odom` and broadcasts the `odom → base_link` TF transform (connecting to Day 14 TF2 concepts).

### 2.4 Hardware Interface Implementation

Here's what a custom hardware interface looks like. This is the bridge between ros2_control and your specific motor controller:

```python
#!/usr/bin/env python3
"""hawonder_hardware.py — ros2_control hardware interface for Hawonder vehicle.

This is a conceptual Python example. In production, hardware interfaces
are typically written in C++ for real-time performance.
"""

import math


class HawonderHardware:
    """
    Hardware interface for the Hawonder differential drive vehicle.

    Command interfaces:
      - left_wheel/velocity   (rad/s)
      - right_wheel/velocity  (rad/s)

    State interfaces:
      - left_wheel/position   (rad, from encoder)
      - left_wheel/velocity   (rad/s, computed from encoder)
      - right_wheel/position  (rad)
      - right_wheel/velocity  (rad/s)
    """

    def __init__(self):
        # Hardware parameters
        self.wheel_radius = 0.033       # 33mm
        self.wheel_separation = 0.30    # 300mm
        self.encoder_cpr = 1440         # Counts per revolution

        # State variables
        self.left_position = 0.0     # radians
        self.right_position = 0.0
        self.left_velocity = 0.0     # rad/s
        self.right_velocity = 0.0

        # Command variables
        self.left_velocity_cmd = 0.0
        self.right_velocity_cmd = 0.0

        # Previous encoder readings
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = 0.0

    def on_configure(self):
        """Initialize hardware communication (serial, GPIO, etc.)."""
        # Open serial port to motor controller
        # self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        print("Hardware configured: serial port opened")
        return True

    def on_activate(self):
        """Enable motors."""
        # Send enable command to motor driver
        # self.serial.write(b'ENABLE\n')
        print("Hardware activated: motors enabled")
        return True

    def read(self, current_time):
        """Read encoder values and compute velocities.

        Called by ros2_control at the control loop frequency.
        """
        # Read raw encoder ticks from hardware
        # left_ticks, right_ticks = self.read_encoders()

        # For demonstration, simulate encoder readings
        left_ticks = self.prev_left_ticks + int(
            self.left_velocity_cmd * self.encoder_cpr / (2 * math.pi) * 0.02
        )
        right_ticks = self.prev_right_ticks + int(
            self.right_velocity_cmd * self.encoder_cpr / (2 * math.pi) * 0.02
        )

        dt = current_time - self.prev_time
        if dt > 0:
            # Convert tick deltas to angular displacement
            d_left = (left_ticks - self.prev_left_ticks) * 2 * math.pi / self.encoder_cpr
            d_right = (right_ticks - self.prev_right_ticks) * 2 * math.pi / self.encoder_cpr

            # Update position (cumulative)
            self.left_position += d_left
            self.right_position += d_right

            # Compute velocity
            self.left_velocity = d_left / dt
            self.right_velocity = d_right / dt

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_time = current_time

    def write(self):
        """Send velocity commands to motors.

        Called by ros2_control at the control loop frequency.
        """
        # Convert rad/s to motor-specific command format
        # E.g., for a motor driver that accepts RPM:
        left_rpm = self.left_velocity_cmd * 60 / (2 * math.pi)
        right_rpm = self.right_velocity_cmd * 60 / (2 * math.pi)

        # Send to hardware
        # self.serial.write(f'M {left_rpm:.1f} {right_rpm:.1f}\n'.encode())

    def on_deactivate(self):
        """Disable motors (safety stop)."""
        self.left_velocity_cmd = 0.0
        self.right_velocity_cmd = 0.0
        self.write()
        # self.serial.write(b'DISABLE\n')
        print("Hardware deactivated: motors disabled")
        return True
```

### 2.5 ros2_control Configuration (YAML)

The controller configuration is specified in a YAML file:

```yaml
# config/diff_drive_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz — control loop frequency

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    # Joint names (must match URDF)
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    # Wheel geometry
    wheel_separation: 0.30        # meters (baseline L)
    wheel_radius: 0.033           # meters

    # Odometry configuration
    publish_rate: 50.0            # Hz
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    publish_odom_tf: true

    # Velocity limits
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 1.0    # m/s
    linear.x.min_velocity: -0.5   # m/s (reverse)
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 2.0

    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 2.0   # rad/s
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 3.0

    # Timeout: stop if no cmd_vel received for 500ms
    cmd_vel_timeout: 0.5
```

---

## 3. URDF and XACRO: Describing Robot Geometry

### 3.1 What Is URDF?

**URDF (Unified Robot Description Format)** is an XML format that describes a robot's physical structure — links (rigid bodies), joints (connections), visual appearance, and collision geometry.

Every ROS2 robot needs a URDF because:
- TF2 uses it to compute transforms between frames
- rviz2 uses it to visualize the robot
- ros2_control uses it to identify joints and their types
- Nav2 uses it for the robot's footprint

### 3.2 URDF Structure

A URDF consists of **links** (parts) connected by **joints**:

```
              joint: base_to_left_wheel (continuous)
              ┌─────────────────┐
link: base ──►│                 │──► link: left_wheel
              └─────────────────┘

              joint: base_to_right_wheel (continuous)
              ┌─────────────────┐
link: base ──►│                 │──► link: right_wheel
              └─────────────────┘

              joint: base_to_camera (fixed)
              ┌─────────────────┐
link: base ──►│                 │──► link: camera_link
              └─────────────────┘
```

### 3.3 Complete URDF Example

```xml
<?xml version="1.0"?>
<robot name="hawonder_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ════════════════════════════════════════════ -->
  <!-- Base Link: Main body of the vehicle          -->
  <!-- ════════════════════════════════════════════ -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.30 0.20 0.08"/>  <!-- 30cm x 20cm x 8cm -->
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.30 0.20 0.08"/>
      </geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>  <!-- 2 kg -->
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- ════════════════════════════════════════════ -->
  <!-- Base Footprint: Ground projection            -->
  <!-- ════════════════════════════════════════════ -->
  <link name="base_footprint"/>

  <joint name="base_footprint_to_base" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.033" rpy="0 0 0"/>  <!-- wheel radius above ground -->
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- Left Wheel                                   -->
  <!-- ════════════════════════════════════════════ -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>  <!-- r=33mm, width=20mm -->
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0"
               izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="${-pi/2} 0 0"/>  <!-- left side, rotated -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- Right Wheel                                  -->
  <!-- ════════════════════════════════════════════ -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0"
               izz="0.0001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- Front Caster Wheel (passive)                 -->
  <!-- ════════════════════════════════════════════ -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0"
               iyy="0.00001" iyz="0"
               izz="0.00001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.12 0 -0.018" rpy="0 0 0"/>
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- Camera                                       -->
  <!-- ════════════════════════════════════════════ -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.06 0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.06" rpy="0 0 0"/>  <!-- front, slightly above -->
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- LiDAR                                        -->
  <!-- ════════════════════════════════════════════ -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0.1 0.8 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>  <!-- top center -->
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- IMU                                          -->
  <!-- ════════════════════════════════════════════ -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.8 0.8 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.04" rpy="0 0 0"/>  <!-- center of body -->
  </joint>

  <!-- ════════════════════════════════════════════ -->
  <!-- ros2_control Hardware Interface              -->
  <!-- ════════════════════════════════════════════ -->
  <ros2_control name="HawonderSystem" type="system">
    <hardware>
      <plugin>hawonder_hardware/HawonderSystemHardware</plugin>
      <param name="serial_port">/dev/ttyUSB0</param>
      <param name="baud_rate">115200</param>
    </hardware>

    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

</robot>
```

### 3.4 XACRO: Macros for DRY URDF

Raw URDF is verbose. **XACRO** (XML Macros) adds variables, macros, and includes:

```xml
<?xml version="1.0"?>
<robot name="hawonder_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties (variables) -->
  <xacro:property name="wheel_radius" value="0.033"/>
  <xacro:property name="wheel_width" value="0.02"/>
  <xacro:property name="wheel_separation" value="0.30"/>
  <xacro:property name="body_length" value="0.30"/>
  <xacro:property name="body_width" value="0.20"/>
  <xacro:property name="body_height" value="0.08"/>
  <xacro:property name="pi" value="3.14159265359"/>

  <!-- Wheel macro — define once, use twice -->
  <xacro:macro name="wheel" params="prefix y_offset">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0"
                 iyy="0.0001" iyz="0"
                 izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${y_offset} 0" rpy="${-pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="left" y_offset="${wheel_separation/2}"/>
  <xacro:wheel prefix="right" y_offset="${-wheel_separation/2}"/>

</robot>
```

Process XACRO to generate URDF:

```bash
# Convert XACRO to URDF
xacro hawonder.urdf.xacro > hawonder.urdf

# Validate URDF
check_urdf hawonder.urdf

# View in rviz2
ros2 launch urdf_tutorial display.launch.py model:=hawonder.urdf.xacro
```

---

## 4. Nav2: The Complete Navigation Stack

### 4.1 What Nav2 Does

Nav2 (Navigation 2) is ROS2's autonomous navigation framework. Given a goal position on a map, Nav2 will:

1. Plan a global path from current position to the goal
2. Generate local velocity commands to follow the path
3. Avoid obstacles detected in real time
4. Recover from stuck situations
5. Report success or failure

### 4.2 Architecture Overview

```
                          ┌──────────────────────┐
                          │    BT Navigator       │
                          │  (Behavior Tree)      │
                          │                       │
                          │  "Navigate to Pose"   │
                          └──────────┬───────────┘
                                     │
                    ┌────────────────┼────────────────┐
                    │                │                 │
           ┌────────▼─────┐  ┌──────▼──────┐  ┌─────▼──────────┐
           │   Global     │  │   Local     │  │   Recovery     │
           │   Planner    │  │   Planner   │  │   Behaviors    │
           │              │  │             │  │                │
           │  NavFn /     │  │  DWB /      │  │  Spin / Wait / │
           │  Smac /      │  │  MPPI /     │  │  Backup /      │
           │  Theta*      │  │  RPP        │  │  Clear costmap │
           └──────┬───────┘  └──────┬──────┘  └────────────────┘
                  │                 │
                  │                 │
           ┌──────▼─────────────────▼──────┐
           │         Costmap2D             │
           │                               │
           │  ┌─────────────────────────┐  │
           │  │    Inflation Layer      │  │
           │  │  (safety margin)        │  │
           │  ├─────────────────────────┤  │
           │  │    Obstacle Layer       │  │
           │  │  (real-time sensors)    │  │
           │  ├─────────────────────────┤  │
           │  │    Static Layer         │  │
           │  │  (pre-built map)        │  │
           │  └─────────────────────────┘  │
           └───────────────────────────────┘
                          │
                    ┌─────▼─────┐
                    │  /cmd_vel │
                    │  (Twist)  │
                    └─────┬─────┘
                          │
                    ┌─────▼─────────────┐
                    │  ros2_control      │
                    │  diff_drive_ctrl   │
                    └───────────────────┘
```

### 4.3 BT Navigator: Behavior Tree Orchestration

Nav2 uses a **Behavior Tree (BT)** to orchestrate navigation. Unlike a simple state machine, a behavior tree is hierarchical and composable.

The default navigation behavior tree:

```
                    NavigateRecovery (recovery node)
                    /                              \
           NavigateWithReplanning          RecoveryActions
           (pipeline sequence)              (round robin)
           /          |          \           /    |    \
      RateController  ComputePath  FollowPath  Spin  Wait  Backup
      (1 Hz replan)
```

This tree says:
1. Compute a global path and follow it, replanning every 1 second
2. If following fails (stuck), try recovery actions: spin in place, wait, back up
3. If all recoveries fail, abort

Behavior trees are powerful because you can customize the navigation logic by editing XML:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}"
                             planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.3" backup_speed="0.1"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 4.4 Global Planner: Finding the Path

The global planner finds a path from the robot's current position to the goal on the global costmap. Nav2 provides several planner plugins:

**NavFn (Navigation Function)**:
- Implements **Dijkstra's algorithm** (guaranteed shortest path) or **A*** (faster with heuristic)
- Works on a 2D grid costmap
- Simple, reliable, widely used
- Path cost considers distance and obstacle proximity

The algorithm finds the minimum-cost path through the costmap grid:

$$
g(n) = \min_{m \in \text{neighbors}(n)} \left[ g(m) + c(m, n) \right]
$$

where \(g(n)\) is the cost to reach cell \(n\) and \(c(m, n)\) is the traversal cost from \(m\) to \(n\).

For A*, the priority queue uses:

$$
f(n) = g(n) + h(n)
$$

where \(h(n)\) is the heuristic (typically Euclidean distance to goal).

**Smac Planner (State Lattice)**:
- Plans in \((x, y, \theta)\) space, not just \((x, y)\)
- Respects vehicle kinematic constraints (minimum turning radius)
- Better paths for non-holonomic vehicles (cars, trucks)
- More computationally expensive

### 4.5 Local Planner: Following the Path

The local planner generates velocity commands to follow the global path while avoiding dynamic obstacles. It runs at a higher frequency (typically 20Hz) than the global planner.

**DWB (Dynamic Window Based)**:

DWB searches the **velocity space** — all possible \((v, \omega)\) pairs — and evaluates each trajectory against multiple critics:

```
Velocity space:

ω (angular)
  ↑
  │    × bad    ● good
  │  × bad  ● ● ● good
  │    × ●  ★ ● good        ★ = selected velocity
  │      ●  ● ● × bad
  │        ×   × × bad
  └──────────────────→ v (linear)

Each (v, ω) pair generates a trajectory.
Critics score each trajectory.
Best trajectory wins.
```

DWB critics include:
- **GoalDist**: prefer trajectories that end close to the global path
- **PathDist**: prefer trajectories that stay close to the global path
- **ObstacleCost**: avoid trajectories that pass through obstacles
- **GoalAlign**: prefer trajectories heading toward the goal
- **RotateToGoal**: rotate to match the goal orientation at the end

The cost function is a weighted sum:

$$
J(v, \omega) = w_1 \cdot \text{GoalDist} + w_2 \cdot \text{PathDist} + w_3 \cdot \text{ObstacleCost} + w_4 \cdot \text{GoalAlign}
$$

The velocity pair with the **lowest cost** is sent as `cmd_vel`.

**MPPI (Model Predictive Path Integral)**:
- Samples thousands of random trajectories
- Evaluates each against a cost function
- Uses weighted average of best trajectories as the command
- More computationally expensive but produces smoother paths

### 4.6 Costmap2D: The World Model

The costmap is a 2D grid where each cell has a cost value from 0 (free) to 254 (lethal obstacle). It's built from multiple **layers** stacked together:

```
           Final Costmap (merged)
           ┌────────────────────────┐
           │ 0  0  0  50 100 254 254│
           │ 0  0  0  50 100 254    │
           │ 0  0  0  50 100        │  ← inflation gradient
           │ 0  0  0  50            │     around obstacle
           │ 0  0  0  0   0   0   0 │
           └────────────────────────┘

           = Static Layer + Obstacle Layer + Inflation Layer
```

**Static Layer**: Loaded from a pre-built map (e.g., from SLAM). Provides known walls, furniture, boundaries. Does not change at runtime.

**Obstacle Layer**: Updated in real time from sensor data (LiDAR, depth camera). Marks cells where obstacles are detected. Clears cells when obstacles move away (raycasting).

**Inflation Layer**: Expands obstacles by the robot's radius plus a safety margin. This creates a **gradient** around obstacles:

$$
\text{cost}(d) = \begin{cases}
254 & \text{if } d \leq r_{\text{robot}} \\
\text{exponential decay} & \text{if } r_{\text{robot}} < d \leq r_{\text{inflation}} \\
0 & \text{if } d > r_{\text{inflation}}
\end{cases}
$$

where \(d\) is the distance from the nearest obstacle.

The inflation radius ensures the planner keeps the robot's center far enough from obstacles that the robot body won't collide:

```
        Real obstacle: ███

        After inflation:
                 ░░░░░░░
               ░░░▒▒▒▒▒░░░
             ░░░▒▒▓▓▓▓▓▒▒░░░
            ░░▒▒▓▓████▓▓▒▒░░
             ░░░▒▒▓▓▓▓▓▒▒░░░
               ░░░▒▒▒▒▒░░░
                 ░░░░░░░

        ███ = lethal (254)    ← actual obstacle
        ▓▓ = inscribed (253)  ← robot center here = collision
        ▒▒ = high cost        ← robot center here = too close
        ░░ = low cost         ← safe but close
```

### 4.7 Recovery Behaviors

When the robot gets stuck (local planner fails to find a valid trajectory), Nav2 executes recovery behaviors:

| Recovery | What It Does | When It Helps |
|----------|-------------|---------------|
| **Spin** | Rotate in place (default: 90 degrees) | Clears sensor blind spots, gets new view |
| **Backup** | Drive backward a short distance | Gets away from close obstacle |
| **Wait** | Stop and wait (default: 5 seconds) | Dynamic obstacle may move away |
| **Clear Costmap** | Reset obstacle layer | Stale sensor data causing phantom obstacles |

These are tried in sequence. If all fail, the navigation goal is aborted.

### 4.8 Nav2 Configuration

```yaml
# config/nav2_params.yaml

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning.xml"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05            # 5cm per cell
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55    # robot radius + safety margin

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      decel_lim_x: -2.5
      acc_lim_theta: 3.2
      decel_lim_theta: -3.2
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint",
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

---

## 5. Launch Files: Bringing It All Together

### 5.1 ROS2 Launch System

ROS2 uses Python launch files (or XML/YAML) to start multiple nodes with specific configurations:

```python
#!/usr/bin/env python3
"""launch/hawonder_bringup.launch.py — Complete vehicle bringup."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('hawonder_bringup')

    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'hawonder.urdf.xacro')

    # Declare launch arguments
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulation instead of real hardware'
    )

    # ─── Robot State Publisher ───
    # Publishes URDF to /robot_description and TF static transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', urdf_file]),
            'publish_frequency': 30.0,
        }]
    )

    # ─── ros2_control Node ───
    # Manages controllers and hardware interface
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'diff_drive_controller.yaml')
        ],
        output='screen',
    )

    # ─── Spawn Controllers ───
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    spawn_joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # ─── Camera Node ───
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
        }],
        output='screen',
    )

    # ─── LiDAR Node ───
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': '/scan',
            'frame_id': 'lidar_link',
            'port_name': '/dev/ttyUSB1',
        }],
        output='screen',
    )

    # ─── IMU Node ───
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        parameters=[{
            'port': '/dev/ttyUSB2',
            'frame_id': 'imu_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim,
        robot_state_publisher,
        ros2_control_node,
        spawn_diff_drive,
        spawn_joint_broadcaster,
        camera_node,
        lidar_node,
        imu_node,
    ])
```

### 5.2 Running the Launch File

```bash
# Build the workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch the vehicle
ros2 launch hawonder_bringup hawonder_bringup.launch.py

# In another terminal, verify everything is running:
ros2 node list
# Expected output:
# /robot_state_publisher
# /controller_manager
# /diff_drive_controller
# /joint_state_broadcaster
# /v4l2_camera
# /ldlidar_stl_ros2_node
# /imu_node

ros2 topic list
# Expected output:
# /camera/image_raw
# /scan
# /imu/data
# /odom
# /cmd_vel
# /joint_states
# /tf
# /tf_static
# /robot_description
```

---

## 6. Hands-On Lab: First Vehicle Drive

### Lab 1: Verify Node Startup

After launching the vehicle, run the following checks:

```bash
# Check all nodes are alive
ros2 node list

# Check topic flow
ros2 topic hz /camera/image_raw    # Should show ~30 Hz
ros2 topic hz /scan                 # Should show ~10 Hz
ros2 topic hz /imu/data             # Should show ~100 Hz
ros2 topic hz /odom                 # Should show ~50 Hz

# Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf — should show the full tree from map down to sensors

# Visualize the full topology
ros2 run rqt_graph rqt_graph
```

### Lab 2: Manual Driving with cmd_vel

Drive the vehicle manually by publishing velocity commands:

```bash
# Drive forward at 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotate left at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Drive in a circle (forward + rotate)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

For interactive driving, use `teleop_twist_keyboard`:

```bash
# Install
sudo apt install ros-humble-teleop-twist-keyboard

# Run
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Controls:
#   u    i    o         ← forward + turn
#   j    k    l         ← stop / turn in place
#   m    ,    .         ← backward + turn
```

### Lab 3: Verify Odometry

While driving, check that odometry is reasonable:

```bash
# Watch odometry in real time
ros2 topic echo /odom --field pose.pose.position

# Expected: x increases when driving forward, y changes when turning

# Check odom → base_link transform
ros2 run tf2_ros tf2_echo odom base_link
# Should show smooth, continuously changing transform
```

Drive the robot in a 1-meter square and check if odometry says it returned close to the origin. The error gives you an idea of the odometry accuracy:

$$
\text{odometry error} = \sqrt{(x_{\text{final}} - x_{\text{start}})^2 + (y_{\text{final}} - y_{\text{start}})^2}
$$

For a well-calibrated differential drive, this should be less than 10% of the total distance traveled.

### Lab 4: rqt_graph Topology Analysis

Generate and analyze the full node topology:

```bash
ros2 run rqt_graph rqt_graph
```

Verify these connections exist:

```
Expected data flow:

Camera → /camera/image_raw → (available for perception)
LiDAR → /scan → (available for costmap)
IMU → /imu/data → (available for sensor fusion)
Encoders → HW Interface → diff_drive_controller → /odom → TF2 (odom→base_link)
/cmd_vel → diff_drive_controller → HW Interface → Motors
robot_state_publisher → /tf_static (all static sensor frames)
```

Screenshot this graph for the Day 16 team presentation.

---

## 7. Team Module Assignment

For tomorrow's code review presentation (Day 16), each team is assigned a module of the Hawonder vehicle codebase. Here are the assignments and what to investigate:

### Team A: Motor Driver + ros2_control + Hall Odometry
- How does the hardware interface plugin communicate with the motor controller board?
- What is the control loop frequency? Is it sufficient for stable control?
- How are encoder ticks converted to odometry? Check the math against Section 2.3.
- What happens if the serial connection drops?

### Team B: Camera Node + Depth Stream Publishing
- What QoS profile is used for image topics? Is it appropriate? (Reference Day 13 QoS)
- Is the camera using compressed transport or raw?
- How is the depth image aligned with the RGB image?
- What is the actual publishing frequency vs. the configured frequency?

### Team C: IMU + 1D LiDAR Nodes + TF2 Frame Configuration
- Are the TF2 static transforms correct? Measure the physical sensor positions.
- What coordinate conventions does the IMU driver use? (NED vs ENU)
- How is the LiDAR scan data structured? What are the min/max angles?
- Is there a TF2 tree break (disconnected frames)?

### Team D: Launch Files + Parameter Management + RTAB-Map Integration
- Are all parameters in YAML files or hardcoded?
- What happens if a node fails to start? Is there error handling?
- How does RTAB-Map integrate with the TF tree?
- Can you switch between SLAM and localization modes?

---

## 8. Review

### Key Takeaways

1. **ros2_control separates controllers from hardware** — the diff_drive_controller doesn't know if it's talking to a real motor or a simulation. The hardware interface plugin handles the specifics.

2. **Differential drive kinematics** converts between robot velocity \((v, \omega)\) and wheel velocities \((v_L, v_R)\). The controller does this automatically using the configured wheel separation and radius.

3. **Nav2 is a complete navigation stack** with three main components: global planner (path finding), local planner (velocity commands), and costmap (world model). Behavior trees orchestrate the whole process.

4. **Costmap layers stack**: static (pre-built map) + obstacle (real-time sensors) + inflation (safety margin). The inflation layer is critical — without it, the planner would plan paths right next to walls.

5. **URDF describes the robot's geometry** — links, joints, sensor positions. This is the single source of truth for TF2 transforms, visualization, and planning.

6. **Launch files** bring everything together — they start all nodes with the correct parameters and configurations in the right order.

### Connection to Other Days

- **Day 6 (PWM/Motor Control)**: ros2_control's hardware interface is where Day 6's PWM and H-bridge code lives
- **Day 9 (Sensors)**: All sensor data flows through the topics we set up today
- **Day 13 (ROS2 Architecture)**: QoS policies from Day 13 determine how reliably sensor data reaches Nav2
- **Day 14 (Executors/TF2)**: The TF2 tree from Day 14 is populated by the URDF and odometry we configured today
- **Day 16 (Tomorrow)**: Teams present their analysis of the actual codebase running on this vehicle

### Quick Self-Check

1. What are the three main components of the ros2_control architecture?
2. Given cmd_vel = (0.5 m/s, 1.0 rad/s) and wheel separation L = 0.3m, what are the left and right wheel velocities?
3. What is the difference between the global costmap and the local costmap?
4. Why does the inflation layer exist? What would happen without it?
5. What is the difference between a URDF `fixed` joint and a `continuous` joint?

> **Answer to Q2**: \(v_L = 0.5 - \frac{1.0 \times 0.3}{2} = 0.35\) m/s, \(v_R = 0.5 + \frac{1.0 \times 0.3}{2} = 0.65\) m/s

---

*Next up: [Day 16 — Team Code Review and Architecture Presentation](/posts/embedded-day-16/) — where each team presents their analysis of a vehicle subsystem, connecting all the knowledge from Weeks 1-3 into a complete understanding of the autonomous driving stack.*
