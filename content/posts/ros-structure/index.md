---
title: "ROS Structure"
date: 2024-06-25
description: "Understanding Robot Operating System architecture and workspace organization"
categories: ["Autonomous Driving"]
tags: ["ROS", "Turtlebot3", "Robotics"]
draft: false
---

## Overview

ROS (Robot Operating System) is a flexible framework for writing robot software. Understanding its structure is essential for robotics development.

## Workspace Structure

```
catkin_ws/
├── src/                    # Source space
│   ├── package1/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src/
│   │   ├── include/
│   │   ├── launch/
│   │   └── config/
│   └── package2/
├── build/                  # Build space
├── devel/                  # Development space
└── install/                # Install space (optional)
```

## Core Concepts

### Nodes

Independent executable processes that perform computation.

```python
#!/usr/bin/env python
import rospy

rospy.init_node('my_node')
rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    # Do work
    rate.sleep()
```

### Topics

Named buses for nodes to exchange messages (publish/subscribe).

```python
# Publisher
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub.publish(msg)

# Subscriber
def callback(msg):
    rospy.loginfo(msg.data)

sub = rospy.Subscriber('/scan', LaserScan, callback)
```

### Services

Request/response communication between nodes.

```python
# Service Server
def handle_request(req):
    return MyServiceResponse(result)

srv = rospy.Service('my_service', MyService, handle_request)

# Service Client
rospy.wait_for_service('my_service')
client = rospy.ServiceProxy('my_service', MyService)
response = client(request)
```

### Messages

Data structures for communication.

```
# geometry_msgs/Twist.msg
Vector3 linear
Vector3 angular
```

## Package Structure

### package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Package description</description>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <exec_depend>rospy</exec_depend>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/my_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Launch Files

```xml
<!-- my_launch.launch -->
<launch>
  <node pkg="my_package" type="node1.py" name="node1" output="screen"/>
  <node pkg="my_package" type="node2.py" name="node2">
    <param name="rate" value="10"/>
  </node>
</launch>
```

## Build Process

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# Initialize
catkin_make

# Source environment
source devel/setup.bash

# Build specific package
catkin_make --pkg my_package
```

## Common Commands

| Command | Description |
|---------|-------------|
| `roscore` | Start ROS master |
| `rosrun pkg node` | Run a node |
| `roslaunch pkg file.launch` | Launch multiple nodes |
| `rostopic list` | List active topics |
| `rostopic echo /topic` | Print topic messages |
| `rosnode list` | List active nodes |
| `rosmsg show Type` | Show message definition |
| `rqt_graph` | Visualize node graph |

## TurtleBot3 Packages

```
turtlebot3/
├── turtlebot3_bringup/     # Robot startup
├── turtlebot3_slam/        # SLAM mapping
├── turtlebot3_navigation/  # Autonomous nav
├── turtlebot3_description/ # URDF models
└── turtlebot3_simulations/ # Gazebo sim
```
