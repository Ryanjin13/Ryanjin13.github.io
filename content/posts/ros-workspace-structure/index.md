---
title: "ROS Workspace Structure"
date: 2024-07-03
description: "Understanding ROS catkin workspace organization and build process"
categories: ["Autonomous Driving"]
tags: ["ROS", "Catkin", "Build System"]
draft: false
---

## Overview

ROS uses the catkin build system to manage packages. Understanding the workspace structure is essential for ROS development.

## Catkin Workspace Structure

```
catkin_ws/
├── src/                 ← Source code
│   ├── CMakeLists.txt   ← Top-level cmake
│   ├── package1/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src/
│   │   ├── include/
│   │   └── launch/
│   └── package2/
├── build/               ← Build artifacts
├── devel/               ← Development environment
│   ├── setup.bash
│   ├── lib/
│   └── share/
└── install/             ← Installation (optional)
```

## Directory Purposes

### Source Space (src/)

Contains all package source code:

| Contents | Purpose |
|----------|---------|
| Package directories | Individual ROS packages |
| CMakeLists.txt | Top-level cmake file |
| .rosinstall | Workspace dependencies |

### Build Space (build/)

Compilation and dependency optimization:
- CMake cache files
- Makefile outputs
- Intermediate build files

### Devel Space (devel/)

Development environment ready for execution:

| Contents | Purpose |
|----------|---------|
| setup.bash | Environment setup script |
| lib/ | Compiled libraries |
| share/ | Package resources |
| bin/ | Executables |

### Install Space (install/)

Optional production deployment:
- Isolated from build artifacts
- Clean installation structure

## Build Process

### Workflow

```
Source Code → catkin_make → Build Directory → Devel/Install
                  │
                  ↓
            CMake + Make
                  │
                  ↓
           Compiled Binaries
```

### Building Workspace

```bash
cd ~/catkin_ws
catkin_make
```

### Clean Build

```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

## TurtleBot3 Packages

Example package structure:

```
src/
├── turtlebot3/
│   ├── turtlebot3_bringup/
│   ├── turtlebot3_description/
│   ├── turtlebot3_example/
│   ├── turtlebot3_navigation/
│   ├── turtlebot3_slam/
│   └── turtlebot3_teleop/
└── turtlebot3_simulations/
    ├── turtlebot3_gazebo/
    └── turtlebot3_fake/
```

## Package Components

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/my_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Package description</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
</package>
```

### Launch Files

```xml
<launch>
  <node pkg="my_package" type="my_node.py" name="my_node" output="screen">
    <param name="rate" value="10"/>
  </node>
</launch>
```

## Environment Setup

### Source Workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

### Add to Bashrc

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Check Environment

```bash
echo $ROS_PACKAGE_PATH
```

Should include: `/home/user/catkin_ws/src`

## Common Commands

| Command | Purpose |
|---------|---------|
| catkin_make | Build workspace |
| catkin_make -j4 | Build with 4 threads |
| catkin_make clean | Clean build |
| rospack list | List all packages |
| roscd package_name | Navigate to package |

## Best Practices

1. **One workspace** for related packages
2. **Source devel/setup.bash** after building
3. **Use package dependencies** properly
4. **Document packages** in package.xml
5. **Version control** the src/ directory
