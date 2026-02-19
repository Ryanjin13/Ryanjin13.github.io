---
title: "ROS2 Architecture: A Structural Deep Dive"
date: 2026-02-19
description: "A comprehensive analysis of ROS2's layered architecture, DDS-based communication, Node-Executor model, and the structural relationships between each component"
categories: ["Robotics"]
tags: ["ROS2", "Robot Operating System", "DDS", "Middleware", "Robotics Architecture"]
draft: false
---

{{< katex >}}

## Overview

ROS2 (Robot Operating System 2) is a **middleware framework** for robot software development. Despite the name "Operating System," it is not an actual OS — it's a **communication framework + development toolchain** running on top of Linux, Windows, or macOS.

ROS2 was **fundamentally redesigned** from ROS1 to overcome its limitations (single Master dependency, lack of real-time support, no security). The most significant structural change is the adoption of **DDS (Data Distribution Service)** as the communication layer.

---

## 1. Overall Architecture Layers

ROS2's architecture is divided into **four horizontal layers**:

```
┌──────────────────────────────────────────────────┐
│              Application Layer                    │
│   (User Nodes, Launch Files, Parameters)         │
├──────────────────────────────────────────────────┤
│              ROS Client Library (rcl)             │
│   rclcpp (C++)  │  rclpy (Python)  │  rclrs     │
├──────────────────────────────────────────────────┤
│              ROS Middleware Interface (rmw)       │
│   rmw_fastrtps  │  rmw_cyclonedds  │  rmw_...   │
├──────────────────────────────────────────────────┤
│              DDS Implementation                   │
│   Fast DDS  │  Cyclone DDS  │  Connext DDS      │
├──────────────────────────────────────────────────┤
│              Transport Layer                      │
│   UDP / Shared Memory / TCP                      │
└──────────────────────────────────────────────────┘
```

The core design principle behind this layering is **Separation of Concerns**. Each layer interacts with the one below only through well-defined interfaces, without needing to know the implementation details.

---

## 2. Role of Each Layer

### 2.1 Application Layer

The topmost layer where user code resides. The fundamental unit of execution is the **Node**.

```
Application Layer
├── Node (basic unit of execution)
│   ├── Publisher    ─── Topic ──→  Subscriber
│   ├── Service Server  ←── Request/Response ──→  Service Client
│   ├── Action Server   ←── Goal/Feedback/Result ──→  Action Client
│   └── Parameter Server
├── Launch System (multi-Node orchestration)
├── Lifecycle Management (Node state transitions)
└── Component (composing Nodes within a single process)
```

### 2.2 ROS Client Library (rcl)

`rcl` is a **language-independent C library** that serves as the common foundation for all client libraries. `rclcpp` (C++) and `rclpy` (Python) are language-specific bindings built on top of `rcl`.

```
rclcpp / rclpy / rclrs
       │
       ▼
   ┌───────┐
   │  rcl   │  ← Language-independent C library
   │        │     (Node creation, Topic management, QoS config)
   └───┬───┘
       │
       ▼
   ┌───────┐
   │  rmw   │  ← Middleware abstraction interface
   └───────┘
```

Thanks to this structure, **identical DDS behavior is guaranteed regardless of the language used**. Communication between a C++ Node and a Python Node happens directly without intermediate translation.

### 2.3 ROS Middleware Interface (rmw)

`rmw` is the most critical **abstraction layer** in the ROS2 architecture. It applies the **Adapter Pattern** to make DDS vendor implementations interchangeable.

```
         rmw API (abstract interface)
        ╱          │          ╲
rmw_fastrtps   rmw_cyclonedds   rmw_connextdds
       │           │                │
   Fast DDS    Cyclone DDS    Connext DDS
```

This allows switching DDS implementations with a single environment variable — no user code changes required:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2.4 DDS Layer

**DDS (Data Distribution Service)** is a **publish-subscribe communication standard** defined by the OMG (Object Management Group). ROS2 adopted DDS for these structural advantages:

- **Decentralized**: Automatic participant discovery without ROS1's Master Node
- **QoS (Quality of Service)**: Fine-grained control over communication behavior
- **Real-time**: RTPS (Real-Time Publish-Subscribe) protocol support
- **Security**: Built-in DDS Security standard

---

## 3. Communication Patterns

ROS2 provides three core communication patterns, each serving a different structural purpose.

### 3.1 Topic (Asynchronous Streaming)

```
Publisher ──── Topic ────→ Subscriber
   │         "/cmd_vel"        │
   │                           │
   │    (1:N, N:1, N:N)       │
   │                           │
Publisher ──── Topic ────→ Subscriber
                           Subscriber
```

- **Pattern**: Publish-Subscribe (asynchronous, unidirectional)
- **Use case**: Sensor data streaming, continuous state updates
- **Structural property**: **Loose coupling** between publishers and subscribers — they don't need to know about each other

### 3.2 Service (Synchronous Request-Response)

```
Client ─── Request ──→ Server
       ←── Response ──┘

   (1:1 synchronous call)
```

- **Pattern**: Request-Response (synchronous, bidirectional)
- **Use case**: Configuration changes, state queries
- **Structural property**: **Tight coupling** — the client blocks until the response is received

### 3.3 Action (Asynchronous Long-Running Tasks)

```
Client ─── Goal ──────→ Server
       ←── Feedback ───┘ (progress, repeated)
       ←── Result ─────┘ (final result, once)
       ─── Cancel ─────→ (cancellation request)
```

- **Pattern**: Goal-Feedback-Result (asynchronous, bidirectional)
- **Use case**: Navigation, manipulation, and other long-duration tasks
- **Structural property**: Internally composed of **2 Topics + 3 Services**

Breaking down the internal structure of an Action:

```
Action
├── Service: SendGoal      (send the goal)
├── Service: CancelGoal    (request cancellation)
├── Service: GetResult     (receive the result)
├── Topic: FeedbackMessage (progress updates)
└── Topic: GoalStatusArray (status updates)
```

---

## 4. Executor and Callback Structure

In ROS2, Node callback functions are scheduled by an **Executor**. This structure determines ROS2's concurrency model.

```
┌─────────────────────────────────────┐
│              Executor               │
│  ┌───────────┐  ┌───────────┐      │
│  │ Callback  │  │ Callback  │      │
│  │ Group 1   │  │ Group 2   │      │
│  │           │  │           │      │
│  │ ┌──────┐  │  │ ┌──────┐  │      │
│  │ │Timer │  │  │ │Sub   │  │      │
│  │ │CB    │  │  │ │CB    │  │      │
│  │ └──────┘  │  │ └──────┘  │      │
│  │ ┌──────┐  │  │ ┌──────┐  │      │
│  │ │Sub   │  │  │ │Srv   │  │      │
│  │ │CB    │  │  │ │CB    │  │      │
│  │ └──────┘  │  │ └──────┘  │      │
│  └───────────┘  └───────────┘      │
└─────────────────────────────────────┘
```

### Executor Types

| Executor | Threads | Characteristics |
|----------|---------|-----------------|
| **SingleThreadedExecutor** | 1 | Sequential callback execution. Simple and safe |
| **MultiThreadedExecutor** | N | Parallel callback execution. Better performance, requires synchronization |
| **StaticSingleThreadedExecutor** | 1 | No runtime Node addition. Minimized overhead |

### Callback Groups

- **MutuallyExclusiveCallbackGroup**: Only one callback in the group executes at a time
- **ReentrantCallbackGroup**: Multiple callbacks in the group can execute concurrently

---

## 5. QoS (Quality of Service)

Being DDS-based, ROS2 offers rich QoS policies that **structurally define communication behavior** for each Topic.

| QoS Policy | Options | Description |
|------------|---------|-------------|
| **Reliability** | RELIABLE / BEST_EFFORT | Whether message delivery is guaranteed |
| **Durability** | TRANSIENT_LOCAL / VOLATILE | Whether late-joining subscribers receive past messages |
| **History** | KEEP_LAST(N) / KEEP_ALL | Message buffer policy |
| **Deadline** | Duration | Guaranteed message reception interval |
| **Liveliness** | AUTOMATIC / MANUAL | Node liveness detection method |
| **Lifespan** | Duration | Message validity period |

QoS **compatibility rules** exist — if the Publisher and Subscriber QoS settings are incompatible, communication will not be established:

```
Publisher(BEST_EFFORT) ←→ Subscriber(BEST_EFFORT)  ✓ Compatible
Publisher(RELIABLE)    ←→ Subscriber(RELIABLE)     ✓ Compatible
Publisher(RELIABLE)    ←→ Subscriber(BEST_EFFORT)  ✓ Compatible
Publisher(BEST_EFFORT) ←→ Subscriber(RELIABLE)     ✗ Incompatible!
```

---

## 6. Node Lifecycle

ROS2 provides **Managed Nodes (Lifecycle Nodes)** to explicitly manage Node state transitions:

```
           ┌──────────────┐
    create │              │ destroy
    ──────→│  Unconfigured │←──────
           │              │
           └──────┬───────┘
                  │ configure
                  ▼
           ┌──────────────┐
           │   Inactive    │
           │              │
           └──────┬───────┘
                  │ activate
                  ▼
           ┌──────────────┐
    deact- │    Active     │
    ivate  │              │←─┐
    ←──────└──────────────┘  │
                             │
           ┌──────────────┐  │
           │   Finalized   │  │
           └──────────────┘  │
                             │
           (error → ErrorProcessing → Unconfigured/Finalized)
```

User-defined callbacks execute at each transition, enabling safe sequential boot processes like **sensor initialization → validation → activation**.

---

## 7. Discovery Mechanism

The most significant structural difference from ROS1 is **decentralized automatic discovery**:

```
ROS1:
Node A ──→ Master ←── Node B
           (SPOF)

ROS2 (DDS SPDP/SEDP):
Node A ←──── Multicast ────→ Node B
       ←──── Unicast  ────→ Node C

  Phase 1: SPDP (Simple Participant Discovery Protocol)
           → Announce presence via multicast

  Phase 2: SEDP (Simple Endpoint Discovery Protocol)
           → Exchange Topic/QoS info via unicast
```

This structure provides:
- **No single point of failure (SPOF)**
- Automatic discovery of other Nodes upon joining the network
- Communication with Nodes on different machines without additional configuration

---

## 8. Package and Build System

### Package Structure

```
my_robot_pkg/
├── package.xml          ← Package metadata + dependency declarations
├── CMakeLists.txt       ← C++ build rules (or setup.py for Python)
├── setup.cfg            ← Python package config
├── src/                 ← C++ source code
│   └── my_node.cpp
├── my_robot_pkg/        ← Python module
│   └── my_node.py
├── launch/              ← Launch files
│   └── robot.launch.py
├── config/              ← Configuration files (YAML)
│   └── params.yaml
├── msg/                 ← Custom message definitions
├── srv/                 ← Custom service definitions
└── action/              ← Custom action definitions
```

### Build Tool Hierarchy

```
colcon build
    │
    ▼
┌─────────┐    ┌─────────┐    ┌─────────┐
│ ament_  │    │ ament_  │    │ ament_  │
│ cmake   │    │ python  │    │ cmake   │
│ (C++)   │    │(Python) │    │ (mixed) │
└────┬────┘    └────┬────┘    └────┬────┘
     │              │              │
     ▼              ▼              ▼
  CMake          setuptools     CMake +
                               setuptools
```

`colcon` is a **meta build tool** that analyzes inter-package dependencies to determine the correct build order, then invokes each package's build system (`ament_cmake` or `ament_python`).

---

## 9. Summary: ROS2 vs ROS1

| Design Principle | ROS1 | ROS2 |
|-----------------|------|------|
| **Communication** | Centralized (Master-based) | Decentralized (DDS-based) |
| **Middleware** | Custom TCPROS/UDPROS | Standard DDS (swappable) |
| **Real-time** | Not supported | RTPS protocol + QoS |
| **Security** | Not supported | DDS Security (auth/encryption/access control) |
| **Language Support** | Independent per-language impl | Unified via rcl |
| **Lifecycle** | None | Lifecycle Nodes |
| **Build System** | catkin | colcon + ament |
| **OS Support** | Linux-centric | Linux, Windows, macOS, RTOS |

ROS2's architecture is designed to meet the reliability, real-time performance, security, and scalability demands of **industrial robotic systems**. Adopting the proven DDS communication standard and eliminating vendor lock-in through the `rmw` abstraction are highly practical architectural decisions.
