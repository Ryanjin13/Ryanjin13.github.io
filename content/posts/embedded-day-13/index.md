---
title: "Day 13 — ROS2 Architecture: DDS, QoS, and Message Types"
date: 2026-03-06
description: "ROS1 to ROS2 evolution, DDS middleware with RTPS protocol, QoS policies for robotics, and Node/Topic/Service/Action/Parameter concepts"
categories: ["Autonomous Driving"]
tags: ["ROS2", "DDS", "QoS", "RTPS", "Lifecycle Node"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 13
draft: false
---

{{< katex >}}

## What You'll Learn

Welcome to Day 13. We are entering Week 3 of the Embedded Basics for Autonomous Car series, where the focus shifts from low-level hardware and OS concepts to **robot middleware** — the software layer that ties sensors, actuators, and algorithms together into a functioning autonomous system.

In this post, you will learn:

1. **Why ROS2 exists** — the architectural problems in ROS1 that forced a complete redesign
2. **DDS middleware** — the industrial-grade publish-subscribe protocol beneath ROS2
3. **QoS policies** — how to guarantee data delivery (or intentionally not) for different sensor types
4. **Communication primitives** — Topics, Services, Actions, and Parameters in detail
5. **Lifecycle Nodes** — deterministic startup and shutdown for safety-critical systems
6. **colcon and ament** — the build system that holds it all together

By the end of this post, you will understand the ROS2 architecture well enough to design communication patterns for a real autonomous vehicle.

---

## 1. From ROS1 to ROS2: Why a Complete Rewrite?

### 1.1 The ROS1 Architecture

ROS1 (Robot Operating System, first generation) was created at Willow Garage around 2007. It provided:

- A **publish-subscribe** message passing system
- A **service** call mechanism (request-response)
- A **parameter server** for runtime configuration
- A central coordinator called **rosmaster**

The architecture looked like this:

```
                    ┌──────────────┐
                    │   rosmaster  │
                    │  (XML-RPC)   │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
        ┌─────┴─────┐ ┌───┴────┐ ┌────┴─────┐
        │  Camera    │ │ Planner│ │  Motor   │
        │  Node      │ │  Node  │ │  Node    │
        └─────┬─────┘ └───┬────┘ └────┬─────┘
              │            │            │
              └────────────┼────────────┘
                    TCP/UDP (TCPROS)
```

Every node first contacted `rosmaster` to discover which other nodes existed and what topics they published. Then nodes established **direct peer-to-peer TCP connections** (TCPROS) for actual data transfer.

### 1.2 The Fatal Flaws

This design worked remarkably well for research labs, but it had critical problems for production robotics:

**Problem 1: Single Point of Failure (rosmaster)**

If `rosmaster` crashed, no new nodes could discover each other. Existing connections continued working, but the system was in a degraded state. In an autonomous car traveling at 60 km/h, this is unacceptable.

```
rosmaster dies → New sensor node starts → Cannot find planner
                                        → Cannot find motor controller
                                        → Vehicle becomes blind
```

**Problem 2: No Real-Time Support**

ROS1 used plain TCP for message transport. TCP has no concept of deadlines, priorities, or reliability policies. A camera frame and an emergency-stop command received the same network treatment.

**Problem 3: No Quality of Service**

You couldn't tell ROS1 "deliver this lidar scan reliably" or "it's okay to drop old camera frames." Every topic used the same best-effort delivery with an arbitrary queue size.

**Problem 4: No Lifecycle Management**

Nodes started in an undefined state. There was no standard way to say "configure yourself, then activate, then deactivate gracefully." This made deterministic startup sequences very difficult.

**Problem 5: Single-OS, Single-Language Bias**

ROS1 was deeply tied to Linux and had first-class support only for C++ and Python. Cross-platform support was an afterthought.

### 1.3 The ROS2 Solution

ROS2 (first stable release: Foxy Fitzroy, 2020) addressed every one of these problems:

| Problem | ROS1 | ROS2 |
|---------|-------|-------|
| Discovery | Central rosmaster | Distributed (DDS) |
| Transport | TCPROS custom protocol | DDS/RTPS standard |
| QoS | None (queue size only) | Full QoS policies |
| Real-time | Not supported | Real-time capable |
| Lifecycle | No standard | Lifecycle nodes |
| Platforms | Linux only (practical) | Linux, Windows, macOS |

The key architectural decision: **replace the custom ROS1 middleware with DDS (Data Distribution Service)**, an existing OMG (Object Management Group) standard used in military, aerospace, and financial systems since the early 2000s.

---

## 2. DDS Middleware: The Engine Under ROS2

### 2.1 What Is DDS?

DDS (Data Distribution Service) is a **publish-subscribe middleware standard** defined by the OMG. It was designed for systems that need:

- **Decentralized discovery** (no central broker)
- **Rich QoS policies** (reliability, deadlines, lifespan, etc.)
- **Real-time performance** (bounded latency)
- **Scalability** (thousands of participants)

Think of DDS as "MQTT on steroids" — it's a pub-sub system, but one designed for safety-critical, real-time applications rather than IoT telemetry.

### 2.2 RTPS: The Wire Protocol

Under DDS lies the **RTPS (Real-Time Publish Subscribe)** protocol. This is the actual wire format — the bytes that flow over UDP between machines.

```
┌─────────────────────────────────────────────┐
│              ROS2 Application                │
├─────────────────────────────────────────────┤
│              ROS2 Client Library             │
│           (rclpy / rclcpp)                   │
├─────────────────────────────────────────────┤
│              RMW (ROS Middleware)             │
│         Abstraction Layer                    │
├─────────────────────────────────────────────┤
│              DDS Implementation              │
│  (Fast DDS / Cyclone DDS / Connext DDS)      │
├─────────────────────────────────────────────┤
│              RTPS Protocol                   │
│          (over UDP multicast)                │
├─────────────────────────────────────────────┤
│              UDP / IP / Ethernet             │
└─────────────────────────────────────────────┘
```

Key components of RTPS:

- **Participant**: A DDS entity on the network (usually maps to one ROS2 process)
- **Writer**: Sends data for a specific topic
- **Reader**: Receives data for a specific topic
- **History Cache**: Buffer of recently sent/received samples

### 2.3 Domain ID: Network Segmentation

Every DDS participant belongs to a **Domain**, identified by an integer called the **Domain ID**. Participants in different domains cannot see each other.

```bash
# Terminal 1: Robot A operates in Domain 0
export ROS_DOMAIN_ID=0
ros2 run my_package my_node

# Terminal 2: Robot B operates in Domain 1
export ROS_DOMAIN_ID=1
ros2 run my_package my_node

# These two robots are completely isolated — they cannot
# see each other's topics, services, or nodes.
```

This is how you run multiple robots on the same network without interference. The Domain ID maps to specific UDP multicast ports:

$$
\text{port} = 7400 + 250 \times \text{domain\_id} + \text{offset}
$$

where the offset depends on whether it's a discovery port or a data port.

### 2.4 Discovery: How Nodes Find Each Other (No Master!)

This is the most important difference from ROS1. Discovery in ROS2/DDS happens in **two phases**, both fully decentralized:

**Phase 1: SPDP (Simple Participant Discovery Protocol)**

When a new DDS participant (ROS2 node process) starts, it sends **multicast announcements** to a well-known multicast address on the domain's discovery port.

```
Node A starts → Sends SPDP announce to multicast 239.255.0.1:7400
                "I am Participant A, my IP is 192.168.1.10,
                 I support these endpoints..."

All existing participants hear this and respond:
Node B → "I am Participant B at 192.168.1.11..."
Node C → "I am Participant C at 192.168.1.12..."
```

After SPDP completes, every participant knows about every other participant's existence and network address.

**Phase 2: SEDP (Simple Endpoint Discovery Protocol)**

Once participants know each other, they exchange **endpoint information** — which topics each one publishes or subscribes to, with what QoS settings.

```
SEDP exchange:
Node A: "I publish /camera/image [sensor_msgs/Image], BEST_EFFORT"
Node B: "I subscribe /camera/image [sensor_msgs/Image], BEST_EFFORT"
Node C: "I publish /cmd_vel [geometry_msgs/Twist], RELIABLE"

→ DDS automatically matches A's publisher with B's subscriber
→ Direct data flow begins: A → B (no broker needed!)
```

The beauty of this system: **if any node crashes, the others keep working**. There is no single point of failure. New nodes can join at any time and will be discovered automatically.

```
Timeline:
t=0   Node A starts, announces via SPDP
t=1   Node B starts, announces via SPDP, discovers A
t=2   SEDP matches topics, data flows A↔B
t=3   Node A crashes
t=4   Node B detects A is gone (lease expired), stops waiting for data
t=5   Node A restarts, re-announces via SPDP
t=6   Node B rediscovers A, SEDP re-matches, data flows again

No master needed at any point!
```

### 2.5 DDS Implementations

ROS2 supports multiple DDS implementations through the **RMW (ROS Middleware Interface)** abstraction:

| DDS Implementation | Organization | License | Default in... |
|-------------------|-------------|---------|---------------|
| Fast DDS | eProsima | Apache 2.0 | Humble, Iron |
| Cyclone DDS | Eclipse | EPL 2.0 | Jazzy, Rolling |
| Connext DDS | RTI | Commercial | — |

You can switch DDS implementations at runtime:

```bash
# Use Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run my_package my_node

# Use Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run my_package my_node
```

This is one of ROS2's most elegant design decisions — the middleware is pluggable.

---

## 3. QoS Policies: Controlling Data Delivery

### 3.1 Why QoS Matters for Autonomous Vehicles

An autonomous car has many different data streams, each with different requirements:

```
Camera (30 fps, 2MB/frame)
  → High bandwidth, can drop frames, needs low latency

LiDAR (10 Hz, 100KB/scan)
  → Medium bandwidth, should not drop scans

Emergency Stop (rare, tiny message)
  → Must NEVER be lost, latency is critical

Map data (loaded once at startup)
  → Must be delivered even to late-joining subscribers

Odometry (100 Hz, small)
  → High frequency, latest value is most important
```

A single delivery policy cannot serve all of these. QoS lets you tailor the communication behavior for each topic.

### 3.2 Reliability: RELIABLE vs BEST_EFFORT

This is the most fundamental QoS choice.

**BEST_EFFORT**: The publisher sends data once. If the subscriber misses it (network loss, slow processing), that sample is gone forever.

**RELIABLE**: The publisher keeps sent data in its history cache. If the subscriber reports a gap, the publisher **retransmits** the missing samples.

```
BEST_EFFORT:
Publisher: [1] [2] [3] [4] [5] [6] [7] [8]
                ↓    ↓    ↓         ↓    ↓
Subscriber: [1]      [3] [4]       [7] [8]
                  (samples 2, 5, 6 lost forever)

RELIABLE:
Publisher: [1] [2] [3] [4] [5] [6] [7] [8]
                ↓    ↓    ↓    ↓    ↓    ↓
Subscriber: [1]      [3] [4]  ↓   [7] [8]
                ↑                   ↑
                └── NACK: "I missed 2!" → retransmit
                    "I missed 5,6!" → retransmit
Final:      [1] [2] [3] [4] [5] [6] [7] [8]
```

**When to use each in autonomous driving:**

| Data Type | Reliability | Why |
|-----------|------------|-----|
| Camera frames | BEST_EFFORT | Stale frames are useless; retransmitting a 2MB image wastes bandwidth and adds latency |
| LiDAR scans | RELIABLE | Each scan contributes to the map; missing one creates holes |
| Control commands (cmd_vel) | RELIABLE | Missing a stop command could mean a crash |
| Diagnostics/logging | BEST_EFFORT | Informational only; loss is tolerable |
| Emergency stop | RELIABLE | Must never be lost |

### 3.3 Durability: VOLATILE vs TRANSIENT_LOCAL

Durability controls what happens when a subscriber joins **after** some data has already been published.

**VOLATILE**: Late-joining subscribers only receive data published after they connect.

**TRANSIENT_LOCAL**: The publisher keeps recent samples in memory. Late-joining subscribers receive those cached samples immediately.

```
VOLATILE:
t=0  Publisher sends map_data = "full_map_v1"
t=5  Subscriber joins
     → Subscriber receives NOTHING (data was published before it connected)

TRANSIENT_LOCAL:
t=0  Publisher sends map_data = "full_map_v1"
t=5  Subscriber joins
     → Subscriber immediately receives "full_map_v1" from cache!
```

**Use case**: Map servers. The navigation stack needs the costmap even if it starts after the map publisher. Using TRANSIENT_LOCAL ensures the map is available immediately.

### 3.4 History: KEEP_LAST(N) vs KEEP_ALL

History controls how many samples are stored in the internal buffer.

**KEEP_LAST(N)**: Only the most recent \(N\) samples are kept. When a new sample arrives and the buffer is full, the oldest sample is discarded.

**KEEP_ALL**: Every sample is kept until the subscriber acknowledges it (for RELIABLE) or until memory limits are hit.

```
KEEP_LAST(3):
Incoming samples: [1] [2] [3] [4] [5]
Buffer state:     [1]
                  [1][2]
                  [1][2][3]
                  [2][3][4]     ← sample 1 dropped
                  [3][4][5]     ← sample 2 dropped

KEEP_ALL:
Incoming samples: [1] [2] [3] [4] [5]
Buffer state:     [1][2][3][4][5]     ← all kept
```

For most robotics applications, **KEEP_LAST(1)** or **KEEP_LAST(5)** is appropriate. Sensor data is usually only useful when fresh. KEEP_ALL is useful for logging or event systems where every sample matters.

### 3.5 Deadline

Deadline sets the **maximum expected time between messages**. If a publisher misses a deadline, both the publisher and subscriber are notified.

$$
\text{deadline\_period} = \frac{1}{\text{expected\_frequency}} \times \text{safety\_margin}
$$

For a 30 fps camera:

$$
\text{deadline} = \frac{1}{30} \times 1.5 = 50 \text{ ms}
$$

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.duration import Duration

camera_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
    deadline=Duration(seconds=0, nanoseconds=50_000_000)  # 50ms
)
```

If the camera node hangs or the cable disconnects, the subscriber receives a **deadline missed** callback — enabling the system to switch to a degraded mode.

### 3.6 Lifespan and Liveliness

**Lifespan**: How long a published sample is valid. After the lifespan expires, the sample is removed even if undelivered. Useful for time-sensitive data: a 200ms-old velocity command is dangerous.

**Liveliness**: How aggressively to check if the publisher is still alive. Options:
- AUTOMATIC: DDS checks via heartbeat
- MANUAL_BY_PARTICIPANT: The application must assert liveliness periodically

### 3.7 QoS Compatibility

Publishers and subscribers must have **compatible** QoS settings. The rules are:

```
Publisher          Subscriber         Compatible?
─────────          ──────────         ───────────
RELIABLE      ↔    RELIABLE           Yes
BEST_EFFORT   ↔    BEST_EFFORT        Yes
RELIABLE      ↔    BEST_EFFORT        Yes (subscriber "downgrades")
BEST_EFFORT   ↔    RELIABLE           NO! Subscriber demands
                                      reliability but publisher
                                      won't retransmit.
```

The general rule: **a subscriber cannot demand more than a publisher offers**.

This is a common source of debugging headaches. If your subscriber receives no data but `ros2 topic list` shows the topic exists, check QoS compatibility:

```bash
ros2 topic info /camera/image --verbose
# Shows QoS profiles of all publishers and subscribers
```

### 3.8 Putting It All Together: QoS Profiles for an Autonomous Car

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# Camera: high bandwidth, drop-tolerant, low latency
camera_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    deadline=Duration(seconds=0, nanoseconds=50_000_000),   # 50ms
    lifespan=Duration(seconds=0, nanoseconds=100_000_000),  # 100ms
)

# LiDAR: cannot afford to lose scans
lidar_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    deadline=Duration(seconds=0, nanoseconds=150_000_000),  # 150ms
)

# Control commands: must arrive, latest-only
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Map data: must arrive, even to late joiners
map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Emergency stop: absolutely must arrive
estop_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,
)
```

---

## 4. Communication Primitives: Topics, Services, Actions, Parameters

ROS2 provides four communication patterns, each serving a different purpose.

### 4.1 Topics: Continuous Data Streams

Topics implement the **publish-subscribe** pattern. A publisher sends data to a named topic; any number of subscribers can listen.

```
          ┌──────────┐     /camera/image      ┌──────────────┐
          │  Camera   │ ─────────────────────► │  Perception  │
          │  Node     │                        │  Node        │
          └──────────┘                         └──────────────┘
                                                      │
                                                      │  /detected_objects
                                                      ▼
                                               ┌──────────────┐
                                               │  Planner     │
                                               │  Node        │
                                               └──────────────┘
```

Key characteristics:
- **Asynchronous**: publisher sends without waiting for subscribers
- **Many-to-many**: multiple publishers and subscribers on the same topic
- **Continuous**: designed for streaming data
- **Decoupled**: publisher and subscriber don't know about each other

**Use for**: sensor data, odometry, velocity commands, detected objects — anything that flows continuously.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image',
            qos_profile=camera_qos
        )
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)

    def publish_frame(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.data = self.capture_frame()  # your camera capture logic
        self.publisher_.publish(msg)
        self.get_logger().info('Published camera frame')

class PerceptionSubscriber(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            qos_profile=camera_qos
        )

    def image_callback(self, msg):
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}'
        )
        # Run object detection here
```

### 4.2 Services: Request-Response

Services implement a **synchronous request-response** pattern. A client sends a request and waits for a response.

```
┌──────────┐   Request: "What is the current map?"    ┌──────────┐
│  Planner  │ ──────────────────────────────────────► │   Map    │
│  (Client) │                                          │ (Server) │
│           │ ◄────────────────────────────────────── │          │
└──────────┘   Response: OccupancyGrid(...)           └──────────┘
```

Key characteristics:
- **Synchronous**: client blocks until response arrives (or times out)
- **One-to-one**: one client request, one server response
- **On-demand**: triggered by the client, not continuous
- **Typed**: request and response have defined message types

**Use for**: one-time queries (get map, get parameters), mode changes (switch to autonomous), calibration triggers.

**Do NOT use for**: continuous data (use topics) or long-running tasks (use actions).

### 4.3 Actions: Long-Running Tasks with Feedback

Actions are for tasks that take a significant amount of time and where the client wants **progress updates**.

```
┌──────────┐  Goal: "Navigate to (10, 5)"     ┌───────────┐
│ Mission   │ ────────────────────────────────► │  Nav2     │
│ Planner   │                                   │ Navigator │
│ (Client)  │ ◄──── Feedback: "50% complete,    │ (Server)  │
│           │       current pos: (5, 3)"        │           │
│           │ ◄──── Feedback: "80% complete,    │           │
│           │       current pos: (8, 4.5)"      │           │
│           │ ◄──── Result: "Reached (10, 5)"   │           │
└──────────┘                                    └───────────┘
```

An action has three message types:
1. **Goal**: What the client wants (destination coordinates)
2. **Feedback**: Periodic progress updates (current position, percentage)
3. **Result**: Final outcome (success/failure, final position)

Key characteristics:
- **Asynchronous with feedback**: non-blocking with progress updates
- **Cancellable**: the client can cancel mid-execution
- **Preemptable**: a new goal can replace the current one

**Use for**: navigation goals, arm motion planning, calibration procedures — anything that takes more than a few seconds.

Under the hood, an action is built from **topics** (for feedback) and **services** (for goal/cancel/result).

### 4.4 Parameters: Runtime Configuration

Parameters are **named key-value pairs** attached to a node. They can be read and modified at runtime without restarting the node.

```python
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters with defaults
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('model_path', '/models/yolov8.pt')

    def detect_objects(self, image):
        # Read current parameter values
        threshold = self.get_parameter('detection_threshold').value
        max_det = self.get_parameter('max_detections').value

        # Use them in processing
        detections = self.model.predict(image, conf=threshold)
        return detections[:max_det]
```

```bash
# Change parameters at runtime (no restart needed!)
ros2 param set /perception_node detection_threshold 0.7
ros2 param set /perception_node max_detections 50

# List all parameters of a node
ros2 param list /perception_node

# Get a parameter value
ros2 param get /perception_node detection_threshold
```

### 4.5 Comparison Table

| Feature | Topic | Service | Action | Parameter |
|---------|-------|---------|--------|-----------|
| Pattern | Pub-Sub | Req-Resp | Goal-Feedback-Result | Key-Value |
| Async? | Yes | No (blocks) | Yes | N/A |
| Continuous? | Yes | No | During execution | Persistent |
| Feedback? | N/A | N/A | Yes | N/A |
| Cancellable? | N/A | N/A | Yes | N/A |
| Many-to-many? | Yes | 1:1 | 1:1 | Per-node |
| Typical use | Sensor data | Queries | Navigation | Config |

---

## 5. Lifecycle Nodes: Deterministic State Management

### 5.1 The Problem with Regular Nodes

In a regular ROS2 node, when `__init__` finishes, the node is fully active — publishing, subscribing, everything. But what if:

- The camera driver needs to be configured before it starts streaming?
- You want to activate sensors in a specific order?
- You need to cleanly shut down hardware before the node dies?

### 5.2 Lifecycle Node State Machine

A Lifecycle Node (also called a **Managed Node**) follows a strict state machine:

```
                  ┌─────────────────┐
                  │                 │
                  │  Unconfigured   │ ← Node starts here
                  │                 │
                  └────────┬────────┘
                           │ on_configure()
                           ▼
                  ┌─────────────────┐
                  │                 │
                  │    Inactive     │ ← Configured but not running
                  │                 │
                  └────────┬────────┘
                           │ on_activate()
                           ▼
                  ┌─────────────────┐
                  │                 │
                  │     Active      │ ← Fully operational
                  │                 │
                  └────────┬────────┘
                           │ on_deactivate()
                           ▼
                  ┌─────────────────┐
                  │                 │
                  │    Inactive     │ ← Can be re-activated
                  │                 │
                  └────────┬────────┘
                           │ on_cleanup()
                           ▼
                  ┌─────────────────┐
                  │                 │
                  │  Unconfigured   │ ← Can be re-configured
                  │                 │
                  └────────┬────────┘
                           │ on_shutdown()
                           ▼
                  ┌─────────────────┐
                  │                 │
                  │   Finalized     │ ← Terminal state
                  │                 │
                  └─────────────────┘
```

Each transition triggers a callback. If any callback returns FAILURE, an error transition occurs and the node enters an **Error Processing** state.

### 5.3 Lifecycle Node Implementation

```python
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class CameraDriver(LifecycleNode):
    def __init__(self):
        super().__init__('camera_driver')
        self.camera = None
        self.publisher_ = None
        self.timer = None

    def on_configure(self, state):
        """Called during Unconfigured → Inactive transition.
        Initialize hardware, create publishers, load parameters."""
        self.get_logger().info('Configuring camera...')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 30)

        device_id = self.get_parameter('device_id').value
        self.camera = CameraHardware(device_id)

        if not self.camera.open():
            self.get_logger().error('Failed to open camera!')
            return TransitionCallbackReturn.FAILURE

        self.publisher_ = self.create_publisher(
            Image, '/camera/image', camera_qos
        )

        self.get_logger().info('Camera configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called during Inactive → Active transition.
        Start publishing, enable hardware streaming."""
        self.get_logger().info('Activating camera...')

        fps = self.get_parameter('fps').value
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.camera.start_streaming()

        self.get_logger().info('Camera active and streaming')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called during Active → Inactive transition.
        Stop publishing, pause hardware."""
        self.get_logger().info('Deactivating camera...')

        self.timer.cancel()
        self.camera.stop_streaming()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Called during Inactive → Unconfigured transition.
        Release hardware resources."""
        self.get_logger().info('Cleaning up camera...')

        self.camera.close()
        self.camera = None
        self.destroy_publisher(self.publisher_)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """Called during any state → Finalized transition.
        Final cleanup before node destruction."""
        self.get_logger().info('Shutting down camera...')

        if self.camera:
            self.camera.close()

        return TransitionCallbackReturn.SUCCESS

    def publish_frame(self):
        frame = self.camera.capture()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
```

Control lifecycle transitions from the command line:

```bash
# Configure the node
ros2 lifecycle set /camera_driver configure

# Activate it (start streaming)
ros2 lifecycle set /camera_driver activate

# Deactivate (pause)
ros2 lifecycle set /camera_driver deactivate

# Cleanup (release resources)
ros2 lifecycle set /camera_driver cleanup

# Check current state
ros2 lifecycle get /camera_driver
```

### 5.4 Why Lifecycle Nodes Matter for Autonomous Vehicles

In an autonomous car, startup order matters:

```
1. Hardware drivers configure (camera, lidar, IMU)
2. Hardware drivers activate (start streaming)
3. Perception nodes configure (load models)
4. Perception nodes activate (start processing)
5. Planning nodes configure (load maps)
6. Planning nodes activate (start planning)
7. Control nodes activate (start sending commands)
```

A **Lifecycle Manager** can orchestrate this sequence, ensuring each layer is ready before the next one starts. If any node fails to configure, the entire startup is aborted cleanly.

---

## 6. colcon Build System and Package Structure

### 6.1 colcon: The ROS2 Build Tool

ROS2 uses **colcon** (collective construction) as its build tool. It replaces ROS1's `catkin_make`.

```bash
# Install colcon
pip install colcon-common-extensions

# Build entire workspace
cd ~/ros2_ws
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlinks (faster iteration for Python)
colcon build --symlink-install

# Build with parallel jobs
colcon build --parallel-workers 4
```

### 6.2 ament_python Package Structure

For Python-based ROS2 packages, the structure looks like this:

```
my_package/
├── my_package/           # Python package directory
│   ├── __init__.py
│   ├── camera_node.py
│   ├── perception_node.py
│   └── utils.py
├── resource/
│   └── my_package        # Empty marker file for ament
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml           # Package metadata and dependencies
├── setup.py              # Python package setup
└── setup.cfg             # Entry points configuration
```

**package.xml** — declares dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My autonomous driving package</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**setup.py** — defines entry points (executables):

```python
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'camera_node = my_package.camera_node:main',
            'perception_node = my_package.perception_node:main',
        ],
    },
)
```

### 6.3 Workspace Layout

```
ros2_ws/                    # Workspace root
├── src/                    # Source packages go here
│   ├── my_package/
│   ├── my_msgs/
│   └── my_launch/
├── build/                  # Build artifacts (auto-generated)
├── install/                # Installed packages (auto-generated)
└── log/                    # Build logs (auto-generated)
```

```bash
# Create a new package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy std_msgs

# Build
cd ~/ros2_ws
colcon build

# Source the workspace overlay
source install/setup.bash

# Run a node
ros2 run my_package camera_node
```

---

## 7. Hands-On Lab

### Lab 1: Custom Message Type Definition

Let's create a custom message for detected objects in our autonomous vehicle.

**Step 1: Create a message package**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_interfaces
mkdir -p my_interfaces/msg my_interfaces/srv my_interfaces/action
```

Note: message packages must use `ament_cmake` even if your nodes are Python.

**Step 2: Define the message**

Create `my_interfaces/msg/DetectedObject.msg`:

```
# Header with timestamp and frame
std_msgs/Header header

# Object class and confidence
string class_name
float32 confidence

# Bounding box in image coordinates (pixels)
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height

# Estimated distance from vehicle (meters)
float64 distance

# Estimated velocity relative to ego vehicle (m/s)
float64 relative_velocity
```

Create `my_interfaces/msg/DetectedObjectArray.msg`:

```
std_msgs/Header header
my_interfaces/DetectedObject[] objects
int32 total_count
```

**Step 3: Update CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

**Step 4: Update package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_interfaces</name>
  <version>0.0.1</version>
  <description>Custom interfaces for autonomous driving</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

**Step 5: Build and verify**

```bash
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash

# Verify the message is available
ros2 interface show my_interfaces/msg/DetectedObject
```

### Lab 2: Service Server and Client

Let's create a service that returns the number of detected objects in a region of interest.

**Step 1: Define the service**

Create `my_interfaces/srv/CountObjects.srv`:

```
# Request: region of interest
int32 roi_x
int32 roi_y
int32 roi_width
int32 roi_height
string class_filter    # Empty string = all classes
---
# Response
int32 count
string[] detected_classes
bool success
string message
```

Update CMakeLists.txt to include the service:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "srv/CountObjects.srv"
  DEPENDENCIES std_msgs
)
```

**Step 2: Service Server**

```python
#!/usr/bin/env python3
"""count_objects_server.py — Service server that counts detected objects in a ROI."""

import rclpy
from rclpy.node import Node
from my_interfaces.srv import CountObjects
from my_interfaces.msg import DetectedObjectArray


class CountObjectsServer(Node):
    def __init__(self):
        super().__init__('count_objects_server')

        # Store the latest detections
        self.latest_detections = None

        # Subscribe to detected objects
        self.subscription = self.create_subscription(
            DetectedObjectArray,
            '/detected_objects',
            self.detection_callback,
            10
        )

        # Create the service
        self.srv = self.create_service(
            CountObjects,
            '/count_objects',
            self.count_callback
        )

        self.get_logger().info('CountObjects service server ready')

    def detection_callback(self, msg):
        """Store the latest detections."""
        self.latest_detections = msg

    def count_callback(self, request, response):
        """Handle a count request."""
        if self.latest_detections is None:
            response.count = 0
            response.detected_classes = []
            response.success = False
            response.message = 'No detections received yet'
            return response

        # Filter objects within the ROI
        matching_objects = []
        for obj in self.latest_detections.objects:
            # Check if object center is inside ROI
            obj_cx = obj.bbox_x + obj.bbox_width // 2
            obj_cy = obj.bbox_y + obj.bbox_height // 2

            in_roi = (request.roi_x <= obj_cx <= request.roi_x + request.roi_width and
                      request.roi_y <= obj_cy <= request.roi_y + request.roi_height)

            # Check class filter
            class_match = (request.class_filter == '' or
                          obj.class_name == request.class_filter)

            if in_roi and class_match:
                matching_objects.append(obj)

        response.count = len(matching_objects)
        response.detected_classes = list(set(
            obj.class_name for obj in matching_objects
        ))
        response.success = True
        response.message = f'Found {len(matching_objects)} objects in ROI'

        self.get_logger().info(
            f'Count request: ROI=({request.roi_x},{request.roi_y},'
            f'{request.roi_width},{request.roi_height}), '
            f'filter={request.class_filter}, '
            f'result={response.count}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CountObjectsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 3: Service Client**

```python
#!/usr/bin/env python3
"""count_objects_client.py — Service client that queries object counts."""

import rclpy
from rclpy.node import Node
from my_interfaces.srv import CountObjects


class CountObjectsClient(Node):
    def __init__(self):
        super().__init__('count_objects_client')
        self.client = self.create_client(CountObjects, '/count_objects')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /count_objects service...')

        self.get_logger().info('Connected to /count_objects service')

    def send_request(self, x, y, w, h, class_filter=''):
        """Send a count request and return the response."""
        request = CountObjects.Request()
        request.roi_x = x
        request.roi_y = y
        request.roi_width = w
        request.roi_height = h
        request.class_filter = class_filter

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = CountObjectsClient()

    # Query: how many cars in the center of the image?
    response = client.send_request(
        x=200, y=150, w=240, h=180, class_filter='car'
    )

    if response.success:
        print(f'Found {response.count} cars in ROI')
        print(f'Classes detected: {response.detected_classes}')
    else:
        print(f'Query failed: {response.message}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Lab 3: Action Server and Client (Navigation with Progress)

**Step 1: Define the action**

Create `my_interfaces/action/Navigate.action`:

```
# Goal: target position
float64 target_x
float64 target_y
float64 target_theta
float64 max_speed
---
# Result: final outcome
float64 final_x
float64 final_y
float64 final_theta
float64 total_distance
float64 total_time
bool success
string message
---
# Feedback: progress updates
float64 current_x
float64 current_y
float64 current_theta
float64 distance_remaining
float64 estimated_time_remaining
float32 percent_complete
```

**Step 2: Action Server**

```python
#!/usr/bin/env python3
"""navigate_server.py — Action server for navigation with progress feedback."""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_interfaces.action import Navigate


class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')

        # Current position (simulated)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self._action_server = ActionServer(
            self,
            Navigate,
            '/navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Navigate action server ready')

    def goal_callback(self, goal_request):
        """Decide whether to accept or reject the goal."""
        self.get_logger().info(
            f'Received goal: ({goal_request.target_x:.1f}, '
            f'{goal_request.target_y:.1f})'
        )

        # Accept all valid goals
        if goal_request.max_speed <= 0:
            self.get_logger().warn('Rejected: max_speed must be positive')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Decide whether to accept cancellation."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the navigation goal with feedback."""
        self.get_logger().info('Executing navigation goal...')

        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        max_speed = goal_handle.request.max_speed

        # Calculate total distance
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        total_distance = math.sqrt(dx**2 + dy**2)

        start_time = time.time()
        step_size = max_speed * 0.1  # distance per 100ms step

        feedback = Navigate.Feedback()

        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Navigate.Result()
                result.final_x = self.current_x
                result.final_y = self.current_y
                result.success = False
                result.message = 'Navigation cancelled'
                self.get_logger().info('Navigation cancelled')
                return result

            # Calculate remaining distance
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance_remaining = math.sqrt(dx**2 + dy**2)

            # Check if we've arrived
            if distance_remaining < 0.05:  # 5cm tolerance
                break

            # Move toward target
            angle = math.atan2(dy, dx)
            move = min(step_size, distance_remaining)
            self.current_x += move * math.cos(angle)
            self.current_y += move * math.sin(angle)

            # Publish feedback
            elapsed = time.time() - start_time
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_theta = angle
            feedback.distance_remaining = distance_remaining
            feedback.estimated_time_remaining = (
                distance_remaining / max_speed if max_speed > 0 else 0.0
            )
            feedback.percent_complete = float(min(
                100.0, (1.0 - distance_remaining / total_distance) * 100
            ))

            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'Progress: {feedback.percent_complete:.1f}% '
                f'({self.current_x:.2f}, {self.current_y:.2f})'
            )

            time.sleep(0.1)  # 10 Hz update rate

        # Update final orientation
        self.current_theta = target_theta

        # Mark as succeeded
        goal_handle.succeed()

        # Build result
        result = Navigate.Result()
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.final_theta = self.current_theta
        result.total_distance = total_distance
        result.total_time = time.time() - start_time
        result.success = True
        result.message = 'Navigation completed successfully'

        self.get_logger().info(
            f'Navigation complete: distance={total_distance:.2f}m, '
            f'time={result.total_time:.1f}s'
        )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 3: Action Client**

```python
#!/usr/bin/env python3
"""navigate_client.py — Action client that sends navigation goals."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_interfaces.action import Navigate


class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._action_client = ActionClient(
            self, Navigate, '/navigate_to_pose'
        )

    def send_goal(self, x, y, theta=0.0, max_speed=1.0):
        """Send a navigation goal and wait for result."""
        goal_msg = Navigate.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_theta = theta
        goal_msg.max_speed = max_speed

        self.get_logger().info(f'Sending goal: ({x}, {y})')

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the server accepts/rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        # Wait for the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Called when the action completes."""
        result = future.result().result

        if result.success:
            self.get_logger().info(
                f'Navigation succeeded! '
                f'Final position: ({result.final_x:.2f}, {result.final_y:.2f}), '
                f'Distance: {result.total_distance:.2f}m, '
                f'Time: {result.total_time:.1f}s'
            )
        else:
            self.get_logger().warn(f'Navigation failed: {result.message}')

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Called periodically with progress updates."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.percent_complete:.0f}%] '
            f'Position: ({fb.current_x:.2f}, {fb.current_y:.2f}), '
            f'Remaining: {fb.distance_remaining:.2f}m, '
            f'ETA: {fb.estimated_time_remaining:.1f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    client = NavigateClient()

    # Send a navigation goal
    client.send_goal(x=10.0, y=5.0, theta=1.57, max_speed=2.0)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

### Lab 4: QoS Compatibility Experiment

This experiment demonstrates what happens when publisher and subscriber QoS profiles don't match.

```python
#!/usr/bin/env python3
"""qos_experiment.py — Demonstrates QoS compatibility and incompatibility."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class QoSExperimentNode(Node):
    """
    Run this node to see QoS matching behavior.

    Experiment 1: RELIABLE pub + BEST_EFFORT sub → works (sub downgrades)
    Experiment 2: BEST_EFFORT pub + RELIABLE sub → FAILS (incompatible)
    Experiment 3: TRANSIENT_LOCAL pub + VOLATILE sub → works
    Experiment 4: VOLATILE pub + TRANSIENT_LOCAL sub → FAILS
    """

    def __init__(self):
        super().__init__('qos_experiment')

        # Define QoS profiles
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        transient_local_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ─── Experiment 1: RELIABLE pub → BEST_EFFORT sub ───
        # This WORKS. The subscriber simply doesn't request retransmits.
        self.pub1 = self.create_publisher(
            String, '/exp1_reliable_pub', reliable_qos
        )
        self.sub1 = self.create_subscription(
            String, '/exp1_reliable_pub',
            lambda msg: self.get_logger().info(
                f'[Exp1 OK] Received: {msg.data}'
            ),
            best_effort_qos
        )

        # ─── Experiment 2: BEST_EFFORT pub → RELIABLE sub ───
        # This FAILS. Subscriber demands reliability, publisher won't provide it.
        self.pub2 = self.create_publisher(
            String, '/exp2_besteffort_pub', best_effort_qos
        )
        self.sub2 = self.create_subscription(
            String, '/exp2_besteffort_pub',
            lambda msg: self.get_logger().info(
                f'[Exp2 !!] Received: {msg.data} (should NOT appear!)'
            ),
            reliable_qos
        )

        # ─── Experiment 3: TRANSIENT_LOCAL pub → VOLATILE sub ───
        # This WORKS. Subscriber just won't get late-joining data.
        self.pub3 = self.create_publisher(
            String, '/exp3_transient_pub', transient_local_qos
        )
        self.sub3 = self.create_subscription(
            String, '/exp3_transient_pub',
            lambda msg: self.get_logger().info(
                f'[Exp3 OK] Received: {msg.data}'
            ),
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
        )

        # Timer to publish test messages
        self.counter = 0
        self.timer = self.create_timer(1.0, self.publish_all)

        self.get_logger().info('QoS Experiment started. Watch the output:')
        self.get_logger().info('  [Exp1 OK] = RELIABLE→BEST_EFFORT (compatible)')
        self.get_logger().info('  [Exp2 !!] = BEST_EFFORT→RELIABLE (INCOMPATIBLE)')
        self.get_logger().info('  [Exp3 OK] = TRANSIENT_LOCAL→VOLATILE (compatible)')

    def publish_all(self):
        self.counter += 1

        msg1 = String(data=f'Exp1 msg #{self.counter}')
        msg2 = String(data=f'Exp2 msg #{self.counter}')
        msg3 = String(data=f'Exp3 msg #{self.counter}')

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)

        self.get_logger().info(f'Published message set #{self.counter}')


def main(args=None):
    rclpy.init(args=args)
    node = QoSExperimentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Expected output:

```
[INFO] QoS Experiment started. Watch the output:
[INFO] Published message set #1
[INFO] [Exp1 OK] Received: Exp1 msg #1
[INFO] [Exp3 OK] Received: Exp3 msg #1
[INFO] Published message set #2
[INFO] [Exp1 OK] Received: Exp1 msg #2
[INFO] [Exp3 OK] Received: Exp3 msg #2
...
```

Notice that **Experiment 2 never receives anything**. The BEST_EFFORT publisher and RELIABLE subscriber are incompatible — DDS silently refuses to match them.

To debug this in a real system:

```bash
# Check for QoS incompatibility warnings
ros2 topic info /exp2_besteffort_pub --verbose

# You'll see the publisher and subscriber listed separately
# with mismatched QoS, and the subscriber won't be "matched"
```

---

## 8. Review

### Key Takeaways

1. **ROS2 removed rosmaster** — DDS provides distributed discovery through SPDP/SEDP protocols using UDP multicast. No single point of failure.

2. **DDS/RTPS is industrial-grade middleware** — used in military and aerospace since the 2000s. ROS2 adopted it rather than reinventing the wheel.

3. **QoS policies are not optional in production robotics** — camera topics use BEST_EFFORT (low latency, drop-tolerant), control topics use RELIABLE (must arrive). Choosing wrong QoS causes either data loss or unacceptable latency.

4. **Four communication patterns** serve different needs:
   - Topics: continuous sensor data
   - Services: one-time queries
   - Actions: long-running tasks with feedback
   - Parameters: runtime configuration

5. **Lifecycle Nodes** enable deterministic startup/shutdown — critical for safety systems that need ordered initialization.

6. **QoS compatibility follows the "offer/request" model** — a subscriber cannot demand more than a publisher offers.

### Connection to Other Days

- **Day 5 (OS Threading)**: The executor model in Day 14 builds directly on threading concepts from Day 5
- **Day 6 (PWM/Motor Control)**: Motor commands flow through ROS2 topics with RELIABLE QoS
- **Day 9 (Sensors)**: All sensor data is published as ROS2 topics with appropriate QoS
- **Day 14 (Tomorrow)**: We will explore how ROS2 executes callbacks, manages concurrency, and uses TF2 for coordinate transforms

### Quick Self-Check

1. Why can't a BEST_EFFORT publisher satisfy a RELIABLE subscriber?
2. What happens if rosmaster crashes in ROS1? What happens if any node crashes in ROS2?
3. When would you use an Action instead of a Service?
4. What QoS profile would you choose for an emergency stop topic? Why?
5. What is the difference between the SPDP and SEDP discovery phases?

---

*Next up: [Day 14 — ROS2 Executor Model and Concurrency Patterns](/posts/embedded-day-14/) — where we connect Day 5 OS threading to ROS2 callback execution, explore TF2 coordinate transforms, and debug performance bottlenecks with rqt_graph.*
