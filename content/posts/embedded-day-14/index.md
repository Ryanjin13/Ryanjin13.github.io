---
title: "Day 14 — ROS2 Executor Model and Concurrency Patterns"
date: 2026-03-05
description: "TF2 coordinate transforms, executor threading models, callback groups, intra-process communication, and debugging tools for ROS2 robotics"
categories: ["Autonomous Driving"]
tags: ["ROS2", "TF2", "Executor", "Callback Groups", "Concurrency"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 14
draft: false
---

{{< katex >}}

## What You'll Learn

In [Day 13](/posts/embedded-day-13/), we covered the ROS2 architecture from the ground up — DDS middleware, QoS policies, and the communication primitives (Topics, Services, Actions). But we never asked: **how does a ROS2 node actually process incoming messages?**

When three topics arrive simultaneously, which callback runs first? Can two callbacks run in parallel? What happens if a slow image processing callback blocks a time-critical control loop?

These questions take us deep into the **Executor** — the heart of ROS2's callback scheduling. And they connect directly back to [Day 5](/posts/embedded-day-05/) where we studied OS threads, mutexes, and scheduling policies.

In this post, you will learn:

1. **TF2 coordinate transforms** — how frames relate to each other in a robot
2. **Executor models** — SingleThreaded, MultiThreaded, and StaticSingleThreaded
3. **Callback Groups** — MutuallyExclusive and Reentrant patterns
4. **Intra-process communication** — zero-copy data transfer
5. **Python GIL limitations** — when to switch from rclpy to rclcpp
6. **Debugging tools** — rqt_graph, PlotJuggler, Foxglove, and CLI inspection

---

## 1. TF2: The Coordinate Transform Framework

### 1.1 Why Coordinate Transforms Matter

An autonomous vehicle has multiple sensors, each measuring the world from its own perspective:

```
                    ┌─────────────────────┐
                    │     GPS antenna     │  ← measures lat/lon
                    └──────────┬──────────┘
                               │ 0.5m above
                    ┌──────────┴──────────┐
                    │    Forward Camera    │  ← measures pixels
                    └──────────┬──────────┘
       0.3m left               │ 0.2m front
    ┌───────────┐   ┌──────────┴──────────┐   ┌───────────┐
    │   LiDAR   │───│     base_link       │───│   IMU     │
    │  (left)   │   │  (vehicle center)   │   │  (right)  │
    └───────────┘   └──────────┬──────────┘   └───────────┘
                               │
                    ┌──────────┴──────────┐
                    │    Wheel encoders    │  ← measure rotation
                    └─────────────────────┘
```

When the LiDAR detects an obstacle 3 meters ahead in the **lidar_link** frame, and the camera sees a car 5 meters ahead in the **camera_link** frame — are they the same object? To answer this, you need to transform both measurements into a **common frame**.

TF2 (Transform Framework 2) is ROS2's system for tracking coordinate frame relationships over time.

### 1.2 Standard Frames in Mobile Robotics

The ROS community has standardized a set of coordinate frames described in [REP 105](https://www.ros.org/reps/rep-0105.html):

```
    map
     │
     │  (global localization: SLAM, GPS)
     │  May have discrete jumps
     ▼
    odom
     │
     │  (continuous odometry: wheel encoders, IMU)
     │  Smooth but drifts over time
     ▼
  base_link
     │
     │  (static transforms: mechanical mounting)
     │  Fixed relative positions
     ▼
  sensor frames (lidar_link, camera_link, imu_link, ...)
```

**base_link**: The coordinate frame rigidly attached to the vehicle body. Usually at the center of the rear axle, or the geometric center of the robot. All sensor frames are defined relative to base_link.

**odom**: The "local" world frame. It starts where the robot was when it booted up. Odometry (wheel encoders + IMU) provides the transform from odom to base_link. This transform is **continuous and smooth** (no jumps) but **drifts over time** because odometry accumulates error.

**map**: The "global" world frame aligned with a pre-built map or GPS coordinates. SLAM or GPS localization provides the transform from map to odom. This transform **can jump** when the localizer corrects accumulated drift.

Why separate map and odom? Consider this scenario:

```
True robot path:     A ──────────────── B (actual position)

Odometry says:       A ──────────────── B' (slightly wrong due to drift)

SLAM corrects:       map→odom adjusts so that B' maps to B

Result: odom→base_link remains smooth (good for control)
        map→base_link is accurate (good for planning)
```

If there were only one frame, the SLAM correction would cause a **sudden jump** in the control loop's position estimate — potentially causing the steering to jerk.

### 1.3 Transform Mathematics

A transform between two frames consists of a **rotation** and a **translation**. In 3D, this is represented as a 4x4 homogeneous transformation matrix:

$$
T^A_B = \begin{bmatrix} R_{3\times3} & t_{3\times1} \\ 0_{1\times3} & 1 \end{bmatrix}
$$

where:
- \(R\) is a 3x3 rotation matrix (or equivalently, a quaternion)
- \(t\) is a 3x1 translation vector
- The subscript/superscript notation: \(T^A_B\) transforms a point from frame \(B\) to frame \(A\)

To transform a point \(\mathbf{p}^B\) expressed in frame \(B\) into frame \(A\):

$$
\mathbf{p}^A = T^A_B \cdot \mathbf{p}^B
$$

Transforms **chain** via matrix multiplication:

$$
T^{\text{map}}_{\text{lidar}} = T^{\text{map}}_{\text{odom}} \cdot T^{\text{odom}}_{\text{base\_link}} \cdot T^{\text{base\_link}}_{\text{lidar}}
$$

This is exactly what TF2 does internally — it maintains a **tree** of transforms and computes chains automatically.

### 1.4 TF2 in Practice: Broadcasters and Listeners

**Static Transform Broadcaster**: For sensor mounts that never change.

```python
#!/usr/bin/env python3
"""static_tf_broadcaster.py — Publish static transforms for sensor mounts."""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class SensorFramePublisher(Node):
    def __init__(self):
        super().__init__('sensor_frame_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish all static transforms at startup
        self.publish_static_transforms()

    def publish_static_transforms(self):
        transforms = []

        # Camera: 20cm forward, 50cm up from base_link, facing forward
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.20   # 20cm forward
        camera_tf.transform.translation.y = 0.0     # centered
        camera_tf.transform.translation.z = 0.50    # 50cm up
        # Quaternion for no rotation (camera aligned with base_link)
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 1.0
        transforms.append(camera_tf)

        # LiDAR: 30cm left, 40cm up from base_link
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.0
        lidar_tf.transform.translation.y = 0.30     # 30cm left
        lidar_tf.transform.translation.z = 0.40     # 40cm up
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0
        transforms.append(lidar_tf)

        # IMU: at the center of base_link (common mounting)
        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.10       # 10cm up
        imu_tf.transform.rotation.x = 0.0
        imu_tf.transform.rotation.y = 0.0
        imu_tf.transform.rotation.z = 0.0
        imu_tf.transform.rotation.w = 1.0
        transforms.append(imu_tf)

        # Depth camera: 15cm forward, 45cm up, tilted 10 degrees down
        depth_tf = TransformStamped()
        depth_tf.header.stamp = self.get_clock().now().to_msg()
        depth_tf.header.frame_id = 'base_link'
        depth_tf.child_frame_id = 'depth_camera_link'
        depth_tf.transform.translation.x = 0.15
        depth_tf.transform.translation.y = 0.0
        depth_tf.transform.translation.z = 0.45
        # Quaternion for 10-degree downward pitch
        # pitch = -10 degrees = -0.1745 radians
        pitch = -10.0 * math.pi / 180.0
        depth_tf.transform.rotation.x = 0.0
        depth_tf.transform.rotation.y = math.sin(pitch / 2.0)
        depth_tf.transform.rotation.z = 0.0
        depth_tf.transform.rotation.w = math.cos(pitch / 2.0)
        transforms.append(depth_tf)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} static transforms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Dynamic Transform Broadcaster**: For transforms that change over time (odometry).

```python
#!/usr/bin/env python3
"""odom_tf_broadcaster.py — Publish odom → base_link transform from wheel odometry."""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        """Broadcast odom → base_link transform from odometry data."""
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Translation from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation from odometry
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Transform Listener**: Looking up transforms between any two frames.

```python
#!/usr/bin/env python3
"""tf_listener_example.py — Look up transforms between frames."""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # Required for automatic transform of geometry messages


class ObstacleTransformer(Node):
    def __init__(self):
        super().__init__('obstacle_transformer')

        # TF2 buffer stores all known transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.transform_example)

    def transform_example(self):
        """Transform a point from lidar_link to map frame."""
        try:
            # Look up transform from lidar_link to map
            # at the latest available time
            transform = self.tf_buffer.lookup_transform(
                'map',           # target frame
                'lidar_link',    # source frame
                rclpy.time.Time()  # latest available
            )

            self.get_logger().info(
                f'lidar_link → map transform:\n'
                f'  Translation: ({transform.transform.translation.x:.3f}, '
                f'{transform.transform.translation.y:.3f}, '
                f'{transform.transform.translation.z:.3f})\n'
                f'  Rotation: ({transform.transform.rotation.x:.3f}, '
                f'{transform.transform.rotation.y:.3f}, '
                f'{transform.transform.rotation.z:.3f}, '
                f'{transform.transform.rotation.w:.3f})'
            )

            # Transform a specific point (obstacle at 3m ahead in lidar frame)
            obstacle_in_lidar = PointStamped()
            obstacle_in_lidar.header.frame_id = 'lidar_link'
            obstacle_in_lidar.header.stamp = self.get_clock().now().to_msg()
            obstacle_in_lidar.point.x = 3.0   # 3m ahead
            obstacle_in_lidar.point.y = 0.5   # 0.5m left
            obstacle_in_lidar.point.z = 0.0

            # Automatically transform to map frame
            obstacle_in_map = self.tf_buffer.transform(
                obstacle_in_lidar, 'map'
            )

            self.get_logger().info(
                f'Obstacle in lidar_link: ({obstacle_in_lidar.point.x:.1f}, '
                f'{obstacle_in_lidar.point.y:.1f})\n'
                f'Obstacle in map:        ({obstacle_in_map.point.x:.1f}, '
                f'{obstacle_in_map.point.y:.1f})'
            )

        except Exception as e:
            self.get_logger().warn(
                f'Could not get transform: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 1.5 The TF2 Tree

TF2 maintains a **tree** (not a graph — each frame has exactly one parent). You can visualize it:

```bash
# View the TF tree as a PDF
ros2 run tf2_tools view_frames

# Output: frames.pdf showing:
#
#   map
#    └── odom
#         └── base_link
#              ├── camera_link
#              ├── lidar_link
#              ├── imu_link
#              └── depth_camera_link
```

The tree structure guarantees that there is **exactly one path** between any two frames, making transform lookups unambiguous.

---

## 2. The Executor: ROS2's Callback Scheduler

### 2.1 What Is an Executor?

In ROS2, your node has callbacks — functions triggered by incoming messages, timer expirations, or service requests. The **Executor** is the component that:

1. Checks for ready callbacks (new messages, expired timers)
2. Decides which callback to run next
3. Actually invokes the callback

Think of it as the **scheduler** for ROS2 callbacks, analogous to the OS scheduler for threads (Day 5).

```
Incoming data:
  /camera/image   ─┐
  /lidar/scan     ─┤     ┌────────────┐     ┌──────────────────┐
  /imu/data       ─┼────►│  Executor  │────►│ Run callback()   │
  Timer (10Hz)    ─┤     │            │     │ one at a time     │
  Service call    ─┘     └────────────┘     └──────────────────┘
```

### 2.2 SingleThreadedExecutor

The default executor. It runs **one callback at a time**, in a single thread.

```python
import rclpy
from rclpy.executors import SingleThreadedExecutor

rclpy.init()
node = MyNode()

executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()  # Blocks, processing callbacks one by one
```

Execution timeline:

```
Time ──────────────────────────────────────────────────►

Thread: [camera_cb 200ms][lidar_cb 50ms][timer_cb 10ms][camera_cb 200ms]

Only one callback runs at any time.
If camera_cb takes 200ms, ALL other callbacks wait.
```

**Pros:**
- Simple to reason about — no race conditions, no locks needed
- Safe — no shared state issues

**Cons:**
- **Blocking**: A slow callback delays everything else
- If image processing takes 200ms and your control loop needs to run at 100Hz (10ms), the control loop will be starved

This is the **exact same problem** as cooperative scheduling from Day 5 — one "task" hogging the CPU blocks all others.

### 2.3 The Blocking Problem (Demonstration)

Here is a concrete example of the problem. Imagine a node that does both image processing and motor control:

```python
#!/usr/bin/env python3
"""blocking_problem.py — Demonstrates SingleThreadedExecutor blocking issue."""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class BlockingNode(Node):
    def __init__(self):
        super().__init__('blocking_node')

        # Camera subscriber — heavy processing
        self.camera_sub = self.create_subscription(
            Image, '/camera/image', self.camera_callback, 10
        )

        # Control loop timer — must run at 50Hz (every 20ms)
        self.control_timer = self.create_timer(0.02, self.control_callback)

        self.control_call_count = 0
        self.last_control_time = time.time()

    def camera_callback(self, msg):
        """Simulates heavy image processing (200ms)."""
        self.get_logger().info('Camera callback START')
        start = time.time()

        # Simulate neural network inference
        time.sleep(0.2)  # 200ms of processing

        elapsed = (time.time() - start) * 1000
        self.get_logger().info(f'Camera callback END ({elapsed:.0f}ms)')

    def control_callback(self):
        """Motor control loop — should run every 20ms."""
        now = time.time()
        actual_period = (now - self.last_control_time) * 1000
        self.last_control_time = now
        self.control_call_count += 1

        if actual_period > 25:  # More than 25% late
            self.get_logger().warn(
                f'Control loop LATE! Period: {actual_period:.1f}ms '
                f'(expected 20ms)'
            )
        else:
            self.get_logger().info(
                f'Control loop OK. Period: {actual_period:.1f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BlockingNode()

    # Using SingleThreadedExecutor (default)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Output with SingleThreadedExecutor:

```
[INFO] Control loop OK. Period: 20.1ms
[INFO] Control loop OK. Period: 20.0ms
[INFO] Camera callback START
[WARN] Control loop LATE! Period: 220.3ms    ← BLOCKED for 200ms!
[INFO] Camera callback END (200ms)
[INFO] Control loop OK. Period: 20.1ms
[INFO] Control loop OK. Period: 19.9ms
[INFO] Camera callback START
[WARN] Control loop LATE! Period: 218.7ms    ← BLOCKED again!
[INFO] Camera callback END (200ms)
```

The control loop, which needs to run every 20ms, was delayed by **200ms** every time the camera callback ran. In a real vehicle, this would cause jerky steering and potentially dangerous behavior.

### 2.4 MultiThreadedExecutor

The solution: use multiple threads so callbacks can run in parallel.

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

rclpy.init()
node = MyNode()

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

Execution timeline with MultiThreadedExecutor:

```
Time ──────────────────────────────────────────────────►

Thread 1: [camera_cb 200ms          ][camera_cb 200ms          ]
Thread 2: [ctrl][ctrl][ctrl][ctrl][ctrl][ctrl][ctrl][ctrl][ctrl]
Thread 3:          [lidar_cb 50ms]          [lidar_cb 50ms]
Thread 4:                                      [service_cb]

Camera processing no longer blocks the control loop!
```

But there's a catch: if multiple threads can run callbacks simultaneously, you might have **race conditions** on shared data. This is where Callback Groups come in.

### 2.5 StaticSingleThreadedExecutor

An optimization for scenarios where the set of nodes and subscriptions doesn't change at runtime. It pre-computes the callback schedule, avoiding the overhead of checking for new entities every spin cycle.

```python
from rclpy.executors import StaticSingleThreadedExecutor

executor = StaticSingleThreadedExecutor()
executor.add_node(camera_node)
executor.add_node(control_node)
executor.spin()
# Slightly lower latency than SingleThreadedExecutor
# But cannot dynamically add/remove nodes
```

### 2.6 Executor Comparison

| Executor | Threads | Dynamic Nodes | Use Case |
|----------|---------|---------------|----------|
| SingleThreaded | 1 | Yes | Simple nodes, no blocking concern |
| MultiThreaded | N | Yes | Mixed fast/slow callbacks |
| StaticSingleThreaded | 1 | No | Optimized fixed-topology systems |

---

## 3. Callback Groups: Fine-Grained Concurrency Control

### 3.1 The Need for Callback Groups

With a MultiThreadedExecutor, all callbacks can potentially run in parallel. But what if two callbacks share data?

```python
class UnsafeNode(Node):
    def __init__(self):
        super().__init__('unsafe_node')
        self.shared_map = {}  # Shared state!

        self.sub_lidar = self.create_subscription(
            PointCloud2, '/lidar/scan', self.lidar_callback, 10
        )
        self.sub_camera = self.create_subscription(
            Image, '/camera/image', self.camera_callback, 10
        )

    def lidar_callback(self, msg):
        # Writes to shared_map
        self.shared_map['obstacles'] = self.process_lidar(msg)

    def camera_callback(self, msg):
        # Also writes to shared_map — RACE CONDITION!
        self.shared_map['detections'] = self.process_camera(msg)
```

If both callbacks run simultaneously (MultiThreadedExecutor), they might corrupt `shared_map`.

### 3.2 MutuallyExclusiveCallbackGroup

Callbacks in a MutuallyExclusive group **never run at the same time**. This is like a **Mutex** from Day 5 — at most one callback from the group holds the "lock."

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class SafeNode(Node):
    def __init__(self):
        super().__init__('safe_node')
        self.shared_map = {}

        # Both callbacks in the same MutuallyExclusive group
        self.map_group = MutuallyExclusiveCallbackGroup()

        self.sub_lidar = self.create_subscription(
            PointCloud2, '/lidar/scan', self.lidar_callback, 10,
            callback_group=self.map_group
        )
        self.sub_camera = self.create_subscription(
            Image, '/camera/image', self.camera_callback, 10,
            callback_group=self.map_group
        )

    def lidar_callback(self, msg):
        self.shared_map['obstacles'] = self.process_lidar(msg)
        # Safe: camera_callback cannot run while this is running

    def camera_callback(self, msg):
        self.shared_map['detections'] = self.process_camera(msg)
        # Safe: lidar_callback cannot run while this is running
```

```
Time ──────────────────────────────────────────────────►

map_group: [lidar_cb][camera_cb][lidar_cb][camera_cb]
                                           ↑
                Never overlapping — serial within the group
```

### 3.3 ReentrantCallbackGroup

Callbacks in a Reentrant group **can run simultaneously**, including multiple instances of the same callback.

```python
from rclpy.callback_groups import ReentrantCallbackGroup

class ParallelNode(Node):
    def __init__(self):
        super().__init__('parallel_node')

        # Callbacks that are safe to run in parallel
        self.parallel_group = ReentrantCallbackGroup()

        # These two subscriptions have no shared state
        self.sub_log1 = self.create_subscription(
            String, '/log_stream_1', self.log_callback_1, 10,
            callback_group=self.parallel_group
        )
        self.sub_log2 = self.create_subscription(
            String, '/log_stream_2', self.log_callback_2, 10,
            callback_group=self.parallel_group
        )
```

```
Time ──────────────────────────────────────────────────►

Thread 1: [log_cb_1][log_cb_1]    [log_cb_1]
Thread 2:    [log_cb_2][log_cb_2][log_cb_2]
                 ↑
          Can overlap! Both run simultaneously.
```

### 3.4 The Complete Pattern: Mixing Groups

The real power comes from combining both group types. Here is the pattern for an autonomous vehicle node:

```python
#!/usr/bin/env python3
"""multi_executor_fixed.py — Proper callback group design for autonomous driving."""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        # ─── Callback Group Design ───
        #
        # Group 1 (MutuallyExclusive): Control loop
        #   Only one control-related callback at a time.
        #   Timer + cmd_vel publisher share control state.
        self.control_group = MutuallyExclusiveCallbackGroup()

        # Group 2 (MutuallyExclusive): Perception pipeline
        #   Camera and lidar share the detection map.
        self.perception_group = MutuallyExclusiveCallbackGroup()

        # Group 3 (Reentrant): Independent monitoring
        #   Diagnostics callbacks that don't share state.
        self.monitor_group = ReentrantCallbackGroup()

        # ─── Shared State ───
        self.detection_map = {}           # Shared by perception group
        self.current_velocity = Twist()    # Shared by control group

        # ─── Perception callbacks (mutually exclusive) ───
        self.camera_sub = self.create_subscription(
            Image, '/camera/image', self.camera_callback, 10,
            callback_group=self.perception_group
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/scan', self.lidar_callback, 10,
            callback_group=self.perception_group
        )

        # ─── Control callbacks (mutually exclusive) ───
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(
            0.02, self.control_callback,  # 50Hz
            callback_group=self.control_group
        )

        # ─── Monitoring callbacks (reentrant — can overlap) ───
        self.diag_timer = self.create_timer(
            1.0, self.diagnostics_callback,
            callback_group=self.monitor_group
        )
        self.heartbeat_timer = self.create_timer(
            0.5, self.heartbeat_callback,
            callback_group=self.monitor_group
        )

        self.get_logger().info('AutonomousNode started with proper callback groups')

    def camera_callback(self, msg):
        """Heavy image processing — runs in perception_group."""
        self.get_logger().info('Processing camera frame...')
        time.sleep(0.15)  # 150ms inference
        self.detection_map['camera_objects'] = ['car', 'pedestrian']
        self.get_logger().info('Camera processing done')

    def lidar_callback(self, msg):
        """LiDAR processing — runs in perception_group.
        Cannot run simultaneously with camera_callback."""
        self.get_logger().info('Processing LiDAR scan...')
        time.sleep(0.05)  # 50ms processing
        self.detection_map['lidar_obstacles'] = [(3.0, 0.5), (5.0, -1.0)]
        self.get_logger().info('LiDAR processing done')

    def control_callback(self):
        """50Hz control loop — runs in control_group.
        NEVER blocked by perception callbacks!"""
        # Read detection results (read-only access to detection_map is safe)
        obstacles = self.detection_map.get('lidar_obstacles', [])

        # Simple reactive control
        if obstacles and obstacles[0][0] < 2.0:
            self.current_velocity.linear.x = 0.0  # Stop
        else:
            self.current_velocity.linear.x = 0.5  # Cruise

        self.cmd_pub.publish(self.current_velocity)

    def diagnostics_callback(self):
        """System diagnostics — can run in parallel with heartbeat."""
        self.get_logger().info(
            f'Diagnostics: {len(self.detection_map)} detection sources active'
        )

    def heartbeat_callback(self):
        """Heartbeat — can run in parallel with diagnostics."""
        self.get_logger().info('Heartbeat: alive')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()

    # Use MultiThreadedExecutor with enough threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Execution timeline:

```
Time ──────────────────────────────────────────────────►

Thread 1 (perception): [camera_cb 150ms     ][lidar_cb 50ms][camera_cb 150ms]
Thread 2 (control):    [c][c][c][c][c][c][c][c][c][c][c][c][c][c][c][c][c]
Thread 3 (monitor):    [diag]          [hb]  [diag]     [hb]
Thread 4 (monitor):         [hb]                   [hb]

Key observations:
- control_callback runs every 20ms uninterrupted (different group)
- camera and lidar never overlap (same MutuallyExclusive group)
- diagnostics and heartbeat CAN overlap (Reentrant group)
```

### 3.5 Latency Comparison: Before and After

Let's quantify the improvement:

```python
#!/usr/bin/env python3
"""latency_comparison.py — Measure control loop jitter with different executors."""

import time
import statistics
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class LatencyTestNode(Node):
    def __init__(self, use_groups=False):
        super().__init__('latency_test')

        self.control_group = MutuallyExclusiveCallbackGroup() if use_groups else None
        self.perception_group = MutuallyExclusiveCallbackGroup() if use_groups else None

        # Simulated heavy callback
        self.slow_timer = self.create_timer(
            0.1, self.slow_callback,
            callback_group=self.perception_group
        )

        # Control loop at 50Hz
        self.control_periods = []
        self.last_time = time.time()
        self.control_timer = self.create_timer(
            0.02, self.control_callback,
            callback_group=self.control_group
        )

    def slow_callback(self):
        time.sleep(0.15)  # 150ms heavy processing

    def control_callback(self):
        now = time.time()
        period = (now - self.last_time) * 1000  # ms
        self.last_time = now
        self.control_periods.append(period)

        if len(self.control_periods) >= 200:
            periods = self.control_periods[10:]  # skip warmup
            self.get_logger().info(
                f'\n'
                f'  Control Loop Statistics ({len(periods)} samples):\n'
                f'  Mean period:   {statistics.mean(periods):.1f} ms\n'
                f'  Stdev:         {statistics.stdev(periods):.1f} ms\n'
                f'  Max period:    {max(periods):.1f} ms\n'
                f'  Min period:    {min(periods):.1f} ms\n'
                f'  Target:        20.0 ms'
            )
            raise SystemExit()


def main():
    print("=" * 60)
    print("Test 1: SingleThreadedExecutor (BLOCKING)")
    print("=" * 60)
    rclpy.init()
    node1 = LatencyTestNode(use_groups=False)
    executor1 = SingleThreadedExecutor()
    executor1.add_node(node1)
    try:
        executor1.spin()
    except SystemExit:
        pass
    node1.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 60)
    print("Test 2: MultiThreadedExecutor + Callback Groups (FIXED)")
    print("=" * 60)
    rclpy.init()
    node2 = LatencyTestNode(use_groups=True)
    executor2 = MultiThreadedExecutor(num_threads=4)
    executor2.add_node(node2)
    try:
        executor2.spin()
    except SystemExit:
        pass
    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Typical output:

```
============================================================
Test 1: SingleThreadedExecutor (BLOCKING)
============================================================
  Control Loop Statistics (190 samples):
  Mean period:   35.2 ms           ← 75% over target!
  Stdev:         48.3 ms           ← huge jitter
  Max period:    172.1 ms          ← worst case: 8.6x target
  Min period:    2.1 ms
  Target:        20.0 ms

============================================================
Test 2: MultiThreadedExecutor + Callback Groups (FIXED)
============================================================
  Control Loop Statistics (190 samples):
  Mean period:   20.1 ms           ← on target
  Stdev:         1.2 ms            ← minimal jitter
  Max period:    23.4 ms           ← worst case: only 17% over
  Min period:    18.8 ms
  Target:        20.0 ms
```

The MultiThreadedExecutor with proper callback groups reduced worst-case latency from **172ms to 23ms** — a 7.5x improvement.

---

## 4. Intra-Process Communication: Zero-Copy

### 4.1 The Problem with Inter-Process Communication

When two nodes in **separate processes** communicate via a topic, the data path is:

```
Node A (Process 1)              Node B (Process 2)
┌──────────────┐               ┌──────────────┐
│ Create msg   │               │              │
│ Serialize    │──► DDS ──────►│ Deserialize  │
│ (copy 1)     │   network     │ (copy 2)     │
└──────────────┘               └──────────────┘

Two copies: one for serialization, one for deserialization.
For a 2MB camera image at 30fps: 2 × 2MB × 30 = 120 MB/s wasted.
```

### 4.2 Intra-Process Solution

When two nodes are in the **same process**, ROS2 can pass the message pointer directly — **zero copies**.

```python
#!/usr/bin/env python3
"""intra_process_example.py — Zero-copy communication within a single process."""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Create publisher that supports intra-process
        self.publisher = self.create_publisher(Image, '/camera/image', 10)
        self.timer = self.create_timer(1.0/30.0, self.capture)

    def capture(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.data = bytes(640 * 480 * 3)  # 900KB
        self.publisher.publish(msg)


class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.process, 10
        )

    def process(self, msg):
        self.get_logger().info(
            f'Received {msg.width}x{msg.height} image'
        )


def main(args=None):
    rclpy.init(args=args)

    # Both nodes in the SAME process
    camera = CameraNode()
    processor = ProcessorNode()

    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(processor)

    # When both nodes are in the same executor/process,
    # ROS2 can use intra-process communication (zero-copy)
    executor.spin()

    camera.destroy_node()
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

In C++ (rclcpp), intra-process communication is more mature and is enabled via the node options:

```cpp
auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
auto camera_node = std::make_shared<CameraNode>(options);
auto processor_node = std::make_shared<ProcessorNode>(options);
```

### 4.3 When to Use Intra-Process

| Scenario | Use Intra-Process? |
|----------|-------------------|
| Camera → Perception (same machine) | Yes — saves ~120 MB/s |
| Sensor fusion nodes (same machine) | Yes — reduces latency |
| Cross-machine communication | No — must use DDS |
| Debugging (need to echo topics) | Be careful — external subscribers get copies |

---

## 5. The Python GIL Problem

### 5.1 What Is the GIL?

Python has a **Global Interpreter Lock (GIL)** — a mutex that prevents multiple threads from executing Python bytecode simultaneously. Even with a MultiThreadedExecutor using 8 threads, **only one thread runs Python code at any given time**.

```
MultiThreadedExecutor with 4 threads in Python (rclpy):

Thread 1: [Python code][wait][Python code    ][wait]
Thread 2: [wait][Python code][wait][Python code][wait]
Thread 3: [wait       ][Python code][wait      ]
Thread 4: [wait              ][Python code     ]

The GIL means only one thread holds it at a time.
Threads still take turns, but there's no TRUE parallelism
for CPU-bound Python code.
```

### 5.2 Why MultiThreadedExecutor Still Helps in Python

Even with the GIL, the MultiThreadedExecutor helps because:

1. **I/O-bound operations release the GIL**: `time.sleep()`, network calls, and many C-extension operations (NumPy, OpenCV) release the GIL while running.

2. **Callback scheduling is still concurrent**: While one callback sleeps or does I/O, another can run.

```
With GIL, but OpenCV (C extension) releases it:

Thread 1: [Python setup][OpenCV inference (GIL released)     ][Python post]
Thread 2:         [Python control loop][sleep (GIL released)]
                   ↑
          This runs while Thread 1 is in OpenCV!
```

### 5.3 When to Switch to C++ (rclcpp)

Switch from rclpy to rclcpp when:

| Criterion | Stay in Python | Switch to C++ |
|-----------|---------------|---------------|
| Callback frequency | < 100 Hz | > 100 Hz |
| Processing time | I/O bound | CPU bound (pure Python) |
| Memory copies | Tolerable | Must be zero-copy |
| Development speed | Priority | Not priority |
| Control loops | > 10ms budget | < 1ms budget |

A common pattern in production autonomous vehicles:

```
Python nodes:                        C++ nodes:
- Mission planner                    - LiDAR driver
- High-level behavior                - Camera driver
- Visualization                      - Point cloud processing
- Parameter tuning                   - Control loop (50-200 Hz)
- Debugging/testing                  - Sensor fusion
```

---

## 6. Debugging and Visualization Tools

### 6.1 Command-Line Introspection

ROS2 provides powerful CLI tools for inspecting a running system:

```bash
# ─── Node inspection ───
ros2 node list                    # List all active nodes
ros2 node info /camera_node       # Show node's publishers, subscribers, services

# ─── Topic inspection ───
ros2 topic list                   # List all active topics
ros2 topic info /camera/image     # Show publishers/subscribers + types
ros2 topic info /camera/image --verbose  # Show QoS profiles
ros2 topic hz /camera/image       # Measure actual publish rate
ros2 topic bw /camera/image       # Measure bandwidth (bytes/sec)
ros2 topic echo /camera/image     # Print messages to terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.3}}'  # Publish manually

# ─── Service inspection ───
ros2 service list                 # List all active services
ros2 service type /count_objects  # Show service type
ros2 service call /count_objects my_interfaces/srv/CountObjects \
    '{roi_x: 0, roi_y: 0, roi_width: 640, roi_height: 480}'

# ─── Action inspection ───
ros2 action list                  # List all active actions
ros2 action info /navigate_to_pose  # Show action type and servers/clients

# ─── Parameter inspection ───
ros2 param list /perception_node  # List all parameters
ros2 param get /perception_node detection_threshold
ros2 param set /perception_node detection_threshold 0.8

# ─── TF2 inspection ───
ros2 run tf2_tools view_frames    # Generate TF tree PDF
ros2 run tf2_ros tf2_echo map base_link  # Print transform continuously
```

### 6.2 rqt_graph: Visualizing Node Topology

`rqt_graph` shows the complete communication graph — which nodes are connected via which topics.

```bash
ros2 run rqt_graph rqt_graph
```

This produces a graph like:

```
 ┌─────────────┐   /camera/image    ┌──────────────┐
 │ /camera_node │──────────────────►│/perception    │
 └─────────────┘                    │_node          │──┐
                                    └──────────────┘  │
 ┌─────────────┐   /lidar/scan      ┌──────────────┐  │ /detected
 │ /lidar_node  │──────────────────►│/fusion_node   │  │ _objects
 └─────────────┘                    └──────┬───────┘  │
                                           │          │
 ┌─────────────┐   /imu/data               │          │
 │ /imu_node    │──────────────────────────►│          │
 └─────────────┘                           ▼          ▼
                                    ┌──────────────┐
                                    │/planner_node  │
                                    └──────┬───────┘
                                           │ /cmd_vel
                                    ┌──────▼───────┐
                                    │/motor_node    │
                                    └──────────────┘
```

### 6.3 PlotJuggler: Real-Time Data Plotting

PlotJuggler is a powerful tool for plotting ROS2 topic data in real time. It's essential for tuning PID controllers, debugging sensor data, and measuring latency.

```bash
# Install
sudo apt install ros-humble-plotjuggler-ros

# Launch
ros2 run plotjuggler plotjuggler
```

Common uses:
- Plot `/cmd_vel.linear.x` over time to see velocity commands
- Compare `/odom.pose.pose.position.x` vs `/gps.position.x` to see drift
- Plot `/control_loop/period_ms` to measure jitter

### 6.4 Foxglove Studio: Web-Based Visualization

Foxglove Studio is a modern alternative to rviz2 that runs in a browser:

```bash
# Install Foxglove bridge
sudo apt install ros-humble-foxglove-bridge

# Launch the bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# Open https://studio.foxglove.dev in your browser
# Connect to ws://localhost:8765
```

Foxglove can display:
- 3D point clouds and camera images
- TF frames overlaid on sensor data
- Topic message inspection
- Custom panels and layouts

### 6.5 Debugging Workflow

When something isn't working in your ROS2 system, follow this debugging flowchart:

```
Problem: Node B doesn't receive data from Node A

Step 1: Are both nodes running?
  ros2 node list
  → If node missing: check launch file, check for crashes

Step 2: Is the topic being published?
  ros2 topic list
  ros2 topic hz /the_topic
  → If not published: check publisher code, check timer

Step 3: Are the topic types matching?
  ros2 topic info /the_topic
  → If type mismatch: fix message type in publisher or subscriber

Step 4: Are QoS profiles compatible?
  ros2 topic info /the_topic --verbose
  → If QoS mismatch: adjust to compatible policies

Step 5: Are they in the same Domain?
  echo $ROS_DOMAIN_ID (on both machines)
  → If different: set to same domain ID

Step 6: Is the network configured?
  ros2 multicast receive (on subscriber machine)
  ros2 multicast send (on publisher machine)
  → If no multicast: check firewall, network config
```

---

## 7. Hands-On Lab Summary

### Lab 1: Reproduce the Blocking Problem

1. Run `blocking_problem.py` with `SingleThreadedExecutor`
2. Observe control loop latency spikes in the log
3. Measure: what is the worst-case control period?

### Lab 2: Fix with MultiThreadedExecutor + Callback Groups

1. Run `multi_executor_fixed.py` with `MultiThreadedExecutor`
2. Observe that control loop runs at 50Hz without interruption
3. Measure: what is the worst-case control period now?

### Lab 3: TF2 Broadcaster

1. Run `static_tf_broadcaster.py` to publish sensor frames
2. Run `odom_tf_broadcaster.py` to publish odometry transform
3. Run `ros2 run tf2_tools view_frames` to visualize the TF tree
4. Run `ros2 run tf2_ros tf2_echo map lidar_link` to see the chained transform

### Lab 4: Full System Visualization

1. Launch all nodes from Labs 1-3
2. Run `rqt_graph` to visualize the complete node topology
3. Screenshot the graph for your team presentation (Day 16)

---

## 8. Review

### Key Takeaways

1. **TF2 provides a transform tree** that lets any node transform data between any two coordinate frames. The standard frames are map, odom, base_link, and sensor frames.

2. **SingleThreadedExecutor blocks** — a slow callback delays all other callbacks. This is the single most common performance bug in ROS2 Python nodes.

3. **MultiThreadedExecutor + Callback Groups** is the standard solution:
   - MutuallyExclusiveCallbackGroup: serializes callbacks that share state (like a Mutex)
   - ReentrantCallbackGroup: allows parallel execution for independent callbacks

4. **Intra-process communication** eliminates serialization overhead when nodes share a process. Critical for high-bandwidth data like camera images.

5. **Python's GIL limits true parallelism** — but MultiThreadedExecutor still helps for I/O-bound and C-extension operations. Switch to rclcpp for CPU-bound, high-frequency nodes.

6. **Debugging tools** (rqt_graph, topic hz/echo, PlotJuggler, Foxglove) are essential for understanding and troubleshooting a running system.

### Connection to Other Days

- **Day 5 (OS Threading)**: Executors are ROS2's callback schedulers, analogous to OS thread schedulers. MutuallyExclusiveCallbackGroup = Mutex. ReentrantCallbackGroup = independent threads.
- **Day 13 (ROS2 Architecture)**: QoS policies determine what data arrives; executors determine how fast it gets processed.
- **Day 15 (Tomorrow)**: We will use ros2_control to bridge from ROS2 topics to real motor hardware, and Nav2 to put the planning stack together.

### Quick Self-Check

1. What is the difference between the `odom` and `map` frames?
2. Why does SLAM correction go into the `map→odom` transform rather than `odom→base_link`?
3. If you have a callback that takes 100ms and a control loop at 100Hz, which executor do you need?
4. Two callbacks share a `dict`. Which callback group type should they use?
5. Why does MultiThreadedExecutor still help in Python despite the GIL?

---

*Next up: [Day 15 — ros2_control, Nav2, and First Vehicle Setup](/posts/embedded-day-15/) — where we connect ROS2 to real motors, explore the navigation stack, and bring up a physical vehicle for the first time.*
