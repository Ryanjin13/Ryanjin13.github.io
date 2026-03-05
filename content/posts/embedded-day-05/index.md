---
title: "Day 5 — Multithreading and Multiprocessing"
date: 2026-03-05
description: "Process vs thread internals, race conditions, synchronization primitives, Python GIL, and IPC mechanisms for real-time robotics"
categories: ["Autonomous Driving"]
tags: ["Multithreading", "Multiprocessing", "Concurrency", "Python GIL", "IPC"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 5
draft: false
---

{{< katex >}}

## What You'll Learn

- Process vs Thread at the OS level — memory layout, PCB/TCB, context switching costs
- Race conditions, deadlocks, and how to prevent them
- Python's GIL and when to use threading vs multiprocessing
- IPC mechanisms: Pipe, Queue, Shared Memory
- Why this matters: ROS2 Executors (Day 14) are built on these concepts

---

## 1. Process vs Thread

### Process

A **process** is an independent program in execution. Each process has its own:

```
Process A (PID 100)              Process B (PID 101)
┌──────────────────┐            ┌──────────────────┐
│   Code (.text)   │            │   Code (.text)   │
├──────────────────┤            ├──────────────────┤
│   Data (.data)   │            │   Data (.data)   │
├──────────────────┤            ├──────────────────┤
│      Heap        │            │      Heap        │
│    (malloc)      │            │    (malloc)      │
├──────────────────┤            ├──────────────────┤
│                  │            │                  │
│      Stack      │            │      Stack      │
└──────────────────┘            └──────────────────┘
  Completely isolated             Completely isolated
  memory space                    memory space
```

The OS kernel maintains a **Process Control Block (PCB)** for each process:

| PCB Field | Description |
|-----------|-------------|
| PID | Process identifier |
| State | Running, Ready, Blocked, Zombie |
| PC | Program counter (where execution is) |
| Registers | CPU register snapshot |
| Memory map | Page table pointer |
| Open files | File descriptor table |
| Signals | Pending signals |
| Priority | Scheduling priority |

### Thread

A **thread** is a lightweight execution unit **within** a process. Threads share the process's memory but have their own stack and registers:

```
Process A (PID 100)
┌──────────────────────────────────────┐
│   Code (.text)     ← shared         │
├──────────────────────────────────────┤
│   Data (.data)     ← shared         │
├──────────────────────────────────────┤
│   Heap             ← shared         │
├──────────────┬───────────────────────┤
│   Thread 0   │   Thread 1           │
│   Stack      │   Stack              │
│   Registers  │   Registers          │
│   PC         │   PC                 │
└──────────────┴───────────────────────┘
```

The OS maintains a **Thread Control Block (TCB)** — much smaller than a PCB:

| TCB Field | Description |
|-----------|-------------|
| Thread ID | Thread identifier |
| State | Running, Ready, Blocked |
| PC | This thread's program counter |
| Registers | This thread's register snapshot |
| Stack pointer | Points to this thread's stack |

### Context Switching Cost

When the OS switches between processes/threads, it must save and restore state:

**Process context switch** (~1-10 µs):
1. Save all CPU registers to outgoing PCB
2. Save memory mapping (page table base register)
3. Flush TLB (Translation Lookaside Buffer) — **this is expensive**
4. Load new page table from incoming PCB
5. Restore all CPU registers
6. Cache is now "cold" for the new process — performance penalty

**Thread context switch** (~0.1-1 µs):
1. Save CPU registers to outgoing TCB
2. Load CPU registers from incoming TCB
3. **No TLB flush** (same address space!)
4. **No page table switch** (same process!)
5. Cache is more likely to be "warm"

Thread switches are **~10× faster** than process switches because they share the same memory space.

---

## 2. Race Conditions and Synchronization

### Race Condition

A **race condition** occurs when two threads access shared data concurrently and at least one modifies it.

```python
# Shared variable
counter = 0

# Thread A                    # Thread B
# ---------                   # ---------
temp_a = counter  # reads 0   temp_b = counter  # reads 0
temp_a = temp_a + 1  # = 1    temp_b = temp_b + 1  # = 1
counter = temp_a     # = 1    counter = temp_b     # = 1

# Expected: counter = 2
# Actual:   counter = 1  ← BUG!
```

The problem: the read-modify-write sequence is not **atomic**. The OS can preempt a thread between any of these steps.

### Critical Section

A **critical section** is a code region that accesses shared resources and must not be executed by more than one thread simultaneously.

```python
# The fix: wrap the critical section with a lock
lock.acquire()
# --- Critical Section Start ---
temp = counter
temp = temp + 1
counter = temp
# --- Critical Section End ---
lock.release()
```

### Deadlock

**Deadlock** occurs when two or more threads are each waiting for a resource held by the other:

```
Thread A:                    Thread B:
  lock_1.acquire()  ✓         lock_2.acquire()  ✓
  lock_2.acquire()  ← waits   lock_1.acquire()  ← waits
  ...                         ...
  # Neither can proceed — DEADLOCK!
```

**Four conditions for deadlock** (all must hold):
1. **Mutual exclusion**: Only one thread can hold the resource
2. **Hold and wait**: Thread holds one resource while waiting for another
3. **No preemption**: Resources can't be forcibly taken away
4. **Circular wait**: A→waits for B→waits for A

**Prevention**: Always acquire locks in the same order. If all threads acquire lock_1 before lock_2, circular wait is impossible.

### Synchronization Primitives

#### Mutex (Mutual Exclusion)

A mutex allows only one thread into the critical section:

```python
import threading

mutex = threading.Lock()

def safe_increment():
    mutex.acquire()
    try:
        # Only one thread can be here at a time
        global counter
        counter += 1
    finally:
        mutex.release()  # Always release, even on exception

# Better syntax using 'with':
def safe_increment_v2():
    with mutex:
        global counter
        counter += 1
```

#### Semaphore

A semaphore allows up to N threads concurrently (a mutex is a semaphore with N=1):

```python
import threading

# Allow max 3 concurrent database connections
db_semaphore = threading.Semaphore(3)

def query_database(query_id):
    with db_semaphore:
        print(f"Query {query_id} executing (one of max 3)")
        # ... do database work ...
```

#### Condition Variable

A condition variable lets threads wait for a specific condition:

```python
import threading

condition = threading.Condition()
data_ready = False
shared_data = None

def producer():
    global data_ready, shared_data
    with condition:
        shared_data = "sensor_reading_42"
        data_ready = True
        condition.notify()  # Wake up one waiting thread

def consumer():
    global data_ready, shared_data
    with condition:
        while not data_ready:
            condition.wait()  # Sleep until notified
        print(f"Got data: {shared_data}")
```

---

## 3. Python's GIL (Global Interpreter Lock)

### What is the GIL?

CPython (the standard Python) has a **Global Interpreter Lock** — a mutex that protects access to Python objects. Only one thread can execute Python bytecode at a time.

```
Python Process
┌──────────────────────────────────────┐
│                GIL                   │
│         ┌──────────┐                 │
│         │  LOCKED  │                 │
│         └──────────┘                 │
│                                      │
│  Thread 0        Thread 1            │
│  ┌────────┐      ┌────────┐         │
│  │RUNNING │      │BLOCKED │         │
│  │Python  │      │waiting │         │
│  │bytecode│      │for GIL │         │
│  └────────┘      └────────┘         │
└──────────────────────────────────────┘
```

### When Threading Works (I/O Bound)

The GIL is **released during I/O operations** (file read, network, serial port). While one thread waits for I/O, another can run:

```python
import threading
import time

def read_sensor(name, port):
    """I/O bound — GIL is released during serial read."""
    # import serial
    # ser = serial.Serial(port, 115200)
    # data = ser.readline()  # GIL released during this blocking read
    time.sleep(0.1)  # Simulates I/O wait
    print(f"{name}: data received")

# These run concurrently despite GIL (I/O releases it)
t1 = threading.Thread(target=read_sensor, args=("IMU", "/dev/imu"))
t2 = threading.Thread(target=read_sensor, args=("LiDAR", "/dev/lidar"))
t1.start()
t2.start()
t1.join()
t2.join()
```

### When Multiprocessing is Needed (CPU Bound)

For CPU-intensive work, threading gives **no speedup** because of the GIL:

```python
import multiprocessing
import time
import numpy as np

def process_image(image_id):
    """CPU bound — needs separate process to bypass GIL."""
    # Simulate heavy computation
    data = np.random.rand(1000, 1000)
    result = np.linalg.svd(data, compute_uv=False)
    return f"Image {image_id} processed"

# Using multiprocessing.Pool for parallel CPU work
if __name__ == '__main__':
    start = time.time()

    with multiprocessing.Pool(processes=4) as pool:
        results = pool.map(process_image, range(8))

    elapsed = time.time() - start
    print(f"Processed {len(results)} images in {elapsed:.2f}s")
    print(f"Using {multiprocessing.cpu_count()} CPU cores")
```

### Decision Matrix

| Workload | threading | multiprocessing | Why |
|----------|-----------|----------------|-----|
| Reading 5 sensors via serial | Use threading | Overkill | I/O bound — GIL released during I/O |
| Processing 4 camera frames | Don't use | Use multiprocessing | CPU bound — GIL blocks parallelism |
| Web server (waiting for requests) | Use threading | Overkill | I/O bound |
| Training a neural network | Don't use | Use multiprocessing | CPU/GPU bound |
| ROS2 callbacks (mixed) | Use threading | For heavy compute nodes | Depends on callback workload |

### concurrent.futures — The Easy Way

```python
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import time

def io_task(sensor_id):
    time.sleep(0.1)  # Simulates I/O
    return f"Sensor {sensor_id} read"

def cpu_task(image_id):
    total = sum(i * i for i in range(1_000_000))  # CPU work
    return f"Image {image_id}: {total}"

# ThreadPoolExecutor for I/O bound
with ThreadPoolExecutor(max_workers=4) as executor:
    futures = [executor.submit(io_task, i) for i in range(10)]
    for f in futures:
        print(f.result())

# ProcessPoolExecutor for CPU bound
with ProcessPoolExecutor(max_workers=4) as executor:
    futures = [executor.submit(cpu_task, i) for i in range(8)]
    for f in futures:
        print(f.result())
```

---

## 4. IPC — Inter-Process Communication

Since processes have separate memory spaces, they need explicit mechanisms to communicate.

### Pipe

A simple one-way data channel between parent and child:

```python
from multiprocessing import Process, Pipe

def sensor_process(conn):
    """Child process: sends sensor data through pipe."""
    for i in range(5):
        reading = {"id": i, "value": 42.0 + i * 0.1}
        conn.send(reading)
    conn.send(None)  # Sentinel: signals end
    conn.close()

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()

    p = Process(target=sensor_process, args=(child_conn,))
    p.start()

    while True:
        data = parent_conn.recv()
        if data is None:
            break
        print(f"Received: {data}")

    p.join()
```

### Queue

Thread-safe and process-safe FIFO queue — the workhorse of producer-consumer patterns:

```python
from multiprocessing import Process, Queue
import time

def camera_producer(q):
    """Produces camera frames."""
    for frame_id in range(10):
        frame = f"frame_{frame_id}"
        q.put(frame)
        print(f"  [Producer] Captured {frame}")
        time.sleep(0.05)
    q.put(None)  # Poison pill

def processing_consumer(q):
    """Consumes and processes frames."""
    while True:
        frame = q.get()
        if frame is None:
            break
        # Simulate processing time
        time.sleep(0.1)
        print(f"  [Consumer] Processed {frame}")

if __name__ == '__main__':
    q = Queue(maxsize=5)  # Buffer up to 5 frames

    producer = Process(target=camera_producer, args=(q,))
    consumer = Process(target=processing_consumer, args=(q,))

    producer.start()
    consumer.start()

    producer.join()
    consumer.join()
    print("Done!")
```

### Shared Memory

For large data (like images), copying through Queue is slow. Shared memory provides zero-copy access:

```python
from multiprocessing import Process, shared_memory
import numpy as np

def writer_process(shm_name, shape, dtype):
    """Writes data to shared memory."""
    existing_shm = shared_memory.SharedMemory(name=shm_name)
    arr = np.ndarray(shape, dtype=dtype, buffer=existing_shm.buf)

    # Write sensor data
    arr[:] = np.random.rand(*shape) * 100
    print(f"Writer: wrote data, mean={arr.mean():.2f}")

    existing_shm.close()

if __name__ == '__main__':
    shape = (480, 640, 3)  # Camera frame size
    dtype = np.float32

    # Create shared memory
    dummy = np.zeros(shape, dtype=dtype)
    shm = shared_memory.SharedMemory(create=True, size=dummy.nbytes)

    # Main process can also access the array
    arr = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
    arr[:] = 0

    # Launch writer process
    p = Process(target=writer_process, args=(shm.name, shape, dtype))
    p.start()
    p.join()

    # Read what the writer wrote
    print(f"Reader: mean={arr.mean():.2f}")

    # Cleanup
    shm.close()
    shm.unlink()
```

---

## 5. Hands-On Lab

### Lab 1: Reproduce a Race Condition

```python
#!/usr/bin/env python3
"""Demonstrate race condition and fix with Lock."""

import threading
import time

counter = 0
NUM_INCREMENTS = 100_000

def increment_unsafe():
    global counter
    for _ in range(NUM_INCREMENTS):
        counter += 1  # NOT atomic!

def increment_safe(lock):
    global counter
    for _ in range(NUM_INCREMENTS):
        with lock:
            counter += 1

# --- Unsafe version ---
counter = 0
threads = [threading.Thread(target=increment_unsafe) for _ in range(4)]
start = time.time()
for t in threads:
    t.start()
for t in threads:
    t.join()
elapsed_unsafe = time.time() - start

print(f"UNSAFE: counter = {counter} (expected {NUM_INCREMENTS * 4})")
print(f"  Lost {NUM_INCREMENTS * 4 - counter} increments!")
print(f"  Time: {elapsed_unsafe:.3f}s")

# --- Safe version ---
counter = 0
lock = threading.Lock()
threads = [threading.Thread(target=increment_safe, args=(lock,)) for _ in range(4)]
start = time.time()
for t in threads:
    t.start()
for t in threads:
    t.join()
elapsed_safe = time.time() - start

print(f"\nSAFE: counter = {counter} (expected {NUM_INCREMENTS * 4})")
print(f"  Time: {elapsed_safe:.3f}s")
print(f"  Lock overhead: {elapsed_safe / elapsed_unsafe:.1f}x slower")
```

### Lab 2: Multiprocessing Image Batch Benchmark

```python
#!/usr/bin/env python3
"""Benchmark: threading vs multiprocessing for CPU-bound image processing."""

import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

def process_image(image_id):
    """Simulate image processing (CPU-bound)."""
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Gaussian blur simulation
    from scipy.ndimage import gaussian_filter
    blurred = gaussian_filter(img.astype(np.float32), sigma=3)

    # Edge detection simulation
    edges = np.gradient(blurred, axis=(0, 1))

    return image_id

def benchmark(executor_class, name, num_images=16, max_workers=4):
    start = time.time()
    with executor_class(max_workers=max_workers) as executor:
        list(executor.map(process_image, range(num_images)))
    elapsed = time.time() - start
    print(f"  {name}: {elapsed:.2f}s ({num_images/elapsed:.1f} images/sec)")
    return elapsed

if __name__ == '__main__':
    print(f"Processing 16 images on {__import__('os').cpu_count()} cores:")

    # Sequential baseline
    start = time.time()
    for i in range(16):
        process_image(i)
    seq_time = time.time() - start
    print(f"  Sequential: {seq_time:.2f}s ({16/seq_time:.1f} images/sec)")

    # Threading (limited by GIL for CPU work)
    thread_time = benchmark(ThreadPoolExecutor, "Threading", 16, 4)

    # Multiprocessing (bypasses GIL)
    mp_time = benchmark(ProcessPoolExecutor, "Multiprocessing", 16, 4)

    print(f"\nSpeedup: Multiprocessing is {seq_time/mp_time:.1f}x faster than sequential")
    print(f"         Threading is {seq_time/thread_time:.1f}x faster (GIL limited)")
```

### Lab 3: Producer-Consumer with Queue

```python
#!/usr/bin/env python3
"""Producer-consumer pattern: camera → processing pipeline."""

import threading
import queue
import time
import random

frame_queue = queue.Queue(maxsize=10)
result_queue = queue.Queue()
stop_event = threading.Event()

def camera_thread():
    """Simulates camera capturing frames."""
    frame_id = 0
    while not stop_event.is_set():
        frame = {"id": frame_id, "timestamp": time.time(), "data": f"pixels_{frame_id}"}
        try:
            frame_queue.put(frame, timeout=0.5)
            print(f"[Camera] Captured frame {frame_id}")
            frame_id += 1
        except queue.Full:
            print("[Camera] Queue full — dropping frame!")
        time.sleep(0.033)  # ~30 FPS

def processor_thread(worker_id):
    """Simulates image processing."""
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.5)
            # Simulate variable processing time
            process_time = random.uniform(0.02, 0.08)
            time.sleep(process_time)
            result = {
                "frame_id": frame["id"],
                "latency_ms": (time.time() - frame["timestamp"]) * 1000,
                "worker": worker_id
            }
            result_queue.put(result)
            print(f"[Worker {worker_id}] Processed frame {frame['id']} "
                  f"(latency: {result['latency_ms']:.1f}ms)")
        except queue.Empty:
            continue

# Launch threads
camera = threading.Thread(target=camera_thread, daemon=True)
workers = [threading.Thread(target=processor_thread, args=(i,), daemon=True)
           for i in range(3)]

camera.start()
for w in workers:
    w.start()

# Run for 3 seconds
time.sleep(3)
stop_event.set()
camera.join(timeout=1)
for w in workers:
    w.join(timeout=1)

# Statistics
total_processed = result_queue.qsize()
latencies = []
while not result_queue.empty():
    r = result_queue.get()
    latencies.append(r["latency_ms"])

if latencies:
    print(f"\n--- Statistics ---")
    print(f"Frames processed: {total_processed}")
    print(f"Avg latency: {sum(latencies)/len(latencies):.1f}ms")
    print(f"Max latency: {max(latencies):.1f}ms")
    print(f"Queue backlog: {frame_queue.qsize()}")
```

### Lab 4: Monitor CPU Usage with htop

```bash
# Install htop
sudo apt install htop

# Run htop while your multiprocessing script runs
htop

# What to look for:
# - 4 CPU bars at the top (one per Cortex-A76 core)
# - With threading: only 1 core at 100% (GIL!)
# - With multiprocessing: all 4 cores at 100%
# - Memory usage per process
# - Thread count per process
```

---

## 6. Preview: ROS2 Executors (Day 14)

Everything we learned today maps directly to ROS2:

| OS Concept | ROS2 Equivalent |
|-----------|-----------------|
| Thread | Callback execution |
| Mutex | MutuallyExclusiveCallbackGroup |
| Thread pool | MultiThreadedExecutor |
| Single thread | SingleThreadedExecutor |
| Queue | Topic subscription buffer |
| Race condition | Callback data conflicts |

On Day 14, we'll see:
- A camera callback that takes 100ms blocking a motor control callback that needs to run every 10ms
- How MultiThreadedExecutor + ReentrantCallbackGroup solves this
- Why understanding GIL matters for rclpy (Python ROS2) nodes

---

## 7. Review

### Key Takeaways

1. **Process** = isolated memory, expensive context switch. **Thread** = shared memory, cheap context switch.
2. **Race conditions** are prevented with mutexes, semaphores, and condition variables
3. **Python GIL**: Use `threading` for I/O-bound, `multiprocessing` for CPU-bound
4. **Queue** is the safest IPC pattern for producer-consumer (camera → processor)
5. These concepts are the **foundation** for understanding ROS2 Executors

### Discussion Question

"If your camera callback takes 50ms and your motor control loop needs to run every 10ms, what happens in a single-threaded executor?"

**Answer**: The motor control callback gets delayed by up to 50ms every time the camera callback runs. This causes jerky motor behavior and potentially unsafe driving. Solution: MultiThreadedExecutor with separate callback groups (Day 14).

### Looking Ahead

Tomorrow (Day 6), we move to **motors and encoders** — the actuators that make the car move. We'll learn about DC/BLDC motors, H-bridges, Hall effect sensors, and how to measure wheel speed in real-time using the GPIO interrupts we learned on Day 3.
