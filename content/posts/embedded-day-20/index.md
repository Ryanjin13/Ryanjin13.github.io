---
title: "Day 20 — Hailo-10 NPU and Final Integration Demo"
date: 2026-03-05T20:00:00
description: "Hailo-10 dataflow architecture, compilation pipeline from PyTorch to .hef, HailoRT inference, and complete autonomous car integration demo"
categories: ["Autonomous Driving"]
tags: ["Hailo", "NPU", "Edge AI", "Integration", "Autonomous Driving"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 20
draft: false
---

{{< katex >}}

## What You'll Learn

This is the final day of the Embedded Basics for Autonomous Car series. Over the past 19 days you have built, layer by layer, every component of an autonomous driving system: hardware assembly, firmware, communication protocols, motor control, SLAM, lane detection, sensor fusion, safety design, and object detection.

Today we bring it all together.

The morning focuses on the **Hailo-10 NPU** — a dedicated neural network accelerator that transforms your YOLOv5 model from a 2 FPS slideshow on CPU into a 25+ FPS real-time detector. The afternoon is the **final integration demo**: a fully autonomous car that follows lanes, detects objects, avoids obstacles, and does it all safely.

By the end of today you will:

1. Understand the **dataflow architecture** that makes NPUs fundamentally different from CPUs/GPUs.
2. Walk through the complete **compilation pipeline**: PyTorch → ONNX → Hailo Dataflow Compiler → `.hef`.
3. Run HailoRT inference and measure the CPU vs NPU performance difference.
4. Wrap Hailo inference in a ROS2 node.
5. Execute a **complete autonomous driving demo** combining all 20 days of work.
6. Have a roadmap for continuing beyond this course.

---

## Morning Session: Hailo-10 NPU

### 1. Why an NPU?

A neural network performs the same operations billions of times per inference: multiply, accumulate, activate. A **CPU** is designed for diverse tasks — branch prediction, out-of-order execution, cache hierarchies — all wasted overhead for neural networks. A **GPU** is better (massively parallel), but still fetches data from off-chip DRAM repeatedly.

An **NPU** (Neural Processing Unit) is purpose-built silicon that moves data through a pipeline of compute units, keeping intermediate results on-chip. No instruction fetching. No cache misses. Just multiply-accumulate at maximum throughput.

| Processor | Architecture | Strength | NN Efficiency |
|-----------|-------------|----------|--------------|
| CPU (RPi 5) | General purpose, sequential | Flexibility | Low (~1 TOPS) |
| GPU (Jetson) | SIMD parallel, shared memory | Throughput | Medium (~10 TOPS) |
| **NPU (Hailo-10)** | **Dataflow, on-chip SRAM** | **NN-specific** | **High (40 TOPS)** |

### 2. Hailo-10 Dataflow Architecture

#### 2.1 The Dataflow Model

In a traditional **von Neumann** architecture, instructions and data are fetched from memory, processed, and results written back. The processor is constantly waiting for memory.

In Hailo's **dataflow** architecture, data flows through a pipeline of hardware compute units. Each unit performs a specific operation (convolution, pooling, activation) and passes the result directly to the next unit through on-chip interconnect — no round-trip to DRAM.

```
Von Neumann (CPU/GPU):
  DRAM ←→ Cache ←→ ALU ←→ Cache ←→ DRAM
       [memory bottleneck at every step]

Dataflow (Hailo NPU):
  Input → [Conv1] → [BN1] → [ReLU1] → [Conv2] → [BN2] → ... → Output
              ↕          ↕          ↕         ↕
          [On-chip SRAM — no DRAM access for intermediate data]
```

**Key advantage:** Intermediate activations (feature maps between layers) stay **on-chip**. For a model like YOLOv5s, intermediate activations can be tens of megabytes — far larger than the model weights. Keeping them on-chip eliminates the memory bandwidth bottleneck.

#### 2.2 On-Chip SRAM Structure

The Hailo-10 has a large on-chip SRAM divided into memory banks that can be dynamically allocated to different layers. The Hailo compiler performs **memory scheduling** — it determines which layers execute simultaneously and how SRAM banks are shared.

$$
\text{Throughput} \propto \frac{\text{Compute capacity}}{\text{Memory access time}}
$$

By minimizing off-chip memory access, the Hailo NPU achieves near-theoretical throughput. The 40 TOPS (Tera Operations Per Second) rating assumes INT8 operations, which is why quantization (Day 19) is essential.

#### 2.3 Implication for Model Design

Not all models are equally efficient on the Hailo NPU. Models with:
- **Large intermediate activations** benefit most (activation data stays on-chip).
- **Standard operations** (Conv, BN, ReLU, MaxPool, Concat) are natively supported.
- **Exotic operations** (custom layers, dynamic shapes) may not be supported or require workarounds.

YOLOv5 is fully supported and optimized in the Hailo Model Zoo.

### 3. RPi 5 + Hailo M.2 HAT Connection

#### 3.1 Physical Setup

The Hailo-10 module connects to the Raspberry Pi 5 via the **M.2 HAT** (Hardware Attached on Top):

```
┌───────────────────────────┐
│     Raspberry Pi 5         │
│                            │
│  CPU: Cortex-A76 (4 core) │
│  RAM: 8 GB LPDDR4X        │
│                            │
│  ┌─── PCIe 2.0 x1 ──────┐ │
│  │                        │ │
│  │   Hailo M.2 HAT       │ │
│  │   ┌───────────────┐   │ │
│  │   │  Hailo-10      │   │ │
│  │   │  40 TOPS INT8  │   │ │
│  │   │  On-chip SRAM  │   │ │
│  │   └───────────────┘   │ │
│  └────────────────────────┘ │
└───────────────────────────┘
```

#### 3.2 PCIe 2.0 x1 Bandwidth

The PCIe 2.0 x1 interface provides:

$$
\text{Bandwidth} = 5 \text{ GT/s} \times \frac{8}{10} \text{ (encoding)} = 4 \text{ Gbit/s} = 500 \text{ MB/s}
$$

For comparison:

| Interface | Theoretical Bandwidth | Practical |
|-----------|----------------------|-----------|
| **PCIe 2.0 x1** | **500 MB/s** | **~400 MB/s** |
| USB 3.0 | 5 Gbps = 625 MB/s | ~350 MB/s |
| USB 2.0 | 480 Mbps = 60 MB/s | ~35 MB/s |

PCIe has lower overhead and more consistent latency than USB, making it the preferred connection for real-time inference.

**Is 500 MB/s enough?** For YOLOv5s with 640x640 input:

$$
\text{Input size} = 640 \times 640 \times 3 \times 1 \text{ byte (INT8)} = 1.2 \text{ MB}
$$

$$
\text{Output size} \approx 0.1 \text{ MB}
$$

At 30 FPS: \(30 \times 1.3 \text{ MB} = 39 \text{ MB/s}\). This is well within the 400 MB/s practical bandwidth. The PCIe link is not the bottleneck.

#### 3.3 Setup Verification

```bash
# Check if Hailo device is detected
hailortcli fw-control identify

# Expected output:
# Executing on device: 0000:01:00.0
# Identifying board
# Control Protocol Version: 2
# Firmware Version: 4.18.0
# Board Name: Hailo-10
# ...

# Check PCIe link
lspci | grep Hailo
# Expected: 01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-10 (rev 01)

# Check HailoRT version
pip show hailort
```

### 4. Compilation Pipeline

This is the critical path from a trained model to a deployable NPU binary.

#### 4.1 Overview

```
┌─────────────┐     ┌─────────────┐     ┌───────────────────┐     ┌─────────┐
│  PyTorch     │ ──► │   ONNX      │ ──► │ Hailo Dataflow    │ ──► │  .hef   │
│  model.pt   │     │  model.onnx │     │ Compiler (DFC)    │     │ (INT8)  │
└─────────────┘     └─────────────┘     │                   │     └─────────┘
                                         │ - Parse ONNX      │
                                         │ - Quantize (INT8)  │
                                         │ - Optimize graph   │
                                         │ - Schedule memory  │
                                         │ - Generate binary  │
                                         └───────────────────┘
                                                  ↑
                                         Calibration images
                                         (100+ real photos)
```

#### 4.2 Step 1: PyTorch to ONNX

We did this at the end of Day 19:

```bash
cd yolov5
python export.py \
    --weights runs/train/track_signs_v1/weights/best.pt \
    --img 640 \
    --batch 1 \
    --include onnx \
    --simplify \
    --opset 11
```

The `--simplify` flag runs ONNX Simplifier to remove redundant operations. The `--opset 11` ensures compatibility with the Hailo compiler.

#### 4.3 Step 2: ONNX to HAR (Hailo Archive)

The Hailo Dataflow Compiler converts the ONNX model into an internal representation:

```python
"""
hailo_compile.py — Convert ONNX model to Hailo .hef

This script uses the Hailo Dataflow Compiler (DFC) Python API.
Install: pip install hailo_dataflow_compiler
"""

from hailo_sdk_client import ClientRunner

# Step 1: Parse ONNX to HAR
runner = ClientRunner(hw_arch="hailo10")
hn, npz = runner.translate_onnx_model(
    "best.onnx",
    net_name="yolov5s_custom",
    start_node_names=["images"],       # input tensor name
    end_node_names=["output0"],        # output tensor name
    net_input_shapes={"images": [1, 3, 640, 640]},
)
runner.save_har("yolov5s_custom.har")
print("HAR file created.")
```

#### 4.4 Step 3: Quantization (INT8 PTQ via Calibration)

The Hailo compiler performs INT8 Post-Training Quantization. It needs a **calibration dataset** — a set of representative images that the compiler runs through the model to determine the optimal quantization ranges for each layer.

```python
import numpy as np
from pathlib import Path
import cv2

# Prepare calibration data
# Requirements:
#   - At least 100 images from the ACTUAL deployment environment
#   - Diverse lighting, angles, object positions
#   - Same preprocessing as training (resize to 640x640, normalize)

def load_calibration_images(image_dir, input_size=640, max_images=200):
    """Load and preprocess calibration images."""
    images = []
    paths = sorted(Path(image_dir).glob("*.jpg"))[:max_images]

    for p in paths:
        img = cv2.imread(str(p))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Letterbox resize
        h, w = img.shape[:2]
        scale = min(input_size / h, input_size / w)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (new_w, new_h))

        canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        dw = (input_size - new_w) // 2
        dh = (input_size - new_h) // 2
        canvas[dh:dh + new_h, dw:dw + new_w] = resized

        # Normalize to [0, 1] and transpose to CHW
        normalized = canvas.astype(np.float32) / 255.0
        transposed = np.transpose(normalized, (2, 0, 1))
        images.append(transposed)

    calib_data = np.stack(images)
    print(f"Loaded {len(images)} calibration images, shape: {calib_data.shape}")
    return calib_data


# Load calibration data
calib_dataset = load_calibration_images(
    "/path/to/calibration_images/",
    input_size=640,
    max_images=200,
)

# Apply quantization
runner = ClientRunner(hw_arch="hailo10")
runner.load_har("yolov5s_custom.har")

# Configure quantization
runner.optimize(calib_dataset)

# Save quantized HAR
runner.save_har("yolov5s_custom_quantized.har")
print("Quantized HAR saved.")
```

#### 4.5 Step 4: Compile to .hef

```python
# Compile to Hailo Executable Format
hef = runner.compile()

# Save .hef file
with open("yolov5s_custom.hef", "wb") as f:
    f.write(hef)

print("HEF file compiled successfully.")
```

**Compiler optimization levels:**

| Level | Speed | Compile Time | Use Case |
|-------|-------|-------------|----------|
| 0 | Fastest compile | Minutes | Quick testing |
| 1 | Balanced | ~30 minutes | Default |
| 2 | Best performance | Hours | Final deployment |

#### 4.6 Complete Compilation Script

```python
"""
compile_for_hailo.py — Full pipeline from ONNX to .hef
"""

from hailo_sdk_client import ClientRunner
import numpy as np


def compile_model(onnx_path, calib_data, output_hef, hw_arch="hailo10",
                  optimization_level=2, batch_size=1):
    """Complete ONNX → .hef compilation pipeline."""

    print(f"[1/4] Parsing ONNX model: {onnx_path}")
    runner = ClientRunner(hw_arch=hw_arch)
    hn, npz = runner.translate_onnx_model(
        onnx_path,
        net_name="yolov5s_custom",
        start_node_names=["images"],
        end_node_names=["output0"],
        net_input_shapes={"images": [batch_size, 3, 640, 640]},
    )

    print(f"[2/4] Quantizing with {len(calib_data)} calibration images...")
    runner.optimize(calib_data)

    print(f"[3/4] Compiling to HEF (optimization level {optimization_level})...")
    hef = runner.compile(optimization_level=optimization_level)

    print(f"[4/4] Saving to {output_hef}")
    with open(output_hef, "wb") as f:
        f.write(hef)

    print(f"Done. HEF size: {len(hef) / 1e6:.1f} MB")
    return output_hef


if __name__ == "__main__":
    calib_data = load_calibration_images(
        "/path/to/calibration_images/", max_images=200
    )
    compile_model(
        onnx_path="best.onnx",
        calib_data=calib_data,
        output_hef="yolov5s_custom.hef",
        optimization_level=2,
    )
```

### 5. HailoRT Inference

#### 5.1 Using the Hailo Model Zoo

For quick testing, the Hailo Model Zoo provides pre-compiled `.hef` files for popular models:

```bash
# Install Hailo Model Zoo
pip install hailo_model_zoo

# List available models
hailomz list | grep yolov5

# Download pre-compiled YOLOv5s
hailomz compile yolov5s --hw-arch hailo10

# Or use pre-compiled .hef from model zoo
hailomz eval yolov5s --hw-arch hailo10 --target hailo10
```

#### 5.2 HailoRT Python Inference

```python
"""
hailo_inference.py — Run YOLOv5 inference on Hailo-10 NPU.
"""

from hailo_platform import (
    HailoRTDevice,
    VDevice,
    HailoStreamInterface,
    InferVStreams,
    ConfigureParams,
    InputVStreamParams,
    OutputVStreamParams,
    FormatType,
)
import numpy as np
import cv2
import time


class HailoYOLOv5:
    """YOLOv5 inference engine using Hailo-10 NPU."""

    def __init__(self, hef_path, conf_thresh=0.5, iou_thresh=0.45,
                 class_names=None):
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.class_names = class_names or []

        # Initialize Hailo device
        self.params = VDevice.create_params()
        self.vdevice = VDevice(self.params)

        # Load HEF
        self.hef = self.vdevice.create_hef(hef_path)

        # Configure network
        self.configure_params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.vdevice.configure(
            self.hef, self.configure_params
        )[0]

        # Get input/output stream info
        self.input_vstream_info = self.hef.get_input_vstream_infos()
        self.output_vstream_info = self.hef.get_output_vstream_infos()

        self.input_shape = self.input_vstream_info[0].shape
        self.input_size = self.input_shape[1]  # assuming square input

        print(f"Hailo model loaded: {hef_path}")
        print(f"  Input shape: {self.input_shape}")
        print(f"  Output streams: {len(self.output_vstream_info)}")

    def preprocess(self, frame):
        """Preprocess frame for Hailo inference."""
        h, w = frame.shape[:2]
        scale = min(self.input_size / h, self.input_size / w)
        new_w, new_h = int(w * scale), int(h * scale)

        resized = cv2.resize(frame, (new_w, new_h))
        canvas = np.full(
            (self.input_size, self.input_size, 3), 114, dtype=np.uint8
        )
        dw = (self.input_size - new_w) // 2
        dh = (self.input_size - new_h) // 2
        canvas[dh:dh + new_h, dw:dw + new_w] = resized

        # Hailo expects NHWC uint8 (no normalization — HEF includes it)
        input_data = np.expand_dims(canvas, axis=0)
        return input_data, scale, dw, dh

    def postprocess(self, raw_output, scale, dw, dh, orig_h, orig_w):
        """Decode Hailo output to detections."""
        # The exact postprocessing depends on the HEF's output format.
        # For YOLOv5 from Hailo Model Zoo, outputs are typically
        # already decoded bounding boxes.

        detections = []

        # Iterate over output tensors (one per scale)
        for output in raw_output.values():
            data = output[0]  # remove batch dimension
            # Each detection: [x1, y1, x2, y2, confidence, class_id]
            for det in data:
                conf = det[4]
                if conf < self.conf_thresh:
                    continue

                x1 = int((det[0] - dw) / scale)
                y1 = int((det[1] - dh) / scale)
                x2 = int((det[2] - dw) / scale)
                y2 = int((det[3] - dh) / scale)

                x1 = max(0, min(x1, orig_w))
                y1 = max(0, min(y1, orig_h))
                x2 = max(0, min(x2, orig_w))
                y2 = max(0, min(y2, orig_h))

                class_id = int(det[5])
                detections.append({
                    "box": [x1, y1, x2 - x1, y2 - y1],
                    "confidence": float(conf),
                    "class_id": class_id,
                })

        # NMS (if not already applied in HEF)
        if detections:
            boxes = [d["box"] for d in detections]
            confs = [d["confidence"] for d in detections]
            indices = cv2.dnn.NMSBoxes(boxes, confs, self.conf_thresh,
                                        self.iou_thresh)
            detections = [detections[i if isinstance(i, int) else i[0]]
                          for i in indices]

        return detections

    def detect(self, frame):
        """Run full detection pipeline."""
        h, w = frame.shape[:2]
        input_data, scale, dw, dh = self.preprocess(frame)

        # Configure virtual streams
        input_params = InputVStreamParams.make_from_network_group(
            self.network_group, quantized=False,
            format_type=FormatType.UINT8
        )
        output_params = OutputVStreamParams.make_from_network_group(
            self.network_group, quantized=False,
            format_type=FormatType.FLOAT32
        )

        # Run inference
        with InferVStreams(self.network_group, input_params,
                          output_params) as pipeline:
            input_dict = {
                self.input_vstream_info[0].name: input_data
            }
            raw_output = pipeline.infer(input_dict)

        return self.postprocess(raw_output, scale, dw, dh, h, w)

    def __del__(self):
        if hasattr(self, 'vdevice'):
            self.vdevice.release()
```

#### 5.3 CPU vs Hailo Benchmark

```python
"""
benchmark_cpu_vs_hailo.py — Compare CPU and NPU inference performance.
"""

import cv2
import numpy as np
import time


def benchmark(detector, source, n_frames=100, name="Model"):
    """Run benchmark and return results dict."""
    cap = cv2.VideoCapture(source)
    times = []

    for i in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()

        t0 = time.perf_counter()
        results = detector.detect(frame)
        t1 = time.perf_counter()
        times.append(t1 - t0)

    cap.release()

    return {
        "name": name,
        "avg_ms": np.mean(times) * 1000,
        "min_ms": np.min(times) * 1000,
        "max_ms": np.max(times) * 1000,
        "p95_ms": np.percentile(times, 95) * 1000,
        "avg_fps": 1000 / (np.mean(times) * 1000),
    }


if __name__ == "__main__":
    # CPU baseline (OpenCV DNN from Day 19)
    from day19_inference import YOLOv5OpenCV
    cpu_detector = YOLOv5OpenCV("best.onnx")
    cpu_results = benchmark(cpu_detector, "test_video.mp4", name="CPU (OpenCV DNN)")

    # Hailo NPU
    hailo_detector = HailoYOLOv5("yolov5s_custom.hef")
    hailo_results = benchmark(hailo_detector, "test_video.mp4", name="Hailo-10 NPU")

    # Print comparison table
    print(f"\n{'='*60}")
    print(f"{'Metric':<25} {'CPU':>15} {'Hailo NPU':>15}")
    print(f"{'='*60}")
    for key in ["avg_ms", "min_ms", "max_ms", "p95_ms", "avg_fps"]:
        unit = "ms" if "ms" in key else "FPS"
        print(f"{key:<25} {cpu_results[key]:>12.1f} {unit:>2} "
              f"{hailo_results[key]:>10.1f} {unit:>2}")
    speedup = cpu_results["avg_ms"] / hailo_results["avg_ms"]
    print(f"{'Speedup':<25} {'1.0x':>15} {f'{speedup:.1f}x':>15}")
    print(f"{'='*60}")
```

**Expected results:**

| Metric | CPU (OpenCV DNN) | Hailo-10 NPU |
|--------|-----------------|--------------|
| Avg inference time | ~500 ms | ~35 ms |
| Avg FPS | ~2 | ~28 |
| P95 latency | ~600 ms | ~40 ms |
| **Speedup** | **1.0x** | **~14x** |

### 6. Preprocessing / Inference / Postprocessing Pipelining

To maximize throughput, overlap the three stages using threading:

```
Frame N:     [Preprocess] [    Inference    ] [Postprocess]
Frame N+1:                [Preprocess] [    Inference    ] [Postprocess]
Frame N+2:                             [Preprocess] [    Inference    ] ...

Without pipelining: Total = Pre + Inf + Post = 5 + 35 + 5 = 45 ms → 22 FPS
With pipelining:    Total = max(Pre, Inf, Post) = 35 ms → 28 FPS
```

```python
import threading
from queue import Queue


class PipelinedDetector:
    """Three-stage pipelined detector for maximum throughput."""

    def __init__(self, hailo_detector):
        self.detector = hailo_detector
        self.preprocess_q = Queue(maxsize=2)
        self.inference_q = Queue(maxsize=2)
        self.result_q = Queue(maxsize=2)
        self.running = True

        # Start pipeline threads
        self.preprocess_thread = threading.Thread(
            target=self._preprocess_worker, daemon=True
        )
        self.inference_thread = threading.Thread(
            target=self._inference_worker, daemon=True
        )

        self.preprocess_thread.start()
        self.inference_thread.start()

    def submit(self, frame):
        """Submit a frame for detection."""
        self.preprocess_q.put(frame)

    def get_result(self, timeout=1.0):
        """Get detection results (blocking)."""
        return self.result_q.get(timeout=timeout)

    def _preprocess_worker(self):
        while self.running:
            frame = self.preprocess_q.get()
            h, w = frame.shape[:2]
            input_data, scale, dw, dh = self.detector.preprocess(frame)
            self.inference_q.put((input_data, scale, dw, dh, h, w, frame))

    def _inference_worker(self):
        while self.running:
            input_data, scale, dw, dh, h, w, frame = self.inference_q.get()
            # Run inference (simplified — actual Hailo inference call)
            detections = self.detector.detect(frame)
            self.result_q.put((frame, detections))

    def stop(self):
        self.running = False
```

### 7. ROS2 Hailo Detection Node

```python
#!/usr/bin/env python3
"""
hailo_detection_node.py — YOLOv5 object detection via Hailo-10 NPU.

Subscribes to: /camera/image_raw (sensor_msgs/Image)
Publishes:
  /detection/results  (String)  — JSON array of detections
  /detection/image    (Image)   — annotated image
  /detection/fps      (Float32) — current inference FPS
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import time


class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__("hailo_detection_node")

        # Parameters
        self.declare_parameter("hef_path", "yolov5s_custom.hef")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("class_names",
                               ["stop_sign", "speed_limit",
                                "pedestrian", "traffic_cone"])

        hef_path = self.get_parameter("hef_path").value
        conf = self.get_parameter("conf_threshold").value
        iou = self.get_parameter("iou_threshold").value
        self.class_names = self.get_parameter("class_names").value

        # Initialize Hailo detector
        self.detector = HailoYOLOv5(
            hef_path, conf_thresh=conf, iou_thresh=iou,
            class_names=self.class_names
        )

        self.bridge = CvBridge()
        self.frame_times = []

        # Publishers
        self.pub_results = self.create_publisher(String, "/detection/results", 10)
        self.pub_image = self.create_publisher(Image, "/detection/image", 1)
        self.pub_fps = self.create_publisher(Float32, "/detection/fps", 10)

        # Subscriber
        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.get_logger().info(
            f"Hailo detection node started with {hef_path}"
        )

    def image_callback(self, msg):
        t_start = time.perf_counter()

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = self.detector.detect(frame)

        t_end = time.perf_counter()
        dt_ms = (t_end - t_start) * 1000

        # Track FPS (rolling average over 30 frames)
        self.frame_times.append(dt_ms)
        if len(self.frame_times) > 30:
            self.frame_times.pop(0)
        avg_fps = 1000.0 / np.mean(self.frame_times)

        # Publish results as JSON
        results_json = json.dumps([
            {
                "class": self.class_names[d["class_id"]]
                         if d["class_id"] < len(self.class_names)
                         else str(d["class_id"]),
                "confidence": round(d["confidence"], 3),
                "box": d["box"],
            }
            for d in detections
        ])
        self.pub_results.publish(String(data=results_json))

        # Publish FPS
        self.pub_fps.publish(Float32(data=avg_fps))

        # Publish annotated image (every 2nd frame)
        if len(self.frame_times) % 2 == 0:
            vis = frame.copy()
            for d in detections:
                x, y, w, h = d["box"]
                cls = (self.class_names[d["class_id"]]
                       if d["class_id"] < len(self.class_names)
                       else "?")
                label = f"{cls} {d['confidence']:.2f}"
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(vis, label, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(vis, f"FPS: {avg_fps:.1f} | {dt_ms:.0f}ms",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 0), 2)

            self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(vis, "bgr8")
            )


def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Afternoon Session: Final Integration Demo

### 8. System Architecture — All 20 Days Combined

```
┌──────────────────────────────────────────────────────────────┐
│                    COMPLETE SYSTEM ARCHITECTURE                │
│                                                                │
│  ┌──────────┐    /camera/image_raw                            │
│  │ USB Camera│ ──────────────┬────────────────────┐           │
│  └──────────┘                │                    │           │
│                              ▼                    ▼           │
│               ┌──────────────────┐  ┌──────────────────────┐ │
│               │ lane_detection    │  │ hailo_detection       │ │
│               │ (Day 17 pipeline) │  │ (Hailo-10 YOLOv5)    │ │
│               │                  │  │                      │ │
│               │ → /lane/cte      │  │ → /detection/results │ │
│               │ → /lane/conf     │  │ → /detection/fps     │ │
│               └────────┬─────────┘  └──────────┬───────────┘ │
│                        │                        │             │
│  ┌──────────┐          │                        │             │
│  │ 1D LiDAR │ ─────────┼───── /lidar/distance   │             │
│  │ (TF-Luna)│          │                        │             │
│  └──────────┘          ▼                        ▼             │
│               ┌────────────────────────────────────┐          │
│               │       decision_maker_node           │          │
│               │       (Day 18 fusion + safety)      │          │
│               │                                    │          │
│               │  - State machine                   │          │
│               │  - Watchdog timers                 │          │
│               │  - PID steering (Day 9)            │          │
│               │  - Emergency stop logic            │          │
│               └──────────────┬─────────────────────┘          │
│                              │                                │
│                              ▼ /cmd_vel                       │
│               ┌────────────────────────────┐                  │
│               │  motor_controller_node      │                  │
│               │  (Day 6-8 firmware)         │                  │
│               │  → PWM signals to motors    │                  │
│               └────────────────────────────┘                  │
│                                                                │
│  ┌──────────────────────────────────┐                         │
│  │  RTAB-Map (Day 15)               │  (runs in background)   │
│  │  → /map (OccupancyGrid)          │                         │
│  │  → /rtabmap/odom (Odometry)      │                         │
│  └──────────────────────────────────┘                         │
└──────────────────────────────────────────────────────────────┘
```

### 9. Launch File — Starting Everything

```python
"""
launch/full_system.launch.py — Launch the complete autonomous driving system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Camera ─────────────────────────────────
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            parameters=[{
                "image_size": [640, 480],
                "camera_frame_id": "camera_link",
            }],
        ),

        # ── Lane Detection (Day 17) ───────────────
        Node(
            package="my_autonomous_pkg",
            executable="lane_detection_node",
            name="lane_detection",
            parameters=[{
                "calibration_file": "/home/pi/calibration.pkl",
                "lane_width_meters": 0.30,
                "n_windows": 9,
                "window_margin": 80,
            }],
        ),

        # ── Hailo Object Detection (Day 20) ───────
        Node(
            package="my_autonomous_pkg",
            executable="hailo_detection_node",
            name="hailo_detection",
            parameters=[{
                "hef_path": "/home/pi/models/yolov5s_custom.hef",
                "conf_threshold": 0.5,
                "class_names": ["stop_sign", "speed_limit",
                                "pedestrian", "traffic_cone"],
            }],
        ),

        # ── LiDAR Driver ──────────────────────────
        Node(
            package="my_autonomous_pkg",
            executable="lidar_driver_node",
            name="lidar",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "baud_rate": 115200,
            }],
        ),

        # ── Decision Maker (Day 18) ───────────────
        Node(
            package="my_autonomous_pkg",
            executable="decision_maker_node",
            name="decision_maker",
            parameters=[{
                "base_speed": 0.25,
                "kp_steering": 2.0,
                "ki_steering": 0.0,
                "kd_steering": 0.5,
                "obstacle_stop_dist": 0.15,
                "obstacle_slow_dist": 0.50,
                "control_rate": 50.0,
            }],
        ),

        # ── Motor Controller ──────────────────────
        Node(
            package="my_autonomous_pkg",
            executable="motor_controller_node",
            name="motor_controller",
        ),
    ])
```

```bash
# Launch the full system
ros2 launch my_autonomous_pkg full_system.launch.py

# In another terminal: monitor all topics
ros2 topic list
ros2 topic hz /lane/cte /detection/fps /cmd_vel

# Record everything for analysis
ros2 bag record -a -o final_demo_run
```

### 10. Demo Procedure

```
FINAL INTEGRATION DEMO — Checklist

Pre-flight:
  [ ] Battery fully charged
  [ ] Camera connected and streaming: ros2 topic hz /camera/image_raw
  [ ] LiDAR connected: ros2 topic echo /lidar/distance
  [ ] Hailo detected: hailortcli fw-control identify
  [ ] Track clear, lane markings visible
  [ ] Emergency stop button accessible

Demo runs:

  Run 1 — Lane Following Only
    - Start system
    - Observe: smooth lane tracking, CTE < 5 cm
    - Duration: 3 laps
    - Expected: car stays centered in lane

  Run 2 — Lane Following + Object Detection
    - Place traffic cone on track
    - Start system
    - Observe: car detects cone, slows down, stops safely
    - Check: detection result published to /detection/results
    - Expected: stops > 10 cm from cone

  Run 3 — Full System (Lane + Detection + RTAB-Map)
    - Enable RTAB-Map node
    - Run 3 laps
    - Check: map building in real-time via rviz2
    - Expected: occupancy grid shows track layout

  Run 4 — Failure Injection
    - Mid-run: cover camera lens
    - Expected: E-STOP within 1 second
    - Mid-run: unplug LiDAR
    - Expected: DEGRADED state, reduced speed
    - Press manual E-stop button
    - Expected: immediate stop

Post-demo:
  [ ] Stop ros2 bag recording
  [ ] Run analyze_bag.py on recorded data
  [ ] Document all metrics in report
```

### 11. Performance Metrics

| Metric | Target | Actual (fill in) |
|--------|--------|----|
| Lane following CTE | < 5 cm avg | ___ cm |
| Object detection FPS (Hailo) | > 20 FPS | ___ FPS |
| Object detection FPS (CPU) | ~2 FPS | ___ FPS |
| Obstacle stop distance | > 10 cm | ___ cm |
| E-stop latency | < 200 ms | ___ ms |
| End-to-end pipeline latency | < 100 ms | ___ ms |
| System uptime (no crashes) | 3 laps | ___ laps |
| RTAB-Map drift | < 10% of track length | ___ % |

---

## 12. Course Retrospective: KPT

After the demo, conduct a **KPT retrospective** (Keep / Problem / Try):

### Keep (What went well?)
- What parts of the project worked reliably?
- Which design decisions were the best?
- What skills did you develop that are most valuable?

### Problem (What was difficult?)
- Where did you spend the most debugging time?
- Which concepts were hardest to understand?
- What hardware issues caused the most frustration?

### Try (What would you do differently next time?)
- What improvements would you make to the pipeline?
- What would you add if you had more time?
- How would you improve the testing process?

---

## 13. Advanced Learning Roadmap

This course covered the fundamentals. Here is where to go next:

### 13.1 3D LiDAR + PointPillars 3D Object Detection

Upgrade from a 1D LiDAR to a 3D LiDAR (e.g., Livox Mid-360). Use the **PointPillars** algorithm to detect objects in 3D point clouds:

- Input: 3D point cloud (x, y, z, intensity)
- Output: 3D bounding boxes (x, y, z, width, height, length, yaw)
- Key concept: Convert point cloud to 2D pseudo-image using pillars, then apply 2D CNN

### 13.2 Reinforcement Learning (RL) Based Autonomous Control

Replace the PID + rules-based controller with a learned policy:

- **State**: camera image + LiDAR distance + velocity
- **Action**: steering angle + throttle
- **Reward**: positive for staying in lane, negative for collisions or lane departure
- Algorithms: PPO (Proximal Policy Optimization), SAC (Soft Actor-Critic)
- Simulation: CARLA simulator for safe training before real-world deployment

### 13.3 ROS2 Real-Time Patches

For safety-critical systems, standard Linux is not deterministic enough. Real-time patches provide guaranteed worst-case latencies:

- **PREEMPT-RT**: Linux kernel patch for soft real-time (~100 us worst case)
- **Xenomai**: Dual-kernel approach for hard real-time (~10 us worst case)
- Important for: motor control loops, emergency stop response

### 13.4 Hailo QAT + Custom Model Optimization

Go beyond PTQ with Quantization-Aware Training for better INT8 accuracy:

- Use Hailo's QAT plugin for PyTorch
- Optimize custom layers for Hailo hardware
- Profile memory usage and optimize scheduling

---

## Appendix A: Hailo Custom Model Compilation Guide

This appendix provides a detailed reference for compiling your own custom models for the Hailo-10 NPU.

### A.1 Model Registration in Hailo Model Zoo

If you want to use the `hailomz` CLI tool with your custom model, register it:

```yaml
# model_zoo/hailo_model_zoo/cfg/networks/yolov5s_custom.yaml
network:
  network_name: yolov5s_custom
  acceleras:
    pre_quantization_optimization: true
    calibration_set_size: 64
    batch_size: 8
  paths:
    onnx_model_path: /path/to/best.onnx
    hef_model_path: /path/to/output.hef
  info:
    task: object_detection
    input_shape: "1x3x640x640"
    output_shape: "1x25200x9"  # 25200 = sum of all grid cells, 9 = 4+1+4classes
    operations: 16.5G
    parameters: 7.2M
    framework: onnx
    training_data: custom
    validation_data: custom
```

### A.2 Compiler Parameters

Fine-tune the compilation process:

```python
from hailo_sdk_client import ClientRunner

runner = ClientRunner(hw_arch="hailo10")

# Parse ONNX
hn, npz = runner.translate_onnx_model("best.onnx", ...)

# Set optimization parameters
runner.optimize(
    calib_dataset,
    data_type="np_array",
    # Key parameters:
    batch_size=8,              # Calibration batch size
    # Higher = more accurate range estimation but slower
)

# Compile with specific optimization level
hef = runner.compile(
    # Optimization level:
    #   0 = fast compile, lower performance
    #   1 = balanced (default)
    #   2 = maximum performance, slow compile
)
```

### A.3 Calibration Dataset Best Practices

| Aspect | Recommendation |
|--------|---------------|
| **Minimum size** | 100 images (200+ preferred) |
| **Source** | Real deployment environment (your track, your lighting) |
| **Diversity** | Include: day/night, shadows, wet/dry, all object classes |
| **Format** | Same preprocessing as training (resize, normalize) |
| **No labels needed** | Only forward pass — no ground truth required |
| **Storage** | NumPy array, shape: (N, C, H, W) for CHW or (N, H, W, C) for HWC |

### A.4 Profiling with hailortcli

After compilation, profile the model to verify performance:

```bash
# Basic inference benchmark
hailortcli run yolov5s_custom.hef

# Detailed profiling
hailortcli run yolov5s_custom.hef --measure-latency --measure-fps

# Expected output:
# ===================================
# Network: yolov5s_custom
# -----------------------------------
# FPS: 28.5
# Latency: 35.1 ms
# Power: 2.8 W
# ===================================

# Check model info
hailortcli parse-hef yolov5s_custom.hef
```

### A.5 Custom Preprocessing Callback

Register custom preprocessing to run on the host CPU before data is sent to the NPU:

```python
from hailo_platform import VDevice, InferVStreams

def custom_preprocess(frame):
    """Custom preprocessing that matches your training pipeline."""
    # Undistort (Day 11 calibration)
    frame = cv2.undistort(frame, K, dist)

    # Letterbox resize
    h, w = frame.shape[:2]
    scale = min(640 / h, 640 / w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(frame, (new_w, new_h))

    canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
    dw = (640 - new_w) // 2
    dh = (640 - new_h) // 2
    canvas[dh:dh + new_h, dw:dw + new_w] = resized

    return canvas, scale, dw, dh


# Use in inference loop
while True:
    ret, frame = cap.read()
    preprocessed, scale, dw, dh = custom_preprocess(frame)

    # Send to Hailo
    input_data = np.expand_dims(preprocessed, axis=0)
    # ... inference ...
```

### A.6 Troubleshooting Common Issues

| Issue | Cause | Solution |
|-------|-------|---------|
| "Unsupported layer" during compilation | Custom op not in Hailo support list | Replace with supported equivalent or use CPU fallback |
| Low accuracy after quantization | Poor calibration data or outlier activations | Increase calibration set, check data distribution |
| Lower FPS than expected | Large input size, complex postprocessing on CPU | Reduce input to 416x416, optimize postprocessing |
| "Failed to configure" | HEF compiled for wrong HW arch | Recompile with `--hw-arch hailo10` |
| Memory allocation failure | Model too large for on-chip SRAM | Use smaller model variant (yolov5n instead of yolov5s) |

---

## 14. Final Review — All 20 Days

Here is a summary of every skill you have built over this course:

| Day | Topic | Key Skill |
|-----|-------|-----------|
| 1 | Linux + RPi Setup | SSH, apt, systemd |
| 2 | Python + C Basics | Language fundamentals |
| 3 | GPIO + PWM | Hardware control |
| 4 | Serial Communication | UART, I2C, SPI |
| 5 | Sensor Integration | ADC, distance sensors |
| 6 | DC Motor Control | H-bridge, PWM speed control |
| 7 | Servo + Steering | Ackermann geometry |
| 8 | Encoder + Odometry | Wheel speed measurement |
| 9 | PID Control | Proportional-Integral-Derivative, Ziegler-Nichols |
| 10 | LiDAR + Depth Cameras | ToF, phase-shift, structured light |
| 11 | Camera Calibration | Intrinsics, distortion, BEV |
| 12 | SLAM | Visual odometry, RTAB-Map, loop closure |
| 13 | ROS2 Fundamentals | Nodes, topics, services, QoS |
| 14 | ROS2 TF + Executors | Coordinate transforms, callback groups |
| 15 | ros2_control + Nav2 | Hardware interface, navigation stack |
| 16 | Code Review | Team presentations, architecture analysis |
| **17** | **OpenCV + Lane Detection** | **Color spaces, Canny, Hough, BEV, sliding window** |
| **18** | **ROS2 Integration + Safety** | **Fusion, watchdog, state machine** |
| **19** | **YOLOv5 + Transfer Learning** | **Object detection, quantization** |
| **20** | **Hailo NPU + Final Demo** | **Edge AI deployment, system integration** |

### What You Can Do Now

You can take a Raspberry Pi, connect a camera, LiDAR, and motors, and build an autonomous car that:

1. **Sees** — Camera captures road scenes.
2. **Understands lanes** — OpenCV pipeline detects lane boundaries.
3. **Recognizes objects** — YOLOv5 on Hailo identifies traffic signs and obstacles.
4. **Measures distance** — LiDAR provides obstacle range.
5. **Maps the environment** — RTAB-Map builds a real-time occupancy grid.
6. **Steers** — PID controller follows the lane center.
7. **Stays safe** — State machine degrades gracefully on sensor failure.
8. **Runs in real-time** — Hailo NPU enables 25+ FPS detection.

This is the foundation. The advanced roadmap (Section 13) takes you from model car to production autonomous vehicle engineering.

---

**Congratulations on completing the Embedded Basics for Autonomous Car series.** The skills you have built — embedded systems, computer vision, deep learning, ROS2, sensor fusion, safety engineering, and edge AI deployment — form the core of modern autonomous vehicle development. Keep building.
