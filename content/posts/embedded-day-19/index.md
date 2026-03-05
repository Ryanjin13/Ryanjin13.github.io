---
title: "Day 19 — YOLOv5 Object Detection, Transfer Learning, and Quantization"
date: 2026-03-24
description: "YOLOv5 architecture deep dive, YOLO metrics, transfer learning strategies, custom dataset training, and INT8 quantization for edge deployment"
categories: ["Autonomous Driving"]
tags: ["YOLOv5", "Object Detection", "Transfer Learning", "Quantization", "Edge AI"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 19
draft: false
---

{{< katex >}}

## What You'll Learn

On Day 17 you built a lane detection pipeline. On Day 18 you integrated it into ROS2 with sensor fusion and safety. But lane detection alone cannot tell your car **what** is on the road — only that something is there (via LiDAR distance). Today we add **semantic understanding**: the ability to recognize traffic signs, pedestrians, other vehicles, and obstacles by name and location.

This is a big day with three deep sections:

1. **Section 1 — YOLOv5 Architecture + Metrics:** How YOLO works inside, and how to measure its performance rigorously.
2. **Section 2 — Transfer Learning:** How to train YOLOv5 on your own custom dataset with limited data.
3. **Section 3 — Quantization:** How to shrink the model from FP32 to INT8 for real-time inference on edge devices.

By the end you will be able to:

- Explain the CSPNet backbone, PANet neck, and detection head of YOLOv5.
- Calculate Precision, Recall, IoU, mAP@0.5, and mAP@0.5:0.95 by hand.
- Train YOLOv5 on a custom dataset using transfer learning with frozen backbone.
- Apply post-training quantization (PTQ) and measure the accuracy tradeoff.
- Export to ONNX and run inference with OpenCV DNN.

---

## Section 1: YOLOv5 Architecture and Metrics

### 1.1 YOLO — A Brief History

**YOLO** stands for **You Only Look Once**. Unlike two-stage detectors (R-CNN family) that first propose regions and then classify them, YOLO treats object detection as a **single regression problem** — one forward pass through the network outputs all bounding boxes and class probabilities simultaneously.

| Version | Year | Key Innovation |
|---------|------|---------------|
| YOLOv1 | 2016 | Single-shot detection concept |
| YOLOv2 | 2017 | Batch normalization, anchor boxes |
| YOLOv3 | 2018 | Multi-scale detection, Darknet-53 backbone |
| YOLOv4 | 2020 | CSPDarknet, Mish activation, mosaic augmentation |
| **YOLOv5** | **2020** | **PyTorch native, ultralytics, production-ready** |
| YOLOv8 | 2023 | Anchor-free, decoupled head |

We use **YOLOv5** because it has the best balance of documentation, community support, and deployment tooling (especially for Hailo compilation on Day 20).

### 1.2 YOLOv5 Architecture Overview

YOLOv5 has three major components:

```
Input Image (640x640x3)
        │
        ▼
┌────────────────────┐
│    BACKBONE         │    Feature extraction
│    (CSPDarknet53)   │    Learns "what things look like"
└────────┬───────────┘
         │
         ▼
┌────────────────────┐
│    NECK             │    Feature aggregation
│    (PANet + SPP)    │    Combines features at multiple scales
└────────┬───────────┘
         │
         ▼
┌────────────────────┐
│    HEAD             │    Detection output
│    (Detect Layer)   │    Bounding boxes + classes + confidence
└────────────────────┘
         │
         ▼
  3 scale outputs:
  - 80×80 (small objects)
  - 40×40 (medium objects)
  - 20×20 (large objects)
```

#### 1.2.1 Backbone: CSPDarknet53

**CSP** stands for **Cross Stage Partial**. The key idea: split the input feature map into two halves. One half goes through a dense block of convolutional layers; the other half bypasses them. Then concatenate.

**Why?** This reduces computation by ~50% compared to a standard DenseNet block while preserving gradient flow. It prevents the gradient from becoming too diluted across many layers.

Each backbone stage consists of:

- **CBS** (Conv + BatchNorm + SiLU activation): The basic building block.
- **C3 module**: A CSP bottleneck with 3 convolutions. Contains \(n\) bottleneck layers internally.
- **SPPF** (Spatial Pyramid Pooling - Fast): At the end of the backbone, applies max pooling at multiple scales (5x5, 9x9, 13x13) to capture multi-scale context. The "Fast" variant chains three 5x5 pooling operations instead of using three different kernel sizes.

```
CSPDarknet53 (simplified):

Input → CBS(3→32) → CBS(32→64) → C3(64, n=1) →
        CBS(64→128) → C3(128, n=2) →
        CBS(128→256) → C3(256, n=3) → [P3: 80×80×256]
        CBS(256→512) → C3(512, n=3) → [P4: 40×40×512]
        CBS(512→1024) → C3(1024, n=1) → SPPF → [P5: 20×20×1024]
```

#### 1.2.2 Neck: PANet + Feature Pyramid

The backbone extracts features at three scales (P3, P4, P5). Small objects are better detected at high resolution (P3); large objects at low resolution (P5). The **Path Aggregation Network (PANet)** creates a bidirectional feature pyramid:

```
P5 (20×20) ──Upsample──► Concat with P4 → C3 → N4 (40×40)
                                                    │
N4 (40×40) ──Upsample──► Concat with P3 → C3 → N3 (80×80)
                                                    │
N3 (80×80) ──Downsample─► Concat with N4 → C3 → N4' (40×40)
                                                    │
N4' (40×40) ──Downsample─► Concat with P5 → C3 → N5' (20×20)
```

**Top-down path** (FPN): P5 → P4 → P3. Propagates semantic (high-level) information to high-resolution layers.

**Bottom-up path** (PAN): P3 → P4 → P5. Propagates localization (low-level) information to low-resolution layers.

The result: every scale has access to both fine-grained spatial detail and high-level semantic context.

#### 1.2.3 Detection Head

The detection head applies a 1x1 convolution to each of the three scale feature maps, producing tensors of shape:

$$
\text{output}_{s} = B \times (5 + C) \times H_s \times W_s
$$

where:
- \(B = 3\) (number of anchor boxes per grid cell)
- \(5 = [t_x, t_y, t_w, t_h, \text{objectness}]\) (bounding box parameters + confidence)
- \(C\) = number of classes
- \(H_s \times W_s\) = grid resolution at scale \(s\)

**Anchor boxes** are predefined aspect ratios learned from the training dataset via k-means clustering. Each scale uses 3 anchors of different sizes:

| Scale | Grid | Anchor Sizes (typical for COCO) |
|-------|------|---------------------------------|
| P3 (80x80) | Small objects | (10,13), (16,30), (33,23) |
| P4 (40x40) | Medium objects | (30,61), (62,45), (59,119) |
| P5 (20x20) | Large objects | (116,90), (156,198), (373,326) |

**Bounding box decoding:**

The network predicts offsets \((t_x, t_y, t_w, t_h)\) relative to the grid cell and anchor:

$$
b_x = \sigma(t_x) + c_x, \qquad b_y = \sigma(t_y) + c_y
$$

$$
b_w = a_w \cdot e^{t_w}, \qquad b_h = a_h \cdot e^{t_h}
$$

where:
- \(\sigma\) = sigmoid function
- \((c_x, c_y)\) = grid cell top-left corner
- \((a_w, a_h)\) = anchor width and height

**Non-Maximum Suppression (NMS):** After decoding, many overlapping boxes may detect the same object. NMS keeps only the highest-confidence box for each object:

1. Sort all detections by confidence (descending).
2. Take the top detection. Mark it as kept.
3. Remove all other detections that overlap with it (IoU > threshold, typically 0.45).
4. Repeat until no detections remain.

### 1.3 YOLOv5 Metrics — All the Formulas

Evaluating an object detector requires understanding several interconnected metrics.

#### 1.3.1 Intersection over Union (IoU)

$$
\text{IoU} = \frac{\text{Area}(A \cap B)}{\text{Area}(A \cup B)} = \frac{\text{Area of Overlap}}{\text{Area of Union}}
$$

IoU measures how well a predicted box \(A\) matches a ground truth box \(B\). An IoU threshold (typically 0.5) determines whether a prediction is considered correct.

```python
def compute_iou(box1, box2):
    """Compute IoU between two boxes [x1, y1, x2, y2]."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - intersection

    return intersection / union if union > 0 else 0.0
```

#### 1.3.2 Precision and Recall

For a given IoU threshold:

| | Predicted Positive | Predicted Negative |
|---|---|---|
| **Actually Positive** | True Positive (TP) | False Negative (FN) |
| **Actually Negative** | False Positive (FP) | True Negative (TN) |

- **TP**: Predicted box matches a ground truth box (IoU >= threshold) with correct class.
- **FP**: Predicted box has no matching ground truth (IoU < threshold or wrong class).
- **FN**: Ground truth box has no matching prediction.

$$
\text{Precision} = \frac{TP}{TP + FP} = \frac{\text{correct detections}}{\text{all detections}}
$$

$$
\text{Recall} = \frac{TP}{TP + FN} = \frac{\text{correct detections}}{\text{all ground truths}}
$$

**Intuition:**
- **High Precision**: When the model says "there's a stop sign," it's usually right. Few false alarms.
- **High Recall**: The model finds most of the stop signs. Few misses.
- There is a **tradeoff**: lowering the confidence threshold increases recall but decreases precision.

#### 1.3.3 Precision-Recall (PR) Curve

By varying the confidence threshold from 1.0 down to 0.0, you get a series of (Precision, Recall) points. Plotting these gives the **PR curve**.

A perfect detector has a PR curve that goes through the point (1.0, 1.0) — perfect precision at perfect recall.

#### 1.3.4 Average Precision (AP)

$$
\text{AP} = \int_0^1 P(r) \, dr
$$

In practice, this integral is approximated by the **area under the PR curve** using the 101-point interpolation method (COCO style) or the all-point interpolation.

**AP@0.5** = Average Precision computed at IoU threshold 0.5. This is the most common single metric.

#### 1.3.5 mAP (mean Average Precision)

$$
\text{mAP} = \frac{1}{N_{\text{classes}}} \sum_{c=1}^{N_{\text{classes}}} \text{AP}_c
$$

**mAP@0.5** = mean AP across all classes at IoU = 0.5.

**mAP@0.5:0.95** = The COCO metric. Average mAP over 10 IoU thresholds: 0.5, 0.55, 0.60, ..., 0.95:

$$
\text{mAP@0.5:0.95} = \frac{1}{10} \sum_{t \in \{0.50, 0.55, \ldots, 0.95\}} \text{mAP}@t
$$

This is a **much stricter** metric because high IoU thresholds demand very precise bounding boxes.

#### 1.3.6 F1 Score

$$
F_1 = 2 \cdot \frac{\text{Precision} \times \text{Recall}}{\text{Precision} + \text{Recall}}
$$

The F1 score is the harmonic mean of precision and recall. The **F1 curve** plots F1 vs. confidence threshold; the peak gives the optimal confidence threshold for balanced precision and recall.

#### 1.3.7 Confusion Matrix

A confusion matrix for object detection shows, for each true class, how often the model predicted each class (or missed the object). It helps identify:
- Which classes the model confuses with each other.
- Which classes have high miss rates.

### 1.4 Understanding results.csv

YOLOv5 writes a `results.csv` file during training with these columns:

| Column | Meaning | What to Watch |
|--------|---------|--------------|
| `train/box_loss` | Bounding box regression loss | Should decrease steadily |
| `train/obj_loss` | Objectness loss (is there an object?) | Should decrease |
| `train/cls_loss` | Classification loss (which class?) | Should decrease |
| `val/box_loss` | Validation box loss | Should decrease; if it increases while train decreases → **overfitting** |
| `val/obj_loss` | Validation objectness loss | Same |
| `val/cls_loss` | Validation classification loss | Same |
| `metrics/precision` | Validation precision | Should increase → plateau |
| `metrics/recall` | Validation recall | Should increase → plateau |
| `metrics/mAP_0.5` | Validation mAP@0.5 | **Primary metric** — should increase |
| `metrics/mAP_0.5:0.95` | Validation mAP@0.5:0.95 | Stricter metric — increases slower |

**Reading the curves:**

```
Healthy training:
  train_loss ↓  val_loss ↓  mAP ↑  → Keep going

Overfitting:
  train_loss ↓  val_loss ↑  mAP plateau/↓  → Stop or add augmentation

Underfitting:
  train_loss high  val_loss high  mAP low  → More epochs, unfreeze layers

Learning rate too high:
  Losses oscillate wildly  → Reduce lr0
```

---

## Section 2: Transfer Learning

### 2.1 Why Transfer Learning?

Training a neural network from scratch requires:
- **Millions** of labeled images
- **Days to weeks** of GPU time
- **Expert** hyperparameter tuning

Transfer learning sidesteps this by starting from a model **pretrained on a large dataset** (like COCO, with 330K images and 80 classes). The pretrained weights already encode general visual features — edges, textures, shapes, parts of objects. You only need to adapt the final layers to your specific classes.

**Analogy:** It is like hiring a professional photographer who already knows how to see light, composition, and focus. You just need to teach them what your specific subjects look like — much faster than training someone from zero.

### 2.2 Freeze Strategy

Not all layers need to be retrained. YOLOv5 supports **freezing** layers:

```
Backbone (layers 0–9):  General visual features
                        → Freeze these for small datasets
Neck (layers 10–17):    Feature aggregation
                        → Can freeze or fine-tune
Head (layers 18–23):    Detection output
                        → Always train these
```

**Strategy by dataset size:**

| Dataset Size | Strategy | YOLOv5 Command |
|-------------|----------|---------------|
| < 100 images | Freeze backbone + neck (layers 0–17), train head only | `--freeze 17` |
| 100–1000 images | Freeze backbone (layers 0–9), train neck + head | `--freeze 10` |
| > 1000 images | Train everything (no freezing) | (default) |
| > 5000 images | Train everything with longer schedule | `--epochs 100` |

**Gradual unfreezing:** Start fully frozen, train 10 epochs, then unfreeze backbone and train 40 more epochs at a lower learning rate. This prevents the pretrained weights from being destroyed by large early gradients.

### 2.3 Custom Dataset Preparation

#### YOLO Label Format

Each image gets a `.txt` label file with the same name. Each line in the file represents one object:

```
<class_id> <x_center> <y_center> <width> <height>
```

All values are **normalized** to [0, 1] relative to image dimensions.

**Example:** An image `frame_001.jpg` (640x480) with a stop sign at pixel coordinates (200, 150) to (350, 320):

```
# frame_001.txt
0 0.4297 0.4896 0.2344 0.3542
```

where:
- `0` = class ID for "stop_sign"
- `x_center = (200 + 350) / 2 / 640 = 0.4297`
- `y_center = (150 + 320) / 2 / 480 = 0.4896`
- `width = (350 - 200) / 640 = 0.2344`
- `height = (320 - 150) / 480 = 0.3542`

#### Dataset Directory Structure

```
custom_dataset/
├── images/
│   ├── train/
│   │   ├── frame_001.jpg
│   │   ├── frame_002.jpg
│   │   └── ...
│   └── val/
│       ├── frame_100.jpg
│       └── ...
├── labels/
│   ├── train/
│   │   ├── frame_001.txt
│   │   ├── frame_002.txt
│   │   └── ...
│   └── val/
│       ├── frame_100.txt
│       └── ...
└── data.yaml
```

#### data.yaml

```yaml
# data.yaml — Custom dataset configuration
path: /home/user/custom_dataset
train: images/train
val: images/val

nc: 4  # number of classes
names:
  0: stop_sign
  1: speed_limit
  2: pedestrian
  3: traffic_cone
```

#### Labeling Tools

- **labelImg**: Simple, local, free. Install: `pip install labelImg`. Switch to YOLO format before labeling.
- **Roboflow**: Web-based, supports team collaboration, auto-augmentation, export in multiple formats.
- **CVAT**: Open-source, feature-rich, supports video annotation.

**Tip:** Aim for at least **50 images per class** for transfer learning to work well. More is always better. Ensure diverse lighting, angles, and backgrounds.

### 2.4 Training with Transfer Learning

```bash
# Clone YOLOv5
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt

# Train with frozen backbone (10 layers)
python train.py \
    --weights yolov5s.pt \
    --data /path/to/custom_dataset/data.yaml \
    --img 640 \
    --batch 16 \
    --epochs 50 \
    --freeze 10 \
    --name custom_model_v1 \
    --patience 10
```

**Key arguments explained:**

| Argument | Meaning |
|----------|---------|
| `--weights yolov5s.pt` | Start from pretrained YOLOv5-small (7.2M params) |
| `--data data.yaml` | Dataset configuration file |
| `--img 640` | Input image size (square) |
| `--batch 16` | Batch size (reduce if OOM) |
| `--epochs 50` | Maximum training epochs |
| `--freeze 10` | Freeze first 10 layers (backbone) |
| `--name custom_model_v1` | Experiment name (saved in `runs/train/`) |
| `--patience 10` | Early stopping: stop if mAP doesn't improve for 10 epochs |

### 2.5 Overfitting Prevention

With small datasets, overfitting is the primary risk. Here are the defenses:

#### Data Augmentation

YOLOv5 applies these augmentations by default:

- **Mosaic**: Combines 4 training images into one. Forces the model to learn objects at different scales and in varied contexts. Activated for the first 90% of training.
- **MixUp**: Blends two images and their labels with a random weight. Regularizes the model.
- **HSV augmentation**: Randomly shifts hue, saturation, and value. Makes the model robust to lighting changes.
- **Random flip, rotate, scale, translate**: Geometric augmentations.

Configure in the `hyp.scratch-low.yaml` hyperparameter file:

```yaml
# Key augmentation hyperparameters
hsv_h: 0.015       # hue shift
hsv_s: 0.7         # saturation shift
hsv_v: 0.4         # value shift
degrees: 0.0       # rotation range
translate: 0.1     # translation fraction
scale: 0.5         # scale range
shear: 0.0         # shear
perspective: 0.0   # perspective distortion
flipud: 0.0        # vertical flip probability
fliplr: 0.5        # horizontal flip probability
mosaic: 1.0        # mosaic augmentation probability
mixup: 0.0         # mixup probability (set > 0 for small datasets)
```

#### Early Stopping

The `--patience` flag stops training when validation mAP stops improving. This prevents the model from memorizing training data after the useful learning phase.

#### Dropout

YOLOv5 does not use traditional dropout in the convolutional layers (batch normalization serves a similar purpose). However, the augmentation pipeline acts as a strong implicit regularizer.

#### Weight Decay

L2 regularization is applied via the optimizer. Default: `weight_decay=0.0005`. This penalizes large weights:

$$
\mathcal{L}_{\text{total}} = \mathcal{L}_{\text{detection}} + \lambda \sum_{i} w_i^2
$$

### 2.6 Analyzing Training Results

After training, results are saved in `runs/train/custom_model_v1/`:

```
runs/train/custom_model_v1/
├── weights/
│   ├── best.pt       ← Best mAP checkpoint
│   └── last.pt       ← Last epoch checkpoint
├── results.csv        ← Training curves data
├── results.png        ← Training curves plot
├── confusion_matrix.png
├── F1_curve.png
├── PR_curve.png
├── P_curve.png
├── R_curve.png
└── val_batch0_pred.jpg  ← Sample predictions on validation set
```

```python
"""Plot training results from results.csv."""
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("runs/train/custom_model_v1/results.csv",
                  skipinitialspace=True)

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Loss curves
axes[0, 0].plot(df["epoch"], df["train/box_loss"], label="Train")
axes[0, 0].plot(df["epoch"], df["val/box_loss"], label="Val")
axes[0, 0].set_title("Box Loss")
axes[0, 0].legend()

axes[0, 1].plot(df["epoch"], df["train/obj_loss"], label="Train")
axes[0, 1].plot(df["epoch"], df["val/obj_loss"], label="Val")
axes[0, 1].set_title("Objectness Loss")
axes[0, 1].legend()

# mAP curves
axes[1, 0].plot(df["epoch"], df["metrics/mAP_0.5"], label="mAP@0.5")
axes[1, 0].plot(df["epoch"], df["metrics/mAP_0.5:0.95"], label="mAP@0.5:0.95")
axes[1, 0].set_title("mAP")
axes[1, 0].legend()

# Precision / Recall
axes[1, 1].plot(df["epoch"], df["metrics/precision"], label="Precision")
axes[1, 1].plot(df["epoch"], df["metrics/recall"], label="Recall")
axes[1, 1].set_title("Precision & Recall")
axes[1, 1].legend()

plt.tight_layout()
plt.savefig("training_analysis.png", dpi=150)
plt.show()
```

---

## Section 3: Quantization

### 3.1 The Edge Deployment Problem

Your trained YOLOv5s model has ~7.2 million parameters, each stored as a **32-bit floating point** (FP32) number:

$$
\text{Model size} = 7.2 \times 10^6 \times 4 \text{ bytes} = 28.8 \text{ MB}
$$

On a Raspberry Pi 5 CPU, this achieves maybe **1–3 FPS**. For real-time autonomous driving, we need **15–30 FPS**. Two paths to get there:

1. **Hardware acceleration** (Hailo NPU — Day 20)
2. **Model compression** (quantization — today)

These are complementary: the Hailo NPU runs **INT8** models, so quantization is not optional — it is required for deployment.

### 3.2 What Is Quantization?

Quantization maps floating-point values to lower-precision integers:

| Format | Bits per Param | Range | Model Size (7.2M params) |
|--------|---------------|-------|--------------------------|
| FP32 | 32 | \(\pm 3.4 \times 10^{38}\) | 28.8 MB |
| FP16 | 16 | \(\pm 6.5 \times 10^{4}\) | 14.4 MB |
| INT8 | 8 | \(-128\) to \(127\) | 7.2 MB |
| INT4 | 4 | \(-8\) to \(7\) | 3.6 MB |

**INT8 quantization gives a 4x reduction** in model size and (on hardware that supports it) a 2–4x speedup in inference.

### 3.3 The Math of Quantization

#### Linear (Affine) Quantization

To map a floating-point range \([x_{\min}, x_{\max}]\) to an integer range \([0, 2^n - 1]\) (for unsigned) or \([-2^{n-1}, 2^{n-1}-1]\) (for signed):

**Scale factor:**

$$
s = \frac{x_{\max} - x_{\min}}{2^n - 1}
$$

**Zero point** (the integer value that represents floating-point 0):

$$
z = \text{round}\left(-\frac{x_{\min}}{s}\right)
$$

**Quantize (float → int):**

$$
q = \text{round}\left(\frac{x}{s}\right) + z = \text{clamp}\left(\text{round}\left(\frac{x}{s} + z\right), 0, 2^n - 1\right)
$$

**Dequantize (int → float):**

$$
\hat{x} = s \cdot (q - z)
$$

#### Worked Example

Suppose a weight tensor has values in \([-0.5, 1.2]\) and we want INT8 (unsigned, 0–255):

$$
s = \frac{1.2 - (-0.5)}{255} = \frac{1.7}{255} \approx 0.00667
$$

$$
z = \text{round}\left(-\frac{-0.5}{0.00667}\right) = \text{round}(74.96) = 75
$$

To quantize \(x = 0.3\):

$$
q = \text{round}\left(\frac{0.3}{0.00667} + 75\right) = \text{round}(44.98 + 75) = 120
$$

To dequantize back:

$$
\hat{x} = 0.00667 \times (120 - 75) = 0.00667 \times 45 = 0.300
$$

The error is \(|0.3 - 0.300| = 0.0\) in this case, but in general there is a small **quantization error** bounded by \(s/2\).

### 3.4 Post-Training Quantization (PTQ)

PTQ applies quantization **after** the model is fully trained. No retraining required. The process:

1. **Train** the model normally in FP32.
2. **Calibrate**: Run a small representative dataset (100–500 images) through the model to determine the range \([x_{\min}, x_{\max}]\) for each layer's activations.
3. **Quantize**: Compute scale and zero point for each layer and convert weights + activations to INT8.

```python
"""
Post-Training Quantization with PyTorch (simplified demonstration).
For actual deployment, the Hailo compiler performs PTQ automatically.
"""

import torch
from torch.quantization import quantize_dynamic, quantize_static

# Method 1: Dynamic Quantization (quantizes weights, activations at runtime)
# Simplest, but less speedup
model_fp32 = torch.load("runs/train/custom_model_v1/weights/best.pt")["model"]
model_int8_dynamic = quantize_dynamic(
    model_fp32.float(),
    {torch.nn.Linear},  # Conv2d not supported for dynamic quantization
    dtype=torch.qint8
)

# Method 2: Static Quantization (quantizes both weights and activations)
# Better performance, requires calibration
model_fp32.eval()
model_fp32.qconfig = torch.quantization.get_default_qconfig("fbgemm")
model_prepared = torch.quantization.prepare(model_fp32)

# Calibrate with representative data
calibration_dataloader = ...  # 100-500 images from your dataset
with torch.no_grad():
    for images, _ in calibration_dataloader:
        model_prepared(images)

model_int8_static = torch.quantization.convert(model_prepared)
```

**Calibration data requirements:**
- **Minimum 100 images** from the actual deployment environment.
- Should include diverse lighting, angles, and object positions.
- Does NOT need labels — only forward pass is needed.

### 3.5 Quantization-Aware Training (QAT)

QAT inserts **fake quantization** nodes during training. The forward pass simulates INT8 arithmetic; the backward pass uses full FP32 gradients (via the **Straight-Through Estimator** — STE).

```
Forward pass:
  x_fp32 → Quantize → Dequantize → x_approx_fp32 → Convolution → ...
                     ↑
              Simulates INT8 rounding error

Backward pass:
  Gradients flow through as if Quantize/Dequantize were identity functions
  (Straight-Through Estimator)
```

**Why QAT?** The model **learns to compensate** for quantization error during training. The result is typically 1–2% better mAP than PTQ.

**When to use QAT vs PTQ:**

| Criterion | PTQ | QAT |
|-----------|-----|-----|
| mAP drop | 1–5% | 0–2% |
| Extra training needed | No | Yes (10–30 extra epochs) |
| Complexity | Low | Medium |
| When to use | mAP drop acceptable | Every percent matters |

For our project, **PTQ is sufficient** because the Hailo compiler applies it automatically during the `.hef` compilation step (Day 20).

### 3.6 Accuracy Tradeoff: Quantitative Evaluation

Always measure the impact of quantization:

```python
"""
Compare FP32 vs INT8 model accuracy.
Run YOLOv5 validation on the same dataset with both models.
"""

# FP32 baseline
# python val.py --weights best.pt --data data.yaml --img 640

# INT8 (after ONNX export + quantization)
# python val.py --weights best_int8.onnx --data data.yaml --img 640
```

**Expected results (typical):**

| Model | mAP@0.5 | mAP@0.5:0.95 | Size (MB) | RPi5 CPU FPS |
|-------|---------|---------------|-----------|-------------|
| YOLOv5s FP32 | 0.85 | 0.62 | 28.8 | ~2 |
| YOLOv5s FP16 | 0.85 | 0.62 | 14.4 | ~3 |
| YOLOv5s INT8 (PTQ) | 0.83 | 0.59 | 7.2 | ~5 |
| YOLOv5s INT8 (Hailo) | 0.82 | 0.58 | ~4 (.hef) | ~25 (NPU) |

The 2–3% mAP drop from INT8 is acceptable for our application. The 10x+ FPS improvement on the Hailo NPU makes it worthwhile.

### 3.7 Connection to Hailo (Day 20)

The Hailo Dataflow Compiler converts ONNX models to `.hef` (Hailo Executable Format). This process **includes INT8 PTQ automatically**:

```
PyTorch (.pt) → ONNX (.onnx) → Hailo Compiler → .hef (INT8)
                                    ↑
                         Calibration images needed
```

Today we prepare the ONNX export. Tomorrow we complete the Hailo compilation pipeline.

---

## 4. Hands-On Lab

### 4.1 Training a Custom YOLOv5 Model

```bash
# Step 1: Clone and install
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt

# Step 2: Prepare dataset (assuming labeled with labelImg)
# Verify structure:
ls /path/to/custom_dataset/images/train/ | head
ls /path/to/custom_dataset/labels/train/ | head
cat /path/to/custom_dataset/data.yaml

# Step 3: Train with frozen backbone
python train.py \
    --weights yolov5s.pt \
    --data /path/to/custom_dataset/data.yaml \
    --img 640 \
    --batch 16 \
    --epochs 50 \
    --freeze 10 \
    --name track_signs_v1 \
    --patience 10

# Step 4: Validate
python val.py \
    --weights runs/train/track_signs_v1/weights/best.pt \
    --data /path/to/custom_dataset/data.yaml \
    --img 640 \
    --verbose

# Step 5: Test on images
python detect.py \
    --weights runs/train/track_signs_v1/weights/best.pt \
    --source /path/to/test_images/ \
    --img 640 \
    --conf-thres 0.5
```

### 4.2 Analyzing Training Results

```python
"""
Lab: Analyze training results and generate report.
"""

import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def analyze_training(run_dir):
    """Complete analysis of a YOLOv5 training run."""
    run_path = Path(run_dir)

    # Read results
    df = pd.read_csv(run_path / "results.csv", skipinitialspace=True)

    # Best epoch
    best_epoch = df["metrics/mAP_0.5"].idxmax()
    best_map50 = df.loc[best_epoch, "metrics/mAP_0.5"]
    best_map5095 = df.loc[best_epoch, "metrics/mAP_0.5:0.95"]

    print(f"Best epoch: {best_epoch}")
    print(f"  mAP@0.5:     {best_map50:.4f}")
    print(f"  mAP@0.5:0.95: {best_map5095:.4f}")
    print(f"  Precision:    {df.loc[best_epoch, 'metrics/precision']:.4f}")
    print(f"  Recall:       {df.loc[best_epoch, 'metrics/recall']:.4f}")

    # Check for overfitting
    final_train_loss = df["train/box_loss"].iloc[-1]
    final_val_loss = df["val/box_loss"].iloc[-1]
    min_val_loss = df["val/box_loss"].min()
    min_val_epoch = df["val/box_loss"].idxmin()

    if df["val/box_loss"].iloc[-1] > min_val_loss * 1.1:
        print(f"\n[WARNING] Possible overfitting detected!")
        print(f"  Min val loss at epoch {min_val_epoch}: {min_val_loss:.4f}")
        print(f"  Final val loss: {final_val_loss:.4f}")
    else:
        print(f"\nNo overfitting detected. Training looks healthy.")

    return df


# Run analysis
df = analyze_training("runs/train/track_signs_v1")
```

### 4.3 ONNX Export

```bash
# Export to ONNX for deployment
python export.py \
    --weights runs/train/track_signs_v1/weights/best.pt \
    --img 640 \
    --batch 1 \
    --include onnx \
    --simplify
```

This produces `best.onnx` — a framework-independent model that can be loaded by OpenCV DNN, ONNX Runtime, TensorRT, or the Hailo compiler.

### 4.4 OpenCV DNN Inference + FPS Measurement

```python
"""
Lab: Run YOLOv5 inference using OpenCV DNN backend.
Measures FPS on CPU for baseline comparison.
"""

import cv2
import numpy as np
import time


class YOLOv5OpenCV:
    """YOLOv5 inference using OpenCV DNN module."""

    def __init__(self, onnx_path, conf_thresh=0.5, iou_thresh=0.45,
                 input_size=640):
        self.net = cv2.dnn.readNetFromONNX(onnx_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.input_size = input_size

    def preprocess(self, frame):
        """Letterbox resize + normalize."""
        h, w = frame.shape[:2]
        scale = min(self.input_size / h, self.input_size / w)
        new_w, new_h = int(w * scale), int(h * scale)

        resized = cv2.resize(frame, (new_w, new_h))

        # Pad to square
        canvas = np.full((self.input_size, self.input_size, 3), 114,
                         dtype=np.uint8)
        dw = (self.input_size - new_w) // 2
        dh = (self.input_size - new_h) // 2
        canvas[dh:dh + new_h, dw:dw + new_w] = resized

        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0,
                                      (self.input_size, self.input_size),
                                      swapRB=True, crop=False)
        return blob, scale, dw, dh

    def postprocess(self, output, scale, dw, dh, orig_h, orig_w):
        """Extract detections from network output."""
        # output shape: [1, num_detections, 5 + num_classes]
        detections = output[0]

        boxes = []
        confidences = []
        class_ids = []

        for det in detections:
            scores = det[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id] * det[4]  # class_score * objectness

            if confidence < self.conf_thresh:
                continue

            # Center format → corner format, undo letterbox
            cx, cy, bw, bh = det[0], det[1], det[2], det[3]
            x1 = int((cx - bw / 2 - dw) / scale)
            y1 = int((cy - bh / 2 - dh) / scale)
            x2 = int((cx + bw / 2 - dw) / scale)
            y2 = int((cy + bh / 2 - dh) / scale)

            # Clamp to image bounds
            x1 = max(0, min(x1, orig_w))
            y1 = max(0, min(y1, orig_h))
            x2 = max(0, min(x2, orig_w))
            y2 = max(0, min(y2, orig_h))

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            confidences.append(float(confidence))
            class_ids.append(class_id)

        # NMS
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh,
                                    self.iou_thresh)

        results = []
        for i in indices:
            idx = i if isinstance(i, int) else i[0]
            results.append({
                "box": boxes[idx],
                "confidence": confidences[idx],
                "class_id": class_ids[idx],
            })

        return results

    def detect(self, frame):
        """Run full detection pipeline on one frame."""
        h, w = frame.shape[:2]
        blob, scale, dw, dh = self.preprocess(frame)

        self.net.setInput(blob)
        output = self.net.forward()

        return self.postprocess(output, scale, dw, dh, h, w)


def benchmark_fps(detector, source, n_frames=100):
    """Measure average FPS over n_frames."""
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open {source}")
        return

    times = []
    for i in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()

        t_start = time.perf_counter()
        results = detector.detect(frame)
        t_end = time.perf_counter()

        times.append(t_end - t_start)

        if (i + 1) % 10 == 0:
            avg_ms = np.mean(times[-10:]) * 1000
            fps = 1000 / avg_ms
            print(f"Frame {i+1}/{n_frames}: {avg_ms:.1f} ms ({fps:.1f} FPS), "
                  f"{len(results)} detections")

    cap.release()

    avg_time = np.mean(times)
    avg_fps = 1.0 / avg_time

    print(f"\n--- Benchmark Results ---")
    print(f"Average inference time: {avg_time * 1000:.1f} ms")
    print(f"Average FPS: {avg_fps:.1f}")
    print(f"Min time: {min(times) * 1000:.1f} ms")
    print(f"Max time: {max(times) * 1000:.1f} ms")

    return avg_fps


if __name__ == "__main__":
    CLASS_NAMES = ["stop_sign", "speed_limit", "pedestrian", "traffic_cone"]

    detector = YOLOv5OpenCV(
        onnx_path="runs/train/track_signs_v1/weights/best.onnx",
        conf_thresh=0.5,
        iou_thresh=0.45,
        input_size=640,
    )

    # Benchmark
    fps = benchmark_fps(detector, "test_video.mp4", n_frames=100)

    # Visual test
    cap = cv2.VideoCapture("test_video.mp4")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = detector.detect(frame)

        for r in results:
            x, y, w, h = r["box"]
            label = f"{CLASS_NAMES[r['class_id']]} {r['confidence']:.2f}"
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLOv5 Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
```

### 4.5 PTQ Before/After Comparison Script

```python
"""
Lab: Compare FP32 vs INT8 quantized model.
Uses ONNX Runtime for fair comparison on CPU.
"""

import onnxruntime as ort
from onnxruntime.quantization import quantize_dynamic, QuantType
import numpy as np
import time


def quantize_onnx_model(input_path, output_path):
    """Apply dynamic INT8 quantization to ONNX model."""
    quantize_dynamic(
        input_path,
        output_path,
        weight_type=QuantType.QInt8,
    )
    print(f"Quantized model saved to {output_path}")


def benchmark_onnx(model_path, input_shape=(1, 3, 640, 640), n_runs=50):
    """Benchmark ONNX model inference time."""
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    # Warm up
    dummy = np.random.randn(*input_shape).astype(np.float32)
    for _ in range(5):
        session.run(None, {input_name: dummy})

    # Benchmark
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        session.run(None, {input_name: dummy})
        t1 = time.perf_counter()
        times.append(t1 - t0)

    avg_ms = np.mean(times) * 1000
    fps = 1000 / avg_ms
    return avg_ms, fps


def main():
    fp32_path = "runs/train/track_signs_v1/weights/best.onnx"
    int8_path = "runs/train/track_signs_v1/weights/best_int8.onnx"

    # Quantize
    quantize_onnx_model(fp32_path, int8_path)

    # Benchmark both
    fp32_ms, fp32_fps = benchmark_onnx(fp32_path)
    int8_ms, int8_fps = benchmark_onnx(int8_path)

    # Report
    import os
    fp32_size = os.path.getsize(fp32_path) / 1e6
    int8_size = os.path.getsize(int8_path) / 1e6

    print(f"\n{'='*50}")
    print(f"{'Metric':<25} {'FP32':>10} {'INT8':>10}")
    print(f"{'='*50}")
    print(f"{'Model size (MB)':<25} {fp32_size:>10.1f} {int8_size:>10.1f}")
    print(f"{'Inference time (ms)':<25} {fp32_ms:>10.1f} {int8_ms:>10.1f}")
    print(f"{'FPS':<25} {fp32_fps:>10.1f} {int8_fps:>10.1f}")
    print(f"{'Size reduction':<25} {'1.0x':>10} {f'{fp32_size/int8_size:.1f}x':>10}")
    print(f"{'Speedup':<25} {'1.0x':>10} {f'{fp32_ms/int8_ms:.1f}x':>10}")
    print(f"{'='*50}")


if __name__ == "__main__":
    main()
```

---

## 5. Review and Summary

### What We Covered

| Topic | Key Takeaway |
|-------|-------------|
| YOLOv5 Architecture | CSPDarknet backbone + PANet neck + multi-scale head = fast single-shot detection |
| Metrics | mAP@0.5 is the primary metric; mAP@0.5:0.95 is stricter. Always check PR curves. |
| Transfer Learning | Freeze backbone for small datasets, gradual unfreezing for medium datasets |
| Dataset Format | YOLO txt: `class_id x_center y_center width height` (all normalized) |
| Augmentation | Mosaic + HSV + flip are the key defenses against overfitting |
| PTQ | FP32 → INT8 via calibration data. 4x smaller, 2–4x faster, 1–3% mAP drop. |
| QAT | Simulate quantization during training. Better accuracy than PTQ, more effort. |
| ONNX Export | Framework-independent format. Gateway to OpenCV DNN, Hailo, TensorRT. |

### Key Formulas

$$
\text{IoU} = \frac{|A \cap B|}{|A \cup B|}
$$

$$
\text{Precision} = \frac{TP}{TP + FP}, \qquad \text{Recall} = \frac{TP}{TP + FN}
$$

$$
\text{mAP@0.5:0.95} = \frac{1}{10} \sum_{t=0.50}^{0.95} \text{mAP}@t
$$

$$
\text{Quantization: } q = \text{clamp}\left(\text{round}\left(\frac{x}{s} + z\right), 0, 2^n - 1\right)
$$

$$
s = \frac{x_{\max} - x_{\min}}{2^n - 1}, \qquad z = \text{round}\left(-\frac{x_{\min}}{s}\right)
$$

### Connection to Other Days

- **Day 17 (Lane Detection):** Lane detection uses classical CV. Object detection uses deep learning. Both feed into the fusion node from Day 18.
- **Day 18 (Sensor Fusion):** The YOLOv5 detections will be published as a ROS2 topic and fused with lane + LiDAR data.
- **Day 20 (Tomorrow):** We take the ONNX model exported today and compile it for the Hailo-10 NPU. The INT8 quantization concepts from Section 3 are applied by the Hailo compiler during `.hef` generation.

---

**Next up — Day 20:** The grand finale. We deploy YOLOv5 on the Hailo-10 NPU for real-time inference and integrate everything — lane detection, object detection, LiDAR, PID control, and safety — into a complete autonomous driving demo.
