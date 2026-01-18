---
title: "DVS File Type for SNN Vision Input"
date: 2025-07-20
description: "Comparison of Dynamic Vision Sensor event data file formats"
categories: ["Spiking Neural Network"]
tags: ["DVS", "AEDAT", "HDF5", "Neuromorphic Vision"]
draft: false
---

## Overview

This post explains four different file formats used for storing Dynamic Vision Sensor (DVS) event data, which is essential for Spiking Neural Networks (SNN).

## Four Main File Formats

### 1. Text Format (.txt)

Human-readable format. Each line: `timestamp x y polarity`

```
1000 120 80 1
1000 121 80 0
1001 122 81 1
```

Multiple events can occur at the same timestamp, enabling simultaneous event representation.

### 2. HDF5 Format (.h5)

**Developed by:** National Center for Supercomputing Applications (NCSA)

Widely used in scientific fields including climate modeling and astronomy.

**Hierarchical Structure:**
```
/events/
    ├── x (dataset)
    ├── y (dataset)
    ├── t (timestamp)
    └── p (polarity)
/metadata/
    ├── resolution
    └── camera_info
/analysis/
    └── statistics
```

Python `h5py` library enables efficient timestamp-based filtering.

### 3. AEDAT2 Format (.aedat)

**AEDAT = Address Event Data format**

Developed by the neuromorphic engineering community for event-based vision sensors.

**Binary Structure:** 8 bytes per event
- 4 bytes: timestamp
- 4 bytes: address (x, y, polarity encoding)

### 4. AEDAT4 Format (.aedat)

Modern packet-based format with compressed event packets for efficient streaming.

## File Size Comparison

| Format | Size | Characteristics |
|--------|------|-----------------|
| events.txt | 1.2 KB | Human-readable |
| events.h5 | 0.8 KB | Binary structured |
| events.aedat2 | 0.3 KB | Compact binary |
| events.aedat4 | 0.2 KB | Compressed packets |

AEDAT4 provides the most efficient storage space.
