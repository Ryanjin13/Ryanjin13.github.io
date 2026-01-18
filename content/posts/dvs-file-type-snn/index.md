---
title: "DVS File Type for SNN Vision Input"
date: 2025-07-20
description: "Dynamic Vision Sensor 이벤트 데이터 파일 형식 비교"
categories: ["Spiking Neural Network"]
tags: ["DVS", "AEDAT", "HDF5", "Neuromorphic Vision"]
draft: false
---

## Overview

Dynamic Vision Sensor (DVS) 이벤트 데이터 저장에 사용되는 4가지 파일 형식을 설명합니다. SNN (Spiking Neural Network) 비전 입력에 필수적인 내용입니다.

## Four Main File Formats

### 1. Text Format (.txt)

사람이 읽을 수 있는 형식. 각 라인: `timestamp x y polarity`

```
1000 120 80 1
1000 121 80 0
1001 122 81 1
```

동일한 timestamp에 여러 이벤트가 발생할 수 있어 동시 이벤트 표현이 가능합니다.

### 2. HDF5 Format (.h5)

**개발:** National Center for Supercomputing Applications (NCSA)

기후 모델링, 천문학 등 과학 분야에서 널리 사용됩니다.

**계층 구조:**
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

Python `h5py` 라이브러리로 효율적인 timestamp 기반 필터링 가능.

### 3. AEDAT2 Format (.aedat)

**AEDAT = Address Event Data format**

Neuromorphic engineering 커뮤니티에서 개발한 이벤트 기반 비전 센서 포맷.

**바이너리 구조:** 이벤트당 8 bytes
- 4 bytes: timestamp
- 4 bytes: address (x, y, polarity 인코딩)

### 4. AEDAT4 Format (.aedat)

최신 패킷 기반 포맷. 압축된 이벤트 패킷으로 효율적인 스트리밍 지원.

## File Size Comparison

| 형식 | 크기 | 특징 |
|------|------|------|
| events.txt | 1.2 KB | Human-readable |
| events.h5 | 0.8 KB | Binary structured |
| events.aedat2 | 0.3 KB | Compact binary |
| events.aedat4 | 0.2 KB | Compressed packets |

AEDAT4가 가장 효율적인 저장 공간을 제공합니다.
