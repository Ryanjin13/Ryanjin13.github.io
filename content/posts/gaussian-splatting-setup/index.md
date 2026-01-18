---
title: "Gaussian Splatting Test - Basic Setup"
date: 2025-03-20
description: "Complete installation guide for 3D Gaussian Splatting on Ubuntu 22.04"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "3D Reconstruction", "CUDA"]
draft: false
---

## Overview

This guide covers the complete setup process for 3D Gaussian Splatting on Ubuntu 22.04 with CUDA support.

## Prerequisites

- Ubuntu 22.04 LTS
- NVIDIA GPU with CUDA support
- At least 16GB RAM recommended

## 1. Install Core Dependencies

```bash
sudo apt update
sudo apt install -y \
    libglew-dev \
    libassimp-dev \
    libboost-all-dev \
    libgtk-3-dev \
    libopencv-dev \
    libglfw3-dev \
    libavdevice-dev \
    libavcodec-dev \
    libeigen3-dev \
    libxxf86vm-dev \
    libembree-dev \
    cmake \
    ninja-build \
    git
```

## 2. Install CUDA 11.8

```bash
# Download CUDA repository
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update

# Install CUDA 11.8
sudo apt install cuda-11-8

# Add to PATH
echo 'export PATH=/usr/local/cuda-11.8/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

## 3. Setup Conda Environment

```bash
# Install Miniconda (if not installed)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Clone Gaussian Splatting
git clone https://github.com/graphdeco-inria/gaussian-splatting --recursive
cd gaussian-splatting

# Create environment
conda env create --file environment.yml
conda activate gaussian_splatting
```

## 4. Build Submodules

```bash
# Build simple-knn
pip install submodules/simple-knn

# Build diff-gaussian-rasterization
pip install submodules/diff-gaussian-rasterization
```

## 5. Build SIBR Viewers

```bash
cd SIBR_viewers
cmake -B build -DCMAKE_BUILD_TYPE=Release -GNinja
cmake --build build --target install
```

## Troubleshooting

### Issue 1: C++ Standard Library Error

```
fatal error: filesystem: No such file or directory
```

**Solution:**
```bash
sudo apt install libstdc++-11-dev
```

### Issue 2: CUDA/GCC Compatibility

```
nvcc fatal: Unsupported gpu architecture 'compute_XX'
```

**Solution:** Build without CUDA for viewers:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF -GNinja
```

### Issue 3: Running the Viewer

```bash
./SIBR_viewers/install/bin/SIBR_gaussianViewer_app \
    -m output/your_scene/
```

## Usage

### Training

```bash
python train.py -s /path/to/your/data
```

### Rendering

```bash
python render.py -m output/your_model
```

## Tips

- Use `--iterations 30000` for high-quality results
- Start with smaller datasets to verify setup
- Monitor GPU memory usage during training
