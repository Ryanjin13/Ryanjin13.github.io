---
title: "Serial Communication Protocols"
date: 2024-07-20
description: "Overview of major serial communication standards"
categories: ["Computer Science"]
tags: ["Serial Communication", "I2C", "SPI", "UART", "CAN"]
draft: false
---

## Overview

Serial communication protocols enable data transfer between devices using sequential bit transmission. Each protocol addresses specific bandwidth, distance, and reliability requirements.

## Protocol Comparison

| Protocol | Year | Speed | Distance | Wires | Use Case |
|----------|------|-------|----------|-------|----------|
| RS-232 | 1960s | 115.2 kbps | 15m | 3-9 | PC/Modem |
| RS-485 | 1983 | 10 Mbps | 1200m | 2 | Industrial |
| I2C | 1982 | 3.4 Mbps | Short | 2 | IC-to-IC |
| SPI | 1980s | ~50 MHz | Short | 4 | High-speed IC |
| UART | 1960s | 115.2 kbps | Short | 2 | Debug/Serial |
| USB | 1996 | 40 Gbps | 5m | 4+ | Peripherals |
| CAN | 1983 | 1 Mbps | 40m | 2 | Automotive |
| FireWire | 1995 | 800 Mbps | 4.5m | 6 | A/V |

## RS-232

**Purpose:** Computer terminal and modem communication

```
TX ────────── RX
RX ────────── TX
GND ───────── GND
```

- Point-to-point only
- Voltage: ±15V
- Speed: up to 115.2 kbps
- Distance: ~15 meters

## RS-485

**Purpose:** Multi-device industrial networks

```
Device 1 ──┬── Device 2 ──┬── Device 3
           │              │
        A+ │           A+ │
        B- │           B- │
```

- Differential signaling
- Up to 32 devices
- Speed: 10 Mbps max
- Distance: ~1200 meters

## I2C (Inter-Integrated Circuit)

**Purpose:** Short-distance IC communication

```
Master ──── SDA ──── Slave 1
       ──── SCL ──── Slave 2
                     Slave 3...
```

- 2 wires: SDA (data), SCL (clock)
- Up to 128 device addresses
- Speeds: 100 kbps / 400 kbps / 3.4 Mbps
- Multi-master capable

## SPI (Serial Peripheral Interface)

**Purpose:** High-speed full-duplex communication

```
Master          Slave
 MOSI ─────────→ MOSI
 MISO ←───────── MISO
 SCLK ─────────→ SCLK
 SS   ─────────→ SS
```

- 4 wires: MOSI, MISO, SCLK, SS
- Full-duplex
- Speed: tens of MHz
- No addressing (chip select)

## UART (Universal Asynchronous Receiver-Transmitter)

**Purpose:** Simple asynchronous serial

```
Device A        Device B
   TX ─────────→ RX
   RX ←───────── TX
  GND ─────────── GND
```

- No clock line (asynchronous)
- Baud rates: 9600 - 115200 bps common
- Start/stop bits for synchronization

## USB (Universal Serial Bus)

**Purpose:** Standardized peripheral connectivity

| Version | Speed | Name |
|---------|-------|------|
| USB 1.1 | 12 Mbps | Full Speed |
| USB 2.0 | 480 Mbps | High Speed |
| USB 3.0 | 5 Gbps | SuperSpeed |
| USB 3.2 | 20 Gbps | SuperSpeed+ |
| USB 4 | 40 Gbps | - |

## CAN (Controller Area Network)

**Purpose:** Automotive and industrial reliability

```
ECU 1 ──┬── ECU 2 ──┬── ECU 3
     CAN_H         CAN_H
     CAN_L         CAN_L
```

- Differential 2-wire bus
- Built-in error detection
- Priority-based arbitration
- Speed: up to 1 Mbps
- Robust noise immunity

## Selection Guide

| Requirement | Recommended |
|-------------|-------------|
| Long distance | RS-485 |
| Many slow devices | I2C |
| High-speed IC | SPI |
| Simple debug | UART |
| PC peripheral | USB |
| Automotive | CAN |
