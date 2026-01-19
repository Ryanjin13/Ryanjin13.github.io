---
title: "Major Serial Communication Protocols"
date: 2024-07-16
description: "Overview of essential serial communication protocols from RS-232 to CAN"
categories: ["Computer Science"]
tags: ["Serial Communication", "Protocols", "I2C", "SPI", "CAN", "USB"]
draft: false
---

{{< katex >}}

## Overview

Serial communication protocols are fundamental to modern electronics, enabling data transfer between devices. This guide covers eight major protocols, their characteristics, and applications.

## Protocol Comparison

| Protocol | Year | Speed | Distance | Wires |
|----------|------|-------|----------|-------|
| RS-232 | 1960s | 115.2 kbps | 15m | 3-9 |
| RS-485 | 1983 | 10 Mbps | 1200m | 2 (diff) |
| I2C | 1982 | 3.4 Mbps | On-chip | 2 |
| SPI | 1980s | ~50 MHz | On-board | 4+ |
| UART | 1960s | 115.2 kbps | varies | 2 |
| USB | 1996 | 40 Gbps | 5m | 4 |
| FireWire | 1995 | 800 Mbps | 4.5m | 6 |
| CAN | 1983 | 1 Mbps | 40m | 2 |

## RS-232 (1960s)

### Purpose
Developed for computer-terminal and modem communication.

### Characteristics

- Maximum speed: 115.2 kbps
- Maximum distance: 15 meters
- Voltage levels: plus/minus 3V to 15V
- Point-to-point connection

### Signal Pins

| Pin | Signal | Direction |
|-----|--------|-----------|
| TxD | Transmit Data | DTE to DCE |
| RxD | Receive Data | DCE to DTE |
| GND | Ground | - |
| RTS | Request to Send | DTE to DCE |
| CTS | Clear to Send | DCE to DTE |

## RS-485 (1983)

### Purpose
Long-distance, multi-device industrial communication.

### Specifications
- Maximum speed: 10 Mbps
- Maximum distance: ~1200 meters
- Differential signaling for noise immunity
- Multi-drop topology (up to 32 devices)

### Voltage Levels

$$
V_{differential} = V_A - V_B
$$

| Logic | Voltage |
|-------|---------|
| 1 | V_A - V_B > +200mV |
| 0 | V_A - V_B < -200mV |

## I2C (1982)

### Purpose
Inter-IC communication developed by Philips for simple on-chip connectivity.

### Specifications
- Two wires: SDA (data) and SCL (clock)
- 128 addressable devices (7-bit addressing)
- Speed modes: 100 kbps, 400 kbps, 1 Mbps, 3.4 Mbps
- Master-slave architecture

### Speed Modes

| Mode | Speed |
|------|-------|
| Standard | 100 kbps |
| Fast | 400 kbps |
| Fast Plus | 1 Mbps |
| High Speed | 3.4 Mbps |

## SPI (1980s)

### Purpose
Developed by Motorola for high-speed synchronous communication.

### Specifications
- Four wires: SCLK, MOSI, MISO, SS
- Full-duplex communication
- Speeds up to tens of MHz
- No addressing (chip select lines)

### Signal Functions

| Signal | Function |
|--------|----------|
| SCLK | Serial Clock |
| MOSI | Master Out, Slave In |
| MISO | Master In, Slave Out |
| SS/CS | Slave Select / Chip Select |

### SPI Modes

| Mode | CPOL | CPHA | Description |
|------|------|------|-------------|
| 0 | 0 | 0 | Sample on rising edge |
| 1 | 0 | 1 | Sample on falling edge |
| 2 | 1 | 0 | Sample on falling edge |
| 3 | 1 | 1 | Sample on rising edge |

## UART (1960s)

### Purpose
Asynchronous serial interface for bidirectional communication.

### Specifications
- Asynchronous (no clock line)
- Common speeds: 9600, 115200 baud
- Start/stop bits for synchronization
- Optional parity bit

### Baud Rate Calculation

$$
\text{Bit Time} = \frac{1}{\text{Baud Rate}}
$$

At 115200 baud: Bit Time = 8.68 microseconds

## USB (1996)

### Purpose
Standardized peripheral interface for consumer electronics.

### Version Evolution

| Version | Year | Speed |
|---------|------|-------|
| USB 1.1 | 1998 | 12 Mbps |
| USB 2.0 | 2000 | 480 Mbps |
| USB 3.0 | 2008 | 5 Gbps |
| USB 3.1 | 2013 | 10 Gbps |
| USB 3.2 | 2017 | 20 Gbps |
| USB 4.0 | 2019 | 40 Gbps |

### Features
- Hot-pluggable
- Power delivery (up to 240W with USB PD)
- Tiered star topology
- Automatic device enumeration

## FireWire / IEEE 1394 (1995)

### Purpose
Apple's high-speed multimedia protocol for video and storage.

### Specifications
- FireWire 400: 400 Mbps
- FireWire 800: 800 Mbps
- Isochronous data transfer (guaranteed bandwidth)
- Peer-to-peer communication
- Hot-pluggable

## CAN (1983)

### Purpose
Bosch's automotive communication protocol for vehicle networks.

### Specifications
- Maximum speed: 1 Mbps (CAN 2.0)
- CAN FD: Up to 8 Mbps
- Differential signaling
- Multi-master architecture
- Automatic error detection and retransmission

### Arbitration

Priority-based arbitration using identifier:

$$
\text{Lower ID} \rightarrow \text{Higher Priority}
$$

## Protocol Selection Guide

| Application | Recommended Protocol |
|-------------|---------------------|
| Sensor reading | I2C |
| High-speed display | SPI |
| Industrial control | RS-485, CAN |
| Automotive | CAN, CAN FD |
| Consumer devices | USB |
| Debug/console | UART |
| Long distance | RS-485 |

## Summary

Key considerations for protocol selection:
1. **Speed requirements**: USB 4 > SPI > RS-485 > I2C
2. **Distance**: RS-485 > CAN > RS-232 > others
3. **Complexity**: USB > CAN > I2C > SPI > UART
4. **Multi-device**: CAN, RS-485, I2C support multiple nodes
5. **Application domain**: CAN for automotive, USB for consumer
