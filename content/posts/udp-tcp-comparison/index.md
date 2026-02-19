---
title: "TCP vs UDP: Transport Layer Protocol Comparison"
date: 2026-02-19
description: "A deep comparison of TCP and UDP covering structural differences, handshake mechanisms, flow/congestion control, and real-world use cases"
categories: ["Network"]
tags: ["TCP", "UDP", "Transport Layer", "Network Protocol", "OSI Model"]
draft: false
---

{{< katex >}}

## Overview

In the OSI 7-layer model, the **Transport Layer (L4)** is responsible for **end-to-end data delivery**. While the Network Layer (IP) handles **host-to-host routing**, the Transport Layer handles **process-to-process communication** using **port numbers** to distinguish between multiple applications on the same host.

```
Application Layer (L7)   ← HTTP, FTP, DNS, ROS2 DDS
Presentation Layer (L6)
Session Layer (L5)
─────────────────────────────────────
Transport Layer (L4)     ← TCP, UDP  ★
─────────────────────────────────────
Network Layer (L3)       ← IP
Data Link Layer (L2)     ← Ethernet, Wi-Fi
Physical Layer (L1)      ← Electrical / Optical signals
```

This post dives deep into the two core transport protocols: **TCP** and **UDP**.

---

## 1. TCP (Transmission Control Protocol)

### 1.1 Key Characteristics

TCP guarantees **reliable byte stream** delivery.

| Property | Description |
|----------|-------------|
| **Connection-Oriented** | Establishes connection via 3-Way Handshake before communication |
| **Reliable** | Retransmits lost packets, guarantees ordering |
| **Flow Control** | Adjusts transmission rate to match receiver's processing speed |
| **Congestion Control** | Automatically reduces transmission during network congestion |
| **Full-Duplex** | Simultaneous bidirectional communication |

### 1.2 TCP Header Structure

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
├─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┤
│          Source Port          │       Destination Port        │
├───────────────────────────────┼───────────────────────────────┤
│                    Sequence Number                            │
├──────────────────────────────────────────────────────────────┤
│                 Acknowledgment Number                         │
├───────┬───────┬─┼─┼─┼─┼─┼─┼─┼───────────────────────────────┤
│ Data  │       │U│A│P│R│S│F│  │                               │
│Offset │ Rsrvd │R│C│S│S│Y│I│  │         Window Size           │
│       │       │G│K│H│T│N│N│  │                               │
├───────┴───────┴─┴─┴─┴─┴─┴─┴─┼───────────────────────────────┤
│          Checksum             │       Urgent Pointer          │
├───────────────────────────────┼───────────────────────────────┤
│                    Options (variable length)                   │
└──────────────────────────────────────────────────────────────┘
```

**Header size**: Minimum 20 bytes (up to 60 bytes with options)

Key fields:
- **Sequence Number (32-bit)**: Position of the first byte of this segment in the byte stream
- **Acknowledgment Number (32-bit)**: Next byte number the receiver expects
- **Flags**: SYN, ACK, FIN, RST, PSH, URG control bits
- **Window Size (16-bit)**: Available receive buffer size (used for flow control)

### 1.3 3-Way Handshake (Connection Establishment)

```
    Client                              Server
      │                                   │
      │──── SYN (seq=x) ────────────────→│
      │                                   │
      │←─── SYN+ACK (seq=y, ack=x+1) ───│
      │                                   │
      │──── ACK (seq=x+1, ack=y+1) ────→│
      │                                   │
      │       Connection Established       │
      │       (Data transfer begins)       │
```

**Why 3-Way?**

- **1st SYN**: Synchronize sequence number for Client → Server direction
- **2nd SYN+ACK**: Synchronize sequence number for Server → Client direction + confirm first SYN
- **3rd ACK**: Confirm the second SYN

With only 2-Way, the server cannot verify that the client received its SYN. A minimum of **3 exchanges is required to synchronize sequence numbers in both directions**.

### 1.4 4-Way Handshake (Connection Termination)

```
    Client                              Server
      │                                   │
      │──── FIN (seq=u) ───────────────→│
      │                                   │  ← Server may still have
      │←─── ACK (ack=u+1) ──────────────│     data to send
      │                                   │
      │       (Half-Close state)          │
      │                                   │
      │←─── FIN (seq=v) ────────────────│
      │                                   │
      │──── ACK (ack=v+1) ─────────────→│
      │                                   │
      │    TIME_WAIT (2MSL wait)          │
```

The termination requires 4-Way because of **Half-Close**. Since TCP is full-duplex, one side may finish sending while the other still has data to transmit.

### 1.5 Flow Control (Sliding Window)

The receiver controls transmission rate by advertising its available buffer size:

```
Send buffer:
┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
│ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │ 8 │ 9 │10 │
└───┴───┴───┴───┴───┴───┴───┴───┴───┴───┘
 ACK  ACK Sent Sent  ←  Window Size  →
 done done            │  Can send    │ Cannot send

      ├──── Sliding Window ─────┤
```

The receiver communicates its remaining buffer size via the TCP header's **Window Size** field. The sender never transmits data exceeding this size.

### 1.6 Congestion Control

A mechanism to prevent network-wide congestion. Key algorithms:

```
cwnd (Congestion Window)
  │
  │         ★ ssthresh (Slow Start Threshold)
  │        ╱
  │      ╱   ← Congestion Avoidance (linear increase)
  │    ╱
  │  ╱
  │╱ ← Slow Start (exponential increase)
  │
  └────────────────────────── Time
         │
     Packet loss detected → cwnd halved
```

1. **Slow Start**: `cwnd` starts at 1 MSS, doubles per ACK (exponential growth)
2. **Congestion Avoidance**: After reaching `ssthresh`, increases by 1 MSS per RTT (linear growth)
3. **Fast Retransmit**: Upon receiving 3 duplicate ACKs, retransmit immediately before timeout
4. **Fast Recovery**: After packet loss, halve `cwnd` and resume linear increase

---

## 2. UDP (User Datagram Protocol)

### 2.1 Key Characteristics

UDP transmits data with **minimal overhead**.

| Property | Description |
|----------|-------------|
| **Connectionless** | Sends immediately without handshake |
| **Unreliable** | No packet loss detection or recovery, no ordering |
| **Lightweight** | 8-byte header, minimal processing overhead |
| **Message Boundary Preservation** | Each datagram is an independent unit |
| **Broadcast/Multicast** | Native 1:N transmission support |

### 2.2 UDP Header Structure

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
├─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┤
│          Source Port          │       Destination Port        │
├───────────────────────────────┼───────────────────────────────┤
│            Length             │           Checksum            │
└──────────────────────────────────────────────────────────────┘
```

**Header size**: Fixed 8 bytes

Compared to TCP's 20–60 byte header, this is extremely simple. No sequence numbers, window sizes, or flags. UDP is essentially **IP with just port numbers and a checksum added on top**.

### 2.3 UDP Data Transmission

```
    Client                              Server
      │                                   │
      │──── Datagram 1 ────────────────→│  (arrived)
      │──── Datagram 2 ────────── ✗      │  (lost)
      │──── Datagram 3 ────────────────→│  (arrived)
      │                                   │
      │  (No loss detection, no retransmission)
      │  (Datagram 3 may arrive before 2)
```

- Data is sent immediately without connection setup
- Each datagram is processed independently
- Packet loss and reordering are not handled at the protocol level

---

## 3. TCP vs UDP: Detailed Comparison

### 3.1 Structural Comparison

| Comparison | TCP | UDP |
|-----------|-----|-----|
| **Connection** | Connection-oriented (3-Way Handshake) | Connectionless |
| **Reliability** | Guaranteed (ACK, retransmission, ordering) | Not guaranteed |
| **Header Size** | 20–60 bytes | 8 bytes |
| **Data Unit** | Byte stream (no boundaries) | Datagram (boundaries preserved) |
| **Flow Control** | Sliding Window | None |
| **Congestion Control** | Slow Start, AIMD, etc. | None |
| **Ordering** | Guaranteed (Sequence Number) | Not guaranteed |
| **Multicast** | Not supported | Supported |
| **Latency** | Higher (handshake + ACK wait) | Lower (immediate send) |
| **Throughput** | Variable due to congestion control | Up to network bandwidth |

### 3.2 Message Boundary Handling

This is an often-overlooked but critical structural difference:

```
TCP (byte stream):
  Send: [Hello][World]   (two send() calls)
  Recv: [HelloWor][ld]   (boundaries NOT preserved)
    or: [H][elloWorld]
    or: [HelloWorld]

UDP (datagram):
  Send: [Hello][World]   (two sendto() calls)
  Recv: [Hello][World]   (boundaries EXACTLY preserved)
    or: [World][Hello]   (order may change)
    or: [Hello]          (World may be lost)
```

To delineate message boundaries in TCP, **application-level protocols** (e.g., length headers, delimiters) are required.

---

## 4. Real-World Use Cases

### TCP is ideal when:

| Protocol | Reason |
|----------|--------|
| **HTTP/HTTPS** | Web page data integrity is essential |
| **FTP** | File transfers cannot tolerate data loss |
| **SMTP/IMAP** | Email content must be delivered accurately |
| **SSH** | Remote commands require exact delivery |
| **Databases** | Query/result integrity must be guaranteed |

### UDP is ideal when:

| Protocol | Reason |
|----------|--------|
| **DNS** | Short query-response, fast response prioritized |
| **DHCP** | Requires broadcast, communication before connection setup |
| **Real-time video/audio (RTP)** | Latency is more harmful than retransmission |
| **Online gaming** | Only the latest position data matters between frames |
| **ROS2 DDS (default)** | Real-time delivery of robot sensor data |
| **IoT sensors** | Minimal overhead on lightweight devices |

### 4.1 Relationship with ROS2

ROS2's default DDS transport is **UDP-based**. But doesn't a robot need reliability?

DDS builds its **own reliability layer (RTPS) on top of UDP**:

```
┌─────────────────────┐
│    ROS2 Topic       │
├─────────────────────┤
│    DDS / RTPS       │  ← RELIABLE can be set via QoS here
│  (custom ACK/       │     compensating for UDP's unreliability
│   retransmission)   │
├─────────────────────┤
│       UDP           │  ← Default transport layer
├─────────────────────┤
│       IP            │
└─────────────────────┘
```

The advantage of this design:
- Leverages UDP's **low latency and multicast** capabilities
- Adds **reliability at the RTPS level only when needed**
- Effectively allows **selective application of TCP/UDP advantages** per topic

---

## 5. Protocols Building Reliability on UDP

Several protocols have been developed to overcome UDP's limitations while avoiding TCP's overhead:

| Protocol | Description |
|----------|-------------|
| **QUIC** | Developed by Google, foundation of HTTP/3. TLS + multiplexing + retransmission over UDP |
| **RTPS** | DDS transport protocol. Adds reliability/QoS over UDP |
| **DTLS** | Applies TLS security to UDP |
| **KCP** | Low-latency reliable transport for gaming |

All of these take the approach of **selectively implementing only the features needed on top of UDP's flexibility** — in contrast to TCP's "guarantee everything" design.

---

## 6. Summary

```
                    Reliability ↑
                         │
              TCP ●      │
                         │
         QUIC ●  RTPS ●  │
                         │
                         │
                  UDP ●  │
                         │
              ───────────┼──────────→ Speed / Lightweight
                         │
```

- **TCP**: "All data must arrive correctly, in order" → File transfer, web, email
- **UDP**: "Send as fast and light as possible" → Real-time streaming, DNS, gaming
- **UDP + custom reliability**: "Fast, but reliable only as much as needed" → QUIC, DDS/RTPS

Protocol selection ultimately comes down to the **trade-off between reliability and latency**. Understanding your application's requirements and choosing the appropriate transport protocol is the core of network design.
