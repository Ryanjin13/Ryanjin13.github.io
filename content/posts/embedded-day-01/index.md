---
title: "Day 1 — Raspberry Pi 5 and ARM Architecture"
date: 2026-03-05T01:00:00
description: "Understanding embedded systems fundamentals: MCU vs MPU vs SoC, RPi 5 architecture with BCM2712, ARM Cortex-A76 pipeline, and GPIO through RP1 southbridge"
categories: ["Autonomous Driving"]
tags: ["Raspberry Pi", "ARM", "Embedded Systems", "GPIO", "BCM2712"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 1
draft: false
---

{{< katex >}}

## What You'll Learn

Welcome to Day 1 of the **Embedded Basics for Autonomous Car** series. This is where everything starts: understanding the hardware that runs your autonomous driving software stack. Before we write a single line of ROS2 or SLAM code, we need to deeply understand the platform we are building on.

By the end of this post, you will:

- Clearly distinguish between MCU, MPU, and SoC — and know exactly when to use each
- Understand the Raspberry Pi 5 architecture from chip to pin
- Know how the BCM2712 SoC and the RP1 southbridge work together
- Understand the ARM Cortex-A76 microarchitecture: pipeline, caches, and buses
- Appreciate why RISC won the embedded world
- Be able to boot a Pi 5, inspect its hardware from the command line, and control GPIO pins with Python

---

## 1. Embedded Systems: MCU vs MPU vs SoC

### 1.1 What Is an Embedded System?

An **embedded system** is a computer designed to perform a **dedicated function** within a larger system. Unlike a general-purpose PC, an embedded system has constraints: limited power, real-time deadlines, specific I/O requirements.

Examples in an autonomous car:
- **ECU (Electronic Control Unit)**: controls braking, steering, engine timing
- **Camera ISP**: processes raw image sensor data at 30+ FPS
- **LiDAR controller**: generates and times laser pulses
- **Central compute unit**: runs perception, planning, and control stacks

### 1.2 MCU — Microcontroller Unit

A **Microcontroller Unit (MCU)** integrates a processor core, memory (SRAM + Flash), and peripherals onto a **single chip**. Think of it as a complete tiny computer on one die.

```
+-----------------------------+
|         MCU Chip            |
|  +-----+ +-----+ +------+  |
|  | CPU | | SRAM| | Flash|  |
|  +-----+ +-----+ +------+  |
|  +-----+ +-----+ +------+  |
|  | GPIO| | ADC | |Timers|  |
|  +-----+ +-----+ +------+  |
|  +-----+ +-----+ +------+  |
|  | UART| | SPI | | I2C  |  |
|  +-----+ +-----+ +------+  |
+-----------------------------+
```

**Key characteristics:**
- Clock speed: 16 MHz to ~600 MHz
- Memory: KB of SRAM, KB-MB of Flash
- No OS required (bare-metal or RTOS)
- Deterministic timing (critical for real-time)
- Ultra-low power consumption (uA to mA range)

**Common examples:**
- STM32 (ARM Cortex-M series) — the workhorse of automotive ECUs
- ESP32 — Wi-Fi/BLE enabled, popular for IoT
- ATmega328P — the chip inside Arduino Uno

**When to use:** Direct sensor reading, motor PWM control, CAN bus communication, anything needing microsecond-level deterministic response.

### 1.3 MPU — Microprocessor Unit

A **Microprocessor Unit (MPU)** is a processor core that relies on **external** memory and peripherals. It needs a full circuit board with separate RAM chips, storage, and I/O controllers.

```
+---------+    +------+    +-------+
|   MPU   |----|  DDR |    | Flash |
|  (CPU)  |    |  RAM |    |Storage|
+----+----+    +------+    +-------+
     |
+----+-------------------------------+
|  External Peripherals on PCB       |
|  GPIO, USB, Ethernet, etc.         |
+---------+---------+----------------+
```

**Key characteristics:**
- Clock speed: 1 GHz to 4+ GHz
- Memory: GB of external DDR SDRAM
- Requires an OS (Linux, Android, Windows)
- Virtual memory, MMU, multi-process support
- Higher power consumption (Watts range)

**Common examples:**
- Intel Core / AMD Ryzen — desktop/server processors
- ARM Cortex-A series (when used standalone)

**When to use:** Complex computation, running full operating systems, when you need multi-GB RAM and multi-process isolation.

### 1.4 SoC — System on Chip

A **System on Chip (SoC)** takes the MPU concept and integrates everything back onto one die: CPU cores, GPU, memory controller, I/O controllers, specialized accelerators — all on a single chip.

```
+--------------------------------------+
|             SoC Die                  |
|  +----------+  +-----+  +--------+  |
|  | CPU Cores|  | GPU |  |  NPU   |  |
|  | (A76 x4) |  |     |  | (opt.) |  |
|  +----------+  +-----+  +--------+  |
|  +----------+  +-----+  +--------+  |
|  | Mem Ctrl |  | PCIe|  | USB/ETH|  |
|  | (LPDDR)  |  |     |  |        |  |
|  +----------+  +-----+  +--------+  |
+--------------------------------------+
        |
    +---+---+
    |DDR RAM|  (external, but controller is on-chip)
    +-------+
```

**Key characteristics:**
- Multiple CPU cores (often heterogeneous: big.LITTLE)
- Integrated GPU, ISP, DSP, NPU
- On-chip memory controller (connected to external DDR)
- On-chip PCIe, USB, UART, SPI, I2C controllers
- Moderate power (2W to 30W typically)

**Common examples:**
- BCM2712 (Raspberry Pi 5) — Cortex-A76 quad-core + VideoCore VII
- NVIDIA Orin — 12-core ARM + Ampere GPU + DLA (autonomous driving)
- Qualcomm Snapdragon — the SoC in your phone
- Apple M-series — the SoC in MacBooks

**When to use:** When you need high performance with integrated peripherals in a compact form factor. This is the default for modern embedded Linux platforms.

### 1.5 Comparison Table

| Feature | MCU | MPU | SoC |
|---------|-----|-----|-----|
| **Processor** | Single core (Cortex-M) | Multi-core (Cortex-A) | Multi-core + GPU + accelerators |
| **Memory** | On-chip SRAM (KB) | External DDR (GB) | External DDR via on-chip controller |
| **Storage** | On-chip Flash (MB) | External (SSD/eMMC) | External (eMMC/NVMe) |
| **OS** | Bare-metal / RTOS | Full OS (Linux) | Full OS (Linux/Android) |
| **Boot time** | Milliseconds | Seconds | Seconds |
| **Power** | mW | W | W (optimized) |
| **Real-time** | Deterministic | Non-deterministic | Mixed (with RTOS co-processor) |
| **Cost** | $1 - $10 | $20 - $500+ | $5 - $100+ |
| **Example** | STM32F4 | Intel i7 | BCM2712, Orin |

### 1.6 In Autonomous Cars — You Need All Three

A modern autonomous vehicle uses all three types in a layered architecture:

```
+-------------------------------------------------+
|                Central Compute                   |
|       SoC (NVIDIA Orin / Qualcomm SA8650)       |
|    Perception, Planning, Decision Making         |
+-------------------------------------------------+
|             Zone Controllers                     |
|       MPU/SoC (NXP S32G, TI TDA4)              |
|    Domain gateway, sensor preprocessing          |
+-------------------------------------------------+
|               Actuator ECUs                      |
|       MCU (STM32, Infineon AURIX)               |
|    Braking, Steering, Motor, CAN interface       |
+-------------------------------------------------+
```

Our Raspberry Pi 5, with its BCM2712 SoC, sits at the **Central Compute** level in our learning platform. It runs Linux, has enough power for camera processing and basic SLAM, and provides all the I/O we need.

---

## 2. Raspberry Pi 5 Architecture Deep Dive

### 2.1 The BCM2712 SoC

The Raspberry Pi 5 is built around the **Broadcom BCM2712** SoC. This is a massive upgrade from the BCM2711 (Pi 4). Let's break it down:

```
+-------------------------------------------------------------+
|                       BCM2712 SoC                            |
|                                                              |
|  +----------------------------------------------+           |
|  |         4x ARM Cortex-A76 @ 2.4 GHz          |           |
|  |  +---------+ +---------+ +---------+ +---------+         |
|  |  | Core 0  | | Core 1  | | Core 2  | | Core 3  |         |
|  |  | L1I:64K | | L1I:64K | | L1I:64K | | L1I:64K |         |
|  |  | L1D:64K | | L1D:64K | | L1D:64K | | L1D:64K |         |
|  |  +----+----+ +----+----+ +----+----+ +----+----+         |
|  |       +------+----+------+----+           |               |
|  |              |           |                |               |
|  |         +----+-----------+----------------+--+            |
|  |         |        512 KB Shared L2 Cache      |            |
|  |         +----------------+-------------------+            |
|  |                          |                                |
|  |         +----------------+-------------------+            |
|  |         |          2 MB L3 Cache              |            |
|  |         +----------------+-------------------+            |
|  +--------------------------|-------------------+            |
|                             |                                |
|  +--------------------------+-------------------+            |
|  |              AXI Interconnect Bus             |            |
|  +---+----------+----------+----------+---------+            |
|      |          |          |          |                      |
|  +---+---+ +---+---+ +---+---+ +----+--------+             |
|  |Video  | |LPDDR  | | PCIe  | |    RP1      |             |
|  |CoreVII| |4X-4266| |Gen2x4 | | Southbridge |             |
|  | GPU   | | Ctrl  | | Ctrl  | | (via PCIe)  |             |
|  +-------+ +-------+ +-------+ +-------------+             |
|                                                              |
+-------------------------------------------------------------+
```

**Key specs:**
- **CPU**: 4x Cortex-A76 at 2.4 GHz (up from Cortex-A72 at 1.8 GHz in Pi 4)
- **GPU**: VideoCore VII (OpenGL ES 3.1, Vulkan 1.2)
- **Memory controller**: LPDDR4X-4267, supporting 4GB or 8GB
- **PCIe**: Gen 2.0 x4 controller (one lane exposed externally, others to RP1)
- **Process node**: 16nm (TSMC)

### 2.2 Performance Jump: Pi 4 vs Pi 5

| Metric | Pi 4 (BCM2711) | Pi 5 (BCM2712) | Improvement |
|--------|---------------|----------------|-------------|
| CPU | Cortex-A72, 1.8 GHz | Cortex-A76, 2.4 GHz | ~2-3x single-thread |
| L2 Cache | 1 MB shared | 512 KB per cluster | Better per-core |
| L3 Cache | None | 2 MB | New level |
| Memory | LPDDR4-3200 | LPDDR4X-4267 | ~33% bandwidth |
| GPU | VideoCore VI | VideoCore VII | ~2x |
| I/O | On-SoC GPIO | RP1 southbridge | Much more capable |
| PCIe | None exposed | Gen 2.0 x1 slot | NVMe/AI accelerator |

The Cortex-A76 is two **microarchitecture generations** ahead of Cortex-A72. It was designed for laptop-class workloads, so in a tiny Pi form factor, it is quite powerful.

### 2.3 The RP1 Southbridge — A Game-Changing Architecture Decision

This is the most architecturally significant change in Pi 5. Previously, GPIO, SPI, I2C, UART, and USB were all handled by peripheral blocks **inside** the BCM SoC. In Pi 5, Raspberry Pi designed their own custom chip called **RP1** that handles all I/O.

```
+----------------------------------------------------------+
|                  Raspberry Pi 5 Board                     |
|                                                           |
|  +-------------+     PCIe Gen2 x4      +--------------+  |
|  |   BCM2712   |<=====================>|     RP1       |  |
|  |             |    (internal link)     |  Southbridge  |  |
|  |  CPU cores  |                        |               |  |
|  |  GPU        |                        |  2x USB 3.0   |  |
|  |  Memory Ctrl|                        |  2x USB 2.0   |  |
|  |  PCIe Ctrl  |                        |  Gigabit ETH  |  |
|  |             |                        |  2x MIPI DSI  |  |
|  +-------------+                        |  2x MIPI CSI  |  |
|                                         |  28x GPIO      |  |
|        +----------+                     |  6x UART       |  |
|        | PCIe x1  | (external slot)     |  5x SPI        |  |
|        | for user  |                    |  5x I2C        |  |
|        | (NVMe,   |                     |  2x PWM        |  |
|        |  Hailo)  |                     +--------------+  |
|        +----------+                                       |
+----------------------------------------------------------+
```

**Why does this matter?**

1. **GPIO path is different**: When you toggle a GPIO pin on Pi 5, the signal path is: CPU core -> AXI bus -> PCIe controller -> PCIe link -> RP1 -> GPIO pad. This is fundamentally different from Pi 4 where GPIO was memory-mapped directly on the SoC.

2. **RPi.GPIO is broken**: The old `RPi.GPIO` library directly accessed BCM SoC registers via `/dev/mem`. Since GPIO registers now live on RP1 (behind a PCIe link), direct memory-mapped access no longer works. You **must** use `libgpiod` or `gpiozero` (which uses libgpiod as its backend on Pi 5).

3. **More I/O bandwidth**: RP1 connects to BCM2712 via a **PCIe Gen 2 x4 link** (16 Gbit/s total), which is far more bandwidth than the old internal bus. This means USB 3.0 and Ethernet no longer share bandwidth like on Pi 4.

4. **Dual camera/display**: RP1 provides two MIPI CSI-2 and two MIPI DSI ports. For autonomous driving, this means you can connect **two cameras simultaneously** without an external multiplexer.

### 2.4 Why RPi.GPIO Fails on Pi 5 — Technical Details

Let us trace exactly what goes wrong. On Pi 4, the `RPi.GPIO` library works by:

1. Opening `/dev/mem` (or `/dev/gpiomem`)
2. Using `mmap()` to map BCM2711 GPIO registers at physical address `0xFE200000` into user space
3. Directly reading/writing those registers (e.g., GPFSEL, GPSET, GPCLR)

On Pi 5:
- Those physical addresses belong to BCM2712, which has **no GPIO controller** — GPIO was moved to RP1
- RP1 is a separate chip with its own address space, accessible only via PCIe
- The kernel exposes RP1 GPIO through the standard Linux `gpiochip` interface (`/dev/gpiochipN`)
- Any library that bypasses the kernel and goes directly to physical memory **will not work**

The correct stack on Pi 5:

```
Your Python Code
       |
   gpiozero / libgpiod (user-space library)
       |
   /dev/gpiochip4  (Linux character device)
       |
   Linux GPIO subsystem (kernel)
       |
   RP1 PCIe driver (kernel)
       |
   BCM2712 PCIe controller (hardware)
       |
   PCIe link
       |
   RP1 GPIO controller (hardware)
       |
   Physical GPIO pin
```

### 2.5 PCIe 2.0 External Slot — AI Accelerator Gateway

The Pi 5 exposes a **PCIe Gen 2.0 x1 slot** via an FPC connector (you need a HAT+ adapter board). This single lane provides:

$$\text{PCIe Gen2 x1 bandwidth} = 5 \text{ GT/s} \times \frac{8}{10} = 4 \text{ Gbit/s} = 500 \text{ MB/s}$$

(The 8/10 factor is the encoding overhead for PCIe Gen 2.)

This slot is critical for our autonomous car project because it lets us attach:
- **NVMe SSD**: Fast storage for logging camera data and maps
- **Hailo-10 AI accelerator**: dedicated NPU for running YOLO, depth estimation, lane detection at the edge
- **Coral TPU**: Google's edge AI accelerator

We will use this PCIe slot in later days when we integrate the Hailo AI accelerator for real-time object detection.

---

## 3. ARM Cortex-A76 Microarchitecture

### 3.1 RISC vs CISC — Why ARM Won the Embedded World

Before diving into the A76 specifics, let's understand the fundamental philosophy.

**CISC (Complex Instruction Set Computer)** — x86 approach:
- Many complex instructions (e.g., `REP MOVSB` copies a block of memory)
- Variable-length instructions (1 to 15 bytes in x86)
- Instructions can access memory directly
- Hardware decoder is complex and power-hungry
- Example: `ADD [mem], reg` — reads memory, adds, writes back, all in one instruction

**RISC (Reduced Instruction Set Computer)** — ARM approach:
- Simple, uniform instructions
- Fixed-length instructions (32 bits in ARM, 16 bits in Thumb)
- Load/Store architecture: only `LDR`/`STR` access memory
- Arithmetic operates only on registers
- Simpler decoder -> lower power -> more cores in same power budget

| Aspect | RISC (ARM) | CISC (x86) |
|--------|-----------|------------|
| Instruction length | Fixed (32-bit) | Variable (1-15 bytes) |
| Instructions | Simple, one operation | Complex, multi-step |
| Registers | 31 general-purpose (AArch64) | 16 in x86-64 |
| Memory access | Load/Store only | Any instruction can access memory |
| Decode complexity | Simple, low power | Complex, needs micro-op translation |
| Power efficiency | High | Lower |

**The load/store principle in practice:**

```asm
; CISC (x86): Add memory value to register in one instruction
ADD EAX, [memory_address]

; RISC (ARM): Same operation requires two instructions
LDR R1, [R0]        ; Load from memory into register
ADD R2, R2, R1      ; Add registers
```

This might seem like RISC is slower (more instructions), but:

$$\text{Execution Time} = \text{Instruction Count} \times \text{CPI} \times \text{Clock Period}$$

Where **CPI** is Cycles Per Instruction. RISC has more instructions but lower CPI and shorter clock period. The net result, especially at low power, is that RISC wins on **performance per watt**.

**Why ARM dominates today:**
- Desktop/Server: x86 dominated historically — but ARM is now entering (AWS Graviton, Apple M-series)
- Mobile/Embedded: ARM dominates **completely** (99%+ of smartphones, most embedded SoCs)
- Automotive: ARM Cortex-A/R/M across the entire vehicle

### 3.2 ARM Cortex Family Overview

ARM licenses processor **designs** (IP cores) that SoC manufacturers integrate into their chips.

| Series | Profile | Use Case | Example |
|--------|---------|----------|---------|
| **Cortex-A** | Application | Full OS, high performance | A76 (Pi 5), A78 (Orin) |
| **Cortex-R** | Real-time | Safety-critical, deterministic | R5F (automotive ECU) |
| **Cortex-M** | Microcontroller | Low-power, bare-metal/RTOS | M4 (STM32), M0+ (RP2040) |

For autonomous driving:
- **Cortex-A**: Runs Linux, perception stack, SLAM
- **Cortex-R**: Runs safety monitor, ASIL-D rated
- **Cortex-M**: Runs motor control, CAN interface, sensor sampling

### 3.3 Cortex-A76 Pipeline Deep Dive

The Cortex-A76 uses an **out-of-order, superscalar** pipeline with approximately **13 stages**. Let's trace an instruction through it.

```
+---------------------------------------------------------------+
|                    Cortex-A76 Pipeline                         |
|                                                                |
|  FETCH          DECODE         DISPATCH       EXECUTE   RETIRE |
|  +------+      +------+      +------+      +------+  +------+ |
|  | F1   |      | D1   |      | Ren  |      | Int  |  | Ret  | |
|  | Pred |----->| Dec  |----->| ame  |----->| ALU  |->| ire  | |
|  | ict  |      | ode  |      |      |      | x2   |  |      | |
|  |      |      |      |      | Disp |      |      |  | Write| |
|  | I$   |      |      |      | atch |      | FP   |  | Back | |
|  | Fetch|      | Macro|      |      |      | x2   |  |      | |
|  |      |      | Fuse |      | Issue|      |      |  |Commit| |
|  | BPU  |      |      |      | Queue|      | Br   |  |      | |
|  |      |      | Micro|      |      |      | x1   |  |      | |
|  |      |      | Op   |      |      |      |      |  |      | |
|  |      |      |      |      |      |      | LD/ST|  |      | |
|  |      |      |      |      |      |      | x2   |  |      | |
|  +------+      +------+      +------+      +------+  +------+ |
|                                                                |
|  4-wide fetch   4-wide decode  8-wide dispatch  8 exec units   |
|  64KB I-cache   Macro-fusion   128-entry ROB    64KB D-cache   |
|  BTB + TAGE     Micro-op cache                  L2: 512KB     |
|  predictor                                       L3: 2MB      |
+---------------------------------------------------------------+
```

**Stage-by-stage breakdown:**

**1. Fetch (F1-F4):**
- The **Branch Prediction Unit (BPU)** predicts the next PC before the instruction is even decoded
- Uses a **TAGE predictor** (Tagged Geometric History Length) — one of the most accurate branch predictors known
- Fetches **4 instructions per cycle** from the **64 KB L1 Instruction Cache**
- If the I-cache misses, fetch stalls while the line is brought from L2
- The **Branch Target Buffer (BTB)** caches branch destination addresses for fast redirection

**2. Decode (D1-D3):**
- Decodes **4 ARM instructions per cycle** into internal **micro-ops**
- **Macro-fusion**: Combines common instruction pairs (e.g., `CMP` + `B.EQ`) into a single micro-op, effectively increasing throughput
- ARM instructions are fixed-width (32-bit A64 in AArch64), making decode much simpler than x86's variable-length instruction nightmare
- A **micro-op cache** stores previously decoded sequences, allowing the decode stage to be bypassed for hot loops

**3. Rename/Dispatch:**
- **Register renaming** eliminates false data dependencies (WAR and WAW hazards)
- Maps 31 architectural registers to a much larger physical register file (~128 physical registers)
- Dispatches up to **8 micro-ops per cycle** into the issue queues
- The **Reorder Buffer (ROB)** holds ~128 entries, allowing deep out-of-order execution while maintaining the illusion of in-order completion

**4. Execute:**
- **8 execution units** work in parallel:
  - 2x Integer ALU (add, subtract, logic, shift)
  - 2x FP/NEON SIMD (128-bit vector operations)
  - 1x Branch unit
  - 2x Load/Store units (can do 2 memory ops per cycle)
  - 1x Integer multiply/divide
- Out-of-order: instructions execute as soon as their operands are ready, regardless of program order

**5. Retire/Commit:**
- Instructions **commit in program order** (even though they executed out of order)
- This maintains the illusion of sequential execution for software
- Results are written to the architectural register file
- Exceptions and interrupts are handled precisely at the retirement stage

**Why out-of-order matters for autonomous driving code:**

Consider this code pattern (common in image processing):

```python
pixel_a = image[y][x]      # Cache miss! ~100 cycles to DRAM
pixel_b = image[y][x+1]    # Might be in same cache line
result = pixel_a + pixel_b  # Depends on both loads
output[y][x] = result       # Independent store
```

An in-order CPU would stall at the first cache miss, wasting 100 cycles. An out-of-order CPU like the A76 can execute other independent instructions during the stall, keeping the pipeline productive.

### 3.4 Cache Hierarchy

Cache is the single most important performance feature for our autonomous driving workloads. When processing camera frames, data locality determines whether you get 10 FPS or 30 FPS.

```
+----------+
| CPU Core |
|          |
| +------+ |    ~4 cycles    +----------+
| | L1I  | |<--------------->| 64 KB    |
| |      | |                 | I-Cache  |
| +------+ |                 +----------+
|          |
| +------+ |    ~4 cycles    +----------+
| | L1D  | |<--------------->| 64 KB    |
| |      | |                 | D-Cache  |
| +------+ |                 +----------+
+----------+
      |
      | ~9 cycles
      v
+--------------+
|   L2 Cache   |  512 KB per cluster (shared by 4 cores)
|  (Unified)   |
+--------------+
      |
      | ~30 cycles
      v
+--------------+
|   L3 Cache   |  2 MB (shared, system-level)
|  (Unified)   |
+--------------+
      |
      | ~100+ cycles
      v
+--------------+
|  LPDDR4X     |  4 or 8 GB
|  Main Memory |  4267 MT/s
+--------------+
```

| Level | Size | Latency | Shared? |
|-------|------|---------|---------|
| L1 I-Cache | 64 KB per core | ~4 cycles | No (per core) |
| L1 D-Cache | 64 KB per core | ~4 cycles | No (per core) |
| L2 Cache | 512 KB | ~9 cycles | Yes (all 4 cores) |
| L3 Cache | 2 MB | ~30 cycles | Yes (system-wide) |
| LPDDR4X | 4/8 GB | ~100+ cycles | Yes |

**Access latency matters enormously.** Consider processing a 1920x1080 image:

$$\text{Image size} = 1920 \times 1080 \times 3 \text{ channels} = 6{,}220{,}800 \text{ bytes} \approx 6 \text{ MB}$$

This image does **not** fit in L2 (512 KB) or L3 (2 MB). So naive pixel-by-pixel access will constantly miss the cache and go to DRAM (100+ cycle penalty). This is why **tiled processing** and proper memory access patterns are critical — we will explore this in detail in the camera processing days.

The **average memory access time** equation:

$$T_{\text{avg}} = T_{\text{hit}} + \text{Miss Rate} \times T_{\text{miss penalty}}$$

For L1 D-cache on Cortex-A76 with typical image processing:
- \(T_{\text{hit}} = 4\) cycles
- L1 miss rate: ~5-10%, L2 miss rate: ~2-5%
- \(T_{\text{L2 penalty}} \approx 9\) cycles
- \(T_{\text{DRAM penalty}} \approx 100\) cycles

$$T_{\text{avg}} = 4 + 0.07 \times 9 + 0.03 \times 100 = 4 + 0.63 + 3.0 = 7.63 \text{ cycles}$$

This means even a small miss rate to DRAM can nearly double your effective access time. Writing cache-friendly code (sequential access, tiling, prefetching) is essential for real-time performance.

### 3.5 AXI and APB Bus Architecture

Inside the SoC, different components need to communicate. ARM defines standard bus protocols:

**AXI (Advanced eXtensible Interface):**
- High-performance, high-bandwidth bus
- Used for CPU <-> Memory, CPU <-> DMA, CPU <-> PCIe
- Supports burst transfers, out-of-order transactions
- Separate read and write channels (full duplex)
- Up to 128-bit data width

**APB (Advanced Peripheral Bus):**
- Low-power, simple bus for slow peripherals
- Used for configuration registers: UART config, GPIO config, timer config
- Single 32-bit data channel
- No burst, no pipeline — simple and low-power

**AHB (Advanced High-performance Bus):**
- Middle ground between AXI and APB
- Used in some peripheral controllers

```
+----------------------------------------------+
|                AXI Interconnect                |
|   (High bandwidth: CPU, Memory, DMA, PCIe)   |
+---+----------+----------+----------+---------+
    |          |          |          |
+---+---+ +---+---+ +---+---+ +---+-------+
| DDR   | | PCIe  | |  DMA  | | AXI->APB  |
| Ctrl  | | Ctrl  | |       | |  Bridge   |
+-------+ |(to RP1)| +-------+ +----+------+
          +-------+                  |
                              +------+-------------+
                              |       APB Bus       |
                              | (Low-speed periph.) |
                              +---+------+------+--+
                                  |      |      |
                                Timer  UART*  WDT
                                (* internal, not RP1)
```

Understanding this bus hierarchy explains **why GPIO on Pi 5 is different**: the GPIO controller lives on RP1, which sits behind the PCIe controller on the AXI bus. Every GPIO access traverses: CPU -> AXI -> PCIe controller -> PCIe link -> RP1.

---

## 4. RISC Instruction Set — ARM AArch64 Overview

### 4.1 Register Set

AArch64 (the 64-bit ARM instruction set) provides:

- **31 general-purpose registers**: X0-X30 (64-bit) / W0-W30 (32-bit view of the lower half)
- **SP**: Stack Pointer
- **PC**: Program Counter (not directly accessible as a GPR)
- **PSTATE**: Processor state flags (N, Z, C, V — Negative, Zero, Carry, oVerflow)
- **32 SIMD/FP registers**: V0-V31 (128-bit, for NEON vector operations)

**Calling convention (important for understanding disassembly):**

| Register | Purpose |
|----------|---------|
| X0-X7 | Function arguments and return values |
| X8 | Indirect result location |
| X9-X15 | Temporary (caller-saved) |
| X16-X17 | Intra-procedure call scratch |
| X19-X28 | Callee-saved (preserved across calls) |
| X29 (FP) | Frame pointer |
| X30 (LR) | Link register (return address) |

### 4.2 Instruction Categories

| Category | Instructions | Purpose |
|----------|-------------|---------|
| Data Processing | ADD, SUB, MUL, AND, ORR, EOR, LSL, MOV, MVN, CLZ, REV | Arithmetic and logic on registers |
| Memory Access | LDR, STR, LDP, STP, LDRB, LDRH | Load from / Store to memory |
| Branch | B, BL, BR, BLR, RET, B.EQ, B.NE, B.GT, B.LT | Control flow and function calls |
| System | SVC (syscall), MRS, MSR, DMB, DSB, ISB | System calls, register access, barriers |
| SIMD/FP (NEON) | FADD, FMUL, FMADD, vector ops on V registers | Floating point and vector processing |

### 4.3 Load/Store Architecture Example

Let's see a concrete example. Suppose we want to compute `array[i] = array[i] + 5`:

```asm
// AArch64 Assembly
// X0 = base address of array
// X1 = index i

LSL  X2, X1, #2       // X2 = i * 4 (shift left by 2 = multiply by 4 for int32)
LDR  W3, [X0, X2]     // W3 = array[i] (load 32-bit word from memory)
ADD  W3, W3, #5        // W3 = W3 + 5 (operate on register only)
STR  W3, [X0, X2]     // array[i] = W3 (store back to memory)
```

Every single instruction is **32 bits wide** and does exactly one thing. This regularity is what makes the pipeline efficient.

Compare with x86 where a single instruction could do: load from memory, add, and store back. That requires the CPU to do three things in one instruction, making the decode logic much more complex.

### 4.4 NEON SIMD — Why It Matters for Vision

NEON is ARM's 128-bit SIMD (Single Instruction, Multiple Data) extension. It can process **multiple data elements in parallel**:

```
128-bit NEON register V0:
+--------+--------+--------+--------+
| 32-bit | 32-bit | 32-bit | 32-bit |   4x float32
| float  | float  | float  | float  |
+--------+--------+--------+--------+

OR:
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|u8|  16x uint8
+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
```

For image processing, 16 pixels (uint8) can be processed in a single NEON instruction. This gives up to **16x speedup** for operations like brightness adjustment, thresholding, and convolution.

OpenCV uses NEON intrinsics internally when compiled for ARM. This is why `cv2.cvtColor()` runs reasonably fast even on a Pi 5. NumPy also benefits from NEON acceleration for array operations.

**Practical impact on autonomous driving:**

Without NEON: Processing a 640x480 grayscale frame with a 3x3 convolution:
$$640 \times 480 \times 9 \text{ multiplications} = 2{,}764{,}800 \text{ scalar operations}$$

With NEON (processing 16 pixels at a time):
$$\frac{2{,}764{,}800}{16} = 172{,}800 \text{ NEON operations}$$

That is a **16x reduction** in instruction count, which directly translates to higher FPS.

---

## 5. Hands-On Lab

### 5.1 Prerequisites

- Raspberry Pi 5 (4GB or 8GB)
- microSD card (32GB+ recommended, Class 10 / A2)
- USB-C power supply (5V/5A, USB-PD capable — **this is mandatory for Pi 5**)
- Ethernet cable or Wi-Fi connection
- Host computer (Windows/Mac/Linux) for SSH

### 5.2 OS Installation and First Boot

**Step 1: Download and flash the OS**

Use the official Raspberry Pi Imager on your host computer:

```bash
# On your host machine (Linux example)
sudo apt install rpi-imager
rpi-imager
```

Select:
- **OS**: Raspberry Pi OS (64-bit, Bookworm) — we need 64-bit for AArch64
- **Storage**: Your microSD card
- **Settings** (click the gear icon):
  - Enable SSH (Use password authentication initially)
  - Set username: `pi` (or your preferred name)
  - Set password
  - Set hostname: `autocar` (makes SSH easier)
  - Configure WiFi if needed

**Step 2: First boot**

Insert the microSD card, connect Ethernet, and power on. Wait about 60 seconds for first boot to complete.

**Step 3: SSH connection**

```bash
# From your host computer
ssh pi@autocar.local

# If mDNS doesn't work, find the IP:
# Check your router's DHCP client list, or use:
nmap -sn 192.168.1.0/24  # Scan your subnet
```

**Step 4: Set up key-based SSH (more secure, no password prompts)**

```bash
# On your HOST computer, generate a key pair (if you don't have one)
ssh-keygen -t ed25519 -C "autocar-lab"
# Press Enter for default path (~/.ssh/id_ed25519)
# Optionally set a passphrase

# Copy the public key to the Pi
ssh-copy-id pi@autocar.local

# Now you can SSH without a password:
ssh pi@autocar.local
# It should log in directly!

# (Optional but recommended) Disable password auth on Pi for security:
# Edit /etc/ssh/sshd_config on the Pi:
#   PasswordAuthentication no
# Then: sudo systemctl restart sshd
```

### 5.3 Hardware Exploration Commands

Now let's explore the Pi 5 hardware from the command line. This builds deep intuition about what is actually inside.

**CPU Information:**

```bash
# View CPU architecture details
lscpu
```

Expected output (key fields):

```
Architecture:            aarch64
CPU op-mode(s):          32-bit, 64-bit
Byte Order:              Little Endian
CPU(s):                  4
On-line CPU(s) list:     0-3
Model name:              Cortex-A76
Thread(s) per core:      1
Core(s) per socket:      4
Socket(s):               1
Stepping:                r4p1
CPU max MHz:             2400.0000
CPU min MHz:             1500.0000
BogoMIPS:                108.00
L1d cache:               256 KiB    (4 instances, 64 KiB each)
L1i cache:               256 KiB    (4 instances, 64 KiB each)
L2 cache:                512 KiB    (1 instance)
L3 cache:                2 MiB      (1 instance)
```

Study questions: Confirm the cache sizes match our earlier architecture discussion. Note `aarch64` confirming 64-bit ARM. Note `Thread(s) per core: 1` — no Hyper-Threading on ARM (unlike Intel).

```bash
# Detailed per-core info
cat /proc/cpuinfo

# Memory info
free -h
# Shows total RAM, used, free, cached

cat /proc/meminfo | head -20
# Detailed memory statistics
```

**Storage:**

```bash
# Block device listing
lsblk
```

Expected output:
```
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
mmcblk0     179:0    0  29.7G  0 disk
|-mmcblk0p1 179:1    0   512M  0 part /boot/firmware
|-mmcblk0p2 179:2    0  29.2G  0 part /
```

`mmcblk0` is the microSD card. Note the two partitions:
- **p1** (512MB, FAT32): `/boot/firmware` — bootloader, kernel, device tree, config.txt
- **p2** (rest, ext4): `/` — root filesystem

**VideoCore (GPU) and system info:**

```bash
# GPU temperature
vcgencmd measure_temp

# CPU/GPU clock frequencies
vcgencmd measure_clock arm
vcgencmd measure_clock core

# Voltage
vcgencmd measure_volts core

# Memory split between CPU and GPU
vcgencmd get_mem arm
vcgencmd get_mem gpu

# Throttling status (important for thermal management!)
vcgencmd get_throttled
# 0x0 means no throttling -- good!
# Bits indicate: under-voltage, capped frequency, throttled, soft temp limit
```

**PCIe and RP1:**

```bash
# List PCIe devices
lspci
```

Expected output:
```
0000:00:00.0 PCI bridge: Broadcom Inc. BCM2712 PCIe Bridge (rev 21)
0000:01:00.0 Multimedia controller: Broadcom Inc. Device 1001
0001:00:00.0 PCI bridge: Broadcom Inc. BCM2712 PCIe Bridge (rev 21)
0001:01:00.0 Co-processor: Raspberry Pi Ltd RP1 Bar (rev 01)
```

You can see:
- **Bus 0000**: External PCIe slot (for NVMe or Hailo)
- **Bus 0001**: Internal PCIe link to RP1 southbridge

```bash
# More detailed PCIe info
lspci -v

# USB devices (connected through RP1)
lsusb
```

**Create a complete hardware report script:**

```bash
#!/bin/bash
# hw_report.sh -- Generate a comprehensive hardware report for Pi 5
# Usage: bash hw_report.sh > hw_report.txt

echo "========================================="
echo "   Raspberry Pi 5 Hardware Report"
echo "   Generated: $(date)"
echo "========================================="
echo ""

echo "--- CPU ---"
lscpu
echo ""

echo "--- Memory ---"
free -h
echo ""

echo "--- Storage ---"
lsblk
echo ""
df -h
echo ""

echo "--- Temperature ---"
vcgencmd measure_temp
echo ""

echo "--- Clock Speeds ---"
echo "ARM: $(vcgencmd measure_clock arm)"
echo "Core: $(vcgencmd measure_clock core)"
echo ""

echo "--- Voltages ---"
vcgencmd measure_volts core
echo ""

echo "--- Throttle Status ---"
vcgencmd get_throttled
echo ""

echo "--- PCIe Devices ---"
lspci
echo ""

echo "--- USB Devices ---"
lsusb
echo ""

echo "--- GPIO Chips ---"
gpiodetect
echo ""

echo "--- Kernel Version ---"
uname -a
echo ""

echo "--- OS Release ---"
cat /etc/os-release
echo ""

echo "--- Device Tree Model ---"
cat /proc/device-tree/model 2>/dev/null
echo ""

echo "========= End of Report ========="
```

Save and run:

```bash
chmod +x hw_report.sh
./hw_report.sh | tee hw_report.txt
```

### 5.4 GPIO Control with Python

Now let's control hardware. We will use `gpiozero` (which internally uses `libgpiod` on Pi 5).

**Install dependencies:**

```bash
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio python3-libgpiod
```

**Understanding the GPIO header:**

```
                    Raspberry Pi 5 GPIO Header
                    (40-pin, looking at the board from above)

        3V3  (1)  (2)  5V
  GPIO  2   (3)  (4)  5V
  GPIO  3   (5)  (6)  GND
  GPIO  4   (7)  (8)  GPIO 14  (UART TX)
        GND  (9) (10)  GPIO 15  (UART RX)
  GPIO 17  (11) (12)  GPIO 18
  GPIO 27  (13) (14)  GND
  GPIO 22  (15) (16)  GPIO 23
        3V3 (17) (18)  GPIO 24
  GPIO 10  (19) (20)  GND
  GPIO  9  (21) (22)  GPIO 25
  GPIO 11  (23) (24)  GPIO  8
        GND (25) (26)  GPIO  7
  GPIO  0  (27) (28)  GPIO  1
  GPIO  5  (29) (30)  GND
  GPIO  6  (31) (32)  GPIO 12
  GPIO 13  (33) (34)  GND
  GPIO 19  (35) (36)  GPIO 16
  GPIO 26  (37) (38)  GPIO 20
        GND (39) (40)  GPIO 21
```

All GPIO pins on Pi 5 are **3.3V logic**. Never connect a 5V signal directly!

**Lab 1: Blink an LED**

Circuit:
```
GPIO 17 (pin 11) ---- 330 ohm ---- LED anode(+) ---- LED cathode(-) ---- GND (pin 9)
```

Current limiting resistor calculation:

$$R = \frac{V_{\text{GPIO}} - V_{\text{LED}}}{I_{\text{LED}}} = \frac{3.3 - 2.0}{0.010} = 130 \, \Omega$$

We use 330 ohms to be safe (lower current, longer LED life, still visible):

$$I = \frac{3.3 - 2.0}{330} \approx 3.9 \text{ mA}$$

The Pi 5 GPIO can source up to about 8 mA per pin safely, so 3.9 mA is well within limits.

Python code:

```python
#!/usr/bin/env python3
"""
led_blink.py -- Blink an LED on GPIO 17
Demonstrates basic GPIO output on Raspberry Pi 5
"""

from gpiozero import LED
from time import sleep

# GPIO 17 corresponds to physical pin 11
led = LED(17)

print("Starting LED blink on GPIO 17...")
print("Press Ctrl+C to stop")

try:
    while True:
        led.on()
        print("LED ON")
        sleep(0.5)

        led.off()
        print("LED OFF")
        sleep(0.5)

except KeyboardInterrupt:
    print("\nStopping...")
    led.off()
    print("LED turned off. Done.")
```

Run it:

```bash
python3 led_blink.py
```

**Lab 2: Button Input with Interrupt**

Circuit:
```
GPIO 27 (pin 13) ---- Button ---- GND (pin 14)
(Internal pull-up enabled -- no external resistor needed)
```

```python
#!/usr/bin/env python3
"""
button_led.py -- Button controls LED with interrupt-driven input
GPIO 27: Button (active low, internal pull-up)
GPIO 17: LED output
"""

from gpiozero import LED, Button
from signal import pause

led = LED(17)
button = Button(27, pull_up=True, bounce_time=0.05)

def on_press():
    print("Button pressed! LED ON")
    led.on()

def on_release():
    print("Button released! LED OFF")
    led.off()

# Register event callbacks (interrupt-driven, not polling!)
button.when_pressed = on_press
button.when_released = on_release

print("Button-LED controller ready.")
print("Press the button to control the LED.")
print("Press Ctrl+C to exit.")

# Wait for events (low CPU usage -- interrupt-driven)
pause()
```

The key insight here: `gpiozero` uses **interrupts**, not polling. The CPU is not burning cycles constantly checking the pin state. Instead, the kernel wakes up your callback only when the pin state actually changes. This is crucial for battery-powered or multi-tasking systems.

**Lab 3: Using libgpiod directly (the low-level way)**

For cases where you need more control, or to understand what `gpiozero` does underneath:

```python
#!/usr/bin/env python3
"""
libgpiod_direct.py -- Direct libgpiod usage for GPIO control
This shows what happens under the hood on Raspberry Pi 5
"""

import gpiod
import time

# On Pi 5, GPIO is on the RP1 chip
# The gpiochip device for user-accessible GPIOs is typically gpiochip4
CHIP_PATH = "/dev/gpiochip4"
LED_PIN = 17

# Request the GPIO line
chip = gpiod.Chip(CHIP_PATH)

# Configure as output, initially low
led_config = gpiod.LineSettings(
    direction=gpiod.line.Direction.OUTPUT,
    output_value=gpiod.line.Value.INACTIVE
)

request = chip.request_lines(
    consumer="led-blink-demo",
    config={LED_PIN: led_config}
)

print(f"Using chip: {CHIP_PATH}")
print(f"Controlling GPIO {LED_PIN}")
print("Blinking LED... Ctrl+C to stop")

try:
    while True:
        request.set_value(LED_PIN, gpiod.line.Value.ACTIVE)
        time.sleep(0.5)
        request.set_value(LED_PIN, gpiod.line.Value.INACTIVE)
        time.sleep(0.5)
except KeyboardInterrupt:
    request.set_value(LED_PIN, gpiod.line.Value.INACTIVE)
    print("\nDone.")
finally:
    request.release()
    chip.close()
```

**Lab 4: Exploring GPIO chips**

```bash
# List all GPIO chips on the system
gpiodetect

# Expected output on Pi 5:
# gpiochip0 [gpio-brcmstb@107d508500] (32 lines)    <-- BCM2712 internal
# gpiochip1 [gpio-brcmstb@107d508520] (4 lines)      <-- BCM2712 internal
# gpiochip2 [gpio-brcmstb@107d517c00] (17 lines)     <-- BCM2712 internal
# gpiochip3 [gpio-brcmstb@107d517c20] (6 lines)      <-- BCM2712 internal
# gpiochip4 [pinctrl-rp1] (54 lines)                  <-- RP1 user-facing GPIO!

# Show all lines on the RP1 GPIO chip
gpioinfo gpiochip4

# Read the current value of a GPIO line
gpioget gpiochip4 17

# Set a GPIO output
gpioset gpiochip4 17=1   # HIGH
gpioset gpiochip4 17=0   # LOW
```

Notice how `gpiochip0` through `gpiochip3` are BCM2712 internal GPIOs, while `gpiochip4` is the RP1 southbridge — the one connected to the 40-pin header. This is the RP1 detour in action!

**Lab 5: PWM for LED brightness control**

```python
#!/usr/bin/env python3
"""
pwm_led.py -- PWM-based LED brightness control
Demonstrates pulse-width modulation on Pi 5
"""

from gpiozero import PWMLED
from time import sleep

led = PWMLED(17)

print("PWM LED brightness sweep (breathing effect)")
print("Ctrl+C to stop")

try:
    while True:
        # Fade in
        for brightness in range(0, 101, 5):
            led.value = brightness / 100.0
            sleep(0.03)

        # Fade out
        for brightness in range(100, -1, -5):
            led.value = brightness / 100.0
            sleep(0.03)

except KeyboardInterrupt:
    led.off()
    print("\nDone.")
```

PWM duty cycle determines the effective voltage seen by the LED:

$$V_{\text{effective}} = V_{\text{GPIO}} \times \frac{t_{\text{on}}}{t_{\text{on}} + t_{\text{off}}} = 3.3\text{V} \times \text{Duty Cycle}$$

At 50% duty cycle: \(V_{\text{eff}} = 3.3 \times 0.5 = 1.65\text{V}\) — the LED appears roughly half as bright.

At 25% duty cycle: \(V_{\text{eff}} = 3.3 \times 0.25 = 0.825\text{V}\) — dim but visible.

This PWM principle is exactly how we will control motor speed later in the series. Instead of an LED, the PWM signal will drive a motor driver IC, and the duty cycle will control how fast the wheels spin.

---

## 6. Review

### Key Concepts Checklist

1. **MCU vs MPU vs SoC**: An MCU has everything on-chip (for real-time, low-power). An MPU needs external memory (for heavy compute). An SoC integrates MPU + GPU + controllers (best of both worlds for embedded Linux).

2. **BCM2712**: Quad Cortex-A76 at 2.4 GHz, VideoCore VII GPU, LPDDR4X-4267 memory controller, PCIe Gen 2 controller.

3. **RP1 Southbridge**: A separate chip designed by Raspberry Pi handling all I/O (GPIO, USB, Ethernet, MIPI). Connected to BCM2712 via internal PCIe x4. This is why `RPi.GPIO` does not work and `libgpiod` is required.

4. **Cortex-A76 Pipeline**: 4-wide fetch, 4-wide decode, 8-wide dispatch, out-of-order execution, ~128-entry ROB. 13-stage pipeline.

5. **Cache Hierarchy**: L1I/L1D 64KB each per core (~4 cycles), L2 512KB shared (~9 cycles), L3 2MB shared (~30 cycles). DRAM: 100+ cycles.

6. **RISC Philosophy**: Fixed-width instructions, load/store architecture, simple decode, low power per operation. ARM dominates mobile and embedded because of superior performance per watt.

### Architecture Diagram Quiz

**Q1:** Trace the path of a GPIO write from your Python code to the physical pin. How many chips does the signal cross?

**Answer:** Python `gpiozero` -> `libgpiod` -> Linux kernel GPIO subsystem -> RP1 PCIe driver -> BCM2712 PCIe controller -> PCIe link -> RP1 southbridge -> RP1 GPIO controller -> physical pin. The signal crosses **two chips**: BCM2712 and RP1, connected via PCIe.

**Q2:** Why does Pi 5 use a separate RP1 chip instead of integrating I/O into BCM2712?

**Answer:**
1. BCM2712 is manufactured by Broadcom — Raspberry Pi has limited control over its peripheral set.
2. RP1 is designed by Raspberry Pi themselves, giving them full control over GPIO, USB, CSI, DSI features.
3. Separating I/O means future Pi versions can upgrade the CPU SoC without redesigning I/O.
4. The PCIe link provides high bandwidth (16 Gbit/s) so USB 3.0 and Ethernet no longer bottleneck each other.

**Q3:** A self-driving car needs to read a wheel encoder at exactly 10 kHz with zero jitter. Should you use a Cortex-A76 (Pi 5) or a Cortex-M4 (STM32)?

**Answer:** **Cortex-M4 (STM32)**. The Cortex-A76 runs Linux, which is not a real-time OS. Process scheduling, interrupts, and cache misses cause unpredictable latency (jitter). A Cortex-M4 running bare-metal or FreeRTOS can guarantee deterministic interrupt response in microseconds. In a real autonomous car, the M4 reads the encoder and sends the data to the A76 via CAN or UART.

---

## Next: Day 2

Tomorrow we go deeper into the **software** side: the Linux boot sequence from EEPROM to systemd, the filesystem hierarchy, process management, and shell scripting. We will write our first systemd service to auto-start our autonomous car software on boot.

See you in [Day 2 -- Linux Fundamentals and Boot Sequence](/posts/embedded-day-02/).
