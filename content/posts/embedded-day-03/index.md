---
title: "Day 3 — Electronics Basics, UART Debug Console, and GPIO"
date: 2026-03-05
description: "Ohm's law, voltage dividers, pull-up/pull-down resistors, RPi 5 power design, and UART debug console access"
categories: ["Autonomous Driving"]
tags: ["Electronics", "UART", "GPIO", "Voltage Divider", "Debug Console"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 3
draft: false
---

{{< katex >}}

## What You'll Learn

On Day 1 we understood the hardware architecture. On Day 2 we mastered the software boot sequence. Today we bridge the gap: **electronics fundamentals** that every embedded engineer must know, and the **UART debug console** — the most powerful debugging tool you will ever use.

By the end of this post, you will:

- Apply Ohm's law, calculate voltage dividers, and size current-limiting resistors
- Understand pull-up/pull-down resistors and why they prevent floating pins
- Know the RPi 5 power design: 5V/5A USB-C PD requirements and power budgeting
- Connect a UART debug cable and observe the complete boot sequence live
- Read circuit diagrams and understand decoupling capacitors
- Handle GPIO interrupts in Python for event-driven programming

---

## 1. Ohm's Law — The Foundation of Everything

### 1.1 The Three Fundamental Quantities

Every electrical circuit involves three quantities:

- **Voltage (V)**: The electrical "pressure" that pushes electrons through a circuit. Measured in Volts (V). Think of it as water pressure in a pipe.
- **Current (I)**: The flow of electrons. Measured in Amperes (A). Think of it as the flow rate of water.
- **Resistance (R)**: Opposition to current flow. Measured in Ohms (\(\Omega\)). Think of it as pipe diameter — narrow pipe = high resistance.

**Ohm's Law** relates all three:

$$V = I \times R$$

Or equivalently:

$$I = \frac{V}{R} \qquad R = \frac{V}{I}$$

### 1.2 Practical Examples

**Example 1: LED current limiting resistor**

An LED requires a specific forward current (typically 10-20 mA) and has a forward voltage drop (typically 1.8-2.2V for red, 3.0-3.3V for blue/white).

For a red LED connected to RPi 5 GPIO (3.3V output):

```
GPIO pin (3.3V) ---[ R ]---[LED]--- GND
                         Vf = 2.0V
```

The resistor must drop the remaining voltage:

$$V_R = V_{\text{GPIO}} - V_{\text{LED}} = 3.3 - 2.0 = 1.3 \text{ V}$$

For 10 mA target current:

$$R = \frac{V_R}{I} = \frac{1.3}{0.010} = 130 \, \Omega$$

Standard resistor values: 120, 150, 220, 330. We typically choose **220 or 330 ohms** for safety margin:

$$I_{220\Omega} = \frac{1.3}{220} = 5.9 \text{ mA} \qquad I_{330\Omega} = \frac{1.3}{330} = 3.9 \text{ mA}$$

Both values produce visible light while staying well within the GPIO's current limit (~8 mA per pin on RPi 5).

**Example 2: Power dissipation in a resistor**

Every resistor converts electrical energy to heat. The power dissipated is:

$$P = V \times I = I^2 \times R = \frac{V^2}{R}$$

For our 330-ohm resistor with 3.9 mA:

$$P = (0.0039)^2 \times 330 = 0.005 \text{ W} = 5 \text{ mW}$$

Standard 1/4W (250 mW) resistors can handle this easily. But if you are driving a motor with 2A through a 0.5-ohm sense resistor:

$$P = (2)^2 \times 0.5 = 2 \text{ W}$$

You would need a **2W or 5W power resistor**. This is why motor drivers use dedicated current sense chips instead of simple resistors.

### 1.3 Series and Parallel Resistors

**Series** (resistors in a line — currents are equal, voltages add):

$$R_{\text{total}} = R_1 + R_2 + R_3 + \ldots$$

**Parallel** (resistors side by side — voltages are equal, currents add):

$$\frac{1}{R_{\text{total}}} = \frac{1}{R_1} + \frac{1}{R_2} + \frac{1}{R_3} + \ldots$$

For two parallel resistors, the shortcut formula:

$$R_{\text{total}} = \frac{R_1 \times R_2}{R_1 + R_2}$$

**Why this matters:** In a voltage divider (next section), two resistors in series create an intermediate voltage.

---

## 2. Voltage Dividers

### 2.1 The Voltage Divider Formula

A voltage divider is two resistors in series that produce an output voltage proportional to the input:

```
Vin ----[R1]----+----[R2]---- GND
                |
              Vout
```

The output voltage is:

$$V_{\text{out}} = V_{\text{in}} \times \frac{R_2}{R_1 + R_2}$$

**Derivation (intuitive):** The same current flows through both resistors (series circuit):

$$I = \frac{V_{\text{in}}}{R_1 + R_2}$$

The voltage across R2 (which is our output) is:

$$V_{\text{out}} = I \times R_2 = V_{\text{in}} \times \frac{R_2}{R_1 + R_2}$$

### 2.2 Practical Example: Level Shifting 5V to 3.3V

Many sensors (Arduino, certain ultrasonic modules) output 5V logic, but the RPi 5 GPIO is **3.3V tolerant only**. Connecting 5V directly to a GPIO pin can **damage the Pi permanently**.

Solution: a voltage divider to step down 5V to ~3.3V.

```
5V sensor output ---[R1=1.8k]----+----[R2=3.3k]---- GND
                                 |
                           To RPi GPIO input
                           (should be ~3.2V)
```

$$V_{\text{out}} = 5.0 \times \frac{3300}{1800 + 3300} = 5.0 \times \frac{3300}{5100} = 3.24 \text{ V}$$

This is safely within the 3.3V logic threshold. The Pi reads this as a logic HIGH.

**Important caveat:** Voltage dividers are fine for slow signals (a few kHz). For fast signals (SPI at MHz speeds), the resistors form an RC filter with parasitic capacitance and distort the signal. For high-speed level shifting, use a dedicated level shifter IC (like the TXS0108E).

### 2.3 Voltage Divider for Analog Sensing

Voltage dividers are also used to measure battery voltage with an ADC. If your car battery is 12V but your ADC only accepts 0-3.3V:

$$\frac{R_2}{R_1 + R_2} = \frac{3.3}{12} = 0.275$$

Choose R2 = 10k, then:

$$R_1 = R_2 \times \left(\frac{V_{\text{in}}}{V_{\text{out}}} - 1\right) = 10000 \times \left(\frac{12}{3.3} - 1\right) = 10000 \times 2.636 = 26.4 \text{ k}\Omega$$

Use a standard 27k resistor. The conversion formula in software:

$$V_{\text{battery}} = V_{\text{ADC}} \times \frac{R_1 + R_2}{R_2} = V_{\text{ADC}} \times \frac{27000 + 10000}{10000} = V_{\text{ADC}} \times 3.7$$

---

## 3. Pull-Up and Pull-Down Resistors

### 3.1 The Floating Pin Problem

A GPIO input pin that is not connected to anything is "floating" — its voltage is undefined and can randomly fluctuate between HIGH and LOW due to electromagnetic interference, capacitive coupling, and thermal noise.

```
Floating input -- UNRELIABLE:
                    +---------+
   Nothing ----?----| GPIO    |  Reads random values!
                    | Input   |  Could be 0 or 1
                    +---------+
```

This is a real problem: a floating input on a motor controller could cause the motor to randomly turn on and off. In an autonomous car, that is catastrophic.

### 3.2 Pull-Up Resistor

A **pull-up resistor** connects the input to VCC (3.3V) through a resistor. The default state is HIGH. A button or switch grounds the pin to make it LOW.

```
    3.3V
     |
    [R] 10k (pull-up)
     |
     +---------- GPIO Input
     |
   [Button]
     |
    GND
```

- **Button open (not pressed):** GPIO is pulled to 3.3V through the resistor. Reads **HIGH (1)**.
- **Button pressed:** GPIO is connected directly to GND (low impedance path wins). Reads **LOW (0)**.

The resistor value matters:
- **Too low** (100 ohms): Wastes current when button is pressed: \(I = 3.3/100 = 33 \text{ mA}\). Bad for battery life.
- **Too high** (1M ohm): Weak pull-up. Susceptible to noise — the pin might not reliably read HIGH.
- **Sweet spot**: 4.7k to 10k for most applications.

$$I_{\text{button pressed}} = \frac{3.3}{10000} = 0.33 \text{ mA}$$

That is negligible power consumption, yet strong enough to overcome noise.

### 3.3 Pull-Down Resistor

A **pull-down resistor** connects the input to GND through a resistor. The default state is LOW.

```
    3.3V
     |
   [Button]
     |
     +---------- GPIO Input
     |
    [R] 10k (pull-down)
     |
    GND
```

- **Button open:** GPIO is pulled to GND. Reads **LOW (0)**.
- **Button pressed:** GPIO is connected to 3.3V. Reads **HIGH (1)**.

### 3.4 Internal Pull-Up/Down on RPi 5

The RP1 GPIO controller has **built-in configurable pull-up and pull-down resistors** (approximately 50k-65k ohms). You can enable them in software:

```python
from gpiozero import Button

# Enable internal pull-up (default for Button)
button = Button(27, pull_up=True)
# No external resistor needed!

# Enable internal pull-down
button = Button(27, pull_up=False)

# Disable internal pull (floating -- only if you have external pull)
button = Button(27, pull_up=None)
```

**Internal vs External pull-ups:**

| Feature | Internal (RP1) | External |
|---------|---------------|----------|
| Resistance | ~50-65k | You choose (4.7k-10k typical) |
| Noise immunity | Moderate | Better (lower R = stronger pull) |
| Convenience | No extra components | Requires resistor on PCB |
| Current draw | ~50-65 uA | Higher (but still small) |
| Use case | Prototyping, short wires | Production, long wires, noisy environments |

**For our autonomous car:** Use internal pull-ups for prototyping. When we move to a custom PCB, use external 4.7k pull-ups for better noise immunity, especially for signals near motors (electrically noisy).

---

## 4. RPi 5 Power Design

### 4.1 Why 5V/5A USB-C PD?

The Raspberry Pi 5 requires a **5V/5A (25W) USB-C Power Delivery** supply. This is a significant step up from Pi 4's 5V/3A. Why?

**Power budget breakdown:**

| Component | Typical Power | Peak Power |
|-----------|--------------|------------|
| BCM2712 SoC (CPU + GPU) | 3-5W | 8W (all cores loaded) |
| LPDDR4X RAM (4GB) | 0.5W | 0.8W |
| RP1 Southbridge | 0.5W | 1W |
| PCIe devices (NVMe/Hailo) | 1-3W | 5W |
| USB devices (camera, etc.) | 0.5-2W | 4.5W (USB 3.0 ports) |
| Fan/cooling | 0.2W | 0.5W |
| GPIO peripherals | 0.1W | 0.5W |
| **Total** | **~6-12W** | **~20W** |

The 5V/5A supply provides 25W, which gives headroom for peak loads plus connected peripherals.

### 4.2 USB Power Delivery Negotiation

Unlike simple USB chargers, the Pi 5 uses **USB Power Delivery (USB-PD)** protocol to negotiate the power it needs:

1. Pi 5 power controller detects USB-C connection
2. Sends PD request for 5V/5A (25W)
3. If the supply supports it, power is granted at 5A
4. If the supply only supports 5V/3A, the Pi boots but:
   - **Limits USB port current** to 600 mA total (instead of 1.6A)
   - **Disables USB peripherals** power if total draw is too high
   - You may see **voltage warnings** and **throttling**

```bash
# Check if full power is available
vcgencmd get_throttled
# 0x0 = good (no issues)
# Bit 0 (0x1) = under-voltage detected
# Bit 1 (0x2) = frequency capped
# Bit 2 (0x4) = currently throttled

# If you see the lightning bolt icon on screen, your power supply is inadequate!
```

**For autonomous driving:** Always use a proper 5V/5A PD supply. Under-voltage causes clock throttling, which means your perception stack drops frames. In the field, use a regulated 5V/5A DC-DC converter from the car's 12V battery.

### 4.3 Power Budget Calculation for Our Autonomous Car

Let's plan the power for a complete autonomous car setup:

```
Power Source: 12V LiPo Battery (3S, 5000mAh)
                |
        [Buck Converter: 12V -> 5V/5A]
                |
         Raspberry Pi 5 (5V input)
                |
    +-----------+-----------+-----------+
    |           |           |           |
  BCM2712    Camera     Hailo-8L     Servos
  + RP1      (USB)      (PCIe)      (separate
  + RAM                              power!)
  ~5W        ~0.5W       ~3W
```

Total Pi power draw: ~5W (idle) to ~12W (loaded with camera + Hailo)

Battery life calculation:

$$\text{Battery energy} = 12\text{V} \times 5\text{Ah} = 60 \text{ Wh}$$

$$\text{Runtime at 12W} = \frac{60}{12} \times \eta_{\text{converter}} = \frac{60}{12} \times 0.90 = 4.5 \text{ hours}$$

Where \(\eta_{\text{converter}}\) is the DC-DC converter efficiency (typically 85-95%).

$$\text{Runtime at 8W (typical)} = \frac{60}{8} \times 0.90 = 6.75 \text{ hours}$$

### 4.4 Decoupling Capacitors

Every IC in a circuit needs **decoupling capacitors** (also called bypass capacitors) placed physically close to the power pins.

**Why?** When a digital IC switches its transistors, it draws a sudden spike of current. The power supply and PCB traces have inductance, which resists sudden current changes (Lenz's law):

$$V = L \frac{dI}{dt}$$

A sudden \(dI/dt\) creates a voltage dip on the power rail. If the dip is large enough, the IC sees a momentary under-voltage and can malfunction (bit errors, resets, glitches).

A **decoupling capacitor** acts as a local energy reservoir. It provides the instantaneous current spike while the power supply catches up.

**Typical values:**
- 100 nF (0.1 uF) ceramic capacitor: Handles high-frequency transients (MHz)
- 10 uF ceramic or tantalum: Handles lower-frequency bulk decoupling
- Often both are placed in parallel near each IC

**On the Pi 5 board:** If you look closely at the PCB (or the schematic), you will see dozens of small brown/tan components near the BCM2712 and RP1 chips — those are decoupling capacitors.

### 4.5 How to Read Basic Circuit Diagrams

Here are the essential schematic symbols you will encounter:

```
Resistor:     ---[####]--- or ---/\/\/---

Capacitor:    ---|  |---   (ceramic/film)
              ---|(---     (polarized/electrolytic)

LED:          --->|---     (arrow points in current flow direction)

Diode:        ---|>|---    (current flows in arrow direction)

Button/SW:    ---/ ---     (open = no connection)

Ground:       ---+---
                 |
                ===

Power:        VCC or 3V3 or 5V (with a line on top)

GPIO pin:     labeled with pin name, e.g., "GPIO17" or "TXD0"

Transistor:   NPN:  B---\     PNP:  B---\
(MOSFET/BJT)        E   C          E   C
```

**Example: LED circuit with button control (complete schematic)**

```
           3V3
            |
           [10k] R1 (pull-up)
            |
GPIO27 -----+------ [Button] ---- GND
            |
          (input)

GPIO17 ----[330R]----[LED]---- GND
          (output)    anode cathode
```

Reading this schematic:
1. GPIO27 is an input with a 10k pull-up to 3.3V
2. When button is open: GPIO27 reads HIGH (3.3V through R1)
3. When button is pressed: GPIO27 reads LOW (connected to GND)
4. GPIO17 drives an LED through a 330-ohm current limiting resistor
5. Current flows: GPIO17 -> R -> LED -> GND

---

## 5. UART Debug Console

### 5.1 What Is UART?

**UART (Universal Asynchronous Receiver-Transmitter)** is the simplest serial communication protocol. It uses two wires for bidirectional communication:

- **TX (Transmit)**: Data output
- **RX (Receive)**: Data input
- **GND**: Common ground reference

```
Device A                    Device B
+--------+                  +--------+
|     TX |---------->-------| RX     |
|     RX |-------<----------| TX     |
|    GND |------------------| GND    |
+--------+                  +--------+
         Note: TX connects to RX (crossover)
```

Key parameters:
- **Baud rate**: Speed in bits per second (common: 9600, 115200)
- **Data bits**: Usually 8
- **Parity**: Usually None
- **Stop bits**: Usually 1
- Written as: **115200 8N1** (115200 baud, 8 data bits, No parity, 1 stop bit)

### 5.2 UART Frame Format

Each byte transmitted is wrapped in a frame:

```
Idle _____|     |_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____
          |Start| D0  | D1  | D2  | D3  | D4  | D5  | D6  | D7  |Stop |
          | Bit |     |     |     |     |     |     |     |     | Bit |
          |  0  | LSB                                       MSB |  1  |
          |<--->|<------------- 8 Data Bits ------------------>|<--->|
```

- **Idle**: Line is HIGH (mark state)
- **Start bit**: Line goes LOW for one bit period (signals start of data)
- **Data bits**: 8 bits, LSB first
- **Stop bit**: Line goes HIGH for one bit period
- **Total**: 10 bit periods per byte (1 start + 8 data + 1 stop)

**Timing calculation at 115200 baud:**

$$T_{\text{bit}} = \frac{1}{115200} = 8.68 \, \mu\text{s}$$

$$T_{\text{byte}} = 10 \times T_{\text{bit}} = 86.8 \, \mu\text{s}$$

$$\text{Throughput} = \frac{8 \text{ data bits}}{10 \text{ total bits}} \times 115200 = 92{,}160 \text{ bytes/s} \approx 90 \text{ KB/s}$$

### 5.3 Why UART Debug Console Is Essential

The UART debug console gives you a **direct terminal connection** to the Pi that:

1. **Works before Linux boots**: You see EEPROM bootloader messages, GPU firmware messages, and early kernel output
2. **Works when SSH fails**: If the network is misconfigured, SSH is broken, or the GUI crashes, UART still works
3. **Works during kernel panics**: When the kernel crashes, the panic message goes to UART
4. **Has no dependencies**: No network, no display, no USB — just two wires and a ground

**In autonomous driving development:** When your car's Pi crashes in the field, you cannot SSH into it. But if you have a UART debug cable, you can plug in a laptop and see exactly what happened.

### 5.4 Hardware Setup: UART Debug Cable

You need a **USB-to-UART adapter** (also called USB-to-TTL serial cable). Popular options:
- FTDI FT232RL based cable
- CP2102 based adapter
- Raspberry Pi official Debug Probe

**CRITICAL: The adapter must be 3.3V logic level, NOT 5V!** A 5V UART signal will damage the Pi 5.

**Wiring:**

```
USB-to-UART Adapter          Raspberry Pi 5 GPIO Header
+------------------+         +------------------+
|                  |         |                  |
|   GND (Black) ---|---------|--- GND  (Pin 6)  |
|   TXD (Green) ---|---------|--- RXD  (Pin 10, GPIO15) |
|   RXD (White) ---|---------|--- TXD  (Pin 8,  GPIO14) |
|                  |         |                  |
|   VCC (Red)  --- DO NOT CONNECT (Pi has its own power!) |
|                  |         |                  |
+------------------+         +------------------+

    IMPORTANT: TX -> RX crossover!
    Adapter TX connects to Pi RX (Pin 10)
    Adapter RX connects to Pi TX (Pin 8)
    NEVER connect VCC -- the Pi is powered by USB-C
```

**GPIO header pin reference for UART:**

```
        (Pin 1) 3V3    5V (Pin 2)
        (Pin 3) GPIO2  5V (Pin 4)
        (Pin 5) GPIO3  GND (Pin 6)  <-- Connect GND here
        (Pin 7) GPIO4  GPIO14 (Pin 8)  <-- TXD (Pi sends data)
        (Pin 9) GND    GPIO15 (Pin 10) <-- RXD (Pi receives data)
```

### 5.5 Software Setup: Using minicom

On your **host computer** (the one with the USB adapter plugged in):

**Linux:**

```bash
# Install minicom
sudo apt install minicom

# Find the serial device
ls /dev/ttyUSB* /dev/ttyACM*
# Usually /dev/ttyUSB0 for FTDI/CP2102 adapters

# Connect to the Pi's UART console
minicom -b 115200 -D /dev/ttyUSB0

# Minicom controls:
# Ctrl+A then X  = Exit minicom
# Ctrl+A then Z  = Help menu
# Ctrl+A then L  = Log session to file
# Ctrl+A then E  = Local echo toggle
```

**macOS:**

```bash
# Install minicom via Homebrew
brew install minicom

# Find the device
ls /dev/tty.usbserial* /dev/tty.usbmodem*

# Connect
minicom -b 115200 -D /dev/tty.usbserial-XXXX
```

**Windows:**

Use **PuTTY** or **TeraTerm**:
1. Open Device Manager -> Ports (COM & LPT) -> Find the COM port (e.g., COM3)
2. In PuTTY: Connection type = Serial, Serial line = COM3, Speed = 115200

### 5.6 Pi 5 Configuration for UART Console

**Step 1: Enable UART in config.txt**

```bash
# On the Pi (via SSH or SD card editing)
sudo nano /boot/firmware/config.txt

# Add or verify this line:
enable_uart=1
```

**Step 2: Ensure kernel console output goes to UART**

Check `cmdline.txt`:

```bash
cat /boot/firmware/cmdline.txt
# Should contain: console=serial0,115200
# If "quiet" is present, remove it to see all boot messages
```

**Step 3: Reboot and watch**

After connecting the UART cable and opening minicom on your host, reboot the Pi:

```bash
sudo reboot
```

You will see the **complete boot sequence** in your minicom terminal.

### 5.7 What You Will See: Boot Log Walkthrough

Here is what the UART output looks like at each stage:

**Stage 1 — EEPROM Bootloader:**

```
RPi: BOOTLOADER release VERSION:xxx DATE
board: xxx xxx xxx
boot: order  Try SD FIRST
SD: xxxxx
SD: type A2 bus-width 4 clock 100000000
```

**Stage 2 — GPU Firmware (start4.elf):**

```
Net: no ethernet found.
start4.elf
MESS:00:00:04.123456 ...
MESS:00:00:04.234567 ...
```

**Stage 3 — Linux Kernel:**

```
[    0.000000] Booting Linux on physical CPU 0x0000000000 [0x410fd034]
[    0.000000] Linux version 6.6.xx-v8-16k+ (gcc version 12.2.0)
[    0.000000] Machine model: Raspberry Pi 5 Model B Rev 1.0
[    0.000000] Memory: 8192MB
[    0.000000] Zone ranges:
[    0.001234] GIC: Using split EOI/Deactivate mode
[    0.002345] pci 0001:01:00.0: [1de4:0001] type 00 class 0x0b4000
[    0.003456] rp1 0001:01:00.0: RP1 detected
[    0.100000] EXT4-fs (mmcblk0p2): mounted filesystem
[    0.200000] Freeing unused kernel memory: 2048K
[    0.250000] Run /sbin/init as init process
```

**Stage 4 — systemd:**

```
[    0.300000] systemd[1]: System time before build time, advancing clock.
[    0.350000] systemd[1]: Started Journal Service.
[    1.000000] systemd[1]: Started SSH Server.
[    2.000000] systemd[1]: Reached target Multi-User System.

Raspberry Pi OS GNU/Linux 12 autocar ttyAMA0

autocar login:
```

You can log in directly through the UART console — it is a full terminal. This is invaluable for debugging boot failures.

### 5.8 UART for Boot Log Stage Analysis

Create a script that captures and timestamps the UART boot log:

```bash
# On your host computer, capture boot log to a file
# (Run this BEFORE rebooting the Pi)
minicom -b 115200 -D /dev/ttyUSB0 -C boot_log_$(date +%Y%m%d).txt

# After the Pi finishes booting, press Ctrl+A then X to exit
# Analyze the log:
grep -E "^\[" boot_log_*.txt | head -50
# Shows kernel messages with timestamps
```

### 5.9 EEPROM Boot UART Configuration

For the most verbose debugging, enable UART output from the EEPROM bootloader itself:

```bash
# On the Pi:
sudo rpi-eeprom-config --edit

# Add or change:
BOOT_UART=1
```

Now you will see output from the very first moment of the boot process — even before the GPU firmware loads.

---

## 6. GPIO Deep Dive — Interrupt-Driven Programming

### 6.1 Polling vs Interrupts

There are two ways to read a GPIO input:

**Polling (bad for real-time):**
```python
# Polling: CPU constantly checks the pin
while True:
    if button.is_pressed:
        handle_press()
    time.sleep(0.01)  # 10ms polling interval
    # Problem: 10ms latency, wastes CPU even when nothing happens
```

**Interrupts (good for real-time):**
```python
# Interrupt: CPU is notified immediately when pin changes
button.when_pressed = handle_press
pause()  # CPU sleeps, wakes only on interrupt
# Latency: sub-millisecond, zero CPU usage when idle
```

**Why interrupts matter for autonomous driving:**

Consider a wheel encoder producing 1000 pulses per revolution at 600 RPM:

$$\text{Pulse frequency} = \frac{1000 \times 600}{60} = 10{,}000 \text{ Hz}$$

$$T_{\text{pulse}} = \frac{1}{10000} = 100 \, \mu\text{s}$$

Polling at 10ms intervals would miss most pulses! Only interrupt-driven input can capture all 10,000 pulses per second.

### 6.2 Edge Detection: Rising, Falling, Both

GPIO interrupts can trigger on specific signal transitions:

```
        3.3V  _____       _____       _____
             |     |     |     |     |     |
             |     |     |     |     |     |
   0V  _____|     |_____|     |_____|     |_____

             ^     ^     ^     ^     ^     ^
             |     |     |     |     |     |
          RISING  FALLING  RISING  FALLING  RISING FALLING
           edge    edge    edge    edge     edge    edge
```

- **Rising edge**: Signal goes from LOW to HIGH (0 -> 1)
- **Falling edge**: Signal goes from HIGH to LOW (1 -> 0)
- **Both edges**: Trigger on any transition

### 6.3 Debouncing

Mechanical buttons do not produce clean transitions. When pressed, the metal contacts bounce rapidly for a few milliseconds, producing multiple false edges:

```
Ideal button press:          Real button press (bouncing):
                              ___   _   __
   _____|                    |   | | | |  |
        |_________           |   |_| |_|  |___________
                              ^^^^^^^
                              Bounce zone (~1-10ms)
```

Without debouncing, one button press could trigger your interrupt 5-10 times!

**Software debouncing in gpiozero:**

```python
# bounce_time parameter filters out bounces shorter than 50ms
button = Button(27, pull_up=True, bounce_time=0.05)
```

**Hardware debouncing:** Add a 100nF capacitor between the GPIO pin and GND. This creates an RC filter:

$$\tau = R \times C = 10000 \times 100 \times 10^{-9} = 1 \text{ ms}$$

The capacitor smooths out bounces shorter than about \(3\tau = 3\) ms.

```
    3.3V
     |
    [10k] pull-up
     |
     +---------- GPIO Input
     |        |
   [Button] [100nF]  (debounce capacitor)
     |        |
    GND      GND
```

### 6.4 Complete GPIO Interrupt Example

```python
#!/usr/bin/env python3
"""
gpio_interrupts.py -- Comprehensive GPIO interrupt handling
Demonstrates rising edge, falling edge, and both-edge detection.
"""

from gpiozero import Button, LED, PWMLED
from signal import pause
from datetime import datetime
import time

# Hardware setup
led = LED(17)                # Indicator LED
pwm_led = PWMLED(18)         # Brightness-controlled LED
button = Button(27, pull_up=True, bounce_time=0.05)

# State tracking
press_count = 0
last_press_time = 0

def on_button_press():
    """Called on falling edge (button pressed = GPIO goes LOW)."""
    global press_count, last_press_time

    now = time.time()
    press_count += 1
    interval = now - last_press_time if last_press_time > 0 else 0
    last_press_time = now

    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] PRESSED  (#{press_count}, interval: {interval:.3f}s)")

    led.on()

def on_button_release():
    """Called on rising edge (button released = GPIO goes HIGH)."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    hold_duration = time.time() - last_press_time
    print(f"[{timestamp}] RELEASED (held for {hold_duration:.3f}s)")

    led.off()

    # Long press = special action
    if hold_duration > 2.0:
        print("  -> LONG PRESS detected! Triggering LED fade...")
        for i in range(0, 101, 10):
            pwm_led.value = i / 100.0
            time.sleep(0.05)
        for i in range(100, -1, -10):
            pwm_led.value = i / 100.0
            time.sleep(0.05)

# Register interrupt handlers
button.when_pressed = on_button_press
button.when_released = on_button_release

print("GPIO Interrupt Demo")
print("===================")
print("GPIO 27: Button (pull-up, active LOW)")
print("GPIO 17: LED (on when button pressed)")
print("GPIO 18: PWM LED (fades on long press > 2s)")
print()
print("Press Ctrl+C to exit")
print()

# Main thread sleeps -- all action happens in interrupt callbacks
pause()
```

### 6.5 Multimeter Usage Guide

A **multimeter** is essential for debugging hardware. Here are the key measurements:

**Measuring Voltage (Voltmeter mode):**

```
Multimeter set to V (DC):
  Red probe (+) --> Point you want to measure
  Black probe (-) --> GND

Typical measurements:
  - GPIO HIGH:    3.3V (should be 3.0-3.3V)
  - GPIO LOW:     0V   (should be < 0.3V)
  - 5V rail:      5.0V (should be 4.8-5.2V)
  - LED Vf:       ~2.0V (measure across LED while it's on)
```

**Measuring Current (Ammeter mode):**

```
IMPORTANT: Multimeter must be IN SERIES with the circuit!
Never connect ammeter probes across a voltage source (short circuit!)

   GPIO ---[330R]---[LED]--- Multimeter (A mode) --- GND
                               ^
                          Measure current here

Expected: ~4 mA for a typical LED through 330 ohms
```

**Measuring Resistance (Ohmmeter mode):**

```
IMPORTANT: Power must be OFF when measuring resistance!

  Red probe --> One end of resistor
  Black probe --> Other end of resistor

Verify: 330 ohm resistor should read 320-340 ohms (5% tolerance)
Color bands: Orange-Orange-Brown-Gold = 330 ohm +/- 5%
```

**Continuity test (beep mode):**

Most useful for finding broken wires, verifying solder joints, and checking that your ground connections are solid. If the circuit is complete, the multimeter beeps.

---

## 7. Hands-On Lab

### 7.1 Lab 1: UART Debug Console Setup

**Materials needed:**
- USB-to-UART adapter (3.3V logic!)
- 3 jumper wires (female-to-female)

**Steps:**

1. Connect the adapter to the Pi (power OFF):
   - Adapter GND -> Pi Pin 6 (GND)
   - Adapter TXD -> Pi Pin 10 (GPIO15/RXD)
   - Adapter RXD -> Pi Pin 8 (GPIO14/TXD)
   - Do NOT connect VCC

2. Plug the USB adapter into your host computer

3. Open minicom on the host:
```bash
minicom -b 115200 -D /dev/ttyUSB0
```

4. Power on the Pi (plug in USB-C)

5. Watch the boot messages scroll by!

6. After boot completes, log in through the UART console

7. Capture a complete boot log:
```bash
# Start capture before rebooting
minicom -b 115200 -D /dev/ttyUSB0 -C uart_boot_log.txt
# Then on another terminal (or the Pi itself):
sudo reboot
```

### 7.2 Lab 2: Boot Log Analysis

After capturing the boot log, analyze each stage:

```bash
# Count messages per boot stage
echo "=== EEPROM messages ==="
grep -c "RPi:\|board:\|boot:" uart_boot_log.txt

echo "=== GPU firmware messages ==="
grep -c "MESS:\|start4" uart_boot_log.txt

echo "=== Kernel messages ==="
grep -c "^\[" uart_boot_log.txt

echo "=== systemd messages ==="
grep -c "systemd" uart_boot_log.txt

# Find the RP1 detection
grep -i "rp1" uart_boot_log.txt

# Find PCIe enumeration
grep -i "pci" uart_boot_log.txt

# Find any errors or warnings
grep -iE "error|fail|warn" uart_boot_log.txt
```

### 7.3 Lab 3: Voltage Divider Circuit

Build a voltage divider and verify with a multimeter:

```
5V (Pin 2) ---[1.8k]---+---[3.3k]--- GND (Pin 6)
                        |
                    Measure here
                    (should be ~3.24V)
```

**Steps:**
1. Connect 1.8k resistor between Pi 5V pin and a breadboard row
2. Connect 3.3k resistor between that row and GND
3. Set multimeter to DC Voltage
4. Measure voltage at the junction point
5. Compare with calculated value:

$$V_{\text{out}} = 5.0 \times \frac{3300}{1800 + 3300} = 3.24 \text{ V}$$

Try different resistor combinations and verify:

| R1 | R2 | Calculated Vout | Measured Vout |
|----|----|----|-----|
| 1.8k | 3.3k | 3.24V | ? |
| 1k | 1k | 2.50V | ? |
| 2.2k | 1k | 1.56V | ? |
| 10k | 10k | 2.50V | ? |

### 7.4 Lab 4: GPIO Interrupt with Timing Measurement

```python
#!/usr/bin/env python3
"""
interrupt_timing.py -- Measure interrupt response time
Useful for understanding real-time capability limits of Linux on Pi 5
"""

from gpiozero import Button
from signal import pause
import time
import statistics

# Connect a button between GPIO27 and GND
button = Button(27, pull_up=True, bounce_time=0.001)  # Minimal debounce

latencies = []
press_time = None

def on_press():
    global press_time
    press_time = time.perf_counter_ns()

def on_release():
    global press_time
    if press_time is not None:
        release_time = time.perf_counter_ns()
        duration_us = (release_time - press_time) / 1000
        latencies.append(duration_us)

        if len(latencies) % 10 == 0:
            print(f"\n--- After {len(latencies)} presses ---")
            print(f"  Min hold time:  {min(latencies):.1f} us")
            print(f"  Max hold time:  {max(latencies):.1f} us")
            print(f"  Mean hold time: {statistics.mean(latencies):.1f} us")
            if len(latencies) > 1:
                print(f"  Stdev:          {statistics.stdev(latencies):.1f} us")

        press_time = None

button.when_pressed = on_press
button.when_released = on_release

print("Interrupt Timing Test")
print("Press and release button repeatedly.")
print("Stats shown every 10 presses.")
print("Ctrl+C for final results.")
print()

try:
    pause()
except KeyboardInterrupt:
    if latencies:
        print(f"\n\n=== Final Results ({len(latencies)} samples) ===")
        print(f"  Min:    {min(latencies):.1f} us")
        print(f"  Max:    {max(latencies):.1f} us")
        print(f"  Mean:   {statistics.mean(latencies):.1f} us")
        print(f"  Median: {statistics.median(latencies):.1f} us")
        if len(latencies) > 1:
            print(f"  Stdev:  {statistics.stdev(latencies):.1f} us")
    print("\nDone.")
```

### 7.5 Lab 5: Current Measurement with Multimeter

Measure the current drawn by an LED circuit to verify Ohm's law:

```
GPIO 17 ---[330R]---[LED]---(Multimeter in A mode)--- GND

Predicted current: I = (3.3V - 2.0V) / 330 = 3.9 mA
```

**Steps:**
1. Set multimeter to DC Current (mA range)
2. Connect multimeter **in series** between LED cathode and GND
3. Run the LED blink script from Day 1
4. Read the current when LED is ON
5. Compare measured value with calculated value

Also measure:
- Current with 220 ohm resistor (predicted: 5.9 mA)
- Current with 1k ohm resistor (predicted: 1.3 mA)
- No resistor (DO NOT DO THIS -- could damage GPIO! Current would exceed pin limit)

---

## 8. Review

### Key Concepts Checklist

1. **Ohm's Law**: \(V = IR\). Use it to calculate resistor values for LEDs, current limits, and power dissipation.

2. **Voltage dividers**: \(V_{\text{out}} = V_{\text{in}} \times R_2 / (R_1 + R_2)\). Essential for level shifting (5V to 3.3V) and battery voltage sensing.

3. **Pull-up/Pull-down resistors**: Prevent floating inputs. Pull-up = default HIGH (button pulls to GND). Pull-down = default LOW (button connects to VCC). RPi 5 has internal ~50k pull-ups/downs.

4. **RPi 5 power**: 5V/5A USB-PD required. Under-voltage causes throttling. Budget your power carefully. Use proper DC-DC converter for car battery.

5. **Decoupling capacitors**: 100nF ceramic near every IC power pin. Smooths high-frequency current spikes.

6. **UART debug console**: TX/RX crossover at 115200 8N1. Works before Linux boots, during kernel panics, and when SSH fails. The most powerful embedded debugging tool.

7. **GPIO interrupts**: Event-driven, near-zero CPU usage, sub-millisecond latency. Always prefer interrupts over polling. Use debouncing (software: `bounce_time`, hardware: RC filter).

### Self-Test Questions

**Q1:** You need to connect a 5V ultrasonic sensor output to an RPi 5 GPIO input. Design the voltage divider and calculate the output voltage.

**Answer:** Use R1 = 1.8k and R2 = 3.3k.
\(V_{\text{out}} = 5.0 \times 3300 / (1800 + 3300) = 3.24\text{V}\).
This is safely within the 3.3V GPIO threshold. For high-speed signals (>1 MHz), use a level shifter IC instead.

**Q2:** Your autonomous car uses a 12V/10Ah LiPo battery with a 90% efficient buck converter. The Pi 5 draws 10W average. How long will it run?

**Answer:** Battery energy = 12V x 10Ah = 120 Wh. Usable energy = 120 x 0.90 = 108 Wh. Runtime = 108 / 10 = 10.8 hours.

**Q3:** You press a button once, but your interrupt handler fires 7 times. What is happening, and how do you fix it?

**Answer:** Contact bounce. The mechanical contacts are bouncing for a few milliseconds, producing multiple edges. Fix with software debouncing (`bounce_time=0.05` in gpiozero) or hardware debouncing (100nF capacitor in parallel with the button).

---

## Next: Day 4

Tomorrow we dive into **communication protocols**: UART (in depth), SPI, I2C, CAN, and USB. These are how every component in an autonomous car talks to every other component. We will wire up real sensors, capture waveforms, and intentionally break things to understand failure modes.

See you in [Day 4 -- Communication Protocols: UART, SPI, I2C, CAN, and USB](/posts/embedded-day-04/).
