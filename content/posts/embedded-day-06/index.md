---
title: "Day 6 — Motor Fundamentals and Hall Sensor Encoders"
date: 2026-03-06
description: "Lorentz force in DC motors, BLDC 3-phase electronic commutation, H-bridge PWM control, and Hall effect encoder for speed measurement"
categories: ["Autonomous Driving"]
tags: ["DC Motor", "BLDC", "Hall Sensor", "Encoder", "PWM", "H-Bridge"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 6
draft: false
---

{{< katex >}}

## What You'll Learn

- How DC brushed motors work: Lorentz force, Back-EMF, commutation
- BLDC motors: 3-phase electronic commutation and why they're better for robots
- H-Bridge circuits: direction control and PWM speed control
- Hall effect: the physics behind magnetic position sensing
- How to measure wheel RPM in real-time with Hall sensor encoders

---

## 1. DC Brushed Motor

### The Lorentz Force

Every electric motor works on one fundamental principle: a current-carrying conductor in a magnetic field experiences a force.

$$\vec{F} = q\vec{v} \times \vec{B} = I\vec{L} \times \vec{B}$$

For a wire of length \(L\) carrying current \(I\) in a magnetic field \(B\):

$$F = BIL \sin\theta$$

When \(\theta = 90°\) (wire perpendicular to field), force is maximum.

This force creates **torque** on the rotor:

$$\tau = N \cdot B \cdot I \cdot A$$

Where \(N\) = number of turns, \(A\) = coil area.

### Back-EMF

When the rotor spins, the moving conductor cuts through magnetic field lines, generating a **voltage** that opposes the applied voltage (Lenz's law):

$$V_{emf} = k_e \cdot \omega$$

Where:
- \(k_e\) = back-EMF constant (V·s/rad)
- \(\omega\) = angular velocity (rad/s)

The motor equation:

$$V_{applied} = I \cdot R_{coil} + k_e \cdot \omega$$

At steady state:

$$I = \frac{V_{applied} - k_e \cdot \omega}{R_{coil}}$$

**Implication**: As the motor speeds up, back-EMF increases, current decreases, and torque decreases. The motor reaches equilibrium when torque equals the load.

### Commutator and Brushes

```
         N                S
    ┌─────────┐     ┌─────────┐
    │ Permanent│     │ Permanent│
    │ Magnet  │     │ Magnet   │
    │         │     │          │
    │    ┌────┴─────┴────┐     │
    │    │  Rotor Coil   │     │
    │    │     ┌───┐     │     │
    │    │     │   │     │     │
    │    └──┬──┘   └──┬──┘     │
    │       │         │        │
    └───────┼─────────┼────────┘
         ┌──┴──┐   ┌──┴──┐
         │Comm.│   │Comm.│  ← Commutator segments
         └──┬──┘   └──┬──┘
            │         │
         [Brush]   [Brush]  ← Carbon brushes (fixed)
            │         │
          V+         GND
```

The **commutator** reverses current direction every half rotation, keeping the torque in the same direction. **Brushes** are spring-loaded contacts that transfer current to the spinning commutator.

**Problems with brushes**:
- Friction → wear → limited lifetime
- Sparking at contacts → electrical noise
- Speed limited by brush contact reliability
- Carbon dust contamination

---

## 2. BLDC Motor (Brushless DC)

### Why Brushless?

BLDC motors eliminate brushes by moving the coils to the **stator** (fixed part) and placing permanent magnets on the **rotor**:

```
  Stator (fixed, 3 coils)         Rotor (spinning, magnets)
  ┌───────────────────┐           ┌──────────────┐
  │     Coil A        │           │   N     S    │
  │        ↕          │           │  Permanent   │
  │  Coil C   Coil B  │           │  Magnets     │
  │    ↕         ↕    │           │              │
  └───────────────────┘           └──────────────┘

  3 phases (U, V, W)              Rotor position detected
  energized in sequence           by Hall sensors
```

### Electronic Commutation

Instead of mechanical brushes, an **Electronic Speed Controller (ESC)** switches the coils in sequence:

```
Step 1: Energize A+, B-         → Rotor moves to position 1
Step 2: Energize A+, C-         → Rotor moves to position 2
Step 3: Energize B+, C-         → Rotor moves to position 3
Step 4: Energize B+, A-         → Rotor moves to position 4
Step 5: Energize C+, A-         → Rotor moves to position 5
Step 6: Energize C+, B-         → Rotor moves to position 6
... repeat (6-step commutation)
```

Each step rotates the magnetic field by 60°. The rotor follows the rotating field.

### BLDC vs Brushed Comparison

| Feature | Brushed DC | BLDC |
|---------|-----------|------|
| Commutation | Mechanical (brushes) | Electronic (ESC) |
| Lifetime | Limited (brush wear) | Much longer |
| Efficiency | 70-80% | 85-95% |
| Speed range | Lower | Higher |
| Noise | Higher (brush sparking) | Lower |
| Control | Simple (voltage) | Complex (needs ESC) |
| Cost | Cheaper | More expensive |
| Maintenance | Replace brushes | Nearly maintenance-free |

---

## 3. H-Bridge: Motor Direction Control

### How It Works

An **H-Bridge** uses 4 switches (MOSFETs) to control current direction through the motor:

```
     VCC                        VCC
      │                          │
   ┌──┴──┐                   ┌──┴──┐
   │ Q1  │                   │ Q3  │
   │(HIGH│                   │(LOW)│
   │side)│                   │     │
   └──┬──┘                   └──┬──┘
      │                          │
      ├──────── MOTOR ───────────┤
      │        ───→              │
   ┌──┴──┐   (Forward)       ┌──┴──┐
   │ Q2  │                   │ Q4  │
   │(LOW)│                   │(HIGH│
   │     │                   │side)│
   └──┬──┘                   └──┬──┘
      │                          │
     GND                        GND

Forward: Q1=ON, Q4=ON, Q2=OFF, Q3=OFF → Current flows left to right
Reverse: Q2=ON, Q3=ON, Q1=OFF, Q4=OFF → Current flows right to left
Brake:   Q1=ON, Q3=ON (or Q2+Q4) → Motor shorted → active braking
Coast:   All OFF → Motor spins freely
```

**DANGER**: Never turn on Q1+Q2 or Q3+Q4 simultaneously — this creates a short circuit from VCC to GND (called **shoot-through**).

### Dead Time

When switching direction, there must be a brief period (**dead time**, ~100ns to 1µs) where both switches in a leg are OFF to prevent shoot-through:

```
Q1:  ████████        ░░░░░░        ████████
Q2:  ░░░░░░░░        ████████        ░░░░░░
                 ↑            ↑
              dead time    dead time
              (~500ns)     (~500ns)
```

### PWM Speed Control

By switching the H-bridge ON/OFF rapidly (PWM), we control the average voltage across the motor:

$$V_{average} = V_{supply} \times \frac{t_{on}}{t_{on} + t_{off}} = V_{supply} \times D$$

Where \(D\) is the **duty cycle** (0.0 to 1.0).

At 50% duty cycle with 12V supply:

$$V_{average} = 12V \times 0.5 = 6V$$

**PWM frequency** matters:
- Too low (< 1 kHz): Motor hums audibly
- Typical (10-20 kHz): Above human hearing, smooth operation
- Too high (> 100 kHz): Switching losses increase

---

## 4. Hall Effect

### The Physics

When current flows through a conductor in a magnetic field, charge carriers are deflected to one side, creating a voltage perpendicular to both the current and the field:

$$V_H = \frac{IB}{ned}$$

Where:
- \(I\) = current through the conductor
- \(B\) = magnetic field strength
- \(n\) = charge carrier density
- \(e\) = electron charge (\(1.6 \times 10^{-19}\) C)
- \(d\) = conductor thickness

In practice, Hall sensor ICs integrate the conductor, amplifier, and comparator into one package. They output HIGH when a magnetic field is detected and LOW when not.

### Hall Sensor Encoder in Motors

BLDC motors typically have **3 Hall sensors** embedded in the stator, spaced 120° apart electrically:

```
Hall Sensor Signals (one electrical revolution):

Hall A: ████████░░░░░░░░████████░░░░░░░░
Hall B: ░░░░████████░░░░░░░░████████░░░░
Hall C: ░░░░░░░░████████░░░░░░░░████████
         │   │   │   │   │   │
         1   2   3   4   5   6   ← 6 commutation states

         ←── One electrical revolution ──→
```

From these 3 signals, we can determine:

1. **Rotor position** (which of 6 states → commutation timing)
2. **Rotation direction** (which signal leads: A→B→C = forward, A→C→B = reverse)
3. **Speed** (count transitions per unit time)

### Speed Measurement

Each transition of a Hall signal = a known fraction of a revolution.

With 3 Hall sensors and a motor with \(P\) pole pairs:
- One electrical revolution = 6 transitions
- One mechanical revolution = \(6 \times P\) transitions

$$\text{RPM} = \frac{\text{transitions}}{6 \times P} \times \frac{60}{\Delta t}$$

Or using **PPR (Pulses Per Revolution)**:

$$\text{RPM} = \frac{\text{pulse count}}{\text{PPR}} \times \frac{60}{\Delta t_{seconds}}$$

**Example**: Motor has 7 pole pairs. We count 420 Hall transitions in 1 second.

$$\text{RPM} = \frac{420}{6 \times 7} \times \frac{60}{1} = \frac{420}{42} \times 60 = 600 \text{ RPM}$$

### Hall vs Optical Encoder Comparison

| Feature | Hall Sensor | Optical Encoder |
|---------|------------|-----------------|
| Resolution | Low-Medium (6×P per rev) | High (100-10000+ per rev) |
| Robustness | Excellent (sealed, no optics) | Sensitive to dust/oil |
| Cost | Built into BLDC motors | Additional component |
| Speed range | Very wide | Limited at very high speeds |
| Size | Tiny (inside motor) | External, larger |

For our autonomous car, the built-in Hall sensors provide sufficient resolution for speed control. High-resolution optical encoders would be used for precision positioning.

---

## 5. Hands-On Lab

### Lab 1: PWM Motor Control

```python
#!/usr/bin/env python3
"""Motor control using PWM via gpiozero on RPi 5."""

from gpiozero import Motor, PWMOutputDevice
import time

# Motor driver connections (e.g., L298N or TB6612)
# ENA = PWM speed control
# IN1, IN2 = direction control
motor = Motor(forward=17, backward=27, pwm=True)

# Or using a PWM device directly for a simple H-bridge
# pwm = PWMOutputDevice(18, frequency=20000)  # 20 kHz

print("Motor control demo")
print("=" * 40)

# Forward at various speeds
for speed in [0.3, 0.5, 0.7, 1.0]:
    print(f"Forward at {speed*100:.0f}% speed")
    motor.forward(speed=speed)
    time.sleep(2)

# Stop
print("Stop (coast)")
motor.stop()
time.sleep(1)

# Reverse
for speed in [0.3, 0.5, 0.7, 1.0]:
    print(f"Reverse at {speed*100:.0f}% speed")
    motor.backward(speed=speed)
    time.sleep(2)

# Active brake
print("Active brake")
motor.stop()
time.sleep(1)

print("Done!")
```

### Lab 2: Hall Sensor RPM Measurement

```python
#!/usr/bin/env python3
"""Real-time RPM measurement using Hall sensor interrupts."""

from gpiozero import DigitalInputDevice
import time
import threading

# Hall sensor connected to GPIO pins
HALL_PIN = 24  # One Hall sensor output
POLE_PAIRS = 7  # Motor pole pairs (check your motor spec)
PPR = 6 * POLE_PAIRS  # Pulses per revolution (6 states × pole pairs)

# Counters (shared between interrupt and main thread)
pulse_count = 0
pulse_lock = threading.Lock()

def hall_callback():
    """Called on every Hall sensor edge (interrupt-driven)."""
    global pulse_count
    with pulse_lock:
        pulse_count += 1

# Setup Hall sensor input with interrupt
hall_sensor = DigitalInputDevice(HALL_PIN, pull_up=True)
hall_sensor.when_activated = hall_callback
hall_sensor.when_deactivated = hall_callback  # Count both edges

print(f"RPM Measurement (PPR={PPR})")
print(f"{'Time':>8s} {'Pulses':>8s} {'RPM':>10s}")
print("-" * 30)

try:
    while True:
        # Reset counter
        with pulse_lock:
            current_count = pulse_count
            pulse_count = 0

        # Calculate RPM
        dt = 0.1  # Measurement interval (seconds)
        rpm = (current_count / PPR) * (60.0 / dt)

        print(f"{'':>8s} {current_count:>8d} {rpm:>10.1f}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("\nStopped.")
```

### Lab 3: Logic Analyzer — 3-Channel Hall Capture

```python
#!/usr/bin/env python3
"""Capture all 3 Hall sensor channels to verify phase sequence."""

from gpiozero import DigitalInputDevice
import time

HALL_A_PIN = 24
HALL_B_PIN = 25
HALL_C_PIN = 8

hall_a = DigitalInputDevice(HALL_A_PIN, pull_up=True)
hall_b = DigitalInputDevice(HALL_B_PIN, pull_up=True)
hall_c = DigitalInputDevice(HALL_C_PIN, pull_up=True)

print("3-Phase Hall Sensor Monitor")
print(f"{'Time_ms':>10s} {'A':>3s} {'B':>3s} {'C':>3s} {'State':>6s} {'Dir':>5s}")
print("-" * 40)

prev_state = None

try:
    start = time.time()
    while True:
        a = hall_a.value
        b = hall_b.value
        c = hall_c.value
        state = (a, b, c)

        if state != prev_state:
            elapsed = (time.time() - start) * 1000
            # Decode commutation state
            state_map = {
                (1, 0, 1): 1,
                (1, 0, 0): 2,
                (1, 1, 0): 3,
                (0, 1, 0): 4,
                (0, 1, 1): 5,
                (0, 0, 1): 6,
            }
            state_num = state_map.get(state, "?")

            print(f"{elapsed:>10.1f} {a:>3d} {b:>3d} {c:>3d} {str(state_num):>6s}")
            prev_state = state

        time.sleep(0.001)  # 1ms polling (for demo; use interrupts for production)

except KeyboardInterrupt:
    print("\nDone.")
```

### Lab 4: Real-Time RPM Plotting

```python
#!/usr/bin/env python3
"""Real-time RPM plotting with matplotlib animation."""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from gpiozero import DigitalInputDevice
import threading
import time

HALL_PIN = 24
POLE_PAIRS = 7
PPR = 6 * POLE_PAIRS

# Data storage
rpm_history = deque(maxlen=200)
time_history = deque(maxlen=200)
pulse_count = 0
pulse_lock = threading.Lock()

def hall_callback():
    global pulse_count
    with pulse_lock:
        pulse_count += 1

hall = DigitalInputDevice(HALL_PIN, pull_up=True)
hall.when_activated = hall_callback
hall.when_deactivated = hall_callback

# RPM calculation thread
start_time = time.time()

def rpm_calculator():
    global pulse_count
    while True:
        with pulse_lock:
            count = pulse_count
            pulse_count = 0

        rpm = (count / PPR) * (60.0 / 0.05)
        rpm_history.append(rpm)
        time_history.append(time.time() - start_time)
        time.sleep(0.05)  # 20 Hz update

calc_thread = threading.Thread(target=rpm_calculator, daemon=True)
calc_thread.start()

# Matplotlib animation
fig, ax = plt.subplots(figsize=(10, 4))
line, = ax.plot([], [], 'b-', linewidth=1.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('RPM')
ax.set_title('Real-Time Motor RPM')
ax.grid(True, alpha=0.3)

def animate(frame):
    if len(time_history) > 1:
        line.set_data(list(time_history), list(rpm_history))
        ax.set_xlim(max(0, time_history[-1] - 10), time_history[-1] + 0.5)
        ax.set_ylim(0, max(rpm_history) * 1.2 + 10)
    return line,

ani = animation.FuncAnimation(fig, animate, interval=50, blit=True)
plt.tight_layout()
plt.show()
```

---

## 6. Review

### Key Takeaways

1. **Lorentz force** \(F = BIL\) is the principle behind all electric motors
2. **Back-EMF** \(V_{emf} = k_e \omega\) limits motor speed and provides self-regulation
3. **BLDC** motors use electronic commutation — no brushes, longer life, higher efficiency
4. **H-Bridge** with PWM controls both direction and speed
5. **Hall sensors** detect rotor position via the Hall effect — built into BLDC motors
6. **RPM** = (pulses / PPR) × (60 / dt) — measured via GPIO interrupts

### Design Exercise

Given a motor with 7 pole pairs spinning at 1200 RPM, how many Hall transitions per second do we expect?

$$\text{Transitions/sec} = \text{RPM} \times \frac{PPR}{60} = 1200 \times \frac{42}{60} = 840 \text{ transitions/sec}$$

Each transition is ~1.19 ms apart. Our GPIO interrupt handler needs to respond faster than this — easily achievable on RPi 5.

### Looking Ahead

Tomorrow (Day 7), we'll explore **IMU sensors** — the accelerometers and gyroscopes that tell our car which way is up and how fast it's turning. We'll learn the MEMS physics behind these tiny sensors and encounter the noise problems that motivate Day 8's Kalman Filter.
