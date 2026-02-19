---
title: "BLDC Motor Control: Principles and Precision Techniques"
date: 2026-02-19
description: "A comprehensive guide to Brushless DC motor control covering commutation fundamentals, FOC (Field-Oriented Control), sensor strategies, current loops, and advanced precision techniques"
categories: ["Motor Control"]
tags: ["BLDC", "Motor Control", "FOC", "Field-Oriented Control", "Embedded Systems", "Robotics"]
draft: false
---

{{< katex >}}

## Overview

A **Brushless DC (BLDC) motor** replaces the mechanical brushes and commutator of a traditional DC motor with electronic commutation. This eliminates the primary failure point (brush wear) while providing higher efficiency, better torque-to-weight ratio, and longer lifespan.

However, removing the mechanical commutator means the **controller must actively manage** which motor coils are energized at any given moment. This post covers the fundamentals of BLDC motor operation and progressively builds up to advanced precision control techniques.

```
Brushed DC Motor:                    BLDC Motor:
┌─────────────────┐                 ┌─────────────────┐
│                 │                 │                 │
│  Voltage ──→ Brushes ──→ Rotor   │  Voltage ──→ Controller ──→ Stator
│             (mechanical)         │             (electronic)
│                 │                 │                 │
│  Simple!        │                 │  Complex, but:  │
│  Self-commutates│                 │  - No brush wear│
│                 │                 │  - Higher power  │
│                 │                 │  - Precise ctrl  │
└─────────────────┘                 └─────────────────┘
```

---

## 1. BLDC Motor Construction

### 1.1 Physical Structure

A BLDC motor consists of a **permanent magnet rotor** and a **wound stator** with three phases (A, B, C):

```
        Stator (fixed, with coils)
    ┌──────────────────────────────┐
    │         ┌─────┐              │
    │    C ───┤     ├─── A        │
    │         │  N  │              │
    │         │  ↑  │  Rotor      │
    │         │  S  │  (rotating  │
    │    B ───┤     ├─── magnets) │
    │         └─────┘              │
    └──────────────────────────────┘

    Three-phase winding: A, B, C
    Rotor: Permanent magnets (N-S poles)
```

### 1.2 Electrical Model

Each phase can be modeled as an inductor (\(L\)), resistor (\(R\)), and back-EMF source (\(e\)) in series:

$$
V_a = R \cdot i_a + L \frac{di_a}{dt} + e_a
$$

$$
V_b = R \cdot i_b + L \frac{di_b}{dt} + e_b
$$

$$
V_c = R \cdot i_c + L \frac{di_c}{dt} + e_c
$$

The back-EMF is proportional to rotor speed:

$$
e = K_e \cdot \omega
$$

where \(K_e\) is the back-EMF constant and \(\omega\) is the angular velocity.

### 1.3 Torque Production

Torque is produced by the interaction between stator current and rotor magnetic field:

$$
\tau = K_t \cdot i
$$

where \(K_t\) is the torque constant. In a three-phase BLDC:

$$
\tau = K_t (i_a \cdot f_a(\theta) + i_b \cdot f_b(\theta) + i_c \cdot f_c(\theta))
$$

where \(f(\theta)\) is the back-EMF waveform shape as a function of rotor angle \(\theta\).

---

## 2. The Inverter: Power Electronics

The motor is driven by a **three-phase inverter** (also called a 6-switch bridge):

```
          DC Bus (V_DC)
    ──────────┬──────────┬──────────
              │          │          │
           ┌──┴──┐   ┌──┴──┐   ┌──┴──┐
           │ Q1  │   │ Q3  │   │ Q5  │   ← High-side MOSFETs
           └──┬──┘   └──┬──┘   └──┬──┘
              │          │          │
              ├── A      ├── B     ├── C   ← Motor phases
              │          │          │
           ┌──┴──┐   ┌──┴──┐   ┌──┴──┐
           │ Q2  │   │ Q4  │   │ Q6  │   ← Low-side MOSFETs
           └──┬──┘   └──┬──┘   └──┬──┘
              │          │          │
    ──────────┴──────────┴──────────┴──── GND
```

Each motor phase is connected between a high-side and low-side MOSFET (or IGBT). By turning specific switches on/off, the controller determines which phases are energized and in which direction.

**Critical rule**: Never turn on both MOSFETs of the same leg simultaneously — this creates a **shoot-through** short circuit that can destroy the inverter.

---

## 3. Commutation: The Fundamental Challenge

### 3.1 Six-Step (Trapezoidal) Commutation

The simplest BLDC control method. At any instant, only **two of three phases are active** — one sourcing current, one sinking, one floating:

```
Step 1: A+ B-    Step 2: A+ C-    Step 3: B+ C-
Step 4: B+ A-    Step 5: C+ A-    Step 6: C+ B-

Phase Voltages:
     A:  ┌──┐         ┌──┐
─────┘   └────────────┘   └─────────
     B:        ┌──┐         ┌──┐
─────────────┘   └────────────┘   └──
     C:  ──┐         ┌──┐         ┌─
───────┘   └────────┘   └────────┘

Each step = 60° electrical rotation
Full cycle = 6 steps = 360° electrical
```

### 3.2 Rotor Position Detection

To commutate correctly, the controller must know the rotor position. Three main approaches:

**Hall Sensors** (Most Common for Six-Step):

```
Three Hall sensors placed 120° apart on the stator:

Hall A  Hall B  Hall C  │  Step  │  Active Phases
   1       0       1    │   1    │  A+ B-
   1       0       0    │   2    │  A+ C-
   1       1       0    │   3    │  B+ C-
   0       1       0    │   4    │  B+ A-
   0       1       1    │   5    │  C+ A-
   0       0       1    │   6    │  C+ B-
```

The 3 Hall sensors provide 6 unique combinations — one per commutation step. Resolution: **60° electrical**.

**Encoder** (For precision):
- Incremental: Provides relative position via A/B quadrature signals
- Absolute: Provides exact position at power-on
- Typical resolution: 1,000–10,000 CPR (counts per revolution)

**Sensorless** (Back-EMF Zero-Crossing):
- Monitors the floating phase's back-EMF
- Detects zero-crossing to determine rotor position
- Does not work at zero/low speed (no back-EMF generated)

---

## 4. PWM Speed Control

Speed is controlled by varying the **duty cycle** of a PWM signal applied to the active switches:

```
100% Duty Cycle:     50% Duty Cycle:      25% Duty Cycle:
┌────────────────┐   ┌────┐    ┌────┐     ┌──┐      ┌──┐
│                │   │    │    │    │     │  │      │  │
│    V_DC        │   │    │    │    │     │  │      │  │
│                │   │    │    │    │     │  │      │  │
└────────────────┘   └────┘    └────┘     └──┘      └──┘

V_avg = V_DC         V_avg = 0.5·V_DC     V_avg = 0.25·V_DC

Speed ∝ V_avg (approximately, at steady state)
```

PWM frequency is typically **10–100 kHz** — fast enough that the motor's inductance smooths the current into a near-DC waveform.

---

## 5. Field-Oriented Control (FOC)

Six-step commutation is simple but produces **torque ripple** because only two phases are active at a time, and the current waveform is trapezoidal. For precision applications, **Field-Oriented Control (FOC)** — also known as **vector control** — is the gold standard.

### 5.1 Core Idea

FOC transforms the three-phase AC problem into a **DC control problem** using coordinate transformations. The goal: independently control **torque-producing current** and **flux-producing current**.

```
Three-Phase World (complex):        d-q Frame (simple):
┌──────────────────────┐           ┌──────────────────────┐
│  i_a(t) = sin(ωt)    │           │  i_d = constant (DC) │
│  i_b(t) = sin(ωt-120)│  ──→     │  i_q = constant (DC) │
│  i_c(t) = sin(ωt+120)│  Park    │                      │
│                      │  Transform│  Much easier to       │
│  AC signals,         │           │  control with PID!    │
│  time-varying        │           │                      │
└──────────────────────┘           └──────────────────────┘
```

### 5.2 The Transformation Chain

FOC uses two coordinate transformations:

```
         Clarke              Park
  a,b,c ──────→ α,β ──────→ d,q
 (3-phase)     (2-phase     (rotating
               stationary)   frame, DC)
```

**Step 1: Clarke Transform** (3-phase → 2-phase stationary)

$$
\begin{bmatrix} i_\alpha \\\\ i_\beta \end{bmatrix} = \frac{2}{3}\begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\\\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix} \begin{bmatrix} i_a \\\\ i_b \\\\ i_c \end{bmatrix}
$$

This maps three phase currents to a two-axis stationary reference frame.

**Step 2: Park Transform** (stationary → rotating)

$$
\begin{bmatrix} i_d \\\\ i_q \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\\\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} i_\alpha \\\\ i_\beta \end{bmatrix}
$$

where \(\theta\) is the electrical angle of the rotor. This rotates the reference frame to align with the rotor, converting AC signals to **DC values**.

### 5.3 The d-q Current Components

After the Park transform:

- \(i_d\) (**direct axis**): Controls **magnetic flux**. For surface-mount PM motors, set \(i_d = 0\) to maximize efficiency (no need to strengthen or weaken the permanent magnet field).
- \(i_q\) (**quadrature axis**): Controls **torque**. Torque is directly proportional to \(i_q\):

$$
\tau = \frac{3}{2} \cdot p \cdot \lambda_m \cdot i_q
$$

where \(p\) is the number of pole pairs and \(\lambda_m\) is the permanent magnet flux linkage.

### 5.4 The Complete FOC Loop

```
                    ┌─────────────────────────────────────┐
                    │           FOC Control Loop            │
                    │                                       │
  Speed Ref ──→ [Speed PI] ──→ i_q_ref ──→ [Current PI] ──┤
                                                           │
  i_d_ref = 0 ─────────────────────────→ [Current PI] ──┤
                                                           │
                    ┌──────────────────────────────────────┘
                    │
                    ▼
              Inverse Park      Inverse Clarke      PWM
  V_d, V_q ──────────→ V_α, V_β ──────────→ V_a, V_b, V_c ──→ Inverter
                    ↑
              [Park Transform]
                    ↑
              [Clarke Transform]
                    ↑
              i_a, i_b, i_c ←── Current Sensors
                    ↑
              θ (rotor angle) ←── Encoder / Sensorless Estimator
```

The loop runs at **10–40 kHz** (current loop), with the speed loop typically running 10x slower.

### 5.5 Space Vector Modulation (SVM)

Instead of simple sinusoidal PWM, FOC typically uses **Space Vector Modulation** to maximize DC bus utilization:

```
Sinusoidal PWM:                  Space Vector PWM:
Max output = V_DC / 2            Max output = V_DC / √3 ≈ 0.577 · V_DC

SVM achieves ~15% more voltage utilization than sinusoidal PWM
```

SVM treats the three-phase inverter as producing **8 possible voltage vectors** (6 active + 2 zero) and synthesizes any desired output voltage by time-averaging adjacent vectors within each PWM period.

---

## 6. Precision Techniques

### 6.1 Current Sensing

Accurate current measurement is fundamental to precision control. Three main approaches:

```
Low-Side Shunt Resistors:         Phase Shunt Resistors:
┌──────────────────┐              ┌──────────────────┐
│ Q1    Q3    Q5   │              │ Q1    Q3    Q5   │
│  │     │     │   │              │  │     │     │   │
│  ├─A   ├─B   ├─C │              │  ├─A   ├─B   ├─C │
│  │     │     │   │              │  R     R     R   │
│ Q2    Q4    Q6   │              │ Q2    Q4    Q6   │
│  │     │     │   │              │  │     │     │   │
│  R     R         │              └──┴─────┴─────┴───┘
│  │     │         │
└──┴─────┴─────────┘              Pros: Full 3-phase measurement
                                  Cons: More components, routing
Pros: Cheap, simple
Cons: Only 2 of 3 phases directly
      (3rd computed: i_a+i_b+i_c=0)
```

For highest precision, **isolated current sensors** (e.g., hall-effect based ACS712, or sigma-delta modulator based) provide galvanic isolation and bandwidth up to several hundred kHz.

### 6.2 Dead-Time Compensation

When switching MOSFETs, a **dead time** (typically 0.5–2 \(\mu\)s) is inserted to prevent shoot-through. This dead time causes voltage distortion and torque ripple:

```
Ideal PWM:        Actual (with dead time):
┌────────┐        ┌────────┐
│ Q1 ON  │        │ Q1 ON  │
└────────┘        └───┐    │  ← Dead time Δt
┌────────┐            └────┘
│ Q2 ON  │            ┌────────┐
└────────┘            │ Q2 ON  │
                      └────────┘

Voltage error per PWM cycle = ±V_DC · (Δt / T_PWM)
```

**Compensation**: Measure or estimate current direction, then add/subtract the dead-time voltage error from the PWM reference:

$$
V_{comp} = \text{sign}(i_{phase}) \cdot V_{DC} \cdot \frac{\Delta t}{T_{PWM}}
$$

### 6.3 Flux Weakening

To operate above the rated speed, the back-EMF exceeds the available bus voltage. **Flux weakening** injects negative \(i_d\) current to counteract the permanent magnet flux:

$$
i_d = -\frac{\lambda_m - \sqrt{V_{max}^2 / \omega^2 - (L_q i_q)^2}}{L_d}
$$

```
Torque vs Speed:
τ │
  │  ┌──────────┐
  │  │ Constant  │  ┌──────────────────┐
  │  │ Torque    │  │ Flux Weakening   │
  │  │ Region    │  │ (constant power) │
  │  │           │  │                  │
  └──┴───────────┴──┴──────────────────┴──→ ω
     0         ω_base                ω_max

  Below ω_base: i_d = 0, full torque available
  Above ω_base: i_d < 0, torque decreases as 1/ω
```

### 6.4 Observer-Based Sensorless Control

For applications where mechanical sensors are impractical (cost, size, harsh environment), **sensorless FOC** uses state observers to estimate rotor position from voltage and current measurements.

**Back-EMF Observer**:

$$
\hat{e}_\alpha = V_\alpha - R \cdot i_\alpha - L \frac{di_\alpha}{dt}
$$

$$
\hat{e}_\beta = V_\beta - R \cdot i_\beta - L \frac{di_\beta}{dt}
$$

$$
\hat{\theta} = \arctan\left(\frac{-\hat{e}_\alpha}{\hat{e}_\beta}\right)
$$

**Sliding Mode Observer (SMO)**: More robust to parameter variations:

$$
\frac{d\hat{i}_\alpha}{dt} = \frac{1}{L}(V_\alpha - R\hat{i}_\alpha - \hat{e}_\alpha) + k \cdot \text{sign}(i_\alpha - \hat{i}_\alpha)
$$

The sliding mode term \(k \cdot \text{sign}(\cdot)\) forces the estimated current to converge to the actual current, and the back-EMF estimate can be extracted from the switching function.

**Limitations**: Sensorless methods struggle at **zero and very low speeds** where back-EMF is negligible. High-frequency injection (HFI) techniques can extend the operating range to standstill.

### 6.5 Anti-Cogging Compensation

Permanent magnet motors exhibit **cogging torque** — a position-dependent reluctance torque caused by the interaction between rotor magnets and stator teeth. This causes vibration and position errors at low speeds.

```
Cogging Torque Profile:
τ_cog │    ╭╮    ╭╮    ╭╮    ╭╮
      │   ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
──────┼──╱────╲╱────╲╱────╲╱────╲──→ θ
      │ ╱      ╲      ╲      ╲
      │╱        ╰╮    ╰╮    ╰╮
      │          ╰╯    ╰╯    ╰╯

Period = 360° / LCM(poles, slots)
```

**Compensation technique**: Pre-calibrate the cogging torque profile by slowly rotating the motor and recording the torque at each position. Store as a lookup table and inject compensating current during operation:

$$
i_{q,comp}(\theta) = -\frac{\tau_{cog}(\theta)}{K_t}
$$

### 6.6 Vibration Suppression via Notch Filters

Mechanical resonances in the drivetrain can cause instability at specific frequencies. **Notch filters** in the control loop attenuate these resonances:

$$
H_{notch}(s) = \frac{s^2 + 2\zeta_z \omega_n s + \omega_n^2}{s^2 + 2\zeta_p \omega_n s + \omega_n^2}
$$

where \(\omega_n\) is the resonant frequency, \(\zeta_z < \zeta_p\) (the zero damping is less than the pole damping, creating a notch).

```
Magnitude Response:
|H| │
    │──────────────────────────────
    │              ╲    ╱
    │               ╲  ╱
    │                ╲╱   ← Notch at resonant frequency
    │
    └──────────────────────────────→ f
                     f_n
```

---

## 7. Control Loop Tuning

### 7.1 Cascaded Loop Structure

A precision BLDC controller typically uses three cascaded loops:

```
Position Reference ──→ [Position PID] ──→ Speed Reference
                                              │
                         Speed Feedback ←─────┤
                                              ▼
                       ──→ [Speed PI] ──→ i_q Reference
                                              │
                         i_q Feedback ←───────┤
                                              ▼
                       ──→ [Current PI] ──→ PWM Duty
                                              │
                         i_d,q Feedback ←─────┤
                                              ▼
                                          [Inverter + Motor]
```

### 7.2 Bandwidth Hierarchy

Each outer loop must be **slower** than its inner loop (typically 5–10x) to maintain stability:

| Loop | Bandwidth | Update Rate | Tuning Priority |
|------|-----------|-------------|-----------------|
| **Current** | 1–5 kHz | 10–40 kHz | First (innermost) |
| **Speed** | 100–500 Hz | 1–5 kHz | Second |
| **Position** | 10–50 Hz | 100–500 Hz | Third (outermost) |

### 7.3 Current Loop Tuning

The current loop plant model:

$$
G_{plant}(s) = \frac{1}{Ls + R}
$$

With a PI controller \(G_{PI}(s) = K_p + \frac{K_i}{s}\), pole-zero cancellation gives:

$$
K_p = L \cdot \omega_{bw}, \quad K_i = R \cdot \omega_{bw}
$$

where \(\omega_{bw}\) is the desired bandwidth in rad/s.

---

## 8. Practical Implementation Architecture

A typical precision BLDC control system:

```
┌──────────────────────────────────────────────────────┐
│                  MCU (e.g., STM32G4)                  │
│                                                      │
│  ┌────────────┐   ┌────────────┐   ┌──────────────┐ │
│  │ ADC        │   │ Timer/PWM  │   │ Encoder      │ │
│  │ (12-16 bit)│   │ (center-   │   │ Interface    │ │
│  │ i_a, i_b   │   │  aligned)  │   │ (QEP/SPI)   │ │
│  └─────┬──────┘   └─────┬──────┘   └──────┬───────┘ │
│        │                │                  │         │
│  ┌─────┴────────────────┴──────────────────┴───────┐ │
│  │              FOC Algorithm                       │ │
│  │                                                  │ │
│  │  1. Read currents (ADC)                         │ │
│  │  2. Read position (Encoder)                     │ │
│  │  3. Clarke transform (abc → αβ)                 │ │
│  │  4. Park transform (αβ → dq)                    │ │
│  │  5. PI current controllers (d, q)               │ │
│  │  6. Inverse Park (dq → αβ)                      │ │
│  │  7. SVM (αβ → PWM duties)                       │ │
│  │  8. Update PWM registers                        │ │
│  │                                                  │ │
│  │  All within one PWM cycle (~25-100 μs)          │ │
│  └──────────────────────────────────────────────────┘ │
│                                                      │
│  Communication: CAN / EtherCAT / UART               │
└──────────────────────────────────────────────────────┘
         │              │              │
    ┌────┴────┐   ┌────┴────┐   ┌────┴────┐
    │ Gate    │   │ Current │   │ Encoder │
    │ Driver  │   │ Sensors │   │         │
    └────┬────┘   └─────────┘   └─────────┘
         │
    ┌────┴────┐
    │ 3-Phase │
    │ Inverter│
    └────┬────┘
         │
    ┌────┴────┐
    │  BLDC   │
    │  Motor  │
    └─────────┘
```

### Timing Requirement

The entire FOC computation must complete within **one PWM period**. At 20 kHz PWM, that is **50 microseconds** for:
- 2–3 ADC conversions
- Clarke + Park transforms
- 2 PI controllers
- Inverse transforms
- SVM calculation
- PWM register update

This is why FOC is typically implemented on **dedicated motor control MCUs** (STM32G4, TI C2000, Infineon XMC) with hardware-accelerated math peripherals.

---

## 9. Summary

```
BLDC Control Techniques — Complexity vs Performance:

Performance │
            │                          FOC + Anti-cogging
            │                       ●  + Dead-time comp
            │                    ●     + Flux weakening
            │                 ●
            │              FOC (Sinusoidal)
            │           ●
            │        ●  Six-Step + Current Control
            │     ●
            │  Six-Step (Trapezoidal)
            │●
            └──────────────────────────────────→ Complexity
```

| Technique | Torque Ripple | Speed Range | Position Control | Complexity |
|-----------|--------------|-------------|-----------------|------------|
| **Six-Step** | High (~15%) | Limited | No | Low |
| **Six-Step + Current** | Medium (~10%) | Limited | No | Medium |
| **FOC (basic)** | Low (~2%) | Full range | Yes | High |
| **FOC + advanced** | Very low (<1%) | Extended (flux weakening) | Sub-degree | Very High |

For robotics applications like joint actuators in humanoid robots or precision manipulators, **FOC with encoder feedback, anti-cogging compensation, and cascaded position/speed/current control** is the standard approach — providing the smooth, precise, and responsive motion that modern robotic systems demand.
