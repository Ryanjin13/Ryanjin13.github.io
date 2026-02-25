---
title: "PID Control and Laplace Transform: From Mathematical Foundations to Real-World Applications"
date: 2026-02-25
description: "A comprehensive, beginner-friendly guide to the Laplace transform, transfer functions, and PID control — with detailed mathematical derivations and four real-world examples: DC motor with encoder, lane centering, volume control, and thrust control."
categories: ["Control Systems"]
tags: ["PID", "Laplace Transform", "Transfer Function", "Control Theory", "Motor Control", "Autonomous Driving", "Embedded Systems"]
draft: false
---

{{< katex >}}

## Introduction

Imagine you're driving a car and trying to stay in the center of your lane. Your eyes see the offset (sensor), your brain computes how much to turn the wheel (controller), and your hands execute the correction (actuator). You don't just react to **where** you are — you also consider **how fast** you're drifting and **how long** you've been off-center. Congratulations: your brain is running a **PID controller**.

**PID (Proportional-Integral-Derivative)** control is the most widely used control algorithm in engineering. It's inside everything from your home thermostat to SpaceX rockets. And the mathematical tool that makes analyzing these systems elegant and tractable is the **Laplace transform**.

This post covers:
1. **Laplace Transform** — what it is, why we need it, and how to use it
2. **Transfer Functions** — modeling physical systems as input-output relationships
3. **PID Control** — the algorithm, its mathematics, and tuning
4. **Four Real-World Examples** — motor/encoder, lane centering, volume control, thrust control

---

## 1. The Laplace Transform: Turning Calculus into Algebra

### 1.1 The Problem: Differential Equations Are Hard

Physical systems are described by **differential equations**. For example, a simple spring-mass-damper system:

$$
m\ddot{x} + b\dot{x} + kx = F(t)
$$

Solving this directly requires guessing solution forms, matching boundary conditions, and handling convolutions. It's tedious and error-prone.

**What if we could convert these differential equations into simple algebraic equations?** That's exactly what the Laplace transform does.

```
Time Domain (hard)              Frequency Domain (easy)
─────────────────              ──────────────────────
Differential equations    →    Algebraic equations
Convolution              →    Multiplication
Solve with calculus      →    Solve with algebra
                  ↓ Laplace Transform ↓
        f(t)  ──────────────→  F(s)

        Solve algebraically in s-domain

        F(s)  ──────────────→  f(t)
                  ↑ Inverse Laplace ↑
```

### 1.2 Definition

The Laplace transform of a function $f(t)$ is defined as:

$$
\mathcal{L}\{f(t)\} = F(s) = \int_0^{\infty} f(t) \, e^{-st} \, dt
$$

where:
- $t$ is **time** (real, $t \geq 0$)
- $s = \sigma + j\omega$ is a **complex frequency variable**
- $F(s)$ is the **Laplace-domain** representation of $f(t)$

**Intuition:** The Laplace transform decomposes a time signal into a sum of exponentially-weighted sinusoids. The variable $s$ encodes both **growth/decay rate** ($\sigma$) and **oscillation frequency** ($\omega$).

### 1.3 Why Does This Work?

The key insight is what happens to **derivatives** under the Laplace transform:

$$
\mathcal{L}\{\dot{f}(t)\} = sF(s) - f(0)
$$

$$
\mathcal{L}\{\ddot{f}(t)\} = s^2F(s) - sf(0) - \dot{f}(0)
$$

**Differentiation in time becomes multiplication by $s$.** This is the magic — every derivative turns into a power of $s$, converting differential equations into polynomials.

For example, starting from the spring-mass-damper equation (assuming zero initial conditions):

$$
m\ddot{x} + b\dot{x} + kx = F(t)
$$

Applying the Laplace transform to both sides:

$$
ms^2X(s) + bsX(s) + kX(s) = F(s)
$$

$$
X(s)(ms^2 + bs + k) = F(s)
$$

$$
\frac{X(s)}{F(s)} = \frac{1}{ms^2 + bs + k}
$$

That's it. A messy differential equation became a simple fraction. This fraction is called the **transfer function**.

### 1.4 Essential Laplace Transform Table

Here are the transforms you'll use most often:

| Time Domain $f(t)$ | Laplace Domain $F(s)$ | Example |
|:---:|:---:|:---|
| $1$ (unit step) | $\dfrac{1}{s}$ | Constant input (step command) |
| $t$ (ramp) | $\dfrac{1}{s^2}$ | Linearly increasing input |
| $e^{-at}$ | $\dfrac{1}{s+a}$ | Exponential decay (RC circuit) |
| $\sin(\omega t)$ | $\dfrac{\omega}{s^2 + \omega^2}$ | Oscillation |
| $\cos(\omega t)$ | $\dfrac{s}{s^2 + \omega^2}$ | Oscillation |
| $t \cdot e^{-at}$ | $\dfrac{1}{(s+a)^2}$ | Damped ramp |
| $\delta(t)$ (impulse) | $1$ | Instantaneous kick |

### 1.5 Key Properties

| Property | Time Domain | s-Domain |
|:---|:---:|:---:|
| **Linearity** | $af(t) + bg(t)$ | $aF(s) + bG(s)$ |
| **Differentiation** | $\dfrac{df}{dt}$ | $sF(s) - f(0)$ |
| **Integration** | $\displaystyle\int_0^t f(\tau)d\tau$ | $\dfrac{F(s)}{s}$ |
| **Time delay** | $f(t - T)$ | $e^{-Ts}F(s)$ |
| **Final Value Theorem** | $\lim_{t \to \infty} f(t)$ | $\lim_{s \to 0} sF(s)$ |

The **Final Value Theorem** is particularly useful — it tells us the steady-state value of a system's output without solving the full time-domain response.

### 1.6 A Concrete Example: RC Circuit

Consider a simple resistor-capacitor circuit where we apply a voltage step $V_{in}$ and want to find the capacitor voltage $V_c(t)$:

```
    R
Vin ─┤├──┬── Vc(t)
          │
         ═╧═ C
          │
         GND
```

**Time-domain equation (KVL):**

$$
V_{in} = R \cdot i(t) + V_c(t), \quad i(t) = C\frac{dV_c}{dt}
$$

$$
V_{in} = RC\frac{dV_c}{dt} + V_c(t)
$$

**Laplace transform** (with $V_c(0) = 0$):

$$
\frac{V_{in}}{s} = RC \cdot sV_c(s) + V_c(s) = V_c(s)(RCs + 1)
$$

$$
V_c(s) = \frac{V_{in}}{s(RCs + 1)} = \frac{V_{in}}{s} \cdot \underbrace{\frac{1}{RCs + 1}}_{\text{Transfer function}}
$$

**Inverse Laplace** (partial fractions):

$$
V_c(t) = V_{in}\left(1 - e^{-t/RC}\right)
$$

The **transfer function** $H(s) = \frac{1}{RCs + 1}$ completely describes how this circuit responds to any input. The time constant $\tau = RC$ determines how fast the system responds.

```
Vc(t)
 │    ┌─────────────────── Vin
 │   ╱
 │  ╱   63.2% at t = τ = RC
 │ ╱
 │╱
 └────────────────────────── t
 0    τ    2τ    3τ    4τ
```

---

## 2. Transfer Functions: The Language of Control Systems

### 2.1 What Is a Transfer Function?

A **transfer function** $G(s)$ describes the input-output relationship of a linear time-invariant (LTI) system:

$$
G(s) = \frac{Y(s)}{U(s)} = \frac{\text{Output}}{\text{Input}}
$$

```
              ┌──────────┐
  U(s) ─────→│   G(s)   │─────→ Y(s)
  (Input)     └──────────┘     (Output)
```

The transfer function tells us **everything** about how the system behaves — its speed, stability, oscillation, and steady-state accuracy.

### 2.2 Poles and Zeros: The DNA of a System

Every transfer function can be factored as:

$$
G(s) = K \cdot \frac{(s - z_1)(s - z_2) \cdots}{(s - p_1)(s - p_2) \cdots}
$$

- **Zeros** ($z_i$): values of $s$ where $G(s) = 0$ — they shape the response
- **Poles** ($p_i$): values of $s$ where $G(s) \to \infty$ — they determine **stability and speed**

**The Golden Rule:** A system is **stable** if and only if **all poles have negative real parts** (they lie in the left half of the complex plane).

| Pole Location | System Behavior |
|:---:|:---|
| Real, negative ($s = -a$) | Exponential decay $e^{-at}$ — stable |
| Real, positive ($s = +a$) | Exponential growth $e^{at}$ — **unstable!** |
| Complex, negative real part ($s = -a \pm j\omega$) | Damped oscillation — stable |
| Purely imaginary ($s = \pm j\omega$) | Sustained oscillation — marginally stable |
| Complex, positive real part ($s = +a \pm j\omega$) | Growing oscillation — **unstable!** |

```
  Imaginary (jω)
       │
  ×    │    ×         × = pole location
  (stable)  (unstable)
       │
───────┼──────── Real (σ)
       │
  ×    │    ×
  (stable)  (unstable)
       │
  LEFT │ RIGHT
 (stable)(unstable)
```

### 2.3 Standard Second-Order System

Many real systems can be approximated by the **standard second-order form**:

$$
G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

where:
- $\omega_n$ = **natural frequency** (how fast the system wants to oscillate)
- $\zeta$ = **damping ratio** (how quickly oscillations die out)

| $\zeta$ | Behavior | Description |
|:---:|:---|:---|
| $\zeta = 0$ | Undamped | Oscillates forever |
| $0 < \zeta < 1$ | Underdamped | Oscillates with decreasing amplitude |
| $\zeta = 1$ | Critically damped | Fastest response without overshoot |
| $\zeta > 1$ | Overdamped | Slow, no oscillation |

```
Step Response for different ζ:

y(t)
 │     ζ=0 (oscillates forever)
 │    ╱╲  ╱╲  ╱╲  ╱╲
 │   ╱  ╲╱  ╲╱  ╲╱  ╲
1├─·╱·····················─── ζ=1.0 (critically damped)
 │╱   \_____________________  ζ=0.7 (common target)
 │╱  ╱
 │  ╱    ζ=2.0 (overdamped, slow)
 │ ╱   ╱─────────────────────
 │╱  ╱
 └──────────────────────────── t
```

Engineers typically aim for $\zeta \approx 0.7$ — it gives a fast response with minimal overshoot (~5%).

---

## 3. PID Control: The Algorithm

### 3.1 The Control Loop

A standard feedback control loop looks like this:

```
                          ┌──────────────┐
  r(t) ──→(+)──→ e(t) ──→│     PID      │──→ u(t) ──→┌────────┐──→ y(t)
  (setpoint) │            │  Controller  │            │ Plant  │  (output)
             │            └──────────────┘            │ G(s)   │
             │  -                                     └────────┘
             │                                             │
             └─────────────────────────────────────────────┘
                            feedback (sensor)
```

- **$r(t)$** = reference / setpoint (what you want)
- **$y(t)$** = measured output (what you have)
- **$e(t) = r(t) - y(t)$** = error (the difference)
- **$u(t)$** = control signal (what you send to the actuator)
- **$G(s)$** = plant (the physical system you're controlling)

### 3.2 The PID Equation

The PID controller computes the control signal as:

$$
u(t) = \underbrace{K_p \cdot e(t)}_{\text{Proportional}} + \underbrace{K_i \int_0^t e(\tau) \, d\tau}_{\text{Integral}} + \underbrace{K_d \frac{de(t)}{dt}}_{\text{Derivative}}
$$

Each term serves a specific purpose:

### 3.3 P — Proportional: "React to the Present"

$$
u_P(t) = K_p \cdot e(t)
$$

The output is **proportional** to the current error. Bigger error → stronger correction.

```
Error:  e(t) = 10°C    →  Output: Kp × 10 = strong heating
Error:  e(t) = 1°C     →  Output: Kp × 1  = gentle heating
Error:  e(t) = 0°C     →  Output: Kp × 0  = no heating (problem!)
```

**Problem:** With P-only control, the system often settles at a **steady-state error**. Why? Because as the error decreases, so does the control effort — eventually the system reaches equilibrium *before* the error reaches zero.

```
           setpoint ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
y(t)                            _______________
 │                          ╱
 │                       ╱    ← steady-state error
 │                    ╱        (never reaches setpoint)
 │                 ╱
 │              ╱
 └──────────────────────────────────── t
```

**$K_p$ too small:** Sluggish response, large steady-state error.
**$K_p$ too large:** Fast response, but oscillation and potential instability.

### 3.4 I — Integral: "Remember the Past"

$$
u_I(t) = K_i \int_0^t e(\tau) \, d\tau
$$

The integral term **accumulates** past errors over time. Even if the current error is small, the accumulated error keeps growing, pushing the output harder until the error is truly zero.

```
Time     Error    Accumulated (∫e dt)    Action
───────────────────────────────────────────────
t=0      10       0                      Start accumulating
t=1      5        7.5                    Still accumulating
t=2      2        11.0                   Growing stronger
t=3      2        13.0                   Keeps pushing!
t=4      1        14.5                   Won't stop until e=0
t=5      0        15.0                   Finally stops growing
```

**The I-term eliminates steady-state error** — it's the only term that guarantees zero error at steady state.

**Problem — Integral Windup:** If the actuator saturates (e.g., motor at max voltage), the integral keeps accumulating while the system can't respond. When the error finally reverses, the bloated integral causes massive overshoot.

```
         saturation limit
─ ─ ─ ─ ─┬─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
          │  ╱╲
          │ ╱  ╲  ← overshoot from windup
          │╱    ╲___________________
          ╱
        ╱ │
      ╱   │
    ╱     │ ← integral accumulates during saturation
──────────┴─────────────────────────── t
```

**Solution: Anti-windup** — clamp the integral when the actuator is saturated.

### 3.5 D — Derivative: "Predict the Future"

$$
u_D(t) = K_d \frac{de(t)}{dt}
$$

The derivative term responds to the **rate of change** of the error. It acts like a "brake" — if the error is decreasing quickly, it reduces the control effort to prevent overshoot.

```
Scenario 1: Error decreasing fast (de/dt << 0)
→ D-term applies negative force → "Slow down, you're approaching target!"

Scenario 2: Error increasing (de/dt > 0)
→ D-term applies extra positive force → "Speed up, you're falling behind!"

Scenario 3: Error stable (de/dt ≈ 0)
→ D-term contributes nothing
```

```
Without D (P+I only):          With D (full PID):

y(t)                            y(t)
 │      ╱╲                      │
 │     ╱  ╲  ╱╲                 │        _______________
1├────╱────╲╱──╲────── ←oscillates  1├──────╱
 │   ╱                           │    ╱  ← smooth, fast settling
 │  ╱                            │  ╱
 │ ╱                             │ ╱
 └──────────────────── t         └──────────────────── t
```

**Problem:** The D-term amplifies **high-frequency noise**. In practice, a **low-pass filter** is applied:

$$
D_{\text{filtered}}(s) = \frac{K_d s}{1 + \frac{K_d}{N}s}
$$

where $N$ is the filter coefficient (typically 10–100).

### 3.6 Summary: What Each Term Does

| Term | Responds To | Primary Effect | Side Effect |
|:---:|:---:|:---|:---|
| **P** | Present error | Fast response, reduces error | Steady-state error remains |
| **I** | Accumulated past error | Eliminates steady-state error | Overshoot, windup |
| **D** | Rate of error change | Reduces overshoot, damping | Noise amplification |

```
 ┌──────────────────────────────────────────┐
 │              PID Controller              │
 │                                          │
 │  e(t) ──┬──→ [Kp × e] ──────┐           │
 │         │                     │           │
 │         ├──→ [Ki × ∫e dt] ──┼──→ Σ ──→ u(t)
 │         │                     │           │
 │         └──→ [Kd × de/dt] ──┘           │
 │                                          │
 └──────────────────────────────────────────┘
```

---

## 4. PID in the Laplace Domain

### 4.1 PID Transfer Function

Applying the Laplace transform to the PID equation (recall: differentiation → multiplication by $s$, integration → division by $s$):

$$
U(s) = K_p E(s) + K_i \frac{E(s)}{s} + K_d s \, E(s)
$$

$$
C(s) = \frac{U(s)}{E(s)} = K_p + \frac{K_i}{s} + K_d s
$$

Combining into a single fraction:

$$
\boxed{C(s) = \frac{K_d s^2 + K_p s + K_i}{s}}
$$

This is the **PID transfer function**. Notice:
- It has a **pole at $s = 0$** (from the integral term) — this is what ensures zero steady-state error
- It has **two zeros** (from the numerator) — these can be placed to shape the response

### 4.2 Closed-Loop Transfer Function

The closed-loop transfer function with plant $G(s)$ and controller $C(s)$:

$$
T(s) = \frac{C(s) \cdot G(s)}{1 + C(s) \cdot G(s)}
$$

This is the most important equation in control theory. It tells us the **complete behavior** of the controlled system.

### 4.3 Using Laplace to Determine System Parameters

**"How do we find the coefficients?"** — This is a central question in control engineering. The Laplace transform enables a powerful workflow:

**Step 1: Model the plant** — Derive $G(s)$ from physics or system identification.

**Step 2: Define requirements** — Desired settling time, overshoot, steady-state error.

**Step 3: Solve for PID gains** — Use the closed-loop transfer function $T(s)$ and match it to the desired characteristic equation.

**Example: Matching to a desired second-order response.**

Suppose our plant is $G(s) = \frac{K}{s + a}$ and we want the closed-loop to behave like:

$$
T_{\text{desired}}(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

With a PI controller $C(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}$:

$$
T(s) = \frac{\frac{(K_p s + K_i)K}{s(s+a)}}{1 + \frac{(K_p s + K_i)K}{s(s+a)}} = \frac{K(K_p s + K_i)}{s^2 + (a + KK_p)s + KK_i}
$$

Matching the denominator to $s^2 + 2\zeta\omega_n s + \omega_n^2$:

$$
a + KK_p = 2\zeta\omega_n \quad \Rightarrow \quad K_p = \frac{2\zeta\omega_n - a}{K}
$$

$$
KK_i = \omega_n^2 \quad \Rightarrow \quad K_i = \frac{\omega_n^2}{K}
$$

**This is the power of the Laplace transform** — it turns the "what PID gains should I use?" question into a straightforward algebraic calculation.

---

## 5. Example 1: DC Motor with Encoder

### 5.1 The System

A DC motor drives a wheel, and an encoder measures the angular position $\theta$. We want to control the motor's **angular velocity** $\omega$.

```
                    ┌───────────────┐
  Voltage ─────────→│   DC Motor    │─────→ ω (angular velocity)
  u(t)              │               │        │
                    └───────────────┘        │
                           │                 │
                           └── Shaft ────────┤
                                             │
                    ┌───────────────┐         │
  ω_measured ←──────│   Encoder     │←────────┘
                    │ (quadrature)  │
                    └───────────────┘
```

### 5.2 Deriving the Plant Transfer Function

**Motor physics** (electrical and mechanical equations):

**Electrical side (armature circuit):**

$$
V(t) = L\frac{di}{dt} + Ri + K_e\omega
$$

where:
- $V$ = applied voltage
- $L$ = armature inductance
- $R$ = armature resistance
- $i$ = armature current
- $K_e$ = back-EMF constant
- $\omega$ = angular velocity

**Mechanical side (Newton's second law for rotation):**

$$
J\frac{d\omega}{dt} = K_t i - B\omega
$$

where:
- $J$ = moment of inertia
- $K_t$ = torque constant (for an ideal motor, $K_t = K_e = K_m$)
- $B$ = viscous friction coefficient

**Laplace transform** (zero initial conditions):

$$
V(s) = LsI(s) + RI(s) + K_e\Omega(s) = (Ls + R)I(s) + K_e\Omega(s)
$$

$$
Js\Omega(s) = K_t I(s) - B\Omega(s) \quad \Rightarrow \quad I(s) = \frac{(Js + B)\Omega(s)}{K_t}
$$

Substituting $I(s)$ into the electrical equation:

$$
V(s) = (Ls + R)\frac{(Js + B)}{K_t}\Omega(s) + K_e\Omega(s)
$$

$$
V(s) = \left[\frac{(Ls + R)(Js + B) + K_e K_t}{K_t}\right]\Omega(s)
$$

$$
G(s) = \frac{\Omega(s)}{V(s)} = \frac{K_t}{(Ls + R)(Js + B) + K_m^2}
$$

**Simplification** (for small motors, $L$ is often negligible: $L \approx 0$):

$$
G(s) = \frac{K_t}{R(Js + B) + K_m^2} = \frac{K_t / (RB + K_m^2)}{\frac{RJ}{RB + K_m^2}s + 1}
$$

This simplifies to a **first-order system**:

$$
\boxed{G(s) = \frac{K_m}{(\tau_m s + 1)}}
$$

where:
- $K_m = \frac{K_t}{RB + K_m^2}$ (motor gain — steady-state speed per volt)
- $\tau_m = \frac{RJ}{RB + K_m^2}$ (mechanical time constant)

### 5.3 Numerical Example

Typical small DC motor parameters:

| Parameter | Value | Unit |
|:---|:---:|:---:|
| $R$ (resistance) | 2.0 | Ω |
| $J$ (inertia) | 0.01 | kg·m² |
| $B$ (friction) | 0.1 | N·m·s/rad |
| $K_t = K_e$ | 0.5 | V·s/rad |

Computing:

$$
K_m = \frac{0.5}{2.0 \times 0.1 + 0.25} = \frac{0.5}{0.45} \approx 1.11 \text{ rad/s/V}
$$

$$
\tau_m = \frac{2.0 \times 0.01}{0.45} \approx 0.044 \text{ s}
$$

$$
G(s) = \frac{1.11}{0.044s + 1}
$$

### 5.4 PID Design for the Motor

**Requirement:** Reach target speed in 50 ms with < 5% overshoot.

For < 5% overshoot: $\zeta \geq 0.7$. For 50 ms settling time ($t_s \approx \frac{4}{\zeta\omega_n}$):

$$
\omega_n = \frac{4}{\zeta \cdot t_s} = \frac{4}{0.7 \times 0.05} \approx 114 \text{ rad/s}
$$

Using PI control with $G(s) = \frac{1.11}{0.044s + 1}$:

$$
K_p = \frac{2\zeta\omega_n\tau_m - 1}{K_m} = \frac{2 \times 0.7 \times 114 \times 0.044 - 1}{1.11} \approx 5.41
$$

$$
K_i = \frac{\omega_n^2 \tau_m}{K_m} = \frac{114^2 \times 0.044}{1.11} \approx 515
$$

### 5.5 Encoder Feedback

The encoder converts mechanical rotation into digital pulses:

```
Encoder Output (Quadrature):

Ch A: ──┐  ┌──┐  ┌──┐  ┌──
        │  │  │  │  │  │
        └──┘  └──┘  └──┘

Ch B:    ──┐  ┌──┐  ┌──┐  ┌──     ← 90° phase shift
           │  │  │  │  │  │
           └──┘  └──┘  └──┘

Direction: A leads B → Forward
           B leads A → Reverse
```

**Velocity measurement:**

$$
\omega = \frac{\Delta\theta}{\Delta t} = \frac{2\pi \cdot \Delta\text{counts}}{PPR \cdot \Delta t}
$$

where PPR = pulses per revolution (after quadrature decoding: 4× raw PPR).

### 5.6 Implementation (C Code)

```c
// PID controller for DC motor speed control
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_max;     // Anti-windup limit
    float dt;               // Sample period (seconds)
} PID_t;

float pid_compute(PID_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // Proportional
    float P = pid->Kp * error;

    // Integral with anti-windup
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I = pid->Ki * pid->integral;

    // Derivative (on measurement to avoid derivative kick)
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    return P + I + D;
}

// Usage in timer ISR (e.g., every 1 ms)
void TIM2_IRQHandler(void) {
    float rpm_target = 3000.0f;
    float rpm_actual = encoder_get_rpm();

    float voltage = pid_compute(&motor_pid, rpm_target, rpm_actual);

    // Clamp output to valid PWM range
    if (voltage > 12.0f) voltage = 12.0f;
    if (voltage < 0.0f) voltage = 0.0f;

    set_pwm_duty(voltage / 12.0f);    // Normalize to 0.0–1.0
    clear_timer_flag();
}
```

---

## 6. Example 2: Lane Centering (Autonomous Driving)

### 6.1 The System

A camera detects the vehicle's **lateral offset** $e_{\text{lat}}$ from the lane center, and a steering controller brings it back to center.

```
    ┌─────── Lane Boundary ───────────────────────┐
    │                                              │
    │        ┌───────┐                             │
    │        │       │  ← Vehicle                  │
    │        │  ╔═╗  │                             │
    │        │  ║ ║  │  e_lat                      │
    │   ─ ─ ─│─ ╟─╢ ─│─ ─ ─ ← Lane Center        │
    │        │  ║ ║  │  ↕ (lateral offset)         │
    │        │  ╚═╝  │                             │
    │        │       │                             │
    │        └───────┘                             │
    │                                              │
    └─────── Lane Boundary ───────────────────────┘
```

### 6.2 The Lateral Dynamics Model

For a vehicle moving at constant longitudinal velocity $v_x$, the **bicycle model** approximates lateral dynamics:

$$
m\ddot{y} = F_{yf} + F_{yr}
$$

$$
I_z\ddot{\psi} = L_f F_{yf} - L_r F_{yr}
$$

where:
- $y$ = lateral position
- $\psi$ = heading (yaw) angle
- $F_{yf}, F_{yr}$ = front/rear tire lateral forces
- $L_f, L_r$ = distance from CG to front/rear axle
- $m$ = vehicle mass, $I_z$ = yaw moment of inertia

For small angles, tire forces are proportional to slip angles:

$$
F_{yf} = C_f \alpha_f, \quad F_{yr} = C_r \alpha_r
$$

After linearization around straight-line driving, the transfer function from **steering angle $\delta$** to **lateral offset $e_{\text{lat}}$** is approximately:

$$
G_{\text{lat}}(s) = \frac{E_{\text{lat}}(s)}{\Delta(s)} \approx \frac{v_x(C_f L_f s + C_f v_x + C_r v_x)}{s^2(ms^2 + \frac{C_f + C_r}{v_x}s + C_f L_f - C_r L_r)}
$$

**Simplified model** (at constant speed, dominant dynamics):

$$
G(s) \approx \frac{K_{\text{steer}}}{s(\tau s + 1)}
$$

This is an **integrator** (the $1/s$ term: steering angle integrates into lateral position) cascaded with a first-order lag.

### 6.3 PD Controller Design

For lane centering, we typically use a **PD controller** (not full PID) because:
- The plant already has an integrator ($1/s$) — adding another (Ki) can cause oscillation
- We want smooth, damped corrections (no jerky steering)

$$
C(s) = K_p + K_d s
$$

The control law in the time domain:

$$
\delta(t) = K_p \cdot e_{\text{lat}}(t) + K_d \cdot \dot{e}_{\text{lat}}(t)
$$

**Physical interpretation:**
- $K_p \cdot e_{\text{lat}}$: "You're 30 cm to the right → steer left proportionally"
- $K_d \cdot \dot{e}_{\text{lat}}$: "You're drifting right at 10 cm/s → steer left harder to counteract"

### 6.4 Adding a Heading Term

In practice, lane centering uses both lateral offset and **heading error** $e_\psi$ (the angle between the vehicle and the lane):

$$
\delta(t) = K_p \cdot e_{\text{lat}} + K_d \cdot \dot{e}_{\text{lat}} + K_\psi \cdot e_\psi
$$

```
    Lane direction →
    ─────────────────────────────────────

              ╱  Vehicle heading
             ╱ }  e_ψ (heading error)
    ─ ─ ─ ─╱─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
           ╱     ↕ e_lat (lateral offset)
    ─────────────────────────────────────
```

### 6.5 Numerical Example

| Parameter | Value | Description |
|:---|:---:|:---|
| $v_x$ | 20 m/s | Vehicle speed (72 km/h) |
| $K_p$ | 0.05 rad/m | Lateral offset gain |
| $K_d$ | 0.3 rad·s/m | Lateral rate gain |
| $K_\psi$ | 1.0 rad/rad | Heading correction gain |

Scenario: Vehicle is 0.3 m right of center, drifting right at 0.1 m/s, heading error 2°.

$$
\delta = 0.05 \times 0.3 + 0.3 \times 0.1 + 1.0 \times \frac{2\pi}{180} \times 2
$$

$$
\delta = 0.015 + 0.03 + 0.035 = 0.08 \text{ rad} \approx 4.6°
$$

The controller commands 4.6° of left steering — a smooth, gradual correction.

### 6.6 Speed-Dependent Gain Scheduling

At higher speeds, the same steering angle causes a larger lateral response. To maintain consistent behavior, gains are often **scheduled** based on speed:

$$
K_p(v_x) = \frac{K_{p0}}{v_x}, \quad K_d(v_x) = \frac{K_{d0}}{v_x}
$$

```
Kp
 │╲
 │  ╲
 │    ╲
 │      ╲___________
 │
 └──────────────────── vx (speed)
   30    60    90    120 km/h

At low speed:  Aggressive corrections (parking)
At high speed: Gentle corrections (highway)
```

---

## 7. Example 3: Target Volume Control (Audio System)

### 7.1 The System

A digital audio system adjusts the output volume to match a target loudness level, measured in decibels (dB). The human ear perceives loudness **logarithmically**, and the audio amplifier has its own dynamics.

```
  Target     ┌─────────┐     Gain      ┌───────────┐     Actual
  Volume ──→ │   PID   │ ────────────→ │   Audio   │ ──→ Volume
  (dB)       │Controller│   (digital   │ Amplifier │     (dB)
             └─────────┘    gain)      │ + Speaker │
                  ↑                     └───────────┘
                  │                          │
                  │    ┌──────────┐          │
                  └────│ Loudness │←─────────┘
                       │  Meter   │  (microphone)
                       └──────────┘
```

### 7.2 The Plant Model

An audio amplifier with automatic gain control can be modeled as a **first-order system with logarithmic scaling**:

$$
\frac{dL}{dt} = \frac{1}{\tau_a}(G \cdot L_{\text{in}} - L)
$$

where:
- $L$ = output loudness level (dB)
- $G$ = applied gain (controlled by PID output)
- $L_{\text{in}}$ = input signal level
- $\tau_a$ = amplifier time constant

In the Laplace domain:

$$
G_{\text{amp}}(s) = \frac{K_a}{\tau_a s + 1}
$$

However, the feedback measurement (via microphone + RMS computation) adds its own delay:

$$
G_{\text{sensor}}(s) = \frac{1}{\tau_s s + 1}
$$

Combined plant:

$$
G(s) = \frac{K_a}{(\tau_a s + 1)(\tau_s s + 1)}
$$

### 7.3 PID Design

**Requirements:**
- Settle to target volume within 200 ms
- No audible overshoot (< 2 dB, or $\approx$ 1.26× perceived)
- Eliminate steady-state offset (integral action needed)

Typical parameters for a conference room auto-volume system:

| Parameter | Value |
|:---|:---:|
| $\tau_a$ (amplifier) | 10 ms |
| $\tau_s$ (sensor RMS window) | 50 ms |
| $K_a$ (amplifier gain) | 1.0 dB/dB |

Using PI control (D-term omitted to avoid amplifying audio noise):

$$
C(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}
$$

With the combined plant, the closed-loop characteristic equation is:

$$
s(\tau_a s + 1)(\tau_s s + 1) + K_a(K_p s + K_i) = 0
$$

Expanding:

$$
\tau_a \tau_s s^3 + (\tau_a + \tau_s)s^2 + (1 + K_a K_p)s + K_a K_i = 0
$$

For desired $\zeta = 0.8$ and $t_s = 200$ ms ($\omega_n \approx 25$ rad/s), we can use pole placement or root locus to find suitable $K_p$ and $K_i$.

A good starting point using the dominant pole approximation:

$$
K_p = \frac{2\zeta\omega_n(\tau_a + \tau_s) - 1}{K_a} = \frac{2 \times 0.8 \times 25 \times 0.06 - 1}{1.0} = 1.4
$$

$$
K_i = \frac{\omega_n^2(\tau_a + \tau_s)}{K_a} = \frac{625 \times 0.06}{1.0} = 37.5
$$

### 7.4 Practical Considerations

```
Scenario: Noisy conference room

                    Target: 65 dB
                    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
Measured    65 ───────────────────────────────────
Volume (dB) 60    ╱
            55  ╱
            50 ╱    ← speaker adjusting up

            Time →  0    100ms  200ms  300ms

When someone stops talking:
            70   ╲
            65 ───╲─────────────────────────────── Target
                   ╲___________________________

            → Gain increases to compensate for lost input
```

**Anti-windup is critical:** When the speaker is at maximum volume, the integral should stop accumulating. Otherwise, when the audio source suddenly gets louder, the overcharged integral causes painfully loud overshoot.

---

## 8. Example 4: Thrust Control (Drone / Rocket)

### 8.1 The System

A drone (or rocket) must maintain a **target altitude** by controlling the thrust of its motors. This is a classic control problem where gravity provides a constant disturbance.

```
                    ↑ Thrust T(t)
                    │
               ┌────┴────┐
               │  Drone   │ ← mass m
               │  ╔════╗  │
               │  ║    ║  │
               └──╨────╨──┘
                    │
                    ↓ Weight mg

  Altitude h(t)
       ↑
       │  Target: h_ref
       │
```

### 8.2 Deriving the Plant Transfer Function

**Newton's second law** (vertical motion):

$$
m\ddot{h} = T - mg - D(\dot{h})
$$

where:
- $h$ = altitude
- $T$ = thrust force
- $m$ = vehicle mass
- $g$ = gravitational acceleration (9.81 m/s²)
- $D(\dot{h})$ = aerodynamic drag (linearized: $D \approx b\dot{h}$)

**Linearization around hover:** At hover, $T_0 = mg$. Let $\delta T = T - T_0$ be the thrust deviation:

$$
m\ddot{h} = \delta T - b\dot{h}
$$

**Laplace transform:**

$$
ms^2 H(s) = \delta T(s) - bsH(s)
$$

$$
G(s) = \frac{H(s)}{\delta T(s)} = \frac{1}{ms^2 + bs}= \frac{1}{s(ms + b)}
$$

This is a **double integrator** (with damping). Without control, the system is **marginally stable** at best — any uncompensated thrust error leads to unbounded altitude drift.

### 8.3 Simplified Model (Neglecting Drag)

For a small drone at low speeds ($b \approx 0$):

$$
G(s) = \frac{1}{ms^2}
$$

This is a pure **double integrator** — the hardest type of plant to control because it has two poles at $s = 0$ (on the stability boundary).

### 8.4 PID Design

With a double-integrator plant, **PD control is the minimum** needed for stability:

$$
C(s) = K_p + K_d s + \frac{K_i}{s}
$$

Closed-loop with $G(s) = \frac{1}{ms^2}$:

$$
T(s) = \frac{C(s) \cdot G(s)}{1 + C(s) \cdot G(s)} = \frac{K_d s^2 + K_p s + K_i}{ms^3 + K_d s^2 + K_p s + K_i}
$$

The characteristic equation:

$$
ms^3 + K_d s^2 + K_p s + K_i = 0
$$

Using Routh-Hurwitz stability criteria, the system is stable when:

$$
K_d > 0, \quad K_p > 0, \quad K_i > 0, \quad K_d K_p > m K_i
$$

### 8.5 Numerical Example: Quadcopter Altitude Hold

| Parameter | Value | Unit |
|:---|:---:|:---:|
| $m$ (mass) | 1.5 | kg |
| $g$ (gravity) | 9.81 | m/s² |
| Hover thrust $T_0 = mg$ | 14.7 | N |

**Design for:** $\zeta = 0.8$, $\omega_n = 5$ rad/s (settling time ~1 s)

For a third-order system, we place a dominant second-order pair plus a fast real pole at $s = -p$ where $p \gg \zeta\omega_n$:

$$
(s^2 + 2\zeta\omega_n s + \omega_n^2)(s + p) = s^3 + (2\zeta\omega_n + p)s^2 + (\omega_n^2 + 2\zeta\omega_n p)s + \omega_n^2 p
$$

Choosing $p = 10\zeta\omega_n = 40$ rad/s:

$$
s^3 + 48s^2 + 345s + 1000
$$

Matching coefficients with $ms^3 + K_d s^2 + K_p s + K_i$:

$$
K_d = 1.5 \times 48 = 72 \text{ N·s/m}
$$

$$
K_p = 1.5 \times 345 = 517.5 \text{ N/m}
$$

$$
K_i = 1.5 \times 1000 = 1500 \text{ N/(m·s)}
$$

Verify stability: $K_d K_p = 72 \times 517.5 = 37{,}260 > mK_i = 1.5 \times 1500 = 2{,}250$ ✓

### 8.6 The Complete Altitude Control Loop

```
                         ┌──────────────────────────────┐
  h_ref ──→(+)──→ e ──→ │  PID: Kp·e + Ki∫e + Kd·ė    │──→ δT
  (target   │            └──────────────────────────────┘     │
  altitude) │                                                  │
            │  -                                               ▼
            │                                            T = mg + δT
            │                                                  │
            │           ┌──────────────────────┐               │
            │           │    Drone Physics     │               │
            └───────────│  m·ḧ = T - mg - bḣ  │←──────────────┘
                        └──────────────────────┘
                                 │
                                 ▼
                              h(t) (actual altitude)
                                 │
                        ┌────────┴────────┐
                        │   Barometer /   │
                        │   GPS / LiDAR   │
                        └─────────────────┘
```

**Disturbance rejection (wind gust):**

```
h(t)
 │
 │  Target ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
 │                      ╱╲
 │                     ╱  ╲  Wind gust
 │                    ╱    ╲____________________
 │  ──────────────────
 │
 └──────────────────────────────────────────────── t
                        ↑
                    P reacts immediately
                    D counteracts the rate
                    I corrects residual offset
```

### 8.7 Real-World Layers

In practice, drone altitude control uses a **cascaded PID** architecture:

```
  h_ref ──→ [Position PID] ──→ v_ref ──→ [Velocity PID] ──→ a_ref ──→ [Thrust Mapping] ──→ Motors
               (outer loop)                (inner loop)
               ~10 Hz                      ~50-100 Hz

Inner loop: Fast, stabilizes velocity (easier plant: single integrator)
Outer loop: Slow, tracks position (uses velocity as "actuator")
```

This cascade is more robust than a single PID because:
1. The inner loop linearizes the plant for the outer loop
2. Each loop can be tuned independently
3. The inner loop rejects disturbances before they affect position

---

## 9. Tuning Methods: Finding the Right Gains

### 9.1 Ziegler-Nichols Method (Experimental)

When you don't have a mathematical model, Ziegler-Nichols provides a systematic approach:

**Step 1:** Set $K_i = 0$ and $K_d = 0$.

**Step 2:** Increase $K_p$ until the system oscillates with **constant amplitude** (marginally stable). Record this as $K_u$ (ultimate gain) and measure the oscillation period $T_u$.

**Step 3:** Calculate PID gains:

| Controller | $K_p$ | $K_i$ | $K_d$ |
|:---:|:---:|:---:|:---:|
| P | $0.5 K_u$ | — | — |
| PI | $0.45 K_u$ | $\dfrac{0.54 K_u}{T_u}$ | — |
| PID | $0.6 K_u$ | $\dfrac{1.2 K_u}{T_u}$ | $\dfrac{0.075 K_u T_u}{1}$ |

```
Step 2: Finding Ku and Tu

y(t)
 │     ╱╲    ╱╲    ╱╲    ╱╲
 │    ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲   ← Constant amplitude oscillation
─┼───╱────╲╱────╲╱────╲╱────╲──── setpoint
 │  ╱
 │ ╱    │←── Tu ──→│
 │╱
 └────────────────────────────── t
   At this point: Kp = Ku
```

### 9.2 Cohen-Coon Method (Model-Based)

If you can measure the system's **step response**, extract three parameters:

```
Step Response:
y(t)
 │                    ___________________  K (steady-state gain)
 │                 ╱
 │               ╱
 │             ╱  ← steepest slope = R
 │           ╱
 │─────────╱
 │   L     │
 └─────────┴──────────────────────── t
     ↑ dead time (delay)
```

- $K$ = steady-state gain (final value / step size)
- $L$ = dead time (delay before response starts)
- $\tau$ = time constant (time to reach 63.2% of final value)
- $R = K / \tau$ (maximum slope)

### 9.3 Analytical Pole Placement (What We Did Above)

This is the Laplace-based approach we demonstrated in each example:
1. Model the plant as $G(s)$
2. Choose desired pole locations based on specs ($\zeta$, $\omega_n$)
3. Solve for PID gains algebraically

| Method | Requires | Accuracy | Effort |
|:---|:---|:---:|:---:|
| Ziegler-Nichols | Physical system | Rough starting point | Low |
| Cohen-Coon | Step response data | Moderate | Medium |
| Pole Placement | Mathematical model $G(s)$ | High | High |
| Frequency Response (Bode) | Frequency data | High | Medium-High |

---

## 10. Common Pitfalls and Best Practices

### 10.1 Derivative Kick

When the setpoint changes abruptly, the derivative of the error spikes:

$$
\frac{de}{dt} = \frac{d(r - y)}{dt} = \frac{dr}{dt} - \frac{dy}{dt}
$$

**Solution:** Differentiate only the **measurement**, not the error:

$$
u_D = -K_d \frac{dy}{dt} \quad \text{(derivative on measurement)}
$$

This gives the same damping behavior without the spike when $r$ changes.

### 10.2 Integral Windup Protection

```c
// Anti-windup with back-calculation
float pid_with_anti_windup(PID_t *pid, float error, float u_saturated) {
    float u_raw = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * deriv;

    // Back-calculation: reduce integral by saturation difference
    float saturation_error = u_saturated - u_raw;
    pid->integral += (error + saturation_error / pid->Kp) * pid->dt;

    return u_saturated;
}
```

### 10.3 Sample Rate Considerations

The PID controller runs in **discrete time** on a microcontroller. The sample rate must be:

$$
f_s \geq 10 \times f_{\text{bandwidth}}
$$

| Application | Typical Bandwidth | Minimum Sample Rate |
|:---|:---:|:---:|
| Temperature control | 0.01 Hz | 0.1 Hz (10 s) |
| Volume control | 5 Hz | 50 Hz (20 ms) |
| Motor speed | 50 Hz | 500 Hz (2 ms) |
| Lane centering | 10 Hz | 100 Hz (10 ms) |
| Drone altitude | 20 Hz | 200 Hz (5 ms) |
| Drone attitude | 100 Hz | 1000 Hz (1 ms) |

---

## 11. Summary

### The Laplace Transform Connection

The Laplace transform is not just "used for PID" — it is the **fundamental language** of linear control systems:

| What It Does | How |
|:---|:---|
| Converts differential equations to algebra | $\dfrac{d}{dt} \to s$ |
| Represents systems as transfer functions | $G(s) = \dfrac{Y(s)}{U(s)}$ |
| Enables analytical PID tuning | Match desired poles to characteristic equation |
| Determines stability | Check if all poles have $\text{Re}(s) < 0$ |
| Predicts steady-state behavior | Final Value Theorem: $\lim_{s \to 0} sF(s)$ |

### PID at a Glance

| Term | Formula | Role | Analogy |
|:---:|:---:|:---|:---|
| **P** | $K_p \cdot e$ | React to present | "I see the problem" |
| **I** | $K_i \int e \, dt$ | Correct past accumulation | "This has been wrong for too long" |
| **D** | $K_d \dfrac{de}{dt}$ | Anticipate future | "It's getting worse fast" |

### Four Examples Compared

| System | Plant $G(s)$ | Controller | Key Challenge |
|:---|:---:|:---:|:---|
| DC Motor | $\dfrac{K_m}{\tau_m s + 1}$ | PI | Fast response, encoder noise |
| Lane Centering | $\dfrac{K}{s(\tau s + 1)}$ | PD + heading | Speed-dependent dynamics |
| Volume Control | $\dfrac{K_a}{(\tau_a s+1)(\tau_s s+1)}$ | PI | Audio noise, logarithmic perception |
| Drone Altitude | $\dfrac{1}{ms^2}$ | Full PID | Double integrator, wind disturbance |

---

*PID control is deceptively simple in concept but endlessly deep in practice. The Laplace transform gives us the mathematical clarity to understand why each gain does what it does, and how to systematically design controllers for any linear system. Start with the math, verify with simulation, and tune on real hardware — that's the engineering workflow.*
