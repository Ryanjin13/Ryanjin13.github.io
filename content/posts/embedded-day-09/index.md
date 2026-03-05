---
title: "Day 9 — PID Control and Encoder Feedback Loop"
date: 2026-03-14
description: "Feedback control systems, P/I/D terms physical meaning, Ziegler-Nichols tuning, integral windup, derivative kick solutions, and velocity PID with Hall encoder"
categories: ["Autonomous Driving"]
tags: ["PID Control", "Feedback Control", "Motor Control", "Tuning", "Anti-windup"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 9
draft: false
---

{{< katex >}}

## What You'll Learn

In Day 8 we built a Kalman filter to clean up noisy sensor data. But filtering alone does not solve the core problem of autonomous driving: **making the car do what you want**. You need a controller that reads the current state, compares it against a desired setpoint, and commands the actuators to close the gap. That controller is the **PID controller**, and it has been the workhorse of industrial control for nearly a century.

By the end of this post you will be able to:

1. Draw a feedback control block diagram and name every signal.
2. Write the PID equation in both continuous and discrete form.
3. Explain the physical meaning of P, I, and D terms with intuitive analogies.
4. Tune PID gains using the Ziegler-Nichols method.
5. Identify and fix **integral windup** and **derivative kick**.
6. Implement a complete **velocity PID** loop that uses the Hall encoder RPM from Day 6.
7. Sketch how a **steering angle PID** works for lateral control.

---

## 1. The Feedback Control Loop

Every feedback controller shares the same skeleton. Memorize the block diagram and you will be able to read any control system paper.

```
                +----------+       +------------+       +---------+
 Setpoint r(t)  |          | u(t)  |            | y(t)  |         |
 ─────────>(+)──┤Controller├──────>│   Plant    ├──────>│ Sensor  ├──┐
            ^   |  (PID)   |       | (DC Motor) |       | (Hall)  |  |
            |   +----------+       +------------+       +---------+  |
            |                                                        |
            |                  e(t) = r(t) - y(t)                    |
            └────────────────────────────────────────────────────────┘
                                 Feedback path
```

| Symbol | Name | Example |
|--------|------|---------|
| \(r(t)\) | **Setpoint** (reference) | Desired wheel speed 300 RPM |
| \(y(t)\) | **Process variable** (measured output) | Hall-encoder measured RPM |
| \(e(t)\) | **Error** | \(r(t) - y(t)\) |
| \(u(t)\) | **Control signal** (actuator command) | PWM duty cycle to motor driver |
| Plant | The physical system being controlled | DC motor + gearbox + wheel |
| Sensor | Measurement device | Hall encoder from Day 6 |

The controller's only job is to compute \(u(t)\) from \(e(t)\) so that \(y(t)\) tracks \(r(t)\) as closely as possible.

---

## 2. The PID Equation

### 2.1 Continuous-Time Form

The PID controller output is the sum of three terms:

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}
$$

where:

- \(K_p\) is the **proportional gain**,
- \(K_i\) is the **integral gain**,
- \(K_d\) is the **derivative gain**.

Some textbooks use the "standard form" parameterized by a single gain \(K\) and two time constants:

$$
u(t) = K\!\left(e(t) + \frac{1}{T_i}\int_0^t e(\tau)\,d\tau + T_d \frac{de(t)}{dt}\right)
$$

where \(T_i = K_p / K_i\) is the integral time and \(T_d = K_d / K_p\) is the derivative time. Both forms are mathematically equivalent; use whichever your textbook prefers.

### 2.2 Discrete-Time Form

Microcontrollers run at a fixed sample period \(\Delta t\). We replace the integral with a running sum and the derivative with a backward difference:

$$
u[k] = K_p\,e[k] \;+\; K_i \sum_{i=0}^{k} e[i]\,\Delta t \;+\; K_d \frac{e[k] - e[k-1]}{\Delta t}
$$

This is the **positional PID** form. Each term maps directly to a few lines of C or Python.

### 2.3 Transfer-Function View

In the Laplace domain the PID controller is:

$$
C(s) = K_p + \frac{K_i}{s} + K_d s
$$

The integrator \(1/s\) provides infinite DC gain (which eliminates steady-state error), while the differentiator \(s\) adds a phase lead that improves transient response. In a Bode plot, the PID looks like a lead-lag compensator.

---

## 3. Physical Meaning of Each Term

Understanding each term intuitively is far more important than memorizing formulas. Let us use a driving analogy: you want to maintain exactly 60 km/h on a hilly road.

### 3.1 Proportional Term — "React to the present"

$$
u_P(t) = K_p \, e(t)
$$

The P term produces an output that is directly proportional to the current error. If you are going 50 km/h (error = +10), you press the gas pedal a certain amount. If the error doubles to 20, you press twice as hard.

**The problem**: proportional-only control always leaves a residual **steady-state error**. Why? Imagine the car reaches 58 km/h. The error is now only 2, so the controller output is small. But that small output is exactly what is needed to overcome the hill's drag. If the car speeds up to 60, the error drops to zero, the output drops to zero, and the car slows down again. The system settles at some speed below 60 where the P output exactly balances the disturbance. This offset is called **droop**.

$$
e_{ss} = \frac{r}{1 + K_p G(0)}
$$

where \(G(0)\) is the DC gain of the plant. Increasing \(K_p\) shrinks the error but never eliminates it — and too much gain causes oscillations.

### 3.2 Integral Term — "Remember the past"

$$
u_I(t) = K_i \int_0^t e(\tau)\,d\tau
$$

The I term accumulates past errors over time. Even if the current error is tiny (say 0.5 km/h), the integrator keeps adding that 0.5 every sample period. Eventually the accumulated value grows large enough to push the output and close the gap completely.

**Key insight**: the integrator keeps growing until the error is zero. That is why it eliminates steady-state error. In the Laplace domain, the \(1/s\) pole at the origin provides infinite gain at DC.

**The danger**: if the system cannot respond fast enough (e.g., motor is already saturated), the integrator keeps accumulating error — this is **integral windup**, discussed in Section 7.

### 3.3 Derivative Term — "Predict the future"

$$
u_D(t) = K_d \frac{de(t)}{dt}
$$

The D term responds to the *rate of change* of the error. If the error is decreasing rapidly (the car is accelerating toward the target), the derivative is negative and the D term pulls back the output, preventing overshoot. If the error is growing, the D term adds extra correction.

Think of it as a damper on a spring-mass system. Without it, a P+I controller can overshoot and oscillate. With proper D gain, the system settles quickly.

**The danger**: if the setpoint changes abruptly (step input), the derivative of error spikes to infinity — this is **derivative kick**, discussed in Section 8.

### 3.4 Summary Table

| Term | Responds to | Effect | Side effect |
|------|-------------|--------|-------------|
| P | Present error | Fast response | Steady-state error |
| I | Past error (accumulated) | Eliminates steady-state error | Windup, slow oscillations |
| D | Future error (rate of change) | Reduces overshoot, adds damping | Noise amplification |

---

## 4. PID Tuning: Ziegler-Nichols Method

Tuning means finding \(K_p, K_i, K_d\) that give acceptable performance. The Ziegler-Nichols (ZN) method is the most famous heuristic.

### 4.1 Ultimate Gain Method

1. Set \(K_i = 0\) and \(K_d = 0\).
2. Gradually increase \(K_p\) until the system oscillates with constant amplitude. This critical gain is the **ultimate gain** \(K_u\).
3. Measure the period of oscillation \(T_u\).
4. Use the table below:

| Controller | \(K_p\) | \(T_i\) | \(T_d\) |
|------------|---------|---------|---------|
| P only | \(0.50\,K_u\) | — | — |
| PI | \(0.45\,K_u\) | \(T_u / 1.2\) | — |
| PID | \(0.60\,K_u\) | \(T_u / 2\) | \(T_u / 8\) |

Convert to parallel form:

$$
K_i = \frac{K_p}{T_i}, \qquad K_d = K_p \cdot T_d
$$

### 4.2 Manual Tuning Order

When ZN is impractical (the system cannot be allowed to oscillate freely), use this procedure:

1. **P only**: increase \(K_p\) until the system responds briskly but does not oscillate violently. Accept some steady-state error for now.
2. **Add I**: start with a small \(K_i\). Increase until the steady-state error vanishes within a reasonable time. If the system starts to oscillate slowly, reduce \(K_i\) or increase \(K_p\) slightly.
3. **Add D**: increase \(K_d\) to dampen any remaining overshoot. If the output becomes jittery, reduce \(K_d\) — you are amplifying sensor noise.

### 4.3 Tuning for Our Autonomous Car

For a DC motor velocity loop at 100 Hz sample rate, typical starting ranges are:

| Parameter | Starting range |
|-----------|---------------|
| \(K_p\) | 0.5 -- 5.0 |
| \(K_i\) | 0.01 -- 1.0 |
| \(K_d\) | 0.001 -- 0.1 |

These depend on motor characteristics, gear ratio, and wheel inertia, so always start small and increase.

---

## 5. The Closed-Loop Transfer Function

To understand stability formally, derive the closed-loop transfer function. With controller \(C(s)\) and plant \(G(s)\):

$$
\frac{Y(s)}{R(s)} = \frac{C(s)\,G(s)}{1 + C(s)\,G(s)}
$$

For a first-order plant \(G(s) = \frac{K_m}{\tau s + 1}\) (a common DC motor model), substituting the PID controller gives:

$$
\frac{Y(s)}{R(s)} = \frac{(K_d s^2 + K_p s + K_i)\,K_m}{(\tau s + 1)\,s + (K_d s^2 + K_p s + K_i)\,K_m}
$$

Stability requires all poles to have negative real parts. Tools like root-locus or Bode plots help visualize this, but for our embedded work the manual tuning approach is more practical.

---

## 6. Velocity PID vs Position PID

There are two common formulations for the discrete PID. So far we described the **positional** form where the output \(u[k]\) is computed from scratch each step. The **velocity** (or incremental) form computes only the **change** in output:

$$
\Delta u[k] = K_p\bigl(e[k] - e[k-1]\bigr) + K_i\,e[k]\,\Delta t + K_d\frac{e[k] - 2e[k-1] + e[k-2]}{\Delta t}
$$

Then:

$$
u[k] = u[k-1] + \Delta u[k]
$$

### Why velocity form matters

| Property | Positional PID | Velocity PID |
|----------|---------------|--------------|
| Integral term | Explicit sum (can overflow) | Built into incremental update |
| Bumpless transfer | Needs extra logic | Natural (no integral state to manage) |
| Anti-windup | Requires clamping | Simpler — just clamp \(\Delta u\) |
| Setpoint change | Can cause large jump | Smoother |

For our Hall-sensor velocity control, the velocity PID form is a natural match: we measure RPM, compute error, and output a PWM delta.

---

## 7. Integral Windup and Anti-Windup

### 7.1 What Is Windup?

When the actuator saturates (e.g., PWM is already at 100% duty), the error remains non-zero but the integrator keeps accumulating. When the setpoint changes direction, the bloated integral must "unwind" before the output can reverse — causing severe overshoot.

```
         Setpoint drops here
              |
RPM  ────────┐         ┌── Actual RPM keeps going up
              │    ___──┘   because integrator is bloated
              │  /
              │/
              ├── Takes this long to unwind
              │
              └─────── Without anti-windup
```

### 7.2 Anti-Windup: Clamping

The simplest and most common fix: **stop accumulating** when the output is saturated.

```
if u_total > u_max or u_total < u_min:
    # Do NOT add current error to integrator
    pass
else:
    integral += error * dt
```

A more refined version is **back-calculation**: when the output saturates, feed the excess back to reduce the integrator:

$$
\frac{d}{dt}(\text{integral}) = e(t) + \frac{1}{T_t}\bigl(u_{\text{saturated}} - u_{\text{unsaturated}}\bigr)
$$

where \(T_t\) is the tracking time constant, typically set to \(\sqrt{T_i \cdot T_d}\).

---

## 8. Derivative Kick and Derivative-on-Measurement

### 8.1 The Problem

When the setpoint changes abruptly (step change), the error \(e[k] - e[k-1]\) can be huge for one sample, causing a spike in the D term output. This spike drives the motor with a sudden burst — the **derivative kick**.

### 8.2 The Fix: Derivative on Measurement

Instead of differentiating the error, differentiate the **measurement** (process variable) only:

$$
u_D[k] = -K_d \frac{y[k] - y[k-1]}{\Delta t}
$$

Note the negative sign: when the setpoint is constant, \(de/dt = -dy/dt\), so this is mathematically equivalent in steady state. But when the setpoint steps, the measurement changes smoothly (it is a physical quantity), avoiding the spike.

**Always use derivative-on-measurement in practice.** This is a standard best practice that costs nothing.

---

## 9. Steering Angle PID — Lateral Control

Besides velocity (longitudinal control), an autonomous car needs **lateral control** — keeping the car centered in its lane.

### 9.1 Cross-Track Error

Define the **cross-track error (CTE)** as the perpendicular distance from the car's center to the desired path.

$$
e_{\text{lateral}} = \text{CTE}
$$

A PID controller computes the steering angle:

$$
\delta(t) = K_p \cdot \text{CTE} + K_i \int \text{CTE}\,dt + K_d \frac{d(\text{CTE})}{dt}
$$

### 9.2 Heading Error

A more sophisticated approach combines CTE with **heading error** \(\psi_e\), the angle between the car's heading and the path tangent:

$$
\delta(t) = K_{p1}\,\text{CTE} + K_{p2}\,\psi_e + K_d \frac{d(\text{CTE})}{dt}
$$

At higher speeds, pure PID steering becomes insufficient and you move to **Stanley** or **Pure Pursuit** controllers, but PID is an excellent starting point for low-speed indoor autonomous cars.

---

## 10. Hands-On Lab: Velocity PID with Hall Encoder

### 10.1 PID Controller Class

```python
"""
pid_controller.py
Complete PID controller with anti-windup and derivative-on-measurement.
"""

import time
import numpy as np
import matplotlib.pyplot as plt


class PIDController:
    """Discrete PID controller with practical improvements."""

    def __init__(self, kp: float, ki: float, kd: float,
                 dt: float = 0.01,
                 output_min: float = 0.0,
                 output_max: float = 100.0):
        # Gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Sample period
        self.dt = dt

        # Output saturation limits (PWM duty: 0-100%)
        self.output_min = output_min
        self.output_max = output_max

        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0

    def reset(self):
        """Reset integrator and derivative state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        """
        Compute PID output.

        Args:
            setpoint:    desired value (e.g., target RPM)
            measurement: current measured value (e.g., Hall encoder RPM)

        Returns:
            Clamped control output (e.g., PWM duty cycle)
        """
        error = setpoint - measurement

        # --- Proportional term ---
        p_term = self.kp * error

        # --- Integral term (with anti-windup clamping) ---
        # Tentatively accumulate
        tentative_integral = self.integral + error * self.dt
        i_term = self.ki * tentative_integral

        # --- Derivative term (on measurement, not error) ---
        d_measurement = (measurement - self.prev_measurement) / self.dt
        d_term = -self.kd * d_measurement  # negative sign!

        # --- Total output (before clamping) ---
        output_unclamped = p_term + i_term + d_term

        # --- Clamp output ---
        output = np.clip(output_unclamped, self.output_min, self.output_max)

        # --- Anti-windup: only update integral if not saturated ---
        if output == output_unclamped:
            # Not saturated, accept the integral update
            self.integral = tentative_integral
        # else: discard the integral accumulation (clamping anti-windup)

        # --- Save state for next iteration ---
        self.prev_error = error
        self.prev_measurement = measurement

        return output
```

### 10.2 Simulated DC Motor Plant

To test our PID without hardware, we model a DC motor as a first-order system:

$$
\frac{dN}{dt} = \frac{1}{\tau}\bigl(-N(t) + K_m \cdot u(t)\bigr) + w(t)
$$

where \(N\) is RPM, \(\tau\) is the motor time constant, \(K_m\) maps PWM duty to steady-state RPM, and \(w(t)\) is process noise.

```python
class DCMotorSim:
    """Simple first-order DC motor simulator."""

    def __init__(self, tau: float = 0.3, km: float = 5.0,
                 noise_std: float = 5.0, dt: float = 0.01):
        self.tau = tau       # time constant [s]
        self.km = km         # gain: RPM per % duty
        self.noise_std = noise_std
        self.dt = dt
        self.rpm = 0.0       # current RPM

    def step(self, pwm_duty: float) -> float:
        """Advance one time step and return noisy RPM measurement."""
        # First-order dynamics
        dN = (1.0 / self.tau) * (-self.rpm + self.km * pwm_duty) * self.dt
        self.rpm += dN

        # Add measurement noise (simulating Hall encoder jitter)
        measured = self.rpm + np.random.normal(0, self.noise_std)
        return measured

    def reset(self):
        self.rpm = 0.0
```

### 10.3 P-only vs PI vs PID Comparison

```python
def run_simulation(kp, ki, kd, title="PID", duration=5.0, dt=0.01):
    """Run closed-loop simulation and return time history."""
    motor = DCMotorSim(tau=0.3, km=5.0, noise_std=5.0, dt=dt)
    pid = PIDController(kp=kp, ki=ki, kd=kd, dt=dt,
                        output_min=0.0, output_max=100.0)

    setpoint = 200.0  # target RPM
    steps = int(duration / dt)

    t_hist = np.zeros(steps)
    rpm_hist = np.zeros(steps)
    setpoint_hist = np.zeros(steps)
    pwm_hist = np.zeros(steps)

    for k in range(steps):
        t_hist[k] = k * dt

        # Step change in setpoint at t=2.5s to test response
        if k * dt < 2.5:
            sp = 200.0
        else:
            sp = 300.0
        setpoint_hist[k] = sp

        # Measure
        if k == 0:
            measurement = 0.0
        else:
            measurement = motor.step(pwm_hist[k - 1])
        rpm_hist[k] = measurement

        # Compute control
        pwm_hist[k] = pid.compute(sp, measurement)

    return t_hist, rpm_hist, setpoint_hist, pwm_hist, title


# --- Run three controllers ---
results = [
    run_simulation(kp=0.8, ki=0.0, kd=0.0,   title="P only (Kp=0.8)"),
    run_simulation(kp=0.8, ki=0.5, kd=0.0,   title="PI (Kp=0.8, Ki=0.5)"),
    run_simulation(kp=0.8, ki=0.5, kd=0.05,  title="PID (Kp=0.8, Ki=0.5, Kd=0.05)"),
]

# --- Plot comparison ---
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

for i, (t, rpm, sp, pwm, title) in enumerate(results):
    axes[i].plot(t, sp, 'r--', label='Setpoint', linewidth=2)
    axes[i].plot(t, rpm, 'b-', alpha=0.7, label='Measured RPM')
    axes[i].set_ylabel('RPM')
    axes[i].set_title(title)
    axes[i].legend(loc='upper left')
    axes[i].grid(True, alpha=0.3)
    axes[i].set_ylim([0, 400])

axes[2].set_xlabel('Time [s]')
plt.tight_layout()
plt.savefig('pid_comparison.png', dpi=150)
plt.show()
```

**What to observe**:

- **P only**: the RPM rises quickly but never reaches the setpoint — there is a visible steady-state error (droop). When the setpoint steps to 300, the gap persists.
- **PI**: the integrator slowly closes the gap, eventually reaching the setpoint. But notice the slower response and possible overshoot.
- **PID**: the D term dampens the overshoot, giving the fastest settling with minimal oscillation.

### 10.4 Anti-Windup Demonstration

```python
def run_windup_comparison(use_antiwindup: bool, title: str):
    """Demonstrate windup vs anti-windup with actuator saturation."""
    dt = 0.01
    motor = DCMotorSim(tau=0.3, km=5.0, noise_std=3.0, dt=dt)

    if use_antiwindup:
        pid = PIDController(kp=0.8, ki=1.0, kd=0.05, dt=dt,
                            output_min=0.0, output_max=100.0)
    else:
        # "Broken" PID: no clamping on integral
        pid = PIDController(kp=0.8, ki=1.0, kd=0.05, dt=dt,
                            output_min=0.0, output_max=100.0)

    steps = int(8.0 / dt)
    t_hist = np.zeros(steps)
    rpm_hist = np.zeros(steps)
    sp_hist = np.zeros(steps)
    integral_hist = np.zeros(steps)

    for k in range(steps):
        t_hist[k] = k * dt

        # High setpoint forces saturation, then drop at t=4s
        if k * dt < 4.0:
            sp = 600.0  # unreachable! motor max ~ 500 RPM
        else:
            sp = 200.0
        sp_hist[k] = sp

        output = pid.compute(sp, motor.rpm) if k > 0 else 0.0
        measurement = motor.step(output)
        rpm_hist[k] = measurement

        if not use_antiwindup:
            # Force integral to keep growing (disable anti-windup)
            error = sp - measurement
            pid.integral += error * dt

        integral_hist[k] = pid.integral

    return t_hist, rpm_hist, sp_hist, integral_hist, title


fig, axes = plt.subplots(2, 2, figsize=(14, 8))

# Without anti-windup
t, rpm, sp, intg, title = run_windup_comparison(False, "WITHOUT Anti-windup")
axes[0, 0].plot(t, sp, 'r--', label='Setpoint')
axes[0, 0].plot(t, rpm, 'b-', label='RPM')
axes[0, 0].set_title(title + " - RPM")
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)
axes[1, 0].plot(t, intg, 'g-')
axes[1, 0].set_title(title + " - Integral")
axes[1, 0].set_xlabel('Time [s]')
axes[1, 0].grid(True, alpha=0.3)

# With anti-windup
t, rpm, sp, intg, title = run_windup_comparison(True, "WITH Anti-windup")
axes[0, 1].plot(t, sp, 'r--', label='Setpoint')
axes[0, 1].plot(t, rpm, 'b-', label='RPM')
axes[0, 1].set_title(title + " - RPM")
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)
axes[1, 1].plot(t, intg, 'g-')
axes[1, 1].set_title(title + " - Integral")
axes[1, 1].set_xlabel('Time [s]')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('antiwindup_comparison.png', dpi=150)
plt.show()
```

**Without anti-windup**: the integral balloons during saturation. When the setpoint drops at \(t=4\)s, the RPM takes a long time to come down because the integrator must unwind.

**With anti-windup**: the integral stays bounded. The RPM responds promptly when the setpoint changes.

### 10.5 Hall Encoder RPM Integration (from Day 6)

In Day 6 we implemented a Hall encoder ISR that computes RPM from pulse intervals. Here is how to integrate it with our PID:

```python
"""
velocity_pid_loop.py
Real-time velocity PID loop for Raspberry Pi 5 with Hall encoder.
Connects to Day 6 Hall encoder and Day 7 PWM output.
"""

import time
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not available — running in simulation mode")
    GPIO = None

# --- Configuration ---
HALL_PIN = 17         # Hall sensor GPIO (from Day 6)
PWM_PIN = 18          # Motor driver PWM GPIO (from Day 7)
PWM_FREQ = 20000      # 20 kHz PWM frequency
ENCODER_PPR = 12      # Pulses per revolution
GEAR_RATIO = 30       # Motor-to-wheel gear ratio
CONTROL_FREQ = 100    # PID loop frequency [Hz]
DT = 1.0 / CONTROL_FREQ

# --- PID Gains (tune for your motor) ---
KP = 0.5
KI = 0.3
KD = 0.02
TARGET_RPM = 200.0

# --- Global state for ISR ---
pulse_count = 0
last_pulse_time = 0.0


def hall_isr(channel):
    """Interrupt service routine for Hall sensor pulse."""
    global pulse_count, last_pulse_time
    pulse_count += 1
    last_pulse_time = time.monotonic()


def compute_rpm() -> float:
    """Compute wheel RPM from pulse count over the control period."""
    global pulse_count
    count = pulse_count
    pulse_count = 0  # reset for next period

    # Motor RPM = (pulses / PPR) / dt * 60
    motor_rpm = (count / ENCODER_PPR) / DT * 60.0
    # Wheel RPM
    wheel_rpm = motor_rpm / GEAR_RATIO
    return wheel_rpm


def main():
    if GPIO is None:
        print("Cannot run without GPIO. Use simulation instead.")
        return

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HALL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PWM_PIN, GPIO.OUT)

    # Set up PWM
    pwm = GPIO.PWM(PWM_PIN, PWM_FREQ)
    pwm.start(0)

    # Set up Hall interrupt
    GPIO.add_event_detect(HALL_PIN, GPIO.RISING, callback=hall_isr)

    # Create PID controller
    pid = PIDController(kp=KP, ki=KI, kd=KD, dt=DT,
                        output_min=0.0, output_max=100.0)

    print(f"Target: {TARGET_RPM} RPM | Gains: Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"{'Time':>8s} {'Setpoint':>10s} {'RPM':>10s} {'PWM%':>8s} {'Error':>8s}")

    try:
        t_start = time.monotonic()
        while True:
            loop_start = time.monotonic()

            # Measure
            current_rpm = compute_rpm()

            # Compute control
            pwm_duty = pid.compute(TARGET_RPM, current_rpm)

            # Actuate
            pwm.ChangeDutyCycle(pwm_duty)

            # Log
            elapsed = time.monotonic() - t_start
            error = TARGET_RPM - current_rpm
            print(f"{elapsed:8.2f} {TARGET_RPM:10.1f} {current_rpm:10.1f} "
                  f"{pwm_duty:8.1f} {error:8.1f}")

            # Wait for next period
            elapsed_loop = time.monotonic() - loop_start
            sleep_time = DT - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
```

### 10.6 Steering PID Skeleton

```python
"""
steering_pid.py
Basic lateral PID for lane-keeping using cross-track error.
This will be expanded in later days with camera-based CTE detection.
"""


class SteeringPID:
    """PID controller for steering angle based on cross-track error."""

    def __init__(self, kp=1.0, ki=0.01, kd=0.5, dt=0.05,
                 max_steer=30.0):
        """
        Args:
            kp, ki, kd: PID gains
            dt: control period [s]
            max_steer: maximum steering angle [degrees]
        """
        self.pid = PIDController(
            kp=kp, ki=ki, kd=kd, dt=dt,
            output_min=-max_steer, output_max=max_steer
        )

    def compute_steering(self, cte: float) -> float:
        """
        Compute steering angle from cross-track error.

        Args:
            cte: cross-track error [meters], positive = car is to the right

        Returns:
            steering angle [degrees], positive = steer left
        """
        # Setpoint is 0 (we want CTE = 0)
        # Measurement is the current CTE
        # The PID output is the steering angle
        steering = self.pid.compute(setpoint=0.0, measurement=cte)
        return steering


# --- Example usage ---
steering_ctrl = SteeringPID(kp=2.0, ki=0.05, kd=1.0)

# Simulated CTE values as car approaches lane center
cte_values = [0.5, 0.45, 0.35, 0.20, 0.08, 0.01, -0.02, -0.01, 0.0]
for cte in cte_values:
    angle = steering_ctrl.compute_steering(cte)
    print(f"CTE: {cte:+.3f} m  ->  Steering: {angle:+.2f} deg")
```

---

## Review

Today we built the most important controller in all of robotics. Here is what we covered:

| Topic | Key takeaway |
|-------|-------------|
| Feedback loop | Error-driven control: \(e = r - y\) |
| PID equation | \(u = K_p e + K_i \int e\,dt + K_d \dot{e}\) |
| P term | Fast but leaves steady-state error |
| I term | Eliminates steady-state error, risk of windup |
| D term | Adds damping, risk of noise amplification |
| Ziegler-Nichols | Find \(K_u, T_u\) then use table |
| Anti-windup | Stop integrating when actuator saturates |
| Derivative kick | Use derivative-on-measurement: \(-K_d \dot{y}\) |
| Velocity PID | Incremental form, natural anti-windup |
| Steering PID | CTE-based lateral control for lane-keeping |

### Connection to Previous Days

- **Day 6** (Hall Encoder): we use the RPM measurement as the PID feedback signal.
- **Day 6** (PWM): the PID output drives the motor through PWM duty cycle.
- **Day 8** (Kalman Filter): filtering the Hall encoder signal before feeding it to PID reduces the D term noise sensitivity.

### What Comes Next

In **Day 10**, we move from controlling the motors to **sensing the environment**. We will explore 1D LiDAR and depth cameras — the eyes of our autonomous car. The distance measurements from those sensors will eventually become inputs to controllers like the PID we built today.
