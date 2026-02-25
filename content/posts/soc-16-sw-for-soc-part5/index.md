---
title: "[SoC-16] Software for SoC Part 5: Timer and DMA — Precision Timing and Efficient Data Transfer"
date: 2026-02-25
description: "Understanding Timer and DMA hardware operation in embedded SoCs — covering timer modes (counting, PWM, input capture), DMA transfer concepts, and the software required to configure and control them."
categories: ["SoC Design"]
tags: ["SoC", "Timer", "DMA", "PWM", "Embedded Systems", "ARM Cortex-M", "Peripheral Control"]
series: ["SoC Design Course"]
series_order: 16
draft: false
---

{{< katex >}}

## Introduction

In this final post of the SoC Design Course series, we explore two essential peripherals that every embedded engineer must master:

1. **Timers** — for precise timing, PWM generation, and event measurement
2. **DMA (Direct Memory Access)** — for transferring data between peripherals and memory without CPU involvement

Together, these peripherals allow embedded systems to handle time-critical operations and high-throughput data streams while keeping the CPU free for other tasks.

---

## 1. Timer Fundamentals

### 1.1 What Is a Hardware Timer?

A timer is essentially a **counter** driven by a clock signal. It counts up (or down) at a known rate, providing precise timing references:

```
Clock Source (e.g., 48 MHz)
         │
    ┌────┴────┐
    │Prescaler│  ÷ PSC
    │  (PSC)  │
    └────┬────┘
         │
   Timer Clock (e.g., 1 MHz after ÷48)
         │
    ┌────┴────┐
    │ Counter │  Counts 0, 1, 2, ... ARR
    │  (CNT)  │
    └────┬────┘
         │
    ┌────┴─────┐
    │ Compare/ │  Generates events at specific counts
    │ Capture  │
    └──────────┘
```

### 1.2 Key Timer Registers

| Register | Name | Purpose |
|----------|------|---------|
| **CNT** | Counter | Current count value |
| **PSC** | Prescaler | Divides the input clock |
| **ARR** | Auto-Reload | Maximum count value (period) |
| **CCR** | Capture/Compare | Threshold for compare events |
| **CR1** | Control Register 1 | Enable, direction, mode |
| **SR** | Status Register | Event flags (update, capture, compare) |
| **DIER** | DMA/Interrupt Enable | Enable interrupts and DMA requests |

### 1.3 Timer Clock Calculation

$$
f_{timer} = \frac{f_{clock}}{PSC + 1}
$$

$$
T_{period} = \frac{(ARR + 1)}{f_{timer}} = \frac{(ARR + 1) \times (PSC + 1)}{f_{clock}}
$$

**Example:** Generate a 1 ms period with a 48 MHz system clock:

$$
(ARR + 1) \times (PSC + 1) = \frac{48{,}000{,}000}{1{,}000} = 48{,}000
$$

Options:
- PSC = 47, ARR = 999 → Timer clock = 1 MHz, counts to 1000 → 1 ms
- PSC = 0, ARR = 47999 → Timer clock = 48 MHz, counts to 48000 → 1 ms

---

## 2. Timer Modes

### 2.1 Basic Counting Mode

The simplest use: count from 0 to ARR, generate an interrupt on overflow, reset, and repeat.

```
CNT:  0 → 1 → 2 → ... → ARR → 0 → 1 → 2 → ...
                           ↑
                     Update Event (UEV)
                     → Interrupt (if enabled)
```

**Periodic interrupt example:**

```c
// Timer 2 setup for 1 ms periodic interrupt (48 MHz clock)
#define TIM2_CR1    (*(volatile uint32_t *)0x40000000)
#define TIM2_DIER   (*(volatile uint32_t *)0x4000000C)
#define TIM2_SR     (*(volatile uint32_t *)0x40000010)
#define TIM2_CNT    (*(volatile uint32_t *)0x40000024)
#define TIM2_PSC    (*(volatile uint32_t *)0x40000028)
#define TIM2_ARR    (*(volatile uint32_t *)0x4000002C)

volatile uint32_t milliseconds = 0;

void timer2_init(void) {
    // Enable TIM2 clock
    RCC_APB1ENR |= (1 << 0);

    // Set prescaler: 48 MHz / 48 = 1 MHz timer clock
    TIM2_PSC = 47;

    // Set auto-reload: count to 1000 → 1 ms period
    TIM2_ARR = 999;

    // Enable update interrupt
    TIM2_DIER |= (1 << 0);    // UIE = 1

    // Enable TIM2 interrupt in NVIC
    NVIC_ISER = (1 << 28);     // TIM2 = IRQ28 (varies by chip)

    // Start the timer
    TIM2_CR1 |= (1 << 0);     // CEN = 1 (Counter Enable)
}

void TIM2_IRQHandler(void) {
    if (TIM2_SR & (1 << 0)) {  // Update interrupt flag
        TIM2_SR &= ~(1 << 0);  // Clear flag
        milliseconds++;         // Increment system tick
    }
}

void delay_ms(uint32_t ms) {
    uint32_t start = milliseconds;
    while ((milliseconds - start) < ms);
}
```

### 2.2 PWM (Pulse Width Modulation) Mode

PWM generates a periodic signal with controllable duty cycle — essential for:
- LED brightness control
- Motor speed control
- Servo positioning
- Audio generation

```
          ┌──────┐         ┌──────┐         ┌──────┐
          │      │         │      │         │      │
──────────┘      └─────────┘      └─────────┘      └──────
          ← Ton →← Toff →
          ←── Period (ARR) ──→

Duty Cycle = CCR / ARR × 100%
```

**PWM configuration:**

```c
void pwm_init(void) {
    // Enable TIM2 and GPIOA clocks
    RCC_APB1ENR |= (1 << 0);    // TIM2
    RCC_AHB1ENR |= (1 << 0);    // GPIOA

    // Configure PA5 as Alternate Function (TIM2_CH1)
    GPIOA_MODER &= ~(3 << 10);
    GPIOA_MODER |=  (2 << 10);  // AF mode
    GPIOA_AFRL  &= ~(0xF << 20);
    GPIOA_AFRL  |=  (1 << 20);  // AF1 = TIM2

    // Timer configuration
    TIM2_PSC = 47;               // 1 MHz timer clock
    TIM2_ARR = 999;              // 1 kHz PWM frequency

    // PWM Mode 1 on Channel 1
    // OC1M = 110 (PWM Mode 1), OC1PE = 1 (Preload enable)
    TIM2_CCMR1 = (6 << 4) | (1 << 3);

    // Enable Channel 1 output
    TIM2_CCER = (1 << 0);       // CC1E = 1

    // Set duty cycle: 50% = ARR/2 = 500
    TIM2_CCR1 = 500;

    // Start timer
    TIM2_CR1 |= (1 << 0);
}

void set_duty_cycle(uint16_t duty_percent) {
    TIM2_CCR1 = (TIM2_ARR + 1) * duty_percent / 100;
}
```

**PWM Output for different duty cycles:**

```
25% duty (CCR = 250, ARR = 999):
  ┌──┐                 ┌──┐
──┘  └─────────────────┘  └─────────────────

50% duty (CCR = 500, ARR = 999):
  ┌──────┐             ┌──────┐
──┘      └─────────────┘      └─────────────

75% duty (CCR = 750, ARR = 999):
  ┌──────────────┐     ┌──────────────┐
──┘              └─────┘              └─────
```

### 2.3 Input Capture Mode

Measures the **time between external events** (e.g., measuring the frequency of an incoming signal, or measuring pulse width):

```
Input Signal:
──────┐     ┌─────────────┐     ┌──────────
      │     │             │     │
      └─────┘             └─────┘
      ↑                   ↑
   Capture 1           Capture 2
   (CNT = T1)          (CNT = T2)

Period = T2 - T1 (in timer ticks)
Frequency = f_timer / (T2 - T1)
```

```c
volatile uint32_t capture1 = 0, capture2 = 0;
volatile uint32_t period = 0;
volatile int capture_ready = 0;

void input_capture_init(void) {
    // Configure timer channel as input capture
    // CC1S = 01 (IC1 mapped to TI1)
    TIM2_CCMR1 = (1 << 0);

    // Capture on rising edge
    TIM2_CCER = (1 << 0);        // CC1E = 1, CC1P = 0 (rising)

    // Enable capture interrupt
    TIM2_DIER |= (1 << 1);       // CC1IE = 1

    // Start timer (free-running)
    TIM2_ARR = 0xFFFFFFFF;        // Maximum count
    TIM2_CR1 |= (1 << 0);
}

void TIM2_IRQHandler(void) {
    if (TIM2_SR & (1 << 1)) {    // Capture event on CH1
        TIM2_SR &= ~(1 << 1);    // Clear flag

        capture2 = TIM2_CCR1;     // Read captured value
        period = capture2 - capture1;
        capture1 = capture2;
        capture_ready = 1;
    }
}
```

### 2.4 One-Pulse Mode

Generates a **single pulse** of precise duration in response to a trigger:

```
Trigger:    ─────┐
                  │
                  └─────────────────────
Output:     ─────────┐           ┌──────
                      │           │
                      └───────────┘
                      ← Duration →
                        (CCR ticks)
```

Useful for generating precise timing signals, triggering ADC conversions, or controlling stepper motors.

---

## 3. SysTick Timer

### 3.1 Overview

The **SysTick** is a simple 24-bit down-counter built into the Cortex-M core itself (not a peripheral). It's designed to provide a **system tick** for RTOS scheduling:

```c
// SysTick Registers (Core peripherals)
#define SYST_CSR    (*(volatile uint32_t *)0xE000E010)  // Control & Status
#define SYST_RVR    (*(volatile uint32_t *)0xE000E014)  // Reload Value
#define SYST_CVR    (*(volatile uint32_t *)0xE000E018)  // Current Value

void systick_init(uint32_t ticks) {
    SYST_RVR = ticks - 1;        // Set reload value
    SYST_CVR = 0;                // Clear current value
    SYST_CSR = (1 << 2)          // Clock source = processor clock
             | (1 << 1)          // Enable interrupt
             | (1 << 0);         // Enable counter
}

// Called every 1 ms (if configured for 1 ms)
void SysTick_Handler(void) {
    system_ticks++;
}
```

---

## 4. DMA (Direct Memory Access)

### 4.1 The Problem DMA Solves

Without DMA, the CPU must handle every data transfer:

```
Without DMA:
  ADC converts → Interrupt → CPU reads ADC → CPU writes to buffer → CPU resumes
                              ↑ CPU is busy during transfer ↑

With DMA:
  ADC converts → DMA reads ADC → DMA writes to buffer (CPU is free!)
```

For high-throughput peripherals (ADC sampling at 1 MHz, UART at high baud rates, SPI transfers), the CPU would spend most of its time just moving data. DMA offloads this work entirely.

### 4.2 DMA Architecture

```
┌─────────────────────────────────────────────────────┐
│                     DMA Controller                   │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │Channel 1 │  │Channel 2 │  │Channel N │          │
│  │          │  │          │  │          │          │
│  │SRC → DST │  │SRC → DST │  │SRC → DST │          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘          │
│       │             │             │                  │
│  ┌────┴─────────────┴─────────────┴────┐            │
│  │            DMA Bus Arbiter           │            │
│  └──────────────────┬──────────────────┘            │
│                     │                                │
└─────────────────────┼────────────────────────────────┘
                      │
              ┌───────┴───────┐
              │  AHB Bus      │
              ├───────────────┤
       ┌──────┤               ├──────┐
       │      │               │      │
  ┌────┴───┐  │          ┌────┴───┐  │
  │ SRAM   │  │          │  Flash │  │
  └────────┘  │          └────────┘  │
              │                      │
         ┌────┴────┐           ┌─────┴────┐
         │  APB    │           │ Periph   │
         │ Bridge  │           │ (ADC,    │
         └────┬────┘           │  UART,   │
              │                │  SPI)    │
         Peripherals           └──────────┘
```

### 4.3 DMA Transfer Types

| Source | Destination | Example Use Case |
|--------|-------------|-----------------|
| **Peripheral → Memory** | ADC → Buffer | Sampling sensor data |
| **Memory → Peripheral** | Buffer → UART TX | Sending a string |
| **Memory → Memory** | Array → Array | Fast memcpy |

### 4.4 DMA Configuration Parameters

| Parameter | Options | Description |
|-----------|---------|-------------|
| **Source Address** | Peripheral or memory | Where to read data from |
| **Destination Address** | Peripheral or memory | Where to write data to |
| **Transfer Count** | 1–65535 | Number of data items to transfer |
| **Data Width** | Byte, Half-word, Word | Size of each data item (8/16/32 bits) |
| **Direction** | P→M, M→P, M→M | Transfer direction |
| **Circular Mode** | On/Off | Auto-restart when transfer completes |
| **Increment** | Source/Dest/Both/None | Auto-increment address after each transfer |
| **Priority** | Low, Medium, High, Very High | Arbitration between channels |

### 4.5 DMA Example: ADC to Memory Buffer

```c
#define DMA1_CH1_CCR    (*(volatile uint32_t *)0x40020008)
#define DMA1_CH1_CNDTR  (*(volatile uint32_t *)0x4002000C)
#define DMA1_CH1_CPAR   (*(volatile uint32_t *)0x40020010)
#define DMA1_CH1_CMAR   (*(volatile uint32_t *)0x40020014)

#define ADC1_DR         (*(volatile uint32_t *)0x40012440)

uint16_t adc_buffer[256];   // Destination buffer

void dma_adc_init(void) {
    // Enable DMA1 clock
    RCC_AHB1ENR |= (1 << 0);

    // Configure DMA Channel 1
    DMA1_CH1_CCR = 0;           // Disable channel first

    // Peripheral address = ADC data register
    DMA1_CH1_CPAR = (uint32_t)&ADC1_DR;

    // Memory address = our buffer
    DMA1_CH1_CMAR = (uint32_t)adc_buffer;

    // Number of transfers
    DMA1_CH1_CNDTR = 256;

    // Configuration:
    DMA1_CH1_CCR = (1 << 7)     // MINC: Memory increment mode
                 | (1 << 10)    // MSIZE: 16-bit memory
                 | (1 << 8)     // PSIZE: 16-bit peripheral
                 | (1 << 5)     // CIRC: Circular mode
                 | (0 << 4)     // DIR: Read from peripheral
                 | (1 << 1)     // TCIE: Transfer complete interrupt
                 | (1 << 0);    // EN: Enable channel
}

void DMA1_Channel1_IRQHandler(void) {
    // Transfer complete — buffer is full
    DMA1_ISR_CLEAR_FLAG();
    process_adc_data(adc_buffer, 256);
    // In circular mode, DMA automatically restarts
}
```

### 4.6 DMA Transfer Flow

```
Step 1: DMA channel is configured and enabled
Step 2: Peripheral (ADC) signals "data ready" via DMA request
Step 3: DMA arbiter grants access to the channel
Step 4: DMA reads from peripheral data register (ADC_DR)
Step 5: DMA writes to memory buffer (adc_buffer[i])
Step 6: DMA increments memory address, decrements transfer count
Step 7: Repeat Steps 2-6 until transfer count = 0
Step 8: DMA generates Transfer Complete interrupt
        (In circular mode: restart from beginning)
```

### 4.7 Circular Mode vs. Normal Mode

**Normal Mode:**
```
Transfer: [0] [1] [2] ... [N-1] → DONE (interrupt)
          → DMA stops, must be reconfigured to restart
```

**Circular Mode:**
```
Transfer: [0] [1] [2] ... [N-1] [0] [1] [2] ... [N-1] [0] ...
          → DMA auto-restarts, runs continuously
          → Ideal for streaming data (audio, continuous ADC sampling)
```

### 4.8 Double-Buffering

For continuous data processing without missing samples:

```
Buffer A: [DMA writing here]     Buffer B: [CPU processing here]
                ↕ (swap on transfer complete)
Buffer A: [CPU processing here]  Buffer B: [DMA writing here]
```

```c
uint16_t buffer_a[256];
uint16_t buffer_b[256];
volatile int active_buffer = 0;

void DMA_Complete_IRQHandler(void) {
    if (active_buffer == 0) {
        // DMA just filled buffer_a, switch to buffer_b
        DMA1_CH1_CMAR = (uint32_t)buffer_b;
        process_data(buffer_a, 256);    // Process buffer_a
        active_buffer = 1;
    } else {
        DMA1_CH1_CMAR = (uint32_t)buffer_a;
        process_data(buffer_b, 256);    // Process buffer_b
        active_buffer = 0;
    }
}
```

---

## 5. Timer + DMA: Powerful Combinations

### 5.1 Timer-Triggered DMA

A timer can trigger DMA transfers at precise intervals — perfect for periodic ADC sampling:

```
Timer     ──(Update Event)──► DMA Request ──► ADC Read ──► Memory Buffer
(1 kHz)                                                     (1000 samples/sec)

CPU involvement: ZERO (after initial configuration)
```

```c
void timer_triggered_adc_dma(void) {
    // Configure timer for 1 kHz update events
    TIM2_PSC = 47;           // 1 MHz timer clock
    TIM2_ARR = 999;          // 1 ms period = 1 kHz
    TIM2_DIER |= (1 << 8);  // UDE: Update DMA request enable

    // Configure DMA (as above)
    dma_adc_init();

    // Configure ADC for external trigger (TIM2 TRGO)
    // ... ADC configuration ...

    // Start timer
    TIM2_CR1 |= (1 << 0);

    // Now: Timer generates events at 1 kHz
    //       → Each event triggers DMA
    //       → DMA reads ADC and stores in buffer
    //       → CPU is completely free!
}
```

### 5.2 PWM with DMA

For complex LED patterns or motor control waveforms, DMA can automatically update PWM duty cycles from a buffer:

```c
uint16_t pwm_pattern[] = {100, 200, 300, 400, 500, 400, 300, 200};

// DMA reads from pwm_pattern[] and writes to TIM2_CCR1
// Each timer update automatically loads the next duty cycle value
// Result: smooth, complex PWM waveform with zero CPU overhead
```

---

## 6. Putting It All Together: A Complete System

Here's how Timer, DMA, GPIO, and Interrupts work together in a typical embedded application:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application: Motor Controller             │
│                                                              │
│  Timer1 ──(PWM)──► GPIO ──► Motor Driver ──► Motor          │
│     ↑                                                        │
│     │ DMA updates CCR from speed profile buffer              │
│                                                              │
│  Timer2 ──(1kHz)──► DMA Trigger                              │
│                       ↓                                      │
│                    ADC ──(DMA)──► Current Sense Buffer       │
│                                       ↓                      │
│                              DMA Complete Interrupt           │
│                                       ↓                      │
│                              PID Controller (CPU)            │
│                                       ↓                      │
│                              Update Speed Profile            │
│                                                              │
│  EXTI ──(Button Interrupt)──► Start/Stop Motor               │
│                                                              │
│  SysTick ──(1ms)──► System Monitor, Watchdog, LED Blink      │
└─────────────────────────────────────────────────────────────┘
```

The CPU only runs the PID control algorithm and handles button events. All data movement (ADC sampling, PWM updates) is handled by DMA, triggered by timers. This is the essence of efficient embedded system design.

---

## 7. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Timer** | A hardware counter driven by a clock; basis for all timing operations |
| **Prescaler + ARR** | Together determine the timer period: $T = (ARR+1)(PSC+1)/f_{clk}$ |
| **PWM** | Timer compares CNT with CCR to generate variable-duty-cycle waveforms |
| **Input Capture** | Timer captures CNT value on external events to measure timing |
| **SysTick** | Built-in 24-bit timer for RTOS ticks and system timing |
| **DMA** | Hardware data mover — transfers data without CPU involvement |
| **Circular DMA** | Auto-restarts for continuous data streaming |
| **Double-buffering** | Process one buffer while DMA fills the other — no data loss |
| **Timer + DMA** | Timer triggers DMA for precise, periodic, CPU-free data acquisition |

---

## Course Conclusion

Congratulations! You've completed the entire **SoC Design Course** series. Let's recap the journey:

| Posts | Topic Area | What You Learned |
|:-----:|-----------|------------------|
| 01 | AI & SoC | Why hardware matters for AI; SoC overview |
| 02–03 | Digital Foundations | Number systems, logic gates, Boolean algebra, binary arithmetic |
| 04–06 | ISA | Instruction formats, CISC vs RISC, RISC-V in detail |
| 07–09 | CPU Architecture | Single-cycle design, pipelining, hazard resolution |
| 10–11 | Memory | Cache hierarchy, optimization, virtual memory |
| 12–16 | Embedded SW | ARM Cortex-M0+, C-to-assembly, GPIO, interrupts, timers, DMA |

You now have a solid understanding of the **full stack** — from transistors and logic gates up through processor architecture and down to the firmware that controls real hardware. This knowledge is the foundation for designing efficient SoCs, writing performant embedded software, and building the intelligent systems of the future.

---

*Thank you for following the **SoC Design Course** series. Happy engineering!*
