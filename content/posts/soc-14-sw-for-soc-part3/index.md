---
title: "[SoC-14] Software for SoC Part 3: Firmware and GPIO — Controlling the Physical World"
date: 2026-02-25
description: "Understanding how firmware controls peripheral devices through memory-mapped registers, with hands-on GPIO examples including LED control, button input, and register-level programming on ARM Cortex-M."
categories: ["SoC Design"]
tags: ["SoC", "Firmware", "GPIO", "Peripheral", "Embedded Programming", "ARM Cortex-M", "Register Programming"]
series: ["SoC Design Course"]
series_order: 14
draft: false
---

{{< katex >}}

## Introduction

In the previous posts, we learned about the Cortex-M0+ architecture and how C code becomes assembly. Now it's time to use that knowledge for something tangible: **controlling real hardware**.

Firmware is the software that runs directly on the microcontroller, interfacing with peripheral devices like LEDs, buttons, sensors, and communication interfaces. In this post, we focus on the most fundamental peripheral: **GPIO (General Purpose Input/Output)**.

---

## 1. What Is Firmware?

### 1.1 Definition

**Firmware** is software that:
- Runs directly on hardware (bare-metal, no OS, or with a simple RTOS)
- Controls peripheral devices through register manipulation
- Is stored in non-volatile memory (Flash)
- Typically written in C (sometimes with assembly for critical sections)

### 1.2 Firmware vs. Application Software

| Aspect | Firmware | Application Software |
|--------|----------|---------------------|
| Runs on | Microcontroller (bare-metal) | OS (Linux, Windows) |
| Hardware access | Direct register manipulation | Through OS drivers/APIs |
| Memory | KB of Flash/SRAM | GB of RAM |
| Timing | Deterministic, real-time | Best-effort |
| Language | C, C++ (some assembly) | Python, Java, C++, etc. |
| Debugging | JTAG/SWD, logic analyzer | IDE debugger |

---

## 2. Memory-Mapped I/O: The Key Concept

### 2.1 How Peripherals Are Accessed

In ARM Cortex-M systems, peripherals are controlled by reading and writing to specific **memory addresses**. Each peripheral has a set of **registers** at fixed addresses in the memory map:

```
Address         Register            Purpose
─────────────────────────────────────────────
0x40020000      GPIOA_MODER         Mode configuration
0x40020004      GPIOA_OTYPER        Output type
0x40020008      GPIOA_OSPEEDR       Output speed
0x4002000C      GPIOA_PUPDR         Pull-up/pull-down
0x40020010      GPIOA_IDR           Input Data (read pins)
0x40020014      GPIOA_ODR           Output Data (set pins)
0x40020018      GPIOA_BSRR          Bit Set/Reset (atomic)
0x4002001C      GPIOA_LCKR          Lock configuration
0x40020020      GPIOA_AFRL          Alternate function low
0x40020024      GPIOA_AFRH          Alternate function high
```

### 2.2 Register Access in C

```c
// Direct address access (low-level)
#define GPIOA_MODER   (*(volatile uint32_t *)0x40020000)
#define GPIOA_ODR     (*(volatile uint32_t *)0x40020014)

void set_pin5(void) {
    GPIOA_ODR |= (1 << 5);     // Set bit 5 HIGH
}
```

**Why `volatile`?** The `volatile` keyword tells the compiler:
1. The value can change at any time (hardware may modify it)
2. Every read/write must actually access the memory (no caching in registers)
3. Do not reorder or optimize away these accesses

Without `volatile`, the compiler might:
- Cache a register value and skip re-reading it (missing hardware changes)
- Optimize away a "useless" write (that actually controls hardware)
- Reorder operations (breaking timing-dependent sequences)

### 2.3 Struct-Based Register Access

A cleaner approach using C structs:

```c
typedef struct {
    volatile uint32_t MODER;      // Offset 0x00
    volatile uint32_t OTYPER;     // Offset 0x04
    volatile uint32_t OSPEEDR;    // Offset 0x08
    volatile uint32_t PUPDR;      // Offset 0x0C
    volatile uint32_t IDR;        // Offset 0x10
    volatile uint32_t ODR;        // Offset 0x14
    volatile uint32_t BSRR;       // Offset 0x18
    volatile uint32_t LCKR;       // Offset 0x1C
    volatile uint32_t AFRL;       // Offset 0x20
    volatile uint32_t AFRH;       // Offset 0x24
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *)0x40020000)
#define GPIOB  ((GPIO_TypeDef *)0x40020400)

// Usage:
GPIOA->ODR |= (1 << 5);    // Set PA5 HIGH
```

This is how most vendor HAL (Hardware Abstraction Layer) libraries define peripherals.

---

## 3. GPIO Fundamentals

### 3.1 What Is GPIO?

**GPIO (General Purpose Input/Output)** pins are the most basic way for a microcontroller to interact with the outside world. Each GPIO pin can be individually configured as:

```
                    ┌──────────────────┐
                    │    GPIO Pin      │
                    │                  │
   MCU Internal ────┤  ┌────────────┐ ├──── External Connection
                    │  │  Mode:     │ │     (LED, Button, Sensor)
                    │  │  - Input   │ │
                    │  │  - Output  │ │
                    │  │  - Alt Func│ │
                    │  │  - Analog  │ │
                    │  └────────────┘ │
                    └──────────────────┘
```

### 3.2 GPIO Modes

| Mode | Code | Purpose |
|------|:----:|---------|
| **Input** | 00 | Read external signals (buttons, sensors) |
| **Output** | 01 | Drive external devices (LEDs, relays) |
| **Alternate Function** | 10 | Connect to peripheral (UART TX, SPI CLK) |
| **Analog** | 11 | Connect to ADC/DAC |

### 3.3 GPIO Configuration Registers

**MODER (Mode Register):** 2 bits per pin, 16 pins per port.

```
31 30 29 28 .......................... 3  2  1  0
┌─────┬─────┬─────┬─────┬───────────┬─────┬─────┐
│P15  │P14  │P13  │P12  │   ....    │ P1  │ P0  │
│mode │mode │mode │mode │           │mode │mode │
└─────┴─────┴─────┴─────┴───────────┴─────┴─────┘
  2 bits per pin: 00=Input, 01=Output, 10=AltFunc, 11=Analog
```

**OTYPER (Output Type):** 1 bit per pin.

| Bit Value | Type | Description |
|:---------:|------|-------------|
| 0 | Push-Pull | Drives HIGH and LOW actively |
| 1 | Open-Drain | Drives LOW actively, HIGH is floating (needs pull-up) |

**PUPDR (Pull-Up / Pull-Down):** 2 bits per pin.

| Value | Configuration |
|:-----:|--------------|
| 00 | No pull-up, no pull-down (floating) |
| 01 | Pull-up resistor enabled |
| 10 | Pull-down resistor enabled |
| 11 | Reserved |

---

## 4. GPIO Output: Driving an LED

### 4.1 Hardware Setup

```
MCU Pin (PA5) ──── [Resistor 330Ω] ──── [LED] ──── GND

When PA5 = HIGH (3.3V): Current flows → LED ON
When PA5 = LOW  (0V):   No current  → LED OFF
```

### 4.2 Configuration Steps

```c
#include <stdint.h>

// Register definitions
#define RCC_AHB1ENR   (*(volatile uint32_t *)0x40023830)
#define GPIOA_MODER   (*(volatile uint32_t *)0x40020000)
#define GPIOA_ODR     (*(volatile uint32_t *)0x40020014)

void led_init(void) {
    // Step 1: Enable GPIOA clock
    // Without this, the GPIO peripheral is powered off!
    RCC_AHB1ENR |= (1 << 0);   // Bit 0 = GPIOAEN

    // Step 2: Configure PA5 as Output
    // MODER bits [11:10] = 01 (Output mode)
    GPIOA_MODER &= ~(3 << 10);  // Clear bits 11:10
    GPIOA_MODER |=  (1 << 10);  // Set bit 10 (01 = Output)
}

void led_on(void) {
    GPIOA_ODR |= (1 << 5);      // Set PA5 HIGH
}

void led_off(void) {
    GPIOA_ODR &= ~(1 << 5);     // Set PA5 LOW
}

void led_toggle(void) {
    GPIOA_ODR ^= (1 << 5);      // Toggle PA5
}
```

### 4.3 The Clock Enable Step — Why?

```c
RCC_AHB1ENR |= (1 << 0);    // Enable GPIOA clock
```

To save power, peripherals are **clock-gated** by default — they receive no clock signal and consume near-zero power. Before using any peripheral, you must enable its clock through the **RCC (Reset and Clock Control)** registers.

```
Before clock enable:          After clock enable:
┌──────────┐   CLK=OFF       ┌──────────┐   CLK=ON
│  GPIOA   │ ← ╳ ───         │  GPIOA   │ ← ─── Clock
│ (asleep) │                  │ (active) │
└──────────┘                  └──────────┘
```

### 4.4 Atomic Bit Operations with BSRR

The read-modify-write pattern (`ODR |= ...`) is **not atomic** — an interrupt between the read and write could cause data corruption. The **BSRR (Bit Set/Reset Register)** provides atomic bit manipulation:

```c
#define GPIOA_BSRR   (*(volatile uint32_t *)0x40020018)

// Set PA5 (atomic, no read-modify-write needed)
GPIOA_BSRR = (1 << 5);       // Bits 0-15: SET corresponding pin

// Reset PA5 (atomic)
GPIOA_BSRR = (1 << (5 + 16)); // Bits 16-31: RESET corresponding pin
```

```
BSRR Register:
Bits 31:16 = Reset bits (write 1 to clear corresponding ODR bit)
Bits 15:0  = Set bits   (write 1 to set corresponding ODR bit)

Writing 0 to any bit has no effect.
```

---

## 5. GPIO Input: Reading a Button

### 5.1 Hardware Setup

```
VDD (3.3V)
  │
  [Pull-up R 10kΩ]
  │
  ├──── MCU Pin (PA0)
  │
  [Button]
  │
  GND

Button released: PA0 reads HIGH (pulled up to VDD)
Button pressed:  PA0 reads LOW  (connected to GND through button)
```

### 5.2 Configuration and Reading

```c
void button_init(void) {
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);

    // Configure PA0 as Input (MODER bits [1:0] = 00)
    GPIOA_MODER &= ~(3 << 0);   // Clear bits 1:0 (Input mode)

    // Enable internal pull-up (PUPDR bits [1:0] = 01)
    GPIOA_PUPDR &= ~(3 << 0);   // Clear
    GPIOA_PUPDR |=  (1 << 0);   // Set 01 = Pull-up
}

int button_is_pressed(void) {
    // Read IDR bit 0; button is active LOW
    return !(GPIOA_IDR & (1 << 0));  // Returns 1 when pressed
}
```

### 5.3 Debouncing

Mechanical buttons **bounce** — when pressed, the contact rapidly makes and breaks for a few milliseconds:

```
Ideal:    ────┐          ┌─────
              │          │
              └──────────┘

Reality:  ────┐ ┌┐ ┌┐   ┌─────
              │ ││ ││   │
              └─┘└─┘└───┘
              ←─ bounce ─→
              (~5-20 ms)
```

**Software debouncing:**

```c
#define DEBOUNCE_MS 20

int button_debounced(void) {
    if (button_is_pressed()) {
        delay_ms(DEBOUNCE_MS);          // Wait for bounce to settle
        if (button_is_pressed()) {      // Check again
            return 1;                   // Confirmed press
        }
    }
    return 0;
}
```

---

## 6. Complete Example: Button-Controlled LED

```c
#include <stdint.h>

// Register definitions
#define RCC_AHB1ENR    (*(volatile uint32_t *)0x40023830)
#define GPIOA_MODER    (*(volatile uint32_t *)0x40020000)
#define GPIOA_PUPDR    (*(volatile uint32_t *)0x4002000C)
#define GPIOA_IDR      (*(volatile uint32_t *)0x40020010)
#define GPIOA_BSRR     (*(volatile uint32_t *)0x40020018)

void system_init(void) {
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);

    // PA5 = Output (LED)
    GPIOA_MODER &= ~(3 << 10);
    GPIOA_MODER |=  (1 << 10);

    // PA0 = Input (Button) with pull-up
    GPIOA_MODER &= ~(3 << 0);
    GPIOA_PUPDR &= ~(3 << 0);
    GPIOA_PUPDR |=  (1 << 0);
}

void delay_ms(uint32_t ms) {
    // Simple busy-wait delay (not accurate, CPU-dependent)
    for (volatile uint32_t i = 0; i < ms * 4000; i++);
}

int main(void) {
    system_init();

    while (1) {
        if (!(GPIOA_IDR & (1 << 0))) {   // Button pressed (active LOW)
            delay_ms(20);                  // Debounce
            if (!(GPIOA_IDR & (1 << 0))) {
                GPIOA_BSRR = (1 << 5);    // LED ON
            }
        } else {
            GPIOA_BSRR = (1 << (5 + 16)); // LED OFF
        }
    }

    return 0;  // Never reached
}
```

---

## 7. Bit Manipulation Patterns

Embedded programming relies heavily on bit manipulation. Here are the essential patterns:

### 7.1 Set a Bit

```c
register |= (1 << bit_position);

// Example: Set bit 5
GPIOA_ODR |= (1 << 5);    // ODR: xxxx xxxx xx1x xxxx
```

### 7.2 Clear a Bit

```c
register &= ~(1 << bit_position);

// Example: Clear bit 5
GPIOA_ODR &= ~(1 << 5);   // ODR: xxxx xxxx xx0x xxxx
```

### 7.3 Toggle a Bit

```c
register ^= (1 << bit_position);

// Example: Toggle bit 5
GPIOA_ODR ^= (1 << 5);
```

### 7.4 Check a Bit

```c
if (register & (1 << bit_position)) { /* bit is set */ }

// Example: Check if bit 0 is set
if (GPIOA_IDR & (1 << 0)) { /* PA0 is HIGH */ }
```

### 7.5 Set a Multi-Bit Field

```c
// Clear the field first, then set the new value
register &= ~(mask << position);    // Clear
register |=  (value << position);   // Set

// Example: Set MODER bits [11:10] to 01 (Output)
GPIOA_MODER &= ~(0x3 << 10);       // Clear 2 bits
GPIOA_MODER |=  (0x1 << 10);       // Set to 01
```

### 7.6 Macro Helpers

```c
#define BIT_SET(reg, bit)      ((reg) |=  (1U << (bit)))
#define BIT_CLEAR(reg, bit)    ((reg) &= ~(1U << (bit)))
#define BIT_TOGGLE(reg, bit)   ((reg) ^=  (1U << (bit)))
#define BIT_READ(reg, bit)     (((reg) >> (bit)) & 1U)
```

---

## 8. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Firmware** | Software that directly controls hardware through register access |
| **Memory-mapped I/O** | Peripherals accessed via memory addresses — same as regular memory |
| **volatile** | Essential keyword — prevents compiler from optimizing away hardware accesses |
| **Clock enable** | Must enable peripheral clock before use (power saving feature) |
| **GPIO modes** | Input, Output, Alternate Function, Analog (2 bits per pin in MODER) |
| **BSRR** | Atomic bit set/reset — safer than read-modify-write on ODR |
| **Debouncing** | Mechanical buttons bounce — add software delay for reliable reads |
| **Bit manipulation** | Set, clear, toggle, check — the core patterns of embedded C |

In the **next post ([SoC-15])**, we will learn about **interrupts** — the mechanism that allows the CPU to respond to external events efficiently without busy-waiting.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
