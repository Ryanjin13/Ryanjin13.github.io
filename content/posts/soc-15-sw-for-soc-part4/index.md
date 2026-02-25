---
title: "[SoC-15] Software for SoC Part 4: Interrupts — Responding to the Real World"
date: 2026-02-25
description: "Understanding interrupt concepts, the hardware and software mechanisms for handling them, and how to write Interrupt Service Routines (ISRs) on ARM Cortex-M — covering NVIC configuration, priority, nesting, and best practices."
categories: ["SoC Design"]
tags: ["SoC", "Interrupt", "ISR", "NVIC", "ARM Cortex-M", "Embedded Systems", "Exception Handling"]
series: ["SoC Design Course"]
series_order: 15
draft: false
---

{{< katex >}}

## Introduction

In [SoC-14], we used **polling** to check whether a button was pressed — the CPU continuously reads the GPIO pin in a loop. This works, but it wastes CPU cycles and can't respond quickly to time-critical events.

**Interrupts** solve this problem elegantly: the hardware notifies the CPU when an event occurs, and the CPU immediately pauses its current work to handle it. This is how real embedded systems achieve responsive, efficient, real-time behavior.

---

## 1. Polling vs. Interrupts

### 1.1 Polling

```c
// Polling: CPU constantly checks
while (1) {
    if (button_pressed()) {
        handle_button();
    }
    // CPU is stuck here, can't do anything else efficiently
    do_other_work();    // This gets delayed by polling overhead
}
```

**Problems:**
- Wastes CPU cycles checking for events that rarely happen
- Response time depends on how fast the polling loop runs
- Difficult to handle multiple event sources with different priorities

### 1.2 Interrupts

```c
// Interrupt: CPU is notified automatically
void EXTI0_IRQHandler(void) {    // Called automatically when button pressed
    handle_button();
    clear_interrupt_flag();
}

int main(void) {
    setup_interrupt();
    while (1) {
        do_useful_work();     // CPU is free to do other things
        enter_sleep_mode();   // Can even sleep to save power!
    }
}
```

**Advantages:**

| Aspect | Polling | Interrupts |
|--------|---------|------------|
| CPU utilization | Wasted on checking | Free for useful work |
| Response time | Variable (depends on loop) | Deterministic (~15 cycles on M0+) |
| Power consumption | High (CPU always running) | Low (CPU can sleep) |
| Multiple events | Complex scheduling | Natural priority handling |
| Code structure | Monolithic loop | Event-driven, modular |

---

## 2. How Interrupts Work: The Hardware Side

### 2.1 The Interrupt Flow

```
 ┌─────────┐         ┌──────┐         ┌───────────┐
 │Peripheral│──IRQ──►│ NVIC │──IRQ──►│   CPU     │
 │ (Timer,  │        │      │        │ (Cortex-  │
 │  UART,   │        │      │        │   M0+)    │
 │  GPIO)   │        │      │        │           │
 └─────────┘         └──────┘         └───────────┘
```

1. **Peripheral** detects an event (timer overflow, data received, pin change)
2. Peripheral sets its **interrupt flag** and asserts the IRQ line
3. **NVIC** receives the IRQ, checks if it's enabled and if priority allows it
4. NVIC signals the **CPU** to take the interrupt
5. CPU performs the **exception entry sequence**

### 2.2 Exception Entry Sequence (Hardware Steps)

When the CPU accepts an interrupt, the hardware automatically:

```
Step 1: PUSH registers to stack (8 registers)
┌──────────────────────┐
│  xPSR                │ ← SP + 28
│  PC (return address) │ ← SP + 24
│  LR                  │ ← SP + 20
│  R12                 │ ← SP + 16
│  R3                  │ ← SP + 12
│  R2                  │ ← SP + 8
│  R1                  │ ← SP + 4
│  R0                  │ ← SP + 0
└──────────────────────┘ ← New SP

Step 2: Load PC from Vector Table
        PC = VectorTable[IRQ_number + 16]

Step 3: Load LR with EXC_RETURN value
        LR = 0xFFFFFFF1 (return to Handler, MSP)
          or 0xFFFFFFF9 (return to Thread, MSP)
          or 0xFFFFFFFD (return to Thread, PSP)

Step 4: Enter Handler Mode, switch to MSP

Step 5: Begin executing ISR
```

**Total entry latency: ~15 cycles** (on Cortex-M0+)

### 2.3 Exception Return

When the ISR completes (executes `BX LR` with the special EXC_RETURN value):

```
Step 1: POP 8 registers from stack (R0-R3, R12, LR, PC, xPSR)
Step 2: Restore processor mode (Thread/Handler)
Step 3: Continue executing from restored PC
```

The beauty of this design: **the ISR looks like a normal C function** — the hardware handles all the save/restore automatically.

---

## 3. NVIC Configuration

### 3.1 Enabling an Interrupt

```c
// NVIC Registers (System Control Space: 0xE000E000)
#define NVIC_ISER    (*(volatile uint32_t *)0xE000E100)  // Interrupt Set Enable
#define NVIC_ICER    (*(volatile uint32_t *)0xE000E180)  // Interrupt Clear Enable
#define NVIC_ISPR    (*(volatile uint32_t *)0xE000E200)  // Interrupt Set Pending
#define NVIC_ICPR    (*(volatile uint32_t *)0xE000E280)  // Interrupt Clear Pending
#define NVIC_IPR     ((volatile uint32_t *)0xE000E400)   // Interrupt Priority (array)

void enable_irq(int irq_number) {
    NVIC_ISER = (1 << irq_number);    // Enable specific interrupt
}

void disable_irq(int irq_number) {
    NVIC_ICER = (1 << irq_number);    // Disable specific interrupt
}
```

### 3.2 Setting Priority

On Cortex-M0+, each interrupt has a **2-bit priority** (4 levels):

| Priority Value | Level | Urgency |
|:--------------:|:-----:|---------|
| 0x00 | 0 | Highest (most urgent) |
| 0x40 | 1 | High |
| 0x80 | 2 | Medium |
| 0xC0 | 3 | Lowest |

```c
void set_irq_priority(int irq_number, uint8_t priority) {
    // Priority registers are byte-accessible
    // Only top 2 bits are used on M0+ (bits 7:6)
    volatile uint8_t *pri_reg = (volatile uint8_t *)(0xE000E400 + irq_number);
    *pri_reg = (priority << 6);   // Shift to top 2 bits
}
```

### 3.3 Complete Interrupt Setup Example

Setting up EXTI0 (External Interrupt on PA0 — button press):

```c
// 1. Configure GPIO PA0 as input (already covered in SoC-14)

// 2. Configure EXTI (External Interrupt)
#define EXTI_IMR    (*(volatile uint32_t *)0x40013C00)  // Interrupt Mask
#define EXTI_FTSR   (*(volatile uint32_t *)0x40013C0C)  // Falling Trigger
#define EXTI_PR     (*(volatile uint32_t *)0x40013C14)  // Pending Register

#define SYSCFG_EXTICR1  (*(volatile uint32_t *)0x40013808)

void button_interrupt_init(void) {
    // Enable SYSCFG clock
    RCC_APB2ENR |= (1 << 14);

    // Map EXTI0 to PA0
    SYSCFG_EXTICR1 &= ~(0xF << 0);   // EXTI0 = PA0

    // Configure EXTI0 for falling edge (button press = HIGH→LOW)
    EXTI_FTSR |= (1 << 0);

    // Unmask EXTI0
    EXTI_IMR |= (1 << 0);

    // Set priority (medium)
    set_irq_priority(6, 2);   // EXTI0 = IRQ6 on many STM32 chips

    // Enable in NVIC
    NVIC_ISER = (1 << 6);

    // Enable global interrupts
    __enable_irq();
}
```

---

## 4. Writing Interrupt Service Routines (ISRs)

### 4.1 ISR Structure

```c
void EXTI0_IRQHandler(void) {
    // 1. Check which source triggered the interrupt (if shared)
    if (EXTI_PR & (1 << 0)) {

        // 2. Handle the event
        led_toggle();

        // 3. Clear the interrupt flag (CRITICAL!)
        EXTI_PR = (1 << 0);    // Write 1 to clear
    }
}
```

### 4.2 ISR Best Practices

| Rule | Reason |
|------|--------|
| **Keep ISRs short** | Long ISRs block other interrupts and main code |
| **Always clear the flag** | If not cleared, the ISR will be called again immediately |
| **Use volatile for shared variables** | Variables shared between ISR and main must be volatile |
| **Minimize function calls** | Deep call chains increase stack usage |
| **No blocking operations** | Never use delay loops, printf, or malloc in ISRs |
| **Use flags for deferred processing** | Set a flag in ISR, process in main loop |

### 4.3 The Flag Pattern

```c
volatile int button_event = 0;   // Shared between ISR and main

void EXTI0_IRQHandler(void) {
    button_event = 1;            // Just set a flag (fast!)
    EXTI_PR = (1 << 0);         // Clear interrupt
}

int main(void) {
    button_interrupt_init();

    while (1) {
        if (button_event) {
            button_event = 0;    // Clear flag
            handle_button();     // Do the actual work (can be slow)
        }
        // Other tasks...
    }
}
```

---

## 5. Interrupt Priority and Nesting

### 5.1 Priority-Based Preemption

A **higher-priority** interrupt can preempt (interrupt) a **lower-priority** ISR:

```
Main code running...
                    ┌─── Low-priority IRQ fires
                    │
                    ▼
           ┌──── Low-priority ISR ────┐
           │                           │
           │    ┌── High-priority IRQ  │
           │    │                      │
           │    ▼                      │
           │  ┌─ High-pri ISR ─┐      │
           │  │  (preempts!)   │      │
           │  └────────────────┘      │
           │    ↓ (resume low-pri)    │
           └──────────────────────────┘
                    ↓ (resume main)
Main code continues...
```

### 5.2 Tail-Chaining

When one interrupt completes and another is pending, the Cortex-M avoids the full exit+entry sequence:

```
Normal (without tail-chaining):
[ISR-A finish] → POP 8 regs → PUSH 8 regs → [ISR-B start]
                  ~12 cycles    ~12 cycles

Tail-chaining:
[ISR-A finish] → [ISR-B start]  (skip POP+PUSH)
                  ~6 cycles
```

This optimization saves ~18 cycles between back-to-back interrupts.

### 5.3 Late-Arriving Optimization

If a higher-priority interrupt arrives during the stacking phase of a lower-priority interrupt, the CPU switches to the higher-priority ISR without re-stacking:

```
[Low-pri stacking in progress...]
  ↑ High-priority IRQ arrives!
[Continue stacking] → [Execute HIGH-pri ISR first]
                     → [Then tail-chain to LOW-pri ISR]
```

---

## 6. Critical Sections

Sometimes you need to temporarily prevent interrupts from firing (e.g., when updating shared data structures):

### 6.1 Disabling All Interrupts

```c
void critical_section_example(void) {
    __disable_irq();           // PRIMASK = 1 (mask all interrupts)

    // Critical code — no interrupts can fire here
    shared_counter++;
    shared_buffer[index] = value;
    index++;

    __enable_irq();            // PRIMASK = 0 (unmask)
}
```

### 6.2 Save and Restore Pattern

A better approach that handles nested critical sections:

```c
void safe_critical_section(void) {
    uint32_t primask = __get_PRIMASK();  // Save current state
    __disable_irq();

    // Critical code...

    __set_PRIMASK(primask);              // Restore (not just enable!)
}
```

This is important because if interrupts were already disabled when you entered the critical section, you don't want to accidentally re-enable them on exit.

### 6.3 When to Use Critical Sections

| Situation | Need Critical Section? |
|-----------|:---------------------:|
| Reading/writing a single `volatile` variable | No (atomic on 32-bit ARM) |
| Incrementing a shared counter | Yes (read-modify-write is not atomic) |
| Updating a multi-field struct shared with ISR | Yes |
| Reading a multi-byte value shared with ISR | Yes (could get half-updated) |
| Configuring peripheral registers (init code) | Usually no (no ISR running yet) |

---

## 7. Common Interrupt Sources

| Source | Typical Use | Priority |
|--------|------------|:--------:|
| **SysTick** | RTOS tick, periodic tasks | Medium |
| **EXTI** | Button press, external events | Varies |
| **UART RX** | Serial data received | High |
| **Timer** | PWM, timing, periodic events | High |
| **ADC** | Conversion complete | Medium |
| **DMA** | Transfer complete | Low-Medium |
| **I2C/SPI** | Communication events | Medium |

---

## 8. Startup Code and Vector Table

### 8.1 Vector Table in C

The vector table is typically defined in the startup file:

```c
// Startup file (startup_stm32.c)
extern uint32_t _estack;       // Defined by linker script

void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
// ... more handlers

// Default handler for unused interrupts
void Default_Handler(void) {
    while (1);   // Hang (or reset) if unexpected interrupt
}

// Vector table — placed at address 0x00000000
__attribute__((section(".isr_vector")))
const uint32_t vector_table[] = {
    (uint32_t)&_estack,          // 0x00: Initial Stack Pointer
    (uint32_t)Reset_Handler,     // 0x04: Reset
    (uint32_t)NMI_Handler,       // 0x08: NMI
    (uint32_t)HardFault_Handler, // 0x0C: Hard Fault
    0, 0, 0, 0, 0, 0, 0,        // 0x10-0x28: Reserved
    (uint32_t)SVC_Handler,       // 0x2C: SVCall
    0, 0,                        // 0x30-0x34: Reserved
    (uint32_t)PendSV_Handler,    // 0x38: PendSV
    (uint32_t)SysTick_Handler,   // 0x3C: SysTick
    // External interrupts (IRQ0, IRQ1, ...)
    (uint32_t)EXTI0_IRQHandler,  // 0x40: IRQ0
    // ...
};
```

### 8.2 Weak Symbols

In practice, handlers are declared as **weak** symbols:

```c
__attribute__((weak)) void EXTI0_IRQHandler(void) {
    Default_Handler();
}
```

If the user doesn't define `EXTI0_IRQHandler`, it defaults to `Default_Handler`. If the user defines it, their version overrides the weak one. This is a clean way to make all handlers optional.

---

## 9. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Polling vs. Interrupts** | Interrupts free the CPU and provide deterministic response time |
| **Exception entry** | Hardware auto-saves 8 registers, loads ISR address from vector table |
| **NVIC** | Manages enable/disable, priority, pending status for all interrupts |
| **ISR best practices** | Keep short, always clear flag, use volatile, use flag pattern |
| **Priority nesting** | Higher-priority ISRs can preempt lower-priority ones |
| **Tail-chaining** | Cortex-M optimization reduces latency between consecutive interrupts |
| **Critical sections** | Temporarily disable interrupts to protect shared data |
| **Vector table** | Array of function pointers at address 0x0; defines handler for each exception |

In the **final post ([SoC-16])**, we will study **Timer** and **DMA** — two essential peripherals that enable precise timing and efficient data transfer without CPU intervention.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
