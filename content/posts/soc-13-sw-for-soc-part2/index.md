---
title: "[SoC-13] Software for SoC Part 2: From C Code to Cortex-M0+ Assembly"
date: 2026-02-25
description: "Understanding how a C compiler translates high-level code into ARM Cortex-M0+ Thumb assembly — tracing through variables, loops, conditionals, functions, and memory access patterns."
categories: ["SoC Design"]
tags: ["SoC", "ARM", "Cortex-M0+", "Assembly", "C to Assembly", "Thumb", "Compiler"]
series: ["SoC Design Course"]
series_order: 13
draft: false
---

{{< katex >}}

## Introduction

In [SoC-12], we studied the Cortex-M0+ architecture and its Thumb instruction set. Now let's see the complete picture: how does **C code** become the **machine instructions** that the Cortex-M0+ actually executes?

This is a crucial skill for embedded engineers — understanding compiler output helps you write more efficient code, debug hardware-software interactions, and optimize critical code paths.

---

## 1. The Compilation Pipeline

```
C Source Code (.c)
       │
       ▼
┌──────────────┐
│  Preprocessor│  #include, #define, #ifdef
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   Compiler   │  C → Assembly (.s)
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Assembler   │  Assembly → Object (.o)
└──────┬───────┘
       │
       ▼
┌──────────────┐
│    Linker    │  Objects → Executable (.elf)
└──────┬───────┘
       │
       ▼
Binary (.bin / .hex) → Flashed to MCU
```

For ARM Cortex-M, the standard toolchain is **arm-none-eabi-gcc** (GCC cross-compiler for bare-metal ARM).

```bash
# Compile with optimization, see assembly output
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O1 -S main.c -o main.s
```

---

## 2. Cortex-M0+ Calling Convention (AAPCS)

Before we look at C-to-assembly translations, let's understand the calling convention:

| Register | Role | Caller/Callee Saved |
|----------|------|:-------------------:|
| R0–R3 | Arguments & return value | Caller-saved |
| R4–R7 | General purpose (low regs) | Callee-saved |
| R8–R11 | General purpose (high regs) | Callee-saved |
| R12 (IP) | Intra-procedure scratch | Caller-saved |
| R13 (SP) | Stack pointer | Callee-saved |
| R14 (LR) | Link register (return addr) | — |
| R15 (PC) | Program counter | — |

**Key rules:**
- Arguments passed in R0–R3; additional args go on the stack
- Return value in R0 (or R0–R1 for 64-bit)
- Callee must preserve R4–R11 and SP
- Stack must be 8-byte aligned at function entry

---

## 3. Simple Expressions

### 3.1 Variable Assignment and Arithmetic

```c
int compute(int a, int b) {
    int c = a + b;
    int d = c * 3;
    return d - a;
}
```

```asm
compute:
    ADDS  R2, R0, R1        @ c = a + b  (R0=a, R1=b, R2=c)
    MOVS  R3, #3            @ R3 = 3
    MULS  R2, R3, R2        @ d = c * 3  (R2=d)
    SUBS  R0, R2, R0        @ return d - a  (result in R0)
    BX    LR                @ return to caller
```

**Observations:**
- Arguments arrive in R0, R1 (per calling convention)
- Result goes in R0
- No stack usage needed (all fits in registers)
- `MOVS` loads small immediate into register
- `BX LR` returns to the caller (Branch and eXchange to address in LR)

### 3.2 Bitwise Operations

```c
uint32_t mask_and_shift(uint32_t value) {
    uint32_t masked = value & 0xFF;    // Extract low byte
    uint32_t shifted = masked << 4;     // Shift left by 4
    return shifted | 0x0F;              // Set low nibble
}
```

```asm
mask_and_shift:
    UXTB  R0, R0            @ R0 = R0 & 0xFF (unsigned extend byte)
    LSLS  R0, R0, #4        @ R0 = R0 << 4
    MOVS  R1, #0x0F
    ORRS  R0, R0, R1        @ R0 = R0 | 0x0F
    BX    LR
```

Note: `UXTB` (Unsigned eXTend Byte) is a Thumb-2 instruction available on Cortex-M0+ that zero-extends the low byte, effectively doing `& 0xFF`.

---

## 4. Conditional Statements

### 4.1 Simple If-Else

```c
int abs_val(int x) {
    if (x < 0) {
        return -x;
    } else {
        return x;
    }
}
```

```asm
abs_val:
    CMP   R0, #0            @ Compare x with 0 (sets flags)
    BGE   positive           @ if (x >= 0) goto positive
    RSBS  R0, R0, #0        @ R0 = 0 - R0 (negate)
positive:
    BX    LR                @ return R0
```

**Key instructions:**
- `CMP` sets the condition flags (N, Z, C, V) without storing the result
- `BGE` (Branch if Greater or Equal) checks the N and V flags
- `RSBS` (Reverse Subtract) computes `0 - R0`, which negates the value

### 4.2 Multi-Condition

```c
int classify(int x) {
    if (x > 0) return 1;
    else if (x < 0) return -1;
    else return 0;
}
```

```asm
classify:
    CMP   R0, #0
    BGT   positive           @ if (x > 0)
    BLT   negative           @ if (x < 0)
    MOVS  R0, #0             @ x == 0: return 0
    BX    LR
positive:
    MOVS  R0, #1             @ return 1
    BX    LR
negative:
    MOVS  R0, #0
    SUBS  R0, R0, #1         @ R0 = -1 (can't MOVS #-1 directly)
    BX    LR
```

**Note:** Thumb instructions can only load small positive immediates with `MOVS`. For -1, the compiler uses `MOVS R0, #0; SUBS R0, R0, #1` or the more efficient `MVNS R0, R0` after zeroing.

---

## 5. Loops

### 5.1 For Loop (Array Sum)

```c
int sum_array(int *arr, int n) {
    int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum;
}
```

```asm
sum_array:
    @ R0 = arr, R1 = n
    MOVS  R2, #0             @ sum = 0
    MOVS  R3, #0             @ i = 0
loop:
    CMP   R3, R1             @ compare i with n
    BGE   done               @ if (i >= n) exit loop
    LSLS  R4, R3, #2         @ R4 = i * 4 (byte offset)
    LDR   R4, [R0, R4]       @ R4 = arr[i]
    ADDS  R2, R2, R4         @ sum += arr[i]
    ADDS  R3, R3, #1         @ i++
    B     loop               @ repeat
done:
    MOVS  R0, R2             @ return sum (move to R0)
    BX    LR
```

**Wait — there's a problem!** This function uses R4, which is a callee-saved register. The function must save and restore it:

```asm
sum_array:
    PUSH  {R4, LR}           @ Save R4 and return address
    MOVS  R2, #0             @ sum = 0
    MOVS  R3, #0             @ i = 0
loop:
    CMP   R3, R1
    BGE   done
    LSLS  R4, R3, #2
    LDR   R4, [R0, R4]
    ADDS  R2, R2, R4
    ADDS  R3, R3, #1
    B     loop
done:
    MOVS  R0, R2
    POP   {R4, PC}           @ Restore R4; pop LR into PC = return
```

**Clever trick:** `POP {R4, PC}` restores R4 AND loads the saved LR directly into PC, which is equivalent to `POP {R4}; BX LR` but saves one instruction.

### 5.2 While Loop with Pointer

```c
int strlen_custom(const char *s) {
    int len = 0;
    while (*s != '\0') {
        s++;
        len++;
    }
    return len;
}
```

```asm
strlen_custom:
    MOVS  R1, #0             @ len = 0
loop:
    LDRB  R2, [R0]           @ R2 = *s (load byte)
    CMP   R2, #0             @ compare with '\0'
    BEQ   done               @ if (*s == 0) exit
    ADDS  R0, R0, #1         @ s++
    ADDS  R1, R1, #1         @ len++
    B     loop
done:
    MOVS  R0, R1             @ return len
    BX    LR
```

---

## 6. Function Calls

### 6.1 Leaf Function (No Calls to Other Functions)

```c
int square(int x) {
    return x * x;
}
```

```asm
square:
    MULS  R0, R0, R0         @ R0 = x * x
    BX    LR                 @ return
```

No stack frame needed — leaf functions are very efficient.

### 6.2 Non-Leaf Function

```c
int add(int a, int b) { return a + b; }

int compute(int x, int y) {
    int temp = add(x, y);
    return temp + 1;
}
```

```asm
add:
    ADDS  R0, R0, R1
    BX    LR

compute:
    PUSH  {LR}               @ Save return address (we're calling add)
    BL    add                 @ Call add(x, y); LR = return addr
    ADDS  R0, R0, #1         @ temp + 1
    POP   {PC}               @ Return (pop saved LR into PC)
```

`BL` (Branch with Link) saves the return address in LR before jumping. Since `compute` calls `add`, it must save its own LR first.

### 6.3 Function with Local Variables on Stack

```c
int complex_calc(int a, int b, int c, int d) {
    int x = a + b;
    int y = c + d;
    int z = x * y;
    return z;
}
```

```asm
complex_calc:
    @ R0=a, R1=b, R2=c, R3=d
    ADDS  R0, R0, R1         @ x = a + b (R0)
    ADDS  R1, R2, R3         @ y = c + d (R1)
    MULS  R0, R1, R0         @ z = x * y (R0)
    BX    LR                 @ return z
```

The compiler is smart — it reuses registers and avoids stack allocation when possible.

### 6.4 More Than 4 Arguments

```c
int sum5(int a, int b, int c, int d, int e) {
    return a + b + c + d + e;
}
```

```asm
sum5:
    @ R0=a, R1=b, R2=c, R3=d, e is on stack
    ADDS  R0, R0, R1         @ a + b
    ADDS  R0, R0, R2         @ + c
    ADDS  R0, R0, R3         @ + d
    LDR   R1, [SP, #0]       @ Load e from stack
    ADDS  R0, R0, R1         @ + e
    BX    LR
```

The 5th argument (`e`) is passed on the stack because only R0–R3 are used for argument passing.

---

## 7. Stack Frame Layout

For a function that saves registers and has local variables:

```c
void example(int a) {
    int local1 = a + 1;
    int local2 = a * 2;
    other_func(local1, local2);
}
```

```
Stack (before function entry):
            ┌────────────────┐ ← SP (old)
            │  (caller's     │
            │   stack frame)  │
            └────────────────┘

Stack (after prologue):
            ┌────────────────┐
            │  saved LR      │ SP + 12
            ├────────────────┤
            │  saved R4      │ SP + 8
            ├────────────────┤
            │  local2        │ SP + 4
            ├────────────────┤
            │  local1        │ SP + 0
            └────────────────┘ ← SP (new)
```

```asm
example:
    PUSH  {R4, LR}           @ Save callee-saved regs
    SUB   SP, SP, #8         @ Allocate space for 2 local vars
    ADDS  R4, R0, #1         @ local1 = a + 1
    STR   R4, [SP, #0]       @ Store local1
    LSLS  R0, R0, #1         @ local2 = a * 2
    STR   R0, [SP, #4]       @ Store local2
    MOVS  R0, R4             @ arg1 = local1
    LDR   R1, [SP, #4]       @ arg2 = local2
    BL    other_func
    ADD   SP, SP, #8         @ Deallocate locals
    POP   {R4, PC}           @ Restore and return
```

---

## 8. Memory Access Patterns

### 8.1 Accessing Global Variables

```c
volatile int counter;    // At address 0x20000000

void increment(void) {
    counter++;
}
```

```asm
increment:
    LDR   R0, =counter       @ R0 = address of counter (literal pool)
    LDR   R1, [R0]           @ R1 = *counter (read current value)
    ADDS  R1, R1, #1         @ R1 = counter + 1
    STR   R1, [R0]           @ *counter = R1 (write back)
    BX    LR

    .align 2
    .word counter             @ Literal pool: address of counter
```

**Literal pool:** Since Thumb instructions can't encode 32-bit addresses directly, the assembler stores the address in a nearby "literal pool" in memory and loads it with `LDR Rn, =label`.

### 8.2 Accessing Peripheral Registers

```c
#define GPIOA_BASE  0x40020000
#define GPIOA_ODR   (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

void set_pin5_high(void) {
    GPIOA_ODR |= (1 << 5);
}
```

```asm
set_pin5_high:
    LDR   R0, =0x40020014    @ R0 = address of GPIOA_ODR
    LDR   R1, [R0]           @ R1 = current ODR value
    MOVS  R2, #32            @ R2 = (1 << 5) = 32
    ORRS  R1, R1, R2         @ R1 |= (1 << 5)
    STR   R1, [R0]           @ Write back to ODR
    BX    LR
```

This is the **read-modify-write** pattern that's fundamental to peripheral control.

---

## 9. Compiler Optimization Levels

| Level | Flag | Effect |
|:-----:|:----:|--------|
| 0 | `-O0` | No optimization — direct translation, easy to debug |
| 1 | `-O1` | Basic optimization — register allocation, dead code removal |
| 2 | `-O2` | Aggressive — inlining, loop optimization, scheduling |
| s | `-Os` | Optimize for size — critical for Flash-constrained MCUs |
| 3 | `-O3` | Maximum — loop unrolling, vectorization (less useful on M0+) |

For embedded Cortex-M0+ development, **`-Os`** is the most common choice — it produces compact code that fits in limited Flash while still being reasonably fast.

---

## 10. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Compilation pipeline** | C → Preprocessor → Compiler → Assembler → Linker → Binary |
| **Calling convention** | R0–R3 for args/return; R4–R11 callee-saved; stack 8-byte aligned |
| **Thumb instructions** | Mostly 16-bit; limited register access (R0–R7 for most ops) |
| **Stack management** | PUSH/POP for save/restore; SP adjusted for local variables |
| **Memory access** | Literal pool for 32-bit addresses; read-modify-write for peripherals |
| **PUSH/POP trick** | `POP {Rn, PC}` returns by popping LR directly into PC |
| **Optimization** | `-Os` is the standard for embedded — balance of size and speed |

In the **next post ([SoC-14])**, we will learn how to control **peripheral devices** through firmware — starting with GPIO (General Purpose Input/Output).

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
