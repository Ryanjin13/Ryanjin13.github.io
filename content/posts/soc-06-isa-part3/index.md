---
title: "[SoC-06] Instruction Set Architecture Part 3: RISC-V in Action — From C to Machine Code"
date: 2026-02-25
description: "A hands-on exploration of the RISC-V instruction set, showing exactly how C language constructs — variables, loops, conditionals, arrays, and functions — are translated into assembly and machine code."
categories: ["SoC Design"]
tags: ["SoC", "RISC-V", "Assembly", "C to Assembly", "Compiler", "ISA"]
series: ["SoC Design Course"]
series_order: 6
draft: false
---

{{< katex >}}

## Introduction

In [SoC-04] and [SoC-05], we studied ISA concepts, addressing modes, and the RISC-V philosophy. Now it's time to see RISC-V in action — we will take real C code and trace exactly how it becomes assembly instructions and ultimately machine code.

This is where theory meets practice. By the end of this post, you will be able to read RISC-V assembly, understand compiler output, and reason about how your C code executes on hardware.

---

## 1. RISC-V Instruction Reference

Let's first consolidate the key RV32I instructions we will use:

### 1.1 R-Type Instructions (Register-Register)

```
31       25 24    20 19    15 14  12 11     7 6      0
┌──────────┬────────┬────────┬──────┬────────┬────────┐
│  funct7  │   rs2  │   rs1  │funct3│   rd   │ opcode │
└──────────┴────────┴────────┴──────┴────────┴────────┘
```

| Instruction | funct7 | funct3 | Operation |
|-------------|--------|--------|-----------|
| `add` | 0000000 | 000 | rd = rs1 + rs2 |
| `sub` | 0100000 | 000 | rd = rs1 - rs2 |
| `and` | 0000000 | 111 | rd = rs1 & rs2 |
| `or` | 0000000 | 110 | rd = rs1 \| rs2 |
| `xor` | 0000000 | 100 | rd = rs1 ^ rs2 |
| `sll` | 0000000 | 001 | rd = rs1 << rs2 |
| `srl` | 0000000 | 101 | rd = rs1 >> rs2 (logical) |
| `sra` | 0100000 | 101 | rd = rs1 >> rs2 (arithmetic) |
| `slt` | 0000000 | 010 | rd = (rs1 < rs2) ? 1 : 0 |

### 1.2 I-Type Instructions (Immediate)

```
31            20 19    15 14  12 11     7 6      0
┌────────────────┬────────┬──────┬────────┬────────┐
│   imm[11:0]    │   rs1  │funct3│   rd   │ opcode │
└────────────────┴────────┴──────┴────────┴────────┘
```

| Instruction | funct3 | Operation |
|-------------|--------|-----------|
| `addi` | 000 | rd = rs1 + imm |
| `andi` | 111 | rd = rs1 & imm |
| `ori` | 110 | rd = rs1 \| imm |
| `xori` | 100 | rd = rs1 ^ imm |
| `slti` | 010 | rd = (rs1 < imm) ? 1 : 0 |
| `lw` | 010 | rd = Memory[rs1 + imm] |
| `lh` | 001 | rd = sign_ext(Memory[rs1 + imm]) (16-bit) |
| `lb` | 000 | rd = sign_ext(Memory[rs1 + imm]) (8-bit) |
| `lbu` | 100 | rd = zero_ext(Memory[rs1 + imm]) (8-bit) |

### 1.3 S-Type Instructions (Store)

| Instruction | funct3 | Operation |
|-------------|--------|-----------|
| `sw` | 010 | Memory[rs1 + imm] = rs2 (32-bit) |
| `sh` | 001 | Memory[rs1 + imm] = rs2 (16-bit) |
| `sb` | 000 | Memory[rs1 + imm] = rs2 (8-bit) |

### 1.4 B-Type Instructions (Branch)

| Instruction | funct3 | Condition |
|-------------|--------|-----------|
| `beq` | 000 | Branch if rs1 == rs2 |
| `bne` | 001 | Branch if rs1 != rs2 |
| `blt` | 100 | Branch if rs1 < rs2 (signed) |
| `bge` | 101 | Branch if rs1 >= rs2 (signed) |
| `bltu` | 110 | Branch if rs1 < rs2 (unsigned) |
| `bgeu` | 111 | Branch if rs1 >= rs2 (unsigned) |

---

## 2. C to Assembly: Simple Expressions

### 2.1 Variable Assignment

```c
int a = 5;
int b = 3;
int c = a + b;
```

Assembly (assuming a→x10, b→x11, c→x12):

```asm
addi x10, x0, 5      # a = 5
addi x11, x0, 3      # b = 3
add  x12, x10, x11   # c = a + b = 8
```

### 2.2 Complex Expressions

```c
int f = (a + b) - (c + d);
```

Assembly (a→x10, b→x11, c→x12, d→x13, f→x14):

```asm
add  x5, x10, x11    # temp1 = a + b
add  x6, x12, x13    # temp2 = c + d
sub  x14, x5, x6     # f = temp1 - temp2
```

Notice how the compiler uses **temporary registers** (x5, x6) for intermediate results.

### 2.3 Bitwise Operations

```c
int mask = value & 0xFF;        // Extract lowest byte
int shifted = value << 4;       // Multiply by 16
int toggled = flags ^ 0x01;     // Toggle bit 0
```

```asm
andi  x11, x10, 0xFF     # mask = value & 0xFF
slli  x12, x10, 4        # shifted = value << 4  (= value × 16)
xori  x13, x14, 0x01     # toggled = flags ^ 0x01
```

**Key insight:** Shift-left by $n$ is equivalent to multiplying by $2^n$. Compilers use this to replace multiplication by powers of 2, which is much faster than a hardware multiply.

---

## 3. C to Assembly: Conditional Statements

### 3.1 Simple If-Else

```c
if (a == b) {
    c = a + b;
} else {
    c = a - b;
}
```

```asm
      bne  x10, x11, else   # if (a != b) goto else
      add  x12, x10, x11    # c = a + b  (if branch)
      jal  x0, end           # goto end (skip else)
else: sub  x12, x10, x11    # c = a - b  (else branch)
end:  ...                    # continue
```

**Pattern:** The compiler typically inverts the condition and branches to the `else` block. The `jal x0, end` at the end of the `if` block is an unconditional jump (using x0 discards the return address since we don't need it).

### 3.2 Comparison Operators

Different C comparisons map to different branch instructions:

| C Condition | RISC-V Branch | Notes |
|------------|---------------|-------|
| `a == b` | `beq x10, x11, L` | |
| `a != b` | `bne x10, x11, L` | |
| `a < b` | `blt x10, x11, L` | Signed |
| `a >= b` | `bge x10, x11, L` | Signed |
| `a > b` | `blt x11, x10, L` | Swap operands! |
| `a <= b` | `bge x11, x10, L` | Swap operands! |

Notice that RISC-V doesn't have `bgt` or `ble` instructions — the compiler swaps the operands to use `blt` and `bge`. This is an example of "make the common case fast" — fewer instruction types, simpler decoder.

### 3.3 Multi-Way Conditional (Switch)

```c
switch (x) {
    case 0: result = a; break;
    case 1: result = b; break;
    case 2: result = c; break;
    default: result = d;
}
```

**Method 1: Chain of branches** (for small switch):

```asm
      beq  x10, x0, case0     # if x == 0
      addi x5, x0, 1
      beq  x10, x5, case1     # if x == 1
      addi x5, x0, 2
      beq  x10, x5, case2     # if x == 2
      jal  x0, default         # else: default
case0: add  x14, x11, x0      # result = a
      jal  x0, end
case1: add  x14, x12, x0      # result = b
      jal  x0, end
case2: add  x14, x13, x0      # result = c
      jal  x0, end
default: add x14, x15, x0     # result = d
end:  ...
```

**Method 2: Jump table** (for large, dense switch — more efficient):

```asm
      # x10 = switch variable, x20 = base of jump table
      slli  x5, x10, 2        # x5 = x * 4 (each table entry is 4 bytes)
      add   x5, x20, x5       # x5 = &jump_table[x]
      lw    x5, 0(x5)         # x5 = jump_table[x] (target address)
      jalr  x0, 0(x5)         # jump to target
```

---

## 4. C to Assembly: Loops

### 4.1 While Loop

```c
int sum = 0;
int i = 0;
while (i < 10) {
    sum += i;
    i++;
}
```

```asm
      addi x10, x0, 0         # sum = 0
      addi x11, x0, 0         # i = 0
      addi x12, x0, 10        # limit = 10
loop: bge  x11, x12, done     # if (i >= 10) exit loop
      add  x10, x10, x11      # sum += i
      addi x11, x11, 1        # i++
      jal  x0, loop            # goto loop
done: ...                      # sum is in x10 (= 45)
```

### 4.2 For Loop

```c
for (int i = 0; i < n; i++) {
    a[i] = a[i] * 2;
}
```

```asm
      addi x11, x0, 0         # i = 0
      # x12 = n, x13 = base address of a[]
loop: bge  x11, x12, done     # if (i >= n) exit
      slli x5, x11, 2         # x5 = i * 4 (word offset)
      add  x5, x13, x5        # x5 = &a[i]
      lw   x6, 0(x5)          # x6 = a[i]
      slli x6, x6, 1          # x6 = a[i] * 2 (shift left = ×2)
      sw   x6, 0(x5)          # a[i] = a[i] * 2
      addi x11, x11, 1        # i++
      jal  x0, loop            # goto loop
done: ...
```

### 4.3 Do-While Loop

```c
do {
    x = x >> 1;    // divide by 2
    count++;
} while (x != 0);
```

```asm
      # x10 = x, x11 = count
loop: srli x10, x10, 1        # x = x >> 1
      addi x11, x11, 1        # count++
      bne  x10, x0, loop       # if (x != 0) continue
      # loop done; count is in x11
```

The do-while loop places the condition check at the **bottom** — the body always executes at least once.

---

## 5. C to Assembly: Arrays and Memory

### 5.1 Array Access

```c
int a[100];
int x = a[5];       // Load
a[10] = x + 1;      // Store
```

```asm
# x13 = base address of a[]
lw   x10, 20(x13)     # x = a[5]  (5 × 4 = 20 byte offset)
addi x10, x10, 1      # x + 1
sw   x10, 40(x13)     # a[10] = x + 1  (10 × 4 = 40 byte offset)
```

### 5.2 Array Traversal (Sum)

```c
int sum = 0;
for (int i = 0; i < n; i++) {
    sum += a[i];
}
```

**Approach 1: Index-based** (compute address each iteration)

```asm
      addi x10, x0, 0        # sum = 0
      addi x11, x0, 0        # i = 0
loop: bge  x11, x12, done    # if (i >= n) exit
      slli x5, x11, 2        # offset = i * 4
      add  x5, x13, x5       # addr = base + offset
      lw   x6, 0(x5)         # load a[i]
      add  x10, x10, x6      # sum += a[i]
      addi x11, x11, 1       # i++
      jal  x0, loop
done: ...
```

**Approach 2: Pointer-based** (more efficient — increment pointer)

```asm
      addi x10, x0, 0        # sum = 0
      slli x5, x12, 2        # x5 = n * 4
      add  x5, x13, x5       # x5 = &a[n] (end pointer)
      add  x6, x13, x0       # x6 = &a[0] (current pointer)
loop: bge  x6, x5, done      # if (ptr >= end) exit
      lw   x7, 0(x6)         # load *ptr
      add  x10, x10, x7      # sum += *ptr
      addi x6, x6, 4         # ptr++ (advance by 4 bytes)
      jal  x0, loop
done: ...
```

The pointer-based approach avoids the `slli` + `add` for address calculation inside the loop — one fewer instruction per iteration. Optimizing compilers often perform this transformation automatically.

### 5.3 Strings (Character Arrays)

```c
int strlen(char *s) {
    int len = 0;
    while (s[len] != '\0') {
        len++;
    }
    return len;
}
```

```asm
strlen:
      addi x11, x0, 0        # len = 0
loop: add  x5, x10, x11      # addr = s + len
      lb   x6, 0(x5)         # load s[len] (byte)
      beq  x6, x0, done      # if (s[len] == '\0') exit
      addi x11, x11, 1       # len++
      jal  x0, loop
done: add  x10, x11, x0      # return value in a0 (x10)
      jalr x0, 0(x1)         # return to caller
```

---

## 6. C to Assembly: Functions

### 6.1 Function Call Convention

RISC-V defines a **calling convention** that specifies how functions communicate:

| Register | ABI Name | Role | Saved By |
|----------|----------|------|----------|
| x1 | ra | Return address | Caller |
| x2 | sp | Stack pointer | Callee |
| x5–x7 | t0–t2 | Temporaries | Caller |
| x8–x9 | s0–s1 | Saved | Callee |
| x10–x11 | a0–a1 | Arguments / Return value | Caller |
| x12–x17 | a2–a7 | Arguments | Caller |
| x18–x27 | s2–s11 | Saved | Callee |
| x28–x31 | t3–t6 | Temporaries | Caller |

**Caller-saved** registers may be overwritten by the called function — if the caller needs them after the call, it must save them to the stack first.

**Callee-saved** registers must be preserved by the called function — if it uses them, it must save the old values to the stack and restore them before returning.

### 6.2 Simple Function Call

```c
int add(int a, int b) {
    return a + b;
}

int main() {
    int result = add(3, 4);
}
```

```asm
# --- main ---
main:
      addi x10, x0, 3      # a0 = 3 (first argument)
      addi x11, x0, 4      # a1 = 4 (second argument)
      jal  x1, add          # call add; ra = return address
      # x10 now contains 7 (return value)
      ...

# --- add ---
add:
      add  x10, x10, x11   # a0 = a0 + a1 (result in a0)
      jalr x0, 0(x1)       # return to caller (jump to ra)
```

This is a **leaf function** (doesn't call other functions) — no need to save anything on the stack.

### 6.3 Nested Function Calls (Stack Usage)

```c
int multiply(int a, int b) {
    return a * b;   // assume M extension
}

int compute(int x, int y) {
    int temp = multiply(x, y);
    return temp + 1;
}
```

```asm
compute:
      # Prologue: save registers to stack
      addi sp, sp, -12      # allocate 12 bytes on stack
      sw   x1, 8(sp)        # save return address (ra)
      sw   x8, 4(sp)        # save s0
      sw   x9, 0(sp)        # save s1

      add  x8, x10, x0      # s0 = x (save argument)
      add  x9, x11, x0      # s1 = y (save argument)

      # Arguments already in a0, a1 for multiply
      jal  x1, multiply      # call multiply(x, y)
      # x10 = result of multiply

      addi x10, x10, 1      # return temp + 1

      # Epilogue: restore registers from stack
      lw   x1, 8(sp)        # restore ra
      lw   x8, 4(sp)        # restore s0
      lw   x9, 0(sp)        # restore s1
      addi sp, sp, 12       # deallocate stack space

      jalr x0, 0(x1)        # return
```

The **stack frame** for this function:

```
High Address
┌──────────────┐ ← sp (before call)
│   ra (x1)    │  sp + 8
├──────────────┤
│   s0 (x8)   │  sp + 4
├──────────────┤
│   s1 (x9)   │  sp + 0
└──────────────┘ ← sp (after prologue)
Low Address
```

### 6.4 Recursive Function

```c
int factorial(int n) {
    if (n <= 1) return 1;
    return n * factorial(n - 1);
}
```

```asm
factorial:
      # Base case check
      addi x5, x0, 1
      bge  x5, x10, base     # if (1 >= n) goto base

      # Recursive case: save state
      addi sp, sp, -8        # allocate stack space
      sw   x1, 4(sp)         # save return address
      sw   x10, 0(sp)        # save n

      addi x10, x10, -1      # a0 = n - 1
      jal  x1, factorial      # call factorial(n-1)
      # x10 = factorial(n-1)

      lw   x5, 0(sp)         # restore n
      lw   x1, 4(sp)         # restore return address
      addi sp, sp, 8         # deallocate stack

      mul  x10, x5, x10      # return n * factorial(n-1)
      jalr x0, 0(x1)         # return

base:
      addi x10, x0, 1        # return 1
      jalr x0, 0(x1)         # return
```

**Stack evolution for factorial(4):**

```
Call factorial(4): save ra, n=4          Stack: [ra4, 4]
  Call factorial(3): save ra, n=3        Stack: [ra4, 4] [ra3, 3]
    Call factorial(2): save ra, n=2      Stack: [ra4, 4] [ra3, 3] [ra2, 2]
      Call factorial(1): base case → return 1
    Return: 2 × 1 = 2
  Return: 3 × 2 = 6
Return: 4 × 6 = 24
```

---

## 7. Encoding a Complete Instruction

Let's encode a real instruction from start to finish.

**Instruction:** `add x9, x20, x21`

**Step 1: Identify the format** → R-type

**Step 2: Look up the fields:**

| Field | Value | Binary |
|-------|-------|--------|
| funct7 | 0000000 | 0000000 |
| rs2 | x21 | 10101 |
| rs1 | x20 | 10100 |
| funct3 | 000 | 000 |
| rd | x9 | 01001 |
| opcode | 0110011 | 0110011 |

**Step 3: Assemble:**

```
0000000 | 10101 | 10100 | 000 | 01001 | 0110011
funct7    rs2     rs1    f3    rd      opcode
```

Binary: `00000001010110100000010010110011`

Hex: `0x015A04B3`

**Step 4: Verify** — this 32-bit value is what gets stored in instruction memory and what the CPU fetches and decodes.

---

## 8. Pseudo-Instructions

RISC-V assembly provides **pseudo-instructions** — convenient shorthand that the assembler expands into real instructions:

| Pseudo-instruction | Actual Instruction(s) | Meaning |
|--------------------|-----------------------|---------|
| `mv x5, x6` | `addi x5, x6, 0` | Copy register |
| `li x5, 42` | `addi x5, x0, 42` | Load immediate |
| `li x5, 0x12345678` | `lui x5, 0x12345; addi x5, x5, 0x678` | Load large constant |
| `nop` | `addi x0, x0, 0` | No operation |
| `j label` | `jal x0, label` | Unconditional jump |
| `ret` | `jalr x0, 0(x1)` | Return from function |
| `call func` | `auipc x1, ...; jalr x1, ...` | Far function call |
| `not x5, x6` | `xori x5, x6, -1` | Bitwise NOT |
| `neg x5, x6` | `sub x5, x0, x6` | Negate |
| `beqz x5, L` | `beq x5, x0, L` | Branch if zero |
| `bnez x5, L` | `bne x5, x0, L` | Branch if not zero |

These make assembly code more readable without adding hardware complexity.

---

## 9. Complete Example: Bubble Sort

Let's bring everything together with a real algorithm:

```c
void bubble_sort(int *arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                // swap
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}
```

```asm
# x10 = arr (base address), x11 = n
bubble_sort:
      addi x18, x11, -1      # s2 = n - 1 (outer limit)
      addi x19, x0, 0        # s3 = i = 0 (outer counter)

outer:
      bge  x19, x18, done    # if (i >= n-1) exit

      sub  x20, x18, x19     # s4 = (n-1) - i (inner limit)
      addi x21, x0, 0        # s5 = j = 0 (inner counter)

inner:
      bge  x21, x20, next_i  # if (j >= n-1-i) next outer iteration

      slli x5, x21, 2        # x5 = j * 4
      add  x5, x10, x5       # x5 = &arr[j]
      lw   x6, 0(x5)         # x6 = arr[j]
      lw   x7, 4(x5)         # x7 = arr[j+1]

      bge  x7, x6, no_swap   # if (arr[j+1] >= arr[j]) skip swap

      # Swap: arr[j] and arr[j+1]
      sw   x7, 0(x5)         # arr[j] = arr[j+1]
      sw   x6, 4(x5)         # arr[j+1] = arr[j]

no_swap:
      addi x21, x21, 1       # j++
      jal  x0, inner          # continue inner loop

next_i:
      addi x19, x19, 1       # i++
      jal  x0, outer          # continue outer loop

done:
      jalr x0, 0(x1)         # return
```

This example shows every concept we've learned:
- **Loops** (nested for loops with branch instructions)
- **Array access** (slli + add for index calculation, lw/sw for load/store)
- **Conditionals** (bge for comparison, branch to skip swap)
- **Register usage** (saved registers for loop counters, temporaries for addresses/values)

---

## 10. Summary

| Topic | Key Takeaway |
|-------|-------------|
| **R/I/S/B formats** | Each instruction type has a specific encoding; register positions are consistent |
| **Expressions** | Map directly to add, sub, and/or/xor, shift instructions |
| **Conditionals** | Compiler inverts condition and branches to else block |
| **Loops** | Condition check at top (while/for) or bottom (do-while) with backward branch |
| **Arrays** | Index × element_size for byte offset; pointer-based traversal is more efficient |
| **Functions** | Caller/callee-saved registers; stack for saving state; jal/jalr for call/return |
| **Recursion** | Each call pushes state onto stack; stack unwinds on return |
| **Pseudo-instructions** | Convenient shorthand (mv, li, ret, nop) expanded by assembler |

In the **next post ([SoC-07])**, we will start building the actual **hardware** that executes these instructions — beginning with the building blocks of a **single-cycle RISC-V processor**.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
