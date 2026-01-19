---
title: "Procedure Calls in Computer Architecture"
date: 2024-06-21
description: "Understanding procedure calls, stack frames, and calling conventions"
categories: ["Computer Science"]
tags: ["Computer Structure", "Assembly", "Stack"]
draft: false
---

## Overview

Procedure calls are fundamental to structured programming. Understanding how they work at the hardware level is essential for system programming and debugging.

## The Call Stack

```
High Address
┌─────────────────┐
│   Arguments     │
├─────────────────┤
│ Return Address  │
├─────────────────┤
│   Saved FP      │  ← Frame Pointer (FP)
├─────────────────┤
│ Local Variables │
├─────────────────┤
│ Saved Registers │
├─────────────────┤
│      ...        │  ← Stack Pointer (SP)
└─────────────────┘
Low Address
```

## Procedure Call Steps

### 1. Caller Actions (Before Call)

1. Save caller-saved registers
2. Push arguments onto stack (or use registers)
3. Execute `call` instruction
   - Push return address
   - Jump to procedure

### 2. Callee Prologue

```assembly
push    rbp          ; Save old frame pointer
mov     rbp, rsp     ; Set new frame pointer
sub     rsp, N       ; Allocate local variables
push    rbx          ; Save callee-saved registers
```

### 3. Procedure Body

Execute the function code using:
- Parameters (from stack/registers)
- Local variables (on stack)

### 4. Callee Epilogue

```assembly
pop     rbx          ; Restore callee-saved registers
mov     rsp, rbp     ; Deallocate locals
pop     rbp          ; Restore old frame pointer
ret                  ; Return (pop return address, jump)
```

### 5. Caller Actions (After Return)

1. Clean up arguments (if caller-cleanup)
2. Restore caller-saved registers
3. Use return value

## Register Conventions (x86-64)

### Caller-Saved (Volatile)

| Register | Purpose |
|----------|---------|
| RAX | Return value |
| RCX, RDX, R8, R9 | Arguments 1-4 (Windows) |
| RDI, RSI, RDX, RCX, R8, R9 | Arguments 1-6 (Linux) |
| R10, R11 | Temporary |

### Callee-Saved (Non-volatile)

| Register | Purpose |
|----------|---------|
| RBX | General purpose |
| RBP | Frame pointer |
| R12-R15 | General purpose |
| RSP | Stack pointer |

## Calling Conventions

### cdecl (C Declaration)

- Arguments: Right to left on stack
- Caller cleans stack
- Return: EAX/RAX

### stdcall (Windows API)

- Arguments: Right to left on stack
- Callee cleans stack
- Return: EAX/RAX

### System V AMD64 (Linux)

- Arguments: RDI, RSI, RDX, RCX, R8, R9, then stack
- Caller cleans stack
- Return: RAX (+ RDX for 128-bit)

### Microsoft x64 (Windows)

- Arguments: RCX, RDX, R8, R9, then stack
- 32 bytes shadow space required
- Return: RAX

## Stack Frame Example

```c
int add(int a, int b) {
    int result = a + b;
    return result;
}
```

Assembly (x86-64, System V):

```assembly
add:
    push    rbp
    mov     rbp, rsp
    mov     DWORD PTR [rbp-20], edi   ; a
    mov     DWORD PTR [rbp-24], esi   ; b
    mov     edx, DWORD PTR [rbp-20]
    mov     eax, DWORD PTR [rbp-24]
    add     eax, edx
    mov     DWORD PTR [rbp-4], eax    ; result
    mov     eax, DWORD PTR [rbp-4]
    pop     rbp
    ret
```

## Recursive Calls

Each call creates new stack frame:

```
┌─────────────────┐
│ factorial(1)    │
├─────────────────┤
│ factorial(2)    │
├─────────────────┤
│ factorial(3)    │
├─────────────────┤
│ main()          │
└─────────────────┘
```

**Stack overflow** occurs when recursion is too deep.

## Tail Call Optimization

When the last action is a function call, reuse current frame:

```c
// Without TCO: O(n) stack space
int factorial(int n, int acc) {
    if (n <= 1) return acc;
    return factorial(n - 1, n * acc);  // Tail call
}
```

Compiler can optimize to:

```assembly
factorial:
    cmp     edi, 1
    jle     .done
    imul    esi, edi
    dec     edi
    jmp     factorial    ; Jump, not call
.done:
    mov     eax, esi
    ret
```

## Key Concepts

| Term | Description |
|------|-------------|
| Activation Record | Another name for stack frame |
| Leaf Function | Function that makes no calls |
| Prologue | Setup code at function start |
| Epilogue | Cleanup code at function end |
| ABI | Application Binary Interface |
