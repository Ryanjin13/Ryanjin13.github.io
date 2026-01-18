---
title: "C Compile Process"
date: 2025-07-27
description: "C compilation process: object files, static libraries, and linking"
categories: ["Computer Science"]
tags: ["Programming", "C"]
draft: false
---

## Overview

This post explains the C compilation process using practical examples with multiple source files, object files, and static libraries.

## 1. Example Files

### math_utils.h
```c
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

int add(int a, int b);
int multiply(int a, int b);
int subtract(int a, int b);

#endif
```

### add.c
```c
#include "math_utils.h"

int add(int a, int b) {
    return a + b;
}
```

### multiply.c
```c
#include "math_utils.h"

int multiply(int a, int b) {
    return a * b;
}
```

### subtract.c
```c
#include "math_utils.h"

int subtract(int a, int b) {
    return a - b;
}
```

### main.c
```c
#include <stdio.h>
#include "math_utils.h"

int main() {
    int x = 10, y = 5;

    printf("Add: %d\n", add(x, y));
    printf("Multiply: %d\n", multiply(x, y));

    return 0;
}
```

## 2. Compilation to Object Files

Compile each source file separately:

```bash
gcc -c add.c -o add.o
gcc -c multiply.c -o multiply.o
gcc -c subtract.c -o subtract.o
gcc -c main.c -o main.o
```

## 3. Using `nm` Tool

Check symbol table with `nm` command:

```bash
nm add.o
nm main.o
```

Key symbols:
- **T** - Functions defined in that object file
- **U** - Undefined symbols to be resolved elsewhere

## 4. Using `objdump` Tool

```bash
objdump -r main.o    # Relocation entries
objdump -d main.o    # Disassemble
objdump -h main.o    # Section headers
objdump -t myprogram # Symbol table
```

| Flag | Description |
|------|-------------|
| `-d` | Disassemble executable sections (assembly code) |
| `-r` | Display relocation entries (e.g., `R_X86_64_PLT32`) |
| `-h` | Section headers and memory layout |
| `-t` | Symbol table |

## 5. Creating Static Libraries

Bundle multiple object files into a static library:

```bash
ar rcs libmath.a add.o multiply.o subtract.o
```

## 6. Linking Process

Link objects with libraries to create final executable:

```bash
gcc main.o -L. -lmath -o myprogram
```

Real addresses are assigned to functions at this stage.

## 7. Optimization Levels

```bash
gcc -O0 main.c -o program_slow      # No optimization
gcc -O1 main.c -o program_basic     # Basic optimization
gcc -O2 main.c -o program_standard  # Recommended optimization
gcc -O3 main.c -o program_fast      # Aggressive optimization
gcc -Os main.c -o program_small     # Size optimization
```

| Flag | Description |
|------|-------------|
| `-O0` | No optimization (for debugging) |
| `-O1` | Basic optimization |
| `-O2` | Recommended optimization |
| `-O3` | Aggressive optimization |
| `-Os` | Size optimization |
