---
title: "C Functions and Macros: How Data Flows and Code Gets Built"
date: 2026-02-04
description: "Visual guide to call by value, call by reference, static scope, #define macros, conditional compilation, and header guards in C"
categories: ["Computer Science"]
tags: ["C", "Programming", "Pointers", "Functions", "Macros", "Preprocessor"]
draft: false
---

{{< katex >}}

## 1. Call by Value

When you pass a variable to a function **by value**, C copies the value into a brand-new local variable. The original is never touched.

```c
void calibrate(int value) {
    value = value + 10;        // modifies the LOCAL copy only
}

int main(void) {
    int temperature = 25;
    calibrate(temperature);     // passes a COPY of 25
    printf("%d\n", temperature); // 25  -- unchanged!
}
```

### What happens in memory

```
 ┌──────────────────────────────────────────────────────────┐
 │  main() stack frame            calibrate() stack frame   │
 │                                                          │
 │  temperature                   value                     │
 │  ┌──────────┐    copy 25 →    ┌──────────┐              │
 │  │    25    │ ──────────────► │    25    │              │
 │  └──────────┘                 └──────────┘              │
 │       │                            │                     │
 │       │                       value = value + 10         │
 │       │                            │                     │
 │       ▼                            ▼                     │
 │  ┌──────────┐                 ┌──────────┐              │
 │  │    25    │                 │    35    │              │
 │  └──────────┘                 └──────────┘              │
 │   (untouched)              (destroyed on return)         │
 └──────────────────────────────────────────────────────────┘
```

**Key point**: the function receives its own independent copy. Changing `value` inside `calibrate` has zero effect on `temperature` in `main`.

---

## 2. Call by Reference (Pass by Address)

To let a function **modify the original** variable, pass its address using a pointer.

```c
void calibrate(int *value) {
    *value = *value + 10;      // dereferences the pointer → writes to the original
}

int main(void) {
    int temperature = 25;
    calibrate(&temperature);    // passes the ADDRESS of temperature
    printf("%d\n", temperature); // 35  -- changed!
}
```

### What happens in memory

```
 ┌──────────────────────────────────────────────────────────┐
 │  main() stack frame            calibrate() stack frame   │
 │                                                          │
 │  temperature                   value (pointer)           │
 │  addr: 0x1000                  ┌──────────┐              │
 │  ┌──────────┐                 │  0x1000  │──┐           │
 │  │    25    │                 └──────────┘  │           │
 │  └──────────┘                               │           │
 │       ▲                                     │           │
 │       │              *value = 35            │           │
 │       └─────────────────────────────────────┘           │
 │                                                          │
 │  ┌──────────┐                                            │
 │  │    35    │  ← the original is modified!               │
 │  └──────────┘                                            │
 └──────────────────────────────────────────────────────────┘
```

### Side-by-side comparison

```
  Call by VALUE                    Call by REFERENCE (address)
 ─────────────────                ─────────────────────────────
  void f(int val)                  void f(int *val)

  ┌─────┐  copy  ┌─────┐         ┌─────┐  addr   ┌─────────┐
  │  25 │ ────► │  25 │         │  25 │ ◄────── │ &(0x1000)│
  └─────┘        └─────┘         └─────┘         └─────────┘
  original       local copy       original        pointer to it

  original: 25 (safe)             original: 35 (modified via *)
```

---

## 3. The `static` Keyword: Controlling Visibility

`static` limits the scope of a variable or function to the **current file only**. Nothing outside the file can see it.

```c
// sensor.c
static int var = 0;                // only accessible inside sensor.c
static void private_func(void) {}  // only callable inside sensor.c
```

### Visibility diagram

```
 ┌─────────── sensor.c ───────────┐    ┌─────────── main.c ──────────┐
 │                                 │    │                              │
 │  static int var = 0;      OK   │    │  extern int var;      ERROR  │
 │  static void private_func();   │    │  private_func();      ERROR  │
 │                                 │    │                              │
 │  int public_var = 0;      OK   │    │  extern int public_var;  OK  │
 │  void public_func();      OK   │    │  public_func();          OK  │
 │                                 │    │                              │
 └─────────────────────────────────┘    └──────────────────────────────┘

              │                                      │
              ▼                                      ▼
       static = file-private                 non-static = globally visible
```

---

## 4. C Memory Layout: Where Everything Lives

Every C program's memory is divided into distinct regions. Here is where each type of variable and function is stored:

```
 ┌───────────────────────────────────────────────────────────────┐
 │                    C PROGRAM MEMORY MAP                       │
 ├───────────────────────────────────────────────────────────────┤
 │                                                               │
 │  ┌─────────────────────────────────────────────────────────┐  │
 │  │  TEXT (Code) Segment                      [Read-Only]   │  │
 │  │                                                         │  │
 │  │  • Function code: main(), calibrate(), printf()         │  │
 │  │  • String literals: "Hello, World!"                     │  │
 │  │  • const variables (sometimes)                          │  │
 │  └─────────────────────────────────────────────────────────┘  │
 │                                                               │
 │  ┌─────────────────────────────────────────────────────────┐  │
 │  │  DATA Segment                                           │  │
 │  │                                                         │  │
 │  │  Initialized:                                           │  │
 │  │  • int global_var = 10;          (global)               │  │
 │  │  • static int count = 5;         (static)               │  │
 │  │                                                         │  │
 │  │  Uninitialized (BSS):                                   │  │
 │  │  • int global_var;               (defaults to 0)        │  │
 │  │  • static int count;             (defaults to 0)        │  │
 │  └─────────────────────────────────────────────────────────┘  │
 │                                                               │
 │  ┌─────────────────────────────────────────────────────────┐  │
 │  │  HEAP                                  grows ↓          │  │
 │  │                                                         │  │
 │  │  • int *p = malloc(sizeof(int));                        │  │
 │  │  • char *str = calloc(100, 1);                          │  │
 │  │  • Must be freed manually: free(p);                     │  │
 │  │                                                         │  │
 │  │                        ↓ ↓ ↓                            │  │
 │  │                    (grows down)                          │  │
 │  │                                                         │  │
 │  │                    (grows up)                            │  │
 │  │                        ↑ ↑ ↑                            │  │
 │  │                                                         │  │
 │  │  STACK                                 grows ↑          │  │
 │  │                                                         │  │
 │  │  • Local variables: int temperature = 25;               │  │
 │  │  • Function parameters: int value (copy)                │  │
 │  │  • Return addresses                                     │  │
 │  └─────────────────────────────────────────────────────────┘  │
 │                                                               │
 └───────────────────────────────────────────────────────────────┘
```

### Mapping our examples to memory

```c
int global_count = 0;              // DATA segment (initialized)
static int file_count = 0;        // DATA segment (static, initialized)

void calibrate(int value) {        // TEXT segment (function code)
    value = value + 10;            // STACK (local parameter)
}

int main(void) {                   // TEXT segment (function code)
    int temperature = 25;          // STACK (local variable)
    int *p = malloc(sizeof(int));  // p on STACK, *p on HEAP
    calibrate(temperature);
    free(p);
}
```

```
  Code                    Where in memory?
 ──────────────────────  ─────────────────────────────────
  calibrate() code        TEXT    ← function instructions
  main() code             TEXT    ← function instructions
  "Hello, World!"         TEXT    ← string literal

  global_count = 0        DATA    ← global, initialized
  file_count = 0          DATA    ← static, initialized

  *p (malloc'd data)      HEAP    ← dynamically allocated

  temperature = 25        STACK   ← local variable in main()
  value = 25              STACK   ← parameter copy in calibrate()
  p (the pointer itself)  STACK   ← local variable in main()
```

### Lifetime comparison

| Region | Created | Destroyed | Example |
|---|---|---|---|
| **Text** | Program start | Program end | `calibrate()`, `main()` |
| **Data** | Program start | Program end | `global_count`, `static int` |
| **Heap** | `malloc()` call | `free()` call | `*p` |
| **Stack** | Function call | Function return | `temperature`, `value` |

---

## 5. `#define` and Macros

### 5.1 `#define` = Text Substitution

`#define` is **not** a variable. Before the compiler ever sees your code, the **preprocessor** replaces every occurrence with the literal text you defined.

```c
#define MAX_SENSORS    16
#define TEMP_SENSOR_ID 0x01
```

```
 ┌──── Your source code ────┐        ┌─── After preprocessing ───┐
 │                           │        │                            │
 │  int count = MAX_SENSORS; │  ───►  │  int count = 16;          │
 │                           │        │                            │
 │  if (id == TEMP_SENSOR_ID)│  ───►  │  if (id == 0x01)          │
 │                           │        │                            │
 └───────────────────────────┘        └────────────────────────────┘
         you write this                    compiler sees this
```

### 5.2 Conditional Compilation

Entire blocks of code can be included or excluded **at compile time** based on defined symbols.

```c
#define DEBUG_MODE 1

#if DEBUG_MODE
    printf("Temperature: %d\n", temp);   // included when DEBUG_MODE is 1
#endif
```

```
                    ┌──────────────────┐
                    │  DEBUG_MODE = 1? │
                    └────────┬─────────┘
                       yes / \ no
                          /   \
           ┌─────────────┐     ┌──────────────────┐
           │  printf()   │     │  (code removed   │
           │  compiled   │     │   entirely)       │
           └─────────────┘     └──────────────────┘
```

You can also switch between configurations:

```c
#ifdef USE_CELSIUS
    #define TEMP_UNIT "C"
#else
    #define TEMP_UNIT "F"
#endif
```

```
   ┌──────────────────────────────────────────────────────┐
   │                 Compile with flag?                    │
   │                                                      │
   │    gcc -DUSE_CELSIUS main.c        gcc main.c        │
   │          │                              │             │
   │          ▼                              ▼             │
   │    TEMP_UNIT = "C"              TEMP_UNIT = "F"       │
   └──────────────────────────────────────────────────────┘
```

### 5.3 Header Guards

When multiple files `#include` the same header, its contents could be inserted **more than once**, causing duplicate definition errors. Header guards prevent this.

```c
// sensor.h
#ifndef SENSOR_H        // if SENSOR_H is NOT yet defined...
#define SENSOR_H        // ...define it now (so next #include skips)

typedef struct {
    int id;
    float value;
} Sensor;

void sensor_init(Sensor *s);

#endif                  // end of guard
```

### How the guard works across multiple includes

```
 ┌──── First #include "sensor.h" ────┐
 │                                    │
 │  #ifndef SENSOR_H  →  true        │
 │  #define SENSOR_H                  │
 │  ... contents included ...         │
 │  #endif                            │
 └────────────────────────────────────┘

 ┌──── Second #include "sensor.h" ───┐
 │                                    │
 │  #ifndef SENSOR_H  →  false       │
 │  (SENSOR_H already defined)        │
 │  ... entire file SKIPPED ...       │
 │  #endif                            │
 └────────────────────────────────────┘
```

```
  Without header guard:              With header guard:
 ┌──────────────────────┐          ┌──────────────────────┐
 │  main.c              │          │  main.c              │
 │  ┌────────────────┐  │          │  ┌────────────────┐  │
 │  │ #include "s.h" │──┼─► copy   │  │ #include "s.h" │──┼─► copy
 │  └────────────────┘  │          │  └────────────────┘  │
 │  ┌────────────────┐  │          │  ┌────────────────┐  │
 │  │ #include "s.h" │──┼─► copy   │  │ #include "s.h" │──┼─► SKIPPED
 │  └────────────────┘  │          │  └────────────────┘  │
 │                      │          │                      │
 │  ERROR: duplicate!   │          │  OK: only one copy   │
 └──────────────────────┘          └──────────────────────┘
```

---

## 6. Summary

| Concept | Mechanism | Key Takeaway |
|---|---|---|
| **Call by value** | Copies the value | Original is safe, function works on a copy |
| **Call by reference** | Passes the address (`&`) | Function can modify the original via `*` |
| **`static`** | Limits linkage to current file | Hides internal details from other files |
| **Memory layout** | Text, Data, Heap, Stack | Know where each variable lives and when it dies |
| **`#define`** | Text substitution before compile | Not a variable -- just find-and-replace |
| **Conditional compilation** | `#if` / `#ifdef` / `#ifndef` | Include or exclude code at compile time |
| **Header guards** | `#ifndef` + `#define` pattern | Prevents duplicate inclusion of headers |
