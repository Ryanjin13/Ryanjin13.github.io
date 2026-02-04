---
title: "C Callback Functions: From Function Pointers to Real-World Patterns"
date: 2026-02-04
description: "Visual guide to function pointers, typedef, callback registration, volatile, and user data patterns in C"
categories: ["Computer Science"]
tags: ["C", "Programming", "Callbacks", "Function Pointers", "Embedded Systems"]
draft: false
---

{{< katex >}}

## 1. What Is a Callback?

A **callback** is a function that you pass to another module so it can call you back when something happens. You don't call it directly -- the system calls it for you.

```
 ┌─────────────────┐                    ┌─────────────────┐
 │   Your Code     │                    │  Sensor Manager  │
 │   (main.c)      │                    │  (library)       │
 │                  │   1. register      │                  │
 │  my_handler() ──────────────────────►│  stores pointer  │
 │                  │                    │                  │
 │                  │   2. event occurs  │                  │
 │                  │◄──────────────────── calls my_handler │
 │  my_handler()   │                    │                  │
 │  executes!      │                    │                  │
 └─────────────────┘                    └─────────────────┘

 You write the function.
 The library decides WHEN to call it.
```

---

## 2. Function Pointers: The Mechanism Behind Callbacks

A function pointer stores the **address of a function**, just like a regular pointer stores the address of a variable.

```c
void greet(int id) {
    printf("Hello from sensor %d\n", id);
}

int main(void) {
    void (*func_ptr)(int) = greet;   // store address of greet()
    func_ptr(42);                     // call greet(42) via pointer
}
```

```
 Memory layout:

 TEXT segment (code)
 ┌────────────────────────────────┐
 │ 0x4000: greet() instructions   │
 │         printf("Hello ...")    │
 └────────────────────────────────┘
         ▲
         │
 STACK   │
 ┌───────┴────────────────────────┐
 │ func_ptr = 0x4000              │── points to greet()'s code
 └────────────────────────────────┘

 func_ptr(42)  →  jump to 0x4000  →  greet(42) executes
```

### Reading function pointer syntax

```
 void (*func_ptr)(int, float)
  │      │         │
  │      │         └── parameter types this function takes
  │      └── name of the pointer variable
  └── return type of the function

 Read it as: "func_ptr is a pointer to a function
              that takes (int, float) and returns void"
```

---

## 3. Cleaning Up with `typedef`

### 3.1 The Problem: Raw function pointers are hard to read

```c
// Without typedef -- every declaration repeats the full signature
void (*on_temp_cb)(int sensor_id, float value);
void (*on_humid_cb)(int sensor_id, float value);
void (*on_motion_cb)(int sensor_id, int detected);
```

### 3.2 The Solution: `typedef` gives the type a name

```c
typedef void (*TempCallback)(int sensor_id, float value);
typedef void (*HumidCallback)(int sensor_id, float value);
typedef void (*MotionCallback)(int sensor_id, int detected);
```

```
 typedef void (*TempCallback)(int, float);
 │        │      │              │
 │        │      │              └── parameter types
 │        │      └── new type name (you choose this)
 │        └── return type
 └── "define a type alias"

 After this, TempCallback IS a type, just like int or float.
```

Now declarations are clean:

```c
// Use it like any other type
TempCallback   my_temp_handler;
HumidCallback  my_humid_handler;
MotionCallback my_motion_handler;
```

```
 Without typedef:                    With typedef:
 ──────────────────────────         ──────────────────────────
 void (*cb)(int, float);            TempCallback cb;

 void register(                     void register(
   void (*cb)(int, float)             TempCallback cb
 );                                 );

 Hard to read                        Reads like English
```

---

## 4. Practical Example: Sensor Callback System

### 4.1 Architecture Overview

```
 ┌──────────────────────────────────────────────────────────────┐
 │                    Smart Home System                         │
 │                                                              │
 │  ┌─────────────┐    ┌──────────────────┐    ┌────────────┐  │
 │  │  main.c     │    │ sensor_manager.c │    │ sensor_    │  │
 │  │             │    │                  │    │ manager.h  │  │
 │  │ App logic   │    │ Core engine      │    │ Interface  │  │
 │  │ + handlers  │    │ + callback       │    │ (types +   │  │
 │  │             │    │   invocation     │    │  API)      │  │
 │  └──────┬──────┘    └────────┬─────────┘    └─────┬──────┘  │
 │         │                    │                     │         │
 │         │   registers        │    #include         │         │
 │         │   callbacks ──────►│◄────────────────────┘         │
 │         │                    │                               │
 │         │◄── calls back ─────│                               │
 │         │    when event      │                               │
 │         │    occurs          │                               │
 └─────────┴────────────────────┴───────────────────────────────┘
```

### 4.2 Step 1: Header File (sensor_manager.h)

```c
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// --- Callback type definitions ---
typedef void (*TempCallback)(int sensor_id, float temperature);
typedef void (*HumidCallback)(int sensor_id, float humidity);
typedef void (*MotionCallback)(int sensor_id, int detected);
typedef void (*ErrorCallback)(int error_code, const char *message);

// --- Bundle all callbacks into one struct ---
typedef struct {
    TempCallback   on_temperature;
    HumidCallback  on_humidity;
    MotionCallback on_motion;
    ErrorCallback  on_error;
} SensorCallbacks;

// --- API ---
void sensor_init(void);
int  register_callbacks(SensorCallbacks *callbacks);
void start_monitoring(void);
void process_temperature(int sensor_id, float value);
void process_humidity(int sensor_id, float value);
void process_motion(int sensor_id, int detected);

#endif
```

### Why bundle callbacks in a struct?

```
 Individual registration:            Struct registration:
 ────────────────────────           ────────────────────────
 register_temp(handler1);           SensorCallbacks cb = {
 register_humid(handler2);              .on_temperature = h1,
 register_motion(handler3);             .on_humidity    = h2,
 register_error(handler4);              .on_motion      = h3,
                                        .on_error       = h4
 4 separate API calls                };
 Hard to add new types              register_callbacks(&cb);

                                    1 API call
                                    Easy to extend
```

### 4.3 Step 2: Implementation (sensor_manager.c)

```c
#include "sensor_manager.h"
#include <stdio.h>
#include <string.h>

// --- Internal state (static = file-private) ---
static SensorCallbacks g_callbacks;
static int g_initialized = 0;

void sensor_init(void) {
    memset(&g_callbacks, 0, sizeof(g_callbacks));  // all pointers = NULL
    g_initialized = 1;
    printf("[Sensor Manager] Initialized\n");
}

int register_callbacks(SensorCallbacks *callbacks) {
    if (callbacks == NULL) return -1;
    if (!g_initialized)   return -2;

    g_callbacks = *callbacks;   // copy the struct contents
    printf("[Sensor Manager] Callbacks registered\n");
    return 0;
}

void process_temperature(int sensor_id, float value) {
    printf("[Sensor Manager] Temperature: sensor=%d, value=%.1f\n",
           sensor_id, value);

    if (g_callbacks.on_temperature) {           // NULL check first!
        g_callbacks.on_temperature(sensor_id, value);  // invoke callback
    }
}

void process_humidity(int sensor_id, float value) {
    printf("[Sensor Manager] Humidity: sensor=%d, value=%.1f\n",
           sensor_id, value);

    if (g_callbacks.on_humidity) {
        g_callbacks.on_humidity(sensor_id, value);
    }
}

void process_motion(int sensor_id, int detected) {
    printf("[Sensor Manager] Motion: sensor=%d, detected=%d\n",
           sensor_id, detected);

    if (g_callbacks.on_motion) {
        g_callbacks.on_motion(sensor_id, detected);
    }
}

void start_monitoring(void) {
    printf("[Sensor Manager] Monitoring started...\n");
    process_temperature(1, 25.5f);
    process_humidity(2, 65.0f);
    process_motion(3, 1);
    process_temperature(1, 28.0f);
}
```

### The critical line: `g_callbacks = *callbacks`

This is a **value copy** of the entire struct, not a pointer assignment.

```
 register_callbacks(&callbacks)

 ┌──── main.c (caller) ──────┐      ┌──── sensor_manager.c ──────┐
 │                            │      │                             │
 │  callbacks (on stack)      │      │  g_callbacks (static/DATA)  │
 │  ┌──────────────────────┐  │      │  ┌──────────────────────┐   │
 │  │ .on_temperature=0x40 │──┼─copy─┼─►│ .on_temperature=0x40 │   │
 │  │ .on_humidity   =0x41 │──┼─copy─┼─►│ .on_humidity   =0x41 │   │
 │  │ .on_motion     =0x42 │──┼─copy─┼─►│ .on_motion     =0x42 │   │
 │  │ .on_error      =0x43 │──┼─copy─┼─►│ .on_error      =0x43 │   │
 │  └──────────────────────┘  │      │  └──────────────────────┘   │
 │                            │      │                             │
 │  (destroyed after main     │      │  (lives for entire program  │
 │   returns -- that's OK,    │      │   lifetime -- safe to call  │
 │   values were copied)      │      │   anytime)                  │
 └────────────────────────────┘      └─────────────────────────────┘
```

### 4.4 Step 3: Application Code (main.c)

```c
#include "sensor_manager.h"
#include <stdio.h>

// --- Your callback implementations ---

void my_temp_handler(int sensor_id, float temperature) {
    printf("  [APP] Temp alert: sensor %d reads %.1f C\n",
           sensor_id, temperature);
    if (temperature >= 27.0f) {
        printf("  [APP] WARNING: High temp! Turning on AC.\n");
    }
}

void my_humid_handler(int sensor_id, float humidity) {
    printf("  [APP] Humidity alert: sensor %d reads %.1f%%\n",
           sensor_id, humidity);
    if (humidity >= 70.0f) {
        printf("  [APP] WARNING: High humidity! Turning on dehumidifier.\n");
    }
}

void my_motion_handler(int sensor_id, int detected) {
    printf("  [APP] Motion alert: sensor %d - %s\n",
           sensor_id, detected ? "movement detected!" : "idle");
    if (detected) {
        printf("  [APP] Turning on lights.\n");
    }
}

void my_error_handler(int error_code, const char *message) {
    printf("  [APP] Error %d: %s\n", error_code, message);
}

int main(void) {
    printf("=== Smart Home Sensor System ===\n\n");

    // 1. Initialize
    sensor_init();

    // 2. Set up callbacks
    SensorCallbacks callbacks;
    callbacks.on_temperature = my_temp_handler;
    callbacks.on_humidity    = my_humid_handler;
    callbacks.on_motion      = my_motion_handler;
    callbacks.on_error       = my_error_handler;

    // 3. Register
    register_callbacks(&callbacks);

    // 4. Start
    printf("\n--- Monitoring started ---\n\n");
    start_monitoring();

    printf("\n--- Done ---\n");
    return 0;
}
```

### 4.5 Full Execution Flow

```
 main()                          sensor_manager              my_temp_handler()
   │                                   │                           │
   │  sensor_init()                    │                           │
   │──────────────────────────────────►│ g_callbacks = {NULL}      │
   │                                   │                           │
   │  register_callbacks(&cb)          │                           │
   │──────────────────────────────────►│ g_callbacks = cb (copy)   │
   │                                   │                           │
   │  start_monitoring()               │                           │
   │──────────────────────────────────►│                           │
   │                                   │                           │
   │                                   │ process_temperature(1,25.5)
   │                                   │──── g_callbacks            │
   │                                   │     .on_temperature ──────►│
   │                                   │     (sensor=1, val=25.5)   │
   │                                   │◄──────────────────────────│
   │                                   │                           │
   │                                   │ process_humidity(2, 65.0)  │
   │                                   │──── g_callbacks            │
   │                                   │     .on_humidity ─────────►│
   │                                   │                     (similar)
   │                                   │                           │
   │◄──────────────────────────────────│ return                    │
   │                                   │                           │
```

### 4.6 Output

```
=== Smart Home Sensor System ===

[Sensor Manager] Initialized
[Sensor Manager] Callbacks registered

--- Monitoring started ---

[Sensor Manager] Monitoring started...
[Sensor Manager] Temperature: sensor=1, value=25.5
  [APP] Temp alert: sensor 1 reads 25.5 C
[Sensor Manager] Humidity: sensor=2, value=65.0
  [APP] Humidity alert: sensor 2 reads 65.0%
[Sensor Manager] Motion: sensor=3, detected=1
  [APP] Motion alert: sensor 3 - movement detected!
  [APP] Turning on lights.
[Sensor Manager] Temperature: sensor=1, value=28.0
  [APP] Temp alert: sensor 1 reads 28.0 C
  [APP] WARNING: High temp! Turning on AC.

--- Done ---
```

---

## 5. `volatile` and Callbacks in Embedded Systems

### 5.1 The Problem Without `volatile`

In embedded systems, hardware interrupts can change variables at any time. The compiler doesn't know this, so it may **optimize the variable into a register** and never re-read it from memory.

```c
int data_ready = 0;  // shared between main loop and interrupt

void main_loop(void) {
    while (!data_ready) {
        // compiler may optimize this into an infinite loop!
        // it thinks data_ready never changes inside this loop
    }
}
```

```
 What the compiler sees:              What actually happens:

 data_ready = 0                       data_ready = 0
      │                                    │
      ▼                                    ▼
 ┌─────────────────┐                 ┌─────────────────┐
 │ Load data_ready  │                 │ Load data_ready  │
 │ into register    │                 │ into register    │
 │ (value = 0)      │                 │ (value = 0)      │
 └────────┬────────┘                 └────────┬────────┘
          │                                    │
          ▼                               INTERRUPT fires!
 ┌─────────────────┐                 data_ready = 1 (in memory)
 │ Loop forever    │                       │
 │ (register = 0,  │                       ▼
 │  never re-reads │                 ┌─────────────────┐
 │  from memory)   │                 │ But register     │
 └─────────────────┘                 │ still holds 0!   │
                                     │ Loop continues   │
  BUG: infinite loop                 └─────────────────┘
```

### 5.2 The Fix: `volatile`

`volatile` tells the compiler: **"this variable can change at any time -- always read it from memory."**

```c
volatile int data_ready = 0;
volatile float last_temperature = 0.0f;

// Interrupt handler (called by hardware)
void sensor_interrupt_handler(void) {
    last_temperature = read_sensor_value();
    data_ready = 1;
}

// Main loop
void main_loop(void) {
    while (!data_ready) {
        // volatile forces a fresh read from memory every iteration
    }
    process_temperature(1, last_temperature);
    data_ready = 0;
}
```

```
 With volatile:

 Memory              CPU Register
 ┌──────────┐
 │ data_ready│
 │   = 0     │◄──── read (iteration 1) ──── register = 0 → loop
 │           │◄──── read (iteration 2) ──── register = 0 → loop
 │           │
 │   = 1     │◄──── INTERRUPT writes 1
 │           │
 │           │◄──── read (iteration 3) ──── register = 1 → EXIT!
 └──────────┘

 Every loop iteration re-reads from actual memory.
 The interrupt's write is visible immediately.
```

### 5.3 Callbacks in Interrupt Context: Be Careful

Interrupt handlers must be **fast**. Calling a callback inside an interrupt is risky because the callback might do slow work (printf, I/O, etc.).

```
 BAD: callback inside interrupt       GOOD: flag + main loop
 ─────────────────────────────       ──────────────────────────

 ┌──────────────┐                    ┌──────────────┐
 │  Interrupt   │                    │  Interrupt   │
 │              │                    │              │
 │  callback()  │ ← might be slow   │  flag = 1;   │ ← fast!
 │  printf()    │   blocks other     │              │
 │  I/O ops     │   interrupts       └──────┬───────┘
 │              │                            │
 └──────────────┘                            ▼
                                     ┌──────────────┐
                                     │  Main Loop   │
                                     │              │
                                     │  if (flag) { │
                                     │   callback() │ ← safe here
                                     │   flag = 0;  │
                                     │  }           │
                                     └──────────────┘
```

```c
// BAD -- slow callback blocks interrupts
void sensor_interrupt_handler(void) {
    if (g_callbacks.on_temperature) {
        g_callbacks.on_temperature(id, value);  // risky!
    }
}

// GOOD -- set flag, handle in main loop
static volatile int    temp_ready = 0;
static volatile float  temp_value = 0;
static volatile int    temp_sensor_id = 0;

void sensor_interrupt_handler(void) {
    temp_value     = read_sensor_value();
    temp_sensor_id = get_sensor_id();
    temp_ready     = 1;                         // just set the flag
}

void main_loop(void) {
    if (temp_ready) {
        temp_ready = 0;
        if (g_callbacks.on_temperature) {
            g_callbacks.on_temperature(temp_sensor_id, temp_value);
        }
    }
}
```

---

## 6. Advanced: Passing User Data to Callbacks

### 6.1 The Problem

Your callback signature is fixed by the library. But sometimes you need **extra context** that isn't in the parameters.

```c
void my_temp_handler(int sensor_id, float temperature) {
    // I want to know WHICH ROOM this sensor is in...
    // but there's no room_name parameter!
}
```

### 6.2 The Solution: `void *user_data`

Add a generic pointer to the callback signature. The caller stores whatever extra data they want, and the callback casts it back.

```c
// Extended callback type with user_data
typedef void (*TempCallbackEx)(int sensor_id, float temperature,
                                void *user_data);
```

```
 How void* user_data works:

 ┌──── main.c ──────────────────────────────────────────────┐
 │                                                           │
 │  RoomInfo living_room = { .name = "Living Room",          │
 │                           .floor = 1 };                   │
 │                                                           │
 │  register(my_handler, &living_room);                      │
 │                  │            │                            │
 │                  │            └── void* user_data          │
 │                  └── callback function                     │
 └──────────────────┬────────────┬───────────────────────────┘
                    │            │
                    ▼            ▼
 ┌──── sensor_manager.c ────────────────────────────────────┐
 │                                                           │
 │  stores: callback = my_handler                            │
 │          data     = &living_room (as void*)               │
 │                                                           │
 │  on event:                                                │
 │    callback(sensor_id, temp, data);                       │
 │         │                    │                             │
 └─────────┼────────────────────┼────────────────────────────┘
           │                    │
           ▼                    ▼
 ┌──── my_handler() ────────────────────────────────────────┐
 │                                                           │
 │  void my_handler(int id, float temp, void *user_data) {   │
 │      RoomInfo *room = (RoomInfo *)user_data;              │
 │      //                  ▲                                │
 │      //                  cast void* back to original type │
 │      printf("%s: %.1f C\n", room->name, temp);            │
 │  }                                                        │
 │                                                           │
 │  Output: "Living Room: 25.5 C"                            │
 └───────────────────────────────────────────────────────────┘
```

```c
typedef struct {
    char name[32];
    int floor;
} RoomInfo;

RoomInfo living_room = { .name = "Living Room", .floor = 1 };

void my_handler(int sensor_id, float temp, void *user_data) {
    RoomInfo *room = (RoomInfo *)user_data;  // cast back
    printf("[%s, Floor %d] sensor %d: %.1f C\n",
           room->name, room->floor, sensor_id, temp);
}

// Register with user data
register_callback_ex(my_handler, &living_room);
```

### 6.3 Why `void*`?

```
 void* = "pointer to anything"

 ┌──────────┐     ┌──────────┐     ┌──────────┐
 │ RoomInfo  │     │ Config   │     │ Logger   │
 │ struct    │     │ struct   │     │ struct   │
 └─────┬────┘     └─────┬────┘     └─────┬────┘
       │                │                │
       └───────┬────────┴────────────────┘
               │
               ▼
          ┌─────────┐
          │  void*  │   accepts ANY pointer type
          └─────────┘

 The library doesn't need to know your data type.
 You cast it back inside your callback.
```

---

## 7. Summary

| Concept | What It Does | When to Use |
|---|---|---|
| **Function pointer** | Stores address of a function | When behavior must be decided at runtime |
| **`typedef`** | Names a function pointer type | Always -- makes code readable |
| **Callback struct** | Bundles related callbacks | When a module has multiple event types |
| **`g_callbacks = *cb`** | Copies struct by value | Safe: original can be destroyed after |
| **NULL check** | `if (cb.on_temp)` before call | Always -- callback may not be registered |
| **`volatile`** | Forces memory re-read | Variables shared with interrupts |
| **Flag pattern** | Set flag in ISR, handle in main | Keep interrupt handlers fast |
| **`void *user_data`** | Pass extra context to callbacks | When callbacks need app-specific data |
