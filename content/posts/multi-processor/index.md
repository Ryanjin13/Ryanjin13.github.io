---
title: "Multi-Processor Systems"
date: 2024-06-25
description: "Understanding multi-processor architectures and parallel computing"
categories: ["Computer Science"]
tags: ["Computer Structure", "Multi-core", "Parallel Computing"]
draft: false
---

{{< katex >}}

## Overview

Multi-processor systems use multiple processing units to achieve higher performance through parallel execution. This approach became essential after single-core scaling hit the power wall.

## Types of Parallel Systems

### Flynn's Taxonomy

| Type | Description | Example |
|------|-------------|---------|
| SISD | Single Instruction, Single Data | Traditional uniprocessor |
| SIMD | Single Instruction, Multiple Data | GPU, vector processors |
| MISD | Multiple Instruction, Single Data | Rare (fault tolerance) |
| MIMD | Multiple Instruction, Multiple Data | Multi-core CPUs |

## Shared Memory Architecture

### Uniform Memory Access (UMA)

```
┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐
│CPU 0│ │CPU 1│ │CPU 2│ │CPU 3│
└──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘
   │       │       │       │
   └───────┴───┬───┴───────┘
               │
        ┌──────┴──────┐
        │  Shared Bus │
        └──────┬──────┘
               │
        ┌──────┴──────┐
        │   Memory    │
        └─────────────┘
```

All processors have equal access time to memory.

### Non-Uniform Memory Access (NUMA)

```
┌─────────────────┐     ┌─────────────────┐
│     Node 0      │     │     Node 1      │
│ ┌───┐   ┌───┐   │     │ ┌───┐   ┌───┐   │
│ │CPU│   │CPU│   │     │ │CPU│   │CPU│   │
│ └─┬─┘   └─┬─┘   │     │ └─┬─┘   └─┬─┘   │
│   └───┬───┘     │     │   └───┬───┘     │
│   ┌───┴───┐     │←───→│   ┌───┴───┐     │
│   │Local  │     │     │   │Local  │     │
│   │Memory │     │     │   │Memory │     │
│   └───────┘     │     │   └───────┘     │
└─────────────────┘     └─────────────────┘
```

Local memory: fast access
Remote memory: slower access

## Cache Coherence

### The Problem

Multiple caches may hold copies of same data:

```
CPU 0 Cache: X = 5
CPU 1 Cache: X = 5
Memory:      X = 5

CPU 0 writes X = 10
CPU 0 Cache: X = 10
CPU 1 Cache: X = 5  ← Stale!
Memory:      X = 5
```

### Coherence Protocols

**MSI Protocol States:**
- Modified (M): Exclusive, dirty
- Shared (S): Clean, may be in other caches
- Invalid (I): Not valid

**MESI Protocol (adds Exclusive):**
- Exclusive (E): Clean, only copy

### Snooping

Each cache monitors bus transactions:

```
CPU 0 writes X
  ↓
Bus broadcast: "Writing X"
  ↓
CPU 1 snoops, invalidates its copy
```

### Directory-Based

Central directory tracks which caches have each line:

```
Directory entry for X:
- Present in: CPU 0, CPU 2
- State: Shared

CPU 0 wants to write:
- Send invalidate to CPU 2
- Update directory
- Grant write permission
```

## Memory Consistency

### Sequential Consistency

All processors see same order of operations.

Most intuitive, but limits optimizations.

### Relaxed Consistency

Allow reordering for performance:
- Writes may be buffered
- Reads may bypass writes
- Memory barriers needed

## Synchronization

### Atomic Operations

```c
// Compare and Swap
int compare_and_swap(int *ptr, int old, int new) {
    atomic {
        if (*ptr == old) {
            *ptr = new;
            return 1;
        }
        return 0;
    }
}
```

### Lock Implementation

```c
void acquire_lock(int *lock) {
    while (!compare_and_swap(lock, 0, 1)) {
        // Spin or yield
    }
}

void release_lock(int *lock) {
    *lock = 0;
}
```

## Scalability

### Amdahl's Law for Parallel Systems

$$
\text{Speedup} = \frac{1}{(1-p) + \frac{p}{n}}
$$

Where:
- \\(p\\): Parallel fraction
- \\(n\\): Number of processors

### Gustafson's Law

With larger problems:

$$
\text{Speedup} = (1-p) + p \cdot n
$$

Linear scaling possible with scaled workloads.

## Multi-Core vs Multi-Processor

| Aspect | Multi-Core | Multi-Processor |
|--------|------------|-----------------|
| Location | Same chip | Separate chips |
| Cache sharing | Often L3 shared | Typically separate |
| Memory | Single controller | Multiple controllers |
| Communication | Fast on-chip | Slower off-chip |
| Cost | Lower | Higher |

## Performance Considerations

### Bottlenecks

1. **Memory bandwidth:** Limited shared resource
2. **Cache contention:** False sharing
3. **Synchronization:** Lock overhead
4. **Load imbalance:** Idle processors

### False Sharing

```c
// Bad: arr[0] and arr[1] likely same cache line
thread 0: writes arr[0]
thread 1: writes arr[1]
// Constant invalidation!

// Good: Pad to separate cache lines
struct padded {
    int value;
    char padding[60];  // 64-byte cache line
};
```

## Summary

| Concept | Key Point |
|---------|-----------|
| Shared memory | Common address space |
| Cache coherence | Keep caches consistent |
| Memory consistency | Define operation ordering |
| Synchronization | Coordinate access |
| Scalability | Amdahl limits parallel speedup |
