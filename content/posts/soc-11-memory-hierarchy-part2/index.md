---
title: "[SoC-11] Memory Hierarchy Part 2: Cache Performance and Optimization"
date: 2026-02-25
description: "Learning how to measure cache performance using AMAT, and exploring hardware and software techniques to optimize cache behavior — including prefetching, multi-level caches, and virtual memory."
categories: ["SoC Design"]
tags: ["SoC", "Cache Performance", "AMAT", "Prefetching", "Virtual Memory", "TLB", "Memory Optimization"]
series: ["SoC Design Course"]
series_order: 11
draft: false
---

{{< katex >}}

## Introduction

In [SoC-10], we learned how caches work — their structure, addressing, and replacement policies. Now let's focus on **performance**: how to measure it, what affects it, and how to make it better.

---

## 1. Measuring Cache Performance

### 1.1 Average Memory Access Time (AMAT)

The most important metric for memory system performance:

$$
\text{AMAT} = \text{Hit Time} + \text{Miss Rate} \times \text{Miss Penalty}
$$

**Example:**
- L1 hit time = 1 cycle
- L1 miss rate = 5%
- Miss penalty (time to fetch from L2) = 10 cycles

$$
\text{AMAT} = 1 + 0.05 \times 10 = 1.5 \text{ cycles}
$$

### 1.2 Multi-Level AMAT

With multiple cache levels, the formula becomes recursive:

$$
\text{AMAT} = \text{Hit Time}_{L1} + \text{Miss Rate}_{L1} \times (\text{Hit Time}_{L2} + \text{Miss Rate}_{L2} \times \text{Miss Penalty}_{L2})
$$

**Example with three levels:**

| Level | Hit Time | Miss Rate |
|-------|:--------:|:---------:|
| L1 | 1 cycle | 5% |
| L2 | 10 cycles | 20% |
| L3 (or Memory) | 100 cycles | — |

$$
\text{AMAT} = 1 + 0.05 \times (10 + 0.20 \times 100)
$$

$$
= 1 + 0.05 \times 30 = 1 + 1.5 = 2.5 \text{ cycles}
$$

Without any cache: AMAT = 100 cycles. With the hierarchy: AMAT = 2.5 cycles. That's a **40× improvement**!

### 1.3 Local vs. Global Miss Rate

| Metric | Definition | Example |
|--------|-----------|---------|
| **Local miss rate** | Misses at this level / accesses to this level | L2 local miss rate = 20% |
| **Global miss rate** | Misses at this level / total CPU memory accesses | L2 global miss rate = 5% × 20% = 1% |

Global miss rate is more meaningful for overall performance analysis.

### 1.4 Impact on CPI

$$
\text{CPI}_{total} = \text{CPI}_{ideal} + \text{Memory Stall Cycles per Instruction}
$$

$$
\text{Memory Stalls} = \frac{\text{Memory Accesses}}{\text{Instruction}} \times \text{Miss Rate} \times \text{Miss Penalty}
$$

**Example:**
- CPI_ideal = 1.0
- 30% of instructions are loads/stores
- L1 D-cache miss rate = 5%
- Miss penalty = 100 cycles (to main memory)

For data accesses:
$$
\text{Data Stalls} = 0.30 \times 0.05 \times 100 = 1.5 \text{ cycles/instruction}
$$

For instruction accesses (assume I-cache miss rate = 1%):
$$
\text{Instr Stalls} = 1.0 \times 0.01 \times 100 = 1.0 \text{ cycles/instruction}
$$

$$
\text{CPI}_{total} = 1.0 + 1.5 + 1.0 = 3.5
$$

The ideal CPI of 1.0 becomes 3.5 due to memory stalls — memory is the bottleneck, not the pipeline!

---

## 2. Cache Optimization Techniques

We can reduce AMAT by improving any of its three components:

$$
\text{AMAT} = \underbrace{\text{Hit Time}}_{\text{reduce}} + \underbrace{\text{Miss Rate}}_{\text{reduce}} \times \underbrace{\text{Miss Penalty}}_{\text{reduce}}
$$

### 2.1 Reducing Miss Rate

#### Increase Block Size

Larger blocks exploit **spatial locality** more aggressively — when you fetch a 64-byte block on a miss, you get 64 nearby bytes for free.

| Block Size | Compulsory Misses | Capacity Misses | Conflict Misses |
|:----------:|:-----------------:|:---------------:|:---------------:|
| 16 B | High | Low | Low |
| 32 B | Medium | Medium | Medium |
| 64 B | Low | Medium | Medium |
| 128 B | Very Low | High | High |

**Trade-off:** Very large blocks waste bandwidth (most of the fetched data may not be used) and reduce the number of blocks in the cache (increasing capacity misses).

**Sweet spot:** 32–64 bytes is optimal for most workloads.

#### Increase Associativity

Higher associativity reduces **conflict misses**:

```
Miss rate vs. associativity (typical):

Direct-mapped:  10%
2-way:           7%   (-30%)
4-way:           6%   (-15%)
8-way:           5.5% (-8%)
Fully assoc.:    5%   (-10%)
```

**Diminishing returns:** Going from direct-mapped to 2-way gives the biggest improvement. Beyond 8-way, the benefit is minimal.

**Rule of thumb (2:1 rule):** A direct-mapped cache of size $N$ has roughly the same miss rate as a 2-way set-associative cache of size $N/2$.

#### Increase Cache Size

More capacity means fewer capacity misses. But larger caches are:
- Slower (longer wire delays)
- More expensive (more SRAM)
- More power-hungry

This is why we use **multiple levels** — a small, fast L1 and a large, slower L2/L3.

### 2.2 Reducing Miss Penalty

#### Multi-Level Caches

Adding an L2 cache between L1 and main memory dramatically reduces the effective miss penalty:

```
Without L2:  Miss penalty = 100 cycles (go to DRAM)
With L2:     Miss penalty = 10 cycles (90% of L1 misses hit in L2)
                          + 0.10 × 100 = 10 + 10 = 20 cycles (effective)
```

#### Critical Word First

When fetching a cache block on a miss, the **requested word** is sent to the CPU first, before the rest of the block arrives:

```
Normal:   Fetch entire 64-byte block → Send to CPU → Resume
Critical: Fetch requested word → Send to CPU → Resume
          (remaining block fills in background)
```

This reduces the effective miss penalty by allowing the CPU to restart sooner.

#### Write Buffers

A **write buffer** stores pending writes, allowing the CPU to continue without waiting for the write to reach memory:

```
CPU ──► [Write Buffer] ──► Memory
         (4–8 entries)

CPU can continue immediately after writing to buffer.
Buffer drains to memory in background.
```

### 2.3 Reducing Hit Time

#### Small and Simple L1 Cache

Keep the L1 cache small (32–64 KB) and low-associativity (2–4 way) for the fastest hit time. Sacrifice miss rate for speed — misses are handled by L2.

#### Pipeline the Cache

For higher clock frequencies, the cache access can be split across multiple pipeline stages:

```
Standard:  [IF: I-Cache access in 1 cycle]
Pipelined: [IF1: Tag check] [IF2: Data read]  (2 cycles, but at higher clock)
```

#### Virtually-Indexed, Physically-Tagged (VIPT)

Use virtual address bits for the index (fast, no TLB lookup needed) but physical address bits for the tag (correct, avoids aliasing). This allows cache access to begin before the TLB translation is complete.

---

## 3. Prefetching

### 3.1 The Idea

Instead of waiting for a miss, **predict** which blocks will be needed and fetch them into the cache **before** the CPU requests them.

### 3.2 Hardware Prefetching

**Sequential prefetching:** When block $N$ is accessed, automatically prefetch block $N+1$ (or $N+1, N+2, ...$).

```
CPU accesses block 100
  → Prefetcher automatically fetches block 101
CPU accesses block 101  ← HIT (prefetched!)
  → Prefetcher fetches block 102
...
```

Great for sequential access patterns (array traversals, instruction fetch).

**Stride prefetching:** Detects regular access patterns (e.g., every 4th element):

```
Access pattern: 0, 4, 8, 12, 16, ...
Stride = 4
Prefetcher predicts: next access = current + stride
```

### 3.3 Software Prefetching

The compiler or programmer inserts explicit prefetch instructions:

```c
for (int i = 0; i < n; i++) {
    __builtin_prefetch(&a[i + 8]);  // Prefetch 8 elements ahead
    sum += a[i];
}
```

The prefetch instruction is a **hint** — it doesn't stall the pipeline if the data isn't ready, and it doesn't cause an exception if the address is invalid.

### 3.4 Prefetching Trade-offs

| Benefit | Risk |
|---------|------|
| Eliminates compulsory misses | Pollutes cache with unneeded data |
| Hides memory latency | Wastes memory bandwidth |
| Improves throughput | Too aggressive prefetching can hurt |

---

## 4. Virtual Memory

### 4.1 The Problem

Physical memory (DRAM) is limited. Multiple programs need to share it. And programmers don't want to worry about where their data physically resides.

### 4.2 Virtual Memory Concept

Each program sees its own **virtual address space**. The OS and hardware collaborate to translate virtual addresses to physical addresses:

```
Program A sees:          Program B sees:
0x0000 ─ 0xFFFF         0x0000 ─ 0xFFFF
(its own 64KB space)     (its own 64KB space)

                  ┌─────────────┐
Virtual ──────────│  Page Table  │──────────► Physical
Address           │  (mapping)   │            Address
                  └─────────────┘

Program A: VA 0x1000 → PA 0x5000
Program B: VA 0x1000 → PA 0x8000  (different physical location!)
```

### 4.3 Pages

Memory is divided into fixed-size **pages** (typically 4 KB):

```
Virtual Address Space              Physical Memory
┌──────────────────┐              ┌──────────────────┐
│ Virtual Page 0   │─────────────►│ Physical Page 3  │
│ Virtual Page 1   │──────┐      │ Physical Page 0  │
│ Virtual Page 2   │──┐   └─────►│ Physical Page 5  │
│ Virtual Page 3   │  └────────►│ Physical Page 1  │
│ ...              │              │ ...              │
│ Virtual Page N   │──────────►│ Physical Page 7  │
└──────────────────┘              │ Physical Page 2  │ (free)
                                  │ Physical Page 4  │ (free)
                                  │ Physical Page 6  │ (other program)
                                  └──────────────────┘
```

### 4.4 Page Table

The **page table** stores the mapping from virtual page numbers to physical page numbers:

```
Virtual Address (32-bit, 4KB pages):
┌──────────────────────┬──────────────┐
│ Virtual Page Number  │ Page Offset  │
│      (20 bits)       │  (12 bits)   │
└──────────┬───────────┴──────┬───────┘
           │                  │
           ▼                  │
    ┌─────────────┐           │
    │ Page Table  │           │
    │ Entry:      │           │
    │ VPN → PPN   │           │
    │ + Valid bit  │          │
    │ + Dirty bit  │          │
    │ + Access bits│          │
    └──────┬──────┘           │
           │                  │
           ▼                  │
┌──────────────────────┬──────┴───────┐
│Physical Page Number  │ Page Offset  │
│      (20 bits)       │  (12 bits)   │
└──────────────────────┴──────────────┘
Physical Address
```

### 4.5 Translation Lookaside Buffer (TLB)

The page table resides in **main memory** — accessing it for every memory reference would double the access time! The **TLB** is a small, fast cache of recent page table entries:

```
Virtual Address ──► [TLB Lookup]
                      │
                   TLB Hit? ──► Physical Address (fast, ~1 cycle)
                      │
                   TLB Miss? ──► Page Table Walk (slow, ~100 cycles)
                                  │
                                  └► Update TLB with new mapping
```

**Typical TLB parameters:**

| Parameter | Value |
|-----------|-------|
| Entries | 32–512 |
| Associativity | Fully associative or high (8–16 way) |
| Hit time | 0.5–1 cycle |
| Miss penalty | ~10–100 cycles (page table walk) |
| Miss rate | < 1% (very high hit rate) |

### 4.6 Page Fault

When the accessed page is not in physical memory (it's on disk):

```
CPU access → TLB miss → Page Table → Valid bit = 0 → PAGE FAULT
                                                        │
                                              OS takes over:
                                              1. Find the page on disk
                                              2. Find a free physical page
                                                 (or evict one, write to disk if dirty)
                                              3. Load page from disk → physical memory
                                              4. Update page table
                                              5. Restart the instruction
```

Page faults are **extremely expensive** (~10 ms for disk access = millions of CPU cycles). This is why:
- Pages are large (4 KB, sometimes 2 MB "huge pages")
- Replacement is always LRU (can't afford random with such high penalty)
- Write-back is always used (can't write through to disk on every write)

---

## 5. Cache and Virtual Memory Integration

### 5.1 Address Translation and Cache Access

The cache can be indexed using either virtual or physical addresses:

| Configuration | Index | Tag | Pros | Cons |
|:-------------:|:-----:|:---:|------|------|
| PIPT | Physical | Physical | No aliasing | Slow (must translate before access) |
| VIVT | Virtual | Virtual | Fast | Aliasing, flush on context switch |
| VIPT | Virtual | Physical | Fast + no aliasing* | Constraints on cache size |

**VIPT (Virtually Indexed, Physically Tagged)** is the most common for L1 caches because it allows the TLB lookup and cache index to happen **in parallel**:

```
Virtual Address
      │
      ├── [Index bits] ──► Cache Set Lookup  ─┐
      │                                        ├──► Compare → Hit/Miss
      └── [VPN bits] ──► TLB ──► PPN ──► Tag ─┘
                      (in parallel!)
```

*This works when the index bits fall entirely within the page offset (which is the same for virtual and physical addresses).

### 5.2 Putting It All Together: Complete Memory Access

```
CPU generates Virtual Address
         │
    ┌────┴────┐
    │   TLB   │──── TLB Hit ──► Physical Address
    └────┬────┘                        │
         │                       ┌─────┴─────┐
    TLB Miss                     │  L1 Cache  │── Hit ──► Data (1-2 cycles)
         │                       └─────┬─────┘
    Page Table Walk                    │
    (10-100 cycles)              L1 Miss
         │                             │
    Page Fault?                  ┌─────┴─────┐
    (millions of cycles)         │  L2 Cache  │── Hit ──► Data (5-10 cycles)
                                 └─────┬─────┘
                                       │
                                 L2 Miss
                                       │
                                 ┌─────┴─────┐
                                 │  L3 Cache  │── Hit ──► Data (10-30 cycles)
                                 └─────┬─────┘
                                       │
                                 L3 Miss
                                       │
                                 ┌─────┴─────┐
                                 │Main Memory │──► Data (50-100 cycles)
                                 └────────────┘
```

---

## 6. Software Optimization for Cache

Programmers can significantly impact cache performance through code structure:

### 6.1 Loop Interchange

```c
// Bad: stride-N access (poor spatial locality)
for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++)
        sum += A[i][j];  // Jumps by N elements each access

// Good: stride-1 access (excellent spatial locality)
for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++)
        sum += A[i][j];  // Sequential access
```

For a row-major language (C/C++), iterating over the **inner dimension last** gives sequential memory access and maximum cache utilization.

### 6.2 Loop Blocking (Tiling)

For matrix multiplication, process small blocks that fit in cache:

```c
// Naive (poor cache use for large matrices)
for (i = 0; i < N; i++)
    for (j = 0; j < N; j++)
        for (k = 0; k < N; k++)
            C[i][j] += A[i][k] * B[k][j];

// Blocked (excellent cache use)
for (ii = 0; ii < N; ii += BLOCK)
    for (jj = 0; jj < N; jj += BLOCK)
        for (kk = 0; kk < N; kk += BLOCK)
            for (i = ii; i < ii+BLOCK; i++)
                for (j = jj; j < jj+BLOCK; j++)
                    for (k = kk; k < kk+BLOCK; k++)
                        C[i][j] += A[i][k] * B[k][j];
```

Choose BLOCK size so that three BLOCK×BLOCK sub-matrices fit in L1 cache.

### 6.3 Data Structure Layout

**Array of Structures (AoS)** vs. **Structure of Arrays (SoA):**

```c
// AoS: poor spatial locality when accessing one field across all elements
struct Particle { float x, y, z, mass; } particles[N];
for (i = 0; i < N; i++) sum += particles[i].x;  // Stride = 16 bytes

// SoA: excellent spatial locality for per-field access
struct Particles { float x[N], y[N], z[N], mass[N]; } p;
for (i = 0; i < N; i++) sum += p.x[i];  // Stride = 4 bytes (sequential)
```

SoA is often 2–4× faster for data-parallel access patterns (common in graphics and AI).

---

## 7. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **AMAT** | Hit Time + Miss Rate × Miss Penalty — the key metric |
| **Multi-level caches** | Dramatically reduce effective miss penalty |
| **Larger blocks** | Reduce compulsory misses but increase miss penalty |
| **Higher associativity** | Reduce conflict misses; 2:1 rule for estimation |
| **Prefetching** | Hide latency by fetching data before it's needed |
| **Virtual memory** | Provides isolation and abstraction; pages + page tables |
| **TLB** | Caches page table entries; crucial for virtual memory performance |
| **Software optimization** | Loop interchange, blocking, SoA layout can be 2–10× faster |

In the **next post ([SoC-12])**, we transition from processor architecture to **embedded SoC software** — exploring real embedded SoC platforms and the ARM Cortex-M0+ processor core.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
