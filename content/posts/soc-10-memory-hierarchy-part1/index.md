---
title: "[SoC-10] Memory Hierarchy Part 1: Understanding Caches"
date: 2026-02-25
description: "A thorough exploration of the hierarchical memory system — why it exists, how cache memory works, and the principles of locality that make it effective."
categories: ["SoC Design"]
tags: ["SoC", "Cache", "Memory Hierarchy", "SRAM", "DRAM", "Locality", "Computer Architecture"]
series: ["SoC Design Course"]
series_order: 10
draft: false
---

{{< katex >}}

## Introduction

We've built a pipelined processor that can execute one instruction per cycle. But there's a hidden bottleneck we've been ignoring: **memory speed**.

Our pipeline assumes memory access takes one cycle. In reality, main memory (DRAM) takes **50–100 ns** — that's 100–200 clock cycles at 2 GHz! If every load and store stalled for 100 cycles, our pipeline would be useless.

The solution is the **memory hierarchy** — a system of progressively larger, slower, and cheaper memories that creates the illusion of a large, fast memory.

---

## 1. The Memory Wall

### 1.1 The Speed Gap

Over the decades, processor speed has improved much faster than memory speed:

| Year | CPU Speed Improvement | DRAM Speed Improvement |
|------|:---------------------:|:----------------------:|
| 1980–2000 | ~1000× | ~10× |
| 2000–2020 | ~10× (multi-core) | ~4× |

This growing gap is called the **memory wall**:

```
Performance
    ▲
    │     CPU
    │    ╱
    │   ╱
    │  ╱      ← Growing gap = "Memory Wall"
    │ ╱
    │╱  ___────── Memory
    │──
    └────────────────────► Year
    1980   1990   2000   2010   2020
```

### 1.2 Memory Technology Comparison

| Technology | Capacity | Access Time | Cost ($/GB) | Use |
|-----------|----------|-------------|-------------|-----|
| **SRAM** | KB–MB | 0.5–2 ns | ~$500 | Cache |
| **DRAM** | GB | 50–100 ns | ~$5 | Main memory |
| **Flash/SSD** | TB | 25–100 μs | ~$0.10 | Storage |
| **HDD** | TB | 5–10 ms | ~$0.02 | Archival |

SRAM is ~50× faster than DRAM but ~100× more expensive. We can't afford to make all memory from SRAM, but we can't tolerate DRAM speeds. The memory hierarchy solves this dilemma.

---

## 2. The Memory Hierarchy

### 2.1 Structure

```
          ┌─────────┐
          │ Register│  32 × 32-bit = 128 B
          │  File   │  ~0.3 ns
          └────┬────┘
               │
          ┌────┴────┐
          │ L1 Cache│  32–64 KB
          │ (SRAM)  │  ~1–2 ns
          └────┬────┘
               │
          ┌────┴────┐
          │ L2 Cache│  256 KB – 1 MB
          │ (SRAM)  │  ~3–10 ns
          └────┬────┘
               │
          ┌────┴────┐
          │ L3 Cache│  2–32 MB
          │ (SRAM)  │  ~10–30 ns
          └────┬────┘
               │
          ┌────┴────┐
          │  Main   │  4–64 GB
          │ Memory  │  ~50–100 ns
          │ (DRAM)  │
          └────┬────┘
               │
          ┌────┴────┐
          │  Disk   │  256 GB – 4 TB
          │(SSD/HDD)│  ~100 μs – 10 ms
          └─────────┘

  Faster ↑                              ↓ Slower
  Smaller ↑                             ↓ Larger
  More Expensive ↑                      ↓ Cheaper
```

### 2.2 Why Does This Work? — The Principle of Locality

The memory hierarchy works because programs don't access memory randomly. They exhibit two forms of **locality**:

**Temporal Locality:** If you accessed an address recently, you are likely to access it again soon.

- Loop variables, function return addresses, frequently used variables
- Example: `for (i = 0; i < 1000; i++)` — the variable `i` is accessed 1000 times

**Spatial Locality:** If you accessed an address, you are likely to access nearby addresses soon.

- Array traversals, sequential instruction execution, struct fields
- Example: `for (i = 0; i < n; i++) sum += a[i];` — accesses consecutive elements

```
Locality in action:

Code:   for (int i = 0; i < n; i++)
            sum += a[i];

Memory access pattern:
┌────┬────┬────┬────┬────┬────┬────┬────┐
│a[0]│a[1]│a[2]│a[3]│a[4]│a[5]│a[6]│a[7]│  ← Spatial locality
└──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┴──┬─┘
   │    │    │    │    │    │    │    │
   t0   t1   t2   t3   t4   t5   t6   t7     ← Sequential in time

Variable 'sum':  accessed at t0, t1, t2, ... tN  ← Temporal locality
Variable 'i':    accessed at t0, t1, t2, ... tN  ← Temporal locality
```

The cache exploits these patterns: keep recently and nearby accessed data in fast SRAM. Most accesses will **hit** in the cache, achieving near-SRAM speed with near-DRAM capacity.

---

## 3. Cache Basics

### 3.1 Terminology

| Term | Definition |
|------|-----------|
| **Cache hit** | Requested data is found in the cache |
| **Cache miss** | Requested data is not in the cache → must fetch from slower memory |
| **Hit rate** | Fraction of accesses that are hits |
| **Miss rate** | Fraction of accesses that are misses = 1 - hit rate |
| **Hit time** | Time to access data on a cache hit |
| **Miss penalty** | Additional time to fetch data from slower memory on a miss |
| **Block (cache line)** | The unit of data transferred between cache levels (typically 32–64 bytes) |

### 3.2 Cache Operation

```
CPU Request: Load address 0x1000

Step 1: Check L1 cache
        ├── HIT  → Return data in ~1 cycle
        └── MISS → Go to Step 2

Step 2: Check L2 cache
        ├── HIT  → Return data in ~5 cycles; update L1
        └── MISS → Go to Step 3

Step 3: Check L3 cache
        ├── HIT  → Return data in ~20 cycles; update L2, L1
        └── MISS → Go to Step 4

Step 4: Access main memory
        Return data in ~100 cycles; update L3, L2, L1
```

---

## 4. Cache Organization

The fundamental question: given an address, **how do we find** the corresponding data in the cache?

### 4.1 Address Decomposition

A memory address is split into fields that determine where to look in the cache:

```
          Tag          Index       Block Offset
┌──────────────────┬───────────┬────────────────┐
│                  │           │                │
└──────────────────┴───────────┴────────────────┘
MSB                                           LSB
```

| Field | Purpose |
|-------|---------|
| **Block Offset** | Which byte within the cache block |
| **Index** | Which cache set to look in |
| **Tag** | Identifies which memory block is stored here |

### 4.2 Direct-Mapped Cache

The simplest organization: each memory block maps to **exactly one** cache location.

$$
\text{Cache Index} = \text{Block Address} \mod \text{Number of Cache Blocks}
$$

```
Example: 8-block cache, 4 bytes per block

Memory Block    →    Cache Index
    0           →        0
    1           →        1
    2           →        2
    ...
    7           →        7
    8           →        0  (wraps around)
    9           →        1
    ...

Cache Structure:
Index   Valid   Tag     Data (4 bytes)
  0       1    0x05    [byte0][byte1][byte2][byte3]
  1       0    ----    [----][----][----][----]
  2       1    0x12    [byte0][byte1][byte2][byte3]
  3       1    0x00    [byte0][byte1][byte2][byte3]
  4       0    ----    [----][----][----][----]
  5       1    0x03    [byte0][byte1][byte2][byte3]
  6       1    0x07    [byte0][byte1][byte2][byte3]
  7       0    ----    [----][----][----][----]
```

**Hit check:**
1. Use **Index** bits to select a cache entry
2. Compare the stored **Tag** with the tag from the address
3. Check the **Valid** bit
4. **Hit** if valid AND tags match

**Pros:** Simple, fast (only one entry to check)

**Cons:** **Conflict misses** — if two frequently accessed blocks map to the same index, they keep evicting each other.

### 4.3 Fully Associative Cache

Any memory block can go in **any** cache location. No index field needed.

```
Cache (4 entries):
Entry   Valid   Tag        Data
  0       1    0x00A0     [...]
  1       1    0x0150     [...]
  2       1    0x0080     [...]
  3       0    ------     [...]
```

**Hit check:** Compare the tag against **every** cache entry simultaneously (requires N comparators for N entries).

**Pros:** No conflict misses — maximum flexibility in placement

**Cons:** Expensive hardware (many comparators), slow for large caches

### 4.4 Set-Associative Cache

A compromise: the cache is divided into **sets**, each containing **N ways** (N entries). A block maps to a specific set but can go in any way within that set.

```
2-way set-associative cache (8 entries = 4 sets × 2 ways):

         Way 0                    Way 1
Set  Valid  Tag   Data      Valid  Tag   Data
 0     1   0x05  [...]       1   0x09  [...]
 1     0   ----  [...]       1   0x03  [...]
 2     1   0x12  [...]       0   ----  [...]
 3     1   0x00  [...]       1   0x08  [...]
```

$$
\text{Set Index} = \text{Block Address} \mod \text{Number of Sets}
$$

**Hit check:**
1. Use Index to select a **set**
2. Compare tag against **all N ways** in that set simultaneously
3. Hit if any way has matching tag and valid bit

| Associativity | Sets | Ways per Set | Comparators | Conflict Misses |
|:-------------:|:----:|:------------:|:-----------:|:---------------:|
| Direct-mapped | N | 1 | 1 | High |
| 2-way | N/2 | 2 | 2 | Medium |
| 4-way | N/4 | 4 | 4 | Low |
| 8-way | N/8 | 8 | 8 | Very Low |
| Fully assoc. | 1 | N | N | None |

**Common choices:**
- L1 cache: 2-way or 4-way (speed is critical)
- L2 cache: 8-way (balance of hit rate and speed)
- L3 cache: 16-way or more (hit rate is critical)

---

## 5. Cache Address Example

**Given:** 32-bit addresses, 4 KB direct-mapped cache, 16 bytes per block.

**Calculate the address fields:**

$$
\text{Number of blocks} = \frac{4096}{16} = 256 \text{ blocks}
$$

| Field | Bits | Calculation |
|-------|:----:|-------------|
| Block Offset | 4 | $\log_2(16) = 4$ |
| Index | 8 | $\log_2(256) = 8$ |
| Tag | 20 | $32 - 8 - 4 = 20$ |

```
Address: 0x12345678

Binary: 0001 0010 0011 0100 0101 0110 0111 1000

Tag (20 bits):    0001 0010 0011 0100 0101 = 0x12345
Index (8 bits):   0110 0111 = 0x67 = 103
Offset (4 bits):  1000 = 0x8 = 8
```

So address 0x12345678 maps to cache **block 103**, byte **8** within the block, with tag **0x12345**.

---

## 6. Handling Cache Misses

### 6.1 Read Miss

When the processor reads an address that isn't in the cache:

```
1. Stall the CPU pipeline
2. Send address to next level memory (L2 or main memory)
3. Wait for data to arrive (miss penalty)
4. Write the entire block into the cache
5. Restart the stalled instruction
```

### 6.2 Write Policies

When the processor **writes** data, two strategies exist:

**Write-Through:**

```
CPU Write → Update Cache AND Update Memory simultaneously
```

- Simple to implement
- Memory always has the latest data
- Generates lots of memory traffic (every write goes to memory)
- Often uses a **write buffer** to hide the memory write latency

**Write-Back:**

```
CPU Write → Update Cache ONLY; mark block as "dirty"
Eviction → IF dirty, THEN write block back to memory
```

- Fewer memory writes (only when a dirty block is evicted)
- More complex (need dirty bit per block)
- Memory may have stale data (consistency challenge for multi-core)
- Most common in modern processors

### 6.3 Write Miss Policies

What happens when we write to an address not in the cache?

| Policy | Action |
|--------|--------|
| **Write-Allocate** | Fetch the block into cache, then write. Common with write-back. |
| **Write-No-Allocate** | Write directly to memory, don't put in cache. Common with write-through. |

---

## 7. Replacement Policies

When a cache set is full and a new block must be brought in, which existing block do we evict?

| Policy | How It Works | Quality | Cost |
|--------|-------------|:-------:|:----:|
| **Random** | Evict a random block | OK | Low |
| **FIFO** | Evict the oldest block | OK | Low |
| **LRU** | Evict the Least Recently Used block | Best | High |
| **Pseudo-LRU** | Approximate LRU with tree structure | Near-best | Medium |

**LRU (Least Recently Used)** is optimal for exploiting temporal locality — the block you haven't used in the longest time is the least likely to be needed soon.

For a 2-way cache, LRU needs just 1 bit per set (tracking which way was used more recently).

For a 4-way cache, LRU needs 6 bits per set (tracking the full access order of 4 ways).

For 8-way or higher, **pseudo-LRU** is used because true LRU is too expensive.

---

## 8. Types of Cache Misses (The Three C's)

| Miss Type | Cause | Solution |
|-----------|-------|----------|
| **Compulsory** (Cold) | First access to a block — it was never in cache | Prefetching |
| **Capacity** | Cache is too small to hold all active blocks | Increase cache size |
| **Conflict** | Multiple blocks map to the same set and evict each other | Increase associativity |

```
Miss breakdown (typical):
┌──────────────────────────────────────┐
│ Compulsory:  ~5%                     │
│ Capacity:    ~30%                    │
│ Conflict:    ~65%  (in direct-mapped)│
│              ~25%  (in 4-way)        │
└──────────────────────────────────────┘
```

Higher associativity reduces conflict misses significantly.

---

## 9. Cache in the Pipeline

How does the cache fit into our 5-stage RISC-V pipeline?

```
[IF] ──► L1 I-Cache (instruction fetch)
         │
         └── Miss? → Stall pipeline, fetch from L2/L3/Memory

[MEM] ──► L1 D-Cache (data load/store)
          │
          └── Miss? → Stall pipeline, fetch from L2/L3/Memory
```

Modern processors have **separate L1 caches** for instructions (I-cache) and data (D-cache). This eliminates structural hazards between the IF and MEM stages.

**Typical L1 cache parameters:**

| Parameter | I-Cache | D-Cache |
|-----------|:-------:|:-------:|
| Size | 32–64 KB | 32–64 KB |
| Associativity | 4–8 way | 4–8 way |
| Block size | 64 bytes | 64 bytes |
| Hit time | 1–2 cycles | 1–2 cycles |
| Miss rate | 1–3% | 5–10% |

---

## 10. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Memory wall** | CPU speed grew much faster than memory speed |
| **Locality** | Programs access nearby and recently used data — this is what caches exploit |
| **Cache hit/miss** | Hit = data in cache (fast); Miss = must fetch from slower memory |
| **Direct-mapped** | Simple, fast, but high conflict miss rate |
| **Set-associative** | Compromise between direct-mapped and fully associative |
| **Write-back** | Write only to cache; write to memory on eviction (most common) |
| **LRU** | Best replacement policy — evict least recently used block |
| **Three C's** | Compulsory, Capacity, Conflict — three causes of cache misses |

In the **next post ([SoC-11])**, we will study how cache performance is measured and what optimization techniques can be applied to minimize miss rates and improve overall system performance.

---

*This post is part of the **SoC Design Course** series. Navigate to the next post to continue your learning journey.*
