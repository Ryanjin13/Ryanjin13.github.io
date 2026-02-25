---
title: "Linux Virtual Memory: A Complete Deep Dive"
date: 2026-02-25
description: "A thorough exploration of how Linux manages virtual memory — covering page tables, address translation, demand paging, copy-on-write, memory mapping, swap, OOM killer, and practical tools for memory analysis."
categories: ["Linux"]
tags: ["Linux", "Virtual Memory", "Page Table", "MMU", "Swap", "mmap", "Operating System", "Memory Management"]
series: ["Linux Internals"]
series_order: 2
draft: false
---

{{< katex >}}

## Introduction

In the [Linux Architecture](/posts/linux-architecture/) post, we saw that each process has its own **virtual address space** and that the kernel uses page tables to translate virtual addresses to physical addresses. But we only scratched the surface.

Virtual memory is arguably the **most important abstraction** in a modern operating system. It provides:

1. **Isolation** — each process believes it has the entire memory to itself
2. **Protection** — one process cannot read or corrupt another's memory
3. **Flexibility** — programs can use more memory than physically available
4. **Efficiency** — only the pages actually being used consume physical RAM

In this post, we will go deep into every aspect of how Linux implements virtual memory.

---

## 1. The Big Picture: Virtual vs. Physical Memory

### 1.1 Why Virtual Memory?

Without virtual memory, every program would need to know exactly which physical addresses are free. Loading two programs at the same time would require careful coordination to prevent address conflicts. A buggy program could overwrite the kernel or other programs.

With virtual memory, every process sees a clean, independent address space starting from 0:

```
Process A sees:               Process B sees:
┌────────────────┐            ┌────────────────┐
│ 0xFFFF...      │            │ 0xFFFF...      │
│   Kernel       │            │   Kernel       │
│   (shared)     │            │   (shared)     │
├────────────────┤            ├────────────────┤
│   Stack        │            │   Stack        │
│                │            │                │
│   Heap         │            │   Heap         │
│   Data         │            │   Data         │
│   Text         │            │   Text         │
└────────────────┘            └────────────────┘
0x0000...                     0x0000...

Both believe they start at address 0, but they map to
completely different physical memory locations.
```

### 1.2 Address Translation

The **MMU (Memory Management Unit)**, a hardware component inside the CPU, translates every virtual address to a physical address before accessing memory:

```
CPU generates          MMU                    Physical
Virtual Address  ────► translates  ────►      Memory
   (VA)                via page              (PA)
                       tables
                         │
                    ┌────┴────┐
                    │  TLB    │  (cache of recent translations)
                    │ (fast)  │
                    └─────────┘
```

This translation happens on **every single memory access** — instruction fetches, data reads, data writes. The TLB (Translation Lookaside Buffer) caches recent translations so most lookups are nearly free (~1 cycle).

---

## 2. Pages and Page Tables

### 2.1 Pages: The Unit of Memory Management

Linux divides both virtual and physical memory into fixed-size chunks called **pages**:

| Page Size | Name | Use Case |
|:---------:|------|----------|
| 4 KB | Standard page | Default for most systems |
| 2 MB | Huge page | Databases, VMs, large data |
| 1 GB | Gigantic page | Very large memory workloads |

Why 4 KB? It's a good compromise between:
- **Smaller pages** → finer granularity, less wasted memory, but larger page tables
- **Larger pages** → fewer page table entries, faster TLB, but more internal fragmentation

### 2.2 Page Table Entry (PTE)

Each page table entry maps one virtual page to one physical frame and includes metadata:

```
x86_64 Page Table Entry (64 bits):
┌────────────────────────────────────────────────────────────┐
│ 63│62:52│51:12                    │11:9│ 8│ 7│ 6│ 5│ 4│ 3│ 2│ 1│ 0│
│NX │AvL  │ Physical Frame Number  │Avl │ G│PS│ D│ A│PCD│PWT│U/S│R/W│ P│
└────────────────────────────────────────────────────────────┘
```

| Bit | Name | Meaning |
|:---:|------|---------|
| P | Present | Page is in physical memory (1) or on disk (0) |
| R/W | Read/Write | Page is writable (1) or read-only (0) |
| U/S | User/Supervisor | Accessible from user space (1) or kernel only (0) |
| A | Accessed | Page has been read (set by hardware) |
| D | Dirty | Page has been written (set by hardware) |
| PS | Page Size | 0 = 4KB page, 1 = 2MB/1GB huge page |
| NX | No Execute | Page cannot be executed (security: prevents code injection) |

### 2.3 Multi-Level Page Tables

A flat page table for a 48-bit virtual address space with 4KB pages would require $2^{36}$ entries — that's **64 GB** per process! Obviously impractical.

Linux uses **multi-level page tables** where each level covers a portion of the virtual address:

```
x86_64: 4-Level Page Table (48-bit virtual address)

Virtual Address (48 bits used):
┌─────────┬─────────┬─────────┬─────────┬──────────────┐
│ PGD (9) │ PUD (9) │ PMD (9) │ PTE (9) │ Offset (12)  │
│ bits    │ bits    │ bits    │ bits    │ bits         │
│ 47:39   │ 38:30   │ 29:21   │ 20:12   │ 11:0         │
└────┬────┴────┬────┴────┬────┴────┬────┴──────────────┘
     │         │         │         │
     ▼         ▼         ▼         ▼
   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐
   │ PGD │─►│ PUD │─►│ PMD │─►│ PTE │─► Physical Frame + Offset
   │Table│  │Table│  │Table│  │Table│
   └─────┘  └─────┘  └─────┘  └─────┘
   512       512       512       512
   entries   entries   entries   entries

CR3 register points to the PGD for the current process.
```

**Why multi-level?** Most of the address space is unused. With multi-level tables, entire subtrees can simply not exist — saving enormous amounts of memory. A typical process might use only a few hundred page table pages instead of millions.

**5-Level Page Tables (Linux 4.14+):**

For systems needing more than 256 TB of virtual address space, Linux supports 5-level page tables (57-bit virtual addresses). Enabled by `CONFIG_X86_5LEVEL`.

### 2.4 TLB: Making Translation Fast

Walking 4 levels of page tables for every memory access would be devastatingly slow (4 extra memory accesses per real access). The **TLB** caches recent translations:

```
Virtual Address ──► TLB Lookup
                      │
                   ┌──┴──┐
                  Hit    Miss
                   │      │
                   ▼      ▼
           Physical    Page Table Walk
           Address    (4 memory accesses)
           (~1 cycle)     │
                          ▼
                     Update TLB
                          │
                          ▼
                    Physical Address
                    (~100+ cycles total)
```

| TLB Parameter | Typical Value |
|--------------|:-------------:|
| L1 DTLB entries | 64 |
| L1 ITLB entries | 128 |
| L2 TLB entries | 1,536 |
| Hit rate | > 99% |
| Miss penalty | ~20–100 cycles |

**TLB Flush:** When a process context switch occurs, the TLB entries for the old process become invalid. The CPU must flush (invalidate) them. This is one of the major costs of context switching. Linux uses **ASID (Address Space ID)** or **PCID (Process Context ID)** to tag TLB entries per process, avoiding full flushes.

---

## 3. Process Address Space in Detail

### 3.1 Virtual Memory Areas (VMAs)

The kernel tracks each process's memory layout using **Virtual Memory Areas** (VMAs) — contiguous ranges of virtual addresses with the same permissions and backing:

```
Process Virtual Address Space:

High ───────────────────────────────
         Kernel Space (shared)
─────────────────────────────────── 0x7FFF FFFF FFFF (user/kernel boundary)
         Stack VMA ↓
         [rwx] grow-down, anonymous

         (unmapped gap)

         Memory-mapped files
         [r--] file-backed (shared lib .text)
         [rw-] file-backed (shared lib .data)

         (unmapped gap)

         Heap VMA ↑
         [rw-] grow-up, anonymous

         BSS VMA
         [rw-] anonymous

         Data VMA
         [rw-] file-backed (executable .data)

         Text VMA
         [r-x] file-backed (executable .text)
Low  ─────────────────────────────── 0x0000 0000 0000
```

You can inspect a process's VMAs:

```bash
# View memory map of a process
cat /proc/<PID>/maps

# Example output:
# 5594b3a00000-5594b3a02000 r--p 00000000 08:01 12345  /usr/bin/bash
# 5594b3a02000-5594b3ad0000 r-xp 00002000 08:01 12345  /usr/bin/bash
# 5594b3ad0000-5594b3b0a000 r--p 000d0000 08:01 12345  /usr/bin/bash
# 5594b3b0b000-5594b3b0f000 rw-p 0010a000 08:01 12345  /usr/bin/bash
# 5594b4a00000-5594b4b21000 rw-p 00000000 00:00 0       [heap]
# 7f4c8a000000-7f4c8a021000 rw-p 00000000 00:00 0
# 7ffcb7f60000-7ffcb7f81000 rw-p 00000000 00:00 0       [stack]
```

Each line shows: `address range`, `permissions (rwxp/s)`, `offset`, `device`, `inode`, `pathname`.

### 3.2 The `vm_area_struct` in the Kernel

Internally, each VMA is represented by a `vm_area_struct`:

```c
struct vm_area_struct {
    unsigned long vm_start;        // Start address
    unsigned long vm_end;          // End address
    struct vm_area_struct *vm_next; // Next VMA in linked list
    pgprot_t vm_page_prot;         // Access permissions
    unsigned long vm_flags;        // Flags (VM_READ, VM_WRITE, VM_EXEC, ...)
    struct file *vm_file;          // Backing file (NULL for anonymous)
    unsigned long vm_pgoff;        // Offset within file
    // ... more fields
};
```

All VMAs for a process are stored in both a **linked list** (for sequential traversal) and a **red-black tree** (for fast lookup by address) — the tree enables O(log n) lookup when handling page faults.

---

## 4. Demand Paging

### 4.1 The Lazy Approach

Linux does **not** allocate physical memory when a process requests it. Instead, it just creates a VMA (virtual address range) and waits. Physical pages are allocated only when actually accessed — this is called **demand paging**.

```
malloc(1 GB):
  Step 1: Kernel creates VMA [0x7f...000 - 0x7f...000+1GB]
          with permissions rw-, backed by zero-fill
  Step 2: NO physical memory allocated yet!
  Step 3: Process returns pointer immediately

First access to page at 0x7f...100000:
  Step 1: MMU can't translate (no PTE) → PAGE FAULT
  Step 2: Kernel allocates one physical page (4 KB)
  Step 3: Kernel creates PTE mapping VA → PA
  Step 4: Kernel zero-fills the page
  Step 5: Process resumes, access succeeds

Only 4 KB allocated, not 1 GB!
```

This is why you can `malloc` more memory than physically available — the memory isn't real until you touch it.

### 4.2 Page Fault Types

| Fault Type | Cause | Kernel Action |
|------------|-------|---------------|
| **Minor fault** | Page exists but PTE not yet set up | Allocate frame, set PTE |
| **Major fault** | Page must be read from disk (swap or file) | Read from disk, allocate frame, set PTE |
| **Invalid fault** | Access to unmapped region (bug!) | Send SIGSEGV → segmentation fault |
| **Protection fault** | Permission violation (write to read-only) | SIGSEGV or CoW handling |

```bash
# View page fault statistics for a process
/usr/bin/time -v ./my_program 2>&1 | grep "page faults"

# Or in real-time
cat /proc/<PID>/stat  # Fields 10 (minor) and 12 (major) faults
```

---

## 5. Copy-on-Write (CoW)

### 5.1 The Problem

When a process calls `fork()`, the child gets an exact copy of the parent's entire address space. Naively copying all memory would be:
- **Slow** — copying hundreds of MB or GB of data
- **Wasteful** — the child often calls `exec()` immediately, discarding the copied memory

### 5.2 The Solution: Copy-on-Write

Instead of copying, both parent and child **share** the same physical pages, marked as **read-only**:

```
Before fork():
Parent Process
VA Page 0 ──► Physical Frame 5  [RW]
VA Page 1 ──► Physical Frame 8  [RW]
VA Page 2 ──► Physical Frame 3  [RW]

After fork() (with CoW):
Parent Process                      Child Process
VA Page 0 ──┐                  ┌──► VA Page 0
             ├──► Frame 5 [RO] ├
VA Page 1 ──┐                  ┌──► VA Page 1
             ├──► Frame 8 [RO] ├
VA Page 2 ──┐                  ┌──► VA Page 2
             └──► Frame 3 [RO] ┘

Both processes share the same physical pages!
All pages marked Read-Only.
Reference count for each frame: 2
```

### 5.3 What Happens on Write?

When either process tries to **write** to a shared page:

```
Parent writes to Page 1:
1. MMU detects write to RO page → Protection Fault
2. Kernel checks: is this a CoW page? (ref count > 1)
3. Yes → Allocate new physical frame (Frame 12)
4. Copy content: Frame 8 → Frame 12
5. Update Parent's PTE: Page 1 → Frame 12 [RW]
6. Decrement ref count of Frame 8 (now 1)
7. If ref count == 1, mark Frame 8 as [RW] for Child
8. Resume Parent's write operation

After CoW trigger:
Parent Process                      Child Process
VA Page 0 ──┐                  ┌──► VA Page 0
             ├──► Frame 5 [RO] ├
VA Page 1 ──► Frame 12 [RW]        VA Page 1 ──► Frame 8 [RW]
             (new copy!)            (now exclusive)
VA Page 2 ──┐                  ┌──► VA Page 2
             └──► Frame 3 [RO] ┘
```

**Result:** Only the modified page is copied. Pages that are never written are never duplicated. This makes `fork()` nearly instantaneous regardless of process size.

---

## 6. Memory Mapping (mmap)

### 6.1 What Is mmap?

`mmap()` maps a file or device into a process's virtual address space, allowing file I/O through **memory reads and writes** instead of `read()`/`write()` system calls:

```c
#include <sys/mman.h>

// Map a file into memory
void *addr = mmap(NULL,           // Let kernel choose address
                  file_size,      // Length to map
                  PROT_READ,      // Protection: read-only
                  MAP_PRIVATE,    // Private mapping (CoW)
                  fd,             // File descriptor
                  0);             // Offset in file

// Now you can access file contents like an array:
char first_byte = ((char *)addr)[0];
char tenth_byte = ((char *)addr)[9];

// Unmap when done
munmap(addr, file_size);
```

### 6.2 mmap Types

| Type | Flag | Backing | Changes Visible To |
|------|------|---------|-------------------|
| **File-backed, Private** | `MAP_PRIVATE` | File on disk | This process only (CoW) |
| **File-backed, Shared** | `MAP_SHARED` | File on disk | All processes + written to file |
| **Anonymous, Private** | `MAP_ANONYMOUS \| MAP_PRIVATE` | Zero-fill | This process only |
| **Anonymous, Shared** | `MAP_ANONYMOUS \| MAP_SHARED` | Zero-fill | All child processes |

### 6.3 How Shared Libraries Are Loaded

When your program uses `libc.so`, the dynamic linker uses `mmap` to load it:

```
libc.so on disk:
┌────────┬────────┬────────┐
│ .text  │ .rodata│ .data  │
│ (code) │ (const)│ (vars) │
└────────┴────────┴────────┘

Process A:                        Process B:
VA 0x7f...000 ──┐                 VA 0x7f...000 ──┐
  .text [r-x]    ├──► Same physical pages          ├──► Same physical pages
  .rodata [r--]  │    (MAP_PRIVATE,                │    (shared, read-only)
                 │     read-only → shared)          │
VA 0x7f...200 ──┘                 VA 0x7f...200 ──┘

VA 0x7f...300                     VA 0x7f...300
  .data [rw-] ──► Frame 100       .data [rw-] ──► Frame 200
                  (CoW: private    (CoW: private
                   copy per proc)   copy per proc)
```

**The .text and .rodata sections are shared** across all processes using the same library — only one copy in physical memory. The .data section uses CoW — each process gets its own copy only when it modifies the data.

This is why loading shared libraries is extremely efficient.

### 6.4 mmap vs. read/write

| Aspect | mmap | read/write |
|--------|------|------------|
| Copies | Zero-copy (direct page mapping) | Data copied: kernel buffer → user buffer |
| Random access | Excellent (just pointer arithmetic) | Requires `lseek()` |
| Sequential I/O | Good | Slightly better (read-ahead optimized) |
| Small files | Overhead (VMA setup, page faults) | Better |
| Large files | Excellent | Needs manual buffering |
| Shared access | Natural (MAP_SHARED) | Requires explicit IPC |

---

## 7. Swap Space

### 7.1 When Physical Memory Runs Out

When RAM is full and a process needs more pages, the kernel must **evict** some existing pages. If the evicted page is dirty (modified), it must be saved somewhere — that's what **swap** is for.

```
Physical Memory (full):
┌──────┬──────┬──────┬──────┬──────┬──────┐
│Page A│Page B│Page C│Page D│Page E│Page F│
│(used)│(idle)│(used)│(idle)│(used)│(idle)│
└──────┴──────┴──────┴──────┴──────┴──────┘
                                    Need new page!

Kernel selects Page B (idle, LRU) for eviction:
1. If dirty: write Page B to swap partition/file
2. Update PTE: mark as "not present", store swap location
3. Free physical frame
4. Allocate freed frame for new page

Swap partition/file:
┌──────┬──────┬──────┐
│Page B│Page X│ free │
│(saved)│(old)│      │
└──────┴──────┴──────┘
```

### 7.2 Swap-In (Page Fault on Swapped Page)

When the process accesses a swapped-out page:

```
1. MMU: PTE says "not present" → PAGE FAULT (major)
2. Kernel: PTE contains swap entry (device + offset)
3. Kernel: Read page from swap into a free physical frame
4. Kernel: Update PTE to point to new physical frame, mark "present"
5. Process resumes
```

Major page faults are expensive — disk I/O takes milliseconds (vs. nanoseconds for memory). This is why running out of physical RAM causes dramatic slowdowns ("thrashing").

### 7.3 Page Replacement: LRU Approximation

Linux uses a **two-list LRU approximation** to decide which pages to evict:

```
              ┌──────────────────┐
New pages ──► │   Active List     │  (recently accessed pages)
              │ (hot pages)       │
              └────────┬─────────┘
                       │ Not accessed recently
                       ▼
              ┌──────────────────┐
              │  Inactive List   │  (candidates for eviction)
              │ (cold pages)     │
              └────────┬─────────┘
                       │ Still not accessed
                       ▼
                   EVICTED
                   (freed or swapped out)
```

The kernel scans pages periodically using the **kswapd** daemon. Pages are promoted back to the active list if accessed while on the inactive list.

### 7.4 Swappiness

The `vm.swappiness` parameter (0–200, default 60) controls how aggressively the kernel swaps:

| Value | Behavior |
|:-----:|----------|
| 0 | Avoid swapping as much as possible (only under extreme pressure) |
| 60 | Balanced (default) |
| 100 | Swap and page cache treated equally |
| 200 | Aggressively swap anonymous pages |

```bash
# Check current swappiness
cat /proc/sys/vm/swappiness

# Set temporarily
sudo sysctl vm.swappiness=10

# Set permanently (in /etc/sysctl.conf)
vm.swappiness=10
```

For database servers and latency-sensitive applications, lower swappiness (10–20) is common to keep data in RAM.

---

## 8. OOM Killer

### 8.1 When All Else Fails

If the system runs out of both physical memory and swap, the **OOM (Out of Memory) Killer** intervenes to prevent a complete system freeze:

```
Memory pressure increasing...
  │
  ├── kswapd tries to free pages ──► Not enough
  ├── Direct reclaim (blocking) ──► Still not enough
  ├── Compact memory ──► Still not enough
  │
  ▼
OOM Killer activates:
  1. Calculate "badness score" for each process
  2. Select process with highest score
  3. Send SIGKILL to that process
  4. Log the event in dmesg
```

### 8.2 OOM Score

Each process has an OOM score (0–1000) based on:
- **Memory usage** (primary factor — bigger processes score higher)
- **oom_score_adj** (user-configurable adjustment, -1000 to 1000)
- Process age, root status, and other factors

```bash
# View OOM score of a process
cat /proc/<PID>/oom_score

# Protect a critical process from OOM killer
echo -1000 > /proc/<PID>/oom_score_adj   # Never kill this process

# Make a process more likely to be killed
echo 500 > /proc/<PID>/oom_score_adj
```

### 8.3 Overcommit Modes

Linux can be configured to handle memory overcommit differently:

```bash
# /proc/sys/vm/overcommit_memory
```

| Value | Mode | Behavior |
|:-----:|------|----------|
| 0 | Heuristic (default) | Kernel guesses if commit is "reasonable" |
| 1 | Always overcommit | `malloc` never fails (risky!) |
| 2 | No overcommit | `malloc` fails if commit > RAM + swap × ratio |

Mode 2 is used in safety-critical systems where OOM kills are unacceptable.

---

## 9. Huge Pages

### 9.1 Why Huge Pages?

Standard 4 KB pages work well for most cases, but large-memory applications (databases, VMs, AI training) benefit from larger pages:

| Aspect | 4 KB Pages | 2 MB Huge Pages | Improvement |
|--------|:----------:|:---------------:|:-----------:|
| Pages for 1 GB | 262,144 | 512 | 512× fewer |
| Page table memory | ~2 MB | ~4 KB | 500× less |
| TLB coverage (64 entries) | 256 KB | 128 MB | 512× more |
| TLB misses | Frequent | Rare | Major speedup |

### 9.2 Using Huge Pages in Linux

**Transparent Huge Pages (THP):** The kernel automatically merges adjacent 4 KB pages into 2 MB pages when possible.

```bash
# Check THP status
cat /sys/kernel/mm/transparent_hugepage/enabled
# [always] madvise never

# Check usage
grep -i huge /proc/meminfo
```

**Explicit Huge Pages:** Pre-allocate a pool of huge pages at boot:

```bash
# Reserve 1024 huge pages (2 MB each = 2 GB)
echo 1024 > /proc/sys/vm/nr_hugepages

# In application code:
void *p = mmap(NULL, 2*1024*1024, PROT_READ|PROT_WRITE,
               MAP_PRIVATE|MAP_ANONYMOUS|MAP_HUGETLB, -1, 0);
```

---

## 10. Kernel Memory Management

### 10.1 SLAB Allocator

The kernel itself needs to allocate memory for its own data structures (inodes, task_structs, network buffers). The **SLAB allocator** provides efficient allocation of fixed-size objects:

```
SLAB Cache for "task_struct" (size = 6656 bytes):

Slab 1 (one or more physical pages):
┌──────────┬──────────┬──────────┬──────────┐
│task_struct│task_struct│task_struct│  (free)  │
│  #1      │  #2      │  #3      │          │
└──────────┴──────────┴──────────┴──────────┘

Slab 2:
┌──────────┬──────────┬──────────┬──────────┐
│task_struct│  (free)  │  (free)  │  (free)  │
│  #4      │          │          │          │
└──────────┴──────────┴──────────┴──────────┘
```

**Benefits:**
- No fragmentation (all objects same size within a cache)
- Fast allocation (just grab from free list)
- Constructor/destructor support (pre-initialize objects)
- Cache coloring (distribute objects across cache lines)

Linux has evolved through three implementations: SLAB → SLUB (default) → SLOB (for tiny systems).

```bash
# View SLAB statistics
sudo slabtop
cat /proc/slabinfo
```

### 10.2 vmalloc vs. kmalloc

| Function | Physical Memory | Use Case |
|----------|:---------------:|----------|
| `kmalloc` | Physically contiguous | Small allocations, DMA buffers |
| `vmalloc` | Virtually contiguous, physically scattered | Large allocations (modules, buffers) |
| `alloc_pages` | Raw page allocation | Custom allocators |

---

## 11. Practical Tools for Memory Analysis

### 11.1 System-Wide Memory

```bash
# Overview
free -h
#               total    used    free    shared  buff/cache  available
# Mem:           16Gi    4.2Gi   1.8Gi    256Mi      10Gi       11Gi
# Swap:          8.0Gi   0.0Gi   8.0Gi

# "available" ≠ "free"
# available = free + reclaimable cache (what you can actually use)
```

```bash
# Detailed breakdown
cat /proc/meminfo
# MemTotal:       16384000 kB
# MemFree:         1843200 kB
# MemAvailable:   11520000 kB
# Buffers:          204800 kB
# Cached:          9830400 kB  ← Page cache (file data in RAM)
# SwapTotal:       8388608 kB
# SwapFree:        8388608 kB
# AnonPages:       4300800 kB  ← Process heap/stack memory
# Mapped:           512000 kB  ← mmap'd files
# Slab:             409600 kB  ← Kernel SLAB allocator
# PageTables:        51200 kB  ← Page table memory
# ...
```

### 11.2 Per-Process Memory

```bash
# Process memory summary
cat /proc/<PID>/status | grep -i vm
# VmPeak:   524288 kB    ← Peak virtual memory size
# VmSize:   512000 kB    ← Current virtual memory size
# VmRSS:    128000 kB    ← Resident Set Size (in physical RAM)
# VmData:    64000 kB    ← Data + heap
# VmStk:      8192 kB    ← Stack
# VmExe:      2048 kB    ← Code (.text)
# VmLib:     32000 kB    ← Shared libraries
# VmSwap:        0 kB    ← Swapped out pages
```

| Metric | Meaning |
|--------|---------|
| **VSZ / VmSize** | Total virtual memory (including unmapped) — can be huge |
| **RSS / VmRSS** | Physical memory actually used — what matters |
| **PSS** | Proportional Set Size — shared pages divided equally among sharing processes |
| **USS** | Unique Set Size — memory exclusive to this process |

```bash
# Most accurate per-process memory with smaps
cat /proc/<PID>/smaps_rollup
# Pss:        96000 kB   ← Best metric for "real" memory usage
```

### 11.3 Memory Monitoring

```bash
# Real-time per-process
top  # or htop
# Press 'M' to sort by memory

# Page faults and swap
vmstat 1
# procs  memory         swap     io       system      cpu
# r  b  swpd  free   si   so   bi   bo   in   cs  us sy id
# 1  0     0 1843200  0    0    4    8   200  300  5  2 93

# si/so = swap in/out (should be 0 for healthy system)
```

---

## 12. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Virtual memory** | Every process gets its own address space; MMU translates VA → PA |
| **Page tables** | 4-level (or 5-level) tree structure; only populated entries consume memory |
| **TLB** | Cache of recent translations; >99% hit rate; flushed on context switch |
| **Demand paging** | Physical memory allocated only when first accessed (lazy allocation) |
| **Copy-on-Write** | fork() shares pages read-only; copy only on write — makes fork() fast |
| **mmap** | Map files/devices into address space; zero-copy I/O; shared libraries |
| **Swap** | Backs anonymous pages when RAM is full; major page faults are expensive |
| **OOM Killer** | Last resort when memory exhausted; kills highest-scoring process |
| **Huge pages** | 2 MB/1 GB pages reduce TLB misses for large workloads |
| **SLAB allocator** | Efficient kernel object allocation with caching |

Understanding virtual memory is essential for:
- **Performance tuning** — minimizing page faults, TLB misses, and swap usage
- **Debugging** — understanding segfaults, memory leaks, and OOM conditions
- **System design** — choosing appropriate memory allocation strategies for your application

---

*This post is part of the **Linux Internals** series. See also: [Linux Architecture](/posts/linux-architecture/) and [Linux File Systems](/posts/linux-file-systems/).*
