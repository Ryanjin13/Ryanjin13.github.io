---
title: "Linux Architecture: Understanding the OS Internals"
date: 2026-02-19
description: "A structural analysis of the Linux operating system covering Kernel internals, System Calls, process management, virtual memory, file systems, and the boot process"
categories: ["Linux"]
tags: ["Linux", "Operating System", "Kernel", "System Architecture", "Process Management"]
draft: false
---

{{< katex >}}

## Overview

Linux is an operating system built on a **monolithic kernel**. Its architecture is divided into three major layers: **User Space**, the **System Call Interface**, and **Kernel Space**. The separation between User Space and Kernel Space is the most fundamental design principle of the Linux architecture, enforced at the hardware level by the CPU's **privilege rings**.

```
┌──────────────────────────────────────────────────┐
│                  User Space                       │
│  ┌────────────────────────────────────────────┐  │
│  │  Applications (bash, vim, firefox, ROS2...)│  │
│  ├────────────────────────────────────────────┤  │
│  │  Libraries (glibc, libpthread, libm...)    │  │
│  └────────────────────────────────────────────┘  │
├════════════════ System Call Interface ════════════┤
│                  Kernel Space                     │
│  ┌────────────────────────────────────────────┐  │
│  │  Process Management    Memory Management   │  │
│  │  File System (VFS)     Network Stack       │  │
│  │  Device Drivers        IPC                 │  │
│  │  Scheduler             Security (SELinux)  │  │
│  └────────────────────────────────────────────┘  │
├──────────────────────────────────────────────────┤
│                  Hardware                         │
│  CPU  │  RAM  │  Disk  │  NIC  │  GPU  │  ...   │
└──────────────────────────────────────────────────┘
```

---

## 1. Privilege Rings

The CPU enforces the User/Kernel boundary through hardware privilege levels:

```
     ┌──────────────────────┐
     │      Ring 3          │  ← User Space (restricted privileges)
     │  ┌───────────────┐   │
     │  │    Ring 0      │   │  ← Kernel Space (full privileges)
     │  │  (Kernel)      │   │
     │  └───────────────┘   │
     └──────────────────────┘
```

- **Ring 0 (Kernel Mode)**: Full hardware access, entire memory space accessible
- **Ring 3 (User Mode)**: Limited instruction set, no direct hardware access

---

## 2. Kernel Internals

### 2.1 Monolithic vs Microkernel

Linux uses a **monolithic kernel**, meaning all core functionality runs in a **single memory space**:

```
Monolithic Kernel (Linux):          Microkernel (Minix, QNX):
┌─────────────────────┐           ┌─────────────────────┐
│     Kernel Space     │           │     User Space       │
│                     │           │  ┌────┐ ┌────┐      │
│ ┌─────┐ ┌────────┐ │           │  │FS  │ │Net │      │
│ │FS   │ │Network │ │           │  │Srv │ │Srv │      │
│ │     │ │Stack   │ │           │  └──┬─┘ └─┬──┘      │
│ ├─────┤ ├────────┤ │           ├─────┼─────┼──────────┤
│ │Sched│ │Memory  │ │           │  Kernel (minimal)     │
│ │uler │ │Mgmt    │ │           │  IPC + Scheduler     │
│ ├─────┤ ├────────┤ │           └─────────────────────┘
│ │Drvr │ │IPC     │ │
│ └─────┘ └────────┘ │           Pros: Stability (service isolation)
└─────────────────────┘           Cons: IPC overhead → lower performance
Pros: Performance (function calls)
Cons: Single bug can affect the entire system
```

Linux maintains flexibility through **Loadable Kernel Modules (LKM)**, allowing drivers to be dynamically loaded and unloaded at runtime:

```bash
# Load a module
sudo modprobe usb_storage

# List loaded modules
lsmod

# Unload a module
sudo rmmod usb_storage
```

### 2.2 Kernel Subsystems

```
┌───────────────────────────────────────────────┐
│                 Linux Kernel                   │
│                                               │
│  ┌─────────────┐  ┌──────────────┐           │
│  │  Process     │  │   Memory     │           │
│  │  Management  │  │   Management │           │
│  │             │  │              │           │
│  │ - fork/exec │  │ - Virtual    │           │
│  │ - Scheduler │  │   Memory     │           │
│  │ - Signals   │  │ - Page Cache │           │
│  │ - Threads   │  │ - Slab Alloc │           │
│  └──────┬──────┘  └──────┬───────┘           │
│         │                │                    │
│  ┌──────┴──────┐  ┌──────┴───────┐           │
│  │   VFS       │  │   Network    │           │
│  │ (Virtual    │  │   Stack      │           │
│  │  File       │  │              │           │
│  │  System)    │  │ - Socket     │           │
│  │             │  │ - TCP/IP     │           │
│  │ - ext4      │  │ - Netfilter  │           │
│  │ - btrfs     │  │ - Routing    │           │
│  │ - procfs    │  │              │           │
│  └──────┬──────┘  └──────┬───────┘           │
│         │                │                    │
│  ┌──────┴────────────────┴───────┐           │
│  │        Device Drivers          │           │
│  │  char │ block │ network        │           │
│  └───────────────┬───────────────┘           │
└──────────────────┼───────────────────────────┘
                   │
            ┌──────┴──────┐
            │  Hardware    │
            └─────────────┘
```

---

## 3. System Call Interface

System calls are the **only official interface** for User Space to request Kernel Space functionality.

### 3.1 How System Calls Work

```
User Space                    Kernel Space
┌──────────────┐             ┌──────────────┐
│ Application  │             │              │
│              │             │  sys_read()  │
│  read(fd,    │  ──trap──→  │              │
│    buf, n)   │  (int 0x80  │  Perform     │
│              │   or        │  actual      │
│              │   syscall)  │  file I/O    │
│  ←result──── │  ←─return── │              │
└──────────────┘             └──────────────┘
```

The process:
1. Application calls glibc's `read()` wrapper function
2. glibc sets syscall number and arguments in registers
3. Executes `syscall` instruction (x86_64) or `int 0x80` (x86) → **CPU mode switch (Ring 3 → Ring 0)**
4. Kernel's syscall handler processes the system call
5. Result placed in registers, returns to User Space → **CPU mode switch (Ring 0 → Ring 3)**

### 3.2 Major System Call Categories

| Category | Examples | Description |
|----------|---------|-------------|
| **Process** | `fork`, `exec`, `wait`, `exit` | Process creation/execution/termination |
| **File I/O** | `open`, `read`, `write`, `close` | File input/output |
| **Memory** | `mmap`, `brk`, `munmap` | Memory allocation/deallocation |
| **Network** | `socket`, `bind`, `listen`, `accept` | Socket communication |
| **Signals** | `kill`, `signal`, `sigaction` | Inter-process signaling |
| **Info** | `getpid`, `uname`, `time` | System information queries |

---

## 4. Process Management

### 4.1 Process Memory Layout

Each process has an independent **virtual address space**:

```
High address  0xFFFFFFFFFFFFFFFF (64-bit)
    ┌──────────────────────────┐
    │      Kernel Space         │  ← Shared by all processes
    │      (not accessible)     │
    ├══════════════════════════┤  0xFFFF800000000000
    │                          │
    │      Stack ↓             │  ← Function calls, local variables
    │      (grows downward)    │
    │                          │
    │      ↕ (free space)      │
    │                          │
    │      Heap ↑              │  ← malloc/new dynamic allocation
    │      (grows upward)      │
    │                          │
    ├──────────────────────────┤
    │      BSS                 │  ← Uninitialized global variables
    ├──────────────────────────┤
    │      Data                │  ← Initialized global/static variables
    ├──────────────────────────┤
    │      Text (Code)         │  ← Executable code (read-only)
    └──────────────────────────┘
Low address   0x0000000000000000
```

### 4.2 Process State Transitions

```
                    fork()
                      │
                      ▼
               ┌──────────────┐
               │   Created     │
               │  (TASK_NEW)   │
               └──────┬───────┘
                      │
                      ▼
               ┌──────────────┐     I/O request / sleep
      ┌───────│   Ready       │────────────────┐
      │       │  (TASK_RUNNING)│               │
      │       └──────┬───────┘               ▼
      │              │                ┌──────────────┐
      │   Scheduler  │                │   Blocked     │
      │   selects    │                │ (TASK_INTER-  │
      │              ▼                │  RUPTIBLE)    │
      │       ┌──────────────┐       └──────┬───────┘
      │       │   Running     │              │
      └───────│  (on CPU)     │        I/O complete /
   preempt/   └──────┬───────┘        signal
   yield      exit() │                      │
                     ▼                      │
               ┌──────────────┐             │
               │   Zombie      │←────────────┘
               │  (EXIT_ZOMBIE)│     (returns to Ready)
               └──────┬───────┘
                      │ wait()
                      ▼
               ┌──────────────┐
               │   Terminated  │
               └──────────────┘
```

### 4.3 Process vs Thread

In Linux, threads are implemented as **Lightweight Processes (LWP)**. The `clone()` system call's flags determine what is shared:

```
Process (fork):                  Thread (clone + CLONE_VM):

PID 100          PID 200         PID 100, TID 100   PID 100, TID 101
┌──────────┐    ┌──────────┐    ┌──────────┐       ┌──────────┐
│ Text     │    │ Text     │    │ Text     │       │          │
│ Data     │    │ Data     │    │ Data     │ shared│  shared  │
│ Heap     │    │ Heap     │    │ Heap     │←────→│          │
│ Stack    │    │ Stack    │    │ Stack    │       │ Stack    │
│ Page     │    │ Page     │    │ Page     │       │ (own)    │
│ Table    │    │ Table    │    │ Table    │       │          │
└──────────┘    └──────────┘    └──────────┘       └──────────┘
  Fully independent (COW)        Shared memory space, only stack is independent
```

---

## 5. Memory Management

### 5.1 Virtual Memory

Linux translates virtual addresses to physical addresses through **page tables**:

```
Virtual Address                          Physical Memory
┌──────────┐                            ┌──────────┐
│ Page 0   │──── Page Table ──────────→ │ Frame 5  │
│ Page 1   │──── Page Table ──→ (disk)  │ Frame 2  │
│ Page 2   │──── Page Table ──────────→ │ Frame 8  │
│ Page 3   │──── Page Table ──────────→ │ Frame 1  │
│ ...      │                            │ ...      │
└──────────┘                            └──────────┘

     MMU (Memory Management Unit) performs translation
     TLB (Translation Lookaside Buffer) for caching
```

**4-level page table** (x86_64):

```
Virtual Address (48-bit used):
┌────────┬────────┬────────┬────────┬──────────┐
│ PGD(9) │ PUD(9) │ PMD(9) │ PTE(9) │ Offset(12)│
└───┬────┴───┬────┴───┬────┴───┬────┴──────────┘
    │        │        │        │
    ▼        ▼        ▼        ▼
  PGD    → PUD    → PMD    → PTE    → Physical Frame
  Table    Table    Table    Table     + Offset
```

Default page size is **4KB** (\\(2^{12}\\) bytes).

### 5.2 Page Cache

Linux uses **idle memory as file cache** to improve disk I/O performance:

```
Application
    │
    │ read()
    ▼
┌──────────────────┐
│   Page Cache     │  ← File data cached in memory
│  (in RAM)        │
│                  │
│  Cache Hit? ─Yes→ Return immediately (no disk access)
│      │           │
│     No           │
│      ▼           │
│  Read from disk  │
│  → Store in cache│
│  → Return        │
└──────────────────┘
```

This is why `free` often shows little "available" memory on Linux — the system aggressively caches files, but this cache can be reclaimed instantly when needed.

---

## 6. File System

### 6.1 VFS (Virtual File System)

The abstraction layer that implements Linux's core philosophy: **"Everything is a File."**

```
Application
    │
    │  open(), read(), write()
    ▼
┌──────────────────────────────────────┐
│            VFS (Virtual File System)  │
│                                      │
│  Unified interface:                  │
│  struct file_operations {             │
│      .read    = ...                  │
│      .write   = ...                  │
│      .open    = ...                  │
│      .release = ...                  │
│  }                                   │
├──────────┬──────────┬────────────────┤
│  ext4    │  btrfs   │  procfs        │
│  (disk)  │  (disk)  │ (virtual:/proc)│
├──────────┤          ├────────────────┤
│  xfs     │  tmpfs   │  sysfs         │
│  (disk)  │  (RAM)   │ (virtual:/sys) │
└──────────┴──────────┴────────────────┘
```

Thanks to VFS, `cat /proc/cpuinfo` and `cat /etc/hostname` use the same interface. One reads kernel data, the other reads a disk file — but the application sees no difference.

### 6.2 Directory Structure (FHS)

```
/
├── bin/      → Essential user commands (ls, cp, cat)
├── sbin/     → Essential system commands (mount, fdisk)
├── etc/      → System configuration files
├── home/     → User home directories
├── root/     → Root user's home
├── var/      → Variable data (logs, cache, mail)
│   ├── log/
│   └── cache/
├── tmp/      → Temporary files
├── usr/      → User programs (secondary hierarchy)
│   ├── bin/
│   ├── lib/
│   ├── local/
│   └── share/
├── lib/      → Shared libraries (.so files)
├── dev/      → Device files (hardware abstraction)
│   ├── sda   → Disk
│   ├── tty   → Terminal
│   └── null  → /dev/null (black hole)
├── proc/     → Process/kernel info (virtual filesystem)
│   ├── cpuinfo
│   ├── meminfo
│   └── [PID]/
├── sys/      → Kernel/device info (virtual filesystem)
├── boot/     → Bootloader, kernel image
└── mnt/      → Mount points
```

### 6.3 inode Structure

Every file in a Linux filesystem is managed through an **inode**:

```
Directory Entry          inode                    Data Blocks
┌────────────┐       ┌──────────────┐        ┌──────────┐
│ "hello.txt"│──────→│ inode #42    │        │ Block 100│
│ inode: 42  │       │              │        │ "Hello,  │
└────────────┘       │ Owner: user  │        │  World!" │
                     │ Perms: 644   │        └──────────┘
                     │ Size: 13B    │        ┌──────────┐
                     │ Timestamps   │        │ Block 101│
                     │              │        │ (more    │
                     │ Direct ptrs  │───────→│  data)   │
                     │ Indirect ptr │        └──────────┘
                     │ Double indir │
                     │ Triple indir │
                     └──────────────┘

Key insight: The filename is NOT stored in the inode!
      → This is why hard links are possible
```

---

## 7. Inter-Process Communication (IPC)

Linux provides a variety of IPC mechanisms:

```
┌──────────────────────────────────────────┐
│              IPC Mechanisms               │
├──────────────┬───────────────────────────┤
│ Traditional  │ System V / POSIX          │
├──────────────┼───────────────────────────┤
│ Pipe         │ Message Queue             │
│ Named Pipe   │ Shared Memory             │
│ (FIFO)       │ Semaphore                 │
│ Signal       │                           │
├──────────────┼───────────────────────────┤
│ Network-based│ Modern                    │
├──────────────┼───────────────────────────┤
│ Socket       │ D-Bus                     │
│ (Unix Domain)│ eventfd                   │
│              │ io_uring                  │
└──────────────┴───────────────────────────┘
```

### Pipe Structure

```
Process A                              Process B
┌──────────┐    ┌──────────────┐    ┌──────────┐
│          │    │   Pipe       │    │          │
│  stdout ─┼───→│ ┌──────────┐│───→│─ stdin   │
│  (fd[1]) │    │ │ Kernel   ││    │ (fd[0])  │
│          │    │ │ Buffer   ││    │          │
│          │    │ │  (64KB)  ││    │          │
└──────────┘    │ └──────────┘│    └──────────┘
                └──────────────┘

Example: ls -la | grep ".txt" | wc -l
         Process1  Pipe  Process2  Pipe  Process3
```

---

## 8. Boot Process

The Linux boot sequence:

```
Power ON
   │
   ▼
┌──────────────┐
│   BIOS/UEFI  │  ← Hardware initialization (POST)
│              │     Select boot device
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Bootloader  │  ← GRUB2: Load kernel image
│   (GRUB2)    │     Pass kernel parameters
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   Kernel     │  ← Detect hardware
│  Startup     │     Load drivers
│              │     Mount root filesystem
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   init       │  ← PID 1 process
│  (systemd)   │     Service manager
│              │     Start services in parallel
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Login       │  ← getty + login
│  Manager     │     or Display Manager (GDM)
└──────────────┘
```

### systemd Structure

**systemd** is the standard init system on modern Linux:

```
systemd (PID 1)
├── systemd-journald    (logging)
├── systemd-udevd       (device management)
├── systemd-networkd    (networking)
├── systemd-resolved    (DNS)
├── systemd-logind      (login management)
│
├── default.target
│   ├── multi-user.target
│   │   ├── sshd.service
│   │   ├── nginx.service
│   │   ├── NetworkManager.service
│   │   └── ...
│   └── graphical.target (optional)
│       └── gdm.service
│
└── Unit file locations:
    ├── /lib/systemd/system/     (package-provided)
    ├── /etc/systemd/system/     (admin overrides)
    └── /run/systemd/system/     (runtime)
```

---

## 9. Permissions and Security

### 9.1 File Permissions

```
-rwxr-xr-- 1 user group 4096 Feb 19 10:00 script.sh
│├─┤├─┤├─┤
│ │  │  └── Others: r-- (read only)
│ │  └───── Group:  r-x (read + execute)
│ └──────── Owner:  rwx (full access)
└────────── File type: - (regular file)

Octal representation: 754
  Owner: 7 = 4(r) + 2(w) + 1(x)
  Group: 5 = 4(r) + 0(-) + 1(x)
  Other: 4 = 4(r) + 0(-) + 0(-)
```

### 9.2 Security Layers

```
┌─────────────────────────────────────┐
│ DAC (Discretionary Access Control)  │  ← Traditional rwx permissions
├─────────────────────────────────────┤
│ MAC (Mandatory Access Control)      │  ← SELinux / AppArmor
├─────────────────────────────────────┤
│ Capabilities                        │  ← Fine-grained root privileges
├─────────────────────────────────────┤
│ Namespaces + cgroups                │  ← Container isolation (Docker)
├─────────────────────────────────────┤
│ seccomp                             │  ← System call filtering
└─────────────────────────────────────┘
```

---

## 10. Summary: Linux Design Philosophy

| Principle | Implementation |
|-----------|---------------|
| **Everything is a File** | VFS provides file interface for hardware, processes, and network |
| **Do one thing well** | Small utilities composed via pipes |
| **User/Kernel separation** | Ring 0/3, System Call Interface |
| **Everything is a process** | Process tree starting from init (PID 1) |
| **Transparency** | /proc, /sys expose kernel internals as files |

Understanding Linux architecture extends beyond OS knowledge — it forms the foundation for **robotic systems (ROS2)**, **embedded systems**, **server infrastructure**, and **containers (Docker/K8s)**. Knowing how the kernel manages processes, memory, files, and networking enables far more accurate diagnosis of performance issues and system behavior at higher levels.
