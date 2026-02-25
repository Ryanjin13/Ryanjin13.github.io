---
title: "Linux File Systems: How Data Lives on Disk (and How It Differs from Windows)"
date: 2026-02-25
description: "A comprehensive guide to Linux file systems — covering how data is physically stored on disk, the ext4 architecture, journaling, VFS abstraction, and a detailed comparison with Windows NTFS."
categories: ["Linux"]
tags: ["Linux", "File System", "ext4", "NTFS", "VFS", "inode", "Journaling", "Operating System", "Windows"]
series: ["Linux Internals"]
series_order: 3
draft: false
---

{{< katex >}}

## Introduction

In the [Linux Architecture](/posts/linux-architecture/) post, we briefly covered the VFS (Virtual File System) and the inode structure. In this post, we go much deeper:

- How does Linux **actually store** files and directories on a physical disk?
- What is a **journal** and why does it prevent data corruption?
- How does Linux's approach compare to **Windows NTFS**?

Whether you're a developer, sysadmin, or embedded engineer, understanding file systems is essential — it affects performance, reliability, and how you design your storage strategy.

---

## 1. What Is a File System?

A file system is the **data structure** that organizes how data is stored, named, and retrieved on a storage device. Without a file system, a disk is just a sequence of raw bytes with no structure.

```
Without file system:
┌──────────────────────────────────────────────────────┐
│ 0100110101111010010101001010101011101010100101001010...│
│ (just raw bytes — no names, no structure, no meaning) │
└──────────────────────────────────────────────────────┘

With file system:
┌─────────────────────────────────────────────────────────┐
│ Superblock │ Group Descriptors │ Block Bitmap │ ...     │
│                                                          │
│  /home/user/hello.txt  →  inode 42  →  blocks 100, 101 │
│  /var/log/syslog       →  inode 78  →  blocks 200–210  │
│  /bin/bash             →  inode 15  →  blocks 50–80    │
└─────────────────────────────────────────────────────────┘
```

A file system must answer three questions:
1. **Where** is the data physically stored? (block allocation)
2. **What** metadata describes the data? (permissions, timestamps, size)
3. **How** do we find data by name? (directory structure)

---

## 2. Disk Structure Fundamentals

### 2.1 Blocks: The Unit of Storage

Just as memory is divided into pages, disk storage is divided into **blocks** (typically 4 KB). The file system operates on blocks, not individual bytes:

```
Physical Disk:
┌────────┬────────┬────────┬────────┬────────┬────────┐
│Block 0 │Block 1 │Block 2 │Block 3 │Block 4 │Block 5 │ ...
│(4 KB)  │(4 KB)  │(4 KB)  │(4 KB)  │(4 KB)  │(4 KB)  │
└────────┴────────┴────────┴────────┴────────┴────────┘
```

**Why blocks, not bytes?**
- Disk I/O operates on sectors (512 bytes or 4096 bytes) — reading one byte reads the whole sector anyway
- Managing individual bytes would require billions of tracking entries
- 4 KB blocks align with the OS page size, enabling efficient caching

### 2.2 Partitions

A physical disk is divided into **partitions**, each with its own file system:

```
Physical Disk (500 GB):
┌──────────────┬──────────────────┬─────────────────┐
│ Partition 1  │   Partition 2    │  Partition 3     │
│   /boot      │       /          │     /home        │
│   ext4       │     ext4         │     ext4         │
│   (1 GB)     │    (100 GB)      │    (399 GB)      │
└──────────────┴──────────────────┴─────────────────┘

Partition Table (GPT or MBR) at the beginning of the disk
describes the layout.
```

| Partition Scheme | Max Disk Size | Max Partitions | Modern? |
|:----------------:|:-------------:|:--------------:|:-------:|
| MBR | 2 TB | 4 primary | Legacy |
| GPT | 9.4 ZB | 128 | Standard |

---

## 3. The ext4 File System: Linux's Default

### 3.1 History

| File System | Year | Key Innovation |
|:-----------:|:----:|---------------|
| ext | 1992 | First Linux-specific FS |
| ext2 | 1993 | Reliable, no journaling |
| ext3 | 2001 | Added journaling |
| **ext4** | **2008** | Extents, 1 EB max, delayed allocation |

ext4 is the **default file system** for most Linux distributions (Ubuntu, Fedora, Debian, etc.).

### 3.2 ext4 Disk Layout

An ext4 file system is divided into **block groups** for locality and performance:

```
ext4 Disk Layout:
┌─────────┬──────────┬──────────┬──────────┬──────────┬───────┐
│  Boot   │  Block   │  Block   │  Block   │  Block   │       │
│  Sector │  Group 0 │  Group 1 │  Group 2 │  Group 3 │  ...  │
│ (1 KB)  │          │          │          │          │       │
└─────────┴──────────┴──────────┴──────────┴──────────┴───────┘

Each Block Group:
┌───────────┬───────────┬────────┬────────┬────────┬──────────┐
│Superblock │  Group    │ Block  │ inode  │ inode  │   Data   │
│(backup)   │Descriptor │Bitmap  │Bitmap  │ Table  │  Blocks  │
│           │  Table    │        │        │        │          │
│  4 KB     │  varies   │ 4 KB   │ 4 KB   │varies  │  varies  │
└───────────┴───────────┴────────┴────────┴────────┴──────────┘
```

| Component | Purpose |
|-----------|---------|
| **Superblock** | Master record: total blocks, total inodes, block size, FS state |
| **Group Descriptor** | Locations of bitmaps and inode table for this group |
| **Block Bitmap** | 1 bit per block: 0 = free, 1 = used |
| **inode Bitmap** | 1 bit per inode: 0 = free, 1 = used |
| **inode Table** | Array of inode structures for this group |
| **Data Blocks** | Actual file content |

**Why block groups?** By storing a file's inode and data blocks in the same group, the disk head movement is minimized — this greatly improves performance on HDDs.

### 3.3 The Superblock

The superblock is the file system's most critical data structure — without it, the FS is unreadable:

```bash
# View superblock information
sudo dumpe2fs /dev/sda1 | head -40

# Key fields:
# Filesystem volume name:   my-data
# Filesystem UUID:          a1b2c3d4-...
# Block count:              26214400
# Block size:               4096
# Blocks per group:         32768
# Inodes per group:         8192
# Inode size:               256
# Journal size:             128M
# Filesystem state:         clean
```

Superblock copies are stored in multiple block groups for redundancy. If the primary superblock is corrupted:

```bash
# Recover from backup superblock
sudo e2fsck -b 32768 /dev/sda1   # Use backup at block 32768
```

### 3.4 The inode (Index Node)

Every file and directory has an **inode** — a data structure containing all metadata about the file **except its name**:

```
inode (256 bytes in ext4):
┌──────────────────────────────────────┐
│  File Type and Permissions (mode)    │  4 bytes
│  Owner UID                           │  4 bytes
│  Group GID                           │  4 bytes
│  File Size (bytes)                   │  8 bytes (64-bit)
│  Timestamps:                         │
│    - atime (last access)             │  4 bytes
│    - ctime (last inode change)       │  4 bytes
│    - mtime (last data modification)  │  4 bytes
│    - crtime (creation time)          │  4 bytes
│  Hard Link Count                     │  4 bytes
│  Block Count                         │  4 bytes
│  Flags                               │  4 bytes
│                                      │
│  Data Block Pointers:                │
│    Extent Tree (ext4)                │  60 bytes
│    OR                                │
│    12 Direct + Indirect + Double     │
│    + Triple Indirect Pointers (ext2) │
│                                      │
│  Extended Attributes (xattr)         │  remaining space
└──────────────────────────────────────┘
```

**Key insight:** The file **name** is stored in the **directory entry**, not in the inode. This is what makes **hard links** possible — multiple names can point to the same inode:

```bash
# Create a hard link
ln original.txt link.txt

# Both have the same inode number:
ls -li original.txt link.txt
# 42 -rw-r--r-- 2 user group 100 Feb 25 10:00 original.txt
# 42 -rw-r--r-- 2 user group 100 Feb 25 10:00 link.txt
#  ↑                 ↑
# Same inode!    Link count = 2
```

### 3.5 Extents: How ext4 Tracks Data Location

ext2/ext3 used **indirect block pointers** — a tree of pointers for large files:

```
ext2/ext3 (indirect pointers):
inode
├── Direct Pointer 0  → Block 100
├── Direct Pointer 1  → Block 101
├── ...
├── Direct Pointer 11 → Block 111
├── Indirect Pointer  → [Block 200: ptr→300, ptr→301, ...]
├── Double Indirect   → [Block 400: ptr→[500: ptr→600, ...]]
└── Triple Indirect   → [Block 700: ptr→[800: ptr→[900: ...]]]
```

This is inefficient for large contiguous files — thousands of individual pointers needed.

ext4 uses **extents** — each extent describes a **contiguous range** of blocks:

```
ext4 (extents):
inode
├── Extent 0: start=100, length=50    → Blocks 100–149 (200 KB)
├── Extent 1: start=500, length=200   → Blocks 500–699 (800 KB)
└── Extent 2: start=1000, length=1000 → Blocks 1000–1999 (4 MB)

Just 3 entries describe 5 MB of data!
(vs. 1,250 individual pointers in ext2)
```

An extent entry is compact (12 bytes):

```
Extent Entry:
┌────────────────┬──────────┬────────────────┐
│ Logical Block  │ Length   │ Physical Block  │
│  (file offset) │ (blocks) │ (disk location) │
│   4 bytes      │ 2 bytes  │   6 bytes       │
└────────────────┴──────────┴────────────────┘
```

For very large files, extents are organized in a **B-tree** (extent tree) for O(log n) lookup.

### 3.6 Directories in ext4

A **directory** is just a special file whose content is a list of (name, inode) pairs:

```
Directory /home/user/ (inode 200):

Linear format (small directories):
┌─────────┬───────┬─────────┬──────────┐
│ inode   │ reclen│ name_len│ name     │
│ number  │       │         │          │
├─────────┼───────┼─────────┼──────────┤
│   200   │  12   │    1    │  "."     │  (self)
│   100   │  12   │    2    │  ".."    │  (parent)
│    42   │  24   │    9    │"hello.txt"│
│    78   │  20   │    6    │"photos"  │
│    99   │  28   │   11   │"project.c"│
└─────────┴───────┴─────────┴──────────┘

Hash tree format (large directories, >2 block):
Uses HTree (hash-indexed B-tree) for O(1) average lookup
instead of O(n) linear scan.
```

**Path resolution example:** `/home/user/hello.txt`

```
1. Root inode (always inode 2) → read directory entries
2. Find "home" → inode 50 → read directory entries
3. Find "user" → inode 100 → read directory entries
4. Find "hello.txt" → inode 42 → read inode metadata
5. Read data blocks from inode 42's extent tree
```

---

## 4. Journaling: Crash-Proof File Systems

### 4.1 The Crash Problem

Without journaling, a power loss during a write operation can leave the file system in an **inconsistent state**:

```
Writing a new file (3 steps):
1. Allocate inode     ← Power fails here!
2. Write data blocks
3. Update directory

Result: inode allocated but not in any directory
        → "orphan inode" → disk space leaked!
```

Or worse:

```
Deleting a file (3 steps):
1. Remove directory entry  ← Power fails here!
2. Free data blocks
3. Free inode

Result: Directory entry gone, but blocks still marked "used"
        → Blocks leaked forever!
```

After a crash, `fsck` (file system check) must scan the **entire disk** to find and fix inconsistencies. For large disks, this can take **hours**.

### 4.2 How Journaling Works

A **journal** is a dedicated area on disk where the file system writes a **plan** (log) of upcoming changes before making them:

```
Journal Area:
┌──────────────────────────────────────────────────┐
│ Transaction 1:                                    │
│   "About to: allocate inode 42,                  │
│    write blocks 100-102,                          │
│    add 'hello.txt' to dir inode 200"             │
│   Status: COMPLETE                                │
├──────────────────────────────────────────────────┤
│ Transaction 2:                                    │
│   "About to: delete 'old.txt' from dir 200,     │
│    free inode 55, free blocks 300-305"           │
│   Status: IN-PROGRESS                             │
└──────────────────────────────────────────────────┘
```

**The journaling process:**

```
Step 1: JOURNAL WRITE (write plan to journal)
        "I will modify blocks X, Y, Z with data A, B, C"

Step 2: JOURNAL COMMIT (mark transaction as committed)
        "Plan is complete and valid"

Step 3: CHECKPOINT (apply changes to actual file system)
        Write actual data to blocks X, Y, Z

Step 4: JOURNAL CLEANUP (free journal space)
        "Transaction applied, journal entry can be reused"
```

**After a crash:**

```
Case 1: Crash during Step 1 (journal write)
  → Transaction incomplete → discard → no damage

Case 2: Crash during Step 2 (before commit)
  → Transaction not committed → discard → no damage

Case 3: Crash during Step 3 (checkpoint)
  → Transaction committed but not all changes applied
  → REPLAY journal: re-apply committed transactions
  → File system consistent in seconds, not hours!
```

### 4.3 Journal Modes in ext4

| Mode | What's Journaled | Performance | Safety |
|------|:----------------:|:-----------:|:------:|
| **journal** | Metadata + Data | Slowest | Highest |
| **ordered** (default) | Metadata only; data written before metadata | Good | Good |
| **writeback** | Metadata only; data can be written anytime | Fastest | Lowest |

```bash
# Check current journal mode
sudo dumpe2fs /dev/sda1 | grep "Journal features"

# Mount with specific mode
sudo mount -o data=journal /dev/sda1 /mnt
```

**Ordered mode** (default) is a smart compromise: it doesn't journal data, but it ensures data blocks are written to disk **before** the metadata that references them. This prevents the case where metadata points to blocks containing old/garbage data.

---

## 5. Other Linux File Systems

### 5.1 Comparison Table

| Feature | ext4 | XFS | Btrfs | ZFS | F2FS |
|---------|:----:|:---:|:-----:|:---:|:----:|
| Year | 2008 | 1994 | 2009 | 2005 | 2012 |
| Max file size | 16 TB | 8 EB | 16 EB | 16 EB | 3.94 TB |
| Max FS size | 1 EB | 8 EB | 16 EB | 256 ZB | 16 TB |
| Journaling | Yes | Yes | CoW | CoW | Yes |
| Snapshots | No | No | Yes | Yes | No |
| Compression | No | No | Yes | Yes | Yes |
| Checksums | Metadata | Metadata | Data+Meta | Data+Meta | Data+Meta |
| RAID support | No (use mdraid) | No | Built-in | Built-in | No |
| Best for | General use | Large files | Snapshots | Enterprise | Flash/SSD |

### 5.2 Virtual / Pseudo File Systems

Not all file systems store data on disk:

| File System | Mount Point | Contents |
|-------------|-------------|----------|
| **procfs** | `/proc` | Process info, kernel state (virtual) |
| **sysfs** | `/sys` | Device/driver hierarchy (virtual) |
| **tmpfs** | `/tmp`, `/dev/shm` | RAM-backed temporary storage |
| **devtmpfs** | `/dev` | Device nodes |
| **cgroup** | `/sys/fs/cgroup` | Control groups for resource limits |

```bash
# procfs: read CPU info (no file on disk — generated on-the-fly)
cat /proc/cpuinfo

# sysfs: read CPU frequency
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq

# tmpfs: ultra-fast temporary files (in RAM, lost on reboot)
df -h /tmp
# tmpfs   7.8G   48M  7.8G  1% /tmp
```

---

## 6. Linux vs. Windows: File System Comparison

### 6.1 NTFS Overview

Windows uses **NTFS (New Technology File System)** as its primary file system (since Windows NT, 1993):

```
NTFS Structure:
┌──────────────────────────────────────────────┐
│  Boot Sector │ MFT │ MFT Mirror │ Data Area  │
└──────────────────────────────────────────────┘

MFT (Master File Table):
Every file/directory is an entry in the MFT.
Each MFT entry = 1 KB (fixed size).
```

### 6.2 inode vs. MFT Entry

| Aspect | Linux ext4 (inode) | Windows NTFS (MFT entry) |
|--------|:------------------:|:------------------------:|
| Size | 256 bytes | 1024 bytes (1 KB) |
| File name stored in... | Directory entry | MFT entry itself |
| Small file data | Not in inode | Can be **inside** MFT entry ("resident data") |
| Data location | Extent tree | Data runs (similar concept) |
| Hard links | Native support | Limited support |
| Symbolic links | Native (`ln -s`) | Junctions, Symlinks (limited) |
| Max filename | 255 bytes (UTF-8) | 255 chars (UTF-16) |

**Resident data in NTFS:** If a file is small enough (< ~700 bytes), NTFS stores the data **directly in the MFT entry** — no separate data blocks needed. This is very efficient for tiny files. ext4 has a similar feature called **inline data** (if enabled).

### 6.3 Directory Structure

| Aspect | Linux | Windows |
|--------|-------|---------|
| Root | `/` (single tree) | `C:\`, `D:\`, etc. (per-drive trees) |
| Path separator | `/` (forward slash) | `\` (backslash) |
| Case sensitivity | **Yes** (`File.txt` ≠ `file.txt`) | **No** (`File.txt` = `file.txt`) |
| Hidden files | Name starts with `.` | Hidden attribute flag |
| Max path length | ~4096 bytes | 260 chars (MAX_PATH) — extended to 32,767 with prefix |
| Device files | `/dev/sda`, `/dev/null` | `\\.\PhysicalDrive0` |
| Everything is a file? | **Yes** (pipes, sockets, devices are files) | No (different APIs for different object types) |

### 6.4 Mounting vs. Drive Letters

This is one of the most fundamental differences:

**Linux: Single Unified Tree**

```
/                        ← Root (always exists)
├── boot/                ← May be separate partition (/dev/sda1)
├── home/                ← May be separate partition (/dev/sda3)
│   └── user/
├── mnt/
│   └── usb/             ← USB drive mounted here
├── media/
│   └── cdrom/           ← CD-ROM mounted here
└── tmp/                 ← May be tmpfs (RAM)

All storage devices are "grafted" onto the single tree
using the mount command:

    mount /dev/sda3 /home
    mount /dev/sdb1 /mnt/usb
```

**Windows: Separate Drive Trees**

```
C:\                      ← System drive
├── Windows\
├── Program Files\
└── Users\

D:\                      ← Data drive (completely separate tree)
├── Projects\
└── Documents\

E:\                      ← USB drive (yet another separate tree)
└── Backup\

Each drive has its own independent tree.
No concept of a unified root.
```

### 6.5 Permissions Model

**Linux: POSIX Permissions + ACLs**

```
-rwxr-xr--  1 alice developers 4096 Feb 25 10:00 script.sh

Three levels: Owner (alice), Group (developers), Others
Three permissions each: Read (r), Write (w), Execute (x)

Extended: ACLs for fine-grained control
    setfacl -m u:bob:rx script.sh    # Give bob read+execute
```

**Windows: NTFS ACLs (Access Control Lists)**

```
script.bat:
  SYSTEM:        Full Control
  Administrators: Full Control
  alice:          Modify
  bob:            Read & Execute
  developers:     Read

Windows ACLs are more granular by default:
  - 13 individual permissions (vs. Linux's 3)
  - Explicit Allow and Deny entries
  - Inheritance from parent folders
```

| Aspect | Linux Permissions | Windows NTFS ACLs |
|--------|:-----------------:|:------------------:|
| Default model | rwx (3×3 = 9 bits) | ACL entries |
| Granularity | 3 levels (owner, group, others) | Per-user/per-group entries |
| Inheritance | Not by default (umask) | Built-in inheritance model |
| Deny rules | Not in basic model (ACLs support it) | Explicit Deny supported |
| Execute permission | Separate bit | Separate permission |
| File ownership | UID + GID | SID (Security Identifier) |

### 6.6 File System Features Comparison

| Feature | ext4 | NTFS |
|---------|:----:|:----:|
| Journaling | Yes (metadata or data) | Yes (metadata + data) |
| Compression | No (use filesystem-level tools) | Built-in per-file |
| Encryption | No built-in (use LUKS/dm-crypt) | Built-in EFS |
| Quotas | Yes | Yes |
| Snapshots | No | Volume Shadow Copy (VSS) |
| Sparse files | Yes | Yes |
| Alternate Data Streams | No | Yes (multiple data streams per file) |
| Hard links | Yes (full support) | Yes (limited) |
| Symbolic links | Yes (full support) | Yes (requires admin by default) |
| Max file size | 16 TB | 16 TB (practical: 256 TB theoretical) |
| Defragmentation needed | Rarely (extents + delayed alloc) | Frequently (on HDDs) |

### 6.7 NTFS Alternate Data Streams

A unique NTFS feature — each file can have **multiple named data streams**:

```
file.txt                    ← Default (unnamed) stream: "Hello World"
file.txt:hidden             ← Named stream: "Secret data"
file.txt:thumbnail          ← Named stream: (image data)

# Windows:
echo "Secret" > file.txt:hidden
more < file.txt:hidden

# Linux cannot see ADS when mounting NTFS
# (potential security issue when transferring files)
```

This is sometimes used for:
- Zone identifiers (tracking files downloaded from the internet)
- Thumbnails and metadata
- Unfortunately also malware hiding

Linux has no equivalent. Instead, extended attributes (`xattr`) serve a similar but more limited purpose.

---

## 7. File System Operations: Under the Hood

### 7.1 Creating a File

```bash
echo "Hello" > /home/user/hello.txt
```

What actually happens:

```
1. VFS: Resolve path /home/user/ → directory inode

2. ext4: Allocate new inode (scan inode bitmap for free entry)
   → inode 42

3. ext4: Initialize inode 42
   → type=regular file, permissions=644, owner=user
   → size=0, timestamps=now

4. ext4: Allocate data block (scan block bitmap)
   → block 1000

5. ext4: Write "Hello\n" to block 1000

6. ext4: Update inode 42
   → size=6, extent: logical_block=0, physical_block=1000, length=1

7. ext4: Add directory entry to /home/user/
   → "hello.txt" → inode 42

8. ext4: Update directory inode (mtime)

9. ext4: Update superblock (free block/inode counts)

All wrapped in a journal transaction!
```

### 7.2 Deleting a File

```bash
rm /home/user/hello.txt
```

```
1. Remove directory entry "hello.txt" from parent directory
2. Decrement inode 42's link count (hard link count)
3. If link count == 0 AND no process has the file open:
   a. Free data blocks (update block bitmap)
   b. Free inode (update inode bitmap)
   c. Update superblock (free counts)
4. If link count == 0 BUT file is still open:
   → Mark as "orphan" (deleted when last fd closes)

Note: The actual data is NOT erased!
      Only the metadata (bitmaps, directory entry) is updated.
      → This is why "undelete" tools can sometimes recover files.
```

### 7.3 Reading a File

```c
int fd = open("/home/user/hello.txt", O_RDONLY);
char buf[1024];
read(fd, buf, 1024);
```

```
1. open(): Resolve path → inode 42, create file descriptor
2. read():
   a. Check page cache — is the data already in memory?
      YES → Copy from page cache to user buffer (fast!)
      NO  → Continue to step b
   b. Look up extent tree in inode 42
      → Logical block 0 maps to physical block 1000
   c. Submit block I/O request to block layer
   d. Block layer: merge, sort, schedule disk I/O
   e. Disk controller: read physical sector
   f. Data arrives → stored in page cache
   g. Copy from page cache to user buffer
   h. Return to application
```

---

## 8. Practical Commands

### 8.1 File System Management

```bash
# Create a file system
sudo mkfs.ext4 /dev/sdb1

# Mount a file system
sudo mount /dev/sdb1 /mnt/data

# Automatic mounting (edit /etc/fstab)
# /dev/sdb1  /mnt/data  ext4  defaults  0  2

# Check file system for errors
sudo e2fsck -f /dev/sdb1

# View file system information
sudo dumpe2fs /dev/sdb1 | less

# View disk usage
df -h           # File system level
du -sh /home/*  # Directory level
```

### 8.2 inode and Block Inspection

```bash
# View inode number
ls -i hello.txt
# 42 hello.txt

# View detailed inode info
stat hello.txt
#   File: hello.txt
#   Size: 6          Blocks: 8     IO Block: 4096  regular file
# Device: 801h/2049d Inode: 42     Links: 1
# Access: (0644/-rw-r--r--)  Uid: (1000/user)   Gid: (1000/user)
# Access: 2026-02-25 10:00:00
# Modify: 2026-02-25 10:00:00
# Change: 2026-02-25 10:00:00
#  Birth: 2026-02-25 10:00:00

# View physical block locations (requires root)
sudo hdparm --fibmap hello.txt
# or
sudo filefrag -v hello.txt
#  ext:  logical_offset: physical_offset:  length:  flags:
#    0:        0..       0:    1000..   1000:      1:  last,eof
```

---

## 9. Summary

| Concept | Key Takeaway |
|---------|-------------|
| **Block** | 4 KB unit of storage — file system operates on blocks, not bytes |
| **inode** | Metadata structure per file: permissions, size, timestamps, data location |
| **Extent** | Contiguous block range — much more efficient than indirect pointers |
| **Block groups** | Locality optimization — keep related data close on disk |
| **Superblock** | Master record of file system state; backed up in multiple groups |
| **Journaling** | Write-ahead log prevents corruption on crash; replay for recovery |
| **VFS** | Abstraction layer — "everything is a file", same API for all FS types |
| **Linux vs. Windows** | `/` unified tree vs. drive letters; case-sensitive vs. insensitive; POSIX permissions vs. ACLs |
| **ext4 vs. NTFS** | Extents vs. data runs; inode vs. MFT entry; simpler permissions vs. granular ACLs |
| **Page cache** | Linux caches file data in RAM — "free" memory isn't really wasted |

Understanding file systems helps you:
- Choose the right FS for your workload (ext4 for general, XFS for large files, Btrfs for snapshots)
- Debug disk performance issues (check I/O patterns, journal mode, fragmentation)
- Understand cross-platform compatibility when working with both Linux and Windows

---

*This post is part of the **Linux Internals** series. See also: [Linux Architecture](/posts/linux-architecture/) and [Linux Virtual Memory](/posts/linux-virtual-memory/).*
