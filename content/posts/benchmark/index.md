---
title: "Computer Benchmarking"
date: 2024-06-25
description: "Understanding benchmarks and performance evaluation methods"
categories: ["Computer Science"]
tags: ["Computer Structure", "Benchmark", "Performance"]
draft: false
---

{{< katex >}}

## Overview

Benchmarks are standardized tests used to evaluate and compare computer system performance. Proper benchmarking is essential for making informed hardware decisions.

## Types of Benchmarks

### Synthetic Benchmarks

Artificial workloads designed to stress specific components:

| Benchmark | Measures |
|-----------|----------|
| Dhrystone | Integer performance |
| Whetstone | Floating-point performance |
| LINPACK | Dense linear algebra |
| Stream | Memory bandwidth |
| IOzone | Disk I/O |

**Pros:** Reproducible, focused
**Cons:** May not reflect real workloads

### Application Benchmarks

Real programs with defined workloads:

| Benchmark | Domain |
|-----------|--------|
| SPEC CPU | General computing |
| SPEC JBB | Java server |
| TPC-C | Database transactions |
| MLPerf | Machine learning |
| Cinebench | 3D rendering |

**Pros:** Realistic
**Cons:** Complex, many variables

### Microbenchmarks

Test specific operations:

```c
// Memory latency test
for (int i = 0; i < N; i++) {
    p = *p;  // Pointer chasing
}
// Measures cache/memory latency
```

## SPEC Benchmarks

### SPEC CPU 2017

**Integer (SPECint):**
- Compression (gcc, xz)
- Simulation (mcf, omnetpp)
- AI/search (deepsjeng)

**Floating-Point (SPECfp):**
- Physics simulation
- Computational chemistry
- Weather modeling

### Calculating SPEC Score

$$
\text{Ratio} = \frac{\text{Reference Time}}{\text{System Time}}
$$

Overall score (geometric mean):

$$
\text{Score} = \sqrt[n]{\prod_{i=1}^{n} \text{Ratio}_i}
$$

### Why Geometric Mean?

- Normalizes different scales
- Prevents domination by outliers
- Symmetric for speedups and slowdowns

## Memory Benchmarks

### Bandwidth (Stream)

```
Copy:  a[i] = b[i]
Scale: a[i] = q * b[i]
Add:   a[i] = b[i] + c[i]
Triad: a[i] = b[i] + q * c[i]
```

Reports GB/s for each operation.

### Latency

Measure time to access memory at various depths:

| Level | Typical Latency |
|-------|-----------------|
| L1 cache | ~1 ns |
| L2 cache | ~4 ns |
| L3 cache | ~12 ns |
| DRAM | ~60-100 ns |

## Graphics Benchmarks

| Benchmark | Focus |
|-----------|-------|
| 3DMark | Gaming graphics |
| SPECviewperf | Professional graphics |
| Unigine | GPU stress testing |
| FurMark | GPU thermal testing |

## Storage Benchmarks

### Metrics

| Metric | Description |
|--------|-------------|
| IOPS | I/O Operations Per Second |
| Throughput | MB/s transfer rate |
| Latency | Time per operation |

### Tools

- fio (Flexible I/O Tester)
- CrystalDiskMark
- ATTO Disk Benchmark

## Benchmark Methodology

### Best Practices

1. **Warm-up:** Run benchmark once before measuring
2. **Multiple runs:** Report mean and variance
3. **Controlled environment:** Minimal background processes
4. **Full system:** Include OS, drivers, compiler

### Common Mistakes

| Mistake | Why It's Wrong |
|---------|----------------|
| Single run | Statistical noise |
| Peak performance | Rarely achieved |
| Incomparable tests | Different configurations |
| Cherry-picking | Biased results |

## Reporting Results

### What to Include

```
System Configuration:
- CPU: Intel Core i7-12700K @ 4.9 GHz
- RAM: 32 GB DDR5-5600
- OS: Ubuntu 22.04
- Compiler: gcc 12.1 -O3

Results (mean ± std, n=10):
- Test A: 1234 ± 12 units
- Test B: 5678 ± 45 units
```

### Statistical Validity

$$
\text{CI} = \bar{x} \pm t_{\alpha/2} \cdot \frac{s}{\sqrt{n}}
$$

Report 95% confidence intervals when possible.

## Benchmark Suites

### SPEC Suites

| Suite | Application |
|-------|-------------|
| SPEC CPU | Processor |
| SPEC Power | Energy efficiency |
| SPEC JBB | Java business |
| SPEC Cloud | Cloud computing |

### TPC (Transaction Processing)

| Benchmark | Workload |
|-----------|----------|
| TPC-C | OLTP |
| TPC-H | Decision support |
| TPC-DS | Big data analytics |

### MLPerf

- Training benchmarks
- Inference benchmarks
- Edge device benchmarks

## Interpreting Results

### Performance per Dollar

$$
\text{Value} = \frac{\text{Performance}}{\text{Price}}
$$

### Performance per Watt

$$
\text{Efficiency} = \frac{\text{Performance}}{\text{Power}}
$$

### Total Cost of Ownership

$$
\text{TCO} = \text{Acquisition} + \text{Operation} + \text{Maintenance}
$$

## Summary

| Benchmark Type | Best For |
|----------------|----------|
| Synthetic | Component testing |
| Application | Real-world performance |
| Microbenchmark | Specific analysis |
| Standardized | Fair comparison |
