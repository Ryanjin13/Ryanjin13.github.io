---
title: "Computer Performance Metrics"
date: 2024-06-25
description: "Understanding how to measure and compare computer performance"
categories: ["Computer Science"]
tags: ["Computer Structure", "Performance", "Benchmarking"]
draft: false
---

{{< katex >}}

## Overview

Performance measurement is crucial for comparing computers and optimizing systems. This post covers key metrics and analysis methods.

## The Performance Equation

### CPU Time

$$
\text{CPU Time} = \frac{\text{Instructions} \times \text{CPI}}{\text{Clock Rate}}
$$

Or equivalently:

$$
\text{CPU Time} = \text{Instruction Count} \times \text{CPI} \times \text{Clock Period}
$$

### Components

| Factor | Description |
|--------|-------------|
| Instruction Count | Total instructions executed |
| CPI | Cycles Per Instruction (average) |
| Clock Rate | Cycles per second (Hz) |
| Clock Period | Seconds per cycle |

## Performance Definition

### Execution Time

$$
\text{Performance} = \frac{1}{\text{Execution Time}}
$$

### Relative Performance

$$
\frac{\text{Performance}_A}{\text{Performance}_B} = \frac{\text{Time}_B}{\text{Time}_A} = n
$$

"A is n times faster than B"

## CPI Analysis

### Average CPI

$$
\text{CPI} = \frac{\sum_{i=1}^{n} (\text{CPI}_i \times \text{IC}_i)}{\text{Total IC}}
$$

Where:
- \\(\text{CPI}_i\\): Cycles for instruction type i
- \\(\text{IC}_i\\): Count of instruction type i

### Example CPI Calculation

| Instruction Type | CPI | Frequency |
|------------------|-----|-----------|
| ALU | 1 | 50% |
| Load | 3 | 20% |
| Store | 2 | 15% |
| Branch | 2 | 15% |

$$
\text{CPI} = 0.5(1) + 0.2(3) + 0.15(2) + 0.15(2) = 1.7
$$

## MIPS and MFLOPS

### MIPS (Million Instructions Per Second)

$$
\text{MIPS} = \frac{\text{Instruction Count}}{\text{Execution Time} \times 10^6}
$$

$$
\text{MIPS} = \frac{\text{Clock Rate}}{\text{CPI} \times 10^6}
$$

**Limitations:**
- Ignores instruction complexity
- Different ISAs not comparable
- Can be misleading

### MFLOPS (Million Floating Point Operations Per Second)

$$
\text{MFLOPS} = \frac{\text{FP Operations}}{\text{Execution Time} \times 10^6}
$$

Better for scientific computing comparison.

## Amdahl's Law

### Formula

$$
\text{Speedup} = \frac{1}{(1-f) + \frac{f}{S}}
$$

Where:
- \\(f\\): Fraction of execution time improved
- \\(S\\): Speedup of improved portion

### Key Insight

If 90% of code runs 10× faster:

$$
\text{Speedup} = \frac{1}{0.1 + \frac{0.9}{10}} = \frac{1}{0.19} = 5.26×
$$

Maximum possible speedup (if improved portion takes 0 time):

$$
\text{Speedup}_{max} = \frac{1}{1-f} = \frac{1}{0.1} = 10×
$$

### Implications

- Focus optimization on the common case
- Serial portion limits parallel speedup
- Law of diminishing returns

## Benchmarking

### Types of Benchmarks

| Type | Description | Example |
|------|-------------|---------|
| Synthetic | Artificial workloads | Dhrystone, Whetstone |
| Kernel | Small real programs | Linpack, Livermore Loops |
| Application | Full applications | SPEC CPU, Cinebench |

### SPEC Benchmarks

SPECint: Integer performance
SPECfp: Floating-point performance

$$
\text{SPEC ratio} = \frac{\text{Reference Time}}{\text{Test Time}}
$$

Geometric mean of ratios:

$$
\text{Overall Score} = \sqrt[n]{\prod_{i=1}^{n} \text{Ratio}_i}
$$

## Power and Performance

### Power Equation

$$
\text{Power} = \text{Capacitance} \times V^2 \times f
$$

### Energy per Operation

$$
\text{Energy} = \text{Power} \times \text{Time} = C \times V^2
$$

### Performance per Watt

Modern efficiency metric:

$$
\text{Efficiency} = \frac{\text{Performance}}{\text{Power}}
$$

## Comparing Systems

### Fair Comparison

Use same:
- Benchmark suite
- Compiler and flags
- Input data
- Measurement methodology

### Reporting Guidelines

1. Report complete benchmarks
2. Use geometric mean for ratios
3. Include measurement uncertainty
4. Document system configuration

## Performance Pitfalls

### Common Mistakes

| Mistake | Why It's Wrong |
|---------|----------------|
| Using MIPS | Different ISAs incomparable |
| Peak performance | Rarely achieved |
| Synthetic benchmarks | Don't reflect real use |
| Ignoring memory | Memory often bottleneck |
| Single metric | Different workloads vary |

### Best Practices

1. Use application-level benchmarks
2. Consider complete system
3. Include power consumption
4. Report variability
5. Understand workload characteristics

## Summary

| Metric | Use Case |
|--------|----------|
| Execution time | Gold standard |
| CPI | Microarchitecture analysis |
| MIPS | Quick (rough) comparison |
| MFLOPS | Scientific computing |
| SPEC | Standardized comparison |
| Perf/Watt | Mobile, datacenter |
