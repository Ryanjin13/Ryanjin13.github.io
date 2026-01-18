---
title: "C Compile Process"
date: 2025-07-27
description: "C 컴파일 과정: 오브젝트 파일, 정적 라이브러리, 링킹"
categories: ["Computer Science"]
tags: ["Programming", "C"]
draft: false
---

## Overview

C 컴파일 과정을 여러 소스 파일, 오브젝트 파일, 정적 라이브러리를 사용한 실제 예제로 설명합니다.

## 1. Example Files

수학 유틸리티 라이브러리 구성:

- `math_utils.h` - 함수 선언 헤더 (add, multiply, subtract)
- `add.c`, `multiply.c`, `subtract.c` - 구현 파일
- `main.c` - 라이브러리 함수를 사용하는 메인 프로그램

## 2. Compilation to Object Files

각 소스 파일을 개별적으로 컴파일:

```bash
gcc -c add.c -o add.o
gcc -c multiply.c -o multiply.o
gcc -c subtract.c -o subtract.o
gcc -c main.c -o main.o
```

## 3. Using `nm` Tool

`nm` 명령어로 심볼 테이블 확인:

```bash
nm add.o
nm main.o
```

주요 심볼:
- **T** - 해당 오브젝트 파일에 정의된 함수
- **U** - 다른 곳에서 해결해야 하는 미정의 심볼

## 4. Using `objdump` Tool

`objdump` 주요 플래그:

| 플래그 | 설명 |
|--------|------|
| `-d` | 실행 섹션 디스어셈블 (어셈블리 코드) |
| `-r` | 재배치 엔트리 표시 (예: `R_X86_64_PLT32`) |
| `-h` | 섹션 헤더 및 메모리 레이아웃 |

```bash
objdump -d add.o
objdump -r main.o
objdump -h main.o
```

## 5. Creating Static Libraries

여러 오브젝트 파일을 하나의 정적 라이브러리로 묶기:

```bash
ar rcs libmath.a add.o multiply.o subtract.o
```

## 6. Linking Process

오브젝트와 라이브러리를 링킹하여 최종 실행 파일 생성:

```bash
gcc main.o -L. -lmath -o myprogram
```

이 단계에서 함수들에 실제 주소가 할당됩니다.

## 7. Optimization Levels

GCC 최적화 플래그:

| 플래그 | 설명 |
|--------|------|
| `-O0` | 최적화 없음 (디버깅용) |
| `-O1` | 기본 최적화 |
| `-O2` | 권장 최적화 |
| `-O3` | 공격적 최적화 |
| `-Os` | 크기 최적화 |
