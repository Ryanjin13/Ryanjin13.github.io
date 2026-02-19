---
title: "Vision-Language-Action Models: From CLIP to Humanoid Robots"
date: 2026-02-19
description: "A comprehensive timeline and technical analysis of VLA models — how vision-language models evolved into robot controllers, from RT-2 coining the paradigm to pi-0.5 and Gemini Robotics 1.5"
categories: ["Robotics"]
tags: ["VLA", "Vision-Language-Action", "Foundation Model", "Robot Learning", "RT-2", "OpenVLA", "pi-0"]
draft: false
---

{{< katex >}}

## Overview

A **Vision-Language-Action (VLA)** model is a foundation model that takes **camera images** and **language instructions** as input and directly outputs **robot actions**. The core insight: if a large language model can generate text token by token, it can also generate robot actions token by token — provided it has been trained on enough visual and embodied data.

This post traces the full history of VLA models from their precursors to the latest 2025–2026 developments, and analyzes the architectural patterns and challenges shaping the field.

---

## 1. The Precursors: Vision-Language Models (2021–2022)

Before VLAs existed, a series of breakthroughs in **vision-language modeling** laid the groundwork.

### CLIP (OpenAI, January 2021)

CLIP (Contrastive Language-Image Pre-training) was the watershed moment. Trained on 400 million image-text pairs using contrastive learning, it aligned visual and textual representations in a shared embedding space. CLIP became the **foundational vision encoder** used in nearly every subsequent VLA model.

```
Image ──→ [Vision Encoder] ──→ Image Embedding ─┐
                                                  ├──→ Cosine Similarity
Text  ──→ [Text Encoder]  ──→ Text Embedding  ─┘

Training: maximize similarity for matching pairs,
          minimize for non-matching pairs
```

### DINOv2 (Meta, 2023)

A self-supervised vision model that provides rich spatial and geometric understanding — complementary to CLIP's semantic understanding. Many modern VLAs (OpenVLA, SmolVLA) **fuse CLIP/SigLIP + DINOv2** for the best of both worlds.

---

## 2. Language Meets Robotics (2022–2023)

### SayCan (Google, April 2022)

The first major work connecting LLMs to physical robots. SayCan used a **modular approach**: an LLM (PaLM) scored which actions were *useful* ("Say"), while learned affordance functions scored which actions were *feasible* ("Can"). The system multiplied these probabilities to select executable skills.

```
User: "I spilled my drink, can you help?"

PaLM (LLM):                     Affordance Model:
┌────────────────────┐          ┌────────────────────┐
│ "Find a sponge"  0.8│          │ "Find a sponge"  0.3│
│ "Get a towel"    0.7│    ×     │ "Get a towel"    0.9│
│ "Mop the floor"  0.5│          │ "Mop the floor"  0.1│
└────────────────────┘          └────────────────────┘

Combined:
  "Get a towel"    = 0.7 × 0.9 = 0.63  ← Selected!
  "Find a sponge"  = 0.8 × 0.3 = 0.24
  "Mop the floor"  = 0.5 × 0.1 = 0.05
```

SayCan proved that LLM knowledge could be grounded in physical capabilities. But it could not *see* — it relied entirely on language.

### RT-1: Robotics Transformer (Google, December 2022)

The first large-scale multi-task robot transformer. Trained on **130,000 real robot episodes** covering 700+ tasks, collected from 13 Everyday Robots over 17 months.

```
Camera Image ──→ EfficientNet (with FiLM conditioning) ──→ TokenLearner ──→ Transformer ──→ Action Tokens
                        ↑                                                         │
Language Instruction ───┘                                                    7-DoF arm +
                                                                             3-DoF base +
                                                                             mode switching
```

RT-1 achieved **97% success on seen tasks** and **76% on unseen tasks** — demonstrating that large-scale, multi-task robot learning was viable.

### PaLM-E (Google, March 2023)

A 562B parameter model that injected continuous sensor observations directly into PaLM's language embedding space. The key idea: map images and robot states into vectors with the **same dimensionality as word token embeddings**, creating "multimodal sentences."

```
["Pick up the", <image_tokens>, "from the", <robot_state_tokens>, "and place it on the table"]
                     ↑                            ↑
               ViT encoder                  State encoder
            (continuous embeddings mixed with text tokens)
```

PaLM-E demonstrated **positive transfer**: training jointly on internet-scale data *and* robotics data improved performance on both.

---

## 3. The VLA Paradigm Emerges (Mid-2023)

### RT-2: The Paper That Named the Paradigm (Google DeepMind, July 2023)

**RT-2** formally established "Vision-Language-Action" as a concept. The key insight was remarkably simple:

> Robot actions can be represented as strings of numbers in the same token vocabulary as language.

Google took state-of-the-art VLMs (PaLI-X, PaLM-E) and fine-tuned them on robot demonstration data so they could output robot actions as text tokens:

```
Standard VLM:
  Input:  [Image] + "What is in this image?"
  Output: "A red cup on a table"

RT-2 (VLA):
  Input:  [Image] + "Pick up the red cup"
  Output: "1 128 91 241 5 101 127"
           ↑ Discretized robot actions (position, rotation, gripper)
```

Results: RT-2 **doubled performance on novel scenarios** to 62% (vs. RT-1's 32%) across 6,000+ robotic trials. The web-scale pre-training gave the model emergent capabilities — it could follow instructions involving concepts never seen in robot data (e.g., "move the banana to the country that starts with U" → picks up banana, places it on a picture of the USA).

### Open X-Embodiment & RT-2-X (October 2023)

A massive collaboration between **21 institutions** pooling robot data:

```
Open X-Embodiment Dataset:
├── 60 existing robot datasets
├── 34 labs worldwide
├── 22 robot embodiments (arms, bimanual, quadrupeds)
├── 527 skills
├── 160,266 tasks
└── 1,000,000+ trajectories
```

**RT-2-X** (RT-2 trained on this cross-embodiment mixture) achieved **3x improvement on emergent skills**, including spatial reasoning ("on" vs. "near") and cross-embodiment transfer.

---

## 4. The Open-Source Wave (2024)

### Octo (UC Berkeley / Stanford / CMU, May 2024)

One of the first major **open-source** generalist robot policies.

```
Architecture:
Camera Images ──→ Transformer Encoder ──→ Latent Representation ──→ Diffusion Decoder ──→ Actions
       ↑                                                                    ↑
Language / Goal Image                                              Smooth, multi-modal
(flexible conditioning)                                            action distributions
```

- Two sizes: Octo-Small (27M) and Octo-Base (93M parameters)
- Pretrained on 800,000 episodes from Open X-Embodiment
- Can be fine-tuned to new robots in a few hours on consumer GPUs

### OpenVLA (Stanford, June 2024)

The most influential open-source VLA — a **7B parameter** model that outperformed the 55B RT-2-X by **16.5% absolute success rate**.

```
┌─────────────┐
│   SigLIP    │──→ Visual
│   Encoder   │    Tokens  ──┐
├─────────────┤              ├──→ [Llama-2 7B] ──→ Discretized Action Tokens
│   DINOv2    │──→ Visual        ↑                   (256 bins per dimension)
│   Encoder   │    Tokens  ──┘   │
└─────────────┘                  │
                    Language Instruction
```

OpenVLA proved that with the right architecture, a 7B model could beat a 55B model — efficiency matters more than raw scale.

### pi-0 (Physical Intelligence, October 2024)

The breakthrough that changed action generation. Founded by Chelsea Finn, Sergey Levine, and others, Physical Intelligence introduced **flow matching** for action generation:

```
Previous VLAs (discrete tokens):
  Action = [token_1, token_2, ..., token_7]  ← One action at a time

pi-0 (flow matching):
  Action Chunk = [a_1, a_2, ..., a_50]  ← 50 actions at once!
                  at 50Hz               ← Smooth, continuous control
```

Instead of predicting single discrete actions, pi-0 generates **action chunks of 50 actions at 50Hz** using flow matching — enabling smooth, continuous control critical for dexterous tasks like laundry folding, table bussing, and box assembly.

### pi-0-FAST (Late 2024)

An innovation using the **Discrete Cosine Transform (DCT)** to compress action chunks:

```
Time-domain actions  ──→  DCT  ──→  Frequency-domain coefficients  ──→  Sparse integer tokens
[a_1, a_2, ..., a_50]           [c_1, c_2, ..., c_50]                  Most are ~0, few are kept
                                 (energy concentrated                    → Fast token generation
                                  in low frequencies)
```

This allowed faster, more efficient generation while maintaining smooth control.

---

## 5. The Production Era (2025–2026)

### Helix (Figure AI, February 2025)

The **first VLA for full-body humanoid control** — coordinating 35 degrees of freedom at 200Hz.

```
Dual-System Architecture:
┌──────────────────────────────────┐
│  System 2 (Slow Thinking)        │
│  Internet-pretrained VLM         │  ← Scene understanding
│  Scene understanding             │     Language comprehension
│  Language comprehension          │     Runs at lower frequency
└──────────────┬───────────────────┘
               │ Latent context
               ▼
┌──────────────────────────────────┐
│  System 1 (Fast Thinking)        │
│  Visuomotor policy               │  ← Real-time motor control
│  35-DoF at 200Hz                 │     Individual finger control
│  Fingers, arms, torso, head      │     End-effector trajectories
└──────────────────────────────────┘
```

Helix is the first VLA to operate two robots simultaneously for shared tasks, and it is **commercially deployed at BMW factories**.

### GR00T N1 / N1.5 / N1.6 (NVIDIA, March–Late 2025)

NVIDIA's open, customizable foundation model for humanoid robots:

| Version | Key Advancement |
|---------|----------------|
| **N1** | 2.2B params, Eagle-2 VLM + DiT flow-matching, 120Hz actions |
| **N1.5** | Synthetic data training (36 hours vs. 3 months manual), FLARE (learning from human videos) |
| **N1.6** | Cosmos Reason integration, full-body humanoid control |

GR00T N1.5's most striking result: NVIDIA generated training data in **36 hours** using their Cosmos synthetic data pipeline, versus approximately 3 months of manual human data collection for N1.

### Gemini Robotics (Google DeepMind, March 2025)

Built on Gemini 2.0, extending multimodal capabilities to physical actions:

```
Gemini Robotics Architecture:
┌──────────────────────────────────┐
│  Gemini Robotics-ER              │
│  (Embodied Reasoning)            │  ← The "brain"
│  High-level planning             │     Multi-language understanding
│  Conversational understanding    │     Continuous monitoring
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│  Gemini Robotics VLA             │
│  Low-level motor control         │  ← The "motor cortex"
│  Dexterous manipulation          │     Origami folding capable
└──────────────────────────────────┘
```

**Gemini Robotics 1.5 (October 2025)** introduced two groundbreaking features:
- **Embodied Thinking**: Interleaving actions with multi-level internal natural language reasoning
- **Motion Transfer**: Skills trained on one robot (e.g., ALOHA 2) successfully transfer to entirely different platforms (Franka arm, Apptronik Apollo humanoid) **without fine-tuning**

### pi-0.5 (Physical Intelligence, April 2025)

Addressed the critical **generalization gap**. Unlike previous VLAs evaluated in training-like environments, pi-0.5 generalizes to *entirely new environments* — cleaning kitchens and bedrooms in homes never seen during training, performing 10–15 minute multi-stage behaviors.

### SmolVLA (HuggingFace, June 2025)

A **450M parameter** open-source VLA proving bigger is not always better. Despite using fewer than 30,000 training episodes, SmolVLA matches or exceeds OpenVLA and pi-0. Runs on CPUs and consumer GPUs, including MacBooks.

---

## 6. Architecture Patterns: The Canonical VLA

Nearly all VLAs follow a three-component architecture:

```
┌──────────────────────────────────────────────────────────────┐
│                     VLA Architecture                          │
│                                                              │
│  ┌─────────────────┐                                         │
│  │ Vision Encoder   │  CLIP, SigLIP, DINOv2, Eagle-2        │
│  │ (frozen/tuned)   │  Encodes images → visual tokens        │
│  └────────┬────────┘                                         │
│           │                                                  │
│  ┌────────┴────────────────────────────┐                     │
│  │      Language Model Backbone         │                     │
│  │  Llama-2, PaliGemma, PaLM-E, etc.  │                     │
│  │                                      │                     │
│  │  Processes visual tokens +           │                     │
│  │  language tokens as a single         │                     │
│  │  multimodal sequence                 │                     │
│  └────────┬────────────────────────────┘                     │
│           │                                                  │
│  ┌────────┴────────────────────────────┐                     │
│  │      Action Decoder                  │                     │
│  │                                      │                     │
│  │  Option A: Discrete Token Output     │  RT-2, OpenVLA     │
│  │    Actions as text tokens (256 bins) │                     │
│  │                                      │                     │
│  │  Option B: Flow Matching / Diffusion │  pi-0, GR00T N1    │
│  │    Continuous actions at 50-200Hz    │  Octo               │
│  └──────────────────────────────────────┘                     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### Vision Encoder Choices

| Encoder | Strength | Used By |
|---------|----------|---------|
| **CLIP / SigLIP** | Semantic understanding (what objects are) | OpenVLA, RT-2, pi-0 |
| **DINOv2** | Spatial/geometric understanding (where objects are) | OpenVLA, SmolVLA |
| **Fused (SigLIP + DINOv2)** | Both semantic and spatial | OpenVLA, SmolVLA |
| **Eagle-2** | NVIDIA proprietary, integrated reasoning | GR00T N1 |

### Action Representation Comparison

| Method | Resolution | Frequency | Used By |
|--------|-----------|-----------|---------|
| **Discrete tokens** (single-step) | 256 bins | ~3Hz | RT-2, OpenVLA |
| **Action chunking** (flow matching) | Continuous | 50Hz | pi-0 |
| **DCT-compressed tokens** | Continuous → Discrete | 50Hz | pi-0-FAST |
| **Diffusion decoding** | Continuous | ~10Hz | Octo |
| **DiT flow matching** | Continuous | 120–200Hz | GR00T N1, Helix |

---

## 7. The Dual-System Trend (2025 Standard)

Inspired by Kahneman's dual-process theory of human cognition, nearly all 2025 VLAs adopt a **fast/slow architecture**:

```
┌────────────────────────────────────────────────┐
│  System 2: "Thinking" (Slow, Deliberate)        │
│                                                │
│  - Large VLM backbone                           │
│  - Scene understanding & reasoning              │
│  - Language comprehension                       │
│  - High-level planning                          │
│  - Runs at 1-10 Hz                              │
├────────────────────────────────────────────────┤
│  System 1: "Acting" (Fast, Reactive)            │
│                                                │
│  - Lightweight action policy                    │
│  - Flow matching / diffusion                    │
│  - Real-time motor control                      │
│  - Runs at 50-200 Hz                            │
│  - Reacts to physical perturbations             │
└────────────────────────────────────────────────┘
```

This pattern appears across: **Helix** (S1/S2), **GR00T N1** (VLM + DiT), **Gemini Robotics 1.5** (ER + VLA).

---

## 8. Current Trends (2025–2026)

### 8.1 The Humanoid Race

Every major player is targeting humanoid robots: Figure AI (Helix), NVIDIA (GR00T), Google DeepMind (Gemini Robotics on Apptronik Apollo), Tesla (Optimus Gen 3). VCs invested **$7.2 billion in robotics in 2025**, up from $3.1 billion in 2023.

### 8.2 Synthetic Data and World Models

NVIDIA's Cosmos platform uses **world foundation models** to generate training data synthetically — GR00T N1.5 training data was generated in 36 hours vs. 3 months of manual collection. This addresses the fundamental data scarcity bottleneck.

### 8.3 RL Post-Training for VLAs

A major 2025 trend: applying **Reinforcement Learning from Verifiable Rewards (RLVR)** to improve pre-trained VLAs:
- **VLA-R1**: RLVR + Group Relative Policy Optimization, +17.8% affordance perception
- **VLA-RL**: VLM as process reward model
- **SimpleVLA-RL**: Significant sim-to-real transfer improvements

### 8.4 Compact / Efficient VLAs

A parallel track toward deployability: SmolVLA (450M), MiniVLA (1B), Gemini Robotics On-Device (adapts with 50–100 demos). Critical for commercial deployment where robots need low-latency, on-board inference.

---

## 9. Challenges and Future Directions

### The Data Bottleneck

Foundation models for language train on \(\sim 10^{9}\) samples; the largest robotics dataset (Open X-Embodiment) has \(\sim 10^{6}\) episodes — three orders of magnitude smaller. Collecting robot trajectories requires physical setups, diverse objects, and skilled teleoperators.

### Generalization

Most VLAs are evaluated in environments matching training. pi-0.5 made progress on open-world generalization but acknowledged persistent challenges: unfamiliar hardware, partial observability, and high-level reasoning errors.

### Safety

VLMs hallucinate, and in robotics this means potential collisions or unsafe actions. No current system offers certifiable hard safety guarantees.

### Long-Horizon Tasks

Real-world deployment requires 10–15 minute multi-stage behaviors. This demands better high-level planning, memory, and error recovery — areas where current models still struggle.

---

## 10. Summary Timeline

```
2021  CLIP          ─── Vision-language alignment at scale
      │
2022  SayCan        ─── LLM grounded in physical affordances
      RT-1          ─── First large-scale robot transformer
      │
2023  PaLM-E        ─── 562B embodied multimodal LLM
      RT-2          ─── VLA paradigm coined ★
      RT-2-X        ─── Cross-embodiment transfer (21 labs)
      │
2024  Octo          ─── First major open-source robot policy
      OpenVLA       ─── 7B beats 55B (open-source)
      pi-0          ─── Flow matching for continuous control
      pi-0-FAST     ─── DCT compression for action tokens
      │
2025  Helix         ─── First humanoid VLA (35-DoF @ 200Hz)
      GR00T N1/1.5  ─── Open humanoid model + synthetic data
      Gemini Robo.  ─── Gemini 2.0 extended to physical actions
      pi-0.5        ─── Open-world generalization
      SmolVLA       ─── 450M params, runs on MacBooks
      │
2026  Commercial deployment at scale (BMW, Mercedes-Benz)
      RL post-training becoming standard
      World models for scalable data generation
```

The VLA paradigm — treating robot control as a language modeling problem — has matured remarkably fast. From RT-2 coining the term in July 2023 to humanoid robots deployed in BMW factories less than two years later, the field is moving at an unprecedented pace. The convergence of foundation models, synthetic data generation, and RL post-training suggests that truly general-purpose robots may be closer than many expected.
