---
title: "Basic of Prompt Engineering"
date: 2025-02-15
description: "Fundamental techniques for effective LLM prompting"
categories: ["LLM"]
tags: ["Prompt Engineering", "Chain of Thought", "RAG"]
draft: false
---

## Overview

Prompt engineering is the art of crafting effective instructions for Large Language Models (LLMs) to achieve desired outputs.

## Core Techniques

### 1. Chain of Thought (CoT)

**Explicit CoT**: Provide step-by-step reasoning guidance.

```
Solve this problem step by step:
Q: If a train travels 120 km in 2 hours, what is its speed?

Think through:
1. Identify what we know
2. Apply the formula
3. Calculate the answer
```

**Zero-Shot CoT**: Let the model reason independently.

```
Q: If a train travels 120 km in 2 hours, what is its speed?
Let's think step by step.
```

### 2. Self-Consistency

Sample multiple outputs (~20) and vote on the most common answer.

**Parameters:**
- **Temperature**: Controls randomness (0.7-1.0 for diversity)
- **Top-K**: Limits token selection pool

```
Generate 20 solutions with temperature=0.8
→ Select most frequent answer
```

### 3. Sampling-and-Voting (Ensemble)

Use multiple models or personas:

```
As a mathematician, solve: ...
As a physicist, solve: ...
As an engineer, solve: ...
→ Combine answers
```

Smaller ensembles can outperform single large models.

### 4. ReAct (Reasoning + Action)

Interleave reasoning with actions:

```
Thought: I need to find the current weather
Action: search("weather today Seoul")
Observation: 15°C, cloudy
Thought: Now I can answer the user
Response: It's 15°C and cloudy in Seoul today.
```

### 5. Self-Evaluation

**Self-Critique:**
```
[Generate response]
Now critique your answer:
- Is it accurate?
- Is anything missing?
- How can it be improved?
[Revise based on critique]
```

**Constitutional AI:**
```
Evaluate if your response:
- Is helpful
- Is harmless
- Is honest
```

## Advanced Strategies

| Technique | Description |
|-----------|-------------|
| **RAG** | Retrieve external knowledge before generating |
| **Tree of Thought** | Explore multiple reasoning branches |
| **Plan and Solve** | Create plan first, then execute |
| **Prompt Chaining** | Sequential prompts with conditional logic |

## Output Formatting

Structure responses effectively:

| Format | Use Case |
|--------|----------|
| Lists | Step-by-step instructions |
| Tables | Comparisons, data |
| JSON | Structured data extraction |
| Markdown | Documentation |
| YAML | Configuration |

## Best Practices

1. **Be specific** - Clear, unambiguous instructions
2. **Provide examples** - Few-shot learning
3. **Set constraints** - Length, format, style
4. **Iterate** - Refine prompts based on outputs
5. **Use delimiters** - Separate sections clearly

```
### Task ###
[Your task description]

### Context ###
[Relevant background]

### Format ###
[Expected output format]
```
