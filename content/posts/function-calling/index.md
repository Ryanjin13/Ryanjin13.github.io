---
title: "Function Calling"
date: 2024-10-28
description: "How LLMs interact with external tools through function calling"
categories: ["LLM"]
tags: ["Prompt Engineering", "API", "Tool Use"]
draft: false
---

## Overview

Function Calling enables LLMs to interact with external tools and APIs by generating structured outputs that can trigger real-world actions.

## How It Works

```
User Query → LLM analyzes intent → Generates function call →
Execute function → Return result → LLM generates response
```

## Defining Functions

Provide function schemas to the LLM:

```json
{
  "name": "get_weather",
  "description": "Get current weather for a location",
  "parameters": {
    "type": "object",
    "properties": {
      "location": {
        "type": "string",
        "description": "City name"
      },
      "unit": {
        "type": "string",
        "enum": ["celsius", "fahrenheit"]
      }
    },
    "required": ["location"]
  }
}
```

## Example Flow

**User:** "What's the weather in Seoul?"

**LLM Output:**
```json
{
  "function_call": {
    "name": "get_weather",
    "arguments": {
      "location": "Seoul",
      "unit": "celsius"
    }
  }
}
```

**Function Execution:** API returns `{"temp": 15, "condition": "cloudy"}`

**LLM Response:** "It's currently 15°C and cloudy in Seoul."

## Implementation (Python)

```python
import openai

functions = [
    {
        "name": "get_weather",
        "description": "Get weather for a location",
        "parameters": {
            "type": "object",
            "properties": {
                "location": {"type": "string"},
                "unit": {"type": "string", "enum": ["celsius", "fahrenheit"]}
            },
            "required": ["location"]
        }
    }
]

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{"role": "user", "content": "Weather in Seoul?"}],
    functions=functions,
    function_call="auto"
)

# Check if function call was made
if response.choices[0].message.get("function_call"):
    func_name = response.choices[0].message["function_call"]["name"]
    func_args = json.loads(response.choices[0].message["function_call"]["arguments"])

    # Execute the actual function
    result = get_weather(**func_args)

    # Send result back to LLM
    # ...
```

## Use Cases

| Application | Functions |
|-------------|-----------|
| **Assistant** | Calendar, email, reminders |
| **E-commerce** | Search products, place orders |
| **Data Analysis** | Query databases, generate charts |
| **Smart Home** | Control devices, check status |
| **Travel** | Book flights, hotels, check prices |

## Best Practices

1. **Clear descriptions** - Help LLM understand when to use each function
2. **Validate inputs** - Check arguments before execution
3. **Handle errors** - Graceful failure handling
4. **Limit scope** - Only expose necessary functions
5. **Log calls** - Monitor for debugging and security

## Parallel Function Calling

Modern LLMs can call multiple functions simultaneously:

```json
{
  "function_calls": [
    {"name": "get_weather", "arguments": {"location": "Seoul"}},
    {"name": "get_weather", "arguments": {"location": "Tokyo"}}
  ]
}
```

## Security Considerations

- Validate all function arguments
- Implement rate limiting
- Use least-privilege access
- Sanitize outputs before display
