# State Management

> How application state is managed in this firmware project.

---

## Overview

This project uses plain C state plus RTOS primitives, not a frontend state library.
State should be owned by the narrowest possible scope:

- generated peripheral handles stay in generated peripheral modules
- task handles stay in RTOS bootstrap or the owning app module
- application data stays in `User/App` module-private state
- ISR-shared flags and buffers must be explicitly marked and synchronized

Current codebase examples:

- `Core/Src/usart.c`: `huart1` and `hdma_usart1_rx` are module-level hardware state
- `Core/Src/freertos.c`: `defaultTaskHandle` and `defaultTask_attributes` are RTOS state
- `Core/Src/main.c`: system tick progression is tied to `HAL_TIM_PeriodElapsedCallback(...)`

---

## State Categories

Use these state categories:

- local module state: `static` variables private to one `.c` file
- shared task state: exchanged through queues, notifications, semaphores, or mutexes
- hardware handle state: generated `UART_HandleTypeDef`, `DMA_HandleTypeDef`, `TIM_HandleTypeDef`, etc.
- ISR-visible state: only when necessary, with `volatile` and clear ownership

---

## When to Use Global State

Global state is acceptable only when it represents a singleton resource that already has project-wide identity, such as:

- CubeMX-generated peripheral handles
- one scheduler-owned task handle
- one clearly documented system service object

Do not make measurement values, parser buffers, or calibration parameters globally writable without a single owner module.

---

## Server State

There is no server state.
The closest equivalent is external hardware state from sensors, UART streams, or display hardware.

Treat that state as asynchronous input:

- sample it explicitly
- validate it before promoting it to application state
- separate raw readings from filtered/user-visible values

---

## Common Mistakes

Avoid these mistakes:

- sharing one buffer between DMA/ISR and task code without ownership rules
- storing feature state in `main.c` because it is easy to reach
- letting multiple modules write the same measurement or command state
- omitting `volatile` on ISR-updated flags
