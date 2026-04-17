# Quality Guidelines

> Code quality standards for low-level firmware development.

---

## Overview

This codebase is a CubeMX-generated STM32 HAL project with CMSIS-RTOS2/FreeRTOS enabled.
Quality depends less on framework abstractions and more on keeping ownership boundaries clear:

- generated code stays generated
- vendor code stays vendor code
- user logic moves into dedicated `User/App` and `User/Driver` modules
- interrupts remain thin and deterministic

The current codebase already shows the baseline boot flow in `Core/Src/main.c`, peripheral init in `Core/Src/usart.c`, and RTOS bootstrap in `Core/Src/freertos.c`.

---

## Forbidden Patterns

Never do the following:

- edit `Drivers/` or `Middlewares/` vendor sources for feature work
- put feature logic in generated files outside `USER CODE BEGIN/END` blocks
- duplicate CubeMX-managed pin, DMA, or clock configuration in random modules
- block for long periods inside ISR or HAL callback context
- ignore HAL return values during startup
- spread one hardware resource across multiple writers without a clear owner

---

## Required Patterns

Always follow these rules:

- keep system init order consistent with `Core/Src/main.c`
- keep peripheral setup inside generated init functions or dedicated driver wrappers
- add new user modules as `.c/.h` pairs under `User/`
- write Chinese comments for functions, key variables, and non-obvious logic
- use clear ownership: driver acquires data, app task decides behavior
- keep ISR bodies as dispatchers and move work to task context when possible

---

## Testing Requirements

There is no automated test harness yet.
The minimum verification for firmware changes is:

1. build the MDK-ARM project successfully
2. confirm the board boots and reaches the scheduler
3. verify the affected peripheral behavior on hardware
4. verify UART output or debugger-visible state for the changed flow

For timing-sensitive changes, also verify interrupt priority and RTOS interaction on target hardware.

---

## Code Review Checklist

Reviewers should check:

- is the change inside the correct ownership boundary (`Core` vs `User` vs vendor)?
- will CubeMX regeneration preserve the change?
- are HAL failures checked and escalated correctly?
- is ISR/callback code minimal and RTOS-safe?
- are new globals justified and documented?
- are comments sufficient for a future maintainer to understand the hardware flow?
