# Component Guidelines

> How application modules and task-facing components should be built in this project.

---

## Overview

There are no UI components in the web sense.
In this repository, a "component" is an application module that owns one clear responsibility, such as:

- a measurement service
- a serial protocol handler
- a display updater
- a control task

Each component should compose low-level drivers from `User/Driver/` instead of accessing CubeMX-generated globals everywhere.

---

## Component Structure

A typical user component should have:

1. a header with public types, configuration, and API declarations
2. a source file with static state and private helpers
3. an init function
4. either a task entry function or a periodic processing function

Current baseline examples that show the style of thin entry points:

- `Core/Src/freertos.c`: `StartDefaultTask(void *argument)`
- `Core/Src/main.c`: `main(void)` and `MX_FREERTOS_Init()`
- `Core/Src/usart.c`: `MX_USART1_UART_Init(void)`

---

## Props Conventions

Use explicit C types instead of loosely coupled argument lists.

Preferred patterns:

- a config struct when a module needs multiple parameters
- fixed-width integers for protocol fields and sensor values
- `const` pointers for read-only inputs

Current codebase examples:

- `Core/Src/freertos.c`: `osThreadAttr_t` groups task attributes into a struct
- `Core/Src/usart.c`: `UART_HandleTypeDef` and `DMA_HandleTypeDef` group peripheral state
- `Core/Src/main.c`: HAL clock config uses `RCC_OscInitTypeDef` and `RCC_ClkInitTypeDef`

---

## Styling Patterns

There is no visual styling system.
The equivalent concern is user-visible formatting on serial ports or displays.

Rules for user-facing formatting:

- keep message formats stable once defined
- centralize frame/text formatting in one app module
- avoid mixing formatting code into low-level drivers

---

## Accessibility

Traditional accessibility guidance does not apply.
The embedded equivalent is making behavior easy to observe and debug:

- use deterministic serial output formats
- use clear units in displayed or transmitted values
- avoid hidden state transitions with no observable trace

---

## Common Mistakes

Avoid these mistakes:

- mixing hardware register/HAL detail into high-level task logic
- letting one module own unrelated peripherals and protocol state
- exposing mutable globals instead of a small API
- formatting UART/display output from ISR context
