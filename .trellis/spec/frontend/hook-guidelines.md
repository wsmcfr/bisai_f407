# Hook Guidelines

> How callbacks and reusable RTOS helper entry points are used in this project.

---

## Overview

This project does not use React hooks.
The closest embedded equivalents are:

- HAL callbacks
- IRQ handlers
- RTOS task entry functions
- reusable helper functions that bridge interrupts, DMA, and tasks

These entry points should be thin and hand control to normal module functions as quickly as possible.

---

## Custom Hook Patterns

Preferred callback/helper pattern:

1. hardware event enters through ISR or HAL callback
2. callback captures minimal state or notifies a task
3. app logic runs in normal task context

Current codebase examples:

- `Core/Src/main.c`: `HAL_TIM_PeriodElapsedCallback(...)` is a thin callback
- `Core/Src/stm32f4xx_it.c`: `USART1_IRQHandler()` delegates to `HAL_UART_IRQHandler(&huart1)`
- `Core/Src/stm32f4xx_it.c`: `DMA2_Stream2_IRQHandler()` delegates to `HAL_DMA_IRQHandler(&hdma_usart1_rx)`

---

## Data Fetching

There is no server-side data fetching.
Data acquisition comes from hardware, so choose one of these patterns:

- periodic polling in a task
- DMA plus task notification
- IRQ-triggered sampling with deferred processing

Do not perform heavy filtering, logging, or protocol formatting directly inside callbacks.

---

## Naming Conventions

Naming rules are driven by the platform:

- HAL callbacks keep their required vendor names
- IRQ handlers keep the CMSIS startup names
- task entry functions should be descriptive, such as `WeightTask_Entry`
- helper functions should describe the trigger or purpose, not use vague names

---

## Common Mistakes

Avoid these mistakes:

- blocking inside ISR or HAL callback context
- calling complex formatting or transmission code directly from callbacks
- hiding shared-state writes inside multiple unrelated callbacks
- duplicating callback handling logic across files instead of routing through one owner module
