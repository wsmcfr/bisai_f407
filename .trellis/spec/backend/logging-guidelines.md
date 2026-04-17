# Logging Guidelines

> How debug and runtime logging should be done in this firmware project.

---

## Overview

There is no dedicated logging framework in the current repository.
The only configured communication channel is `USART1` at `115200 8N1`, so UART is the expected debug/output path when logging is added.

Current evidence:

- `Core/Src/usart.c`: `MX_USART1_UART_Init()` configures USART1 for TX/RX at 115200 baud
- `Core/Src/usart.c`: RX already has DMA and interrupt wiring, which makes UART the natural diagnostics channel
- `Core/Src/freertos.c`: no `printf` retargeting or log task exists yet

Keep logging lightweight and deterministic because this is a real-time embedded project.

---

## Log Levels

When logs are added, use these levels conceptually even if implemented as plain text prefixes:

- `DEBUG`: raw bring-up details, temporary sensor values, one-shot debug dumps
- `INFO`: boot complete, task start, successful calibration, protocol connection state
- `WARN`: dropped frame, timeout, invalid command, recoverable sensor read failure
- `ERROR`: repeated driver failure, unrecoverable protocol state, task-level fatal condition before `Error_Handler()`

---

## Structured Logging

Prefer single-line UART messages with stable prefixes, for example:

- `[INFO][WEIGHT] tare complete`
- `[WARN][UART] frame timeout`
- `[ERROR][HX711] sample timeout`

Rules:

- keep messages short
- include module name
- include numeric values only when they are actionable
- avoid dynamic allocation and large formatted buffers

---

## What to Log

Good logging candidates for this project:

- system boot milestones
- FreeRTOS task creation success/failure points
- sensor calibration and tare operations
- parser errors on the serial command channel
- state transitions that affect user-visible behavior

---

## What NOT to Log

Do not log:

- inside high-frequency interrupts except for emergency one-shot diagnostics
- every raw sample in a continuous acquisition loop
- large binary buffers through blocking UART prints
- misleading "success" messages before hardware actions are actually complete
