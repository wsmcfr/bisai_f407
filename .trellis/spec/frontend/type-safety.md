# Type Safety

> Type-safety and runtime validation patterns for this C firmware project.

---

## Overview

This repository is written in C on top of STM32 HAL, CMSIS, and FreeRTOS.
Type safety comes from disciplined use of:

- vendor-provided handle/config structs
- fixed-width integer types
- enums and structs for project-specific contracts
- validation at hardware and protocol boundaries

Current examples:

- `Core/Src/main.c`: clock configuration uses `RCC_OscInitTypeDef` and `RCC_ClkInitTypeDef`
- `Core/Src/usart.c`: UART/DMA state is carried in `UART_HandleTypeDef` and `DMA_HandleTypeDef`
- `Core/Src/freertos.c`: task metadata is grouped in `osThreadAttr_t`

---

## Type Organization

Type organization rules:

- vendor and HAL types stay in generated/vendor headers
- feature-specific enums/structs belong in the matching `User/*.h` file
- file-private helper structs should remain in the `.c` file when possible
- avoid exposing low-level handles unless the caller truly owns that resource

---

## Validation

There is no validation library.
Validation must be explicit:

- check HAL return values
- validate buffer lengths before parsing
- validate sensor raw ranges before using them as calibrated values
- validate timeouts when waiting for hardware events or task synchronization

---

## Common Patterns

Preferred patterns:

- use `uint8_t`, `uint16_t`, `uint32_t`, `int32_t`, etc. for protocol and sensor data
- use enums for finite state machines and error/status results
- use config structs for modules with multiple parameters
- use `const` and `volatile` deliberately and document why they are needed

---

## Forbidden Patterns

Avoid these patterns:

- using plain `int` when width matters
- casting away `const` or forcing incompatible pointer casts without explanation
- sharing ISR-updated state without `volatile`
- encoding protocol meaning in anonymous byte offsets with no named struct or constants
