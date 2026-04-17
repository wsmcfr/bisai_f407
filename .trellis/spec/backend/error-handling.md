# Error Handling

> How low-level firmware failures are handled in this project.

---

## Overview

The current firmware uses a fail-fast strategy:

- HAL initialization calls are checked immediately
- fatal initialization failures call `Error_Handler()`
- CPU fault handlers spin forever for debugger inspection
- ISR bodies delegate to HAL and do not attempt complex recovery

Examples in the current codebase:

- `Core/Src/main.c`: `HAL_RCC_OscConfig(...)` and `HAL_RCC_ClockConfig(...)` call `Error_Handler()` on failure
- `Core/Src/usart.c`: `HAL_UART_Init(...)` and `HAL_DMA_Init(...)` call `Error_Handler()` on failure
- `Core/Src/stm32f4xx_it.c`: `HardFault_Handler`, `MemManage_Handler`, and `BusFault_Handler` all trap in infinite loops

---

## Error Types

The project currently has no custom error type system.

Observed categories are:

- `HAL_StatusTypeDef` failures during initialization
- CPU exception handlers for unrecoverable faults
- runtime callback/interrupt entry points that rely on HAL to classify sub-errors

Future user modules may define local enums for recoverable application errors, but low-level init failures should still escalate to `Error_Handler()` unless a safe fallback is explicitly designed.

---

## Error Handling Patterns

Use these patterns consistently:

1. Check every HAL return value immediately.
2. Fail close to the source when system startup cannot continue safely.
3. Keep interrupt and callback code short; set flags or notify tasks instead of recovering inline.
4. Perform user-visible error reporting from task context, not from fault handlers.

Recommended future pattern for `User/*` modules:

- low-level driver returns a typed status
- application task decides whether to retry, reinitialize, or report
- unrecoverable system configuration failures call `Error_Handler()`

---

## API Error Responses

There is no network or RPC API in the current project.

For future serial protocols:

- define a deterministic frame or text format
- report recoverable errors from task context
- include enough context to debug, such as command id or parser state
- never attempt formatted reporting from `HardFault_Handler` or other fatal traps

---

## Common Mistakes

Common embedded mistakes to avoid in this codebase:

- ignoring `HAL_OK` checks during initialization
- placing blocking logic inside ISR or callback error paths
- trying to continue after a clock or peripheral init failure without a designed fallback
- hiding fatal conditions by returning silently from low-level driver init
