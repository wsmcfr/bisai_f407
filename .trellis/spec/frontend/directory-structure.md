# Directory Structure

> How application-layer firmware code is organized in this project.

---

## Overview

This project does not have a GUI frontend.
User-facing behavior currently means RTOS task flow and serial communication.

Application code should be separated from generated hardware bootstrap code:

- `User/App/`: tasks, services, protocol parsing, formatting, state machines
- `User/Driver/`: device-level drivers that app modules depend on
- `Core/Src/freertos.c`: only task/bootstrap wiring, not application logic
- `Core/Src/main.c`: boot glue only

This separation is important because CubeMX may regenerate `Core/` files.

---

## Directory Layout

```
.
├── Core/
│   └── Src/
│       ├── main.c
│       ├── freertos.c
│       └── usart.c
├── User/
│   ├── App/
│   └── Driver/
└── bisai_f407_project.ioc
```

---

## Module Organization

Recommended structure for future features:

- one feature-facing service per module under `User/App/`
- one hardware-facing driver per module under `User/Driver/`
- `freertos.c` creates the task and calls into `User/App/` code
- ISR files stay in `Core/` and notify the owning app/driver module

For example, a scale feature would typically be split into:

- `User/Driver/hx711.c` for raw sensor access
- `User/App/weight_service.c` for filtering, tare, and formatting
- `User/App/serial_protocol.c` for command/response handling

---

## Naming Conventions

Use lowercase file names for user modules.
Keep file pairs aligned:

- `weight_service.c` / `weight_service.h`
- `serial_protocol.c` / `serial_protocol.h`
- `display_service.c` / `display_service.h`

Task entry functions and module APIs should use descriptive prefixes instead of generic names like `process()` or `handle()`.

---

## Examples

The current codebase shows the minimum separation already in place:

- `Core/Src/freertos.c`: task bootstrap is isolated from board init
- `Core/Src/usart.c`: communication hardware config is isolated from app logic
- `Core/Src/main.c`: system init order is isolated from runtime behavior
