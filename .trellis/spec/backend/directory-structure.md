# Directory Structure

> How low-level firmware code is organized in this STM32F407 project.

---

## Overview

This project is generated from STM32CubeMX and currently consists of:

- `Core/`: generated startup, interrupt, peripheral-init, and RTOS bootstrap files
- `Drivers/`: vendor HAL and CMSIS code; treat as third-party code
- `Middlewares/`: third-party FreeRTOS sources; treat as third-party code
- `MDK-ARM/`: Keil project files and debug configuration
- `bisai_f407_project.ioc`: the authoritative CubeMX hardware configuration

Project-specific hand-written firmware should live under `User/`:

- `User/App/`: application services, RTOS tasks, protocol handling, business logic
- `User/Driver/`: non-vendor device drivers such as sensors, displays, or protocol adapters

Generated `Core/*` files may only contain lightweight glue in `USER CODE BEGIN/END` regions.

---

## Directory Layout

```
.
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
├── Middlewares/
├── MDK-ARM/
├── User/
│   ├── App/
│   └── Driver/
└── bisai_f407_project.ioc
```

---

## Module Organization

Use the following ownership rules:

- `Core/Src/main.c`: system entry, HAL init order, clock setup, scheduler start
- `Core/Src/usart.c`, `gpio.c`, `dma.c`: CubeMX-generated peripheral setup only
- `Core/Src/freertos.c`: RTOS object creation and task bootstrap only
- `Core/Src/stm32f4xx_it.c`: ISR entry points only; keep handlers thin
- `User/Driver/*`: hardware-specific modules with a `.c/.h` pair
- `User/App/*`: task-level orchestration, measurement logic, protocol parsing, and formatting

Do not place reusable feature logic in `main.c` or vendor directories.

---

## Naming Conventions

Current naming comes from CubeMX and should be preserved for generated modules:

- generated init functions use `MX_<PERIPHERAL>_Init`
- global HAL handles use vendor naming such as `huart1`, `hdma_usart1_rx`
- ISR names must match the startup table, for example `USART1_IRQHandler`

Hand-written modules should use lowercase file names with matching prefixes, for example:

- `hx711.c` / `hx711.h`
- `weight_service.c` / `weight_service.h`
- `serial_protocol.c` / `serial_protocol.h`

---

## Examples

The current repository already shows the expected split between boot code, peripheral init, and RTOS glue:

- `Core/Src/main.c`: boot sequence and scheduler startup
- `Core/Src/usart.c`: peripheral-specific init plus MSP wiring
- `Core/Src/freertos.c`: task creation separated from `main.c`

These examples should be treated as the baseline for future `User/App` and `User/Driver` additions.
