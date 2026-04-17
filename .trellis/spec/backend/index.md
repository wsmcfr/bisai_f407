# Backend Development Guidelines

> Best practices for low-level firmware and platform code in this project.

---

## Overview

This repository is an STM32F407 embedded firmware project rather than a web service.
Within Trellis, the `backend` layer is used for the platform side of the firmware:

- MCU startup and clock configuration
- CubeMX-generated peripheral initialization
- HAL/CMSIS integration
- interrupt handlers and low-level driver glue
- RTOS bootstrap and hardware-facing runtime code

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Module organization and file layout | Done |
| [Database Guidelines](./database-guidelines.md) | Persistence conventions for this firmware project | Done |
| [Error Handling](./error-handling.md) | HAL failure handling, fault strategy, escalation path | Done |
| [Quality Guidelines](./quality-guidelines.md) | Code standards and generated-code boundaries | Done |
| [Logging Guidelines](./logging-guidelines.md) | UART/debug logging conventions | Done |

---

## Pre-Development Checklist

Read these files before changing low-level firmware code:

1. `directory-structure.md`
2. `quality-guidelines.md`
3. `error-handling.md`
4. `logging-guidelines.md` when adding UART/debug output
5. `database-guidelines.md` only when introducing non-volatile storage or calibration persistence

---

**Language**: All documentation should be written in **English**.
