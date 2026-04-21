# Database Guidelines

> Persistence conventions for this firmware project.

---

## Overview

This repository does not use a database, ORM, filesystem, or non-volatile storage layer yet.
All current state is transient RAM state owned by generated HAL or RTOS modules.

Examples from the current codebase:

- `Core/Src/usart.c`: `huart1` and `hdma_usart1_rx` are runtime-only peripheral handles
- `Core/Src/freertos.c`: `defaultTaskHandle` is a runtime-only RTOS handle
- `Core/Src/main.c`: clock configuration is compiled into code, not loaded from persistent storage

If future work adds calibration data, tare offsets, or configuration retention, extend this document and implement an explicit storage module under `User/Driver/` or `User/App/`.

---

## Query Patterns

Not applicable today. Current firmware reads data directly from peripherals or from RAM-owned module state.

Preferred pattern for future persistent data:

- load once during startup
- validate before use
- expose typed getters/setters through a dedicated module
- never scatter raw flash reads across application code

---

## Migrations

Not applicable today.

Configuration evolution is currently handled by:

- updating `bisai_f407_project.ioc`
- regenerating CubeMX output when needed
- reviewing any generated diffs in `Core/`

If non-volatile storage is introduced later, define an explicit version field and migration path inside that storage module.

---

## Naming Conventions

There are no table or column names in the current project.

For any future persisted fields:

- name keys after their physical meaning, for example `tare_offset`, `scale_factor`, `uart_baudrate`
- keep units obvious in names or comments
- group related values in a versioned struct instead of unrelated standalone globals

### Convention: External Module Persistence Should Default to Volatile Apply

**What**: If a third-party device supports both volatile parameter apply and internal flash save, routine firmware boot should default to volatile apply.

**Why**: This project often recovers hardware state during startup. Repeating an external-device flash write on every reset is usually unnecessary, increases wear risk, and makes debug iterations less reversible.

**Example**:
```c
/*
 * 正确示例：
 * 常规启动默认只恢复当前上电周期的工作状态，
 * 最终固化时才临时打开 flash save。
 */
#define CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH (0U)

status = EMM42_MotorSetControlMode(&motor,
                                   EMM42_MOTOR_CONTROL_MODE_CLOSED_LOOP_FOC,
                                   false);
```

**Related**: See `quality-guidelines.md` scenario "Startup Self-Healing Config for External Motion Modules".

--- 

## Common Mistakes

Avoid these mistakes when persistence is eventually added:

- treating RAM variables as if they survive reset
- writing calibration constants directly into unrelated application files
- editing generated CubeMX files to emulate persistence
- changing persisted struct layout without a version field
- leaving an external device flash-save flag enabled in a normal boot image after one-time commissioning is already finished
