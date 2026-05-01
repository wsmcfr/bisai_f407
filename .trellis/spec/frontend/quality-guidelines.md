# Quality Guidelines

> Code quality standards for application-layer firmware development.

---

## Overview

Application-layer quality in this project means keeping user logic deterministic, observable, and separated from generated platform code.
Most future feature work will live in `User/App/` and depend on `User/Driver/`.

---

## Forbidden Patterns

Avoid these patterns:

- putting app logic in `Core/Src/freertos.c` or `main.c` when it belongs in `User/App`
- directly editing vendor or middleware sources for application features
- sending blocking UART output from ISR/callback context
- mixing raw hardware access into protocol or business logic modules
- using unexplained magic numbers for timing, calibration, or protocol framing

---

## Required Patterns

Required patterns for new app code:

- one clear module owner for each feature
- Chinese comments for functions, important variables, and tricky branches
- descriptive APIs and configuration structs
- explicit task/context ownership for shared data
- serial/display output defined in one formatting path instead of scattered `printf` calls

### Convention: User Code Comment Contract

**What**: Hand-written firmware under `User/App/` and `User/Driver/` must carry detailed Chinese comments at the implementation boundary. `Core/Src/freertos.c` follows the same rule because this project keeps RTOS task attributes, task creation, and task-entry forwarding there.

**Why**: Future changes are usually made under contest time pressure. Explicit comments on state ownership, units, task context, and hardware assumptions prevent AI or developers from silently breaking task synchronization, calibration semantics, and driver contracts.

**Required comment points**:

- every member of every new or modified `typedef struct` must have a member-specific Chinese comment
- function definitions must explain purpose, main flow, key parameters, return value, and side effects
- function bodies must comment non-obvious local variables, branches, state transitions, resource ownership, error paths, and RTOS/ISR context rules
- HAL callbacks, IRQ-facing code, FreeRTOS tasks, DMA/UART/GPIO handoffs, and shared-state boundaries must document trigger source, execution context, and race/blocking risks
- generated files other than `Core/Src/freertos.c` should not receive broad comment-only rewrites; move real app logic into `User/` instead

**Example**:

```c
typedef struct
{
    int32_t latest_error_px;          /* 最新视觉像素误差，单位像素，正负号用于决定电机转向。 */
    uint16_t applied_speed_rpm;       /* 最近一次下发到 Emm42 的速度，单位 RPM，用于避免重复发送相同速度。 */
    uint8_t track_timeout_reported;   /* 跟踪超时日志抑制标志，避免超时期间反复刷同一条告警。 */
} ConveyorMotor_Runtime_t;

/**
 * @brief 根据视觉误差更新传送带速度命令。
 * @param runtime 电机任务私有运行状态，不能为空。
 * @retval None
 *
 * 主要流程：
 * 1. 先判断视觉输入是否超时；
 * 2. 再根据误差绝对值映射速度；
 * 3. 最后只在速度或方向变化时下发新命令，减少串口阻塞。
 */
static void ConveyorMotorService_UpdateTrack(ConveyorMotor_Runtime_t *runtime)
{
    /* 该分支处理视觉数据断流，必须先停机再记录告警，避免传送带沿旧速度继续运动。 */
    if (runtime->track_timeout_reported == 0U)
    {
        runtime->track_timeout_reported = 1U;
    }
}
```

**Related**: See backend `quality-guidelines.md` for generated-code boundaries and the `Core/Src/freertos.c` exception.

---

## Testing Requirements

There is no automated UI or integration test suite.
Minimum validation for app-layer changes:

1. build succeeds
2. firmware boots to the scheduler
3. the target task executes on hardware
4. serial/display behavior matches the expected user-visible format
5. error paths are exercised at least once when practical

---

## Code Review Checklist

Reviewers should check:

- does the change live in `User/App` or `User/Driver` instead of generated files?
- is there a clean boundary between driver code and app logic?
- is shared state owned and synchronized correctly?
- are comments sufficient to explain timing, units, and hardware assumptions?
- is user-visible output stable and easy to parse/debug?
