# Quality Guidelines

> Code quality standards for low-level firmware development.

---

## Overview

This codebase is a CubeMX-generated STM32 HAL project with CMSIS-RTOS2/FreeRTOS enabled.
Quality depends less on framework abstractions and more on keeping ownership boundaries clear:

- generated code stays generated
- vendor code stays vendor code
- user logic moves into dedicated `User/App` and `User/Driver` modules
- interrupts remain thin and deterministic

The current codebase already shows the baseline boot flow in `Core/Src/main.c`, peripheral init in `Core/Src/usart.c`, and RTOS bootstrap in `Core/Src/freertos.c`.

---

## Forbidden Patterns

Never do the following:

- edit `Drivers/` or `Middlewares/` vendor sources for feature work
- put feature logic in generated files outside `USER CODE BEGIN/END` blocks
- duplicate CubeMX-managed pin, DMA, or clock configuration in random modules
- block for long periods inside ISR or HAL callback context
- ignore HAL return values during startup
- spread one hardware resource across multiple writers without a clear owner

---

## Required Patterns

Always follow these rules:

- keep system init order consistent with `Core/Src/main.c`
- keep peripheral setup inside generated init functions or dedicated driver wrappers
- add new user modules as `.c/.h` pairs under `User/`
- write Chinese comments for functions, key variables, and non-obvious logic
- use clear ownership: driver acquires data, app task decides behavior
- keep ISR bodies as dispatchers and move work to task context when possible

### Convention: STM32 FreeRTOS Engineering Baseline

**What**: In STM32 FreeRTOS projects, prioritize functional correctness, thread safety, real-time behavior, and bounded resource usage before convenience or quick implementation.

**Why**: Firmware defects are usually caused by unclear ownership, unsafe sharing, poor task priority decisions, and uncontrolled memory use rather than by missing syntax or framework knowledge.

**Example**:
```c
/*
 * 正确示例：
 * 1. 中断只做采样和事件通知；
 * 2. 任务中完成复杂处理；
 * 3. 串口外设只允许一个发送入口持有互斥锁；
 * 4. 任务间优先通过消息或事件解耦。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart->Instance == USART1)
    {
        /* 中断里只保存必要结果并通知任务，不做格式化打印或复杂解析。 */
        g_uart_rx_length = size;
        xSemaphoreGiveFromISR(g_uart_rx_ready_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void UartCommandTask(void *argument)
{
    for (;;)
    {
        if (xSemaphoreTake(g_uart_rx_ready_sem, portMAX_DELAY) == pdTRUE)
        {
            /* 任务上下文中再完成命令解析、状态机处理和串口回包。 */
            UartCommand_ProcessFrame();
        }
    }
}
```

**Related**: See `error-handling.md` for HAL failure escalation and the scenario sections below for executable RTOS constraints.

### Scenario: FreeRTOS Shared Resource and Task Design

#### 1. Scope / Trigger

- Trigger: adding or modifying any FreeRTOS task, queue, semaphore, mutex, event group, timer, or ISR-to-task handoff
- Trigger: introducing shared global data, shared peripherals, or module state accessed by more than one execution context
- Trigger: adding stack-consuming buffers, DMA frame caches, or long-running work inside a high-priority task

#### 2. Signatures

- Task entry:
  - `void XxxTask(void *argument)`
- ISR-to-task notification APIs:
  - `xSemaphoreGiveFromISR(...)`
  - `xQueueSendFromISR(...)`
  - `vTaskNotifyGiveFromISR(...)`
  - `xEventGroupSetBitsFromISR(...)`
- Task-context synchronization APIs:
  - `xSemaphoreTake(...)`
  - `xSemaphoreGive(...)`
  - `xQueueSend(...)`
  - `xQueueReceive(...)`
  - `xEventGroupWaitBits(...)`
- Shared peripheral serialization example:
  - `HAL_UART_Transmit(...)` protected by a mutex-owned wrapper such as `my_printf(...)`

#### 3. Contracts

- Every shared resource must have a clearly documented owner:
  - owner task or owner module
  - legal readers/writers
  - synchronization primitive used at the boundary
- Prefer message passing over direct shared-state coupling:
  - command frames, sampled data, and state transitions should first consider queue or notification based transfer
  - use mutexes only when a resource truly requires serialized exclusive access
- High-priority tasks must not perform uncontrolled blocking work:
  - no long polling loops without timeout reasoning
  - no large formatted output bursts
  - no lengthy retry loops on shared peripherals
- ISR code must stay short and deterministic:
  - allowed: copy minimal data, capture status, release ISR-safe signal
  - forbidden: formatted printing, long parsing, calibration logic, or multi-step state machines
- Memory use must be justified before task creation:
  - stack size must be derived from call depth, local buffers, and library usage
  - large frame buffers should prefer static/module storage over oversized task stacks
  - do not allocate large local arrays inside frequently scheduled tasks without RAM accounting
- Modules must be easy to inspect and trace:
  - interfaces are single-purpose
  - naming reflects layer and ownership
  - error paths return or log actionable status
  - state changes are externally observable through logs, return codes, or debug variables

#### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| Shared data access | owner and sync primitive are documented | race condition risk | define ownership and replace ad-hoc access with queue/mutex/semaphore design |
| Shared peripheral access | one serialized write path or one owner task | concurrent register/HAL access risk | add wrapper, mutex, or dedicated peripheral task |
| Task priority design | priority matches deadline and workload | starvation or jitter risk | re-evaluate priority and move non-real-time work to lower priority |
| High-priority task body | bounded execution time | real-time deadline may be broken | remove long print, polling, or heavy computation |
| ISR body length | only minimal work and notification | interrupt latency increases | move logic to task context |
| Task stack sizing | stack estimate is explained | overflow or wasted RAM risk | measure high-water mark and resize with evidence |
| Local buffer usage | no unjustified large stack arrays | hidden stack blowup | move buffers to static/module storage |
| Sync primitive choice | matches actual ownership model | deadlock, inversion, or unnecessary coupling | replace with queue/event/notification or shorten lock scope |

#### 5. Good / Base / Bad Cases

- Good:
  - UART RX ISR only stores frame length and wakes a parser task
  - UART TX goes through one mutex-protected output wrapper
  - weight sampling task and command task exchange data via explicit interface
  - task stack size is chosen from buffer size and call path, not guessed
- Base:
  - a shared flag exists between ISR and task
  - but the flag is single-writer, documented, and paired with an ISR-safe notification
  - the access pattern may be accepted if timing and ownership stay simple
- Bad:
  - multiple tasks call the same peripheral transmit API without serialization
  - ISR performs `printf`, parsing, calibration, or blocking HAL operations
  - high-priority task contains unbounded loops or long delays while holding a lock
  - large temporary arrays are placed on task stacks without RAM budgeting

#### 6. Tests Required

- Concurrency design review:
  - assert that each shared resource has an owner and a synchronization rule
  - assert that the chosen primitive matches the producer/consumer pattern
- Task execution review:
  - assert that high-priority tasks contain only bounded work
  - assert that long formatting, calibration, or logging runs in lower-priority task context
- Memory review:
  - assert that each task stack has a sizing reason
  - assert that large buffers are counted in RAM budget
  - assert that no accidental large local arrays were introduced
- ISR review:
  - assert that ISR code only captures data and signals tasks
  - assert that non-ISR-safe APIs are not used in interrupt context
- Hardware validation:
  - on real hardware, verify that the scheduler stays stable, command handling does not starve sampling, and shared peripheral output remains coherent

#### 7. Wrong vs Correct

##### Wrong

- Let multiple tasks directly call one UART handle because "it usually works".
- Put parsing, floating-point formatting, or retry loops inside ISR/callback context.
- Increase task stack size blindly after overflow symptoms instead of checking buffer ownership and actual call depth.
- Use a mutex for every interaction even when one-way message passing would remove coupling.

##### Correct

- Assign one owner for each shared peripheral or protect the shared access path with a clearly bounded mutex.
- Keep ISR as a minimal producer and move complex logic into a task.
- Choose task priorities according to deadline sensitivity and keep high-priority code paths bounded.
- Size stacks and buffers from evidence, then confirm with runtime observation.
- Prefer queue, notification, or event-based decoupling before introducing shared writable state.

### Scenario: UART Command Ownership and Sensor Service Dispatch

#### 1. Scope / Trigger

- Trigger: adding new UART text commands that affect more than one `User/App` service
- Trigger: reusing one `DMA + IDLE` UART RX path across multiple FreeRTOS tasks
- Trigger: introducing calibration or inspection modes where a serial command arms one task and another task performs the actual sensor workflow

#### 2. Signatures

- UART RX single-consumer entry:
  - `uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size, uint32_t timeout_ms)`
- Module command dispatch entry:
  - `uint8_t XxxService_HandleCommand(const char *command_buffer)`
- Current LDC service command entry:
  - `uint8_t Ldc1614Service_HandleCommand(const char *command_buffer)`
- Current LDC calibration commands:
  - `LDCCAL CH1`
  - `LDCCAL CH2 20`
  - `LDCSTOP`

#### 3. Contracts

- `USART1` RX command frames must have exactly one task-context consumer:
  - one owner calls `UartCommand_Fetch(...)`
  - other services must not also fetch directly from the same UART command cache
- Feature modules that need UART commands must expose a dispatch interface instead of competing for RX ownership:
  - the owner task normalizes the command
  - the owner task forwards recognized subcommands to module handlers such as `Ldc1614Service_HandleCommand(...)`
- Calibration/inspection commands must arm a service-local session rather than performing the whole procedure inside the command owner:
  - command task validates format and starts the session
  - sensor task executes settle, sample, statistics, and result reporting in its own runtime context
- LDC calibration sampling semantics are:
  - one target channel per session
  - one physical placement of the part
  - wait for the normal settle phase
  - then capture `N` stable samples from that same placement
  - do not interpret `LDCCAL CHx 20` as “place the part 20 times”
- Public headers that expose fixed-width integer types must include `<stdint.h>` themselves:
  - do not rely on transitive includes from unrelated HAL or application headers

#### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| UART RX ownership | exactly one consumer calls `UartCommand_Fetch(...)` | commands may be stolen or lost between tasks | move parsing to one owner task and dispatch by handler function |
| Module command dispatch | non-owner modules expose `HandleCommand(...)`-style entry | service is tightly coupled to UART driver | add command forwarding interface |
| Calibration command format | channel and optional count parse deterministically | invalid session arming or silent wrong target | reject with explicit error and usage hint |
| Calibration session semantics | one placement produces multiple stable samples | operator may re-place part between samples and corrupt statistics | document command behavior and only count samples after settle |
| Early part removal during calibration | partially collected statistics are discarded or restarted | mean/reference becomes invalid | clear session progress and wait for a new valid placement |
| Public header self-sufficiency | headers compile with their own direct includes | unrelated include-order dependency | add `<stdint.h>` or other direct dependency to the header |

#### 5. Good / Base / Bad Cases

- Good:
  - `WeightService_Task` remains the only UART command consumer
  - `Ldc1614Service_HandleCommand(...)` only arms or stops a calibration session
  - `Ldc1614Service_Task` performs settle, stable sample capture, and summary output itself
  - `LDCCAL CH1 20` means “single placement, 20 stable samples”
- Base:
  - a second module needs serial commands
  - but it receives them through the existing command owner task
  - and only exposes a narrow handler function
- Bad:
  - two tasks both call `UartCommand_Fetch(...)`
  - a command callback performs long sensor sampling inline
  - “20 samples” is implemented as forcing the operator to place the same part 20 separate times without documenting it
  - a public header uses `uint8_t` or `uint16_t` without including `<stdint.h>`

#### 6. Tests Required

- Build contract test:
  - assert that every public header with fixed-width integer types compiles after including its own declared dependencies
- UART ownership review:
  - assert that only one task consumes `UartCommand_Fetch(...)`
  - assert that other modules use dispatch handlers instead of direct UART fetch
- Command parser test:
  - assert that `LDCCAL CH1`, `LDCCAL CH2 20`, and `LDCSTOP` are accepted
  - assert that bad forms such as `LDCCAL`, `LDCCAL CH3`, or oversize counts are rejected
- Hardware behavior test:
  - arm `LDCCAL CHx 20`
  - place one part once
  - assert that the service outputs 20 stable samples from that single settled placement
  - assert that summary statistics are emitted once
- Early-removal test:
  - remove the part before the capture completes
  - assert that the session does not publish a false final reference

#### 7. Wrong vs Correct

##### Wrong

- Let both `WeightService_Task` and `Ldc1614Service_Task` call `UartCommand_Fetch(...)`.
- Parse a calibration command in one task and then busy-loop there until all sensor samples are captured.
- Treat “sample count” as “operator must repeat placement count times” when the intended workflow is one placement plus a stable batch.
- Add `uint8_t` to a public header and assume another include chain will define it.

##### Correct

- Keep one UART RX command owner and dispatch feature-specific commands through explicit service handlers.
- Let the command owner arm or cancel a calibration session, and let the sensor task complete the workflow in task context.
- Define LDC calibration sampling as “one stable placement, N captured samples, one summary”.
- Make every public header self-contained by including the standard type headers it directly uses.

### Scenario: CubeMX-Managed Heartbeat Task and Board LED Ownership

#### 1. Scope / Trigger

- Trigger: adding a heartbeat/status LED task that runs under FreeRTOS
- Trigger: binding an application task to a GPIO already initialized by CubeMX
- Trigger: editing `Core/Src/freertos.c` to create a new thread for a user module
- Trigger: choosing a board pin based on the actual development board resource map rather than only the STM32 pin list

#### 2. Signatures

- Heartbeat task entry:
  - `void SystemHeartbeatService_Task(void *argument)`
- Heartbeat module location:
  - `User/App/system_heartbeat_service.h`
  - `User/App/system_heartbeat_service.c`
- RTOS bootstrap creation example:
  - `heartbeatTaskHandle = osThreadNew(SystemHeartbeatService_Task, NULL, &heartbeatTask_attributes);`
- Timing contract example:
  - `#define SYSTEM_HEARTBEAT_TOGGLE_PERIOD_MS 500U`
  - `vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(...))`
- Board resource source of truth:
  - `M144Z-M4最小系统板IO引脚分配表——STM32F407版.xlsx`
  - current confirmed mapping: `PF9 -> LED0`, not fully independent, not routed out as a general external pin

#### 3. Contracts

- Heartbeat behavior belongs to `User/App/*`, not to generated `Core/*` business code:
  - task body, timing policy, and LED toggle logic must live in a dedicated user module
- `Core/Src/freertos.c` remains RTOS bootstrap only:
  - allowed changes are limited to `USER CODE BEGIN/END` regions
  - if a new task is needed, keep includes, task attributes, and `osThreadNew(...)` glue inside user sections that survive CubeMX regeneration
- `Core/Src/gpio.c` remains the owner of GPIO initialization:
  - if CubeMX already configured `PF9`, the heartbeat module must only call `HAL_GPIO_TogglePin(...)`
  - do not re-run `HAL_GPIO_Init(...)` or duplicate pin mode configuration in `User/App`
- Heartbeat task priority must stay below real-time business tasks:
  - use low or below-normal priority
  - the goal is to observe scheduler health, not to compete with control/sampling tasks
- Heartbeat timing should use periodic RTOS delay rather than busy wait:
  - prefer `vTaskDelayUntil(...)` to reduce drift and avoid wasting CPU
- Board pin selection must first consult the board resource sheet:
  - if a pin is already tied to an on-board LED and not exposed, it is a preferred heartbeat candidate
  - if the sheet marks a pin as shared or “not fully independent”, document the ownership before reusing it

#### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| Board pin source | pin choice is confirmed from the board IO workbook, not guessed from MCU package only | selected pin may conflict with on-board hardware or may not be routed out | check the workbook row first and record the board resource relationship |
| GPIO init ownership | `gpio.c` performs init, heartbeat module only toggles the pin | duplicated initialization may conflict with CubeMX output | remove user-side `HAL_GPIO_Init(...)` and keep GPIO ownership in CubeMX files |
| `freertos.c` edit location | new task glue is inside `USER CODE BEGIN/END` regions | CubeMX regeneration may delete the task | move includes, attributes, and `osThreadNew(...)` calls into user regions |
| Task priority | heartbeat task is lower than control/sampling tasks | heartbeat may interfere with real-time behavior | reduce priority and keep the task body minimal |
| Timing implementation | periodic delay uses `vTaskDelayUntil(...)` or equivalent RTOS delay | busy loop wastes CPU or causes unstable blink period | replace busy wait with scheduler-friendly periodic delay |
| Module placement | heartbeat logic is in `User/App/*.c/.h` and the source is added to `MDK-ARM/*.uvprojx` | build may miss the file or logic may be buried in generated code | move logic to `User/App` and register the file in Keil |

#### 5. Good / Base / Bad Cases

- Good:
  - `PF9/LED0` is chosen after checking the board workbook
  - `system_heartbeat_service.c/.h` is created under `User/App`
  - `freertos.c` only adds a user-section include plus `osThreadNew(...)` glue
  - heartbeat task uses low priority and `vTaskDelayUntil(...)`
- Base:
  - another board LED or dedicated status pin is used
  - but the board workbook confirms ownership and the RTOS glue still stays inside user sections
- Bad:
  - a task body or full business loop is written directly into generated `freertos.c`
  - thread attributes are declared in generated regions that CubeMX may overwrite
  - the user module reconfigures `PF9` even though `gpio.c` already owns it
  - a busy loop toggles the LED without yielding to the scheduler

#### 6. Tests Required

- Board resource review:
  - assert that the selected heartbeat pin exists in the board workbook
  - assert that its row explains whether it is independent, shared, or board-bound
- RTOS bootstrap review:
  - assert that heartbeat-related changes in `Core/Src/freertos.c` are only inside `USER CODE BEGIN/END`
  - assert that the task entry itself lives in `User/App`
- Project file review:
  - assert that the new heartbeat source file is present in `MDK-ARM/bisai_f407_project.uvprojx`
- Hardware behavior test:
  - after boot, assert that `PF9/LED0` toggles with the configured period
  - if the scheduler stalls or a high-priority task monopolizes CPU, assert that the heartbeat visibly freezes or stops changing
- Regeneration safety test:
  - after any future `.ioc` regeneration touching `freertos.c`, assert that the heartbeat task glue still exists in the surviving user sections

#### 7. Wrong vs Correct

##### Wrong

```c
/* 反例：把新增任务定义直接塞进 CubeMX 生成区，后续重新生成容易丢失。 */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
    .name = "heartbeatTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

void MX_FREERTOS_Init(void)
{
    heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);
}
```

##### Correct

```c
/* 正确：业务任务放在 User/App，freertos.c 只在 USER CODE 区挂接任务。 */
/* USER CODE BEGIN Includes */
#include "system_heartbeat_service.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Variables */
static osThreadId_t heartbeatTaskHandle = NULL;
static const osThreadAttr_t heartbeatTask_attributes = {
    .name = "heartbeatTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* USER CODE END Variables */

/* USER CODE BEGIN RTOS_THREADS */
heartbeatTaskHandle = osThreadNew(SystemHeartbeatService_Task, NULL, &heartbeatTask_attributes);
/* USER CODE END RTOS_THREADS */
```

### Common Mistake: Putting extra RTOS task glue outside `USER CODE` regions in `freertos.c`

**Symptom**: A newly added task builds and runs once, but disappears or partially breaks after CubeMX regenerates the project.

**Cause**: Task handles, task attributes, prototypes, or `osThreadNew(...)` calls were inserted into generated sections instead of persistent user sections.

**Fix**: Keep the actual task implementation in `User/App/*`, and keep `freertos.c` changes limited to lightweight glue inside `USER CODE BEGIN/END`.

**Prevention**: Treat `Core/Src/freertos.c` as a bootstrap file only. If a change would not survive regeneration, it belongs either in a user section or in a dedicated `User/App` module.

### Scenario: Startup Self-Healing Config for External Motion Modules

#### 1. Scope / Trigger

- Trigger: adding a device driver or service for a module whose local buttons or internal settings can drift from MCU-side assumptions
- Trigger: adding boot-time “restore working mode” logic for motors, sensors, or smart modules that support both volatile apply and internal flash save
- Trigger: adding startup macros where one flag decides whether to send a config command and another flag decides the target payload
- Trigger: creating FreeRTOS bootstrap glue for a service whose correct runtime depends on those startup recovery commands

#### 2. Signatures

- Driver configuration APIs:
  - `EMM42_MotorStatus_t EMM42_MotorSetControlMode(const EMM42_MotorHandle_t *motor, EMM42_MotorControlMode_t control_mode, bool save_flag)`
  - `EMM42_MotorStatus_t EMM42_MotorSetButtonLock(const EMM42_MotorHandle_t *motor, bool locked, bool save_flag)`
- Service-side startup macros:
  - `CONVEYOR_MOTOR_STARTUP_FORCE_CTRL_MODE`
  - `CONVEYOR_MOTOR_STARTUP_CTRL_MODE`
  - `CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK`
  - `CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED`
  - `CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH`
- Startup recovery step:
  - `static EMM42_MotorStatus_t ConveyorMotorService_ApplyStartupConfig(const EMM42_MotorHandle_t *motor, const char **failed_stage)`
- Startup sequence contract:
  - `Init -> StartupConfig -> Enable -> StopNow`
- RTOS bootstrap examples:
  - `conveyorMotorTaskHandle = osThreadNew(ConveyorMotorService_Task, NULL, &conveyorMotorTask_attributes);`
  - `heartbeatTaskHandle = osThreadNew(SystemHeartbeatService_Task, NULL, &heartbeatTask_attributes);`
- Observable startup log:
  - `[INFO][BELT] startup_fix ctrl_force=%u, btn_lock_force=%u, btn_lock=%u, save=%s`

#### 3. Contracts

- If a module can be reconfigured outside the MCU flow, startup code must actively restore the required working state:
  - do not assume the device still uses the last expected mode
  - examples: panel button mis-touch, internal menu changes, previously saved module parameters
- `FORCE_*` and target-state macros have different meanings and must never be conflated:
  - `CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK != 0` means “send the button-lock command during boot”
  - `CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED != 0` means “the payload requests locked state”
  - therefore `force=1, locked=0` is an explicit unlock, not a lock
- Boot-time recovery must default to volatile apply rather than persistent writes:
  - `CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH` should default to `0U`
  - normal debug and daily boot flows should use RAM/volatile apply only
  - flash save is reserved for one-time commissioning or final parameter solidification
- If flash save is temporarily enabled for commissioning:
  - verify the device accepted the intended values
  - then revert the firmware default back to RAM/volatile apply
  - do not ship a debug image that writes external-module flash on every boot unless this is a deliberate, reviewed requirement
- RTOS bootstrap task creation handles must be checked immediately:
  - if `osThreadNew(...)` returns `NULL` for a critical service task, escalate to `Error_Handler()`
  - do not leave task-handle assignments “write-only” with no failure handling
- Startup logs must expose the actual recovery inputs:
  - print whether control-mode recovery is enabled
  - print whether button-lock recovery is enabled
  - print the actual target lock value
  - print whether the write target is `RAM` or `FLASH`
  - avoid vague “startup ok” logs that hide the effective payload

#### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| Out-of-band config recovery | boot path includes an explicit restore step before normal control loop | module may stay in panel-modified or previously saved wrong mode | add startup recovery function and place it before enable/normal state-machine execution |
| Force vs target semantics | code comments, macro names, and logs distinguish “send command” from “target state” | developers may believe a module is locked when firmware is explicitly unlocking it | keep separate macros and print both values in startup log |
| Save target default | boot recovery uses RAM/volatile apply by default | repeated flash writes to external module may happen on every boot | set default save flag to `0U`; reserve `1U` for reviewed one-shot commissioning |
| Commissioning flow | temporary flash-save usage is reverted after confirmation | debug image silently keeps writing external flash forever | document the one-time write flow and restore RAM default afterward |
| Task bootstrap handle check | every critical `osThreadNew(...)` result is checked for `NULL` | service may never start while firmware continues in a degraded state | fail fast through `Error_Handler()` or equivalent fatal path |
| Startup observability | boot log prints real recovery values such as `btn_lock=1` and `save=RAM` | hardware behavior may be misread from a generic success log | extend startup log to include concrete flags and save target |

#### 5. Good / Base / Bad Cases

- Good:
  - startup sends control-mode recovery and button-lock recovery explicitly
  - `CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH` stays `0U` in normal development images
  - startup log shows `ctrl_force=1, btn_lock_force=1, btn_lock=1, save=RAM`
  - `freertos.c` checks task-handle creation results and fails fast if critical tasks cannot be created
- Base:
  - startup chooses not to force button lock because manual现场操作 is currently required
  - but the code comments and log still make the unlocked behavior explicit
  - and the separation between “force command” and “target state” remains intact
- Bad:
  - assume the motor still uses the intended control mode after panel-button changes
  - set `CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK = 1U` and `CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED = 0U`, then mistakenly believe the panel is locked
  - keep `SAVE_TO_FLASH = 1U` in a normal boot image so the module writes internal flash every reset
  - ignore `osThreadNew(...)` results and leave task-handle assignments with no error path

#### 6. Tests Required

- Code review assertions:
  - assert that startup recovery happens before normal service motion/control logic
  - assert that `FORCE_*` macros and target-state macros are not merged into one ambiguous flag
  - assert that critical `osThreadNew(...)` results are checked immediately
- Boot log assertion:
  - power on the board
  - assert that startup log prints `ctrl_force`, `btn_lock_force`, `btn_lock`, and `save`
  - assert that printed values match the current macros in source
- Hardware recovery assertion:
  - deliberately modify the external module state out of band if possible
  - reboot
  - assert that the module returns to the MCU-expected working mode
- Button-lock assertion:
  - with `btn_lock=1`, verify on hardware that panel keys no longer alter the target behavior after boot
  - with `btn_lock=0`, verify that panel control remains available when intentionally allowed
- Flash-write policy assertion:
  - assert that routine development builds keep `SAVE_TO_FLASH == 0U`
  - if a commissioning build temporarily sets `SAVE_TO_FLASH == 1U`, assert that the default is reverted after confirmation

#### 7. Wrong vs Correct

##### Wrong

```c
/*
 * 反例：
 * 1. 只打开“是否发送锁键命令”，却把真正的目标状态留成 0；
 * 2. 每次启动都写外设内部 FLASH；
 * 3. 创建任务后不检查句柄是否为空。
 */
#define CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK (1U)
#define CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED     (0U)
#define CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH     (1U)

conveyorMotorTaskHandle = osThreadNew(ConveyorMotorService_Task, NULL, &conveyorMotorTask_attributes);
```

##### Correct

```c
/*
 * 正确：
 * 1. “是否发送命令”和“目标状态”分开定义；
 * 2. 常规启动默认只写 RAM/volatile 状态；
 * 3. 关键任务创建后立即判空。
 */
#define CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK (1U)
#define CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED     (1U)
#define CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH     (0U)

conveyorMotorTaskHandle = osThreadNew(ConveyorMotorService_Task, NULL, &conveyorMotorTask_attributes);
if (conveyorMotorTaskHandle == NULL)
{
    Error_Handler();
}
```

### Common Mistake: Confusing “send config command” with “target config state”

**Symptom**: Startup log shows `btn_lock_force=1`, but the actual target state is still unlock and the panel remains operable.

**Cause**: The code changed the “force apply” flag, but did not change the payload flag that encodes the final target state.

**Fix**: Review both `CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK` and `CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED`, and confirm the startup log prints the intended pair.

**Prevention**: Any startup self-healing config that uses multiple flags must log both the “apply” flag and the final payload state instead of logging only a generic success message.

--- 

## Testing Requirements

There is no automated test harness yet.
The minimum verification for firmware changes is:

1. build the MDK-ARM project successfully
2. confirm the board boots and reaches the scheduler
3. verify the affected peripheral behavior on hardware
4. verify UART output or debugger-visible state for the changed flow

For timing-sensitive changes, also verify interrupt priority and RTOS interaction on target hardware.

### Scenario: Keil CLI Rebuild Verification

#### 1. Scope / Trigger

- Trigger: any change that requires proving the `MDK-ARM` project still builds after source, group, include-path, or file-list updates
- Trigger: any debugging session where Keil command-line output looks incomplete, stale, or does not clearly prove linker success

#### 2. Signatures

- Preferred CLI entry:
  - `E:\Keil_v5\UV4\uVision.com -r MDK-ARM\bisai_f407_project.uvprojx -t "bisai_f407_project"`
- Build log artifact:
  - `MDK-ARM/bisai_f407_project/bisai_f407_project.build_log.htm`
- Primary binary artifact:
  - `MDK-ARM/bisai_f407_project/bisai_f407_project.axf`
- Secondary artifact:
  - `MDK-ARM/bisai_f407_project/bisai_f407_project.hex`

#### 3. Contracts

- A successful rebuild is only considered proven when all of the following are true:
  - the build log file is refreshed for the current run
  - the log contains linker completion evidence
  - the `.axf` output exists and has a current timestamp
- Required success markers inside `bisai_f407_project.build_log.htm`:
  - `Rebuild target 'bisai_f407_project'`
  - `linking...`
  - `FromELF: creating hex file...`
  - `"bisai_f407_project\bisai_f407_project.axf" - 0 Error(s), 0 Warning(s).`
- Terminal stdout alone is not a sufficient success contract when the CLI wrapper does not refresh output reliably.

#### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| CLI process returns | exit code `0` | tool launch failure or build failure | rerun with the preferred `uVision.com` entry and inspect artifacts |
| `build_log.htm` timestamp | refreshed by current run | log was not regenerated or the command did not drive a real build | do not claim success; rerun and re-check |
| log contains `linking...` | present | compile may have stopped before link stage | treat build as unproven |
| log contains `.axf` with `0 Error(s), 0 Warning(s)` | present | linker may have failed or log is stale | treat build as failed or stale |
| `.axf` timestamp | matches current rebuild window | previous binary is being reused as stale evidence | do not use it as proof |
| `.hex` timestamp | updated with `.axf` | FromELF stage may not have completed | inspect build log and fix target settings |

#### 5. Good / Base / Bad Cases

- Good:
  - `uVision.com` is used
  - `build_log.htm` contains the success markers
  - `.axf` and `.hex` timestamps match the rebuild window
- Base:
  - terminal output is short or looks incomplete
  - but `build_log.htm` is refreshed and contains complete success evidence
  - result may still be accepted
- Bad:
  - only terminal text is checked
  - no refreshed `build_log.htm`
  - or `.axf` timestamp is old
  - or success claim is made without linker evidence

#### 6. Tests Required

- Build verification test:
  - run the preferred `uVision.com` rebuild command
  - assert that `MDK-ARM/bisai_f407_project/bisai_f407_project.build_log.htm` is updated
  - assert that the log contains `linking...`
  - assert that the log contains `"bisai_f407_project\bisai_f407_project.axf" - 0 Error(s), 0 Warning(s).`
- Artifact verification test:
  - assert that `MDK-ARM/bisai_f407_project/bisai_f407_project.axf` exists
  - assert that its timestamp belongs to the current rebuild attempt
  - assert that `MDK-ARM/bisai_f407_project/bisai_f407_project.hex` is regenerated
- Change-scope verification test:
  - when adding new user modules, assert that their corresponding `.o` files are regenerated in the same rebuild window

#### 7. Wrong vs Correct

##### Wrong

- Run a CLI build once, see partial terminal output, and immediately claim success.
- Use an old `.axf` timestamp as proof after changing project files.
- Treat `UV4.exe` terminal refresh behavior as authoritative evidence when the log file was not checked.

##### Correct

- Prefer `uVision.com` for scripted rebuild verification.
- Read `MDK-ARM/bisai_f407_project/bisai_f407_project.build_log.htm` as the primary build record.
- Confirm current-run `.axf` and `.hex` timestamps before declaring the build passed.
- When the terminal output is incomplete, trust refreshed artifacts and the build log, not the shell transcript.

### Common Mistake: Treating terminal output as the only build evidence

**Symptom**: The shell output looks truncated or does not refresh, so the build result is guessed from partial text.

**Cause**: Keil command-line wrappers do not always provide a complete, fresh console transcript for the whole rebuild.

**Fix**: Read `MDK-ARM/bisai_f407_project/bisai_f407_project.build_log.htm` and verify that the `.axf` and `.hex` artifacts were regenerated.

**Prevention**: Never report "build passed" unless the current-run log and current-run artifacts both confirm success.

### Common Mistake: Relying on indirect type includes in public headers

**Symptom**: The project builds until one header is included from a different translation unit, then errors such as `identifier "uint8_t" is undefined` appear.

**Cause**: The header exposes fixed-width integer types but does not include `<stdint.h>` itself, and instead depends on an unrelated upstream include order.

**Fix**: Any public header that declares `uint8_t`, `uint16_t`, `int32_t`, and similar types must include `<stdint.h>` directly.

**Prevention**: Review public headers as standalone interfaces; they must compile from their own declared dependencies rather than from lucky transitive includes.

---

## Code Review Checklist

Reviewers should check:

- is the change inside the correct ownership boundary (`Core` vs `User` vs vendor)?
- will CubeMX regeneration preserve the change?
- are HAL failures checked and escalated correctly?
- is ISR/callback code minimal and RTOS-safe?
- are new globals justified and documented?
- are comments sufficient for a future maintainer to understand the hardware flow?
