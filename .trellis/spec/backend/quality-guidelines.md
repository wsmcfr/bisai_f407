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

---

## Code Review Checklist

Reviewers should check:

- is the change inside the correct ownership boundary (`Core` vs `User` vs vendor)?
- will CubeMX regeneration preserve the change?
- are HAL failures checked and escalated correctly?
- is ISR/callback code minimal and RTOS-safe?
- are new globals justified and documented?
- are comments sufficient for a future maintainer to understand the hardware flow?
