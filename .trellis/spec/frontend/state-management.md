# State Management

> How application state is managed in this firmware project.

---

## Overview

This project uses plain C state plus RTOS primitives, not a frontend state library.
State should be owned by the narrowest possible scope:

- generated peripheral handles stay in generated peripheral modules
- task handles stay in RTOS bootstrap or the owning app module
- application data stays in `User/App` module-private state
- ISR-shared flags and buffers must be explicitly marked and synchronized

Current codebase examples:

- `Core/Src/usart.c`: `huart1` and `hdma_usart1_rx` are module-level hardware state
- `Core/Src/freertos.c`: `defaultTaskHandle` and `defaultTask_attributes` are RTOS state
- `Core/Src/main.c`: system tick progression is tied to `HAL_TIM_PeriodElapsedCallback(...)`

---

## State Categories

Use these state categories:

- local module state: `static` variables private to one `.c` file
- shared task state: exchanged through queues, notifications, semaphores, or mutexes
- hardware handle state: generated `UART_HandleTypeDef`, `DMA_HandleTypeDef`, `TIM_HandleTypeDef`, etc.
- ISR-visible state: only when necessary, with `volatile` and clear ownership

---

## When to Use Global State

Global state is acceptable only when it represents a singleton resource that already has project-wide identity, such as:

- CubeMX-generated peripheral handles
- one scheduler-owned task handle
- one clearly documented system service object

Do not make measurement values, parser buffers, or calibration parameters globally writable without a single owner module.

---

## Server State

There is no server state.
The closest equivalent is external hardware state from sensors, UART streams, or display hardware.

Treat that state as asynchronous input:

- sample it explicitly
- validate it before promoting it to application state
- separate raw readings from filtered/user-visible values

---

## Common Mistakes

Avoid these mistakes:

- sharing one buffer between DMA/ISR and task code without ownership rules
- storing feature state in `main.c` because it is easy to reach
- letting multiple modules write the same measurement or command state
- omitting `volatile` on ISR-updated flags

---

## Scenario: Sensor Detect/Settle/Remove State Machines

### 1. Scope / Trigger

- Trigger: adding or modifying any application task that decides `detected`, `stable`, `defect`, `removed`, or similar user-visible states from sampled sensor values
- Trigger: sensors whose raw value may drift during startup or may move in either direction when a part approaches
- Trigger: state machines that maintain a moving baseline and also emit serial logs for transition visibility

### 2. Signatures

- Task entry:
  - `void Ldc1614Service_Task(void *argument)`
- Command entry:
  - `uint8_t Ldc1614Service_HandleCommand(const char *command_buffer)`
- Typical module-private runtime fields:
  - `baseline`
  - `latest_filtered_sample`
  - `detect_threshold`
  - `release_threshold`
  - `state`
  - `last_stable_delta`
  - `detect_armed`
  - `startup_quiet_count`
- Typical user-visible log frames:
  - `[EVENT][LDC] <channel> part detected, settling`
  - `[WARN][LDC] <channel> settling cancelled, part removed before stable`
  - `[WARN][LDC] <channel> measurement aborted, part removed early`
  - `[RESULT][LDC] <channel> stable_delta=<n>, reference=unset`
  - `[EVENT][LDC] <channel> part removed, ready (delta=<n>)`

### 3. Contracts

- Keep raw sample, filtered sample, moving baseline, and stable result as separate state variables.
- Do not promote startup samples directly into normal detect logic.
  - After building a baseline, require a startup arm phase.
  - The channel may enter normal `IDLE -> DETECTED/SETTLING` logic only after `N` consecutive samples are back inside the quiet or release window.
- If sensor polarity is not hardware-proven, compare the absolute delta from baseline.
  - Signed rise/fall logic is allowed only when the direction has been validated on real hardware and documented per channel.
- `IDLE` may update the baseline only when the current sample is still classified as empty or quiet.
  - If the baseline keeps moving while a real part is present, the first placement can be absorbed into the baseline and later removal will look like the actual detection event.
- `SETTLING` is a visible transitional state, not a silent delay.
  - If the signal returns below the release threshold before the settle deadline, cancel the current detection and emit a warning log.
- `MEASURING` must abort cleanly if the part leaves early.
  - Early removal during the measurement window must not emit a stable result.
  - It must emit one warning log and return to the idle path.
- `WAIT_REMOVE` must require release confirmation before reporting removal.
  - After confirmed removal, the latest empty sample may be promoted to the new idle baseline.
- User-visible transition paths must be observable.
  - Silent cancellation is forbidden for `SETTLING` aborts and `MEASURING` aborts.
  - Each transition path should emit exactly one deterministic line.

### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| Startup with no part | no `part detected` log after boot settles | startup drift is entering detect path | add startup arm gate and keep baseline in empty-only tracking |
| First placement after boot | `part detected, settling` appears before any removal log | first placement was absorbed into baseline | stop updating baseline during effective detection and verify detect arming |
| Unknown polarity channel | detection still works when raw value rises or falls | code assumes wrong sign | use absolute delta or validate and document per-channel signed logic |
| Remove during settling | `settling cancelled, part removed before stable` is printed | transitional abort is invisible | add explicit cancellation log and idle re-entry |
| Remove during measuring | `measurement aborted, part removed early` is printed and no result is emitted | invalid result can be reported from incomplete data | abort measurement and clear progress |
| Stable result path | one result line after settle + measure window completes | result path is duplicated or skipped | make result emission single-owner inside measuring completion |
| Removal after stable result | one `part removed, ready` line after release confirm | release path is too eager or silent | keep release confirm counter and log confirmed removal only |

### 5. Good / Base / Bad Cases

- Good:
  - baseline is built from empty samples only
  - startup drift is absorbed by an arm phase before detection is enabled
  - part detection works regardless of whether raw values rise or fall
  - removing the part during `SETTLING` or `MEASURING` produces one explicit warning line
- Base:
  - signed direction is still used
  - but each channel direction is validated on real hardware and documented with evidence
  - startup still has a quiet-arm phase before detection is enabled
- Bad:
  - baseline is continuously updated even while a real part may already be present
  - the first placement after boot is silently absorbed into the baseline
  - removal is the first event that produces a detect-like transition
  - transitional aborts return to idle without any user-visible log

### 6. Tests Required

- Startup empty test:
  - boot with no part present
  - assert that no detect log is emitted during startup stabilization
- First placement test:
  - place a part once after boot
  - assert that `part detected, settling` occurs before any removal log
- Early remove during settling:
  - place a part and remove it before settle delay expires
  - assert that `settling cancelled, part removed before stable` is emitted exactly once
- Early remove during measuring:
  - allow the task to enter measuring and remove the part before the window completes
  - assert that `measurement aborted, part removed early` is emitted and no result line follows
- Polarity robustness test:
  - validate one channel whose raw values rise and one channel whose raw values fall
  - assert that both can enter detect and remove paths correctly under the same state machine contract
- Repeat cycle test:
  - place, stabilize, remove, then repeat without reboot
  - assert that detection works on the second cycle without needing a “priming” placement

### 7. Wrong vs Correct

#### Wrong

```c
/* Wrong: signed direction is assumed even though channel polarity is not proven. */
delta = (sample - baseline) * detect_direction;

if (delta >= detect_threshold)
{
    ++detect_confirm_count;
}
else
{
    /* Wrong: this can absorb the first real placement into baseline. */
    baseline = update_baseline(baseline, sample);
}
```

#### Correct

```c
/* Correct: use absolute delta until hardware direction is proven. */
delta = sample - baseline;
if (delta < 0)
{
    delta = -delta;
}

if (detect_armed == 0U)
{
    if (delta <= release_threshold)
    {
        baseline = update_baseline(baseline, sample);
        ++startup_quiet_count;
        if (startup_quiet_count >= STARTUP_ARM_CONFIRM_COUNT)
        {
            detect_armed = 1U;
        }
    }
    else
    {
        baseline = sample;
        startup_quiet_count = 0U;
    }
}
else if (delta >= detect_threshold)
{
    ++detect_confirm_count;
}
```
