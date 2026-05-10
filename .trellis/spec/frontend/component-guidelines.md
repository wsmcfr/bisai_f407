# Component Guidelines

> How application modules and task-facing components should be built in this project.

---

## Overview

There are no UI components in the web sense.
In this repository, a "component" is an application module that owns one clear responsibility, such as:

- a measurement service
- a serial protocol handler
- a display updater
- a control task

Each component should compose low-level drivers from `User/Driver/` instead of accessing CubeMX-generated globals everywhere.

---

## Component Structure

A typical user component should have:

1. a header with public types, configuration, and API declarations
2. a source file with static state and private helpers
3. an init function
4. either a task entry function or a periodic processing function

Current baseline examples that show the style of thin entry points:

- `Core/Src/freertos.c`: `StartDefaultTask(void *argument)`
- `Core/Src/main.c`: `main(void)` and `MX_FREERTOS_Init()`
- `Core/Src/usart.c`: `MX_USART1_UART_Init(void)`

---

## Props Conventions

Use explicit C types instead of loosely coupled argument lists.

Preferred patterns:

- a config struct when a module needs multiple parameters
- fixed-width integers for protocol fields and sensor values
- `const` pointers for read-only inputs

Current codebase examples:

- `Core/Src/freertos.c`: `osThreadAttr_t` groups task attributes into a struct
- `Core/Src/usart.c`: `UART_HandleTypeDef` and `DMA_HandleTypeDef` group peripheral state
- `Core/Src/main.c`: HAL clock config uses `RCC_OscInitTypeDef` and `RCC_ClkInitTypeDef`

---

## Styling Patterns

There is no visual styling system.
The equivalent concern is user-visible formatting on serial ports or displays.

Rules for user-facing formatting:

- keep message formats stable once defined
- centralize frame/text formatting in one app module
- avoid mixing formatting code into low-level drivers

---

## Accessibility

Traditional accessibility guidance does not apply.
The embedded equivalent is making behavior easy to observe and debug:

- use deterministic serial output formats
- use clear units in displayed or transmitted values
- avoid hidden state transitions with no observable trace

---

## Common Mistakes

Avoid these mistakes:

- mixing hardware register/HAL detail into high-level task logic
- letting one module own unrelated peripherals and protocol state
- exposing mutable globals instead of a small API
- formatting UART/display output from ISR context

## Scenario: End-Effector Vision and Vacuum Gripper Integration

### 1. Scope / Trigger

- Trigger: adding MaixCAM2, WonderMV, OpenMV, or any external vision module to the robot arm workflow
- Trigger: changing the robot arm end effector from a servo claw to a vacuum suction cup, pump, valve, or PWM electronic switch
- Trigger: adding cross-controller behavior where STM32F407, ESP32, a vision module, and the arm controller share one pick workflow
- Trigger: assigning board connectors or GPIOs for robot-arm vision, suction, release, or pick-status reporting

### 2. Signatures

- STM32-to-ESP32 high-level command examples:
  - `ARMGRAB`
  - `ARMSTOP`
  - `ARMSTATUS`
  - or equivalent LeArm frame wrappers forwarded through STM32 USART3 to ESP32
- ESP32-side pick command handlers:
  - `uint8_t ArmVisionGrab_Start(void)`
  - `uint8_t ArmVisionGrab_Stop(void)`
  - `ArmVisionGrab_Status_t ArmVisionGrab_GetStatus(void)`
- MaixCAM2 vision result payload fields:
  - `uint8_t found`
  - `int16_t dx_px`
  - `int16_t dy_px`
  - `int16_t angle_deg`
  - `uint8_t score`
  - `uint16_t area`
  - `uint16_t frame_id`
- Vacuum output controls:
  - `VAC_PUMP`: vacuum pump PWM electronic switch or GPIO output
  - `VAC_RELEASE`: three-way valve PWM electronic switch or GPIO output

### 3. Contracts

- Keep controller ownership clear:
  - STM32F407 is the top-level coordinator and may only send high-level arm commands or query status
  - ESP32 owns robot-arm motion, MaixCAM2 result consumption, suction pump control, release-valve control, and pick-state progression
  - MaixCAM2 owns image processing and should send compact target results, not raw images, to ESP32
  - STM32F407 must not directly interpret MaixCAM2 pixels unless the architecture is deliberately redesigned and documented
- Preserve board connector ownership:
  - J2 `PA5/PA4` stays reserved for STM32F407-to-ESP32 communication
  - robot-arm bus servo connector stays reserved for bus servos
  - I2C `SDA/SCL` may be used for MaixCAM2 vision-result transfer when no spare UART is available
  - PWM servo outputs may drive suction pump and release-valve PWM electronic switches when the arm uses bus servos
- Use stable, compact vision data:
  - `dx_px` and `dy_px` are target offsets relative to the calibrated gripper pickup pixel, not necessarily the image center
  - `frame_id` must change when a new MaixCAM2 result is produced
  - ESP32 must reject stale vision data before moving the arm
- Use an explicit pick state machine:
  - idle
  - waiting for target
  - aligning
  - descending
  - suction on
  - lifting
  - placing
  - releasing
  - done or failed
- Default outputs must be safe:
  - pump and release valve are off during boot and after fault handling
  - ESP32 must turn pump off and open/close the release valve according to a bounded release sequence before returning to idle
- Document wiring near the code and in project docs:
  - connector names
  - GPIO numbers
  - voltage levels
  - shared ground requirements
  - startup-pin risks such as ESP32 `GPIO0` and `GPIO15`
  - whether the pump/valve are driven through PWM electronic switches or direct MOSFET drivers

### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| STM32 command scope | F4 sends start/stop/status only | vision/motion ownership is split across controllers | move pixel interpretation and pick state to ESP32 or document a new architecture |
| J2 ownership | J2 remains dedicated to F4-ESP32 communication | MaixCAM2 or suction control can collide with F4 commands | move MaixCAM2 to I2C/spare UART and suction to PWM/GPIO outputs |
| Vision freshness | `frame_id` changes and sample age is within timeout | ESP32 may chase stale coordinates | reject stale target and report no-target or timeout |
| Pickup pixel calibration | target offset is measured from gripper pickup pixel | centering on image center may miss the part | calibrate and store `pickup_u/pickup_v` |
| Pump/valve boot state | both outputs are inactive after reset | suction or release can trigger during boot | set safe defaults before enabling motion |
| ESP32 startup pins | GPIO0/GPIO15 loads do not break boot or trigger outputs | PWM electronic switches may affect flashing or startup | test reset/download and move outputs to S1-S4 if needed |
| Power budget | pump, valve, servos, ESP32, and MaixCAM2 share adequate supply margin | brownout or random reset risk | add external 5V supply and common ground |
| Fault path | pump is turned off and release sequence is bounded | part may remain attached or pump may run indefinitely | add fault cleanup and maximum action timers |

### 5. Good / Base / Bad Cases

- Good:
  - F4 sends `ARMGRAB`, ESP32 reads the latest MaixCAM2 I2C payload, aligns with `dx_px/dy_px`, activates pump through a PWM electronic switch, releases through a valve switch, then reports status
  - MaixCAM2 is mounted on the end effector and reports offsets relative to a calibrated pickup pixel
  - pump and valve outputs have safe boot defaults and timeout-based cleanup
- Base:
  - suction uses one pump output only and releases by turning the pump off
  - acceptable for early testing, but release latency and pickup confirmation must be validated on hardware
- Bad:
  - MaixCAM2 is connected to J2 and shares the same ESP32 serial port as STM32F407 commands
  - F4 receives pixel coordinates and tries to micromanage ESP32 motion one step at a time
  - pump or valve is connected directly to an MCU GPIO without a driver
  - image center is assumed to be the gripper pickup point without calibration

### 6. Tests Required

- Wiring validation:
  - assert J2 still carries only F4-to-ESP32 communication
  - assert MaixCAM2 uses the documented I2C or spare-UART path
  - assert pump and valve use PWM electronic switches or proper MOSFET drivers
- Boot safety test:
  - power cycle the system with suction hardware attached
  - assert pump and release valve remain inactive until ESP32 explicitly starts a pick
- Vision freshness test:
  - stop MaixCAM2 updates
  - assert ESP32 rejects stale `frame_id` or stale timestamp and does not move to pick
- Pickup calibration test:
  - place a part under the suction cup center
  - assert the stored pickup pixel produces near-zero `dx_px/dy_px`
- Pick workflow test:
  - run one full pick cycle
  - assert state transitions occur in order and each output has a bounded active time
- Fault cleanup test:
  - inject no-target, IK-fail, timeout, and manual stop paths
  - assert pump is off and release valve returns to the documented safe state

### 7. Wrong vs Correct

#### Wrong

```c
/* Wrong: F4 tries to consume camera pixels and directly drive pick micro-steps. */
if (strcmp(command, "CAMERA") == 0)
{
    parse_pixel_coordinates_on_stm32(command, &x_px, &y_px);
    send_small_arm_step_to_esp32(x_px, y_px);
}

/* Wrong: image center is treated as the gripper pickup point without calibration. */
dx_px = target_x - (image_width / 2);
dy_px = target_y - (image_height / 2);
```

#### Correct

```c
/*
 * Correct: F4 only authorizes the workflow. ESP32 owns vision, motion,
 * suction outputs, timeout handling, and final status reporting.
 */
if (strcmp(command, "ARMGRAB") == 0)
{
    RobotArmService_SendGrabStartFrame();
}

/*
 * Correct: ESP32 computes offsets against the calibrated pickup pixel
 * from the end-effector camera view.
 */
dx_px = vision_result.target_x - pickup_u;
dy_px = vision_result.target_y - pickup_v;

if ((vision_result.found != 0U) &&
    (vision_result.score >= ARM_VISION_MIN_SCORE) &&
    (ArmVision_IsFresh(vision_result.frame_id, vision_result.timestamp_ms) != 0U))
{
    ArmVisionGrab_AdvanceState(dx_px, dy_px, vision_result.angle_deg);
}
else
{
    ArmVisionGrab_Fail(ARM_VISION_GRAB_NO_TARGET);
}
```

**Related**: Keep the current wiring summary in `docs/robot-arm-vision-vacuum-wiring.md` synchronized whenever connector ownership, GPIO assignment, voltage level, or pump/valve driver choice changes.

## Scenario: MaixCAM2-to-ESP32 I2C Vision Link

### 1. Scope / Trigger

- Trigger: changing MaixCAM2 vision-result transport, I2C pins, I2C address, frame format, or link diagnostics.
- Trigger: modifying ESP32 robot-arm firmware that consumes MaixCAM2 vision frames.
- Trigger: adding code that runs after `arm.init()`, `pc_ble_obj.init()`, camera initialization, or display initialization and may affect I2C timing.
- Trigger: fixing UI freezes, camera stutter, `SCAN NONE`, `I2C ERR`, MaixPy `write failed/read failed`, or ESP32 `RX=0` symptoms.

### 2. Signatures

- MaixCAM2 known-good bus contract:
  - bus: `I2C6`
  - pins: `A1 -> I2C6_SCL`, `A0 -> I2C6_SDA`
  - frequency: `50000`
  - target 7-bit address: `0x42`
  - diagnostic scripts: `Vision/maixcam2/i2c_probe.py`, `Vision/maixcam2/i2c_probe_with_camera.py`, `Vision/maixcam2/i2c_line_monitor.py`
- ESP32 known-good slave contract:
  - driver: ESP-IDF native I2C slave driver, not Arduino `Wire` slave
  - port: `I2C_NUM_1`
  - pins: `SDA=GPIO17`, `SCL=GPIO16`
  - address: `VISION_I2C_SLAVE_ADDR == 0x42`
  - frequency limit: `VISION_I2C_FREQ_HZ == 50000U`
  - public APIs: `VisionI2CLink::Init(...)`, `VisionI2CLink::ProcessPending()`, `VisionI2CLink::FetchLatest(...)`, `VisionI2CLink::FetchDebugCounters(...)`
- Vision frame format, length `16` bytes:
  - `[0] 0xA5`
  - `[1] 0x5A`
  - `[2] version, currently 0x01`
  - `[3] flags, bit0 means `found`
  - `[4..5] frame_id, little-endian `uint16_t`
  - `[6..7] dx_px, little-endian `int16_t`
  - `[8..9] dy_px, little-endian `int16_t`
  - `[10..11] angle_deg, little-endian `int16_t`
  - `[12] score`
  - `[13..14] area, little-endian `uint16_t`
  - `[15] checksum, low 8 bits of sum over bytes `[0..14]`
- Required user-visible Maix statuses:
  - `SCAN 42 OK` means the target address was visible in the last scan.
  - `I2C OK` means the last Maix `writeto(0x42, frame)` completed successfully.
  - `I2C OFF` means the link is intentionally disabled and must not touch the I2C bus.

### 3. Contracts

- Use I2C6 only for the current MaixCAM2 wiring:
  - `A1/A0 + I2C6` is the verified path.
  - Do not switch to `I2C7 A8/A9` during normal operation; it is not connected in the current robot-arm wiring and can crash or destabilize MaixPy diagnostics when probed after a good I2C6 test.
- Use ESP-IDF native I2C slave on ESP32:
  - Do not use Arduino `Wire.begin(addr, sda, scl, freq)` for this link.
  - The previous Arduino `Wire/Wire1` slave approach returned `ready=1` but did not ACK MaixCAM2 reliably.
  - Install the slave with `i2c_param_config(...)` and `i2c_driver_install(...)` on `I2C_NUM_1`.
- Reinstall ESP32 I2C after robot-arm initialization:
  - `VisionI2CLink::Init()` must run before early diagnostics and again after `arm.init()` and `pc_ble_obj.init(0)`.
  - The final `setup done` state must expose a fresh `0x42` slave, because arm initialization can disturb timing, power, GPIO state, or driver state.
- Keep Maix UI and camera non-blocking:
  - `main.py` must not call blocking `scan()` or `writeto()` from the camera/display loop.
  - I2C work must remain in the background worker.
  - The user must explicitly tap `LINK` before the worker touches I2C.
  - If a scan or write fails, disable the link for that attempt and require another manual `LINK` tap instead of retrying continuously.
  - Prefer targeted `scan(0x42)` over repeated full-bus scans in the tracking loop.
- Diagnose in this order:
  - Run `i2c_line_monitor.py` only to prove SCL/SDA wiring by forcing ESP32 line-low tests.
  - Run `i2c_probe.py` to prove I2C6 address, read, and write without camera.
  - Run `i2c_probe_with_camera.py` to prove I2C6 still works after camera initialization.
  - Run `main.py` only after both probe scripts show `FOUND 42`.
- Treat scan, write, and ESP32 RX as different layers:
  - `FOUND 42` proves address ACK only.
  - `write=OK 16` or `I2C OK` proves Maix wrote a frame to the address.
  - ESP32 `RX` and `OK` counters prove the frame reached and decoded in application logic.
  - Do not claim the arm received valid vision data from `FOUND 42` alone.

### 4. Validation & Error Matrix

| Check | Expected | Failure Meaning | Required Action |
|-------|----------|-----------------|-----------------|
| Maix pinmap | `A1 => I2C6_SCL`, `A0 => I2C6_SDA` | Wrong Maix pin function or wrong script version | Fix constants before checking ESP32 |
| Line idle | I2C6 `SCL=1 SDA=1` | No pull-up, short, wrong connector, or held-low bus | Check 3.3V pull-ups, common ground, and connector direction |
| Line-low test | ESP32 pulling GPIO16 low maps to Maix SCL low; GPIO17 maps to SDA low | SCL/SDA swapped or wrong connector | Fix wiring before any protocol test |
| Probe without camera | `FOUND 42`, `write=OK 16`, optional `read=OK` | ESP32 slave not ACKing or wrong driver/address | Use ESP-IDF slave on `I2C_NUM_1`, address `0x42` |
| Probe with camera | Still `FOUND 42` after camera warmup | Camera initialization affects Maix I2C access | Do not proceed to `main.py`; change Maix bus/pins or init strategy |
| Main tracking page | `SCAN 42 OK` and `I2C OK` after tapping `LINK` | Worker failed, link disabled, or ESP32 final init lost slave state | Check worker state and re-run ESP32 I2C init after arm init |
| ESP32 counters | `RX` increments and `OK` increments for valid 16-byte frames | ESP32 sees address but app does not decode the frame | Check `ProcessPending()` cadence, frame length, header, version, checksum |
| UI frame rate | Camera/display remains responsive when link is missing | I2C call is blocking the display loop | Move I2C access back to worker and add link-failure fuse |
| Repeated failures | Link turns off instead of retrying forever | Continuous retry can freeze Maix UI | Require manual `LINK` tap to retry after a failed scan/write |

### 5. Good / Base / Bad Cases

- Good:
  - Maix uses `I2C6 A1/A0` at `50kHz`, starts camera, then a background worker scans `0x42` only after the user taps `LINK`.
  - ESP32 installs native `I2C_NUM_1` slave on GPIO17/GPIO16, reinitializes it after arm setup, and polls `ProcessPending()` in `loop()`.
  - Diagnostics show `i2c_probe.py: FOUND 42`, `i2c_probe_with_camera.py: FOUND 42`, `main.py: SCAN 42 OK`, and ESP32 `RX/OK` counters increasing.
- Base:
  - Maix scan and write work, but ESP32 only logs `RX` and `BAD`; acceptable during frame-format debugging, but not for motion control.
  - Maix requires manual `LINK` retry after a transient failure; acceptable because it protects the display loop from blocking.
- Bad:
  - ESP32 uses Arduino `Wire` slave for this link after it has already failed ACK tests.
  - Maix tracking loop calls direct `scan()` or `writeto()` on every frame and freezes when ESP32 is disconnected.
  - A diagnostic switches to I2C7 after I2C6 has already found `0x42` and crashes MaixPy.
  - The code assumes `FOUND 42` means the ESP32 application consumed a valid vision frame.
  - The code initializes I2C before `arm.init()` only and never reinstalls it after the arm stack finishes setup.

### 6. Tests Required

- ESP32 compile test:
  - compile `Vision/maixcam2/reference/esp32_factory_base/LeArm_ESP32_Arduino`
  - assert it builds with ESP32 Arduino core `2.0.17`
- Maix pure I2C probe:
  - run `i2c_probe.py`
  - assert serial output contains `version=...`, `I2C6 A1/A0`, `FOUND 42`, and `write=OK 16`
- Maix camera-active probe:
  - run `i2c_probe_with_camera.py`
  - assert camera warmup completes and I2C6 still reports `FOUND 42`
- Main UI no-freeze test:
  - run `main.py` with ESP32 connected and disconnected
  - assert the image remains responsive and the link displays `I2C OFF`, `SCAN 42 OK`, or `I2C OK` without long freezes
- ESP32 application receive test:
  - run `main.py`, enter tracking mode, tap `LINK`
  - assert ESP32 `[VISION_I2C_DBG]` shows `RX` incrementing
  - assert `OK` increments for valid frames and `BAD` increments only for deliberate malformed-frame tests
- Reinitialization test:
  - power-cycle ESP32 with arm connected
  - assert logs show initial I2C init, `arm init done`, `vision i2c reinit after arm done`, and final `setup done`
  - assert Maix can still scan `0x42` after `setup done`

### 7. Wrong vs Correct

#### Wrong

```cpp
/*
 * Wrong: Arduino Wire slave can print ready=1 but still not ACK MaixCAM2
 * reliably in this robot-arm firmware.
 */
Wire.onReceive(onReceive);
Wire.begin((uint8_t)0x42, 17, 16, 100000U);

/*
 * Wrong: only initializing before arm.init() leaves the final running state
 * dependent on whatever the arm stack did during setup.
 */
vision_i2c_obj.Init();
arm.init();
pc_ble_obj.init(0);
```

```python
# Wrong: direct I2C calls in the camera/display loop can freeze the UI
# when the ESP32 is missing, brownout-resetting, or not ACKing.
while not app.need_exit():
    frame = cam.read()
    i2c_bus.scan()
    i2c_bus.writeto(0x42, frame_bytes)
    disp.show(frame)
```

#### Correct

```cpp
/*
 * Correct: use ESP-IDF native I2C slave on ESP32 I2C1, then reinstall the
 * slave after robot-arm initialization so the final runtime state ACKs 0x42.
 */
vision_i2c_obj.Init();      /* early diagnostic state */
arm.init();
pc_ble_obj.init(0);
vision_i2c_obj.Init();      /* final runtime state */

void loop()
{
    vision_i2c_obj.ProcessPending();
    vision_i2c_obj.FetchLatest(&latest_vision_frame, 1U);
}
```

```python
# Correct: let the display loop publish only the latest frame to a worker.
# The worker owns scan/write calls and disables the link on failure.
submit_i2c_result_frame(i2c_worker, result_frame)

if scan_failed_or_missing_42:
    worker_state.link_enabled = False
    comm_state.link_enabled = False
```

**Related**: When this link regresses, debug by evidence in this order: line-low mapping, `i2c_probe.py`, `i2c_probe_with_camera.py`, `main.py` status text, then ESP32 `RX/OK/BAD` counters. Do not change pins, address, driver, and UI retry behavior in the same debugging step.

### 8. Regression Memory: 2026-05-10 IIC/I2C Bring-up

`IIC` and `I2C` refer to the same MaixCAM2-to-ESP32 vision link in this project. Future debugging must start from the verified evidence below instead of re-trying earlier failed assumptions.

| Layer | Verified Evidence | Do Not Repeat | Correct Next Step |
|-------|-------------------|---------------|-------------------|
| Maix bus | Runtime log must show `[VISION] I2C init bus=6 freq=50000Hz`, `A1 => I2C6_SCL`, `A0 => I2C6_SDA` | Do not switch normal operation back to `I2C7 A8/A9` just because a connector exposes several pins | Keep `main.py`, `i2c_probe.py`, and `i2c_probe_with_camera.py` aligned to I2C6 A1/A0 |
| Maix scan | Good runtime log is `I2C scan bus=6 mode=target addrs=['0x42'] target=0x42 found=True` | Do not treat `target+full addrs=[]` in `main.py` as a hardware proof when `i2c_probe.py` already finds `0x42` | Compare the app scan path with the probe scan path and remove slow full-bus fallback from the runtime loop |
| ESP32 slave | Good boot log is `driver=esp-idf port=1 addr=0x42 sda=17 scl=16 freq=50000 ... ready=1` | Do not use Arduino `Wire` or `Wire1` slave again for this link, even if it prints `ready=1` or beeps twice | Keep the ESP-IDF native slave driver on `I2C_NUM_1` and verify address ACK from Maix |
| Arm init ordering | Good boot sequence includes I2C init, `arm init done`, then a final I2C reinstall before `setup done` | Do not assume I2C initialized before arm setup remains valid after arm/BLE initialization | Re-run `VisionI2CLink::Init()` after robot-arm initialization and before normal loop processing |
| UI performance | `LINK OFF` or `I2C OFF` must remain smooth with ESP32 disconnected or powered down | Do not put scan/write calls in the camera/display loop or retry forever after failure | Keep all I2C calls in the worker, use target-only scan, and disable the link after one failed attempt |
| Success meaning | `FOUND 42` means address ACK; `I2C OK` means Maix write returned success; ESP32 `RX/OK` means the application consumed a valid frame | Do not say communication is fully successful from `FOUND 42` alone | Require `SCAN 42 OK`, `I2C OK`, and increasing ESP32 `RX/OK` counters before enabling motion decisions |

Regression checklist before changing code again:

1. Confirm hardware first with `i2c_line_monitor.py`: Maix I2C6 A1/A0 must follow ESP32 GPIO16/GPIO17 line-low tests.
2. Confirm address and frame write without camera using `i2c_probe.py`: expected `FOUND 42` and `write=OK 16`.
3. Confirm camera does not break the bus using `i2c_probe_with_camera.py`: expected `FOUND 42` after camera warmup.
4. Confirm `main.py` uses the same proven bus and target-only scan path: expected `SCAN 42 OK` after tapping `LINK`.
5. Confirm ESP32 application counters: expected `[VISION_I2C_DBG]` `RX` and `OK` increasing for valid 16-byte frames.
6. If the display becomes slow, stop debugging protocol details and first restore the no-freeze rule: worker-owned I2C, no direct display-loop I2C, one failure disables link until manual retry.

### 9. Open Issue Memory: Vision Tracking Control Not Good Enough

As of 2026-05-10, the MaixCAM2-to-ESP32 I2C transport is usable, but the visual tracking control behavior is not accepted. Do not treat the current tracking loop as finished just because the code compiles, `SCAN 42 OK` appears, or ESP32 logs `[VISION_ARM]`.

| Symptom | Current Evidence | Likely Layer | Next Debug Action |
|---------|------------------|--------------|-------------------|
| Target is visibly far from the screen center but the arm barely moves or stops | User moved the target from far-left to far-right and motion remained small or absent | ESP32 control law, saturation, IK failure, or stale `frame_id` gating | Log and compare `dx/dy/area`, `base`, `pose`, `duty`, `move_ok`, and coordinate limits before changing I2C |
| Front arm servos jitter | Frequent `coordinate_set()` calls recompute multiple joints from noisy 2D target data | ESP32 pose controller plus Maix target jitter | Keep high-rate correction on base yaw only; throttle or low-pass `coordinate_set()` for reach/height |
| Maix target judgment is unstable | Color blob edges and reflections can change `cx/cy/area` frame-to-frame | Maix blob selection and smoothing | Keep `find_blobs(... merge=True, margin=8)` when supported, add adaptive smoothing, and verify on-screen `dx/dy/area` follows the real object |
| Reach estimate is poor | Single camera has no depth; `area` is only a proxy for distance | Cross-layer perception/control contract | Calibrate `VISION_ARM_TARGET_AREA` at the desired pickup distance before tuning reach gains |

Current experimental control state:

- Maix side:
  - `main.py` sends fixed 16-byte frames over I2C6 A1/A0.
  - `I2C_SEND_INTERVAL_MS` was reduced to `25`.
  - `find_blobs()` prefers `merge=True, margin=8` when the firmware supports it.
  - `smooth_target_for_control()` uses adaptive smoothing: stronger smoothing for small jitter, faster following for large target movement.
- ESP32 side:
  - `dx_px` was moved from `coordinate_set(y)` to direct base-yaw control with `knot_run(6, duty, time)`.
  - `area` controls reach `x`; `dy_px` slowly controls height `z`.
  - `coordinate_set()` is throttled separately from base yaw to reduce multi-joint jitter.
  - Logs use `[VISION_ARM] base=... pose=... dx=... dy=... area=... duty=... xyz=... delta=...`.

Do not repeat these mistakes next session:

- Do not restart by changing I2C pins, address, or ESP32 slave driver if `RX/OK` still increments.
- Do not assume `coordinate_set()` succeeded; inspect `pose=1/0` and verify whether requested `x/z` are inside reachable limits.
- Do not tune gains from visual impression alone; first record several `[VISION_ARM]` lines while moving the object left, center, right, near, and far.
- Do not use only fixed tiny steps for large `dx`; far-from-center targets need proportional or velocity-style correction.
- Do not call `coordinate_set()` at camera frame rate; it can create front-servo jitter even when the base yaw path is correct.

Minimum next-session validation matrix:

| Test | Expected Evidence | If It Fails |
|------|-------------------|-------------|
| Maix visual data follows object | On-screen `dx` changes from large negative to near zero to large positive when moving object left-to-right | Fix threshold, blob merge, target selection, or smoothing first |
| I2C app data is fresh | ESP32 `[VISION_ARM] frame=` increases and `dx/dy/area` match Maix display directionally | Debug packing, worker send rate, or ESP32 decode before changing control |
| Base yaw responds to large `dx` | `[VISION_ARM] base=1`, `duty` changes significantly before hitting min/max | Tune base sign/gain/limits; if duty is saturated, widen safe limit only after mechanical check |
| Pose controller does not jitter | `pose=1` appears at the throttled interval and front servos move slowly | Lower reach/height gains, increase pose interval, or hold `z` constant until yaw is centered |
| Reach control is meaningful | At calibrated pickup distance, `area` is near `VISION_ARM_TARGET_AREA` | Recalibrate target area; do not tune reach gain before this |

Recommended next direction:

1. Keep I2C fixed and instrument control first.
2. Add a debug page or serial mode that prints Maix `dx/dy/area` and ESP32 `[VISION_ARM]` lines side-by-side for the same object positions.
3. Temporarily disable reach and height control; make base yaw smoothly center the target first.
4. After yaw works, enable reach using calibrated area thresholds.
5. Only after stable tracking should grabbing/descending logic be added.
