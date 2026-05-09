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
