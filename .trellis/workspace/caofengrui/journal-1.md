# Journal - caofengrui (Part 1)

> AI development session journal
> Started: 2026-04-17

---



## Session 1: FreeRTOS weight UART session without hardware validation

**Date**: 2026-04-18
**Task**: FreeRTOS weight UART session without hardware validation
**Branch**: `main`

### Summary

Implemented the FreeRTOS weight-query firmware changes, but this session did not include any on-board hardware validation.

### Main Changes

| Item | Details |
|------|---------|
| Work Summary | Implemented the FreeRTOS weight task, HX711 driver split, UART command-on-demand response, and Keil build verification notes. |
| Hardware Validation | Not performed in this session because the physical board was not available. |
| Build Evidence | Keil rebuild success was verified from `MDK-ARM/bisai_f407_project/bisai_f407_project.build_log.htm` and current `.axf` / `.hex` artifacts, not from partial terminal output alone. |
| Task State | The task must remain in progress until on-board verification is completed. |

**Testing**
- [NO] No physical board validation was performed in this session.
- [NO] No live HX711 load-cell measurement was checked on hardware.
- [NO] No UART command round-trip was verified on the target board.

**Next Steps**
- Re-test on the actual STM32F407 board with the HX711 connected.
- Send `GET` from the PC and confirm that weight is returned only on command.
- Re-check calibration constants for the 5 kg load cell after real measurements.


### Git Commits

| Hash | Message |
|------|---------|
| `dc34f7f` | (see git log) |

### Testing

- [NO] No physical board validation was performed in this session.
- [NO] No live HX711 measurement was verified on target hardware.
- [NO] No UART `GET` command/response was verified on the STM32F407 board.

### Status

[P] **In Progress** - firmware changes exist, but hardware validation is still pending.

### Next Steps

- Re-test on the actual STM32F407 board when the hardware is available.
- Verify that UART only returns weight after receiving `GET`.
- Validate HX711 wiring, tare behavior, and 5 kg load-cell calibration on real hardware.


## Session 2: FreeRTOS 电子称串口查询与标定完成

**Date**: 2026-04-19
**Task**: FreeRTOS 电子称串口查询与标定完成
**Branch**: `main`

### Summary

(Add summary)

### Main Changes

| 项目 | 内容 |
|---|---|
| 功能完成 | 基于 FreeRTOS 实现 HX711 电子称采样、串口指令查询、去皮与已知重量标定 |
| 串口方案 | USART1 使用 DMA + 空闲中断接收，按完整帧唤醒任务处理，仅在收到指令时返回当前重量 |
| 驱动与应用分层 | 驱动层放在 `User/Driver`，应用层放在 `User/App`，补充了中文函数头注释和关键逻辑注释 |
| 引脚与配置 | 当前 HX711 接线为 `DOUT->PB0`、`SCK->PB2`，CubeMX GPIO 配置已同步 |
| 称重策略 | 增加 5 点中值滤波、启动去皮失败处理、`GET/TARE/CAL <g>` 命令入口和 5kg 量程标定入口 |
| 规范沉淀 | 更新 `.trellis/spec/backend/quality-guidelines.md`，补充 STM32 FreeRTOS 共享资源、任务设计和实时性约束 |
| 实测状态 | 用户已完成标定，反馈测试比较成功；本次 session 记录为已有人体外设实测验证 |


### Git Commits

| Hash | Message |
|------|---------|
| `c137d64` | (see git log) |

### Testing

- [OK] 记录此前已通过 `python -m unittest Vision.maixcam2.tests.test_vision_core`。
- [OK] 记录此前 ESP32 Arduino 工程可编译通过。
- [WARN] 当前未重新运行 Keil/MDK；STM32 工程改动等待用户在 Keil 中验证。
- [WARN] 当前视觉跟踪控制效果未验收，后续仍需继续调试。

### Status

[WARN] **Session recorded; tracking control remains open**

### Next Steps

- 固定 I2C6 A1/A0 与 ESP-IDF slave，不再重复改线或改回 Arduino Wire slave。
- 采集 Maix `dx/dy/area` 与 ESP32 `[VISION_ARM]` 日志，先做 base-only 底座居中调试。
- 底座跟踪流畅后，再低频加入伸缩、高度和抓取控制。


## Session 3: 仓库规范化与构建产物清理

**Date**: 2026-04-19
**Task**: 仓库规范化与构建产物清理
**Branch**: `main`

### Summary

(Add summary)

### Main Changes

| 项目 | 内容 |
|---|---|
| 代码状态 | 电子称功能提交后，继续完成仓库整洁性治理并保持 `main` 分支干净 |
| 换行符治理 | 新增 `.gitattributes` 与 `.editorconfig`，统一源码/配置/文档为 `LF`，`*.bat` 保持 `CRLF` |
| Git 本地策略 | 本仓库已覆盖为 `core.autocrlf=false`、`core.eol=lf`、`core.safecrlf=warn`，避免 Windows 下反复出现 CRLF 脏文件 |
| 脏文件结论 | 已确认 `Core/Src/freertos.c` 当时属于纯行尾/索引状态问题，不是漏提业务代码 |
| MDK-ARM 清理 | 将 `uvguix/uvoptx/axf/hex/map/htm/lnp/dep/crf/d/o/lst` 等 Keil 构建产物从 Git 索引移除，并交由 `.gitignore` 管控 |
| 工程保留项 | 保留 `MDK-ARM/*.uvprojx`、链接脚本、startup 源文件、`RTE` 与 `DebugConfig` 等工程必需文件 |
| 参考资料处理 | `HX711相关资料/` 当前整目录忽略，本地保留，不纳入仓库 |
| 相关提交 | `919834d` 统一换行符规范；`31aa0d6` 清理 Keil 构建产物并忽略参考资料 |


### Git Commits

| Hash | Message |
|------|---------|
| `919834d` | (see git log) |
| `31aa0d6` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 4: LDC1614 标定采样模式联调

**Date**: 2026-04-19
**Task**: LDC1614 标定采样模式联调
**Branch**: `main`

### Summary

LDC1614 标定采样模式已跑通；单轮稳定，跨轮差异主要来自放置位置/姿态，需要非金属治具限位后再定参考值与容差。

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `b70b8f1` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 5: Emm42 conveyor integration and startup recovery

**Date**: 2026-04-21
**Task**: Emm42 conveyor integration and startup recovery
**Branch**: `main`

### Summary

(Add summary)

### Main Changes

| Item | Details |
|------|---------|
| Scope | Completed Emm42 conveyor control integration, startup self-healing config, heartbeat task glue, LDC startup false-trigger stabilization, and spec/documentation sync |
| Firmware Commits | `cd418c0` ignore local reference materials; `1e483ca` add Emm42 conveyor control and stabilize LDC startup; `a9fae94` document startup recovery and persistence rules |
| Hardware / Build Verification | User provided Keil build success, AXF download success, board boot logs, and startup logs confirming `ctrl=CLOSED_LOOP_FOC`, `startup_fix`, and `btn_lock=1` |
| Key Decisions | External module startup recovery must distinguish force-apply flags from target-state flags; routine boot keeps external-module save target in RAM/volatile mode rather than FLASH |
| Repo Hygiene | Reference material directories were excluded from Git via `.gitignore`; only source, project config, and spec/docs were pushed |

**Delivered**:
- Added `User/Driver/emm42_motor.c/.h` to wrap Emm42 TTL control-mode, lock-button, velocity, and stop commands.
- Added `User/App/conveyor_motor_service.c/.h` for SCAN/TRACK/STOP state-machine control and startup recovery.
- Added `User/App/system_heartbeat_service.c/.h` and FreeRTOS glue in `Core/Src/freertos.c`.
- Updated CubeMX/Keil-integrated files (`usart.c`, `gpio.c`, `dma.c`, `stm32f4xx_it.c`, `.ioc`, `.uvprojx`) to support USART2 motor control and board heartbeat behavior.
- Stabilized LDC1614 startup detection behavior and improved early-removal logging.
- Synced executable specs in `.trellis/spec/backend/quality-guidelines.md` and `.trellis/spec/backend/database-guidelines.md`.
- Added `User/App/emm42_conveyor_code_guide.md` to explain the Emm42 chain and related bootstrap files.

**Manual Verification Notes**:
- This session did not include a new full physical regression sweep with all modules together.
- The user confirmed compile/download success and provided live startup logs from the board.
- A remaining recommended manual spot-check is to confirm motor panel keys are physically ineffective after boot when `btn_lock=1`.


### Git Commits

| Hash | Message |
|------|---------|
| `cd418c0` | (see git log) |
| `1e483ca` | (see git log) |
| `a9fae94` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 6: STM32 LeArm 串口桥接与协议注释沉淀

**Date**: 2026-05-01
**Task**: STM32 LeArm 串口桥接与协议注释沉淀
**Branch**: `main`

### Summary

完成 ESP32 LeArm 机械臂 USART3 9600 桥接和 USART1 原始帧透传；启动阶段发送 STM32 通讯模式握手和版本探测；补充机械臂、传送带、LDC、HX711、Emm42、USART1 命令底座的协议表和中文注释规范；代码已提交并推送到 origin/main，Keil 编译由用户执行并反馈结果。

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `d092c0f` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 7: Record robot arm UART recovery and diagnostics

**Date**: 2026-05-01
**Task**: Record robot arm UART recovery and diagnostics
**Branch**: `main`

### Summary

Fixed robot arm UART diagnostics after human testing: made ARMCC5-safe ASCII runtime logs, documented no-reply motion commands, expanded UART log buffer, and captured ESP32 link-mode recovery context.

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `3b76725` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 8: Robot arm MaixCAM2 vision and vacuum wiring plan

**Date**: 2026-05-09
**Task**: Robot arm MaixCAM2 vision and vacuum wiring plan
**Branch**: `main`

### Summary

Documented the robot-arm end-effector architecture: F4 remains the high-level coordinator, ESP32 owns MaixCAM2 vision-result consumption, visual pick state, bus-servo motion, and PWM-controlled vacuum pump/release valve. Added wiring summary and updated frontend component code-spec with executable contracts, validation matrix, and test points.

### Main Changes

| Area | Details |
|------|---------|
| Wiring documentation | Added `docs/robot-arm-vision-vacuum-wiring.md` with F4-ESP32 J2 wiring, MaixCAM2 I2C path, suction pump and release valve PWM electronic-switch wiring, connector ownership, and power/common-ground requirements. |
| Code-spec memory | Added the `End-Effector Vision and Vacuum Gripper Integration` scenario to `.trellis/spec/frontend/component-guidelines.md`, including controller ownership, payload fields, validation matrix, good/base/bad cases, and required tests. |
| Architecture decision | Captured that STM32F407 remains the high-level coordinator while ESP32 owns MaixCAM2 result consumption, arm motion, vacuum pump control, release valve control, and pick-state progression. |

### Git Commits

| Hash | Message |
|------|---------|
| `05fd1a5` | docs(arm): record vision vacuum wiring plan |

### Testing

- [OK] Reviewed `D:\机械臂\SCH_3in1 Servo Controller V3.1.pdf` and ESP32 `Config.h` to confirm J2, I2C, bus-servo, PWM-servo, and GPIO ownership.
- [OK] Verified the committed diff only included the wiring document and frontend code-spec update.
- [NO] No hardware wiring, firmware compile, or on-board pick test was performed in this documentation session.

### Status

[OK] **Completed**

### Next Steps

- Implement the ESP32 MaixCAM2 I2C reader, visual pick state machine, and vacuum pump/release-valve control after hardware wiring is finalized.
- Add STM32F407 `ARMGRAB/ARMSTOP/ARMSTATUS` commands only after the ESP32 command IDs and status frames are fixed.


## Session 9: MaixCAM2视觉I2C联调与跟踪状态记录

**Date**: 2026-05-10
**Task**: MaixCAM2视觉I2C联调与跟踪状态记录
**Branch**: `main`

### Summary

记录 MaixCAM2 颜色识别、I2C6 到 ESP32 通讯打通、卡顿优化经验，以及当前视觉跟踪控制仍需继续优化的状态。

### Main Changes

| 模块 | 本次记录内容 |
|------|--------------|
| MaixCAM2视觉工程 | 创建 `Vision/maixcam2`，包含主程序、阈值设置、I2C探测脚本、README、测试和参考代码目录。 |
| 颜色识别 | 默认追踪颜色切换为黄色，使用用户提供的 LAB 阈值 `L=55..100, A=-80..20, B=40..127`；阈值调节长按步进周期改为 100ms。 |
| 触摸界面 | 保留二值化调参界面与追踪抓取界面，修正触摸点击和长按调节体验。 |
| I2C链路 | 经过多轮排查，确定 MaixCAM2 使用 I2C6，`A1=SCL`、`A0=SDA`、50kHz；ESP32 使用 ESP-IDF 原生 I2C slave，`GPIO16=SCL`、`GPIO17=SDA`、地址 `0x42`。 |
| 卡顿问题 | 记录经验：Maix 主循环不能直接阻塞式扫总线或写 I2C，I2C 操作必须放到 worker，失败后关闭 LINK，避免画面卡死。 |
| ESP32机械臂程序 | 将机械臂相关源码复制到 `Vision/maixcam2/reference/esp32_factory_base/LeArm_ESP32_Arduino`，补齐 Arduino IDE 直接打开所需依赖，并加入视觉 I2C 接收逻辑。 |
| 当前遗留问题 | 通讯层已经打通，但视觉跟踪控制效果仍未验收：目标偏离中心时动作小/停止，前端舵机抖动明显，后续应先采集 `dx/dy/area` 与 `[VISION_ARM]` 日志再调控制律。 |
| 验证记录 | 已记录曾通过 `python -m unittest Vision.maixcam2.tests.test_vision_core`，以及 ESP32 Arduino 编译通过；当前会话未重新运行 Keil/MDK。 |

**下一次继续建议**：固定 I2C 不再反复换线；先做 base-only 底座居中调试，确认 Maix 的 `dx/dy/area` 和 ESP32 日志方向一致，再低频加入伸缩和高度控制。


### Git Commits

| Hash | Message |
|------|---------|
| `997d194` | chore: record MaixCAM2 vision i2c session |
| `b7a6b0c` | feat(vision): add MaixCAM2 color tracking link |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 10: STM32F407 peripheral DMA and serial link update

**Date**: 2026-05-26
**Task**: STM32F407 peripheral DMA and serial link update
**Branch**: `main`

### Summary

(Add summary)

### Main Changes

| 项目 | 本次记录 |
|---|---|
| 固件验证 | 用户在 Keil/MDK-ARM 编译通过：`0 Error(s), 0 Warning(s)`，生成 axf/hex。 |
| Git 提交 | `e7d7ab4 feat(firmware): update peripheral links and DMA IO`，已推送到 `origin/main`。 |
| LDC1614 I2C | I2C2 改为 PF0/PF1，INTB 改为 PF2 EXTI 下降沿；I2C2 RX/TX 启用 DMA；驱动读写改为 `HAL_I2C_Mem_Read_DMA` / `HAL_I2C_Mem_Write_DMA`，通过 FreeRTOS 静态二值信号量等待 DMA/I2C 回调完成。 |
| 机械臂 ESP32 | USART3 与 ESP32 PA5/PA4 串口统一为 115200 8N1，源码、CubeMX `.ioc`、运行日志和参考文档均同步。 |
| 传送带 Emm42 | 旧 USART2/PA2/PA3 链路迁移到 USART6/PC6/PC7；`ConveyorMotorService_Task()` 绑定 `huart6`，驱动注释、启动日志和 Emm42 代码指南同步更新。 |
| MP157 心跳 | 新增 `User/App/mp157_f4_heartbeat.md` 记录 F407 USART1 `STATUS\r\n` 心跳协议和验证方式。 |
| Vision/ESP32 参考 | MaixCAM2/ESP32 参考侧同步串口波特率和相关说明，避免后续资料回拷时恢复旧 9600。 |
| 未纳入提交 | `.claude/`、BOM、Gerber、网表和 EasyEDA 工程导出文件仍保持未跟踪本地状态。 |

**关键文件**：
- `bisai_f407_project.ioc`
- `Core/Src/i2c.c`
- `Core/Src/usart.c`
- `Core/Src/stm32f4xx_it.c`
- `Core/Src/freertos.c`
- `User/Driver/ldc1614.c`
- `User/App/conveyor_motor_service.c`
- `User/Driver/emm42_motor.c`
- `User/App/robot_arm_service.c`
- `Vision/maixcam2/reference/esp32_factory_base/LeArm_ESP32_Arduino/LeArm_ESP32_Arduino.ino`


### Git Commits

| Hash | Message |
|------|---------|
| `e7d7ab4` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 11: Conveyor dual stepper and power PCB review record

**Date**: 2026-06-05
**Task**: Conveyor dual stepper and power PCB review record
**Branch**: `main`

### Summary

Recorded the conveyor dual-stepper hardware plan and the EasyEDA/AP63205 power-layout review notes.

### Main Changes

| 项目 | 本次记录 |
|---|---|
| 嘉立创EDA电源设计 | 复查 3S 锂电输入、自恢复保险丝、TVS、P-MOS 防反接、Gate 下拉、G-S 稳压和 AP63205 降压布局；确认 GND 过孔到底层可用，SW/BST 节点不建议打到底层。 |
| AP63205 PCB 布局 | 多轮检查 `U3/C2/C3/C7/L1/C5/C6` 位置，建议输入/输出电容贴近芯片和电感，`SW` 回路短粗，GND 多过孔回到底层。 |
| 传送带步进电机 | 记录新增步进电机属于传送带；传送带机构现在按两个步进电机设计，不归机械臂、称重或 LDC 模块。 |
| 固件边界 | 当前固件文档仍说明单 Emm42 电机链路；第二个传送带电机后续接入前，需要单独规划地址、串口共享或新增串口、同步控制策略。 |
| 记录文件 | 更新 `User/App/emm42_conveyor_code_guide.md`，并追加本次 Trellis 会话记录。 |

**注意**：本次只做文档和会话记录，不修改嘉立创EDA工程文件，也不提交当前工作区里已有的固件/IOC/PDF未提交改动。


### Git Commits

(No commits - planning session)

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete
