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

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


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
