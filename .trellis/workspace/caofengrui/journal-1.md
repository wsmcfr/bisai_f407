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
