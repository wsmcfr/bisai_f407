# F4 传送带电机服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/conveyor_motor_service.c`、`User/App/conveyor_motor_service.h` |
| 模块用途 | 管理 F407 到传送带张大头 Emm42 步进电机的 UART4 通信、SCAN 上料、TRACK 视觉对中、手动速度运动、相对位置运动、当前位置设零和运行时参数更新。 |
| 当前阶段 | 已接入 MP157 二进制自动流程，支持传送带双速度：`scan_speed_rpm` 用于未检测到零件时的上料扫描，`normal_speed_rpm` 用于检测到零件后的视觉对中和短步微调。 |

## 修改文件清单

| 文件 | 类型 | 修改原因 |
|---|---|---|
| `User/App/conveyor_motor_service.h` | 修改 | `ConveyorMotorService_RequestRuntimeConfig()` 增加 `scan_speed_rpm` 参数，使 MP157 参数页可以分别设置传送带上料扫描速度和对中/短步速度。 |
| `User/App/conveyor_motor_service.c` | 修改 | 运行时配置增加 `scan_speed_rpm`；`BELTSCAN/START_CYCLE` 使用上料扫描速度；`VISION_POS/BELTTRACK/ACTUATOR_POS_MOVE actuator=0 speed_rpm=0` 使用 `normal_speed_rpm`；`BELTINFO` 输出 `track/scan` 两档速度。 |
| `User/App/binary_protocol_service.c` | 修改 | `STEPPER_PARAM_SET 0x42` 解码从旧 25 字节扩展为 31 字节，并把传送带 `scan_speed_rpm` 传入本模块。 |
| `User/App/binary_protocol_service.h` | 修改 | 步进参数结构体增加 `scan_speed_rpm` 字段，协议常量更新为 31 字节负载。 |
| `User/App/binary_protocol_service_host_test.c` | 修改 | 主机测试按 31 字节负载构造 `STEPPER_PARAM_SET`，覆盖 `normal_speed_rpm` 和 `scan_speed_rpm` 解码。 |

## 硬件资源

| 资源 | 用途 | 约束 |
|---|---|---|
| `UART4` | F4 到传送带 Emm42 的 TTL 串口 | 115200 8N1，`PC10=TX`、`PC11=RX`。 |
| Emm42 地址 | 传送带电机站号 | 默认 `0x01`，可由 MP157 `STEPPER_PARAM_SET` 运行时更新，合法范围 `1~247`。 |
| FreeRTOS 队列 | 协议层向传送带任务投递控制命令 | 队列长度为 1，使用覆盖写入，视觉坐标以最新误差为准。 |
| MP157 主链路 | 二进制协议入口 | USART1 收到 `START_CYCLE/VISION_POS/STEPPER_PARAM_SET/ACTUATOR_POS_MOVE` 后转交本模块。 |
| Emm42 Response | 位置到位事件 | 位置模式必须配置为 `Reached` 或 `Both`，否则 F4 收不到 `[addr FD 9F 6B]` 到位回包。 |

## 使用方法

| 场景 | 操作 |
|---|---|
| 上料扫描 | MP157 首页发送 `START_CYCLE`，或串口助手发送文本 `BELTSCAN`，传送带使用当前 `scan_speed_rpm` 转动。 |
| 视觉对中 | MP157 周期发送 `VISION_POS`，协议层计算 `axis_px-target_px` 后调用 `ConveyorMotorService_RequestTrack()`，传送带速度上限由 `normal_speed_rpm` 控制。 |
| 手动持续运动 | MP157 手动三轴弹窗发送 `ACTUATOR_VEL_MOVE actuator=0`，传送带持续运动到 `ACTUATOR_STOP`。 |
| 短步微调 | MP157 自动 ROI 复查时发送 `ACTUATOR_POS_MOVE actuator=0 speed_rpm=0 steps>0`，本模块使用 `normal_speed_rpm` 执行相对位置运动。 |
| 参数同步 | MP157 参数页保存或 Qt 开机后发送 `STEPPER_PARAM_SET 0x42`，F4 更新运行内存，不写 Flash 或 Emm42 EEPROM。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 主机协议测试 | `E:\hal\bisai_f407_project` | `gcc -std=c99 -Wall -Wextra -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\binary_protocol_service_host_test.c User\App\binary_protocol_service.c -o tmp\binary_protocol_service_host_test.exe; .\tmp\binary_protocol_service_host_test.exe` | 输出 `binary protocol host tests passed`，`STEPPER_PARAM_SET` 按 31 字节负载解码。 | 若提示长度错误，检查 `BINARY_PROTOCOL_STEPPER_PARAM_PAYLOAD_LENGTH` 是否为 31，单条记录是否为 9 字节。 |
| Keil 编译 | `E:\hal\bisai_f407_project` | 使用 Keil/命令行构建 `MDK-ARM\bisai_f407_project.uvprojx` | 编译 0 Error；warning 需要逐条确认是否与本次修改相关。 | 若 `ConveyorMotorService_RequestRuntimeConfig` 参数不匹配，检查 `binary_protocol_service.c`、`conveyor_motor_service.h` 和调用点是否同步。 |
| 查询运行参数 | F4 USART1 串口助手 | `BELTINFO` | 日志含 `track=<normal_speed_rpm> rpm, scan=<scan_speed_rpm> rpm`。 | 若没有 `track/scan` 字段，说明 F4 未烧录新固件或串口连接到旧程序。 |
| 验证上料速度 | F4 USART1 串口助手 | 先下发 `STEPPER_PARAM_SET` 设置 `scan_speed_rpm=60`，再发 `BELTSCAN` | 传送带按 60rpm 附近扫描，`BELTINFO` 显示 `scan=60 rpm`。 | 若仍按旧速度转，检查 MP157 是否收到 `ACK acked_cmd=0x42`、F4 是否打印 `Runtime config applied`。 |
| 验证对中速度 | F4 USART1 串口助手 | 设置 `normal_speed_rpm=137` 后发送 `BELTTRACK -120` | `BELTINFO` 显示 `track=137 rpm`，实际跟踪速度不超过该值。 | 若速度仍受固定 80rpm 限制，检查 `ConveyorMotorService_MapErrorToSpeed()` 是否使用 `runtime->config.normal_speed_rpm`。 |
| 验证位置到位事件 | MP157 Qt 自动流程或二进制串口工具 | 发送 `ACTUATOR_POS_MOVE actuator=0 speed_rpm=0 steps>0` | F4 先 ACK，传送带到位后再发 `EVENT_REPORT event=0x14 related_seq=<本次SEQ>`。 | 若只有 ACK 没 DONE，检查 Emm42 Response、UART4 RX `PC11`、地址和共地。 |

## 读写验证

| 数据路径 | 写入怎么做 | 读取/确认怎么做 |
|---|---|---|
| MP157 -> F4 参数 | MP157 发送 `STEPPER_PARAM_SET 0x42`，负载为 31 字节。 | F4 返回 `ACK status=0` 后，用 `BELTINFO` 读取 `track/scan`，用 `CAMINFO` 读取摄像头两轴参数。 |
| F4 -> 传送带 Emm42 | 本模块调用 `EMM42_MotorSetVelocity()`、`EMM42_MotorMoveRelativePosition()` 或 `EMM42_MotorStopNow()` 通过 UART4 发送帧。 | 观察传送带动作；位置模式还要读取 Emm42 `[addr FD 9F 6B]` 到位回包。 |
| F4 -> MP157 到位事件 | 位置运动成功发送后，任务轮询 Emm42 到位回包。 | 到位时发送 `EVENT_REPORT event=0x14`，超时或串口异常发送 `event=0x15`，MP157 自动流程只能用该事件推进。 |
| 文本调试命令 | 串口助手发送 `BELTSCAN/BELTTRACK/BELTSTOP/BELTINFO`。 | 只用于现场单模块排查；正式自动流程以二进制命令为准。 |

## 失败排查

| 现象 | 优先排查 |
|---|---|
| MP157 参数页保存后传送带速度不变 | 确认 MP157 已部署新 Qt 二进制、F4 已烧录新固件、`STEPPER_PARAM_SET` 收到 `ACK status=0`，再看 `BELTINFO track/scan`。 |
| 收到 `NACK error_code=4` | 负载长度不匹配，优先检查 MP157 是否仍发送旧 25 字节负载；当前必须为 31 字节。 |
| 收到 `NACK error_code=5` | 字段越界，检查地址 `1~247`、步长 `1~10000`、速度 `0~5000`、方向 `1/-1`。 |
| 自动流程 Z 轴等待时提前推进 | 问题不在本模块的 ACK，而在 MP157 必须等 `EVENT_REPORT event=0x14`；检查 C++ `runF4ActuatorPositionMoveAndWaitDone()` 和 QML `autoVisionHandleActuatorMoveDone()`。 |
| 传送带越调越远 | 先确认 `BELTTRACK -80` 是否沿扫描送入方向运动，再检查 `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW` 和 MP157 参数页传送带 `direction`。 |
| 传送带小误差不动 | 先看 `normal_speed_rpm` 是否被设为 0；若非 0 仍不动，再调高对中速度或最小爬行速度，并确认机械负载和供电。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-05 | 新增本模块说明文档，记录传送带服务、双速度参数、读写验证和失败排查。 |
| 2026-07-05 | `ConveyorMotorService_RequestRuntimeConfig()` 增加 `scan_speed_rpm`，传送带 SCAN 使用上料速度，TRACK 和短步位置运动使用 `normal_speed_rpm`。 |
| 2026-07-05 | `BELTINFO` 增加 `track/scan` 输出，用于现场确认 MP157 参数页是否同步到 F4 运行内存。 |
