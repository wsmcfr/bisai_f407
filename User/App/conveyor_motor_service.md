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
| MP157 主链路 | 二进制协议入口 | USART2 收到 `START_CYCLE/VISION_POS/STEPPER_PARAM_SET/ACTUATOR_POS_MOVE` 后转交本模块。 |
| Emm42 Response | 位置到位事件 | 配置为 `Reached` 或 `Both` 时 F4 可收到 `[addr FD 9F 6B]` 主动到位回包；若现场没有主动回包，F4 按步数、速度和安全余量估算到期后上报 `DONE status=5 estimated-done`。 |

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
| 查询运行参数 | F4 USART2 命令输入 + USART1 日志 | 向 USART2 发送 `BELTINFO` | USART1 调试口日志含 `track=<normal_speed_rpm> rpm, scan=<scan_speed_rpm> rpm`。 | 若没有 `track/scan` 字段，说明 F4 未烧录新固件、USART2 命令未到达或 USART1 日志线未接好。 |
| 验证上料速度 | F4 USART2 命令输入 + USART1 日志 | 先下发 `STEPPER_PARAM_SET` 设置 `scan_speed_rpm=60`，再向 USART2 发 `BELTSCAN` | 传送带按 60rpm 附近扫描，USART1 日志中 `BELTINFO` 显示 `scan=60 rpm`。 | 若仍按旧速度转，检查 MP157 是否收到 `ACK acked_cmd=0x42`、F4 是否在 USART1 打印 `Runtime config applied`。 |
| 验证对中速度 | F4 USART2 命令输入 + USART1 日志 | 设置 `normal_speed_rpm=137` 后向 USART2 发送 `BELTTRACK -120` | USART1 日志中 `BELTINFO` 显示 `track=137 rpm`，实际跟踪速度不超过该值。 | 若速度仍受固定 80rpm 限制，检查 `ConveyorMotorService_MapErrorToSpeed()` 是否使用 `runtime->config.normal_speed_rpm`。 |
| 验证位置到位事件 | MP157 Qt 自动流程或二进制串口工具 | 发送 `ACTUATOR_POS_MOVE actuator=0 speed_rpm=0 steps>0` | F4 先 ACK；收到 `[addr FD 9F 6B]` 时发 `EVENT_REPORT event=0x14 status=0 related_seq=<本次SEQ>`；没有主动回包但估算运动时间到期时发 `event=0x14 status=5`；UART 读取错误或发送失败才发 `event=0x15`。 | 若总是 `status=5`，检查 Emm42 Response、UART4 RX `PC11`、地址和共地；流程不会再因缺少主动回包永久等待。 |

## 读写验证

| 数据路径 | 写入怎么做 | 读取/确认怎么做 |
|---|---|---|
| MP157 -> F4 参数 | MP157 发送 `STEPPER_PARAM_SET 0x42`，负载为 31 字节。 | F4 返回 `ACK status=0` 后，用 `BELTINFO` 读取 `track/scan`，用 `CAMINFO` 读取摄像头两轴参数。 |
| F4 -> 传送带 Emm42 | 本模块调用 `EMM42_MotorSetVelocity()`、`EMM42_MotorMoveRelativePosition()` 或 `EMM42_MotorStopNow()` 通过 UART4 发送帧。 | 观察传送带动作；位置模式优先读取 Emm42 `[addr FD 9F 6B]` 到位回包，未收到时按估算运动时间兜底。 |
| F4 -> MP157 到位事件 | 位置运动成功发送后，任务轮询 Emm42 到位回包并同步检查估算运动时间。 | 收到主动回包时发送 `EVENT_REPORT event=0x14 status=0`；估算完成时发送 `event=0x14 status=5`；真实通信错误发送 `event=0x15`，MP157 自动流程只能用该事件推进。 |
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
| 2026-07-05 | 位置运动完成机制改为“主动到位回包优先、估算完成兜底”：张大头官方位置模式例程没有证明默认一定主动返回完成帧，所以没有 `[addr FD 9F 6B]` 时不再发 `TIMEOUT` 卡住 MP157，而是发 `EVENT_REPORT event=0x14 status=5 estimated-done`。 |
| 2026-07-10 | 安全 STOP 不再因为 `applied_mode==STOP` 跳过底层发送；每次 STOP 都向 UART4 发送 `[addr FE 98 00 6B]`，并输出发送结果、任务栈水位、当前堆和历史最小堆。 |
| 2026-07-16 | MP157 主链路迁移到 USART2(PA2/PA3)，传送带 STOP 和运行日志从 USART1(PA9) 调试口输出。 |

## 2026-07-10 STOP 故障验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认传送带真实执行停止 | F4 USART1 调试口 `PA9(TX)`，115200 8N1 | MP157 自动流程触发超时 STOP，或通过 USART2 发送 `ACTUATOR_STOP actuator=0xFF` | USART1 出现 `[STOP][BELT] addr=1 uart=4 status=0 ...`，传送带停止。 | `status!=0` 时检查 UART4、PC10、驱动器 RX、共地和供电；完全无日志时先查 STOP 是否到达协议层和队列，再查 PA9 日志线。 |
| 判断是否任务栈不足 | F4 USART1 调试口 | 观察 `[STOP][BELT]` 中的 `stack_hw` | 数值明显大于 0，且运行中不持续逼近 0。 | 出现 `[FATAL] F4 STACK OVERFLOW` 或 `stack_hw` 接近 0 才增加对应任务栈。 |
| 判断是否 FreeRTOS 堆不足 | F4 USART1 调试口 | 观察 `heap`、`heap_min` | 两者大于 0；`heap_min` 是上电以来最差余量。 | 出现 `[FATAL] F4 MALLOC FAILED` 才确认动态堆耗尽，并检查任务、队列和信号量创建。 |
