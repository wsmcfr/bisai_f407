# 摄像头运动电机服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/camera_motor_service.c`、`User/App/camera_motor_service.h` |
| 模块用途 | 在 F407 上独占 `USART6 PC6/PC7`，串行控制摄像头前进/后退轴和上下轴两台张大头 Emm42 步进电机。 |
| 当前阶段 | 支持 `CAMINFO/CAMSTOP/CAMFWD/CAMZ` 文本调试命令，并支持 MP157 通过 `STEPPER_PARAM_SET 0x42` 更新两台摄像头电机运行时地址、最小步长、常规速度和方向。 |

## 本次修改文件

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/camera_motor_service.c` | 新增 `CAMERA_MOTOR_COMMAND_CONFIG`、运行时配置结构、方向映射、默认速度解析和配置快照。 | MP157 下发参数后，摄像头两轴点动可使用新的地址、速度和方向。 |
| `User/App/camera_motor_service.h` | 新增 `CameraMotorService_RequestRuntimeConfig()` 声明。 | 二进制协议层可以向摄像头电机任务投递参数配置，不直接操作 `USART6`。 |
| `User/App/binary_protocol_service.c` | `STEPPER_PARAM_SET` 校验通过后调用摄像头运行时配置接口。 | F4 收到 `0x42` 后会把 role 2/3 分发给本模块。 |
| `User/App/binary_protocol_service.md` | 记录 `0x42` 负载、验证命令和运行内存边界。 | 后续 MP157/F4 联调可按文档查字段。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| `USART6` | F4 到两个摄像头 Emm42 的 TTL 总线 | `PC6(TX) / PC7(RX)`，115200 8N1，两个电机共线必须靠地址区分 |
| 前进/后退轴 Emm42 | 摄像头前后方向点动 | 默认地址 `0x02`，运行时可配置为 `1~247` |
| 上下轴 Emm42 | 摄像头上下方向点动 | 默认地址 `0x03`，运行时可配置为 `1~247`，不能和前后轴相同 |
| FreeRTOS 队列 | 协议层/文本命令到电机任务的异步控制 | 长度为 1 的覆盖队列，只保留最新 STOP/JOG/CONFIG 意图 |

## 接口契约

| 接口 | 作用 | 参数边界 |
|---|---|---|
| `CameraMotorService_RequestForwardJog(forward_flag, speed_rpm)` | 请求前后轴点动。 | `speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestZJog(up_flag, speed_rpm)` | 请求上下轴点动。 | `speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestStopAll()` | 请求两个摄像头轴立即停止。 | 无额外参数。 |
| `CameraMotorService_RequestRuntimeConfig(...)` | 更新两个摄像头轴运行时地址、最小步长、常规速度和方向。 | 地址 `1~247` 且两轴不同；步长 `1~10000`；速度 `0~5000 rpm`；方向只能 `1/-1`。 |

说明：运行时配置只保存在 F4 RAM；不会写 F4 Flash，也不会写 Emm42 EEPROM。F4 断电重启后恢复代码默认值，MP157 需要重新下发。

## 使用与验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 查询当前运行时配置 | USART1 串口助手 | `CAMINFO` | 输出 `USART6=PC6/PC7`、`forward_addr`、`z_addr`、`min_step`、`speed`、`dir` 和最近动作。 | 若未知命令，检查 `weight_service.c` 是否调用 `CameraMotorService_HandleCommand()`；若地址不对，确认 F4 是否已烧录新固件。 |
| 前后轴默认速度点动 | USART1 串口助手 | `CAMFWD FORWARD` | 前后轴按当前 `forward_normal_speed_rpm` 点动；上电默认 `30 rpm`。 | 若不动，检查 `USART6` 接线、地址、使能和供电；若速度不符，先发 `CAMINFO` 查运行时速度。 |
| 上下轴指定速度点动 | USART1 串口助手 | `CAMZ UP 137` | 上下轴以 `137 rpm` 点动。 | 若速度被截断，检查是否已使用支持 `0~5000 rpm` 的新固件。 |
| 停止两个摄像头轴 | USART1 串口助手 | `CAMSTOP` | 前后轴和上下轴都停止。 | 检查两个电机地址是否不同，`USART6` 是否被其它任务占用。 |
| MP157 下发参数 | MP157 Qt 参数页 | `参数设置 -> 步进参数 -> 保存并下发` | F4 返回 `ACK acked_cmd=0x42 status=0`；再发 `CAMINFO` 可看到地址、步长、速度、方向变化。 | 若返回 `NACK error_code=5`，检查字段范围；若返回 `error_code=10`，检查摄像头电机任务队列是否已创建。 |

## 读写验证

| 数据通路 | 写操作 | 读/确认操作 | 判定 |
|---|---|---|---|
| USART1 文本到摄像头任务 | 发送 `CAMFWD FORWARD 137` 或 `CAMZ UP 137`。 | 观察对应轴动作，再发送 `CAMSTOP`。 | 只有目标轴动作，说明文本分发和 `USART6` 地址区分正常。 |
| MP157 二进制参数到摄像头任务 | 发送 `STEPPER_PARAM_SET 0x42`，role 2/3 分别填写摄像头前后和上下轴参数。 | F4 回 ACK 后发送 `CAMINFO`。 | `CAMINFO` 中 role 2/3 对应地址、速度、步长、方向与 MP157 参数页一致。 |
| 运行时速度为 0 | 把某个摄像头轴常规速度下发为 `0`，再发省略 rpm 的 `CAMFWD FORWARD` 或 `CAMZ UP`。 | 目标轴保持停止，最近动作速度显示 `0 rpm`。 | 若仍转动，检查 `CameraMotorService_ResolveJogSpeed()` 是否使用运行时配置。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-03 | 新增摄像头两轴运行时配置队列命令，支持 MP157 下发地址、最小步长、常规速度和方向。 |
| 2026-07-03 | 文本点动命令省略 rpm 时改用运行时常规速度；速度范围对齐 Emm42 底层 `0~5000 rpm`。 |
