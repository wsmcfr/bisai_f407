# 摄像头运动电机服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/camera_motor_service.c`、`User/App/camera_motor_service.h` |
| 模块用途 | 在 F407 上独占 `USART6 PC6/PC7`，串行控制摄像头前进/后退轴和上下轴两台张大头 Emm42 步进电机。 |
| 当前阶段 | 支持 `CAMINFO/CAMSTOP/CAMFWD/CAMZ` 文本调试命令，支持 MP157 通过 `STEPPER_PARAM_SET 0x42` 更新两台摄像头电机运行时地址、最小步长、常规速度和方向；支持 `ACTUATOR_VEL_MOVE/ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_HOME` 控制前后轴连续运动、两轴固定步数、停止和当前位置设零。 |

## 本次修改文件

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/camera_motor_service.c` | 新增 `CAMERA_MOTOR_COMMAND_CONFIG`、运行时配置结构、方向映射、默认速度解析和配置快照；新增 `CAMERA_MOTOR_COMMAND_POSITION` 和 `CAMERA_MOTOR_COMMAND_SET_ZERO`。 | MP157 下发参数后，摄像头两轴点动可使用新的地址、速度和方向；前后轴/上下轴可按固定步数移动，当前位置可设为新的零点。 |
| `User/App/camera_motor_service.h` | 新增 `CameraMotorService_RequestRuntimeConfig()`、前后轴/上下轴位置移动和设零接口声明。 | 二进制协议层可以向摄像头电机任务投递参数、运动和设零请求，不直接操作 `USART6`。 |
| `User/App/binary_protocol_service.c` | `STEPPER_PARAM_SET` 校验通过后调用摄像头运行时配置接口；`ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE/ACTUATOR_HOME` 校验通过后调用摄像头电机服务。 | F4 收到 `0x42` 后会把 role 2/3 分发给本模块；收到 `0x50/0x51/0x52/0x53` 后按 actuator 1/2 控制摄像头两轴。 |
| `User/Driver/emm42_motor.c`、`User/Driver/emm42_motor.h` | 新增 `EMM42_MotorMoveRelativePosition()` 和 `EMM42_MotorResetCurrentPositionToZero()`。 | 摄像头两轴可使用张大头 `0xFD` 相对位置模式和 `[addr 0A 6D 6B]` 当前位置清零命令。 |
| `User/App/binary_protocol_service.md` | 记录 `0x42`、`0x50~0x53` 负载、验证命令和运行内存边界。 | 后续 MP157/F4 联调可按文档查字段。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| `USART6` | F4 到两个摄像头 Emm42 的 TTL 总线 | `PC6(TX) / PC7(RX)`，115200 8N1，两个电机共线必须靠地址区分 |
| 前进/后退轴 Emm42 | 摄像头前后方向点动 | 现场默认地址 `0x03`，运行时可配置为 `1~247` |
| 上下轴 Emm42 | 摄像头上下方向点动 | 现场默认地址 `0x02`，运行时可配置为 `1~247`，不能和前后轴相同 |
| FreeRTOS 队列 | 协议层/文本命令到电机任务的异步控制 | 长度为 4 的短 FIFO；STOP 走队首优先，并递增 `stop_epoch` 丢弃 STOP 之前残留的旧 JOG/POSITION/HOME 运动命令，避免停止后旧前后轴点动再次执行 |

## 接口契约

| 接口 | 作用 | 参数边界 |
|---|---|---|
| `CameraMotorService_RequestForwardJog(forward_flag, speed_rpm)` | 请求前后轴点动。 | `speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestZJog(up_flag, speed_rpm)` | 请求上下轴点动。 | `speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestForwardPosition(forward_flag, speed_rpm, pulse_count)` | 请求前后轴按相对位置模式移动固定步数。 | `pulse_count` 必须大于 0；`speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestZPosition(up_flag, speed_rpm, pulse_count)` | 请求上下轴按相对位置模式移动固定步数。 | `pulse_count` 必须大于 0；`speed_rpm=0` 时使用当前运行时常规速度。 |
| `CameraMotorService_RequestForwardSetCurrentPositionZero()` | 请求前后轴停止后把当前位置设为零点。 | 供 `ACTUATOR_HOME actuator=1` 使用，不主动寻找限位。 |
| `CameraMotorService_RequestZSetCurrentPositionZero()` | 请求上下轴停止后把当前位置设为零点。 | 供 `ACTUATOR_HOME actuator=2` 使用，不主动寻找限位。 |
| `CameraMotorService_RequestStopAll()` | 请求两个摄像头轴立即停止。 | STOP 入队时会开启新的 `stop_epoch`；旧的 JOG/POSITION/HOME 即使已经排队，也会在任务取出时被丢弃；CONFIG 不丢弃，避免参数 ACK 后未应用。 |
| `CameraMotorService_RequestRuntimeConfig(...)` | 更新两个摄像头轴运行时地址、最小步长、常规速度和方向。 | 地址 `1~247` 且两轴不同；步长 `1~10000`；速度 `0~5000 rpm`；方向只能 `1/-1`。 |

说明：运行时配置只保存在 F4 RAM；不会写 F4 Flash，也不会写 Emm42 EEPROM。F4 断电重启后恢复代码默认值，MP157 需要重新下发。

## 使用与验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 查询当前运行时配置 | USART1 串口助手 | `CAMINFO` | 输出 `USART6=PC6/PC7`、`forward_addr`、`z_addr`、`min_step`、`speed`、`dir` 和最近动作。 | 若未知命令，检查 `weight_service.c` 是否调用 `CameraMotorService_HandleCommand()`；若地址不对，确认 F4 是否已烧录新固件。 |
| 前后轴默认速度点动 | USART1 串口助手 | `CAMFWD FORWARD` | 前后轴按当前 `forward_normal_speed_rpm` 点动；上电默认 `30 rpm`。 | 若不动，检查 `USART6` 接线、地址、使能和供电；若速度不符，先发 `CAMINFO` 查运行时速度。 |
| 上下轴指定速度点动 | USART1 串口助手 | `CAMZ UP 137` | 上下轴以 `137 rpm` 点动。 | 若速度被截断，检查是否已使用支持 `0~5000 rpm` 的新固件。 |
| 停止两个摄像头轴 | USART1 串口助手 | `CAMSTOP` | 前后轴和上下轴都停止。 | 检查两个电机地址是否不同，`USART6` 是否被其它任务占用。 |
| MP157 下发参数 | MP157 Qt 参数页 | `参数设置 -> 步进参数 -> 保存并下发` | F4 返回 `ACK acked_cmd=0x42 status=0`；随后 USART1 输出 `Runtime config applied`；再发 `CAMINFO` 可看到地址、步长、速度、方向变化。 | 若返回 `NACK error_code=5`，检查字段范围；若返回 `error_code=10`，检查摄像头电机任务队列是否已创建或队列是否已满。 |
| MP157 前后轴持续运动 | MP157 Qt 手动三轴弹窗 | 切到前后轴页，点击前进或后退，再点击停止。 | F4 先返回 `ACK acked_cmd=0x52 status=0`，前后轴持续运动；停止时返回 `ACK acked_cmd=0x51 status=0`，USART1 日志出现 `[OK][CAM] Stop applied...`，若停止前还有旧 JOG 排队会出现 `Drop stale motion after STOP`。 | 若点击一次只动一下，确认 MP157 是否仍在发旧 `ACTUATOR_POS_MOVE`；若返回 `status=1/2`，确认 F4 是否烧录 ACK 修复；若 ACK 后仍继续动，确认是否已经烧录包含 `stop_epoch` 的新固件。 |
| MP157 上下轴固定步数 | MP157 Qt 手动三轴弹窗或首页自动流程 | 切到上下轴页，点击下降、上升或回原位。 | 每点一次只按 `zDownFixedSteps/zUpFixedSteps` 运动一次，回原位复用 `zUpFixedSteps`，F4 返回 `ACK acked_cmd=0x50 status=0`。 | 若一直动，确认上下轴没有走 `ACTUATOR_VEL_MOVE`；若不动，检查步数是否为 0、地址是否为 `0x02`。 |
| MP157 摄像头轴当前位置设零 | MP157 Qt 参数页步进弹窗 | 切到前后轴或上下轴，点击 `设当前位置为零点`。 | F4 返回 `ACK acked_cmd=0x53 status=0`；目标轴不主动运动，只把当前位置作为新的零点。 | 若返回未知命令，F4 不是最新固件；若 ACK 后无效果，检查 Emm42 `[addr 0A 6D 6B]` 支持和电机地址。 |

## 读写验证

| 数据通路 | 写操作 | 读/确认操作 | 判定 |
|---|---|---|---|
| USART1 文本到摄像头任务 | 发送 `CAMFWD FORWARD 137` 或 `CAMZ UP 137`。 | 观察对应轴动作，再发送 `CAMSTOP`。 | 只有目标轴动作，说明文本分发和 `USART6` 地址区分正常。 |
| MP157 二进制参数到摄像头任务 | 发送 `STEPPER_PARAM_SET 0x42`，role 2/3 分别填写摄像头前后和上下轴参数。 | F4 回 ACK 后发送 `CAMINFO`。 | `CAMINFO` 中 role 2/3 对应地址、速度、步长、方向与 MP157 参数页一致。 |
| MP157 二进制运动到摄像头任务 | 发送 `ACTUATOR_VEL_MOVE actuator=1`、`ACTUATOR_POS_MOVE actuator=1/2` 或 `ACTUATOR_STOP actuator=1/2/0xFF`。 | 观察目标轴动作或停机，再查 ACK 详情。 | ACK `status=0` 且只有目标轴动作，说明协议层、队列和 `USART6` 地址区分正常。 |
| MP157 二进制设零到摄像头任务 | 发送 `ACTUATOR_HOME actuator=1/2 flags=0`。 | 观察目标轴不主动运动，F4 返回 ACK；后续相对位置移动以新位置作为基准。 | ACK 成功但位置基准未变时，优先查 Emm42 命令兼容性和地址。 |
| 运行时速度为 0 | 把某个摄像头轴常规速度下发为 `0`，再发省略 rpm 的 `CAMFWD FORWARD` 或 `CAMZ UP`。 | 目标轴保持停止，最近动作速度显示 `0 rpm`。 | 若仍转动，检查 `CameraMotorService_ResolveJogSpeed()` 是否使用运行时配置。 |
| STOP 后旧运动命令丢弃 | 快速连续发送 `ACTUATOR_VEL_MOVE actuator=1` 和 `ACTUATOR_STOP actuator=1`。 | 前后轴停止，USART1 日志至少出现 `Stop applied`；若 STOP 插队时旧运动命令还在队列中，应出现 `Drop stale motion after STOP`。 | 若停止后又继续动，检查 `CameraMotorService_PostCommand()` 是否给 STOP 递增 `stop_epoch`，以及 `CameraMotorService_IsStaleMotionCommand()` 是否被编译进 `CameraMotorService_ApplyCommand()`。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-03 | 新增摄像头两轴运行时配置队列命令，支持 MP157 下发地址、最小步长、常规速度和方向。 |
| 2026-07-03 | 文本点动命令省略 rpm 时改用运行时常规速度；速度范围对齐 Emm42 底层 `0~5000 rpm`。 |
| 2026-07-04 | 新增摄像头前后轴 `ACTUATOR_VEL_MOVE` 连续速度运动，停止键通过 `ACTUATOR_STOP` 结束。 |
| 2026-07-04 | 新增摄像头前后轴/上下轴 `ACTUATOR_POS_MOVE` 固定步数位置运动，上下轴手动按钮每次只走 `zDownFixedSteps/zUpFixedSteps`。 |
| 2026-07-04 | 新增 `ACTUATOR_HOME` 当前位置设零，前后轴和上下轴分别停止后发送 Emm42 `[addr 0A 6D 6B]` 清零帧。 |
| 2026-07-04 | 现场默认地址改为前后轴 `0x03`、上下轴 `0x02`；摄像头任务队列改为长度 4 的 FIFO，STOP 队首优先，避免保存参数 ACK 后 CONFIG 被后续手动动作覆盖。 |
| 2026-07-04 | 修正前后轴停止无效：STOP 现在递增 `stop_epoch`，任务会丢弃 STOP 之前残留的旧 JOG/POSITION/HOME 运动命令，避免 STOP 执行后旧前后轴点动再次启动。 |
