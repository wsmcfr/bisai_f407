# Emm42 Conveyor Code Guide

## 1. Scope

这份文档用于解释当前 F407 工程中张大头 Emm42 步进电机的传送带控制链路。

当前硬件绑定已经调整为：

| 功能 | Emm42 地址 ID | F4 串口 | F4 引脚 | 负责模块 |
|---|---:|---|---|---|
| 传送带电机 | 默认 `0x01` | `UART4` | `PC10(TX) / PC11(RX)` | `User/App/conveyor_motor_service.c` |
| 摄像头前进/后退电机 | 现场默认 `0x03` | `USART6` | `PC6(TX) / PC7(RX)` | `User/App/camera_motor_service.c` |
| 摄像头上下电机 | 现场默认 `0x02` | `USART6` | `PC6(TX) / PC7(RX)` | `User/App/camera_motor_service.c` |

结论：

| 问题 | 答案 |
|---|---|
| 传送带是否还使用 `USART6` | 不使用，传送带固定使用 `UART4 PC10/PC11`。 |
| 摄像头前进/后退步进电机 ID | 现场默认 `0x03`，可由 MP157 `STEPPER_PARAM_SET` 更新 F4 运行时参数 |
| 摄像头上下移动步进电机 ID | 现场默认 `0x02`，可由 MP157 `STEPPER_PARAM_SET` 更新 F4 运行时参数 |

## 2. Current File Map

| 文件 | 角色 | 当前关键点 |
|---|---|---|
| `Core/Src/usart.c` | 串口底层初始化 | `huart4` 对应 `UART4 PC10/PC11`，`huart6` 对应 `USART6 PC6/PC7`。 |
| `Core/Src/freertos.c` | FreeRTOS 任务启动胶水 | 创建传送带任务 `ConveyorMotorService_Task()` 和摄像头电机任务 `CameraMotorService_Task()`。 |
| `User/App/uart_command.c` | `USART1` 命令接收与线程安全打印 | 只负责收完整命令和安全打印，不直接控制电机。 |
| `User/App/weight_service.c` | `USART1` 统一命令分发入口 | 分发 `BELT...` 到传送带服务，分发 `CAM...` 到摄像头电机服务。 |
| `User/App/conveyor_motor_service.c` | 传送带状态机 | 绑定 `huart4`，默认地址 `0x01`；通过运行时配置支持地址、最小步长、常规速度和方向映射；支持 `ACTUATOR_VEL_MOVE/ACTUATOR_POS_MOVE/ACTUATOR_HOME`。 |
| `User/App/conveyor_motor_service.h` | 传送带服务公共接口 | 暴露任务入口、`RequestScan/RequestStop/RequestTrack`、`RequestRuntimeConfig`、`RequestJog`、`RequestPosition` 和 `RequestSetCurrentPositionZero`。 |
| `User/App/camera_motor_service.c` | 摄像头运动电机服务 | 绑定 `huart6`，现场默认前后轴地址 `0x03`、上下轴地址 `0x02`；通过运行时配置支持两轴地址、最小步长、常规速度和方向映射；支持前后轴持续速度运动、两轴固定步数位置运动和当前位置设零。 |
| `User/App/camera_motor_service.h` | 摄像头电机公共接口 | 暴露任务入口、`CAM...` 命令处理入口、`RequestRuntimeConfig`、前后轴/上下轴运动和设零接口。 |
| `User/App/binary_protocol_service.c/.h` | MP157-F4 二进制协议 | 新增 `STEPPER_PARAM_SET 0x42`，把 MP157 参数页的三台电机配置投递到对应电机任务；新增 `ACTUATOR_POS_MOVE/STOP/VEL_MOVE/HOME 0x50~0x53` 控制三台执行器。 |
| `User/Driver/emm42_motor.c` | Emm42 TTL 协议帧发送 | 通用驱动层，按句柄中的 `huart` 和 `address` 发送速度、停止、相对位置和当前位置清零命令。 |
| `User/Driver/emm42_motor.h` | Emm42 协议类型声明 | 定义电机句柄、方向、控制模式、速度模式、相对位置模式和当前位置清零接口。 |
| `User/App/emm42_motor_uart_binding.md` | 串口与 ID 绑定总文档 | 现场调试时优先查这份文档。 |

## 3. Data Flow

当前传送带从 MP157 或串口助手到电机的链路如下：

| 步骤 | 文件 | 关键函数 | 作用 |
|---:|---|---|---|
| 1 | `User/App/uart_command.c` | `HAL_UARTEx_RxEventCallback()` | `USART1` 通过 DMA + IDLE 收到一帧命令。 |
| 2 | `User/App/uart_command.c` | `UartCommand_Fetch()` / `UartCommand_FetchRaw()` | 把完整命令交给任务上下文。 |
| 3 | `User/App/weight_service.c` | `WeightService_ProcessCommand()` | 统一分发二进制协议、机械臂帧和 ASCII 文本命令。 |
| 4 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_HandleCommand()` | 解析 `BELTSCAN/BELTSTOP/BELTTRACK/BELTCAM/BELTENABLE/BELTINFO`。 |
| 5 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_RequestScan/Stop/Track/RuntimeConfig()` | 把控制意图或运行时参数投递到传送带任务队列。 |
| 6 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_Task()` | 独占 `UART4` 推进 `SCAN/TRACK/STOP` 状态机。 |
| 7 | `User/Driver/emm42_motor.c` | `EMM42_MotorSetVelocity()` / `EMM42_MotorStopNow()` | 按当前运行时地址组帧并通过 `UART4` 发送给传送带电机。 |

摄像头两个运动轴的调试链路如下：

| 步骤 | 文件 | 关键函数 | 作用 |
|---:|---|---|---|
| 1 | `User/App/weight_service.c` | `WeightService_ProcessCommand()` | 收到 `CAM...` 后转交摄像头电机服务。 |
| 2 | `User/App/camera_motor_service.c` | `CameraMotorService_HandleCommand()` | 解析 `CAMINFO/CAMSTOP/CAMFWD/CAMZ`。 |
| 3 | `User/App/camera_motor_service.c` | `CameraMotorService_Task()` | 独占 `USART6`，串行控制两个不同地址的 Emm42 电机。 |
| 4 | `User/Driver/emm42_motor.c` | `EMM42_MotorSetVelocity()` / `EMM42_MotorStopNow()` | 地址 `0x03` 控制前进/后退轴，地址 `0x02` 控制上下轴。 |

## 4. Command Quick Reference

### 4.1 传送带命令

| 命令 | 作用 | 实际硬件 |
|---|---|---|
| `BELTSCAN` | 传送带进入低速扫描。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTSTOP` | 传送带立即停止。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTTRACK <error>` | 按视觉误差进入跟踪。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTCAM <enable> <current_x> <center_x>` | F4 自己计算 `current_x - center_x` 后跟踪。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTENABLE <0|1> [error]` | 兼容主机视觉使能语义。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTINFO` | 查询传送带状态。 | 不动作，只打印快照。 |

### 4.2 摄像头电机命令

| 命令 | 作用 | 实际硬件 |
|---|---|---|
| `CAMINFO` | 查询摄像头两个电机的地址、串口和最近动作。 | 不动作，只打印快照。 |
| `CAMSTOP` | 同时停止前进/后退轴和上下轴。 | `USART6 PC6/PC7 addr=0x03/0x02` |
| `CAMFWD FORWARD [rpm]` | 摄像头前进/后退轴正向点动。 | `USART6 PC6/PC7 addr=0x03` |
| `CAMFWD BACKWARD [rpm]` | 摄像头前进/后退轴反向点动。 | `USART6 PC6/PC7 addr=0x03` |
| `CAMZ UP [rpm]` | 摄像头上下轴向上点动。 | `USART6 PC6/PC7 addr=0x02` |
| `CAMZ DOWN [rpm]` | 摄像头上下轴向下点动。 | `USART6 PC6/PC7 addr=0x02` |

### 4.3 MP157 二进制参数命令

| 命令 | 作用 | 生效范围 |
|---|---|---|
| `STEPPER_PARAM_SET 0x42` | 一次下发三台电机的地址、最小步长、常规速度和方向映射。 | 只更新 F4 运行内存；不会写 F4 Flash，也不会写 Emm42 EEPROM。 |
| `ACTUATOR_POS_MOVE 0x50` | 让传送带、前后轴或上下轴按相对位置模式移动固定步数。 | 三台执行器均支持；上下轴手动上升/下降用该命令。 |
| `ACTUATOR_STOP 0x51` | 停止指定执行器，`actuator=0xFF` 停止全部可停止执行器。 | 传送带、前后轴、上下轴。 |
| `ACTUATOR_VEL_MOVE 0x52` | 让传送带或前后轴按速度模式持续运动，直到收到 STOP。 | 只允许 `actuator=0/1`；上下轴不允许连续速度模式。 |
| `ACTUATOR_HOME 0x53` | 停止目标电机后把当前位置设为新的零点。 | 三台执行器均支持；不做主动回零运动，不支持 `actuator=0xFF`。 |

字段约束：地址 `1~247`，最小步长 `1~10000 step`，常规速度 `0~5000 rpm`，方向只能 `1/-1`。其中传送带常规速度用于扫描，摄像头两轴常规速度用于省略 rpm 时的默认点动速度。

## 5. Startup Behavior

| 阶段 | 传送带任务行为 |
|---|---|
| 创建队列 | 创建长度为 1 的覆盖队列，只保留最新控制意图。 |
| 绑定驱动 | `EMM42_MotorLoadDefaultConfig(&motor, &huart4)`，随后把地址覆盖为运行时配置默认值 `0x01`。 |
| 启动修复 | 恢复闭环 FOC 控制模式，并按配置锁定电机面板按键。 |
| 使能电机 | 发送 Emm42 使能帧。 |
| 启动停机 | 发送立即停止帧，保证上电后处于已知静止态。 |
| 默认模式 | 当前 `CONVEYOR_MOTOR_STARTUP_SCAN_ENABLE` 为 `0`，上电不自动巡航，等待 MP157 `START_CYCLE` 或调试命令 `BELTSCAN`。 |
| 运行时参数 | MP157 下发 `STEPPER_PARAM_SET` 后，任务先进入 STOP，再更新当前地址、常规速度和方向映射。 |

## 6. Hardware Setup Notes

| 检查项 | 要求 |
|---|---|
| 传送带接线 | F4 `PC10(TX)` 接传送带 Emm42 `RX`，F4 `PC11(RX)` 接传送带 Emm42 `TX`。 |
| 摄像头接线 | F4 `PC6(TX)` 接两个摄像头 Emm42 `RX`，F4 `PC7(RX)` 接两个摄像头 Emm42 `TX`。 |
| 共地 | F4、Emm42 驱动器、电机电源必须共地。 |
| 地址 | 同一条 `USART6` 上两个摄像头电机现场默认分别设置为前后轴 `0x03`、上下轴 `0x02`；运行时下发时也必须互不相同。 |
| 禁止混接 | 传送带不能再接到 `USART6 PC6/PC7`，否则会和摄像头两个电机抢总线。 |

## 7. Verification

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认传送带串口 | USART1 串口助手 | `BELTINFO` | 输出 `[INFO][BELT] ...`，启动日志应含 `UART4=PC10/PC11, addr=1`。 | 检查 `conveyor_motor_service.c` 是否绑定 `huart4`，检查 PC10/PC11 接线和共地。 |
| 启动传送带扫描 | USART1 串口助手 | `BELTSCAN` | 只有传送带电机动作。 | 如果摄像头电机动作，检查接线是否把传送带接到了 USART6。 |
| 验证上方来料跟踪方向 | USART1 串口助手或 MP157 自动流程 | 先 `BELTSCAN` 观察扫描送入方向，再发 `BELTTRACK -80` | 传送带应沿扫描送入方向继续运动，把上方刚入画的零件送向 ROI 中心。 | 如果 `BELTTRACK -80` 把零件推回上方，确认 `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW` 已为 `0U` 且 F407 已重新编译下载；再检查 MP157 参数页是否把传送带 direction 下发为 `-1`。 |
| 验证视觉跟踪速度 | USART1 串口助手或 MP157 自动流程 | 依次发送 `BELTTRACK -120`、`BELTTRACK -60`、`BELTTRACK -25`、`BELTTRACK 0` | `BELTINFO` 中跟踪速度最高不超过 `80 rpm`；误差进入 `24 px` 死区后传送带停止，零件应减少 ROI 前后往返。 | 如果仍过冲，先把 `CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM` 继续降到 `60U`；如果停得太早，把 `CONVEYOR_MOTOR_CENTER_DEADBAND_PX` 从 `24` 缩到 `20` 或 `18` 后重新编译下载。 |
| 验证小误差不再卡滞 | USART1 串口助手或 MP157 自动流程 | 依次发送 `BELTTRACK 23`、`BELTTRACK 25`、`BELTTRACK -25` | `BELTTRACK 23` 应直接进入死区停机；`BELTTRACK 25/-25` 应以 `20 rpm` 附近低速实际动作，不能只显示发命令但传送带不动。 | 如果 `25px` 仍不动，把 `CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM` 改到 `25U`；如果 `25px` 动作后又过冲，先把死区缩到 `20`，不要再降低最小速度到无法启动。 |
| 停止传送带 | USART1 串口助手 | `BELTSTOP` | 传送带停止。 | 检查 UART4 TX/RX 是否交叉、Emm42 地址是否为 `0x01`。 |
| 查询摄像头电机 | USART1 串口助手 | `CAMINFO` | 输出 `USART6=PC6/PC7, forward_addr=3, z_addr=2`，并显示两轴 `min_step/speed/dir`。 | 如果未知命令，检查 `weight_service.c` 是否接入 `CameraMotorService_HandleCommand()`；如果仍是 `forward_addr=2,z_addr=3`，说明 F4 还没有重新编译下载新固件或 MP157 参数没有重新下发。 |
| 下发三电机参数 | MP157 Qt 参数页 | `参数设置 -> 步进参数 -> 保存并下发` | F4 返回 `ACK acked_cmd=0x42 status=0`；再发 `CAMINFO` 能看到摄像头两轴参数变化。 | 如果只保存 JSON 没 ACK，说明没有下发；如果 F4 没变化，确认 F4 固件已重新编译下载。 |
| 手动传送带持续运动 | MP157 Qt 手动三轴弹窗 | 切到传送带页，点击正转或反转，再点击停止。 | F4 返回 `ACK acked_cmd=0x52 status=0` 后传送带持续运动；停止返回 `ACK acked_cmd=0x51 status=0` 并停机。 | 若点击一次只动一下，确认 MP157 发的是 `ACTUATOR_VEL_MOVE`；若显示 `status=1/2`，确认 F4 已烧录 ACK 修复。 |
| 传送带当前位置设零 | MP157 Qt 参数页步进弹窗 | 切到传送带页，点击 `设当前位置为零点`。 | F4 返回 `ACK acked_cmd=0x53 status=0`，传送带不主动运动，只发送 Emm42 `[addr 0A 6D 6B]`。 | 若返回未知命令，F4 仍是旧固件；若 ACK 但设零无效，检查传送带地址、Emm42 命令支持和 `[addr 0A 6D 6B]` 帧。 |
| 测试摄像头前进轴 | USART1 串口助手 | `CAMFWD FORWARD 30` | 只有摄像头前进/后退轴动作。 | 如果上下轴动作，检查两个摄像头电机 ID 是否接反。 |
| 测试摄像头上下轴 | USART1 串口助手 | `CAMZ UP 30` | 只有摄像头上下轴动作。 | 如果两个电机都动，检查两个电机是否仍是相同地址。 |
| 停止摄像头两个轴 | USART1 串口助手 | `CAMSTOP` | 两个摄像头运动轴停止。 | 检查 USART6 接线、地址和供电。 |

## 8. Modification Record

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 将传送带 Emm42 控制串口统一为 `UART4 PC10/PC11`，地址固定 `0x01`。 |
| 2026-07-02 | 早期规划摄像头前进/后退电机使用 `USART6 PC6/PC7 addr=0x02`，后续已按现场实物调整。 |
| 2026-07-02 | 早期规划摄像头上下电机使用 `USART6 PC6/PC7 addr=0x03`，后续已按现场实物调整。 |
| 2026-07-02 | 移除旧版“传送带继续复用 USART6/huart6”的说明，避免现场调试误接线。 |
| 2026-07-03 | 增加 `STEPPER_PARAM_SET 0x42` 运行时参数说明；三台 Emm42 常规速度统一支持 `0~5000 rpm`。 |
| 2026-07-04 | 修正上方来料视觉跟踪方向：负误差现在沿扫描送入方向运动，避免零件刚出现在画面上方就被传送带反推回去。 |
| 2026-07-04 | 降低视觉跟踪速度：最大跟踪速度 `80 rpm`、加速度 `2`、低速爬行区 `50 px`、中心死区 `18 px`，用于减少 ROI 中央附近的过冲往返。 |
| 2026-07-04 | 修正小误差卡滞：最小跟踪速度提高到 `20 rpm`，中心死区扩大到 `24 px`，避免误差略大于死区时因静摩擦导致传送带不动。 |
| 2026-07-04 | 新增 MP157 `ACTUATOR_VEL_MOVE` 手动连续速度运动，传送带和前后轴点击一次持续运动，直到 `ACTUATOR_STOP`。 |
| 2026-07-04 | 新增 MP157 `ACTUATOR_POS_MOVE` 固定步数位置运动和 `ACTUATOR_HOME` 当前位置设零，当前位置设零使用 Emm42 `[addr 0A 6D 6B]`，不主动寻找限位。 |
| 2026-07-04 | 现场摄像头电机地址改为前进/后退轴 `0x03`、上下轴 `0x02`；摄像头服务命令队列改为短 FIFO，STOP 队首优先。 |
