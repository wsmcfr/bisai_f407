# Emm42 Conveyor Code Guide

## 1. Scope

这份文档用于解释当前 F407 工程中张大头 Emm42 步进电机的传送带控制链路。

当前硬件绑定已经调整为：

| 功能 | Emm42 地址 ID | F4 串口 | F4 引脚 | 负责模块 |
|---|---:|---|---|---|
| 传送带电机 | `0x01` | `UART4` | `PC10(TX) / PC11(RX)` | `User/App/conveyor_motor_service.c` |
| 摄像头前进/后退电机 | `0x02` | `USART6` | `PC6(TX) / PC7(RX)` | `User/App/camera_motor_service.c` |
| 摄像头上下电机 | `0x03` | `USART6` | `PC6(TX) / PC7(RX)` | `User/App/camera_motor_service.c` |

结论：

| 问题 | 答案 |
|---|---|
| 传送带是否还使用 `USART6` | 不使用，传送带固定使用 `UART4 PC10/PC11`。 |
| 摄像头前进/后退步进电机 ID | `0x02` |
| 摄像头上下移动步进电机 ID | `0x03` |

## 2. Current File Map

| 文件 | 角色 | 当前关键点 |
|---|---|---|
| `Core/Src/usart.c` | 串口底层初始化 | `huart4` 对应 `UART4 PC10/PC11`，`huart6` 对应 `USART6 PC6/PC7`。 |
| `Core/Src/freertos.c` | FreeRTOS 任务启动胶水 | 创建传送带任务 `ConveyorMotorService_Task()` 和摄像头电机任务 `CameraMotorService_Task()`。 |
| `User/App/uart_command.c` | `USART1` 命令接收与线程安全打印 | 只负责收完整命令和安全打印，不直接控制电机。 |
| `User/App/weight_service.c` | `USART1` 统一命令分发入口 | 分发 `BELT...` 到传送带服务，分发 `CAM...` 到摄像头电机服务。 |
| `User/App/conveyor_motor_service.c` | 传送带状态机 | 绑定 `huart4`，电机地址固定 `0x01`。 |
| `User/App/conveyor_motor_service.h` | 传送带服务公共接口 | 暴露任务入口和 `RequestScan/RequestStop/RequestTrack`。 |
| `User/App/camera_motor_service.c` | 摄像头运动电机服务 | 绑定 `huart6`，前进/后退轴地址 `0x02`，上下轴地址 `0x03`。 |
| `User/App/camera_motor_service.h` | 摄像头电机公共接口 | 暴露任务入口和 `CAM...` 命令处理入口。 |
| `User/Driver/emm42_motor.c` | Emm42 TTL 协议帧发送 | 通用驱动层，按句柄中的 `huart` 和 `address` 发送命令。 |
| `User/Driver/emm42_motor.h` | Emm42 协议类型声明 | 定义电机句柄、方向、控制模式和发送接口。 |
| `User/App/emm42_motor_uart_binding.md` | 串口与 ID 绑定总文档 | 现场调试时优先查这份文档。 |

## 3. Data Flow

当前传送带从 MP157 或串口助手到电机的链路如下：

| 步骤 | 文件 | 关键函数 | 作用 |
|---:|---|---|---|
| 1 | `User/App/uart_command.c` | `HAL_UARTEx_RxEventCallback()` | `USART1` 通过 DMA + IDLE 收到一帧命令。 |
| 2 | `User/App/uart_command.c` | `UartCommand_Fetch()` / `UartCommand_FetchRaw()` | 把完整命令交给任务上下文。 |
| 3 | `User/App/weight_service.c` | `WeightService_ProcessCommand()` | 统一分发二进制协议、机械臂帧和 ASCII 文本命令。 |
| 4 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_HandleCommand()` | 解析 `BELTSCAN/BELTSTOP/BELTTRACK/BELTCAM/BELTENABLE/BELTINFO`。 |
| 5 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_RequestScan/Stop/Track()` | 把控制意图投递到传送带任务队列。 |
| 6 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_Task()` | 独占 `UART4` 推进 `SCAN/TRACK/STOP` 状态机。 |
| 7 | `User/Driver/emm42_motor.c` | `EMM42_MotorSetVelocity()` / `EMM42_MotorStopNow()` | 按地址 `0x01` 组帧并通过 `UART4` 发送给传送带电机。 |

摄像头两个运动轴的调试链路如下：

| 步骤 | 文件 | 关键函数 | 作用 |
|---:|---|---|---|
| 1 | `User/App/weight_service.c` | `WeightService_ProcessCommand()` | 收到 `CAM...` 后转交摄像头电机服务。 |
| 2 | `User/App/camera_motor_service.c` | `CameraMotorService_HandleCommand()` | 解析 `CAMINFO/CAMSTOP/CAMFWD/CAMZ`。 |
| 3 | `User/App/camera_motor_service.c` | `CameraMotorService_Task()` | 独占 `USART6`，串行控制两个不同地址的 Emm42 电机。 |
| 4 | `User/Driver/emm42_motor.c` | `EMM42_MotorSetVelocity()` / `EMM42_MotorStopNow()` | 地址 `0x02` 控制前进/后退轴，地址 `0x03` 控制上下轴。 |

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
| `CAMSTOP` | 同时停止前进/后退轴和上下轴。 | `USART6 PC6/PC7 addr=0x02/0x03` |
| `CAMFWD FORWARD [rpm]` | 摄像头前进/后退轴正向点动。 | `USART6 PC6/PC7 addr=0x02` |
| `CAMFWD BACKWARD [rpm]` | 摄像头前进/后退轴反向点动。 | `USART6 PC6/PC7 addr=0x02` |
| `CAMZ UP [rpm]` | 摄像头上下轴向上点动。 | `USART6 PC6/PC7 addr=0x03` |
| `CAMZ DOWN [rpm]` | 摄像头上下轴向下点动。 | `USART6 PC6/PC7 addr=0x03` |

## 5. Startup Behavior

| 阶段 | 传送带任务行为 |
|---|---|
| 创建队列 | 创建长度为 1 的覆盖队列，只保留最新控制意图。 |
| 绑定驱动 | `EMM42_MotorLoadDefaultConfig(&motor, &huart4)`，随后把地址覆盖为 `0x01`。 |
| 启动修复 | 恢复闭环 FOC 控制模式，并按配置锁定电机面板按键。 |
| 使能电机 | 发送 Emm42 使能帧。 |
| 启动停机 | 发送立即停止帧，保证上电后处于已知静止态。 |
| 默认模式 | 当前 `CONVEYOR_MOTOR_STARTUP_SCAN_ENABLE` 为 `0`，上电不自动巡航，等待 MP157 `START_CYCLE` 或调试命令 `BELTSCAN`。 |

## 6. Hardware Setup Notes

| 检查项 | 要求 |
|---|---|
| 传送带接线 | F4 `PC10(TX)` 接传送带 Emm42 `RX`，F4 `PC11(RX)` 接传送带 Emm42 `TX`。 |
| 摄像头接线 | F4 `PC6(TX)` 接两个摄像头 Emm42 `RX`，F4 `PC7(RX)` 接两个摄像头 Emm42 `TX`。 |
| 共地 | F4、Emm42 驱动器、电机电源必须共地。 |
| 地址 | 同一条 `USART6` 上两个摄像头电机必须分别设置为 `0x02` 和 `0x03`。 |
| 禁止混接 | 传送带不能再接到 `USART6 PC6/PC7`，否则会和摄像头两个电机抢总线。 |

## 7. Verification

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认传送带串口 | USART1 串口助手 | `BELTINFO` | 输出 `[INFO][BELT] ...`，启动日志应含 `UART4=PC10/PC11, addr=1`。 | 检查 `conveyor_motor_service.c` 是否绑定 `huart4`，检查 PC10/PC11 接线和共地。 |
| 启动传送带扫描 | USART1 串口助手 | `BELTSCAN` | 只有传送带电机动作。 | 如果摄像头电机动作，检查接线是否把传送带接到了 USART6。 |
| 停止传送带 | USART1 串口助手 | `BELTSTOP` | 传送带停止。 | 检查 UART4 TX/RX 是否交叉、Emm42 地址是否为 `0x01`。 |
| 查询摄像头电机 | USART1 串口助手 | `CAMINFO` | 输出 `USART6=PC6/PC7, forward_addr=2, z_addr=3`。 | 如果未知命令，检查 `weight_service.c` 是否接入 `CameraMotorService_HandleCommand()`。 |
| 测试摄像头前进轴 | USART1 串口助手 | `CAMFWD FORWARD 30` | 只有摄像头前进/后退轴动作。 | 如果上下轴动作，检查两个摄像头电机 ID 是否接反。 |
| 测试摄像头上下轴 | USART1 串口助手 | `CAMZ UP 30` | 只有摄像头上下轴动作。 | 如果两个电机都动，检查两个电机是否仍是相同地址。 |
| 停止摄像头两个轴 | USART1 串口助手 | `CAMSTOP` | 两个摄像头运动轴停止。 | 检查 USART6 接线、地址和供电。 |

## 8. Modification Record

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 将传送带 Emm42 控制串口统一为 `UART4 PC10/PC11`，地址固定 `0x01`。 |
| 2026-07-02 | 明确摄像头前进/后退电机使用 `USART6 PC6/PC7 addr=0x02`。 |
| 2026-07-02 | 明确摄像头上下电机使用 `USART6 PC6/PC7 addr=0x03`。 |
| 2026-07-02 | 移除旧版“传送带继续复用 USART6/huart6”的说明，避免现场调试误接线。 |
