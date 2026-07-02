# Emm42 电机串口绑定与摄像头电机 ID 文档

## 1. 本次修改目的

| 项目 | 内容 |
|---|---|
| 目标 | 把 3 个张大头 Emm42 步进电机绑定到正确串口，并明确同一条串口上的摄像头电机 ID。 |
| 适用工程 | `E:\hal\bisai_f407_project` |
| 芯片平台 | STM32F407，HAL + FreeRTOS |
| 修改日期 | 2026-07-02 |

## 2. 最终硬件资源分配

| 功能 | Emm42 地址 ID | F4 串口 | F4 引脚 | 说明 |
|---|---:|---|---|---|
| 传送带电机 | `0x01` | `UART4` | `PC10(TX) / PC11(RX)` | 负责输送零件和根据视觉坐标做主要对中。 |
| 摄像头前进/后退电机 | `0x02` | `USART6` | `PC6(TX) / PC7(RX)` | 负责摄像头前后方向微调。 |
| 摄像头上下电机 | `0x03` | `USART6` | `PC6(TX) / PC7(RX)` | 负责摄像头上下方向调节或焦距/高度标定。 |

结论：

| 问题 | 答案 |
|---|---|
| 摄像头前进/后退步进电机 ID | `0x02` |
| 摄像头上下移动步进电机 ID | `0x03` |

## 3. 本次修改文件清单

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/Driver/emm42_motor.c` | 把驱动注释从“传送带独占 USART6”改为“通用 UART + 地址句柄”。 | 驱动层不再写死业务电机和串口，方便 UART4 与 USART6 同时复用同一个驱动。 |
| `User/Driver/emm42_motor.h` | 明确默认地址只是驱动默认值，实际地址由应用层覆盖。 | 防止后续把两个摄像头电机都留在默认 `0x01`。 |
| `User/App/conveyor_motor_service.c` | 将传送带电机绑定到 `huart4`，地址固定 `0x01`，启动日志改为 `UART4=PC10/PC11`。 | 传送带不再占用 `USART6`，避免和摄像头两个电机冲突。 |
| `User/App/conveyor_motor_service.h` | 更新传送带任务说明为 `UART4 PC10/PC11`。 | 头文件说明和真实硬件一致。 |
| `User/App/camera_motor_service.c` | 新增摄像头运动电机服务，绑定 `huart6`，内部维护地址 `0x02` 和 `0x03` 两个 Emm42 句柄。 | 可以通过 `CAM...` 文本命令调试摄像头前后轴和上下轴。 |
| `User/App/camera_motor_service.h` | 新增摄像头运动电机服务对外接口。 | 供 FreeRTOS 创建任务、USART1 命令入口和后续二进制协议层调用。 |
| `User/App/weight_service.c` | 在 USART1 统一命令分发入口中加入 `CameraMotorService_HandleCommand()`。 | `CAMINFO/CAMSTOP/CAMFWD/CAMZ` 能从 USART1 串口助手或 MP157 下发。 |
| `User/App/uart_command.c` | 更新 USART1 命令总表，补充摄像头电机命令。 | 打开串口底座文件即可查到 `CAM...` 命令用途。 |
| `Core/Src/freertos.c` | 创建 `cameraMotorTask`，启动摄像头电机服务任务。 | 新服务真正进入 FreeRTOS 调度。 |
| `MDK-ARM/bisai_f407_project.uvprojx` | 把 `camera_motor_service.c` 加入 Keil 工程。 | Keil 编译时会编译新增服务文件。 |
| `User/App/emm42_motor_uart_binding.md` | 新增本说明文档。 | 后续调试时可以直接查串口、ID、命令和验证步骤。 |
| `User/App/emm42_conveyor_code_guide.md` | 同步旧传送带说明文档。 | 删除旧版“传送带复用 USART6/huart6”的说法，避免现场误接线。 |
| `User/App/binary_protocol_service.md` | 同步二进制协议服务的硬件资源表和排查提示。 | 自动检测协议文档中的传送带资源改为 `UART4 PC10/PC11 addr=0x01`。 |

## 4. 串口命令

所有命令都从 `USART1 PA9/PA10 115200 8N1` 进入 F4，最终由 `weight_service.c` 统一分发。

### 4.1 传送带命令

| 命令 | 作用 | 实际电机 |
|---|---|---|
| `BELTSCAN` | 传送带进入低速扫描。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTSTOP` | 传送带停止。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTTRACK <error>` | 按视觉误差控制传送带方向和速度。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTCAM <enable> <current_x> <center_x>` | F4 计算 `current_x-center_x` 后控制传送带。 | `UART4 PC10/PC11 addr=0x01` |
| `BELTINFO` | 查询传送带当前状态。 | 不动作，只打印状态。 |

### 4.2 摄像头运动电机命令

| 命令 | 作用 | 实际电机 |
|---|---|---|
| `CAMINFO` | 查询摄像头两个电机的串口、地址和最近动作。 | 不动作，只打印状态。 |
| `CAMSTOP` | 同时停止摄像头前后轴和上下轴。 | `USART6 PC6/PC7 addr=0x02/0x03` |
| `CAMFWD FORWARD [rpm]` | 摄像头前进/后退轴按工程约定前进方向点动。 | `USART6 PC6/PC7 addr=0x02` |
| `CAMFWD BACKWARD [rpm]` | 摄像头前进/后退轴按工程约定后退方向点动。 | `USART6 PC6/PC7 addr=0x02` |
| `CAMZ UP [rpm]` | 摄像头上下轴按工程约定上升方向点动。 | `USART6 PC6/PC7 addr=0x03` |
| `CAMZ DOWN [rpm]` | 摄像头上下轴按工程约定下降方向点动。 | `USART6 PC6/PC7 addr=0x03` |

说明：

| 项目 | 规则 |
|---|---|
| 默认点动转速 | 省略 `[rpm]` 时使用 `30 rpm`。 |
| 最大点动转速 | 用户输入超过上限时限制到 `120 rpm`。 |
| 停止方式 | 发送 `CAMSTOP`，F4 会分别给地址 `0x02` 和 `0x03` 发送立即停止命令。 |
| 方向说明 | `FORWARD/BACKWARD/UP/DOWN` 是工程约定方向；如果实物方向反了，先在现场记录，再调整方向映射。 |

## 5. Emm42 地址设置建议

两个摄像头电机共用 `USART6`，地址必须先设置好。

| 步骤 | 操作 | 目的 |
|---:|---|---|
| 1 | 只给摄像头前进/后退电机上电，断开摄像头上下电机。 | 避免两个默认地址电机同时响应地址设置命令。 |
| 2 | 把摄像头前进/后退电机地址设置为 `0x02`。 | 后续 `CAMFWD ...` 只会控制这个轴。 |
| 3 | 断开前进/后退电机，只给摄像头上下电机上电。 | 单独设置第二个电机地址。 |
| 4 | 把摄像头上下电机地址设置为 `0x03`。 | 后续 `CAMZ ...` 只会控制这个轴。 |
| 5 | 两个摄像头电机都接回 `USART6 PC6/PC7`，并贴上 ID 标签。 | 避免后续接线或维护时混淆。 |

## 6. 编译和下载

| 步骤 | 执行位置 | 操作 | 预期 |
|---:|---|---|---|
| 1 | Windows | 打开 `E:\hal\bisai_f407_project\MDK-ARM\bisai_f407_project.uvprojx`。 | Keil 能看到 `User/App/camera_motor_service.c`。 |
| 2 | Keil | 编译工程。 | 无 `camera_motor_service` 未定义、未加入工程或头文件找不到错误。 |
| 3 | Keil/ST-Link | 下载到 F407。 | F4 上电后 USART1 日志能看到传送带和摄像头电机服务启动信息。 |

说明：本次未在 Codex 里主动运行 Keil 编译，等待用户在 Keil 中编译并贴出结果。

## 7. 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认传送带串口 | USART1 串口助手 | `BELTINFO` | 输出 `[INFO][BELT] ...`，启动日志应显示 `UART4=PC10/PC11, addr=1`。 | 检查 `conveyor_motor_service.c` 是否仍绑定 `huart6`，检查 PC10/PC11 接线和共地。 |
| 启动传送带扫描 | USART1 串口助手 | `BELTSCAN` | 传送带电机动作，摄像头两个电机不动。 | 如果摄像头电机动，说明接线或地址混乱；如果都不动，查 UART4 接线和 Emm42 地址。 |
| 停止传送带 | USART1 串口助手 | `BELTSTOP` | 传送带停止。 | 查 Emm42 停止命令是否发到 UART4。 |
| 查询摄像头电机 | USART1 串口助手 | `CAMINFO` | 输出 `forward_addr=2`、`z_addr=3`、`USART6=PC6/PC7`。 | 如果提示未知命令，检查 `weight_service.c` 是否已接入 `CameraMotorService_HandleCommand()`。 |
| 测试摄像头前进轴 | USART1 串口助手 | `CAMFWD FORWARD 30` | 只有摄像头前进/后退轴动作。 | 如果上下轴动作，两个电机 ID 可能接反；如果两个都动，两个电机可能都是同一 ID。 |
| 测试摄像头后退轴 | USART1 串口助手 | `CAMFWD BACKWARD 30` | 只有摄像头前进/后退轴反向动作。 | 如果方向反了，记录现场方向并调整方向映射。 |
| 测试摄像头上升轴 | USART1 串口助手 | `CAMZ UP 30` | 只有摄像头上下轴动作。 | 如果前后轴动作，检查地址 `0x02/0x03` 是否设置反。 |
| 测试摄像头下降轴 | USART1 串口助手 | `CAMZ DOWN 30` | 只有摄像头上下轴反向动作。 | 检查机械限位、方向映射和 USART6 接线。 |
| 停止摄像头两个轴 | USART1 串口助手 | `CAMSTOP` | 两个摄像头运动轴停止。 | 查 USART6 是否被其它任务占用，查两个 Emm42 地址是否正确。 |

## 8. 读写验证

| 数据通路 | 写操作 | 读/确认操作 | 判定 |
|---|---|---|---|
| USART1 命令入口 | 串口助手发送 `CAMINFO`。 | 查看 USART1 返回文本。 | 有返回表示命令入口和分发链路工作。 |
| UART4 到传送带 | 串口助手发送 `BELTSCAN`。 | 观察传送带动作，再发 `BELTSTOP`。 | 只有传送带动作表示 UART4 绑定正确。 |
| USART6 到摄像头前后轴 | 串口助手发送 `CAMFWD FORWARD 30`。 | 观察摄像头前后轴动作，再发 `CAMSTOP`。 | 只有地址 `0x02` 电机动作表示 ID 正确。 |
| USART6 到摄像头上下轴 | 串口助手发送 `CAMZ UP 30`。 | 观察摄像头上下轴动作，再发 `CAMSTOP`。 | 只有地址 `0x03` 电机动作表示 ID 正确。 |

## 9. 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 将传送带 Emm42 从 `USART6 PC6/PC7` 改为 `UART4 PC10/PC11`，地址固定 `0x01`。 |
| 2026-07-02 | 新增摄像头运动电机服务，摄像头前进/后退电机 ID 为 `0x02`，摄像头上下电机 ID 为 `0x03`。 |
| 2026-07-02 | 新增 `CAMINFO/CAMSTOP/CAMFWD/CAMZ` 调试命令，用于现场确认两个摄像头电机是否能独立动作。 |
| 2026-07-02 | 同步 `emm42_conveyor_code_guide.md` 和 `binary_protocol_service.md`，保证旧文档不再把传送带写成 `USART6/huart6`。 |
