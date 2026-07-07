# USART1 命令入口与二进制回包说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/uart_command.c`、`User/App/uart_command.h` |
| 模块用途 | 负责 F407 USART1 的 DMA+IDLE 接收、任务级取帧、线程安全发送和二进制原始帧发送。 |
| 当前主链路 | STM32MP157 通过 USART1 与 F407 通信，正式联调时只使用二进制协议帧。 |

## 本次创建或修改文件

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/uart_command.c` | 增加 USART1 文本静默开关，并保留 `UartCommand_SendRaw()` 原始字节发送。 | MP157 不再收到 `[OK]`、`[ERROR]`、`[INFO]` 文本；二进制 `ACK/NACK/STATUS_REPORT/FAULT_REPORT` 仍正常发送。 |
| `User/App/uart_command.h` | 对外提供 `UartCommand_FetchRaw()` 和 `UartCommand_SendRaw()`。 | 上层协议可以收发包含 `0x00` 的二进制短帧。 |
| `User/App/weight_service.c` | 作为 USART1 唯一任务级消费者，先处理 `A5 5A ... 6B` 二进制帧。 | CRC 错误、未知命令、硬件故障都走二进制回包，不再落入文本成功/失败判断。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| USART1 | MP157 与 F407 主控制链路 | 115200 8N1，PA9(TX)、PA10(RX)，两端共地。 |
| DMA + IDLE | 接收一整帧命令 | IDLE 中断触发后复制当前帧到单帧缓存。 |
| `g_uart_tx_mutex` | 发送互斥 | 保护 `my_printf()` 和 `UartCommand_SendRaw()` 不交叉发送。 |

## 正式链路回包规则

| 类型 | 返回帧 | 说明 |
|---|---|---|
| 正确执行普通控制命令 | `ACK 0x80` | 负载 7 字节，包含 `cycle_id`、被确认 `seq/cmd`、`status` 和 F4 状态。 |
| 正确执行状态查询 | `STATUS_REPORT 0x82` | 负载 24 字节，包含 F4 状态、传送带模式、速度、误差和故障位。 |
| 命令错误或状态不允许 | `NACK 0x81` | 负载 9 字节，包含错误码、F4 状态和 detail。 |
| 模块主动发现故障 | `FAULT_REPORT 0x87` | 负载 16 字节，包含故障来源、严重等级、底层状态和故障位图。 |

## 文本输出边界

| 场景 | 处理 |
|---|---|
| `my_printf(&huart1, ...)` | `UART_COMMAND_USART1_TEXT_ENABLE` 为 `0U` 时直接丢弃文本，返回 0；当前已恢复为 `0U`，继续保证 MP157 主链路只走二进制。 |
| `UartCommand_SendRaw(&huart1, ...)` | 不受文本静默影响，原样发送二进制帧。 |
| Windows 串口助手临时维护 | 可以在专用调试固件中把 `UART_COMMAND_USART1_TEXT_ENABLE` 改为 `1U`，但正式接 MP157 前必须改回 `0U`；本轮机械臂链路调试已迁移到独立 `USART2`。 |
| MP157 成功/失败判断 | 只能解析二进制帧，不能搜索 `OK/ERROR/READY/BELTINFO` 文本。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认文本静默开关 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\uart_command.c -Pattern "UART_COMMAND_USART1_TEXT_ENABLE"` | 正式联调阶段应看到 `#define UART_COMMAND_USART1_TEXT_ENABLE    (0U)`。 | 如果误改成 `1U`，MP157 可能被文本日志干扰。 |
| 确认二进制发送路径存在 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\uart_command.c -Pattern "UartCommand_SendRaw"` | 能定位到原始字节发送函数和 `HAL_UART_Transmit`。 | 如果函数不存在，说明工程不是最新版本。 |
| 确认 MP157 不再走文本心跳 | Windows PowerShell | `Select-String -Path C:\Users\caofengrui\Desktop\linux\20_uvc_camera\qt_camera_display\main.cpp -Pattern "BINARY_PROTOCOL_CMD_HEARTBEAT"` | Qt 侧存在二进制心跳命令。 | 如果仍搜索到 `STATUS\r\n` 发送路径，必须先清掉文本心跳。 |
| 确认 F4 正确/错误回包 | MP157 串口工具 | 发送合法 `HEARTBEAT` 二进制帧，再发送 CRC 错帧。 | 合法帧回 `ACK 0x80`；CRC 错帧回 `NACK 0x81` 或被记录为解析错误，不执行硬件动作。 | 检查帧头、帧尾、CRC 覆盖范围、USART1 TX/RX 交叉和共地。 |
| 确认 USART1 不再混入文本 | MP157 自动流程联调 | 保持 MP157 正常连接 `USART1` 后运行自动流程。 | MP157 只会收到二进制 `ACK/NACK/STATUS_REPORT/FAULT_REPORT`，不会再夹杂 `[ARM-LOOP]` 或 `[ARM]` 文本。 | 若 MP157 解析异常，先确认 `UART_COMMAND_USART1_TEXT_ENABLE` 是否仍为 `0U`。 |

## 读写验证

| 数据方向 | 写操作 | 读操作 |
|---|---|---|
| MP157 -> F4 | MP157 向 `/dev/ttySTM2` 写入 `A5 5A ... 6B`。 | F4 `WeightService_ProcessCommand()` 通过 `UartCommand_FetchRaw()` 取出原始字节。 |
| F4 -> MP157 | F4 `BinaryProtocolService_SendFrame()` 调用 `UartCommand_SendRaw()`。 | MP157 `readF4BinaryReply()` 扫描合法二进制帧，解析 `ACK/NACK/STATUS_REPORT/FAULT_REPORT`。 |
| 串口助手维护文本 | 串口助手发送 `GET/TARE/CAL/LDCCAL`。 | 正式固件 USART1 文本静默；维护时需要专用调试固件打开文本开关。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 明确 USART1 正式链路只允许二进制回包；文本日志默认静默，正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT`。 |
| 2026-07-07 | `USART3` 本机回环调试结束后，把 `UART_COMMAND_USART1_TEXT_ENABLE` 恢复为 `0U`；机械臂链路调试日志改由独立 `USART2` 输出，不再占用 MP157 主链路。 |
