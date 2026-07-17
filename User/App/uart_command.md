# USART1 命令入口与二进制回包说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/uart_command.c`、`User/App/uart_command.h` |
| 模块用途 | 负责 F407 MP157 主链路的 DMA+IDLE 原始字节接收、任务级取块、线程安全文本发送和二进制原始帧发送。 |
| 当前主链路 | STM32MP157 通过 F407 USART1(PA9/PA10) 与 F407 通信，正式联调只使用二进制协议帧。 |
| 调试口状态 | USART1 已恢复为 MP157 正式主链路，`my_printf(&huart1, ...)` 在正式联调默认静默，避免文本混入二进制协议。 |

## 本次创建或修改文件

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/uart_command.c` | 将 `UART_COMMAND_MP157_HUART` 从 `&huart2` 改回 `&huart1`，并新增 `UartCommand_GetMp157Huart()`。 | USART1 PA9/PA10 重新作为 MP157-F4 正式收发链路；协议发送端不再硬编码 `&huart2`。 |
| `User/App/uart_command.h` | 新增 `UartCommand_GetMp157Huart()` 声明，并把接口注释从 USART2 固定描述改为 MP157 主链路描述。 | 协议层通过统一函数获取主链路句柄，后续换线只改一处。 |
| `User/App/binary_protocol_service.c` | `BinaryProtocolService_SendFrame()` 改为调用 `UartCommand_GetMp157Huart()` 后再 `UartCommand_SendRaw()`。 | ACK/NACK/STATUS/FAULT 回包跟随 USART1 主链路，不会出现接收在 USART1、回包仍发 USART2 的分裂。 |
| `User/App/weight_service.c` | 同步入口说明，明确命令消费者现在服务 MP157 主链路 USART1。 | 注释与实际接收串口保持一致，减少现场接线排查误判。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| USART1 | MP157 与 F407 主控制链路 | 115200 8N1，PA9(TX)、PA10(RX)，两端共地，RX 使用 DMA2 Stream2 Channel4 + IDLE。 |
| USART2 | 非 MP157 正式主链路 | 115200 8N1，PA2(TX)、PA3(RX)，当前不再由 `uart_command.c` 作为 MP157 命令入口消费。 |
| DMA + IDLE | 接收 MP157 主链路原始字节块 | IDLE 只表示线路暂时空闲，不等于协议帧边界；USART1 字节先进入 256B 环形缓存。 |
| MP157 流解析缓存 | 拆分连续帧并保存半帧 | 由 `mp157_rx_parser.c` 处理粘包、半包和坏帧重同步。 |
| `g_uart_tx_mutex` | 发送互斥 | 保护 `my_printf()` 和 `UartCommand_SendRaw()` 不交叉发送。 |

## 正式链路回包规则

| 类型 | 返回帧 | 说明 |
|---|---|---|
| 正确执行普通控制命令 | `ACK 0x80` | 负载包含 `cycle_id`、被确认 `seq/cmd`、状态和 F4 状态位。 |
| 正确执行状态查询 | `STATUS_REPORT 0x82` | 负载包含 F4 状态、传送带模式、速度、误差和故障位。 |
| 命令错误或状态不允许 | `NACK 0x81` | 负载包含错误码、F4 状态和 detail。 |
| 模块主动发现故障 | `FAULT_REPORT 0x87` | 负载包含故障来源、严重等级、底层状态和故障位图。 |

## 文本输出边界

| 场景 | 处理 |
|---|---|
| `my_printf(&huart1, ...)` | `UART_COMMAND_MP157_TEXT_ENABLE` 为 `0U` 时直接丢弃文本，正式 MP157 主链路只走二进制。 |
| `UartCommand_SendRaw(UartCommand_GetMp157Huart(), ...)` | 不受文本静默影响，原样通过 USART1 发送二进制 ACK/NACK/STATUS/FAULT 帧。 |
| `my_printf(&huart2, ...)` | 仍可按普通串口发送文本，但当前业务模块没有统一迁移到 USART2 调试口。 |
| MP157 成功/失败判断 | 只能解析二进制帧，不能搜索 `OK/ERROR/READY/BELTINFO` 文本。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认 USART1 是 MP157 主链路 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\uart_command.c -Pattern "UART_COMMAND_MP157_HUART"` | 看到 `#define UART_COMMAND_MP157_HUART           (&huart1)`。 | 如果不是 `&huart1`，MP157 会继续接错 F4 串口。 |
| 确认 USART1 文本静默 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\uart_command.c -Pattern "UART_COMMAND_MP157_TEXT_ENABLE"` | 正式联调阶段应看到 `#define UART_COMMAND_MP157_TEXT_ENABLE     (0U)`。 | 如果误改成 `1U`，MP157 可能被文本日志干扰。 |
| 确认回显/诊断测试已删除 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\uart_command.c,E:\hal\bisai_f407_project\User\App\uart_command.h -Pattern "ECHO|UART2-DIAG|MP157-RAW|PrintRxDiagnostics|ProcessUsart1EchoTest"` | 没有匹配结果。 | 如果仍有匹配，说明还有临时测试代码或文档没有清理干净。 |
| 确认 F4 正确/错误回包 | MP157 串口工具 | 发送合法 `HEARTBEAT` 二进制帧，再发送 CRC 错帧。 | 合法帧从 USART1 回 `ACK 0x80`；CRC 错帧回 `NACK 0x81` 或被解析器丢弃且不执行硬件动作。 | 检查帧头、帧尾、CRC 覆盖范围、USART1 PA9/PA10 TX/RX 交叉和共地。 |
| Keil 编译 | F4 工程 `E:\hal\bisai_f407_project` | 在 Keil 中重新编译 `bisai_f407_project` | 无 `undefined symbol` 或语法错误。 | 若提示找不到已删除接口，检查是否还有任务代码调用旧的诊断/回显函数。 |

## 读写验证

| 数据方向 | 写操作 | 读操作 |
|---|---|---|
| MP157 -> F4 | MP157 向 `/dev/ttySTM2` 写入 `A5 5A ... 6B`，线接 F4 USART1 RX/PA10。 | F4 `WeightService_ProcessCommand()` 通过 `UartCommand_FetchRaw()` 取出 MP157 主链路原始字节。 |
| F4 -> MP157 | F4 `BinaryProtocolService_SendFrame()` 调用 `UartCommand_SendRaw(UartCommand_GetMp157Huart(), ...)`。 | MP157 `readF4BinaryReply()` 扫描合法二进制帧，解析 `ACK/NACK/STATUS_REPORT/FAULT_REPORT`。 |
| F4 -> 调试串口 | F4 业务模块当前大量调用 `my_printf(&huart1, ...)`。 | USART1 是 MP157 主链路时这些文本默认被静默；如需现场日志，先把调试输出迁移到非主链路串口或临时断开 MP157 后再打开文本。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 历史阶段明确原 USART1 正式链路只允许二进制回包；文本日志默认静默，正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT`。 |
| 2026-07-10 | 原 MP157 主链路 ISR 到任务缓存从“最近一块覆盖”改为 256B 环形缓存；新增 MP157 流式解析器，修复连续三帧 STOP 被 DMA-IDLE 合并后因整块长度不符而全部拒绝的问题。 |
| 2026-07-16 | 按现场接线方案把 MP157 主链路迁移到 F4 USART2(PA2/PA3)，USART1(PA9) 作为文本调试输出口。 |
| 2026-07-16 | 删除 USART1/USART2 临时回显测试和 USART2 诊断打印，恢复 USART2 专门用于 MP157 正式二进制通讯。 |
| 2026-07-17 | 按现场要求把 MP157-F4 主链路改回 USART1(PA9/PA10)，并让二进制回包通过 `UartCommand_GetMp157Huart()` 跟随同一主链路。 |
