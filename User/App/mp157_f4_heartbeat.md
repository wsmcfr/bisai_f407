# STM32MP157 到 STM32F407 心跳通讯说明

## 模块目的

本模块说明 STM32MP157 通过串口探测 STM32F407 是否在线的最小协议。当前正式实现已经改回 MP157 通过 `/dev/ttySTM2` 发送二进制 `HEARTBEAT 0x02`，F407 通过 `USART1(PA9/PA10)` 返回 `ACK 0x80`；旧 `STATUS` 文本只保留为断开 MP157 后的现场维护命令，不再作为 Qt 健康检测依据。

## 本次修改文件清单

| 文件 | 修改原因 | 影响范围 |
|---|---|---|
| `User/App/binary_protocol_service.c` | 处理 `HEARTBEAT 0x02`，成功返回二进制 `ACK 0x80`。 | MP157 健康检测只看二进制 ACK，不再解析 `[OK][F4] READY` 文本。 |
| `User/App/uart_command.c` | MP157 主接收口改为 `USART1(PA9/PA10)`，`UartCommand_GetMp157Huart()` 负责向协议层提供二进制回包串口。 | `/dev/ttySTM2` 接 F4 USART1 后即可完成心跳、状态查询和自动流程命令。 |
| `User/App/weight_service.c` | 仍保留 `STATUS` 文本维护命令，但 USART1 是正式主链路时文本输出默认静默。 | 旧文本心跳不能再作为 MP157 在线判断。 |
| `User/App/mp157_f4_heartbeat.md` | 记录当前二进制心跳、接线、编译部署和验证方式。 | 模块说明文档。 |

## 通讯协议

| 项目 | 内容 |
|---|---|
| 通讯方向 | STM32MP157 -> STM32F407 查询；STM32F407 -> STM32MP157 回复。 |
| F407 主链路串口 | USART1，PA9 为 TX，PA10 为 RX。 |
| F407 调试串口 | 当前未单独迁移；USART1 作为正式主链路时文本输出默认静默。 |
| MP157 设备节点 | 当前 Qt 健康检测默认使用 `/dev/ttySTM2`。 |
| 串口参数 | 115200，8 数据位，无校验，1 停止位，关闭硬件流控。 |
| 查询帧 | 二进制 `HEARTBEAT 0x02`，帧格式为 `A5 5A VER CMD LEN SEQ_L SEQ_H CRC_L CRC_H 6B`。 |
| 示例查询帧 | `A5 5A 01 02 00 01 00 04 65 6B`，表示 `VER=1`、`CMD=0x02`、`LEN=0`、`SEQ=1`、CRC16-CCITT-FALSE。 |
| F407 回复 | 二进制 `ACK 0x80`，负载包含被确认的 `seq/cmd/cycle_id/status/f4_state`。 |
| MP157 判定 | 收到匹配 `HEARTBEAT` 的 `ACK` 且 `status=0` 才认为 F4 接入。 |
| 硬件动作 | `HEARTBEAT` 只返回在线状态，不执行去皮、标定、电机、机械臂或 LDC 操作。 |
| 超时判断 | MP157 当前等待二进制 ACK；无回复、CRC 错误、NACK 或命令不匹配时显示 F4 待接入。 |

## 硬件资源

| 硬件资源 | 用途 | 接线要求 |
|---|---|---|
| F407 USART1_TX / PA9 | F407 向 MP157 回发二进制 `ACK/NACK/STATUS_REPORT/FAULT_REPORT`。 | 连接到 MP157 `/dev/ttySTM2` 对应 RX。 |
| F407 USART1_RX / PA10 | F407 接收 MP157 发来的二进制 `HEARTBEAT/QUERY_STATUS/自动流程命令`。 | 连接到 MP157 `/dev/ttySTM2` 对应 TX。 |
| GND | 两块板串口电平参考地。 | STM32MP157 与 STM32F407 必须共地。 |

## 编译与部署

| 步骤 | 执行位置 | 命令或操作 | 预期结果 |
|---|---|---|---|
| 打开工程 | Windows | 用 Keil/MDK 打开 `E:\hal\bisai_f407_project\MDK-ARM` 下的工程文件。 | 工程能正常加载。 |
| 编译固件 | Keil/MDK | 执行 Build。 | 无新增编译错误。 |
| 烧录 F407 | Keil/MDK 或 ST-Link 工具 | 将新固件下载到 STM32F407。 | F407 重启后 USART1 命令任务运行。 |
| 连接主链路 | 硬件现场 | MP157 `/dev/ttySTM2` TX 接 F407 PA10，MP157 `/dev/ttySTM2` RX 接 F407 PA9，两板共地。 | 主链路串口物理连通。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认 MP157 串口节点存在 | STM32MP157 开发板 | `test -e /dev/ttySTM2 && echo ttySTM2-ok` | 输出 `ttySTM2-ok`。 | 若无输出，检查设备树串口节点、内核日志和串口是否被其它程序占用。 |
| 配置串口参数 | STM32MP157 开发板 | `stty -F /dev/ttySTM2 115200 raw -echo -crtscts` | 命令无报错。 | 若报错，检查 `/dev/ttySTM2` 权限、节点名称和驱动加载状态。 |
| 手动发送二进制心跳 | STM32MP157 开发板或支持 HEX 的串口工具 | 向 `/dev/ttySTM2` 发送 HEX：`A5 5A 01 02 00 01 00 04 65 6B`。 | F4 从 USART1 返回二进制 `ACK 0x80`，串口工具需用 HEX 显示。 | 若无回复，检查 USART1 PA9/PA10 是否交叉、两板是否共地、F407 是否烧录新固件、115200 8N1 和 CRC 是否正确。 |
| 串口助手验证维护文本 | Windows 串口助手，断开 MP157 正式链路后使用 | 向 F4 USART1 输入文本 `STATUS\r\n`。 | 正式联调固件中 USART1 文本默认静默；维护阶段若临时打开 `UART_COMMAND_MP157_TEXT_ENABLE` 才能看到文本。 | 若正式联调需要判断在线状态，不要依赖文本，直接验证二进制 ACK。 |
| Qt 健康状态验证 | STM32MP157 Qt 界面 | 启动 Qt 摄像头界面，等待健康检测刷新。 | F4 状态从“待接入”变为“接入”。 | 若仍待接入，先用上一条手动命令验证，再检查 Qt 默认串口是否仍为 `/dev/ttySTM2`。 |

## 修改记录

| 日期 | 修改点 |
|---|---|
| 2026-05-21 | 新增 `STATUS` 文本命令，F407 回复 `[OK][F4] READY`，用于响应 STM32MP157 健康心跳探测。 |
| 2026-07-16 | MP157 健康检测改为 `/dev/ttySTM2` 发送二进制 `HEARTBEAT 0x02` 到 F4 USART2(PA2/PA3)，F4 从 USART2 返回二进制 ACK；旧 `STATUS` 文本仅保留为维护入口，响应从 USART1(PA9) 调试口输出。 |
| 2026-07-17 | MP157-F4 心跳主链路按现场要求改回 F4 USART1(PA9/PA10)，F4 仍只用二进制 ACK 作为健康检测依据。 |
