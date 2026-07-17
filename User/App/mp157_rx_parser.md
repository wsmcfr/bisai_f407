# MP157-F4 主链路流式解析器

## 模块用途

| 项目 | 说明 |
|---|---|
| 源码 | `User/App/mp157_rx_parser.c`、`User/App/mp157_rx_parser.h` |
| 主机测试 | `User/App/mp157_rx_parser_host_test.c` |
| 调用方 | `User/App/weight_service.c` |
| 解决问题 | MP157 主链路 DMA-IDLE 可能返回粘包或半包，不能把一次回调数据直接当成一帧。 |

## 修改文件与原因

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/mp157_rx_parser.c` | 新增 192B 累积缓存、帧头同步、长度判断、CRC 校验和逐帧移除。 | 连续三帧 STOP 可逐帧执行，半帧会等待后续字节。 |
| `User/App/mp157_rx_parser.h` | 定义解析上下文、结果枚举和公开接口。 | 业务层不需要了解缓存移动细节。 |
| `User/App/mp157_rx_parser_host_test.c` | 覆盖三帧粘包与半帧续传。 | 可在 Windows 主机验证，不依赖 F4 板。 |
| `User/App/uart_command.c` | ISR 到任务改为 256B 环形缓存。 | 多个 DMA 回调来不及消费时不再覆盖上一块。 |
| `User/App/weight_service.c` | 唯一消费者循环提取所有完整帧。 | STOP 不会因 `frame_length != expected_length` 被整体拒绝。 |
| `User/App/weight_service.c` 静态解析上下文 | 192B 累积缓存放在静态区，不放入默认任务栈。 | 修复接收问题的同时不额外挤压约 1KB 的称重/协议任务栈。 |
| `MDK-ARM/bisai_f407_project.uvprojx`、`MDK-ARM/bisai_f407_project.uvoptx` | 把解析器源码加入 Keil 工程。 | 新实现会实际进入 F4 固件。 |

## 关键契约

| 场景 | 处理 |
|---|---|
| 一次收到三帧 | 连续返回三次 `FRAME_READY`，每次只移除一帧。 |
| 一帧分两次到达 | 第一次返回 `NEED_MORE` 并保留字节，第二次拼接后返回 `FRAME_READY`。 |
| 前导噪声 | 丢弃到最近合法 `A5 5A` 帧头并继续解析。 |
| CRC/帧尾错误 | 每次只滑动一个字节重同步，继续寻找后续安全 STOP。 |
| 缓存溢出 | 丢弃最旧字节，优先保留最新控制意图。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 主机回归 | `E:\hal\bisai_f407_project` | `gcc -std=c99 -Wall -Wextra -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\mp157_rx_parser_host_test.c User\App\mp157_rx_parser.c User\App\binary_protocol_service.c -o tmp\mp157_rx_parser_host_test.exe; .\tmp\mp157_rx_parser_host_test.exe` | 输出 `mp157 rx parser host tests passed`。 | 检查长度字段偏移、缓存移除和 CRC 覆盖范围。 |
| Keil 编译 | Keil MDK | Rebuild `MDK-ARM\bisai_f407_project.uvprojx` | `mp157_rx_parser.c` 参与编译，0 Error。 | 若 undefined symbol，确认工程重新加载且文件位于 `User/App` 组。 |
| 板端连续 STOP | MP157 自动流程 + F4 USART1 主链路 | 触发 ROI 微调超时，MP157 通过 `/dev/ttySTM2` 连续发送三次 STOP 二进制帧。 | 电机停止；MP157 侧收到 USART1 返回的 ACK/NACK 二进制帧。 | MP157 有写串口日志但 F4 无 STOP 动作或无回包时，检查 USART1 PA9/PA10 主链路、波特率、共地和 F4 是否烧录新固件；USART1 当前是正式主链路，文本日志默认静默。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-10 | 创建流式解析器，修复 MP157 连续 STOP 粘包、半包和 ISR 单块覆盖问题。 |
| 2026-07-16 | MP157-F4 主链路迁移到 USART2(PA2/PA3)，STOP 验证改为从 USART1(PA9) 调试口观察日志、从 USART2 二进制链路接收回包。 |
| 2026-07-17 | MP157-F4 主链路改回 USART1(PA9/PA10)，流式解析器文档改为描述“MP157 主链路”，不再绑定 USART2 名称。 |
