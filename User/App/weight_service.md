# weight_service 模块说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/weight_service.c`、`User/App/weight_service.h` |
| 模块用途 | 在 FreeRTOS 任务中初始化 HX711、周期采样称重原始值，并作为 MP157 主链路唯一任务级消费者分发 MP157 二进制协议和维护文本命令。 |
| 当前阶段 | USART1(PA9/PA10) 恢复为 MP157 正式通讯口；USART1 文本输出默认静默，不能作为 MP157 成功或失败依据。 |

## 本次创建或修改文件

| 文件 | 类型 | 修改原因 |
|---|---|---|
| `User/App/weight_service.c` | 修改 | 将命令入口说明从 USART2 固定描述改为 MP157 主链路描述，并明确当前主链路为 USART1。 |
| `User/App/weight_service.h` | 修改 | 将任务职责注释同步为 MP157 主链路唯一消费者。 |
| `User/App/uart_command.c` | 修改 | 将 MP157 主链路句柄改回 `&huart1`，并保持主链路文本静默。 |
| `User/App/uart_command.h` | 修改 | 新增 `UartCommand_GetMp157Huart()`，供协议层复用当前主链路串口。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| USART1 | MP157 或串口助手输入入口 | 115200 8N1，PA9(TX)、PA10(RX)，通过 `UartCommand_FetchRaw()` 取出原始字节。 |
| USART2 | 非 MP157 正式主链路 | 115200 8N1，PA2(TX)、PA3(RX)，当前不再由 `uart_command.c` 作为 MP157 命令入口消费。 |
| HX711 DOUT | 称重 ADC 数据输出 | 默认 `PB0`，低电平表示数据就绪。 |
| HX711 SCK | 称重 ADC 时钟输入 | 默认 `PB2`，F4 输出采样时钟。 |
| FreeRTOS 任务 | 称重采样和命令消费 | `WeightService_Task()` 周期 50ms 更新最近一次滤波值。 |

## 二进制称重标定

| 字段 | 类型 | 合法值 | 说明 |
|---|---|---|---|
| `cycle_id` | `u16` | 固定 `0` | 人工维护命令，不绑定自动检测流程。 |
| `known_weight_g` | `u16` | `1~5000` | 已知砝码重量，单位克。 |
| `flags` | `u8` | 固定 `0` | 首版不自动去皮、不保存 Flash。 |

| 标定结果 | F4 回包 | 说明 |
|---|---|---|
| 已去皮、克重合法、最近采样有效、净计数非 0 | `ACK 0x80` | HX711 运行时 `scale_counts_per_g` 已更新。 |
| 未去皮或称重上下文未建立 | `NACK ERR_STATE_NOT_ALLOWED` | 先空载去皮，再放砝码标定。 |
| 克重、`cycle_id` 或 `flags` 越界 | `NACK ERR_FIELD_RANGE` | MP157 应只发 `cycle_id=0, flags=0, known_weight_g=1~5000`。 |
| 最近采样失败或净计数为 0 | `NACK ERR_HARDWARE_FAULT` | 检查 HX711 供电、DOUT/SCK、砝码是否放稳。 |

## 编译、部署和验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 确认临时回显测试已删除 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\weight_service.c -Pattern "ECHO|MP157-RAW|UART2-DIAG|PrintRxDiagnostics|ProcessUsart1EchoTest"` | 没有匹配结果。 | 如果仍有匹配，继续清理临时测试代码。 |
| 主机侧协议解码测试 | `E:\hal\bisai_f407_project` | `gcc -std=c99 -Wall -Wextra -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\binary_protocol_service_host_test.c User\App\binary_protocol_service.c -o tmp\binary_protocol_service_host_test.exe; .\tmp\binary_protocol_service_host_test.exe` | 输出 `binary protocol host tests passed`；`WriteI32Le` 未使用 warning 是 host-test 裁剪硬件路径导致，可接受。 | 若 `WEIGHT_CALIBRATE` 解码失败，检查 5 字节负载偏移和小端序。 |
| Keil 编译 | F4 工程 `E:\hal\bisai_f407_project` | 在 Keil 中重新编译 `bisai_f407_project` | 无错误，`weight_service.c`、`uart_command.c` 均参与编译。 | 若找不到已删除接口，确认没有其它任务仍调用旧调试函数。 |
| MP157 UI 标定 | STM32MP157 开发板屏幕 | `参数设置 -> 称重标定 -> 输入 1000 -> 发二进制` | F4 已去皮且砝码放稳时显示 `ACK WEIGHT_CALIBRATE`；失败时显示结构化 NACK。 | 若仍是 `UNKNOWN_0x30`，说明 F4 没烧录新固件；若无回包，检查 `/dev/ttySTM2`、F4 USART1 PA9/PA10、共地和 115200。 |
| 文本维护标定 | USART1 串口助手输入，断开 MP157 正式链路后使用 | 向 USART1 先发 `TARE`，再发 `CAL 1000` | 文本维护命令会复用称重核心；正式联调时 USART1 文本输出默认静默，不能用文本判断成功。 | 若无结构化二进制回包，先查 PA10 输入线、PA9 输出线、波特率和 F4 是否烧录新固件。 |

## 读写验证

| 数据路径 | 写入怎么做 | 读取/确认怎么做 |
|---|---|---|
| MP157 -> F4 标定请求 | Qt 组 `A5 5A 01 30 05 SEQ_L SEQ_H 00 00 E8 03 00 CRC_L CRC_H 6B`，其中 `E8 03` 表示 1000g。 | F4 返回 `ACK` 或 `NACK` 二进制帧；MP157 弹窗显示 `WEIGHT_CALIBRATE`，不再显示 `UNKNOWN_0x30`。 |
| 称重服务 -> HX711 标定参数 | `WeightService_RequestCalibration()` 调用 `HX711_CalibrateByKnownWeight()` 写入 `scale_counts_per_g`。 | 当前只保存在 F4 运行内存；F4 断电或复位后需要重新标定，除非后续新增 Flash 持久化。 |
| 文本 CAL 维护路径 | 断开 MP157 正式链路后，串口助手向 USART1 发送 `CAL 1000`。 | 仍复用同一标定核心；正式联调时文本输出默认静默，维护阶段如需文本需临时打开主链路文本开关。 |
| MP157 -> F4 正式二进制帧 | MP157 向 `/dev/ttySTM2` 写入 `A5 5A ... 6B`。 | F4 `WeightService_ProcessCommand()` 将原始字节送入 `Mp157RxParser_TryExtractFrame()`，完整帧交给 `BinaryProtocolService_HandleFrame()`。 |

## 失败排查

| 现象 | 优先排查 |
|---|---|
| `NACK ERR_CMD_UNKNOWN detail=48` | F4 仍是旧固件；重新编译并烧录包含 `WEIGHT_CALIBRATE 0x30` 的工程。 |
| `NACK ERR_STATE_NOT_ALLOWED` | 称重任务尚未建立上下文或空载去皮未成功；确认 `WeightService_Task()` 正常运行并先执行去皮。 |
| `NACK ERR_FIELD_RANGE` | 检查 MP157 payload：`cycle_id=0`、`known_weight_g=1~5000`、`flags=0`。 |
| `NACK ERR_HARDWARE_FAULT` | 检查 HX711 最近采样状态、DOUT/SCK 接线、供电、砝码是否放稳以及净计数是否为 0。 |
| 标定成功但重启后失效 | 当前只更新运行内存，没有写 F4 Flash 或 HX711 外部存储；需要重新标定或后续新增持久化。 |
| MP157 一直发但 F4 没 ACK | 先确认线序：MP157 TX -> F4 PA10，MP157 RX <- F4 PA9，两板 GND 共地；再确认 F4 已烧录当前固件。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-03 | 新增二进制称重标定入口，`WEIGHT_CALIBRATE 0x30` 通过 `WeightService_RequestCalibration()` 调用 HX711 标定核心。 |
| 2026-07-03 | 文本 `CAL <克重>` 与二进制 `WEIGHT_CALIBRATE` 共用同一套去皮、量程、采样和硬件错误校验逻辑。 |
| 2026-07-16 | MP157-F4 主链路迁移到 USART2(PA2/PA3)，称重维护文本从 USART2 输入，响应和日志从 USART1(PA9) 输出。 |
| 2026-07-16 | 删除 USART1/USART2 临时回显测试和 USART2 接收诊断打印，恢复 USART2 专门用于 MP157 正式二进制通讯。 |
| 2026-07-17 | MP157-F4 主链路按现场要求改回 USART1(PA9/PA10)，称重任务继续作为主链路唯一命令消费者，成功/失败仍以二进制 ACK/NACK 为准。 |
