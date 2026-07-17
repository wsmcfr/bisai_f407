# F4 与 ESP32S3 机械臂正式协议服务

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/robot_arm_service.c`、`User/App/robot_arm_service.h`、`User/App/robot_arm_rx_parser.c`、`User/App/robot_arm_rx_parser.h` |
| 硬件链路 | STM32F407 USART3 `<->` ESP32S3 UART，固定 `115200 8N1`，3.3V TTL，共地 |
| 当前协议 | 只使用 `A5 5A VER CMD LEN SEQ_L SEQ_H PAYLOAD CRC_L CRC_H 6B` 正式二进制协议 |
| 上游入口 | `binary_protocol_service.c` 调用 `RobotArmService_RequestPlaceWeight()`、`RobotArmService_RequestPlaceLdc()`、`RobotArmService_RequestFinalSort()` |
| 下游回包 | ESP32S3 必须回 `ARM_ACK`，动作真实完成后再回 `ARM_STAGE_DONE` |
| 当前超时 | F4 发给 ESP32S3 的动作 `timeout_ms=100000ms`，按 `u32` 小端发送；F4 等 DONE 总窗口为 `100000+10000=110000ms` |
| 当前诊断增强 | `robotArmTask` 会打印最小栈水位；USART3 等待改为分片轮询并主动让出调度，不再长时间霸占 CPU |
| 当前串口定位 | `robot_arm_service.c` 会把 `[ARM]` 和 `[ARM-LOOP]` 调试日志输出到独立 `USART1(PA9/PA10)`，用于确认 PD9 是否真的收到 ESP 回传而不占用 MP157-F4 的 `USART2(PA2/PA3)` 主链路 |
| 当前接收策略 | 正式模式下新增 `robot_arm_rx_parser` 缓存与重同步层；ACK/DONE 等待从缓存取帧，空闲时每 `5ms` 也会轮询 USART3 把主动故障帧先收进缓存，不再只在发送后临时读 RX。 |
| 当前自检开关 | `robot_arm_service.c` 的 `ROBOT_ARM_SERVICE_ENABLE_FORMAL_PROTOCOL_FLOW` 当前已恢复为 `1U`，默认进入正式协议自动流程；只有单独做 `USART3` 回环时才临时改成 `0U` |

## 修改文件清单

| 文件 | 修改原因 | 影响的契约 |
|---|---|---|
| `Core/Src/freertos.c` | 扩大称重、LDC、电机、机械臂相关任务栈，降低自动流程叠加日志和协议解析时的栈边界风险。 | 任务创建仍由 CubeMX/FreeRTOS 负责，但关键业务任务可用栈余量增加。 |
| `User/App/robot_arm_service.c` | 移除旧动作组兼容发送，改为正式 `A5 5A` 组帧、ACK/NACK/DONE 解析和 DONE 后推进自动流程。 | F4 发给 ESP32S3 的 TX 不再出现旧动作组帧；ESP32S3 必须实现正式协议。 |
| `User/App/robot_arm_service.h` | 更新接口说明，明确旧协议停用，业务入口等待 ACK/DONE。 | 上游 `binary_protocol_service.c` 接口保持不变。 |
| `User/App/robot_arm_rx_parser.c` | 新增机械臂串口接收缓存与重同步实现，负责跳过噪声、坏帧、重叠 `A5 A5 5A` 帧头，并在坏帧后继续寻找下一帧。 | F4 不再因一帧坏数据就把整轮 ACK/DONE 直接判死。 |
| `User/App/robot_arm_rx_parser.h` | 新增接收缓存公开结构和解析接口声明。 | `robot_arm_service.c` 与主机侧测试共用同一套缓存/提帧契约。 |
| `User/App/robot_arm_rx_parser_host_test.c` | 新增主机侧回归测试，覆盖“坏帧后继续取下一帧”和“前导噪声/重叠帧头重同步”两类问题。 | 后续再改接收层时，可先在 Windows 主机验证缓存重同步逻辑。 |
| `User/App/robot_arm_service.md` | 新增本模块说明、验证方式和故障排查。 | 后续联调可直接按本文确认协议、接线和回包。 |
| `User/App/weight_service.c` | 更新 USART2 命令分发说明，删除旧动作组示例。 | 现场调试说明不再引导用户发送旧协议。 |
| `User/App/uart_command.c` | 更新命令入口注释，把机械臂链路说明改为正式协议 ACK/DONE。 | 串口调试说明与当前 F4-ESP32S3 协议一致。 |
| `User/App/uart_command.h` | 更新原始二进制读取接口说明。 | 明确保留 `0x00` 是为了正式二进制帧，不是旧协议。 |
| `User/App/binary_protocol_service.h` | 更新 MP157-F4 与 F4-ESP32S3 二进制协议分发说明。 | 避免继续把旧协议作为回退路径。 |

## 协议命令

| 阶段 | F4 发给 ESP32S3 | ESP32S3 立即回 | ESP32S3 动作完成后回 |
|---|---|---|---|
| 启动握手 | `ARM_LINK_HELLO 0x01` | `ARM_ACK` | 无 |
| 启动心跳 | `ARM_LINK_HEARTBEAT 0x02` | `ARM_ACK` | 无 |
| 抓到称重模块 | `ARM_MOVE_TO_WEIGHT 0x20` | `ARM_ACK acked_cmd=0x20` | `ARM_STAGE_DONE stage=1 result=0` |
| 称重到电感 | `ARM_MOVE_TO_LDC 0x21` | `ARM_ACK acked_cmd=0x21` | `ARM_STAGE_DONE stage=2 result=0` |
| 最终分拣 | `ARM_SORT_RESULT 0x22` | `ARM_ACK acked_cmd=0x22` | `ARM_STAGE_DONE stage=3 result=0` |

## 使用方法

| 步骤 | 执行位置 | 操作 |
|---:|---|---|
| 1 | Keil/MDK F4 工程 | 重新编译 `E:\hal\bisai_f407_project`。 |
| 2 | F4 开发板 | 下载新固件并复位。 |
| 3 | ESP32S3 | 烧录支持 `A5 5A` 正式协议的机械臂固件。 |
| 4 | 串口监听 | 监听 F4 USART3 TX，自动流程触发抓取时应看到 `A5 5A 01 20 ... 6B`。 |
| 5 | MP157 首页 | 执行一次自动检测，Z 轴回升后等待机械臂抓取到称重模块。 |

## USART3 回环自检模式

| 步骤 | 执行位置 | 操作 |
|---:|---|---|
| 1 | `User/App/robot_arm_service.c` | 把 `ROBOT_ARM_SERVICE_ENABLE_FORMAL_PROTOCOL_FLOW` 改成 `0U`。 |
| 2 | F4 硬件接线 | 把 `PD8(USART3_TX)` 与 `PD9(USART3_RX)` 直接短接，或者接到你现场要验证的回环路径上；必须共地。 |
| 3 | `Core/Src/usart.c` + 外部串口工具 | 把 USB-TTL RX 接到 `PA9(USART1_TX)` 和 GND，波特率 `115200 8N1`；如需双向调试再接 `PA10(USART1_RX)`。 |
| 4 | Keil/MDK F4 工程 | 重新编译 `E:\hal\bisai_f407_project`。 |
| 5 | F4 开发板 | 下载新固件并复位。 |
| 6 | USART1 调试口 | 观察日志，确认每秒都会出现一组 `TX` / `RX` 回环结果。 |
| 7 | 恢复正式流程 | 测完后把 `ROBOT_ARM_SERVICE_ENABLE_FORMAL_PROTOCOL_FLOW` 改回 `1U`，重新编译下载，再继续联调 ESP32S3 正式协议。 |

## 验证方式

| 测试目标 | 执行位置 | 命令/方法 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 源码不再发旧协议 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\robot_arm_service.c -Pattern "ROBOT_ARM_SERVICE_CMD_ACTION_GROUP_RUN","55 55"` | 无旧动作组发送标记。 | 若仍命中，说明旧兼容逻辑未删干净。 |
| 正式协议组帧存在 | Windows PowerShell | `Select-String -Path E:\hal\bisai_f407_project\User\App\robot_arm_service.c -Pattern "ROBOT_ARM_CMD_MOVE_TO_WEIGHT","BinaryProtocolService_BuildFrame","RobotArmService_WaitStageDone"` | 能看到正式命令、复用组帧函数和 DONE 等待函数。 | 若缺失，说明 F4 没切到正式协议。 |
| 应用层无旧透传提示 | Windows PowerShell | `rg -n "55 55|LeArm" E:\hal\bisai_f407_project\User\App --glob '!robot_arm_service.md'` | 无输出。 | 若命中，检查是否还有旧协议入口或误导性注释。 |
| F4 TX 字节 | 串口分析仪或串口助手 | 自动流程触发机械臂抓取，抓 USART3 TX。 | 首字节应为 `A5 5A`，CMD 应为 `0x20`，帧尾 `6B`。 | 若仍是旧动作组帧，确认 F4 是否重新编译、下载和复位。 |
| ESP32S3 ACK | ESP32S3 串口日志或 F4 USART1 日志 | 观察 `ARM_ACK`。 | F4 日志出现 `ACK OK: ARM_MOVE_TO_WEIGHT`。 | 检查 ESP32S3 是否实现 ACK、CRC 是否一致、TX/RX 是否交叉。 |
| ESP32S3 DONE | 实物机械臂 | 让机械臂真实放到称重模块后回 `ARM_STAGE_DONE stage=1 result=0`。 | F4 调用称重并向 MP157 发 `WEIGHT_RESULT`。 | 检查 ESP32S3 是否把 ACK 误当 DONE、stage_id 是否填错。 |
| 动作超时窗口 | F4 USART3 抓包 + MP157 日志 | 触发自动流程，观察 `ARM_MOVE_TO_WEIGHT/MOVE_TO_LDC/SORT_RESULT` payload 的 `timeout_ms` 字段。 | payload 偏移 6~9 应为 `A0 86 01 00`，表示 `100000ms`；F4 DONE 日志最大等待约 `110000ms`。 | 若仍是 2 字节旧字段或 `60 EA`，说明 F4 仍运行旧固件或未重新下载；若 MP157 先超时，确认 Qt 参数页 `机械臂等待` 是否大于 F4 最长等待。 |
| 机械臂任务栈余量 | F4 USART1 日志 | 上电后触发一次机械臂流程，观察 `[ARM] Stack watermark:` 日志。 | 至少能看到 `task-start`、`after-startup`，后续在 `ack-ok/done-ok` 或新低水位时继续打印；`remain_words` 不应持续逼近 0。 | 若剩余字数长期小于 `96` 或继续下降，说明还需继续扩栈或减少局部缓存。 |
| 心跳灯不再假死 | 板端现象 + F4 日志 | 让 ESP32S3 故意晚一点回 ACK/DONE，观察等待阶段。 | 即便机械臂仍在等待回包，低优先级心跳灯也应继续闪烁；F4 日志仍能在超时后打印 `ACK timeout` 或 `DONE timeout`。 | 若 LED 仍完全停住，优先继续查栈溢出、HardFault 或其它高优先级死循环。 |
| ESP->F4 回传链路 | F4 USART1 日志 | 触发一次 `ARM_MOVE_TO_WEIGHT`，观察 `[ARM] USART3 RX byte` 和 `[ARM] ESP32 frame parsed`。 | 只要 PD9 收到任何回传，至少应看到 `USART3 RX byte`；若帧完整，还应看到 `ESP32 frame parsed: cmd=0x80/0x30...`。 | 若一直没有任何 `USART3 RX byte`，优先查 `ESP TX -> F4 PD9` 接线、电平、共地和串口号；若有原始字节但没有 `frame parsed`，再查帧格式、波特率或噪声。 |
| 主机侧接收缓存回归 | Windows PowerShell | `gcc -std=c99 -Wall -Wextra -DBINARY_PROTOCOL_HOST_TEST -I E:\hal\bisai_f407_project\User\App E:\hal\bisai_f407_project\User\App\robot_arm_rx_parser_host_test.c E:\hal\bisai_f407_project\User\App\robot_arm_rx_parser.c E:\hal\bisai_f407_project\User\App\binary_protocol_service.c -o E:\hal\bisai_f407_project\tmp\robot_arm_rx_parser_host_test.exe; E:\hal\bisai_f407_project\tmp\robot_arm_rx_parser_host_test.exe` | 输出 `robot arm rx parser host tests passed`，证明“坏帧后继续找下一帧”和“前导噪声/重叠帧头重同步”两项契约成立。 | 若编译失败，先检查 `gcc` 和头文件路径；若测试失败，优先回看 `robot_arm_rx_parser.c` 的前缀丢弃、CRC 失败后滑动重同步和缓存溢出处理。 |
| 空闲期主动故障上报 | F4 USART1 日志 + ESP32S3 | 在 F4 没有新动作待发时，让 ESP32S3 主动回一帧 `FAULT_REPORT` 或 `STAGE_REPORT`。 | 不需要等到下一次 F4 发送动作，F4 空闲轮询也应看到相应 `[ARM]` 日志。 | 若只有下一次发送前后才出现日志，说明正式模式下空闲轮询没有运行，优先检查 `ROBOT_ARM_SERVICE_IDLE_QUEUE_WAIT_MS` 和 `RobotArmService_ServiceIdleRx()`。 |
| USART3 本机回环 | F4 USART1 日志 | 关闭正式流程宏，短接 `PD8->PD9`，上电观察 `[ARM-LOOP]` 日志。 | 每秒应先看到 `TX: A5 5A 33 CC 55 0D`，随后看到完全相同的 `RX:`，并打印 `RX match, USART3 loopback OK`。当前实现已改为“逐字节发送、逐字节回读”，避免 6 字节一次性发完后因 RX 读取不及时只留下第 1 个字节。 | 若只有 `TX` 没有 `RX`，优先查短接/焊点/引脚复用；若 `RX timeout` 出现在第 1 字节，重点查 `PD9` 输入链路；若只收到部分字节，查波特率、串口错误标志、线路接触不良，或是否仍在跑旧固件。 |
| 故障上报 | ESP32S3 | 人为制造忙状态或抓取失败。 | ESP32S3 回 `ARM_NACK` 或 `ARM_STAGE_DONE result!=0`，F4 上报 ARM 故障。 | 检查 error_code、result、fault_bits 是否按文档填写。 |

## 修改记录

| 日期 | 修改点 | 说明 |
|---|---|---|
| 2026-07-06 | 切换到正式机械臂二进制协议 | F4 不再发送旧动作组兼容帧，改为 `A5 5A` 正式协议，并等待 ACK/DONE。 |
| 2026-07-06 | 放大机械臂单阶段等待时间 | `ROBOT_ARM_SERVICE_ACTION_TIMEOUT_MS` 从 `15000ms` 调整为 `60000ms`，DONE 额外余量从 `2000ms` 调整为 `5000ms`，避免真实机械臂动作慢时 F4 过早超时。 |
| 2026-07-07 | 扩展动作超时字段 | `timeout_ms` 从旧 `u16` 发送字段扩展为 `u32`，当前可完整发送 `100000ms`，避免被截断成 `34464ms`。 |
| 2026-07-07 | 扩大关键任务栈并增加栈水位日志 | `robotArmTask`、称重/LDC/电机相关任务扩大栈；`robot_arm_service.c` 增加最小栈水位日志，便于区分“真栈爆”和“只是等待回包”。 |
| 2026-07-07 | USART3 等待改为分片轮询 | ACK/DONE 等待不再把整段剩余超时一次性交给 `HAL_UART_Receive()`，改为 5ms 分片并主动让出调度，降低低优先级心跳灯假死现象。 |
| 2026-07-07 | 增加 USART3 原始接收诊断 | F4 只要从 PD9 收到任何字节就低频打印原始值，完整解析成功后再打印 `ESP32 frame parsed`，用于快速定位是否是 ESP->F4 物理回传链路断开。 |
| 2026-07-07 | 增加 USART3 回环自检模式 | 新增 `ROBOT_ARM_SERVICE_ENABLE_FORMAL_PROTOCOL_FLOW` 宏；关闭后不再推进正式自动流程，而是每秒从 USART3 TX 发固定 6 字节并从 RX 读回，直接确认 PD9 是否能收到本机发送数据。 |
| 2026-07-07 | 修正回环自检发送节拍 | 原先先整帧发送再集中读取，短接 `TX/RX` 时容易因为 RX 数据寄存器未及时读取而只保留首字节 `0xA5`。现改为“发 1 字节、立刻读 1 字节”，更适合现场确认 PD9 是否逐字节收到回环数据。 |
| 2026-07-07 | 历史阶段调试日志迁移到独立 USART2 | 当时为避免自动流程期间 `USART1` 被 MP157 占用，机械臂服务调试日志曾从 `USART1` 迁移到新增 `USART2(PA2/PA3)`；该安排已被 2026-07-16 的 USART2 主链路方案取代。 |
| 2026-07-07 | 新增接收缓存与空闲期轮询 | 新增 `robot_arm_rx_parser.c/.h` 和主机侧测试，正式模式下 ACK/DONE 不再直接按阻塞读拼帧，而是先进入解析缓存；坏帧后继续找下一帧，且空闲期也轮询 USART3，避免主动 `FAULT_REPORT` 被下一次发送前的 flush 静默丢掉。 |
| 2026-07-16 | 调试日志迁移到 USART1 | 因 MP157-F4 主链路改用 `USART2(PA2/PA3)`，机械臂 `[ARM]` 和 `[ARM-LOOP]` 调试日志改到 `USART1(PA9)`，避免污染 MP157 二进制回包。 |

## 硬件资源

| 资源 | 用途 | 要求 |
|---|---|---|
| USART3 TX | F4 发送正式机械臂命令到 ESP32S3 RX | 115200 8N1，3.3V TTL |
| USART3 RX | F4 接收 ESP32S3 ACK/DONE/NACK/FAULT | 115200 8N1，3.3V TTL |
| GND | 两板信号参考地 | 必须共地 |
| USART1 TX | F4 输出机械臂调试日志到串口助手 | `PA9(TX)`，115200 8N1，接 USB-TTL RX |
| USART1 RX | 预留调试输入 | `PA10(RX)`，当前机械臂服务不主动读取 |
| USART2 TX/RX | MP157-F4 主链路 | `PA2(TX)`、`PA3(RX)`，115200 8N1，只承载 MP157 二进制协议和必要维护命令输入 |
