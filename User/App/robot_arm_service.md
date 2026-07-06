# F4 与 ESP32S3 机械臂正式协议服务

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/robot_arm_service.c`、`User/App/robot_arm_service.h` |
| 硬件链路 | STM32F407 USART3 `<->` ESP32S3 UART，固定 `115200 8N1`，3.3V TTL，共地 |
| 当前协议 | 只使用 `A5 5A VER CMD LEN SEQ_L SEQ_H PAYLOAD CRC_L CRC_H 6B` 正式二进制协议 |
| 上游入口 | `binary_protocol_service.c` 调用 `RobotArmService_RequestPlaceWeight()`、`RobotArmService_RequestPlaceLdc()`、`RobotArmService_RequestFinalSort()` |
| 下游回包 | ESP32S3 必须回 `ARM_ACK`，动作真实完成后再回 `ARM_STAGE_DONE` |
| 当前超时 | F4 发给 ESP32S3 的动作 `timeout_ms=100000ms`，按 `u32` 小端发送；F4 等 DONE 总窗口为 `100000+10000=110000ms` |

## 修改文件清单

| 文件 | 修改原因 | 影响的契约 |
|---|---|---|
| `User/App/robot_arm_service.c` | 移除旧动作组兼容发送，改为正式 `A5 5A` 组帧、ACK/NACK/DONE 解析和 DONE 后推进自动流程。 | F4 发给 ESP32S3 的 TX 不再出现旧动作组帧；ESP32S3 必须实现正式协议。 |
| `User/App/robot_arm_service.h` | 更新接口说明，明确旧协议停用，业务入口等待 ACK/DONE。 | 上游 `binary_protocol_service.c` 接口保持不变。 |
| `User/App/robot_arm_service.md` | 新增本模块说明、验证方式和故障排查。 | 后续联调可直接按本文确认协议、接线和回包。 |
| `User/App/weight_service.c` | 更新 USART1 命令分发说明，删除旧动作组示例。 | 现场调试说明不再引导用户发送旧协议。 |
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
| 故障上报 | ESP32S3 | 人为制造忙状态或抓取失败。 | ESP32S3 回 `ARM_NACK` 或 `ARM_STAGE_DONE result!=0`，F4 上报 ARM 故障。 | 检查 error_code、result、fault_bits 是否按文档填写。 |

## 修改记录

| 日期 | 修改点 | 说明 |
|---|---|---|
| 2026-07-06 | 切换到正式机械臂二进制协议 | F4 不再发送旧动作组兼容帧，改为 `A5 5A` 正式协议，并等待 ACK/DONE。 |
| 2026-07-06 | 放大机械臂单阶段等待时间 | `ROBOT_ARM_SERVICE_ACTION_TIMEOUT_MS` 从 `15000ms` 调整为 `60000ms`，DONE 额外余量从 `2000ms` 调整为 `5000ms`，避免真实机械臂动作慢时 F4 过早超时。 |
| 2026-07-07 | 扩展动作超时字段 | `timeout_ms` 从旧 `u16` 发送字段扩展为 `u32`，当前可完整发送 `100000ms`，避免被截断成 `34464ms`。 |

## 硬件资源

| 资源 | 用途 | 要求 |
|---|---|---|
| USART3 TX | F4 发送正式机械臂命令到 ESP32S3 RX | 115200 8N1，3.3V TTL |
| USART3 RX | F4 接收 ESP32S3 ACK/DONE/NACK/FAULT | 115200 8N1，3.3V TTL |
| GND | 两板信号参考地 | 必须共地 |
