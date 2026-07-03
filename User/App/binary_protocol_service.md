# STM32MP157-F407 二进制协议服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/binary_protocol_service.c`、`User/App/binary_protocol_service.h` |
| 模块用途 | 解析 STM32MP157 发给 STM32F407 的自动检测二进制协议帧，并把已接入命令分发给传送带服务和摄像头电机服务。 |
| 当前阶段 | 已接入传送带居中闭环、手动传送带控制、状态查询、三台步进电机运行时参数下发和称重标定；机械臂、电感结果命令字先保留。 |

## 本次创建或修改文件

| 文件 | 类型 | 修改原因 |
|---|---|---|
| `User/App/binary_protocol_service.h` | 修改 | 定义协议帧格式、命令字、错误码、负载结构、CRC/解帧/分发接口；新增 `STEPPER_PARAM_SET(0x42)` 和 25 字节步进参数负载，新增 `WEIGHT_CALIBRATE(0x30)` 和 5 字节称重标定负载。 |
| `User/App/binary_protocol_service.c` | 修改 | 实现 CRC16-CCITT-FALSE、组帧、解帧、负载解码、ACK/NACK 发送和命令分发；新增三台步进电机参数校验和分发，新增称重标定解码与 `WeightService_RequestCalibration()` 调用。 |
| `User/App/uart_command.h` | 修改 | 增加 `UartCommand_SendRaw()`，用于发送包含 `0x00` 的二进制 ACK/NACK 帧。 |
| `User/App/uart_command.c` | 修改 | 让二进制原始帧发送和 `my_printf()` 共用 USART1 发送互斥锁，避免文本日志和二进制帧交叉。 |
| `User/App/conveyor_motor_service.h` | 修改 | 增加 `ConveyorMotorService_RequestScan/Stop/Track()` 和 `ConveyorMotorService_RequestRuntimeConfig()` 公共入口，供协议服务投递传送带控制和参数配置请求。 |
| `User/App/conveyor_motor_service.c` | 修改 | 复用已有队列和状态机实现公共入口；新增运行时地址、最小步长、常规速度和方向配置，避免二进制协议层直接操作 Emm42 串口。 |
| `User/App/camera_motor_service.h` | 修改 | 增加 `CameraMotorService_RequestRuntimeConfig()`，供 `STEPPER_PARAM_SET` 一次性更新摄像头前后轴和上下轴参数。 |
| `User/App/camera_motor_service.c` | 修改 | 摄像头电机任务新增运行时参数配置队列命令；默认点动速度支持 MP157 下发的 `0~5000 rpm`。 |
| `User/App/weight_service.h` | 修改 | 新增二进制标定请求结果枚举和 `WeightService_RequestCalibration()`，让协议层不直接访问 HX711 静态上下文。 |
| `User/App/weight_service.c` | 修改 | 在 USART1 唯一命令消费者中，优先识别 `A5 5A` 二进制帧，再处理机械臂帧和 ASCII 文本命令；新增二进制称重标定上下文和文本/二进制共用标定核心。 |
| `MDK-ARM/bisai_f407_project.uvprojx` | 修改 | 把 `binary_protocol_service.c` 加入 Keil 编译工程。 |
| `MDK-ARM/bisai_f407_project.uvoptx` | 修改 | 把 `binary_protocol_service.c` 加入 Keil 工程视图。 |
| `User/App/binary_protocol_service_host_test.c` | 新增 | Windows 主机侧测试 CRC、解帧、CRC 错误、`PAUSE_CYCLE/RESUME_CYCLE`、`VISION_POS`、`STEPPER_PARAM_SET` 和 `WEIGHT_CALIBRATE` 负载解码，不依赖 F4 硬件。 |
| `20_uvc_camera/qt_camera_display/main.cpp` | 修改 | MP157 侧把 `ACK status=1` 识别为重复帧未重新执行，不再显示为新的启动成功。 |
| `20_uvc_camera/qt_camera_display/test_qt_kms_overlay_assets.sh` | 修改 | 增加静态契约检查，防止 Qt 再把重复 ACK 当成成功，也防止 F4 重复 `START_CYCLE` 分支退化为只 ACK 不动作。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| USART1 | MP157/串口助手到 F4 的主通信入口 | 115200 8N1，PA9(TX)、PA10(RX)，DMA + IDLE 接收 |
| UART4 | F4 到张大头 Emm42 传送带电机 | 115200 8N1，PC10(TX)、PC11(RX)，传送带地址 `0x01` |
| USART6 | F4 到两个摄像头运动 Emm42 电机 | 115200 8N1，PC6(TX)、PC7(RX)，前进/后退轴地址 `0x02`，上下轴地址 `0x03` |
| FreeRTOS 队列 | 协议层到传送带任务和摄像头电机任务的异步控制 | 复用 `conveyor_motor_service.c` 与 `camera_motor_service.c` 内部长度为 1 的覆盖队列 |
| USART1 发送互斥锁 | 防止 ACK/NACK 与文本日志交叉 | 复用 `uart_command.c` 内部 `g_uart_tx_mutex` |

## 帧格式

```text
A5 5A VER CMD LEN SEQ_L SEQ_H PAYLOAD... CRC_L CRC_H 6B
```

| 字段 | 字节数 | 说明 |
|---|---:|---|
| `A5 5A` | 2 | 固定帧头。 |
| `VER` | 1 | 首版固定 `0x01`。 |
| `CMD` | 1 | 命令字。 |
| `LEN` | 1 | `PAYLOAD` 字节数，首版最大 48。 |
| `SEQ_L/SEQ_H` | 2 | 发送方帧序号，小端序。 |
| `PAYLOAD` | N | 命令负载，多字节整数小端序。 |
| `CRC_L/CRC_H` | 2 | CRC16-CCITT-FALSE，覆盖 `VER~PAYLOAD`。 |
| `6B` | 1 | 固定帧尾。 |

## 已接入命令

| CMD | 名称 | F4 动作 |
|---:|---|---|
| `0x01` | `HELLO` | 回 ACK，不触发硬件动作。 |
| `0x02` | `HEARTBEAT` | 回 ACK，不触发硬件动作。 |
| `0x10` | `START_CYCLE` | 保存 `cycle_id`，请求传送带进入 `SCAN`；若收到相同 `cycle_id` 的重复开始帧，F4 会重新投递一次 `SCAN`，避免 MP157 重启后只收到 ACK 但传送带没有新动作。 |
| `0x11` | `PAUSE_CYCLE` | 校验 `cycle_id`，请求传送带停止，保存暂停前状态并进入 `PAUSED`。 |
| `0x12` | `RESUME_CYCLE` | 校验 `cycle_id`，只允许从 `PAUSED` 继续；扫描阶段回 `SCAN`，跟踪阶段等待新的 `VISION_POS`。 |
| `0x13` | `STOP_CYCLE` | 校验 `cycle_id`，请求传送带停止并清当前流程。 |
| `0x20` | `VISION_POS` | 校验 `cycle_id` 和坐标有效位，计算 `axis_px - target_px` 后请求传送带 `TRACK`，成功回 `ACK`。 |
| `0x21` | `VISION_LOST` | 相机离线时停机，其它视觉丢失原因回到扫描，成功回 `ACK`，队列不可用回 `NACK`。 |
| `0x22` | `BELT_STOP_CENTERED` | 校验 `cycle_id`，请求传送带停止，并回 ACK。 |
| `0x30` | `WEIGHT_CALIBRATE` | 校验 5 字节负载，调用称重服务用已知砝码更新 HX711 运行时比例系数；成功回 `ACK`，未去皮、克重越界或采样异常回 `NACK`。 |
| `0x40` | `QUERY_STATUS` | 查询 F4 协议状态和传送带状态，成功直接回 `STATUS_REPORT`。 |
| `0x41` | `BELT_MANUAL_CONTROL` | 手动调试传送带扫描/停止，成功回 `ACK`，失败回 `NACK`。 |
| `0x42` | `STEPPER_PARAM_SET` | 校验并下发三台 Emm42 的地址、最小步长、常规速度和方向；成功只更新 F4 运行内存，不写 F4 Flash，也不写 Emm42 EEPROM。 |

## `STEPPER_PARAM_SET` 负载

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | 固定 `0` | 参数下发不绑定某一轮自动检测流程。 |
| 2 | `motor_count` | `u8` | 固定 `3` | 三台步进电机记录。 |
| 3 | `flags` | `u8` | 固定 `0` | 首版不表示持久化，也不写 EEPROM。 |
| 4 | `motor[0]` | 7 字节记录 | 见下表 | 电机记录顺序不作为业务依据，F4 按 `role_id` 识别。 |
| 11 | `motor[1]` | 7 字节记录 | 见下表 | 同上。 |
| 18 | `motor[2]` | 7 字节记录 | 见下表 | 同上。 |

单条电机记录格式：

| 记录内偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `role_id` | `u8` | `1/2/3` | `1=conveyor`，`2=camera_forward`，`3=camera_z`，三条记录必须刚好覆盖且不能重复。 |
| 1 | `address` | `u8` | `1~247` | Emm42 普通站号地址。 |
| 2 | `min_step` | `u16` | `1~10000` | 最小步长，当前 F4 保存给后续位置步进命令使用。 |
| 4 | `normal_speed_rpm` | `u16` | `0~5000` | 常规速度；传送带用于扫描速度，摄像头轴用于默认点动速度，`0` 表示默认运动保持停止。 |
| 6 | `direction` | `i8` | `1` 或 `-1` | 方向映射，`-1` 会反转该电机逻辑方向。 |

## `WEIGHT_CALIBRATE` 负载

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | 固定 `0` | 称重标定是人工维护动作，不绑定某一轮自动检测流程。 |
| 2 | `known_weight_g` | `u16` | `1~5000` | 已知砝码重量，单位克；F4 仍按 HX711 当前额定量程复核。 |
| 4 | `flags` | `u8` | 固定 `0` | 首版不表示自动去皮、不保存 Flash，也不触发其它扩展动作。 |

## 正确返回和错误返回

| 场景 | F4 返回帧 | 负载长度 | MP157 判断规则 |
|---|---|---:|---|
| `HELLO/HEARTBEAT/START_CYCLE/PAUSE_CYCLE/RESUME_CYCLE/STOP_CYCLE/VISION_POS/VISION_LOST/BELT_STOP_CENTERED/WEIGHT_CALIBRATE/BELT_MANUAL_CONTROL/STEPPER_PARAM_SET` 执行成功 | `ACK 0x80` | 7 | `acked_seq`、`acked_cmd`、`cycle_id` 匹配，且 `status=0` 才算本次命令成功。 |
| `QUERY_STATUS` 执行成功 | `STATUS_REPORT 0x82` | 24 | `replied_seq`、`replied_cmd=QUERY_STATUS`、`cycle_id` 匹配才算查询成功。 |
| 命令字未知、CRC 错、长度错、状态不允许、cycle 不匹配、硬件队列未就绪 | `NACK 0x81` | 9 | MP157 读取 `error_code/state/detail` 显示失败原因，不再解析任何 `[ERROR]` 文本。 |
| LDC、称重、传送带、摄像头电机、机械臂等模块主动发现故障 | `FAULT_REPORT 0x87` | 16 | MP157 读取 `fault_source/severity/fault_code/detail_i32/fault_bits`，作为结构化故障展示和上传依据。 |

> USART1 上的 `[OK]`、`[ERROR]`、`[INFO]` 文本在正式 MP157 主链路默认关闭，不能作为正确或错误判断依据。

## 编译与验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 主机侧协议测试 | `E:\hal\bisai_f407_project` | `gcc -std=c99 -Wall -Wextra -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\binary_protocol_service_host_test.c User\App\binary_protocol_service.c -o tmp\binary_protocol_service_host_test.exe; .\tmp\binary_protocol_service_host_test.exe` | 输出 `binary protocol host tests passed`；当前 host-test 裁剪硬件发送路径，会出现 `WriteI32Le` 未使用 warning，可接受。 | 若提示头文件找不到，检查 `-I User\App`；若 CRC、`STEPPER_PARAM_SET` 或 `WEIGHT_CALIBRATE` 解码失败，检查 CRC 覆盖范围和负载偏移。 |
| Keil 工程包含新文件 | `E:\hal\bisai_f407_project` | `Select-String -Path MDK-ARM\bisai_f407_project.uvprojx -Pattern "binary_protocol_service.c"` | 能看到 `../User/App/binary_protocol_service.c`。 | 如果查不到，Keil 不会编译新模块，需要重新加入工程文件。 |
| 二进制握手 | MP157 串口工具 | 发送合法 `HELLO` 或 `HEARTBEAT` 帧 | F4 返回 `ACK 0x80` 二进制帧，负载 7 字节。 | 若无 ACK，检查帧头、帧尾、CRC 和 `BinaryProtocolService_HandleFrame()` 是否已在 `weight_service.c` 中优先调用。 |
| 二进制状态查询 | MP157 串口工具 | 发送合法 `QUERY_STATUS 0x40` 帧，`query_mask=0x03` | F4 返回 `STATUS_REPORT 0x82`，负载 24 字节，能看到传送带模式、速度、误差和故障位。 | 若收到 `NACK`，按 `error_code` 排查；若无帧，先查 USART1 TX/RX、共地、波特率和 F4 是否烧录最新固件。 |
| 步进参数下发 | MP157 Qt 参数页或串口工具 | 发送合法 `STEPPER_PARAM_SET 0x42` 帧，速度分别覆盖 `0`、`137`、`5000` 等值 | F4 返回 `ACK 0x80`，`acked_cmd=0x42`、`status=0`；随后 `CAMINFO` 可看到摄像头轴运行时地址、步长、速度和方向映射。 | 若收到 `NACK error_code=5`，按 detail 查字段范围；若 `error_code=10`，检查传送带任务或摄像头电机任务队列是否已创建。 |
| 称重标定 | MP157 Qt 参数页或串口工具 | 发送合法 `WEIGHT_CALIBRATE 0x30` 帧，payload 为 `00 00 E8 03 00` 表示 `cycle_id=0, known_weight_g=1000, flags=0` | F4 已去皮且最近一次 HX711 采样有效时返回 `ACK 0x80`，`acked_cmd=0x30`、`status=0`。 | 若收到 `NACK error_code=6`，先确认空载去皮成功；若 `error_code=5`，检查克重是否 1~5000 且 flags/cycle_id 是否为 0；若 `error_code=10`，查 HX711 接线、供电和最近采样状态。 |
| 重复开始不空 ACK | MP157 串口工具或 Qt 首页 | 在 F4 未断电且同一 `cycle_id` 仍有效时再次发送 `START_CYCLE` | F4 重新投递传送带 `SCAN`，并返回 `ACK status=0 state=SCANNING`。 | 若 Qt 显示 `ACK重复帧`，说明 F4 仍按旧语义返回 `status=1`；若 status=0 但电机不动，继续查 UART4 接线、电机地址、使能和供电。 |
| 视觉坐标闭环 | MP157 串口工具 | 先发 `START_CYCLE`，再周期发 `VISION_POS` | 传送带进入扫描后按 `axis-target` 误差跟踪。 | 若传送带不动，查 `ConveyorMotorService_RequestTrack()` 是否返回成功、Emm42 `UART4 PC10/PC11` 接线、地址 `0x01` 和电机使能。 |
| 暂停继续语义 | MP157 串口工具 | 先发 `START_CYCLE`，再发 `PAUSE_CYCLE`，最后发 `RESUME_CYCLE` | 暂停时传送带停止；继续同一 `cycle_id`，扫描阶段恢复扫描，跟踪阶段等待下一帧坐标。 | 若继续失败，检查 F4 是否处于 `PAUSED`、`cycle_id` 是否一致、`resume_mode` 是否为合法值。 |

## 读写验证

| 数据路径 | 写入怎么做 | 读取/确认怎么做 |
|---|---|---|
| MP157 -> F4 二进制帧 | MP157 或串口工具向 USART1 写入 `A5 5A ... 6B`。 | F4 只用二进制 `ACK/NACK/STATUS_REPORT/FAULT_REPORT` 表达正确、错误和故障。 |
| F4 -> MP157 ACK/NACK/STATUS/FAULT | `BinaryProtocolService_SendFrame()` 调用 `UartCommand_SendRaw()` 原样发送。 | MP157 按相同帧格式解析；若串口助手查看，必须用 HEX 显示。 |
| 协议层 -> 传送带 | `START_CYCLE/PAUSE_CYCLE/RESUME_CYCLE/VISION_POS/STOP_CYCLE/BELT_MANUAL_CONTROL` 调用 `ConveyorMotorService_Request...()` 写入传送带队列。 | 发送二进制 `QUERY_STATUS` 查看 desired/applied/error/speed/centered，也可观察 Emm42 实际动作。 |
| 协议层 -> 三台步进电机运行参数 | `STEPPER_PARAM_SET` 调用 `ConveyorMotorService_RequestRuntimeConfig()` 和 `CameraMotorService_RequestRuntimeConfig()` 写入各自任务队列。 | F4 回 `ACK` 只表示运行内存已接收；发送 `CAMINFO` 可读摄像头两个轴参数，传送带参数会影响下一次 `BELTSCAN/START_CYCLE` 扫描速度和方向。 |
| 协议层 -> HX711 称重标定 | `WEIGHT_CALIBRATE` 调用 `WeightService_RequestCalibration()`，复用称重任务最近一次去皮状态和采样值。 | F4 回 `ACK` 表示 HX711 运行时 `scale_counts_per_g` 已更新；该值当前不写 Flash，F4 断电后需要重新标定或后续扩展持久化。 |

## 失败排查

| 现象 | 优先排查 |
|---|---|
| F4 把二进制帧当未知文本命令 | 检查帧头是否为 `A5 5A`，以及 `weight_service.c` 中二进制协议是否在机械臂和文本命令之前调用。 |
| F4 打印 CRC 错误 | 检查 CRC 是否覆盖 `VER CMD LEN SEQ PAYLOAD`，不要覆盖 `A5 5A`、CRC 字段和 `6B`。 |
| MP157 收不到 ACK | 检查 `UartCommand_SendRaw()` 是否被编译进工程，USART1 TX 线是否接到 MP157 RX。 |
| MP157 显示 `ACK重复帧` | 说明 F4 返回了 `ACK status=1`，Qt 不再把它当作新的启动成功；先确认 F4 是否已经烧录“重复 `START_CYCLE` 重新投递 `SCAN`”的固件，再按 `STOP_CYCLE` 或复位 F4 清理旧流程。 |
| Qt 首页显示自动流程 ACK 但传送带仍不动 | 先看 ACK 详情必须是 `status=0 state=SCANNING`；再发送二进制 `QUERY_STATUS` 看 `desired/applied/speed`；如果 `speed=60 rpm` 但电机不动，重点查 `UART4 PC10/PC11` 是否接到传送带驱动、TX/RX 是否交叉、F4 与电机是否共地、传送带 Emm42 地址是否为 `0x01`。 |
| 传送带方向越调越远 | 检查 `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW`，以及 MP157 发送的 `axis_px` 是否选对传送带运动方向坐标。 |
| 步进参数保存后 F4 没变化 | 确认 Qt 弹窗点的是 `保存并下发`，F4 是否返回 `ACK acked_cmd=0x42`；代码已写不等于 F4 已烧录生效，必须重新编译并下载 F407 固件。 |
| 称重标定返回 `ERR_CMD_UNKNOWN detail=48` | 说明 F4 仍是旧固件，没有烧录包含 `WEIGHT_CALIBRATE 0x30` 的版本；重新编译下载 F407。 |
| 称重标定返回 `ERR_STATE_NOT_ALLOWED` | 称重任务上下文未就绪或尚未成功空载去皮；先执行去皮流程，再放砝码点击标定。 |
| 称重标定返回 `ERR_FIELD_RANGE` | 检查 MP157 发送的 `cycle_id` 是否为 0、`flags` 是否为 0、克重是否在 `1~5000g`。 |
| 称重标定返回 `ERR_HARDWARE_FAULT` | 最近一次 HX711 采样失败或净计数为 0；检查 DOUT/SCK、供电、砝码是否放稳和 `latest_status`。 |
| 暂停后继续没有立刻运动 | 如果暂停前是 `TRACKING`，F4 不复用旧坐标，必须等 MP157 发送新的 `VISION_POS`；如果要强制重新扫描，`RESUME_CYCLE.resume_mode` 填 `1`。 |
| 停止后继续旧流程 | 停止会清 `active_cycle_id`，MP157 应重新发送新的 `START_CYCLE` 和新 `cycle_id`。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-02 | 新增二进制协议服务首版，实现 CRC、解帧、ACK/NACK、START/PAUSE/RESUME/STOP/VISION_POS/VISION_LOST/BELT_STOP_CENTERED，并接入传送带控制。 |
| 2026-07-02 | 同步 Emm42 串口资源：传送带改为 `UART4 PC10/PC11 addr=0x01`，摄像头两个电机共用 `USART6 PC6/PC7 addr=0x02/0x03`。 |
| 2026-07-02 | 修正重复 `START_CYCLE` 处理：F4 对相同 `cycle_id` 的开始命令重新投递 `SCAN`，MP157 对 `ACK status=1` 明确显示为重复帧失败提示，避免界面显示成功但传送带未动作。 |
| 2026-07-02 | USART1 主链路返回收敛为二进制：正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT`；文本 `[OK]/[ERROR]` 默认静默，不再作为 MP157 判断依据。 |
| 2026-07-02 | 补齐 `VISION_POS` 和 `VISION_LOST` 成功回 `ACK`，保证已实现命令不会出现“动作成功但主链路无二进制回包”的情况。 |
| 2026-07-03 | 新增 `STEPPER_PARAM_SET 0x42`，MP157 可下发三台 Emm42 的地址、最小步长、常规速度和方向；F4 只更新运行内存，不写 F4 Flash 或 Emm42 EEPROM。 |
| 2026-07-03 | 新增 `WEIGHT_CALIBRATE 0x30`，MP157 可发送 `cycle_id=0、known_weight_g、flags=0` 触发 HX711 运行时标定；成功回 ACK，未去皮、克重越界和采样异常回结构化 NACK。 |
