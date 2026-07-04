# STM32MP157-F407 二进制协议服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/binary_protocol_service.c`、`User/App/binary_protocol_service.h` |
| 模块用途 | 解析 STM32MP157 发给 STM32F407 的自动检测二进制协议帧，并把已接入命令分发给传送带服务和摄像头电机服务。 |
| 当前阶段 | 已接入传送带居中闭环、手动传送带控制、状态查询、三台步进电机运行时参数下发、三轴位置/速度/停止执行器命令、当前位置设零和称重标定；机械臂、电感结果命令字先保留。 |

## 本次创建或修改文件

| 文件 | 类型 | 修改原因 |
|---|---|---|
| `User/App/binary_protocol_service.h` | 修改 | 定义协议帧格式、命令字、错误码、负载结构、CRC/解帧/分发接口；新增 `STEPPER_PARAM_SET(0x42)` 和 25 字节步进参数负载，新增 `WEIGHT_CALIBRATE(0x30)` 和 5 字节称重标定负载；新增 `ACTUATOR_POS_MOVE(0x50)`、`ACTUATOR_STOP(0x51)`、`ACTUATOR_VEL_MOVE(0x52)`、`ACTUATOR_HOME(0x53)` 执行器负载。 |
| `User/App/binary_protocol_service.c` | 修改 | 实现 CRC16-CCITT-FALSE、组帧、解帧、负载解码、ACK/NACK 发送和命令分发；新增三台步进电机参数校验和分发，新增称重标定解码与 `WeightService_RequestCalibration()` 调用；新增三轴位置运动、速度连续运动、停止和当前位置设零分发，并保证成功 ACK 的 `status=0`。 |
| `User/App/uart_command.h` | 修改 | 增加 `UartCommand_SendRaw()`，用于发送包含 `0x00` 的二进制 ACK/NACK 帧。 |
| `User/App/uart_command.c` | 修改 | 让二进制原始帧发送和 `my_printf()` 共用 USART1 发送互斥锁，避免文本日志和二进制帧交叉。 |
| `User/App/conveyor_motor_service.h` | 修改 | 增加 `ConveyorMotorService_RequestScan/Stop/Track()` 和 `ConveyorMotorService_RequestRuntimeConfig()` 公共入口，供协议服务投递传送带控制和参数配置请求；新增 `RequestJog/RequestPosition/RequestSetCurrentPositionZero` 执行器接口。 |
| `User/App/conveyor_motor_service.c` | 修改 | 复用已有队列和状态机实现公共入口；新增运行时地址、最小步长、常规速度和方向配置，避免二进制协议层直接操作 Emm42 串口；新增 JOG、相对位置和当前位置设零命令处理。 |
| `User/App/camera_motor_service.h` | 修改 | 增加 `CameraMotorService_RequestRuntimeConfig()`，供 `STEPPER_PARAM_SET` 一次性更新摄像头前后轴和上下轴参数；新增前后轴/上下轴 JOG、位置移动和当前位置设零接口。 |
| `User/App/camera_motor_service.c` | 修改 | 摄像头电机任务新增运行时参数配置队列命令；默认点动速度支持 MP157 下发的 `0~5000 rpm`；新增前后轴速度连续运动、前后轴/上下轴相对位置移动和当前位置设零。 |
| `User/Driver/emm42_motor.c`、`User/Driver/emm42_motor.h` | 修改 | 新增 `EMM42_MotorMoveRelativePosition()` 和 `EMM42_MotorResetCurrentPositionToZero()`，分别发送张大头 Emm42 `0xFD` 相对位置帧和 `[addr 0A 6D 6B]` 当前位置清零帧。 |
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
| USART6 | F4 到两个摄像头运动 Emm42 电机 | 115200 8N1，PC6(TX)、PC7(RX)，现场当前前进/后退轴地址 `0x03`，上下轴地址 `0x02` |
| FreeRTOS 队列 | 协议层到传送带任务和摄像头电机任务的异步控制 | 传送带复用内部队列；摄像头电机服务使用长度为 4 的短 FIFO，STOP 队首优先，避免运行时配置被后续动作覆盖 |
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
| `0x50` | `ACTUATOR_POS_MOVE` | 校验执行器、方向、速度和步数后，把传送带、摄像头前后轴或摄像头上下轴投递到相对位置模式；成功回 `ACK status=0`。 |
| `0x51` | `ACTUATOR_STOP` | 停止指定执行器；`actuator=0xFF` 停止传送带、摄像头前后轴和摄像头上下轴；成功回 `ACK status=0`。 |
| `0x52` | `ACTUATOR_VEL_MOVE` | 手动调试速度连续运动，只允许传送带和摄像头前后轴，直到收到 `ACTUATOR_STOP`；上下轴速度连续运动会返回 `NACK`。 |
| `0x53` | `ACTUATOR_HOME` | 参数页当前位置设零，F4 先停目标轴，再发送 Emm42 `[addr 0A 6D 6B]` 清零帧；不做主动回零运动，也不支持 `actuator=0xFF`。 |

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

## 执行器命令负载

### `ACTUATOR_POS_MOVE 0x50`

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | `0` 或当前活动流程 | 手动调试可为 `0`；自动流程必须匹配当前 `active_cycle_id`。 |
| 2 | `actuator` | `u8` | `0/1/2` | `0=传送带`，`1=摄像头前后轴`，`2=摄像头上下轴`。 |
| 3 | `direction` | `u8` | `0/1` | `0=后退/下降`，`1=前进/上升`，再由运行时方向映射到实际 CW/CCW。 |
| 4 | `mode` | `u8` | 固定 `0` | 首版只支持相对位置模式。 |
| 5 | `speed_rpm` | `u16` | `0~5000` | `0` 表示由对应电机服务使用运行时常规速度。 |
| 7 | `steps` | `u32` | `1~4294967295` | 相对移动步数，单位 step。 |
| 11 | `flags` | `u8` | 固定 `0` | 首版保留。 |

### `ACTUATOR_STOP 0x51`

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | `0` 或当前活动流程 | 手动停止可为 `0`。 |
| 2 | `actuator` | `u8` | `0/1/2/0xFF` | `0xFF` 表示停止全部可停止执行器。 |
| 3 | `flags` | `u8` | 固定 `0` | 首版保留。 |

### `ACTUATOR_VEL_MOVE 0x52`

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | `0` 或当前活动流程 | 手动连续运动通常为 `0`。 |
| 2 | `actuator` | `u8` | `0/1` | 只允许传送带和摄像头前后轴；上下轴必须用固定步数位置模式。 |
| 3 | `direction` | `u8` | `0/1` | `0=后退`，`1=前进`。 |
| 4 | `speed_rpm` | `u16` | `1~5000` | 持续速度，收到 `ACTUATOR_STOP` 前不会自动停。 |
| 6 | `flags` | `u8` | 固定 `0` | 首版保留。 |

### `ACTUATOR_HOME 0x53`

| 偏移 | 字段 | 类型 | 合法范围 | 说明 |
|---:|---|---|---|---|
| 0 | `cycle_id` | `u16` | `0` 或当前活动流程 | 参数页标定通常为 `0`。 |
| 2 | `actuator` | `u8` | `0/1/2` | 指定一台电机；禁止 `0xFF`，避免误触清零全部标定基准。 |
| 3 | `flags` | `u8` | 固定 `0` | 首版保留。 |

## 正确返回和错误返回

| 场景 | F4 返回帧 | 负载长度 | MP157 判断规则 |
|---|---|---:|---|
| `HELLO/HEARTBEAT/START_CYCLE/PAUSE_CYCLE/RESUME_CYCLE/STOP_CYCLE/VISION_POS/VISION_LOST/BELT_STOP_CENTERED/WEIGHT_CALIBRATE/BELT_MANUAL_CONTROL/STEPPER_PARAM_SET/ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE/ACTUATOR_HOME` 执行成功 | `ACK 0x80` | 7 | `acked_seq`、`acked_cmd`、`cycle_id` 匹配，且 `status=0` 才算本次命令成功；执行器命令不能把 `actuator` 填到 `status`。 |
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
| 手动速度连续运动 | MP157 Qt 手动三轴弹窗或串口工具 | 发送合法 `ACTUATOR_VEL_MOVE actuator=0/1 direction=0/1 speed_rpm>0`，再发送对应 `ACTUATOR_STOP`。 | 传送带或前后轴持续运动，直到 STOP；成功 ACK 必须是 `acked_cmd=0x52 status=0`，停止 ACK 必须是 `acked_cmd=0x51 status=0`。 | 若 MP157 显示失败且 `status=1/2`，说明 F4 仍把 actuator 填进 ACK status，需要重新烧录；若上下轴速度模式被拒绝，这是预期，应改用位置固定步数。 |
| 上下轴固定步数 | MP157 Qt 手动三轴弹窗或自动流程 | 发送 `ACTUATOR_POS_MOVE actuator=2 direction=0/1 steps=zDownFixedSteps/zUpFixedSteps`。 | 上下轴每点一次只移动对应固定步数，成功 ACK 为 `acked_cmd=0x50 status=0`。 | 若不动，检查上下轴地址 `0x02`、`USART6` 接线、运行时常规速度是否为 0、步数是否为 0。 |
| 参数页当前位置设零 | MP157 Qt 参数页步进弹窗或串口工具 | 发送合法 `ACTUATOR_HOME actuator=0/1/2 flags=0`。 | 目标轴先停止，再发送 Emm42 `[addr 0A 6D 6B]`；F4 返回 `ACK acked_cmd=0x53 status=0`，电机不主动寻找限位。 | 若返回 `NACK error_code=5`，检查 actuator 是否误填 `0xFF` 或 flags 非 0；若无效，确认 `EMM42_MotorResetCurrentPositionToZero()` 已编译进工程。 |
| 视觉坐标闭环 | MP157 串口工具 | 先发 `START_CYCLE`，再周期发 `VISION_POS` | 传送带进入扫描后按 `axis-target` 误差跟踪。 | 若传送带不动，查 `ConveyorMotorService_RequestTrack()` 是否返回成功、Emm42 `UART4 PC10/PC11` 接线、地址 `0x01` 和电机使能。 |
| 暂停继续语义 | MP157 串口工具 | 先发 `START_CYCLE`，再发 `PAUSE_CYCLE`，最后发 `RESUME_CYCLE` | 暂停时传送带停止；继续同一 `cycle_id`，扫描阶段恢复扫描，跟踪阶段等待下一帧坐标。 | 若继续失败，检查 F4 是否处于 `PAUSED`、`cycle_id` 是否一致、`resume_mode` 是否为合法值。 |

## 读写验证

| 数据路径 | 写入怎么做 | 读取/确认怎么做 |
|---|---|---|
| MP157 -> F4 二进制帧 | MP157 或串口工具向 USART1 写入 `A5 5A ... 6B`。 | F4 只用二进制 `ACK/NACK/STATUS_REPORT/FAULT_REPORT` 表达正确、错误和故障。 |
| F4 -> MP157 ACK/NACK/STATUS/FAULT | `BinaryProtocolService_SendFrame()` 调用 `UartCommand_SendRaw()` 原样发送。 | MP157 按相同帧格式解析；若串口助手查看，必须用 HEX 显示。 |
| 协议层 -> 传送带 | `START_CYCLE/PAUSE_CYCLE/RESUME_CYCLE/VISION_POS/STOP_CYCLE/BELT_MANUAL_CONTROL` 调用 `ConveyorMotorService_Request...()` 写入传送带队列。 | 发送二进制 `QUERY_STATUS` 查看 desired/applied/error/speed/centered，也可观察 Emm42 实际动作。 |
| 协议层 -> 三台步进电机运行参数 | `STEPPER_PARAM_SET` 调用 `ConveyorMotorService_RequestRuntimeConfig()` 和 `CameraMotorService_RequestRuntimeConfig()` 写入各自任务队列。 | F4 回 `ACK` 只表示运行内存已接收；发送 `CAMINFO` 可读摄像头两个轴参数，传送带参数会影响下一次 `BELTSCAN/START_CYCLE` 扫描速度和方向。 |
| 协议层 -> 三台执行器运动 | `ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE` 调用传送带或摄像头电机服务的 JOG、POSITION、STOP 接口。 | 观察目标电机动作，并核对 ACK `status=0`；传送带还可用 `QUERY_STATUS` 查看 JOG/POSITION 状态，摄像头轴用 `CAMINFO` 看最近动作。 |
| 协议层 -> Emm42 当前位置清零 | `ACTUATOR_HOME` 调用 `ConveyorMotorService_RequestSetCurrentPositionZero()`、`CameraMotorService_RequestForwardSetCurrentPositionZero()` 或 `CameraMotorService_RequestZSetCurrentPositionZero()`。 | F4 回 `ACK status=0` 后，目标电机不会转动；若再用相对位置移动，驱动器以新的当前位置作为位置基准。 |
| 协议层 -> HX711 称重标定 | `WEIGHT_CALIBRATE` 调用 `WeightService_RequestCalibration()`，复用称重任务最近一次去皮状态和采样值。 | F4 回 `ACK` 表示 HX711 运行时 `scale_counts_per_g` 已更新；该值当前不写 Flash，F4 断电后需要重新标定或后续扩展持久化。 |

## 失败排查

| 现象 | 优先排查 |
|---|---|
| F4 把二进制帧当未知文本命令 | 检查帧头是否为 `A5 5A`，以及 `weight_service.c` 中二进制协议是否在机械臂和文本命令之前调用。 |
| F4 打印 CRC 错误 | 检查 CRC 是否覆盖 `VER CMD LEN SEQ PAYLOAD`，不要覆盖 `A5 5A`、CRC 字段和 `6B`。 |
| MP157 收不到 ACK | 检查 `UartCommand_SendRaw()` 是否被编译进工程，USART1 TX 线是否接到 MP157 RX。 |
| MP157 显示 `ACK重复帧` | 说明 F4 返回了 `ACK status=1`，Qt 不再把它当作新的启动成功；先确认 F4 是否已经烧录“重复 `START_CYCLE` 重新投递 `SCAN`”的固件，再按 `STOP_CYCLE` 或复位 F4 清理旧流程。 |
| Qt 首页显示自动流程 ACK 但传送带仍不动 | 先看 ACK 详情必须是 `status=0 state=SCANNING`；再发送二进制 `QUERY_STATUS` 看 `desired/applied/speed`；如果 `speed=60 rpm` 但电机不动，重点查 `UART4 PC10/PC11` 是否接到传送带驱动、TX/RX 是否交叉、F4 与电机是否共地、传送带 Emm42 地址是否为 `0x01`。 |
| 传送带方向越调越远 | 上方来料时 MP157 会发送 `axis_px=center_y`、`target_px=height/2`，零件刚入画通常是负误差；当前 F4 要求 `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW=0U`，让负误差沿扫描方向继续送入 ROI 中心。若现场又反向，先查该宏、运行时 direction 是否被参数页反转，以及 MP157 发送的 `axis_px` 是否仍选对传送带运动方向坐标。 |
| 视觉跟踪速度太快、停在 ROI 前后反复往返 | 当前 F4 已把跟踪上限降到 `80 rpm`、加速度降到 `2`、低速爬行区扩大到 `50 px`、中心死区扩大到 `24 px`。若仍过冲，优先继续降低 `CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM` 到 `60 rpm`；若停得太早，再把 `CONVEYOR_MOTOR_CENTER_DEADBAND_PX` 从 `24` 缩到 `20` 或 `18`。 |
| 小误差持续发坐标但传送带不动 | 这是误差刚超过死区、但低速命令不足以克服静摩擦的典型现象；当前 F4 已把中心死区从 `18 px` 放大到 `24 px`，并把最小跟踪速度从 `10 rpm` 提高到 `20 rpm`。若 `BELTTRACK 25` 仍不动，继续把 `CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM` 提到 `25U`；若开始轻微过冲，先保持 `20 rpm`，只把死区缩回 `20 px`。 |
| 步进参数保存后 F4 没变化 | 确认 Qt 弹窗点的是 `保存并下发`，界面显示的下发 ID 是否为现场期望值，例如 `前后=3，上下=2`；F4 是否返回 `ACK acked_cmd=0x42`；随后用 `CAMINFO` 或 USART1 `Runtime config applied` 日志确认运行时地址已变；代码已写不等于 F4 已烧录生效，必须重新编译并下载 F407 固件。 |
| 手动传送带或前后轴按一次后没有持续运动 | 确认 Qt 发的是 `ACTUATOR_VEL_MOVE 0x52`，不是旧的 `ACTUATOR_POS_MOVE`；确认 ACK `status=0`，再查传送带/前后轴电机供电、地址和 UART。 |
| 前后轴停止键 ACK 后仍继续运动 | 优先确认 F4 已烧录包含 `camera_motor_service.c` 的 `stop_epoch` 修复版本；旧版本虽然把 STOP 插到队首，但队列里残留的旧 `JOG` 可能在 STOP 后继续执行，表现为“停止没反应”。新版本 USART1 应看到 `[OK][CAM] Stop applied...`，必要时还会看到 `Drop stale motion after STOP`。 |
| 上下轴方向按钮变成一直运动 | 上下轴不允许 `ACTUATOR_VEL_MOVE`，应只发送 `ACTUATOR_POS_MOVE actuator=2` 和固定 `zDownFixedSteps/zUpFixedSteps`；检查 MP157 是否部署了最新 Qt 二进制。 |
| 参数页设零没有效果 | 先确认 F4 返回 `ACK acked_cmd=0x53 status=0`；若返回未知命令，说明 F4 未烧录；若 ACK 但后续位置基准没有变化，检查 Emm42 驱动器是否支持 `[addr 0A 6D 6B]` 清零命令、地址是否匹配。 |
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
| 2026-07-02 | 同步 Emm42 串口资源：传送带改为 `UART4 PC10/PC11 addr=0x01`，摄像头两个电机共用 `USART6 PC6/PC7`；早期规划为前后 `addr=0x02`、上下 `addr=0x03`，2026-07-04 已按现场实物改为前后 `0x03`、上下 `0x02`。 |
| 2026-07-02 | 修正重复 `START_CYCLE` 处理：F4 对相同 `cycle_id` 的开始命令重新投递 `SCAN`，MP157 对 `ACK status=1` 明确显示为重复帧失败提示，避免界面显示成功但传送带未动作。 |
| 2026-07-02 | USART1 主链路返回收敛为二进制：正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT`；文本 `[OK]/[ERROR]` 默认静默，不再作为 MP157 判断依据。 |
| 2026-07-02 | 补齐 `VISION_POS` 和 `VISION_LOST` 成功回 `ACK`，保证已实现命令不会出现“动作成功但主链路无二进制回包”的情况。 |
| 2026-07-03 | 新增 `STEPPER_PARAM_SET 0x42`，MP157 可下发三台 Emm42 的地址、最小步长、常规速度和方向；F4 只更新运行内存，不写 F4 Flash 或 Emm42 EEPROM。 |
| 2026-07-03 | 新增 `WEIGHT_CALIBRATE 0x30`，MP157 可发送 `cycle_id=0、known_weight_g、flags=0` 触发 HX711 运行时标定；成功回 ACK，未去皮、克重越界和采样异常回结构化 NACK。 |
| 2026-07-04 | 修正上方来料视觉跟踪方向：`CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW` 改为 `0U`，使 `axis_px-target_px<0` 时传送带继续把零件送入 ROI 中心，而不是反推回画面上方。 |
| 2026-07-04 | 降低视觉跟踪速度：传送带跟踪最大速度改为 `80 rpm`，加速度改为 `2`，低速爬行区改为 `50 px`，中心死区改为 `18 px`，减少 ROI 附近过冲往返。 |
| 2026-07-04 | 修正小误差卡滞：中心死区继续放大到 `24 px`，最小跟踪速度提高到 `20 rpm`，避免误差略大于死区时 F4 持续下发坐标但传送带因静摩擦不动。 |
| 2026-07-04 | 新增 `ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE/ACTUATOR_HOME` 执行器协议；传送带和前后轴手动速度模式按一次持续运动，停止键结束；上下轴继续使用固定步数位置模式；参数页可发送 `ACTUATOR_HOME` 把当前位置设为零点。 |
| 2026-07-04 | 修正执行器命令成功 ACK：`ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE/ACTUATOR_HOME` 成功时统一返回 `status=0`，不能把 `actuator` 写入 ACK `status`，避免 MP157 把前后轴/上下轴成功回包误判为失败。 |
| 2026-07-04 | 摄像头轴默认地址按现场实物改为前后轴 `0x03`、上下轴 `0x02`；摄像头电机服务队列改为短 FIFO，STOP 队首优先，避免 `STEPPER_PARAM_SET` 已 ACK 但 CONFIG 被下一条手动动作覆盖。 |
| 2026-07-04 | 修正前后轴停止键无效：摄像头电机服务新增 `stop_epoch`，STOP 后自动丢弃旧 JOG/POSITION/HOME 运动命令，避免旧队列命令在 STOP 后重新启动前后轴。 |
