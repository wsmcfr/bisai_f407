# 摄像头运动电机服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/camera_motor_service.c`、`User/App/camera_motor_service.h` |
| 模块用途 | 在 F407 上独占 `USART6 PC6/PC7`，串行控制摄像头左右轴和上下轴两台张大头 Emm42 步进电机。 |
| 当前阶段 | `CAMLAT LEFT/RIGHT` 负责左右轴文本调试，`CAMZ UP/DOWN` 负责上下轴文本调试；MP157 通过 `STEPPER_PARAM_SET 0x42` 下发两台摄像头电机运行时地址、最小步长、常规速度和方向；`ACTUATOR_POS_MOVE/ACTUATOR_STOP/ACTUATOR_VEL_MOVE/ACTUATOR_HOME` 分别支持位置微调、停止、持续速度模式和当前位置设零。 |

## 本次修改文件

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `User/App/camera_motor_service.c` | 把原前进/后退轴业务语义改为左右轴，新增 `CAMERA_MOTOR_LATERAL_ADDRESS`、`CAMLAT LEFT/RIGHT`、`CameraMotorService_RequestLateral...()` 接口，并保留旧 `CAMFWD/RequestForward...` 兼容包装。 | MP157 自动 ROI 复查时，`actuator=1` 可以用于 X 方向左右微调；现场串口主调试命令改为 `CAMLAT`。 |
| `User/App/camera_motor_service.h` | 对外声明左右轴点动、位置移动和设零接口，同时注明旧前进/后退接口只是兼容别名。 | 二进制协议层可以用新接口表达真实机构含义，旧调用点不会立刻编译失败。 |
| `User/App/binary_protocol_service.c/.h` | 把 `role_id=2`、`actuator=1` 的注释和分发目标改为摄像头左右轴，编号不变。 | MP157/F4 二进制协议保持兼容，只改变业务语义。 |
| `User/Driver/emm42_motor.c/.h` | 同步通用驱动注释中的摄像头左右轴地址 `0x03`、上下轴地址 `0x02`。 | 避免现场按旧地址文档接线或排查。 |

## 硬件资源

| 资源 | 用途 | 参数 |
|---|---|---|
| `USART6` | F4 到两个摄像头 Emm42 的 TTL 总线 | `PC6(TX) / PC7(RX)`，115200 8N1，两个电机共线必须靠地址区分 |
| 左右轴 Emm42 | 摄像头横向微调，让零件 X 方向更接近 ROI 中心 | 现场默认地址 `0x03`，运行时可配置为 `1~247` |
| 上下轴 Emm42 | 摄像头下探检测和回升 | 现场默认地址 `0x02`，运行时可配置为 `1~247`，不能和左右轴相同 |
| 传送带 Emm42 | 前后方向上料和 ROI Y 方向微调 | 由 `conveyor_motor_service.c` 独占 `UART4 PC10/PC11`，不占用 `USART6` |

## 接口契约

| 接口 | 作用 | 参数边界 |
|---|---|---|
| `CameraMotorService_RequestLateralJog(right_flag, speed_rpm)` | 请求左右轴持续点动，`right_flag=1` 右移，`0` 左移。 | `speed_rpm=0` 时使用当前运行时常规速度，最大 `5000 rpm`。 |
| `CameraMotorService_RequestLateralPosition(right_flag, speed_rpm, pulse_count)` | 请求左右轴按相对位置模式移动固定步数。 | `pulse_count>0`；`speed_rpm=0` 时使用运行时常规速度。 |
| `CameraMotorService_RequestLateralSetCurrentPositionZero()` | 请求左右轴停止后把当前位置设为零点。 | 供 `ACTUATOR_HOME actuator=1` 使用，不主动寻找限位。 |
| `CameraMotorService_RequestZPosition(up_flag, speed_rpm, pulse_count)` | 请求上下轴固定步数下降或上升。 | 首页自动流程下降后检测，检测结束后回升。 |
| `CameraMotorService_RequestStopAll()` | 请求两个摄像头轴立即停止。 | STOP 队首优先，并递增 `stop_epoch` 丢弃旧运动命令。 |
| `CameraMotorService_RequestRuntimeConfig(...)` | 更新左右轴和上下轴运行时地址、最小步长、常规速度和方向。 | 地址 `1~247` 且两轴不同；步长 `1~10000`；速度 `0~5000 rpm`；方向只能 `1/-1`。 |
| `CameraMotorService_RequestForward...()` | 旧接口兼容包装。 | 只映射到左右轴；新代码不要再用它表达业务含义。 |

说明：运行时配置只保存在 F4 RAM；不会写 F4 Flash，也不会写 Emm42 EEPROM。F4 断电重启后恢复代码默认值，MP157 需要重新下发。

## 使用与验证

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 查询当前运行时配置 | USART1 串口助手 | `CAMINFO` | 输出 `USART6=PC6/PC7`、`lateral_addr=3`、`z_addr=2`、`min_step/speed/dir` 和最近动作。 | 若未知命令，检查 `weight_service.c` 是否调用 `CameraMotorService_HandleCommand()`；若仍输出旧 `forward_addr`，说明 F4 还没烧录新固件。 |
| 左右轴左移 | USART1 串口助手 | `CAMLAT LEFT 30` | 只有地址 `0x03` 的左右轴动作。 | 若上下轴动作，检查两个摄像头电机 ID 是否接反；若方向反，调整 MP157 参数页方向映射。 |
| 左右轴右移 | USART1 串口助手 | `CAMLAT RIGHT 30` | 只有地址 `0x03` 的左右轴反向动作。 | 查 `USART6` 接线、地址、使能和供电。 |
| 旧命令兼容 | USART1 串口助手 | `CAMFWD FORWARD 30` | 仍能驱动左右轴右移，仅作为旧调试别名。 | 新文档和新 UI 不再使用该命令作为主路径。 |
| 上下轴指定速度点动 | USART1 串口助手 | `CAMZ UP 137` | 上下轴以 `137 rpm` 点动。 | 若速度被截断，检查是否已使用支持 `0~5000 rpm` 的新固件。 |
| 停止两个摄像头轴 | USART1 串口助手 | `CAMSTOP` | 左右轴和上下轴都停止，日志出现 `Stop applied. lateral_addr=3, z_addr=2`。 | 检查两个电机地址是否不同，`USART6` 是否被其它任务占用。 |
| MP157 下发参数 | MP157 Qt 参数页 | `参数设置 -> 步进参数 -> 保存并下发` | F4 返回 `ACK acked_cmd=0x42 status=0`；随后 `CAMINFO` 可看到左右轴地址、步长、速度、方向变化。 | 若返回 `NACK error_code=5`，检查字段范围；若 `error_code=10`，检查摄像头电机任务队列是否已创建或队列是否已满。 |
| MP157 左右轴持续运动 | MP157 Qt 手动三轴弹窗 | 切到 `摄像头左右电机`，点击 `左移` 或 `右移`，再点击停止。 | F4 先返回 `ACK acked_cmd=0x52 status=0`；停止时返回 `ACK acked_cmd=0x51 status=0`，左右轴停止。 | 若点击一次只动一下，确认 MP157 发的是 `ACTUATOR_VEL_MOVE`；若 STOP 后又动，确认已烧录包含 `stop_epoch` 的固件。 |
| 自动 ROI 微调 | MP157 首页自动流程 | Z 轴下降并对焦等待后，ROI 复查 `errorY` 用传送带短步微调，`errorX` 用 `ACTUATOR_POS_MOVE actuator=1` 左右微调。 | X/Y 都进入死区后进入模型检测，检测完成后 Z 轴回升。 | 若现场仍按旧前后语义动作，检查 F4 是否还是旧固件、MP157 是否部署新 Qt 二进制。 |

## 读写验证

| 数据通路 | 写操作 | 读/确认操作 | 判定 |
|---|---|---|---|
| USART1 文本到摄像头任务 | 发送 `CAMLAT LEFT 137`、`CAMLAT RIGHT 137` 或 `CAMZ UP 137`。 | 观察对应轴动作，再发送 `CAMSTOP`。 | 只有目标轴动作，说明文本分发和 `USART6` 地址区分正常。 |
| MP157 二进制参数到摄像头任务 | 发送 `STEPPER_PARAM_SET 0x42`，role 2/3 分别填写摄像头左右和上下轴参数。 | F4 回 ACK 后发送 `CAMINFO`。 | `CAMINFO` 中 role 2/3 对应地址、速度、步长、方向与 MP157 参数页一致。 |
| MP157 二进制运动到摄像头任务 | 发送 `ACTUATOR_VEL_MOVE actuator=1`、`ACTUATOR_POS_MOVE actuator=1/2` 或 `ACTUATOR_STOP actuator=1/2/0xFF`。 | 观察目标轴动作或停机，再查 ACK 详情。 | ACK `status=0` 且只有目标轴动作，说明协议层、队列和 `USART6` 地址区分正常。 |
| STOP 后旧运动命令丢弃 | 快速连续发送 `ACTUATOR_VEL_MOVE actuator=1` 和 `ACTUATOR_STOP actuator=1`。 | 左右轴停止，USART1 日志至少出现 `Stop applied`；若 STOP 插队时旧运动命令还在队列中，应出现 `Drop stale motion after STOP`。 | 若停止后又继续动，检查 `CameraMotorService_PostCommand()` 是否给 STOP 递增 `stop_epoch`。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-03 | 新增摄像头两轴运行时配置队列命令，支持 MP157 下发地址、最小步长、常规速度和方向。 |
| 2026-07-04 | 新增执行器位置、速度、停止和当前位置设零协议，传送带和摄像头横向轴支持速度持续运动，上下轴使用固定步数位置模式。 |
| 2026-07-04 | 现场默认地址为左右轴 `0x03`、上下轴 `0x02`；摄像头任务队列改为长度 4 的 FIFO，STOP 队首优先。 |
| 2026-07-04 | 原前进/后退轴业务语义改为左右轴；新增 `CAMLAT LEFT/RIGHT` 主调试命令，`CAMFWD FORWARD/BACKWARD` 仅作为兼容别名。 |
| 2026-07-04 | 自动检测流程调整为：传送带负责前后/Y 方向微调，左右轴负责 X 方向微调。 |
