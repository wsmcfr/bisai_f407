# Emm42 Conveyor Code Guide

## 1. Scope

这份文档只解释当前 Emm42 传送带链路里真正参与工作的文件，不展开所有 CubeMX 生成文件。

说明：

- 下面所有“行号”都基于你当前这份本地源码版本。
- 以后如果你继续加功能，行号会往后漂；但只要按“文件 + 函数名 + 代码段职责”去找，仍然能快速定位。

| 文件 | 角色 | 你最该关注的内容 |
| --- | --- | --- |
| `Core/Src/usart.c` | `USART1/USART2` 外设初始化 | `huart2` 有没有真正初始化成 Emm42 需要的串口口子 |
| `Core/Src/gpio.c` | GPIO 初始化 | `PF9` 心跳灯和 `PB0/PB2` 等外设引脚是否由 CubeMX 正确初始化 |
| `Core/Src/freertos.c` | 任务启动胶水 | 电机任务有没有真正被创建 |
| `User/App/uart_command.c` | `USART1` 命令接收与线程安全打印 | `DMA + 空闲中断` 怎么把命令交给任务 |
| `User/App/weight_service.c` | 当前统一串口命令入口拥有者 | 为什么电机命令不是自己直接读串口 |
| `User/App/conveyor_motor_service.h` | 电机服务对外接口 | 只有任务入口和命令入口两个公开点 |
| `User/App/conveyor_motor_service.c` | 传送带电机主状态机 | 巡航、跟踪、居中停止、启动修复都在这里 |
| `User/App/system_heartbeat_service.h` | 心跳灯任务接口 | FreeRTOS 里那个 `PF9` 任务到底对外暴露了什么 |
| `User/App/system_heartbeat_service.c` | 心跳灯任务实现 | 为什么闪、多久闪、靠什么 API 保持周期稳定 |
| `User/Driver/emm42_motor.h` | Emm42 驱动协议声明 | 状态码、方向、控制模式、驱动接口 |
| `User/Driver/emm42_motor.c` | Emm42 协议帧发送实现 | 每条串口命令具体发了什么字节 |

## 2. Whole Flow

当前 Emm42 传送带链路从上位机到电机的真实数据流如下：

| 步骤 | 文件 | 关键函数/行号 | 作用 |
| --- | --- | --- | --- |
| 1 | `User/App/uart_command.c` | `HAL_UARTEx_RxEventCallback()`，232 行 | `USART1` 收到一帧命令后，用 `DMA + IDLE` 判定“这一帧收完了” |
| 2 | `User/App/uart_command.c` | `UartCommand_Fetch()`，120 行 | 把完整命令安全取给任务上下文 |
| 3 | `User/App/weight_service.c` | `WeightService_ProcessCommand()`，371 行 | 当前由称重任务统一做串口命令分发 |
| 4 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_HandleCommand()`，983 行 | 识别 `BELTSCAN/BELTSTOP/BELTTRACK/BELTCAM/BELTENABLE/BELTINFO` |
| 5 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_PostCommand()`，394 行 | 把命令投递到电机任务队列 |
| 6 | `User/App/conveyor_motor_service.c` | `ConveyorMotorService_Task()`，1080 行 | 电机任务独占 `USART2`，周期推进状态机 |
| 7 | `User/Driver/emm42_motor.c` | `EMM42_MotorSetVelocity()` 等，179/219 行 | 组织 Emm42 TTL 协议帧并通过 `USART2` 发送 |
| 8 | `Core/Src/freertos.c` | `osThreadNew(ConveyorMotorService_Task, ...)`，132 行 | 真正把电机任务挂进 FreeRTOS 调度器 |

## 3. Why It Was “Not Rotating”

这次我顺手修了两个会直接导致“你以为代码没问题，但电机不按预期动”的点：

| 问题 | 现在怎么处理 |
| --- | --- |
| 电机面板误按后，电机内部控制模式可能被改乱 | 启动时在 `conveyor_motor_service.c:748` 强制恢复当前工程要的控制模式 |
| 当前 `freertos.c` 里一度没有重新创建电机任务 | 已在 `freertos.c:132` 把 `ConveyorMotorService_Task` 重新挂回调度器 |

## 4. `User/Driver/emm42_motor.h`

### 4.1 文件定位

这个头文件只做一件事：把 Emm42 需要的“协议概念”定义成应用层能看懂的 C 类型和函数声明。

### 4.2 逐段说明

| 行号 | 代码段 | 作用 | 调试时怎么理解 |
| --- | --- | --- | --- |
| 1-10 | 头文件保护 + `extern "C"` | 防止重复包含，兼容 C++ | 标准模板，没有业务逻辑 |
| 12-17 | `#include "main.h"` + 标准头 | 让 `UART_HandleTypeDef`、`bool`、定宽整数可见 | 驱动层需要直接持有 `UART` 句柄 |
| 21-28 | `EMM42_MotorStatus_t` | 统一驱动返回状态 | 以后所有底层协议失败都走这个枚举，不到处写魔法数字 |
| 38-43 | `EMM42_MotorDirection_t` | 方向定义，`CW/CCW` | 应用层不再直接写 `0/1` |
| 54-58 | `EMM42_MotorControlMode_t` | 控制模式定义，`OPEN_LOOP/CLOSED_LOOP_FOC` | 这次新增，就是为了解决“误按后模式错乱” |
| 71-76 | `EMM42_MotorHandle_t` | 驱动句柄 | 当前最小必要信息只有 `huart/address/tx_timeout_ms` |
| 84-101 | 4 个 `#define` 常量 | 默认地址、最大转速、最大加速度、默认超时 | 都是协议边界值，不该散落在应用层 |
| 111 行 | `EMM42_MotorLoadDefaultConfig()` | 绑定串口并写入默认地址/超时 | 只是填句柄，不发命令 |
| 123 行 | `EMM42_MotorInit()` | 校验句柄完整性 | 当前相当于统一初始化入口，后续能扩展 |
| 132 行 | `EMM42_MotorSetEnable()` | 发使能/失能帧 | 电机开始可响应运动命令前先走这里 |
| 148 行 | `EMM42_MotorSetControlMode()` | 发“开环/闭环 FOC”切换帧 | 这次新增的核心接口 |
| 162 行 | `EMM42_MotorSetButtonLock()` | 发“锁/解锁面板按键”帧 | 现在默认不启用，但能力已经补好 |
| 178 行 | `EMM42_MotorSetVelocity()` | 发速度模式帧 | 巡航和跟踪最后都走到这里 |
| 190 行 | `EMM42_MotorStopNow()` | 发立即停机帧 | 进入死区、超时、启动复位都要用 |

## 5. `User/Driver/emm42_motor.c`

### 5.1 文件定位

这个源文件是纯协议层，不做状态机，不碰视觉误差，也不碰 FreeRTOS 队列。它只管一件事：把上层意图变成一帧 UART 字节流。

### 5.2 逐段说明

| 行号 | 代码段 | 作用 | 关键点 |
| --- | --- | --- | --- |
| 1 行 | `#include "emm42_motor.h"` | 引入本模块声明 | 标准入口 |
| 16-34 | `EMM42_MotorTransmitFrame()` | 统一发一帧 TTL 协议 | 当前项目约定电机任务独占 `USART2`，所以直接阻塞发送 |
| 20-23 | 参数判空 | 防止空句柄、空帧、零长度 | 出错直接返回 `INVALID_PARAM` |
| 25-31 | `HAL_UART_Transmit()` | 真正把字节送到 `USART2` | 如果这里失败，上层只会看到一个统一状态码 |
| 41-51 | `EMM42_MotorLoadDefaultConfig()` | 写句柄默认值 | 只改内存，不访问硬件 |
| 58-66 | `EMM42_MotorInit()` | 校验句柄 | 当前没做额外寄存器初始化，主要是防呆 |
| 75-98 | `EMM42_MotorSetEnable()` | 组织 `[addr F3 AB state sync 6B]` 帧 | 这是例程原始使能协议 |
| 107-136 | `EMM42_MotorSetControlMode()` | 组织 `[addr 46 69 save mode 6B]` 帧 | 这次新增，用于启动时强制恢复闭环 FOC |
| 118-122 | 控制模式合法性检查 | 只允许 `0/1` | 防止应用层把无效模式值直接发给电机 |
| 145-168 | `EMM42_MotorSetButtonLock()` | 组织 `[addr D0 B3 save lock 6B]` 帧 | 这条也是直接照官方例程封装 |
| 179-210 | `EMM42_MotorSetVelocity()` | 速度控制主命令 | 巡航和跟踪最终都发这个帧 |
| 192-195 | 速度/加速度范围检查 | 防止越界 | 超范围直接报 `RANGE_ERROR` |
| 219-239 | `EMM42_MotorStopNow()` | 组织 `[addr FE 98 sync 6B]` 帧 | “立即停止”专用 |

## 6. `User/App/conveyor_motor_service.h`

### 6.1 文件定位

这是电机应用层的公共入口头文件。它故意只暴露两个接口，避免外部模块随意伸手碰内部状态。

### 6.2 逐段说明

| 行号 | 代码段 | 作用 |
| --- | --- | --- |
| 1-13 | 头文件保护 + `stdint.h` | 保证接口自洽 |
| 21 行 | `ConveyorMotorService_Task()` | FreeRTOS 任务入口，电机任务真正从这里跑起来 |
| 23-38 | 任务职责注释 | 交代这个任务独占 `USART2`、维护三态、根据误差调速 |
| 39 行 | `ConveyorMotorService_HandleCommand()` | 串口命令分发入口，当前由称重任务调用它 |

## 7. `User/App/conveyor_motor_service.c`

### 7.1 文件定位

这是整套 Emm42 逻辑的核心文件。它既是协议上层封装，也是电机状态机拥有者。

### 7.2 从上到下怎么读

| 行号 | 代码段 | 作用 | 你最该记住什么 |
| --- | --- | --- | --- |
| 20-79 | 控制周期、超时、巡航/跟踪速度等宏 | 定义所有运动参数 | 真机调速度，优先看这里 |
| 88-127 | 启动修复配置宏 | 定义“启动时强制恢复控制模式”的策略 | 这次新增，解决误按后模式错乱 |
| 135-169 | 死区、稳定帧数、方向映射、上电巡航、队列长度 | 定义状态机外围行为 | 小误差停机、默认巡航都在这里改 |
| 174-191 | `ConveyorMotor_Mode_t` / `ConveyorMotor_CommandType_t` | 任务高层状态和队列命令类型 | 一个是“想运行成什么模式”，一个是“收到什么命令” |
| 198-239 | 3 个结构体 | 队列命令、运行时快照、任务内部状态 | 快照给 `BELTINFO` 看，运行时只给任务自己用 |
| 243-261 | 全局静态对象 | 命令队列 + 共享快照 | 队列是任务间边界，快照是只读观察口 |
| 266-559 | 字符串解析辅助 | 负责把 BELT 命令里的参数拆出来 | 命令格式错误基本都卡在这里 |
| 568-621 | 误差到速度/方向的映射 | 把像素误差变成 `rpm + dir` | 视觉误差不会直接喂给驱动层 |
| 625-738 | `ApplyStop/ApplyScan/ApplyTrack` | 把状态机意图转换成实际电机命令 | 状态机和驱动层的真正边界在这里 |
| 748-788 | `ConveyorMotorService_ApplyStartupConfig()` | 启动时先恢复电机基础模式 | 这次新增的关键修复函数 |
| 794-815 | `ConveyorMotorService_ReportReady()` | 启动日志 | 现在会把 `ctrl`、`startup_fix` 都打印出来 |
| 822-828 | `ConveyorMotorService_ReportDriverError()` | 统一错误日志输出 | 便于快速知道失败在 init/config/enable/stop 哪一步 |
| 835-870 | `ConveyorMotorService_HandleQueuedCommand()` | 把队列命令写进运行时状态 | 这里只改“期望状态”，不直接发运动帧 |
| 883-976 | `ConveyorMotorService_ControlStep()` | 电机主状态机 | 巡航、跟踪、死区停止、超时停机都在这里 |
| 983-1067 | `ConveyorMotorService_HandleCommand()` | 统一处理 `BELT...` 文本命令 | 它不直接碰串口 DMA，只处理已经取出的完整字符串 |
| 1080-1181 | `ConveyorMotorService_Task()` | 任务主体 | 创建队列、初始化电机、执行启动修复、进入主循环 |

### 7.3 启动修复这一段要重点看

| 行号 | 代码段 | 具体在干什么 |
| --- | --- | --- |
| 748-756 | 启动修复函数开头 | 先判空，保证句柄和失败阶段指针有效 |
| 758-770 | 控制模式修复 | 如果打开 `CONVEYOR_MOTOR_STARTUP_FORCE_CTRL_MODE`，就给电机发一次“恢复到闭环 FOC”命令 |
| 773-785 | 按键锁定修复 | 预留“锁面板按键”的能力，当前默认关闭 |
| 788 行 | 返回 `OK` | 启动修复完成，交回上层继续使能 |

### 7.4 任务初始化这一段要重点看

| 行号 | 代码段 | 具体在干什么 |
| --- | --- | --- |
| 1102-1115 | 创建命令队列 | 如果队列建不出来，任务直接报错并停在原地 |
| 1117 行 | `EMM42_MotorLoadDefaultConfig(&motor, &huart2)` | 把电机驱动绑定到 `USART2` |
| 1119-1152 | 初始化重试循环 | 依次做 `Init -> StartupConfig -> Enable -> StopNow`，任何一步失败都打印阶段名并重试 |
| 1125-1126 | `ConveyorMotorService_ApplyStartupConfig()` | 真正执行“恢复模式” |
| 1131-1132 | `EMM42_MotorSetEnable()` | 让电机进入可响应命令状态 |
| 1141-1142 | `EMM42_MotorStopNow()` | 启动时再补一条停机，确保电机回到静止已知态 |
| 1154-1165 | 初始化运行时状态 | 设定默认模式、方向、误差、稳定计数，并同步快照 |
| 1167 行 | `ConveyorMotorService_ReportReady()` | 把当前启动参数打印出来，方便串口确认 |
| 1169-1180 | 主循环 | 收队列命令 -> 推进一步状态机 -> 更新快照 |

### 7.5 命令处理这一段要重点看

| 行号 | 命令 | 作用 |
| --- | --- | --- |
| 995-1007 | `BELTSTOP` | 让任务进入停止模式 |
| 1010-1023 | `BELTSCAN` | 让任务进入巡航模式 |
| 1025-1040 | `BELTINFO` | 打印当前快照，包括 `desired/applied/error/speed/dir/stable/centered` |
| 1043-1048 | `BELTTRACK <error>` | 直接按误差进入跟踪 |
| 1051-1057 | `BELTENABLE <0|1> [error]` | 更贴近“主机使能位”语义，`0` 切巡航，`1` 切跟踪 |
| 1059-1064 | `BELTCAM <enable> <current_x> <center_x>` | 下位机自己算 `error = current_x - center_x` |

## 8. `User/App/weight_service.c`

### 8.1 为什么电机命令会出现在称重文件里

因为当前工程约定：`USART1` 文本命令只能有一个任务来真正消费。否则多个任务同时从同一串口缓存取命令，就会互相“抢命令”。

### 8.2 你只需要看这一段

| 行号 | 代码段 | 作用 |
| --- | --- | --- |
| 371-474 | `WeightService_ProcessCommand()` | 当前统一串口命令分发中心 |
| 386-388 | `UartCommand_Fetch(...)` | 从 `uart_command.c` 取一条完整命令 |
| 393 行 | `WeightService_NormalizeCommand()` | 把大小写和尾部换行处理掉 |
| 395-455 | `GET/TARE/CAL` | 称重模块自己的命令 |
| 456-459 | `Ldc1614Service_HandleCommand(...)` | 把 LDC 命令转给 LDC 服务 |
| 460-463 | `ConveyorMotorService_HandleCommand(...)` | 把 BELT 命令转给传送带电机服务 |
| 471-472 | 未知命令提示 | 所有模块都没接住时才走这里 |
| 487-550 | `WeightService_Task()` | 称重任务主循环；它一边采样，一边承担“统一命令入口”的角色 |

### 8.3 这一层和电机的关系

| 关系 | 含义 |
| --- | --- |
| `weight_service.c` 不控制电机转速 | 它只负责把 BELT 文本命令转发给电机服务 |
| 电机任务不直接读 `USART1` | 避免和称重任务争用同一帧命令 |
| 这样设计的好处 | 串口入口唯一，任务职责清楚，后面再加模块也不会乱 |

## 9. `User/App/uart_command.c`

### 9.1 文件定位

这个文件是当前整个工程的串口文本命令底座。所有 `GET/TARE/CAL/LDCCAL/BELT...` 最终都先经过它。

### 9.2 逐段说明

| 行号 | 代码段 | 作用 | 关键点 |
| --- | --- | --- | --- |
| 17-24 | 接收/发送缓冲区大小宏 | 定义单帧命令和打印缓存大小 | 当前 BELT 日志也共享这个发送缓存 |
| 31-40 | DMA 缓冲 + 单帧缓存 + 长度 | 保存一帧最新命令 | 当前策略不是环形缓冲，而是“最近一帧覆盖旧帧” |
| 48-49 | 信号量 + 互斥锁 | 接收通知和发送串口同步 | 多任务打印靠这个互斥锁防串行打架 |
| 57-72 | `UartCommand_RestartReceive()` | 重启一次 `DMA + IDLE` 接收 | 每次回调结束后都要重新挂接 |
| 79-90 | `UartCommand_InitSyncObjects()` | 惰性创建同步对象 | 避免重复创建 |
| 98-108 | `UartCommand_StartReceive()` | 启动第一轮接收 | 称重任务启动时会先调用它 |
| 120-153 | `UartCommand_Fetch()` | 取一条完整命令 | 任务上下文里安全拷贝 |
| 163-216 | `my_printf()` | 线程安全格式化输出 | 现在所有模块日志都最好走这里 |
| 232-273 | `HAL_UARTEx_RxEventCallback()` | 空闲中断回调 | 只做“停 DMA -> 拷贝整帧 -> 重启接收 -> 发信号量”四件事 |

## 10. `Core/Src/freertos.c`

### 10.1 文件定位

这个文件只能做 RTOS 启动胶水，不能写业务状态机。电机功能有没有真正活起来，关键就看这里有没有把任务挂进调度器。

### 10.2 本次和电机有关的关键行

| 行号 | 代码段 | 作用 |
| --- | --- | --- |
| 52-60 | `conveyorMotorTaskHandle` + `conveyorMotorTask_attributes` | 定义电机任务句柄和任务属性 |
| 132 行 | `osThreadNew(ConveyorMotorService_Task, NULL, &conveyorMotorTask_attributes)` | 真正创建电机任务 |
| 130-135 | `USER CODE BEGIN RTOS_THREADS` 区域 | 这说明电机任务的挂接放在可重生的用户区，不会被 CubeMX 重新生成冲掉 |

### 10.3 这里为什么重要

如果这里只有 `conveyor_motor_service.c` 文件，但 `freertos.c` 没有创建任务，那么：

1. 电机任务根本不会启动。
2. `USART2` 不会被这个模块真正占用。
3. `BELTSCAN/BELTTRACK` 命令只会显示 “service not ready” 或完全没效果。

## 11. What You Should Check First on Hardware

| 优先级 | 看什么 | 对应源码 |
| --- | --- | --- |
| 1 | 上电后串口是否打印 `ctrl=CLOSED_LOOP_FOC` | `conveyor_motor_service.c:796-814` |
| 2 | 上电后有没有先执行启动修复、使能、停机 | `conveyor_motor_service.c:1119-1152` |
| 3 | FreeRTOS 里电机任务是否真的被创建 | `freertos.c:130-135` |
| 4 | 发 `BELTSCAN` 后是否能巡航 | `conveyor_motor_service.c:1010-1023` + `656-699` |
| 5 | 发 `BELTTRACK 100` 后是否进入跟踪 | `conveyor_motor_service.c:1043-1048` + `701-738` |
| 6 | 发 `BELTINFO` 看 `desired/applied` 是否符合预期 | `conveyor_motor_service.c:1025-1040` |

## 12. Current Startup Behavior

当前这版代码上电后，电机启动阶段的顺序是固定的：

| 顺序 | 代码位置 | 动作 |
| --- | --- | --- |
| 1 | `conveyor_motor_service.c:1117` | 把驱动句柄绑定到 `huart2` |
| 2 | `conveyor_motor_service.c:1122` | 校验驱动句柄 |
| 3 | `conveyor_motor_service.c:1125-1126` | 执行启动修复配置 |
| 4 | `conveyor_motor_service.c:1131-1132` | 发送使能命令 |
| 5 | `conveyor_motor_service.c:1141-1142` | 发送立即停止命令，确保从静止态开始 |
| 6 | `conveyor_motor_service.c:1154-1165` | 初始化软件状态机状态 |
| 7 | `conveyor_motor_service.c:1167` | 打印启动日志 |
| 8 | `conveyor_motor_service.c:1169-1180` | 进入正常主循环 |

## 13. If You Want To Change Behavior Later

| 需求 | 先改哪里 |
| --- | --- |
| 巡航太快/太慢 | `conveyor_motor_service.c:36` 的 `CONVEYOR_MOTOR_SCAN_SPEED_RPM` |
| 小误差跟踪太快 | `conveyor_motor_service.c:45`、`53`、`71` |
| 居中后太容易抖动 | `conveyor_motor_service.c:135`、`144` |
| 方向反了 | `conveyor_motor_service.c:152` |
| 想上电先静止不巡航 | `conveyor_motor_service.c:161` 改成 `0U` |
| 想彻底防止误按面板键 | `conveyor_motor_service.c:104` 改成 `1U`，必要时 `111` 也改成 `1U` |
| 想把启动修复参数写进电机内部存储 | `conveyor_motor_service.c:119` 改成 `1U`，但不要长期每次上电都这么做 |

## 14. Final Conclusion

你可以把当前 Emm42 这套代码理解成三层：

| 层次 | 文件 | 本质职责 |
| --- | --- | --- |
| 串口入口层 | `uart_command.c` + `weight_service.c` | 只负责“拿到完整命令并分发给对的人” |
| 应用状态机层 | `conveyor_motor_service.c/.h` | 只负责“根据命令和误差决定电机现在该怎么动” |
| 协议驱动层 | `emm42_motor.c/.h` | 只负责“把上层意图翻译成 Emm42 协议帧发出去” |

这次新增的“启动强制恢复控制模式”也严格放在这个分层里：

- 协议格式定义在 `emm42_motor.*`
- 启动修复策略放在 `conveyor_motor_service.c`
- 任务真正启动放在 `freertos.c`

这样后面你再查“为什么不转”“为什么方向反了”“为什么串口命令发了没反应”，就知道应该先去哪一层看，不会一上来就在 1000 多行里乱找。

## 15. `Core/Src/usart.c`

### 15.1 这个文件在 Emm42 链路里的作用

这个文件不写业务逻辑，但它决定了 `huart2` 这个句柄是不是一个可用的 Emm42 串口口。  
如果这里没初始化对，`conveyor_motor_service.c` 再正确也发不出有效速度命令。

### 15.2 你最该看的代码段

| 行号 | 代码段 | 作用 | 你应该怎么理解 |
| --- | --- | --- | --- |
| 28 行 | `UART_HandleTypeDef huart2;` | `USART2` 的全局 HAL 句柄 | 电机驱动里最终就是拿这个句柄发 TTL 命令 |
| 63-87 | `MX_USART2_UART_Init()` | 初始化 `USART2` 基本参数 | 当前配置是 `115200 8N1`，与你现在的电机协议一致 |
| 73-80 | `huart2.Init...` | 波特率、数据位、停止位、收发模式 | 这里一旦改错，最直接表现就是“任务在跑，但电机没响应” |
| 81-84 | `HAL_UART_Init(&huart2)` | 真正把参数下发到硬件 | 如果这里失败，会直接进 `Error_Handler()` |
| 141-182 | `HAL_UART_MspInit()` 里的 `USART2` 分支 | GPIO、DMA、中断绑定 | 这里说明了 `PA2=TX`、`PA3=RX`，也是你接线要对照的底层来源 |
| 150-158 | `PA2/PA3` 复用配置 | 把 GPIO 切到 `GPIO_AF7_USART2` | 不做这一步，`PA2/PA3` 只是普通 GPIO，不是串口 |

## 16. `User/App/system_heartbeat_service.h`

### 16.1 文件定位

这个头文件只暴露一个入口：`SystemHeartbeatService_Task()`。  
它的作用不是控制电机，而是让你判断“FreeRTOS 调度器是不是还活着”。

### 16.2 逐段说明

| 行号 | 代码段 | 作用 |
| --- | --- | --- |
| 1-8 | 头文件保护 + `extern "C"` | 保证头文件可以被重复包含且兼容 C++ |
| 10-24 | `SystemHeartbeatService_Task()` 注释 | 把心跳灯任务的职责、优先级定位、用途写清楚 |
| 25 行 | `SystemHeartbeatService_Task()` 声明 | 供 `freertos.c` 创建任务时引用 |

## 17. `User/App/system_heartbeat_service.c`

### 17.1 文件定位

这个源文件实现了最简单也最有用的一条系统监控链路：  
“只要调度器还在正常切任务，`PF9` 就持续按固定周期闪烁”。

### 17.2 逐段说明

| 行号 | 代码段 | 作用 | 关键点 |
| --- | --- | --- | --- |
| 1-5 | 头文件包含 | 引入 FreeRTOS、GPIO 和任务 API | 心跳灯只依赖这几个基础模块 |
| 7-8 | `SYSTEM_HEARTBEAT_LED_GPIO_PORT/PIN` | 固定绑定 `PF9/LED0` | 不在这里重新初始化 GPIO，只负责使用它 |
| 10 行 | `SYSTEM_HEARTBEAT_TOGGLE_PERIOD_MS` | 定义翻转周期 500ms | 完整亮灭周期约 1 秒，肉眼容易判断 |
| 20-40 | `SystemHeartbeatService_Task()` | 心跳灯任务主体 | 周期翻转 LED，并通过 `vTaskDelayUntil()` 保持固定节拍 |
| 23-24 | `last_wake_tick` / `toggle_period_tick` | 周期调度基准 | 这两个量决定了闪烁是不是稳定 |
| 34 行 | `HAL_GPIO_TogglePin(...)` | 真正翻转 PF9 电平 | 如果任务卡死，这一行就不再执行 |
| 37 行 | `vTaskDelayUntil(...)` | 使用绝对节拍延时 | 这比普通 `vTaskDelay()` 更适合做周期心跳 |

## 18. `Core/Src/gpio.c`

### 18.1 这个文件在心跳灯链路里的作用

这个文件是 GPIO 初始化真正的拥有者。  
`system_heartbeat_service.c` 只负责“用”，不负责“重新初始化”。

### 18.2 你最该看的代码段

| 行号 | 代码段 | 作用 | 为什么重要 |
| --- | --- | --- | --- |
| 49-53 | GPIO 时钟使能 | 打开 `GPIOF/H/A/C/B` 时钟 | 没时钟，后面所有 GPIO 初始化都白做 |
| 55 行 | `HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET)` | 设置 PF9 上电默认电平 | 决定 LED 初始亮灭状态 |
| 60-65 | `PF9` 输出模式配置 | 把 PF9 初始化成推挽输出 | 如果不是输出模式，心跳灯任务翻转不会生效 |
| 67-71 | `LDC1614_INTB_Pin` 中断配置 | LDC 模块中断口初始化 | 说明这个文件已经同时承担了多个外设 GPIO 的底层配置 |
| 73-76 | `PB0` 输入配置 | HX711 `DOUT` 输入口初始化 | 也能帮助你理解“为什么 GPIO 配置都该留在 CubeMX 文件里” |

## 19. 现在你可以怎么用这份文档

| 你遇到的问题 | 先看哪一节 |
| --- | --- |
| 上电后电机还是不转 | 3、7.4、10、15 |
| `BELTSCAN` 发了没反应 | 2、7.5、10、15 |
| 想知道启动到底给电机发了什么修复命令 | 4、5、7.3、12 |
| 想查 `USART2` 到底绑在哪两个引脚 | 15 |
| 想确认心跳灯为什么能判断系统是否卡死 | 16、17、18 |
