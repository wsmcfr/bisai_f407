# F4 补光灯舵机服务说明

| 项目 | 内容 |
|---|---|
| 模块位置 | `User/App/fill_light_service.c`、`User/App/fill_light_service.h` |
| 芯片与框架 | STM32F407ZGT6，STM32 HAL + FreeRTOS/CMSIS-RTOS2 |
| 控制引脚 | `PB6 / AF2 / TIM4_CH1` |
| 业务用途 | 模型检测前把 270 度舵机转到开灯位置，检测结束后回到关灯位置 |
| 协议入口 | `FILL_LIGHT_CONTROL 0x23`，完成事件 `EVENT_REPORT event=0x16` |

## 本次创建或修改文件

| 文件 | 类型 | 修改原因 |
|---|---|---|
| `User/App/fill_light_service.h` | 新建 | 集中定义 0~270 度范围、开关角度、500~2500 us 标定范围、2 秒 PWM 保持时间和服务 API。 |
| `User/App/fill_light_service.c` | 新建 | 初始化 PB6/TIM4_CH1、执行角度到脉宽换算、串行处理开关请求并在动作后停止 PWM。 |
| `User/App/fill_light_service_host_test.c` | 新建 | 在 Windows 主机验证 0/135/270 度映射和 300 度上限保护，不依赖 STM32 硬件。 |
| `User/App/binary_protocol_service.c/.h` | 修改 | 定义并分发 `FILL_LIGHT_CONTROL 0x23`，上报 `FILL_LIGHT_MOVE_DONE 0x16` 和补光故障。 |
| `Core/Src/freertos.c` | 修改 | 启动前初始化补光队列和 PWM，并创建独立普通优先级任务，避免阻塞协议收包。 |
| `MDK-ARM/bisai_f407_project.uvprojx` | 修改 | 把 `fill_light_service.c` 加入 Keil `User/App` 编译组。 |

## PWM 舵机原理与角度公式

位置舵机不是靠“输出多少个脉冲”决定相对旋转角度，而是周期性读取高电平脉宽，把脉宽解释为一个绝对目标位置。本模块使用 50 Hz，即每 20 ms 输出一个控制周期；默认 500 us 对应 0 度，2500 us 对应 270 度。

```text
pulse_us = min_pulse_us
         + angle_deg * (max_pulse_us - min_pulse_us) / 270
```

当前参数代入后：

| 目标角度 | 计算结果 | TIM4_CH1 比较值 |
|---:|---:|---:|
| 0° | 500 us | `CCR1=500` |
| 135° | 1500 us | `CCR1=1500` |
| 270° | 2500 us | `CCR1=2500` |

`FillLightService_AngleToPulseUs()` 会把大于 270 度的输入限制到 270 度。开灯和关灯角度分别由 `FILL_LIGHT_SERVO_ON_ANGLE_DEG`、`FILL_LIGHT_SERVO_OFF_ANGLE_DEG` 定义，因此角度可以在头文件中按机械结构修改；首版协议仍只下发绝对开/关动作，不接受任意角度，避免产线流程误写危险行程。

## 硬件资源与接线

| 资源 | 配置 | 说明 |
|---|---|---|
| GPIO | `PB6` | 复用推挽、无上下拉、低速，AF2 映射到 TIM4_CH1。 |
| TIM4 输入时钟 | 50 MHz | APB1 为 25 MHz 且分频不为 1，定时器时钟为 APB1 的 2 倍。 |
| 预分频 | `PSC=49` | 50 MHz / 50 = 1 MHz，因此 1 个计数为 1 us。 |
| 自动重装 | `ARR=19999` | 20000 个计数形成 20 ms 周期，即 50 Hz。 |
| PWM 保持 | 2000 ms | 动作任务输出目标脉宽 2 秒后调用 `HAL_TIM_PWM_Stop()` 并清零 CCR1。 |
| RTOS | 长度 1 的命令队列 + 独立任务 | 协议层只负责非阻塞入队，舵机等待不会阻塞 USART1 收包。 |

接线要求：

1. PB6 只接舵机信号线，不能给舵机供电。
2. 舵机使用能承受启动和堵转电流的外部电源，电压按具体舵机铭牌确定。
3. 舵机电源地必须与 F4 GND 共地。
4. 板级 CAMERA/OLED 接口把 PB6 同时标为 `DCMI_D5`；使用舵机时不得再把该 DCMI 数据线接入。
5. 断开机械负载做首次标定，确认 500~2500 us 不会撞限位后再连接补光开关机构。

## 运行流程

1. `MX_FREERTOS_Init()` 调用 `FillLightService_Init()`，创建队列并初始化 PB6/TIM4_CH1，但不启动 PWM。
2. MP157 发送 `FILL_LIGHT_CONTROL action=1`，协议层校验活动 `cycle_id` 后调用 `FillLightService_Request()`。
3. 补光任务把 action=1 映射为 270 度和 2500 us，输出 50 Hz PWM 2 秒。
4. 任务停止 PWM、清零 CCR1，再发送 `EVENT_REPORT event=0x16 action=1 angle=270`。
5. MP157 收到严格匹配的完成事件后等待 5 秒，再运行模型检测。
6. 检测完成后 MP157 发送 `action=0`；F4 输出 500 us 2 秒让舵机回 0 度，停止 PWM并上报 `event=0x16 action=0 angle=0`。

## 编译、下载与运行

1. 用 Keil MDK 打开 `E:\hal\bisai_f407_project\MDK-ARM\bisai_f407_project.uvprojx`。
2. 确认 `User/App` 组包含 `fill_light_service.c`，然后执行 Build/Rebuild。
3. 编译成功后由用户连接 F4 调试器下载固件并复位。
4. 先不连接机械负载，用示波器验证 PB6；波形正确后再连接舵机信号线和外部电源。

代码已写入工程不等于已经完成 Keil 编译、下载或上板生效。本模块不由 Codex 代编译和烧录，必须由用户完成并反馈编译错误、串口日志或示波器结果。

## 验证方式

| 测试目标 | 执行位置 | 命令/操作 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| 角度公式主机测试 | `E:\hal\bisai_f407_project` PowerShell | `gcc -std=c99 -Wall -Wextra -Werror -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\fill_light_service_host_test.c User\App\fill_light_service.c -o tmp\fill_light_service_host_test.exe; .\tmp\fill_light_service_host_test.exe` | 输出 `fill light service host tests passed`。 | 查 GCC 是否安装、包含路径、角度宏和 32 位中间乘法。 |
| 协议解码主机测试 | 同上 | `gcc -std=c99 -Wall -Wextra -Werror -DBINARY_PROTOCOL_HOST_TEST -I User\App User\App\binary_protocol_service_host_test.c User\App\binary_protocol_service.c -o tmp\binary_protocol_service_host_test.exe; .\tmp\binary_protocol_service_host_test.exe` | 输出 `binary protocol host tests passed`。 | 查 `0x23`、4 字节负载、action/flags 范围和 CRC。 |
| Keil 工程编译 | Keil MDK | 打开工程后执行 Rebuild | 用户本机应得到 `0 Error(s)`；warning 需逐条确认。 | 查 `fill_light_service.c` 是否入组、HAL TIM 头文件、FreeRTOS queue API 和链接内存。 |
| PB6 周期 | F4 板 + 示波器 | 下载后发送一次开灯命令，示波器探头接 PB6/GND | 周期约 20 ms，频率约 50 Hz。 | 查 APB1 定时器时钟是否仍为 50 MHz、PSC/ARR、PB6 AF2 和共地。 |
| 开灯脉宽 | F4 板 + 示波器 | 发送 action=1 | 高电平约 2500 us，持续约 2 秒后 PB6 停止 PWM；F4 回 `event=0x16 angle=270`。 | 查 `ON_ANGLE`、最大脉宽、PWM Start/Stop 和补光任务是否运行。 |
| 关灯脉宽 | F4 板 + 示波器 | 模型检测完成后发送 action=0 | 高电平约 500 us，持续约 2 秒后停止；F4 回 `event=0x16 angle=0`。 | 查 MP157 是否在 STOP_CYCLE 清 cycle 前发关灯、F4 active cycle 是否匹配。 |
| 实物开关 | F4 板 + 外部舵机电源 | 低风险行程下逐步标定后运行一轮自动检测 | 先开灯、稳定 5 秒、模型检测、再关灯；舵机不堵转、不撞限位。 | 立即断开舵机电源，缩小脉宽或开关角度，检查机构、供电能力和共地。 |

## 修改记录

| 日期 | 修改内容 | 原因 |
|---|---|---|
| 2026-07-25 | 新增 PB6/TIM4_CH1 270 度舵机服务、角度公式、2 秒停 PWM、完成事件和主机测试 | 在模型检测前机械打开补光灯，检测后可靠关闭 |
