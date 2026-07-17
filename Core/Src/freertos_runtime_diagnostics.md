# FreeRTOS 运行资源诊断

## 修改说明

| 文件 | 修改原因 | 影响 |
|---|---|---|
| `Core/Inc/FreeRTOSConfig.h` | 开启 `configCHECK_FOR_STACK_OVERFLOW=2` 和 `configUSE_MALLOC_FAILED_HOOK=1`。 | 真正的栈越界或动态堆耗尽不再静默。 |
| `Core/Src/freertos.c` | 实现两个故障钩子，通过非 MP157 主链路 USART2 输出固定致命标记并安全停机。 | 可明确区分资源故障与协议/队列逻辑故障，且不会污染 USART1 二进制主链路。 |
| `User/App/conveyor_motor_service.c` | STOP 时打印传送带任务栈水位和堆水位。 | 现场可观察 `stack_hw/heap/heap_min`。 |
| `User/App/camera_motor_service.c` | STOP 时打印摄像头电机任务栈水位和堆水位。 | 同时记录左右轴、Z 轴底层停止结果。 |

## 硬件资源

| 资源 | 参数 | 用途 |
|---|---|---|
| USART1 TX/RX | `PA9/PA10`，115200 8N1 | MP157-F4 正式二进制主链路，不能混入 `[FATAL]` 文本。 |
| USART2 TX | `PA2`，115200 8N1 | 输出 FreeRTOS `[FATAL]` 致命故障标记，接 USB-TTL RX。 |
| USART2 RX | `PA3` | 当前不需要输入。 |
| GND | F4 与 USB-TTL 共地 | 不共地会导致日志乱码或无输出。 |

## 验证方式

| 测试目标 | 执行位置 | 命令 | 预期输出/现象 | 失败时排查 |
|---|---|---|---|---|
| STOP 资源水位 | USART1 串口助手 | 115200 8N1 打开串口后触发自动流程 STOP | 出现 `[STOP][BELT]` 和 `[STOP][CAM]`，其中 `status=0`、`stack_hw>0`、`heap>0`、`heap_min>0`。 | 无日志先查 PA9 接线；status 非 0 查电机 UART；水位接近 0 再调整资源。 |
| 栈溢出钩子 | USART2 串口助手 | 正常运行无需主动制造 | 正常情况下永不出现 `[FATAL] F4 STACK OVERFLOW`。 | 若出现，结合最近一条 STOP 的 `stack_hw` 找到对应任务并增加栈；若只监听 USART1，将不会看到该文本，因为 USART1 是 MP157 主链路。 |
| 堆耗尽钩子 | USART2 串口助手 | 正常运行无需主动制造 | 正常情况下永不出现 `[FATAL] F4 MALLOC FAILED`。 | 若出现，检查任务、队列、互斥锁和信号量创建数量及 `configTOTAL_HEAP_SIZE`；若只监听 USART1，将不会看到该文本。 |

## 修改记录

| 日期 | 修改 |
|---|---|
| 2026-07-10 | 新增运行时栈溢出、堆耗尽钩子和 STOP 资源水位日志。 |
| 2026-07-17 | MP157-F4 主链路改回 USART1 后，FreeRTOS 致命故障钩子的直发文本改到 USART2，避免绕过 `my_printf()` 静默保护污染二进制协议。 |
