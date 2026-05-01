<!-- TRELLIS:START -->
# Trellis Instructions

These instructions are for AI assistants working in this project.

Use the `/trellis:start` command when starting a new session to:
- Initialize your developer identity
- Understand current project context
- Read relevant guidelines

Use `@/.trellis/` to learn:
- Development workflow (`workflow.md`)
- Project structure guidelines (`spec/`)
- Developer workspace (`workspace/`)

If you're using Codex, project-scoped helpers may also live in:
- `.agents/skills/` for reusable Trellis skills
- `.codex/agents/` for optional custom subagents

Keep this managed block so 'trellis update' can refresh the instructions.

<!-- TRELLIS:END -->

# 项目补充提示词

## 外部参考资料

- `D:\机械臂` 存放六自由度总线舵机机械臂相关资料，包括出厂程序、二维运动学源码、动作组文件、上位机资料和串口通信课程。
- 该机械臂系统以 ESP32 为主控；涉及机械臂代码、引脚、串口、PWM、总线舵机协议或运动控制时，应按 ESP32 平台上下文分析，不要默认套用当前 STM32F407 工程的 HAL 外设配置。
- 若任务是让当前 STM32F407 工程与机械臂联动，应优先把问题建模为 STM32F407 与 ESP32/总线舵机控制器之间的通信协议、供电隔离、串口电平和任务协同问题。
- 当任务涉及机械臂、总线舵机、动作组控制、串口舵机协议、运动学、舵机上位机或机械臂联调时，应优先检查并参考该目录下的资料。
- 参考该目录资料时，只提取与当前任务直接相关的文件，避免无目的地批量读取大目录。
- 若资料内容与当前工程代码或硬件实测现象冲突，应以当前工程代码、实际接线和可复现实测结果为准，并在回复中说明差异。

## 代码注释要求

- 以下注释规则主要适用于 `User/` 目录下的手写应用层和驱动层代码。
- `Core/Src/freertos.c` 是例外：该文件虽然由 CubeMX 生成，但本项目会在其中维护 FreeRTOS 任务创建、任务属性和任务入口转接逻辑，因此允许并要求按本规则补充中文注释。
- 除 `Core/Src/freertos.c` 外，`Core/`、`Drivers/`、`Middlewares/` 等 CubeMX 或第三方生成代码不做大规模注释改造；只有在 `USER CODE BEGIN/END` 区域内新增手写逻辑，或必须维护手写补丁时，才按本规则补注释。
- 新增或修改 `User/` 目录下的结构体时，结构体内每一个成员变量后面都必须追加详细中文注释，说明该成员的用途、单位或取值范围、所有权关系、生命周期以及特殊边界含义。
- 结构体成员注释必须紧跟在成员声明后面，优先使用行尾注释；当说明较长时，可在成员声明上一行补充中文块注释，但仍要保证每个成员都有一一对应的说明。
- 新增或修改 `User/` 目录下的函数时，函数定义前必须写中文注释，说明该函数的作用、主要执行流程、关键参数含义、返回值含义以及可能产生的副作用。
- 函数内部也必须补充中文注释，说明关键变量、关键逻辑分支、状态切换、资源申请与释放、错误处理、并发或中断上下文相关逻辑分别是干什么的，以及为什么这样处理。
- 对 HAL 回调、中断处理、FreeRTOS 任务、DMA/UART/GPIO 等硬件相关逻辑，必须额外说明触发来源、执行上下文、与共享状态的关系，以及需要避免的阻塞或竞态风险。
- 注释必须服务于理解代码，避免只重复代码表面含义；当代码意图、边界条件或硬件时序不直观时，必须写出设计原因。
