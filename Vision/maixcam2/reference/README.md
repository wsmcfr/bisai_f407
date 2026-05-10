# reference 目录说明

这个目录只放“后续做视觉抓取联动时会直接用到的基础参考代码副本”，目的不是立即参与编译，而是把分散在当前工程和 `D:\机械臂` 资料目录里的关键文件收口到一个地方。

## 1. 目录来源

| 子目录 | 来源 | 用途 |
| --- | --- | --- |
| `stm32_bridge/` | 当前仓库 `User/App/robot_arm_service.*` | 参考 STM32F407 现有的机械臂桥接协议和日志策略 |
| `docs/` | 当前仓库 `docs/robot-arm-vision-vacuum-wiring.md` | 参考已经确认的接线、角色分工和后续固件边界 |
| `esp32_factory_base/LeArm_ESP32_Arduino/` | `D:\机械臂\6.6 串口通信实操课程\03 LeArm从机端程序\LeArm_ESP32_Arduino\` | 作为 ESP32 机械臂原厂程序的当前 Arduino sketch 副本 |

## 2. 当前复制的 ESP32 基础文件

当前你后续要继续改、并直接在 Arduino IDE 里打开的 ESP32 程序副本，就在：

`Vision/maixcam2/reference/esp32_factory_base/LeArm_ESP32_Arduino/`

| 路径 | 为什么复制它 |
| --- | --- |
| `LeArm_ESP32_Arduino.ino` | 查看原厂 `setup/loop` 和串口初始化入口 |
| `Config.h` | 查看 J2、I2C、PWM 舵机口等 GPIO 定义 |
| `IIC.cpp` / `IIC.hpp` | 参考原厂 I2C 读写封装，后续给 MaixCAM2 视觉结果通信用 |
| `vision_i2c_link.cpp` / `vision_i2c_link.hpp` | 当前新增的视觉 I2C 从机模块，负责接收 MaixCAM2 结果帧并缓存给后续抓取状态机 |
| `Hiwonder.hpp` / `Robot_arm.hpp` / `LeArm_Kinematics.hpp` | 保留原厂全局声明、机械臂对象和运动学入口定义 |
| `src/PC_BLE/PC_BLE_CTL.cpp` / `PC_BLE_CTL.hpp` | 参考原厂上位机串口协议入口，后续要在这里接 F4/视觉相关命令 |
| `src/robot_arm/Robot_arm.cpp` | 参考机械臂动作执行主逻辑 |
| `src/robot_arm/SerialServo.cpp` | 参考总线舵机控制细节 |
| `src/robot_arm/Flash_ctl.cpp` | 参考动作组读写与保存 |
| `src/robot_arm/Pwmservo.cpp` | 参考 PWM 舵机/电子开关控制基础 |
| `src/robot_arm/Servo.cpp` / `Servo.h` | 参考舵机底层对象封装 |
| `src/robot_arm/kinematics.cpp` / `kinematics.hpp` | 参考末端坐标和关节角换算 |

## 3. 当前已加的视觉联调指示

为了方便你接线后先确认 MaixCAM2 到 ESP32 的 I2C 链路是否打通，当前已经在
`esp32_factory_base/LeArm_ESP32_Arduino/LeArm_ESP32_Arduino.ino` 里加了最小 LED 联调逻辑：

| 条件 | LED 行为 |
| --- | --- |
| 收到新视觉帧，但 `found=0` | 快闪一下，表示链路在线 |
| 最近视觉帧仍有效，且 `found=1` | 常亮，表示当前已经识别到目标 |
| 超过超时窗口没有新帧 | 熄灭，表示链路断开或视觉侧已停发 |

## 4. 使用边界

1. 这些文件当前只是“副本参考”，不会被当前 STM32F407 工程编译。
2. 真正开始改 ESP32 抓取状态机时，优先在这个目录内先分析和做方案，再决定同步回原工程或独立导出。
3. 如果后续继续从 `D:\机械臂` 补文件，只补和视觉抓取直接相关的最小集合，不要把整个原厂工程整包复制进来。

## 5. 当前 I2C 对接边界

| 项目 | 当前约定 |
| --- | --- |
| 主从角色 | MaixCAM2 做 I2C 主机，ESP32 做 I2C 从机 |
| 地址 | `0x42` |
| MaixCAM2 引脚 | 当前主程序试用 `A1 -> I2C6_SCL`，`A0 -> I2C6_SDA`；`i2c_probe.py` 仍保留 `A8/A9 + I2C7` 对照测试 |
| ESP32 引脚 | `GPIO16 -> SCL`，`GPIO17 -> SDA` |
| 数据格式 | 固定 16 字节视觉结果帧，含帧头、版本、`frame_id`、`dx/dy`、`score`、`area` 和 checksum |
| 当前动作范围 | 只接收、校验、缓存视觉结果，不在这一层直接驱动机械臂运动 |
