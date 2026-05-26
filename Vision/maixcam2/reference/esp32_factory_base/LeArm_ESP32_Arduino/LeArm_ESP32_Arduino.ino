#include <string.h>

#include "Config.h"
#include "Hiwonder.hpp"
#include "Robot_arm.hpp"
#include "vision_i2c_link.hpp"
#include "./src/PC_BLE/PC_BLE_CTL.hpp"

Led_t led_obj;
Buzzer_t buzzer_obj;
LeArm_t arm;
PC_BLE_CTL pc_ble_obj;
VisionI2CLink vision_i2c_obj;
VisionI2CFrame_t latest_vision_frame;

/*
 * 视觉 I2C 联调阶段的串口选择。
 *
 * 设为 1：`Serial` 走 ESP32 默认 USB 下载串口，Arduino IDE 串口监视器能直接看到
 *         `[BOOT]` 和 `[VISION_I2C]` 日志，便于现在排查 MaixCAM2 扫不到 0x42 的问题。
 * 设为 0：`Serial` 重映射到机械臂 J2 的 PA5/PA4，恢复原厂 PC/蓝牙控制通道；
 *         这种模式下 Arduino IDE 通常只能看到 ROM 启动日志，看不到应用层日志。
 */
#define VISION_DEBUG_USB_SERIAL 1

/*
 * I2C 物理线序测试开关。
 *
 * 默认 0：正常运行 ESP32 I2C 从机程序。
 * 改成 1：不启动机械臂和 I2C 从机，只轮流把 SCL(GPIO16)、SDA(GPIO17) 拉低。
 *
 * 使用方法：
 * 1. 把这里改为 1 并重新烧录 ESP32；
 * 2. MaixCAM2 运行 `i2c_probe.py`；
 * 3. 当 ESP32 打印 `pull SCL low` 时，Maix 的 I2C6 诊断应看到 `scl=0 sda=1`；
 * 4. 当 ESP32 打印 `pull SDA low` 时，Maix 的 I2C6 诊断应看到 `scl=1 sda=0`。
 *
 * 如果 Maix 看到的不是这个对应关系，就说明 SCL/SDA 线序、插头方向或接线网络不对。
 */
#define VISION_I2C_LINE_TEST 0

/*
 * ESP32 I2C 最小从机测试开关。
 *
 * 默认 0：启动完整机械臂程序。
 * 改成 1：只启动 USB 串口、蜂鸣器、LED 和 I2C 从机，不初始化机械臂、PC/蓝牙协议。
 *
 * 使用场景：
 * - 线序测试已经证明 A1/A0 与 GPIO16/GPIO17 连通；
 * - 但 MaixCAM2 仍然 `write ERR -14`；
 * - 需要排除机械臂初始化、串口协议任务等其它模块对 I2C 从机的影响。
 */
#define VISION_I2C_MINIMAL_SLAVE_TEST 0

/*
 * 视觉联调阶段的 LED 指示策略：
 * 1. 收到“新帧但未识别到目标”时，LED 短闪一下，证明 I2C 链路仍在刷新；
 * 2. 最近视觉帧仍在有效期内且 `found=1` 时，LED 常亮，表示当前已经锁定目标；
 * 3. 若超过视觉超时时间没有新帧，则 LED 熄灭，表示链路掉线或视觉侧已停发。
 *
 * 这里故意不在 I2C 回调里直接控制 LED，而是在 `loop()` 中基于缓存结果更新，
 * 这样可以避免回调上下文里做外设切换，减少后续接抓取状态机时的竞态风险。
 */
static uint32_t g_vision_link_pulse_deadline_ms = 0U;
static const uint32_t VISION_LINK_PULSE_WIDTH_MS = 60U;
static const uint16_t VISION_I2C_BUZZER_OK_FREQ_HZ = 1500U;
static const uint16_t VISION_I2C_BUZZER_FAIL_FREQ_HZ = 800U;
static const uint16_t VISION_I2C_BUZZER_OK_ON_MS = 80U;
static const uint16_t VISION_I2C_BUZZER_OK_OFF_MS = 80U;
static const uint16_t VISION_I2C_BUZZER_FAIL_ON_MS = 600U;
static const uint16_t VISION_I2C_BUZZER_FAIL_OFF_MS = 100U;
static const uint32_t VISION_I2C_LINE_TEST_STEP_MS = 3000U;
static const uint32_t VISION_I2C_DEBUG_REPORT_INTERVAL_MS = 1000U;
static uint32_t g_vision_i2c_last_debug_report_ms = 0U;
static VisionI2CDebugCounters_t g_vision_i2c_last_debug_counters;

/*
 * 视觉驱动机械臂的最小跟踪开关。
 *
 * 当前先只做“识别到目标后机械臂按像素偏差小步调整”，不自动下降抓取。
 * 原因是你现在还在手持 MaixCAM2，摄像头坐标和机械臂末端坐标没有刚性标定，
 * 直接做抓取容易让机械臂朝错误方向运动。先让底座/大臂按 `dx/dy` 动起来，
 * 后续再根据实际安装方式调整方向、比例和抓取状态机。
 */
#define VISION_ARM_TRACK_ENABLE 1

/*
 * 视觉跟踪参数。
 *
 * - 死区：目标靠近屏幕中心时不动，避免色块抖动导致舵机来回震荡；
 * - 方向：按上一版”原来的方向”恢复，X 轴横向修正使用 -1，Y 轴高低修正使用 +1；
 * - 远近：MaixCAM2 只给 2D 色块，没有深度传感器；理论上可以用目标面积估算远近，
 *   但当前摄像头是手持的，手抖造成的面积变化远大于真实距离变化，
 *   所以默认通过 `VISION_ARM_USE_AREA_FOR_REACH = 0` 关闭这条控制路径，
 *   以后如果摄像头改为机械臂上固定安装，再把它打开做精细调；
 * - 横向：`dx_px` 直接控制 6 号底座舵机，避免 `coordinate_set(y)` 逆解失败后目标很偏也不动；
 * - 坐标：只把高度交给 `coordinate_set()`，降低前端几个舵机反复重算造成的抖动；
 * - 比例：偏差越大，单次坐标修正越大，解决目标从画面左边移动到右边时机械臂仍只动一点的问题；
 * - 抑振：连续多帧偏差都落入死区时，强制不动，避免视觉端 ±2 像素噪声透到舵机层。
 */
/*
 * 远近控制开关。
 *
 * 项目最终目标是把摄像头装在机械臂末端做 eye-in-hand 跟踪，那时面积↔距离是
 * 物理刚性关系，必须打开；当前手持过渡阶段只是把死区加大、增益放慢，避免
 * 手抖造成的面积变化让 X 方向不停喘息。
 */
#define VISION_ARM_USE_AREA_FOR_REACH 1
static const uint8_t VISION_ARM_MIN_SCORE = 20U;
static const uint16_t VISION_ARM_MIN_AREA = 220U;
/*
 * 目标面积。
 *
 * 1800 是按"摄像头装在机械臂末端、能贴近物体"假设设的；但当前手持调试期，
 * 物体面积通常只有 600~800，TARGET_AREA=1800 会让 reach 控制一直想"再靠近一点"，
 * 把 x 一路推到 X_MAX_CM 上限再也下不来。
 *
 * 降到 700 后，手持期实际面积就在期望值附近，reach 不会一上来就饱和；
 * 装到机械臂末端后，根据现场抓取距离再调回到 1500~2500 即可。
 */
static const uint16_t VISION_ARM_TARGET_AREA = 700U;
/*
 * 面积死区。
 *
 * 配合 TARGET_AREA=700，死区 ±400 把手抖造成的 ±100 像素面积波动屏蔽住，
 * 又让用户主动把物体显著拉近或推远（±400+）能触发 x 移动。
 */
static const uint16_t VISION_ARM_AREA_DEADBAND = 400U;
/*
 * 横向死区保持 10，纵向死区收到 8。日志里 dy=22 才走 0.32cm，dy 死区不再是瓶颈，
 * 真正的瓶颈是 HEIGHT_GAIN 太小，因此重点把 Y 增益提到原来的 ~3 倍。
 */
static const int16_t VISION_ARM_X_DEADBAND_PX = 10;
static const int16_t VISION_ARM_Y_DEADBAND_PX = 8;
static const uint8_t VISION_ARM_IDLE_HOLD_FRAMES = 3U;
static const uint32_t VISION_ARM_FRAME_REPLAY_MIN_MS = 50U;
static const int VISION_ARM_BASE_SIGN = -1;
/*
 * 高度方向：之前写成 +1 实际是反的。
 *
 * MaixCAM2 协议里 dy 定义为"下正上负"。在标准 eye-in-hand（摄像头装机械臂末端、
 * 朝前/略下视）的姿态下，目标在画面上方 (dy<0) 应该让末端**抬高**(z 增大)。
 *
 * 现场日志验证：dy=-50 持续时 delta_z=-1.5，z 一路撞下限 1.0；改成 -1 后
 * delta_z=+1.5，z 才会真正往上走。
 */
static const int VISION_ARM_HEIGHT_SIGN = -1;
/*
 * 底座 duty 范围扩大到硬件实际允许的 125~875（来自 Config.h 的 MIN_DUTY/MAX_DUTY）。
 * 之前 180~820 的限制太严，日志里 duty=180 撞死下限就是这个原因，导致 dx>0
 * 时 base 完全不动；放宽到 125~875 后舵机仍在硬件安全范围内，但跟踪不会再被卡死。
 */
static const int VISION_ARM_BASE_MIN_DUTY = 125;
static const int VISION_ARM_BASE_MAX_DUTY = 875;
/*
 * 横向"步进 + 增益"减半，根除"轻轻动一下转一大圈"。
 * 之前 step=8+dx_eff*0.6, MAX=70，35ms 节流下相当于 30°/s 起步、100°/s 峰值；
 * 现在 step=4+dx_eff*0.30, MAX=35，峰值降到 ~50°/s，对手持调试更跟手。
 */
static const int VISION_ARM_BASE_MIN_STEP_DUTY = 4;
static const int VISION_ARM_BASE_MAX_STEP_DUTY = 35;
static const float VISION_ARM_BASE_GAIN_DUTY_PER_PX = 0.30f;
/*
 * 远近控制：单步从 1.5cm 收到 0.8cm。
 *
 * 之前 1.5cm 一步配合 LeArm 5-DOF IK 在 (x≈10, z≈2, pitch=-30°) 这种位姿组合下
 * 解空间已经很窄，单步过大会让 next_xy 直接落到 IK 不可达点，导致 coordinate_set
 * 持续返回 0、xyz 卡死不动；改成 0.8cm 后，绝大多数中间位姿仍可达，逐步逼近目标。
 */
static const float VISION_ARM_REACH_GAIN_CM_PER_AREA = 0.0030f;
static const float VISION_ARM_REACH_MIN_STEP_CM = 0.40f;
static const float VISION_ARM_REACH_MAX_STEP_CM = 0.80f;
/*
 * 高度增益 0.012→0.040（约 3.3 倍）、最大步长 0.9→1.5cm。
 * 日志里 dy=22 时只走 0.32cm 明显不够；提到 0.040 后，dy=22 一次走 ~0.80cm，
 * 配合 100ms 节流大约 8cm/s，肉眼能立刻感受到机械臂跟随上下。
 */
static const float VISION_ARM_HEIGHT_GAIN_CM_PER_PX = 0.040f;
static const float VISION_ARM_HEIGHT_MIN_STEP_CM = 0.30f;
static const float VISION_ARM_HEIGHT_MAX_STEP_CM = 1.5f;
/*
 * 末端坐标安全范围。
 *
 * 之前 Z_MAX=8.0、Z_MIN=1.0 把 z 限制在 7cm 范围内，导致用户感觉"上下幅度太小"，
 * 而且日志里 z 一直撞 1.0 下限完全没法响应。
 *
 * LeArm 6 自由度机械臂实际可达 z 大约 0~20cm（取决于 x、pitch 组合），把范围
 * 扩到 0.5~14.0 后，上下幅度提升到 13.5cm，肉眼能立刻看到机械臂跟随上下；
 * X_MAX 收到 22.0 给关节留点余量，避免 IK 频繁逼近末端可达边界后再钳回来。
 */
static const float VISION_ARM_X_MIN_CM = 8.0f;
static const float VISION_ARM_X_MAX_CM = 22.0f;
static const float VISION_ARM_Z_MIN_CM = 0.5f;
static const float VISION_ARM_Z_MAX_CM = 14.0f;
static const float VISION_ARM_PITCH_DEG = -30.0f;
static const uint32_t VISION_ARM_BASE_INTERVAL_MS = 50U;
static const uint32_t VISION_ARM_BASE_MOVE_TIME_MS = 80U;
static const uint32_t VISION_ARM_POSE_INTERVAL_MS = 100U;
static const uint32_t VISION_ARM_POSE_MOVE_TIME_MS = 130U;
static const uint32_t VISION_ARM_DEBUG_REPORT_INTERVAL_MS = 250U;
static int g_vision_arm_base_duty = SERVO6_RESET_DUTY;
static float g_vision_arm_pose_x_cm = DEFAULT_X;
static float g_vision_arm_pose_z_cm = DEFAULT_Z;
static uint32_t g_vision_arm_last_base_move_ms = 0U;
static uint32_t g_vision_arm_last_pose_move_ms = 0U;
static uint32_t g_vision_arm_last_report_ms = 0U;
static uint32_t g_vision_arm_last_consume_ms = 0U;
static uint16_t g_vision_arm_last_frame_id = 0xFFFFU;
static uint8_t g_vision_arm_idle_streak = 0U;

/**
 * @brief 轮流拉低 ESP32 I2C SCL/SDA，用于验证 MaixCAM2 实际接到了哪根线。
 * @retval None
 *
 * 主要流程：
 * 1. 先把两根线释放为输入上拉，让总线回到空闲高电平；
 * 2. 拉低 SCL(GPIO16) 一段时间，Maix 侧应读到 SCL=0、SDA=1；
 * 3. 释放 SCL 后拉低 SDA(GPIO17) 一段时间，Maix 侧应读到 SCL=1、SDA=0；
 * 4. 周期重复，直到断电或重新烧录正常程序。
 *
 * 副作用：
 * - 该模式不会启动 I2C 从机，也不会初始化机械臂；
 * - 只用于物理线序排查，测试完成后必须把 `VISION_I2C_LINE_TEST` 改回 0。
 */
static void RunVisionI2CLineTest(void)
{
  Serial.println("[LINE_TEST] start, release both I2C lines");

  while (true)
  {
    pinMode(IIC_SCL, INPUT_PULLUP);
    pinMode(IIC_SDA, INPUT_PULLUP);
    Serial.println("[LINE_TEST] release SCL/SDA");
    delay(VISION_I2C_LINE_TEST_STEP_MS);

    pinMode(IIC_SDA, INPUT_PULLUP);
    pinMode(IIC_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(IIC_SCL, LOW);
    Serial.println("[LINE_TEST] pull SCL(GPIO16) low");
    delay(VISION_I2C_LINE_TEST_STEP_MS);

    pinMode(IIC_SCL, INPUT_PULLUP);
    pinMode(IIC_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(IIC_SDA, LOW);
    Serial.println("[LINE_TEST] pull SDA(GPIO17) low");
    delay(VISION_I2C_LINE_TEST_STEP_MS);
  }
}

/**
 * @brief 用蜂鸣器播报 I2C 从机初始化结果。
 * @param init_ok 1 表示 `Wire.begin()` 成功，0 表示失败。
 * @retval None
 *
 * 提示规则：
 * 1. 初始化成功时短响两次，方便你只靠耳朵就能确认 ESP32 从机已经就绪；
 * 2. 初始化失败时长响一次，提示当前应优先查电源、接线或 I2C 总线占用。
 */
static void PlayVisionI2CInitBuzzerFeedback(uint8_t init_ok)
{
  if (init_ok != 0U)
  {
    buzzer_obj.blink(VISION_I2C_BUZZER_OK_FREQ_HZ,
                     VISION_I2C_BUZZER_OK_ON_MS,
                     VISION_I2C_BUZZER_OK_OFF_MS,
                     2U);
    return;
  }

  buzzer_obj.blink(VISION_I2C_BUZZER_FAIL_FREQ_HZ,
                   VISION_I2C_BUZZER_FAIL_ON_MS,
                   VISION_I2C_BUZZER_FAIL_OFF_MS,
                   1U);
}

/**
 * @brief 根据最近视觉结果更新板载 LED 联调指示。
 * @param frame 最近一次 `FetchLatest()` 取到的视觉缓存，不能为空。
 * @retval None
 *
 * 主要流程：
 * 1. 如果本轮刚收到新帧但 `found=0`，给 LED 一个短脉冲，表示链路在线；
 * 2. 如果最近帧仍在有效期内且 `found=1`，保持 LED 常亮；
 * 3. 如果视觉结果超时，则熄灭 LED，避免把旧状态误看成实时识别成功。
 */
static void UpdateVisionDebugLed(const VisionI2CFrame_t *frame)
{
  uint32_t now_ms = millis();
  uint8_t has_fresh_frame = 0U;

  if (frame == NULL)
  {
    led_obj.on_off(0);
    return;
  }

  has_fresh_frame = vision_i2c_obj.HasFreshFrame();

  if ((frame->updated != 0U) && (frame->found == 0U))
  {
    g_vision_link_pulse_deadline_ms = now_ms + VISION_LINK_PULSE_WIDTH_MS;
  }

  if ((has_fresh_frame != 0U) && (frame->found != 0U))
  {
    led_obj.on_off(1);
    return;
  }

  if ((has_fresh_frame != 0U) && (g_vision_link_pulse_deadline_ms > now_ms))
  {
    led_obj.on_off(1);
    return;
  }

  led_obj.on_off(0);
}

/**
 * @brief 周期打印 ESP32 I2C 从机诊断计数。
 * @retval None
 *
 * 主要流程：
 * 1. 每 1 秒读取一次 `VisionI2CLink` 内部计数；
 * 2. 只有计数变化或周期到达时才打印一行稳定格式日志；
 * 3. 通过 `REQ/RX` 判断 MaixCAM2 的读写事务是否真正进入 ESP32。
 *
 * 诊断含义：
 * - `REQ` 增加：Maix 的 `readfrom(0x42, n)` 触发了 ESP32 读请求回调；
 * - `RX` 增加：Maix 的 `writeto(0x42, frame)` 触发了 ESP32 写接收回调；
 * - `REQ/RX` 都一直为 0：地址阶段仍没有被 ESP32 硬件 ACK，优先查电气层或从机控制器。
 */
static void ReportVisionI2CDebugCounters(void)
{
  VisionI2CDebugCounters_t counters;
  uint32_t now_ms = millis();
  uint8_t counters_changed = 0U;

  if ((uint32_t)(now_ms - g_vision_i2c_last_debug_report_ms) < VISION_I2C_DEBUG_REPORT_INTERVAL_MS)
  {
    return;
  }
  g_vision_i2c_last_debug_report_ms = now_ms;

  vision_i2c_obj.FetchDebugCounters(&counters);
  counters_changed =
      (memcmp(&counters, &g_vision_i2c_last_debug_counters, sizeof(counters)) != 0) ? 1U : 0U;
  g_vision_i2c_last_debug_counters = counters;

  Serial.printf("[VISION_I2C_DBG] ready=%u changed=%u TXPRE=%lu RX=%lu OK=%lu BAD=%lu last_recv=%u last_rx=%u last_tx_ms=%lu last_rx_ms=%lu\r\n",
                vision_i2c_obj.IsReady(),
                counters_changed,
                (unsigned long)counters.request_count,
                (unsigned long)counters.receive_count,
                (unsigned long)counters.valid_frame_count,
                (unsigned long)counters.bad_frame_count,
                counters.last_recv_len,
                counters.last_rx_len,
                (unsigned long)counters.last_req_ms,
                (unsigned long)counters.last_rx_ms);
}

/**
 * @brief 将浮点坐标限制到给定闭区间，避免视觉控制目标超出机械臂可达安全范围。
 * @param value 待限制的原始坐标，单位通常为 cm。
 * @param min_value 允许输出的最小坐标。
 * @param max_value 允许输出的最大坐标。
 * @return float 被限制后的坐标，始终满足 `min_value <= return <= max_value`。
 *
 * 主要流程：
 * 1. 先比较下限，小于下限时直接返回下限；
 * 2. 再比较上限，大于上限时直接返回上限；
 * 3. 中间范围保持原值。
 *
 * 副作用：无。该函数只做纯计算，供视觉跟踪末端坐标限幅复用。
 */
static float ClampVisionArmCoord(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/**
 * @brief 将底座舵机目标脉宽限制到安全范围。
 * @param value 待限制的舵机脉宽值。
 * @param min_value 允许输出的最小脉宽。
 * @param max_value 允许输出的最大脉宽。
 * @return int 限幅后的舵机脉宽。
 *
 * 设计原因：
 * - 底座舵机用 `knot_run(6, duty, time)` 直接跟踪横向偏差；
 * - 必须在软件层限制 duty，避免连续大偏差把底座打到机械极限。
 */
static int ClampVisionArmDuty(int value, int min_value, int max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/**
 * @brief 根据误差大小计算比例控制步长，并限制在安全范围内。
 * @param effective_error 已经扣除死区后的正误差，单位可以是像素或面积。
 * @param gain 单位误差对应的坐标修正比例。
 * @param min_step_cm 最小坐标步长，避免误差刚出死区时完全不动。
 * @param max_step_cm 最大坐标步长，避免误差很大时一次跳太远。
 * @return float 本次控制允许使用的正步长，单位 cm。
 *
 * 主要流程：
 * 1. 使用 `min_step + error * gain` 让误差越大步长越大；
 * 2. 通过最大步长限幅，防止目标快速晃动时机械臂大幅甩动；
 * 3. 只返回正数，方向由调用方根据误差符号决定。
 */
static float CalcVisionArmProportionalStep(int32_t effective_error,
                                           float gain,
                                           float min_step_cm,
                                           float max_step_cm)
{
  float step_cm = 0.0f;

  if (effective_error <= 0)
  {
    return 0.0f;
  }

  step_cm = min_step_cm + ((float)effective_error * gain);
  return ClampVisionArmCoord(step_cm, min_step_cm, max_step_cm);
}

/**
 * @brief 根据目标横向像素偏差计算底座舵机 duty 增量。
 * @param dx_px 目标中心相对抓取参考点的 X 偏差，右正左负，单位像素。
 * @return int 需要叠加到底座舵机 duty 上的增量；返回 0 表示横向已进入死区。
 *
 * 主要流程：
 * 1. `dx` 在死区内不动，避免目标中心附近抖动；
 * 2. 超出死区后按误差大小做比例步进，偏得越远底座转得越快；
 * 3. 使用 `VISION_ARM_BASE_SIGN` 保持现场确认过的原方向。
 *
 * 设计原因：
 * - 横向跟踪最适合由底座舵机承担；
 * - 之前把横向偏差转成 `coordinate_set(y)`，遇到 IK 无解或坐标限幅时会表现为目标很偏但机械臂不动。
 */
static int CalcVisionArmBaseDutyDelta(int16_t dx_px)
{
  int32_t abs_error_px = (dx_px >= 0) ? dx_px : -dx_px;
  int32_t effective_error_px = abs_error_px - VISION_ARM_X_DEADBAND_PX;
  float step_float = 0.0f;
  int step_duty = 0;

  if (effective_error_px <= 0)
  {
    return 0;
  }

  step_float = (float)VISION_ARM_BASE_MIN_STEP_DUTY +
               ((float)effective_error_px * VISION_ARM_BASE_GAIN_DUTY_PER_PX);
  if (step_float > (float)VISION_ARM_BASE_MAX_STEP_DUTY)
  {
    step_float = (float)VISION_ARM_BASE_MAX_STEP_DUTY;
  }
  step_duty = (int)(step_float + 0.5f);

  if (dx_px > 0)
  {
    return VISION_ARM_BASE_SIGN * step_duty;
  }

  return -VISION_ARM_BASE_SIGN * step_duty;
}

/**
 * @brief 根据单轴像素偏差计算一次末端坐标比例修正量。
 * @param error_px 目标中心相对抓取参考点的像素偏差，正负号由 MaixCAM2 协议定义。
 * @param deadband_px 该轴死区，绝对值不超过死区时认为已经对准。
 * @param gain_cm_per_px 每 1 像素有效误差对应的坐标修正比例。
 * @param min_step_cm 最小坐标步长，单位 cm。
 * @param max_step_cm 最大坐标步长，单位 cm。
 * @param sign 方向符号，用于适配实际安装方向；取 1 或 -1。
 * @return float 本轮应该叠加到末端坐标上的增量；返回 0 表示该轴不需要移动。
 *
 * 主要流程：
 * 1. 先判断是否进入死区，死区内不动；
 * 2. 对死区之外的有效误差做比例放大，偏差越大移动越快；
 * 3. 根据误差正负号和安装方向符号输出最终坐标增量。
 *
 * 设计原因：
 * - 固定步进会导致目标从画面边缘移动到另一侧时机械臂仍只动一点；
 * - 比例步进能让大误差快速收敛，小误差慢速微调。
 */
static float CalcVisionArmCoordDelta(int16_t error_px,
                                     int16_t deadband_px,
                                     float gain_cm_per_px,
                                     float min_step_cm,
                                     float max_step_cm,
                                     int sign)
{
  int32_t abs_error_px = (error_px >= 0) ? error_px : -error_px;
  int32_t effective_error_px = abs_error_px - deadband_px;
  float step_cm = 0.0f;

  if ((error_px >= -deadband_px) && (error_px <= deadband_px))
  {
    return 0.0f;
  }

  step_cm = CalcVisionArmProportionalStep(effective_error_px,
                                          gain_cm_per_px,
                                          min_step_cm,
                                          max_step_cm);
  if (error_px > 0)
  {
    return (float)sign * step_cm;
  }

  return (float)(-sign) * step_cm;
}

/**
 * @brief 根据目标面积计算机械臂前伸/回缩步进。
 * @param area 当前视觉目标面积，单位像素，由 MaixCAM2 色块检测输出。
 * @return float 末端 `x` 方向修正量，正数表示前伸，负数表示回缩，0 表示距离合适。
 *
 * 主要流程：
 * 1. 把 `VISION_ARM_TARGET_AREA` 作为期望抓取距离对应的目标面积；
 * 2. 面积小于 `目标面积 - 死区` 时，认为目标更远，返回正步进让机械臂前伸；
 * 3. 面积大于 `目标面积 + 死区` 时，认为目标更近，返回负步进让机械臂回缩；
 * 4. 面积偏差特别大时使用双倍步进，加快远距离收敛，但仍受坐标限幅保护。
 *
 * 设计原因：
 * - 单目相机没有真实深度，色块面积是当前最稳定的远近近似量；
 * - 直接用 `dy` 抬大臂不能解决“目标远，需要伸长”的问题，所以远近必须独立成 `x` 轴控制。
 */
static float CalcVisionArmReachDelta(uint16_t area)
{
  int32_t area_error = (int32_t)VISION_ARM_TARGET_AREA - (int32_t)area;
  int32_t abs_area_error = (area_error >= 0) ? area_error : -area_error;
  int32_t effective_area_error = abs_area_error - (int32_t)VISION_ARM_AREA_DEADBAND;
  float step_cm = 0.0f;

  if (effective_area_error <= 0)
  {
    return 0.0f;
  }

  step_cm = CalcVisionArmProportionalStep(effective_area_error,
                                          VISION_ARM_REACH_GAIN_CM_PER_AREA,
                                          VISION_ARM_REACH_MIN_STEP_CM,
                                          VISION_ARM_REACH_MAX_STEP_CM);
  if (area_error > 0)
  {
    return step_cm;
  }

  return -step_cm;
}

/**
 * @brief 消费最新视觉帧并驱动机械臂做小步跟踪动作。
 * @param robot 机械臂控制对象，不能为空；函数内部通过 `coordinate_set()` 下发末端目标坐标。
 * @param frame 最近一次从 `VisionI2CLink::FetchLatest()` 取出的视觉帧缓存，不能为空。
 * @retval None
 *
 * 主要流程：
 * 1. 只处理 `updated=1` 的新帧，避免同一帧被主循环重复消费；
 * 2. 校验视觉帧仍新鲜、确实找到目标、分数和面积达到最低门限；
 * 3. `dx_px` 直接控制 6 号底座舵机，保证目标离中心很远时一定能横向跟踪；
 * 4. 根据目标面积控制末端 `x` 前伸/回缩，根据 `dy_px` 慢速控制末端 `z` 高度；
 * 5. 底座和末端位姿使用不同节流周期，避免前端舵机被每帧重算导致抖动；
 * 6. 所有 duty 和坐标都做限幅，避免连续大偏差打到机械极限。
 *
 * 注意：
 * - 该函数故意不在 I2C 接收流程中调用，所有机械臂动作都在 `loop()` 上下文执行；
 * - 当前只做跟踪演示，不自动闭合爪子或下降抓取，防止未标定时误动作；
 * - 若现场横向/高低方向相反，优先调整 `VISION_ARM_BASE_SIGN` 和 `VISION_ARM_HEIGHT_SIGN`。
 */
static void UpdateVisionArmTracking(LeArm_t *robot, const VisionI2CFrame_t *frame)
{
#if (VISION_ARM_TRACK_ENABLE != 0)
  uint32_t now_ms = millis();
  int base_delta_duty = 0;
  int next_base_duty = 0;
  float reach_delta_cm = 0.0f;
  float height_delta_cm = 0.0f;
  float next_x_cm = 0.0f;
  float next_z_cm = 0.0f;
  uint8_t pose_move_ok = 0U;
  uint8_t base_moved = 0U;
  uint8_t pose_due = 0U;
  uint8_t inside_x_deadband = 0U;
  uint8_t inside_y_deadband = 0U;

  if ((robot == NULL) || (frame == NULL))
  {
    return;
  }

  if (frame->updated == 0U)
  {
    return;
  }

  /*
   * frame_id 防重判断放宽：
   * - 之前只要 frame_id 相同就立刻跳过，但 MaixCAM2 端 100Hz 发帧时 frame_id 也只是
   *   16 位单调递增，连续两帧之间的内容差异往往才是真正驱动机械臂的依据；
   * - 现在改成"frame_id 不变 且 距上次执行不到 50ms"才跳过，避免高速发帧下连续被丢。
   */
  if ((frame->frame_id == g_vision_arm_last_frame_id) &&
      ((uint32_t)(now_ms - g_vision_arm_last_consume_ms) < VISION_ARM_FRAME_REPLAY_MIN_MS))
  {
    return;
  }
  g_vision_arm_last_consume_ms = now_ms;

  if (vision_i2c_obj.HasFreshFrame() == 0U)
  {
    return;
  }

  if ((frame->found == 0U) ||
      (frame->score < VISION_ARM_MIN_SCORE) ||
      (frame->area < VISION_ARM_MIN_AREA))
  {
    /*
     * 找不到目标或目标置信度过低时，把抑振计数清零。
     * 这样下次重新看到目标时，第一帧就允许动作，避免被旧的"静止状态"卡住。
     */
    g_vision_arm_idle_streak = 0U;
    return;
  }

  /*
   * 静止抑振：
   * - 当前帧 dx/dy 都落在死区内时，认为目标已经稳定居中，本轮强制不动；
   * - 连续 N 帧都在死区时，进一步把节流时间窗口"刷新"为最近一次时间，
   *   避免目标停下后底层 ±2 像素噪声偶尔越过死区也让舵机抖一下；
   * - 任何一帧 dx/dy 超出死区，就立即清零计数允许后续比例控制。
   */
  inside_x_deadband = ((frame->dx_px >= -VISION_ARM_X_DEADBAND_PX) &&
                       (frame->dx_px <= VISION_ARM_X_DEADBAND_PX)) ? 1U : 0U;
  inside_y_deadband = ((frame->dy_px >= -VISION_ARM_Y_DEADBAND_PX) &&
                       (frame->dy_px <= VISION_ARM_Y_DEADBAND_PX)) ? 1U : 0U;

  if ((inside_x_deadband != 0U) && (inside_y_deadband != 0U))
  {
    if (g_vision_arm_idle_streak < VISION_ARM_IDLE_HOLD_FRAMES)
    {
      g_vision_arm_idle_streak = (uint8_t)(g_vision_arm_idle_streak + 1U);
    }

    if (g_vision_arm_idle_streak >= VISION_ARM_IDLE_HOLD_FRAMES)
    {
      g_vision_arm_last_base_move_ms = now_ms;
      g_vision_arm_last_pose_move_ms = now_ms;
      g_vision_arm_last_frame_id = frame->frame_id;
      return;
    }
  }
  else
  {
    g_vision_arm_idle_streak = 0U;
  }

  base_delta_duty = CalcVisionArmBaseDutyDelta(frame->dx_px);
  if ((base_delta_duty != 0) &&
      ((uint32_t)(now_ms - g_vision_arm_last_base_move_ms) >= VISION_ARM_BASE_INTERVAL_MS))
  {
    next_base_duty = ClampVisionArmDuty(g_vision_arm_base_duty + base_delta_duty,
                                        VISION_ARM_BASE_MIN_DUTY,
                                        VISION_ARM_BASE_MAX_DUTY);
    if (next_base_duty != g_vision_arm_base_duty)
    {
      g_vision_arm_base_duty = next_base_duty;
      robot->knot_run(6U, g_vision_arm_base_duty, VISION_ARM_BASE_MOVE_TIME_MS);
      g_vision_arm_last_base_move_ms = now_ms;
      base_moved = 1U;
    }
  }

  /*
   * 远近控制路径：
   * - 摄像头最终装在机械臂末端做 eye-in-hand 抓取，面积↔距离是物理刚性关系，
   *   通过 `VISION_ARM_USE_AREA_FOR_REACH = 1` 默认开启；
   * - 手持调试期手抖会让面积有 ±100 像素噪声，靠 `VISION_ARM_AREA_DEADBAND` 屏蔽；
   * - 关闭这条路径只需把宏置 0，远近就交给手动调节。
   */
#if (VISION_ARM_USE_AREA_FOR_REACH != 0)
  reach_delta_cm = CalcVisionArmReachDelta(frame->area);
#else
  reach_delta_cm = 0.0f;
#endif

  height_delta_cm = CalcVisionArmCoordDelta(frame->dy_px,
                                            VISION_ARM_Y_DEADBAND_PX,
                                            VISION_ARM_HEIGHT_GAIN_CM_PER_PX,
                                            VISION_ARM_HEIGHT_MIN_STEP_CM,
                                            VISION_ARM_HEIGHT_MAX_STEP_CM,
                                            VISION_ARM_HEIGHT_SIGN);

  pose_due = ((uint32_t)(now_ms - g_vision_arm_last_pose_move_ms) >= VISION_ARM_POSE_INTERVAL_MS) ? 1U : 0U;
  if ((pose_due != 0U) &&
      ((reach_delta_cm != 0.0f) || (height_delta_cm != 0.0f)))
  {
    next_x_cm = ClampVisionArmCoord(g_vision_arm_pose_x_cm + reach_delta_cm,
                                    VISION_ARM_X_MIN_CM,
                                    VISION_ARM_X_MAX_CM);
    next_z_cm = ClampVisionArmCoord(g_vision_arm_pose_z_cm + height_delta_cm,
                                    VISION_ARM_Z_MIN_CM,
                                    VISION_ARM_Z_MAX_CM);

    if ((next_x_cm != g_vision_arm_pose_x_cm) || (next_z_cm != g_vision_arm_pose_z_cm))
    {
      /*
       * IK 调用三段尝试：
       * 1. 先尝试同时移动 x/z 到目标，给 IK 完整的 pitch 搜索范围 [-90, +90]，
       *    避免之前 max_pitch=0 把搜索空间限死、解明明存在却找不到；
       * 2. 第一步失败时，只移动 z 保持 x 不变；
       * 3. 第二步还失败时，只移动 x 保持 z 不变；
       * 这样即便目标点本身在工作空间边缘，单轴方向上仍能逐步逼近，避免 xyz 卡死。
       */
      pose_move_ok = robot->coordinate_set(next_x_cm,
                                           DEFAULT_Y,
                                           next_z_cm,
                                           VISION_ARM_PITCH_DEG,
                                           MIN_PITCH,
                                           MAX_PITCH,
                                           VISION_ARM_POSE_MOVE_TIME_MS);
      if (pose_move_ok != 0U)
      {
        g_vision_arm_pose_x_cm = next_x_cm;
        g_vision_arm_pose_z_cm = next_z_cm;
      }
      else if (next_z_cm != g_vision_arm_pose_z_cm)
      {
        /* 同步两轴失败时，先尝试只动 z，至少把高度跟上来。 */
        pose_move_ok = robot->coordinate_set(g_vision_arm_pose_x_cm,
                                             DEFAULT_Y,
                                             next_z_cm,
                                             VISION_ARM_PITCH_DEG,
                                             MIN_PITCH,
                                             MAX_PITCH,
                                             VISION_ARM_POSE_MOVE_TIME_MS);
        if (pose_move_ok != 0U)
        {
          g_vision_arm_pose_z_cm = next_z_cm;
        }
      }

      if ((pose_move_ok == 0U) && (next_x_cm != g_vision_arm_pose_x_cm))
      {
        /* z 方向也没成功时，最后尝试只动 x，至少让远近响应起来。 */
        pose_move_ok = robot->coordinate_set(next_x_cm,
                                             DEFAULT_Y,
                                             g_vision_arm_pose_z_cm,
                                             VISION_ARM_PITCH_DEG,
                                             MIN_PITCH,
                                             MAX_PITCH,
                                             VISION_ARM_POSE_MOVE_TIME_MS);
        if (pose_move_ok != 0U)
        {
          g_vision_arm_pose_x_cm = next_x_cm;
        }
      }

      g_vision_arm_last_pose_move_ms = now_ms;
    }
  }

  if ((base_moved != 0U) || (pose_move_ok != 0U))
  {
    g_vision_arm_last_frame_id = frame->frame_id;
  }

  if ((base_moved != 0U) ||
      (pose_move_ok != 0U) ||
      ((uint32_t)(now_ms - g_vision_arm_last_report_ms) >= VISION_ARM_DEBUG_REPORT_INTERVAL_MS))
  {
    g_vision_arm_last_report_ms = now_ms;
    Serial.printf("[VISION_ARM] base=%u pose=%u idle=%u frame=%u dx=%d dy=%d score=%u area=%u duty=%d xyz=%.2f,%.2f,%.2f delta=%.2f,%.2f\r\n",
                base_moved,
                pose_move_ok,
                g_vision_arm_idle_streak,
                frame->frame_id,
                frame->dx_px,
                frame->dy_px,
                frame->score,
                frame->area,
                g_vision_arm_base_duty,
                g_vision_arm_pose_x_cm,
                (float)DEFAULT_Y,
                g_vision_arm_pose_z_cm,
                reach_delta_cm,
                height_delta_cm);
  }
#else
  (void)robot;
  (void)frame;
#endif
}

void setup() {
  delay(1000);

  /*
   * 串口必须尽早打开。
   * 之前 `Serial.begin()` 放在 `arm.init()` 后面，如果机械臂初始化阶段卡住、
   * 复位或电源不稳，串口监视器只能看到 ESP32 ROM 启动日志，看不到应用层
   * 到底执行到了哪一步。
   */
#if (VISION_DEBUG_USB_SERIAL != 0)
  Serial.begin(115200);
#else
  Serial.begin(115200 , SERIAL_8N1 , PA5 , PA4);
#endif
  delay(200);
  Serial.println("[BOOT] LeArm vision firmware setup begin");

  pinMode(IO_BLE_CTL, OUTPUT);
  digitalWrite(IO_BLE_CTL, LOW);  // 设置蓝牙控制引脚为低电平时，断开蓝牙模块电源

  led_obj.init(IO_LED);
  buzzer_obj.init(IO_BUZZER);

#if (VISION_I2C_LINE_TEST != 0)
  RunVisionI2CLineTest();
#endif

  /*
   * MaixCAM2 视觉链路挂在控制板 I2C 口：
   * - SDA -> GPIO17
   * - SCL -> GPIO16
   * 这里仅负责把 ESP32 注册成 I2C 从机；真正的机械臂动作在 `loop()` 中
   * 基于已校验的新鲜视觉帧执行，不能放进 I2C 接收处理流程里。
   */
  Serial.println("[BOOT] vision i2c init begin");
  vision_i2c_obj.Init();
  PlayVisionI2CInitBuzzerFeedback(vision_i2c_obj.IsReady());
  Serial.println("[BOOT] vision i2c init done");

#if (VISION_I2C_MINIMAL_SLAVE_TEST != 0)
  Serial.println("[BOOT] minimal i2c slave test mode, skip arm init");
  return;
#endif

  /*
   * 机械臂初始化放在 I2C 从机初始化之后。
   * 机械臂初始化会访问外部 Flash、串口总线舵机并下发复位动作，若供电或舵机总线异常，
   * 这里可能耗时较长；前面的 I2C 日志可以先证明视觉从机是否已经挂上总线。
   */
  Serial.println("[BOOT] arm init begin");
  arm.init();
  Serial.println("[BOOT] arm init done");

  pc_ble_obj.init(0);

  /*
   * 机械臂初始化会配置串口、Flash、舵机和若干 GPIO。虽然当前代码没有直接占用
   * I2C1，但实测 Maix 单独探测能找到 0x42、主程序联调时又偶发扫不到，因此这里
   * 在所有机械臂外设初始化完成后重新安装一次视觉 I2C slave。
   * 这样最终进入循环前，ESP32 对 MaixCAM2 暴露的一定是最新的 0x42 从机状态。
   */
  Serial.println("[BOOT] vision i2c reinit after arm begin");
  vision_i2c_obj.Init();
  Serial.println("[BOOT] vision i2c reinit after arm done");

  Serial.println("[BOOT] setup done");
}

void loop() {
#if (VISION_I2C_MINIMAL_SLAVE_TEST != 0)
  vision_i2c_obj.ProcessPending();
  ReportVisionI2CDebugCounters();
  delay(20);
  return;
#endif

  /*
   * 主循环每轮先刷新最近视觉结果缓存。
   * 基础跟踪动作从 `latest_vision_frame` 和 `vision_i2c_obj.HasFreshFrame()`
   * 读取目标偏差，后续完整抓取状态机也应沿用这个入口，而不是直接在 I2C
   * 接收流程里控制机械臂。
   */
  vision_i2c_obj.ProcessPending();
  vision_i2c_obj.FetchLatest(&latest_vision_frame, 1U);
  UpdateVisionDebugLed(&latest_vision_frame);
  UpdateVisionArmTracking(&arm, &latest_vision_frame);
  ReportVisionI2CDebugCounters();
  pc_ble_obj.PC_BLE_Task(&arm , &led_obj , &buzzer_obj);
}

