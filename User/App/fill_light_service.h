#ifndef USER_APP_FILL_LIGHT_SERVICE_H
#define USER_APP_FILL_LIGHT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> /* 提供 uint8_t、uint16_t 等定宽类型，固定协议与角度换算的数据宽度。 */

/**
 * @brief 270 度舵机允许的最小绝对角度，单位为度。
 */
#define FILL_LIGHT_SERVO_MIN_ANGLE_DEG       (0U)

/**
 * @brief 270 度舵机允许的最大绝对角度，单位为度。
 */
#define FILL_LIGHT_SERVO_MAX_ANGLE_DEG       (270U)

/**
 * @brief 关闭补光灯时的舵机绝对角度，单位为度。
 *
 * 如果现场机械开关的关灯位置不是 0 度，只需在完成安全标定后修改本宏。
 */
#define FILL_LIGHT_SERVO_OFF_ANGLE_DEG       (0U)

/**
 * @brief 打开补光灯时的舵机绝对角度，单位为度。
 *
 * 当前按用户确认值设为 270 度；现场需要减小行程时应优先修改本宏，不改换算公式。
 */
#define FILL_LIGHT_SERVO_ON_ANGLE_DEG        (270U)

/**
 * @brief 舵机 0 度对应的默认高电平脉宽，单位为微秒。
 *
 * 500 us 是首版标定值，必须结合具体 270 度舵机说明书和实物行程确认。
 */
#define FILL_LIGHT_SERVO_MIN_PULSE_US        (500U)

/**
 * @brief 舵机 270 度对应的默认高电平脉宽，单位为微秒。
 *
 * 2500 us 是首版标定值；若舵机出现堵转或撞限位，应减小该值再重新标定。
 */
#define FILL_LIGHT_SERVO_MAX_PULSE_US        (2500U)

/**
 * @brief 每次舵机动作保持 PWM 的时间，单位为毫秒。
 *
 * 专用 FreeRTOS 任务保持目标脉宽 2 秒后停止 PWM，避免舵机长期通电发热。
 */
#define FILL_LIGHT_SERVO_MOVE_HOLD_MS        (2000U)

/**
 * @brief 把 270 度舵机目标角度线性换算为 PWM 高电平脉宽。
 * @param angle_deg 目标绝对角度，单位为度；超过 270 度时自动限制到 270 度。
 * @return uint16_t 返回目标高电平脉宽，单位为微秒，默认范围为 500~2500 us。
 *
 * 换算公式：pulse_us = min_us + angle * (max_us - min_us) / 270。
 */
uint16_t FillLightService_AngleToPulseUs(uint16_t angle_deg);

/**
 * @brief 初始化补光舵机命令队列和 PB6/TIM4_CH1 PWM 硬件。
 * @return uint8_t 1 表示队列和 PWM 初始化成功，0 表示队列分配或 HAL 初始化失败。
 *
 * 本函数应在创建补光任务前调用一次；失败时不得启动自动检测流程。
 */
uint8_t FillLightService_Init(void);

/**
 * @brief 非阻塞投递一次补光灯舵机开关请求。
 * @param cycle_id 当前自动检测流程 ID，用于完成事件匹配。
 * @param action 绝对动作，0 表示关灯回 0 度，1 表示开灯到 270 度。
 * @param related_seq 触发本动作的 MP157 命令序号，用于完成事件精确匹配。
 * @return uint8_t 1 表示请求已进入队列，0 表示服务未初始化、参数非法或队列已满。
 */
uint8_t FillLightService_Request(uint16_t cycle_id,
                                uint8_t action,
                                uint16_t related_seq);

/**
 * @brief 补光舵机专用 FreeRTOS 任务入口。
 * @param argument RTOS 创建任务时传入的参数，当前未使用。
 * @return 无返回值；任务永久等待队列并串行执行每次 2 秒 PWM 动作。
 */
void FillLightService_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
