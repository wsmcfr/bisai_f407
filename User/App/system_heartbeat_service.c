#include "system_heartbeat_service.h"

#include "FreeRTOS.h"
#include "gpio.h"
#include "task.h"

/* 板载 PF9 对应最小系统板上的 LED0，适合作为系统心跳灯输出口。 */
#define SYSTEM_HEARTBEAT_LED_GPIO_PORT    GPIOF
#define SYSTEM_HEARTBEAT_LED_PIN          GPIO_PIN_9

/* 心跳灯每 500ms 翻转一次，完整亮灭周期约 1s，肉眼容易观察。 */
#define SYSTEM_HEARTBEAT_TOGGLE_PERIOD_MS 500U

/**
 * @brief 系统心跳灯任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 主要流程：
 * 1. 记录当前任务基准节拍；
 * 2. 周期性翻转 PF9 电平；
 * 3. 使用 vTaskDelayUntil 保证心跳节拍稳定，避免累积漂移。
 *
 * @return 无返回值。
 */
void SystemHeartbeatService_Task(void *argument)
{
    TickType_t last_wake_tick;
    const TickType_t toggle_period_tick = pdMS_TO_TICKS(SYSTEM_HEARTBEAT_TOGGLE_PERIOD_MS);

    /* 当前任务不使用入参，显式转换可避免编译器未使用参数告警。 */
    (void)argument;

    /* 记录第一次唤醒基准点，后续使用绝对节拍延时保证闪烁周期稳定。 */
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        /* 周期翻转板载 LED0；若该任务停止运行，心跳灯也会停止闪烁。 */
        HAL_GPIO_TogglePin(SYSTEM_HEARTBEAT_LED_GPIO_PORT, SYSTEM_HEARTBEAT_LED_PIN);

        /* 使用固定节拍延时，避免普通相对延时带来的累计误差。 */
        vTaskDelayUntil(&last_wake_tick, toggle_period_tick);
    }
}
