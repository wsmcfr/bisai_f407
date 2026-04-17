#ifndef USER_APP_WEIGHT_SERVICE_H
#define USER_APP_WEIGHT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电子称应用任务入口。
 * @param argument FreeRTOS任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 初始化HX711驱动；
 * 2. 执行空载去皮；
 * 3. 周期性读取重量；
 * 4. 通过USART1输出串口文本结果。
 */
void WeightService_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
