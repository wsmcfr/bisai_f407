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
 * 1. 初始化 HX711 驱动，未接称重模块时只置位二进制故障位，不阻塞 USART1 命令消费；
 * 2. 执行空载去皮，并周期性读取、滤波最近一次重量原始值；
 * 3. 作为 USART1 唯一任务级消费者，优先分发 `A5 5A ... 6B` 二进制协议帧；
 * 4. MP157 正式链路的正确返回只允许 `ACK/STATUS_REPORT` 二进制帧；
 * 5. MP157 正式链路的错误返回只允许 `NACK/FAULT_REPORT` 二进制帧；
 * 6. 旧 `STATUS/GET/TARE/CAL <克重>` 文本入口只保留为断开 MP157 后的离线维护入口，
 *    正式接 MP157 时 USART1 文本输出默认静默，不能作为成功或失败依据。
 */
void WeightService_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
