#ifndef USER_APP_SYSTEM_HEARTBEAT_SERVICE_H
#define USER_APP_SYSTEM_HEARTBEAT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 系统心跳灯任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 独占板载 PF9/LED0 作为系统心跳指示灯；
 * 2. 以固定周期翻转 LED 电平，形成持续闪烁效果；
 * 3. 用最低业务优先级持续运行，用于辅助判断调度器是否正常工作。
 *
 * 设计说明：
 * 1. 心跳灯单独放在独立任务中，而不是塞进任意业务任务；
 * 2. 任务优先级设置为较低，便于暴露“高优先级任务长时间占用 CPU”
 *    或“系统调度异常”这类问题；
 * 3. 如果系统卡死、调度停止或高优先级任务跑飞，LED 往往会停止闪烁，
 *    便于现场快速判断系统状态。
 */
void SystemHeartbeatService_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
