#ifndef USER_APP_CONVEYOR_MOTOR_SERVICE_H
#define USER_APP_CONVEYOR_MOTOR_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 传送带 Emm42 电机任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 独占 `USART2` 作为 Emm42 TTL 控制口；
 * 2. 初始化电机驱动，并在启动阶段恢复当前工程要求的控制模式；
 * 3. 使能电机并把电机拉回到已知静止态；
 * 4. 维护 `SCAN / TRACK / STOP` 三态控制；
 * 5. 在跟踪模式下根据主机坐标误差动态调速；
 * 6. 在目标进入摄像头中心死区后立即停止。
 */
void ConveyorMotorService_Task(void *argument);

/**
 * @brief 处理一条发给传送带电机服务的串口命令。
 * @param command_buffer 已经规范化后的命令字符串，不能为空。
 * @return uint8_t 1 表示该命令已由电机服务处理，0 表示不是电机命令。
 *
 * 当前支持：
 * 1. `BELTSCAN`
 * 2. `BELTSTOP`
 * 3. `BELTTRACK <signed_error_px>`
 * 4. `BELTCAM <enable> <current_x> <center_x>`
 * 5. `BELTENABLE <0|1> [signed_error_px]`
 * 6. `BELTINFO`
 *
 * 命令仍然沿用当前 `USART1` 统一入口进行分发，
 * 避免多个任务同时直接消费串口接收缓存。
 */
uint8_t ConveyorMotorService_HandleCommand(const char *command_buffer);

#ifdef __cplusplus
}
#endif

#endif
