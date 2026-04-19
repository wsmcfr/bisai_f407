#ifndef USER_APP_LDC1614_SERVICE_H
#define USER_APP_LDC1614_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief LDC1614 双通道缺陷检测应用任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 初始化 I2C2 和 LDC1614 驱动；
 * 2. 为 CH0 / CH1 建立空载基线和动态阈值；
 * 3. 在检测到工件进入后，先等待放稳，再采集稳定窗口；
 * 4. 使用稳定平台值与参考值比较，输出 OK / DEFECT 或 reference=unset；
 * 5. 结果上报后等待工件移开，再重新进入下一轮检测。
 */
void Ldc1614Service_Task(void *argument);

/**
 * @brief 处理一条发给 LDC1614 服务的串口命令。
 * @param command_buffer 已经规范化后的命令字符串，不能为空。
 * @return uint8_t 1 表示该命令已由 LDC 服务处理，0 表示不是 LDC 命令。
 *
 * 当前支持：
 * 1. `LDCCAL CH1`
 * 2. `LDCCAL CH2 20`
 * 3. `LDCSTOP`
 *
 * 命令入口放在重量任务里统一分发，
 * 避免多个任务同时直接读取串口缓存。
 */
uint8_t Ldc1614Service_HandleCommand(const char *command_buffer);

#ifdef __cplusplus
}
#endif

#endif
