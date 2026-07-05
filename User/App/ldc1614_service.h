#ifndef USER_APP_LDC1614_SERVICE_H
#define USER_APP_LDC1614_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 供二进制自动流程读取的 LDC1614 电感检测快照。
 *
 * 该结构保存最近一次稳定测量结果，不主动触发 I2C 新采样。
 * F4 应在 ESP32S3 确认“零件已放到电磁感应模块并放稳”后读取它，再打包 LDC_RESULT。
 */
typedef struct
{
    uint16_t sample_id;              /* LDC 服务本地递增样本号，用于 MP157 判断是否拿到新快照。 */
    uint8_t channel_mask;            /* 有效通道位图，bit0=CH0，bit1=CH1。 */
    uint8_t decision;                /* 0=unknown，1=pass，2=fail，3=review；任一通道 fail 则整体 fail。 */
    uint8_t status;                  /* 最近一次底层或服务层状态，0 表示 OK。 */
    uint32_t ch0_raw;                /* CH0 最近滤波原始值。 */
    int32_t ch0_delta;               /* CH0 最近稳定变化量。 */
    uint32_t ch1_raw;                /* CH1 最近滤波原始值。 */
    int32_t ch1_delta;               /* CH1 最近稳定变化量。 */
    uint16_t duration_ms;            /* 测量稳定窗口估计耗时，单位 ms。 */
    uint16_t option_bits;            /* 扩展位，bit0=存在参考值，bit1=结果来自稳定窗口。 */
} Ldc1614Service_Snapshot_t;

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

/**
 * @brief 获取 LDC1614 服务最近一次稳定测量快照。
 * @param snapshot 输出电感快照，不能为空。
 * @return uint8_t 1 表示已有可上报快照，0 表示尚未产生稳定测量结果。
 *
 * 该函数不阻塞、不访问 I2C，只复制 Ldc1614Service_Task 最近维护的结果。
 */
uint8_t Ldc1614Service_GetLatestSnapshot(Ldc1614Service_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
