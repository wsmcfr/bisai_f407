#ifndef USER_DRIVER_LDC1614_H
#define USER_DRIVER_LDC1614_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief LDC1614 驱动返回状态码。
 *
 * 该枚举统一描述驱动层寄存器访问、设备识别和数据读取的执行结果，
 * 方便应用层根据结果决定是重试、告警还是进入故障等待。
 */
typedef enum
{
    LDC1614_STATUS_OK = 0,
    LDC1614_STATUS_ERROR,
    LDC1614_STATUS_TIMEOUT,
    LDC1614_STATUS_NOT_READY,
    LDC1614_STATUS_SENSOR_ERROR,
    LDC1614_STATUS_DEVICE_MISMATCH,
    LDC1614_STATUS_INVALID_PARAM
} LDC1614_Status_t;

/**
 * @brief LDC1614 通道编号。
 *
 * 当前项目已经使用通道 0 和通道 1，
 * 但保留完整枚举是为了后续扩展更多通道时不必重写接口。
 */
typedef enum
{
    LDC1614_CHANNEL_0 = 0U,
    LDC1614_CHANNEL_1 = 1U,
    LDC1614_CHANNEL_2 = 2U,
    LDC1614_CHANNEL_3 = 3U
} LDC1614_Channel_t;

/**
 * @brief LDC1614 驱动句柄。
 *
 * 该结构体保存 LDC1614 工作所需的硬件绑定和默认配置寄存器值。
 * 当前设计目标是尽量把“可调参数”集中收口，避免应用层散落硬编码。
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t device_address;
    LDC1614_Channel_t active_channel;
    uint16_t reference_count;
    uint16_t settle_count;
    uint16_t clock_dividers;
    uint16_t error_config;
    uint16_t config;
    uint16_t mux_config;
    uint16_t drive_current;
} LDC1614_Handle_t;

#define LDC1614_DEFAULT_DEVICE_ADDRESS      (0x54U)
#define LDC1614_DEVICE_ID_VALUE             (0x3055U)
#define LDC1614_MANUFACTURER_ID_VALUE       (0x5449U)

void LDC1614_LoadDefaultConfig(LDC1614_Handle_t *ldc, I2C_HandleTypeDef *hi2c);
LDC1614_Status_t LDC1614_Init(LDC1614_Handle_t *ldc);
LDC1614_Status_t LDC1614_WriteRegister(const LDC1614_Handle_t *ldc,
                                       uint8_t register_address,
                                       uint16_t value);
LDC1614_Status_t LDC1614_ReadRegister(const LDC1614_Handle_t *ldc,
                                      uint8_t register_address,
                                      uint16_t *value);
LDC1614_Status_t LDC1614_ReadStatus(const LDC1614_Handle_t *ldc, uint16_t *status_register);
uint8_t LDC1614_HasUnreadConversion(uint16_t status_register, LDC1614_Channel_t channel);
LDC1614_Status_t LDC1614_ReadChannelRaw(const LDC1614_Handle_t *ldc,
                                        LDC1614_Channel_t channel,
                                        uint32_t *raw_value);
LDC1614_Status_t LDC1614_ReadActiveChannelRaw(const LDC1614_Handle_t *ldc, uint32_t *raw_value);

#ifdef __cplusplus
}
#endif

#endif
