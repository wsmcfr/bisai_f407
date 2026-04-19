#include "ldc1614.h"

#define LDC1614_I2C_TIMEOUT_MS                 (100U)
#define LDC1614_REG_DATA0_MSB                  (0x00U)
#define LDC1614_REG_DATA0_LSB                  (0x01U)
#define LDC1614_REG_DATA1_MSB                  (0x02U)
#define LDC1614_REG_DATA1_LSB                  (0x03U)
#define LDC1614_REG_DATA2_MSB                  (0x04U)
#define LDC1614_REG_DATA2_LSB                  (0x05U)
#define LDC1614_REG_DATA3_MSB                  (0x06U)
#define LDC1614_REG_DATA3_LSB                  (0x07U)
#define LDC1614_REG_RCOUNT0                    (0x08U)
#define LDC1614_REG_RCOUNT1                    (0x09U)
#define LDC1614_REG_OFFSET0                    (0x0CU)
#define LDC1614_REG_OFFSET1                    (0x0DU)
#define LDC1614_REG_SETTLECOUNT0               (0x10U)
#define LDC1614_REG_SETTLECOUNT1               (0x11U)
#define LDC1614_REG_CLOCK_DIVIDERS0            (0x14U)
#define LDC1614_REG_CLOCK_DIVIDERS1            (0x15U)
#define LDC1614_REG_STATUS                     (0x18U)
#define LDC1614_REG_ERROR_CONFIG               (0x19U)
#define LDC1614_REG_CONFIG                     (0x1AU)
#define LDC1614_REG_MUX_CONFIG                 (0x1BU)
#define LDC1614_REG_RESET_DEVICE               (0x1CU)
#define LDC1614_REG_DRIVE_CURRENT0             (0x1EU)
#define LDC1614_REG_DRIVE_CURRENT1             (0x1FU)
#define LDC1614_REG_MANUFACTURER_ID            (0x7EU)
#define LDC1614_REG_DEVICE_ID                  (0x7FU)
#define LDC1614_DATA_ERROR_MASK                (0xF000U)
#define LDC1614_ERROR_CONFIG_DRDY_TO_INTB      (0x0001U)
#define LDC1614_CONFIG_CONTINUOUS_INTB_EN      (0x1E01U)
#define LDC1614_MUX_CONFIG_SEQ_CH0_CH1         (0x820FU)

/**
 * @brief 把 HAL I2C 返回值转换为驱动层状态码。
 * @param hal_status HAL 层返回状态。
 * @return LDC1614_Status_t 转换后的驱动状态。
 */
static LDC1614_Status_t LDC1614_ConvertHalStatus(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return LDC1614_STATUS_OK;

        case HAL_TIMEOUT:
            return LDC1614_STATUS_TIMEOUT;

        default:
            return LDC1614_STATUS_ERROR;
    }
}

/**
 * @brief 根据通道号返回对应的数据寄存器地址。
 * @param channel 目标通道编号。
 * @param msb_register 用于返回 MSB 地址的输出指针。
 * @param lsb_register 用于返回 LSB 地址的输出指针。
 * @return LDC1614_Status_t 地址映射结果。
 */
static LDC1614_Status_t LDC1614_GetDataRegisterAddress(LDC1614_Channel_t channel,
                                                       uint8_t *msb_register,
                                                       uint8_t *lsb_register)
{
    if ((msb_register == NULL) || (lsb_register == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    switch (channel)
    {
        case LDC1614_CHANNEL_0:
            *msb_register = LDC1614_REG_DATA0_MSB;
            *lsb_register = LDC1614_REG_DATA0_LSB;
            break;

        case LDC1614_CHANNEL_1:
            *msb_register = LDC1614_REG_DATA1_MSB;
            *lsb_register = LDC1614_REG_DATA1_LSB;
            break;

        case LDC1614_CHANNEL_2:
            *msb_register = LDC1614_REG_DATA2_MSB;
            *lsb_register = LDC1614_REG_DATA2_LSB;
            break;

        case LDC1614_CHANNEL_3:
            *msb_register = LDC1614_REG_DATA3_MSB;
            *lsb_register = LDC1614_REG_DATA3_LSB;
            break;

        default:
            return LDC1614_STATUS_INVALID_PARAM;
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 根据通道号返回对应的配置寄存器地址。
 * @param channel 目标通道编号。
 * @param rcount_register 用于返回参考计数寄存器地址的输出指针。
 * @param offset_register 用于返回偏移寄存器地址的输出指针。
 * @param settle_register 用于返回稳定时间寄存器地址的输出指针。
 * @param divider_register 用于返回时钟分频寄存器地址的输出指针。
 * @param drive_current_register 用于返回驱动电流寄存器地址的输出指针。
 * @return LDC1614_Status_t 地址映射结果。
 *
 * 当前项目只启用 CH0 和 CH1。
 * 若后续需要扩展更多通道，可继续在这里补齐映射关系。
 */
static LDC1614_Status_t LDC1614_GetChannelConfigRegisters(LDC1614_Channel_t channel,
                                                          uint8_t *rcount_register,
                                                          uint8_t *offset_register,
                                                          uint8_t *settle_register,
                                                          uint8_t *divider_register,
                                                          uint8_t *drive_current_register)
{
    if ((rcount_register == NULL) || (offset_register == NULL) ||
        (settle_register == NULL) || (divider_register == NULL) ||
        (drive_current_register == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    switch (channel)
    {
        case LDC1614_CHANNEL_0:
            *rcount_register = LDC1614_REG_RCOUNT0;
            *offset_register = LDC1614_REG_OFFSET0;
            *settle_register = LDC1614_REG_SETTLECOUNT0;
            *divider_register = LDC1614_REG_CLOCK_DIVIDERS0;
            *drive_current_register = LDC1614_REG_DRIVE_CURRENT0;
            break;

        case LDC1614_CHANNEL_1:
            *rcount_register = LDC1614_REG_RCOUNT1;
            *offset_register = LDC1614_REG_OFFSET1;
            *settle_register = LDC1614_REG_SETTLECOUNT1;
            *divider_register = LDC1614_REG_CLOCK_DIVIDERS1;
            *drive_current_register = LDC1614_REG_DRIVE_CURRENT1;
            break;

        default:
            return LDC1614_STATUS_INVALID_PARAM;
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 按统一参数初始化一个目标通道。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param channel 需要初始化的通道编号。
 * @return LDC1614_Status_t 初始化结果。
 *
 * 当前 CH0/CH1 使用相同的采样窗口、稳定时间和驱动电流，
 * 这样可以保证两个检测通道的比较口径一致。
 */
static LDC1614_Status_t LDC1614_InitChannel(const LDC1614_Handle_t *ldc, LDC1614_Channel_t channel)
{
    LDC1614_Status_t status;
    uint8_t rcount_register;
    uint8_t offset_register;
    uint8_t settle_register;
    uint8_t divider_register;
    uint8_t drive_current_register;

    if (ldc == NULL)
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    status = LDC1614_GetChannelConfigRegisters(channel,
                                               &rcount_register,
                                               &offset_register,
                                               &settle_register,
                                               &divider_register,
                                               &drive_current_register);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, rcount_register, ldc->reference_count);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, offset_register, 0x0000U);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, settle_register, ldc->settle_count);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, divider_register, ldc->clock_dividers);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, drive_current_register, ldc->drive_current);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 加载 LDC1614 默认配置。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param hi2c 绑定的 I2C 句柄，不能为空。
 */
void LDC1614_LoadDefaultConfig(LDC1614_Handle_t *ldc, I2C_HandleTypeDef *hi2c)
{
    if ((ldc == NULL) || (hi2c == NULL))
    {
        return;
    }

    ldc->hi2c = hi2c;
    ldc->device_address = LDC1614_DEFAULT_DEVICE_ADDRESS;
    ldc->active_channel = LDC1614_CHANNEL_0;
    ldc->reference_count = 0xFFFFU;
    ldc->settle_count = 0x0013U;
    ldc->clock_dividers = 0x1001U;
    /*
     * 打开 DRDY_2INT，使每次新转换完成后都能通过 INTB 输出中断事件。
     * 这样应用层就能从轮询模式切到“外部中断唤醒 + 任务读取”模式。
     */
    ldc->error_config = LDC1614_ERROR_CONFIG_DRDY_TO_INTB;
    ldc->config = LDC1614_CONFIG_CONTINUOUS_INTB_EN;
    /*
     * 采用 CH0/CH1 顺序扫描，让两路线圈轮流转换并通过同一个 INTB 上报数据就绪。
     */
    ldc->mux_config = LDC1614_MUX_CONFIG_SEQ_CH0_CH1;
    ldc->drive_current = 0xB000U;
}

/**
 * @brief 向 16bit 寄存器写入数据。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param register_address 8bit 寄存器地址。
 * @param value 需要写入的 16bit 数据。
 * @return LDC1614_Status_t 写寄存器结果。
 */
LDC1614_Status_t LDC1614_WriteRegister(const LDC1614_Handle_t *ldc,
                                       uint8_t register_address,
                                       uint16_t value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[2];

    if ((ldc == NULL) || (ldc->hi2c == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    tx_buffer[0] = (uint8_t)((value >> 8) & 0xFFU);
    tx_buffer[1] = (uint8_t)(value & 0xFFU);

    hal_status = HAL_I2C_Mem_Write(ldc->hi2c,
                                   ldc->device_address,
                                   register_address,
                                   I2C_MEMADD_SIZE_8BIT,
                                   tx_buffer,
                                   sizeof(tx_buffer),
                                   LDC1614_I2C_TIMEOUT_MS);

    return LDC1614_ConvertHalStatus(hal_status);
}

/**
 * @brief 从 16bit 寄存器读取数据。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param register_address 8bit 寄存器地址。
 * @param value 用于接收寄存器值的输出指针，不能为空。
 * @return LDC1614_Status_t 读寄存器结果。
 */
LDC1614_Status_t LDC1614_ReadRegister(const LDC1614_Handle_t *ldc,
                                      uint8_t register_address,
                                      uint16_t *value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t rx_buffer[2];

    if ((ldc == NULL) || (ldc->hi2c == NULL) || (value == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    hal_status = HAL_I2C_Mem_Read(ldc->hi2c,
                                  ldc->device_address,
                                  register_address,
                                  I2C_MEMADD_SIZE_8BIT,
                                  rx_buffer,
                                  sizeof(rx_buffer),
                                  LDC1614_I2C_TIMEOUT_MS);

    if (hal_status != HAL_OK)
    {
        return LDC1614_ConvertHalStatus(hal_status);
    }

    *value = (uint16_t)(((uint16_t)rx_buffer[0] << 8) | rx_buffer[1]);
    return LDC1614_STATUS_OK;
}

/**
 * @brief 初始化 LDC1614 芯片。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @return LDC1614_Status_t 初始化结果。
 */
LDC1614_Status_t LDC1614_Init(LDC1614_Handle_t *ldc)
{
    LDC1614_Status_t status;
    uint16_t manufacturer_id;
    uint16_t device_id;

    if ((ldc == NULL) || (ldc->hi2c == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    status = LDC1614_WriteRegister(ldc, LDC1614_REG_RESET_DEVICE, 0x8000U);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    HAL_Delay(5U);

    status = LDC1614_ReadRegister(ldc, LDC1614_REG_MANUFACTURER_ID, &manufacturer_id);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_ReadRegister(ldc, LDC1614_REG_DEVICE_ID, &device_id);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    if ((manufacturer_id != LDC1614_MANUFACTURER_ID_VALUE) ||
        (device_id != LDC1614_DEVICE_ID_VALUE))
    {
        return LDC1614_STATUS_DEVICE_MISMATCH;
    }

    /*
     * 当前任务使用 CH0 与 CH1 两个通道，因此这里统一初始化两路寄存器。
     * 这样应用层可以直接按通道读取，而不必再在任务里手工补配置。
     */
    status = LDC1614_InitChannel(ldc, LDC1614_CHANNEL_0);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_InitChannel(ldc, LDC1614_CHANNEL_1);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, LDC1614_REG_ERROR_CONFIG, ldc->error_config);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, LDC1614_REG_MUX_CONFIG, ldc->mux_config);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_WriteRegister(ldc, LDC1614_REG_CONFIG, ldc->config);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 读取 STATUS 寄存器。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param status_register 用于接收寄存器值的输出指针，不能为空。
 * @return LDC1614_Status_t 读取结果。
 */
LDC1614_Status_t LDC1614_ReadStatus(const LDC1614_Handle_t *ldc, uint16_t *status_register)
{
    return LDC1614_ReadRegister(ldc, LDC1614_REG_STATUS, status_register);
}

/**
 * @brief 判断指定通道是否存在未读取的新转换值。
 * @param status_register STATUS 寄存器值。
 * @param channel 需要判断的通道。
 * @return uint8_t 1 表示该通道有未读新值，0 表示没有。
 */
uint8_t LDC1614_HasUnreadConversion(uint16_t status_register, LDC1614_Channel_t channel)
{
    uint16_t bit_mask;

    switch (channel)
    {
        case LDC1614_CHANNEL_0:
            bit_mask = 0x0008U;
            break;

        case LDC1614_CHANNEL_1:
            bit_mask = 0x0004U;
            break;

        case LDC1614_CHANNEL_2:
            bit_mask = 0x0002U;
            break;

        case LDC1614_CHANNEL_3:
            bit_mask = 0x0001U;
            break;

        default:
            return 0U;
    }

    return ((status_register & bit_mask) != 0U) ? 1U : 0U;
}

/**
 * @brief 读取指定通道原始 28bit 转换值。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param channel 需要读取的通道编号。
 * @param raw_value 用于接收原始值的输出指针，不能为空。
 * @return LDC1614_Status_t 读取结果。
 */
LDC1614_Status_t LDC1614_ReadChannelRaw(const LDC1614_Handle_t *ldc,
                                        LDC1614_Channel_t channel,
                                        uint32_t *raw_value)
{
    LDC1614_Status_t status;
    uint8_t msb_register;
    uint8_t lsb_register;
    uint16_t data_msb;
    uint16_t data_lsb;

    if ((ldc == NULL) || (raw_value == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    status = LDC1614_GetDataRegisterAddress(channel, &msb_register, &lsb_register);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_ReadRegister(ldc, msb_register, &data_msb);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    status = LDC1614_ReadRegister(ldc, lsb_register, &data_lsb);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    if ((data_msb & LDC1614_DATA_ERROR_MASK) != 0U)
    {
        return LDC1614_STATUS_SENSOR_ERROR;
    }

    *raw_value = (((uint32_t)(data_msb & 0x0FFFU)) << 16) | (uint32_t)data_lsb;
    *raw_value &= 0x0FFFFFFFUL;

    return LDC1614_STATUS_OK;
}

/**
 * @brief 读取当前激活通道原始值。
 * @param ldc LDC1614 句柄指针，不能为空。
 * @param raw_value 用于接收原始值的输出指针，不能为空。
 * @return LDC1614_Status_t 读取结果。
 */
LDC1614_Status_t LDC1614_ReadActiveChannelRaw(const LDC1614_Handle_t *ldc, uint32_t *raw_value)
{
    if (ldc == NULL)
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    return LDC1614_ReadChannelRaw(ldc, ldc->active_channel, raw_value);
}
