#include "emm42_motor.h"

/**
 * @brief 统一发送一帧 Emm42 TTL 命令。
 * @param motor 电机句柄指针，不能为空。
 * @param frame 待发送的命令帧缓存区，不能为空。
 * @param length 命令帧长度，必须大于 0。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 当前项目中由“传送带电机任务”独占 `USART2`，
 * 因此这里直接使用阻塞发送即可：
 * 1. 实现简单，便于定位问题；
 * 2. 命令帧很短，阻塞时间可控；
 * 3. 不依赖额外 TX DMA 配置，适合当前工程现状。
 */
static EMM42_MotorStatus_t EMM42_MotorTransmitFrame(const EMM42_MotorHandle_t *motor,
                                                    const uint8_t *frame,
                                                    uint16_t length)
{
    if ((motor == NULL) || (motor->huart == NULL) || (frame == NULL) || (length == 0U))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if (HAL_UART_Transmit(motor->huart,
                          (uint8_t *)frame,
                          length,
                          motor->tx_timeout_ms) != HAL_OK)
    {
        return EMM42_MOTOR_STATUS_ERROR;
    }

    return EMM42_MOTOR_STATUS_OK;
}

/**
 * @brief 加载 Emm42 驱动默认配置。
 * @param motor 电机句柄指针，不能为空。
 * @param huart 绑定的串口句柄，不能为空。
 */
void EMM42_MotorLoadDefaultConfig(EMM42_MotorHandle_t *motor, UART_HandleTypeDef *huart)
{
    if ((motor == NULL) || (huart == NULL))
    {
        return;
    }

    motor->huart = huart;
    motor->address = EMM42_MOTOR_DEFAULT_ADDRESS;
    motor->tx_timeout_ms = EMM42_MOTOR_DEFAULT_TX_TIMEOUT_MS;
}

/**
 * @brief 校验 Emm42 驱动句柄是否可用。
 * @param motor 电机句柄指针，不能为空。
 * @return EMM42_MotorStatus_t 校验结果。
 */
EMM42_MotorStatus_t EMM42_MotorInit(EMM42_MotorHandle_t *motor)
{
    if ((motor == NULL) || (motor->huart == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    return EMM42_MOTOR_STATUS_OK;
}

/**
 * @brief 发送电机使能/失能命令。
 * @param motor 电机句柄指针，不能为空。
 * @param enabled `true` 表示使能，`false` 表示失能。
 * @param sync_flag 同步运动标志。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorSetEnable(const EMM42_MotorHandle_t *motor,
                                         bool enabled,
                                         bool sync_flag)
{
    uint8_t frame[6];

    if (motor == NULL)
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    /*
     * 帧格式来自张大头 Emm_V5 例程：
     * [地址][0xF3][0xAB][使能状态][同步标志][0x6B]
     */
    frame[0] = motor->address;
    frame[1] = 0xF3U;
    frame[2] = 0xABU;
    frame[3] = (uint8_t)(enabled ? 1U : 0U);
    frame[4] = (uint8_t)(sync_flag ? 1U : 0U);
    frame[5] = 0x6BU;

    return EMM42_MotorTransmitFrame(motor, frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 设置电机控制模式。
 * @param motor 电机句柄指针，不能为空。
 * @param control_mode 控制模式。
 * @param save_flag `true` 表示写入电机参数存储，`false` 表示仅本次上电生效。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorSetControlMode(const EMM42_MotorHandle_t *motor,
                                              EMM42_MotorControlMode_t control_mode,
                                              bool save_flag)
{
    uint8_t frame[6];

    if (motor == NULL)
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if ((control_mode != EMM42_MOTOR_CONTROL_MODE_OPEN_LOOP) &&
        (control_mode != EMM42_MOTOR_CONTROL_MODE_CLOSED_LOOP_FOC))
    {
        return EMM42_MOTOR_STATUS_RANGE_ERROR;
    }

    /*
     * 帧格式来自张大头 Emm_V5 例程 `Emm_V5_Modify_Ctrl_Mode()`：
     * [地址][0x46][0x69][保存标志][控制模式][0x6B]
     */
    frame[0] = motor->address;
    frame[1] = 0x46U;
    frame[2] = 0x69U;
    frame[3] = (uint8_t)(save_flag ? 1U : 0U);
    frame[4] = (uint8_t)control_mode;
    frame[5] = 0x6BU;

    return EMM42_MotorTransmitFrame(motor, frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 设置电机面板按键锁定状态。
 * @param motor 电机句柄指针，不能为空。
 * @param locked `true` 表示锁定面板按键，`false` 表示解锁。
 * @param save_flag `true` 表示写入电机参数存储，`false` 表示仅本次上电生效。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorSetButtonLock(const EMM42_MotorHandle_t *motor,
                                             bool locked,
                                             bool save_flag)
{
    uint8_t frame[6];

    if (motor == NULL)
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    /*
     * 帧格式来自张大头 Emm_V5 例程 `Emm_V5_Modify_Lock_Btn()`：
     * [地址][0xD0][0xB3][保存标志][锁定状态][0x6B]
     */
    frame[0] = motor->address;
    frame[1] = 0xD0U;
    frame[2] = 0xB3U;
    frame[3] = (uint8_t)(save_flag ? 1U : 0U);
    frame[4] = (uint8_t)(locked ? 1U : 0U);
    frame[5] = 0x6BU;

    return EMM42_MotorTransmitFrame(motor, frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 发送速度模式命令。
 * @param motor 电机句柄指针，不能为空。
 * @param direction 旋转方向。
 * @param velocity_rpm 目标转速，单位 RPM。
 * @param acceleration 加速度参数。
 * @param sync_flag 同步运动标志。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorSetVelocity(const EMM42_MotorHandle_t *motor,
                                           EMM42_MotorDirection_t direction,
                                           uint16_t velocity_rpm,
                                           uint8_t acceleration,
                                           bool sync_flag)
{
    uint8_t frame[8];

    if (motor == NULL)
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if ((velocity_rpm > EMM42_MOTOR_MAX_SPEED_RPM) || (acceleration > EMM42_MOTOR_MAX_ACCEL))
    {
        return EMM42_MOTOR_STATUS_RANGE_ERROR;
    }

    /*
     * 帧格式来自张大头 Emm_V5 速度模式例程：
     * [地址][0xF6][方向][速度高字节][速度低字节][加速度][同步标志][0x6B]
     */
    frame[0] = motor->address;
    frame[1] = 0xF6U;
    frame[2] = (uint8_t)direction;
    frame[3] = (uint8_t)(velocity_rpm >> 8);
    frame[4] = (uint8_t)(velocity_rpm & 0xFFU);
    frame[5] = acceleration;
    frame[6] = (uint8_t)(sync_flag ? 1U : 0U);
    frame[7] = 0x6BU;

    return EMM42_MotorTransmitFrame(motor, frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 发送立即停止命令。
 * @param motor 电机句柄指针，不能为空。
 * @param sync_flag 同步运动标志。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorStopNow(const EMM42_MotorHandle_t *motor, bool sync_flag)
{
    uint8_t frame[5];

    if (motor == NULL)
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    /*
     * 帧格式来自张大头 Emm_V5 例程：
     * [地址][0xFE][0x98][同步标志][0x6B]
     */
    frame[0] = motor->address;
    frame[1] = 0xFEU;
    frame[2] = 0x98U;
    frame[3] = (uint8_t)(sync_flag ? 1U : 0U);
    frame[4] = 0x6BU;

    return EMM42_MotorTransmitFrame(motor, frame, (uint16_t)sizeof(frame));
}
