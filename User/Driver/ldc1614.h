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
    LDC1614_STATUS_OK = 0,          /* 操作成功，寄存器读写或数据读取流程已经完成。 */
    LDC1614_STATUS_ERROR,           /* HAL 返回普通错误，无法进一步区分为超时或参数问题。 */
    LDC1614_STATUS_TIMEOUT,         /* I2C 访问超时，通常需要检查 I2C2 接线、上拉、电源或芯片响应。 */
    LDC1614_STATUS_NOT_READY,       /* 预留状态，表示目标数据尚未就绪，当前底层实现主要由服务层判断。 */
    LDC1614_STATUS_SENSOR_ERROR,    /* DATAx_MSB 中携带错误标志，说明该次转换结果不应参与判定。 */
    LDC1614_STATUS_DEVICE_MISMATCH, /* 制造商 ID 或器件 ID 不匹配，总线上不是预期的 LDC1614。 */
    LDC1614_STATUS_INVALID_PARAM    /* 调用参数非法，例如句柄为空、输出指针为空或通道号不支持。 */
} LDC1614_Status_t;

/**
 * @brief LDC1614 通道编号。
 *
 * 当前项目已经使用通道 0 和通道 1，
 * 但保留完整枚举是为了后续扩展更多通道时不必重写接口。
 */
typedef enum
{
    LDC1614_CHANNEL_0 = 0U, /* LDC1614 CH0，当前项目作为第 1 路检测线圈使用。 */
    LDC1614_CHANNEL_1 = 1U, /* LDC1614 CH1，当前项目作为第 2 路检测线圈使用。 */
    LDC1614_CHANNEL_2 = 2U, /* LDC1614 CH2，当前项目未启用，保留给后续扩展。 */
    LDC1614_CHANNEL_3 = 3U  /* LDC1614 CH3，当前项目未启用，保留给后续扩展。 */
} LDC1614_Channel_t;

/**
 * @brief LDC1614 驱动句柄。
 *
 * 该结构体保存 LDC1614 工作所需的硬件绑定和默认配置寄存器值。
 * 当前设计目标是尽量把“可调参数”集中收口，避免应用层散落硬编码。
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;            /* LDC1614 所绑定的 I2C 句柄，当前由底层驱动独占用于寄存器读写。 */
    uint16_t device_address;            /* LDC1614 7 位器件地址左移后的 HAL 地址值，用于 I2C 主机寻址。 */
    LDC1614_Channel_t active_channel;   /* 当前默认读取的活动通道，供 ReadActiveChannelRaw 选择数据寄存器。 */
    uint16_t reference_count;           /* RCOUNTx 寄存器默认值，决定单次转换积分时间和分辨率。 */
    uint16_t settle_count;              /* SETTLECOUNTx 寄存器默认值，决定通道切换后的传感器稳定等待时间。 */
    uint16_t clock_dividers;            /* CLOCK_DIVIDERSx 寄存器默认值，用于设置传感器和参考时钟分频。 */
    uint16_t error_config;              /* ERROR_CONFIG 寄存器默认值，用于选择 LDC1614 错误检测和 INTB 输出行为。 */
    uint16_t config;                    /* CONFIG 寄存器默认值，用于配置转换模式、睡眠状态和参考时钟来源。 */
    uint16_t mux_config;                /* MUX_CONFIG 寄存器默认值，用于配置自动扫描通道范围和消抖策略。 */
    uint16_t drive_current;             /* DRIVE_CURRENTx 寄存器默认值，用于设置线圈激励电流幅度。 */
} LDC1614_Handle_t;

#define LDC1614_DEFAULT_DEVICE_ADDRESS      (0x54U)
#define LDC1614_DEVICE_ID_VALUE             (0x3055U)
#define LDC1614_MANUFACTURER_ID_VALUE       (0x5449U)

/**
 * @brief 加载 LDC1614 默认配置到驱动句柄。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param hi2c 绑定到 LDC1614 的 STM32 HAL I2C 句柄，当前工程使用 I2C2。
 *
 * 函数作用：
 * 1. 绑定 I2C 外设句柄和 HAL 使用的器件地址；
 * 2. 写入 CH0/CH1 顺序扫描所需的默认采样窗口、稳定时间、分频和驱动电流；
 * 3. 只修改 RAM 中的句柄配置，不立即访问芯片寄存器。
 *
 * @return 无返回值；参数非法时直接返回，调用者应在后续 `LDC1614_Init()` 中得到错误状态。
 */
void LDC1614_LoadDefaultConfig(LDC1614_Handle_t *ldc, I2C_HandleTypeDef *hi2c);

/**
 * @brief 初始化 LDC1614 芯片并写入当前句柄中的默认寄存器配置。
 * @param ldc 已经调用 `LDC1614_LoadDefaultConfig()` 填好的驱动句柄，不能为 NULL。
 * @return LDC1614_Status_t 初始化结果；`OK` 表示芯片 ID 匹配且寄存器配置写入成功。
 *
 * 主要流程：
 * 1. 发送软复位；
 * 2. 读取制造商 ID 和器件 ID，确认总线上确实是 LDC1614；
 * 3. 初始化 CH0/CH1 的 RCOUNT、SETTLECOUNT、CLOCK_DIVIDERS、DRIVE_CURRENT；
 * 4. 写入 MUX/ERROR/CONFIG，使芯片进入 CH0/CH1 连续转换模式。
 */
LDC1614_Status_t LDC1614_Init(LDC1614_Handle_t *ldc);

/**
 * @brief 向 LDC1614 的 16bit 寄存器写入一个值。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param register_address 8bit 寄存器地址，例如 `LDC1614_REG_CONFIG`。
 * @param value 要写入的 16bit 寄存器值，函数内部按高字节在前发送。
 * @return LDC1614_Status_t 写寄存器结果；I2C 超时会转换为 `LDC1614_STATUS_TIMEOUT`。
 *
 * 该函数是所有配置写入的统一出口，便于集中维护 I2C 超时和错误码转换逻辑。
 */
LDC1614_Status_t LDC1614_WriteRegister(const LDC1614_Handle_t *ldc,
                                       uint8_t register_address,
                                       uint16_t value);

/**
 * @brief 从 LDC1614 的 16bit 寄存器读取一个值。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param register_address 8bit 寄存器地址，例如 `LDC1614_REG_STATUS`。
 * @param value 输出参数，用于接收读出的 16bit 寄存器值，不能为 NULL。
 * @return LDC1614_Status_t 读寄存器结果；成功时 `*value` 已经更新。
 *
 * 函数内部按 LDC1614 大端格式组合高/低字节，应用层不需要再关心字节序。
 */
LDC1614_Status_t LDC1614_ReadRegister(const LDC1614_Handle_t *ldc,
                                      uint8_t register_address,
                                      uint16_t *value);

/**
 * @brief 读取 LDC1614 STATUS 状态寄存器。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param status_register 输出参数，用于接收 STATUS 寄存器原始值，不能为 NULL。
 * @return LDC1614_Status_t 读取结果。
 *
 * STATUS 中包含通道数据就绪位和错误位，上层通常会继续调用
 * `LDC1614_HasUnreadConversion()` 判断某个通道是否有新数据。
 */
LDC1614_Status_t LDC1614_ReadStatus(const LDC1614_Handle_t *ldc, uint16_t *status_register);

/**
 * @brief 判断 STATUS 寄存器中某个通道是否有未读取的新转换值。
 * @param status_register `LDC1614_ReadStatus()` 读到的 STATUS 原始值。
 * @param channel 需要判断的通道号，当前业务主要使用 CH0/CH1。
 * @return uint8_t 1 表示该通道有新转换值，0 表示没有新值或通道非法。
 *
 * 该函数只解析状态位，不访问 I2C，适合在任务状态机中反复调用。
 */
uint8_t LDC1614_HasUnreadConversion(uint16_t status_register, LDC1614_Channel_t channel);

/**
 * @brief 读取指定通道的 28bit 原始转换值。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param channel 目标通道号，当前业务主要使用 CH0/CH1。
 * @param raw_value 输出参数，用于接收 28bit 原始计数值，不能为 NULL。
 * @return LDC1614_Status_t 读取结果；若 DATA_MSB 中携带错误标志，返回 `SENSOR_ERROR`。
 *
 * 主要流程：
 * 1. 根据通道号找到 DATAx_MSB/DATAx_LSB；
 * 2. 连续读取两个 16bit 数据寄存器；
 * 3. 屏蔽 MSB 中的错误/状态位，只保留 28bit 传感器计数。
 */
LDC1614_Status_t LDC1614_ReadChannelRaw(const LDC1614_Handle_t *ldc,
                                        LDC1614_Channel_t channel,
                                        uint32_t *raw_value);

/**
 * @brief 读取句柄当前 `active_channel` 指定通道的 28bit 原始值。
 * @param ldc LDC1614 驱动句柄指针，不能为 NULL。
 * @param raw_value 输出参数，用于接收原始计数值，不能为 NULL。
 * @return LDC1614_Status_t 读取结果。
 *
 * 该函数是 `LDC1614_ReadChannelRaw()` 的便捷封装，适合单通道轮询场景；
 * 当前双通道服务为了明确区分 CH0/CH1，通常直接调用 `LDC1614_ReadChannelRaw()`。
 */
LDC1614_Status_t LDC1614_ReadActiveChannelRaw(const LDC1614_Handle_t *ldc, uint32_t *raw_value);

#ifdef __cplusplus
}
#endif

#endif
