#include "ldc1614.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/**
 * @brief LDC1614 I2C 寄存器协议速查。
 *
 * 本文件是 LDC1614 的底层 I2C 寄存器驱动，不直接处理 USART1 用户命令。
 * 用户能发送的 `LDCCAL CH1 [N]`、`LDCCAL CH2 [N]`、`LDCSTOP` 由 `ldc1614_service.c`
 * 解析；解析后最终会调用本文件的寄存器读写函数完成采样、初始化和状态查询。
 *
 * 硬件链路：
 * - STM32 I2C2：PF1=SCL，PF0=SDA，100kHz，SDA/SCL 需要外部或模块板上 3.3V 上拉；
 * - LDC1614 INTB：PF2，下降沿 EXTI2，用于通知应用任务有新转换结果待读取；
 * - LDC1614 地址：`LDC1614_DEFAULT_DEVICE_ADDRESS=0x54`，这是 STM32 HAL 传参使用的左移后地址；
 * - 当前业务只启用 CH0/CH1，CH2/CH3 的寄存器映射保留在驱动层，便于后续扩展。
 *
 * 寄存器访问格式：
 * | 操作 | HAL 调用 | 地址宽度 | 数据宽度 | 本文件函数 |
 * | --- | --- | --- | --- | --- |
 * | 写寄存器 | `HAL_I2C_Mem_Write_DMA` + DMA 完成等待 | 8 bit 寄存器地址 | 16 bit 大端数据 | `LDC1614_WriteRegister()` |
 * | 读寄存器 | `HAL_I2C_Mem_Read_DMA` + DMA 完成等待 | 8 bit 寄存器地址 | 16 bit 大端数据 | `LDC1614_ReadRegister()` |
 * | 读通道原始值 | 先读 DATAx_MSB，再读 DATAx_LSB | 8 bit | 28 bit 有效值 | `LDC1614_ReadChannelRaw()` |
 *
 * 常用寄存器效果：
 * | 寄存器 | 作用 | 当前使用方式 |
 * | --- | --- | --- |
 * | `RCOUNTx` | 设置通道转换积分窗口，影响分辨率和转换时间 | CH0/CH1 使用同一默认值 |
 * | `SETTLECOUNTx` | 设置传感线圈稳定等待时间 | CH0/CH1 使用同一默认值 |
 * | `CLOCK_DIVIDERSx` | 设置传感器时钟分频 | CH0/CH1 使用同一默认值 |
 * | `DRIVE_CURRENTx` | 设置线圈驱动电流 | CH0/CH1 使用同一默认值 |
 * | `MUX_CONFIG` | 设置通道扫描模式 | 当前配置为 CH0/CH1 顺序扫描 |
 * | `ERROR_CONFIG` | 设置 DRDY/错误输出行为 | 打开 DRDY 到 INTB，供任务等待新数据 |
 * | `CONFIG` | 设置连续转换、INTB 等全局行为 | 初始化最后写入，使芯片进入连续转换 |
 * | `STATUS` | 查询数据就绪与错误状态 | `LDC1614_HasUnreadConversion()` 按通道解析 |
 *
 * 维护要求：
 * 1. 新增寄存器或改变默认值时，必须在本速查表写清楚寄存器用途、影响和上层可观察效果；
 * 2. 本文件只返回 `LDC1614_Status_t`，不直接串口打印，用户可见日志统一放在 `ldc1614_service.c`；
 * 3. I2C 失败只转换为状态码，由应用任务决定是否重试、报警或停止检测。
 */

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
 * @brief LDC1614 I2C DMA 完成信号量的静态控制块。
 *
 * 驱动只在 LDC1614 服务任务上下文中发起 I2C2 DMA 传输，DMA/I2C 中断回调负责释放该信号量。
 * 使用静态信号量可以避免运行期堆分配失败，也便于在比赛调试时确认内存所有权。
 */
static StaticSemaphore_t g_ldc1614_i2c_dma_semaphore_buffer;

/**
 * @brief LDC1614 I2C DMA 完成信号量句柄。
 *
 * 该信号量只表示“一次寄存器 DMA 读写已经结束或出错”，不能与 INTB 数据就绪任务通知混用。
 */
static SemaphoreHandle_t g_ldc1614_i2c_dma_semaphore = NULL;

/**
 * @brief 当前是否存在由本驱动发起、尚未结束的 I2C2 DMA 传输。
 *
 * HAL I2C 回调是全局弱符号覆盖入口，因此必须用该标志过滤掉非本驱动或已超时清理的回调。
 */
static volatile uint8_t g_ldc1614_i2c_dma_active = 0U;

/**
 * @brief 最近一次 I2C2 DMA 传输是否由错误或中止路径结束。
 *
 * 该变量由 I2C/DMA 中断回调写入，由任务上下文在等待结束后读取，因此使用 volatile 防止编译器缓存。
 */
static volatile uint8_t g_ldc1614_i2c_dma_error = 0U;

/**
 * @brief 最近一次 I2C2 DMA 错误码快照。
 *
 * 错误回调里立即读取 HAL 错误码，任务上下文据此把 HAL 错误转换成 LDC1614 驱动状态码。
 */
static volatile uint32_t g_ldc1614_i2c_dma_hal_error = HAL_I2C_ERROR_NONE;

/**
 * @brief 当前正在等待 DMA 回调的 I2C 句柄。
 *
 * 回调通过句柄匹配确认事件确实属于本次 LDC1614 I2C2 传输，避免未来增加其它 I2C 设备时串扰。
 */
static I2C_HandleTypeDef *g_ldc1614_i2c_dma_active_handle = NULL;

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
 * @brief 把 HAL I2C 错误码转换为驱动层状态码。
 * @param error_code `HAL_I2C_GetError()` 返回的错误位图。
 * @return LDC1614_Status_t 转换后的驱动状态。
 *
 * 主要流程：
 * 1. 无错误返回 OK；
 * 2. HAL 明确标记超时时返回 TIMEOUT；
 * 3. 其它 NACK、总线错误、DMA 错误统一返回 ERROR，由应用层按普通 I2C 故障处理。
 */
static LDC1614_Status_t LDC1614_ConvertHalErrorCode(uint32_t error_code)
{
    if (error_code == HAL_I2C_ERROR_NONE)
    {
        return LDC1614_STATUS_OK;
    }

    if ((error_code & HAL_I2C_ERROR_TIMEOUT) != 0U)
    {
        return LDC1614_STATUS_TIMEOUT;
    }

    return LDC1614_STATUS_ERROR;
}

/**
 * @brief 确保 I2C DMA 完成信号量已经创建并处于空状态。
 * @return LDC1614_Status_t `OK` 表示信号量可用，否则表示 RTOS 对象创建失败。
 *
 * 该函数在任务上下文调用。每次新传输前都会清掉可能残留的完成信号，
 * 避免上一次超时后迟到的回调让下一次 DMA 等待误判为已经完成。
 */
static LDC1614_Status_t LDC1614_PrepareDmaSemaphore(void)
{
    if (g_ldc1614_i2c_dma_semaphore == NULL)
    {
        g_ldc1614_i2c_dma_semaphore =
            xSemaphoreCreateBinaryStatic(&g_ldc1614_i2c_dma_semaphore_buffer);
        if (g_ldc1614_i2c_dma_semaphore == NULL)
        {
            return LDC1614_STATUS_ERROR;
        }
    }

    while (xSemaphoreTake(g_ldc1614_i2c_dma_semaphore, 0U) == pdTRUE)
    {
        /* 清理迟到回调留下的完成信号，保证本次等待只对应本次 DMA 传输。 */
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 标记一次新的 LDC1614 I2C DMA 传输即将开始。
 * @param hi2c 当前使用的 I2C 句柄，不能为空。
 * @return LDC1614_Status_t `OK` 表示内部等待状态已经准备好。
 *
 * 该函数只准备软件状态，不启动硬件 DMA。真正的 DMA 由调用者随后调用 HAL DMA API 发起。
 */
static LDC1614_Status_t LDC1614_BeginDmaTransfer(I2C_HandleTypeDef *hi2c)
{
    LDC1614_Status_t status;

    if (hi2c == NULL)
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    status = LDC1614_PrepareDmaSemaphore();
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    g_ldc1614_i2c_dma_hal_error = HAL_I2C_ERROR_NONE;
    g_ldc1614_i2c_dma_error = 0U;
    g_ldc1614_i2c_dma_active_handle = hi2c;
    g_ldc1614_i2c_dma_active = 1U;

    return LDC1614_STATUS_OK;
}

/**
 * @brief 结束当前 DMA 等待状态。
 *
 * 该函数只清理本驱动的等待标志，不直接修改 HAL 状态机。
 * 超时场景下 HAL 后续可能还会产生迟到回调，回调会因为 active 已清零而被忽略。
 */
static void LDC1614_ClearDmaTransferState(void)
{
    g_ldc1614_i2c_dma_active = 0U;
    g_ldc1614_i2c_dma_active_handle = NULL;
}

/**
 * @brief 等待当前寄存器 DMA 传输完成。
 * @param hi2c 当前使用的 I2C 句柄，不能为空。
 * @param device_address HAL 使用的左移后 LDC1614 器件地址，超时时用于请求中止当前主机事务。
 * @return LDC1614_Status_t DMA 传输结果。
 *
 * 主要流程：
 * 1. 任务阻塞等待 DMA/I2C 回调释放信号量，期间 CPU 可调度其它任务；
 * 2. 等到信号后检查回调记录的 HAL 错误码；
 * 3. 超时时尝试触发 HAL I2C Abort，避免 I2C 状态机长期停在 BUSY。
 */
static LDC1614_Status_t LDC1614_WaitDmaTransfer(I2C_HandleTypeDef *hi2c, uint16_t device_address)
{
    uint32_t error_code;

    if ((hi2c == NULL) || (g_ldc1614_i2c_dma_semaphore == NULL))
    {
        LDC1614_ClearDmaTransferState();
        return LDC1614_STATUS_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_ldc1614_i2c_dma_semaphore,
                       pdMS_TO_TICKS(LDC1614_I2C_TIMEOUT_MS)) != pdTRUE)
    {
        /*
         * DMA 完成回调长时间未到，说明总线可能被拉住或 HAL 状态机未收到预期事件。
         * 这里在任务上下文发起中止请求，后续迟到的中止回调会被 active 标志过滤掉。
         */
        LDC1614_ClearDmaTransferState();
        (void)HAL_I2C_Master_Abort_IT(hi2c, device_address);
        return LDC1614_STATUS_TIMEOUT;
    }

    error_code = (uint32_t)g_ldc1614_i2c_dma_hal_error;
    if (g_ldc1614_i2c_dma_error == 0U)
    {
        error_code = HAL_I2C_GetError(hi2c);
    }

    LDC1614_ClearDmaTransferState();
    return LDC1614_ConvertHalErrorCode(error_code);
}

/**
 * @brief 从 HAL I2C 回调中通知任务当前 DMA 传输已经结束。
 * @param hi2c 触发回调的 I2C 句柄。
 * @param is_error 1 表示错误/中止路径，0 表示正常完成路径。
 *
 * 该函数运行在中断上下文，只做错误码快照和信号量释放。
 * 不允许在这里访问 LDC1614 状态机或串口打印，避免 ISR 阻塞和共享状态竞态。
 */
static void LDC1614_NotifyDmaTransferFromIsr(I2C_HandleTypeDef *hi2c, uint8_t is_error)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((g_ldc1614_i2c_dma_active == 0U) ||
        (g_ldc1614_i2c_dma_active_handle != hi2c) ||
        (g_ldc1614_i2c_dma_semaphore == NULL))
    {
        return;
    }

    if (is_error != 0U)
    {
        g_ldc1614_i2c_dma_error = 1U;
        g_ldc1614_i2c_dma_hal_error = HAL_I2C_GetError(hi2c);
    }
    else
    {
        g_ldc1614_i2c_dma_error = 0U;
        g_ldc1614_i2c_dma_hal_error = HAL_I2C_ERROR_NONE;
    }

    (void)xSemaphoreGiveFromISR(g_ldc1614_i2c_dma_semaphore, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
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
    LDC1614_Status_t status;
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[2];

    if ((ldc == NULL) || (ldc->hi2c == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    tx_buffer[0] = (uint8_t)((value >> 8) & 0xFFU);
    tx_buffer[1] = (uint8_t)(value & 0xFFU);

    status = LDC1614_BeginDmaTransfer(ldc->hi2c);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    /*
     * tx_buffer 是栈上缓冲区，但本函数会阻塞等待 DMA 完成后才返回，
     * 因此 DMA 传输期间缓冲区生命周期始终有效。
     */
    hal_status = HAL_I2C_Mem_Write_DMA(ldc->hi2c,
                                       ldc->device_address,
                                       register_address,
                                       I2C_MEMADD_SIZE_8BIT,
                                       tx_buffer,
                                       sizeof(tx_buffer));
    if (hal_status != HAL_OK)
    {
        LDC1614_ClearDmaTransferState();
        return LDC1614_ConvertHalStatus(hal_status);
    }

    return LDC1614_WaitDmaTransfer(ldc->hi2c, ldc->device_address);
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
    LDC1614_Status_t status;
    HAL_StatusTypeDef hal_status;
    uint8_t rx_buffer[2];

    if ((ldc == NULL) || (ldc->hi2c == NULL) || (value == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    status = LDC1614_BeginDmaTransfer(ldc->hi2c);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
    }

    /*
     * rx_buffer 在等待完成前不会离开作用域，DMA 写入结束后再组合大端寄存器值。
     */
    hal_status = HAL_I2C_Mem_Read_DMA(ldc->hi2c,
                                      ldc->device_address,
                                      register_address,
                                      I2C_MEMADD_SIZE_8BIT,
                                      rx_buffer,
                                      sizeof(rx_buffer));
    if (hal_status != HAL_OK)
    {
        LDC1614_ClearDmaTransferState();
        return LDC1614_ConvertHalStatus(hal_status);
    }

    status = LDC1614_WaitDmaTransfer(ldc->hi2c, ldc->device_address);
    if (status != LDC1614_STATUS_OK)
    {
        return status;
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

/**
 * @brief HAL I2C 内存写 DMA 完成回调。
 * @param hi2c 触发完成事件的 I2C 句柄。
 *
 * 触发来源：
 * - `HAL_I2C_Mem_Write_DMA()` 完成 LDC1614 16bit 寄存器写入后由 HAL 在中断上下文调用。
 *
 * 处理原则：
 * - 只通知等待中的任务，不做寄存器访问、状态机推进或串口打印，避免中断上下文阻塞。
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    LDC1614_NotifyDmaTransferFromIsr(hi2c, 0U);
}

/**
 * @brief HAL I2C 内存读 DMA 完成回调。
 * @param hi2c 触发完成事件的 I2C 句柄。
 *
 * 触发来源：
 * - `HAL_I2C_Mem_Read_DMA()` 完成 LDC1614 寄存器读取后由 HAL 在中断上下文调用。
 *
 * 处理原则：
 * - 只释放 DMA 完成信号量，实际字节组合和状态判断仍在任务上下文完成。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    LDC1614_NotifyDmaTransferFromIsr(hi2c, 0U);
}

/**
 * @brief HAL I2C 错误回调。
 * @param hi2c 触发错误事件的 I2C 句柄。
 *
 * 触发来源：
 * - I2C2 事件/错误中断或 DMA 错误路径检测到总线错误、NACK、超时等异常。
 *
 * 处理原则：
 * - 快照 HAL 错误码并唤醒等待任务，由 `LDC1614_WaitDmaTransfer()` 统一转换为驱动状态码。
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    LDC1614_NotifyDmaTransferFromIsr(hi2c, 1U);
}

/**
 * @brief HAL I2C 中止完成回调。
 * @param hi2c 触发中止完成事件的 I2C 句柄。
 *
 * 触发来源：
 * - DMA 等待超时后任务上下文调用 `HAL_I2C_Master_Abort_IT()`，HAL 在中止完成时回调这里。
 *
 * 处理原则：
 * - 将中止视为错误完成，避免等待任务在异常路径中永久阻塞。
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    LDC1614_NotifyDmaTransferFromIsr(hi2c, 1U);
}
