#include "ldc1614_service.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "ldc1614.h"
#include "task.h"
#include "uart_command.h"
#include "usart.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 等待 LDC1614 INTB 事件的最长时间，单位毫秒。
 *
 * 该超时只是为了防止任务永久阻塞，不代表超时一定是硬件故障。
 */
#define LDC1614_SERVICE_INTB_WAIT_TIMEOUT_MS         (200U)

/**
 * @brief 建立空载基线时，每个通道需要采集的样本数。
 */
#define LDC1614_SERVICE_BASELINE_SAMPLES             (24U)

/**
 * @brief 中值滤波窗口长度。
 *
 * LDC1614 原始值在放置工件和手部离开过程中会有瞬态毛刺，
 * 先做一个小窗口中值滤波，可以降低误判概率。
 */
#define LDC1614_SERVICE_FILTER_SIZE                  (5U)

/**
 * @brief 触发“检测到工件进入”所需的连续样本数。
 */
#define LDC1614_SERVICE_DETECT_CONFIRM_COUNT         (3U)

/**
 * @brief 触发“工件已移开”所需的连续样本数。
 */
#define LDC1614_SERVICE_RELEASE_CONFIRM_COUNT        (5U)
#define LDC1614_SERVICE_STARTUP_ARM_CONFIRM_COUNT    (6U)

/**
 * @brief 工件放上去后额外等待的稳定时间，单位毫秒。
 *
 * 这段时间用于避开“手还没有完全离开”“夹具还在晃动”等干扰。
 */
#define LDC1614_SERVICE_SETTLE_DELAY_MS              (3000U)

/**
 * @brief 进入测量窗口后，需要累计的稳定样本数。
 */
#define LDC1614_SERVICE_MEASURE_SAMPLE_COUNT         (12U)

/**
 * @brief 稳定窗口允许的最小波动带宽下限。
 *
 * 如果测量窗口内的最大值和最小值差异过大，说明工件仍在晃动，
 * 本轮结果不可信，需要重新回到“等待放稳”状态。
 */
#define LDC1614_SERVICE_MIN_MEASURE_SPREAD_LIMIT     (3000U)

/**
 * @brief 自动检测阈值的下限，单位为原始计数。
 */
#define LDC1614_SERVICE_MIN_DETECT_THRESHOLD         (1500U)

/**
 * @brief 自动释放阈值的下限，单位为原始计数。
 */
#define LDC1614_SERVICE_MIN_RELEASE_THRESHOLD        (800U)

/**
 * @brief 检测阈值与噪声估计的倍率关系。
 */
#define LDC1614_SERVICE_DETECT_NOISE_MULTIPLIER      (8U)

/**
 * @brief 释放阈值与噪声估计的倍率关系。
 */
#define LDC1614_SERVICE_RELEASE_NOISE_MULTIPLIER     (4U)

/**
 * @brief 结果上报后，等待移除时使用的“相对释放阈值”分母。
 *
 * 双通道线圈在实际安装后可能存在耦合。
 * 某一路结果上报后，如果还要求它必须严格回到接近空载基线的位置，
 * 就可能长期卡在 WAIT_REMOVE，导致后续再也不触发。
 * 因此这里增加一个补充条件：当当前变化量已经衰减到
 * “上次稳定平台变化量”的 1/N 以下时，也认为该通道已经基本移开。
 */
#define LDC1614_SERVICE_REMOVE_RATIO_DIVISOR         (8U)

/**
 * @brief 通道检测方向为“原始值上升”。
 */
#define LDC1614_SERVICE_DIRECTION_RISE               (1)

/**
 * @brief 通道检测方向为“原始值下降”。
 */
#define LDC1614_SERVICE_DIRECTION_FALL               (-1)

/**
 * @brief 空闲态下基线缓慢跟踪使用的移位系数。
 */
#define LDC1614_SERVICE_BASELINE_TRACK_SHIFT         (4U)

/**
 * @brief 本项目当前启用的检测通道数量。
 */
#define LDC1614_SERVICE_CHANNEL_COUNT                (2U)

/**
 * @brief 一次 INTB 唤醒后，最多连续把挂起转换结果读出的轮数。
 *
 * 某些情况下 INTB 拉低后，任务读取 STATUS 的同时下一路转换也可能刚好完成。
 * 这里做有限次“排空”读取，提升双通道连续采样的鲁棒性。
 */
#define LDC1614_SERVICE_MAX_DRAIN_ROUNDS            (4U)

/**
 * @brief 标定采样模式的默认采样次数。
 *
 * 对重复性评估而言，20 次足够初步观察离散程度，
 * 又不会让一次人工操作持续太久。
 */
#define LDC1614_SERVICE_CAL_DEFAULT_COUNT            (20U)

/**
 * @brief 标定采样模式允许的最大采样次数。
 *
 * 限制上限可以避免误输入过大数字后，任务长时间停在批量采样会话里。
 */
#define LDC1614_SERVICE_CAL_MAX_COUNT                (50U)

/**
 * @brief 通道 1 是否已经配置“标准良品参考值”。
 *
 * 设为 0 时，本通道会输出稳定测量值，但不直接下 OK/DEFECT 结论。
 */
#define LDC1614_SERVICE_REF_ENABLED_CH1              (0U)

/**
 * @brief 通道 2 是否已经配置“标准良品参考值”。
 */
#define LDC1614_SERVICE_REF_ENABLED_CH2              (0U)

/**
 * @brief 通道 1 的标准良品参考变化量。
 *
 * 该值表示“标准件放稳后，相对空载基线的稳定变化量”。
 */
#define LDC1614_SERVICE_REF_DELTA_CH1                (0UL)

/**
 * @brief 通道 2 的标准良品参考变化量。
 */
#define LDC1614_SERVICE_REF_DELTA_CH2                (0UL)

/**
 * @brief 通道 1 判定为合格时允许的偏差范围。
 */
#define LDC1614_SERVICE_DEFECT_TOLERANCE_CH1         (3000UL)

/**
 * @brief 通道 2 判定为合格时允许的偏差范围。
 */
#define LDC1614_SERVICE_DEFECT_TOLERANCE_CH2         (3000UL)

/**
 * @brief LDC1614 INTB 引脚端口。
 */
#define LDC1614_SERVICE_INTB_GPIO_PORT               LDC1614_INTB_GPIO_Port

/**
 * @brief LDC1614 INTB 引脚编号。
 */
#define LDC1614_SERVICE_INTB_GPIO_PIN                LDC1614_INTB_Pin

/**
 * @brief LDC1614 通道运行状态。
 *
 * - IDLE：空闲等待工件进入；
 * - SETTLING：已经检测到工件进入，但还在等待机械动作结束；
 * - MEASURING：开始采集稳定窗口；
 * - WAIT_REMOVE：结果已经上报，等待工件完全移开后重新武装。
 */
typedef enum
{
    LDC1614_SERVICE_STATE_IDLE = 0,
    LDC1614_SERVICE_STATE_SETTLING,
    LDC1614_SERVICE_STATE_MEASURING,
    LDC1614_SERVICE_STATE_WAIT_REMOVE
} LDC1614_ServiceState_t;

/**
 * @brief 小窗口中值滤波器。
 */
typedef struct
{
    uint32_t samples[LDC1614_SERVICE_FILTER_SIZE];
    uint8_t sample_count;
    uint8_t write_index;
} LDC1614_Filter_t;

/**
 * @brief 单个检测通道的运行时上下文。
 *
 * 每个通道独立维护：
 * 1. 基线与阈值；
 * 2. 放稳等待状态；
 * 3. 稳定测量窗口；
 * 4. 结果判定参数。
 *
 * 这样即使双通道同时工作，也不会互相污染状态。
 */
typedef struct
{
    LDC1614_Channel_t channel;
    const char *label;
    LDC1614_Filter_t filter;
    LDC1614_ServiceState_t state;
    uint32_t baseline;
    uint32_t detect_threshold;
    uint32_t release_threshold;
    uint32_t latest_filtered_sample;
    uint32_t reference_delta;
    uint32_t defect_tolerance;
    uint32_t last_stable_delta;
    uint64_t measure_sum;
    uint32_t measure_min;
    uint32_t measure_max;
    TickType_t settle_deadline_tick;
    uint16_t measure_count;
    uint8_t reference_enabled;
    int8_t detect_direction;
    uint8_t detect_confirm_count;
    uint8_t release_confirm_count;
    uint8_t detect_armed;
    uint8_t startup_quiet_count;
} LDC1614_ChannelContext_t;

/**
 * @brief LDC 标定采样会话状态。
 *
 * 该结构描述的是“某个通道正在执行一次单件标定采样”：
 * 1. 工件放上去并放稳后；
 * 2. 连续采集 N 个稳定样本；
 * 3. 输出均值、最小值、最大值与建议参考值。
 */
typedef struct
{
    uint8_t active;
    uint8_t target_channel_index;
    uint16_t target_sample_count;
    uint16_t captured_sample_count;
    uint32_t min_raw;
    uint32_t max_raw;
    uint64_t raw_sum;
    uint32_t min_delta;
    uint32_t max_delta;
    uint64_t delta_sum;
} LDC1614_CalibrationSession_t;

/**
 * @brief LDC1614 任务句柄。
 *
 * EXTI 回调只负责通知该任务，不在中断里做 I2C 访问。
 */
static TaskHandle_t g_ldc1614_task_handle = NULL;

/**
 * @brief 当前标定采样会话。
 *
 * 该对象由串口命令入口发起，由 LDC 任务在测量结果产生时推进。
 * 访问时通过临界区保护，避免两个任务同时修改。
 */
static LDC1614_CalibrationSession_t g_ldc1614_calibration_session = {0};

/**
 * @brief 当前启用的底层通道映射。
 */
static const LDC1614_Channel_t g_ldc1614_channels[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    LDC1614_CHANNEL_0,
    LDC1614_CHANNEL_1
};

/**
 * @brief 当前启用通道对应的人类可读标签。
 *
 * 日志里把逻辑名称和底层 CH 编号一起输出，
 * 便于后续排查“上/下线圈到底对应哪一路”。
 */
static const char *g_ldc1614_channel_labels[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    "Channel 1(CH0)",
    "Channel 2(CH1)"
};

/**
 * @brief 默认参考值是否有效。
 */
static const uint8_t g_ldc1614_reference_enabled[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    LDC1614_SERVICE_REF_ENABLED_CH1,
    LDC1614_SERVICE_REF_ENABLED_CH2
};

/**
 * @brief 默认参考变化量。
 */
static const uint32_t g_ldc1614_reference_delta[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    LDC1614_SERVICE_REF_DELTA_CH1,
    LDC1614_SERVICE_REF_DELTA_CH2
};

/**
 * @brief 默认缺陷容差。
 */
static const uint32_t g_ldc1614_defect_tolerance[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    LDC1614_SERVICE_DEFECT_TOLERANCE_CH1,
    LDC1614_SERVICE_DEFECT_TOLERANCE_CH2
};

/**
 * @brief 每个通道各自的检测方向。
 *
 * 当前根据你的最新板上实测先配置为：
 * - CH0：放上工件后原始值上升；
 * - CH1：放上工件后原始值也按上升处理。
 *
 * 如果后续再次实测发现方向相反，只需要改这里，不必重写状态机。
 */
static const int8_t g_ldc1614_detect_direction[LDC1614_SERVICE_CHANNEL_COUNT] =
{
    LDC1614_SERVICE_DIRECTION_RISE,
    LDC1614_SERVICE_DIRECTION_RISE
};

/**
 * @brief 清空滤波器状态。
 * @param filter 滤波器对象指针，不能为空。
 */
static void Ldc1614Service_FilterReset(LDC1614_Filter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    (void)memset(filter->samples, 0, sizeof(filter->samples));
    filter->sample_count = 0U;
    filter->write_index = 0U;
}

/**
 * @brief 对一个小数组做原地升序排序。
 * @param values 待排序数组。
 * @param length 有效长度。
 */
static void Ldc1614Service_SortAscending(uint32_t *values, uint8_t length)
{
    uint8_t outer_index;

    if ((values == NULL) || (length < 2U))
    {
        return;
    }

    for (outer_index = 1U; outer_index < length; ++outer_index)
    {
        uint32_t key_value;
        int32_t inner_index;

        key_value = values[outer_index];
        inner_index = (int32_t)outer_index - 1;

        while ((inner_index >= 0) && (values[inner_index] > key_value))
        {
            values[inner_index + 1] = values[inner_index];
            --inner_index;
        }

        values[inner_index + 1] = key_value;
    }
}

/**
 * @brief 推入一个新样本并返回当前中值滤波结果。
 * @param filter 滤波器对象指针，不能为空。
 * @param new_sample 新采样值。
 * @return uint32_t 当前滤波结果。
 */
static uint32_t Ldc1614Service_FilterPush(LDC1614_Filter_t *filter, uint32_t new_sample)
{
    uint32_t sorted_samples[LDC1614_SERVICE_FILTER_SIZE];
    uint8_t index;
    uint8_t valid_count;

    if (filter == NULL)
    {
        return new_sample;
    }

    filter->samples[filter->write_index] = new_sample;
    filter->write_index = (uint8_t)((filter->write_index + 1U) % LDC1614_SERVICE_FILTER_SIZE);

    if (filter->sample_count < LDC1614_SERVICE_FILTER_SIZE)
    {
        ++filter->sample_count;
    }

    valid_count = filter->sample_count;
    for (index = 0U; index < valid_count; ++index)
    {
        sorted_samples[index] = filter->samples[index];
    }

    Ldc1614Service_SortAscending(sorted_samples, valid_count);
    return sorted_samples[valid_count / 2U];
}

/**
 * @brief 计算两个无符号采样值之间的绝对差。
 * @param left 左操作数。
 * @param right 右操作数。
 * @return uint32_t 差值绝对值。
 */
static uint32_t Ldc1614Service_AbsoluteDifference(uint32_t left, uint32_t right)
{
    return (left >= right) ? (left - right) : (right - left);
}

/**
 * @brief 计算“样本值相对基线”的有符号差值。
 * @param sample 当前样本值。
 * @param baseline 当前基线值。
 * @return int32_t 有符号差值，正数表示样本高于基线，负数表示低于基线。
 */
static int32_t Ldc1614Service_GetSignedDelta(uint32_t sample, uint32_t baseline)
{
    return (int32_t)sample - (int32_t)baseline;
}

/**
 * @brief 计算“按通道方向解释后”的有符号变化量。
 * @param channel_context 通道上下文指针，不能为空。
 * @param sample 当前样本值。
 * @return int32_t 方向化后的变化量。
 *
 * 说明：
 * - 对“rise”通道：样本高于基线越多，结果越大；
 * - 对“fall”通道：样本低于基线越多，结果越大。
 *
 * 这样同一套状态机就能同时兼容“放上去值升高”和“放上去值降低”两种线圈响应。
 */
static int32_t Ldc1614Service_GetDirectionalDelta(const LDC1614_ChannelContext_t *channel_context,
                                                  uint32_t sample)
{
    int32_t signed_delta;

    if (channel_context == NULL)
    {
        return 0;
    }

    signed_delta = Ldc1614Service_GetSignedDelta(sample, channel_context->baseline);
    /*
     * 不再依赖固定的 rise/fall 方向。
     * 实测中一旦方向配置与真实线圈响应方向相反，
     * IDLE 状态下的基线会在“第一次放上去”时被慢慢带偏，
     * 随后就会出现“放上去显示离开、拿开反而检测到”的反向现象。
     *
     * 这里统一返回相对基线的绝对变化量，
     * 让状态机只关心“偏离了多少”，不关心“往上还是往下偏”。
     */
    if (signed_delta < 0)
    {
        signed_delta = -signed_delta;
    }

    return signed_delta;
}

/**
 * @brief 返回当前通道的方向名称。
 * @param detect_direction 通道方向配置值。
 * @return const char* `"rise"` 或 `"fall"`。
 */
/**
 * @brief 复位一次标定采样会话。
 * @param session 会话对象指针，不能为空。
 *
 * 该函数会清空单次标定采样计数与统计结果。
 * 当用户重新发起新会话或主动停止会话时，都应走这里。
 */
static void Ldc1614Service_ResetCalibrationSession(LDC1614_CalibrationSession_t *session)
{
    if (session == NULL)
    {
        return;
    }

    session->active = 0U;
    session->target_channel_index = 0U;
    session->target_sample_count = 0U;
    session->captured_sample_count = 0U;
    session->min_raw = UINT32_MAX;
    session->max_raw = 0U;
    session->raw_sum = 0U;
    session->min_delta = UINT32_MAX;
    session->max_delta = 0U;
    session->delta_sum = 0U;
}

/**
 * @brief 启动一轮新的标定采样会话。
 * @param channel_index 目标通道下标，0 表示 CH0，1 表示 CH1。
 * @param sample_count 目标采样次数。
 *
 * 这里采用“覆盖旧会话”的策略：
 * 如果用户再次发送 `LDCCAL`，则以最新命令为准。
 */
static void Ldc1614Service_StartCalibrationSession(uint8_t channel_index, uint16_t sample_count)
{
    taskENTER_CRITICAL();

    Ldc1614Service_ResetCalibrationSession(&g_ldc1614_calibration_session);
    g_ldc1614_calibration_session.active = 1U;
    g_ldc1614_calibration_session.target_channel_index = channel_index;
    g_ldc1614_calibration_session.target_sample_count = sample_count;
    g_ldc1614_calibration_session.min_raw = UINT32_MAX;
    g_ldc1614_calibration_session.min_delta = UINT32_MAX;

    taskEXIT_CRITICAL();
}

/**
 * @brief 主动停止当前标定采样会话。
 */
static void Ldc1614Service_StopCalibrationSession(void)
{
    taskENTER_CRITICAL();
    Ldc1614Service_ResetCalibrationSession(&g_ldc1614_calibration_session);
    taskEXIT_CRITICAL();
}

/**
 * @brief 清空当前标定采样会话已经累计的样本统计，但保留会话本身激活状态。
 *
 * 当工件在采样中途被拿走，或者放稳后波动仍然过大时，
 * 应丢弃本次半成品统计，等待下一次有效放置重新开始采样。
 */
static void Ldc1614Service_ClearCalibrationProgress(void)
{
    taskENTER_CRITICAL();

    if (g_ldc1614_calibration_session.active != 0U)
    {
        g_ldc1614_calibration_session.captured_sample_count = 0U;
        g_ldc1614_calibration_session.min_raw = UINT32_MAX;
        g_ldc1614_calibration_session.max_raw = 0U;
        g_ldc1614_calibration_session.raw_sum = 0U;
        g_ldc1614_calibration_session.min_delta = UINT32_MAX;
        g_ldc1614_calibration_session.max_delta = 0U;
        g_ldc1614_calibration_session.delta_sum = 0U;
    }

    taskEXIT_CRITICAL();
}

/**
 * @brief 解析 `LDCCAL` 命令中的通道号和采样次数。
 * @param command_buffer 已经规范化后的命令字符串。
 * @param channel_index 输出的通道下标，不能为空。
 * @param sample_count 输出的采样次数，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示格式不合法。
 *
 * 支持格式：
 * 1. `LDCCAL CH1`
 * 2. `LDCCAL CH2 20`
 * 3. `LDCCAL 1 10`
 */
static uint8_t Ldc1614Service_ParseCalibrationCommand(const char *command_buffer,
                                                      uint8_t *channel_index,
                                                      uint16_t *sample_count)
{
    char *end_pointer;
    unsigned long parsed_channel;
    unsigned long parsed_count;

    if ((command_buffer == NULL) || (channel_index == NULL) || (sample_count == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "LDCCAL", 6U) != 0)
    {
        return 0U;
    }

    command_buffer += 6;
    while ((*command_buffer == ' ') || (*command_buffer == '\t'))
    {
        ++command_buffer;
    }

    if (strncmp(command_buffer, "CH", 2U) == 0)
    {
        command_buffer += 2;
    }

    parsed_channel = strtoul(command_buffer, &end_pointer, 10);
    if ((command_buffer == end_pointer) || ((parsed_channel != 1UL) && (parsed_channel != 2UL)))
    {
        return 0U;
    }

    *channel_index = (uint8_t)(parsed_channel - 1UL);

    while ((*end_pointer == ' ') || (*end_pointer == '\t'))
    {
        ++end_pointer;
    }

    if (*end_pointer == '\0')
    {
        *sample_count = LDC1614_SERVICE_CAL_DEFAULT_COUNT;
        return 1U;
    }

    parsed_count = strtoul(end_pointer, &end_pointer, 10);
    while ((*end_pointer == ' ') || (*end_pointer == '\t'))
    {
        ++end_pointer;
    }

    if ((*end_pointer != '\0') ||
        (parsed_count == 0UL) ||
        (parsed_count > (unsigned long)LDC1614_SERVICE_CAL_MAX_COUNT))
    {
        return 0U;
    }

    *sample_count = (uint16_t)parsed_count;
    return 1U;
}

/**
 * @brief 判断当前是否存在针对指定通道的标定采样会话。
 * @param channel_context 通道上下文指针，不能为空。
 * @param target_sample_count 输出目标采样数，允许为空。
 * @return uint8_t 1 表示当前通道正处于标定采样模式，0 表示不是。
 */
static uint8_t Ldc1614Service_IsCalibrationTargetChannel(const LDC1614_ChannelContext_t *channel_context,
                                                         uint16_t *target_sample_count)
{
    uint8_t is_target = 0U;

    if (channel_context == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    if ((g_ldc1614_calibration_session.active != 0U) &&
        (g_ldc1614_calibration_session.target_channel_index < LDC1614_SERVICE_CHANNEL_COUNT) &&
        (g_ldc1614_channels[g_ldc1614_calibration_session.target_channel_index] == channel_context->channel))
    {
        is_target = 1U;
        if (target_sample_count != NULL)
        {
            *target_sample_count = g_ldc1614_calibration_session.target_sample_count;
        }
    }

    taskEXIT_CRITICAL();

    return is_target;
}

/**
 * @brief 在标定采样中记录一个稳定样本。
 * @param raw_sample 当前滤波后的原始值。
 * @param delta_sample 当前方向化后的变化量。
 * @param captured_count 输出当前已记录样本数，允许为空。
 * @param target_count 输出目标样本数，允许为空。
 * @return uint8_t 1 表示当前会话已经采满，0 表示还未采满。
 */
static uint8_t Ldc1614Service_RecordCalibrationSample(uint32_t raw_sample,
                                                      uint32_t delta_sample,
                                                      uint16_t *captured_count,
                                                      uint16_t *target_count)
{
    uint8_t completed = 0U;

    taskENTER_CRITICAL();

    if (g_ldc1614_calibration_session.active != 0U)
    {
        ++g_ldc1614_calibration_session.captured_sample_count;
        g_ldc1614_calibration_session.raw_sum += raw_sample;
        g_ldc1614_calibration_session.delta_sum += delta_sample;

        if (raw_sample < g_ldc1614_calibration_session.min_raw)
        {
            g_ldc1614_calibration_session.min_raw = raw_sample;
        }
        if (raw_sample > g_ldc1614_calibration_session.max_raw)
        {
            g_ldc1614_calibration_session.max_raw = raw_sample;
        }

        if (delta_sample < g_ldc1614_calibration_session.min_delta)
        {
            g_ldc1614_calibration_session.min_delta = delta_sample;
        }
        if (delta_sample > g_ldc1614_calibration_session.max_delta)
        {
            g_ldc1614_calibration_session.max_delta = delta_sample;
        }

        if (captured_count != NULL)
        {
            *captured_count = g_ldc1614_calibration_session.captured_sample_count;
        }
        if (target_count != NULL)
        {
            *target_count = g_ldc1614_calibration_session.target_sample_count;
        }

        if (g_ldc1614_calibration_session.captured_sample_count >=
            g_ldc1614_calibration_session.target_sample_count)
        {
            completed = 1U;
        }
    }

    taskEXIT_CRITICAL();

    return completed;
}

/**
 * @brief 输出一次标定采样的统计汇总。
 * @param channel_context 通道上下文指针，不能为空。
 *
 * 汇总内容包括：
 * 1. 平均原始值；
 * 2. 平均变化量；
 * 3. 最小/最大变化量；
 * 4. 建议参考值。
 */
static void Ldc1614Service_ReportCalibrationSummary(const LDC1614_ChannelContext_t *channel_context)
{
    LDC1614_CalibrationSession_t session_snapshot;
    uint32_t average_raw;
    uint32_t average_delta;
    uint32_t delta_span;

    if (channel_context == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    session_snapshot = g_ldc1614_calibration_session;
    taskEXIT_CRITICAL();

    if ((session_snapshot.active == 0U) || (session_snapshot.captured_sample_count == 0U))
    {
        return;
    }

    average_raw = (uint32_t)(session_snapshot.raw_sum / (uint64_t)session_snapshot.captured_sample_count);
    average_delta = (uint32_t)(session_snapshot.delta_sum / (uint64_t)session_snapshot.captured_sample_count);
    delta_span = session_snapshot.max_delta - session_snapshot.min_delta;

    my_printf(&huart1,
              "[CAL][LDC] %s summary count=%u, avg_raw=%lu, avg_delta=%lu\r\n",
              channel_context->label,
              (unsigned int)session_snapshot.captured_sample_count,
              (unsigned long)average_raw,
              (unsigned long)average_delta);

    my_printf(&huart1,
              "[CAL][LDC] %s delta_min=%lu, delta_max=%lu, delta_span=%lu, suggested_reference=%lu\r\n",
              channel_context->label,
              (unsigned long)session_snapshot.min_delta,
              (unsigned long)session_snapshot.max_delta,
              (unsigned long)delta_span,
              (unsigned long)average_delta);
}

/**
 * @brief 判断 LDC1614 的 INTB 是否已经处于有效低电平。
 * @return uint8_t 1 表示当前已有待读取数据，0 表示暂无事件。
 */
static uint8_t Ldc1614Service_IsIntbAsserted(void)
{
    return (HAL_GPIO_ReadPin(LDC1614_SERVICE_INTB_GPIO_PORT,
                             LDC1614_SERVICE_INTB_GPIO_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

/**
 * @brief 等待一次 LDC1614 数据就绪事件。
 * @param timeout_ms 最长等待时间，单位毫秒。
 * @return uint8_t 1 表示等到事件，0 表示超时。
 */
static uint8_t Ldc1614Service_WaitDataReadyEvent(uint32_t timeout_ms)
{
    TickType_t wait_ticks;

    if (Ldc1614Service_IsIntbAsserted() != 0U)
    {
        return 1U;
    }

    if (g_ldc1614_task_handle == NULL)
    {
        return 0U;
    }

    wait_ticks = pdMS_TO_TICKS(timeout_ms);
    return (ulTaskNotifyTake(pdTRUE, wait_ticks) > 0U) ? 1U : 0U;
}

/**
 * @brief 在空闲态下缓慢更新基线。
 * @param baseline 当前基线值。
 * @param latest_sample 最新滤波采样值。
 * @return uint32_t 更新后的基线值。
 */
static uint32_t Ldc1614Service_UpdateBaseline(uint32_t baseline, uint32_t latest_sample)
{
    int32_t signed_delta;

    signed_delta = (int32_t)latest_sample - (int32_t)baseline;
    return (uint32_t)((int32_t)baseline + (signed_delta >> LDC1614_SERVICE_BASELINE_TRACK_SHIFT));
}

/**
 * @brief 根据启动阶段噪声水平计算检测阈值和释放阈值。
 * @param noise_estimate 启动阶段估算得到的噪声计数。
 * @param detect_threshold 输出的检测阈值，不能为空。
 * @param release_threshold 输出的释放阈值，不能为空。
 */
static void Ldc1614Service_CalculateThresholds(uint32_t noise_estimate,
                                               uint32_t *detect_threshold,
                                               uint32_t *release_threshold)
{
    uint32_t detect_value;
    uint32_t release_value;

    if ((detect_threshold == NULL) || (release_threshold == NULL))
    {
        return;
    }

    detect_value = noise_estimate * LDC1614_SERVICE_DETECT_NOISE_MULTIPLIER;
    release_value = noise_estimate * LDC1614_SERVICE_RELEASE_NOISE_MULTIPLIER;

    if (detect_value < LDC1614_SERVICE_MIN_DETECT_THRESHOLD)
    {
        detect_value = LDC1614_SERVICE_MIN_DETECT_THRESHOLD;
    }

    if (release_value < LDC1614_SERVICE_MIN_RELEASE_THRESHOLD)
    {
        release_value = LDC1614_SERVICE_MIN_RELEASE_THRESHOLD;
    }

    if (release_value >= detect_value)
    {
        release_value = detect_value / 2U;
        if (release_value < LDC1614_SERVICE_MIN_RELEASE_THRESHOLD)
        {
            release_value = LDC1614_SERVICE_MIN_RELEASE_THRESHOLD;
        }
    }

    *detect_threshold = detect_value;
    *release_threshold = release_value;
}

/**
 * @brief 返回当前通道测量窗口允许的最大波动范围。
 * @param channel_context 通道上下文指针，不能为空。
 * @return uint32_t 允许的最大波动范围。
 *
 * 阈值过小会导致正常平台值也被误认为“还没放稳”，
 * 阈值过大又会把明显晃动当成稳定结果，所以这里用释放阈值的两倍，
 * 并给一个固定下限。
 */
static uint32_t Ldc1614Service_GetMeasureSpreadLimit(const LDC1614_ChannelContext_t *channel_context)
{
    uint32_t spread_limit;

    if (channel_context == NULL)
    {
        return LDC1614_SERVICE_MIN_MEASURE_SPREAD_LIMIT;
    }

    spread_limit = channel_context->release_threshold * 2U;
    if (spread_limit < LDC1614_SERVICE_MIN_MEASURE_SPREAD_LIMIT)
    {
        spread_limit = LDC1614_SERVICE_MIN_MEASURE_SPREAD_LIMIT;
    }

    return spread_limit;
}

/**
 * @brief 获取等待移除阶段使用的动态释放阈值。
 * @param channel_context 通道上下文指针，不能为空。
 * @return uint32_t 动态释放阈值。
 *
 * 判定策略：
 * 1. 保留原来的绝对释放阈值，用于真正回到空载附近的场景；
 * 2. 再补一个与“上次稳定平台值”相关的相对阈值；
 * 3. 两者取较大值，缓解双通道耦合导致的“通道回不到 800”问题。
 */
static uint32_t Ldc1614Service_GetRemoveThreshold(const LDC1614_ChannelContext_t *channel_context)
{
    uint32_t remove_threshold;

    if (channel_context == NULL)
    {
        return LDC1614_SERVICE_MIN_RELEASE_THRESHOLD;
    }

    remove_threshold = channel_context->release_threshold;

    if (channel_context->last_stable_delta > 0U)
    {
        uint32_t ratio_threshold;

        ratio_threshold = channel_context->last_stable_delta / LDC1614_SERVICE_REMOVE_RATIO_DIVISOR;
        if (ratio_threshold > remove_threshold)
        {
            remove_threshold = ratio_threshold;
        }
    }

    return remove_threshold;
}

/**
 * @brief 清空一次稳定测量窗口的累计状态。
 * @param channel_context 通道上下文指针，不能为空。
 */
static void Ldc1614Service_ResetMeasureWindow(LDC1614_ChannelContext_t *channel_context)
{
    if (channel_context == NULL)
    {
        return;
    }

    channel_context->measure_sum = 0U;
    channel_context->measure_min = UINT32_MAX;
    channel_context->measure_max = 0U;
    channel_context->measure_count = 0U;
}

/**
 * @brief 将通道切回空闲等待状态。
 * @param channel_context 通道上下文指针，不能为空。
 * @param reset_to_latest_baseline 1 表示把当前样本重新作为空闲基线，0 表示保留现有基线。
 */
static void Ldc1614Service_EnterIdle(LDC1614_ChannelContext_t *channel_context,
                                     uint8_t reset_to_latest_baseline)
{
    if (channel_context == NULL)
    {
        return;
    }

    channel_context->state = LDC1614_SERVICE_STATE_IDLE;
    channel_context->detect_confirm_count = 0U;
    channel_context->release_confirm_count = 0U;
    channel_context->settle_deadline_tick = 0U;
    Ldc1614Service_ResetMeasureWindow(channel_context);

    if (reset_to_latest_baseline != 0U)
    {
        channel_context->baseline = channel_context->latest_filtered_sample;
        channel_context->detect_armed = 1U;
        channel_context->startup_quiet_count = 0U;
    }
}

/**
 * @brief 让通道进入“等待放稳”状态。
 * @param channel_context 通道上下文指针，不能为空。
 *
 * 这里不立刻判缺陷，而是先等固定时间，让工件和夹具动作完全结束。
 */
static void Ldc1614Service_EnterSettling(LDC1614_ChannelContext_t *channel_context)
{
    if (channel_context == NULL)
    {
        return;
    }

    channel_context->state = LDC1614_SERVICE_STATE_SETTLING;
    channel_context->detect_confirm_count = 0U;
    channel_context->release_confirm_count = 0U;
    channel_context->settle_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(LDC1614_SERVICE_SETTLE_DELAY_MS);
    Ldc1614Service_ResetMeasureWindow(channel_context);
}

/**
 * @brief 让通道进入稳定测量窗口。
 * @param channel_context 通道上下文指针，不能为空。
 */
static void Ldc1614Service_EnterMeasuring(LDC1614_ChannelContext_t *channel_context)
{
    if (channel_context == NULL)
    {
        return;
    }

    channel_context->state = LDC1614_SERVICE_STATE_MEASURING;
    channel_context->release_confirm_count = 0U;
    Ldc1614Service_ResetMeasureWindow(channel_context);
}

/**
 * @brief 初始化所有通道上下文。
 * @param channel_contexts 通道上下文数组，不能为空。
 * @param channel_count 通道数量。
 */
static void Ldc1614Service_InitChannelContexts(LDC1614_ChannelContext_t *channel_contexts,
                                               uint8_t channel_count)
{
    uint8_t index;

    if (channel_contexts == NULL)
    {
        return;
    }

    for (index = 0U; index < channel_count; ++index)
    {
        channel_contexts[index].channel = g_ldc1614_channels[index];
        channel_contexts[index].label = g_ldc1614_channel_labels[index];
        channel_contexts[index].state = LDC1614_SERVICE_STATE_IDLE;
        channel_contexts[index].baseline = 0U;
        channel_contexts[index].detect_threshold = 0U;
        channel_contexts[index].release_threshold = 0U;
        channel_contexts[index].latest_filtered_sample = 0U;
        channel_contexts[index].reference_enabled = g_ldc1614_reference_enabled[index];
        channel_contexts[index].reference_delta = g_ldc1614_reference_delta[index];
        channel_contexts[index].defect_tolerance = g_ldc1614_defect_tolerance[index];
        channel_contexts[index].detect_direction = g_ldc1614_detect_direction[index];
        channel_contexts[index].last_stable_delta = 0U;
        channel_contexts[index].measure_sum = 0U;
        channel_contexts[index].measure_min = UINT32_MAX;
        channel_contexts[index].measure_max = 0U;
        channel_contexts[index].settle_deadline_tick = 0U;
        channel_contexts[index].measure_count = 0U;
        channel_contexts[index].detect_confirm_count = 0U;
        channel_contexts[index].release_confirm_count = 0U;
        channel_contexts[index].detect_armed = 0U;
        channel_contexts[index].startup_quiet_count = 0U;
        Ldc1614Service_FilterReset(&channel_contexts[index].filter);
    }
}

/**
 * @brief 读取一次中断唤醒后所有已经准备好的通道样本。
 * @param ldc LDC1614 驱动句柄，不能为空。
 * @param channel_contexts 通道上下文数组，不能为空。
 * @param channel_count 通道数量。
 * @param updated_mask 用于返回本次更新到的通道位图，不能为空。
 * @return LDC1614_Status_t 读取结果。
 *
 * 设计原则：
 * 1. 每次先等一次 INTB；
 * 2. 醒来后尽量把当前已经挂起的转换结果读干净；
 * 3. 避免双通道扫描时只读到一路、另一路因为时序边界被漏掉。
 */
static LDC1614_Status_t Ldc1614Service_ReadReadyChannels(LDC1614_Handle_t *ldc,
                                                         LDC1614_ChannelContext_t *channel_contexts,
                                                         uint8_t channel_count,
                                                         uint32_t *updated_mask)
{
    LDC1614_Status_t status;
    uint8_t round_index;

    if ((ldc == NULL) || (channel_contexts == NULL) || (updated_mask == NULL))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    *updated_mask = 0U;

    if (Ldc1614Service_WaitDataReadyEvent(LDC1614_SERVICE_INTB_WAIT_TIMEOUT_MS) == 0U)
    {
        return LDC1614_STATUS_NOT_READY;
    }

    for (round_index = 0U; round_index < LDC1614_SERVICE_MAX_DRAIN_ROUNDS; ++round_index)
    {
        uint16_t status_register;
        uint8_t channel_index;
        uint8_t round_has_update = 0U;

        status = LDC1614_ReadStatus(ldc, &status_register);
        if (status != LDC1614_STATUS_OK)
        {
            return status;
        }

        for (channel_index = 0U; channel_index < channel_count; ++channel_index)
        {
            uint32_t raw_sample;

            if (LDC1614_HasUnreadConversion(status_register, channel_contexts[channel_index].channel) == 0U)
            {
                continue;
            }

            status = LDC1614_ReadChannelRaw(ldc, channel_contexts[channel_index].channel, &raw_sample);
            if (status != LDC1614_STATUS_OK)
            {
                return status;
            }

            channel_contexts[channel_index].latest_filtered_sample =
                Ldc1614Service_FilterPush(&channel_contexts[channel_index].filter, raw_sample);

            *updated_mask |= (1UL << channel_index);
            round_has_update = 1U;
        }

        /*
         * 如果这一轮没有读到新样本，说明当前挂起数据已经处理完毕。
         * 这里直接结束，避免做无意义的 I2C 轮询。
         */
        if (round_has_update == 0U)
        {
            break;
        }

        /*
         * INTB 已经恢复高电平，说明当前未读转换已经清空。
         * 可以提前结束本次排空过程。
         */
        if (Ldc1614Service_IsIntbAsserted() == 0U)
        {
            break;
        }
    }

    return (*updated_mask != 0U) ? LDC1614_STATUS_OK : LDC1614_STATUS_NOT_READY;
}

/**
 * @brief 建立所有启用通道的空载基线和噪声估计。
 * @param ldc LDC1614 驱动句柄，不能为空。
 * @param channel_contexts 通道上下文数组，不能为空。
 * @param channel_count 通道数量。
 * @return LDC1614_Status_t 建立结果。
 */
static LDC1614_Status_t Ldc1614Service_BuildBaselines(LDC1614_Handle_t *ldc,
                                                      LDC1614_ChannelContext_t *channel_contexts,
                                                      uint8_t channel_count)
{
    uint64_t sample_sum[LDC1614_SERVICE_CHANNEL_COUNT] = {0U};
    uint64_t noise_sum[LDC1614_SERVICE_CHANNEL_COUNT] = {0U};
    uint32_t previous_sample[LDC1614_SERVICE_CHANNEL_COUNT] = {0U};
    uint32_t valid_samples[LDC1614_SERVICE_CHANNEL_COUNT] = {0U};
    uint32_t updated_mask;
    LDC1614_Status_t status;
    uint8_t channel_index;
    uint8_t baseline_ready;

    if ((ldc == NULL) || (channel_contexts == NULL) || (channel_count == 0U))
    {
        return LDC1614_STATUS_INVALID_PARAM;
    }

    Ldc1614Service_InitChannelContexts(channel_contexts, channel_count);

    for (;;)
    {
        baseline_ready = 1U;
        for (channel_index = 0U; channel_index < channel_count; ++channel_index)
        {
            if (valid_samples[channel_index] < LDC1614_SERVICE_BASELINE_SAMPLES)
            {
                baseline_ready = 0U;
                break;
            }
        }

        if (baseline_ready != 0U)
        {
            break;
        }

        status = Ldc1614Service_ReadReadyChannels(ldc, channel_contexts, channel_count, &updated_mask);
        if (status == LDC1614_STATUS_NOT_READY)
        {
            continue;
        }
        if (status != LDC1614_STATUS_OK)
        {
            return status;
        }

        for (channel_index = 0U; channel_index < channel_count; ++channel_index)
        {
            if ((updated_mask & (1UL << channel_index)) == 0U)
            {
                continue;
            }

            if (valid_samples[channel_index] > 0U)
            {
                noise_sum[channel_index] +=
                    Ldc1614Service_AbsoluteDifference(channel_contexts[channel_index].latest_filtered_sample,
                                                      previous_sample[channel_index]);
            }

            previous_sample[channel_index] = channel_contexts[channel_index].latest_filtered_sample;
            sample_sum[channel_index] += channel_contexts[channel_index].latest_filtered_sample;
            ++valid_samples[channel_index];
        }
    }

    for (channel_index = 0U; channel_index < channel_count; ++channel_index)
    {
        channel_contexts[channel_index].baseline =
            (uint32_t)(sample_sum[channel_index] / LDC1614_SERVICE_BASELINE_SAMPLES);

        if (LDC1614_SERVICE_BASELINE_SAMPLES > 1U)
        {
            Ldc1614Service_CalculateThresholds(
                (uint32_t)(noise_sum[channel_index] / (LDC1614_SERVICE_BASELINE_SAMPLES - 1U)),
                &channel_contexts[channel_index].detect_threshold,
                &channel_contexts[channel_index].release_threshold);
        }
        else
        {
            Ldc1614Service_CalculateThresholds(0U,
                                               &channel_contexts[channel_index].detect_threshold,
                                               &channel_contexts[channel_index].release_threshold);
        }
    }

    return LDC1614_STATUS_OK;
}

/**
 * @brief 输出任务初始化完成后的关键信息。
 * @param channel_contexts 通道上下文数组，不能为空。
 * @param channel_count 通道数量。
 */
static void Ldc1614Service_ReportReady(const LDC1614_ChannelContext_t *channel_contexts, uint8_t channel_count)
{
    uint8_t index;

    if (channel_contexts == NULL)
    {
        return;
    }

    my_printf(&huart1, "[OK][LDC] Defect inspection task started. I2C2=PB10/PB11\r\n");
    for (index = 0U; index < channel_count; ++index)
    {
        my_printf(&huart1,
                  "[INFO][LDC] %s baseline=%lu, detect=%lu, release=%lu, settle=%ums, samples=%u, mode=abs-delta\r\n",
                  channel_contexts[index].label,
                  (unsigned long)channel_contexts[index].baseline,
                  (unsigned long)channel_contexts[index].detect_threshold,
                  (unsigned long)channel_contexts[index].release_threshold,
                  (unsigned int)LDC1614_SERVICE_SETTLE_DELAY_MS,
                  (unsigned int)LDC1614_SERVICE_MEASURE_SAMPLE_COUNT);

        if (channel_contexts[index].reference_enabled != 0U)
        {
            my_printf(&huart1,
                      "[INFO][LDC] %s reference_delta=%lu, tolerance=%lu\r\n",
                      channel_contexts[index].label,
                      (unsigned long)channel_contexts[index].reference_delta,
                      (unsigned long)channel_contexts[index].defect_tolerance);
        }
        else
        {
            my_printf(&huart1,
                      "[INFO][LDC] %s reference=unset\r\n",
                      channel_contexts[index].label);
        }
    }
}

/**
 * @brief 上报驱动层读数错误。
 * @param channel_label 出错通道的名称。
 * @param status 驱动返回状态码。
 */
static void Ldc1614Service_ReportError(const char *channel_label, LDC1614_Status_t status)
{
    my_printf(&huart1,
              "[ERROR][LDC] %s read failed, status=%d\r\n",
              (channel_label != NULL) ? channel_label : "LDC",
              (int)status);
}

/**
 * @brief 对一个已经测得的稳定平台值做结果判定并输出日志。
 * @param channel_context 通道上下文指针，不能为空。
 * @param stable_sample 稳定窗口平均值。
 */
static void Ldc1614Service_ReportMeasurementResult(LDC1614_ChannelContext_t *channel_context,
                                                   uint32_t stable_sample)
{
    uint32_t stable_delta;
    int32_t directional_delta;

    if (channel_context == NULL)
    {
        return;
    }

    directional_delta = Ldc1614Service_GetDirectionalDelta(channel_context, stable_sample);
    stable_delta = (directional_delta > 0) ? (uint32_t)directional_delta : 0U;
    channel_context->last_stable_delta = stable_delta;

    if (channel_context->reference_enabled == 0U)
    {
        my_printf(&huart1,
                  "[RESULT][LDC] %s stable_delta=%lu, reference=unset\r\n",
                  channel_context->label,
                  (unsigned long)stable_delta);
        return;
    }

    if (Ldc1614Service_AbsoluteDifference(stable_delta, channel_context->reference_delta) <=
        channel_context->defect_tolerance)
    {
        my_printf(&huart1,
                  "[RESULT][LDC] %s OK, stable_delta=%lu, reference=%lu\r\n",
                  channel_context->label,
                  (unsigned long)stable_delta,
                  (unsigned long)channel_context->reference_delta);
    }
    else
    {
        my_printf(&huart1,
                  "[RESULT][LDC] %s DEFECT, stable_delta=%lu, reference=%lu\r\n",
                  channel_context->label,
                  (unsigned long)stable_delta,
                  (unsigned long)channel_context->reference_delta);
    }
}

/**
 * @brief 处理一条发给 LDC 服务的串口命令。
 * @param command_buffer 已经规范化后的命令字符串。
 * @return uint8_t 1 表示该命令已处理，0 表示不是 LDC 命令。
 *
 * 当前支持：
 * 1. `LDCCAL CH1`
 * 2. `LDCCAL CH2 20`
 * 3. `LDCSTOP`
 */
uint8_t Ldc1614Service_HandleCommand(const char *command_buffer)
{
    uint8_t channel_index;
    uint16_t sample_count;

    if (command_buffer == NULL)
    {
        return 0U;
    }

    if (strcmp(command_buffer, "LDCSTOP") == 0)
    {
        Ldc1614Service_StopCalibrationSession();
        my_printf(&huart1, "[OK][LDC] Calibration sampling stopped.\r\n");
        return 1U;
    }

    if (Ldc1614Service_ParseCalibrationCommand(command_buffer, &channel_index, &sample_count) == 0U)
    {
        return 0U;
    }

    Ldc1614Service_StartCalibrationSession(channel_index, sample_count);
    my_printf(&huart1,
              "[OK][LDC] Calibration sampling armed. target=%s, count=%u\r\n",
              g_ldc1614_channel_labels[channel_index],
              (unsigned int)sample_count);
    my_printf(&huart1,
              "[INFO][LDC] Place the part on %s and keep it still until %u samples are captured.\r\n",
              g_ldc1614_channel_labels[channel_index],
              (unsigned int)sample_count);

    return 1U;
}

/**
 * @brief 处理单个通道的一次新采样。
 * @param channel_context 通道上下文指针，不能为空。
 *
 * 状态机流程如下：
 * 1. IDLE：检测工件是否进入；
 * 2. SETTLING：工件已进入，但先等待放稳；
 * 3. MEASURING：采集一个稳定窗口；
 * 4. WAIT_REMOVE：结果已输出，等待工件拿走后重新武装。
 */
static void Ldc1614Service_ProcessChannelSample(LDC1614_ChannelContext_t *channel_context)
{
    int32_t directional_delta;

    if (channel_context == NULL)
    {
        return;
    }

    directional_delta = Ldc1614Service_GetDirectionalDelta(channel_context,
                                                           channel_context->latest_filtered_sample);

    switch (channel_context->state)
    {
        case LDC1614_SERVICE_STATE_IDLE:
            /*
             * 上电刚完成基线建立时，先要求通道回到空载稳定区再放开检测。
             * 否则某些通道第一次较大的自然漂移会被误判成工件放入，
             * 用户就会看到“什么都没放先检测到，放上去反而没反应”的现象。
             */
            if (channel_context->detect_armed == 0U)
            {
                if (directional_delta <= (int32_t)channel_context->release_threshold)
                {
                    channel_context->baseline = Ldc1614Service_UpdateBaseline(channel_context->baseline,
                                                                              channel_context->latest_filtered_sample);

                    if (channel_context->startup_quiet_count < LDC1614_SERVICE_STARTUP_ARM_CONFIRM_COUNT)
                    {
                        ++channel_context->startup_quiet_count;
                    }

                    if (channel_context->startup_quiet_count >= LDC1614_SERVICE_STARTUP_ARM_CONFIRM_COUNT)
                    {
                        channel_context->detect_armed = 1U;
                        channel_context->startup_quiet_count = 0U;
                    }
                }
                else
                {
                    /*
                     * 启动阶段如果仍存在大幅漂移，说明当前基线还没有真正收敛。
                     * 这里直接把最新空载样本吸收到基线中，尽快消除启动假触发。
                     */
                    channel_context->baseline = channel_context->latest_filtered_sample;
                    channel_context->startup_quiet_count = 0U;
                }

                channel_context->detect_confirm_count = 0U;
                break;
            }

            if (directional_delta >= (int32_t)channel_context->detect_threshold)
            {
                ++channel_context->detect_confirm_count;
                if (channel_context->detect_confirm_count >= LDC1614_SERVICE_DETECT_CONFIRM_COUNT)
                {
                    Ldc1614Service_EnterSettling(channel_context);
                    my_printf(&huart1,
                              "[EVENT][LDC] %s part detected, settling\r\n",
                              channel_context->label);
                }
            }
            else
            {
                channel_context->baseline = Ldc1614Service_UpdateBaseline(channel_context->baseline,
                                                                          channel_context->latest_filtered_sample);
                channel_context->detect_confirm_count = 0U;
            }
            break;

        case LDC1614_SERVICE_STATE_SETTLING:
            /*
             * 如果等待放稳期间信号已经回落到释放阈值以内，
             * 说明刚才大概率只是手部/夹具掠过，撤销本次检测。
             */
            if (directional_delta <= (int32_t)channel_context->release_threshold)
            {
                ++channel_context->release_confirm_count;
                if (channel_context->release_confirm_count >= LDC1614_SERVICE_RELEASE_CONFIRM_COUNT)
                {
                    my_printf(&huart1,
                              "[WARN][LDC] %s settling cancelled, part removed before stable\r\n",
                              channel_context->label);
                    Ldc1614Service_EnterIdle(channel_context, 0U);
                }
            }
            else
            {
                channel_context->release_confirm_count = 0U;

                if (xTaskGetTickCount() >= channel_context->settle_deadline_tick)
                {
                    Ldc1614Service_EnterMeasuring(channel_context);
                }
            }
            break;

        case LDC1614_SERVICE_STATE_MEASURING:
        {
            uint16_t calibration_target_count = 0U;

            /*
             * 测量窗口内如果工件又提前离开，本轮直接作废，避免错误结论。
             */
            if (directional_delta <= (int32_t)channel_context->release_threshold)
            {
                ++channel_context->release_confirm_count;
                if (channel_context->release_confirm_count >= LDC1614_SERVICE_RELEASE_CONFIRM_COUNT)
                {
                    my_printf(&huart1,
                              "[WARN][LDC] %s measurement aborted, part removed early\r\n",
                              channel_context->label);
                    if (Ldc1614Service_IsCalibrationTargetChannel(channel_context, NULL) != 0U)
                    {
                        Ldc1614Service_ClearCalibrationProgress();
                    }
                    Ldc1614Service_EnterIdle(channel_context, 0U);
                }
                break;
            }

            channel_context->release_confirm_count = 0U;

            if (Ldc1614Service_IsCalibrationTargetChannel(channel_context, &calibration_target_count) != 0U)
            {
                uint16_t captured_count = 0U;
                uint32_t delta_sample;

                delta_sample = (directional_delta > 0) ? (uint32_t)directional_delta : 0U;
                if (Ldc1614Service_RecordCalibrationSample(channel_context->latest_filtered_sample,
                                                           delta_sample,
                                                           &captured_count,
                                                           &calibration_target_count) != 0U)
                {
                    channel_context->last_stable_delta = delta_sample;
                    my_printf(&huart1,
                              "[CAL][LDC] %s sample %u/%u raw=%lu, delta=%lu\r\n",
                              channel_context->label,
                              (unsigned int)captured_count,
                              (unsigned int)calibration_target_count,
                              (unsigned long)channel_context->latest_filtered_sample,
                              (unsigned long)delta_sample);
                    Ldc1614Service_ReportCalibrationSummary(channel_context);
                    Ldc1614Service_StopCalibrationSession();

                    channel_context->state = LDC1614_SERVICE_STATE_WAIT_REMOVE;
                    channel_context->detect_confirm_count = 0U;
                    channel_context->release_confirm_count = 0U;
                    Ldc1614Service_ResetMeasureWindow(channel_context);
                }
                else
                {
                    my_printf(&huart1,
                              "[CAL][LDC] %s sample %u/%u raw=%lu, delta=%lu\r\n",
                              channel_context->label,
                              (unsigned int)captured_count,
                              (unsigned int)calibration_target_count,
                              (unsigned long)channel_context->latest_filtered_sample,
                              (unsigned long)delta_sample);
                }
            }
            else
            {
                channel_context->measure_sum += channel_context->latest_filtered_sample;

                if (channel_context->latest_filtered_sample < channel_context->measure_min)
                {
                    channel_context->measure_min = channel_context->latest_filtered_sample;
                }

                if (channel_context->latest_filtered_sample > channel_context->measure_max)
                {
                    channel_context->measure_max = channel_context->latest_filtered_sample;
                }

                ++channel_context->measure_count;
                if (channel_context->measure_count >= LDC1614_SERVICE_MEASURE_SAMPLE_COUNT)
                {
                    uint32_t spread;
                    uint32_t stable_sample;

                    spread = channel_context->measure_max - channel_context->measure_min;
                    if (spread > Ldc1614Service_GetMeasureSpreadLimit(channel_context))
                    {
                        my_printf(&huart1,
                                  "[INFO][LDC] %s unstable window, resettle (spread=%lu)\r\n",
                                  channel_context->label,
                                  (unsigned long)spread);
                        Ldc1614Service_EnterSettling(channel_context);
                        break;
                    }

                    stable_sample = (uint32_t)(channel_context->measure_sum /
                                               (uint64_t)LDC1614_SERVICE_MEASURE_SAMPLE_COUNT);
                    Ldc1614Service_ReportMeasurementResult(channel_context, stable_sample);

                    channel_context->state = LDC1614_SERVICE_STATE_WAIT_REMOVE;
                    channel_context->detect_confirm_count = 0U;
                    channel_context->release_confirm_count = 0U;
                    Ldc1614Service_ResetMeasureWindow(channel_context);
                }
            }
            break;
        }

        case LDC1614_SERVICE_STATE_WAIT_REMOVE:
        {
            int32_t remove_threshold;

            remove_threshold = (int32_t)Ldc1614Service_GetRemoveThreshold(channel_context);
            if (directional_delta <= remove_threshold)
            {
                ++channel_context->release_confirm_count;
                if (channel_context->release_confirm_count >= LDC1614_SERVICE_RELEASE_CONFIRM_COUNT)
                {
                    my_printf(&huart1,
                              "[EVENT][LDC] %s part removed, ready (delta=%lu)\r\n",
                              channel_context->label,
                              (unsigned long)((directional_delta > 0) ? directional_delta : 0));
                    Ldc1614Service_EnterIdle(channel_context, 1U);
                }
            }
            else
            {
                channel_context->release_confirm_count = 0U;
            }
            break;
        }

        default:
            Ldc1614Service_EnterIdle(channel_context, 0U);
            break;
    }
}

/**
 * @brief LDC1614 双通道缺陷检测任务。
 * @param argument FreeRTOS 任务参数，当前未使用。
 */
void Ldc1614Service_Task(void *argument)
{
    LDC1614_Handle_t ldc;
    LDC1614_ChannelContext_t channel_contexts[LDC1614_SERVICE_CHANNEL_COUNT];
    uint32_t updated_mask;
    uint32_t consecutive_error_count = 0U;
    LDC1614_Status_t status;
    uint8_t index;

    (void)argument;
    g_ldc1614_task_handle = xTaskGetCurrentTaskHandle();

    LDC1614_LoadDefaultConfig(&ldc, &hi2c2);
    Ldc1614Service_InitChannelContexts(channel_contexts, LDC1614_SERVICE_CHANNEL_COUNT);

    for (;;)
    {
        status = LDC1614_Init(&ldc);
        if (status == LDC1614_STATUS_OK)
        {
            status = Ldc1614Service_BuildBaselines(&ldc,
                                                   channel_contexts,
                                                   LDC1614_SERVICE_CHANNEL_COUNT);
        }

        if (status == LDC1614_STATUS_OK)
        {
            Ldc1614Service_ReportReady(channel_contexts, LDC1614_SERVICE_CHANNEL_COUNT);
            break;
        }

        my_printf(&huart1, "[ERROR][LDC] Dual-channel init failed, status=%d\r\n", (int)status);
        osDelay(1000U);
    }

    for (;;)
    {
        status = Ldc1614Service_ReadReadyChannels(&ldc,
                                                  channel_contexts,
                                                  LDC1614_SERVICE_CHANNEL_COUNT,
                                                  &updated_mask);
        if (status == LDC1614_STATUS_OK)
        {
            consecutive_error_count = 0U;

            for (index = 0U; index < LDC1614_SERVICE_CHANNEL_COUNT; ++index)
            {
                if ((updated_mask & (1UL << index)) == 0U)
                {
                    continue;
                }

                Ldc1614Service_ProcessChannelSample(&channel_contexts[index]);
            }
        }
        else if (status != LDC1614_STATUS_NOT_READY)
        {
            ++consecutive_error_count;
            if (consecutive_error_count <= 3U)
            {
                Ldc1614Service_ReportError("Dual-channel", status);
            }
        }
    }
}

/**
 * @brief HAL GPIO 外部中断回调。
 * @param GPIO_Pin 当前触发中断的 GPIO 引脚编号。
 *
 * 该回调运行在中断上下文中，因此只负责唤醒任务。
 * 所有 I2C 访问、状态机推进和串口打印都必须放到任务里做。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((GPIO_Pin == LDC1614_SERVICE_INTB_GPIO_PIN) && (g_ldc1614_task_handle != NULL))
    {
        vTaskNotifyGiveFromISR(g_ldc1614_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
