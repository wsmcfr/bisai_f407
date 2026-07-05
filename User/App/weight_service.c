#include "weight_service.h"

#include "binary_protocol_service.h"
#include "camera_motor_service.h"
#include "cmsis_os.h"
#include "conveyor_motor_service.h"
#include "hx711.h"
#include "ldc1614_service.h"
#include "robot_arm_service.h"
#include "uart_command.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

/**
 * @brief USART1 用户命令总入口速查。
 *
 * 通信入口：
 * - USART1：115200 8N1，PA9(TX)/PA10(RX)，由串口助手、MP157 或其它上位机发送命令；
 * - 本文件的 `WeightService_ProcessCommand()` 是当前 USART1 命令的唯一任务级消费者；
 * - 自动检测二进制帧会先走 `BinaryProtocolService_HandleFrame()`，识别成功后不会继续按文本命令解析；
 * - 机械臂二进制帧会先走 `RobotArmService_HandleFrame()`，识别成功后直接转发到 USART3，不再按文本命令解析；
 * - 文本命令会去掉首尾空白并转成大写，因此 `get`、`GET\r\n`、`Get` 都等价于 `GET`。
 *
 * MP157 主链路当前只允许发送自动检测二进制帧：
 * | 命令类型 | 作用 | 正确返回 | 错误返回 |
 * | --- | --- | --- | --- |
 * | `HEARTBEAT` | 查询 F4 二进制协议入口是否在线 | `ACK` | `NACK` |
 * | `START/PAUSE/RESUME/STOP/VISION/BELT` | 控制传送带和自动检测状态机 | `ACK` 或 `STATUS_REPORT` | `NACK` 或 `FAULT_REPORT` |
 *
 * 旧的 `GET/STATUS/TARE/CAL/LDCCAL/BELTSCAN/CAM...` 文本命令只作为断开 MP157 后的串口助手维护入口。
 * USART1 文本输出默认静默，因此 MP157 不再依赖 `[OK]`、`[ERROR]` 或 `[INFO]` 文本判断成功失败。
 *
 * 用户可从 USART1 直接发送的机械臂二进制帧：
 * - 例如 `55 55 02 01` 查询 ESP32 版本，`55 55 05 06 03 01 00` 运行 3 号动作组 1 次；
 * - 详细帧表和接线要求在 `User/App/robot_arm_service.c` 顶部维护。
 *
 * 用户可从 USART1 直接发送的自动检测二进制帧：
 * - 帧头固定 `A5 5A`，帧尾固定 `6B`，CRC16 覆盖 `VER~PAYLOAD`；
 * - 首轮已接入 `START_CYCLE`、`PAUSE_CYCLE`、`RESUME_CYCLE`、`STOP_CYCLE`、`VISION_POS`、`VISION_LOST` 和 `BELT_STOP_CENTERED`；
 * - 详细帧格式、命令字和负载字段在 `User/App/binary_protocol_service.h` 顶部维护。
 */

/**
 * @brief 周期性重量采样周期，单位毫秒。
 *
 * 当前任务不再周期主动上报串口，因此这里只需要保证重量值持续更新即可。
 * 50ms 可以兼顾响应速度和CPU占用。
 */
#define WEIGHT_SERVICE_SAMPLE_PERIOD_MS        (50U)

/**
 * @brief 启动时空载去皮采样次数。
 *
 * 通过多次平均降低单次抖动对offset的影响。当前数值适合作为初始默认值。
 */
#define WEIGHT_SERVICE_TARE_SAMPLES            (10U)

/**
 * @brief 单次等待HX711数据就绪的超时时间，单位毫秒。
 *
 * 若在该时间内仍未就绪，通常说明接线、供电或时序存在问题，
 * 应由上层通过串口看到超时提示后继续排查。
 */
#define WEIGHT_SERVICE_READ_TIMEOUT_MS         (100U)

/**
 * @brief 原始采样中值滤波窗口长度。
 *
 * 资料例程使用了 5 点中值滤波。这里沿用同样的窗口长度，
 * 在不明显增加延迟的前提下改善重量抖动。
 */
#define WEIGHT_SERVICE_MEDIAN_FILTER_SIZE      (5U)

/**
 * @brief 默认推荐标定砝码重量入口，单位克。
 *
 * 对 5kg 量程称重模块而言，1000g 是较容易准备、也较适合的标定重量。
 * 真正标定时，用户仍可通过串口命令传入其他合法重量。
 */
#define WEIGHT_SERVICE_DEFAULT_CAL_WEIGHT_G    (1000U)

/**
 * @brief 串口命令等待超时时间，单位毫秒。
 *
 * 当前主任务本身就以固定周期运行，因此这里采用非阻塞轮询即可。
 */
#define WEIGHT_SERVICE_COMMAND_WAIT_MS         (0U)

/**
 * @brief 称重中值滤波器状态。
 *
 * 该结构体保存最近若干个原始采样值，用于执行中值滤波。
 * 中值滤波对电子称场景中的偶发尖峰噪声比简单平均更稳健。
 */
typedef struct
{
    int32_t raw_samples[WEIGHT_SERVICE_MEDIAN_FILTER_SIZE]; /* 最近的 HX711 原始采样窗口，单位为 ADC 计数，用于计算中值。 */
    uint8_t sample_count;                                   /* 当前窗口内有效样本数量，未填满窗口时只在已有样本内取中值。 */
    uint8_t write_index;                                    /* 环形写入位置，指向下一次新样本要覆盖的数组下标。 */
} WeightService_Filter_t;

/**
 * @brief 二进制标定入口使用的称重运行上下文。
 *
 * USART1 二进制协议由 WeightService_Task 消费，所以协议处理函数和 HX711 状态天然处于同一个任务调用链。
 * 这里保存指针和最近一次采样快照，让 binary_protocol_service.c 能通过公开函数请求标定，
 * 但仍不直接访问 weight_service.c 内部的静态局部变量。
 */
typedef struct
{
    HX711_Handle_t *hx711;              /* 当前称重任务持有的 HX711 句柄，标定时写入 scale_counts_per_g。 */
    HX711_Status_t latest_status;       /* 最近一次 HX711 采样状态，用于拒绝超时或参数错误的样本。 */
    int32_t latest_raw_value;           /* 最近一次中值滤波后的原始计数，单位为 HX711 ADC counts。 */
    uint8_t *tare_ready;                /* 指向称重任务去皮完成标志，1 表示 offset 可用于标定。 */
    uint16_t sample_id;                 /* 最近一次成功刷新快照的本地样本号，循环递增即可。 */
    uint8_t filter_sample_count;        /* 当前中值滤波窗口内有效样本数量，用于上报 sample_count。 */
} WeightService_BinaryContext_t;

/**
 * @brief 保存最近一次可供二进制协议使用的称重上下文。
 *
 * 该变量只在 WeightService_Task 所在线程更新和读取；当前没有跨任务并发写入。
 */
static WeightService_BinaryContext_t g_weight_service_binary_context = {
    (HX711_Handle_t *)0,
    HX711_STATUS_INVALID_PARAM,
    0,
    (uint8_t *)0,
    0U,
    0U
};

/**
 * @brief 复位滤波器状态。
 * @param filter 滤波器对象指针，不能为空。
 *
 * 去皮成功后应立即清空旧样本，避免新的零点基准混入历史数据。
 */
static void WeightService_FilterReset(WeightService_Filter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    (void)memset(filter->raw_samples, 0, sizeof(filter->raw_samples));
    filter->sample_count = 0U;
    filter->write_index = 0U;
}

/**
 * @brief 对一组整数样本执行原地升序插入排序。
 * @param values 待排序数组。
 * @param length 有效样本长度。
 *
 * 中值滤波窗口很小，因此用插入排序即可，逻辑清晰且足够快。
 */
static void WeightService_SortAscending(int32_t *values, uint8_t length)
{
    uint8_t outer_index;

    if ((values == NULL) || (length < 2U))
    {
        return;
    }

    for (outer_index = 1U; outer_index < length; ++outer_index)
    {
        int32_t key_value;
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
 * @brief 压入一个新原始值，并返回当前中值滤波结果。
 * @param filter 滤波器对象指针，不能为空。
 * @param raw_value 新采样到的原始值。
 * @return int32_t 当前滤波后的原始值。
 *
 * 当滤波窗口尚未填满时，会在“已有样本”范围内求中值。
 */
static int32_t WeightService_FilterPush(WeightService_Filter_t *filter, int32_t raw_value)
{
    int32_t sorted_samples[WEIGHT_SERVICE_MEDIAN_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t valid_count;

    if (filter == NULL)
    {
        return raw_value;
    }

    filter->raw_samples[filter->write_index] = raw_value;
    filter->write_index = (uint8_t)((filter->write_index + 1U) % WEIGHT_SERVICE_MEDIAN_FILTER_SIZE);

    if (filter->sample_count < WEIGHT_SERVICE_MEDIAN_FILTER_SIZE)
    {
        ++filter->sample_count;
    }

    valid_count = filter->sample_count;
    for (sample_index = 0U; sample_index < valid_count; ++sample_index)
    {
        sorted_samples[sample_index] = filter->raw_samples[sample_index];
    }

    WeightService_SortAscending(sorted_samples, valid_count);
    return sorted_samples[valid_count / 2U];
}

/**
 * @brief 规范化串口命令文本。
 * @param command_buffer 待处理命令缓存，必须可写。
 *
 * 该函数会原地去掉命令前后的空白字符，并把小写字母转换为大写，
 * 从而让上位机发送 `GET`、`get`、`GET\r\n` 都能落到同一解析结果。
 */
static void WeightService_NormalizeCommand(char *command_buffer)
{
    uint16_t read_index = 0U;
    uint16_t write_index = 0U;
    char current_char;

    if (command_buffer == NULL)
    {
        return;
    }

    while ((command_buffer[read_index] == ' ') ||
           (command_buffer[read_index] == '\r') ||
           (command_buffer[read_index] == '\n') ||
           (command_buffer[read_index] == '\t'))
    {
        ++read_index;
    }

    while (command_buffer[read_index] != '\0')
    {
        current_char = command_buffer[read_index];
        if ((current_char >= 'a') && (current_char <= 'z'))
        {
            current_char = (char)(current_char - ('a' - 'A'));
        }

        command_buffer[write_index] = current_char;
        ++write_index;
        ++read_index;
    }

    while ((write_index > 0U) &&
           ((command_buffer[write_index - 1U] == ' ') ||
            (command_buffer[write_index - 1U] == '\r') ||
            (command_buffer[write_index - 1U] == '\n') ||
            (command_buffer[write_index - 1U] == '\t')))
    {
        --write_index;
    }

    command_buffer[write_index] = '\0';
}

/**
 * @brief 解析 `CAL <克重>` 命令中的砝码重量。
 * @param command_buffer 已规范化后的命令字符串。
 * @param known_weight_g 解析得到的砝码重量输出参数。
 * @return uint8_t 1表示解析成功，0表示格式不合法。
 *
 * 当前接受的命令格式示例：
 * - `CAL 1000`
 * - `CAL 2000`
 */
static uint8_t WeightService_ParseCalibrationWeight(const char *command_buffer, uint32_t *known_weight_g)
{
    char *end_pointer;
    unsigned long parsed_value;

    if ((command_buffer == NULL) || (known_weight_g == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "CAL", 3U) != 0)
    {
        return 0U;
    }

    command_buffer += 3;
    while ((*command_buffer == ' ') || (*command_buffer == '\t'))
    {
        ++command_buffer;
    }

    if (*command_buffer == '\0')
    {
        return 0U;
    }

    parsed_value = strtoul(command_buffer, &end_pointer, 10);
    if ((command_buffer == end_pointer) || (*end_pointer != '\0'))
    {
        return 0U;
    }

    *known_weight_g = (uint32_t)parsed_value;
    return 1U;
}

/**
 * @brief 执行一次去皮，并补齐成功/失败后的状态维护。
 * @param hx711 HX711句柄指针，不能为空。
 * @param tare_ready 去皮状态输出标志，不能为空。
 * @param filter 滤波器对象，不能为空。
 * @return HX711_Status_t 去皮结果。
 *
 * 该函数把“去皮成功后要清滤波器”和“去皮失败后要明确状态无效”集中处理，
 * 避免初始化路径和串口 `TARE` 命令路径维护两套逻辑。
 */
static HX711_Status_t WeightService_ExecuteTare(HX711_Handle_t *hx711,
                                                uint8_t *tare_ready,
                                                WeightService_Filter_t *filter)
{
    HX711_Status_t status;

    if ((hx711 == NULL) || (tare_ready == NULL) || (filter == NULL))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    status = HX711_Tare(hx711, WEIGHT_SERVICE_TARE_SAMPLES, WEIGHT_SERVICE_READ_TIMEOUT_MS);
    if (status == HX711_STATUS_OK)
    {
        *tare_ready = 1U;
        WeightService_FilterReset(filter);
    }
    else
    {
        *tare_ready = 0U;
    }

    return status;
}

/**
 * @brief 更新二进制协议标定所需的称重快照。
 * @param hx711 HX711 句柄指针，不能为空。
 * @param latest_status 最近一次采样状态。
 * @param latest_raw_value 最近一次滤波后的原始计数。
 * @param tare_ready 去皮状态指针，不能为空。
 *
 * 每次从 USART1 取到命令后、进入二进制协议分发前调用本函数，
 * 让协议层回调 WeightService_RequestCalibration() 时能拿到同一轮任务中的最新状态。
 */
static void WeightService_UpdateBinaryContext(HX711_Handle_t *hx711,
                                              HX711_Status_t latest_status,
                                              int32_t latest_raw_value,
                                              uint8_t *tare_ready,
                                              const WeightService_Filter_t *filter)
{
    g_weight_service_binary_context.hx711 = hx711;
    g_weight_service_binary_context.latest_status = latest_status;
    g_weight_service_binary_context.latest_raw_value = latest_raw_value;
    g_weight_service_binary_context.tare_ready = tare_ready;
    if (filter != NULL)
    {
        g_weight_service_binary_context.filter_sample_count = filter->sample_count;
    }
    if (latest_status == HX711_STATUS_OK)
    {
        ++g_weight_service_binary_context.sample_id;
        if (g_weight_service_binary_context.sample_id == 0U)
        {
            g_weight_service_binary_context.sample_id = 1U;
        }
    }
}

/**
 * @brief 使用指定称重快照执行一次标定。
 * @param hx711 HX711 句柄指针，不能为空。
 * @param latest_status 最近一次采样状态。
 * @param latest_raw_value 最近一次滤波后的原始计数。
 * @param tare_ready 去皮状态，1 表示可以标定。
 * @param known_weight_g 已知砝码重量，单位克。
 * @param detail 输出失败细节或 0，不能为空。
 * @return WeightService_CalibrationResult_t 标定业务结果。
 *
 * 该函数是文本 `CAL <克重>` 和二进制 `WEIGHT_CALIBRATE` 的共同实现：
 * 1. 先检查去皮状态，避免用未定义 offset 计算比例；
 * 2. 再检查克重是否落在 HX711 量程内；
 * 3. 再检查最近一次采样是否有效；
 * 4. 最后调用 HX711_CalibrateByKnownWeight() 写入运行时比例系数。
 */
static WeightService_CalibrationResult_t WeightService_ExecuteCalibration(HX711_Handle_t *hx711,
                                                                          HX711_Status_t latest_status,
                                                                          int32_t latest_raw_value,
                                                                          uint8_t tare_ready,
                                                                          uint16_t known_weight_g,
                                                                          uint16_t *detail)
{
    HX711_Status_t status;

    if (detail == NULL)
    {
        return WEIGHT_SERVICE_CALIBRATION_NO_CONTEXT;
    }

    *detail = 0U;
    if (hx711 == NULL)
    {
        *detail = 0U;
        return WEIGHT_SERVICE_CALIBRATION_NO_CONTEXT;
    }

    if (tare_ready == 0U)
    {
        *detail = 0U;
        return WEIGHT_SERVICE_CALIBRATION_TARE_NOT_READY;
    }

    if ((known_weight_g == 0U) || ((float)known_weight_g > hx711->rated_capacity_g))
    {
        *detail = known_weight_g;
        return WEIGHT_SERVICE_CALIBRATION_WEIGHT_RANGE;
    }

    if (latest_status != HX711_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)latest_status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)latest_status,
                                          0U);
        *detail = (uint16_t)latest_status;
        return WEIGHT_SERVICE_CALIBRATION_SAMPLE_INVALID;
    }

    status = HX711_CalibrateByKnownWeight(hx711, latest_raw_value, (float)known_weight_g);
    if (status == HX711_STATUS_OK)
    {
        BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
        *detail = 0U;
        return WEIGHT_SERVICE_CALIBRATION_OK;
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
    BinaryProtocolService_ReportFault((uint16_t)status,
                                      BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                      BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                      (int32_t)status,
                                      0U);
    *detail = (uint16_t)status;
    return WEIGHT_SERVICE_CALIBRATION_HARDWARE_ERROR;
}

/**
 * @brief 执行一次由 MP157 二进制协议触发的称重标定。
 * @param known_weight_g 已知砝码重量，单位克。
 * @param detail 输出失败细节或 0，不能为空。
 * @return WeightService_CalibrationResult_t 标定业务结果。
 *
 * 二进制协议处理函数不拥有 HX711 句柄，也不读取称重任务局部变量。
 * 它只调用本函数，由称重服务使用最近一次快照完成校验和标定。
 */
WeightService_CalibrationResult_t WeightService_RequestCalibration(uint16_t known_weight_g,
                                                                   uint16_t *detail)
{
    if ((detail == NULL) ||
        (g_weight_service_binary_context.hx711 == NULL) ||
        (g_weight_service_binary_context.tare_ready == NULL))
    {
        if (detail != NULL)
        {
            *detail = 0U;
        }
        return WEIGHT_SERVICE_CALIBRATION_NO_CONTEXT;
    }

    return WeightService_ExecuteCalibration(g_weight_service_binary_context.hx711,
                                            g_weight_service_binary_context.latest_status,
                                            g_weight_service_binary_context.latest_raw_value,
                                            *(g_weight_service_binary_context.tare_ready),
                                            known_weight_g,
                                            detail);
}

/**
 * @brief 获取称重任务最近一次快照。
 * @param snapshot 输出称重快照，不能为空。
 * @return uint8_t 1 表示快照可用于上报，0 表示称重上下文尚未建立。
 *
 * 主要流程：
 * 1. 检查 HX711 句柄和去皮状态指针是否已经由 WeightService_Task 建立；
 * 2. 复制最近一次原始值、样本号和窗口样本数；
 * 3. 已标定时换算成 mg，未标定或未去皮时把判定置为 review；
 * 4. 通过 option_bits 明确告诉 MP157 该快照是否已标定、已去皮和样本有效。
 */
uint8_t WeightService_GetLatestSnapshot(WeightService_Snapshot_t *snapshot)
{
    HX711_Handle_t *hx711;
    uint8_t tare_ready;
    uint8_t sample_valid;
    int32_t net_weight_mg = 0;
    uint32_t option_bits = 0U;

    if (snapshot == NULL)
    {
        return 0U;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));
    hx711 = g_weight_service_binary_context.hx711;
    if ((hx711 == NULL) || (g_weight_service_binary_context.tare_ready == NULL))
    {
        return 0U;
    }

    tare_ready = *(g_weight_service_binary_context.tare_ready);
    sample_valid = (g_weight_service_binary_context.latest_status == HX711_STATUS_OK) ? 1U : 0U;

    if (HX711_IsCalibrated(hx711) != 0U)
    {
        float grams = HX711_ConvertToGrams(hx711,
                                           g_weight_service_binary_context.latest_raw_value);
        net_weight_mg = (int32_t)(grams * 1000.0f);
        option_bits |= 0x00000001UL;
    }

    if (tare_ready != 0U)
    {
        option_bits |= 0x00000002UL;
    }
    if (sample_valid != 0U)
    {
        option_bits |= 0x00000004UL;
    }

    snapshot->sample_id = g_weight_service_binary_context.sample_id;
    snapshot->stable = ((sample_valid != 0U) && (tare_ready != 0U)) ? 1U : 0U;
    snapshot->decision = (snapshot->stable != 0U) ? 1U : 3U;
    snapshot->gross_weight_mg = net_weight_mg;
    snapshot->net_weight_mg = net_weight_mg;
    snapshot->raw_adc = g_weight_service_binary_context.latest_raw_value;
    snapshot->sample_count = g_weight_service_binary_context.filter_sample_count;
    snapshot->stable_window_mg = 0U;
    snapshot->duration_ms = WEIGHT_SERVICE_SAMPLE_PERIOD_MS;
    snapshot->option_bits = option_bits;
    return 1U;
}

/**
 * @brief 输出当前重量、净计数差值或错误信息。
 * @param hx711 HX711句柄指针，不能为空。
 * @param status 当前读数状态。
 * @param raw_value 当前滤波后的原始值。
 * @param tare_ready 1表示去皮成功，0表示去皮无效。
 *
 * 该函数统一负责所有用户可见的称重输出格式，避免串口文本散落在多个位置。
 */
static void WeightService_ReportSample(const HX711_Handle_t *hx711,
                                       HX711_Status_t status,
                                       int32_t raw_value,
                                       uint8_t tare_ready)
{
    float weight_grams;
    int32_t weight_x100;
    int32_t weight_fraction;

    if (status != HX711_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          0U);
        my_printf(&huart1, "[ERROR][WEIGHT] HX711 read failed, status=%d\r\n", (int)status);
        return;
    }

    if (tare_ready == 0U)
    {
        /*
         * 去皮尚未成功时，绝不把结果解释成重量。
         * 这样可以避免用户看到一个“看似正常”的假克重。
         */
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)HX711_STATUS_INVALID_PARAM,
                                          BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          raw_value,
                                          0U);
        my_printf(&huart1, "[ERROR][WEIGHT] Tare not ready. raw=%ld\r\n", (long)raw_value);
        return;
    }

    BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);

    if (HX711_IsCalibrated(hx711) == 0U)
    {
        /*
         * 未标定时返回净计数差值，而不是伪装成克重。
         * 这样更符合当前真实状态，也方便用户执行后续标定。
         */
        my_printf(&huart1,
                  "raw=%ld, delta=%ld counts, scale=UNCALIBRATED\r\n",
                  (long)raw_value,
                  (long)(raw_value - hx711->offset));
        return;
    }

    weight_grams = HX711_ConvertToGrams(hx711, raw_value);
    weight_x100 = (int32_t)(weight_grams * 100.0f);
    weight_fraction = weight_x100 % 100;
    if (weight_fraction < 0)
    {
        weight_fraction = -weight_fraction;
    }

    my_printf(&huart1,
              "raw=%ld, weight=%ld.%02ld g\r\n",
              (long)raw_value,
              (long)(weight_x100 / 100),
              (long)weight_fraction);
}

/**
 * @brief 根据上位机命令决定是否返回重量或执行控制动作。
 * @param hx711 HX711句柄指针，不能为空。
 * @param latest_status 最近一次采样状态。
 * @param latest_raw_value 最近一次滤波后的原始值。
 * @param tare_ready 去皮状态指针，不能为空。
 * @param filter 滤波器对象，供重新去皮后复位窗口。
 *
 * 当前支持以下输入：
 * - `A5 5A ... 6B`：MP157 主链路二进制协议帧，先交给 BinaryProtocolService 处理；
 * - 旧文本命令：仅作为串口助手维护入口，USART1 面向 MP157 时文本输出默认静默。
 */
static void WeightService_ProcessCommand(HX711_Handle_t *hx711,
                                         HX711_Status_t latest_status,
                                         int32_t latest_raw_value,
                                         uint8_t *tare_ready,
                                         WeightService_Filter_t *filter)
{
    uint8_t raw_frame[64];
    uint16_t raw_frame_length;
    char command_buffer[64];
    uint16_t text_length;
    uint32_t known_weight_g;
    HX711_Status_t status;

    if ((hx711 == NULL) || (tare_ready == NULL) || (filter == NULL))
    {
        return;
    }

    WeightService_UpdateBinaryContext(hx711, latest_status, latest_raw_value, tare_ready, filter);

    if (UartCommand_FetchRaw(raw_frame,
                             sizeof(raw_frame),
                             &raw_frame_length,
                             WEIGHT_SERVICE_COMMAND_WAIT_MS) == 0U)
    {
        return;
    }

    if (BinaryProtocolService_HandleFrame(raw_frame, raw_frame_length) != 0U)
    {
        /*
         * 自动检测二进制协议使用 `A5 5A` 帧头和 CRC 校验。
         * 不管业务命令最终 ACK 还是 NACK，只要识别为本协议帧，就不能再落入机械臂或文本命令解析分支，
         * 否则 CRC 错帧可能被误当作乱码文本处理，现场排查会更混乱。
         */
        return;
    }

    if (RobotArmService_HandleFrame(raw_frame, raw_frame_length) != 0U)
    {
        /*
         * 机械臂帧是二进制协议，不能继续走文本命令规范化流程。
         * 这里直接返回，后续由 RobotArmService_Task 通过 USART3 转发给 ESP32。
         */
        return;
    }

    text_length = raw_frame_length;
    if (text_length >= sizeof(command_buffer))
    {
        /*
         * 文本命令必须预留字符串结束符；超过缓存时截断到最大可解析长度，
         * 防止后续 strcmp/strncmp 访问越界。
         */
        text_length = (uint16_t)(sizeof(command_buffer) - 1U);
    }

    (void)memcpy(command_buffer, raw_frame, text_length);
    command_buffer[text_length] = '\0';

    WeightService_NormalizeCommand(command_buffer);

    if (strcmp(command_buffer, "GET") == 0)
    {
        WeightService_ReportSample(hx711, latest_status, latest_raw_value, *tare_ready);
    }
    else if (strcmp(command_buffer, "STATUS") == 0)
    {
        /*
         * STATUS 是 STM32MP157 周期发送的健康探测命令。
         * 回复中同时包含 OK、F4、READY 三个关键字，匹配 MP157 侧现有
         * DeviceHealthController 的判断条件；该命令只证明 USART1 命令任务仍可调度，
         * 不会触发称重去皮、LDC 标定、电机运动或机械臂动作。
         */
        my_printf(&huart1, "[OK][F4] READY\r\n");
    }
    else if (strcmp(command_buffer, "TARE") == 0)
    {
        status = WeightService_ExecuteTare(hx711, tare_ready, filter);
        if (status == HX711_STATUS_OK)
        {
            BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
            my_printf(&huart1, "[OK][WEIGHT] Tare success. offset=%ld\r\n", (long)hx711->offset);
        }
        else
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1, "[ERROR][WEIGHT] Tare failed, status=%d\r\n", (int)status);
        }
    }
    else if (WeightService_ParseCalibrationWeight(command_buffer, &known_weight_g) == 1U)
    {
        uint16_t calibration_detail;
        WeightService_CalibrationResult_t calibration_result;

        if (known_weight_g > 0xFFFFUL)
        {
            calibration_result = WEIGHT_SERVICE_CALIBRATION_WEIGHT_RANGE;
            calibration_detail = 0xFFFFU;
        }
        else
        {
            calibration_result = WeightService_ExecuteCalibration(hx711,
                                                                  latest_status,
                                                                  latest_raw_value,
                                                                  *tare_ready,
                                                                  (uint16_t)known_weight_g,
                                                                  &calibration_detail);
        }

        if (calibration_result == WEIGHT_SERVICE_CALIBRATION_OK)
        {
            int32_t scale_x100;
            int32_t scale_fraction;

            scale_x100 = (int32_t)(hx711->scale_counts_per_g * 100.0f);
            scale_fraction = scale_x100 % 100;
            if (scale_fraction < 0)
            {
                scale_fraction = -scale_fraction;
            }

            my_printf(&huart1,
                      "[OK][WEIGHT] Calibration success. weight=%lu g, scale=%ld.%02ld counts/g\r\n",
                      (unsigned long)known_weight_g,
                      (long)(scale_x100 / 100),
                      (long)scale_fraction);
        }
        else if (calibration_result == WEIGHT_SERVICE_CALIBRATION_TARE_NOT_READY)
        {
            my_printf(&huart1, "[ERROR][WEIGHT] CAL rejected. Tare is not ready.\r\n");
        }
        else if (calibration_result == WEIGHT_SERVICE_CALIBRATION_WEIGHT_RANGE)
        {
            my_printf(&huart1,
                      "[ERROR][WEIGHT] CAL rejected. Weight must be 1~%ld g.\r\n",
                      (long)hx711->rated_capacity_g);
        }
        else if (calibration_result == WEIGHT_SERVICE_CALIBRATION_SAMPLE_INVALID)
        {
            my_printf(&huart1,
                      "[ERROR][WEIGHT] CAL rejected. Latest sample invalid, status=%u\r\n",
                      (unsigned int)calibration_detail);
        }
        else
        {
            my_printf(&huart1,
                      "[ERROR][WEIGHT] Calibration failed, status=%u\r\n",
                      (unsigned int)calibration_detail);
        }
    }
    else if (Ldc1614Service_HandleCommand(command_buffer) != 0U)
    {
        /* LDC 命令已由对应模块接管，这里不再重复输出。 */
    }
    else if (ConveyorMotorService_HandleCommand(command_buffer) != 0U)
    {
        /* 传送带电机命令已由对应模块接管，这里不再重复输出。 */
    }
    else if (CameraMotorService_HandleCommand(command_buffer) != 0U)
    {
        /* 摄像头运动电机命令已由对应模块接管，这里不再重复输出。 */
    }
    else
    {
        /*
         * 这里故意把提示压短。
         * 当前串口格式化发送缓存只有 128 字节，若把所有命令完整展开，
         * 日志会被截断并和其它事件日志混在一起，反而更难看清。
         */
        my_printf(&huart1,
                  "[ERROR][UART] Unknown cmd. Use STATUS/GET/TARE/CAL/LDCCAL/LDCSTOP/BELTSCAN/BELTSTOP/BELTTRACK/BELTINFO/CAMINFO/CAMSTOP/CAMLAT/CAMZ.\r\n");
    }
}

/**
 * @brief 电子称主任务。
 * @param argument FreeRTOS任务参数，当前未使用。
 *
 * 该任务的主流程如下：
 * 1. 启动串口DMA+空闲中断接收；
 * 2. 加载HX711默认配置并初始化GPIO；
 * 3. 尝试一次启动去皮；
 * 4. 周期更新最近一次滤波后的重量原始值；
 * 5. 仅当收到上位机命令时，返回重量或执行去皮/标定。
 */
void WeightService_Task(void *argument)
{
    HX711_Handle_t hx711;
    WeightService_Filter_t filter;
    HX711_Status_t latest_status;
    int32_t latest_raw_value = 0;
    uint8_t tare_ready = 0U;
    uint8_t hx711_ready = 0U;
    uint8_t weight_fault_reported = 0U;

    (void)argument;

    WeightService_FilterReset(&filter);
    UartCommand_StartReceive();

    HX711_LoadDefaultConfig(&hx711);
    latest_status = HX711_Init(&hx711);
    if (latest_status != HX711_STATUS_OK)
    {
        /*
         * HX711 没接或 GPIO 初始化异常时，不能让任务停在死循环。
         * WeightService_Task 同时是 USART1 命令消费者；如果这里阻塞，
         * MP157 的 START_CYCLE/HEARTBEAT 二进制帧也会没人处理，传送带调试会被称重模块牵连。
         * 因此只置位称重故障并继续跑主循环，让 MP157 通过 STATUS_REPORT/FAULT_REPORT 得到结构化错误。
         */
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)latest_status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)latest_status,
                                          0U);
        weight_fault_reported = 1U;
        my_printf(&huart1, "[ERROR][WEIGHT] HX711 init failed.\r\n");
    }
    else
    {
        hx711_ready = 1U;

        /*
         * 启动阶段先尝试一次自动去皮。
         * 若此时失败，不再像之前那样静默继续，而是保留错误状态并允许用户稍后手动发送 TARE。
         */
        latest_status = WeightService_ExecuteTare(&hx711, &tare_ready, &filter);
        if (latest_status == HX711_STATUS_OK)
        {
            BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
            my_printf(&huart1, "[OK][WEIGHT] Startup tare success. offset=%ld\r\n", (long)hx711.offset);
        }
        else
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
            BinaryProtocolService_ReportFault((uint16_t)latest_status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)latest_status,
                                              0U);
            weight_fault_reported = 1U;
            my_printf(&huart1,
                      "[ERROR][WEIGHT] Startup tare failed, status=%d. Use TARE after checking wiring.\r\n",
                      (int)latest_status);
        }

        my_printf(&huart1,
                  "[INFO][WEIGHT] HX711 ready. DOUT=PB0, SCK=PB2, capacity=%ld g, default CAL=%u g\r\n",
                  (long)hx711.rated_capacity_g,
                  (unsigned int)WEIGHT_SERVICE_DEFAULT_CAL_WEIGHT_G);
    }

    for (;;)
    {
        /*
         * 主循环只负责持续刷新“最近一次滤波后采样值”，
         * 不主动周期上报。
         */
        if (hx711_ready != 0U)
        {
            latest_status = HX711_ReadRaw(&hx711, &latest_raw_value, WEIGHT_SERVICE_READ_TIMEOUT_MS);
            if (latest_status == HX711_STATUS_OK)
            {
                latest_raw_value = WeightService_FilterPush(&filter, latest_raw_value);
                if (tare_ready != 0U)
                {
                    BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
                    weight_fault_reported = 0U;
                }
            }
            else
            {
                BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY);
                if (weight_fault_reported == 0U)
                {
                    BinaryProtocolService_ReportFault((uint16_t)latest_status,
                                                      BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT,
                                                      BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                                      (int32_t)latest_status,
                                                      0U);
                    weight_fault_reported = 1U;
                }
            }
        }
        else
        {
            latest_status = HX711_STATUS_TIMEOUT;
        }

        WeightService_ProcessCommand(&hx711,
                                     latest_status,
                                     latest_raw_value,
                                     &tare_ready,
                                     &filter);
        osDelay(WEIGHT_SERVICE_SAMPLE_PERIOD_MS);
    }
}
