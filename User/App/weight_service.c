#include "weight_service.h"

#include "cmsis_os.h"
#include "hx711.h"
#include "ldc1614_service.h"
#include "uart_command.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

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
    int32_t raw_samples[WEIGHT_SERVICE_MEDIAN_FILTER_SIZE];
    uint8_t sample_count;
    uint8_t write_index;
} WeightService_Filter_t;

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
        my_printf(&huart1, "[ERROR][WEIGHT] HX711 read failed, status=%d\r\n", (int)status);
        return;
    }

    if (tare_ready == 0U)
    {
        /*
         * 去皮尚未成功时，绝不把结果解释成重量。
         * 这样可以避免用户看到一个“看似正常”的假克重。
         */
        my_printf(&huart1, "[ERROR][WEIGHT] Tare not ready. raw=%ld\r\n", (long)raw_value);
        return;
    }

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
 * 当前支持以下命令：
 * - `GET`：返回当前重量或净计数差值
 * - `TARE`：重新执行一次去皮
 * - `CAL <克重>`：用当前带载值和已知砝码重量完成标定
 * - `LDCCAL CHx [N]`：启动 LDC 通道的稳定批量采样模式
 * - `LDCSTOP`：停止当前 LDC 标定采样会话
 */
static void WeightService_ProcessCommand(HX711_Handle_t *hx711,
                                         HX711_Status_t latest_status,
                                         int32_t latest_raw_value,
                                         uint8_t *tare_ready,
                                         WeightService_Filter_t *filter)
{
    char command_buffer[64];
    uint32_t known_weight_g;
    HX711_Status_t status;

    if ((hx711 == NULL) || (tare_ready == NULL) || (filter == NULL))
    {
        return;
    }

    if (UartCommand_Fetch(command_buffer,
                          sizeof(command_buffer),
                          WEIGHT_SERVICE_COMMAND_WAIT_MS) == 0U)
    {
        return;
    }

    WeightService_NormalizeCommand(command_buffer);

    if (strcmp(command_buffer, "GET") == 0)
    {
        WeightService_ReportSample(hx711, latest_status, latest_raw_value, *tare_ready);
    }
    else if (strcmp(command_buffer, "TARE") == 0)
    {
        status = WeightService_ExecuteTare(hx711, tare_ready, filter);
        if (status == HX711_STATUS_OK)
        {
            my_printf(&huart1, "[OK][WEIGHT] Tare success. offset=%ld\r\n", (long)hx711->offset);
        }
        else
        {
            my_printf(&huart1, "[ERROR][WEIGHT] Tare failed, status=%d\r\n", (int)status);
        }
    }
    else if (WeightService_ParseCalibrationWeight(command_buffer, &known_weight_g) == 1U)
    {
        if (*tare_ready == 0U)
        {
            my_printf(&huart1, "[ERROR][WEIGHT] CAL rejected. Tare is not ready.\r\n");
        }
        else if ((known_weight_g == 0U) || ((float)known_weight_g > hx711->rated_capacity_g))
        {
            my_printf(&huart1,
                      "[ERROR][WEIGHT] CAL rejected. Weight must be 1~%ld g.\r\n",
                      (long)hx711->rated_capacity_g);
        }
        else if (latest_status != HX711_STATUS_OK)
        {
            my_printf(&huart1,
                      "[ERROR][WEIGHT] CAL rejected. Latest sample invalid, status=%d\r\n",
                      (int)latest_status);
        }
        else
        {
            status = HX711_CalibrateByKnownWeight(hx711, latest_raw_value, (float)known_weight_g);
            if (status == HX711_STATUS_OK)
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
            else
            {
                my_printf(&huart1, "[ERROR][WEIGHT] Calibration failed, status=%d\r\n", (int)status);
            }
        }
    }
    else if (Ldc1614Service_HandleCommand(command_buffer) != 0U)
    {
        /* LDC 命令已由对应模块接管，这里不再重复输出。 */
    }
    else
    {
        my_printf(&huart1,
                  "[ERROR][UART] Unknown command. Use GET / TARE / CAL <g> / LDCCAL CHx [N] / LDCSTOP.\r\n");
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

    (void)argument;

    WeightService_FilterReset(&filter);
    UartCommand_StartReceive();

    HX711_LoadDefaultConfig(&hx711);
    latest_status = HX711_Init(&hx711);
    if (latest_status != HX711_STATUS_OK)
    {
        my_printf(&huart1, "[ERROR][WEIGHT] HX711 init failed.\r\n");
        for (;;)
        {
            osDelay(1000U);
        }
    }

    /*
     * 启动阶段先尝试一次自动去皮。
     * 若此时失败，不再像之前那样静默继续，而是保留错误状态并允许用户稍后手动发送 TARE。
     */
    latest_status = WeightService_ExecuteTare(&hx711, &tare_ready, &filter);
    if (latest_status == HX711_STATUS_OK)
    {
        my_printf(&huart1, "[OK][WEIGHT] Startup tare success. offset=%ld\r\n", (long)hx711.offset);
    }
    else
    {
        my_printf(&huart1,
                  "[ERROR][WEIGHT] Startup tare failed, status=%d. Use TARE after checking wiring.\r\n",
                  (int)latest_status);
    }

    my_printf(&huart1,
              "[INFO][WEIGHT] HX711 ready. DOUT=PB0, SCK=PB2, capacity=%ld g, default CAL=%u g\r\n",
              (long)hx711.rated_capacity_g,
              (unsigned int)WEIGHT_SERVICE_DEFAULT_CAL_WEIGHT_G);

    for (;;)
    {
        /*
         * 主循环只负责持续刷新“最近一次滤波后采样值”，
         * 不主动周期上报。
         */
        latest_status = HX711_ReadRaw(&hx711, &latest_raw_value, WEIGHT_SERVICE_READ_TIMEOUT_MS);
        if (latest_status == HX711_STATUS_OK)
        {
            latest_raw_value = WeightService_FilterPush(&filter, latest_raw_value);
        }

        WeightService_ProcessCommand(&hx711,
                                     latest_status,
                                     latest_raw_value,
                                     &tare_ready,
                                     &filter);
        osDelay(WEIGHT_SERVICE_SAMPLE_PERIOD_MS);
    }
}
