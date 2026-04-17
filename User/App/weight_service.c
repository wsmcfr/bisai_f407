#include "weight_service.h"

#include "cmsis_os.h"
#include "hx711.h"
#include "uart_command.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

/**
 * @brief 周期性重量采样周期，单位毫秒。
 *
 * 当前任务不再周期主动上报串口，因此这里只需要保证重量值持续更新即可。
 * 50ms 可以兼顾响应速度和CPU占用。
 */
#define WEIGHT_SERVICE_SAMPLE_PERIOD_MS   (50U)

/**
 * @brief 启动时空载去皮采样次数。
 *
 * 通过多次平均降低单次抖动对offset的影响。当前数值适合作为初始默认值。
 */
#define WEIGHT_SERVICE_TARE_SAMPLES       (10U)

/**
 * @brief 单次等待HX711数据就绪的超时时间，单位毫秒。
 *
 * 若在该时间内仍未就绪，通常说明接线、供电或时序存在问题，
 * 应由上层通过串口看到超时提示后继续排查。
 */
#define WEIGHT_SERVICE_READ_TIMEOUT_MS    (100U)

/**
 * @brief 串口发送文本消息。
 * @param text 以`\0`结尾的ASCII文本。
 *
 * 该函数只在任务上下文中调用，使用阻塞式发送即可，避免增加额外DMA TX复杂度。
 */
static void WeightService_SendText(const char *text)
{
    size_t text_length;

    if (text == NULL)
    {
        return;
    }

    text_length = strlen(text);
    if (text_length == 0U)
    {
        return;
    }

    /* 这里只在任务上下文发送，因此使用阻塞发送即可，逻辑最简单稳定。 */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)text_length, 100U);
}

/**
 * @brief 输出HX711读数或错误信息。
 * @param hx711 HX711句柄指针，不能为空。
 * @param status 本次读数状态。
 * @param raw_value 原始读数。
 *
 * 该函数把底层原始值和换算后的克重一起输出，便于后续调试标定系数。
 */
static void WeightService_ReportSample(const HX711_Handle_t *hx711, HX711_Status_t status, int32_t raw_value)
{
    char tx_buffer[128];
    float weight_grams;
    int32_t weight_x100;
    int32_t weight_fraction;

    if (status != HX711_STATUS_OK)
    {
        /* 当最近一次采样失败时，直接把错误状态回给上位机，避免返回伪造重量。 */
        (void)snprintf(tx_buffer,
                       sizeof(tx_buffer),
                       "[ERROR][WEIGHT] HX711 read failed, status=%d\r\n",
                       (int)status);
        WeightService_SendText(tx_buffer);
        return;
    }

    /* 先把浮点重量转成两位小数的定点格式，避免依赖浮点printf。 */
    weight_grams = HX711_ConvertToGrams(hx711, raw_value);
    weight_x100 = (int32_t)(weight_grams * 100.0f);
    weight_fraction = weight_x100 % 100;
    if (weight_fraction < 0)
    {
        weight_fraction = -weight_fraction;
    }

    (void)snprintf(tx_buffer,
                   sizeof(tx_buffer),
                   "raw=%ld, weight=%ld.%02ld g\r\n",
                   (long)raw_value,
                   (long)(weight_x100 / 100),
                   (long)weight_fraction);
    WeightService_SendText(tx_buffer);
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
        /* 先跳过帧头多余空白，兼容上位机发送时附带的换行和空格。 */
        ++read_index;
    }

    while (command_buffer[read_index] != '\0')
    {
        current_char = command_buffer[read_index];
        if ((current_char >= 'a') && (current_char <= 'z'))
        {
            /* 统一转大写，后续命令解析只需要维护一套关键字。 */
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
        /* 去掉帧尾空白，确保 `GET`、`GET\r\n`、`GET ` 解析结果一致。 */
        --write_index;
    }

    command_buffer[write_index] = '\0';
}

/**
 * @brief 根据上位机命令决定是否返回重量。
 * @param hx711 HX711句柄指针，不能为空。
 * @param latest_status 最近一次称重状态。
 * @param latest_raw_value 最近一次原始采样值。
 *
 * 当前仅支持一个最小命令：
 * - `GET`：返回最近一次重量值
 *
 * 这样可以满足“发送指令才发送数据”的需求，不再周期主动打印重量。
 */
static void WeightService_ProcessCommand(const HX711_Handle_t *hx711,
                                         HX711_Status_t latest_status,
                                         int32_t latest_raw_value)
{
    char command_buffer[64];

    if (UartCommand_Fetch(command_buffer, sizeof(command_buffer)) == 0U)
    {
        /* 没有新命令时直接返回，任务继续执行称重采样即可。 */
        return;
    }

    WeightService_NormalizeCommand(command_buffer);

    if (strcmp(command_buffer, "GET") == 0)
    {
        /* 按用户要求，只有收到查询命令时才回传当前重量。 */
        WeightService_ReportSample(hx711, latest_status, latest_raw_value);
    }
    else
    {
        /* 暂未定义的命令统一返回提示，避免上位机误以为链路无响应。 */
        WeightService_SendText("[ERROR][UART] Unknown command. Use GET.\r\n");
    }
}

/**
 * @brief 电子称主任务。
 * @param argument FreeRTOS任务参数，当前未使用。
 *
 * 该任务的主流程如下：
 * 1. 加载HX711默认配置并初始化GPIO；
 * 2. 执行一次空载去皮，建立offset基线；
 * 3. 启动串口DMA+空闲中断接收；
 * 4. 周期更新最近一次重量值；
 * 5. 仅当收到上位机查询命令时，返回当前重量。
 */
void WeightService_Task(void *argument)
{
    HX711_Handle_t hx711;
    HX711_Status_t latest_status;
    int32_t latest_raw_value = 0;

    (void)argument;

    HX711_LoadDefaultConfig(&hx711);
    latest_status = HX711_Init(&hx711);
    if (latest_status != HX711_STATUS_OK)
    {
        WeightService_SendText("[ERROR][WEIGHT] HX711 init failed.\r\n");
        for (;;)
        {
            osDelay(1000U);
        }
    }

    /* 任务启动时先做一次去皮，让后续查询返回的是相对净重。 */
    latest_status = HX711_Tare(&hx711, WEIGHT_SERVICE_TARE_SAMPLES, WEIGHT_SERVICE_READ_TIMEOUT_MS);

    /* 启动串口接收后，上位机随时都可以发 GET 查询当前重量。 */
    UartCommand_StartReceive();

    for (;;)
    {
        /* 主循环只负责持续刷新“最近一次采样值”，不主动周期上报。 */
        latest_status = HX711_ReadRaw(&hx711, &latest_raw_value, WEIGHT_SERVICE_READ_TIMEOUT_MS);

        /* 每轮采样后都检查是否有新的串口命令待处理。 */
        WeightService_ProcessCommand(&hx711, latest_status, latest_raw_value);
        osDelay(WEIGHT_SERVICE_SAMPLE_PERIOD_MS);
    }
}
