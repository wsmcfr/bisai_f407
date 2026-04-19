#include "uart_command.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief DMA接收缓存区大小。
 *
 * 当前命令集非常小，64字节足够容纳一条文本命令以及换行符。
 * 若后续扩展复杂协议，再统一调整，不在本轮提前放大。
 */
#define UART_COMMAND_RX_DMA_BUFFER_SIZE   (64U)

/**
 * @brief 串口发送格式化缓存区大小。
 *
 * 当前仅用于输出状态、错误和重量文本，128字节足够覆盖本任务。
 */
#define UART_COMMAND_TX_BUFFER_SIZE       (128U)

/**
 * @brief DMA接收原始缓存区。
 *
 * 该缓存由DMA直接写入，因此必须保持静态存储期。
 */
static uint8_t g_uart_dma_rx_buffer[UART_COMMAND_RX_DMA_BUFFER_SIZE];

/**
 * @brief ISR与任务之间共享的单帧命令缓存。
 *
 * 本项目当前命令非常短，而且业务模型是“上位机发一条命令，设备回一条结果”，
 * 因此保留最近一帧即可，无需额外引入环形缓冲区。
 */
static uint8_t g_uart_pending_command[UART_COMMAND_RX_DMA_BUFFER_SIZE];
static volatile uint16_t g_uart_pending_length = 0U;

/**
 * @brief 串口同步对象。
 *
 * - 接收信号量：表示“有一帧新命令已到达”
 * - 发送互斥锁：保证多个任务打印时内容不会互相穿插
 */
static SemaphoreHandle_t g_uart_rx_semaphore = NULL;
static SemaphoreHandle_t g_uart_tx_mutex = NULL;

/**
 * @brief 重新启动一次USART1 DMA+空闲中断接收。
 *
 * HAL在某次接收事件结束后，需要用户重新挂起下一次接收。
 * 这里统一封装，避免同一逻辑散落在多个位置。
 */
static void UartCommand_RestartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                     g_uart_dma_rx_buffer,
                                     UART_COMMAND_RX_DMA_BUFFER_SIZE) == HAL_OK)
    {
        /*
         * 半传输中断会导致命令还没收完整就进入回调。
         * 对短命令场景而言，这只会增加噪声，因此直接关闭。
         */
        if (huart1.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }
    }
}

/**
 * @brief 初始化串口同步资源。
 *
 * 该函数只在首次启动时创建资源，避免重复创建导致句柄泄漏。
 */
static void UartCommand_InitSyncObjects(void)
{
    if (g_uart_tx_mutex == NULL)
    {
        g_uart_tx_mutex = xSemaphoreCreateMutex();
    }

    if (g_uart_rx_semaphore == NULL)
    {
        g_uart_rx_semaphore = xSemaphoreCreateBinary();
    }
}

/**
 * @brief 初始化串口命令接收状态并启动第一轮DMA接收。
 *
 * 该函数在任务初始化阶段调用，负责清空上一轮残留状态，
 * 然后挂起一次新的 `DMA + IDLE` 接收流程。
 */
void UartCommand_StartReceive(void)
{
    UartCommand_InitSyncObjects();

    /* 启动前先清空共享缓存，避免误读到上电前残留内容。 */
    g_uart_pending_length = 0U;
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    (void)memset(g_uart_pending_command, 0, sizeof(g_uart_pending_command));

    UartCommand_RestartReceive();
}

/**
 * @brief 从接收模块中安全取出一条完整命令。
 * @param command_buffer 调用者提供的输出缓存区。
 * @param buffer_size 输出缓存区大小，必须大于0。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到命令，0表示未取到命令。
 *
 * 该函数先等待接收信号量，再在临界区内复制单帧缓存。
 * 因为当前只保留“最近一帧”，所以这里的逻辑比环形缓冲区更轻量。
 */
uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size, uint32_t timeout_ms)
{
    uint16_t copy_length;
    TickType_t wait_ticks;

    if ((command_buffer == NULL) || (buffer_size == 0U) || (g_uart_rx_semaphore == NULL))
    {
        return 0U;
    }

    wait_ticks = (timeout_ms == 0U) ? 0U : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(g_uart_rx_semaphore, wait_ticks) != pdTRUE)
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    copy_length = g_uart_pending_length;
    if (copy_length >= buffer_size)
    {
        copy_length = (uint16_t)(buffer_size - 1U);
    }

    (void)memcpy(command_buffer, g_uart_pending_command, copy_length);
    command_buffer[copy_length] = '\0';

    /* 当前帧被取走后立刻清空长度，等待下一帧覆盖写入。 */
    g_uart_pending_length = 0U;

    taskEXIT_CRITICAL();

    return 1U;
}

/**
 * @brief 串口线程安全格式化输出。
 * @param huart 目标串口句柄，不能为空。
 * @param format `printf` 风格格式串。
 * @return int 实际输出字符数；小于0表示格式化失败。
 *
 * 该接口统一管理串口发送路径，后续若新增更多任务打印，也无需到处复制发送代码。
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    static char tx_buffer[UART_COMMAND_TX_BUFFER_SIZE];
    va_list argument_list;
    int text_length;

    if ((huart == NULL) || (format == NULL))
    {
        return -1;
    }

    /*
     * 项目里已经出现多个任务共用同一串口打印，
     * 因此这里补一层惰性初始化，避免某个任务首次打印早于
     * `UartCommand_StartReceive()` 执行，从而丢失发送互斥保护。
     */
    if (g_uart_tx_mutex == NULL)
    {
        UartCommand_InitSyncObjects();
    }

    if (g_uart_tx_mutex != NULL)
    {
        (void)xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }

    va_start(argument_list, format);
    text_length = vsnprintf(tx_buffer, sizeof(tx_buffer), format, argument_list);
    va_end(argument_list);

    if (text_length < 0)
    {
        if (g_uart_tx_mutex != NULL)
        {
            (void)xSemaphoreGive(g_uart_tx_mutex);
        }
        return text_length;
    }

    if (text_length >= (int)sizeof(tx_buffer))
    {
        /* 输出过长时直接截断，保证发送长度始终合法。 */
        text_length = (int)sizeof(tx_buffer) - 1;
    }

    (void)HAL_UART_Transmit(huart, (uint8_t *)tx_buffer, (uint16_t)text_length, 0xFFU);

    if (g_uart_tx_mutex != NULL)
    {
        (void)xSemaphoreGive(g_uart_tx_mutex);
    }

    return text_length;
}

/**
 * @brief HAL串口DMA+空闲中断接收事件回调。
 * @param huart 当前触发回调的串口句柄。
 * @param Size 本次已经收到的数据长度。
 *
 * 该回调运行在中断上下文中，因此只做四件事：
 * 1. 过滤非USART1事件；
 * 2. 停止当前DMA会话，避免复制过程中仍被DMA改写；
 * 3. 把本次收到的一整帧复制到共享缓存；
 * 4. 立即重启下一轮DMA接收，并释放信号量通知任务。
 *
 * 这里的“释放信号量”只发生在 HAL 已经判定一帧接收结束之后，
 * 因此任务侧拿到的始终是“完整一帧”，而不是半包数据。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint16_t copy_length;

    if ((huart != &huart1) || (Size == 0U))
    {
        if (huart == &huart1)
        {
            /* 即便本次没有拿到有效数据，也要恢复下一轮接收，避免链路中断。 */
            UartCommand_RestartReceive();
        }
        return;
    }

    /* 先停DMA，再复制完整帧，避免任务侧读到DMA尚未写完的脏数据。 */
    (void)HAL_UART_DMAStop(huart);

    copy_length = Size;
    if (copy_length >= UART_COMMAND_RX_DMA_BUFFER_SIZE)
    {
        copy_length = (uint16_t)(UART_COMMAND_RX_DMA_BUFFER_SIZE - 1U);
    }

    (void)memcpy(g_uart_pending_command, g_uart_dma_rx_buffer, copy_length);
    g_uart_pending_command[copy_length] = '\0';
    g_uart_pending_length = copy_length;

    /* 清空DMA缓存后立即重启下一轮接收，保证后续命令仍能继续进入。 */
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    UartCommand_RestartReceive();

    if (g_uart_rx_semaphore != NULL)
    {
        /*
         * 当前采用“最近一帧覆盖旧帧”的策略。
         * 这对短文本交互足够简单有效，也符合本任务“用户发指令才查询”的使用方式。
         */
        (void)xSemaphoreGiveFromISR(g_uart_rx_semaphore, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
