#include "main.h"
#include "uart_command.h"
#include "usart.h"

#include <string.h>

/**
 * @brief DMA接收缓存区大小。
 *
 * 当前命令集非常小，64字节足够容纳一条文本命令以及换行符。
 * 若后续扩展复杂协议，再统一调整，不在本轮提前放大。
 */
#define UART_COMMAND_RX_DMA_BUFFER_SIZE   (64U)

/**
 * @brief DMA接收原始缓存区。
 *
 * 该缓存由DMA直接写入，因此必须保持静态存储期。
 */
static uint8_t g_uart_dma_rx_buffer[UART_COMMAND_RX_DMA_BUFFER_SIZE];

/**
 * @brief ISR与任务之间共享的待处理命令缓存。
 *
 * 回调在中断上下文中把一帧串口数据复制到这里，
 * 任务上下文再统一取走并执行解析。
 */
static volatile uint8_t g_uart_pending_command[UART_COMMAND_RX_DMA_BUFFER_SIZE];
static volatile uint16_t g_uart_pending_length = 0U;
static volatile uint8_t g_uart_command_ready = 0U;

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
         * 半传输中断会导致命令还没收完整时就进入回调。
         * 本任务是短文本命令接收，因此直接关闭HT中断，只保留空闲/完成事件。
         */
        if (huart1.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }
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
    /* 启动新接收前先清空“已有命令待处理”的状态位。 */
    g_uart_pending_length = 0U;
    g_uart_command_ready = 0U;

    /* DMA原始缓存清零主要是为了调试时观察更直观，不依赖其内容是否为0。 */
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    UartCommand_RestartReceive();
}

/**
 * @brief 从ISR共享缓存中安全取出一条完整命令。
 * @param command_buffer 调用者提供的输出缓存区。
 * @param buffer_size 输出缓存区大小，必须大于0。
 * @return uint8_t 1表示成功取到命令，0表示当前没有待处理命令。
 *
 * 该函数运行在任务上下文。由于命令缓冲区会被中断回调异步写入，
 * 因此这里把“检查是否有命令”和“复制命令内容”都放进同一个临界区，
 * 避免读取过程中被ISR打断后拿到半更新数据。
 */
uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size)
{
    uint16_t copy_length;

    if ((command_buffer == NULL) || (buffer_size == 0U))
    {
        return 0U;
    }

    /* 进入临界区，保证“判空 + 拷贝 + 清标志”是一组原子操作。 */
    __disable_irq();

    if (g_uart_command_ready == 0U)
    {
        __enable_irq();
        return 0U;
    }

    copy_length = g_uart_pending_length;
    if (copy_length >= buffer_size)
    {
        copy_length = (uint16_t)(buffer_size - 1U);
    }

    /* 将ISR缓存拷贝到调用者缓冲区，避免后续解析直接操作共享内存。 */
    (void)memcpy(command_buffer, (const void *)g_uart_pending_command, copy_length);
    command_buffer[copy_length] = '\0';

    /* 命令一旦被取走，就立刻清空待处理标志，等待下一帧串口数据。 */
    g_uart_pending_length = 0U;
    g_uart_command_ready = 0U;

    __enable_irq();

    return 1U;
}

/**
 * @brief HAL串口DMA+空闲中断接收事件回调。
 * @param huart 当前触发回调的串口句柄。
 * @param Size 本次已经收到的数据长度。
 *
 * 该回调运行在中断上下文中，因此只做三件事：
 * 1. 过滤非USART1事件；
 * 2. 把本次收到的数据搬运到待处理缓存；
 * 3. 立刻重启下一轮DMA接收。
 *
 * 真正的命令解析与应答统一留到任务上下文执行。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
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

    copy_length = Size;
    if (copy_length >= UART_COMMAND_RX_DMA_BUFFER_SIZE)
    {
        copy_length = (uint16_t)(UART_COMMAND_RX_DMA_BUFFER_SIZE - 1U);
    }

    /* ISR中只做最小搬运，把一整帧命令快照到共享缓存，不在这里做字符串解析。 */
    (void)memcpy((void *)g_uart_pending_command, g_uart_dma_rx_buffer, copy_length);
    g_uart_pending_command[copy_length] = '\0';
    g_uart_pending_length = copy_length;
    g_uart_command_ready = 1U;

    /* 清空DMA缓存后立即重新挂起下一轮接收，保证连续命令不会丢。 */
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    UartCommand_RestartReceive();
}
