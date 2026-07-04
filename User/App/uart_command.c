#include "uart_command.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief USART1 命令接收底座与业务命令分发约定。
 *
 * 本文件只负责“收完整一帧”和“串口安全打印”，不直接解释业务含义。
 * 用户真正能从 USART1 发送的命令由上层服务解释，维护时必须按下面索引同步更新：
 * | 命令类型 | 示例 | 处理文件 | 主要效果 | 是否回包 |
 * | --- | --- | --- | --- | --- |
 * | MP157 心跳命令 | 二进制 `HEARTBEAT` | `binary_protocol_service.c` | 查询 STM32F407 主任务和 USART1 二进制协议入口是否在线 | 返回二进制 `ACK` 或 `NACK` |
 * | 自动检测 HEX 帧 | `A5 5A 01 10 ... 6B` | `binary_protocol_service.c` | MP157 控制自动检测流程，例如开始、暂停、继续、停止和视觉坐标闭环 | 正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT` |
 * | 称重调试入口 | `GET` / `TARE` / `CAL 1000` | `weight_service.c` | 仅作为串口助手维护入口；MP157 主链路不再等待文本回包 | USART1 文本默认静默，正式联调需补二进制命令 |
 * | LDC 标定入口 | `LDCCAL CH1 20` / `LDCSTOP` | `ldc1614_service.c` | 仅作为串口助手维护入口；自动流程故障走 `FAULT_REPORT` | USART1 文本默认静默 |
 * | 传送带调试入口 | 二进制 `BELT_MANUAL_CONTROL` / `QUERY_STATUS` | `binary_protocol_service.c` | 切换巡航、停止或查询结构化状态 | 正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT` |
 * | 摄像头电机维护入口 | `CAMINFO` / `CAMSTOP` / `CAMLAT LEFT 30` / `CAMLAT RIGHT 30` / `CAMZ UP 30` | `camera_motor_service.c` | 调试摄像头左右轴和上下轴，两个电机共用 USART6 但地址不同 | USART1 文本默认静默，自动流程走二进制执行器命令 |
 * | 机械臂 HEX 帧 | `55 55 02 01` / `55 55 05 06 03 01 00` | `robot_arm_service.c` | 透传到 USART3/ESP32，查询或执行 LeArm 动作 | 查询类有 `[ARM] RX...` 日志，运动类通常看机械臂动作 |
 *
 * 串口链路：
 * - USART1：115200 8N1，PA9(TX)/PA10(RX)，面向串口助手、MP157 或其它上位机；
 * - 本文件使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收，空闲中断认为“一帧命令结束”；
 * - 文本命令通过 `UartCommand_Fetch()` 取出并补 `\0`，自动检测二进制帧和机械臂二进制帧通过
 *   `UartCommand_FetchRaw()` 取出，避免帧内 `0x00` 被字符串逻辑截断。
 *
 * 维护要求：
 * 1. 后续新增 USART1 可发送命令时，必须在本注释表和具体处理文件的命令表里同时写清楚“能发什么、有什么效果、正确帧和错误帧分别是什么”；
 * 2. 本文件只允许做接收缓存、ISR 到任务通知、串口打印互斥，不把业务状态机塞进串口底座；
 * 3. HAL 回调中只复制数据和释放信号量，禁止在中断上下文解析命令或格式化打印。
 */

/**
 * @brief DMA接收缓存区大小。
 *
 * 当前命令集非常小，64字节足够容纳一条文本命令以及换行符。
 * 若后续扩展复杂协议，再统一调整，不在本轮提前放大。
 */
#define UART_COMMAND_RX_DMA_BUFFER_SIZE   (64U)

/**
 * @brief USART1 文本输出总开关。
 *
 * 当前 USART1 是 STM32MP157 与 F407 的主控制链路。
 * 自动检测阶段只允许 ACK/NACK/STATUS_REPORT/FAULT_REPORT 等二进制帧返回，
 * 所以默认关闭 `my_printf(&huart1, ...)` 的文本发送，防止 `[OK]`、`[ERROR]`
 * 这类人工调试日志混入 MP157 的二进制解析窗口。
 *
 * 若后续需要用 Windows 串口助手临时看文本日志，可以在现场调试固件中改为 1U；
 * 正式接 MP157 时必须保持 0U。
 */
#define UART_COMMAND_USART1_TEXT_ENABLE    (0U)

/**
 * @brief 串口发送格式化缓存区大小。
 *
 * 当前仅用于输出状态、错误和重量文本，128字节足够覆盖本任务。
 */
/*
 * 机械臂调试日志改为更长的可读英文句子，避免 ARMCC5 解析 UTF-8 中文字符串时报错。
 * 384 字节可以容纳动作组排查建议和接线检查提示，避免日志被截断。
 */
#define UART_COMMAND_TX_BUFFER_SIZE       (384U)

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
/**
 * @brief 从接收模块中安全取出一帧原始串口数据。
 * @param frame_buffer 调用者提供的原始字节输出缓存，不能为 NULL。
 * @param buffer_size 输出缓存大小，必须大于0。
 * @param frame_length 实际拷贝出的字节数输出参数，不能为 NULL。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到一帧数据，0表示没有新数据或参数非法。
 *
 * 该函数和 UartCommand_Fetch 使用同一个 USART1 单消费者缓存。
 * 与文本接口不同的是，这里不会补字符串结束符，也不会因为帧内存在 0x00 而提前截断，
 * 因此可用于机械臂 `55 55 ...` 二进制协议透传。
 */
uint8_t UartCommand_FetchRaw(uint8_t *frame_buffer,
                             uint16_t buffer_size,
                             uint16_t *frame_length,
                             uint32_t timeout_ms)
{
    uint16_t copy_length;
    TickType_t wait_ticks;

    if ((frame_buffer == NULL) ||
        (buffer_size == 0U) ||
        (frame_length == NULL) ||
        (g_uart_rx_semaphore == NULL))
    {
        return 0U;
    }

    *frame_length = 0U;

    wait_ticks = (timeout_ms == 0U) ? 0U : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(g_uart_rx_semaphore, wait_ticks) != pdTRUE)
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    copy_length = g_uart_pending_length;
    if (copy_length > buffer_size)
    {
        /*
         * 调用者缓存比 DMA 缓存小时，只复制调用者能容纳的部分。
         * 当前机械臂帧和文本命令都很短，正常不会触发该分支；保留该边界处理防止越界。
         */
        copy_length = buffer_size;
    }

    (void)memcpy(frame_buffer, g_uart_pending_command, copy_length);
    *frame_length = copy_length;

    /* 当前帧已交给唯一消费者，清零长度等待下一次 DMA 空闲中断写入新帧。 */
    g_uart_pending_length = 0U;

    taskEXIT_CRITICAL();

    return 1U;
}

/**
 * @brief 从接收模块中安全取出一条文本命令。
 * @param command_buffer 调用者提供的文本输出缓存，不能为 NULL。
 * @param buffer_size 文本输出缓存大小，必须大于0，并且函数会预留 1 字节写入 `\0`。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到命令，0表示没有新命令或参数非法。
 *
 * 该接口适合 `GET/TARE/CAL/BELT...` 等 ASCII 文本命令。
 * 如果需要保留帧内 `0x00`，例如机械臂二进制协议，应使用 UartCommand_FetchRaw。
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
     * USART1 面向 MP157 时只允许二进制协议帧返回。
     * 这里直接丢弃 USART1 文本日志，但保留返回 0，表示调用者无需因为“调试文本未发送”
     * 改变业务状态；其它串口如果复用 my_printf，仍按原逻辑输出。
     */
    if ((huart == &huart1) && (UART_COMMAND_USART1_TEXT_ENABLE == 0U))
    {
        return 0;
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
 * @brief 串口线程安全原始字节发送。
 * @param huart 目标串口句柄，不能为空。
 * @param data 待发送的原始字节缓存，不能为 NULL。
 * @param length 待发送字节数，必须大于 0。
 * @param timeout_ms HAL_UART_Transmit 的阻塞发送超时时间，单位毫秒。
 * @return HAL_StatusTypeDef HAL 串口发送结果。
 *
 * 设计原因：
 * 1. 二进制 ACK/NACK 帧中可能包含 `0x00`，不能使用 `printf` 风格字符串发送；
 * 2. USART1 同时会输出文本调试日志，必须和 `my_printf()` 使用同一把互斥锁；
 * 3. 该函数只负责“原样发送字节”，不解释协议，也不追加 `\r\n`。
 */
HAL_StatusTypeDef UartCommand_SendRaw(UART_HandleTypeDef *huart,
                                      const uint8_t *data,
                                      uint16_t length,
                                      uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((huart == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * 和 my_printf() 保持同样的惰性初始化策略。
     * 这样即使二进制协议早于文本日志发送，也能获得互斥保护。
     */
    if (g_uart_tx_mutex == NULL)
    {
        UartCommand_InitSyncObjects();
    }

    if (g_uart_tx_mutex != NULL)
    {
        (void)xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }

    status = HAL_UART_Transmit(huart, (uint8_t *)data, length, timeout_ms);

    if (g_uart_tx_mutex != NULL)
    {
        (void)xSemaphoreGive(g_uart_tx_mutex);
    }

    return status;
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
