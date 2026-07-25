#include "uart_command.h"

#include "FreeRTOS.h"
#include "robot_arm_service.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief MP157 主链路命令接收底座与业务命令分发约定。
 *
 * 本文件只负责“接收 MP157 主链路原始字节块”和“串口安全发送”，不直接解释业务含义。
 * 用户真正能从 MP157 主链路发送的命令由上层服务解释，维护时必须按下面索引同步更新：
 * | 命令类型 | 示例 | 处理文件 | 主要效果 | 是否回包 |
 * | --- | --- | --- | --- | --- |
 * | MP157 心跳命令 | 二进制 `HEARTBEAT` | `binary_protocol_service.c` | 查询 STM32F407 主任务和 USART1 二进制协议入口是否在线 | 返回二进制 `ACK` 或 `NACK` |
 * | 自动检测 HEX 帧 | `A5 5A 01 10 ... 6B` | `binary_protocol_service.c` | MP157 控制自动检测流程，例如开始、暂停、继续、停止和视觉坐标闭环 | 正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT` |
 * | 称重调试入口 | `GET` / `TARE` / `CAL 1000` | `weight_service.c` | 仅作为串口助手维护入口；MP157 主链路不再等待文本回包 | USART1 文本默认静默，正式联调需补二进制命令 |
 * | LDC 标定入口 | `LDCCAL CH1 20` / `LDCSTOP` | `ldc1614_service.c` | 仅作为串口助手维护入口；自动流程故障走 `FAULT_REPORT` | USART1 文本默认静默 |
 * | 传送带调试入口 | 二进制 `BELT_MANUAL_CONTROL` / `QUERY_STATUS` | `binary_protocol_service.c` | 切换巡航、停止或查询结构化状态 | 正确返回 `ACK/STATUS_REPORT`，错误返回 `NACK/FAULT_REPORT` |
 * | 摄像头电机维护入口 | `CAMINFO` / `CAMSTOP` / `CAMLAT LEFT 30` / `CAMLAT RIGHT 30` / `CAMZ UP 30` | `camera_motor_service.c` | 调试摄像头左右轴和上下轴，两个电机共用 USART6 但地址不同 | USART1 文本默认静默，自动流程走二进制执行器命令 |
 * | 机械臂正式帧 | `A5 5A 01 20 ... 6B` | `robot_arm_service.c` | 通过 USART3 发给 ESP32S3，并等待 ACK/DONE | `[ARM] ACK OK` 后继续等 `[ARM] DONE received` |
 *
 * 串口链路：
 * - USART1：115200 8N1，PA9(TX)/PA10(RX)，面向 MP157 或串口助手维护输入；
 * - USART2：115200 8N1，PA2(TX)/PA3(RX)，当前不再作为 MP157 正式主链路；
 * - 本文件使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收，空闲中断只表示当前线路暂时空闲；
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
 * @brief MP157 主链路 ISR 到任务之间的原始字节环形缓存容量。
 *
 * MP157 可能连续写入三帧安全 STOP，DMA-IDLE 也可能把多帧合并成一个回调块。
 * 256 字节能够缓存四个 DMA 满块，避免称重任务短时忙于 HX711 时后来的 STOP 覆盖前一块数据。
 */
#define UART_COMMAND_RX_RING_BUFFER_SIZE  (256U)

/**
 * @brief MP157 主链路文本输出总开关。
 *
 * 当前 USART1 是 STM32MP157 与 F407 的主控制链路。
 * 自动检测阶段只允许 ACK/NACK/STATUS_REPORT/FAULT_REPORT 等二进制帧返回，
 * 所以默认关闭 `my_printf(&huart1, ...)` 的文本发送，防止 `[OK]`、`[ERROR]`
 * 这类人工调试日志混入 MP157 的二进制解析窗口。
 *
 * 若后续需要用 Windows 串口助手临时看文本日志，可以在现场调试固件中改为 1U；
 * 正式接 MP157 时必须保持 0U。
 *
 * 由于 USART1 已恢复为 MP157 正式主链路，这里保持 `0U`，保证 MP157-F4 的
 * USART1 正式链路不混入文本调试输出；如需现场日志，应另行迁移到非主链路串口。
 */
#define UART_COMMAND_MP157_TEXT_ENABLE     (0U)

/**
 * @brief MP157-F4 正式主链路串口句柄。
 *
 * 用户已决定把 F4 的 `USART1 PA9/PA10` 接回 MP157 `/dev/ttySTM2` 的 TTL 侧，
 * 所以接收、二进制 ACK/NACK 回包以及文本静默判断都必须以该句柄为准。
 */
#define UART_COMMAND_MP157_HUART           (&huart1)

/**
 * @brief 获取当前 MP157-F407 正式主链路串口句柄。
 * @return UART_HandleTypeDef* 返回接收 MP157 命令、发送 ACK/NACK/STATUS/FAULT 的串口句柄。
 *
 * 该函数让协议发送端复用同一个主链路定义，避免接收路径已经切回 USART1，
 * 但二进制回包仍误发到 USART2 的分裂问题。
 */
UART_HandleTypeDef *UartCommand_GetMp157Huart(void)
{
    return UART_COMMAND_MP157_HUART;
}

/**
 * @brief 串口发送格式化缓存区大小。
 *
 * 当前仅用于输出状态、错误和重量文本，128字节足够覆盖本任务。
 */
/*
 * 机械臂调试日志改为更长的可读英文句子，避免 ARMCC5 解析 UTF-8 中文字符串时报错。
 * 384 字节可以容纳正式协议 ACK/DONE 排查建议和接线检查提示，避免日志被截断。
 */
#define UART_COMMAND_TX_BUFFER_SIZE       (384U)

/**
 * @brief DMA接收原始缓存区。
 *
 * 该缓存由DMA直接写入，因此必须保持静态存储期。
 */
static uint8_t g_uart_dma_rx_buffer[UART_COMMAND_RX_DMA_BUFFER_SIZE];

/**
 * @brief ISR 与任务之间共享的原始字节环形缓存。
 *
 * DMA 回调只追加字节，不在中断中解释协议；任务批量取出后再由流式解析器拆分粘包和半包。
 */
static uint8_t g_uart_rx_ring_buffer[UART_COMMAND_RX_RING_BUFFER_SIZE];
static volatile uint16_t g_uart_rx_ring_read_index = 0U;
static volatile uint16_t g_uart_rx_ring_write_index = 0U;
static volatile uint16_t g_uart_rx_ring_length = 0U;

/**
 * @brief 串口同步对象。
 *
 * - 接收信号量：表示“有一帧新命令已到达”
 * - 发送互斥锁：保证多个任务打印时内容不会互相穿插
 */
static SemaphoreHandle_t g_uart_rx_semaphore = NULL;
static SemaphoreHandle_t g_uart_tx_mutex = NULL;

/**
 * @brief 重新启动一次 MP157 主链路 DMA+空闲中断接收。
 *
 * HAL在某次接收事件结束后，需要用户重新挂起下一次接收。
 * 这里统一封装，避免同一逻辑散落在多个位置。
 */
static void UartCommand_RestartReceive(void)
{
    HAL_StatusTypeDef receive_status;

    receive_status = HAL_UARTEx_ReceiveToIdle_DMA(UART_COMMAND_MP157_HUART,
                                                  g_uart_dma_rx_buffer,
                                                  UART_COMMAND_RX_DMA_BUFFER_SIZE);
    if (receive_status != HAL_OK)
    {
        /* 接收重启失败时直接返回，避免在串口底座层打印调试文本影响正式协议链路。 */
        return;
    }

    /*
     * 半传输中断会导致命令还没收完整就进入回调。
     * 对短命令场景而言，这只会增加噪声，因此直接关闭。
     */
    if (UART_COMMAND_MP157_HUART->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(UART_COMMAND_MP157_HUART->hdmarx, DMA_IT_HT);
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

    /* 启动前清空环形缓存读写位置，避免误读到上电前或复位前残留数据。 */
    g_uart_rx_ring_read_index = 0U;
    g_uart_rx_ring_write_index = 0U;
    g_uart_rx_ring_length = 0U;
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    (void)memset(g_uart_rx_ring_buffer, 0, sizeof(g_uart_rx_ring_buffer));

    UartCommand_RestartReceive();
}

/**
 * @brief 从接收模块中安全取出当前已到达的一批原始串口字节。
 * @param frame_buffer 调用者提供的原始字节输出缓存，不能为 NULL。
 * @param buffer_size 输出缓存大小，必须大于0。
 * @param frame_length 实际拷贝出的字节数输出参数，不能为 NULL。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到至少一个字节，0表示没有新数据或参数非法。
 *
 * 重要边界：一次返回可能包含多帧，也可能只是半帧，DMA-IDLE 不是协议帧边界。
 * 唯一消费者必须把这些字节交给上层流式解析器，不能直接把整块当成一帧。
 */
uint8_t UartCommand_FetchRaw(uint8_t *frame_buffer,
                             uint16_t buffer_size,
                             uint16_t *frame_length,
                             uint32_t timeout_ms)
{
    uint16_t copy_length;
    uint16_t copy_index;
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

    copy_length = g_uart_rx_ring_length;
    if (copy_length > buffer_size)
    {
        /*
         * 调用者缓存小于当前积压字节数时，只取出能容纳的前缀，
         * 剩余字节保留在环形缓存内，下一轮继续取出，避免截断后永久丢失。
         */
        copy_length = buffer_size;
    }

    for (copy_index = 0U; copy_index < copy_length; copy_index++)
    {
        frame_buffer[copy_index] = g_uart_rx_ring_buffer[g_uart_rx_ring_read_index];
        g_uart_rx_ring_read_index = (uint16_t)((g_uart_rx_ring_read_index + 1U) %
                                               UART_COMMAND_RX_RING_BUFFER_SIZE);
    }
    *frame_length = copy_length;
    g_uart_rx_ring_length = (uint16_t)(g_uart_rx_ring_length - copy_length);

    taskEXIT_CRITICAL();

    if ((g_uart_rx_ring_length > 0U) && (g_uart_rx_semaphore != NULL))
    {
        /*
         * 本次调用者缓存没有取完全部积压字节时，重新释放二值信号量，
         * 确保唯一消费者下一轮无需等待新的 DMA 回调也能继续清空剩余数据。
         */
        (void)xSemaphoreGive(g_uart_rx_semaphore);
    }

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

    if ((command_buffer == NULL) || (buffer_size <= 1U))
    {
        return 0U;
    }

    if (UartCommand_FetchRaw((uint8_t *)command_buffer,
                             (uint16_t)(buffer_size - 1U),
                             &copy_length,
                             timeout_ms) == 0U)
    {
        return 0U;
    }

    command_buffer[copy_length] = '\0';
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
     * MP157 主链路只允许二进制协议帧返回。
     * 这里直接丢弃主链路文本日志，但保留返回 0，表示调用者无需因为“调试文本未发送”
     * 改变业务状态；其它串口如果复用 my_printf，仍按原逻辑输出。
     */
    if ((huart == UART_COMMAND_MP157_HUART) && (UART_COMMAND_MP157_TEXT_ENABLE == 0U))
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
 * 2. MP157 主链路会发送二进制回包，必须和 `my_printf()` 使用同一把互斥锁；
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
 * 1. 过滤非 MP157 主链路事件；
 * 2. 停止当前DMA会话，避免复制过程中仍被DMA改写；
 * 3. 把本次收到的原始字节追加到环形缓存；
 * 4. 立即重启下一轮DMA接收，并释放信号量通知任务。
 *
 * HAL 的空闲事件只表示当前线路短暂停顿，不保证恰好位于协议帧边界。
 * 因此任务侧拿到的可能是粘包或半包，必须继续交给 MP157 流式解析器处理。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint16_t copy_length;
    uint16_t copy_index;
    UBaseType_t critical_state;

    /* USART3 现在用单字节中断接收，不再走 RxEventCallback。 */
    if (huart == &huart3)
    {
        return;
    }

    if ((huart != UART_COMMAND_MP157_HUART) || (Size == 0U))
    {
        if (huart == UART_COMMAND_MP157_HUART)
        {
            /* 即便本次没有拿到有效数据，也要恢复下一轮接收，避免链路中断。 */
            UartCommand_RestartReceive();
        }
        return;
    }

    /* 先停DMA，再复制本次原始接收块，避免任务侧读到DMA尚未写完的脏数据。 */
    (void)HAL_UART_DMAStop(huart);

    copy_length = Size;
    if (copy_length > UART_COMMAND_RX_DMA_BUFFER_SIZE)
    {
        copy_length = UART_COMMAND_RX_DMA_BUFFER_SIZE;
    }

    critical_state = taskENTER_CRITICAL_FROM_ISR();
    for (copy_index = 0U; copy_index < copy_length; copy_index++)
    {
        if (g_uart_rx_ring_length >= UART_COMMAND_RX_RING_BUFFER_SIZE)
        {
            /*
             * 环形缓存已满时丢弃最旧字节，优先保留最新 STOP 等安全控制意图。
             * 这里不在中断中打印或上报，避免调试文本影响 USART1 正式协议链路。
             */
            g_uart_rx_ring_read_index = (uint16_t)((g_uart_rx_ring_read_index + 1U) %
                                                   UART_COMMAND_RX_RING_BUFFER_SIZE);
            g_uart_rx_ring_length--;
        }

        g_uart_rx_ring_buffer[g_uart_rx_ring_write_index] = g_uart_dma_rx_buffer[copy_index];
        g_uart_rx_ring_write_index = (uint16_t)((g_uart_rx_ring_write_index + 1U) %
                                                UART_COMMAND_RX_RING_BUFFER_SIZE);
        g_uart_rx_ring_length++;
    }
    taskEXIT_CRITICAL_FROM_ISR(critical_state);

    /* 清空DMA缓存后立即重启下一轮接收，保证后续命令仍能继续进入。 */
    (void)memset(g_uart_dma_rx_buffer, 0, sizeof(g_uart_dma_rx_buffer));
    UartCommand_RestartReceive();

    if (g_uart_rx_semaphore != NULL)
    {
        /*
         * 二值信号量只负责唤醒唯一消费者；即使多次回调合并为一次唤醒，
         * 所有字节仍保存在环形缓存中，不再发生“后一块覆盖前一块”。
         */
        (void)xSemaphoreGiveFromISR(g_uart_rx_semaphore, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/**
 * @brief HAL 串口错误回调。
 * @param huart 发生错误的串口句柄。
 * @return 无返回值。
 *
 * 设计目的：
 * - USART1 当前是 MP157 主链路，如果 PA10 悬空、接反、波特率不一致或线路有噪声，
 *   HAL 可能在 ORE/FE/NE/PE 后中止 DMA 接收；
 * - USART3 是 F4 与 ESP32S3 机械臂的 DMA+空闲接收链路，同样需要在错误后重新挂起接收；
 * - 默认 weak 回调不会恢复接收，现场表现就是外部设备一直发但 F4 不再进回调；
 * - 这里只按串口归属分发错误恢复，不在 ISR 中打印，也不在串口底座里解析业务协议。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        RobotArmService_RecoverRxFromISR();
        return;
    }

    if (huart != UART_COMMAND_MP157_HUART)
    {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    UartCommand_RestartReceive();
}


/**
 * @brief HAL 串口接收完成回调（单字节中断模式）。
 * @param huart 完成接收的串口句柄。
 *
 * USART3 使用单字节中断接收（9600 波特率下避免 DMA 空闲中断分帧问题），
 * 每收到 1 字节由 HAL 调用此回调，交给 robot_arm_service 压入环形缓冲区。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        RobotArmService_RxCpltFromISR();
        return;
    }
}
