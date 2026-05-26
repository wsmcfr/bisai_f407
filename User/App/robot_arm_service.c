#include "robot_arm_service.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uart_command.h"
#include "usart.h"

#include <string.h>

/**
 * @brief LeArm 机械臂串口协议速查。
 *
 * 通信链路：
 * 1. 上位机/MP157/串口助手 -> STM32 USART1：115200 8N1，用于输入命令和查看 `[ARM]` 日志；
 * 2. STM32 USART3 -> ESP32 LeArm：115200 8N1，PD8(TX) 接 ESP32 PA5(GPIO33/RX)，PD9(RX) 接 ESP32 PA4(GPIO32/TX)，两板必须共地；
 * 3. ESP32 固件必须使用 `Serial.begin(115200, SERIAL_8N1, PA5, PA4)`，否则 Type-C 能通信不代表 STM32 外部串口能通信。
 *
 * 帧格式：
 * - 固定格式：`55 55 Length CMD Params...`
 * - 总字节数：`Length + 2`，其中两个 `55` 不计入 Length；
 * - STM32 允许串口助手在帧后追加 `0D 0A`，但只会把 `Length + 2` 个协议字节转发给 ESP32；
 * - 除查询/读取/下载确认类命令外，多数运动命令不会回包，判断是否执行主要看机械臂动作和 STM32 `[ARM]` 日志。
 *
 * 用户可直接从 USART1 发送的常用 HEX 命令：
 * | HEX 命令 | 作用 | 是否会让机械臂动作 | 期望回包/现象 |
 * | --- | --- | --- | --- |
 * | `55 55 02 18` | 进入/确认 STM32 通讯模式，自定义命令；ESP32 端会关闭蓝牙占用并 ACK | 否 | 修改过的 ESP32 固件回 `55 55 03 18 00` |
 * | `55 55 02 01` | 查询 ESP32 LeArm 固件版本和舵机类型 | 否 | 回 `55 55 04 01 <servo_type> <software_version>` |
 * | `55 55 02 0C` | 复位机械臂姿态 | 是 | 通常无回包，机械臂回默认姿态 |
 * | `55 55 02 07` | 停止当前动作组 | 是 | 通常无回包，正在运行的动作组停止 |
 * | `55 55 02 0D` | 读取 6 个舵机当前位置 | 否 | 回 `55 55 14 0D ...`，包含各舵机 ID 和位置低/高字节 |
 * | `55 55 05 06 03 01 00` | 运行 3 号动作组 1 次 | 是 | 通常无回包，前提是 ESP32 内部已存在 3 号动作组 |
 *
 * 维护要求：
 * - 如果后续新增 CMD，必须在本注释表里同步写清楚“能发什么、有什么效果、有没有回包”；
 * - 启动阶段只能自动发送无运动诊断帧，禁止在启动握手里直接运行抓取动作。
 */

/**
 * @brief 机械臂协议帧队列深度。
 *
 * 队列只缓存上位机/MP157 短时间连续下发的几帧动作命令。
 * 如果队列过深，机械臂可能执行明显滞后的旧动作；因此这里保持较小深度，优先保证动作响应的实时性。
 */
#define ROBOT_ARM_SERVICE_QUEUE_LENGTH       (4U)

/**
 * @brief USART3 单帧发送超时时间，单位为毫秒。
 *
 * LeArm 动作组控制帧通常只有数个字节，200ms 对当前 115200 波特率已经非常宽裕。
 * 若超过该时间仍未发送完成，通常说明串口状态异常，应丢弃本帧并等待下一帧。
 */
#define ROBOT_ARM_SERVICE_TX_TIMEOUT_MS      (200U)

/**
 * @brief 等待 ESP32 首字节回包的超时时间，单位为毫秒。
 *
 * 机械臂出厂协议中“查询版本/读取舵机”等命令会通过同一串口回包。
 * 首字节等待时间需要覆盖 ESP32 主循环 10ms 延时和命令处理耗时，因此给到 80ms。
 */
#define ROBOT_ARM_SERVICE_RX_FIRST_TIMEOUT_MS (80U)

/**
 * @brief ESP32 回包后续字节读取超时时间，单位为毫秒。
 *
 * 一旦收到第一个字节，后续字节理论上会连续到达。
 * 这里使用较短超时来判断一帧回包已经结束，避免机械臂任务长期阻塞。
 */
#define ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS  (5U)

/**
 * @brief ESP32 单次回包最大诊断缓存长度，单位为字节。
 *
 * 当前只需要观察查询版本、读取舵机等短回包。
 * 缓存保持较小，避免机械臂任务栈承担不必要的长帧压力。
 */
#define ROBOT_ARM_SERVICE_RX_BUFFER_SIZE      (32U)

/**
 * @brief 机械臂任务启动后等待 ESP32 出厂固件完成初始化的时间。
 *
 * ESP32 setup 中包含舵机类型识别、蜂鸣器提示和若干 delay。
 * STM32 过早发送查询帧可能被 ESP32 启动日志或初始化阶段吞掉，因此任务启动后先短暂等待。
 */
#define ROBOT_ARM_SERVICE_STARTUP_DELAY_MS    (1500U)

/**
 * @brief LeArm 上位机协议双字节帧头。
 *
 * ESP32 从机程序已经验证能响应 `55 55 05 06 03 01 00`，
 * 因此 STM32 侧只识别并透传该协议族，不在这里重新解释动作组含义。
 */
#define ROBOT_ARM_SERVICE_FRAME_HEADER       (0x55U)

/**
 * @brief LeArm 协议最短完整帧长度。
 *
 * 停止动作组示例为 `55 55 02 07`，总长度 4 字节，因此小于 4 字节的输入一定不是完整帧。
 */
#define ROBOT_ARM_SERVICE_MIN_FRAME_LENGTH   (4U)

/**
 * @brief LeArm 协议中的查询版本命令号。
 *
 * 该命令不会驱动机械臂运动，只用于确认 STM32 USART3 到 ESP32 PC/BLE 协议口是否真正连通。
 */
#define ROBOT_ARM_SERVICE_CMD_VERSION_QUERY  (0x01U)

/**
 * @brief LeArm 协议中的读取舵机命令号。
 *
 * 读取类命令会产生 ESP32 回包，因此用于判断是否需要短暂读取 USART3 返回数据。
 */
#define ROBOT_ARM_SERVICE_CMD_SERVOS_READ    (0x0DU)

/**
 * @brief LeArm 协议中的动作组擦除确认命令号。
 *
 * 出厂固件执行擦除后会回传确认帧，因此同样需要尝试读取 USART3。
 */
#define ROBOT_ARM_SERVICE_CMD_ACTION_ERASE   (0x08U)

/**
 * @brief LeArm 协议中的动作下载确认命令号。
 *
 * 动作下载成功或失败时 ESP32 会返回确认帧，读取回包有助于后续下载动作组时排查。
 */
#define ROBOT_ARM_SERVICE_CMD_ACTION_DOWNLOAD (0x19U)

/**
 * @brief LeArm 协议中的运行动作组命令号。
 *
 * 该命令只要求 ESP32 从 Flash 中取出已经保存的动作组并执行，出厂协议不会主动回包。
 * 如果指定编号下没有动作组，STM32 仍然只能看到“已发送”，机械臂不会动作。
 */
#define ROBOT_ARM_SERVICE_CMD_ACTION_GROUP_RUN (0x06U)

/**
 * @brief LeArm 协议中的复位机械臂姿态命令号。
 *
 * 该命令不依赖 ESP32 预存动作组，适合作为“串口链路已经通，但动作组不动”时的直控验证命令。
 */
#define ROBOT_ARM_SERVICE_CMD_SERVOS_RESET     (0x0CU)

/**
 * @brief 自定义 STM32 通讯模式命令号。
 *
 * ESP32 出厂固件原始协议没有“串口切换到 STM32 模式”的命令。
 * 这里约定使用未占用的 `0x18`，需要 ESP32 端同步增加同名命令处理：
 * 收到 `55 55 02 18` 后关闭蓝牙供电控制脚，并回包确认。
 */
#define ROBOT_ARM_SERVICE_CMD_STM32_LINK_MODE (0x18U)

/**
 * @brief STM32 启动后先发送给 ESP32 的自定义通讯模式帧。
 *
 * 该帧格式为 `55 55 02 18`，不驱动机械臂运动。
 * 只有同步修改过 ESP32 固件后该命令才会生效；未修改的出厂固件会忽略该命令。
 */
static const uint8_t g_robot_arm_stm32_link_mode_frame[] = {
    ROBOT_ARM_SERVICE_FRAME_HEADER,
    ROBOT_ARM_SERVICE_FRAME_HEADER,
    0x02U,
    ROBOT_ARM_SERVICE_CMD_STM32_LINK_MODE
};

/**
 * @brief STM32 启动后主动发送给 ESP32 的无运动探测帧。
 *
 * 该帧等价于上位机发送“查询版本”：`55 55 02 01`。
 * 它用于验证“STM32 USART3 接线、波特率、ESP32 PC/BLE 协议任务”是否连通。
 */
static const uint8_t g_robot_arm_startup_probe_frame[] = {
    ROBOT_ARM_SERVICE_FRAME_HEADER,
    ROBOT_ARM_SERVICE_FRAME_HEADER,
    0x02U,
    ROBOT_ARM_SERVICE_CMD_VERSION_QUERY
};

/**
 * @brief 机械臂转发队列句柄。
 *
 * 该队列由 USART1 命令分发层写入，由 RobotArmService_Task 独占读取。
 * 通过队列解耦接收和发送，避免 USART1 命令处理流程被 USART3 阻塞发送拖慢。
 */
static QueueHandle_t g_robot_arm_frame_queue = NULL;

/**
 * @brief 判断字节是否为串口助手常见的行尾字符。
 * @param value 待判断的原始字节。
 * @return uint8_t 1 表示 `\r` 或 `\n`，0 表示普通协议字节。
 *
 * 串口助手勾选“回车发送”时会在 HEX 帧后追加 `0D 0A`。
 * ESP32 端状态机能够在完成一帧后重新等待帧头，但 STM32 严格按长度判帧时会先把这类输入丢掉。
 */
static uint8_t RobotArmService_IsLineEnding(uint8_t value)
{
    return ((value == (uint8_t)'\r') || (value == (uint8_t)'\n')) ? 1U : 0U;
}

/**
 * @brief 从原始 USART1 输入中提取 LeArm 协议帧的真实长度。
 * @param frame_buffer 原始串口数据缓存，不能为 NULL。
 * @param frame_length 原始串口数据长度，单位为字节。
 * @return uint16_t 大于 0 表示合法协议帧长度，0 表示不是完整 LeArm 帧。
 *
 * 主要流程：
 * 1. 检查 `55 55` 帧头；
 * 2. 按 ESP32 出厂协议计算总长度 `Length + 2`；
 * 3. 允许真实协议帧后面只追加 `\r`/`\n`；
 * 4. 返回应转发给 ESP32 的协议字节数，避免把串口助手行尾也发过去。
 */
static uint16_t RobotArmService_GetProtocolLength(const uint8_t *frame_buffer, uint16_t frame_length)
{
    uint16_t expected_total_length;
    uint16_t suffix_index;

    if ((frame_buffer == NULL) || (frame_length < ROBOT_ARM_SERVICE_MIN_FRAME_LENGTH))
    {
        return 0U;
    }

    if ((frame_buffer[0] != ROBOT_ARM_SERVICE_FRAME_HEADER) ||
        (frame_buffer[1] != ROBOT_ARM_SERVICE_FRAME_HEADER))
    {
        return 0U;
    }

    /*
     * ESP32 的 PC_BLE unpack() 把 Length=2 当作“只有 CMD 无参数”的最短命令。
     * 小于 2 的长度字段不符合当前 LeArm 协议，继续处理会造成 STM32 与 ESP32 对帧边界理解不一致。
     */
    if (frame_buffer[2] < 2U)
    {
        return 0U;
    }

    expected_total_length = (uint16_t)frame_buffer[2] + 2U;
    if ((expected_total_length < ROBOT_ARM_SERVICE_MIN_FRAME_LENGTH) ||
        (expected_total_length > ROBOT_ARM_SERVICE_FRAME_MAX_SIZE) ||
        (expected_total_length > frame_length))
    {
        return 0U;
    }

    for (suffix_index = expected_total_length; suffix_index < frame_length; ++suffix_index)
    {
        if (RobotArmService_IsLineEnding(frame_buffer[suffix_index]) == 0U)
        {
            return 0U;
        }
    }

    return expected_total_length;
}

/**
 * @brief 判断某个 LeArm 命令是否按出厂固件逻辑会产生回包。
 * @param command LeArm 协议 CMD 字节。
 * @return uint8_t 1 表示发送后应短暂读取 USART3，0 表示该命令通常没有回包。
 *
 * 动作组运行命令一般只让机械臂运动，不返回确认。
 * 查询/读取/下载确认类命令才需要打印 ESP32 回包，避免每个运动命令都额外等待超时。
 */
static uint8_t RobotArmService_CommandExpectsReply(uint8_t command)
{
    if ((command == ROBOT_ARM_SERVICE_CMD_VERSION_QUERY) ||
        (command == ROBOT_ARM_SERVICE_CMD_SERVOS_READ) ||
        (command == ROBOT_ARM_SERVICE_CMD_ACTION_ERASE) ||
        (command == ROBOT_ARM_SERVICE_CMD_ACTION_DOWNLOAD) ||
        (command == ROBOT_ARM_SERVICE_CMD_STM32_LINK_MODE))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 把 LeArm 命令字节翻译成串口日志里能直接看懂的动作说明。
 * @param command LeArm 协议中的 CMD 字节。
 * @return const char* 面向操作者的命令说明字符串。
 *
 * 这里集中维护“命令字节 -> 人话说明”的映射，避免日志里只出现 `cmd=0x18` 这类内部术语。
 * 后续如果新增机械臂命令，也要同步补充本函数和文件顶部的协议速查表。
 */
static const char *RobotArmService_GetCommandDescription(uint8_t command)
{
    switch (command)
    {
        case ROBOT_ARM_SERVICE_CMD_VERSION_QUERY:
            return "query ESP32 firmware version";

        case ROBOT_ARM_SERVICE_CMD_STM32_LINK_MODE:
            return "switch ESP32 to STM32 link mode";

        case ROBOT_ARM_SERVICE_CMD_SERVOS_READ:
            return "read all 6 servo positions";

        case ROBOT_ARM_SERVICE_CMD_ACTION_ERASE:
            return "erase action groups";

        case ROBOT_ARM_SERVICE_CMD_ACTION_DOWNLOAD:
            return "download action group";

        case 0x03U:
            return "move one or more servos to target position";

        case 0x04U:
            return "move arm tip by XYZ coordinate";

        case ROBOT_ARM_SERVICE_CMD_ACTION_GROUP_RUN:
            return "run saved action group";

        case 0x07U:
            return "stop current action group";

        case ROBOT_ARM_SERVICE_CMD_SERVOS_RESET:
            return "reset arm to default pose";

        default:
            return "unknown arm command";
    }
}

/**
 * @brief 把内部来源标签翻译成串口日志里能看懂的来源说明。
 * @param source_label 发送入口传入的内部来源标签，例如 `startup` 或 `uart1`。
 * @return const char* 面向操作者的来源说明字符串。
 *
 * 来源说明用于区分“系统上电自动发送”和“用户从串口1手动发送”，
 * 这样看到日志时能判断当前动作是不是自己刚才触发的。
 */
static const char *RobotArmService_GetSourceDescription(const char *source_label)
{
    if (source_label == NULL)
    {
        return "unknown source";
    }

    if (strcmp(source_label, "startup-mode") == 0)
    {
        return "startup mode request";
    }

    if (strcmp(source_label, "startup") == 0)
    {
        return "startup link probe";
    }

    if (strcmp(source_label, "uart1") == 0)
    {
        return "UART1 user command";
    }

    return source_label;
}

/**
 * @brief 把 ESP32 返回的舵机类型编号翻译成可读名称。
 * @param servo_type ESP32 版本查询回包中的舵机类型字节。
 * @return const char* 舵机类型说明。
 *
 * 出厂固件中 0 通常表示 PWM 舵机，非 0 表示总线舵机。
 * 日志同时保留原始数字，便于资料或源码中继续对照。
 */
static const char *RobotArmService_GetServoTypeDescription(uint8_t servo_type)
{
    return (servo_type == 0U) ? "PWM servo" : "bus servo";
}

/**
 * @brief 清理 USART3 上可能残留的 ESP32 启动输出或错误标志。
 *
 * ESP32 的 PA5/PA4 现在作为 STM32 二进制协议口使用。
 * 如果 ESP32 在 STM32 发送探测帧之前已经输出过旧调试字节，USART3 可能留下 RXNE/ORE 状态。
 * 发送新命令前先清掉这些旧状态，避免后续 `HAL_UART_Receive()` 把旧字节误当成本次回包。
 */
static void RobotArmService_FlushUsart3Rx(void)
{
    volatile uint32_t discarded_register;

    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        discarded_register = huart3.Instance->DR;
        (void)discarded_register;
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE) != RESET)
    {
        __HAL_UART_CLEAR_FEFLAG(&huart3);
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE) != RESET)
    {
        __HAL_UART_CLEAR_NEFLAG(&huart3);
    }

    huart3.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 从 USART3 读取一段 ESP32 回包，用于调试显示。
 * @param reply_buffer 回包缓存，不能为 NULL。
 * @param buffer_size 回包缓存大小，必须大于 0。
 * @return uint16_t 实际读取到的字节数，0 表示超时未收到回包。
 *
 * 读取策略：
 * 1. 首字节等待稍长，覆盖 ESP32 主循环和处理延迟；
 * 2. 收到首字节后，用短超时连续读取后续字节；
 * 3. 缓存满或短超时到达即认为本次回包结束。
 */
static uint16_t RobotArmService_ReadReply(uint8_t *reply_buffer, uint16_t buffer_size)
{
    uint16_t reply_length = 0U;
    uint32_t timeout_ms;

    if ((reply_buffer == NULL) || (buffer_size == 0U))
    {
        return 0U;
    }

    while (reply_length < buffer_size)
    {
        timeout_ms = (reply_length == 0U) ? ROBOT_ARM_SERVICE_RX_FIRST_TIMEOUT_MS
                                          : ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS;
        if (HAL_UART_Receive(&huart3, &reply_buffer[reply_length], 1U, timeout_ms) != HAL_OK)
        {
            break;
        }

        ++reply_length;
    }

    return reply_length;
}

/**
 * @brief 把 ESP32 回包整理成 USART1 上可读的诊断日志。
 * @param command 本次 STM32 发送的 LeArm 命令号。
 * @param reply_buffer ESP32 返回的原始字节缓存。
 * @param reply_length ESP32 返回的原始字节长度。
 *
 * 查询版本回包有稳定格式，直接打印舵机类型和软件版本。
 * 其它回包只打印长度、帧头、命令字和前两个数据字节，避免长二进制数据污染 USART1 日志。
 */
static void RobotArmService_ReportReply(uint8_t command,
                                        const uint8_t *reply_buffer,
                                        uint16_t reply_length)
{
    if ((reply_buffer == NULL) || (reply_length == 0U))
    {
        my_printf(&huart1,
                  "[ARM] No ESP32 reply: sent '%s'(0x%02X). Burn updated ESP32 firmware first; then check PD8->PA5, PD9<-PA4, common GND, 115200 baud.\r\n",
                  RobotArmService_GetCommandDescription(command),
                  (unsigned int)command);
        return;
    }

    if ((reply_length >= 5U) &&
        (reply_buffer[0] == ROBOT_ARM_SERVICE_FRAME_HEADER) &&
        (reply_buffer[1] == ROBOT_ARM_SERVICE_FRAME_HEADER) &&
        (reply_buffer[3] == ROBOT_ARM_SERVICE_CMD_STM32_LINK_MODE) &&
        (reply_buffer[4] == 0U))
    {
        my_printf(&huart1,
                  "[ARM] Link mode ready: ESP32 left PS2/offline mode. STM32 can now send arm commands on USART3.\r\n");
        return;
    }

    if ((reply_length >= 6U) &&
        (reply_buffer[0] == ROBOT_ARM_SERVICE_FRAME_HEADER) &&
        (reply_buffer[1] == ROBOT_ARM_SERVICE_FRAME_HEADER) &&
        (reply_buffer[3] == ROBOT_ARM_SERVICE_CMD_VERSION_QUERY))
    {
        my_printf(&huart1,
                  "[ARM] Link OK: ESP32 version reply. len=%u, servo=%s(%u), fw=%u. You can send motion commands now.\r\n",
                  (unsigned int)reply_length,
                  RobotArmService_GetServoTypeDescription(reply_buffer[4]),
                  (unsigned int)reply_buffer[4],
                  (unsigned int)reply_buffer[5]);
        return;
    }

    my_printf(&huart1,
              "[ARM] ESP32 replied to '%s': len=%u, raw=%02X %02X %02X %02X %02X %02X.\r\n",
              RobotArmService_GetCommandDescription(command),
              (unsigned int)reply_length,
              (reply_length > 0U) ? reply_buffer[0] : 0U,
              (reply_length > 1U) ? reply_buffer[1] : 0U,
              (reply_length > 2U) ? reply_buffer[2] : 0U,
              (reply_length > 3U) ? reply_buffer[3] : 0U,
              (reply_length > 4U) ? reply_buffer[4] : 0U,
              (reply_length > 5U) ? reply_buffer[5] : 0U);
}

/**
 * @brief 说明那些“发送后不会回包”的机械臂命令该如何判断结果。
 * @param command 本次发送的 LeArm 命令号。
 * @param frame_data 本次发送的完整 LeArm 协议帧，不能为 NULL。
 * @param frame_length 本次发送的协议帧长度。
 *
 * LeArm 出厂协议里，动作类命令多数不会返回确认帧。
 * 如果只打印“已发送”，操作者容易误以为 ESP32 一定执行了动作；
 * 因此这里把最容易误解的动作组编号和验证方法单独打印出来。
 */
static void RobotArmService_ReportNoReplyCommand(uint8_t command,
                                                 const uint8_t *frame_data,
                                                 uint16_t frame_length)
{
    if ((frame_data == NULL) || (frame_length < ROBOT_ARM_SERVICE_MIN_FRAME_LENGTH))
    {
        return;
    }

    if ((command == ROBOT_ARM_SERVICE_CMD_ACTION_GROUP_RUN) && (frame_length >= 7U))
    {
        uint16_t repeat_times = (uint16_t)frame_data[5] | ((uint16_t)frame_data[6] << 8);

        my_printf(&huart1,
                  "[ARM] Motion command has no reply: requested saved action group %u, repeat %u time(s). If the arm does not move, ESP32 may not have this group saved. Test direct reset with 55 55 02 0C.\r\n",
                  (unsigned int)frame_data[4],
                  (unsigned int)repeat_times);
        return;
    }

    if (command == ROBOT_ARM_SERVICE_CMD_SERVOS_RESET)
    {
        my_printf(&huart1,
                  "[ARM] Reset command has no reply. The arm should move to default pose if servo power and bus wiring are OK.\r\n");
        return;
    }

    my_printf(&huart1,
              "[ARM] '%s' has no reply frame. Judge it by arm movement or by the next query command.\r\n",
              RobotArmService_GetCommandDescription(command));
}

/**
 * @brief 通过 USART3 发送一帧 LeArm 协议，并按需读取 ESP32 回包。
 * @param frame_data 待发送协议帧，不能为 NULL。
 * @param frame_length 待发送协议帧长度，单位为字节。
 * @param source_label 日志中标记帧来源，例如 `startup` 或 `uart1`。
 *
 * 该函数是机械臂任务内唯一的 USART3 发送入口。
 * 统一放在这里可以保证发送日志、回包读取和错误处理保持一致。
 */
static void RobotArmService_TransmitFrame(const uint8_t *frame_data,
                                          uint16_t frame_length,
                                          const char *source_label)
{
    HAL_StatusTypeDef tx_status;
    uint8_t reply_buffer[ROBOT_ARM_SERVICE_RX_BUFFER_SIZE];
    uint16_t reply_length;
    uint8_t command;

    if ((frame_data == NULL) || (frame_length < ROBOT_ARM_SERVICE_MIN_FRAME_LENGTH))
    {
        return;
    }

    command = frame_data[3];
    RobotArmService_FlushUsart3Rx();

    tx_status = HAL_UART_Transmit(&huart3,
                                  (uint8_t *)frame_data,
                                  frame_length,
                                  ROBOT_ARM_SERVICE_TX_TIMEOUT_MS);

    if (tx_status == HAL_OK)
    {
        my_printf(&huart1,
                  "[ARM] Sent to ESP32: %s. source=%s, len=%u, cmd=0x%02X.\r\n",
                  RobotArmService_GetCommandDescription(command),
                  RobotArmService_GetSourceDescription(source_label),
                  (unsigned int)frame_length,
                  (unsigned int)command);
    }
    else
    {
        my_printf(&huart1,
                  "[ARM] Send failed: '%s' did not leave USART3. HAL=%d, len=%u, cmd=0x%02X. Check USART3 wiring or pin conflict.\r\n",
                  RobotArmService_GetCommandDescription(command),
                  (int)tx_status,
                  (unsigned int)frame_length,
                  (unsigned int)command);
        return;
    }

    if (RobotArmService_CommandExpectsReply(command) != 0U)
    {
        reply_length = RobotArmService_ReadReply(reply_buffer, sizeof(reply_buffer));
        RobotArmService_ReportReply(command, reply_buffer, reply_length);
    }
    else
    {
        RobotArmService_ReportNoReplyCommand(command, frame_data, frame_length);
    }
}

/**
 * @brief 初始化机械臂服务内部队列。
 * @return uint8_t 1 表示队列可用，0 表示创建失败。
 *
 * 若队列已经存在，直接返回成功，避免重复创建导致内存泄漏。
 */
uint8_t RobotArmService_Init(void)
{
    if (g_robot_arm_frame_queue != NULL)
    {
        return 1U;
    }

    g_robot_arm_frame_queue = xQueueCreate(ROBOT_ARM_SERVICE_QUEUE_LENGTH,
                                           sizeof(RobotArmService_Frame_t));
    return (g_robot_arm_frame_queue != NULL) ? 1U : 0U;
}

/**
 * @brief 判断一段原始串口数据是否符合 LeArm 上位机协议帧格式。
 * @param frame_buffer 原始串口数据缓存，不能为 NULL。
 * @param frame_length 原始串口数据长度，单位为字节。
 * @return uint8_t 1 表示帧格式合法，0 表示不是完整 LeArm 帧。
 *
 * 校验点：
 * 1. 至少包含 `55 55 Length CMD` 四个字节；
 * 2. 前两个字节必须是 LeArm 固定帧头；
 * 3. Length 字段必须能和本次 DMA 空闲中断给出的总长度对齐；
 * 4. 总长度不能超过本模块的队列缓存上限。
 */
uint8_t RobotArmService_IsProtocolFrame(const uint8_t *frame_buffer, uint16_t frame_length)
{
    return (RobotArmService_GetProtocolLength(frame_buffer, frame_length) != 0U) ? 1U : 0U;
}

/**
 * @brief 处理一帧来自 USART1 的原始数据，若识别为机械臂协议则投递到 USART3 转发任务。
 * @param frame_buffer 原始串口数据缓存，不能为 NULL。
 * @param frame_length 原始串口数据长度，单位为字节。
 * @return uint8_t 1 表示该帧已经被机械臂服务接管，0 表示该帧不是机械臂协议帧。
 *
 * 这里不直接调用 HAL_UART_Transmit，是为了让 USART1 命令分发路径保持轻量。
 * 如果队列已满，函数仍然返回 1，表示该帧属于机械臂协议但已被丢弃，防止二进制帧继续落入文本命令解析分支。
 */
uint8_t RobotArmService_HandleFrame(const uint8_t *frame_buffer, uint16_t frame_length)
{
    RobotArmService_Frame_t queued_frame;
    uint16_t protocol_length;
    BaseType_t queue_status;

    protocol_length = RobotArmService_GetProtocolLength(frame_buffer, frame_length);
    if (protocol_length == 0U)
    {
        return 0U;
    }

    if ((g_robot_arm_frame_queue == NULL) && (RobotArmService_Init() == 0U))
    {
        my_printf(&huart1,
                  "[ARM] Queue create failed: '%s'(0x%02X) was not sent. Check FreeRTOS heap.\r\n",
                  RobotArmService_GetCommandDescription(frame_buffer[3]),
                  (unsigned int)frame_buffer[3]);
        return 1U;
    }

    (void)memset(&queued_frame, 0, sizeof(queued_frame));
    (void)memcpy(queued_frame.data, frame_buffer, protocol_length);
    queued_frame.length = protocol_length;

    /*
     * 队列发送使用 0 tick 等待，避免称重主任务或命令分发路径因为机械臂队列拥塞而阻塞。
     * 队列满时直接丢弃当前帧，操作者可以重新下发动作命令。
     */
    queue_status = xQueueSend(g_robot_arm_frame_queue, &queued_frame, 0U);
    if (queue_status == pdTRUE)
    {
        my_printf(&huart1,
                  "[ARM] UART1 accepted: %s. rx_len=%u, tx_len=%u, cmd=0x%02X.\r\n",
                  RobotArmService_GetCommandDescription(queued_frame.data[3]),
                  (unsigned int)frame_length,
                  (unsigned int)protocol_length,
                  (unsigned int)queued_frame.data[3]);
    }
    else
    {
        my_printf(&huart1,
                  "[ARM] Command queue is busy: '%s' was dropped. Retry after the previous arm log ends. len=%u, cmd=0x%02X.\r\n",
                  RobotArmService_GetCommandDescription(queued_frame.data[3]),
                  (unsigned int)protocol_length,
                  (unsigned int)queued_frame.data[3]);
    }

    return 1U;
}

/**
 * @brief 机械臂服务任务入口，负责把队列中的机械臂协议帧通过 USART3 发送给 ESP32。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 任务内部不解析动作组编号、不修改帧内容，只做 USART1 到 USART3 的可靠转发。
 * 这样 ESP32 仍然保持机械臂协议和总线舵机控制的唯一执行者，STM32 只承担调度与上游指令桥接。
 */
void RobotArmService_Task(void *argument)
{
    RobotArmService_Frame_t frame;

    (void)argument;

    if (RobotArmService_Init() == 0U)
    {
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    /*
     * 启动阶段先发送自定义 STM32 通讯模式帧，再发送查询版本帧。
     * 两条帧都不驱动机械臂运动：前者要求 ESP32 端关闭蓝牙并确认，后者用于确认协议回包。
     */
    vTaskDelay(pdMS_TO_TICKS(ROBOT_ARM_SERVICE_STARTUP_DELAY_MS));
    RobotArmService_TransmitFrame(g_robot_arm_stm32_link_mode_frame,
                                  (uint16_t)sizeof(g_robot_arm_stm32_link_mode_frame),
                                  "startup-mode");
    vTaskDelay(pdMS_TO_TICKS(20U));
    RobotArmService_TransmitFrame(g_robot_arm_startup_probe_frame,
                                  (uint16_t)sizeof(g_robot_arm_startup_probe_frame),
                                  "startup");

    for (;;)
    {
        if (xQueueReceive(g_robot_arm_frame_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            /*
             * USART3 已在 CubeMX 中配置为 115200 8N1，需要和 ESP32 当前 PA5/PA4 串口波特率保持一致。
             * 当前链路对接 ESP32 的 PA5/PA4 扩展串口；如果 ESP32 后续再次改动波特率，这里和 CubeMX 配置也要同步调整。
             * 这里按原始字节发送，确保 `55 55 ...` 二进制协议不被字符串处理破坏。
             */
            RobotArmService_TransmitFrame(frame.data, frame.length, "uart1");
        }
    }
}
