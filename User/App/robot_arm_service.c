#include "robot_arm_service.h"

#include "binary_protocol_service.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uart_command.h"
#include "usart.h"

#include <string.h>

/**
 * @brief F4 与 ESP32S3 机械臂正式二进制协议说明。
 *
 * 本文件只使用 `A5 5A VER CMD LEN SEQ_L SEQ_H PAYLOAD CRC_L CRC_H 6B`
 * 正式协议，不再发送旧动作组兼容帧。
 *
 * 链路约定：
 * 1. STM32F407 USART3 TX/RX 与 ESP32S3 UART RX/TX 交叉连接；
 * 2. 两端固定 `115200 8N1`，必须共地，电平必须为 3.3V TTL；
 * 3. F4 是主控，只在 MP157 自动流程推进到机械臂阶段时发送动作命令；
 * 4. ESP32S3 收到合法命令后先回 `ARM_ACK`，动作真实完成后再回 `ARM_STAGE_DONE`；
 * 5. F4 只有收到对应阶段 `ARM_STAGE_DONE result=0` 后，才继续读取称重、电感或上报整轮完成。
 */

/**
 * @brief 机械臂协议帧队列深度。
 *
 * 队列只缓存短时间连续到来的机械臂业务命令。
 * 队列过深会让机械臂执行滞后的旧动作，因此保持较小深度。
 */
#define ROBOT_ARM_SERVICE_QUEUE_LENGTH              (4U)

/**
 * @brief USART3 单帧发送超时时间，单位毫秒。
 *
 * 正式机械臂帧最长不超过 58 字节，115200 波特率下 200ms 已有足够余量。
 */
#define ROBOT_ARM_SERVICE_TX_TIMEOUT_MS             (200U)

/**
 * @brief 等待 ESP32S3 ACK 首字节的超时时间，单位毫秒。
 *
 * ACK 只表示 ESP32S3 已经接收并接受命令，应快速返回；超过该时间说明链路或固件状态异常。
 */
#define ROBOT_ARM_SERVICE_ACK_TIMEOUT_MS            (500U)

/**
 * @brief 读取一帧内后续字节的短超时时间，单位毫秒。
 *
 * 首字节到达后，同一帧后续字节应连续出现；短超时可避免任务被坏帧长时间拖住。
 */
#define ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS        (10U)

/**
 * @brief 寻找正式帧头时允许跳过的前置噪声字节数量。
 *
 * 串口分析中如果看到前导 `FF` 或其它悬空噪声，F4 不能把第一个噪声字节直接当成本次 ACK 失败。
 * 这里限制最多跳过 16 个字节，避免线路持续乱码时任务长时间困在同步过程。
 */
#define ROBOT_ARM_SERVICE_SYNC_SKIP_LIMIT           (16U)

/**
 * @brief 机械臂任务启动后等待 ESP32S3 初始化的时间，单位毫秒。
 *
 * ESP32S3 上电后可能先初始化舵机、GPIO 和动作任务，F4 延迟发送 HELLO 可减少误判。
 */
#define ROBOT_ARM_SERVICE_STARTUP_DELAY_MS          (1500U)

/**
 * @brief 业务动作写入 payload 的默认动作超时，单位毫秒。
 *
 * ESP32S3 应使用该值作为本地动作超时阈值；F4 等 DONE 时会再额外留出串口调度余量。
 * 当前实物机械臂从抓取、移动到放稳可能明显超过旧版 15 秒，所以正式流程按单阶段 100 秒保护。
 */
#define ROBOT_ARM_SERVICE_ACTION_TIMEOUT_MS         (100000U)

/**
 * @brief F4 等待 DONE 时相对业务超时额外增加的余量，单位毫秒。
 *
 * 该余量覆盖 ESP32S3 动作任务到串口任务之间的调度延迟和 UART 传输时间。
 * F4 实际等待 DONE 的最长时间为 `ACTION_TIMEOUT + DONE_EXTRA`，当前即 110 秒。
 */
#define ROBOT_ARM_SERVICE_DONE_EXTRA_TIMEOUT_MS     (10000U)

/**
 * @brief HELLO 和 HEARTBEAT 这类无运动命令写入 payload 的默认超时，单位毫秒。
 */
#define ROBOT_ARM_SERVICE_CONTROL_TIMEOUT_MS        (1000U)

/**
 * @brief F4 发给 ESP32S3 的动作命令 payload 长度。
 *
 * 固定格式：
 * cycle_id:u16、stage_id:u8、part_type:u8、model_result:u8、target_bin:u8、
 * timeout_ms:u32、motion_profile:u8、flags:u16、reserved:u8。
 */
#define ROBOT_ARM_SERVICE_STAGE_COMMAND_PAYLOAD_LEN (14U)

/**
 * @brief ESP32S3 回给 F4 的 ARM_ACK payload 长度。
 */
#define ROBOT_ARM_SERVICE_ACK_PAYLOAD_LEN           (7U)

/**
 * @brief ESP32S3 回给 F4 的 ARM_STAGE_DONE payload 长度。
 */
#define ROBOT_ARM_SERVICE_STAGE_DONE_PAYLOAD_LEN    (10U)

/**
 * @brief ESP32S3 回给 F4 的 ARM_NACK payload 长度。
 */
#define ROBOT_ARM_SERVICE_NACK_PAYLOAD_LEN          (9U)

/**
 * @brief 机械臂正式协议命令字。
 *
 * 命令值必须和 `docs/f4_esp32s3_arm_protocol/f4_esp32s3_integration_guide.md` 保持一致。
 */
typedef enum
{
    ROBOT_ARM_CMD_HELLO = 0x01U,           /* 双向握手命令，F4 启动后用于确认 ESP32S3 正式协议在线。 */
    ROBOT_ARM_CMD_HEARTBEAT = 0x02U,       /* 双向心跳命令，用于确认链路仍在线。 */
    ROBOT_ARM_CMD_MOVE_TO_WEIGHT = 0x20U,  /* F4 要求 ESP32S3 从 ROI/传送带抓取零件并放到称重模块。 */
    ROBOT_ARM_CMD_MOVE_TO_LDC = 0x21U,     /* F4 要求 ESP32S3 从称重模块搬运到电磁感应模块。 */
    ROBOT_ARM_CMD_SORT_RESULT = 0x22U,     /* F4 要求 ESP32S3 按最终结果放入对应分拣盘。 */
    ROBOT_ARM_CMD_HOME = 0x23U,            /* F4 要求 ESP32S3 回安全初始位。 */
    ROBOT_ARM_CMD_STOP = 0x24U,            /* F4 要求 ESP32S3 立即停止或进入安全状态。 */
    ROBOT_ARM_CMD_STAGE_DONE = 0x30U,      /* ESP32S3 上报某个动作阶段真实完成或失败。 */
    ROBOT_ARM_CMD_STAGE_REPORT = 0x31U,    /* ESP32S3 可选进度上报，F4 只打印日志，不靠它推进流程。 */
    ROBOT_ARM_CMD_JOB_DONE = 0x32U,        /* ESP32S3 可选整套任务完成，首版 F4 不依赖。 */
    ROBOT_ARM_CMD_ACK = 0x80U,             /* ESP32S3 接受命令后的立即确认。 */
    ROBOT_ARM_CMD_NACK = 0x81U,            /* ESP32S3 拒绝命令或状态不允许时返回。 */
    ROBOT_ARM_CMD_FAULT_REPORT = 0x87U     /* ESP32S3 主动上报机械臂故障。 */
} RobotArmService_Command_t;

/**
 * @brief 机械臂阶段编号。
 *
 * stage_id 进入 payload，也用于 DONE 回包匹配当前等待阶段。
 */
typedef enum
{
    ROBOT_ARM_STAGE_NONE = 0U,                 /* 无动作阶段。 */
    ROBOT_ARM_STAGE_PICK_BELT_TO_WEIGHT = 1U,  /* 从 ROI/传送带抓取并放到称重模块。 */
    ROBOT_ARM_STAGE_WEIGHT_TO_LDC = 2U,        /* 从称重模块抓取并放到电磁感应模块。 */
    ROBOT_ARM_STAGE_LDC_TO_SORT_BIN = 3U,      /* 从电磁感应模块抓取并放到最终盘。 */
    ROBOT_ARM_STAGE_HOME = 4U,                 /* 回安全初始位。 */
    ROBOT_ARM_STAGE_STOP_SAFE = 5U             /* 停止并进入安全状态。 */
} RobotArmService_StageId_t;

/**
 * @brief 分拣目标编号。
 */
typedef enum
{
    ROBOT_ARM_BIN_NONE = 0U,    /* 非分拣动作使用。 */
    ROBOT_ARM_BIN_GOOD = 1U,    /* 良品盘。 */
    ROBOT_ARM_BIN_BAD = 2U,     /* 不良品盘。 */
    ROBOT_ARM_BIN_REVIEW = 3U   /* 待复核盘。 */
} RobotArmService_TargetBin_t;

/**
 * @brief 单次发送上下文。
 *
 * 该结构把已经组好的 UART 帧和业务校验字段放在一起。
 * 发送任务拿到队列项后，先发送 frame_data，再用 sequence、cycle_id、stage_id 校验 ACK/DONE 是否匹配。
 */
typedef struct
{
    uint8_t data[ROBOT_ARM_SERVICE_FRAME_MAX_SIZE]; /* 完整 A5 二进制帧缓存。 */
    uint16_t length;                                /* 完整帧长度。 */
    uint8_t command;                                /* 本帧 CMD，用于 ACK/NACK 匹配。 */
    uint16_t sequence;                              /* F4 发送序号，用于 ACK/NACK 匹配。 */
    uint16_t cycle_id;                              /* 当前自动检测流程号，DONE 必须带回同一值。 */
    uint16_t job_id;                                /* 当前机械臂任务号，传给 MP157-F4 主状态机。 */
    uint8_t stage_id;                               /* 当前等待的动作阶段。 */
    uint8_t wait_stage_done;                        /* 1 表示 ACK 后还要继续等待 DONE；0 表示只需要 ACK。 */
    uint32_t timeout_ms;                            /* ESP32S3 动作超时和 F4 等 DONE 的基础时间。 */
    const char *source_label;                       /* 日志来源标签。 */
} RobotArmService_TxContext_t;

/**
 * @brief 机械臂发送队列句柄。
 *
 * 队列由 MP157-F4 二进制协议分发层写入，由 `RobotArmService_Task()` 独占读取。
 */
static QueueHandle_t g_robot_arm_frame_queue = NULL;

/**
 * @brief F4 发往 ESP32S3 的机械臂协议序号。
 *
 * 每发一帧递增，ACK/NACK 必须带回对应序号。
 */
static uint16_t g_robot_arm_tx_sequence = 1U;

/**
 * @brief 读取小端 u16。
 * @param data 指向低字节的地址，不能为 NULL。
 * @return uint16_t 解析后的 16 位整数。
 */
static uint16_t RobotArmService_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief 写入小端 u16。
 * @param data 输出地址，不能为 NULL。
 * @param value 要写入的数值。
 */
static void RobotArmService_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/**
 * @brief 写入小端 u32。
 * @param data 输出地址，不能为 NULL。
 * @param value 要写入的 32 位数。
 *
 * 机械臂动作超时可能超过 65535ms，因此 F4 发给 ESP32S3 的 timeout_ms
 * 必须按 32 位小端写入 payload，避免 100000ms 被截断成 34464ms。
 */
static void RobotArmService_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFUL);
    data[1] = (uint8_t)((value >> 8) & 0xFFUL);
    data[2] = (uint8_t)((value >> 16) & 0xFFUL);
    data[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

/**
 * @brief 分配一帧 F4->ESP32S3 发送序号。
 * @return uint16_t 本次使用的序号。
 *
 * 序号从 1 开始递增，回绕到 0 时跳回 1，避免 0 和未初始化值混淆。
 */
static uint16_t RobotArmService_AllocateSequence(void)
{
    uint16_t sequence = g_robot_arm_tx_sequence;

    ++g_robot_arm_tx_sequence;
    if (g_robot_arm_tx_sequence == 0U)
    {
        g_robot_arm_tx_sequence = 1U;
    }

    return sequence;
}

/**
 * @brief 获取命令字说明。
 * @param command 正式机械臂协议 CMD。
 * @return const char* 可打印的命令说明。
 */
static const char *RobotArmService_GetCommandDescription(uint8_t command)
{
    switch (command)
    {
        case ROBOT_ARM_CMD_HELLO:
            return "ARM_LINK_HELLO";
        case ROBOT_ARM_CMD_HEARTBEAT:
            return "ARM_LINK_HEARTBEAT";
        case ROBOT_ARM_CMD_MOVE_TO_WEIGHT:
            return "ARM_MOVE_TO_WEIGHT";
        case ROBOT_ARM_CMD_MOVE_TO_LDC:
            return "ARM_MOVE_TO_LDC";
        case ROBOT_ARM_CMD_SORT_RESULT:
            return "ARM_SORT_RESULT";
        case ROBOT_ARM_CMD_HOME:
            return "ARM_HOME";
        case ROBOT_ARM_CMD_STOP:
            return "ARM_STOP";
        case ROBOT_ARM_CMD_STAGE_DONE:
            return "ARM_STAGE_DONE";
        case ROBOT_ARM_CMD_STAGE_REPORT:
            return "ARM_STAGE_REPORT";
        case ROBOT_ARM_CMD_JOB_DONE:
            return "ARM_JOB_DONE";
        case ROBOT_ARM_CMD_ACK:
            return "ARM_ACK";
        case ROBOT_ARM_CMD_NACK:
            return "ARM_NACK";
        case ROBOT_ARM_CMD_FAULT_REPORT:
            return "ARM_FAULT_REPORT";
        default:
            return "ARM_UNKNOWN";
    }
}

/**
 * @brief 获取来源说明。
 * @param source_label 内部来源标签。
 * @return const char* 日志中的来源说明。
 */
static const char *RobotArmService_GetSourceDescription(const char *source_label)
{
    if (source_label == NULL)
    {
        return "unknown";
    }

    if (strcmp(source_label, "startup-hello") == 0)
    {
        return "startup hello";
    }

    if (strcmp(source_label, "startup-heartbeat") == 0)
    {
        return "startup heartbeat";
    }

    if (strcmp(source_label, "auto-flow") == 0)
    {
        return "MP157 auto flow";
    }

    if (strcmp(source_label, "uart1-formal") == 0)
    {
        return "UART1 formal arm frame";
    }

    return source_label;
}

/**
 * @brief 清理 USART3 上残留的旧字节和错误标志。
 *
 * 如果 ESP32S3 上电日志、线缆抖动或上一次坏帧残留在 RXNE 中，F4 等 ACK 时可能读到错误帧头。
 * 每次发送新命令前先清理接收侧，确保后续读取尽量对应本次命令。
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
 * @brief 从 USART3 读取一个字节。
 * @param value 输出字节，不能为 NULL。
 * @param timeout_ms 等待时间，单位毫秒。
 * @return uint8_t 1 表示读取成功，0 表示超时或 UART 错误。
 */
static uint8_t RobotArmService_ReadByte(uint8_t *value, uint32_t timeout_ms)
{
    if (value == NULL)
    {
        return 0U;
    }

    return (HAL_UART_Receive(&huart3, value, 1U, timeout_ms) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief 从 USART3 读取一帧完整正式机械臂协议并完成 CRC 校验。
 * @param frame_buffer 原始帧缓存，不能为 NULL。
 * @param frame_buffer_size 原始帧缓存容量。
 * @param parsed_frame 解析结果输出，不能为 NULL。
 * @param first_timeout_ms 等待帧头首字节的超时时间，单位毫秒。
 * @return uint8_t 1 表示收到合法帧，0 表示超时、坏帧或 CRC 错误。
 *
 * 主要流程：
 * 1. 等待 `A5`；
 * 2. 继续读取 `5A VER CMD LEN SEQ_L SEQ_H`；
 * 3. 按 LEN 读取 payload、CRC 和 `6B`；
 * 4. 复用 MP157-F4 主协议解析器校验帧头、版本、长度、CRC 和帧尾。
 */
static uint8_t RobotArmService_ReadFormalFrame(uint8_t *frame_buffer,
                                               uint16_t frame_buffer_size,
                                               BinaryProtocol_Frame_t *parsed_frame,
                                               uint32_t first_timeout_ms)
{
    uint16_t expected_length;
    uint16_t index;
    uint8_t skipped_count = 0U;
    BinaryProtocol_ParseStatus_t parse_status;

    if ((frame_buffer == NULL) ||
        (parsed_frame == NULL) ||
        (frame_buffer_size < BINARY_PROTOCOL_MIN_FRAME_LENGTH))
    {
        return 0U;
    }

    for (;;)
    {
        if (RobotArmService_ReadByte(&frame_buffer[0],
                                     (skipped_count == 0U) ? first_timeout_ms : ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS) == 0U)
        {
            return 0U;
        }

        if (frame_buffer[0] == BINARY_PROTOCOL_SOF0)
        {
            break;
        }

        ++skipped_count;
        if (skipped_count >= ROBOT_ARM_SERVICE_SYNC_SKIP_LIMIT)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
            my_printf(&huart1,
                      "[ARM] Too many bytes before formal frame: skipped=%u, last=0x%02X.\r\n",
                      (unsigned int)skipped_count,
                      (unsigned int)frame_buffer[0]);
            return 0U;
        }
    }

    if (skipped_count > 0U)
    {
        my_printf(&huart1,
                  "[ARM] Skipped %u byte(s) before ESP32 formal frame.\r\n",
                  (unsigned int)skipped_count);
    }

    for (index = 1U; index < 7U; ++index)
    {
        if (RobotArmService_ReadByte(&frame_buffer[index], ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS) == 0U)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
            my_printf(&huart1,
                      "[ARM] ESP32 frame header timeout at byte %u.\r\n",
                      (unsigned int)index);
            return 0U;
        }
    }

    if (frame_buffer[1] != BINARY_PROTOCOL_SOF1)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        my_printf(&huart1,
                  "[ARM] Bad ESP32 frame header: %02X %02X.\r\n",
                  (unsigned int)frame_buffer[0],
                  (unsigned int)frame_buffer[1]);
        return 0U;
    }

    if (frame_buffer[4] > BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        my_printf(&huart1,
                  "[ARM] ESP32 payload too long: len=%u.\r\n",
                  (unsigned int)frame_buffer[4]);
        return 0U;
    }

    expected_length = (uint16_t)(BINARY_PROTOCOL_MIN_FRAME_LENGTH + frame_buffer[4]);
    if (expected_length > frame_buffer_size)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        my_printf(&huart1,
                  "[ARM] ESP32 frame exceeds local buffer: frame=%u, buffer=%u.\r\n",
                  (unsigned int)expected_length,
                  (unsigned int)frame_buffer_size);
        return 0U;
    }

    for (index = 7U; index < expected_length; ++index)
    {
        if (RobotArmService_ReadByte(&frame_buffer[index], ROBOT_ARM_SERVICE_RX_NEXT_TIMEOUT_MS) == 0U)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
            my_printf(&huart1,
                      "[ARM] ESP32 frame body timeout at byte %u/%u.\r\n",
                      (unsigned int)index,
                      (unsigned int)expected_length);
            return 0U;
        }
    }

    parse_status = BinaryProtocolService_ParseFrame(frame_buffer, expected_length, parsed_frame);
    if (parse_status != BINARY_PROTOCOL_PARSE_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        my_printf(&huart1,
                  "[ARM] ESP32 frame parse failed: status=%u, len=%u, cmd=0x%02X.\r\n",
                  (unsigned int)parse_status,
                  (unsigned int)expected_length,
                  (unsigned int)frame_buffer[3]);
        return 0U;
    }

    return 1U;
}

/**
 * @brief 处理 ESP32S3 返回的 ACK。
 * @param context 当前 F4 发送上下文，不能为 NULL。
 * @param frame 已通过 CRC 校验的 ACK 帧，不能为 NULL。
 * @return uint8_t 1 表示 ACK 与当前命令匹配，0 表示不匹配或状态拒绝。
 */
static uint8_t RobotArmService_HandleAckFrame(const RobotArmService_TxContext_t *context,
                                              const BinaryProtocol_Frame_t *frame)
{
    uint16_t cycle_id;
    uint16_t acked_sequence;
    uint8_t acked_command;
    uint8_t status;
    uint8_t arm_state;

    if ((context == NULL) ||
        (frame == NULL) ||
        (frame->payload == NULL) ||
        (frame->payload_length != ROBOT_ARM_SERVICE_ACK_PAYLOAD_LEN))
    {
        return 0U;
    }

    cycle_id = RobotArmService_ReadU16Le(&frame->payload[0]);
    acked_sequence = RobotArmService_ReadU16Le(&frame->payload[2]);
    acked_command = frame->payload[4];
    status = frame->payload[5];
    arm_state = frame->payload[6];

    if (((context->cycle_id != 0U) && (cycle_id != context->cycle_id)) ||
        (acked_sequence != context->sequence) ||
        (acked_command != context->command) ||
        (status > 1U))
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault((uint16_t)acked_command,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          context->sequence);
        my_printf(&huart1,
                  "[ARM] ACK mismatch: rx_cycle=%u, exp_cycle=%u, ack_seq=%u, exp_seq=%u, ack_cmd=0x%02X, exp_cmd=0x%02X, status=%u.\r\n",
                  (unsigned int)cycle_id,
                  (unsigned int)context->cycle_id,
                  (unsigned int)acked_sequence,
                  (unsigned int)context->sequence,
                  (unsigned int)acked_command,
                  (unsigned int)context->command,
                  (unsigned int)status);
        return 0U;
    }

    BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    my_printf(&huart1,
              "[ARM] ACK OK: %s, cycle=%u, seq=%u, status=%u, arm_state=%u.\r\n",
              RobotArmService_GetCommandDescription(context->command),
              (unsigned int)cycle_id,
              (unsigned int)acked_sequence,
              (unsigned int)status,
              (unsigned int)arm_state);
    return 1U;
}

/**
 * @brief 处理 ESP32S3 返回的 NACK。
 * @param context 当前 F4 发送上下文，不能为 NULL。
 * @param frame 已通过 CRC 校验的 NACK 帧，不能为 NULL。
 */
static void RobotArmService_HandleNackFrame(const RobotArmService_TxContext_t *context,
                                            const BinaryProtocol_Frame_t *frame)
{
    uint16_t cycle_id = 0U;
    uint16_t rejected_sequence = 0U;
    uint8_t rejected_command = 0U;
    uint8_t error_code = 0U;
    uint8_t arm_state = 0U;
    uint16_t detail = 0U;

    if ((frame != NULL) &&
        (frame->payload != NULL) &&
        (frame->payload_length == ROBOT_ARM_SERVICE_NACK_PAYLOAD_LEN))
    {
        cycle_id = RobotArmService_ReadU16Le(&frame->payload[0]);
        rejected_sequence = RobotArmService_ReadU16Le(&frame->payload[2]);
        rejected_command = frame->payload[4];
        error_code = frame->payload[5];
        arm_state = frame->payload[6];
        detail = RobotArmService_ReadU16Le(&frame->payload[7]);
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    BinaryProtocolService_ReportFault((uint16_t)error_code,
                                      BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                      BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                      (int32_t)detail,
                                      (context != NULL) ? context->sequence : 0U);
    my_printf(&huart1,
              "[ARM] NACK from ESP32: cycle=%u, rejected_seq=%u, rejected_cmd=0x%02X, error=%u, state=%u, detail=%u.\r\n",
              (unsigned int)cycle_id,
              (unsigned int)rejected_sequence,
              (unsigned int)rejected_command,
              (unsigned int)error_code,
              (unsigned int)arm_state,
              (unsigned int)detail);
}

/**
 * @brief 打印 ESP32S3 可选阶段进度帧。
 * @param frame 已解析帧，不能为 NULL。
 *
 * 阶段进度只是诊断信息，不能替代 DONE 推进自动流程。
 */
static void RobotArmService_HandleStageReportFrame(const BinaryProtocol_Frame_t *frame)
{
    uint16_t cycle_id = 0U;
    uint8_t stage_id = 0U;
    uint8_t progress = 0U;

    if ((frame != NULL) && (frame->payload != NULL) && (frame->payload_length >= 4U))
    {
        cycle_id = RobotArmService_ReadU16Le(&frame->payload[0]);
        stage_id = frame->payload[2];
        progress = frame->payload[3];
    }

    my_printf(&huart1,
              "[ARM] Stage report: cycle=%u, stage=%u, progress=%u, len=%u.\r\n",
              (unsigned int)cycle_id,
              (unsigned int)stage_id,
              (unsigned int)progress,
              (frame != NULL) ? (unsigned int)frame->payload_length : 0U);
}

/**
 * @brief 处理 ESP32S3 返回的 DONE，并把结果交给 MP157-F4 主流程。
 * @param context 当前 F4 发送上下文，不能为 NULL。
 * @param frame 已通过 CRC 校验的 DONE 帧，不能为 NULL。
 * @return uint8_t 1 表示对应阶段成功完成，0 表示失败或不匹配。
 */
static uint8_t RobotArmService_HandleStageDoneFrame(const RobotArmService_TxContext_t *context,
                                                    const BinaryProtocol_Frame_t *frame)
{
    uint16_t cycle_id;
    uint8_t stage_id;
    uint8_t result;
    uint16_t detail_code;
    uint16_t elapsed_ms;
    uint16_t fault_bits;

    if ((context == NULL) ||
        (frame == NULL) ||
        (frame->payload == NULL) ||
        (frame->payload_length != ROBOT_ARM_SERVICE_STAGE_DONE_PAYLOAD_LEN))
    {
        return 0U;
    }

    cycle_id = RobotArmService_ReadU16Le(&frame->payload[0]);
    stage_id = frame->payload[2];
    result = frame->payload[3];
    detail_code = RobotArmService_ReadU16Le(&frame->payload[4]);
    elapsed_ms = RobotArmService_ReadU16Le(&frame->payload[6]);
    fault_bits = RobotArmService_ReadU16Le(&frame->payload[8]);

    if ((cycle_id != context->cycle_id) || (stage_id != context->stage_id))
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault(detail_code,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)stage_id,
                                          context->sequence);
        my_printf(&huart1,
                  "[ARM] DONE mismatch: rx_cycle=%u, exp_cycle=%u, rx_stage=%u, exp_stage=%u, result=%u.\r\n",
                  (unsigned int)cycle_id,
                  (unsigned int)context->cycle_id,
                  (unsigned int)stage_id,
                  (unsigned int)context->stage_id,
                  (unsigned int)result);
        return 0U;
    }

    if (result == 0U)
    {
        BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    }
    else
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    }

    my_printf(&huart1,
              "[ARM] DONE received: cycle=%u, job=%u, stage=%u, result=%u, detail=%u, elapsed=%u, faults=0x%04X.\r\n",
              (unsigned int)cycle_id,
              (unsigned int)context->job_id,
              (unsigned int)stage_id,
              (unsigned int)result,
              (unsigned int)detail_code,
              (unsigned int)elapsed_ms,
              (unsigned int)fault_bits);

    BinaryProtocolService_HandleArmStageDone(cycle_id,
                                             context->job_id,
                                             stage_id,
                                             result,
                                             detail_code,
                                             elapsed_ms,
                                             fault_bits);
    return (result == 0U) ? 1U : 0U;
}

/**
 * @brief 处理 ESP32S3 主动故障帧。
 * @param frame 已解析帧，可以为 NULL。
 */
static void RobotArmService_HandleFaultFrame(const BinaryProtocol_Frame_t *frame)
{
    uint16_t fault_code = 0U;
    uint16_t fault_bits = 0U;

    if ((frame != NULL) && (frame->payload != NULL) && (frame->payload_length >= 4U))
    {
        fault_code = RobotArmService_ReadU16Le(&frame->payload[0]);
        fault_bits = RobotArmService_ReadU16Le(&frame->payload[2]);
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    BinaryProtocolService_ReportFault(fault_code,
                                      BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                      BINARY_PROTOCOL_FAULT_SEVERITY_STOP,
                                      (int32_t)fault_bits,
                                      0U);
    my_printf(&huart1,
              "[ARM] Fault report from ESP32: fault_code=%u, fault_bits=0x%04X, len=%u.\r\n",
              (unsigned int)fault_code,
              (unsigned int)fault_bits,
              (frame != NULL) ? (unsigned int)frame->payload_length : 0U);
}

/**
 * @brief 等待 ESP32S3 对当前命令返回 ACK 或 NACK。
 * @param context 当前发送上下文，不能为 NULL。
 * @return uint8_t 1 表示收到匹配 ACK，0 表示超时、NACK 或其它错误帧。
 */
static uint8_t RobotArmService_WaitAck(const RobotArmService_TxContext_t *context)
{
    uint8_t rx_frame[ROBOT_ARM_SERVICE_FRAME_MAX_SIZE];
    BinaryProtocol_Frame_t parsed_frame;

    if (context == NULL)
    {
        return 0U;
    }

    if (RobotArmService_ReadFormalFrame(rx_frame,
                                        (uint16_t)sizeof(rx_frame),
                                        &parsed_frame,
                                        ROBOT_ARM_SERVICE_ACK_TIMEOUT_MS) == 0U)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault((uint16_t)context->command,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          0,
                                          context->sequence);
        my_printf(&huart1,
                  "[ARM] ACK timeout: cmd=%s, cycle=%u, seq=%u. Check ESP32 formal protocol, TX/RX, GND and 115200 8N1.\r\n",
                  RobotArmService_GetCommandDescription(context->command),
                  (unsigned int)context->cycle_id,
                  (unsigned int)context->sequence);
        return 0U;
    }

    if (parsed_frame.command == ROBOT_ARM_CMD_ACK)
    {
        return RobotArmService_HandleAckFrame(context, &parsed_frame);
    }

    if (parsed_frame.command == ROBOT_ARM_CMD_NACK)
    {
        RobotArmService_HandleNackFrame(context, &parsed_frame);
        return 0U;
    }

    if (parsed_frame.command == ROBOT_ARM_CMD_FAULT_REPORT)
    {
        RobotArmService_HandleFaultFrame(&parsed_frame);
        return 0U;
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    my_printf(&huart1,
              "[ARM] Unexpected frame while waiting ACK: cmd=0x%02X(%s), seq=%u.\r\n",
              (unsigned int)parsed_frame.command,
              RobotArmService_GetCommandDescription(parsed_frame.command),
              (unsigned int)parsed_frame.sequence);
    return 0U;
}

/**
 * @brief 等待 ESP32S3 对当前动作返回 STAGE_DONE。
 * @param context 当前发送上下文，不能为 NULL。
 * @return uint8_t 1 表示 DONE 成功，0 表示失败、超时或故障。
 *
 * ACK 只代表动作进入 ESP32S3 队列；本函数等待真实放置完成。
 * 期间如果收到可选进度帧，只记录日志继续等 DONE。
 */
static uint8_t RobotArmService_WaitStageDone(const RobotArmService_TxContext_t *context)
{
    uint8_t rx_frame[ROBOT_ARM_SERVICE_FRAME_MAX_SIZE];
    BinaryProtocol_Frame_t parsed_frame;
    TickType_t deadline_tick;
    TickType_t now_tick;
    TickType_t remain_tick;
    uint32_t remain_ms;

    if (context == NULL)
    {
        return 0U;
    }

    deadline_tick = xTaskGetTickCount() +
                    pdMS_TO_TICKS((uint32_t)context->timeout_ms + ROBOT_ARM_SERVICE_DONE_EXTRA_TIMEOUT_MS);

    for (;;)
    {
        now_tick = xTaskGetTickCount();
        if ((int32_t)(deadline_tick - now_tick) <= 0)
        {
            break;
        }

        remain_tick = deadline_tick - now_tick;
        remain_ms = (uint32_t)remain_tick * (uint32_t)portTICK_PERIOD_MS;
        if (remain_ms == 0U)
        {
            remain_ms = 1U;
        }

        if (RobotArmService_ReadFormalFrame(rx_frame,
                                            (uint16_t)sizeof(rx_frame),
                                            &parsed_frame,
                                            remain_ms) == 0U)
        {
            break;
        }

        if (parsed_frame.command == ROBOT_ARM_CMD_STAGE_DONE)
        {
            return RobotArmService_HandleStageDoneFrame(context, &parsed_frame);
        }

        if (parsed_frame.command == ROBOT_ARM_CMD_STAGE_REPORT)
        {
            RobotArmService_HandleStageReportFrame(&parsed_frame);
            continue;
        }

        if (parsed_frame.command == ROBOT_ARM_CMD_NACK)
        {
            RobotArmService_HandleNackFrame(context, &parsed_frame);
            return 0U;
        }

        if (parsed_frame.command == ROBOT_ARM_CMD_FAULT_REPORT)
        {
            RobotArmService_HandleFaultFrame(&parsed_frame);
            return 0U;
        }

        my_printf(&huart1,
                  "[ARM] Ignore frame while waiting DONE: cmd=0x%02X(%s), seq=%u.\r\n",
                  (unsigned int)parsed_frame.command,
                  RobotArmService_GetCommandDescription(parsed_frame.command),
                  (unsigned int)parsed_frame.sequence);
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    BinaryProtocolService_ReportFault((uint16_t)context->stage_id,
                                      BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                      BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                      (int32_t)context->timeout_ms,
                                      context->sequence);
    my_printf(&huart1,
              "[ARM] DONE timeout: cmd=%s, cycle=%u, job=%u, stage=%u, wait_ms=%u.\r\n",
              RobotArmService_GetCommandDescription(context->command),
              (unsigned int)context->cycle_id,
              (unsigned int)context->job_id,
              (unsigned int)context->stage_id,
              (unsigned int)((uint32_t)context->timeout_ms + ROBOT_ARM_SERVICE_DONE_EXTRA_TIMEOUT_MS));
    return 0U;
}

/**
 * @brief 发送正式协议帧，并按协议等待 ACK/DONE。
 * @param context 发送上下文，不能为 NULL。
 */
static void RobotArmService_TransmitContext(const RobotArmService_TxContext_t *context)
{
    HAL_StatusTypeDef tx_status;

    if ((context == NULL) ||
        (context->length < BINARY_PROTOCOL_MIN_FRAME_LENGTH) ||
        (context->length > ROBOT_ARM_SERVICE_FRAME_MAX_SIZE))
    {
        return;
    }

    RobotArmService_FlushUsart3Rx();
    tx_status = HAL_UART_Transmit(&huart3,
                                  (uint8_t *)context->data,
                                  context->length,
                                  ROBOT_ARM_SERVICE_TX_TIMEOUT_MS);
    if (tx_status != HAL_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault((uint16_t)tx_status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)tx_status,
                                          context->sequence);
        my_printf(&huart1,
                  "[ARM] Send failed: cmd=%s, HAL=%d, len=%u, seq=%u.\r\n",
                  RobotArmService_GetCommandDescription(context->command),
                  (int)tx_status,
                  (unsigned int)context->length,
                  (unsigned int)context->sequence);
        return;
    }

    my_printf(&huart1,
              "[ARM] Sent formal frame: cmd=%s, source=%s, cycle=%u, job=%u, stage=%u, seq=%u, len=%u.\r\n",
              RobotArmService_GetCommandDescription(context->command),
              RobotArmService_GetSourceDescription(context->source_label),
              (unsigned int)context->cycle_id,
              (unsigned int)context->job_id,
              (unsigned int)context->stage_id,
              (unsigned int)context->sequence,
              (unsigned int)context->length);

    if (RobotArmService_WaitAck(context) == 0U)
    {
        return;
    }

    if (context->wait_stage_done != 0U)
    {
        (void)RobotArmService_WaitStageDone(context);
    }
}

/**
 * @brief 填充机械臂阶段命令 payload。
 * @param payload 输出 payload，长度必须至少为 14 字节。
 * @param cycle_id 当前单件流程号。
 * @param stage_id 当前机械臂阶段。
 * @param part_type 零件类型，未知填 0。
 * @param model_result 模型结果，0=未知，1=良品，2=不良品，3=待复核。
 * @param target_bin 分拣盘，非分拣阶段填 0。
 * @param timeout_ms 本动作超时时间。
 */
static void RobotArmService_FillStagePayload(uint8_t *payload,
                                             uint16_t cycle_id,
                                             uint8_t stage_id,
                                             uint8_t part_type,
                                             uint8_t model_result,
                                             uint8_t target_bin,
                                             uint32_t timeout_ms)
{
    (void)memset(payload, 0, ROBOT_ARM_SERVICE_STAGE_COMMAND_PAYLOAD_LEN);
    RobotArmService_WriteU16Le(&payload[0], cycle_id);
    payload[2] = stage_id;
    payload[3] = part_type;
    payload[4] = model_result;
    payload[5] = target_bin;
    RobotArmService_WriteU32Le(&payload[6], timeout_ms);
    payload[10] = 0U;
    RobotArmService_WriteU16Le(&payload[11], 0U);
    payload[13] = 0U;
}

/**
 * @brief 构建并入队一个正式机械臂动作命令。
 * @param command 机械臂命令字。
 * @param cycle_id 当前自动检测流程号。
 * @param job_id 当前机械臂任务号。
 * @param stage_id 当前阶段编号。
 * @param part_type 零件类型。
 * @param model_result 模型结果。
 * @param target_bin 分拣目标。
 * @return uint8_t 1 表示已进入发送队列，0 表示组帧或入队失败。
 */
static uint8_t RobotArmService_EnqueueStageCommand(uint8_t command,
                                                   uint16_t cycle_id,
                                                   uint16_t job_id,
                                                   uint8_t stage_id,
                                                   uint8_t part_type,
                                                   uint8_t model_result,
                                                   uint8_t target_bin)
{
    RobotArmService_TxContext_t queued_frame;
    uint8_t payload[ROBOT_ARM_SERVICE_STAGE_COMMAND_PAYLOAD_LEN];
    BaseType_t queue_status;

    if (RobotArmService_Init() == 0U)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        return 0U;
    }

    (void)memset(&queued_frame, 0, sizeof(queued_frame));
    RobotArmService_FillStagePayload(payload,
                                     cycle_id,
                                     stage_id,
                                     part_type,
                                     model_result,
                                     target_bin,
                                     ROBOT_ARM_SERVICE_ACTION_TIMEOUT_MS);

    queued_frame.sequence = RobotArmService_AllocateSequence();
    queued_frame.length = BinaryProtocolService_BuildFrame(command,
                                                           queued_frame.sequence,
                                                           payload,
                                                           (uint8_t)sizeof(payload),
                                                           queued_frame.data,
                                                           (uint16_t)sizeof(queued_frame.data));
    if (queued_frame.length == 0U)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        return 0U;
    }

    queued_frame.command = command;
    queued_frame.cycle_id = cycle_id;
    queued_frame.job_id = job_id;
    queued_frame.stage_id = stage_id;
    queued_frame.wait_stage_done = 1U;
    queued_frame.timeout_ms = ROBOT_ARM_SERVICE_ACTION_TIMEOUT_MS;
    queued_frame.source_label = "auto-flow";

    queue_status = xQueueSend(g_robot_arm_frame_queue, &queued_frame, 0U);
    if (queue_status != pdTRUE)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault((uint16_t)command,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)job_id,
                                          queued_frame.sequence);
        my_printf(&huart1,
                  "[ARM] Queue busy: cmd=%s, cycle=%u, job=%u, stage=%u.\r\n",
                  RobotArmService_GetCommandDescription(command),
                  (unsigned int)cycle_id,
                  (unsigned int)job_id,
                  (unsigned int)stage_id);
        return 0U;
    }

    BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
    my_printf(&huart1,
              "[ARM] Queued formal action: cmd=%s, cycle=%u, job=%u, part=%u, model=%u, bin=%u, seq=%u.\r\n",
              RobotArmService_GetCommandDescription(command),
              (unsigned int)cycle_id,
              (unsigned int)job_id,
              (unsigned int)part_type,
              (unsigned int)model_result,
              (unsigned int)target_bin,
              (unsigned int)queued_frame.sequence);
    return 1U;
}

/**
 * @brief 构建并立即发送一个启动诊断命令。
 * @param command HELLO 或 HEARTBEAT 命令。
 * @param source_label 日志来源标签。
 *
 * 启动诊断不进入动作队列，也不等待 DONE，只验证 ESP32S3 正式协议 ACK。
 */
static void RobotArmService_SendStartupControl(uint8_t command, const char *source_label)
{
    RobotArmService_TxContext_t context;

    (void)memset(&context, 0, sizeof(context));
    context.sequence = RobotArmService_AllocateSequence();
    context.length = BinaryProtocolService_BuildFrame(command,
                                                      context.sequence,
                                                      (const uint8_t *)0,
                                                      0U,
                                                      context.data,
                                                      (uint16_t)sizeof(context.data));
    if (context.length == 0U)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        return;
    }

    context.command = command;
    context.cycle_id = 0U;
    context.job_id = 0U;
    context.stage_id = ROBOT_ARM_STAGE_NONE;
    context.wait_stage_done = 0U;
    context.timeout_ms = ROBOT_ARM_SERVICE_CONTROL_TIMEOUT_MS;
    context.source_label = source_label;
    RobotArmService_TransmitContext(&context);
}

uint8_t RobotArmService_Init(void)
{
    if (g_robot_arm_frame_queue != NULL)
    {
        return 1U;
    }

    g_robot_arm_frame_queue = xQueueCreate(ROBOT_ARM_SERVICE_QUEUE_LENGTH,
                                           sizeof(RobotArmService_TxContext_t));
    return (g_robot_arm_frame_queue != NULL) ? 1U : 0U;
}

uint8_t RobotArmService_RequestPlaceWeight(uint16_t cycle_id,
                                           uint16_t job_id,
                                           uint8_t part_type,
                                           uint8_t model_result)
{
    return RobotArmService_EnqueueStageCommand(ROBOT_ARM_CMD_MOVE_TO_WEIGHT,
                                               cycle_id,
                                               job_id,
                                               ROBOT_ARM_STAGE_PICK_BELT_TO_WEIGHT,
                                               part_type,
                                               model_result,
                                               ROBOT_ARM_BIN_NONE);
}

uint8_t RobotArmService_RequestPlaceLdc(uint16_t cycle_id,
                                        uint16_t job_id,
                                        uint8_t part_type,
                                        uint8_t model_result)
{
    return RobotArmService_EnqueueStageCommand(ROBOT_ARM_CMD_MOVE_TO_LDC,
                                               cycle_id,
                                               job_id,
                                               ROBOT_ARM_STAGE_WEIGHT_TO_LDC,
                                               part_type,
                                               model_result,
                                               ROBOT_ARM_BIN_NONE);
}

uint8_t RobotArmService_RequestFinalSort(uint16_t cycle_id,
                                         uint16_t job_id,
                                         uint8_t part_type,
                                         uint8_t model_result,
                                         uint8_t final_bin)
{
    if ((final_bin != ROBOT_ARM_BIN_GOOD) &&
        (final_bin != ROBOT_ARM_BIN_BAD) &&
        (final_bin != ROBOT_ARM_BIN_REVIEW))
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault((uint16_t)final_bin,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)job_id,
                                          0U);
        return 0U;
    }

    return RobotArmService_EnqueueStageCommand(ROBOT_ARM_CMD_SORT_RESULT,
                                               cycle_id,
                                               job_id,
                                               ROBOT_ARM_STAGE_LDC_TO_SORT_BIN,
                                               part_type,
                                               model_result,
                                               final_bin);
}

void RobotArmService_Task(void *argument)
{
    RobotArmService_TxContext_t frame;

    (void)argument;

    if (RobotArmService_Init() == 0U)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_ARM_LINK);
        BinaryProtocolService_ReportFault(1U,
                                          BINARY_PROTOCOL_FAULT_SOURCE_ARM,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          0,
                                          0U);
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(ROBOT_ARM_SERVICE_STARTUP_DELAY_MS));
    RobotArmService_SendStartupControl(ROBOT_ARM_CMD_HELLO, "startup-hello");
    vTaskDelay(pdMS_TO_TICKS(20U));
    RobotArmService_SendStartupControl(ROBOT_ARM_CMD_HEARTBEAT, "startup-heartbeat");

    for (;;)
    {
        if (xQueueReceive(g_robot_arm_frame_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            /*
             * USART3 已在 CubeMX 中配置为 115200 8N1。
             * 队列中的每一项都已经是完整正式二进制帧，发送后必须先等 ACK，
             * 对动作命令还必须继续等 STAGE_DONE，避免 ACK 被误当作动作完成。
             */
            RobotArmService_TransmitContext(&frame);
        }
    }
}
