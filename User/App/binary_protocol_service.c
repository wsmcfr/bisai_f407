#include "binary_protocol_service.h"

#ifndef BINARY_PROTOCOL_HOST_TEST
#include "conveyor_motor_service.h"
#include "uart_command.h"
#include "usart.h"
#endif

#include <string.h>

/**
 * @brief F4 二进制协议运行状态。
 *
 * 该结构体只保存协议服务自身的轻量上下文：
 * 1. 当前正在执行的 cycle_id；
 * 2. F4 自己发包使用的序号；
 * 3. 最近一次收到的 MP157 命令序号；
 * 4. 当前自动流程状态。
 */
typedef struct
{
    uint16_t active_cycle_id;             /* 当前有效的自动检测流程 ID，0 表示没有正在运行的流程。 */
    uint16_t tx_sequence;                 /* F4 发送 ACK/NACK/状态帧时使用的本地序号，每发一帧自增。 */
    uint16_t last_rx_sequence;            /* 最近一次成功解析到的 MP157 帧序号，用于状态上报和排查重复帧。 */
    uint16_t fault_bits;                  /* 当前 F4 已知故障位图，会进入 STATUS_REPORT，供 MP157 二进制查询。 */
    uint8_t state;                        /* F4 当前协议主状态，首轮使用简化状态：0=IDLE，1=SCANNING，2=TRACKING，3=CENTERED，11=STOPPED。 */
    uint8_t paused_state;                 /* 暂停前的主状态，用于 RESUME_CYCLE 决定是否回扫描或等待新的视觉坐标。 */
} BinaryProtocol_Runtime_t;

#ifndef BINARY_PROTOCOL_HOST_TEST
/**
 * @brief F4 协议服务的静态运行时状态。
 *
 * 主机侧单元测试只验证 CRC、组帧、解帧和负载解码，不会调用硬件动作分发，
 * 因此该运行时状态只在真实 F4 固件编译时启用，避免主机测试产生无意义 warning。
 */
static BinaryProtocol_Runtime_t g_binary_protocol_runtime = {
    0U,
    0U,
    0U,
    0U,
    0U,
    0U
};
#endif

/**
 * @brief 按小端序读取一个 u16。
 * @param data 指向低字节的地址，不能为空。
 * @return uint16_t 解析出的无符号 16 位整数。
 */
static uint16_t BinaryProtocolService_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief 按小端序读取一个 i16。
 * @param data 指向低字节的地址，不能为空。
 * @return int16_t 解析出的有符号 16 位整数。
 */
static int16_t BinaryProtocolService_ReadI16Le(const uint8_t *data)
{
    return (int16_t)BinaryProtocolService_ReadU16Le(data);
}

/**
 * @brief 按小端序读取一个 u32。
 * @param data 指向最低字节的地址，不能为空。
 * @return uint32_t 解析出的无符号 32 位整数。
 */
static uint32_t BinaryProtocolService_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief 按小端序写入一个 i32。
 * @param data 指向输出最低字节地址，不能为空。
 * @param value 需要写入的有符号 32 位整数。
 */
static void BinaryProtocolService_WriteI32Le(uint8_t *data, int32_t value)
{
    uint32_t raw_value = (uint32_t)value;

    data[0] = (uint8_t)(raw_value & 0xFFU);
    data[1] = (uint8_t)((raw_value >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw_value >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw_value >> 24) & 0xFFU);
}

/**
 * @brief 按小端序写入一个 u16。
 * @param data 指向输出低字节地址，不能为空。
 * @param value 需要写入的无符号 16 位整数。
 */
static void BinaryProtocolService_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/**
 * @brief 判断原始输入是否以本协议二进制帧头开头。
 * @param frame_buffer 原始输入缓存。
 * @param frame_length 原始输入长度。
 * @return uint8_t 1 表示看起来是本协议帧，0 表示不是。
 */
uint8_t BinaryProtocolService_IsBinaryFrame(const uint8_t *frame_buffer, uint16_t frame_length)
{
    if ((frame_buffer == NULL) || (frame_length < 2U))
    {
        return 0U;
    }

    return ((frame_buffer[0] == BINARY_PROTOCOL_SOF0) &&
            (frame_buffer[1] == BINARY_PROTOCOL_SOF1)) ? 1U : 0U;
}

/**
 * @brief 计算 CRC16-CCITT-FALSE。
 * @param data 需要参与 CRC 的数据首地址，不能为空。
 * @param length 需要参与 CRC 的字节数。
 * @return uint16_t CRC 计算结果，组帧时低字节在前。
 *
 * 参数约定：
 * - 本协议实际调用时覆盖 `VER CMD LEN SEQ_L SEQ_H PAYLOAD...`；
 * - 帧头、CRC 字段本身和帧尾不参与计算。
 */
uint16_t BinaryProtocolService_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    for (index = 0U; index < length; ++index)
    {
        crc ^= ((uint16_t)data[index] << 8);
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/**
 * @brief 解析一帧完整二进制协议。
 * @param frame_buffer 原始帧缓存，必须从 `A5 5A` 开始。
 * @param frame_length 原始帧总长度，单位字节。
 * @param parsed_frame 解析结果输出对象，不能为空。
 * @return BinaryProtocol_ParseStatus_t 解析状态。
 *
 * 该函数只做协议层校验，不触发任何硬件动作。
 * 业务动作必须在 `BinaryProtocolService_HandleFrame()` 中根据命令字明确分发。
 */
BinaryProtocol_ParseStatus_t BinaryProtocolService_ParseFrame(const uint8_t *frame_buffer,
                                                              uint16_t frame_length,
                                                              BinaryProtocol_Frame_t *parsed_frame)
{
    uint8_t payload_length;
    uint16_t expected_frame_length;
    uint16_t received_crc;
    uint16_t computed_crc;

    if ((frame_buffer == NULL) || (parsed_frame == NULL))
    {
        return BINARY_PROTOCOL_PARSE_PARAM_ERROR;
    }

    (void)memset(parsed_frame, 0, sizeof(*parsed_frame));

    if (BinaryProtocolService_IsBinaryFrame(frame_buffer, frame_length) == 0U)
    {
        return BINARY_PROTOCOL_PARSE_NOT_BINARY;
    }

    if (frame_length < BINARY_PROTOCOL_MIN_FRAME_LENGTH)
    {
        return BINARY_PROTOCOL_PARSE_TOO_SHORT;
    }

    if (frame_length > BINARY_PROTOCOL_MAX_FRAME_LENGTH)
    {
        return BINARY_PROTOCOL_PARSE_TOO_LONG;
    }

    if (frame_buffer[2] != BINARY_PROTOCOL_VERSION)
    {
        return BINARY_PROTOCOL_PARSE_VERSION_ERROR;
    }

    payload_length = frame_buffer[4];
    if (payload_length > BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH)
    {
        return BINARY_PROTOCOL_PARSE_LENGTH_ERROR;
    }

    expected_frame_length = (uint16_t)(BINARY_PROTOCOL_MIN_FRAME_LENGTH + payload_length);
    if (frame_length != expected_frame_length)
    {
        return BINARY_PROTOCOL_PARSE_LENGTH_ERROR;
    }

    if (frame_buffer[expected_frame_length - 1U] != BINARY_PROTOCOL_EOF)
    {
        return BINARY_PROTOCOL_PARSE_EOF_ERROR;
    }

    received_crc = BinaryProtocolService_ReadU16Le(&frame_buffer[7U + payload_length]);
    computed_crc = BinaryProtocolService_Crc16CcittFalse(&frame_buffer[2], (uint16_t)(5U + payload_length));
    if (received_crc != computed_crc)
    {
        return BINARY_PROTOCOL_PARSE_CRC_ERROR;
    }

    parsed_frame->version = frame_buffer[2];
    parsed_frame->command = frame_buffer[3];
    parsed_frame->payload_length = payload_length;
    parsed_frame->sequence = BinaryProtocolService_ReadU16Le(&frame_buffer[5]);
    parsed_frame->payload = (payload_length == 0U) ? (const uint8_t *)0 : &frame_buffer[7];

    return BINARY_PROTOCOL_PARSE_OK;
}

/**
 * @brief 组装一帧二进制协议。
 * @param command 命令字。
 * @param sequence 发送方帧序号。
 * @param payload 负载地址，没有负载时可以为 NULL。
 * @param payload_length 负载长度，单位字节。
 * @param output_buffer 输出帧缓存，不能为空。
 * @param output_size 输出帧缓存大小，必须能容纳完整帧。
 * @return uint16_t 实际组出的帧长度，0 表示参数非法或缓存不足。
 */
uint16_t BinaryProtocolService_BuildFrame(uint8_t command,
                                          uint16_t sequence,
                                          const uint8_t *payload,
                                          uint8_t payload_length,
                                          uint8_t *output_buffer,
                                          uint16_t output_size)
{
    uint16_t frame_length;
    uint16_t crc;

    if ((output_buffer == NULL) ||
        (payload_length > BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH) ||
        ((payload_length > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    frame_length = (uint16_t)(BINARY_PROTOCOL_MIN_FRAME_LENGTH + payload_length);
    if (output_size < frame_length)
    {
        return 0U;
    }

    output_buffer[0] = BINARY_PROTOCOL_SOF0;
    output_buffer[1] = BINARY_PROTOCOL_SOF1;
    output_buffer[2] = BINARY_PROTOCOL_VERSION;
    output_buffer[3] = command;
    output_buffer[4] = payload_length;
    BinaryProtocolService_WriteU16Le(&output_buffer[5], sequence);

    if (payload_length > 0U)
    {
        (void)memcpy(&output_buffer[7], payload, payload_length);
    }

    crc = BinaryProtocolService_Crc16CcittFalse(&output_buffer[2], (uint16_t)(5U + payload_length));
    BinaryProtocolService_WriteU16Le(&output_buffer[7U + payload_length], crc);
    output_buffer[frame_length - 1U] = BINARY_PROTOCOL_EOF;

    return frame_length;
}

/**
 * @brief 解码 START_CYCLE 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeStartCycle(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_StartCyclePayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_START_CYCLE_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->mode = payload[2];
    decoded_payload->option_bits = BinaryProtocolService_ReadU16Le(&payload[3]);
    decoded_payload->camera_profile = payload[5];
    return 1U;
}

/**
 * @brief 解码 PAUSE_CYCLE 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodePauseCycle(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_PauseCyclePayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_PAUSE_CYCLE_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->pause_reason = payload[2];
    decoded_payload->pause_mode = payload[3];
    return 1U;
}

/**
 * @brief 解码 RESUME_CYCLE 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeResumeCycle(const uint8_t *payload,
                                                uint8_t payload_length,
                                                BinaryProtocol_ResumeCyclePayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_RESUME_CYCLE_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->resume_mode = payload[2];
    return 1U;
}

/**
 * @brief 解码 STOP_CYCLE 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeStopCycle(const uint8_t *payload,
                                              uint8_t payload_length,
                                              BinaryProtocol_StopCyclePayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_STOP_CYCLE_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->stop_reason = payload[2];
    decoded_payload->stop_level = payload[3];
    return 1U;
}

/**
 * @brief 解码 VISION_POS 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeVisionPos(const uint8_t *payload,
                                              uint8_t payload_length,
                                              BinaryProtocol_VisionPosPayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_VISION_POS_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->frame_id = BinaryProtocolService_ReadU16Le(&payload[2]);
    decoded_payload->flags = payload[4];
    decoded_payload->part_type = payload[5];
    decoded_payload->axis_px = BinaryProtocolService_ReadI16Le(&payload[6]);
    decoded_payload->target_px = BinaryProtocolService_ReadI16Le(&payload[8]);
    decoded_payload->center_x_px = BinaryProtocolService_ReadI16Le(&payload[10]);
    decoded_payload->center_y_px = BinaryProtocolService_ReadI16Le(&payload[12]);
    decoded_payload->bbox_x_px = BinaryProtocolService_ReadI16Le(&payload[14]);
    decoded_payload->bbox_y_px = BinaryProtocolService_ReadI16Le(&payload[16]);
    decoded_payload->bbox_w_px = BinaryProtocolService_ReadI16Le(&payload[18]);
    decoded_payload->bbox_h_px = BinaryProtocolService_ReadI16Le(&payload[20]);
    decoded_payload->confidence = payload[22];
    decoded_payload->reserved = payload[23];
    decoded_payload->capture_ms = BinaryProtocolService_ReadU32Le(&payload[24]);
    return 1U;
}

/**
 * @brief 解码 VISION_LOST 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeVisionLost(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_VisionLostPayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_VISION_LOST_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->frame_id = BinaryProtocolService_ReadU16Le(&payload[2]);
    decoded_payload->reason = payload[4];
    decoded_payload->confidence = payload[5];
    decoded_payload->ms_since_seen = BinaryProtocolService_ReadU16Le(&payload[6]);
    return 1U;
}

/**
 * @brief 解码 BELT_STOP_CENTERED 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeBeltCentered(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_BeltCenteredPayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_BELT_CENTERED_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->frame_id = BinaryProtocolService_ReadU16Le(&payload[2]);
    decoded_payload->reason = payload[4];
    decoded_payload->hold_ms = BinaryProtocolService_ReadU16Le(&payload[5]);
    decoded_payload->reserved = payload[7];
    return 1U;
}

/**
 * @brief 解码 QUERY_STATUS 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeQueryStatus(const uint8_t *payload,
                                                uint8_t payload_length,
                                                BinaryProtocol_QueryStatusPayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_QUERY_STATUS_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->query_mask = payload[2];
    return 1U;
}

/**
 * @brief 解码 BELT_MANUAL_CONTROL 负载。
 * @param payload 原始负载。
 * @param payload_length 原始负载长度。
 * @param decoded_payload 解码输出对象。
 * @return uint8_t 1 表示解码成功，0 表示长度或参数非法。
 */
uint8_t BinaryProtocolService_DecodeBeltManual(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_BeltManualPayload_t *decoded_payload)
{
    if ((payload == NULL) ||
        (decoded_payload == NULL) ||
        (payload_length != BINARY_PROTOCOL_BELT_MANUAL_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    decoded_payload->cycle_id = BinaryProtocolService_ReadU16Le(&payload[0]);
    decoded_payload->action = payload[2];
    decoded_payload->flags = payload[3];
    return 1U;
}

#ifndef BINARY_PROTOCOL_HOST_TEST
/**
 * @brief 通过 USART1 发送一帧二进制协议。
 * @param command 命令字。
 * @param payload 负载地址，没有负载时可以为 NULL。
 * @param payload_length 负载长度。
 *
 * 该函数集中处理 F4 侧发送序号递增和原始字节发送。
 * 发送失败时当前首轮不重试，因为 ACK/NACK 本身只是上位机调试提示，不应阻塞称重主任务。
 */
static void BinaryProtocolService_SendFrame(uint8_t command, const uint8_t *payload, uint8_t payload_length)
{
    uint8_t tx_buffer[BINARY_PROTOCOL_MAX_FRAME_LENGTH];
    uint16_t frame_length;

    frame_length = BinaryProtocolService_BuildFrame(command,
                                                    g_binary_protocol_runtime.tx_sequence,
                                                    payload,
                                                    payload_length,
                                                    tx_buffer,
                                                    (uint16_t)sizeof(tx_buffer));
    if (frame_length == 0U)
    {
        return;
    }

    ++g_binary_protocol_runtime.tx_sequence;
    (void)UartCommand_SendRaw(&huart1, tx_buffer, frame_length, 0xFFU);
}

/**
 * @brief 发送 ACK。
 * @param cycle_id 当前流程 ID，没有流程时填 0。
 * @param acked_seq 被确认帧的序号。
 * @param acked_cmd 被确认帧的命令字。
 * @param status ACK 状态，0=接受，1=重复但状态正常。
 */
static void BinaryProtocolService_SendAck(uint16_t cycle_id,
                                          uint16_t acked_seq,
                                          uint8_t acked_cmd,
                                          uint8_t status)
{
    uint8_t payload[BINARY_PROTOCOL_ACK_PAYLOAD_LENGTH];

    BinaryProtocolService_WriteU16Le(&payload[0], cycle_id);
    BinaryProtocolService_WriteU16Le(&payload[2], acked_seq);
    payload[4] = acked_cmd;
    payload[5] = status;
    payload[6] = g_binary_protocol_runtime.state;
    BinaryProtocolService_SendFrame(BINARY_PROTOCOL_CMD_ACK, payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 发送 NACK。
 * @param cycle_id 当前流程 ID，没有流程时填 0。
 * @param rejected_seq 被拒绝帧的序号。
 * @param rejected_cmd 被拒绝帧的命令字。
 * @param error_code 错误码。
 * @param detail 附加错误信息，没有时填 0。
 */
static void BinaryProtocolService_SendNack(uint16_t cycle_id,
                                           uint16_t rejected_seq,
                                           uint8_t rejected_cmd,
                                           uint8_t error_code,
                                           uint16_t detail)
{
    uint8_t payload[BINARY_PROTOCOL_NACK_PAYLOAD_LENGTH];

    BinaryProtocolService_WriteU16Le(&payload[0], cycle_id);
    BinaryProtocolService_WriteU16Le(&payload[2], rejected_seq);
    payload[4] = rejected_cmd;
    payload[5] = error_code;
    payload[6] = g_binary_protocol_runtime.state;
    BinaryProtocolService_WriteU16Le(&payload[7], detail);
    BinaryProtocolService_SendFrame(BINARY_PROTOCOL_CMD_NACK, payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 设置 F4 当前故障位。
 * @param fault_bit 要置位的故障位，见 BinaryProtocol_FaultBit_t。
 *
 * 该函数只更新协议运行时位图，不打印文本。
 * MP157 后续通过 STATUS_REPORT 或 FAULT_REPORT 获取结构化故障。
 */
void BinaryProtocolService_SetFaultBit(uint16_t fault_bit)
{
    g_binary_protocol_runtime.fault_bits |= fault_bit;
}

/**
 * @brief 清除 F4 当前故障位。
 * @param fault_bit 要清除的故障位，见 BinaryProtocol_FaultBit_t。
 */
void BinaryProtocolService_ClearFaultBit(uint16_t fault_bit)
{
    g_binary_protocol_runtime.fault_bits =
        (uint16_t)(g_binary_protocol_runtime.fault_bits & (uint16_t)(~fault_bit));
}

/**
 * @brief 发送 FAULT_REPORT 二进制故障帧。
 * @param fault_code 故障码，按具体模块保存底层状态或自定义编号。
 * @param fault_source 故障来源，见 BinaryProtocol_FaultSource_t。
 * @param severity 故障严重等级，见 BinaryProtocol_FaultSeverity_t。
 * @param detail_i32 附加 32 位有符号细节，例如底层 status。
 * @param related_seq 触发该故障的 MP157 命令序号，没有则填 0。
 *
 * 负载固定 16 字节：
 * cycle_id:u16、fault_code:u16、fault_source:u8、severity:u8、state:u8、reserved:u8、
 * detail_i32:i32、related_seq:u16、fault_bits:u16。
 */
void BinaryProtocolService_ReportFault(uint16_t fault_code,
                                       uint8_t fault_source,
                                       uint8_t severity,
                                       int32_t detail_i32,
                                       uint16_t related_seq)
{
    uint8_t payload[BINARY_PROTOCOL_FAULT_REPORT_PAYLOAD_LENGTH];

    BinaryProtocolService_WriteU16Le(&payload[0], g_binary_protocol_runtime.active_cycle_id);
    BinaryProtocolService_WriteU16Le(&payload[2], fault_code);
    payload[4] = fault_source;
    payload[5] = severity;
    payload[6] = g_binary_protocol_runtime.state;
    payload[7] = 0U;
    BinaryProtocolService_WriteI32Le(&payload[8], detail_i32);
    BinaryProtocolService_WriteU16Le(&payload[12], related_seq);
    BinaryProtocolService_WriteU16Le(&payload[14], g_binary_protocol_runtime.fault_bits);
    BinaryProtocolService_SendFrame(BINARY_PROTOCOL_CMD_FAULT_REPORT, payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 对解析失败的二进制帧尽量返回 NACK。
 * @param frame_buffer 原始帧缓存。
 * @param frame_length 原始帧长度。
 * @param parse_status 解析失败原因。
 *
 * 坏帧不触发硬件动作，但仍通过二进制 NACK 告诉 MP157 为什么失败。
 * 如果帧太短导致取不到 SEQ/CMD，则用 0 填充被拒绝字段。
 */
static void BinaryProtocolService_SendParseErrorNack(const uint8_t *frame_buffer,
                                                     uint16_t frame_length,
                                                     BinaryProtocol_ParseStatus_t parse_status)
{
    uint8_t rejected_cmd = 0U;
    uint16_t rejected_seq = 0U;
    uint8_t error_code = BINARY_PROTOCOL_ERROR_FRAME_LENGTH;

    if ((frame_buffer != NULL) && (frame_length > 3U))
    {
        rejected_cmd = frame_buffer[3];
    }

    if ((frame_buffer != NULL) && (frame_length > 6U))
    {
        rejected_seq = BinaryProtocolService_ReadU16Le(&frame_buffer[5]);
    }

    if (parse_status == BINARY_PROTOCOL_PARSE_CRC_ERROR)
    {
        error_code = BINARY_PROTOCOL_ERROR_CRC;
    }
    else if (parse_status == BINARY_PROTOCOL_PARSE_VERSION_ERROR)
    {
        error_code = BINARY_PROTOCOL_ERROR_FIELD_RANGE;
    }

    BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                   rejected_seq,
                                   rejected_cmd,
                                   error_code,
                                   (uint16_t)parse_status);
}

/**
 * @brief 发送 STATUS_REPORT。
 * @param cycle_id 查询命令中的流程 ID。
 * @param replied_seq 被回复帧的序号。
 * @param replied_cmd 被回复帧的命令字，当前应为 QUERY_STATUS。
 *
 * 负载格式固定 24 字节：
 * cycle_id:u16、replied_seq:u16、replied_cmd:u8、f4_state:u8、active_cycle_id:u16、
 * paused_state:u8、belt_desired:u8、belt_applied:u8、belt_dir:u8、belt_centered:u8、
 * belt_stable:u8、belt_speed_rpm:u16、belt_error_px:i32、feature_bits:u16、fault_bits:u16。
 */
static void BinaryProtocolService_SendStatusReport(uint16_t cycle_id,
                                                   uint16_t replied_seq,
                                                   uint8_t replied_cmd)
{
    ConveyorMotor_Status_t belt_status;
    uint8_t payload[BINARY_PROTOCOL_STATUS_REPORT_PAYLOAD_LENGTH];
    uint16_t feature_bits = 0x0001U;

    if (ConveyorMotorService_GetStatus(&belt_status) == 0U)
    {
        belt_status.desired_mode = 0U;
        belt_status.applied_mode = 0U;
        belt_status.latest_error_px = 0;
        belt_status.speed_rpm = 0U;
        belt_status.direction = 0U;
        belt_status.stable_count = 0U;
        belt_status.centered = 0U;
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
    }
    else
    {
        BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
    }

    BinaryProtocolService_WriteU16Le(&payload[0], cycle_id);
    BinaryProtocolService_WriteU16Le(&payload[2], replied_seq);
    payload[4] = replied_cmd;
    payload[5] = g_binary_protocol_runtime.state;
    BinaryProtocolService_WriteU16Le(&payload[6], g_binary_protocol_runtime.active_cycle_id);
    payload[8] = g_binary_protocol_runtime.paused_state;
    payload[9] = belt_status.desired_mode;
    payload[10] = belt_status.applied_mode;
    payload[11] = belt_status.direction;
    payload[12] = belt_status.centered;
    payload[13] = belt_status.stable_count;
    BinaryProtocolService_WriteU16Le(&payload[14], belt_status.speed_rpm);
    BinaryProtocolService_WriteI32Le(&payload[16], belt_status.latest_error_px);
    BinaryProtocolService_WriteU16Le(&payload[20], feature_bits);
    BinaryProtocolService_WriteU16Le(&payload[22], g_binary_protocol_runtime.fault_bits);
    BinaryProtocolService_SendFrame(BINARY_PROTOCOL_CMD_STATUS_REPORT, payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 判断命令 cycle_id 是否允许作用于当前流程。
 * @param cycle_id 命令中的流程 ID。
 * @return uint8_t 1 表示允许，0 表示不允许。
 *
 * 规则：
 * - 当前没有 active cycle 时，只允许 START_CYCLE 创建新流程；
 * - 已有 active cycle 时，后续视觉、停止和居中命令必须使用同一个 cycle_id。
 */
static uint8_t BinaryProtocolService_IsActiveCycle(uint16_t cycle_id)
{
    return ((g_binary_protocol_runtime.active_cycle_id != 0U) &&
            (g_binary_protocol_runtime.active_cycle_id == cycle_id)) ? 1U : 0U;
}

/**
 * @brief 处理 START_CYCLE。
 * @param frame 已解析帧。
 */
static void BinaryProtocolService_HandleStartCycle(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_StartCyclePayload_t payload;

    if (BinaryProtocolService_DecodeStartCycle(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(0U,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (g_binary_protocol_runtime.active_cycle_id != 0U)
    {
        if (g_binary_protocol_runtime.active_cycle_id == payload.cycle_id)
        {
            /*
             * 同一个 cycle_id 的 START_CYCLE 视为重发帧：
             * 1. 如果 ACK 在串口链路上丢失，MP157 会按重发策略再次发送同一命令；
             * 2. 对扫描/跟踪等可输送阶段，重复 START_CYCLE 需要重新投递一次 SCAN，
             *    避免 MP157 程序重启后 cycle_id 又从 1 开始时只收到 ACK，但传送带没有新动作；
             * 3. 暂停态例外，暂停后的继续必须使用 RESUME_CYCLE，避免首页“开始”误恢复旧流程；
             * 4. 若传送带队列未就绪，则返回硬件故障 NACK，不能用 ACK 掩盖电机服务不可用。
             */
            if (g_binary_protocol_runtime.state == 10U)
            {
                BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                               frame->sequence,
                                               frame->command,
                                               BINARY_PROTOCOL_ERROR_STATE_NOT_ALLOWED,
                                               g_binary_protocol_runtime.state);
            }
            else
            {
                if (ConveyorMotorService_RequestScan() == 0U)
                {
                    BinaryProtocolService_SendNack(payload.cycle_id,
                                                   frame->sequence,
                                                   frame->command,
                                                   BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                                   0U);
                    return;
                }

                g_binary_protocol_runtime.state = 1U;
                g_binary_protocol_runtime.paused_state = 0U;
                BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
            }
        }
        else
        {
            /*
             * 已有流程运行时拒绝新的 cycle_id，避免一个误触或上位机状态错乱把当前流程覆盖掉。
             * 首页若要“暂停后重新开始”，应先发 STOP_CYCLE 作废旧 cycle，再新建 cycle 发送 START_CYCLE。
             */
            BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                           frame->sequence,
                                           frame->command,
                                           BINARY_PROTOCOL_ERROR_BUSY,
                                           payload.cycle_id);
        }
        return;
    }

    if (ConveyorMotorService_RequestScan() == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    /*
     * 只有传送带扫描请求成功投递后，才把本轮 cycle 记为有效。
     * 如果队列尚未创建或传送带任务未就绪，F4 已经回 NACK，此时不能提前占用 active_cycle_id，
     * 否则 MP157 重发 START_CYCLE 或新建一轮时会被误判为忙。
     */
    g_binary_protocol_runtime.active_cycle_id = payload.cycle_id;
    g_binary_protocol_runtime.state = 1U;
    g_binary_protocol_runtime.paused_state = 0U;

    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 PAUSE_CYCLE。
 * @param frame 已解析帧。
 *
 * 首轮暂停策略：
 * 1. 校验 cycle_id 必须匹配当前流程；
 * 2. 立即请求传送带停止；
 * 3. 保存暂停前状态，后续 RESUME_CYCLE 根据该状态恢复。
 */
static void BinaryProtocolService_HandlePauseCycle(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_PauseCyclePayload_t payload;

    if (BinaryProtocolService_DecodePauseCycle(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if (g_binary_protocol_runtime.state == 10U)
    {
        BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 1U);
        return;
    }

    if (ConveyorMotorService_RequestStop() == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    /*
     * 停止请求成功投递后再记录暂停前状态。
     * 如果传送带队列未就绪导致 NACK，协议状态仍保持原来的运行态，方便 MP157 重试或直接 STOP。
     */
    g_binary_protocol_runtime.paused_state = g_binary_protocol_runtime.state;
    g_binary_protocol_runtime.state = 10U;
    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 RESUME_CYCLE。
 * @param frame 已解析帧。
 *
 * 首轮继续策略：
 * - `resume_mode=1` 强制回扫描；
 * - 暂停前是扫描时回扫描；
 * - 暂停前是跟踪时不立刻使用旧坐标运动，等待 MP157 下一帧 VISION_POS；
 * - 暂停前已经居中时保持停止，等待模型或机械臂流程继续。
 */
static void BinaryProtocolService_HandleResumeCycle(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_ResumeCyclePayload_t payload;

    if (BinaryProtocolService_DecodeResumeCycle(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if (g_binary_protocol_runtime.state != 10U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_STATE_NOT_ALLOWED,
                                       g_binary_protocol_runtime.state);
        return;
    }

    if ((payload.resume_mode == 1U) || (g_binary_protocol_runtime.paused_state == 1U))
    {
        if (ConveyorMotorService_RequestScan() == 0U)
        {
            BinaryProtocolService_SendNack(payload.cycle_id,
                                           frame->sequence,
                                           frame->command,
                                           BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                           0U);
            return;
        }
        g_binary_protocol_runtime.state = 1U;
    }
    else
    {
        g_binary_protocol_runtime.state = g_binary_protocol_runtime.paused_state;
    }

    g_binary_protocol_runtime.paused_state = 0U;
    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 STOP_CYCLE。
 * @param frame 已解析帧。
 */
static void BinaryProtocolService_HandleStopCycle(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_StopCyclePayload_t payload;

    if (BinaryProtocolService_DecodeStopCycle(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if (ConveyorMotorService_RequestStop() == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    g_binary_protocol_runtime.active_cycle_id = 0U;
    g_binary_protocol_runtime.state = 11U;
    g_binary_protocol_runtime.paused_state = 0U;
    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 VISION_POS。
 * @param frame 已解析帧。
 */
static void BinaryProtocolService_HandleVisionPos(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_VisionPosPayload_t payload;
    int32_t error_px;

    if (BinaryProtocolService_DecodeVisionPos(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if ((payload.flags & 0x01U) == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.flags);
        return;
    }

    if (payload.confidence > 100U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.confidence);
        return;
    }

    error_px = (int32_t)payload.axis_px - (int32_t)payload.target_px;
    if (ConveyorMotorService_RequestTrack(error_px) == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    /*
     * 只有视觉误差成功投递给传送带任务后，协议主状态才进入 TRACKING。
     * 这样 MP157 看到 NACK 时，F4 内部状态不会和实际电机任务状态不一致。
     */
    g_binary_protocol_runtime.state = 2U;
    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 VISION_LOST。
 * @param frame 已解析帧。
 */
static void BinaryProtocolService_HandleVisionLost(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_VisionLostPayload_t payload;
    uint8_t accepted;

    if (BinaryProtocolService_DecodeVisionLost(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if ((payload.reason < 1U) || (payload.reason > 4U))
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.reason);
        return;
    }

    if (payload.confidence > 100U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.confidence);
        return;
    }

    if (payload.reason == 4U)
    {
        accepted = ConveyorMotorService_RequestStop();
        if (accepted == 0U)
        {
            BinaryProtocolService_SendNack(payload.cycle_id,
                                           frame->sequence,
                                           frame->command,
                                           BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                           payload.reason);
            return;
        }

        g_binary_protocol_runtime.state = 12U;
    }
    else
    {
        accepted = ConveyorMotorService_RequestScan();
        if (accepted == 0U)
        {
            BinaryProtocolService_SendNack(payload.cycle_id,
                                           frame->sequence,
                                           frame->command,
                                           BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                           payload.reason);
            return;
        }

        g_binary_protocol_runtime.state = 1U;
    }

    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 BELT_STOP_CENTERED。
 * @param frame 已解析帧。
 */
static void BinaryProtocolService_HandleBeltCentered(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_BeltCenteredPayload_t payload;

    if (BinaryProtocolService_DecodeBeltCentered(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (BinaryProtocolService_IsActiveCycle(payload.cycle_id) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH,
                                       payload.cycle_id);
        return;
    }

    if (ConveyorMotorService_RequestStop() == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    g_binary_protocol_runtime.state = 3U;
    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理 QUERY_STATUS。
 * @param frame 已解析帧。
 *
 * 查询类命令成功时不回 ACK，而是直接回 STATUS_REPORT，
 * 这样 MP157 能拿到结构化状态，不再解析 BELTINFO 文本。
 */
static void BinaryProtocolService_HandleQueryStatus(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_QueryStatusPayload_t payload;

    if (BinaryProtocolService_DecodeQueryStatus(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if ((payload.query_mask & 0x03U) == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.query_mask);
        return;
    }

    BinaryProtocolService_SendStatusReport(payload.cycle_id, frame->sequence, frame->command);
}

/**
 * @brief 处理 BELT_MANUAL_CONTROL。
 * @param frame 已解析帧。
 *
 * 首轮只开放传送带调试需要的两个动作：
 * - action=0：停止；
 * - action=1：扫描巡航。
 */
static void BinaryProtocolService_HandleBeltManual(const BinaryProtocol_Frame_t *frame)
{
    BinaryProtocol_BeltManualPayload_t payload;
    uint8_t accepted = 0U;

    if (BinaryProtocolService_DecodeBeltManual(frame->payload, frame->payload_length, &payload) == 0U)
    {
        BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH,
                                       frame->payload_length);
        return;
    }

    if (payload.flags != 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.flags);
        return;
    }

    if (payload.action == 0U)
    {
        accepted = ConveyorMotorService_RequestStop();
    }
    else if (payload.action == 1U)
    {
        accepted = ConveyorMotorService_RequestScan();
    }
    else
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_FIELD_RANGE,
                                       payload.action);
        return;
    }

    if (accepted == 0U)
    {
        BinaryProtocolService_SendNack(payload.cycle_id,
                                       frame->sequence,
                                       frame->command,
                                       BINARY_PROTOCOL_ERROR_HARDWARE_FAULT,
                                       0U);
        return;
    }

    BinaryProtocolService_SendAck(payload.cycle_id, frame->sequence, frame->command, 0U);
}

/**
 * @brief 处理一帧来自 USART1 的二进制协议。
 * @param frame_buffer 原始帧缓存。
 * @param frame_length 原始帧长度。
 * @return uint8_t 1 表示该帧属于本协议且已经处理，0 表示不是本协议帧。
 *
 * 注意：
 * - 返回 1 不代表命令执行成功，只表示该帧已被本协议消费；
 * - 执行成功或失败只通过 ACK/NACK/STATUS_REPORT 二进制帧体现；
 * - CRC 错误不会继续落入 ASCII 或机械臂协议解析，避免误动作。
 */
uint8_t BinaryProtocolService_HandleFrame(const uint8_t *frame_buffer, uint16_t frame_length)
{
    BinaryProtocol_Frame_t frame;
    BinaryProtocol_ParseStatus_t parse_status;

    if (BinaryProtocolService_IsBinaryFrame(frame_buffer, frame_length) == 0U)
    {
        return 0U;
    }

    parse_status = BinaryProtocolService_ParseFrame(frame_buffer, frame_length, &frame);
    if (parse_status != BINARY_PROTOCOL_PARSE_OK)
    {
        BinaryProtocolService_SendParseErrorNack(frame_buffer, frame_length, parse_status);
        return 1U;
    }

    g_binary_protocol_runtime.last_rx_sequence = frame.sequence;

    switch (frame.command)
    {
        case BINARY_PROTOCOL_CMD_HELLO:
        case BINARY_PROTOCOL_CMD_HEARTBEAT:
            BinaryProtocolService_SendAck(g_binary_protocol_runtime.active_cycle_id,
                                          frame.sequence,
                                          frame.command,
                                          0U);
            break;

        case BINARY_PROTOCOL_CMD_START_CYCLE:
            BinaryProtocolService_HandleStartCycle(&frame);
            break;

        case BINARY_PROTOCOL_CMD_PAUSE_CYCLE:
            BinaryProtocolService_HandlePauseCycle(&frame);
            break;

        case BINARY_PROTOCOL_CMD_RESUME_CYCLE:
            BinaryProtocolService_HandleResumeCycle(&frame);
            break;

        case BINARY_PROTOCOL_CMD_STOP_CYCLE:
            BinaryProtocolService_HandleStopCycle(&frame);
            break;

        case BINARY_PROTOCOL_CMD_VISION_POS:
            BinaryProtocolService_HandleVisionPos(&frame);
            break;

        case BINARY_PROTOCOL_CMD_VISION_LOST:
            BinaryProtocolService_HandleVisionLost(&frame);
            break;

        case BINARY_PROTOCOL_CMD_BELT_STOP_CENTERED:
            BinaryProtocolService_HandleBeltCentered(&frame);
            break;

        case BINARY_PROTOCOL_CMD_QUERY_STATUS:
            BinaryProtocolService_HandleQueryStatus(&frame);
            break;

        case BINARY_PROTOCOL_CMD_BELT_MANUAL_CONTROL:
            BinaryProtocolService_HandleBeltManual(&frame);
            break;

        default:
            BinaryProtocolService_SendNack(g_binary_protocol_runtime.active_cycle_id,
                                           frame.sequence,
                                           frame.command,
                                           BINARY_PROTOCOL_ERROR_CMD_UNKNOWN,
                                           frame.command);
            break;
    }

    return 1U;
}
#else
uint8_t BinaryProtocolService_HandleFrame(const uint8_t *frame_buffer, uint16_t frame_length)
{
    (void)frame_buffer;
    (void)frame_length;
    return 0U;
}
#endif
