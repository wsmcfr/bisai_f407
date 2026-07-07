#include "robot_arm_rx_parser.h" /* 本文件实现机械臂串口接收缓存与正式协议重同步逻辑。 */

#include <string.h> /* 使用 memset/memmove/memcpy 维护线性接收缓存。 */

/**
 * @brief 从缓存头部移除指定字节数。
 * @param context 接收缓存上下文，不能为空。
 * @param count 需要移除的字节数。
 *
 * 说明：
 * - 当前缓存实现使用线性数组而非环形队列；
 * - 每次提取出完整帧或确认前导噪声无效后，就把剩余数据整体前移；
 * - 当前机械臂链路帧频很低，这种实现足够直接，维护成本也更低。
 */
static void RobotArmRxParser_ConsumePrefix(RobotArmRxParser_Context_t *context, uint16_t count)
{
    if ((context == NULL) || (count == 0U))
    {
        return;
    }

    if (count >= context->length)
    {
        context->length = 0U;
        return;
    }

    (void)memmove(context->buffer,
                  &context->buffer[count],
                  (size_t)(context->length - count));
    context->length = (uint16_t)(context->length - count);
}

/**
 * @brief 在当前缓存中查找第一个 `0xA5` 帧头候选字节。
 * @param context 接收缓存上下文，不能为空。
 * @return int32_t 找到时返回索引；未找到返回 -1。
 */
static int32_t RobotArmRxParser_FindSof0(const RobotArmRxParser_Context_t *context)
{
    uint16_t index;

    if (context == NULL)
    {
        return -1;
    }

    for (index = 0U; index < context->length; ++index)
    {
        if (context->buffer[index] == BINARY_PROTOCOL_SOF0)
        {
            return (int32_t)index;
        }
    }

    return -1;
}

void RobotArmRxParser_Init(RobotArmRxParser_Context_t *context)
{
    if (context == NULL)
    {
        return;
    }

    (void)memset(context, 0, sizeof(*context));
}

void RobotArmRxParser_PushBytes(RobotArmRxParser_Context_t *context,
                                const uint8_t *data,
                                uint16_t length)
{
    uint16_t overflow = 0U;

    if ((context == NULL) || (length == 0U))
    {
        return;
    }

    if (data == NULL)
    {
        return;
    }

    if (length >= ROBOT_ARM_RX_PARSER_BUFFER_SIZE)
    {
        context->dropped_bytes = (uint16_t)(context->dropped_bytes +
                                            (length - ROBOT_ARM_RX_PARSER_BUFFER_SIZE));
        (void)memcpy(context->buffer,
                     &data[length - ROBOT_ARM_RX_PARSER_BUFFER_SIZE],
                     ROBOT_ARM_RX_PARSER_BUFFER_SIZE);
        context->length = ROBOT_ARM_RX_PARSER_BUFFER_SIZE;
        return;
    }

    if ((uint16_t)(context->length + length) > ROBOT_ARM_RX_PARSER_BUFFER_SIZE)
    {
        overflow = (uint16_t)(context->length + length - ROBOT_ARM_RX_PARSER_BUFFER_SIZE);
        RobotArmRxParser_ConsumePrefix(context, overflow);
        context->dropped_bytes = (uint16_t)(context->dropped_bytes + overflow);
    }

    (void)memcpy(&context->buffer[context->length], data, length);
    context->length = (uint16_t)(context->length + length);
}

RobotArmRxParser_Result_t RobotArmRxParser_TryExtractFrame(RobotArmRxParser_Context_t *context,
                                                           uint8_t *out_raw_frame,
                                                           uint16_t out_raw_frame_size,
                                                           uint16_t *out_raw_frame_length,
                                                           BinaryProtocol_Frame_t *out_parsed_frame)
{
    int32_t sof_index;
    uint8_t payload_length;
    uint16_t frame_length;
    BinaryProtocol_ParseStatus_t parse_status;

    if (out_raw_frame_length != NULL)
    {
        *out_raw_frame_length = 0U;
    }

    if (out_parsed_frame != NULL)
    {
        (void)memset(out_parsed_frame, 0, sizeof(*out_parsed_frame));
    }

    if (context == NULL)
    {
        return ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE;
    }

    for (;;)
    {
        sof_index = RobotArmRxParser_FindSof0(context);
        if (sof_index < 0)
        {
            if (context->length > 0U)
            {
                context->dropped_bytes = (uint16_t)(context->dropped_bytes + context->length);
                context->length = 0U;
            }
            return ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE;
        }

        if (sof_index > 0)
        {
            RobotArmRxParser_ConsumePrefix(context, (uint16_t)sof_index);
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + (uint16_t)sof_index);
            continue;
        }

        if (context->length < 2U)
        {
            return ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE;
        }

        if (context->buffer[1] != BINARY_PROTOCOL_SOF1)
        {
            RobotArmRxParser_ConsumePrefix(context, 1U);
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + 1U);
            continue;
        }

        if (context->length < BINARY_PROTOCOL_MIN_FRAME_LENGTH)
        {
            return ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE;
        }

        payload_length = context->buffer[4];
        if (payload_length > BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH)
        {
            RobotArmRxParser_ConsumePrefix(context, 1U);
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + 1U);
            continue;
        }

        frame_length = (uint16_t)(BINARY_PROTOCOL_MIN_FRAME_LENGTH + payload_length);
        if (context->length < frame_length)
        {
            return ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE;
        }

        parse_status = BINARY_PROTOCOL_PARSE_PARAM_ERROR;
        if (out_parsed_frame != NULL)
        {
            parse_status = BinaryProtocolService_ParseFrame(context->buffer, frame_length, out_parsed_frame);
        }
        else
        {
            BinaryProtocol_Frame_t temp_frame;
            parse_status = BinaryProtocolService_ParseFrame(context->buffer, frame_length, &temp_frame);
        }

        if (parse_status != BINARY_PROTOCOL_PARSE_OK)
        {
            RobotArmRxParser_ConsumePrefix(context, 1U);
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + 1U);
            continue;
        }

        if ((out_raw_frame != NULL) && (out_raw_frame_size >= frame_length))
        {
            (void)memcpy(out_raw_frame, context->buffer, frame_length);
        }

        if (out_raw_frame_length != NULL)
        {
            *out_raw_frame_length = frame_length;
        }

        RobotArmRxParser_ConsumePrefix(context, frame_length);
        return ROBOT_ARM_RX_PARSER_RESULT_FRAME_READY;
    }
}
