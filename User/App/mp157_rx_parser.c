#include "mp157_rx_parser.h" /* 引入解析器公开接口和协议常量。 */

#include <string.h> /* 提供 memset、memcpy 和 memmove，维护连续字节缓存。 */

/**
 * @brief 从缓存头部移除指定数量字节。
 * @param context 解析上下文，不能为空。
 * @param remove_length 要移除的字节数；超过有效长度时清空全部缓存。
 * @return 无返回值。
 */
static void Mp157RxParser_RemovePrefix(Mp157RxParser_Context_t *context,
                                       uint16_t remove_length)
{
    if ((context == NULL) || (remove_length == 0U))
    {
        return;
    }

    if (remove_length >= context->length)
    {
        context->length = 0U;
        return;
    }

    (void)memmove(context->buffer,
                  &context->buffer[remove_length],
                  (size_t)(context->length - remove_length));
    context->length = (uint16_t)(context->length - remove_length);
}

/**
 * @brief 初始化 MP157 字节流解析上下文。
 * @param context 待初始化上下文。
 * @return 无返回值。
 */
void Mp157RxParser_Init(Mp157RxParser_Context_t *context)
{
    if (context == NULL)
    {
        return;
    }

    (void)memset(context, 0, sizeof(*context));
}

/**
 * @brief 把新字节追加到解析缓存，溢出时保留最新数据。
 * @param context 解析上下文。
 * @param data 新数据首地址。
 * @param length 新数据长度。
 * @return 无返回值。
 */
void Mp157RxParser_PushBytes(Mp157RxParser_Context_t *context,
                             const uint8_t *data,
                             uint16_t length)
{
    uint16_t free_length;
    uint16_t remove_length;

    if ((context == NULL) || (length == 0U))
    {
        return;
    }

    if (data == NULL)
    {
        return;
    }

    if (length >= MP157_RX_PARSER_BUFFER_SIZE)
    {
        data = &data[length - MP157_RX_PARSER_BUFFER_SIZE];
        context->dropped_bytes = (uint16_t)(context->dropped_bytes + context->length +
                                            length - MP157_RX_PARSER_BUFFER_SIZE);
        context->length = 0U;
        length = MP157_RX_PARSER_BUFFER_SIZE;
    }

    free_length = (uint16_t)(MP157_RX_PARSER_BUFFER_SIZE - context->length);
    if (length > free_length)
    {
        remove_length = (uint16_t)(length - free_length);
        Mp157RxParser_RemovePrefix(context, remove_length);
        context->dropped_bytes = (uint16_t)(context->dropped_bytes + remove_length);
    }

    (void)memcpy(&context->buffer[context->length], data, length);
    context->length = (uint16_t)(context->length + length);
}

/**
 * @brief 尝试提取一帧合法 MP157-F4 帧，并在坏帧后按最小步长重同步。
 * @param context 解析上下文。
 * @param out_raw_frame 原始帧输出缓存。
 * @param out_raw_frame_size 原始帧输出容量。
 * @param out_raw_frame_length 原始帧长度输出。
 * @param out_parsed_frame 解析字段输出。
 * @return Mp157RxParser_Result_t 本轮解析结果。
 */
Mp157RxParser_Result_t Mp157RxParser_TryExtractFrame(Mp157RxParser_Context_t *context,
                                                     uint8_t *out_raw_frame,
                                                     uint16_t out_raw_frame_size,
                                                     uint16_t *out_raw_frame_length,
                                                     BinaryProtocol_Frame_t *out_parsed_frame)
{
    uint16_t header_index;
    uint16_t expected_length;
    BinaryProtocol_ParseStatus_t parse_status;

    if ((context == NULL) ||
        (out_raw_frame == NULL) ||
        (out_raw_frame_length == NULL) ||
        (out_parsed_frame == NULL) ||
        (out_raw_frame_size < BINARY_PROTOCOL_MAX_FRAME_LENGTH))
    {
        return MP157_RX_PARSER_RESULT_NEED_MORE;
    }

    *out_raw_frame_length = 0U;

    if (context->length < 2U)
    {
        return MP157_RX_PARSER_RESULT_NEED_MORE;
    }

    header_index = 0U;
    while ((header_index + 1U) < context->length)
    {
        if ((context->buffer[header_index] == BINARY_PROTOCOL_SOF0) &&
            (context->buffer[header_index + 1U] == BINARY_PROTOCOL_SOF1))
        {
            break;
        }
        header_index++;
    }

    if (header_index > 0U)
    {
        Mp157RxParser_RemovePrefix(context, header_index);
        context->dropped_bytes = (uint16_t)(context->dropped_bytes + header_index);
        return MP157_RX_PARSER_RESULT_DROPPED_BYTES;
    }

    if ((context->buffer[0] != BINARY_PROTOCOL_SOF0) ||
        (context->buffer[1] != BINARY_PROTOCOL_SOF1))
    {
        if (context->buffer[context->length - 1U] == BINARY_PROTOCOL_SOF0)
        {
            context->buffer[0] = BINARY_PROTOCOL_SOF0;
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + context->length - 1U);
            context->length = 1U;
        }
        else
        {
            context->dropped_bytes = (uint16_t)(context->dropped_bytes + context->length);
            context->length = 0U;
        }
        return MP157_RX_PARSER_RESULT_DROPPED_BYTES;
    }

    if (context->length < 5U)
    {
        return MP157_RX_PARSER_RESULT_NEED_MORE;
    }

    if (context->buffer[4] > BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH)
    {
        Mp157RxParser_RemovePrefix(context, 1U);
        context->dropped_bytes++;
        return MP157_RX_PARSER_RESULT_DROPPED_BYTES;
    }

    expected_length = (uint16_t)(BINARY_PROTOCOL_MIN_FRAME_LENGTH + context->buffer[4]);
    if (context->length < expected_length)
    {
        return MP157_RX_PARSER_RESULT_NEED_MORE;
    }

    parse_status = BinaryProtocolService_ParseFrame(context->buffer,
                                                    expected_length,
                                                    out_parsed_frame);
    if (parse_status != BINARY_PROTOCOL_PARSE_OK)
    {
        Mp157RxParser_RemovePrefix(context, 1U);
        context->dropped_bytes++;
        return MP157_RX_PARSER_RESULT_DROPPED_BYTES;
    }

    (void)memcpy(out_raw_frame, context->buffer, expected_length);
    *out_raw_frame_length = expected_length;
    Mp157RxParser_RemovePrefix(context, expected_length);
    return MP157_RX_PARSER_RESULT_FRAME_READY;
}
