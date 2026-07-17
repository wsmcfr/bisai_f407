#include "mp157_rx_parser.h" /* 引入待验证的 MP157 字节流解析器接口。 */

#include <stdio.h>  /* 提供 printf，输出主机侧测试结果。 */
#include <string.h> /* 提供 memset 和 memcpy，构造连续串口字节流。 */

/** @brief 记录测试失败数量，主函数据此返回非零退出码。 */
static int g_mp157_rx_test_failed_count = 0;

/**
 * @brief 比较无符号整数测试结果。
 * @param name 当前断言名称，用于失败时定位具体测试点。
 * @param actual 实际值。
 * @param expected 期望值。
 * @return 无返回值；失败时累计全局失败计数。
 */
static void Mp157RxTest_ExpectU32(const char *name, unsigned int actual, unsigned int expected)
{
    if (actual != expected)
    {
        printf("FAIL: %s actual=%u expected=%u\n", name, actual, expected);
        g_mp157_rx_test_failed_count++;
    }
}

/**
 * @brief 构造一帧无负载的合法 MP157-F4 二进制帧。
 * @param command 命令字。
 * @param sequence 帧序号。
 * @param output 输出缓存。
 * @param output_size 输出缓存容量。
 * @return uint16_t 成功返回完整帧长度，参数非法返回 0。
 */
static uint16_t Mp157RxTest_BuildEmptyFrame(uint8_t command,
                                            uint16_t sequence,
                                            uint8_t *output,
                                            uint16_t output_size)
{
    uint16_t crc;

    if ((output == NULL) || (output_size < BINARY_PROTOCOL_MIN_FRAME_LENGTH))
    {
        return 0U;
    }

    output[0] = BINARY_PROTOCOL_SOF0;
    output[1] = BINARY_PROTOCOL_SOF1;
    output[2] = BINARY_PROTOCOL_VERSION;
    output[3] = command;
    output[4] = 0U;
    output[5] = (uint8_t)(sequence & 0xFFU);
    output[6] = (uint8_t)((sequence >> 8U) & 0xFFU);
    crc = BinaryProtocolService_Crc16CcittFalse(&output[2], 5U);
    output[7] = (uint8_t)(crc & 0xFFU);
    output[8] = (uint8_t)((crc >> 8U) & 0xFFU);
    output[9] = BINARY_PROTOCOL_EOF;
    return BINARY_PROTOCOL_MIN_FRAME_LENGTH;
}

/**
 * @brief 验证一次 DMA-IDLE 数据块中连续三帧都能逐帧提取。
 *
 * 该场景直接复现 MP157 为提高安全性连续发送三次 STOP 时，
 * F4 可能一次收到 30~42 字节并把整块误当成单帧的问题。
 */
static void Mp157RxTest_ShouldExtractThreeCoalescedFrames(void)
{
    Mp157RxParser_Context_t context;
    BinaryProtocol_Frame_t frame;
    uint8_t stream[3U * BINARY_PROTOCOL_MIN_FRAME_LENGTH];
    uint8_t single_frame[BINARY_PROTOCOL_MIN_FRAME_LENGTH];
    uint8_t extracted_raw_frame[BINARY_PROTOCOL_MAX_FRAME_LENGTH];
    uint16_t extracted_raw_frame_length = 0U;
    uint16_t frame_length;
    uint16_t index;

    (void)memset(&context, 0, sizeof(context));
    (void)memset(&frame, 0, sizeof(frame));
    (void)memset(stream, 0, sizeof(stream));
    Mp157RxParser_Init(&context);

    for (index = 0U; index < 3U; index++)
    {
        frame_length = Mp157RxTest_BuildEmptyFrame(0x44U,
                                                   (uint16_t)(0x1200U + index),
                                                   single_frame,
                                                   (uint16_t)sizeof(single_frame));
        (void)memcpy(&stream[index * frame_length], single_frame, frame_length);
    }

    Mp157RxParser_PushBytes(&context, stream, (uint16_t)sizeof(stream));

    for (index = 0U; index < 3U; index++)
    {
        Mp157RxTest_ExpectU32("coalesced result",
                              (unsigned int)Mp157RxParser_TryExtractFrame(&context,
                                                                         extracted_raw_frame,
                                                                         (uint16_t)sizeof(extracted_raw_frame),
                                                                         &extracted_raw_frame_length,
                                                                         &frame),
                              (unsigned int)MP157_RX_PARSER_RESULT_FRAME_READY);
        Mp157RxTest_ExpectU32("coalesced command", frame.command, 0x44U);
        Mp157RxTest_ExpectU32("coalesced sequence", frame.sequence, (unsigned int)(0x1200U + index));
    }

    Mp157RxTest_ExpectU32("coalesced drained",
                          (unsigned int)Mp157RxParser_TryExtractFrame(&context,
                                                                     extracted_raw_frame,
                                                                     (uint16_t)sizeof(extracted_raw_frame),
                                                                     &extracted_raw_frame_length,
                                                                     &frame),
                          (unsigned int)MP157_RX_PARSER_RESULT_NEED_MORE);
}

/**
 * @brief 验证半帧分两次到达时不会被误丢弃，并能在后半段到达后成功提取。
 */
static void Mp157RxTest_ShouldKeepPartialFrameUntilCompleted(void)
{
    Mp157RxParser_Context_t context;
    BinaryProtocol_Frame_t frame;
    uint8_t complete_frame[BINARY_PROTOCOL_MIN_FRAME_LENGTH];
    uint8_t raw_frame[BINARY_PROTOCOL_MAX_FRAME_LENGTH];
    uint16_t raw_frame_length = 0U;
    uint16_t frame_length;

    (void)memset(&context, 0, sizeof(context));
    (void)memset(&frame, 0, sizeof(frame));
    Mp157RxParser_Init(&context);
    frame_length = Mp157RxTest_BuildEmptyFrame(0x44U,
                                               0x3344U,
                                               complete_frame,
                                               (uint16_t)sizeof(complete_frame));

    Mp157RxParser_PushBytes(&context, complete_frame, 4U);
    Mp157RxTest_ExpectU32("partial need more",
                          (unsigned int)Mp157RxParser_TryExtractFrame(&context,
                                                                     raw_frame,
                                                                     (uint16_t)sizeof(raw_frame),
                                                                     &raw_frame_length,
                                                                     &frame),
                          (unsigned int)MP157_RX_PARSER_RESULT_NEED_MORE);

    Mp157RxParser_PushBytes(&context, &complete_frame[4], (uint16_t)(frame_length - 4U));
    Mp157RxTest_ExpectU32("partial ready",
                          (unsigned int)Mp157RxParser_TryExtractFrame(&context,
                                                                     raw_frame,
                                                                     (uint16_t)sizeof(raw_frame),
                                                                     &raw_frame_length,
                                                                     &frame),
                          (unsigned int)MP157_RX_PARSER_RESULT_FRAME_READY);
    Mp157RxTest_ExpectU32("partial sequence", frame.sequence, 0x3344U);
}

/**
 * @brief 主机侧回归测试入口。
 * @return int 全部通过返回 0，存在失败返回 1。
 */
int main(void)
{
    Mp157RxTest_ShouldExtractThreeCoalescedFrames();
    Mp157RxTest_ShouldKeepPartialFrameUntilCompleted();

    if (g_mp157_rx_test_failed_count != 0)
    {
        printf("mp157 rx parser host tests failed: %d\n", g_mp157_rx_test_failed_count);
        return 1;
    }

    printf("mp157 rx parser host tests passed\n");
    return 0;
}
