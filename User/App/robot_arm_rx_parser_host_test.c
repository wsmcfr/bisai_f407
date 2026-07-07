#include "binary_protocol_service.h" /* 复用正式二进制协议的组帧/解帧常量与 CRC，实现和 F4 运行时一致。 */
#include "robot_arm_rx_parser.h"     /* 待实现的机械臂串口接收缓存/重同步模块公开接口。 */

#include <stdio.h>   /* 主机侧测试使用 printf 输出最终通过/失败信息。 */
#include <string.h>  /* 主机侧测试使用 memset/memcpy/memcmp 组装输入数据。 */

/**
 * @brief 记录当前主机侧测试累计失败次数。
 *
 * 每个断言失败时只递增计数，不立刻退出，便于一次运行看全所有失败点。
 */
static int g_robot_arm_rx_test_failed_count = 0;

/**
 * @brief 断言两个无符号整数相等。
 * @param label 当前断言的业务标签，便于失败时快速定位是哪一步不符合预期。
 * @param actual 实际值。
 * @param expected 预期值。
 *
 * 如果断言失败，就打印详细值并累计失败次数。
 */
static void RobotArmRxTest_ExpectU32(const char *label, unsigned int actual, unsigned int expected)
{
    if (actual != expected)
    {
        ++g_robot_arm_rx_test_failed_count;
        printf("[FAIL] %s: actual=%u expected=%u\n", label, actual, expected);
    }
}

/**
 * @brief 断言一段字节流完全相等。
 * @param label 当前断言标签。
 * @param actual 实际字节流首地址。
 * @param expected 预期字节流首地址。
 * @param length 需要比较的字节数。
 *
 * 如果发现任一字节不一致，就打印第一个不一致的位置。
 */
static void RobotArmRxTest_ExpectBytes(const char *label,
                                       const uint8_t *actual,
                                       const uint8_t *expected,
                                       uint16_t length)
{
    uint16_t index;

    if ((actual == NULL) || (expected == NULL))
    {
        ++g_robot_arm_rx_test_failed_count;
        printf("[FAIL] %s: NULL buffer\n", label);
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        if (actual[index] != expected[index])
        {
            ++g_robot_arm_rx_test_failed_count;
            printf("[FAIL] %s: byte[%u]=0x%02X expected=0x%02X\n",
                   label,
                   (unsigned int)index,
                   (unsigned int)actual[index],
                   (unsigned int)expected[index]);
            return;
        }
    }
}

/**
 * @brief 构造一帧空 payload 正式协议帧。
 * @param command 需要写入的命令字。
 * @param sequence 需要写入的小端序帧序号。
 * @param out_frame 输出帧缓存。
 * @param out_size 输出缓存容量。
 * @return uint16_t 实际帧长度；返回 0 表示组帧失败。
 *
 * 主机侧测试只需要验证接收缓存行为，因此这里统一用空 payload 帧降低输入样本复杂度。
 */
static uint16_t RobotArmRxTest_BuildEmptyFrame(uint8_t command,
                                               uint16_t sequence,
                                               uint8_t *out_frame,
                                               uint16_t out_size)
{
    return BinaryProtocolService_BuildFrame(command,
                                            sequence,
                                            (const uint8_t *)0,
                                            0U,
                                            out_frame,
                                            out_size);
}

/**
 * @brief 验证“坏帧后仍能继续提取下一帧”。
 *
 * 复现场景：
 * 1. 输入一帧 CRC 被故意破坏的 ACK；
 * 2. 紧跟一帧完整合法的 STAGE_DONE；
 * 3. 解析器必须跳过坏帧并继续交出后面的合法帧，而不是整轮直接判死。
 */
static void RobotArmRxTest_ShouldRecoverAfterBadFrame(void)
{
    RobotArmRxParser_Context_t context;
    BinaryProtocol_Frame_t parsed_frame;
    uint8_t raw_frame[ROBOT_ARM_RX_PARSER_FRAME_BUFFER_SIZE];
    uint16_t raw_length = 0U;
    uint8_t bad_frame[32];
    uint8_t good_frame[32];
    uint8_t input_stream[64];
    uint16_t bad_length;
    uint16_t good_length;
    RobotArmRxParser_Result_t result;

    (void)memset(&context, 0, sizeof(context));
    (void)memset(&parsed_frame, 0, sizeof(parsed_frame));
    (void)memset(raw_frame, 0, sizeof(raw_frame));
    (void)memset(input_stream, 0, sizeof(input_stream));

    RobotArmRxParser_Init(&context);

    bad_length = RobotArmRxTest_BuildEmptyFrame(0x80U, 0x1234U, bad_frame, (uint16_t)sizeof(bad_frame));
    good_length = RobotArmRxTest_BuildEmptyFrame(0x30U, 0x5678U, good_frame, (uint16_t)sizeof(good_frame));

    RobotArmRxTest_ExpectU32("bad frame build", bad_length > 0U ? 1U : 0U, 1U);
    RobotArmRxTest_ExpectU32("good frame build", good_length > 0U ? 1U : 0U, 1U);
    if ((bad_length == 0U) || (good_length == 0U))
    {
        return;
    }

    bad_frame[bad_length - 3U] ^= 0x5AU;

    (void)memcpy(input_stream, bad_frame, bad_length);
    (void)memcpy(&input_stream[bad_length], good_frame, good_length);

    RobotArmRxParser_PushBytes(&context, input_stream, (uint16_t)(bad_length + good_length));
    result = RobotArmRxParser_TryExtractFrame(&context,
                                              raw_frame,
                                              (uint16_t)sizeof(raw_frame),
                                              &raw_length,
                                              &parsed_frame);

    RobotArmRxTest_ExpectU32("recover result", (unsigned int)result, (unsigned int)ROBOT_ARM_RX_PARSER_RESULT_FRAME_READY);
    RobotArmRxTest_ExpectU32("recover command", parsed_frame.command, 0x30U);
    RobotArmRxTest_ExpectU32("recover sequence", parsed_frame.sequence, 0x5678U);
    RobotArmRxTest_ExpectU32("recover raw length", raw_length, good_length);
    RobotArmRxTest_ExpectBytes("recover raw bytes", raw_frame, good_frame, good_length);
}

/**
 * @brief 验证“前导噪声与重叠帧头不会让解析器卡死”。
 *
 * 复现场景：
 * 1. 串口前面先出现若干噪声字节；
 * 2. 再出现 `A5 A5 5A ...` 这种常见错位头；
 * 3. 解析器应把第二个 `A5` 当成新的候选帧头，最终成功交付合法帧。
 */
static void RobotArmRxTest_ShouldResyncAfterNoiseAndOverlappedHeader(void)
{
    RobotArmRxParser_Context_t context;
    BinaryProtocol_Frame_t parsed_frame;
    uint8_t raw_frame[ROBOT_ARM_RX_PARSER_FRAME_BUFFER_SIZE];
    uint16_t raw_length = 0U;
    uint8_t good_frame[32];
    uint8_t input_stream[64];
    uint16_t good_length;
    RobotArmRxParser_Result_t result;

    (void)memset(&context, 0, sizeof(context));
    (void)memset(&parsed_frame, 0, sizeof(parsed_frame));
    (void)memset(raw_frame, 0, sizeof(raw_frame));
    (void)memset(input_stream, 0, sizeof(input_stream));

    RobotArmRxParser_Init(&context);

    good_length = RobotArmRxTest_BuildEmptyFrame(0x81U, 0x3344U, good_frame, (uint16_t)sizeof(good_frame));
    RobotArmRxTest_ExpectU32("resync frame build", good_length > 0U ? 1U : 0U, 1U);
    if (good_length == 0U)
    {
        return;
    }

    input_stream[0] = 0xFFU;
    input_stream[1] = 0x00U;
    input_stream[2] = 0xA5U;
    (void)memcpy(&input_stream[3], good_frame, good_length);

    RobotArmRxParser_PushBytes(&context, input_stream, (uint16_t)(good_length + 3U));
    result = RobotArmRxParser_TryExtractFrame(&context,
                                              raw_frame,
                                              (uint16_t)sizeof(raw_frame),
                                              &raw_length,
                                              &parsed_frame);

    RobotArmRxTest_ExpectU32("resync result", (unsigned int)result, (unsigned int)ROBOT_ARM_RX_PARSER_RESULT_FRAME_READY);
    RobotArmRxTest_ExpectU32("resync command", parsed_frame.command, 0x81U);
    RobotArmRxTest_ExpectU32("resync sequence", parsed_frame.sequence, 0x3344U);
    RobotArmRxTest_ExpectU32("resync raw length", raw_length, good_length);
    RobotArmRxTest_ExpectBytes("resync raw bytes", raw_frame, good_frame, good_length);
}

/**
 * @brief 主机侧测试入口。
 * @return int 0 表示全部通过；1 表示至少存在一项失败。
 */
int main(void)
{
    RobotArmRxTest_ShouldRecoverAfterBadFrame();
    RobotArmRxTest_ShouldResyncAfterNoiseAndOverlappedHeader();

    if (g_robot_arm_rx_test_failed_count != 0)
    {
        printf("robot arm rx parser host tests failed: %d\n", g_robot_arm_rx_test_failed_count);
        return 1;
    }

    printf("robot arm rx parser host tests passed\n");
    return 0;
}
