#ifndef USER_APP_MP157_RX_PARSER_H
#define USER_APP_MP157_RX_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> /* 提供固定宽度整数类型，保证协议缓存长度在 F4 与主机测试中一致。 */

#include "binary_protocol_service.h" /* 复用 MP157-F4 正式协议的帧长度、帧头和解析结构定义。 */

/**
 * @brief MP157 主链路字节流累积缓存容量，单位字节。
 *
 * 最大协议帧为 58 字节，192 字节可以同时容纳 3 个最大帧和少量错位噪声，
 * 用于覆盖 MP157 连续发送三次安全 STOP 以及 UART DMA-IDLE 合并多帧的现场情况。
 */
#define MP157_RX_PARSER_BUFFER_SIZE (192U)

/** @brief MP157 字节流解析结果。 */
typedef enum
{
    MP157_RX_PARSER_RESULT_NEED_MORE = 0, /* 当前缓存不足一帧，需要继续接收。 */
    MP157_RX_PARSER_RESULT_FRAME_READY,   /* 已提取一帧 CRC、长度和帧尾均正确的完整帧。 */
    MP157_RX_PARSER_RESULT_DROPPED_BYTES  /* 为了从噪声或坏帧恢复同步，本轮丢弃了部分字节。 */
} Mp157RxParser_Result_t;

/**
 * @brief MP157 主链路字节流解析上下文。
 *
 * 该结构只负责缓存、拆帧和重同步，不执行电机、称重或自动流程业务动作。
 */
typedef struct
{
    uint8_t buffer[MP157_RX_PARSER_BUFFER_SIZE]; /* 尚未消费的 MP157 主链路原始字节流。 */
    uint16_t length;                             /* buffer 中当前有效字节数。 */
    uint16_t dropped_bytes;                      /* 因溢出、噪声或坏帧累计丢弃的字节数。 */
} Mp157RxParser_Context_t;

/**
 * @brief 初始化 MP157 字节流解析上下文。
 * @param context 待初始化的上下文，不能为空。
 * @return 无返回值。
 */
void Mp157RxParser_Init(Mp157RxParser_Context_t *context);

/**
 * @brief 把新收到的 MP157 主链路字节追加到解析缓存。
 * @param context 解析上下文，不能为空。
 * @param data 新收到的数据首地址；length 为 0 时允许为空。
 * @param length 本次追加的字节数。
 * @return 无返回值；空间不足时优先丢弃最旧字节，保留最新控制命令。
 */
void Mp157RxParser_PushBytes(Mp157RxParser_Context_t *context,
                             const uint8_t *data,
                             uint16_t length);

/**
 * @brief 尝试从缓存中提取一帧完整合法协议帧。
 * @param context 解析上下文，不能为空。
 * @param out_raw_frame 完整原始帧输出缓存，不能为空。
 * @param out_raw_frame_size 输出缓存容量，至少应为 BINARY_PROTOCOL_MAX_FRAME_LENGTH。
 * @param out_raw_frame_length 实际帧长度输出，不能为空。
 * @param out_parsed_frame 解析后的字段输出，不能为空。
 * @return Mp157RxParser_Result_t 返回帧就绪、数据不足或已丢弃坏字节。
 */
Mp157RxParser_Result_t Mp157RxParser_TryExtractFrame(Mp157RxParser_Context_t *context,
                                                     uint8_t *out_raw_frame,
                                                     uint16_t out_raw_frame_size,
                                                     uint16_t *out_raw_frame_length,
                                                     BinaryProtocol_Frame_t *out_parsed_frame);

#ifdef __cplusplus
}
#endif

#endif
