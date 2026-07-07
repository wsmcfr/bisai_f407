#ifndef USER_APP_ROBOT_ARM_RX_PARSER_H
#define USER_APP_ROBOT_ARM_RX_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>                 /* 提供 uint8_t/uint16_t 等固定宽度类型，确保协议缓存长度稳定。 */
#include "binary_protocol_service.h"/* 复用正式协议帧定义、最小长度、最大负载和解析结果类型。 */

/**
 * @brief 机械臂串口接收缓存总容量，单位字节。
 *
 * 设计原因：
 * 1. 单帧最大长度当前为 58 字节；
 * 2. 接收侧既可能先积累一个坏帧，又可能紧跟一个好帧；
 * 3. 再加上一些前导噪声和重叠帧头，128 字节足够覆盖当前现场联调场景，
 *    同时不会让 F4 RAM 占用明显上升。
 */
#define ROBOT_ARM_RX_PARSER_BUFFER_SIZE       (128U)

/**
 * @brief 对外导出的完整原始帧缓存长度，单位字节。
 *
 * 该值与 `robot_arm_service.h` 当前对机械臂完整帧的最大缓存保持一致，
 * 主机侧测试和 F4 运行时都可以直接复用，不必为“原始帧复制”再开更大的局部数组。
 */
#define ROBOT_ARM_RX_PARSER_FRAME_BUFFER_SIZE (BINARY_PROTOCOL_MAX_FRAME_LENGTH)

/**
 * @brief 串口接收缓存解析结果。
 *
 * `robot_arm_service.c` 会根据该状态决定：
 * - 当前是否已经拿到一帧可交给 ACK/DONE 逻辑处理的合法协议帧；
 * - 只是暂时数据不够，应该继续等；
 * - 还是本轮为了重同步丢掉了部分噪声/坏数据。
 */
typedef enum
{
    ROBOT_ARM_RX_PARSER_RESULT_NEED_MORE = 0, /* 当前缓存还不足以组成一帧合法协议帧，需要继续接收更多字节。 */
    ROBOT_ARM_RX_PARSER_RESULT_FRAME_READY,    /* 已成功提取一帧合法协议帧。 */
    ROBOT_ARM_RX_PARSER_RESULT_DROPPED_BYTES   /* 本轮为了重同步丢弃了噪声或坏帧字节，上层应继续尝试提取下一帧。 */
} RobotArmRxParser_Result_t;

/**
 * @brief 机械臂串口接收缓存上下文。
 *
 * 本结构只负责“字节流 -> 合法协议帧”的缓存与重同步，
 * 不直接关心 ACK、DONE、FAULT 的业务语义。
 */
typedef struct
{
    uint8_t buffer[ROBOT_ARM_RX_PARSER_BUFFER_SIZE]; /* 尚未被消费的原始串口字节流缓存。 */
    uint16_t length;                                 /* 当前缓存中有效字节数。 */
    uint16_t dropped_bytes;                          /* 累计因噪声/坏帧/溢出被丢弃的字节数，供日志观察同步质量。 */
} RobotArmRxParser_Context_t;

/**
 * @brief 初始化接收缓存上下文。
 * @param context 待初始化的上下文，不能为空。
 *
 * 作用：
 * 1. 清空尚未解析的字节；
 * 2. 清零累计丢弃计数；
 * 3. 让同一结构可在 F4 上电初始化和错误恢复后重复复用。
 */
void RobotArmRxParser_Init(RobotArmRxParser_Context_t *context);

/**
 * @brief 向接收缓存压入一段新收到的串口字节。
 * @param context 接收缓存上下文，不能为空。
 * @param data 新收到的字节流首地址，允许为 NULL，但此时 `length` 必须为 0。
 * @param length 本次要压入的字节数。
 *
 * 设计说明：
 * - 如果新数据太多导致缓存装不下，会优先丢弃最旧的前导字节，保证最新字节可进入缓存；
 * - 这样即使现场出现少量长噪声，也不会把后面真正的 ACK/DONE 完整挡在缓存外。
 */
void RobotArmRxParser_PushBytes(RobotArmRxParser_Context_t *context,
                                const uint8_t *data,
                                uint16_t length);

/**
 * @brief 尝试从当前缓存中提取一帧合法协议帧。
 * @param context 接收缓存上下文，不能为空。
 * @param out_raw_frame 输出原始完整帧缓存，允许为 NULL；若不为 NULL，长度至少应覆盖最大帧长。
 * @param out_raw_frame_size `out_raw_frame` 的容量。
 * @param out_raw_frame_length 输出实际原始帧长度，允许为 NULL。
 * @param out_parsed_frame 输出解析后的协议帧结构，允许为 NULL。
 * @return RobotArmRxParser_Result_t 提取结果。
 *
 * 关键行为：
 * 1. 能跳过前导噪声；
 * 2. 遇到 `A5 A5 5A` 这类重叠帧头时，会把第二个 `A5` 当成新的候选帧头继续同步；
 * 3. CRC、帧尾、长度错误时，不会把整轮等待直接判死，而是丢弃最少必要字节后继续找下一帧；
 * 4. 只有真正拿到一帧 `ParseFrame()==OK` 的合法帧，才返回 `FRAME_READY`。
 */
RobotArmRxParser_Result_t RobotArmRxParser_TryExtractFrame(RobotArmRxParser_Context_t *context,
                                                           uint8_t *out_raw_frame,
                                                           uint16_t out_raw_frame_size,
                                                           uint16_t *out_raw_frame_length,
                                                           BinaryProtocol_Frame_t *out_parsed_frame);

#ifdef __cplusplus
}
#endif

#endif
