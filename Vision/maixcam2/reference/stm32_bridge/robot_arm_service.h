#ifndef USER_APP_ROBOT_ARM_SERVICE_H
#define USER_APP_ROBOT_ARM_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 机械臂协议帧最大缓存长度，单位为字节。
 *
 * 当前 LeArm 动作组运行帧只有 7 字节，例如 `55 55 05 06 03 01 00`。
 * 这里预留到 64 字节，和 USART1 命令接收缓存保持一致，便于后续扩展坐标控制或更长参数帧。
 */
#define ROBOT_ARM_SERVICE_FRAME_MAX_SIZE (64U)

/**
 * @brief 机械臂待发送帧缓存结构。
 *
 * 该结构体只在任务队列中传递，避免 USART1 接收任务直接阻塞等待 USART3 发送完成。
 */
typedef struct
{
    uint8_t data[ROBOT_ARM_SERVICE_FRAME_MAX_SIZE]; /* 待转发到 ESP32 的原始机械臂协议字节，保持上位机/MP157 发来的顺序不变。 */
    uint16_t length;                                /* data 中有效字节数量，发送到 USART3 时只发送该长度范围内的数据。 */
} RobotArmService_Frame_t;

/**
 * @brief 初始化机械臂服务内部队列。
 * @return uint8_t 1 表示初始化成功，0 表示队列创建失败。
 *
 * 该函数应在 FreeRTOS 任务创建阶段调用一次，确保后续 USART1 收到机械臂帧时可以投递给机械臂任务。
 */
uint8_t RobotArmService_Init(void);

/**
 * @brief 判断一段原始串口数据是否符合 LeArm 上位机协议帧格式。
 * @param frame_buffer 原始串口数据缓存，不能为 NULL。
 * @param frame_length 原始串口数据长度，单位为字节。
 * @return uint8_t 1 表示帧头和长度字段合法，0 表示不是一帧完整机械臂协议。
 *
 * 当前使用的动作组协议格式为：`55 55 Length CMD Params...`，
 * 其中总字节数应等于 `Length + 2`，例如 `55 55 05 06 03 01 00` 的总长度为 7。
 */
uint8_t RobotArmService_IsProtocolFrame(const uint8_t *frame_buffer, uint16_t frame_length);

/**
 * @brief 处理一帧来自 USART1 的原始数据，若识别为机械臂协议则投递到 USART3 转发任务。
 * @param frame_buffer 原始串口数据缓存，不能为 NULL。
 * @param frame_length 原始串口数据长度，单位为字节。
 * @return uint8_t 1 表示该帧已经被机械臂服务接管，0 表示该帧不是机械臂协议帧。
 *
 * 该函数由 USART1 的唯一命令消费者调用，避免机械臂服务自己再抢 USART1 接收缓存。
 */
uint8_t RobotArmService_HandleFrame(const uint8_t *frame_buffer, uint16_t frame_length);

/**
 * @brief 机械臂服务任务入口，负责把队列中的机械臂协议帧通过 USART3 发送给 ESP32。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 主要流程：
 * 1. 阻塞等待 USART1 命令分发层投递机械臂协议帧；
 * 2. 从队列中取出完整帧；
 * 3. 使用 USART3 以 115200 8N1 原样发送到 ESP32，波特率必须和 ESP32 PA5/PA4 串口保持一致；
 * 4. 发送失败时丢弃当前帧，等待下一帧，避免任务卡死。
 */
void RobotArmService_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
