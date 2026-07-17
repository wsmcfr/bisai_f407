#ifndef USER_APP_ROBOT_ARM_SERVICE_H
#define USER_APP_ROBOT_ARM_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 机械臂正式协议帧最大缓存长度，单位字节。
 *
 * F4-ESP32S3 正式协议复用 `A5 5A VER CMD LEN SEQ PAYLOAD CRC 6B` 帧格式，
 * 当前最大 payload 为 48 字节，因此完整帧最大为 58 字节。
 * 这里保留 64 字节，与 USART2 原始命令缓存对齐，方便后续扩展。
 */
#define ROBOT_ARM_SERVICE_FRAME_MAX_SIZE (64U)

/**
 * @brief 初始化机械臂服务内部发送队列。
 * @return uint8_t 1 表示队列可用，0 表示队列创建失败。
 *
 * 该函数应在 FreeRTOS 任务创建阶段调用一次。
 * 如果任务入口再次调用，函数会检测已有队列并直接返回成功。
 */
uint8_t RobotArmService_Init(void);

/**
 * @brief 请求 ESP32S3 机械臂把 ROI/传送带上的零件抓取并放到称重模块。
 * @param cycle_id 当前 MP157-F4 自动检测流程号。
 * @param job_id MP157 分配的机械臂任务号。
 * @param part_type 零件类型，未知填 0。
 * @param model_result MP157 模型结果，0=未知，1=良品，2=不良品，3=待复核。
 * @return uint8_t 1 表示正式协议命令已进入发送队列，0 表示队列或参数异常。
 *
 * 发送任务会先等待 ESP32S3 `ARM_ACK`，再等待 `ARM_STAGE_DONE stage=1`。
 * 如果 `robot_arm_service.c` 里把正式流程宏关闭用于 USART3 回环自检，
 * 则本接口不会真正入队，只会打印“当前处于 loopback 模式”并返回 0。
 */
uint8_t RobotArmService_RequestPlaceWeight(uint16_t cycle_id,
                                           uint16_t job_id,
                                           uint8_t part_type,
                                           uint8_t model_result);

/**
 * @brief 请求 ESP32S3 机械臂把零件从称重模块搬到电磁感应模块。
 * @param cycle_id 当前 MP157-F4 自动检测流程号。
 * @param job_id MP157 分配的机械臂任务号。
 * @param part_type 零件类型，未知填 0。
 * @param model_result MP157 模型结果，供 ESP32S3 选择动作细节。
 * @return uint8_t 1 表示正式协议命令已进入发送队列，0 表示队列或参数异常。
 *
 * 发送任务会先等待 ESP32S3 `ARM_ACK`，再等待 `ARM_STAGE_DONE stage=2`。
 * 如果 `robot_arm_service.c` 里把正式流程宏关闭用于 USART3 回环自检，
 * 则本接口不会真正入队，只会打印“当前处于 loopback 模式”并返回 0。
 */
uint8_t RobotArmService_RequestPlaceLdc(uint16_t cycle_id,
                                        uint16_t job_id,
                                        uint8_t part_type,
                                        uint8_t model_result);

/**
 * @brief 请求 ESP32S3 机械臂把零件从电磁感应模块搬到最终分拣区。
 * @param cycle_id 当前 MP157-F4 自动检测流程号。
 * @param job_id MP157 分配的机械臂任务号。
 * @param part_type 零件类型，未知填 0。
 * @param model_result MP157 最终模型/云端综合结果。
 * @param final_bin 最终分拣目标，1=良品，2=不良品，3=待复核。
 * @return uint8_t 1 表示正式协议命令已进入发送队列，0 表示队列或参数异常。
 *
 * 发送任务会先等待 ESP32S3 `ARM_ACK`，再等待 `ARM_STAGE_DONE stage=3`。
 * 如果 `robot_arm_service.c` 里把正式流程宏关闭用于 USART3 回环自检，
 * 则本接口不会真正入队，只会打印“当前处于 loopback 模式”并返回 0。
 */
uint8_t RobotArmService_RequestFinalSort(uint16_t cycle_id,
                                         uint16_t job_id,
                                         uint8_t part_type,
                                         uint8_t model_result,
                                         uint8_t final_bin);

/**
 * @brief 机械臂服务任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 主要流程：
 * 1. 初始化内部队列；
 * 2. 启动后发送正式协议 `ARM_LINK_HELLO` 和 `ARM_LINK_HEARTBEAT`；
 * 3. 阻塞等待业务命令入队；
 * 4. 通过 USART3 发送正式二进制帧；
 * 5. 动作命令先等 ACK，再等 DONE，并把 DONE 交给 MP157-F4 主状态机。
 *
 * 如果 `robot_arm_service.c` 里关闭了正式流程宏，
 * 则该任务不会进入自动流程，而是周期性执行 USART3 TX->RX 回环自检，
 * 并把结果打印到 UART1，供现场先确认 PD9 是否真的能收到数据。
 */
void RobotArmService_Task(void *argument);

/**
 * @brief USART3 DMA+空闲中断接收事件回调（从ISR中调用）。
 * @param size 本次DMA传输的实际字节数。
 *
 * 由 HAL_UARTEx_RxEventCallback 在 huart==&huart3 时调用，
 * 负责把DMA缓冲区中收到的字节压入内部环形缓冲区并唤醒等待任务。
 */
void RobotArmService_RxEventFromISR(uint16_t size);

/**
 * @brief USART3 接收错误后的恢复入口（从 HAL 错误回调中调用）。
 *
 * 当 USART3 在 DMA+空闲接收过程中出现 ORE/FE/NE/PE 等错误时，
 * HAL 会中止当前接收流程；该函数负责清除 USART3 错误标志、
 * 丢弃可能已经不完整的接收缓存，并重新挂起下一轮 DMA+空闲中断接收。
 * 该函数运行在中断上下文，内部只做状态复位和接收重启，不做日志输出。
 */
void RobotArmService_RecoverRxFromISR(void);

#ifdef __cplusplus
}
#endif

#endif
