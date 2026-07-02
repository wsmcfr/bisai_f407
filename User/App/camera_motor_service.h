#ifndef USER_APP_CAMERA_MOTOR_SERVICE_H
#define USER_APP_CAMERA_MOTOR_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 摄像头运动 Emm42 电机服务任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 独占 `USART6` 作为摄像头运动 Emm42 TTL 总线，PC6(TX) 接两个电机 RX，PC7(RX) 接两个电机 TX；
 * 2. 初始化摄像头前进/后退电机，电机地址固定为 `0x02`；
 * 3. 初始化摄像头上下电机，电机地址固定为 `0x03`；
 * 4. 串行执行来自 USART1 文本调试命令或后续二进制协议层的点动/停止请求；
 * 5. 确保同一条 `USART6` 总线上不会有两个任务同时发送 Emm42 命令。
 */
void CameraMotorService_Task(void *argument);

/**
 * @brief 处理一条发给摄像头运动电机服务的 USART1 文本命令。
 * @param command_buffer 已经规范化后的命令字符串，不能为空。
 * @return uint8_t 1 表示该命令已由摄像头电机服务处理，0 表示不是本模块命令。
 *
 * 当前支持的调试命令：
 * 1. `CAMINFO`：查询摄像头两个电机的地址、串口和最近动作；
 * 2. `CAMSTOP`：停止摄像头前进/后退轴和上下轴；
 * 3. `CAMFWD FORWARD [rpm]`：摄像头前进/后退轴正向点动；
 * 4. `CAMFWD BACKWARD [rpm]`：摄像头前进/后退轴反向点动；
 * 5. `CAMZ UP [rpm]`：摄像头上下轴向上点动；
 * 6. `CAMZ DOWN [rpm]`：摄像头上下轴向下点动。
 */
uint8_t CameraMotorService_HandleCommand(const char *command_buffer);

/**
 * @brief 请求摄像头前进/后退轴点动。
 * @param forward_flag 1 表示按工程约定的前进方向点动，0 表示按工程约定的后退方向点动。
 * @param speed_rpm 点动转速，单位 RPM，传 0 时使用服务默认低速值。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardJog(uint8_t forward_flag, uint16_t speed_rpm);

/**
 * @brief 请求摄像头上下轴点动。
 * @param up_flag 1 表示按工程约定的向上方向点动，0 表示按工程约定的向下方向点动。
 * @param speed_rpm 点动转速，单位 RPM，传 0 时使用服务默认低速值。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestZJog(uint8_t up_flag, uint16_t speed_rpm);

/**
 * @brief 请求摄像头两个运动轴立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestStopAll(void);

#ifdef __cplusplus
}
#endif

#endif
