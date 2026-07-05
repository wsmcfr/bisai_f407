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
 * 2. 初始化摄像头左右电机，现场默认电机地址为 `0x03`；
 * 3. 初始化摄像头上下电机，现场默认电机地址为 `0x02`；
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
 * 2. `CAMSTOP`：停止摄像头左右轴和上下轴；
 * 3. `CAMLAT LEFT [rpm]`：摄像头左右轴向左点动；
 * 4. `CAMLAT RIGHT [rpm]`：摄像头左右轴向右点动；
 *    `CAMFWD FORWARD/BACKWARD [rpm]` 只作为旧调试命令兼容入口，实际仍控制左右轴；
 * 5. `CAMZ UP [rpm]`：摄像头上下轴向上点动；
 * 6. `CAMZ DOWN [rpm]`：摄像头上下轴向下点动。
 */
uint8_t CameraMotorService_HandleCommand(const char *command_buffer);

/**
 * @brief 请求摄像头左右轴点动。
 * @param right_flag 1 表示按工程约定的右移方向点动，0 表示按工程约定的左移方向点动。
 * @param speed_rpm 点动转速，单位 RPM，传 0 时使用服务默认低速值。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestLateralJog(uint8_t right_flag, uint16_t speed_rpm);

/**
 * @brief 旧版前进/后退接口兼容包装。
 * @param forward_flag 旧命名参数，1 会映射为右移，0 会映射为左移。
 * @param speed_rpm 点动转速，单位 RPM，传 0 时使用服务默认低速值。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 *
 * 该函数只保留给旧调用点和临时调试代码，新的 MP157/F4 协议语义应使用左右轴接口。
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
 * @brief 请求摄像头左右轴按相对位置模式移动固定步数。
 * @param right_flag 1 表示按工程约定的右移方向移动，0 表示按工程约定的左移方向移动。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用该轴默认速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestLateralPosition(uint8_t right_flag,
                                                  uint16_t speed_rpm,
                                                  uint32_t pulse_count);

/**
 * @brief 请求摄像头左右轴位置运动，并在真实到位或超时时向 MP157 上报事件。
 * @param right_flag 1 表示按工程约定的右移方向移动，0 表示按工程约定的左移方向移动。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用该轴默认速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @param cycle_id 自动检测流程 ID，用于让 MP157 匹配本轮流程。
 * @param related_seq 原始 `ACTUATOR_POS_MOVE` 帧序号，用于让 MP157 匹配具体命令。
 * @param actuator 协议执行器编号，左右轴固定为 1。
 * @param direction 原始协议方向字段，按 MP157 下发值原样回填到完成事件。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或摄像头电机服务尚未就绪。
 *
 * 注意：协议 ACK 只说明请求已投递，真实物理完成必须等待
 * `EVENT_REPORT/ACTUATOR_MOVE_DONE`。
 */
uint8_t CameraMotorService_RequestLateralPositionWithReport(uint8_t right_flag,
                                                            uint16_t speed_rpm,
                                                            uint32_t pulse_count,
                                                            uint16_t cycle_id,
                                                            uint16_t related_seq,
                                                            uint8_t actuator,
                                                            uint8_t direction);

/**
 * @brief 旧版前进/后退位置接口兼容包装。
 * @param forward_flag 旧命名参数，1 会映射为右移，0 会映射为左移。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用该轴默认速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardPosition(uint8_t forward_flag,
                                                  uint16_t speed_rpm,
                                                  uint32_t pulse_count);

/**
 * @brief 请求摄像头上下轴按相对位置模式移动固定步数。
 * @param up_flag 1 表示按工程约定的向上方向移动，0 表示按工程约定的向下方向移动。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用该轴默认速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestZPosition(uint8_t up_flag,
                                            uint16_t speed_rpm,
                                            uint32_t pulse_count);

/**
 * @brief 请求摄像头上下轴位置运动，并在真实到位或超时时向 MP157 上报事件。
 * @param up_flag 1 表示按工程约定的上升方向移动，0 表示按工程约定的下降方向移动。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用该轴默认速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @param cycle_id 自动检测流程 ID，用于让 MP157 匹配本轮流程。
 * @param related_seq 原始 `ACTUATOR_POS_MOVE` 帧序号，用于让 MP157 匹配具体命令。
 * @param actuator 协议执行器编号，上下轴固定为 2。
 * @param direction 原始协议方向字段，按 MP157 下发值原样回填到完成事件。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或摄像头电机服务尚未就绪。
 *
 * MP157 自动流程必须等该接口对应的 DONE 事件后，才能认为 Z 轴下降或回升真正完成。
 */
uint8_t CameraMotorService_RequestZPositionWithReport(uint8_t up_flag,
                                                      uint16_t speed_rpm,
                                                      uint32_t pulse_count,
                                                      uint16_t cycle_id,
                                                      uint16_t related_seq,
                                                      uint8_t actuator,
                                                      uint8_t direction);

/**
 * @brief 请求摄像头左右轴把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 *
 * 该接口供二进制 `ACTUATOR_HOME actuator=1` 使用。
 * F4 会先停止目标轴，再发送 Emm42 当前位置清零命令，不会让电机主动运动。
 */
uint8_t CameraMotorService_RequestLateralSetCurrentPositionZero(void);

/**
 * @brief 旧版前进/后退设零接口兼容包装。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardSetCurrentPositionZero(void);

/**
 * @brief 请求摄像头上下轴把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 *
 * 该接口供二进制 `ACTUATOR_HOME actuator=2` 使用。
 * F4 会先停止目标轴，再发送 Emm42 当前位置清零命令，不会让电机主动运动。
 */
uint8_t CameraMotorService_RequestZSetCurrentPositionZero(void);

/**
 * @brief 请求摄像头两个运动轴立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机服务尚未就绪。
 */
uint8_t CameraMotorService_RequestStopAll(void);

/**
 * @brief 更新两个摄像头运动轴的运行时参数。
 * @param lateral_address 左右轴 Emm42 地址，允许 1~247。
 * @param lateral_min_step 左右轴最小步长，单位 step，当前保存给后续位置步进命令使用。
 * @param lateral_normal_speed_rpm 左右轴默认点动速度，单位 RPM，允许 0~5000。
 * @param lateral_direction 左右轴方向映射，1 表示保持逻辑方向，-1 表示反转逻辑方向。
 * @param z_address 上下轴 Emm42 地址，允许 1~247，且不能和左右轴相同。
 * @param z_min_step 上下轴最小步长，单位 step，当前保存给后续位置步进命令使用。
 * @param z_normal_speed_rpm 上下轴默认点动速度，单位 RPM，允许 0~5000。
 * @param z_direction 上下轴方向映射，1 表示保持逻辑方向，-1 表示反转逻辑方向。
 * @return uint8_t 1 表示配置请求已投递，0 表示参数非法或摄像头电机任务尚未就绪。
 *
 * 该接口由 MP157 二进制 `STEPPER_PARAM_SET` 调用，只更新 F4 运行内存参数；
 * 当前不写 F4 Flash，也不改 Emm42 驱动器自身 EEPROM。
 */
uint8_t CameraMotorService_RequestRuntimeConfig(uint8_t lateral_address,
                                                uint16_t lateral_min_step,
                                                uint16_t lateral_normal_speed_rpm,
                                                int8_t lateral_direction,
                                                uint8_t z_address,
                                                uint16_t z_min_step,
                                                uint16_t z_normal_speed_rpm,
                                                int8_t z_direction);

#ifdef __cplusplus
}
#endif

#endif
