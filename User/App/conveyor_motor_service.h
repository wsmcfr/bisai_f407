#ifndef USER_APP_CONVEYOR_MOTOR_SERVICE_H
#define USER_APP_CONVEYOR_MOTOR_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 传送带服务对外状态快照。
 *
 * 这个结构体专门给二进制协议 `STATUS_REPORT` 使用，
 * 字段全部使用基础整数类型，避免把本文件内部枚举或 Emm42 驱动细节泄漏给协议层。
 */
typedef struct
{
    uint8_t desired_mode;        /* 命令层期望模式：0=STOP，1=SCAN，2=TRACK，3=POSITION，4=JOG。 */
    uint8_t applied_mode;        /* 实际下发模式：0=STOP，1=SCAN，2=TRACK，3=POSITION，4=JOG。 */
    int32_t latest_error_px;     /* 最近一次视觉误差，单位像素。 */
    uint16_t speed_rpm;          /* 当前已下发转速，单位 RPM。 */
    uint8_t direction;           /* 当前方向：0=CW，1=CCW。 */
    uint8_t stable_count;        /* 连续进入中心死区的帧数。 */
    uint8_t centered;            /* 是否已经稳定对中：0=否，1=是。 */
} ConveyorMotor_Status_t;

/**
 * @brief 传送带 Emm42 电机任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 独占 `UART4` 作为传送带 Emm42 TTL 控制口，PC10(TX) 接传送带 Emm42 RX、PC11(RX) 接传送带 Emm42 TX；
 * 2. 初始化电机驱动，并在启动阶段恢复当前工程要求的控制模式；
 * 3. 使能电机并把电机拉回到已知静止态；
 * 4. 维护 `SCAN / TRACK / STOP` 三态控制；
 * 5. 在跟踪模式下根据主机坐标误差动态调速；
 * 6. 在目标进入摄像头中心死区后立即停止。
 */
void ConveyorMotorService_Task(void *argument);

/**
 * @brief 处理一条发给传送带电机服务的串口命令。
 * @param command_buffer 已经规范化后的命令字符串，不能为空。
 * @return uint8_t 1 表示该命令已由电机服务处理，0 表示不是电机命令。
 *
 * 当前支持：
 * 1. `BELTSCAN`
 * 2. `BELTSTOP`
 * 3. `BELTTRACK <signed_error_px>`
 * 4. `BELTCAM <enable> <current_x> <center_x>`
 * 5. `BELTENABLE <0|1> [signed_error_px]`
 * 6. `BELTINFO`
 *
 * 命令仍然沿用当前 `USART2` 统一入口进行分发，
 * 避免多个任务同时直接消费串口接收缓存。
 */
uint8_t ConveyorMotorService_HandleCommand(const char *command_buffer);

/**
 * @brief 请求传送带进入低速扫描模式。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该接口供二进制协议服务调用，行为等价于文本命令 `BELTSCAN`，
 * 但不会额外解析字符串，也不会输出文本成功提示。
 */
uint8_t ConveyorMotorService_RequestScan(void);

/**
 * @brief 请求传送带立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该接口供二进制协议服务调用，行为等价于文本命令 `BELTSTOP`。
 */
uint8_t ConveyorMotorService_RequestStop(void);

/**
 * @brief 请求传送带按照视觉误差进入跟踪模式。
 * @param error_px 视觉目标相对中心的带符号像素误差，单位像素。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该接口供二进制 `VISION_POS` 调用，行为等价于文本命令 `BELTTRACK <error>`。
 */
uint8_t ConveyorMotorService_RequestTrack(int32_t error_px);

/**
 * @brief 请求传送带按指定方向持续速度运动。
 * @param forward_flag 1 表示按工程默认前进方向运动，0 表示按工程默认后退方向运动。
 * @param speed_rpm 速度模式转速，单位 RPM，范围 1~5000。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或传送带任务尚未就绪。
 *
 * 该接口供二进制 `ACTUATOR_VEL_MOVE` 手动控制使用。
 * 它不会自动停止，必须由 `ConveyorMotorService_RequestStop()` 或自动流程 STOP 命令结束。
 */
uint8_t ConveyorMotorService_RequestJog(uint8_t forward_flag, uint16_t speed_rpm);

/**
 * @brief 请求传送带按相对位置模式移动固定步数。
 * @param forward_flag 1 表示按工程默认前进方向移动，0 表示按工程默认后退方向移动。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 时使用传送带常规速度。
 * @param pulse_count 相对移动步数，单位 step，范围 1~4294967295。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或传送带任务尚未就绪。
 */
uint8_t ConveyorMotorService_RequestPosition(uint8_t forward_flag,
                                             uint16_t speed_rpm,
                                             uint32_t pulse_count);

/**
 * @brief 请求传送带按相对位置模式移动，并在真实到位或超时时向 MP157 上报事件。
 * @param forward_flag 1 表示工程默认前进方向，0 表示工程默认后退方向。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用运行参数常规速度。
 * @param pulse_count 相对移动步数，单位 step，必须大于 0。
 * @param cycle_id 自动检测流程 ID，用于让 MP157 匹配本轮流程。
 * @param related_seq 原始 `ACTUATOR_POS_MOVE` 帧序号，用于让 MP157 匹配具体命令。
 * @param actuator 协议执行器编号，传送带固定为 0，但保留入参便于 detail 字段一致打包。
 * @param direction 原始协议方向字段，按 MP157 下发值原样回填到完成事件。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或任务队列尚未创建。
 *
 * 注意：协议 ACK 只说明本函数投递成功，不说明电机已经到位。
 * 真正到位必须等待 F4 后续发送 `EVENT_REPORT/ACTUATOR_MOVE_DONE`。
 */
uint8_t ConveyorMotorService_RequestPositionWithReport(uint8_t forward_flag,
                                                       uint16_t speed_rpm,
                                                       uint32_t pulse_count,
                                                       uint16_t cycle_id,
                                                       uint16_t related_seq,
                                                       uint8_t actuator,
                                                       uint8_t direction);

/**
 * @brief 请求传送带把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该接口供二进制 `ACTUATOR_HOME` 参数标定使用。
 * F4 任务会先停止传送带电机，再发送 Emm42 当前位置清零命令；不会让电机主动运动。
 */
uint8_t ConveyorMotorService_RequestSetCurrentPositionZero(void);

/**
 * @brief 更新传送带步进电机运行参数。
 * @param address Emm42 电机地址，允许 1~247。
 * @param min_step 最小步长，单位 step，当前保存给后续位置步进命令使用。
 * @param normal_speed_rpm 视觉对中/短步位置速度，单位 RPM，允许 0~5000，0 表示对应动作保持停止。
 * @param scan_speed_rpm 上料扫描速度，单位 RPM，允许 0~5000，0 表示扫描阶段保持停止。
 * @param direction 方向映射，正数表示按工程默认正向，负数表示反向。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或传送带任务尚未就绪。
 *
 * 该接口由 MP157 二进制 `STEPPER_PARAM_SET` 调用，只更新 F4 运行内存参数；
 * 当前不写 F4 Flash，也不改 Emm42 驱动器自身 EEPROM。
 */
uint8_t ConveyorMotorService_RequestRuntimeConfig(uint8_t address,
                                                  uint16_t min_step,
                                                  uint16_t normal_speed_rpm,
                                                  uint16_t scan_speed_rpm,
                                                  int8_t direction);

/**
 * @brief 读取传送带服务当前状态快照。
 * @param status 状态输出结构体，不能为空。
 * @return uint8_t 1 表示读取成功，0 表示参数为空或服务尚未就绪。
 *
 * 该接口不发送任何文本日志，只把状态交给二进制协议层组装 `STATUS_REPORT`。
 */
uint8_t ConveyorMotorService_GetStatus(ConveyorMotor_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
