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
    uint8_t desired_mode;        /* 命令层期望模式：0=STOP，1=SCAN，2=TRACK。 */
    uint8_t applied_mode;        /* 实际下发模式：0=STOP，1=SCAN，2=TRACK。 */
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
 * 命令仍然沿用当前 `USART1` 统一入口进行分发，
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
 * @brief 更新传送带步进电机运行参数。
 * @param address Emm42 电机地址，允许 1~247。
 * @param min_step 最小步长，单位 step，当前保存给后续位置步进命令使用。
 * @param normal_speed_rpm 常规扫描速度，单位 RPM，允许 0~5000，0 表示保存后保持停止。
 * @param direction 方向映射，正数表示按工程默认正向，负数表示反向。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或传送带任务尚未就绪。
 *
 * 该接口由 MP157 二进制 `STEPPER_PARAM_SET` 调用，只更新 F4 运行内存参数；
 * 当前不写 F4 Flash，也不改 Emm42 驱动器自身 EEPROM。
 */
uint8_t ConveyorMotorService_RequestRuntimeConfig(uint8_t address,
                                                  uint16_t min_step,
                                                  uint16_t normal_speed_rpm,
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
