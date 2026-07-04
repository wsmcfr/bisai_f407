#ifndef USER_DRIVER_EMM42_MOTOR_H
#define USER_DRIVER_EMM42_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Emm42 串口驱动返回状态码。
 *
 * 该枚举用于统一描述：
 * 1. 参数是否合法；
 * 2. 速度/加速度是否超出协议允许范围；
 * 3. 串口底层发送是否成功。
 */
typedef enum
{
    EMM42_MOTOR_STATUS_OK = 0,       /* 命令帧已经成功交给 HAL UART 发送，不代表电机运动一定已完成。 */
    EMM42_MOTOR_STATUS_ERROR,        /* UART 发送或底层 HAL 调用失败，调用者应输出阶段信息便于排查。 */
    EMM42_MOTOR_STATUS_INVALID_PARAM,/* 句柄、串口指针或模式参数非法，驱动不会发送任何字节。 */
    EMM42_MOTOR_STATUS_RANGE_ERROR   /* 速度、加速度等数值超出协议允许范围，驱动拒绝组帧发送。 */
} EMM42_MotorStatus_t;

/**
 * @brief Emm42 电机转动方向定义。
 *
 * 张大头例程里速度模式命令的方向字段约定为：
 * - 0：CW
 * - 非 0：CCW
 *
 * 这里把它明确成枚举，避免应用层到处写魔法数字。
 */
typedef enum
{
    EMM42_MOTOR_DIRECTION_CW = 0U,  /* 协议方向字段为 0，对应电机顺时针方向，实际传送带方向需结合安装确认。 */
    EMM42_MOTOR_DIRECTION_CCW = 1U  /* 协议方向字段为 1，对应电机逆时针方向，实际传送带方向需结合安装确认。 */
} EMM42_MotorDirection_t;

/**
 * @brief Emm42 控制模式定义。
 *
 * 根据张大头 Y42/Emm V5 协议：
 * - 0：开环模式
 * - 1：闭环 FOC 模式
 *
 * 当前传送带项目默认使用闭环 FOC，
 * 这样更符合“低速稳定巡航 + 视觉纠偏”的应用场景。
 */
typedef enum
{
    EMM42_MOTOR_CONTROL_MODE_OPEN_LOOP = 0U,       /* 开环模式，当前传送带不默认使用，保留用于故障排查。 */
    EMM42_MOTOR_CONTROL_MODE_CLOSED_LOOP_FOC = 1U  /* 闭环 FOC 模式，当前工程启动时会强制恢复到该控制模式。 */
} EMM42_MotorControlMode_t;

/**
 * @brief Emm42 电机驱动句柄。
 *
 * 该结构体保存协议层真正需要的最小硬件绑定信息：
 * 1. `huart`：实际负责 TTL 命令发送的串口；
 * 2. `address`：电机地址；
 * 3. `tx_timeout_ms`：阻塞发送超时时间。
 *
 * 该驱动层不固定某一个物理串口：
 * 1. 传送带服务可以把句柄绑定到 `UART4`；
 * 2. 摄像头运动服务可以把多个不同地址的电机绑定到 `USART6`；
 * 3. 上层服务必须保证同一条 UART 总线在同一时刻只有一个任务发送命令。
 */
typedef struct
{
    UART_HandleTypeDef *huart;  /* 电机 TTL 通信所绑定的 UART 句柄，由具体应用服务决定绑定 UART4、USART6 或其它串口。 */
    uint8_t address;            /* Emm42 电机站号地址，范围由电机协议决定，同一条串口总线上的多个电机必须使用不同地址。 */
    uint32_t tx_timeout_ms;     /* 阻塞发送单帧命令的超时时间，单位毫秒，用于限制串口异常时的等待时长。 */
} EMM42_MotorHandle_t;

/**
 * @brief 默认电机地址。
 *
 * 张大头例程默认按地址 1 演示，因此驱动默认值仍保留为 1。
 * 当前工程的实际地址分配由应用层覆盖：
 * - 传送带电机：地址 1，UART4，PC10/PC11；
 * - 摄像头前进/后退电机：地址 2，USART6，PC6/PC7；
 * - 摄像头上下电机：地址 3，USART6，PC6/PC7。
 */
#define EMM42_MOTOR_DEFAULT_ADDRESS            (1U)

/**
 * @brief 协议允许的最大速度，单位 RPM。
 */
#define EMM42_MOTOR_MAX_SPEED_RPM              (5000U)

/**
 * @brief 协议允许的最大加速度参数。
 */
#define EMM42_MOTOR_MAX_ACCEL                  (255U)

/**
 * @brief 默认串口发送超时时间，单位毫秒。
 *
 * 电机命令帧很短，20ms 对 115200 波特率已经足够保守。
 */
#define EMM42_MOTOR_DEFAULT_TX_TIMEOUT_MS      (20U)

/**
 * @brief 加载 Emm42 驱动默认配置。
 * @param motor 电机句柄指针，不能为空。
 * @param huart 绑定的串口句柄，不能为空。
 *
 * 该函数只负责把默认地址、默认超时和串口句柄写入句柄结构，
 * 不会立即发送任何控制命令。
 */
void EMM42_MotorLoadDefaultConfig(EMM42_MotorHandle_t *motor, UART_HandleTypeDef *huart);

/**
 * @brief 校验 Emm42 驱动句柄是否可用。
 * @param motor 电机句柄指针，不能为空。
 * @return EMM42_MotorStatus_t 校验结果。
 *
 * 当前电机驱动不需要额外寄存器初始化，因此 `Init` 的职责主要是：
 * 1. 检查句柄是否完整；
 * 2. 为上层提供统一初始化入口；
 * 3. 便于后续扩展为更复杂的上电自检流程。
 */
EMM42_MotorStatus_t EMM42_MotorInit(EMM42_MotorHandle_t *motor);

/**
 * @brief 发送电机使能/失能命令。
 * @param motor 电机句柄指针，不能为空。
 * @param enabled `true` 表示使能，`false` 表示失能。
 * @param sync_flag 同步运动标志，当前通常传 `false`。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorSetEnable(const EMM42_MotorHandle_t *motor,
                                         bool enabled,
                                         bool sync_flag);

/**
 * @brief 设置电机当前控制模式。
 * @param motor 电机句柄指针，不能为空。
 * @param control_mode 控制模式，0 为开环，1 为闭环 FOC。
 * @param save_flag `true` 表示写入电机参数存储，`false` 表示仅本次上电生效。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 设计说明：
 * 1. 该接口对应张大头例程中的 `Emm_V5_Modify_Ctrl_Mode()`；
 * 2. 本工程默认在每次任务启动时重新下发一次模式修复命令；
 * 3. 默认不写入存储，避免系统每次上电都重复擦写电机内部参数区。
 */
EMM42_MotorStatus_t EMM42_MotorSetControlMode(const EMM42_MotorHandle_t *motor,
                                              EMM42_MotorControlMode_t control_mode,
                                              bool save_flag);

/**
 * @brief 设置电机面板按键锁定状态。
 * @param motor 电机句柄指针，不能为空。
 * @param locked `true` 表示锁定面板按键，`false` 表示允许面板按键操作。
 * @param save_flag `true` 表示写入电机参数存储，`false` 表示仅本次上电生效。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 该接口用于防止现场误触电机面板按键后把工作模式改乱。
 * 当前应用层默认保留该能力，但是否启用由上层服务决定。
 */
EMM42_MotorStatus_t EMM42_MotorSetButtonLock(const EMM42_MotorHandle_t *motor,
                                             bool locked,
                                             bool save_flag);

/**
 * @brief 发送速度模式命令。
 * @param motor 电机句柄指针，不能为空。
 * @param direction 旋转方向。
 * @param velocity_rpm 目标转速，单位 RPM，范围 0~5000。
 * @param acceleration 加速度参数，范围 0~255。
 * @param sync_flag 同步运动标志，当前通常传 `false`。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 该接口对应张大头例程中的 `Emm_V5_Vel_Control()`，
 * 但这里去掉了对 `huart1` 的硬编码，改为使用传入句柄绑定的串口。
 */
EMM42_MotorStatus_t EMM42_MotorSetVelocity(const EMM42_MotorHandle_t *motor,
                                           EMM42_MotorDirection_t direction,
                                           uint16_t velocity_rpm,
                                           uint8_t acceleration,
                                           bool sync_flag);

/**
 * @brief 发送相对位置模式运动命令。
 * @param motor 电机句柄指针，不能为空。
 * @param direction 旋转方向。
 * @param velocity_rpm 目标转速，单位 RPM，范围 0~5000。
 * @param acceleration 加速度参数，范围 0~255。
 * @param pulse_count 相对移动脉冲数，单位 step，范围 1~4294967295。
 * @param sync_flag 同步运动标志，当前通常传 `false`。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 该接口对应张大头 Emm42 `0xFD` 位置模式命令。
 * 本工程只开放相对位置模式：MP157/F4 上层传入固定步数，
 * 电机按当前位置移动 pulse_count 个脉冲，运动完成状态首版不解析回包。
 */
EMM42_MotorStatus_t EMM42_MotorMoveRelativePosition(const EMM42_MotorHandle_t *motor,
                                                    EMM42_MotorDirection_t direction,
                                                    uint16_t velocity_rpm,
                                                    uint8_t acceleration,
                                                    uint32_t pulse_count,
                                                    bool sync_flag);

/**
 * @brief 将电机当前位置清零。
 * @param motor 电机句柄指针，不能为空。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 该接口对应张大头 Emm V5 的“将当前的位置角度清零”命令。
 * 命令不会让电机转动，而是让驱动器把当前位置角度、位置误差和脉冲数等内部位置量清零。
 * 当前驱动只发送命令帧，不解析电机回包，因此返回 OK 只表示帧已成功发出。
 */
EMM42_MotorStatus_t EMM42_MotorResetCurrentPositionToZero(const EMM42_MotorHandle_t *motor);

/**
 * @brief 发送立即停止命令。
 * @param motor 电机句柄指针，不能为空。
 * @param sync_flag 同步运动标志，当前通常传 `false`。
 * @return EMM42_MotorStatus_t 发送结果。
 */
EMM42_MotorStatus_t EMM42_MotorStopNow(const EMM42_MotorHandle_t *motor, bool sync_flag);

#ifdef __cplusplus
}
#endif

#endif
