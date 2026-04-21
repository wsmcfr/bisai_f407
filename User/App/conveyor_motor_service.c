#include "conveyor_motor_service.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "emm42_motor.h"
#include "queue.h"
#include "uart_command.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

/**
 * @brief 控制任务主循环周期，单位毫秒。
 *
 * 该周期同时承担两件事：
 * 1. 周期检查“视觉坐标是否超时”；
 * 2. 在没有新命令时维持状态机推进频率。
 */
#define CONVEYOR_MOTOR_CONTROL_PERIOD_MS            (20U)

/**
 * @brief 主机在跟踪模式下的坐标更新超时时间，单位毫秒。
 *
 * 如果主机长时间不再发送坐标，而电机仍保持上一次速度继续运动，
 * 对传送带场景是不安全的，因此这里超时后直接停机。
 */
#define CONVEYOR_MOTOR_TRACK_TIMEOUT_MS             (300U)

/**
 * @brief 巡航模式的默认转速，单位 RPM。
 *
 * 用户要求“自己匀速转动，不需要太快”，
 * 因此这里默认给一个偏低的保守值。
 */
#define CONVEYOR_MOTOR_SCAN_SPEED_RPM               (60U)

/**
 * @brief 跟踪模式最小转速，单位 RPM。
 *
 * 这里的“最小转速”同时也作为“中心附近爬行速度”使用。
 * 你当前反馈 `BELTTRACK 11` 仍然偏快，因此把这个基础速度降下来，
 * 让小误差区域先以更慢的速度贴近中心。
 */
#define CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM          (20U)

/**
 * @brief 小误差爬行区上限，单位像素。
 *
 * 当误差刚刚走出死区时，仍然不适合马上给较高转速。
 * 因此在这个范围内，统一使用低速爬行，提升人工调试手感。
 */
#define CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX     (30U)

/**
 * @brief 跟踪模式最大转速，单位 RPM。
 *
 * 初版先限在一个相对保守的速度上限，避免刚接入视觉后电机动作过猛。
 */
#define CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM          (250U)

/**
 * @brief 像素误差到 RPM 的线性映射系数。
 *
 * 当前改为更柔和的映射：
 * 1. 小误差区先保持低速爬行；
 * 2. 超出爬行区后，再按线性关系缓慢增速。
 *
 * 真机联调时如果发现响应偏慢或偏快，只需要改这个宏。
 */
#define CONVEYOR_MOTOR_TRACK_RPM_PER_PIXEL          (1U)

/**
 * @brief 默认加速度参数。
 *
 * 该参数直接对应张大头速度模式协议中的 `acc` 字段。
 * 在低速视觉纠偏场景里，适当减小加速度可以让动作更柔和。
 */
#define CONVEYOR_MOTOR_ACCEL                        (5U)

/**
 * @brief 启动时是否强制恢复 Emm42 的控制模式。
 *
 * 你反馈“误按电机按键后不转了”，
 * 很可能是电机当前面板配置已偏离当前工程默认假设。
 * 因此这里在任务启动时主动下发一次模式修复命令。
 */
#define CONVEYOR_MOTOR_STARTUP_FORCE_CTRL_MODE      (1U)

/**
 * @brief 启动时强制恢复到的控制模式。
 *
 * 当前传送带采用串口速度控制 + 闭环 FOC 更稳，
 * 因此默认固定恢复成闭环 FOC 模式。
 */
#define CONVEYOR_MOTOR_STARTUP_CTRL_MODE            (EMM42_MOTOR_CONTROL_MODE_CLOSED_LOOP_FOC)

/**
 * @brief 启动时是否锁定电机面板按键。
 *
 * 当前默认在启动阶段主动下发一次“按键锁定”命令，
 * 用于防止现场误触电机面板按键后把工作模式改乱。
 */
#define CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK    (1U)

/**
 * @brief 启动时希望设置的按键锁定状态。
 *
 * 仅当 `CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK` 打开时生效。
 */
#define CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED        (1U)

/**
 * @brief 启动修复配置是否写入电机内部存储。
 *
 * 默认只在当前上电周期生效，不反复写内部参数区，
 * 避免 MCU 每次重启都执行一次无意义的存储写入。
 */
#define CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH        (0U)

/**
 * @brief 启动阶段配置命令之间的间隔，单位毫秒。
 *
 * 某些参数修改命令发完后，电机侧需要一个很短的处理时间。
 * 这里保守留 20ms，换取启动修复流程更稳定。
 */
#define CONVEYOR_MOTOR_STARTUP_CONFIG_GAP_MS        (20U)

/**
 * @brief 摄像头中心死区，单位像素。
 *
 * 当目标误差绝对值落入这个区间时，电机立即停止，
 * 以避免在中心附近来回抖动。
 */
#define CONVEYOR_MOTOR_CENTER_DEADBAND_PX           (10)

/**
 * @brief 连续多少帧都落在中心死区内，才认为“已经稳定对中”。
 *
 * 注意：
 * 1. 电机会在第一帧进入死区时就立即停止；
 * 2. 这里只是补一个“稳定达标”计数，方便后续日志和状态观察。
 */
#define CONVEYOR_MOTOR_CENTER_STABLE_FRAMES         (3U)

/**
 * @brief 正误差默认对应 CW 方向。
 *
 * 传送带的实际机械安装方向和相机坐标正方向有关，
 * 真机联调时如果发现“误差越大却往反方向跑”，只需要把这里改成 0。
 */
#define CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW         (1U)

/**
 * @brief 是否在任务启动后默认进入巡航模式。
 *
 * 用户当前目标是“坐标未使能时自己匀速转动”，
 * 因此初版默认直接进入 SCAN。
 * 若后续希望上电先静止，再等待主机命令，只需改成 0。
 */
#define CONVEYOR_MOTOR_STARTUP_SCAN_ENABLE          (1U)

/**
 * @brief 命令队列长度。
 *
 * 当前电机控制只关心“最新控制意图”，
 * 因此把队列长度限制为 1，并使用覆盖写入策略，避免堆积陈旧坐标。
 */
#define CONVEYOR_MOTOR_COMMAND_QUEUE_LENGTH         (1U)

/**
 * @brief 传送带电机服务的高层状态。
 */
typedef enum
{
    CONVEYOR_MOTOR_MODE_STOP = 0,
    CONVEYOR_MOTOR_MODE_SCAN,
    CONVEYOR_MOTOR_MODE_TRACK
} ConveyorMotor_Mode_t;

/**
 * @brief 发给电机任务的内部命令类型。
 */
typedef enum
{
    CONVEYOR_MOTOR_COMMAND_STOP = 0,
    CONVEYOR_MOTOR_COMMAND_SCAN,
    CONVEYOR_MOTOR_COMMAND_TRACK
} ConveyorMotor_CommandType_t;

/**
 * @brief 电机任务消息队列中的命令结构。
 *
 * 当前只需要表达两类信息：
 * 1. 模式切换；
 * 2. 跟踪模式下的最新像素误差。
 */
typedef struct
{
    ConveyorMotor_CommandType_t type;
    int32_t error_px;
} ConveyorMotor_Command_t;

/**
 * @brief 供 `BELTINFO` 查询使用的运行时快照。
 *
 * 这里把“期望模式”和“实际执行状态”分开保存，
 * 方便区分“正在 TRACK，但当前因进入死区而已停住”的情况。
 */
typedef struct
{
    ConveyorMotor_Mode_t desired_mode;
    ConveyorMotor_Mode_t applied_mode;
    int32_t latest_error_px;
    uint16_t applied_speed_rpm;
    EMM42_MotorDirection_t applied_direction;
    uint8_t center_stable_count;
    uint8_t centered_flag;
} ConveyorMotor_RuntimeSnapshot_t;

/**
 * @brief 电机任务内部的完整运行时状态。
 */
typedef struct
{
    ConveyorMotor_Mode_t desired_mode;
    ConveyorMotor_Mode_t applied_mode;
    int32_t latest_error_px;
    uint16_t applied_speed_rpm;
    EMM42_MotorDirection_t applied_direction;
    uint8_t center_stable_count;
    uint8_t centered_flag;
    uint8_t fresh_track_sample_flag;
    uint8_t track_timeout_reported_flag;
    TickType_t last_track_update_tick;
} ConveyorMotor_Runtime_t;

/**
 * @brief 传送带电机命令队列。
 *
 * 由 `USART1` 命令分发入口写入，由电机任务独占读取。
 */
static QueueHandle_t g_conveyor_motor_command_queue = NULL;

/**
 * @brief 对外可读的运行时快照。
 *
 * `BELTINFO` 命令需要从另一个任务上下文读取当前状态，
 * 因此这里保留一个受临界区保护的轻量快照。
 */
static ConveyorMotor_RuntimeSnapshot_t g_conveyor_motor_runtime_snapshot =
{
    CONVEYOR_MOTOR_MODE_STOP,
    CONVEYOR_MOTOR_MODE_STOP,
    0,
    0U,
    EMM42_MOTOR_DIRECTION_CW,
    0U,
    0U
};

/**
 * @brief 跳过命令参数中的空格和制表符。
 * @param cursor 字符串游标指针，不能为空。
 */
static void ConveyorMotorService_SkipSpaces(const char **cursor)
{
    if ((cursor == NULL) || (*cursor == NULL))
    {
        return;
    }

    while ((**cursor == ' ') || (**cursor == '\t'))
    {
        ++(*cursor);
    }
}

/**
 * @brief 返回模式名称，便于日志输出。
 * @param mode 运行模式。
 * @return const char* 模式字符串。
 */
static const char *ConveyorMotorService_GetModeName(ConveyorMotor_Mode_t mode)
{
    switch (mode)
    {
        case CONVEYOR_MOTOR_MODE_SCAN:
            return "SCAN";

        case CONVEYOR_MOTOR_MODE_TRACK:
            return "TRACK";

        case CONVEYOR_MOTOR_MODE_STOP:
        default:
            return "STOP";
    }
}

/**
 * @brief 返回控制模式名称，便于启动日志输出。
 * @param control_mode Emm42 控制模式。
 * @return const char* 模式名称字符串。
 */
static const char *ConveyorMotorService_GetControlModeName(EMM42_MotorControlMode_t control_mode)
{
    switch (control_mode)
    {
        case EMM42_MOTOR_CONTROL_MODE_OPEN_LOOP:
            return "OPEN_LOOP";

        case EMM42_MOTOR_CONTROL_MODE_CLOSED_LOOP_FOC:
        default:
            return "CLOSED_LOOP_FOC";
    }
}

/**
 * @brief 把当前运行时状态同步到全局快照。
 * @param runtime 当前任务内部状态，不能为空。
 */
static void ConveyorMotorService_UpdateSnapshot(const ConveyorMotor_Runtime_t *runtime)
{
    if (runtime == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_conveyor_motor_runtime_snapshot.desired_mode = runtime->desired_mode;
    g_conveyor_motor_runtime_snapshot.applied_mode = runtime->applied_mode;
    g_conveyor_motor_runtime_snapshot.latest_error_px = runtime->latest_error_px;
    g_conveyor_motor_runtime_snapshot.applied_speed_rpm = runtime->applied_speed_rpm;
    g_conveyor_motor_runtime_snapshot.applied_direction = runtime->applied_direction;
    g_conveyor_motor_runtime_snapshot.center_stable_count = runtime->center_stable_count;
    g_conveyor_motor_runtime_snapshot.centered_flag = runtime->centered_flag;
    taskEXIT_CRITICAL();
}

/**
 * @brief 把带符号误差转换成绝对值。
 * @param error_px 像素误差。
 * @return uint32_t 绝对值结果。
 */
static uint32_t ConveyorMotorService_GetAbsoluteError(int32_t error_px)
{
    return (error_px >= 0) ? (uint32_t)error_px : (uint32_t)(-error_px);
}

/**
 * @brief 解析一个带符号十进制整数。
 * @param cursor 当前字符串游标，不能为空。
 * @param value 输出结果，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示格式不合法。
 *
 * 该函数会在成功后把游标推进到数字之后，
 * 便于同一条命令连续读取多个参数。
 */
static uint8_t ConveyorMotorService_ParseSignedInt32(const char **cursor, int32_t *value)
{
    char *end_pointer;
    long parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return 0U;
    }

    ConveyorMotorService_SkipSpaces(cursor);
    if (**cursor == '\0')
    {
        return 0U;
    }

    parsed_value = strtol(*cursor, &end_pointer, 10);
    if (*cursor == end_pointer)
    {
        return 0U;
    }

    *value = (int32_t)parsed_value;
    *cursor = end_pointer;
    return 1U;
}

/**
 * @brief 向电机任务投递一条“覆盖旧值”的最新命令。
 * @param command 待投递命令，不能为空。
 * @return uint8_t 1 表示投递成功，0 表示任务尚未就绪或投递失败。
 *
 * 当前采用长度为 1 的队列，因此这里使用覆盖写入策略，
 * 确保视觉坐标更新时始终以“最新误差”为准，而不是排队执行历史误差。
 */
static uint8_t ConveyorMotorService_PostCommand(const ConveyorMotor_Command_t *command)
{
    if ((command == NULL) || (g_conveyor_motor_command_queue == NULL))
    {
        return 0U;
    }

    (void)xQueueOverwrite(g_conveyor_motor_command_queue, command);
    return 1U;
}

/**
 * @brief 解析 `BELTTRACK <error>` 命令。
 * @param command_buffer 已规范化后的命令字符串。
 * @param error_px 输出误差值，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示格式不合法。
 */
static uint8_t ConveyorMotorService_ParseTrackCommand(const char *command_buffer, int32_t *error_px)
{
    const char *cursor;

    if ((command_buffer == NULL) || (error_px == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "BELTTRACK", 9U) != 0)
    {
        return 0U;
    }

    cursor = command_buffer + 9U;
    if (ConveyorMotorService_ParseSignedInt32(&cursor, error_px) == 0U)
    {
        return 0U;
    }

    ConveyorMotorService_SkipSpaces(&cursor);
    return (*cursor == '\0') ? 1U : 0U;
}

/**
 * @brief 解析 `BELTENABLE <0|1> [error]` 命令。
 * @param command_buffer 已规范化后的命令字符串。
 * @param enable_flag 输出使能标志，不能为空。
 * @param error_px 输出误差值，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示格式不合法。
 *
 * 设计目的：
 * 让上位机可以直接用“坐标使能标志位”语义下发命令，
 * 更贴合当前任务原始需求描述。
 */
static uint8_t ConveyorMotorService_ParseEnableCommand(const char *command_buffer,
                                                       uint8_t *enable_flag,
                                                       int32_t *error_px)
{
    const char *cursor;
    int32_t parsed_enable;

    if ((command_buffer == NULL) || (enable_flag == NULL) || (error_px == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "BELTENABLE", 10U) != 0)
    {
        return 0U;
    }

    cursor = command_buffer + 10U;
    if (ConveyorMotorService_ParseSignedInt32(&cursor, &parsed_enable) == 0U)
    {
        return 0U;
    }

    if ((parsed_enable != 0) && (parsed_enable != 1))
    {
        return 0U;
    }

    *enable_flag = (uint8_t)parsed_enable;
    if (*enable_flag == 0U)
    {
        /*
         * 主机侧有可能始终按固定字段数发送命令，
         * 即使“未使能跟踪”时也会把误差位占着发出来。
         * 这里对 `enable=0` 采取宽松接受策略，直接切回巡航模式。
         */
        return 1U;
    }

    if (ConveyorMotorService_ParseSignedInt32(&cursor, error_px) == 0U)
    {
        return 0U;
    }

    ConveyorMotorService_SkipSpaces(&cursor);
    return (*cursor == '\0') ? 1U : 0U;
}

/**
 * @brief 解析 `BELTCAM <enable> <current_x> <center_x>` 命令。
 * @param command_buffer 已规范化后的命令字符串。
 * @param enable_flag 输出使能标志，不能为空。
 * @param error_px 输出误差值，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示格式不合法。
 *
 * 这里把误差定义为：
 * `error_px = current_x - center_x`
 *
 * 这样上位机既可以直接发送误差，也可以把“当前坐标 + 中心坐标”都发下来，
 * 由下位机本地统一换算。
 */
static uint8_t ConveyorMotorService_ParseCameraCommand(const char *command_buffer,
                                                       uint8_t *enable_flag,
                                                       int32_t *error_px)
{
    const char *cursor;
    int32_t parsed_enable;
    int32_t current_x;
    int32_t center_x;

    if ((command_buffer == NULL) || (enable_flag == NULL) || (error_px == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "BELTCAM", 7U) != 0)
    {
        return 0U;
    }

    cursor = command_buffer + 7U;
    if (ConveyorMotorService_ParseSignedInt32(&cursor, &parsed_enable) == 0U)
    {
        return 0U;
    }

    if ((parsed_enable != 0) && (parsed_enable != 1))
    {
        return 0U;
    }

    *enable_flag = (uint8_t)parsed_enable;
    if (*enable_flag == 0U)
    {
        /*
         * 为了兼容“固定长度上位机协议”，
         * 这里允许 `BELTCAM 0` 后面继续跟坐标字段，但下位机直接忽略。
         */
        return 1U;
    }

    if ((ConveyorMotorService_ParseSignedInt32(&cursor, &current_x) == 0U) ||
        (ConveyorMotorService_ParseSignedInt32(&cursor, &center_x) == 0U))
    {
        return 0U;
    }

    ConveyorMotorService_SkipSpaces(&cursor);
    if (*cursor != '\0')
    {
        return 0U;
    }

    *error_px = current_x - center_x;
    return 1U;
}

/**
 * @brief 把误差绝对值映射成跟踪转速。
 * @param absolute_error_px 像素误差绝对值。
 * @return uint16_t 目标转速，单位 RPM。
 */
static uint16_t ConveyorMotorService_MapErrorToSpeed(uint32_t absolute_error_px)
{
    uint32_t mapped_speed_rpm;

    /*
     * 中心附近先给一个固定低速爬行值，
     * 避免误差刚走出死区时速度突然偏快。
     */
    if (absolute_error_px <= CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX)
    {
        return CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM;
    }

    mapped_speed_rpm = CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM +
                       ((absolute_error_px - CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX) *
                        CONVEYOR_MOTOR_TRACK_RPM_PER_PIXEL);

    if (mapped_speed_rpm > CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM)
    {
        mapped_speed_rpm = CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM;
    }

    return (uint16_t)mapped_speed_rpm;
}

/**
 * @brief 根据误差符号决定电机方向。
 * @param error_px 带符号像素误差。
 * @return EMM42_MotorDirection_t 目标方向。
 *
 * 若真机发现方向反了，只需要修改
 * `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW` 宏，而不需要改状态机逻辑。
 */
static EMM42_MotorDirection_t ConveyorMotorService_GetDirectionByError(int32_t error_px)
{
    uint8_t positive_is_cw;
    uint8_t error_is_positive;

    positive_is_cw = CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW;
    error_is_positive = (error_px >= 0) ? 1U : 0U;

    if (positive_is_cw != 0U)
    {
        return (error_is_positive != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    }

    return (error_is_positive != 0U) ? EMM42_MOTOR_DIRECTION_CCW : EMM42_MOTOR_DIRECTION_CW;
}

/**
 * @brief 应用一次“立即停止”到电机。
 * @param motor 电机句柄，不能为空。
 * @param runtime 任务运行时状态，不能为空。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 该函数会同步更新“已应用状态”，避免后续重复发送相同的停止命令。
 */
static EMM42_MotorStatus_t ConveyorMotorService_ApplyStop(const EMM42_MotorHandle_t *motor,
                                                          ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;

    if ((motor == NULL) || (runtime == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if (runtime->applied_mode == CONVEYOR_MOTOR_MODE_STOP)
    {
        return EMM42_MOTOR_STATUS_OK;
    }

    status = EMM42_MotorStopNow(motor, false);
    if (status == EMM42_MOTOR_STATUS_OK)
    {
        runtime->applied_mode = CONVEYOR_MOTOR_MODE_STOP;
        runtime->applied_speed_rpm = 0U;
    }

    return status;
}

/**
 * @brief 应用一次“巡航匀速转动”到电机。
 * @param motor 电机句柄，不能为空。
 * @param runtime 任务运行时状态，不能为空。
 * @return EMM42_MotorStatus_t 发送结果。
 */
static EMM42_MotorStatus_t ConveyorMotorService_ApplyScan(const EMM42_MotorHandle_t *motor,
                                                          ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;
    EMM42_MotorDirection_t direction;

    if ((motor == NULL) || (runtime == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    /*
     * 巡航方向先统一固定为 CW。
     * 若现场需要反向巡航，可后续再补成可配置项。
     */
    direction = EMM42_MOTOR_DIRECTION_CW;

    if ((runtime->applied_mode == CONVEYOR_MOTOR_MODE_SCAN) &&
        (runtime->applied_speed_rpm == CONVEYOR_MOTOR_SCAN_SPEED_RPM) &&
        (runtime->applied_direction == direction))
    {
        return EMM42_MOTOR_STATUS_OK;
    }

    status = EMM42_MotorSetVelocity(motor,
                                    direction,
                                    CONVEYOR_MOTOR_SCAN_SPEED_RPM,
                                    CONVEYOR_MOTOR_ACCEL,
                                    false);
    if (status == EMM42_MOTOR_STATUS_OK)
    {
        runtime->applied_mode = CONVEYOR_MOTOR_MODE_SCAN;
        runtime->applied_speed_rpm = CONVEYOR_MOTOR_SCAN_SPEED_RPM;
        runtime->applied_direction = direction;
    }

    return status;
}

/**
 * @brief 根据最新误差应用一次跟踪速度命令。
 * @param motor 电机句柄，不能为空。
 * @param runtime 任务运行时状态，不能为空。
 * @return EMM42_MotorStatus_t 发送结果。
 */
static EMM42_MotorStatus_t ConveyorMotorService_ApplyTrack(const EMM42_MotorHandle_t *motor,
                                                           ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;
    EMM42_MotorDirection_t direction;
    uint16_t speed_rpm;

    if ((motor == NULL) || (runtime == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    direction = ConveyorMotorService_GetDirectionByError(runtime->latest_error_px);
    speed_rpm = ConveyorMotorService_MapErrorToSpeed(
        ConveyorMotorService_GetAbsoluteError(runtime->latest_error_px));

    if ((runtime->applied_mode == CONVEYOR_MOTOR_MODE_TRACK) &&
        (runtime->applied_speed_rpm == speed_rpm) &&
        (runtime->applied_direction == direction))
    {
        return EMM42_MOTOR_STATUS_OK;
    }

    status = EMM42_MotorSetVelocity(motor, direction, speed_rpm, CONVEYOR_MOTOR_ACCEL, false);
    if (status == EMM42_MOTOR_STATUS_OK)
    {
        runtime->applied_mode = CONVEYOR_MOTOR_MODE_TRACK;
        runtime->applied_speed_rpm = speed_rpm;
        runtime->applied_direction = direction;
    }

    return status;
}

/**
 * @brief 在任务启动阶段把电机恢复到当前工程预期的基础模式。
 * @param motor 电机句柄，不能为空。
 * @param failed_stage 输出失败阶段描述，不能为空。
 * @return EMM42_MotorStatus_t 配置结果。
 *
 * 当前主要做两类“修复型初始化”：
 * 1. 强制恢复成闭环 FOC 控制模式；
 * 2. 按需要设置面板按键锁定状态。
 *
 * 这些命令只负责恢复电机内部配置，
 * 不负责电机运动控制，真正的运动命令仍由状态机统一下发。
 */
static EMM42_MotorStatus_t ConveyorMotorService_ApplyStartupConfig(const EMM42_MotorHandle_t *motor,
                                                                   const char **failed_stage)
{
    EMM42_MotorStatus_t status;

    if ((motor == NULL) || (failed_stage == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if (CONVEYOR_MOTOR_STARTUP_FORCE_CTRL_MODE != 0U)
    {
        *failed_stage = "startup control mode";
        status = EMM42_MotorSetControlMode(
            motor,
            CONVEYOR_MOTOR_STARTUP_CTRL_MODE,
            (CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH != 0U));
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            return status;
        }

        osDelay(CONVEYOR_MOTOR_STARTUP_CONFIG_GAP_MS);
    }

    if (CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK != 0U)
    {
        *failed_stage = "startup button lock";
        status = EMM42_MotorSetButtonLock(
            motor,
            (CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED != 0U),
            (CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH != 0U));
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            return status;
        }

        osDelay(CONVEYOR_MOTOR_STARTUP_CONFIG_GAP_MS);
    }

    return EMM42_MOTOR_STATUS_OK;
}

/**
 * @brief 打印服务启动后的关键信息。
 */
static void ConveyorMotorService_ReportReady(void)
{
    my_printf(&huart1,
              "[OK][BELT] Emm42 conveyor service started. USART2=PA2/PA3, addr=%u, mode=velocity, ctrl=%s\r\n",
              (unsigned int)EMM42_MOTOR_DEFAULT_ADDRESS,
              ConveyorMotorService_GetControlModeName(CONVEYOR_MOTOR_STARTUP_CTRL_MODE));
    my_printf(&huart1,
              "[INFO][BELT] scan=%u rpm, track=%u~%u rpm, crawl<=%u px, deadband=%d px, stable=%u, timeout=%u ms\r\n",
              (unsigned int)CONVEYOR_MOTOR_SCAN_SPEED_RPM,
              (unsigned int)CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM,
              (unsigned int)CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM,
              (unsigned int)CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX,
              (int)CONVEYOR_MOTOR_CENTER_DEADBAND_PX,
              (unsigned int)CONVEYOR_MOTOR_CENTER_STABLE_FRAMES,
              (unsigned int)CONVEYOR_MOTOR_TRACK_TIMEOUT_MS);
    my_printf(&huart1,
              "[INFO][BELT] startup_fix ctrl_force=%u, btn_lock_force=%u, btn_lock=%u, save=%s\r\n",
              (unsigned int)CONVEYOR_MOTOR_STARTUP_FORCE_CTRL_MODE,
              (unsigned int)CONVEYOR_MOTOR_STARTUP_FORCE_BUTTON_LOCK,
              (unsigned int)CONVEYOR_MOTOR_STARTUP_BUTTON_LOCKED,
              (CONVEYOR_MOTOR_STARTUP_SAVE_TO_FLASH != 0U) ? "FLASH" : "RAM");
}

/**
 * @brief 输出一次发送失败日志。
 * @param stage 出错阶段描述。
 * @param status 驱动返回状态。
 */
static void ConveyorMotorService_ReportDriverError(const char *stage, EMM42_MotorStatus_t status)
{
    my_printf(&huart1,
              "[ERROR][BELT] %s failed, status=%d\r\n",
              (stage != NULL) ? stage : "belt command",
              (int)status);
}

/**
 * @brief 处理一条从命令队列取出的内部控制命令。
 * @param command 内部命令对象，不能为空。
 * @param runtime 任务运行时状态，不能为空。
 */
static void ConveyorMotorService_HandleQueuedCommand(const ConveyorMotor_Command_t *command,
                                                     ConveyorMotor_Runtime_t *runtime)
{
    if ((command == NULL) || (runtime == NULL))
    {
        return;
    }

    switch (command->type)
    {
        case CONVEYOR_MOTOR_COMMAND_SCAN:
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_SCAN;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;
            break;

        case CONVEYOR_MOTOR_COMMAND_TRACK:
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_TRACK;
            runtime->latest_error_px = command->error_px;
            runtime->fresh_track_sample_flag = 1U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->last_track_update_tick = xTaskGetTickCount();
            break;

        case CONVEYOR_MOTOR_COMMAND_STOP:
        default:
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_STOP;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;
            break;
    }
}

/**
 * @brief 执行一次状态机控制步进。
 * @param motor 电机句柄，不能为空。
 * @param runtime 任务运行时状态，不能为空。
 *
 * 状态机规则：
 * 1. `SCAN`：固定低速巡航；
 * 2. `TRACK`：有新误差就更新速度；进入死区立刻停；
 * 3. `STOP`：保持停止；
 * 4. 跟踪超时：强制停机。
 */
static void ConveyorMotorService_ControlStep(const EMM42_MotorHandle_t *motor,
                                             ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;
    TickType_t current_tick;

    if ((motor == NULL) || (runtime == NULL))
    {
        return;
    }

    current_tick = xTaskGetTickCount();

    switch (runtime->desired_mode)
    {
        case CONVEYOR_MOTOR_MODE_SCAN:
            status = ConveyorMotorService_ApplyScan(motor, runtime);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("scan command", status);
            }
            break;

        case CONVEYOR_MOTOR_MODE_TRACK:
            if ((current_tick - runtime->last_track_update_tick) >
                pdMS_TO_TICKS(CONVEYOR_MOTOR_TRACK_TIMEOUT_MS))
            {
                runtime->desired_mode = CONVEYOR_MOTOR_MODE_STOP;
                runtime->fresh_track_sample_flag = 0U;
                runtime->center_stable_count = 0U;
                runtime->centered_flag = 0U;
                status = ConveyorMotorService_ApplyStop(motor, runtime);
                if (status != EMM42_MOTOR_STATUS_OK)
                {
                    ConveyorMotorService_ReportDriverError("track timeout stop", status);
                }
                if (runtime->track_timeout_reported_flag == 0U)
                {
                    runtime->track_timeout_reported_flag = 1U;
                    my_printf(&huart1, "[WARN][BELT] Track timeout. Motor stopped.\r\n");
                }
                break;
            }

            if (runtime->fresh_track_sample_flag != 0U)
            {
                runtime->fresh_track_sample_flag = 0U;

                if (ConveyorMotorService_GetAbsoluteError(runtime->latest_error_px) <=
                    (uint32_t)CONVEYOR_MOTOR_CENTER_DEADBAND_PX)
                {
                    status = ConveyorMotorService_ApplyStop(motor, runtime);
                    if (status != EMM42_MOTOR_STATUS_OK)
                    {
                        ConveyorMotorService_ReportDriverError("center stop", status);
                    }

                    if (runtime->center_stable_count < CONVEYOR_MOTOR_CENTER_STABLE_FRAMES)
                    {
                        ++runtime->center_stable_count;
                    }

                    if ((runtime->center_stable_count >= CONVEYOR_MOTOR_CENTER_STABLE_FRAMES) &&
                        (runtime->centered_flag == 0U))
                    {
                        runtime->centered_flag = 1U;
                        my_printf(&huart1,
                                  "[EVENT][BELT] Target centered. Motor stopped.\r\n");
                    }
                }
                else
                {
                    runtime->center_stable_count = 0U;
                    runtime->centered_flag = 0U;

                    status = ConveyorMotorService_ApplyTrack(motor, runtime);
                    if (status != EMM42_MOTOR_STATUS_OK)
                    {
                        ConveyorMotorService_ReportDriverError("track command", status);
                    }
                }
            }
            break;

        case CONVEYOR_MOTOR_MODE_STOP:
        default:
            status = ConveyorMotorService_ApplyStop(motor, runtime);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("stop command", status);
            }
            break;
    }
}

/**
 * @brief 处理一条发给电机服务的串口命令。
 * @param command_buffer 已经规范化后的命令字符串。
 * @return uint8_t 1 表示已处理，0 表示不是本模块命令。
 */
uint8_t ConveyorMotorService_HandleCommand(const char *command_buffer)
{
    ConveyorMotor_Command_t command;
    ConveyorMotor_RuntimeSnapshot_t snapshot;
    int32_t error_px = 0;
    uint8_t enable_flag = 0U;

    if (command_buffer == NULL)
    {
        return 0U;
    }

    if (strcmp(command_buffer, "BELTSTOP") == 0)
    {
        command.type = CONVEYOR_MOTOR_COMMAND_STOP;
        command.error_px = 0;
        if (ConveyorMotorService_PostCommand(&command) != 0U)
        {
            my_printf(&huart1, "[OK][BELT] Mode set to STOP.\r\n");
        }
        else
        {
            my_printf(&huart1, "[ERROR][BELT] Service not ready.\r\n");
        }
        return 1U;
    }

    if (strcmp(command_buffer, "BELTSCAN") == 0)
    {
        command.type = CONVEYOR_MOTOR_COMMAND_SCAN;
        command.error_px = 0;
        if (ConveyorMotorService_PostCommand(&command) != 0U)
        {
            my_printf(&huart1, "[OK][BELT] Mode set to SCAN.\r\n");
        }
        else
        {
            my_printf(&huart1, "[ERROR][BELT] Service not ready.\r\n");
        }
        return 1U;
    }

    if (strcmp(command_buffer, "BELTINFO") == 0)
    {
        taskENTER_CRITICAL();
        snapshot = g_conveyor_motor_runtime_snapshot;
        taskEXIT_CRITICAL();

        my_printf(&huart1,
                  "[INFO][BELT] desired=%s, applied=%s, error=%ld px, speed=%u rpm, dir=%s, stable=%u, centered=%u\r\n",
                  ConveyorMotorService_GetModeName(snapshot.desired_mode),
                  ConveyorMotorService_GetModeName(snapshot.applied_mode),
                  (long)snapshot.latest_error_px,
                  (unsigned int)snapshot.applied_speed_rpm,
                  (snapshot.applied_direction == EMM42_MOTOR_DIRECTION_CW) ? "CW" : "CCW",
                  (unsigned int)snapshot.center_stable_count,
                  (unsigned int)snapshot.centered_flag);
        return 1U;
    }

    if (ConveyorMotorService_ParseTrackCommand(command_buffer, &error_px) != 0U)
    {
        command.type = CONVEYOR_MOTOR_COMMAND_TRACK;
        command.error_px = error_px;
        (void)ConveyorMotorService_PostCommand(&command);
        return 1U;
    }

    if (ConveyorMotorService_ParseEnableCommand(command_buffer, &enable_flag, &error_px) != 0U)
    {
        command.type = (enable_flag == 0U) ? CONVEYOR_MOTOR_COMMAND_SCAN : CONVEYOR_MOTOR_COMMAND_TRACK;
        command.error_px = error_px;
        (void)ConveyorMotorService_PostCommand(&command);
        return 1U;
    }

    if (ConveyorMotorService_ParseCameraCommand(command_buffer, &enable_flag, &error_px) != 0U)
    {
        command.type = (enable_flag == 0U) ? CONVEYOR_MOTOR_COMMAND_SCAN : CONVEYOR_MOTOR_COMMAND_TRACK;
        command.error_px = error_px;
        (void)ConveyorMotorService_PostCommand(&command);
        return 1U;
    }

    return 0U;
}

/**
 * @brief 传送带 Emm42 电机任务。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 任务职责：
 * 1. 初始化队列和电机驱动；
 * 2. 独占 `USART2` 发送运动命令；
 * 3. 执行三态控制逻辑；
 * 4. 周期性同步运行时快照，供 `BELTINFO` 查询。
 */
void ConveyorMotorService_Task(void *argument)
{
    EMM42_MotorHandle_t motor;
    ConveyorMotor_Command_t command;
    ConveyorMotor_Runtime_t runtime =
    {
        CONVEYOR_MOTOR_MODE_STOP,
        CONVEYOR_MOTOR_MODE_STOP,
        0,
        0U,
        EMM42_MOTOR_DIRECTION_CW,
        0U,
        0U,
        0U,
        0U,
        0U
    };
    EMM42_MotorStatus_t status;
    const char *failed_stage = "motor init";

    (void)argument;

    if (g_conveyor_motor_command_queue == NULL)
    {
        g_conveyor_motor_command_queue = xQueueCreate(CONVEYOR_MOTOR_COMMAND_QUEUE_LENGTH,
                                                      sizeof(ConveyorMotor_Command_t));
    }

    if (g_conveyor_motor_command_queue == NULL)
    {
        my_printf(&huart1, "[ERROR][BELT] Command queue create failed.\r\n");
        for (;;)
        {
            osDelay(1000U);
        }
    }

    EMM42_MotorLoadDefaultConfig(&motor, &huart2);

    for (;;)
    {
        failed_stage = "motor init";
        status = EMM42_MotorInit(&motor);
        if (status == EMM42_MOTOR_STATUS_OK)
        {
            failed_stage = "startup config";
            status = ConveyorMotorService_ApplyStartupConfig(&motor, &failed_stage);
        }

        if (status == EMM42_MOTOR_STATUS_OK)
        {
            failed_stage = "motor enable";
            status = EMM42_MotorSetEnable(&motor, true, false);
        }

        if (status == EMM42_MOTOR_STATUS_OK)
        {
            /*
             * 启动阶段再补一条“立即停止”，
             * 确保即便电机上一次掉电前仍在运动，也能回到已知静止态。
             */
            failed_stage = "startup stop";
            status = EMM42_MotorStopNow(&motor, false);
        }

        if (status == EMM42_MOTOR_STATUS_OK)
        {
            break;
        }

        ConveyorMotorService_ReportDriverError(failed_stage, status);
        osDelay(1000U);
    }

    runtime.desired_mode =
        (CONVEYOR_MOTOR_STARTUP_SCAN_ENABLE != 0U) ? CONVEYOR_MOTOR_MODE_SCAN : CONVEYOR_MOTOR_MODE_STOP;
    runtime.applied_mode = CONVEYOR_MOTOR_MODE_STOP;
    runtime.applied_speed_rpm = 0U;
    runtime.applied_direction = EMM42_MOTOR_DIRECTION_CW;
    runtime.latest_error_px = 0;
    runtime.center_stable_count = 0U;
    runtime.centered_flag = 0U;
    runtime.fresh_track_sample_flag = 0U;
    runtime.track_timeout_reported_flag = 0U;
    runtime.last_track_update_tick = xTaskGetTickCount();
    ConveyorMotorService_UpdateSnapshot(&runtime);

    ConveyorMotorService_ReportReady();

    for (;;)
    {
        if (xQueueReceive(g_conveyor_motor_command_queue,
                          &command,
                          pdMS_TO_TICKS(CONVEYOR_MOTOR_CONTROL_PERIOD_MS)) == pdPASS)
        {
            ConveyorMotorService_HandleQueuedCommand(&command, &runtime);
        }

        ConveyorMotorService_ControlStep(&motor, &runtime);
        ConveyorMotorService_UpdateSnapshot(&runtime);
    }
}
