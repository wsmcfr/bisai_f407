#include "camera_motor_service.h"

#include "binary_protocol_service.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "emm42_motor.h"
#include "queue.h"
#include "uart_command.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

/**
 * @brief 摄像头运动电机文本命令和 Emm42 地址速查。
 *
 * 通信链路：
 * 1. 用户/MP157 通过 USART1 发送 ASCII 文本命令，命令先由 `weight_service.c` 统一取出并规范化；
 * 2. 本文件只处理 `CAM...` 前缀命令，不直接读取 USART1 DMA 缓存；
 * 3. 摄像头两个运动电机共用 USART6，PC6(TX) 接两个 Emm42 RX，PC7(RX) 接两个 Emm42 TX，115200 8N1，必须共地；
 * 4. 摄像头前进/后退轴电机地址固定为 `0x02`；
 * 5. 摄像头上下轴电机地址固定为 `0x03`；
 * 6. 同一条 USART6 总线上禁止使用广播运动命令，否则两个摄像头轴可能同时动作。
 *
 * 用户可发送的 CAM 命令：
 * | 命令 | 参数含义 | 电机效果 | 典型返回/观察方式 |
 * | --- | --- | --- | --- |
 * | `CAMINFO` | 无参数 | 不改变电机，只打印串口、地址和最近动作 | `[INFO][CAM] ...` |
 * | `CAMSTOP` | 无参数 | 同时停止前进/后退轴和上下轴 | `[OK][CAM] Stop requested.` |
 * | `CAMFWD FORWARD [rpm]` | 前进/后退轴按工程约定前进方向点动，rpm 可省略 | 地址 0x02 电机转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMFWD BACKWARD [rpm]` | 前进/后退轴按工程约定后退方向点动，rpm 可省略 | 地址 0x02 电机反向转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMZ UP [rpm]` | 上下轴按工程约定上升方向点动，rpm 可省略 | 地址 0x03 电机转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMZ DOWN [rpm]` | 上下轴按工程约定下降方向点动，rpm 可省略 | 地址 0x03 电机反向转动 | 可看机械结构和 `CAMINFO` |
 *
 * 调试注意：
 * - 设置 Emm42 地址时建议一次只给一个摄像头电机上电，设置完成后贴标签；
 * - 如果 `CAMFWD` 命令让上下轴动作，说明电机地址或接线标签不匹配；
 * - 如果 `CAMZ` 命令让前后轴动作，说明两个电机地址可能接反；
 * - 如果两个轴同时动作，优先检查是否两个电机都还是默认地址 `0x01` 或误用了广播地址。
 */

/**
 * @brief 摄像头前进/后退轴 Emm42 地址。
 */
#define CAMERA_MOTOR_FORWARD_ADDRESS             (2U)

/**
 * @brief 摄像头上下轴 Emm42 地址。
 */
#define CAMERA_MOTOR_Z_ADDRESS                   (3U)

/**
 * @brief 摄像头电机任务主循环周期，单位毫秒。
 *
 * 当前任务只在收到队列命令时动作，周期等待用于降低空转占用。
 */
#define CAMERA_MOTOR_TASK_WAIT_MS                (20U)

/**
 * @brief 摄像头电机默认点动转速，单位 RPM。
 *
 * 摄像头运动用于微调，不适合高速动作，所以默认值比传送带扫描更低。
 */
#define CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM       (30U)

/**
 * @brief 摄像头电机点动最大转速，单位 RPM。
 *
 * 调试命令允许用户传入 rpm，但必须限制上限，避免误输入导致摄像头机构动作过猛。
 */
#define CAMERA_MOTOR_MAX_JOG_SPEED_RPM           (120U)

/**
 * @brief 摄像头电机加速度参数。
 *
 * 该参数直接进入 Emm42 速度模式命令的 `acc` 字段。
 */
#define CAMERA_MOTOR_ACCEL                       (5U)

/**
 * @brief 启动阶段配置命令之间的间隔，单位毫秒。
 *
 * 两个电机共用 USART6，初始化时需要给驱动器留出短暂处理时间。
 */
#define CAMERA_MOTOR_STARTUP_CONFIG_GAP_MS       (20U)

/**
 * @brief 摄像头电机命令队列长度。
 *
 * 这里使用长度为 1 的覆盖队列，保证调试点动始终执行最新意图，不堆积旧点动命令。
 */
#define CAMERA_MOTOR_COMMAND_QUEUE_LENGTH        (1U)

/**
 * @brief 摄像头运动轴选择。
 */
typedef enum
{
    CAMERA_MOTOR_AXIS_FORWARD = 0, /* 摄像头前进/后退轴，对应 USART6 上地址 0x02 的 Emm42 电机。 */
    CAMERA_MOTOR_AXIS_Z            /* 摄像头上下轴，对应 USART6 上地址 0x03 的 Emm42 电机。 */
} CameraMotor_Axis_t;

/**
 * @brief 摄像头电机内部命令类型。
 */
typedef enum
{
    CAMERA_MOTOR_COMMAND_STOP_ALL = 0, /* 队列命令：停止摄像头两个运动轴。 */
    CAMERA_MOTOR_COMMAND_JOG           /* 队列命令：指定某一个摄像头运动轴按给定方向和速度点动。 */
} CameraMotor_CommandType_t;

/**
 * @brief 摄像头电机任务队列命令。
 */
typedef struct
{
    CameraMotor_CommandType_t type;       /* 命令类型，用于区分停止两个轴还是点动某一个轴。 */
    CameraMotor_Axis_t axis;              /* 目标轴，点动命令使用；停止全部命令会忽略该字段。 */
    EMM42_MotorDirection_t direction;     /* 目标方向，点动命令使用；实际前进/上升含义需要结合机械安装确认。 */
    uint16_t speed_rpm;                   /* 目标转速，单位 RPM，点动命令使用，已在投递前限制到安全范围。 */
} CameraMotor_Command_t;

/**
 * @brief 摄像头电机对外观察快照。
 */
typedef struct
{
    CameraMotor_Axis_t last_axis;          /* 最近一次动作的轴，用于 `CAMINFO` 判断刚才控制的是前后轴还是上下轴。 */
    EMM42_MotorDirection_t last_direction; /* 最近一次动作方向，用于现场核对方向映射是否正确。 */
    uint16_t last_speed_rpm;               /* 最近一次下发的速度，单位 RPM，0 表示最近动作是停止。 */
    uint8_t forward_initialized;           /* 前进/后退轴初始化标志，1 表示地址 0x02 句柄已经通过初始化和启动配置。 */
    uint8_t z_initialized;                 /* 上下轴初始化标志，1 表示地址 0x03 句柄已经通过初始化和启动配置。 */
} CameraMotor_RuntimeSnapshot_t;

/**
 * @brief 摄像头电机服务命令队列。
 *
 * 由 USART1 命令分发入口写入，由摄像头电机任务独占读取。
 */
static QueueHandle_t g_camera_motor_command_queue = NULL;

/**
 * @brief 摄像头电机运行快照。
 *
 * `CAMINFO` 会在称重任务上下文读取该快照，因此更新和读取时都要进入临界区。
 */
static CameraMotor_RuntimeSnapshot_t g_camera_motor_snapshot =
{
    CAMERA_MOTOR_AXIS_FORWARD,
    EMM42_MOTOR_DIRECTION_CW,
    0U,
    0U,
    0U
};

/**
 * @brief 跳过命令参数中的空格和制表符。
 * @param cursor 字符串游标指针，不能为空。
 *
 * 该函数只移动游标，不修改原始字符串，便于多个解析函数复用。
 */
static void CameraMotorService_SkipSpaces(const char **cursor)
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
 * @brief 解析一个无符号十进制转速。
 * @param cursor 当前字符串游标，不能为空。
 * @param value 输出转速，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示没有合法数字。
 *
 * 该函数用于解析可选 rpm 参数，调用者负责决定没有 rpm 时是否使用默认值。
 */
static uint8_t CameraMotorService_ParseOptionalU16(const char **cursor, uint16_t *value)
{
    char *end_pointer;
    unsigned long parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return 0U;
    }

    CameraMotorService_SkipSpaces(cursor);
    if (**cursor == '\0')
    {
        return 0U;
    }

    parsed_value = strtoul(*cursor, &end_pointer, 10);
    if (*cursor == end_pointer)
    {
        return 0U;
    }

    if (parsed_value > 65535UL)
    {
        parsed_value = 65535UL;
    }

    *value = (uint16_t)parsed_value;
    *cursor = end_pointer;
    return 1U;
}

/**
 * @brief 把用户传入的转速限制到摄像头机构允许范围。
 * @param speed_rpm 用户传入或默认的转速，单位 RPM。
 * @return uint16_t 限幅后的转速，单位 RPM。
 */
static uint16_t CameraMotorService_LimitJogSpeed(uint16_t speed_rpm)
{
    if (speed_rpm == 0U)
    {
        return CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM;
    }

    if (speed_rpm > CAMERA_MOTOR_MAX_JOG_SPEED_RPM)
    {
        return CAMERA_MOTOR_MAX_JOG_SPEED_RPM;
    }

    return speed_rpm;
}

/**
 * @brief 向摄像头电机任务投递一条覆盖旧命令的新命令。
 * @param command 待投递命令，不能为空。
 * @return uint8_t 1 表示投递成功，0 表示任务队列尚未创建。
 */
static uint8_t CameraMotorService_PostCommand(const CameraMotor_Command_t *command)
{
    if ((command == NULL) || (g_camera_motor_command_queue == NULL))
    {
        return 0U;
    }

    (void)xQueueOverwrite(g_camera_motor_command_queue, command);
    return 1U;
}

/**
 * @brief 请求摄像头前进/后退轴点动。
 * @param forward_flag 1 表示前进方向，0 表示后退方向。
 * @param speed_rpm 点动速度，单位 RPM，传 0 使用默认值。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardJog(uint8_t forward_flag, uint16_t speed_rpm)
{
    CameraMotor_Command_t command;

    command.type = CAMERA_MOTOR_COMMAND_JOG;
    command.axis = CAMERA_MOTOR_AXIS_FORWARD;
    command.direction = (forward_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 请求摄像头上下轴点动。
 * @param up_flag 1 表示向上方向，0 表示向下方向。
 * @param speed_rpm 点动速度，单位 RPM，传 0 使用默认值。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestZJog(uint8_t up_flag, uint16_t speed_rpm)
{
    CameraMotor_Command_t command;

    command.type = CAMERA_MOTOR_COMMAND_JOG;
    command.axis = CAMERA_MOTOR_AXIS_Z;
    command.direction = (up_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 请求摄像头两个运动轴立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestStopAll(void)
{
    CameraMotor_Command_t command;

    command.type = CAMERA_MOTOR_COMMAND_STOP_ALL;
    command.axis = CAMERA_MOTOR_AXIS_FORWARD;
    command.direction = EMM42_MOTOR_DIRECTION_CW;
    command.speed_rpm = 0U;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 返回轴名称，便于串口日志输出。
 * @param axis 摄像头运动轴。
 * @return const char* 轴名称字符串。
 */
static const char *CameraMotorService_GetAxisName(CameraMotor_Axis_t axis)
{
    switch (axis)
    {
        case CAMERA_MOTOR_AXIS_Z:
            return "Z";

        case CAMERA_MOTOR_AXIS_FORWARD:
        default:
            return "FORWARD";
    }
}

/**
 * @brief 解析 `CAMFWD ...` 或 `CAMZ ...` 点动命令。
 * @param command_buffer 已规范化后的命令字符串，不能为空。
 * @param axis 输出目标轴，不能为空。
 * @param direction 输出目标方向，不能为空。
 * @param speed_rpm 输出目标速度，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示不是合法点动命令。
 *
 * 支持格式：
 * - `CAMFWD FORWARD [rpm]`
 * - `CAMFWD BACKWARD [rpm]`
 * - `CAMZ UP [rpm]`
 * - `CAMZ DOWN [rpm]`
 */
static uint8_t CameraMotorService_ParseJogCommand(const char *command_buffer,
                                                  CameraMotor_Axis_t *axis,
                                                  EMM42_MotorDirection_t *direction,
                                                  uint16_t *speed_rpm)
{
    const char *cursor;
    uint16_t parsed_speed = 0U;

    if ((command_buffer == NULL) || (axis == NULL) || (direction == NULL) || (speed_rpm == NULL))
    {
        return 0U;
    }

    if (strncmp(command_buffer, "CAMFWD", 6U) == 0)
    {
        *axis = CAMERA_MOTOR_AXIS_FORWARD;
        cursor = command_buffer + 6U;
        CameraMotorService_SkipSpaces(&cursor);

        if (strncmp(cursor, "FORWARD", 7U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CW;
            cursor += 7U;
        }
        else if (strncmp(cursor, "BACKWARD", 8U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CCW;
            cursor += 8U;
        }
        else
        {
            return 0U;
        }
    }
    else if (strncmp(command_buffer, "CAMZ", 4U) == 0)
    {
        *axis = CAMERA_MOTOR_AXIS_Z;
        cursor = command_buffer + 4U;
        CameraMotorService_SkipSpaces(&cursor);

        if (strncmp(cursor, "UP", 2U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CW;
            cursor += 2U;
        }
        else if (strncmp(cursor, "DOWN", 4U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CCW;
            cursor += 4U;
        }
        else
        {
            return 0U;
        }
    }
    else
    {
        return 0U;
    }

    if (CameraMotorService_ParseOptionalU16(&cursor, &parsed_speed) == 0U)
    {
        parsed_speed = CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM;
    }

    CameraMotorService_SkipSpaces(&cursor);
    if (*cursor != '\0')
    {
        return 0U;
    }

    *speed_rpm = CameraMotorService_LimitJogSpeed(parsed_speed);
    return 1U;
}

/**
 * @brief 处理一条摄像头电机文本命令。
 * @param command_buffer 已规范化后的命令字符串。
 * @return uint8_t 1 表示命令已处理，0 表示不是本模块命令。
 */
uint8_t CameraMotorService_HandleCommand(const char *command_buffer)
{
    CameraMotor_RuntimeSnapshot_t snapshot;
    CameraMotor_Axis_t axis;
    EMM42_MotorDirection_t direction;
    uint16_t speed_rpm;

    if (command_buffer == NULL)
    {
        return 0U;
    }

    if (strcmp(command_buffer, "CAMINFO") == 0)
    {
        taskENTER_CRITICAL();
        snapshot = g_camera_motor_snapshot;
        taskEXIT_CRITICAL();

        my_printf(&huart1,
                  "[INFO][CAM] USART6=PC6/PC7, forward_addr=%u init=%u, z_addr=%u init=%u, last_axis=%s, last_dir=%s, last_speed=%u rpm\r\n",
                  (unsigned int)CAMERA_MOTOR_FORWARD_ADDRESS,
                  (unsigned int)snapshot.forward_initialized,
                  (unsigned int)CAMERA_MOTOR_Z_ADDRESS,
                  (unsigned int)snapshot.z_initialized,
                  CameraMotorService_GetAxisName(snapshot.last_axis),
                  (snapshot.last_direction == EMM42_MOTOR_DIRECTION_CW) ? "CW" : "CCW",
                  (unsigned int)snapshot.last_speed_rpm);
        return 1U;
    }

    if (strcmp(command_buffer, "CAMSTOP") == 0)
    {
        if (CameraMotorService_RequestStopAll() != 0U)
        {
            my_printf(&huart1, "[OK][CAM] Stop requested.\r\n");
        }
        else
        {
            my_printf(&huart1, "[ERROR][CAM] Service not ready.\r\n");
        }
        return 1U;
    }

    if (CameraMotorService_ParseJogCommand(command_buffer, &axis, &direction, &speed_rpm) != 0U)
    {
        CameraMotor_Command_t command;

        command.type = CAMERA_MOTOR_COMMAND_JOG;
        command.axis = axis;
        command.direction = direction;
        command.speed_rpm = speed_rpm;

        if (CameraMotorService_PostCommand(&command) != 0U)
        {
            my_printf(&huart1,
                      "[OK][CAM] Jog requested. axis=%s, dir=%s, speed=%u rpm\r\n",
                      CameraMotorService_GetAxisName(axis),
                      (direction == EMM42_MOTOR_DIRECTION_CW) ? "CW" : "CCW",
                      (unsigned int)speed_rpm);
        }
        else
        {
            my_printf(&huart1, "[ERROR][CAM] Service not ready.\r\n");
        }
        return 1U;
    }

    return 0U;
}

/**
 * @brief 把当前初始化状态写入共享快照。
 * @param forward_initialized 前进/后退轴初始化状态。
 * @param z_initialized 上下轴初始化状态。
 */
static void CameraMotorService_UpdateInitSnapshot(uint8_t forward_initialized, uint8_t z_initialized)
{
    taskENTER_CRITICAL();
    g_camera_motor_snapshot.forward_initialized = forward_initialized;
    g_camera_motor_snapshot.z_initialized = z_initialized;
    taskEXIT_CRITICAL();
}

/**
 * @brief 把最近动作写入共享快照。
 * @param axis 最近动作轴。
 * @param direction 最近动作方向。
 * @param speed_rpm 最近动作速度，单位 RPM。
 */
static void CameraMotorService_UpdateActionSnapshot(CameraMotor_Axis_t axis,
                                                    EMM42_MotorDirection_t direction,
                                                    uint16_t speed_rpm)
{
    taskENTER_CRITICAL();
    g_camera_motor_snapshot.last_axis = axis;
    g_camera_motor_snapshot.last_direction = direction;
    g_camera_motor_snapshot.last_speed_rpm = speed_rpm;
    taskEXIT_CRITICAL();
}

/**
 * @brief 对单个摄像头 Emm42 电机执行启动初始化。
 * @param motor 电机句柄，不能为空。
 * @param address 目标 Emm42 地址。
 * @param name 日志中的轴名称，不能为空。
 * @return EMM42_MotorStatus_t 初始化结果。
 *
 * 初始化顺序：
 * 1. 给句柄写入当前电机地址；
 * 2. 校验句柄；
 * 3. 使能电机；
 * 4. 立即停止，保证上电后处在静止态。
 */
static EMM42_MotorStatus_t CameraMotorService_InitOneMotor(EMM42_MotorHandle_t *motor,
                                                           uint8_t address,
                                                           const char *name)
{
    EMM42_MotorStatus_t status;

    if ((motor == NULL) || (name == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    motor->address = address;

    status = EMM42_MotorInit(motor);
    if (status != EMM42_MOTOR_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          0U);
        my_printf(&huart1, "[ERROR][CAM] %s init failed, status=%d\r\n", name, (int)status);
        return status;
    }

    status = EMM42_MotorSetEnable(motor, true, false);
    if (status != EMM42_MOTOR_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          0U);
        my_printf(&huart1, "[ERROR][CAM] %s enable failed, status=%d\r\n", name, (int)status);
        return status;
    }

    osDelay(CAMERA_MOTOR_STARTUP_CONFIG_GAP_MS);

    status = EMM42_MotorStopNow(motor, false);
    if (status != EMM42_MOTOR_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          0U);
        my_printf(&huart1, "[ERROR][CAM] %s startup stop failed, status=%d\r\n", name, (int)status);
        return status;
    }

    osDelay(CAMERA_MOTOR_STARTUP_CONFIG_GAP_MS);
    return EMM42_MOTOR_STATUS_OK;
}

/**
 * @brief 按队列命令执行一次摄像头电机动作。
 * @param command 队列命令，不能为空。
 * @param forward_motor 摄像头前进/后退轴电机句柄，不能为空。
 * @param z_motor 摄像头上下轴电机句柄，不能为空。
 */
static void CameraMotorService_ApplyCommand(const CameraMotor_Command_t *command,
                                            const EMM42_MotorHandle_t *forward_motor,
                                            const EMM42_MotorHandle_t *z_motor)
{
    const EMM42_MotorHandle_t *target_motor;
    EMM42_MotorStatus_t status;

    if ((command == NULL) || (forward_motor == NULL) || (z_motor == NULL))
    {
        return;
    }

    if (command->type == CAMERA_MOTOR_COMMAND_STOP_ALL)
    {
        status = EMM42_MotorStopNow(forward_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1, "[ERROR][CAM] forward stop failed, status=%d\r\n", (int)status);
        }

        status = EMM42_MotorStopNow(z_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1, "[ERROR][CAM] z stop failed, status=%d\r\n", (int)status);
        }

        CameraMotorService_UpdateActionSnapshot(CAMERA_MOTOR_AXIS_FORWARD,
                                                EMM42_MOTOR_DIRECTION_CW,
                                                0U);
        return;
    }

    target_motor = (command->axis == CAMERA_MOTOR_AXIS_Z) ? z_motor : forward_motor;
    status = EMM42_MotorSetVelocity(target_motor,
                                    command->direction,
                                    command->speed_rpm,
                                    CAMERA_MOTOR_ACCEL,
                                    false);
    if (status != EMM42_MOTOR_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          0U);
        my_printf(&huart1,
                  "[ERROR][CAM] jog failed. axis=%s, status=%d\r\n",
                  CameraMotorService_GetAxisName(command->axis),
                  (int)status);
        return;
    }

    CameraMotorService_UpdateActionSnapshot(command->axis,
                                            command->direction,
                                            command->speed_rpm);
}

/**
 * @brief 摄像头运动 Emm42 电机服务任务。
 * @param argument FreeRTOS 任务参数，当前未使用。
 *
 * 任务职责：
 * 1. 创建命令队列；
 * 2. 在同一条 USART6 总线上初始化地址 0x02 和 0x03 两个电机；
 * 3. 周期等待最新点动或停止命令；
 * 4. 串行发送 Emm42 命令，保证两个摄像头电机共线时不会发生发送交叉。
 */
void CameraMotorService_Task(void *argument)
{
    EMM42_MotorHandle_t forward_motor;
    EMM42_MotorHandle_t z_motor;
    CameraMotor_Command_t command;
    EMM42_MotorStatus_t forward_status;
    EMM42_MotorStatus_t z_status;

    (void)argument;

    if (g_camera_motor_command_queue == NULL)
    {
        g_camera_motor_command_queue = xQueueCreate(CAMERA_MOTOR_COMMAND_QUEUE_LENGTH,
                                                    sizeof(CameraMotor_Command_t));
    }

    if (g_camera_motor_command_queue == NULL)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
        BinaryProtocolService_ReportFault(1U,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          0,
                                          0U);
        my_printf(&huart1, "[ERROR][CAM] Command queue create failed.\r\n");
        for (;;)
        {
            osDelay(1000U);
        }
    }

    EMM42_MotorLoadDefaultConfig(&forward_motor, &huart6);
    EMM42_MotorLoadDefaultConfig(&z_motor, &huart6);

    for (;;)
    {
        forward_status = CameraMotorService_InitOneMotor(&forward_motor,
                                                         CAMERA_MOTOR_FORWARD_ADDRESS,
                                                         "forward");
        z_status = CameraMotorService_InitOneMotor(&z_motor,
                                                   CAMERA_MOTOR_Z_ADDRESS,
                                                   "z");

        CameraMotorService_UpdateInitSnapshot((forward_status == EMM42_MOTOR_STATUS_OK) ? 1U : 0U,
                                              (z_status == EMM42_MOTOR_STATUS_OK) ? 1U : 0U);

        if ((forward_status == EMM42_MOTOR_STATUS_OK) && (z_status == EMM42_MOTOR_STATUS_OK))
        {
            BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            break;
        }

        /*
         * 两个摄像头电机共用 USART6，任意一个初始化失败都可能是总线、地址或供电问题。
         * 这里不让任务退出，而是每秒重试，方便现场修正接线或地址后观察串口日志恢复。
         */
        osDelay(1000U);
    }

    my_printf(&huart1,
              "[OK][CAM] Camera motor service started. USART6=PC6/PC7, forward_addr=%u, z_addr=%u\r\n",
              (unsigned int)CAMERA_MOTOR_FORWARD_ADDRESS,
              (unsigned int)CAMERA_MOTOR_Z_ADDRESS);

    for (;;)
    {
        if (xQueueReceive(g_camera_motor_command_queue,
                          &command,
                          pdMS_TO_TICKS(CAMERA_MOTOR_TASK_WAIT_MS)) == pdPASS)
        {
            CameraMotorService_ApplyCommand(&command, &forward_motor, &z_motor);
        }
    }
}
