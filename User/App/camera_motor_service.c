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
 * 4. 摄像头左右轴电机现场默认地址为 `0x03`，该硬件通道复用旧前进/后退电机；
 * 5. 摄像头上下轴电机现场默认地址为 `0x02`；
 * 6. 同一条 USART6 总线上禁止使用广播运动命令，否则两个摄像头轴可能同时动作。
 *
 * 用户可发送的 CAM 命令：
 * | 命令 | 参数含义 | 电机效果 | 典型返回/观察方式 |
 * | --- | --- | --- | --- |
 * | `CAMINFO` | 无参数 | 不改变电机，只打印串口、地址和最近动作 | `[INFO][CAM] ...` |
 * | `CAMSTOP` | 无参数 | 同时停止左右轴和上下轴 | `[OK][CAM] Stop requested.` |
 * | `CAMLAT LEFT [rpm]` | 左右轴按工程约定左移方向点动，rpm 可省略 | 地址 0x03 电机转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMLAT RIGHT [rpm]` | 左右轴按工程约定右移方向点动，rpm 可省略 | 地址 0x03 电机反向转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMFWD FORWARD/BACKWARD [rpm]` | 旧前进/后退调试别名，兼容保留 | 实际仍控制左右轴 | 新调试优先使用 `CAMLAT` |
 * | `CAMZ UP [rpm]` | 上下轴按工程约定上升方向点动，rpm 可省略 | 地址 0x02 电机转动 | 可看机械结构和 `CAMINFO` |
 * | `CAMZ DOWN [rpm]` | 上下轴按工程约定下降方向点动，rpm 可省略 | 地址 0x02 电机反向转动 | 可看机械结构和 `CAMINFO` |
 *
 * 调试注意：
 * - 设置 Emm42 地址时建议一次只给一个摄像头电机上电，设置完成后贴标签；
 * - 如果 `CAMLAT` 命令让上下轴动作，说明电机地址或接线标签不匹配；
 * - 如果 `CAMZ` 命令让左右轴动作，说明两个电机地址可能接反；
 * - 如果两个轴同时动作，优先检查是否两个电机都还是默认地址 `0x01` 或误用了广播地址。
 */

/**
 * @brief 摄像头左右轴 Emm42 地址。
 */
#define CAMERA_MOTOR_LATERAL_ADDRESS             (3U)

/**
 * @brief 旧版前进/后退轴地址宏兼容别名。
 *
 * 新业务语义已经改为左右轴；保留该宏只为避免旧编译单元或临时调试代码失效。
 */
#define CAMERA_MOTOR_FORWARD_ADDRESS             CAMERA_MOTOR_LATERAL_ADDRESS

/**
 * @brief 摄像头左右轴默认最小步长，单位 step。
 *
 * 当前摄像头服务仍以速度点动为主，暂不直接使用该字段；
 * F4 先保存该参数，后续补相机轴位置步进命令时可以直接复用。
 */
#define CAMERA_MOTOR_LATERAL_MIN_STEP_DEFAULT    (5U)

/**
 * @brief 旧版前进/后退轴最小步长宏兼容别名。
 */
#define CAMERA_MOTOR_FORWARD_MIN_STEP_DEFAULT    CAMERA_MOTOR_LATERAL_MIN_STEP_DEFAULT

/**
 * @brief 摄像头上下轴 Emm42 地址。
 */
#define CAMERA_MOTOR_Z_ADDRESS                   (2U)

/**
 * @brief 摄像头上下轴默认最小步长，单位 step。
 */
#define CAMERA_MOTOR_Z_MIN_STEP_DEFAULT          (5U)

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
 * MP157 参数页要求三台电机速度都能设置 `0~5000 rpm`，
 * 底层 Emm42 驱动也支持该范围，因此这里不再保留旧的 120 rpm 截断。
 */
#define CAMERA_MOTOR_MAX_JOG_SPEED_RPM           (5000U)

/**
 * @brief 摄像头电机参数页最小步长最大值，单位 step。
 */
#define CAMERA_MOTOR_CONFIG_MAX_MIN_STEP         (10000U)

/**
 * @brief Emm42 普通站号地址允许范围。
 */
#define CAMERA_MOTOR_ADDRESS_MIN                 (1U)
#define CAMERA_MOTOR_ADDRESS_MAX                 (247U)

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
 * 这里使用短 FIFO 队列，保证 STEPPER_PARAM_SET 配置命令不会被下一条手动运动覆盖。
 * STOP 命令单独走队首优先投递，确保手动连续运动时停止意图尽快执行。
 */
#define CAMERA_MOTOR_COMMAND_QUEUE_LENGTH        (4U)

/**
 * @brief 摄像头运动轴选择。
 */
typedef enum
{
    CAMERA_MOTOR_AXIS_LATERAL = 0,              /* 摄像头左右轴，对应 USART6 上现场默认地址 0x03 的 Emm42 电机。 */
    CAMERA_MOTOR_AXIS_FORWARD = CAMERA_MOTOR_AXIS_LATERAL, /* 旧前进/后退枚举兼容别名，实际语义为左右轴。 */
    CAMERA_MOTOR_AXIS_Z                         /* 摄像头上下轴，对应 USART6 上现场默认地址 0x02 的 Emm42 电机。 */
} CameraMotor_Axis_t;

/**
 * @brief 摄像头电机内部命令类型。
 */
typedef enum
{
    CAMERA_MOTOR_COMMAND_STOP_ALL = 0, /* 队列命令：停止摄像头两个运动轴。 */
    CAMERA_MOTOR_COMMAND_JOG,          /* 队列命令：指定某一个摄像头运动轴按给定方向和速度点动。 */
    CAMERA_MOTOR_COMMAND_POSITION,     /* 队列命令：指定某一个摄像头运动轴按相对位置模式移动固定步数。 */
    CAMERA_MOTOR_COMMAND_SET_ZERO,     /* 队列命令：指定某一个摄像头运动轴停止后把当前位置设为零点。 */
    CAMERA_MOTOR_COMMAND_CONFIG        /* 队列命令：更新两个摄像头运动轴的运行时参数。 */
} CameraMotor_CommandType_t;

/**
 * @brief 单个摄像头运动轴的运行时参数。
 */
typedef struct
{
    uint8_t address;                   /* Emm42 地址，两个摄像头轴共用 USART6 时必须互不相同。 */
    uint16_t min_step;                 /* 最小步长，单位 step，当前保存给后续位置控制使用。 */
    uint16_t normal_speed_rpm;         /* 默认点动速度，单位 RPM，0 表示默认点动时保持停止。 */
    int8_t direction;                  /* 方向映射，1 保持逻辑方向，-1 反转逻辑方向。 */
} CameraMotor_RuntimeConfig_t;

/**
 * @brief 两个摄像头运动轴的运行时参数集合。
 */
typedef struct
{
    CameraMotor_RuntimeConfig_t lateral; /* 摄像头左右轴运行时参数。 */
    CameraMotor_RuntimeConfig_t z;       /* 摄像头上下轴运行时参数。 */
} CameraMotor_RuntimeConfigSet_t;

/**
 * @brief 摄像头电机任务队列命令。
 */
typedef struct
{
    CameraMotor_CommandType_t type;       /* 命令类型，用于区分停止两个轴还是点动某一个轴。 */
    CameraMotor_Axis_t axis;              /* 目标轴，点动命令使用；停止全部命令会忽略该字段。 */
    EMM42_MotorDirection_t direction;     /* 逻辑方向，左右轴 CW=右移/CCW=左移，上下轴 CW=上升/CCW=下降，任务内会再套用运行时方向映射。 */
    uint16_t speed_rpm;                   /* 请求转速，单位 RPM；0 表示使用当前轴的运行时默认速度。 */
    uint32_t pulse_count;                 /* 位置模式相对移动步数，单位 step；只有 POSITION 命令使用。 */
    uint32_t stop_epoch;                  /* STOP 代际编号，用于丢弃 STOP 之前还没执行的旧运动命令。 */
    CameraMotor_RuntimeConfigSet_t config; /* CONFIG 命令携带的新运行时参数，其它命令忽略该字段。 */
} CameraMotor_Command_t;

/**
 * @brief 摄像头电机任务内部运行时状态。
 */
typedef struct
{
    CameraMotor_RuntimeConfigSet_t config; /* 当前生效的两个摄像头轴运行时参数，只在摄像头任务内直接读写。 */
} CameraMotor_Runtime_t;

/**
 * @brief 摄像头电机对外观察快照。
 */
typedef struct
{
    CameraMotor_Axis_t last_axis;          /* 最近一次动作的轴，用于 `CAMINFO` 判断刚才控制的是左右轴还是上下轴。 */
    EMM42_MotorDirection_t last_direction; /* 最近一次动作方向，用于现场核对方向映射是否正确。 */
    uint16_t last_speed_rpm;               /* 最近一次下发的速度，单位 RPM，0 表示最近动作是停止。 */
    uint8_t lateral_initialized;           /* 左右轴初始化标志，1 表示现场地址 0x03 句柄已经通过初始化和启动配置。 */
    uint8_t z_initialized;                 /* 上下轴初始化标志，1 表示现场地址 0x02 句柄已经通过初始化和启动配置。 */
    uint8_t lateral_address;               /* 当前左右轴运行时地址。 */
    uint8_t z_address;                     /* 当前上下轴运行时地址。 */
    uint16_t lateral_min_step;             /* 当前左右轴最小步长，单位 step。 */
    uint16_t z_min_step;                   /* 当前上下轴最小步长，单位 step。 */
    uint16_t lateral_normal_speed_rpm;     /* 当前左右轴默认点动速度，单位 RPM。 */
    uint16_t z_normal_speed_rpm;           /* 当前上下轴默认点动速度，单位 RPM。 */
    int8_t lateral_direction;              /* 当前左右轴方向映射。 */
    int8_t z_direction;                    /* 当前上下轴方向映射。 */
} CameraMotor_RuntimeSnapshot_t;

/**
 * @brief 摄像头电机服务命令队列。
 *
 * 由 USART1 命令分发入口写入，由摄像头电机任务独占读取。
 */
static QueueHandle_t g_camera_motor_command_queue = NULL;

/**
 * @brief 摄像头电机 STOP 代际编号。
 *
 * 协议层收到 STOP 后会递增该编号；普通 JOG/POSITION/HOME 命令入队时记录当前编号。
 * 如果 STOP 插到队首后队列里还残留旧 JOG，摄像头任务会发现旧命令的 stop_epoch 落后，
 * 直接丢弃它，避免现场出现“按了停止但旧左右轴点动又继续执行”的现象。
 */
static volatile uint32_t g_camera_motor_stop_epoch = 1U;

/**
 * @brief 摄像头电机运行快照。
 *
 * `CAMINFO` 会在称重任务上下文读取该快照，因此更新和读取时都要进入临界区。
 */
static CameraMotor_RuntimeSnapshot_t g_camera_motor_snapshot =
{
    CAMERA_MOTOR_AXIS_LATERAL,
    EMM42_MOTOR_DIRECTION_CW,
    0U,
    0U,
    0U,
    CAMERA_MOTOR_LATERAL_ADDRESS,
    CAMERA_MOTOR_Z_ADDRESS,
    CAMERA_MOTOR_LATERAL_MIN_STEP_DEFAULT,
    CAMERA_MOTOR_Z_MIN_STEP_DEFAULT,
    CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM,
    CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM,
    1,
    1
};

/**
 * @brief 返回摄像头两个运动轴的默认运行时配置。
 * @return CameraMotor_RuntimeConfigSet_t 默认地址、最小步长、速度和方向映射。
 */
static CameraMotor_RuntimeConfigSet_t CameraMotorService_GetDefaultConfigSet(void)
{
    CameraMotor_RuntimeConfigSet_t config_set;

    config_set.lateral.address = CAMERA_MOTOR_LATERAL_ADDRESS;
    config_set.lateral.min_step = CAMERA_MOTOR_LATERAL_MIN_STEP_DEFAULT;
    config_set.lateral.normal_speed_rpm = CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM;
    config_set.lateral.direction = 1;

    config_set.z.address = CAMERA_MOTOR_Z_ADDRESS;
    config_set.z.min_step = CAMERA_MOTOR_Z_MIN_STEP_DEFAULT;
    config_set.z.normal_speed_rpm = CAMERA_MOTOR_DEFAULT_JOG_SPEED_RPM;
    config_set.z.direction = 1;

    return config_set;
}

/**
 * @brief 读取当前 STOP 代际编号。
 * @return uint32_t 当前 STOP 代际编号。
 *
 * 该值会被 USART1 协议任务和摄像头电机任务同时访问，因此读取时进入 FreeRTOS 临界区。
 */
static uint32_t CameraMotorService_ReadStopEpoch(void)
{
    uint32_t epoch;

    taskENTER_CRITICAL();
    epoch = g_camera_motor_stop_epoch;
    taskEXIT_CRITICAL();

    return epoch;
}

/**
 * @brief 递增并返回新的 STOP 代际编号。
 * @return uint32_t 新的 STOP 代际编号。
 *
 * STOP 是安全动作，每次 STOP 都开启一个新的代际；编号回绕到 0 时拉回 1，
 * 避免刚上电时默认清零的旧命令和新命令混淆。
 */
static uint32_t CameraMotorService_AdvanceStopEpoch(void)
{
    uint32_t epoch;

    taskENTER_CRITICAL();
    g_camera_motor_stop_epoch++;
    if (g_camera_motor_stop_epoch == 0U)
    {
        g_camera_motor_stop_epoch = 1U;
    }
    epoch = g_camera_motor_stop_epoch;
    taskEXIT_CRITICAL();

    return epoch;
}

/**
 * @brief 判断一条摄像头运动命令是否已经被后续 STOP 作废。
 * @param command 摄像头电机队列命令，不能为空。
 * @return uint8_t 1 表示应丢弃，0 表示可以继续执行。
 *
 * CONFIG 不属于运动命令，不能因为 STOP 被丢弃，否则会重新引入“参数 ACK 后未应用”的问题。
 * STOP_ALL 自身也不做旧命令判断，因为它正是用于提升 stop_epoch 的安全命令。
 */
static uint8_t CameraMotorService_IsStaleMotionCommand(const CameraMotor_Command_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    if ((command->type == CAMERA_MOTOR_COMMAND_STOP_ALL) ||
        (command->type == CAMERA_MOTOR_COMMAND_CONFIG))
    {
        return 0U;
    }

    return (command->stop_epoch != CameraMotorService_ReadStopEpoch()) ? 1U : 0U;
}

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
        return 0U;
    }

    if (speed_rpm > CAMERA_MOTOR_MAX_JOG_SPEED_RPM)
    {
        return CAMERA_MOTOR_MAX_JOG_SPEED_RPM;
    }

    return speed_rpm;
}

/**
 * @brief 向摄像头电机任务投递一条命令。
 * @param command 待投递命令，不能为空。
 * @return uint8_t 1 表示投递成功，0 表示任务队列尚未创建或队列已满。
 *
 * 普通 CONFIG/JOG/POSITION/HOME 命令按 FIFO 顺序进入队列，避免刚下发的运行时地址配置
 * 被后续手动动作覆盖。STOP_ALL 是安全动作，优先插到队首；若队列已满，先清掉未执行的
 * 普通命令再投递 STOP，保证连续运动可以被停止。
 */
static uint8_t CameraMotorService_PostCommand(const CameraMotor_Command_t *command)
{
    CameraMotor_Command_t queued_command;
    BaseType_t send_status;

    if ((command == NULL) || (g_camera_motor_command_queue == NULL))
    {
        return 0U;
    }

    queued_command = *command;

    if (queued_command.type == CAMERA_MOTOR_COMMAND_STOP_ALL)
    {
        queued_command.stop_epoch = CameraMotorService_AdvanceStopEpoch();
        send_status = xQueueSendToFront(g_camera_motor_command_queue, &queued_command, 0U);
        if (send_status != pdPASS)
        {
            (void)xQueueReset(g_camera_motor_command_queue);
            send_status = xQueueSendToFront(g_camera_motor_command_queue, &queued_command, 0U);
        }
    }
    else
    {
        queued_command.stop_epoch = CameraMotorService_ReadStopEpoch();
        send_status = xQueueSendToBack(g_camera_motor_command_queue, &queued_command, 0U);
    }

    return (send_status == pdPASS) ? 1U : 0U;
}

/**
 * @brief 请求摄像头左右轴点动。
 * @param right_flag 1 表示右移方向，0 表示左移方向。
 * @param speed_rpm 点动速度，单位 RPM，传 0 使用默认值。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestLateralJog(uint8_t right_flag, uint16_t speed_rpm)
{
    CameraMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_JOG;
    command.axis = CAMERA_MOTOR_AXIS_LATERAL;
    command.direction = (right_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 旧版前进/后退点动接口兼容包装。
 * @param forward_flag 旧语义中的前进标志，1 映射为右移，0 映射为左移。
 * @param speed_rpm 点动速度，单位 RPM，传 0 使用默认值。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardJog(uint8_t forward_flag, uint16_t speed_rpm)
{
    return CameraMotorService_RequestLateralJog(forward_flag, speed_rpm);
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

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_JOG;
    command.axis = CAMERA_MOTOR_AXIS_Z;
    command.direction = (up_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 请求摄像头左右轴按相对位置模式移动固定步数。
 * @param right_flag 1 表示右移方向，0 表示左移方向。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用默认值。
 * @param pulse_count 相对移动步数，单位 step。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或服务尚未就绪。
 */
uint8_t CameraMotorService_RequestLateralPosition(uint8_t right_flag,
                                                  uint16_t speed_rpm,
                                                  uint32_t pulse_count)
{
    CameraMotor_Command_t command;

    if (pulse_count == 0U)
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_POSITION;
    command.axis = CAMERA_MOTOR_AXIS_LATERAL;
    command.direction = (right_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    command.pulse_count = pulse_count;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 旧版前进/后退位置接口兼容包装。
 * @param forward_flag 旧语义中的前进标志，1 映射为右移，0 映射为左移。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用默认值。
 * @param pulse_count 相对移动步数，单位 step。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或服务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardPosition(uint8_t forward_flag,
                                                  uint16_t speed_rpm,
                                                  uint32_t pulse_count)
{
    return CameraMotorService_RequestLateralPosition(forward_flag, speed_rpm, pulse_count);
}

/**
 * @brief 请求摄像头上下轴按相对位置模式移动固定步数。
 * @param up_flag 1 表示向上方向，0 表示向下方向。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用默认值。
 * @param pulse_count 相对移动步数，单位 step。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或服务尚未就绪。
 */
uint8_t CameraMotorService_RequestZPosition(uint8_t up_flag,
                                            uint16_t speed_rpm,
                                            uint32_t pulse_count)
{
    CameraMotor_Command_t command;

    if (pulse_count == 0U)
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_POSITION;
    command.axis = CAMERA_MOTOR_AXIS_Z;
    command.direction = (up_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = CameraMotorService_LimitJogSpeed(speed_rpm);
    command.pulse_count = pulse_count;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 请求摄像头左右轴把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机任务尚未就绪。
 */
uint8_t CameraMotorService_RequestLateralSetCurrentPositionZero(void)
{
    CameraMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_SET_ZERO;
    command.axis = CAMERA_MOTOR_AXIS_LATERAL;
    command.direction = EMM42_MOTOR_DIRECTION_CW;
    command.speed_rpm = 0U;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 旧版前进/后退设零接口兼容包装。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机任务尚未就绪。
 */
uint8_t CameraMotorService_RequestForwardSetCurrentPositionZero(void)
{
    return CameraMotorService_RequestLateralSetCurrentPositionZero();
}

/**
 * @brief 请求摄像头上下轴把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示摄像头电机任务尚未就绪。
 */
uint8_t CameraMotorService_RequestZSetCurrentPositionZero(void)
{
    CameraMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_SET_ZERO;
    command.axis = CAMERA_MOTOR_AXIS_Z;
    command.direction = EMM42_MOTOR_DIRECTION_CW;
    command.speed_rpm = 0U;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 请求摄像头两个运动轴立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示服务尚未就绪。
 */
uint8_t CameraMotorService_RequestStopAll(void)
{
    CameraMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_STOP_ALL;
    command.axis = CAMERA_MOTOR_AXIS_LATERAL;
    command.direction = EMM42_MOTOR_DIRECTION_CW;
    command.speed_rpm = 0U;
    return CameraMotorService_PostCommand(&command);
}

/**
 * @brief 更新两个摄像头运动轴的运行时参数。
 * @param lateral_address 左右轴 Emm42 地址，允许 1~247。
 * @param lateral_min_step 左右轴最小步长，单位 step，允许 1~10000。
 * @param lateral_normal_speed_rpm 左右轴默认点动速度，单位 RPM，允许 0~5000。
 * @param lateral_direction 左右轴方向映射，1 表示保持逻辑方向，-1 表示反转逻辑方向。
 * @param z_address 上下轴 Emm42 地址，允许 1~247，不能和左右轴相同。
 * @param z_min_step 上下轴最小步长，单位 step，允许 1~10000。
 * @param z_normal_speed_rpm 上下轴默认点动速度，单位 RPM，允许 0~5000。
 * @param z_direction 上下轴方向映射，1 表示保持逻辑方向，-1 表示反转逻辑方向。
 * @return uint8_t 1 表示配置命令已投递，0 表示参数非法或任务队列尚未创建。
 *
 * 该接口只更新 F4 运行内存，不写 F4 Flash，也不修改 Emm42 驱动器 EEPROM。
 * 真正访问 `USART6` 的停止动作和句柄地址更新都在摄像头任务内完成。
 */
uint8_t CameraMotorService_RequestRuntimeConfig(uint8_t lateral_address,
                                                uint16_t lateral_min_step,
                                                uint16_t lateral_normal_speed_rpm,
                                                int8_t lateral_direction,
                                                uint8_t z_address,
                                                uint16_t z_min_step,
                                                uint16_t z_normal_speed_rpm,
                                                int8_t z_direction)
{
    CameraMotor_Command_t command;

    if ((lateral_address < CAMERA_MOTOR_ADDRESS_MIN) ||
        (lateral_address > CAMERA_MOTOR_ADDRESS_MAX) ||
        (z_address < CAMERA_MOTOR_ADDRESS_MIN) ||
        (z_address > CAMERA_MOTOR_ADDRESS_MAX) ||
        (lateral_address == z_address) ||
        (lateral_min_step == 0U) ||
        (lateral_min_step > CAMERA_MOTOR_CONFIG_MAX_MIN_STEP) ||
        (z_min_step == 0U) ||
        (z_min_step > CAMERA_MOTOR_CONFIG_MAX_MIN_STEP) ||
        (lateral_normal_speed_rpm > CAMERA_MOTOR_MAX_JOG_SPEED_RPM) ||
        (z_normal_speed_rpm > CAMERA_MOTOR_MAX_JOG_SPEED_RPM) ||
        ((lateral_direction != 1) && (lateral_direction != -1)) ||
        ((z_direction != 1) && (z_direction != -1)))
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CAMERA_MOTOR_COMMAND_CONFIG;
    command.config.lateral.address = lateral_address;
    command.config.lateral.min_step = lateral_min_step;
    command.config.lateral.normal_speed_rpm = lateral_normal_speed_rpm;
    command.config.lateral.direction = lateral_direction;
    command.config.z.address = z_address;
    command.config.z.min_step = z_min_step;
    command.config.z.normal_speed_rpm = z_normal_speed_rpm;
    command.config.z.direction = z_direction;
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

        case CAMERA_MOTOR_AXIS_LATERAL:
        default:
            return "LATERAL";
    }
}

/**
 * @brief 按运行时方向映射修正摄像头轴点动方向。
 * @param logical_direction 逻辑方向，左右轴 CW=右移/CCW=左移，上下轴 CW=上升/CCW=下降。
 * @param direction_mapping MP157 下发的方向映射，1 保持逻辑方向，-1 反转逻辑方向。
 * @return EMM42_MotorDirection_t 最终发送给 Emm42 的方向。
 */
static EMM42_MotorDirection_t CameraMotorService_MapRuntimeDirection(EMM42_MotorDirection_t logical_direction,
                                                                     int8_t direction_mapping)
{
    if (direction_mapping >= 0)
    {
        return logical_direction;
    }

    return (logical_direction == EMM42_MOTOR_DIRECTION_CW) ?
           EMM42_MOTOR_DIRECTION_CCW :
           EMM42_MOTOR_DIRECTION_CW;
}

/**
 * @brief 根据请求速度和轴配置得到最终点动速度。
 * @param requested_speed_rpm 请求速度，0 表示使用轴配置中的常规速度。
 * @param config 当前轴运行时配置，不能为空。
 * @return uint16_t 最终速度，单位 RPM，范围 0~5000。
 */
static uint16_t CameraMotorService_ResolveJogSpeed(uint16_t requested_speed_rpm,
                                                   const CameraMotor_RuntimeConfig_t *config)
{
    uint16_t resolved_speed;

    if (config == NULL)
    {
        return 0U;
    }

    resolved_speed = (requested_speed_rpm == 0U) ? config->normal_speed_rpm : requested_speed_rpm;
    return CameraMotorService_LimitJogSpeed(resolved_speed);
}

/**
 * @brief 解析 `CAMLAT ...`、旧 `CAMFWD ...` 或 `CAMZ ...` 点动命令。
 * @param command_buffer 已规范化后的命令字符串，不能为空。
 * @param axis 输出目标轴，不能为空。
 * @param direction 输出目标方向，不能为空。
 * @param speed_rpm 输出目标速度，不能为空。
 * @return uint8_t 1 表示解析成功，0 表示不是合法点动命令。
 *
 * 支持格式：
 * - `CAMLAT LEFT [rpm]`
 * - `CAMLAT RIGHT [rpm]`
 * - `CAMFWD FORWARD [rpm]`，旧调试别名，实际映射为右移
 * - `CAMFWD BACKWARD [rpm]`，旧调试别名，实际映射为左移
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

    if (strncmp(command_buffer, "CAMLAT", 6U) == 0)
    {
        *axis = CAMERA_MOTOR_AXIS_LATERAL;
        cursor = command_buffer + 6U;
        CameraMotorService_SkipSpaces(&cursor);

        if (strncmp(cursor, "LEFT", 4U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CCW;
            cursor += 4U;
        }
        else if (strncmp(cursor, "RIGHT", 5U) == 0)
        {
            *direction = EMM42_MOTOR_DIRECTION_CW;
            cursor += 5U;
        }
        else
        {
            return 0U;
        }
    }
    else if (strncmp(command_buffer, "CAMFWD", 6U) == 0)
    {
        *axis = CAMERA_MOTOR_AXIS_LATERAL;
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
        /*
         * 省略 rpm 时保留 0，由任务内按当前轴运行时配置解析为 normal_speed_rpm。
         * 这样 MP157 下发新速度后，CAMLAT/CAMZ 和旧 CAMFWD 文本调试命令也能复用同一份配置。
         */
        parsed_speed = 0U;
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
                  "[INFO][CAM] USART6=PC6/PC7, lateral_addr=%u init=%u min_step=%u speed=%u dir=%d, z_addr=%u init=%u min_step=%u speed=%u dir=%d, last_axis=%s, last_dir=%s, last_speed=%u rpm\r\n",
                  (unsigned int)snapshot.lateral_address,
                  (unsigned int)snapshot.lateral_initialized,
                  (unsigned int)snapshot.lateral_min_step,
                  (unsigned int)snapshot.lateral_normal_speed_rpm,
                  (int)snapshot.lateral_direction,
                  (unsigned int)snapshot.z_address,
                  (unsigned int)snapshot.z_initialized,
                  (unsigned int)snapshot.z_min_step,
                  (unsigned int)snapshot.z_normal_speed_rpm,
                  (int)snapshot.z_direction,
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
 * @param lateral_initialized 左右轴初始化状态。
 * @param z_initialized 上下轴初始化状态。
 */
static void CameraMotorService_UpdateInitSnapshot(uint8_t lateral_initialized, uint8_t z_initialized)
{
    taskENTER_CRITICAL();
    g_camera_motor_snapshot.lateral_initialized = lateral_initialized;
    g_camera_motor_snapshot.z_initialized = z_initialized;
    taskEXIT_CRITICAL();
}

/**
 * @brief 把当前摄像头轴运行时配置写入共享快照。
 * @param config_set 两个摄像头轴的运行时配置，不能为空。
 *
 * 该快照只用于 `CAMINFO` 观察和后续状态扩展，
 * 不参与实际电机控制，实际控制仍以摄像头任务内的 `runtime.config` 为准。
 */
static void CameraMotorService_UpdateConfigSnapshot(const CameraMotor_RuntimeConfigSet_t *config_set)
{
    if (config_set == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_camera_motor_snapshot.lateral_address = config_set->lateral.address;
    g_camera_motor_snapshot.z_address = config_set->z.address;
    g_camera_motor_snapshot.lateral_min_step = config_set->lateral.min_step;
    g_camera_motor_snapshot.z_min_step = config_set->z.min_step;
    g_camera_motor_snapshot.lateral_normal_speed_rpm = config_set->lateral.normal_speed_rpm;
    g_camera_motor_snapshot.z_normal_speed_rpm = config_set->z.normal_speed_rpm;
    g_camera_motor_snapshot.lateral_direction = config_set->lateral.direction;
    g_camera_motor_snapshot.z_direction = config_set->z.direction;
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
 * @param lateral_motor 摄像头左右轴电机句柄，不能为空。
 * @param z_motor 摄像头上下轴电机句柄，不能为空。
 */
static void CameraMotorService_ApplyCommand(const CameraMotor_Command_t *command,
                                            EMM42_MotorHandle_t *lateral_motor,
                                            EMM42_MotorHandle_t *z_motor,
                                            CameraMotor_Runtime_t *runtime)
{
    const EMM42_MotorHandle_t *target_motor;
    const CameraMotor_RuntimeConfig_t *target_config;
    EMM42_MotorStatus_t status;
    EMM42_MotorDirection_t mapped_direction;
    uint16_t speed_rpm;

    if ((command == NULL) || (lateral_motor == NULL) || (z_motor == NULL) || (runtime == NULL))
    {
        return;
    }

    if (CameraMotorService_IsStaleMotionCommand(command) != 0U)
    {
        my_printf(&huart1,
                  "[INFO][CAM] Drop stale motion after STOP. type=%u, axis=%s\r\n",
                  (unsigned int)command->type,
                  CameraMotorService_GetAxisName(command->axis));
        return;
    }

    if (command->type == CAMERA_MOTOR_COMMAND_STOP_ALL)
    {
        status = EMM42_MotorStopNow(lateral_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1, "[ERROR][CAM] lateral stop failed, status=%d\r\n", (int)status);
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

        CameraMotorService_UpdateActionSnapshot(CAMERA_MOTOR_AXIS_LATERAL,
                                                EMM42_MOTOR_DIRECTION_CW,
                                                0U);
        my_printf(&huart1,
                  "[OK][CAM] Stop applied. lateral_addr=%u, z_addr=%u, epoch=%lu\r\n",
                  (unsigned int)lateral_motor->address,
                  (unsigned int)z_motor->address,
                  (unsigned long)command->stop_epoch);
        return;
    }

    if (command->type == CAMERA_MOTOR_COMMAND_CONFIG)
    {
        /*
         * 配置切换只改 F4 运行内存，不写电机 EEPROM。
         * 先按旧地址尝试停机，再更新两个句柄地址，保证后续命令走新地址。
         * 如果旧地址本来就是错的，停止帧可能发不到目标电机，但仍允许更新，
         * 这样现场可以通过 MP157 参数页把地址修正回来。
         */
        status = EMM42_MotorStopNow(lateral_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1, "[ERROR][CAM] runtime config lateral pre-stop failed, status=%d\r\n", (int)status);
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
            my_printf(&huart1, "[ERROR][CAM] runtime config z pre-stop failed, status=%d\r\n", (int)status);
        }

        runtime->config = command->config;
        lateral_motor->address = runtime->config.lateral.address;
        z_motor->address = runtime->config.z.address;
        CameraMotorService_UpdateConfigSnapshot(&runtime->config);
        CameraMotorService_UpdateActionSnapshot(CAMERA_MOTOR_AXIS_LATERAL,
                                                EMM42_MOTOR_DIRECTION_CW,
                                                0U);
        my_printf(&huart1,
                  "[OK][CAM] Runtime config applied. lateral_addr=%u, z_addr=%u, lateral_speed=%u, z_speed=%u\r\n",
                  (unsigned int)runtime->config.lateral.address,
                  (unsigned int)runtime->config.z.address,
                  (unsigned int)runtime->config.lateral.normal_speed_rpm,
                  (unsigned int)runtime->config.z.normal_speed_rpm);
        return;
    }

    target_motor = (command->axis == CAMERA_MOTOR_AXIS_Z) ? z_motor : lateral_motor;
    target_config = (command->axis == CAMERA_MOTOR_AXIS_Z) ?
                    &runtime->config.z :
                    &runtime->config.lateral;

    if (command->type == CAMERA_MOTOR_COMMAND_SET_ZERO)
    {
        status = EMM42_MotorStopNow(target_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1,
                      "[ERROR][CAM] home pre-stop failed. axis=%s, status=%d\r\n",
                      CameraMotorService_GetAxisName(command->axis),
                      (int)status);
            return;
        }

        status = EMM42_MotorResetCurrentPositionToZero(target_motor);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1,
                      "[ERROR][CAM] home set-zero failed. axis=%s, status=%d\r\n",
                      CameraMotorService_GetAxisName(command->axis),
                      (int)status);
            return;
        }

        CameraMotorService_UpdateActionSnapshot(command->axis,
                                                EMM42_MOTOR_DIRECTION_CW,
                                                0U);
        my_printf(&huart1,
                  "[OK][CAM] Current position set to zero. axis=%s\r\n",
                  CameraMotorService_GetAxisName(command->axis));
        return;
    }

    mapped_direction = CameraMotorService_MapRuntimeDirection(command->direction,
                                                             target_config->direction);
    speed_rpm = CameraMotorService_ResolveJogSpeed(command->speed_rpm, target_config);

    if (command->type == CAMERA_MOTOR_COMMAND_POSITION)
    {
        if (speed_rpm == 0U)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault(EMM42_MOTOR_STATUS_RANGE_ERROR,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              0,
                                              0U);
            my_printf(&huart1,
                      "[ERROR][CAM] position speed is zero. axis=%s\r\n",
                      CameraMotorService_GetAxisName(command->axis));
            return;
        }

        status = EMM42_MotorMoveRelativePosition(target_motor,
                                                 mapped_direction,
                                                 speed_rpm,
                                                 CAMERA_MOTOR_ACCEL,
                                                 command->pulse_count,
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
                      "[ERROR][CAM] position failed. axis=%s, pulse=%lu, status=%d\r\n",
                      CameraMotorService_GetAxisName(command->axis),
                      (unsigned long)command->pulse_count,
                      (int)status);
            return;
        }

        CameraMotorService_UpdateActionSnapshot(command->axis,
                                                mapped_direction,
                                                speed_rpm);
        return;
    }

    if (speed_rpm == 0U)
    {
        status = EMM42_MotorStopNow(target_motor, false);
        if (status != EMM42_MOTOR_STATUS_OK)
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR);
            BinaryProtocolService_ReportFault((uint16_t)status,
                                              BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                              (int32_t)status,
                                              0U);
            my_printf(&huart1,
                      "[ERROR][CAM] zero-speed stop failed. axis=%s, status=%d\r\n",
                      CameraMotorService_GetAxisName(command->axis),
                      (int)status);
            return;
        }

        CameraMotorService_UpdateActionSnapshot(command->axis,
                                                mapped_direction,
                                                0U);
        return;
    }

    status = EMM42_MotorSetVelocity(target_motor,
                                    mapped_direction,
                                    speed_rpm,
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
                                            mapped_direction,
                                            speed_rpm);
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
    EMM42_MotorHandle_t lateral_motor;
    EMM42_MotorHandle_t z_motor;
    CameraMotor_Command_t command;
    CameraMotor_Runtime_t runtime;
    EMM42_MotorStatus_t lateral_status;
    EMM42_MotorStatus_t z_status;

    (void)argument;
    runtime.config = CameraMotorService_GetDefaultConfigSet();
    CameraMotorService_UpdateConfigSnapshot(&runtime.config);

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

    EMM42_MotorLoadDefaultConfig(&lateral_motor, &huart6);
    EMM42_MotorLoadDefaultConfig(&z_motor, &huart6);
    lateral_motor.address = runtime.config.lateral.address;
    z_motor.address = runtime.config.z.address;

    for (;;)
    {
        lateral_status = CameraMotorService_InitOneMotor(&lateral_motor,
                                                         runtime.config.lateral.address,
                                                         "lateral");
        z_status = CameraMotorService_InitOneMotor(&z_motor,
                                                   runtime.config.z.address,
                                                   "z");

        CameraMotorService_UpdateInitSnapshot((lateral_status == EMM42_MOTOR_STATUS_OK) ? 1U : 0U,
                                              (z_status == EMM42_MOTOR_STATUS_OK) ? 1U : 0U);

        if ((lateral_status == EMM42_MOTOR_STATUS_OK) && (z_status == EMM42_MOTOR_STATUS_OK))
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
              "[OK][CAM] Camera motor service started. USART6=PC6/PC7, lateral_addr=%u, z_addr=%u\r\n",
              (unsigned int)runtime.config.lateral.address,
              (unsigned int)runtime.config.z.address);

    for (;;)
    {
        if (xQueueReceive(g_camera_motor_command_queue,
                          &command,
                          pdMS_TO_TICKS(CAMERA_MOTOR_TASK_WAIT_MS)) == pdPASS)
        {
            CameraMotorService_ApplyCommand(&command, &lateral_motor, &z_motor, &runtime);
        }
    }
}
