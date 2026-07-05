#include "conveyor_motor_service.h"

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
 * @brief 传送带文本命令和 Emm42 控制效果速查。
 *
 * 通信链路：
 * 1. 用户/MP157 通过 USART1 发送 ASCII 文本命令，命令先由 `weight_service.c` 统一取出并规范化；
 * 2. 本文件只处理 `BELT...` 前缀命令，不直接读取 USART1 DMA 缓存；
 * 3. 传送带任务独占 UART4，PC10(TX) 接传送带 Emm42 RX、PC11(RX) 接传送带 Emm42 TX，115200 8N1，必须共地；
 * 4. 摄像头左右和上下两个 Emm42 电机共用 USART6，不能再让传送带占用 USART6。
 *
 * 用户可发送的 BELT 命令：
 * | 命令 | 参数含义 | 电机效果 | 典型返回/观察方式 |
 * | --- | --- | --- | --- |
 * | `BELTSTOP` | 无参数 | 进入 STOP，立即下发停止命令 | `[OK][BELT] Mode set to STOP.` |
 * | `BELTSCAN` | 无参数 | 进入 SCAN，使用当前运行参数 `scan_speed_rpm` 巡航；未收到 MP157 参数前使用 `CONVEYOR_MOTOR_SCAN_SPEED_RPM` 默认值 | `[OK][BELT] Mode set to SCAN.` |
 * | `BELTINFO` | 无参数 | 不改变电机，只读取运行快照 | `[INFO][BELT] desired/applied/error/speed/dir/track/scan/stable/centered` |
 * | `BELTTRACK <error>` | `error` 为目标 x 坐标相对中心的有符号像素误差 | 进入 TRACK；误差在死区内停机，超出死区按方向和大小调速 | 可继续发 `BELTINFO` 看速度/方向 |
 * | `BELTENABLE 0 [error]` | `0` 表示视觉未使能，后续 error 可省略 | 回到 SCAN 巡航 | 无固定成功回包，必要时查 `BELTINFO` |
 * | `BELTENABLE 1 <error>` | `1` 表示视觉使能，error 为像素误差 | 进入 TRACK 并按 error 调速 | 无固定成功回包，必要时查 `BELTINFO` |
 * | `BELTCAM 0 [current_x center_x]` | `0` 表示视觉未使能，坐标字段即使存在也会忽略 | 回到 SCAN 巡航 | 无固定成功回包 |
 * | `BELTCAM 1 <current_x> <center_x>` | 下位机计算 `error=current_x-center_x` | 进入 TRACK 并按计算出的误差调速 | 无固定成功回包 |
 *
 * 运行边界：
 * - 如果 TRACK 模式超过 `CONVEYOR_MOTOR_TRACK_TIMEOUT_MS` 没有新视觉误差，会主动停机；
 * - 正误差默认映射到 CCW，保证上方来料的负误差沿扫描方向继续送入 ROI 中心；
 * - 本文件只决定模式和速度，具体 Emm42 字节帧见 `User/Driver/emm42_motor.c` 顶部协议表。
 */

/**
 * @brief 控制任务主循环周期，单位毫秒。
 *
 * 该周期同时承担两件事：
 * 1. 周期检查“视觉坐标是否超时”；
 * 2. 在没有新命令时维持状态机推进频率。
 */
#define CONVEYOR_MOTOR_CONTROL_PERIOD_MS            (20U)

/**
 * @brief 传送带 Emm42 电机地址。
 *
 * 当前硬件分配：
 * - 传送带电机：UART4 PC10/PC11，地址 0x01；
 * - 摄像头左右电机：USART6 PC6/PC7，现场地址 0x03；
 * - 摄像头上下电机：USART6 PC6/PC7，现场地址 0x02。
 *
 * 传送带电机虽然独占 UART4，但仍明确写出地址，
 * 方便以后通过串口日志核对现场电机 ID 是否设置正确。
 */
#define CONVEYOR_MOTOR_ADDRESS                      (1U)

/**
 * @brief 传送带默认最小步长，单位 step。
 *
 * 当前速度模式不直接使用该字段，但 MP157 参数页已经把它作为现场可调参数保存和下发，
 * F4 侧先保存到运行时配置，后续补位置步进命令时直接复用。
 */
#define CONVEYOR_MOTOR_MIN_STEP_DEFAULT             (20U)

/**
 * @brief MP157 参数页允许下发的最大常规速度，单位 RPM。
 *
 * 底层 `EMM42_MotorSetVelocity()` 已按张大头协议支持 `0~5000 rpm`，
 * 因此应用层配置也保持同样范围，避免 Qt 能输入的速度在 F4 侧被意外截断。
 */
#define CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM         (5000U)

/**
 * @brief MP157 参数页允许下发的最小步长最大值，单位 step。
 *
 * 当前传送带速度模式暂不使用该字段，但仍在 F4 侧做范围约束，
 * 避免异常 JSON 或误发帧把后续位置控制预留字段写成无意义大数。
 */
#define CONVEYOR_MOTOR_CONFIG_MAX_MIN_STEP          (10000U)

/**
 * @brief Emm42 单机地址允许范围。
 *
 * 地址 0 通常不作为普通单机站号使用，248 以上保留给特殊场景或非法值，
 * 所以运行时配置只接受 1~247。
 */
#define CONVEYOR_MOTOR_ADDRESS_MIN                  (1U)
#define CONVEYOR_MOTOR_ADDRESS_MAX                  (247U)

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
#define CONVEYOR_MOTOR_SCAN_SPEED_RPM               (40U)

/**
 * @brief 跟踪模式最小转速，单位 RPM。
 *
 * 这里的“最小转速”同时也作为“中心附近爬行速度”使用。
 * 现场出现过“小误差仍在持续下发坐标，但 10rpm 推不动传送带”的卡滞现象，
 * 说明原来的爬行速度已经低于当前机械负载下的静摩擦启动速度。
 * 因此这里提高到 20rpm，保证只要真的需要微调，传送带就能实际动起来。
 */
#define CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM          (20U)

/**
 * @brief 小误差爬行区上限，单位像素。
 *
 * 当误差刚刚走出死区时，仍然不适合马上给较高转速。
 * 当前现场反馈“速度太快、需要来回移动几次才停”，
 * 因此把爬行区扩大到 50px，让零件靠近 ROI 中心前就进入低速段。
 */
#define CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX     (50U)

/**
 * @brief 跟踪模式最大转速，单位 RPM。
 *
 * 视觉闭环只负责把零件慢慢送到 ROI 中央，不能像手动点动一样追求速度。
 * 当前把上限从 250rpm 降到 80rpm，用于减少远距离跟踪时的物理惯性过冲。
 */
#define CONVEYOR_MOTOR_TRACK_MAX_SPEED_RPM          (80U)

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
 * 在低速视觉纠偏场景里，减小加速度可以降低速度切换和停机前的冲量，
 * 让零件更容易一次停在 ROI 中央附近。
 */
#define CONVEYOR_MOTOR_ACCEL                        (2U)

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
 * 当前从 18px 继续扩到 24px，目的是把“肉眼已经接近 ROI 中央”的小误差
 * 直接判定为居中，避免 F4 在 18~24px 这种微小误差上反复给低速命令。
 * 如果现场发现零件停得明显偏离 ROI 中央，再逐步缩小到 20px 或 18px。
 */
#define CONVEYOR_MOTOR_CENTER_DEADBAND_PX           (24)

/**
 * @brief 连续多少帧都落在中心死区内，才认为“已经稳定对中”。
 *
 * 注意：
 * 1. 电机会在第一帧进入死区时就立即停止；
 * 2. 这里只是补一个“稳定达标”计数，方便后续日志和状态观察。
 */
#define CONVEYOR_MOTOR_CENTER_STABLE_FRAMES         (3U)

/**
 * @brief 正误差默认对应 CCW 方向。
 *
 * MP157 现在按上方来料下发 `axis_px=center_y`、`target_px=height/2`，
 * 零件刚从画面上方进入时 `axis_px-target_px` 为负数。
 * 现场验证负误差走 CCW 会把零件推回上方离开视野，
 * 因此这里把“正误差是否为 CW”固定为 0，让负误差走 CW 扫描送入方向。
 * 后续如果现场安装方向再次变化，只改这个宏，不要在 MP157 协议层偷偷反号。
 */
#define CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW         (0U)

/**
 * @brief 是否在任务启动后默认进入巡航模式。
 *
 * 当前自动检测流程要求 MP157 首页按“开始”后才启动传送带，
 * 因此默认上电保持静止，等待二进制 `START_CYCLE` 或调试命令 `BELTSCAN`。
 * 这样可以避免 F4 一上电就把零件送进相机视野，导致 MP157 还没准备好就开始运动。
 */
#define CONVEYOR_MOTOR_STARTUP_SCAN_ENABLE          (0U)

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
    CONVEYOR_MOTOR_MODE_STOP = 0, /* 停止模式，电机保持静止，等待下一条巡航或跟踪命令。 */
    CONVEYOR_MOTOR_MODE_SCAN,     /* 巡航扫描模式，传送带按固定低速匀速运行，用于寻找或输送目标。 */
    CONVEYOR_MOTOR_MODE_TRACK,    /* 视觉跟踪模式，根据相机中心误差动态调整速度和方向。 */
    CONVEYOR_MOTOR_MODE_POSITION, /* 位置模式，已下发一次固定步数运动，等待下一条 STOP/SCAN/TRACK 命令。 */
    CONVEYOR_MOTOR_MODE_JOG       /* 手动持续运动模式，按指定方向和速度运行，直到收到 STOP。 */
} ConveyorMotor_Mode_t;

/**
 * @brief 发给电机任务的内部命令类型。
 */
typedef enum
{
    CONVEYOR_MOTOR_COMMAND_STOP = 0, /* 队列命令：要求电机任务切换到 STOP，并立即下发停止帧。 */
    CONVEYOR_MOTOR_COMMAND_SCAN,     /* 队列命令：要求电机任务切换到 SCAN，并按巡航速度运行。 */
    CONVEYOR_MOTOR_COMMAND_TRACK,    /* 队列命令：要求电机任务使用最新像素误差执行视觉对中。 */
    CONVEYOR_MOTOR_COMMAND_POSITION, /* 队列命令：要求电机任务按位置模式移动固定步数。 */
    CONVEYOR_MOTOR_COMMAND_JOG,      /* 队列命令：要求电机任务按指定方向和速度持续运行。 */
    CONVEYOR_MOTOR_COMMAND_SET_ZERO, /* 队列命令：要求电机任务停止后把当前位置设为零点。 */
    CONVEYOR_MOTOR_COMMAND_CONFIG    /* 队列命令：更新 MP157 下发的地址、步长、对中速度、上料速度和方向。 */
} ConveyorMotor_CommandType_t;

/**
 * @brief MP157 下发的传送带运行参数。
 */
typedef struct
{
    uint8_t address;                 /* Emm42 地址，传送带当前使用 UART4 独占总线。 */
    uint16_t min_step;               /* 最小步长，单位 step，当前保存给后续位置控制使用。 */
    uint16_t normal_speed_rpm;       /* 视觉对中/短步位置速度，单位 RPM，0 表示对中和位置动作保持停止。 */
    uint16_t scan_speed_rpm;         /* 上料扫描速度，单位 RPM，零件未入画时 SCAN 模式使用。 */
    int8_t direction;                /* 方向映射，>=0 表示 CW，<0 表示 CCW。 */
} ConveyorMotor_RuntimeConfig_t;

/**
 * @brief 传送带位置运动完成事件的等待上下文。
 *
 * 该结构只在传送带任务内读写，用来把“一次 MP157 位置命令”与
 * “随后 Emm42 返回的到位字节帧”绑定起来，防止 ACK 被误当成真实运动完成。
 */
typedef struct
{
    uint8_t active_flag;                     /* 1 表示当前有一条位置运动正在等待 Emm42 到位回包。 */
    uint16_t cycle_id;                       /* MP157 自动检测流程 ID，事件上报时原样返回。 */
    uint16_t related_seq;                    /* 原始 ACTUATOR_POS_MOVE 帧序号，MP157 用它匹配等待中的命令。 */
    uint8_t actuator;                        /* 协议执行器编号，传送带为 0，写入 detail_i32 高字节。 */
    uint8_t direction;                       /* 原始协议方向字段，写入 detail_i32，便于 MP157 做诊断展示。 */
    TickType_t start_tick;                   /* 开始等待到位回包的 tick，用于按差值判断超时并兼容 tick 回绕。 */
    TickType_t timeout_ticks;                /* 本次允许等待的 tick 数，由步数和转速估算得到。 */
    EMM42_MotorReachedAckParser_t parser;    /* 到位回包滑动窗口解析器，识别 [addr FD 9F 6B]。 */
} ConveyorMotor_PendingMoveReport_t;

/**
 * @brief 电机任务消息队列中的命令结构。
 *
 * 当前只需要表达两类信息：
 * 1. 模式切换；
 * 2. 跟踪模式下的最新像素误差。
 */
typedef struct
{
    ConveyorMotor_CommandType_t type; /* 命令类型，决定电机任务本轮切换到停止、巡航还是视觉跟踪模式。 */
    int32_t error_px;                 /* 视觉目标相对中心的带符号像素误差，单位像素，仅 TRACK 命令使用。 */
    EMM42_MotorDirection_t direction; /* 位置模式逻辑方向，CW 表示工程默认前进，CCW 表示工程默认后退。 */
    uint16_t speed_rpm;               /* 位置模式速度，单位 RPM；0 表示使用当前运行参数常规速度。 */
    uint32_t pulse_count;             /* 位置模式相对移动步数，单位 step。 */
    uint8_t report_enabled;           /* 1 表示本位置命令完成后要用 EVENT_REPORT 回报 MP157。 */
    uint16_t report_cycle_id;         /* 完成事件所属流程 ID，由二进制协议层从原始帧带入。 */
    uint16_t report_related_seq;      /* 完成事件关联的原始帧序号，MP157 按该序号解除等待。 */
    uint8_t report_actuator;          /* 完成事件里的执行器编号，传送带固定为 0。 */
    uint8_t report_direction;         /* 完成事件里的方向字段，按 MP157 原始 direction 回填。 */
    ConveyorMotor_RuntimeConfig_t config; /* CONFIG 命令携带的新运行参数，其它命令忽略该字段。 */
} ConveyorMotor_Command_t;

/**
 * @brief 供 `BELTINFO` 查询使用的运行时快照。
 *
 * 这里把“期望模式”和“实际执行状态”分开保存，
 * 方便区分“正在 TRACK，但当前因进入死区而已停住”的情况。
 */
typedef struct
{
    ConveyorMotor_Mode_t desired_mode;             /* 命令层期望进入的模式，用于反映用户或视觉主机最近一次控制意图。 */
    ConveyorMotor_Mode_t applied_mode;             /* 实际已经下发到电机侧的模式，用于区分期望跟踪但当前已因死区停机的情况。 */
    int32_t latest_error_px;                       /* 最近一次视觉跟踪误差，单位像素，用于 BELTINFO 查询和调试观察。 */
    uint16_t applied_speed_rpm;                    /* 当前已下发的电机转速，单位 RPM，0 表示当前命令为停止。 */
    EMM42_MotorDirection_t applied_direction;      /* 当前已下发的电机方向，结合速度用于判断传送带实际运动方向。 */
    uint16_t config_normal_speed_rpm;              /* 当前运行时对中/短步速度，来自 MP157 参数页 normal_speed_rpm。 */
    uint16_t config_scan_speed_rpm;                /* 当前运行时上料扫描速度，来自 MP157 参数页 scan_speed_rpm。 */
    uint8_t center_stable_count;                   /* 连续落入中心死区的帧数，用于判断视觉目标是否已经稳定对中。 */
    uint8_t centered_flag;                         /* 对中状态标志，1 表示已经连续达到中心稳定条件，0 表示仍需调整。 */
} ConveyorMotor_RuntimeSnapshot_t;

/**
 * @brief 电机任务内部的完整运行时状态。
 */
typedef struct
{
    ConveyorMotor_Mode_t desired_mode;             /* 任务内部期望模式，由命令队列更新，是状态机决策的主输入。 */
    ConveyorMotor_Mode_t applied_mode;             /* 任务内部实际模式，记录最近一次已经执行的电机控制状态。 */
    int32_t latest_error_px;                       /* 最新视觉像素误差，单位像素，正负号用于决定电机转向。 */
    uint16_t applied_speed_rpm;                    /* 最近一次下发到 Emm42 的速度，单位 RPM，用于避免重复发送相同速度。 */
    EMM42_MotorDirection_t applied_direction;      /* 最近一次下发到 Emm42 的方向，用于判断是否需要重新发送速度命令。 */
    uint8_t center_stable_count;                   /* 连续中心稳定帧计数，达到阈值后置位 centered_flag。 */
    uint8_t centered_flag;                         /* 当前目标是否已经稳定对中，影响日志输出和后续控制策略。 */
    uint8_t fresh_track_sample_flag;               /* 新跟踪样本标志，1 表示本周期刚收到新的 BELTTRACK 误差。 */
    uint8_t track_timeout_reported_flag;           /* 跟踪超时日志抑制标志，避免超时期间反复刷同一条告警。 */
    TickType_t last_track_update_tick;             /* 最近一次收到视觉跟踪样本的 RTOS tick，用于计算跟踪输入超时。 */
    EMM42_MotorDirection_t jog_direction;          /* 手动持续运动的逻辑方向，由 ACTUATOR_VEL_MOVE 写入。 */
    uint16_t jog_speed_rpm;                        /* 手动持续运动的速度，单位 RPM，由 ACTUATOR_VEL_MOVE 写入。 */
    ConveyorMotor_RuntimeConfig_t config;          /* 当前生效的 MP157 步进电机参数，任务内独占读写。 */
    ConveyorMotor_PendingMoveReport_t pending_move; /* 当前等待到位事件的位置运动上下文。 */
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
    CONVEYOR_MOTOR_SCAN_SPEED_RPM,
    CONVEYOR_MOTOR_SCAN_SPEED_RPM,
    0U,
    0U
};

/**
 * @brief 返回传送带电机的默认运行时配置。
 * @return ConveyorMotor_RuntimeConfig_t 默认地址、最小步长、对中速度、上料速度和方向。
 *
 * 该函数把编译期宏集中转换成运行时结构，
 * 后续 MP157 下发新参数时只需要覆盖 `runtime.config`，不需要改散落宏值。
 */
static ConveyorMotor_RuntimeConfig_t ConveyorMotorService_GetDefaultRuntimeConfig(void)
{
    ConveyorMotor_RuntimeConfig_t config;

    config.address = CONVEYOR_MOTOR_ADDRESS;
    config.min_step = CONVEYOR_MOTOR_MIN_STEP_DEFAULT;
    config.normal_speed_rpm = CONVEYOR_MOTOR_SCAN_SPEED_RPM;
    config.scan_speed_rpm = CONVEYOR_MOTOR_SCAN_SPEED_RPM;
    config.direction = 1;
    return config;
}

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

        case CONVEYOR_MOTOR_MODE_POSITION:
            return "POSITION";

        case CONVEYOR_MOTOR_MODE_JOG:
            return "JOG";

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
    g_conveyor_motor_runtime_snapshot.config_normal_speed_rpm = runtime->config.normal_speed_rpm;
    g_conveyor_motor_runtime_snapshot.config_scan_speed_rpm = runtime->config.scan_speed_rpm;
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
 * @brief 向 MP157 发送传送带位置运动完成或超时事件。
 * @param cycle_id 自动检测流程 ID。
 * @param related_seq 原始 ACTUATOR_POS_MOVE 帧序号。
 * @param actuator 协议执行器编号，传送带为 0。
 * @param direction 原始协议方向字段。
 * @param event_code `ACTUATOR_MOVE_DONE` 或 `ACTUATOR_MOVE_TIMEOUT`。
 * @param status_code 底层状态码，0 表示成功，其它值说明失败来源。
 *
 * 该函数只负责事件打包，不判断是否应该发送；调用者必须先确认本次运动确实需要上报。
 */
static void ConveyorMotorService_SendMoveReport(uint16_t cycle_id,
                                                uint16_t related_seq,
                                                uint8_t actuator,
                                                uint8_t direction,
                                                uint8_t event_code,
                                                uint16_t status_code)
{
    int32_t detail_i32;

    detail_i32 = BinaryProtocolService_BuildActuatorMoveDetail(actuator,
                                                               direction,
                                                               status_code);
    BinaryProtocolService_SendEventReport(cycle_id,
                                          event_code,
                                          0U,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CONVEYOR,
                                          detail_i32,
                                          related_seq);
}

/**
 * @brief 清除当前传送带位置运动等待上下文。
 * @param runtime 传送带任务运行时状态，不能为空。
 *
 * STOP、SCAN、TRACK、JOG、CONFIG、SET_ZERO 都会改变运动语义，
 * 因此执行这些命令前必须清空旧的到位等待，避免旧回包误触发 MP157 流程推进。
 */
static void ConveyorMotorService_ClearPendingMoveReport(ConveyorMotor_Runtime_t *runtime)
{
    if (runtime == NULL)
    {
        return;
    }

    runtime->pending_move.active_flag = 0U;
    runtime->pending_move.cycle_id = 0U;
    runtime->pending_move.related_seq = 0U;
    runtime->pending_move.actuator = 0U;
    runtime->pending_move.direction = 0U;
    runtime->pending_move.start_tick = 0U;
    runtime->pending_move.timeout_ticks = 0U;
    EMM42_MotorReachedAckParserReset(&runtime->pending_move.parser);
}

/**
 * @brief 根据已成功下发的位置命令启动到位回包等待。
 * @param runtime 传送带任务运行时状态，不能为空。
 * @param command 已执行成功的位置命令，不能为空。
 * @param speed_rpm 实际下发的速度，单位 RPM。
 *
 * 只有 `report_enabled=1` 的二进制位置命令才会启动等待；
 * 旧接口或文本调试命令不需要通知 MP157，就不会占用 pending 上下文。
 */
static void ConveyorMotorService_StartPendingMoveReport(ConveyorMotor_Runtime_t *runtime,
                                                        const ConveyorMotor_Command_t *command,
                                                        uint16_t speed_rpm)
{
    uint32_t timeout_ms;
    TickType_t timeout_ticks;

    if ((runtime == NULL) || (command == NULL) || (command->report_enabled == 0U))
    {
        return;
    }

    ConveyorMotorService_ClearPendingMoveReport(runtime);
    timeout_ms = EMM42_MotorEstimateReachedTimeoutMs(command->pulse_count, speed_rpm);
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U)
    {
        timeout_ticks = 1U;
    }

    runtime->pending_move.active_flag = 1U;
    runtime->pending_move.cycle_id = command->report_cycle_id;
    runtime->pending_move.related_seq = command->report_related_seq;
    runtime->pending_move.actuator = command->report_actuator;
    runtime->pending_move.direction = command->report_direction;
    runtime->pending_move.start_tick = xTaskGetTickCount();
    runtime->pending_move.timeout_ticks = timeout_ticks;
    EMM42_MotorReachedAckParserReset(&runtime->pending_move.parser);
}

/**
 * @brief 位置命令还没进入 pending 阶段就失败时，立即通知 MP157。
 * @param command 原始队列命令，不能为空。
 * @param status_code 失败状态码。
 *
 * 例如速度解析为 0、清空 RX 失败或位置帧发送失败时，
 * MP157 已经收到 ACK，此处必须再发超时/失败事件解除上位机等待。
 */
static void ConveyorMotorService_ReportPositionCommandFailure(const ConveyorMotor_Command_t *command,
                                                              uint16_t status_code)
{
    if ((command == NULL) || (command->report_enabled == 0U))
    {
        return;
    }

    BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
    ConveyorMotorService_SendMoveReport(command->report_cycle_id,
                                        command->report_related_seq,
                                        command->report_actuator,
                                        command->report_direction,
                                        BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_TIMEOUT,
                                        status_code);
}

/**
 * @brief 周期轮询传送带 Emm42 到位回包并上报 MP157。
 * @param motor 传送带 Emm42 电机句柄，不能为空。
 * @param runtime 传送带任务运行时状态，不能为空。
 *
 * 该函数不阻塞等待串口数据，每个控制周期只读少量已到达字节。
 * 正常识别 `[addr FD 9F 6B]` 后发送 DONE；超过估算时间或 UART RX 出错后发送 TIMEOUT。
 */
static void ConveyorMotorService_PollPendingMoveReport(const EMM42_MotorHandle_t *motor,
                                                       ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;
    uint8_t reached_flag;
    TickType_t current_tick;

    if ((motor == NULL) || (runtime == NULL) || (runtime->pending_move.active_flag == 0U))
    {
        return;
    }

    status = EMM42_MotorPollReachedAck(motor,
                                       &runtime->pending_move.parser,
                                       &reached_flag);
    if (status != EMM42_MOTOR_STATUS_OK)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)status,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CONVEYOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          (int32_t)status,
                                          runtime->pending_move.related_seq);
        ConveyorMotorService_SendMoveReport(runtime->pending_move.cycle_id,
                                            runtime->pending_move.related_seq,
                                            runtime->pending_move.actuator,
                                            runtime->pending_move.direction,
                                            BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_TIMEOUT,
                                            (uint16_t)status);
        ConveyorMotorService_ClearPendingMoveReport(runtime);
        return;
    }

    if (reached_flag != 0U)
    {
        BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
        ConveyorMotorService_SendMoveReport(runtime->pending_move.cycle_id,
                                            runtime->pending_move.related_seq,
                                            runtime->pending_move.actuator,
                                            runtime->pending_move.direction,
                                            BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_DONE,
                                            (uint16_t)EMM42_MOTOR_STATUS_OK);
        ConveyorMotorService_ClearPendingMoveReport(runtime);
        return;
    }

    current_tick = xTaskGetTickCount();
    if ((current_tick - runtime->pending_move.start_tick) >= runtime->pending_move.timeout_ticks)
    {
        BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY);
        BinaryProtocolService_ReportFault((uint16_t)EMM42_MOTOR_STATUS_TIMEOUT,
                                          BINARY_PROTOCOL_FAULT_SOURCE_CONVEYOR,
                                          BINARY_PROTOCOL_FAULT_SEVERITY_WARNING,
                                          BinaryProtocolService_BuildActuatorMoveDetail(runtime->pending_move.actuator,
                                                                                       runtime->pending_move.direction,
                                                                                       (uint16_t)EMM42_MOTOR_STATUS_TIMEOUT),
                                          runtime->pending_move.related_seq);
        ConveyorMotorService_SendMoveReport(runtime->pending_move.cycle_id,
                                            runtime->pending_move.related_seq,
                                            runtime->pending_move.actuator,
                                            runtime->pending_move.direction,
                                            BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_TIMEOUT,
                                            (uint16_t)EMM42_MOTOR_STATUS_TIMEOUT);
        ConveyorMotorService_ClearPendingMoveReport(runtime);
    }
}

/**
 * @brief 请求传送带进入低速扫描模式。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该函数是二进制协议层调用传送带服务的公共入口。
 * 它只投递内部队列命令，不直接碰 Emm42 串口，确保 USART1 命令分发路径保持轻量。
 */
uint8_t ConveyorMotorService_RequestScan(void)
{
    ConveyorMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_SCAN;
    command.error_px = 0;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带立即停止。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 该函数复用传送带任务已有 STOP 状态机，
 * 避免二进制协议层绕过任务队列直接操作电机导致并发问题。
 */
uint8_t ConveyorMotorService_RequestStop(void)
{
    ConveyorMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_STOP;
    command.error_px = 0;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带按照视觉误差进入跟踪模式。
 * @param error_px 视觉目标相对中心的带符号像素误差，单位像素。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 二进制协议中的 `VISION_POS` 会先计算 `axis_px - target_px`，
 * 再通过本函数把误差投递给传送带任务。
 */
uint8_t ConveyorMotorService_RequestTrack(int32_t error_px)
{
    ConveyorMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_TRACK;
    command.error_px = error_px;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带按指定方向持续速度运动。
 * @param forward_flag 1 表示工程默认前进方向，0 表示工程默认后退方向。
 * @param speed_rpm 速度模式转速，单位 RPM，范围 1~5000。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或任务队列尚未创建。
 *
 * 该函数只投递手动持续运动命令，不会自动计步或自动停机。
 * 上位机必须继续通过 `ACTUATOR_STOP` 或 `STOP_CYCLE` 结束这次速度运动。
 */
uint8_t ConveyorMotorService_RequestJog(uint8_t forward_flag, uint16_t speed_rpm)
{
    ConveyorMotor_Command_t command;

    if ((speed_rpm == 0U) || (speed_rpm > CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM))
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_JOG;
    command.direction = (forward_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = speed_rpm;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带按相对位置模式移动固定步数。
 * @param forward_flag 1 表示工程默认前进方向，0 表示工程默认后退方向。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用运行参数常规速度。
 * @param pulse_count 相对移动步数，单位 step。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或任务队列尚未创建。
 */
uint8_t ConveyorMotorService_RequestPosition(uint8_t forward_flag,
                                             uint16_t speed_rpm,
                                             uint32_t pulse_count)
{
    ConveyorMotor_Command_t command;

    if ((speed_rpm > CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM) || (pulse_count == 0U))
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_POSITION;
    command.direction = (forward_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = speed_rpm;
    command.pulse_count = pulse_count;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带位置运动，并在到位或超时时通过 EVENT_REPORT 回告 MP157。
 * @param forward_flag 1 表示工程默认前进方向，0 表示工程默认后退方向。
 * @param speed_rpm 位置运动速度，单位 RPM，传 0 使用当前运行参数常规速度。
 * @param pulse_count 相对移动步数，单位 step。
 * @param cycle_id 自动检测流程 ID。
 * @param related_seq 原始 ACTUATOR_POS_MOVE 帧序号。
 * @param actuator 协议执行器编号。
 * @param direction 原始协议方向字段。
 * @return uint8_t 1 表示请求已投递，0 表示参数非法或任务未就绪。
 *
 * 该接口专门用于二进制自动流程。
 * ACK 只表示队列投递成功；后续 DONE/TIMEOUT 事件才表示物理运动结果。
 */
uint8_t ConveyorMotorService_RequestPositionWithReport(uint8_t forward_flag,
                                                       uint16_t speed_rpm,
                                                       uint32_t pulse_count,
                                                       uint16_t cycle_id,
                                                       uint16_t related_seq,
                                                       uint8_t actuator,
                                                       uint8_t direction)
{
    ConveyorMotor_Command_t command;

    if ((speed_rpm > CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM) || (pulse_count == 0U))
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_POSITION;
    command.direction = (forward_flag != 0U) ? EMM42_MOTOR_DIRECTION_CW : EMM42_MOTOR_DIRECTION_CCW;
    command.speed_rpm = speed_rpm;
    command.pulse_count = pulse_count;
    command.report_enabled = 1U;
    command.report_cycle_id = cycle_id;
    command.report_related_seq = related_seq;
    command.report_actuator = actuator;
    command.report_direction = direction;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 请求传送带把当前位置设为新的零点。
 * @return uint8_t 1 表示请求已投递，0 表示传送带任务尚未就绪。
 *
 * 这里只投递队列命令，真正停机和 Emm42 清零帧发送在传送带任务上下文执行，
 * 避免协议解析任务直接占用 UART4。
 */
uint8_t ConveyorMotorService_RequestSetCurrentPositionZero(void)
{
    ConveyorMotor_Command_t command;

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_SET_ZERO;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 更新传送带步进电机运行参数。
 * @param address Emm42 电机地址，允许 1~247。
 * @param min_step 最小步长，单位 step，允许 1~10000。
 * @param normal_speed_rpm 视觉对中/短步位置速度，单位 RPM，允许 0~5000。
 * @param scan_speed_rpm 上料扫描速度，单位 RPM，允许 0~5000。
 * @param direction 方向映射，1 表示保持默认方向，-1 表示反转默认方向。
 * @return uint8_t 1 表示配置命令已投递，0 表示参数非法或任务队列尚未创建。
 *
 * 该函数只把新参数投递给传送带任务，
 * 真正更新 `motor.address` 和 `runtime.config` 的动作在任务上下文完成，
 * 避免协议层和传送带任务同时访问 UART4 或电机句柄。
 */
uint8_t ConveyorMotorService_RequestRuntimeConfig(uint8_t address,
                                                  uint16_t min_step,
                                                  uint16_t normal_speed_rpm,
                                                  uint16_t scan_speed_rpm,
                                                  int8_t direction)
{
    ConveyorMotor_Command_t command;

    if ((address < CONVEYOR_MOTOR_ADDRESS_MIN) ||
        (address > CONVEYOR_MOTOR_ADDRESS_MAX) ||
        (min_step == 0U) ||
        (min_step > CONVEYOR_MOTOR_CONFIG_MAX_MIN_STEP) ||
        (normal_speed_rpm > CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM) ||
        (scan_speed_rpm > CONVEYOR_MOTOR_CONFIG_MAX_SPEED_RPM) ||
        ((direction != 1) && (direction != -1)))
    {
        return 0U;
    }

    (void)memset(&command, 0, sizeof(command));
    command.type = CONVEYOR_MOTOR_COMMAND_CONFIG;
    command.config.address = address;
    command.config.min_step = min_step;
    command.config.normal_speed_rpm = normal_speed_rpm;
    command.config.scan_speed_rpm = scan_speed_rpm;
    command.config.direction = direction;
    return ConveyorMotorService_PostCommand(&command);
}

/**
 * @brief 读取传送带服务当前状态快照。
 * @param status 状态输出结构体，不能为空。
 * @return uint8_t 1 表示读取成功，0 表示参数为空或传送带任务尚未创建队列。
 *
 * 该函数只做一次受临界区保护的内存拷贝，不访问 Emm42 串口，
 * 因此可以被二进制协议层在 USART1 命令处理上下文中快速调用。
 */
uint8_t ConveyorMotorService_GetStatus(ConveyorMotor_Status_t *status)
{
    ConveyorMotor_RuntimeSnapshot_t snapshot;

    if (status == NULL)
    {
        return 0U;
    }

    if (g_conveyor_motor_command_queue == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    snapshot = g_conveyor_motor_runtime_snapshot;
    taskEXIT_CRITICAL();

    status->desired_mode = (uint8_t)snapshot.desired_mode;
    status->applied_mode = (uint8_t)snapshot.applied_mode;
    status->latest_error_px = snapshot.latest_error_px;
    status->speed_rpm = snapshot.applied_speed_rpm;
    status->direction = (snapshot.applied_direction == EMM42_MOTOR_DIRECTION_CCW) ? 1U : 0U;
    status->stable_count = snapshot.center_stable_count;
    status->centered = snapshot.centered_flag;
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
 * @param track_speed_limit_rpm MP157 参数页下发的对中速度上限，单位 RPM。
 * @return uint16_t 目标转速，单位 RPM。
 *
 * 视觉 TRACK 的速度必须受 MP157 参数页控制：
 * - scan_speed_rpm 只用于零件未入画时的 SCAN 上料；
 * - normal_speed_rpm 作为零件入画后的对中/短步速度上限；
 * - 当 normal_speed_rpm 为 0 时，TRACK 保持停止，便于现场临时禁用自动对中。
 */
static uint16_t ConveyorMotorService_MapErrorToSpeed(uint32_t absolute_error_px,
                                                     uint16_t track_speed_limit_rpm)
{
    uint32_t mapped_speed_rpm;
    uint16_t crawl_speed_rpm;

    if (track_speed_limit_rpm == 0U)
    {
        return 0U;
    }

    crawl_speed_rpm = CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM;
    if (crawl_speed_rpm > track_speed_limit_rpm)
    {
        crawl_speed_rpm = track_speed_limit_rpm;
    }

    /*
     * 中心附近先给一个固定低速爬行值，
     * 避免误差刚走出死区时速度突然偏快。
     */
    if (absolute_error_px <= CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX)
    {
        return crawl_speed_rpm;
    }

    mapped_speed_rpm = crawl_speed_rpm +
                       ((absolute_error_px - CONVEYOR_MOTOR_TRACK_CRAWL_MAX_ERROR_PX) *
                        CONVEYOR_MOTOR_TRACK_RPM_PER_PIXEL);

    if (mapped_speed_rpm > track_speed_limit_rpm)
    {
        mapped_speed_rpm = track_speed_limit_rpm;
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
 * @brief 按 MP157 运行时方向映射修正电机方向。
 * @param base_direction 代码默认计算出来的方向。
 * @param direction_mapping MP157 下发的方向映射，正数保留默认方向，负数反转方向。
 * @return EMM42_MotorDirection_t 最终下发给 Emm42 的方向。
 *
 * 该函数用于现场快速修正“坐标越调越远”或传送带扫描方向相反的问题，
 * 避免每次都重新修改 `CONVEYOR_MOTOR_POSITIVE_ERROR_IS_CW` 并重新编译烧录。
 */
static EMM42_MotorDirection_t ConveyorMotorService_MapRuntimeDirection(EMM42_MotorDirection_t base_direction,
                                                                       int8_t direction_mapping)
{
    if (direction_mapping >= 0)
    {
        return base_direction;
    }

    return (base_direction == EMM42_MOTOR_DIRECTION_CW) ?
           EMM42_MOTOR_DIRECTION_CCW :
           EMM42_MOTOR_DIRECTION_CW;
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
    uint16_t speed_rpm;

    if ((motor == NULL) || (runtime == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    /*
     * 运行时方向映射来自 MP157 参数页：
     * - direction=1：沿用工程默认 CW 扫描方向；
     * - direction=-1：把扫描方向反过来，方便现场快速纠正安装方向。
     */
    direction = ConveyorMotorService_MapRuntimeDirection(EMM42_MOTOR_DIRECTION_CW,
                                                         runtime->config.direction);
    speed_rpm = runtime->config.scan_speed_rpm;

    if (speed_rpm == 0U)
    {
        return ConveyorMotorService_ApplyStop(motor, runtime);
    }

    if ((runtime->applied_mode == CONVEYOR_MOTOR_MODE_SCAN) &&
        (runtime->applied_speed_rpm == speed_rpm) &&
        (runtime->applied_direction == direction))
    {
        return EMM42_MOTOR_STATUS_OK;
    }

    status = EMM42_MotorSetVelocity(motor,
                                    direction,
                                    speed_rpm,
                                    CONVEYOR_MOTOR_ACCEL,
                                    false);
    if (status == EMM42_MOTOR_STATUS_OK)
    {
        runtime->applied_mode = CONVEYOR_MOTOR_MODE_SCAN;
        runtime->applied_speed_rpm = speed_rpm;
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

    direction = ConveyorMotorService_MapRuntimeDirection(
        ConveyorMotorService_GetDirectionByError(runtime->latest_error_px),
        runtime->config.direction);
    speed_rpm = ConveyorMotorService_MapErrorToSpeed(
        ConveyorMotorService_GetAbsoluteError(runtime->latest_error_px),
        runtime->config.normal_speed_rpm);

    if (speed_rpm == 0U)
    {
        return ConveyorMotorService_ApplyStop(motor, runtime);
    }

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
 * @brief 执行手动持续运动速度命令。
 * @param motor 电机句柄，不能为空。
 * @param runtime 任务运行时状态，不能为空，里面保存手动方向和速度。
 * @return EMM42_MotorStatus_t 发送结果。
 *
 * 手动 JOG 模式只负责让电机保持某个速度运行。
 * 停止条件由 STOP 命令触发，不能在这里自动停，否则会破坏“按一次持续动”的调试语义。
 */
static EMM42_MotorStatus_t ConveyorMotorService_ApplyJog(const EMM42_MotorHandle_t *motor,
                                                         ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;
    EMM42_MotorDirection_t mapped_direction;

    if ((motor == NULL) || (runtime == NULL))
    {
        return EMM42_MOTOR_STATUS_INVALID_PARAM;
    }

    if (runtime->jog_speed_rpm == 0U)
    {
        return ConveyorMotorService_ApplyStop(motor, runtime);
    }

    mapped_direction = ConveyorMotorService_MapRuntimeDirection(runtime->jog_direction,
                                                               runtime->config.direction);

    if ((runtime->applied_mode == CONVEYOR_MOTOR_MODE_JOG) &&
        (runtime->applied_speed_rpm == runtime->jog_speed_rpm) &&
        (runtime->applied_direction == mapped_direction))
    {
        return EMM42_MOTOR_STATUS_OK;
    }

    status = EMM42_MotorSetVelocity(motor,
                                    mapped_direction,
                                    runtime->jog_speed_rpm,
                                    CONVEYOR_MOTOR_ACCEL,
                                    false);
    if (status == EMM42_MOTOR_STATUS_OK)
    {
        runtime->applied_mode = CONVEYOR_MOTOR_MODE_JOG;
        runtime->applied_speed_rpm = runtime->jog_speed_rpm;
        runtime->applied_direction = mapped_direction;
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
    ConveyorMotor_RuntimeConfig_t default_config;

    default_config = ConveyorMotorService_GetDefaultRuntimeConfig();

    my_printf(&huart1,
              "[OK][BELT] Emm42 conveyor service started. UART4=PC10/PC11, addr=%u, mode=velocity, ctrl=%s\r\n",
              (unsigned int)CONVEYOR_MOTOR_ADDRESS,
              ConveyorMotorService_GetControlModeName(CONVEYOR_MOTOR_STARTUP_CTRL_MODE));
    my_printf(&huart1,
              "[INFO][BELT] default scan=%u rpm, track_limit=%u rpm, crawl_min=%u rpm, crawl<=%u px, deadband=%d px, stable=%u, timeout=%u ms\r\n",
              (unsigned int)default_config.scan_speed_rpm,
              (unsigned int)default_config.normal_speed_rpm,
              (unsigned int)CONVEYOR_MOTOR_TRACK_MIN_SPEED_RPM,
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
                                                     EMM42_MotorHandle_t *motor,
                                                     ConveyorMotor_Runtime_t *runtime)
{
    EMM42_MotorStatus_t status;

    if ((command == NULL) || (motor == NULL) || (runtime == NULL))
    {
        return;
    }

    switch (command->type)
    {
        case CONVEYOR_MOTOR_COMMAND_SCAN:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_SCAN;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;
            break;

        case CONVEYOR_MOTOR_COMMAND_TRACK:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_TRACK;
            runtime->latest_error_px = command->error_px;
            runtime->fresh_track_sample_flag = 1U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->last_track_update_tick = xTaskGetTickCount();
            break;

        case CONVEYOR_MOTOR_COMMAND_POSITION:
        {
            EMM42_MotorDirection_t mapped_direction;
            uint16_t speed_rpm;

            ConveyorMotorService_ClearPendingMoveReport(runtime);
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_POSITION;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;

            mapped_direction = ConveyorMotorService_MapRuntimeDirection(command->direction,
                                                                        runtime->config.direction);
            speed_rpm = (command->speed_rpm == 0U) ?
                        runtime->config.normal_speed_rpm :
                        command->speed_rpm;
            if (speed_rpm == 0U)
            {
                ConveyorMotorService_ReportDriverError("position speed zero",
                                                       EMM42_MOTOR_STATUS_RANGE_ERROR);
                ConveyorMotorService_ReportPositionCommandFailure(command,
                                                                  (uint16_t)EMM42_MOTOR_STATUS_RANGE_ERROR);
                break;
            }

            status = EMM42_MotorFlushReceive(motor);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("position rx flush", status);
                ConveyorMotorService_ReportPositionCommandFailure(command, (uint16_t)status);
                break;
            }

            status = EMM42_MotorMoveRelativePosition(motor,
                                                     mapped_direction,
                                                     speed_rpm,
                                                     CONVEYOR_MOTOR_ACCEL,
                                                     command->pulse_count,
                                                     false);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("position command", status);
                ConveyorMotorService_ReportPositionCommandFailure(command, (uint16_t)status);
                break;
            }

            runtime->applied_mode = CONVEYOR_MOTOR_MODE_POSITION;
            runtime->applied_speed_rpm = speed_rpm;
            runtime->applied_direction = mapped_direction;
            ConveyorMotorService_StartPendingMoveReport(runtime, command, speed_rpm);
            break;
        }

        case CONVEYOR_MOTOR_COMMAND_JOG:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_JOG;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;
            runtime->jog_direction = command->direction;
            runtime->jog_speed_rpm = command->speed_rpm;
            break;

        case CONVEYOR_MOTOR_COMMAND_SET_ZERO:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
            /*
             * 当前位置清零是参数标定动作，不应该在运动中执行。
             * 因此先切到 STOP 并发送停止帧，再发送 Emm42 当前位置清零帧。
             */
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_STOP;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;
            runtime->jog_speed_rpm = 0U;

            status = ConveyorMotorService_ApplyStop(motor, runtime);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("home pre-stop", status);
                break;
            }

            status = EMM42_MotorResetCurrentPositionToZero(motor);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("home set-zero", status);
                break;
            }

            runtime->applied_mode = CONVEYOR_MOTOR_MODE_STOP;
            runtime->applied_speed_rpm = 0U;
            my_printf(&huart1, "[OK][BELT] Current position set to zero.\r\n");
            break;

        case CONVEYOR_MOTOR_COMMAND_CONFIG:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
            /*
             * 配置命令只更新 F4 运行内存，不写 F4 Flash，也不写 Emm42 EEPROM。
             * 先把业务状态切到 STOP，再尝试按旧地址发停止帧，避免参数切换时继续运动。
             * 如果旧地址本来就是错的，停止帧可能发不到目标电机，但仍允许更新为新地址，
             * 否则现场无法通过参数页把错误地址修回来。
             */
            runtime->desired_mode = CONVEYOR_MOTOR_MODE_STOP;
            runtime->fresh_track_sample_flag = 0U;
            runtime->track_timeout_reported_flag = 0U;
            runtime->center_stable_count = 0U;
            runtime->centered_flag = 0U;

            status = ConveyorMotorService_ApplyStop(motor, runtime);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("runtime config pre-stop", status);
            }

            runtime->config = command->config;
            motor->address = command->config.address;
            runtime->applied_mode = CONVEYOR_MOTOR_MODE_STOP;
            runtime->applied_speed_rpm = 0U;
            runtime->applied_direction = ConveyorMotorService_MapRuntimeDirection(EMM42_MOTOR_DIRECTION_CW,
                                                                                  runtime->config.direction);
            my_printf(&huart1,
                      "[OK][BELT] Runtime config applied: addr=%u, min_step=%u, track=%u rpm, scan=%u rpm, dir=%d\r\n",
                      (unsigned int)runtime->config.address,
                      (unsigned int)runtime->config.min_step,
                      (unsigned int)runtime->config.normal_speed_rpm,
                      (unsigned int)runtime->config.scan_speed_rpm,
                      (int)runtime->config.direction);
            break;

        case CONVEYOR_MOTOR_COMMAND_STOP:
        default:
            ConveyorMotorService_ClearPendingMoveReport(runtime);
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
 * 3. `JOG`：手动指定方向和速度持续运行，直到收到停止；
 * 4. `STOP`：保持停止；
 * 5. 跟踪超时：强制停机。
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

        case CONVEYOR_MOTOR_MODE_JOG:
            status = ConveyorMotorService_ApplyJog(motor, runtime);
            if (status != EMM42_MOTOR_STATUS_OK)
            {
                ConveyorMotorService_ReportDriverError("jog command", status);
            }
            break;

        case CONVEYOR_MOTOR_MODE_POSITION:
            /*
             * 位置模式是一条一次性 Emm42 0xFD 命令。
             * 在没有电机回包解析的首版中，周期任务不重复发送，也不自动追加 stop，
             * 否则会在电机还未完成固定步数运动时立刻打断。
             */
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
    ConveyorMotor_RuntimeSnapshot_t snapshot;
    int32_t error_px = 0;
    uint8_t enable_flag = 0U;

    if (command_buffer == NULL)
    {
        return 0U;
    }

    if (strcmp(command_buffer, "BELTSTOP") == 0)
    {
        if (ConveyorMotorService_RequestStop() != 0U)
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
        if (ConveyorMotorService_RequestScan() != 0U)
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
                  "[INFO][BELT] desired=%s, applied=%s, error=%ld px, speed=%u rpm, dir=%s, track=%u rpm, scan=%u rpm, stable=%u, centered=%u\r\n",
                  ConveyorMotorService_GetModeName(snapshot.desired_mode),
                  ConveyorMotorService_GetModeName(snapshot.applied_mode),
                  (long)snapshot.latest_error_px,
                  (unsigned int)snapshot.applied_speed_rpm,
                  (snapshot.applied_direction == EMM42_MOTOR_DIRECTION_CW) ? "CW" : "CCW",
                  (unsigned int)snapshot.config_normal_speed_rpm,
                  (unsigned int)snapshot.config_scan_speed_rpm,
                  (unsigned int)snapshot.center_stable_count,
                  (unsigned int)snapshot.centered_flag);
        return 1U;
    }

    if (ConveyorMotorService_ParseTrackCommand(command_buffer, &error_px) != 0U)
    {
        (void)ConveyorMotorService_RequestTrack(error_px);
        return 1U;
    }

    if (ConveyorMotorService_ParseEnableCommand(command_buffer, &enable_flag, &error_px) != 0U)
    {
        if (enable_flag == 0U)
        {
            (void)ConveyorMotorService_RequestScan();
        }
        else
        {
            (void)ConveyorMotorService_RequestTrack(error_px);
        }
        return 1U;
    }

    if (ConveyorMotorService_ParseCameraCommand(command_buffer, &enable_flag, &error_px) != 0U)
    {
        if (enable_flag == 0U)
        {
            (void)ConveyorMotorService_RequestScan();
        }
        else
        {
            (void)ConveyorMotorService_RequestTrack(error_px);
        }
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
 * 2. 独占 `UART4` 发送传送带运动命令，避免和 USART6 上的两个摄像头运动电机冲突；
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
    runtime.config = ConveyorMotorService_GetDefaultRuntimeConfig();

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

    EMM42_MotorLoadDefaultConfig(&motor, &huart4);
    motor.address = runtime.config.address;

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
    runtime.applied_direction = ConveyorMotorService_MapRuntimeDirection(EMM42_MOTOR_DIRECTION_CW,
                                                                         runtime.config.direction);
    runtime.latest_error_px = 0;
    runtime.center_stable_count = 0U;
    runtime.centered_flag = 0U;
    runtime.fresh_track_sample_flag = 0U;
    runtime.track_timeout_reported_flag = 0U;
    runtime.last_track_update_tick = xTaskGetTickCount();
    runtime.jog_direction = EMM42_MOTOR_DIRECTION_CW;
    runtime.jog_speed_rpm = 0U;
    ConveyorMotorService_UpdateSnapshot(&runtime);

    ConveyorMotorService_ReportReady();

    for (;;)
    {
        if (xQueueReceive(g_conveyor_motor_command_queue,
                          &command,
                          pdMS_TO_TICKS(CONVEYOR_MOTOR_CONTROL_PERIOD_MS)) == pdPASS)
        {
            ConveyorMotorService_HandleQueuedCommand(&command, &motor, &runtime);
        }

        ConveyorMotorService_ControlStep(&motor, &runtime);
        ConveyorMotorService_PollPendingMoveReport(&motor, &runtime);
        ConveyorMotorService_UpdateSnapshot(&runtime);
    }
}
