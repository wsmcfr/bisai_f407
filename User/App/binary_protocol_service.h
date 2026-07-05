#ifndef USER_APP_BINARY_PROTOCOL_SERVICE_H
#define USER_APP_BINARY_PROTOCOL_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief STM32MP157 与 STM32F407 自动检测二进制协议速查。
 *
 * 通信链路：
 * - MP157 通过串口连接 F4 USART1，参数为 115200 8N1，PA9(TX)/PA10(RX)，两端必须共地；
 * - 本协议用于 MP157-F4 正式主链路，正确返回只允许 `ACK/STATUS_REPORT` 二进制帧；
 * - 本协议用于 MP157-F4 正式主链路，错误返回只允许 `NACK/FAULT_REPORT` 二进制帧；
 * - 现有 `STATUS/GET/BELT...` ASCII 文本命令只保留作为断开 MP157 后的现场维护入口；
 * - 正式接 MP157 时，USART1 文本输出默认静默，不能用 `[OK]`、`[ERROR]`、`READY` 判断成功失败；
 * - F4 收到 USART1 原始帧后，应先判断是否为 `A5 5A ... 6B` 二进制帧，再回退到机械臂 `55 55 ...` 或 ASCII 文本命令。
 *
 * 帧格式：
 * | 字段 | 字节数 | 说明 |
 * | --- | --- | --- |
 * | SOF0 | 1 | 固定 `0xA5`。 |
 * | SOF1 | 1 | 固定 `0x5A`。 |
 * | VER | 1 | 首版固定 `0x01`。 |
 * | CMD | 1 | 命令字，例如 `START_CYCLE`、`VISION_POS`。 |
 * | LEN | 1 | PAYLOAD 字节数，首版最大 48 字节。 |
 * | SEQ_L/SEQ_H | 2 | 小端序帧序号，用于 ACK/NACK 对应。 |
 * | PAYLOAD | N | 命令负载，多字节整数均为小端序。 |
 * | CRC_L/CRC_H | 2 | CRC16-CCITT-FALSE，覆盖 `VER~PAYLOAD`。 |
 * | EOF | 1 | 固定 `0x6B`。 |
 *
 * 当前已实现的首轮联调命令：
 * | CMD | 名称 | 方向 | F4 动作 |
 * | --- | --- | --- | --- |
 * | `0x01` | HELLO | MP157 -> F4 | 校验协议链路并回 ACK。 |
 * | `0x02` | HEARTBEAT | MP157 -> F4 | 回 ACK，证明二进制链路可用。 |
 * | `0x10` | START_CYCLE | MP157 -> F4 | 进入本轮 cycle，并让传送带进入扫描。 |
 * | `0x11` | PAUSE_CYCLE | MP157 -> F4 | 停止传送带，保存暂停前状态。 |
 * | `0x12` | RESUME_CYCLE | MP157 -> F4 | 继续同一 cycle，扫描阶段回扫描，跟踪阶段等待新视觉坐标。 |
 * | `0x13` | STOP_CYCLE | MP157 -> F4 | 停止传送带，并清除当前 cycle。 |
 * | `0x20` | VISION_POS | MP157 -> F4 | 解出 `axis_px-target_px`，转成传送带视觉跟踪误差，成功回 ACK。 |
 * | `0x21` | VISION_LOST | MP157 -> F4 | 按原因回到扫描或停止，成功回 ACK。 |
 * | `0x22` | BELT_STOP_CENTERED | MP157 -> F4 | 停止传送带，表示零件已进入中心 ROI。 |
 * | `0x40` | QUERY_STATUS | MP157 -> F4 | 查询 F4 协议状态和传送带状态，成功回 STATUS_REPORT。 |
 * | `0x41` | BELT_MANUAL_CONTROL | MP157 -> F4 | 手动调试传送带扫描/停止，成功回 ACK。 |
 * | `0x42` | STEPPER_PARAM_SET | MP157 -> F4 | 下发三台 Emm42 的地址、最小步长、常规/对中速度、传送带上料速度和方向，成功回 ACK。 |
 * | `0x80` | ACK | F4 -> MP157 | 确认命令被接受。 |
 * | `0x81` | NACK | F4 -> MP157 | 拒绝命令并返回错误码。 |
 * | `0x82` | STATUS_REPORT | F4 -> MP157 | 查询成功后的结构化状态回包。 |
 *
 * 副作用：
 * - `START_CYCLE` 会让传送带开始巡航扫描；
 * - `PAUSE_CYCLE` 会让传送带停止，并保留当前 `cycle_id` 和暂停前状态；
 * - `RESUME_CYCLE` 会继续同一个 `cycle_id`，如果暂停前在跟踪阶段，则等待 MP157 发新的 `VISION_POS`；
 * - `VISION_POS` 会让传送带按视觉误差运动；
 * - `STOP_CYCLE` 和 `BELT_STOP_CENTERED` 会下发传送带停止命令；
 * - CRC 错误、长度错误、状态不允许等情况不会触发硬件动作。
 */

/**
 * @brief 固定帧头第 1 字节。
 */
#define BINARY_PROTOCOL_SOF0                         (0xA5U)

/**
 * @brief 固定帧头第 2 字节。
 */
#define BINARY_PROTOCOL_SOF1                         (0x5AU)

/**
 * @brief 首版协议版本。
 */
#define BINARY_PROTOCOL_VERSION                      (0x01U)

/**
 * @brief 固定帧尾。
 */
#define BINARY_PROTOCOL_EOF                          (0x6BU)

/**
 * @brief 首版允许的最大负载长度，单位字节。
 *
 * 当前 USART1 DMA 单帧缓存为 64 字节，扣除帧头、版本、命令、长度、序号、CRC 和帧尾后，
 * 48 字节能保证整帧不超过 58 字节，给字符串结束符和边界处理留下空间。
 */
#define BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH           (48U)

/**
 * @brief 协议最短帧长度，单位字节。
 *
 * 最短帧表示没有 PAYLOAD 的命令，即 `A5 5A VER CMD 00 SEQ_L SEQ_H CRC_L CRC_H 6B`。
 */
#define BINARY_PROTOCOL_MIN_FRAME_LENGTH             (10U)

/**
 * @brief 协议最大帧长度，单位字节。
 */
#define BINARY_PROTOCOL_MAX_FRAME_LENGTH             (BINARY_PROTOCOL_MIN_FRAME_LENGTH + BINARY_PROTOCOL_MAX_PAYLOAD_LENGTH)

/**
 * @brief `VISION_POS` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_VISION_POS_PAYLOAD_LENGTH    (28U)

/**
 * @brief `START_CYCLE` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_START_CYCLE_PAYLOAD_LENGTH   (6U)

/**
 * @brief `PAUSE_CYCLE` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_PAUSE_CYCLE_PAYLOAD_LENGTH   (4U)

/**
 * @brief `RESUME_CYCLE` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_RESUME_CYCLE_PAYLOAD_LENGTH  (3U)

/**
 * @brief `STOP_CYCLE` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_STOP_CYCLE_PAYLOAD_LENGTH    (4U)

/**
 * @brief `VISION_LOST` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_VISION_LOST_PAYLOAD_LENGTH   (8U)

/**
 * @brief `BELT_STOP_CENTERED` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_BELT_CENTERED_PAYLOAD_LENGTH (8U)

/**
 * @brief `QUERY_STATUS` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_QUERY_STATUS_PAYLOAD_LENGTH (3U)

/**
 * @brief `BELT_MANUAL_CONTROL` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_BELT_MANUAL_PAYLOAD_LENGTH  (4U)

/**
 * @brief `STEPPER_PARAM_SET` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、motor_count:u8、flags:u8、
 * 然后三条 9 字节电机记录：
 * role_id、address、min_step、normal_speed_rpm、scan_speed_rpm、direction。
 */
#define BINARY_PROTOCOL_STEPPER_PARAM_PAYLOAD_LENGTH (31U)

/**
 * @brief `ACTUATOR_POS_MOVE` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、actuator:u8、direction:u8、mode:u8、speed_rpm:u16、steps:u32、flags:u8。
 */
#define BINARY_PROTOCOL_ACTUATOR_POS_MOVE_PAYLOAD_LENGTH (12U)

/**
 * @brief `ACTUATOR_STOP` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、actuator:u8、flags:u8。
 */
#define BINARY_PROTOCOL_ACTUATOR_STOP_PAYLOAD_LENGTH     (4U)

/**
 * @brief `ACTUATOR_VEL_MOVE` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、actuator:u8、direction:u8、speed_rpm:u16、flags:u8。
 * 该命令只用于手动连续运动，收到停止命令前电机会保持速度模式运行。
 */
#define BINARY_PROTOCOL_ACTUATOR_VEL_MOVE_PAYLOAD_LENGTH (7U)

/**
 * @brief `ACTUATOR_HOME` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、actuator:u8、flags:u8。
 * 当前语义是“把当前位置设为新的零点”，不会让电机主动寻找限位或运动。
 */
#define BINARY_PROTOCOL_ACTUATOR_HOME_PAYLOAD_LENGTH     (4U)

/**
 * @brief `WEIGHT_CALIBRATE` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、known_weight_g:u16、flags:u8。
 * 当前称重标定是人工维护动作，不绑定自动检测流程，所以 cycle_id 固定为 0。
 */
#define BINARY_PROTOCOL_WEIGHT_CALIBRATION_PAYLOAD_LENGTH (5U)

/**
 * @brief `MODEL_READY` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、model_result:u8、part_type:u8、defect_type:u8、
 * top1_confidence:u8、image_seq:u16、model_ms:u16、option_bits:u16。
 * MP157 完成本地模型检测和 SD 卡保存后发送本命令，F4 只缓存结果，不上传云端。
 */
#define BINARY_PROTOCOL_MODEL_READY_PAYLOAD_LENGTH   (12U)

/**
 * @brief `ARM_JOB_START` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、job_id:u16、job_profile:u8、part_type:u8、
 * final_bin_hint:u8、option_bits:u16。
 * MP157 等 Z 轴回升 ACK 后发送本命令，F4 再通知 ESP32S3 机械臂抓取。
 */
#define BINARY_PROTOCOL_ARM_JOB_START_PAYLOAD_LENGTH (9U)

/**
 * @brief `FINAL_SORT_RESULT` 命令负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、job_id:u16、final_result:u8、final_bin:u8、
 * upload_status:u8、final_confidence:u8、option_bits:u16。
 * MP157 必须在完整数据和图片上传完成后再发送本命令，F4 才通知 ESP32S3 最终分拣。
 */
#define BINARY_PROTOCOL_FINAL_SORT_RESULT_PAYLOAD_LENGTH (10U)

/**
 * @brief `EVENT_REPORT` 回包负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、event_code:u8、state:u8、step_code:u8、source:u8、
 * detail_i32:i32、related_seq:u16、fault_bits:u16、reserved:u16。
 */
#define BINARY_PROTOCOL_EVENT_REPORT_PAYLOAD_LENGTH  (16U)

/**
 * @brief EVENT_REPORT 事件：执行器位置运动已经真实到位。
 *
 * F4 只有在对应张大头 Emm42 返回 `[addr FD 9F 6B]` 后才能发送该事件。
 */
#define BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_DONE     (0x14U)

/**
 * @brief EVENT_REPORT 事件：执行器位置运动等待到位超时。
 *
 * 常见原因是 Emm42 Response 参数未设置为 Reached/Both、RX 未接好、地址错误或电机堵转。
 */
#define BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_TIMEOUT  (0x15U)

/**
 * @brief `WEIGHT_RESULT` 回包负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、sample_id:u16、stable:u8、decision:u8、
 * gross_weight_mg:i32、net_weight_mg:i32、raw_adc:i32、sample_count:u16、
 * stable_window_mg:u16、duration_ms:u16、option_bits:u32。
 */
#define BINARY_PROTOCOL_WEIGHT_RESULT_PAYLOAD_LENGTH (28U)

/**
 * @brief `LDC_RESULT` 回包负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、sample_id:u16、channel_mask:u8、decision:u8、status:u8、reserved:u8、
 * ch0_raw:u32、ch0_delta:i32、ch1_raw:u32、ch1_delta:i32、duration_ms:u16、option_bits:u16。
 */
#define BINARY_PROTOCOL_LDC_RESULT_PAYLOAD_LENGTH    (28U)

/**
 * @brief `CYCLE_DONE` 回包负载长度，单位字节。
 *
 * 固定格式：cycle_id:u16、job_id:u16、final_bin:u8、model_result:u8、weight_decision:u8、
 * ldc_decision:u8、f4_state:u8、fault_level:u8、fault_bits:u16、duration_ms:u16、option_bits:u16。
 */
#define BINARY_PROTOCOL_CYCLE_DONE_PAYLOAD_LENGTH    (16U)

/**
 * @brief `ACK` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_ACK_PAYLOAD_LENGTH           (7U)

/**
 * @brief `NACK` 命令负载长度，单位字节。
 */
#define BINARY_PROTOCOL_NACK_PAYLOAD_LENGTH          (9U)

/**
 * @brief `STATUS_REPORT` 回包负载长度，单位字节。
 */
#define BINARY_PROTOCOL_STATUS_REPORT_PAYLOAD_LENGTH (24U)

/**
 * @brief `FAULT_REPORT` 回包负载长度，单位字节。
 */
#define BINARY_PROTOCOL_FAULT_REPORT_PAYLOAD_LENGTH  (16U)

/**
 * @brief 二进制协议命令字。
 */
typedef enum
{
    BINARY_PROTOCOL_CMD_HELLO = 0x01U,              /* 上电握手命令，用于确认协议版本和串口链路。 */
    BINARY_PROTOCOL_CMD_HEARTBEAT = 0x02U,          /* 二进制心跳命令，用于 MP157 周期确认 F4 在线。 */
    BINARY_PROTOCOL_CMD_START_CYCLE = 0x10U,        /* 开始一轮自动检测，F4 进入传送带扫描状态。 */
    BINARY_PROTOCOL_CMD_PAUSE_CYCLE = 0x11U,        /* 暂停当前检测流程，F4 停传送带并保存暂停前状态。 */
    BINARY_PROTOCOL_CMD_RESUME_CYCLE = 0x12U,       /* 继续当前检测流程，F4 按暂停前状态恢复或等待新视觉坐标。 */
    BINARY_PROTOCOL_CMD_STOP_CYCLE = 0x13U,         /* 停止当前检测流程，并停止传送带。 */
    BINARY_PROTOCOL_CMD_VISION_POS = 0x20U,         /* 视觉坐标命令，F4 根据坐标误差控制传送带。 */
    BINARY_PROTOCOL_CMD_VISION_LOST = 0x21U,        /* 视觉丢失命令，F4 根据原因回扫描或停机。 */
    BINARY_PROTOCOL_CMD_BELT_STOP_CENTERED = 0x22U, /* 零件已进中心 ROI，要求 F4 停止传送带。 */
    BINARY_PROTOCOL_CMD_WEIGHT_CALIBRATE = 0x30U,   /* 称重标定命令，使用已知砝码更新 HX711 运行时比例系数。 */
    BINARY_PROTOCOL_CMD_ARM_JOB_START = 0x31U,      /* 机械臂任务开始，F4 通知 ESP32S3 抓取并依次放称重/电感/分拣。 */
    BINARY_PROTOCOL_CMD_MODEL_READY = 0x32U,        /* MP157 模型检测和 SD 卡保存完成，F4 缓存模型结果但不触发云端上传。 */
    BINARY_PROTOCOL_CMD_FINAL_SORT_RESULT = 0x33U,  /* MP157 完整上传后下发最终分拣结果，F4 再通知 ESP32S3 放入对应盘。 */
    BINARY_PROTOCOL_CMD_QUERY_STATUS = 0x40U,       /* 查询 F4 和传送带结构化状态，成功返回 STATUS_REPORT。 */
    BINARY_PROTOCOL_CMD_BELT_MANUAL_CONTROL = 0x41U, /* 手动调试传送带扫描/停止，成功返回 ACK。 */
    BINARY_PROTOCOL_CMD_STEPPER_PARAM_SET = 0x42U,  /* 下发三台 Emm42 运行时参数，成功返回 ACK。 */
    BINARY_PROTOCOL_CMD_ACTUATOR_POS_MOVE = 0x50U,  /* 执行器位置运动命令，按 actuator 分发给传送带或摄像头电机服务。 */
    BINARY_PROTOCOL_CMD_ACTUATOR_STOP = 0x51U,      /* 执行器停止命令，actuator=0xFF 表示全部停止。 */
    BINARY_PROTOCOL_CMD_ACTUATOR_VEL_MOVE = 0x52U,  /* 执行器速度运动命令，手动调试时按方向持续运行到 STOP。 */
    BINARY_PROTOCOL_CMD_ACTUATOR_HOME = 0x53U,      /* 执行器当前位置设零命令，用于参数页把当前位置设为标定零点。 */
    BINARY_PROTOCOL_CMD_ACK = 0x80U,                /* ACK 回包，表示命令已被接受。 */
    BINARY_PROTOCOL_CMD_NACK = 0x81U,               /* NACK 回包，表示命令被拒绝并携带错误码。 */
    BINARY_PROTOCOL_CMD_STATUS_REPORT = 0x82U,      /* 状态上报，首轮保留给 MP157 查询和 UI 展示。 */
    BINARY_PROTOCOL_CMD_EVENT_REPORT = 0x83U,       /* 事件上报，首轮保留。 */
    BINARY_PROTOCOL_CMD_WEIGHT_RESULT = 0x84U,      /* 称重结果上报，首轮保留。 */
    BINARY_PROTOCOL_CMD_LDC_RESULT = 0x85U,         /* 电感结果上报，首轮保留。 */
    BINARY_PROTOCOL_CMD_CYCLE_DONE = 0x86U,         /* 整轮 F4 侧动作完成，首轮保留。 */
    BINARY_PROTOCOL_CMD_FAULT_REPORT = 0x87U        /* 故障上报，首轮保留。 */
} BinaryProtocol_Command_t;

/**
 * @brief 解析一帧二进制协议时可能返回的状态。
 */
typedef enum
{
    BINARY_PROTOCOL_PARSE_OK = 0,              /* 帧格式、长度、版本、CRC 和帧尾全部正确。 */
    BINARY_PROTOCOL_PARSE_NOT_BINARY,          /* 不是 `A5 5A` 开头的二进制协议帧，应继续交给其它协议解析。 */
    BINARY_PROTOCOL_PARSE_TOO_SHORT,           /* 数据长度小于最短帧长度。 */
    BINARY_PROTOCOL_PARSE_TOO_LONG,            /* 数据长度超过首版协议最大帧长。 */
    BINARY_PROTOCOL_PARSE_VERSION_ERROR,       /* 版本号不是当前 F4 支持的版本。 */
    BINARY_PROTOCOL_PARSE_LENGTH_ERROR,        /* LEN 字段和实际帧长度不匹配，或负载长度超过上限。 */
    BINARY_PROTOCOL_PARSE_EOF_ERROR,           /* 帧尾不是固定 `0x6B`。 */
    BINARY_PROTOCOL_PARSE_CRC_ERROR,           /* CRC16 校验失败。 */
    BINARY_PROTOCOL_PARSE_PARAM_ERROR          /* 调用参数为空或输出缓存非法。 */
} BinaryProtocol_ParseStatus_t;

/**
 * @brief 协议层 NACK 错误码。
 */
typedef enum
{
    BINARY_PROTOCOL_ERROR_CRC = 1U,             /* CRC 错误。 */
    BINARY_PROTOCOL_ERROR_FRAME_LENGTH = 2U,    /* 总帧长错误。 */
    BINARY_PROTOCOL_ERROR_CMD_UNKNOWN = 3U,     /* 命令不支持。 */
    BINARY_PROTOCOL_ERROR_PAYLOAD_LENGTH = 4U,  /* 命令负载长度不符合定义。 */
    BINARY_PROTOCOL_ERROR_FIELD_RANGE = 5U,     /* 字段值越界。 */
    BINARY_PROTOCOL_ERROR_STATE_NOT_ALLOWED = 6U, /* 当前状态不允许执行该命令。 */
    BINARY_PROTOCOL_ERROR_BUSY = 7U,            /* 下位机忙，暂时不接受该命令。 */
    BINARY_PROTOCOL_ERROR_CYCLE_MISMATCH = 8U,  /* cycle_id 不匹配。 */
    BINARY_PROTOCOL_ERROR_TIMEOUT = 9U,         /* 等待硬件或子模块超时。 */
    BINARY_PROTOCOL_ERROR_HARDWARE_FAULT = 10U  /* 底层硬件故障。 */
} BinaryProtocol_ErrorCode_t;

/**
 * @brief 二进制故障来源编号。
 */
typedef enum
{
    BINARY_PROTOCOL_FAULT_SOURCE_UART = 1U,        /* USART1 或协议解析相关故障。 */
    BINARY_PROTOCOL_FAULT_SOURCE_CONVEYOR = 2U,    /* 传送带 Emm42 或 UART4 控制故障。 */
    BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR = 3U, /* 摄像头运动电机或 USART6 控制故障。 */
    BINARY_PROTOCOL_FAULT_SOURCE_ARM = 4U,         /* ESP32 机械臂桥接故障。 */
    BINARY_PROTOCOL_FAULT_SOURCE_WEIGHT = 5U,      /* HX711 称重故障。 */
    BINARY_PROTOCOL_FAULT_SOURCE_LDC = 6U          /* LDC1614 电感检测故障。 */
} BinaryProtocol_FaultSource_t;

/**
 * @brief 二进制故障严重等级。
 */
typedef enum
{
    BINARY_PROTOCOL_FAULT_SEVERITY_INFO = 1U,      /* 提示级故障，不影响当前已接模块调试。 */
    BINARY_PROTOCOL_FAULT_SEVERITY_WARNING = 2U,   /* 告警级故障，需要在界面提示人工处理。 */
    BINARY_PROTOCOL_FAULT_SEVERITY_STOP = 3U       /* 停机级故障，F4 应停止可停止执行器。 */
} BinaryProtocol_FaultSeverity_t;

/**
 * @brief F4 状态故障位。
 */
typedef enum
{
    BINARY_PROTOCOL_FAULT_BIT_CONVEYOR_NOT_READY = 0x0001U, /* 传送带状态不可读或任务未就绪。 */
    BINARY_PROTOCOL_FAULT_BIT_LDC_NOT_READY = 0x0002U,      /* LDC1614 未初始化成功或当前未接入。 */
    BINARY_PROTOCOL_FAULT_BIT_WEIGHT_NOT_READY = 0x0004U,   /* HX711 未初始化成功或称重无效。 */
    BINARY_PROTOCOL_FAULT_BIT_CAMERA_MOTOR = 0x0008U,       /* 摄像头运动电机服务故障。 */
    BINARY_PROTOCOL_FAULT_BIT_ARM_LINK = 0x0010U            /* ESP32 机械臂链路故障。 */
} BinaryProtocol_FaultBit_t;

/**
 * @brief 已解析的一帧协议数据。
 *
 * 该结构体只保存指向原始负载的指针，不复制负载内容。
 * 调用者必须保证 `payload` 指向的原始帧缓存，在业务处理期间仍然有效。
 */
typedef struct
{
    uint8_t version;                              /* 协议版本号，首版应为 `0x01`。 */
    uint8_t command;                              /* 命令字，取值见 BinaryProtocol_Command_t。 */
    uint8_t payload_length;                       /* 负载长度，单位字节，范围 0~48。 */
    uint16_t sequence;                            /* 发送方帧序号，小端解析后保存，用于 ACK/NACK 对应。 */
    const uint8_t *payload;                       /* 指向原始帧中 PAYLOAD 首字节，没有负载时为 NULL。 */
} BinaryProtocol_Frame_t;

/**
 * @brief `VISION_POS` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前检测流程 ID，由 MP157 在 START_CYCLE 时分配。 */
    uint16_t frame_id;                            /* MP157 视觉帧编号，用于排查旧坐标和掉帧。 */
    uint8_t flags;                                /* 视觉标志位，bit0=坐标有效，bit1=进入 ROI，bit2=分类有效。 */
    uint8_t part_type;                            /* 零件类型枚举，F4 首轮只保存和回显，不参与分类决策。 */
    int16_t axis_px;                              /* 沿传送带运动方向的当前坐标，单位像素。 */
    int16_t target_px;                            /* 希望对准的目标线坐标，单位像素。 */
    int16_t center_x_px;                          /* 原图中心 X 坐标，单位像素，用于调试记录。 */
    int16_t center_y_px;                          /* 原图中心 Y 坐标，单位像素，用于调试记录。 */
    int16_t bbox_x_px;                            /* 视觉检测框左上角 X，单位像素。 */
    int16_t bbox_y_px;                            /* 视觉检测框左上角 Y，单位像素。 */
    int16_t bbox_w_px;                            /* 视觉检测框宽度，单位像素。 */
    int16_t bbox_h_px;                            /* 视觉检测框高度，单位像素。 */
    uint8_t confidence;                           /* 视觉定位或综合置信度，范围 0~100。 */
    uint8_t reserved;                             /* 保留字段，首版要求 MP157 填 0。 */
    uint32_t capture_ms;                          /* MP157 采集该帧时的毫秒计数，用于排查延迟。 */
} BinaryProtocol_VisionPosPayload_t;

/**
 * @brief `START_CYCLE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 本轮检测流程 ID。 */
    uint8_t mode;                                 /* 启动模式，0=完整自动检测，1=只跑传送带居中。 */
    uint16_t option_bits;                         /* 启用项位图，bit0=称重，bit1=电感，bit2=分拣。 */
    uint8_t camera_profile;                       /* 摄像头位置方案编号，调试阶段通常为 0。 */
} BinaryProtocol_StartCyclePayload_t;

/**
 * @brief `PAUSE_CYCLE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 要暂停的检测流程 ID。 */
    uint8_t pause_reason;                         /* 暂停原因，0=用户按下暂停，1=视觉不稳定，2=上位机调试。 */
    uint8_t pause_mode;                           /* 暂停方式，0=安全点暂停，1=立即停止可停止的执行器。 */
} BinaryProtocol_PauseCyclePayload_t;

/**
 * @brief `RESUME_CYCLE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 要继续的检测流程 ID。 */
    uint8_t resume_mode;                          /* 继续方式，0=从暂停点继续，1=回到扫描阶段继续。 */
} BinaryProtocol_ResumeCyclePayload_t;

/**
 * @brief `STOP_CYCLE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 要停止的检测流程 ID。 */
    uint8_t stop_reason;                          /* 停止原因，0=用户停止，1=视觉异常，2=上位机取消，3=安全故障。 */
    uint8_t stop_level;                           /* 停止等级，0=普通停止，1=急停级停止。 */
} BinaryProtocol_StopCyclePayload_t;

/**
 * @brief `VISION_LOST` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前检测流程 ID。 */
    uint16_t frame_id;                            /* MP157 视觉帧编号。 */
    uint8_t reason;                               /* 丢失原因，1=未找到目标，2=多目标，3=置信度低，4=相机离线。 */
    uint8_t confidence;                           /* 当前置信度，范围 0~100。 */
    uint16_t ms_since_seen;                       /* 距离上次看到目标的时间，单位毫秒。 */
} BinaryProtocol_VisionLostPayload_t;

/**
 * @brief `BELT_STOP_CENTERED` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前检测流程 ID。 */
    uint16_t frame_id;                            /* 触发停止的视觉帧编号。 */
    uint8_t reason;                               /* 停止原因，0=进入中心 ROI，1=MP157 主动要求停机拍照。 */
    uint16_t hold_ms;                             /* 建议保持静止等待时间，单位毫秒。 */
    uint8_t reserved;                             /* 保留字段，首版填 0。 */
} BinaryProtocol_BeltCenteredPayload_t;

/**
 * @brief `QUERY_STATUS` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* MP157 当前关注的流程 ID，0 表示只查询设备整体状态。 */
    uint8_t query_mask;                           /* 查询掩码，bit0=协议状态，bit1=传送带状态。 */
} BinaryProtocol_QueryStatusPayload_t;

/**
 * @brief `BELT_MANUAL_CONTROL` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 手动调试流程 ID，当前可为 0。 */
    uint8_t action;                               /* 手动动作，0=STOP，1=SCAN。 */
    uint8_t flags;                                /* 标志位，首版填 0。 */
} BinaryProtocol_BeltManualPayload_t;

/**
 * @brief 单台步进电机参数记录。
 */
typedef struct
{
    uint8_t role_id;                              /* 电机角色：1=传送带，2=摄像头左右，3=摄像头上下。 */
    uint8_t address;                              /* Emm42 地址，允许 1~247。 */
    uint16_t min_step;                            /* 最小步长，单位 step，允许 1~10000。 */
    uint16_t normal_speed_rpm;                    /* 常规/对中速度，单位 RPM，允许 0~5000。 */
    uint16_t scan_speed_rpm;                      /* 传送带上料扫描速度，单位 RPM；非传送带角色当前保留为 0。 */
    int8_t direction;                             /* 方向映射，1=正向，-1=反向。 */
} BinaryProtocol_StepperMotorConfig_t;

/**
 * @brief `STEPPER_PARAM_SET` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前不绑定自动检测流程，MP157 首版固定填 0。 */
    uint8_t motor_count;                          /* 电机记录数量，首版固定为 3。 */
    uint8_t flags;                                /* 保留标志位，首版固定为 0。 */
    BinaryProtocol_StepperMotorConfig_t motors[3]; /* 三台电机参数，按 role_id 识别业务角色。 */
} BinaryProtocol_StepperParamPayload_t;

/**
 * @brief `ACTUATOR_POS_MOVE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 自动流程号；手动调试允许为 0。 */
    uint8_t actuator;                             /* 执行器编号：0=传送带，1=摄像头左右，2=摄像头上下。 */
    uint8_t direction;                            /* 逻辑方向：传送带 0=后退/1=前进，左右轴 0=左移/1=右移，上下轴 0=下降/1=上升。 */
    uint8_t mode;                                 /* 位置模式，首版 0=相对位置模式。 */
    uint16_t speed_rpm;                           /* 运动速度，单位 RPM，0 表示使用对应轴默认速度。 */
    uint32_t steps;                               /* 相对移动步数，单位 step，范围 1~4294967295。 */
    uint8_t flags;                                /* 保留标志位，首版固定为 0。 */
} BinaryProtocol_ActuatorPosMovePayload_t;

/**
 * @brief `ACTUATOR_STOP` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 自动流程号；手动调试允许为 0。 */
    uint8_t actuator;                             /* 执行器编号：0=传送带，1=摄像头左右，2=摄像头上下，0xFF=全部。 */
    uint8_t flags;                                /* 保留标志位，首版固定为 0。 */
} BinaryProtocol_ActuatorStopPayload_t;

/**
 * @brief `ACTUATOR_VEL_MOVE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 自动流程号；手动调试允许为 0，当前 Qt 手动页固定使用 0。 */
    uint8_t actuator;                             /* 执行器编号：0=传送带，1=摄像头左右；上下轴手动不允许连续速度模式。 */
    uint8_t direction;                            /* 逻辑方向：传送带 0=后退/1=前进，左右轴 0=左移/1=右移。 */
    uint16_t speed_rpm;                           /* 速度模式转速，单位 RPM，范围 1~5000。 */
    uint8_t flags;                                /* 保留标志位，首版固定为 0。 */
} BinaryProtocol_ActuatorVelMovePayload_t;

/**
 * @brief `ACTUATOR_HOME` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 自动流程号；参数页标定通常为 0。 */
    uint8_t actuator;                             /* 执行器编号：0=传送带，1=摄像头左右，2=摄像头上下。 */
    uint8_t flags;                                /* 保留标志位，首版固定为 0。 */
} BinaryProtocol_ActuatorHomePayload_t;

/**
 * @brief `WEIGHT_CALIBRATE` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前人工标定不绑定自动检测流程，首版固定填 0。 */
    uint16_t known_weight_g;                      /* 已知砝码重量，单位克，F4 按当前 HX711 量程校验。 */
    uint8_t flags;                                /* 保留标志位，首版固定填 0，不能表示自动去皮或持久化。 */
} BinaryProtocol_WeightCalibrationPayload_t;

/**
 * @brief `MODEL_READY` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件自动检测流程 ID，必须匹配 active_cycle_id。 */
    uint8_t model_result;                         /* 模型综合结果：0=unknown，1=good，2=bad，3=review/uncertain。 */
    uint8_t part_type;                            /* 零件类型枚举，F4 只缓存并转发给 ESP32S3 和云端上下文。 */
    uint8_t defect_type;                          /* 缺陷类型枚举，F4 不判定，只随事件上下文保留。 */
    uint8_t top1_confidence;                      /* 模型 top1 置信度百分制，范围 0~100。 */
    uint16_t image_seq;                           /* MP157 本地图片/检测序号，用于和 SD 卡记录对齐。 */
    uint16_t model_ms;                            /* MP157 本地模型耗时，单位 ms。 */
    uint16_t option_bits;                         /* 扩展选项位，首版由 MP157 填本地检测能力位图。 */
} BinaryProtocol_ModelReadyPayload_t;

/**
 * @brief `ARM_JOB_START` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件自动检测流程 ID。 */
    uint16_t job_id;                              /* MP157 分配的机械臂任务号，用于跨阶段追踪。 */
    uint8_t job_profile;                          /* 机械臂动作方案，0=默认动作组或默认轨迹。 */
    uint8_t part_type;                            /* 零件类型，未知填 0。 */
    uint8_t final_bin_hint;                       /* 历史预留字段；最终分拣必须等待 FINAL_SORT_RESULT，不能在这里执行。 */
    uint16_t option_bits;                         /* 选项位，首版 bit0=称重，bit1=电感，bit2=最终分拣。 */
} BinaryProtocol_ArmJobStartPayload_t;

/**
 * @brief `FINAL_SORT_RESULT` 负载解析结果。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件自动检测流程 ID。 */
    uint16_t job_id;                              /* 机械臂任务号，必须和 ARM_JOB_START 缓存的任务号一致。 */
    uint8_t final_result;                         /* MP157 综合判定：1=good，2=bad，3=review/uncertain。 */
    uint8_t final_bin;                            /* 最终分拣盘：1=良品盘，2=不良品盘，3=待复核盘。 */
    uint8_t upload_status;                        /* 上传状态：1=完整数据已上传成功；2=上传失败但本地已保存，此时只能放待复核盘。 */
    uint8_t final_confidence;                     /* MP157 综合置信度百分制，范围 0~100。 */
    uint16_t option_bits;                         /* 扩展位，首版填 0 或能力位，不直接改变机械臂动作。 */
} BinaryProtocol_FinalSortResultPayload_t;

/**
 * @brief `WEIGHT_RESULT` 回包负载。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件流程 ID。 */
    uint16_t sample_id;                           /* 称重样本序号或 F4 本地递增编号。 */
    uint8_t stable;                               /* 1 表示称重窗口稳定，0 表示只能作为待复核数据。 */
    uint8_t decision;                             /* 称重判定：0=unknown，1=pass，2=fail，3=review。 */
    int32_t gross_weight_mg;                      /* 毛重，单位 mg。 */
    int32_t net_weight_mg;                        /* 净重，单位 mg。 */
    int32_t raw_adc;                              /* HX711 原始 ADC 计数。 */
    uint16_t sample_count;                        /* 本次稳定窗口样本数量。 */
    uint16_t stable_window_mg;                    /* 稳定窗口波动范围，单位 mg。 */
    uint16_t duration_ms;                         /* 本次称重耗时，单位 ms。 */
    uint32_t option_bits;                         /* 扩展位或标定状态，MP157 保存到 SD 卡和云端上下文。 */
} BinaryProtocol_WeightResultPayload_t;

/**
 * @brief `LDC_RESULT` 回包负载。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件流程 ID。 */
    uint16_t sample_id;                           /* 电感样本序号或 F4 本地递增编号。 */
    uint8_t channel_mask;                         /* 有效通道位图，bit0=CH0，bit1=CH1。 */
    uint8_t decision;                             /* 电感判定：0=unknown，1=pass，2=fail，3=review。 */
    uint8_t status;                               /* LDC 服务状态或底层错误码。 */
    uint8_t reserved;                             /* 保留字段，发送时填 0。 */
    uint32_t ch0_raw;                             /* CH0 最近稳定原始值。 */
    int32_t ch0_delta;                            /* CH0 相对基线变化量。 */
    uint32_t ch1_raw;                             /* CH1 最近稳定原始值。 */
    int32_t ch1_delta;                            /* CH1 相对基线变化量。 */
    uint16_t duration_ms;                         /* 本次电感检测耗时，单位 ms。 */
    uint16_t option_bits;                         /* 扩展位，MP157 保存到上下文。 */
} BinaryProtocol_LdcResultPayload_t;

/**
 * @brief `CYCLE_DONE` 回包负载。
 */
typedef struct
{
    uint16_t cycle_id;                            /* 当前单件流程 ID。 */
    uint16_t job_id;                              /* 机械臂任务号。 */
    uint8_t final_bin;                            /* 最终分拣目标：1=良品，2=不良品，3=待复核。 */
    uint8_t model_result;                         /* 本轮缓存的模型结果。 */
    uint8_t weight_decision;                      /* 本轮称重判定。 */
    uint8_t ldc_decision;                         /* 本轮电感判定。 */
    uint8_t f4_state;                             /* F4 协议状态。 */
    uint8_t fault_level;                          /* 0=无故障，1=提示，2=告警，3=停机。 */
    uint16_t fault_bits;                          /* 当前 F4 故障位图。 */
    uint16_t duration_ms;                         /* F4 侧动作耗时，单位 ms。 */
    uint16_t option_bits;                         /* 扩展位。 */
} BinaryProtocol_CycleDonePayload_t;

uint8_t BinaryProtocolService_IsBinaryFrame(const uint8_t *frame_buffer, uint16_t frame_length);
uint16_t BinaryProtocolService_Crc16CcittFalse(const uint8_t *data, uint16_t length);
BinaryProtocol_ParseStatus_t BinaryProtocolService_ParseFrame(const uint8_t *frame_buffer,
                                                              uint16_t frame_length,
                                                              BinaryProtocol_Frame_t *parsed_frame);
uint16_t BinaryProtocolService_BuildFrame(uint8_t command,
                                          uint16_t sequence,
                                          const uint8_t *payload,
                                          uint8_t payload_length,
                                          uint8_t *output_buffer,
                                          uint16_t output_size);
uint8_t BinaryProtocolService_DecodeStartCycle(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_StartCyclePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodePauseCycle(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_PauseCyclePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeResumeCycle(const uint8_t *payload,
                                                uint8_t payload_length,
                                                BinaryProtocol_ResumeCyclePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeStopCycle(const uint8_t *payload,
                                              uint8_t payload_length,
                                              BinaryProtocol_StopCyclePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeVisionPos(const uint8_t *payload,
                                              uint8_t payload_length,
                                              BinaryProtocol_VisionPosPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeVisionLost(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_VisionLostPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeBeltCentered(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_BeltCenteredPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeQueryStatus(const uint8_t *payload,
                                                uint8_t payload_length,
                                                BinaryProtocol_QueryStatusPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeBeltManual(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_BeltManualPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeStepperParam(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_StepperParamPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeActuatorPosMove(const uint8_t *payload,
                                                    uint8_t payload_length,
                                                    BinaryProtocol_ActuatorPosMovePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeActuatorStop(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_ActuatorStopPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeActuatorVelMove(const uint8_t *payload,
                                                    uint8_t payload_length,
                                                    BinaryProtocol_ActuatorVelMovePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeActuatorHome(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_ActuatorHomePayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeWeightCalibration(const uint8_t *payload,
                                                      uint8_t payload_length,
                                                      BinaryProtocol_WeightCalibrationPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeModelReady(const uint8_t *payload,
                                               uint8_t payload_length,
                                               BinaryProtocol_ModelReadyPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeArmJobStart(const uint8_t *payload,
                                                uint8_t payload_length,
                                                BinaryProtocol_ArmJobStartPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeFinalSortResult(const uint8_t *payload,
                                                    uint8_t payload_length,
                                                    BinaryProtocol_FinalSortResultPayload_t *decoded_payload);
uint8_t BinaryProtocolService_DecodeWeightResult(const uint8_t *payload,
                                                 uint8_t payload_length,
                                                 BinaryProtocol_WeightResultPayload_t *decoded_payload);
uint8_t BinaryProtocolService_HandleFrame(const uint8_t *frame_buffer, uint16_t frame_length);
void BinaryProtocolService_SetFaultBit(uint16_t fault_bit);
void BinaryProtocolService_ClearFaultBit(uint16_t fault_bit);
void BinaryProtocolService_ReportFault(uint16_t fault_code,
                                       uint8_t fault_source,
                                       uint8_t severity,
                                       int32_t detail_i32,
                                       uint16_t related_seq);
void BinaryProtocolService_SendEventReport(uint16_t cycle_id,
                                           uint8_t event_code,
                                           uint8_t step_code,
                                           uint8_t source,
                                           int32_t detail_i32,
                                           uint16_t related_seq);
int32_t BinaryProtocolService_BuildActuatorMoveDetail(uint8_t actuator,
                                                      uint8_t direction,
                                                      uint16_t status_code);
void BinaryProtocolService_SendWeightResult(const BinaryProtocol_WeightResultPayload_t *payload);
void BinaryProtocolService_SendLdcResult(const BinaryProtocol_LdcResultPayload_t *payload);
void BinaryProtocolService_SendCycleDone(const BinaryProtocol_CycleDonePayload_t *payload);
void BinaryProtocolService_HandleArmStageDone(uint16_t cycle_id,
                                              uint16_t job_id,
                                              uint8_t stage_id,
                                              uint8_t result,
                                              uint16_t detail_code,
                                              uint16_t elapsed_ms,
                                              uint16_t arm_fault_bits);

#ifdef __cplusplus
}
#endif

#endif
