#ifndef USER_APP_WEIGHT_SERVICE_H
#define USER_APP_WEIGHT_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 二进制称重标定请求的任务层执行结果。
 *
 * 该枚举只描述称重服务自己的业务结论，不直接暴露二进制协议错误码。
 * `binary_protocol_service.c` 会把这些结果映射成 ACK/NACK 的 error_code。
 */
typedef enum
{
    WEIGHT_SERVICE_CALIBRATION_OK = 0U,              /* 已完成标定，HX711 标定系数已经写入运行内存。 */
    WEIGHT_SERVICE_CALIBRATION_NO_CONTEXT = 1U,      /* 称重任务尚未建立 HX711 运行上下文，通常表示任务未启动。 */
    WEIGHT_SERVICE_CALIBRATION_TARE_NOT_READY = 2U,  /* 尚未成功空载去皮，不能用带载值计算比例系数。 */
    WEIGHT_SERVICE_CALIBRATION_WEIGHT_RANGE = 3U,    /* 标定砝码重量不在当前量程允许范围内。 */
    WEIGHT_SERVICE_CALIBRATION_SAMPLE_INVALID = 4U,  /* 最近一次 HX711 采样失败，不能作为标定原始值。 */
    WEIGHT_SERVICE_CALIBRATION_HARDWARE_ERROR = 5U   /* HX711 标定函数返回错误，通常是净计数为 0 或参数非法。 */
} WeightService_CalibrationResult_t;

/**
 * @brief 供二进制自动流程读取的称重快照。
 *
 * 该结构只保存最近一次称重任务已经采到的数据，不主动等待新样本。
 * F4 应在 ESP32S3 确认“零件已放到称重模块并放稳”后读取它，再打包 WEIGHT_RESULT。
 */
typedef struct
{
    uint16_t sample_id;              /* 称重任务本地递增样本号，用于 MP157 判断是否拿到新快照。 */
    uint8_t stable;                  /* 1 表示最近样本有效且去皮完成，0 表示只能作为待复核数据。 */
    uint8_t decision;                /* 0=unknown，1=pass，2=fail，3=review；首版未做重量阈值时稳定即 pass。 */
    int32_t gross_weight_mg;         /* 毛重，单位 mg；当前无独立毛重模型时与净重保持同量级。 */
    int32_t net_weight_mg;           /* 净重，单位 mg，由 HX711 标定系数换算。 */
    int32_t raw_adc;                 /* 最近一次中值滤波后的 HX711 原始计数。 */
    uint16_t sample_count;           /* 当前滤波窗口有效样本数量。 */
    uint16_t stable_window_mg;       /* 稳定窗口估计波动，首版无窗口统计时填 0。 */
    uint16_t duration_ms;            /* 本次快照对应采样周期估计耗时，单位 ms。 */
    uint32_t option_bits;            /* bit0=HX711已标定，bit1=去皮完成，bit2=最近样本有效。 */
} WeightService_Snapshot_t;

/**
 * @brief 电子称应用任务入口。
 * @param argument FreeRTOS任务参数，当前未使用。
 *
 * 该任务负责：
 * 1. 初始化 HX711 驱动，未接称重模块时只置位二进制故障位，不阻塞 MP157 主链路命令消费；
 * 2. 执行空载去皮，并周期性读取、滤波最近一次重量原始值；
 * 3. 作为 MP157 主链路唯一任务级消费者，优先分发 `A5 5A ... 6B` 二进制协议帧；
 * 4. MP157 正式链路的正确返回只允许 `ACK/STATUS_REPORT` 二进制帧；
 * 5. MP157 正式链路的错误返回只允许 `NACK/FAULT_REPORT` 二进制帧；
 * 6. 旧 `STATUS/GET/TARE/CAL <克重>` 文本入口只保留为断开 MP157 后的离线维护入口，
 *    正式接 MP157 时 USART1 文本输出默认静默，不能作为成功或失败依据。
 */
void WeightService_Task(void *argument);

/**
 * @brief 执行一次由 MP157 二进制协议触发的称重标定。
 * @param known_weight_g 已知砝码重量，单位克，合法范围由 HX711 额定量程决定。
 * @param detail 输出失败细节；成功时写 0，失败时写入状态码、克重或底层错误，不能为空。
 * @return WeightService_CalibrationResult_t 标定结果，调用方据此映射 ACK/NACK。
 *
 * 该函数只复用称重任务最近一次采样和去皮状态，不主动等待新采样。
 * 因此 MP157 侧应先让砝码放稳，再点击标定按钮。
 */
WeightService_CalibrationResult_t WeightService_RequestCalibration(uint16_t known_weight_g,
                                                                   uint16_t *detail);

/**
 * @brief 获取称重任务最近一次快照。
 * @param snapshot 输出称重快照，不能为空。
 * @return uint8_t 1 表示快照有效，0 表示称重任务尚未建立上下文。
 *
 * 该函数不阻塞、不触发 HX711 新采样，只复制 WeightService_Task 最近维护的结果。
 */
uint8_t WeightService_GetLatestSnapshot(WeightService_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
