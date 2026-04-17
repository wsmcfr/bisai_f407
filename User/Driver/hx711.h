#ifndef USER_DRIVER_HX711_H
#define USER_DRIVER_HX711_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief HX711驱动返回状态。
 *
 * 该枚举用于统一表示HX711底层读数过程是否成功，方便应用层决定是重试、
 * 上报错误，还是执行故障恢复。
 */
typedef enum
{
    HX711_STATUS_OK = 0,
    HX711_STATUS_TIMEOUT,
    HX711_STATUS_INVALID_PARAM
} HX711_Status_t;

/**
 * @brief HX711通道/增益脉冲配置。
 *
 * HX711在读取24位数据后，需要继续补1~3个时钟脉冲来选择下一次转换的
 * 通道和增益。当前任务默认使用通道A、128倍增益。
 */
typedef enum
{
    HX711_GAIN_PULSES_A128 = 1U,
    HX711_GAIN_PULSES_B32  = 2U,
    HX711_GAIN_PULSES_A64  = 3U
} HX711_GainPulses_t;

/**
 * @brief HX711句柄。
 *
 * 该结构体保存HX711驱动工作所需的全部硬件映射与标定参数。
 * 其中：
 * - sck_port/sck_pin：时钟引脚，驱动主动输出。
 * - dout_port/dout_pin：数据引脚，HX711拉低表示数据就绪。
 * - offset：去皮偏移量，通常由空载平均值得到。
 * - scale_counts_per_g：每克对应的ADC计数差值，需用已知砝码标定。
 */
typedef struct
{
    GPIO_TypeDef *sck_port;
    uint16_t sck_pin;
    GPIO_TypeDef *dout_port;
    uint16_t dout_pin;
    HX711_GainPulses_t gain_pulses;
    int32_t offset;
    float scale_counts_per_g;
} HX711_Handle_t;

/*
 * 默认接线配置说明：
 * 1. 这里先给出一个可编译的默认映射，方便项目先跑起来。
 * 2. 若你的实际接线不是PB12/PB13，请只修改下面4个宏即可。
 * 3. 若后续改用CubeMX管理这些引脚，也应保持这里与.ioc一致。
 */
#define HX711_DEFAULT_SCK_GPIO_PORT   GPIOB
#define HX711_DEFAULT_SCK_GPIO_PIN    GPIO_PIN_12
#define HX711_DEFAULT_DOUT_GPIO_PORT  GPIOB
#define HX711_DEFAULT_DOUT_GPIO_PIN   GPIO_PIN_13

/*
 * 默认标定系数说明：
 * 1. 该值不是通用真值，只是一个占位默认值。
 * 2. 实际重量换算必须用已知重量砝码重新标定。
 * 3. 当该值保持为1.0f时，应用层输出的“克重”本质上只是计数差值。
 */
#define HX711_DEFAULT_SCALE_COUNTS_PER_G  (1.0f)

/**
 * @brief 加载HX711默认硬件配置与默认标定参数。
 * @param hx711 HX711句柄指针，不能为空。
 *
 * 该函数用于把默认接线、默认增益和默认标定参数写入句柄，
 * 便于应用层统一初始化流程。
 */
void HX711_LoadDefaultConfig(HX711_Handle_t *hx711);

/**
 * @brief 初始化HX711使用的GPIO。
 * @param hx711 HX711句柄指针，不能为空。
 * @retval HX711_Status_t 初始化结果。
 *
 * 该函数只完成GPIO方向与初始电平配置，不触发读数。
 */
HX711_Status_t HX711_Init(HX711_Handle_t *hx711);

/**
 * @brief 读取一次HX711原始24位带符号值。
 * @param hx711 HX711句柄指针，不能为空。
 * @param raw_value 用于接收原始读数的输出指针，不能为空。
 * @param timeout_ms 等待数据就绪的超时时间，单位毫秒。
 * @retval HX711_Status_t 读数结果。
 *
 * 该函数会等待DOUT拉低后，再按HX711时序移出24位数据，并补发增益选择脉冲。
 */
HX711_Status_t HX711_ReadRaw(HX711_Handle_t *hx711, int32_t *raw_value, uint32_t timeout_ms);

/**
 * @brief 对HX711执行去皮，更新offset。
 * @param hx711 HX711句柄指针，不能为空。
 * @param sample_count 参与平均的样本数量，必须大于0。
 * @param timeout_ms 每次取样等待超时时间，单位毫秒。
 * @retval HX711_Status_t 去皮结果。
 *
 * 该函数通过空载多次采样平均得到offset，用于后续重量换算。
 */
HX711_Status_t HX711_Tare(HX711_Handle_t *hx711, uint8_t sample_count, uint32_t timeout_ms);

/**
 * @brief 将原始ADC读数转换为克重。
 * @param hx711 HX711句柄指针，不能为空。
 * @param raw_value 当前原始读数。
 * @return float 换算后的克重。
 *
 * 当scale_counts_per_g尚未标定时，返回值仅作趋势观察，不代表真实重量。
 */
float HX711_ConvertToGrams(const HX711_Handle_t *hx711, int32_t raw_value);

#ifdef __cplusplus
}
#endif

#endif
