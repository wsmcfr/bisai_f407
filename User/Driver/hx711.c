#include "hx711.h"

/**
 * @brief 为指定GPIO端口打开时钟。
 * @param gpio_port 目标GPIO端口。
 *
 * HX711驱动不依赖CubeMX单独为其配GPIO，因此这里在驱动内部按端口启用时钟。
 * 这样可以把用户自定义引脚的初始化完全留在User/Driver层。
 */
static void HX711_EnableGpioClock(GPIO_TypeDef *gpio_port)
{
    if (gpio_port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (gpio_port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (gpio_port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (gpio_port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (gpio_port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (gpio_port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (gpio_port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (gpio_port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
}

/**
 * @brief 产生一个很短的时序稳定延时。
 *
 * HX711时钟不需要很高速度，但要求时钟高低电平变化后有极短稳定时间。
 * 这里使用若干NOP形成与编译器无关的轻量延时，避免引入额外定时器依赖。
 */
static void HX711_ShortDelay(void)
{
    volatile uint32_t delay_index;

    for (delay_index = 0U; delay_index < 32U; ++delay_index)
    {
        __NOP();
    }
}

/**
 * @brief 判断HX711当前是否已有新数据可读。
 * @param hx711 HX711句柄指针，不能为空。
 * @return uint8_t 1表示数据就绪，0表示尚未就绪。
 *
 * HX711在数据准备好时会把DOUT拉低，因此这里只读取输入脚电平。
 */
static uint8_t HX711_IsReady(const HX711_Handle_t *hx711)
{
    return (HAL_GPIO_ReadPin(hx711->dout_port, hx711->dout_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/**
 * @brief 加载HX711默认接线和默认标定参数。
 * @param hx711 HX711句柄指针，不能为空。
 *
 * 该函数用于在应用层初始化开始前，把默认GPIO映射、增益配置和
 * 默认标定系数一次性写入句柄，避免应用层手动逐项赋值。
 */
void HX711_LoadDefaultConfig(HX711_Handle_t *hx711)
{
    if (hx711 == NULL)
    {
        return;
    }

    hx711->sck_port = HX711_DEFAULT_SCK_GPIO_PORT;
    hx711->sck_pin = HX711_DEFAULT_SCK_GPIO_PIN;
    hx711->dout_port = HX711_DEFAULT_DOUT_GPIO_PORT;
    hx711->dout_pin = HX711_DEFAULT_DOUT_GPIO_PIN;
    hx711->gain_pulses = HX711_GAIN_PULSES_A128;
    hx711->offset = 0;
    hx711->scale_counts_per_g = HX711_DEFAULT_SCALE_COUNTS_PER_G;
    hx711->rated_capacity_g = HX711_DEFAULT_RATED_CAPACITY_G;
    hx711->is_scale_calibrated = 0U;
}

/**
 * @brief 初始化HX711相关GPIO方向和默认电平。
 * @param hx711 HX711句柄指针，不能为空。
 * @retval HX711_Status_t 初始化结果。
 *
 * 该函数只负责GPIO层初始化，不启动采样。
 * 初始化完成后会把SCK拉低，保证HX711保持正常工作状态。
 */
HX711_Status_t HX711_Init(HX711_Handle_t *hx711)
{
    GPIO_InitTypeDef gpio_init = {0};

    if ((hx711 == NULL) || (hx711->sck_port == NULL) || (hx711->dout_port == NULL))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    /* 分别打开SCK和DOUT所在端口时钟，兼容它们不在同一个GPIO端口的情况。 */
    HX711_EnableGpioClock(hx711->sck_port);
    HX711_EnableGpioClock(hx711->dout_port);

    /* SCK 由MCU主动输出时钟脉冲，因此配置为推挽输出。 */
    gpio_init.Pin = hx711->sck_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hx711->sck_port, &gpio_init);

    /* DOUT 由HX711输出，因此这里只配置为输入。 */
    gpio_init.Pin = hx711->dout_pin;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hx711->dout_port, &gpio_init);

    /* 上电后保持SCK为低，避免HX711误进入掉电状态。 */
    HAL_GPIO_WritePin(hx711->sck_port, hx711->sck_pin, GPIO_PIN_RESET);

    return HX711_STATUS_OK;
}

/**
 * @brief 读取一次HX711的24位原始ADC值。
 * @param hx711 HX711句柄指针，不能为空。
 * @param raw_value 用于接收原始值的输出指针，不能为空。
 * @param timeout_ms 等待数据就绪的超时时间，单位毫秒。
 * @retval HX711_Status_t 读取结果。
 *
 * 函数流程：
 * 1. 等待 DOUT 拉低，表示新数据已准备好；
 * 2. 输出 24 个 SCK 脉冲，依次读出 24 位数据；
 * 3. 补发增益选择脉冲，确定下一次转换通道/增益；
 * 4. 完成 24 位补码到 32 位有符号整数的扩展。
 */
HX711_Status_t HX711_ReadRaw(HX711_Handle_t *hx711, int32_t *raw_value, uint32_t timeout_ms)
{
    uint32_t bit_index;
    uint32_t start_tick;
    uint32_t raw_data = 0U;

    if ((hx711 == NULL) || (raw_value == NULL))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    start_tick = HAL_GetTick();

    /* 在规定超时时间内等待 HX711 把 DOUT 拉低，否则返回超时错误。 */
    while (HX711_IsReady(hx711) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HX711_STATUS_TIMEOUT;
        }
    }

    for (bit_index = 0U; bit_index < 24U; ++bit_index)
    {
        /* SCK 上升沿后，HX711会把当前位数据放到DOUT上。 */
        HAL_GPIO_WritePin(hx711->sck_port, hx711->sck_pin, GPIO_PIN_SET);
        HX711_ShortDelay();

        raw_data <<= 1U;
        if (HAL_GPIO_ReadPin(hx711->dout_port, hx711->dout_pin) == GPIO_PIN_SET)
        {
            raw_data |= 1U;
        }

        HAL_GPIO_WritePin(hx711->sck_port, hx711->sck_pin, GPIO_PIN_RESET);
        HX711_ShortDelay();
    }

    for (bit_index = 0U; bit_index < (uint32_t)hx711->gain_pulses; ++bit_index)
    {
        /* 补发 1~3 个时钟脉冲，告诉 HX711 下一轮采样要使用的通道/增益。 */
        HAL_GPIO_WritePin(hx711->sck_port, hx711->sck_pin, GPIO_PIN_SET);
        HX711_ShortDelay();
        HAL_GPIO_WritePin(hx711->sck_port, hx711->sck_pin, GPIO_PIN_RESET);
        HX711_ShortDelay();
    }

    /* HX711输出为24位二进制补码，这里手动符号扩展到32位。 */
    if ((raw_data & 0x00800000UL) != 0U)
    {
        raw_data |= 0xFF000000UL;
    }

    *raw_value = (int32_t)raw_data;
    return HX711_STATUS_OK;
}

/**
 * @brief 对HX711执行去皮操作。
 * @param hx711 HX711句柄指针，不能为空。
 * @param sample_count 用于平均的采样点数，必须大于0。
 * @param timeout_ms 每次读取等待超时时间，单位毫秒。
 * @retval HX711_Status_t 去皮结果。
 *
 * 该函数通过多次空载采样求平均值，并把平均值保存到 `offset`，
 * 后续重量换算时会自动减去这个零点偏移。
 */
HX711_Status_t HX711_Tare(HX711_Handle_t *hx711, uint8_t sample_count, uint32_t timeout_ms)
{
    uint8_t sample_index;
    int32_t raw_value = 0;
    int64_t sum = 0;
    HX711_Status_t status;

    if ((hx711 == NULL) || (sample_count == 0U))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    for (sample_index = 0U; sample_index < sample_count; ++sample_index)
    {
        /* 逐次读取原始值，任意一次失败都直接返回，避免写入错误 offset。 */
        status = HX711_ReadRaw(hx711, &raw_value, timeout_ms);
        if (status != HX711_STATUS_OK)
        {
            return status;
        }

        sum += raw_value;
    }

    hx711->offset = (int32_t)(sum / (int64_t)sample_count);
    return HX711_STATUS_OK;
}

/**
 * @brief 将原始ADC值按当前标定参数换算为克重。
 * @param hx711 HX711句柄指针，不能为空。
 * @param raw_value 当前采样到的原始ADC值。
 * @return float 换算后的重量值，单位克。
 *
 * 若标定系数仍为0，则返回0，避免除零错误。
 * 正常情况下换算公式为 `(raw - offset) / scale`。
 */
float HX711_ConvertToGrams(const HX711_Handle_t *hx711, int32_t raw_value)
{
    if ((hx711 == NULL) || (hx711->scale_counts_per_g == 0.0f))
    {
        return 0.0f;
    }

    return ((float)(raw_value - hx711->offset)) / hx711->scale_counts_per_g;
}

/**
 * @brief 判断当前HX711是否已经具备有效标定系数。
 * @param hx711 HX711句柄指针，不能为空。
 * @return uint8_t 1表示已标定，0表示未标定。
 *
 * 该状态位由驱动层维护，避免应用层到处比较特殊浮点值。
 */
uint8_t HX711_IsCalibrated(const HX711_Handle_t *hx711)
{
    if (hx711 == NULL)
    {
        return 0U;
    }

    return hx711->is_scale_calibrated;
}

/**
 * @brief 直接设置HX711标定系数。
 * @param hx711 HX711句柄指针，不能为空。
 * @param counts_per_g 每克对应的计数差值，必须大于0。
 * @retval HX711_Status_t 设置结果。
 *
 * 该函数把标定值合法性检查统一放在驱动层处理，
 * 避免应用层命令解析和后续配置加载出现两套边界逻辑。
 */
HX711_Status_t HX711_SetScaleCountsPerGram(HX711_Handle_t *hx711, float counts_per_g)
{
    if ((hx711 == NULL) || (counts_per_g <= 0.0f))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    hx711->scale_counts_per_g = counts_per_g;
    hx711->is_scale_calibrated = 1U;
    return HX711_STATUS_OK;
}

/**
 * @brief 根据当前带载读数和已知砝码完成标定。
 * @param hx711 HX711句柄指针，不能为空。
 * @param raw_value 当前带载原始值。
 * @param known_weight_g 已知砝码重量，单位克，必须大于0。
 * @retval HX711_Status_t 标定结果。
 *
 * 若“带载值与offset相同”，说明当前没有测到有效净载荷，
 * 此时拒绝标定，避免把无意义结果写成系数。
 */
HX711_Status_t HX711_CalibrateByKnownWeight(HX711_Handle_t *hx711,
                                            int32_t raw_value,
                                            float known_weight_g)
{
    float counts_per_g;
    int32_t net_counts;

    if ((hx711 == NULL) || (known_weight_g <= 0.0f))
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    net_counts = raw_value - hx711->offset;
    if (net_counts == 0)
    {
        return HX711_STATUS_INVALID_PARAM;
    }

    counts_per_g = ((float)net_counts) / known_weight_g;
    if (counts_per_g < 0.0f)
    {
        counts_per_g = -counts_per_g;
    }

    return HX711_SetScaleCountsPerGram(hx711, counts_per_g);
}
