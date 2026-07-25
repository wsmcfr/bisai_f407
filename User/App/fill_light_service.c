#include "fill_light_service.h" /* 引入补光舵机公开接口和集中标定参数。 */

#include <stdint.h> /* 提供角度、脉宽、cycle_id 和命令序号使用的定宽整数类型。 */

/**
 * @brief 把目标绝对角度换算为舵机 PWM 高电平脉宽。
 * @param angle_deg 目标角度，单位为度；大于 270 度时限制到 270 度。
 * @return uint16_t 目标脉宽，单位为微秒，默认范围为 500~2500 us。
 *
 * 公式使用 32 位中间值，避免 `(角度 * 2000)` 在 16 位计算环境中溢出。
 */
uint16_t FillLightService_AngleToPulseUs(uint16_t angle_deg)
{
    uint16_t limited_angle_deg = angle_deg; /* 保存限幅后的角度，保证输出不会超过舵机标定上限。 */
    uint32_t pulse_span_us;                 /* 保存最大与最小脉宽差值，单位为微秒。 */
    uint32_t scaled_pulse_us;               /* 保存 32 位线性换算中间结果，避免乘法溢出。 */

    if (limited_angle_deg > FILL_LIGHT_SERVO_MAX_ANGLE_DEG) /* 调用方给出越界角度时执行上限保护。 */
    {
        limited_angle_deg = FILL_LIGHT_SERVO_MAX_ANGLE_DEG; /* 把越界角度限制到 270 度物理行程。 */
    }

    pulse_span_us = (uint32_t)FILL_LIGHT_SERVO_MAX_PULSE_US - /* 计算可用于线性映射的脉宽跨度。 */
                    (uint32_t)FILL_LIGHT_SERVO_MIN_PULSE_US;
    scaled_pulse_us = (uint32_t)FILL_LIGHT_SERVO_MIN_PULSE_US + /* 从 0 度对应的最小脉宽开始累加。 */
                      (((uint32_t)limited_angle_deg * pulse_span_us) /
                       (uint32_t)FILL_LIGHT_SERVO_MAX_ANGLE_DEG);
    return (uint16_t)scaled_pulse_us; /* 返回可直接写入 TIM4 CCR1 的微秒计数值。 */
}

#ifndef BINARY_PROTOCOL_HOST_TEST

#include "binary_protocol_service.h" /* 提供补光动作枚举、完成事件和结构化故障上报接口。 */
#include "FreeRTOS.h"                /* 提供 FreeRTOS 基础类型和动态队列资源管理接口。 */
#include "cmsis_os.h"                /* 提供 osDelay，使 2 秒保持只阻塞补光专用任务。 */
#include "main.h"                    /* 提供 STM32F407 HAL、GPIO 和 TIM4 外设定义。 */
#include "queue.h"                   /* 提供 xQueueCreate/xQueueSend/xQueueReceive 非阻塞命令队列。 */

/**
 * @brief 补光舵机命令队列容量。
 *
 * 容量 1 仍允许任务取出开灯动作后暂存一条关灯请求；更多并发请求返回 BUSY，避免陈旧动作堆积。
 */
#define FILL_LIGHT_COMMAND_QUEUE_LENGTH       (1U)

/**
 * @brief TIM4 输入时钟预分频值。
 *
 * 当前 APB1 定时器时钟为 50 MHz，PSC=49 后计数频率为 1 MHz，即 1 计数等于 1 us。
 */
#define FILL_LIGHT_TIM4_PRESCALER             (49U)

/**
 * @brief TIM4 自动重装值。
 *
 * 1 MHz 计数下 ARR=19999 形成 20000 us 周期，即标准 50 Hz 舵机 PWM。
 */
#define FILL_LIGHT_TIM4_PERIOD_COUNTS         (19999U)

/**
 * @brief 补光舵机内部异步命令。
 */
typedef struct
{
    uint16_t cycle_id;   /* 发起动作的自动检测流程 ID，由协议任务写入、补光任务读取。 */
    uint16_t related_seq; /* 原始 MP157 命令序号，完成事件必须原样回传供严格匹配。 */
    uint8_t action;      /* 绝对开关动作：0=关灯/0 度，1=开灯/270 度。 */
} FillLightService_Command_t;

/**
 * @brief TIM4 PWM HAL 句柄。
 *
 * 该句柄只由补光服务初始化和任务线程使用，不与其它模块共享。
 */
static TIM_HandleTypeDef g_fill_light_tim4_handle;

/**
 * @brief 补光命令队列句柄。
 *
 * 初始化函数创建队列，协议线程只写入，补光任务独占读取。
 */
static QueueHandle_t g_fill_light_command_queue = NULL;

/**
 * @brief 初始化 PB6/AF2 和 TIM4_CH1 为 50 Hz 舵机 PWM。
 * @return uint8_t 1 表示 GPIO、TIM4 和通道配置成功，0 表示任一 HAL 配置失败。
 *
 * 初始化完成后不启动 PWM，只有队列收到动作请求时才开始输出目标脉宽。
 */
static uint8_t FillLightService_InitPwmHardware(void)
{
    GPIO_InitTypeDef gpio_init;      /* 保存 PB6 复用输出配置，函数内初始化后立即交给 HAL。 */
    TIM_OC_InitTypeDef pwm_channel = {0}; /* 保存 TIM4_CH1 PWM1 输出参数；整体清零避免保留字段含随机值。 */

    __HAL_RCC_GPIOB_CLK_ENABLE(); /* 先打开 GPIOB 时钟，否则 PB6 配置不会生效。 */
    __HAL_RCC_TIM4_CLK_ENABLE();  /* 再打开 TIM4 外设时钟，后续 HAL 初始化才能访问寄存器。 */

    gpio_init.Pin = GPIO_PIN_6;              /* PB6 对应 TIM4 通道 1 输出脚。 */
    gpio_init.Mode = GPIO_MODE_AF_PP;        /* 舵机 PWM 使用复用推挽输出。 */
    gpio_init.Pull = GPIO_NOPULL;            /* 控制线由定时器主动驱动，不启用内部上下拉。 */
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;   /* 50 Hz 信号无需高速翻转，降低不必要的边沿干扰。 */
    gpio_init.Alternate = GPIO_AF2_TIM4;      /* 数据手册规定 PB6 的 AF2 为 TIM4_CH1。 */
    HAL_GPIO_Init(GPIOB, &gpio_init);         /* 将配置写入 PB6 复用和输出寄存器。 */

    g_fill_light_tim4_handle.Instance = TIM4;                              /* 选择 APB1 上的 TIM4 外设。 */
    g_fill_light_tim4_handle.Init.Prescaler = FILL_LIGHT_TIM4_PRESCALER;    /* 把 50 MHz 分频到 1 MHz。 */
    g_fill_light_tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 使用向上计数产生固定周期 PWM。 */
    g_fill_light_tim4_handle.Init.Period = FILL_LIGHT_TIM4_PERIOD_COUNTS;   /* 20 ms 周期对应 50 Hz。 */
    g_fill_light_tim4_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 不再对定时器采样时钟额外分频。 */
    g_fill_light_tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /* 参数固定，关闭预装载简化启动。 */
    if (HAL_TIM_PWM_Init(&g_fill_light_tim4_handle) != HAL_OK)              /* 初始化 TIM4 PWM 基础寄存器。 */
    {
        return 0U;                                                          /* HAL 初始化失败，禁止创建可用服务。 */
    }

    pwm_channel.OCMode = TIM_OCMODE_PWM1;          /* CCR1 之前输出高电平，之后输出低电平。 */
    pwm_channel.Pulse = 0U;                        /* 初始化阶段保持无有效高脉宽，避免舵机突然动作。 */
    pwm_channel.OCPolarity = TIM_OCPOLARITY_HIGH;  /* 标准舵机使用高电平脉宽表达目标角度。 */
    pwm_channel.OCFastMode = TIM_OCFAST_DISABLE;   /* 50 Hz 控制无需快速比较模式。 */
    if (HAL_TIM_PWM_ConfigChannel(&g_fill_light_tim4_handle, /* 把 PWM1 参数绑定到 PB6 对应的通道 1。 */
                                  &pwm_channel,
                                  TIM_CHANNEL_1) != HAL_OK)
    {
        return 0U;                                  /* 通道配置失败时不得输出不确定波形。 */
    }

    __HAL_TIM_SET_COMPARE(&g_fill_light_tim4_handle, TIM_CHANNEL_1, 0U); /* 确保首次启动前 CCR1 为 0。 */
    return 1U;                                                           /* GPIO 和定时器配置均成功。 */
}

/**
 * @brief 初始化补光舵机服务的队列和 PWM 硬件。
 * @return uint8_t 1 表示初始化成功，0 表示队列创建或硬件初始化失败。
 */
uint8_t FillLightService_Init(void)
{
    if (g_fill_light_command_queue != NULL) /* 服务已经初始化时直接返回成功，避免重复分配队列。 */
    {
        return 1U;
    }

    g_fill_light_command_queue = xQueueCreate(FILL_LIGHT_COMMAND_QUEUE_LENGTH, /* 创建固定长度的异步动作队列。 */
                                              sizeof(FillLightService_Command_t));
    if (g_fill_light_command_queue == NULL) /* FreeRTOS 堆不足时队列创建失败。 */
    {
        return 0U;                          /* 不存在队列时协议层不能安全接收动作请求。 */
    }

    if (FillLightService_InitPwmHardware() == 0U) /* 队列成功后配置 PB6/TIM4_CH1。 */
    {
        vQueueDelete(g_fill_light_command_queue); /* 释放已创建队列，避免初始化失败后的资源泄漏。 */
        g_fill_light_command_queue = NULL;         /* 清空句柄，让后续请求明确返回未就绪。 */
        return 0U;                                 /* 硬件初始化失败，交由 freertos.c 进入统一错误处理。 */
    }

    return 1U; /* 队列和 PWM 硬件均已准备好。 */
}

/**
 * @brief 非阻塞投递补光舵机绝对开关动作。
 * @param cycle_id 当前自动检测流程 ID。
 * @param action 0 表示关灯回 0 度，1 表示开灯到 270 度。
 * @param related_seq 原始 MP157 命令序号。
 * @return uint8_t 1 表示入队成功，0 表示参数非法、服务未就绪或队列已满。
 */
uint8_t FillLightService_Request(uint16_t cycle_id,
                                uint8_t action,
                                uint16_t related_seq)
{
    FillLightService_Command_t command; /* 保存一次完整异步动作请求，按值复制到队列。 */

    if ((g_fill_light_command_queue == NULL) ||                   /* 初始化尚未完成时不能接收命令。 */
        (cycle_id == 0U) ||                                       /* 自动补光动作必须绑定非零检测流程。 */
        (action > (uint8_t)BINARY_PROTOCOL_FILL_LIGHT_ACTION_ON)) /* action 只允许关灯 0 或开灯 1。 */
    {
        return 0U;                                                /* 参数或状态非法，由协议层返回结构化 NACK。 */
    }

    command.cycle_id = cycle_id;       /* 保存流程号，用于完成事件跨线程匹配。 */
    command.related_seq = related_seq; /* 保存原命令序号，用于区分同一轮的开灯和关灯请求。 */
    command.action = action;           /* 保存绝对目标动作，任务读取后映射到角度宏。 */

    if (xQueueSend(g_fill_light_command_queue, &command, 0U) != pdPASS) /* 协议线程禁止等待 2 秒动作完成。 */
    {
        return 0U; /* 队列已满时立即返回失败，让 MP157 收到 BUSY 后决定重试或停止。 */
    }

    return 1U; /* 命令已复制到队列，真实完成仍需等待 EVENT_REPORT。 */
}

/**
 * @brief 补光舵机专用任务，串行输出目标 PWM 并在 2 秒后停止。
 * @param argument RTOS 任务参数，当前未使用。
 * @return 无返回值；任务永久等待队列。
 */
void FillLightService_Task(void *argument)
{
    FillLightService_Command_t command; /* 保存从队列取出的当前动作请求。 */
    uint16_t target_angle_deg;          /* 保存本次开关动作对应的绝对舵机角度。 */
    uint16_t pulse_us;                  /* 保存写入 TIM4 CCR1 的高电平脉宽，单位微秒。 */

    (void)argument; /* 当前任务不需要外部上下文，显式忽略参数避免编译告警。 */

    for (;;) /* 补光任务与系统同生命周期运行，只在收到队列命令时动作。 */
    {
        if (xQueueReceive(g_fill_light_command_queue, &command, portMAX_DELAY) != pdPASS) /* 阻塞等待下一条动作。 */
        {
            continue; /* 理论上的队列接收异常不访问硬件，继续等待后续有效命令。 */
        }

        if (command.action == (uint8_t)BINARY_PROTOCOL_FILL_LIGHT_ACTION_ON) /* 开灯动作选择可标定的 270 度位置。 */
        {
            target_angle_deg = FILL_LIGHT_SERVO_ON_ANGLE_DEG;
        }
        else /* 解码和入队已确保其余唯一合法值为关灯动作。 */
        {
            target_angle_deg = FILL_LIGHT_SERVO_OFF_ANGLE_DEG; /* 关灯动作回到可标定的 0 度位置。 */
        }

        pulse_us = FillLightService_AngleToPulseUs(target_angle_deg); /* 使用统一公式得到目标高电平宽度。 */
        __HAL_TIM_SET_COMPARE(&g_fill_light_tim4_handle,              /* CCR1 的计数单位为 1 us，可直接写脉宽。 */
                              TIM_CHANNEL_1,
                              pulse_us);
        if (HAL_TIM_PWM_Start(&g_fill_light_tim4_handle, TIM_CHANNEL_1) != HAL_OK) /* 开始从 PB6 输出 50 Hz PWM。 */
        {
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_FILL_LIGHT); /* 记录补光硬件故障位。 */
            BinaryProtocolService_ReportFault(1U,                                 /* 故障码 1 表示 PWM 启动失败。 */
                                              BINARY_PROTOCOL_FAULT_SOURCE_FILL_LIGHT,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_STOP,
                                              (int32_t)target_angle_deg,
                                              command.related_seq);
            continue; /* 启动失败时不等待 2 秒，也不发送伪完成事件。 */
        }

        osDelay(FILL_LIGHT_SERVO_MOVE_HOLD_MS); /* 仅阻塞本专用任务 2 秒，让舵机到达目标机械位置。 */
        if (HAL_TIM_PWM_Stop(&g_fill_light_tim4_handle, TIM_CHANNEL_1) != HAL_OK) /* 到时停止 PWM，降低舵机持续发热。 */
        {
            __HAL_TIM_SET_COMPARE(&g_fill_light_tim4_handle, TIM_CHANNEL_1, 0U); /* 即使 HAL 停止失败也先把高脉宽清零。 */
            BinaryProtocolService_SetFaultBit(BINARY_PROTOCOL_FAULT_BIT_FILL_LIGHT); /* 记录 PWM 无法可靠停止的硬件故障。 */
            BinaryProtocolService_ReportFault(2U,                                  /* 故障码 2 表示 PWM 停止失败。 */
                                              BINARY_PROTOCOL_FAULT_SOURCE_FILL_LIGHT,
                                              BINARY_PROTOCOL_FAULT_SEVERITY_STOP,
                                              (int32_t)target_angle_deg,
                                              command.related_seq);
            continue; /* 停止失败时不得向 MP157 上报伪完成事件。 */
        }
        __HAL_TIM_SET_COMPARE(&g_fill_light_tim4_handle, TIM_CHANNEL_1, 0U); /* 清零 CCR1，确保下次启动前输出安全。 */

        BinaryProtocolService_ClearFaultBit(BINARY_PROTOCOL_FAULT_BIT_FILL_LIGHT); /* 动作完成后清除历史 PWM 故障位。 */
        BinaryProtocolService_SendEventReport(command.cycle_id,                  /* 上报严格可匹配的动作完成事件。 */
                                              BINARY_PROTOCOL_EVENT_FILL_LIGHT_MOVE_DONE,
                                              command.action,
                                              BINARY_PROTOCOL_FAULT_SOURCE_FILL_LIGHT,
                                              (int32_t)target_angle_deg,
                                              command.related_seq);
    }
}

#endif
