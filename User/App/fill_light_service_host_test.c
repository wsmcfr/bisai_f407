#include "fill_light_service.h" /* 引入补光舵机角度换算公开接口，主机测试不访问 STM32 硬件。 */

#include <stdint.h> /* 提供 uint16_t 等定宽整数类型，保证角度和脉宽断言宽度稳定。 */
#include <stdio.h>  /* 提供 printf，用于输出可由脚本识别的测试结果。 */

/**
 * @brief 保存当前主机测试累计失败数量。
 *
 * 每个断言失败时递增，main() 最后据此返回非零退出码。
 */
static int g_fill_light_test_failed_count = 0;

/**
 * @brief 比较两个 16 位无符号数并记录断言结果。
 * @param name 当前断言名称，用于定位具体失败项目。
 * @param actual 被测函数实际返回值。
 * @param expected 协议或舵机标定公式要求的期望值。
 * @return 无返回值；失败时打印实际值和期望值并累计失败数。
 */
static void expect_u16(const char *name, uint16_t actual, uint16_t expected)
{
    if (actual != expected) /* 实际值与期望脉宽不一致，说明角度映射或限幅存在错误。 */
    {
        printf("FAIL: %s actual=%u expected=%u\n", /* 输出名称和数值，方便在 Windows 主机直接排查。 */
               name,
               (unsigned int)actual,
               (unsigned int)expected);
        g_fill_light_test_failed_count++; /* 记录失败，main() 最终返回 1。 */
    }
}

/**
 * @brief 验证 0~270 度到 500~2500 us 的线性换算和上限保护。
 *
 * 主要流程：
 *   1. 验证两个端点：0 度对应 500 us，270 度对应 2500 us；
 *   2. 验证中点：135 度对应 1500 us，证明整数公式没有偏移；
 *   3. 验证越界角度 300 度被限制到 270 度，防止输出超过标定上限。
 *
 * 返回值：
 *   无返回值；所有结果由 expect_u16() 汇总。
 */
static void test_angle_to_pulse_mapping(void)
{
    expect_u16("angle_0_is_500us", FillLightService_AngleToPulseUs(0U), 500U);       /* 最小角度必须输出最小脉宽。 */
    expect_u16("angle_135_is_1500us", FillLightService_AngleToPulseUs(135U), 1500U); /* 半行程必须落在脉宽中点。 */
    expect_u16("angle_270_is_2500us", FillLightService_AngleToPulseUs(270U), 2500U); /* 最大角度必须输出最大脉宽。 */
    expect_u16("angle_300_clamped", FillLightService_AngleToPulseUs(300U), 2500U);   /* 越界角度必须限制到舵机上限。 */
}

/**
 * @brief 补光舵机主机回归测试入口。
 * @return int 0 表示全部角度换算测试通过，1 表示至少一项失败。
 */
int main(void)
{
    test_angle_to_pulse_mapping(); /* 运行当前补光服务的纯算法测试，不接触 GPIO、TIM4 或 RTOS。 */

    if (g_fill_light_test_failed_count != 0) /* 存在失败时输出固定失败摘要并返回非零。 */
    {
        printf("fill light service host tests failed: %d\n", g_fill_light_test_failed_count);
        return 1;
    }

    printf("fill light service host tests passed\n"); /* 固定成功文本供脚本和人工判断。 */
    return 0;
}
