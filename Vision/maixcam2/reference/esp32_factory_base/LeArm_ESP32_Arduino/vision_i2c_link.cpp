#include <Arduino.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "vision_i2c_link.hpp"

/**
 * @brief 视觉结果共享缓存临界区锁。
 *
 * ESP-IDF I2C slave 轮询处理函数和 `loop()` 主循环都会访问 `latest_frame`、
 * `new_frame_ready` 与诊断计数。这里使用 ESP32 临界区保护，避免复制结构体时
 * 读到半更新的数据。
 */
static portMUX_TYPE g_vision_i2c_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief 视觉链路固定使用 ESP32 的 I2C1 控制器。
 *
 * 设计原因：
 * - 原厂 Arduino `Wire` 默认占用 I2C0；
 * - 视觉链路使用 ESP-IDF 原生 slave 驱动挂在 I2C1，避免和原厂 `IIC` 类互相覆盖；
 * - 引脚仍使用机械臂控制板原 IIC 口：SDA=GPIO17，SCL=GPIO16。
 */
static const i2c_port_t VISION_I2C_PORT = I2C_NUM_1;
static const size_t VISION_I2C_RX_BUFFER_SIZE = 128U;
static const size_t VISION_I2C_TX_BUFFER_SIZE = 128U;
static const TickType_t VISION_I2C_NO_WAIT = 0;
static const TickType_t VISION_I2C_RX_WAIT_TICKS = pdMS_TO_TICKS(2);
static const uint32_t VISION_I2C_READ_REPLY_PRELOAD_INTERVAL_MS = 250U;

/**
 * @brief 计算一帧视觉结果的 8 位求和校验。
 * @param frame_data 原始帧缓存，不能为空。
 * @param data_len 参与校验的字节数，不包含最后一个 checksum 字节。
 * @return uint8_t 返回低 8 位求和结果。
 *
 * MaixCAM2 侧当前使用“前 N-1 字节求和取低 8 位”的轻量校验。
 */
static uint8_t VisionI2CLink_CalcChecksum(const uint8_t *frame_data, uint8_t data_len)
{
    uint16_t sum_value = 0U;
    uint8_t index = 0U;

    if (frame_data == NULL)
    {
        return 0U;
    }

    for (index = 0U; index < data_len; ++index)
    {
        sum_value += frame_data[index];
    }
    return (uint8_t)(sum_value & 0xFFU);
}

/**
 * @brief 把两个小端字节恢复为有符号 16 位整数。
 * @param low_byte 低字节。
 * @param high_byte 高字节。
 * @return int16_t 还原后的有符号值。
 */
static int16_t VisionI2CLink_UnpackInt16(uint8_t low_byte, uint8_t high_byte)
{
    uint16_t raw_value = (uint16_t)low_byte | ((uint16_t)high_byte << 8);
    return (int16_t)raw_value;
}

/**
 * @brief 把两个小端字节恢复为无符号 16 位整数。
 * @param low_byte 低字节。
 * @param high_byte 高字节。
 * @return uint16_t 还原后的无符号值。
 */
static uint16_t VisionI2CLink_UnpackUInt16(uint8_t low_byte, uint8_t high_byte)
{
    return (uint16_t)low_byte | ((uint16_t)high_byte << 8);
}

/**
 * @brief 为视觉 I2C 引脚启用 ESP32 内部上拉。
 * @param sda I2C SDA 对应的 ESP32 GPIO 编号。
 * @param scl I2C SCL 对应的 ESP32 GPIO 编号。
 * @retval None
 *
 * I2C 是开漏总线，SCL/SDA 空闲高电平必须依赖上拉。内部上拉只是诊断兜底，
 * 最终稳定联调仍建议确认外部有 4.7k~5.1k 上拉到 3.3V，不能上拉到 5V。
 */
static void VisionI2CLink_EnableInternalPullups(uint8_t sda, uint8_t scl)
{
    gpio_pulldown_dis((gpio_num_t)sda);
    gpio_pulldown_dis((gpio_num_t)scl);
    gpio_pullup_en((gpio_num_t)sda);
    gpio_pullup_en((gpio_num_t)scl);
}

/**
 * @brief 打印 ESP32 侧 I2C 引脚当前电平。
 * @param tag 日志阶段标签，例如 `before_idf_begin` 或 `after_idf_begin`。
 * @param sda I2C SDA 对应的 ESP32 GPIO 编号。
 * @param scl I2C SCL 对应的 ESP32 GPIO 编号。
 * @retval None
 */
static void VisionI2CLink_PrintLineLevels(const char *tag, uint8_t sda, uint8_t scl)
{
    Serial.printf("[VISION_I2C_LINE] %s sda(GPIO%u)=%d scl(GPIO%u)=%d\r\n",
                  tag,
                  sda,
                  gpio_get_level((gpio_num_t)sda),
                  scl,
                  gpio_get_level((gpio_num_t)scl));
}

void VisionI2CLink::Init(uint8_t slave_addr, uint8_t sda, uint8_t scl)
{
    i2c_config_t config;
    esp_err_t delete_ret = ESP_OK;
    esp_err_t param_ret = ESP_FAIL;
    esp_err_t install_ret = ESP_FAIL;

    /* 上电后先清空缓存，避免主循环误把随机 RAM 当成有效视觉结果。 */
    memset(&latest_frame, 0, sizeof(latest_frame));
    memset(&debug_counters, 0, sizeof(debug_counters));
    link_ready = 0U;
    new_frame_ready = 0U;

    /*
     * 删除旧驱动用于处理热重启或上一次程序残留。未安装时可能返回错误，
     * 这里不视为失败，因为后续 `i2c_driver_install()` 会给出真实初始化结果。
     */
    delete_ret = i2c_driver_delete(VISION_I2C_PORT);

    VisionI2CLink_EnableInternalPullups(sda, scl);
    VisionI2CLink_PrintLineLevels("before_idf_begin", sda, scl);

    memset(&config, 0, sizeof(config));
    config.mode = I2C_MODE_SLAVE;
    config.sda_io_num = (int)sda;
    config.scl_io_num = (int)scl;
    config.sda_pullup_en = true;
    config.scl_pullup_en = true;
    config.slave.addr_10bit_en = 0U;
    config.slave.slave_addr = (uint16_t)slave_addr;
    config.slave.maximum_speed = VISION_I2C_FREQ_HZ;
    config.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    param_ret = i2c_param_config(VISION_I2C_PORT, &config);
    if (param_ret == ESP_OK)
    {
        install_ret = i2c_driver_install(VISION_I2C_PORT,
                                         I2C_MODE_SLAVE,
                                         VISION_I2C_RX_BUFFER_SIZE,
                                         VISION_I2C_TX_BUFFER_SIZE,
                                         0);
    }

    if (install_ret == ESP_OK)
    {
        link_ready = 1U;
        PreloadReadReply();
    }

    VisionI2CLink_EnableInternalPullups(sda, scl);
    VisionI2CLink_PrintLineLevels("after_idf_begin", sda, scl);

    Serial.printf("[VISION_I2C] driver=esp-idf port=%d addr=0x%02X sda=%u scl=%u freq=%u delete=%s param=%s install=%s ready=%u\r\n",
                  (int)VISION_I2C_PORT,
                  slave_addr,
                  sda,
                  scl,
                  (unsigned int)VISION_I2C_FREQ_HZ,
                  esp_err_to_name(delete_ret),
                  esp_err_to_name(param_ret),
                  esp_err_to_name(install_ret),
                  link_ready);
}

void VisionI2CLink::PreloadReadReply(void)
{
    uint8_t reply_frame[4];
    uint32_t preload_count = 0U;
    uint32_t now_ms = millis();
    int write_len = 0;
    static uint32_t s_last_preload_ms = 0U;

    if (link_ready == 0U)
    {
        return;
    }

    /*
     * 读响应只用于 `i2c_probe.py` 诊断，不需要每一轮 loop 都填满 TX buffer。
     * 限速后仍能支持手动重扫，同时避免 `TXPRE` 快速增长造成误解。
     */
    if ((s_last_preload_ms != 0U) &&
        ((uint32_t)(now_ms - s_last_preload_ms) < VISION_I2C_READ_REPLY_PRELOAD_INTERVAL_MS))
    {
        return;
    }

    portENTER_CRITICAL(&g_vision_i2c_lock);
    preload_count = debug_counters.request_count + 1U;
    portEXIT_CRITICAL(&g_vision_i2c_lock);

    reply_frame[0] = 0xA5U;
    reply_frame[1] = 0x42U;
    reply_frame[2] = (uint8_t)(preload_count & 0xFFU);
    reply_frame[3] = (uint8_t)((reply_frame[0] + reply_frame[1] + reply_frame[2]) & 0xFFU);

    /*
     * ESP-IDF slave 读请求没有 Arduino Wire 那种 onRequest 回调。
     * 因此提前把 4 字节诊断响应放入 TX ring buffer，Maix `readfrom(0x42, 4)`
     * 如果地址 ACK 成功，就能读到 `A5 42 xx checksum`。
     */
    write_len = i2c_slave_write_buffer(VISION_I2C_PORT,
                                       reply_frame,
                                       sizeof(reply_frame),
                                       VISION_I2C_NO_WAIT);
    if (write_len > 0)
    {
        s_last_preload_ms = now_ms;
        portENTER_CRITICAL(&g_vision_i2c_lock);
        ++debug_counters.request_count;
        debug_counters.last_req_ms = now_ms;
        portEXIT_CRITICAL(&g_vision_i2c_lock);
    }
}

void VisionI2CLink::ProcessPending(void)
{
    uint8_t rx_buffer[VISION_I2C_RX_BUFFER_SIZE];
    int rx_len = 0;

    if (link_ready == 0U)
    {
        return;
    }

    /*
     * 给 Maix 读诊断预装少量响应。TX ring buffer 满时该调用会返回 0，
     * 不会阻塞主循环，也不会影响后续写帧接收。
     */
    PreloadReadReply();

    /*
     * 第一次读取给 RX ring buffer 2ms 等待时间，解决 Maix 刚写完后 ESP32
     * 轮询恰好早于驱动搬运数据时读到 0 的问题；后续用 0 tick 快速清空积压帧。
     */
    rx_len = i2c_slave_read_buffer(VISION_I2C_PORT,
                                   rx_buffer,
                                   sizeof(rx_buffer),
                                   VISION_I2C_RX_WAIT_TICKS);
    while (rx_len > 0)
    {
        HandleRxBytes(rx_buffer, (uint16_t)rx_len);
        rx_len = i2c_slave_read_buffer(VISION_I2C_PORT,
                                       rx_buffer,
                                       sizeof(rx_buffer),
                                       VISION_I2C_NO_WAIT);
    }
}

void VisionI2CLink::HandleRxBytes(const uint8_t *rx_buffer, uint16_t rx_len)
{
    VisionI2CFrame_t decoded_frame;

    if (rx_buffer == NULL)
    {
        return;
    }

    if (rx_len != VISION_I2C_FRAME_LENGTH)
    {
        portENTER_CRITICAL(&g_vision_i2c_lock);
        ++debug_counters.receive_count;
        ++debug_counters.bad_frame_count;
        debug_counters.last_rx_ms = millis();
        debug_counters.last_recv_len = rx_len;
        debug_counters.last_rx_len = rx_len;
        portEXIT_CRITICAL(&g_vision_i2c_lock);
        return;
    }

    if (DecodeFrame(rx_buffer, VISION_I2C_FRAME_LENGTH, &decoded_frame) == 0U)
    {
        portENTER_CRITICAL(&g_vision_i2c_lock);
        ++debug_counters.receive_count;
        ++debug_counters.bad_frame_count;
        debug_counters.last_rx_ms = millis();
        debug_counters.last_recv_len = rx_len;
        debug_counters.last_rx_len = rx_len;
        portEXIT_CRITICAL(&g_vision_i2c_lock);
        return;
    }

    portENTER_CRITICAL(&g_vision_i2c_lock);
    latest_frame = decoded_frame;
    new_frame_ready = 1U;
    ++debug_counters.receive_count;
    ++debug_counters.valid_frame_count;
    debug_counters.last_rx_ms = decoded_frame.received_ms;
    debug_counters.last_recv_len = rx_len;
    debug_counters.last_rx_len = rx_len;
    portEXIT_CRITICAL(&g_vision_i2c_lock);
}

uint8_t VisionI2CLink::FetchLatest(VisionI2CFrame_t *out_frame, uint8_t clear_new_flag)
{
    uint8_t had_valid_frame = 0U;

    if (out_frame == NULL)
    {
        return 0U;
    }

    portENTER_CRITICAL(&g_vision_i2c_lock);
    *out_frame = latest_frame;
    out_frame->updated = new_frame_ready;
    had_valid_frame = (latest_frame.received_ms != 0U) ? 1U : 0U;
    if (clear_new_flag != 0U)
    {
        new_frame_ready = 0U;
    }
    portEXIT_CRITICAL(&g_vision_i2c_lock);

    return had_valid_frame;
}

uint8_t VisionI2CLink::HasFreshFrame(uint32_t timeout_ms)
{
    uint32_t latest_received_ms = 0U;

    portENTER_CRITICAL(&g_vision_i2c_lock);
    latest_received_ms = latest_frame.received_ms;
    portEXIT_CRITICAL(&g_vision_i2c_lock);

    if (latest_received_ms == 0U)
    {
        return 0U;
    }
    return ((uint32_t)(millis() - latest_received_ms) <= timeout_ms) ? 1U : 0U;
}

void VisionI2CLink::FetchDebugCounters(VisionI2CDebugCounters_t *out_counters)
{
    if (out_counters == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_vision_i2c_lock);
    *out_counters = debug_counters;
    portEXIT_CRITICAL(&g_vision_i2c_lock);
}

uint8_t VisionI2CLink::IsReady(void) const
{
    return link_ready;
}

uint8_t VisionI2CLink::DecodeFrame(const uint8_t *frame_data,
                                   uint8_t frame_len,
                                   VisionI2CFrame_t *out_frame)
{
    if ((frame_data == NULL) || (out_frame == NULL))
    {
        return 0U;
    }

    if (frame_len != VISION_I2C_FRAME_LENGTH)
    {
        return 0U;
    }

    /* 帧头和版本号不对时直接丢弃，防止把其它 I2C 数据误当视觉结果。 */
    if ((frame_data[0] != 0xA5U) || (frame_data[1] != 0x5AU) || (frame_data[2] != 0x01U))
    {
        return 0U;
    }

    if (VisionI2CLink_CalcChecksum(frame_data, VISION_I2C_FRAME_LENGTH - 1U) != frame_data[VISION_I2C_FRAME_LENGTH - 1U])
    {
        return 0U;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->found = (frame_data[3] & 0x01U) ? 1U : 0U;
    out_frame->frame_id = VisionI2CLink_UnpackUInt16(frame_data[4], frame_data[5]);
    out_frame->dx_px = VisionI2CLink_UnpackInt16(frame_data[6], frame_data[7]);
    out_frame->dy_px = VisionI2CLink_UnpackInt16(frame_data[8], frame_data[9]);
    out_frame->angle_deg = VisionI2CLink_UnpackInt16(frame_data[10], frame_data[11]);
    out_frame->score = frame_data[12];
    out_frame->area = VisionI2CLink_UnpackUInt16(frame_data[13], frame_data[14]);
    out_frame->updated = 1U;
    out_frame->received_ms = millis();
    return 1U;
}
