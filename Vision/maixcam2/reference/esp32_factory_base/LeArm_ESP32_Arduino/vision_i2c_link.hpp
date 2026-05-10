#ifndef __VISION_I2C_LINK_HPP_
#define __VISION_I2C_LINK_HPP_

#include <Arduino.h>
#include "Config.h"

/**
 * @brief MaixCAM2 发送到 ESP32 的视觉结果 I2C 从机地址。
 *
 * 当前约定由 MaixCAM2 作为 I2C 主机主动写入，ESP32 机械臂控制板作为从机被动接收。
 * 该地址必须和 MaixCAM2 `main.py` 里的 `I2C_SLAVE_ADDR` 保持一致。
 * `0x42` 不是硬件固定值，而是当前视觉协议人为选定的 7 位 I2C 地址。
 * 之所以选这个值，只是因为它不落在保留地址区间里、又比较容易识别和记忆；
 * 如果后续你想改地址，必须同时修改 MaixCAM2 与 ESP32 两侧常量。
 */
#define VISION_I2C_SLAVE_ADDR     (0x42U)

/**
 * @brief 视觉结果 I2C 固定帧长度，单位字节。
 *
 * 当前帧格式为：
 * `A5 5A ver flags frame_id dx dy angle score area checksum`
 * 其中 `frame_id/dx/dy/angle/area` 都按小端编码。
 */
#define VISION_I2C_FRAME_LENGTH   (16U)

/**
 * @brief 视觉 I2C 从机总线频率，单位 Hz。
 *
 * MaixCAM2 和 ESP32 之间当前是杜邦线/外接线联调，先使用 50kHz，
 * 牺牲一点刷新速度换取更宽松的上升沿和线缆容差；等地址能稳定扫到后再提高。
 */
#define VISION_I2C_FREQ_HZ        (50000U)

/**
 * @brief 视觉结果超时时间，单位毫秒。
 *
 * 若超过该时间还没有收到新帧，后续机械臂状态机应把视觉结果视为失效，
 * 避免继续沿用已经过时的像素偏差。
 */
#define VISION_I2C_TIMEOUT_MS     (200U)

/**
 * @brief I2C 视觉结果缓存结构。
 *
 * 该结构体由 I2C 接收回调更新，由主循环读取。
 * 所有像素偏差都以“抓取参考点”为零点，和 MaixCAM2 侧约定保持一致。
 */
typedef struct
{
    uint16_t frame_id;       /* 最近一次成功校验通过的视觉帧序号，用于判断数据是否更新。 */
    int16_t dx_px;           /* 目标中心相对抓取参考点的 X 像素偏差，右正左负。 */
    int16_t dy_px;           /* 目标中心相对抓取参考点的 Y 像素偏差，下正上负。 */
    int16_t angle_deg;       /* 目标角度，单位度；当前版本若视觉侧未计算可保持 0。 */
    uint16_t area;           /* 目标面积，单位像素，用于后续过滤误识别。 */
    uint8_t found;           /* 1 表示本帧找到目标，0 表示本帧未找到目标。 */
    uint8_t score;           /* 0-100 的粗略置信度，当前由视觉侧按面积映射得到。 */
    uint8_t updated;         /* 1 表示本次 `FetchLatest()` 取到的是新帧，0 表示只是旧缓存。 */
    uint32_t received_ms;    /* 最近一次成功收帧的本地毫秒时间戳，用于超时判断。 */
} VisionI2CFrame_t;

/**
 * @brief ESP32 视觉 I2C 从机诊断计数。
 *
 * 这些计数用于判断 MaixCAM2 的 I2C 操作到底有没有进入 ESP32：
 * - `request_count` 在 ESP-IDF 从机实现里表示已预装读响应的次数；
 * - `receive_count` 增加表示 Maix 主机发起过写请求，通常来自 `writeto()`；
 * - 两者都不变而 Maix 仍报 `NO 42`，说明问题仍停留在地址 ACK/电气层。
 */
typedef struct
{
    uint32_t receive_count;      /* ESP32 收到 Maix 写事务的次数，单位次，由 I2C 从机回调递增。 */
    uint32_t request_count;      /* ESP32 预装读响应的次数，单位次；Maix 读是否成功以 `read:` 日志为准。 */
    uint32_t valid_frame_count;  /* 成功通过帧头、长度和校验的视觉帧数量，单位帧。 */
    uint32_t bad_frame_count;    /* 长度、帧头或校验失败的帧数量，单位帧，用于定位协议层错误。 */
    uint32_t last_rx_ms;         /* 最近一次收到写事务的本地毫秒时间戳；0 表示从未收到。 */
    uint32_t last_req_ms;        /* 最近一次收到读事务的本地毫秒时间戳；0 表示从未收到。 */
    uint16_t last_rx_len;        /* 最近一次写事务实际读取到的字节数，单位字节。 */
    uint16_t last_recv_len;      /* Arduino Wire 回调传入的写事务长度，单位字节。 */
} VisionI2CDebugCounters_t;

class VisionI2CLink
{
private:
    VisionI2CFrame_t latest_frame;  /* 最近一次通过帧头/版本/校验和检查的视觉结果缓存。 */
    uint8_t link_ready;             /* I2C 从机初始化标志，1 表示 `Wire.begin()` 成功。 */
    volatile uint8_t new_frame_ready; /* 新帧到达标志，由 I2C 接收回调置位，由主循环读取后清零。 */
    VisionI2CDebugCounters_t debug_counters; /* I2C 读写事务诊断计数，由回调更新、主循环定期打印。 */

    void PreloadReadReply(void);
    void HandleRxBytes(const uint8_t *rx_buffer, uint16_t rx_len);
    uint8_t DecodeFrame(const uint8_t *frame_data,
                        uint8_t frame_len,
                        VisionI2CFrame_t *out_frame);

public:
    /**
     * @brief 初始化 ESP32 视觉 I2C 从机。
     * @param slave_addr I2C 从机地址，默认使用 `VISION_I2C_SLAVE_ADDR`。
     * @param sda I2C SDA 引脚，默认使用 `Config.h` 里的 `IIC_SDA`。
     * @param scl I2C SCL 引脚，默认使用 `Config.h` 里的 `IIC_SCL`。
     * @retval None
     *
     * 主要流程：
     * 1. 使用 ESP32 `Wire` 把当前控制板初始化为 I2C 从机；
     * 2. 注册 `onReceive` 回调，让 MaixCAM2 每次写入一帧时都能被动接收；
     * 3. 清空最新帧缓存和新帧标志，避免上电后误读旧状态。
     */
    void Init(uint8_t slave_addr = VISION_I2C_SLAVE_ADDR,
              uint8_t sda = IIC_SDA,
              uint8_t scl = IIC_SCL);

    /**
     * @brief 轮询处理 ESP-IDF I2C slave 接收缓存。
     * @retval None
     *
     * Arduino `Wire` 版本依赖回调；ESP-IDF 原生 slave 版本改为主循环轮询。
     * 因此 `loop()` 必须周期调用本函数，把 Maix 写入的视觉帧从驱动缓存搬到
     * `latest_frame`，否则地址会应答但应用层看不到新帧。
     */
    void ProcessPending(void);

    /**
     * @brief 读取最近一次视觉结果缓存。
     * @param out_frame 输出缓存，不能为空。
     * @param clear_new_flag 1 表示读取后清除“新帧已到达”标志，0 表示只读不清除。
     * @return uint8_t 1 表示当前已有有效缓存，0 表示还从未收到过有效帧。
     *
     * 该函数只负责把最近帧拷贝给调用方，不直接驱动机械臂动作。
     * 后续抓取状态机应基于这里返回的数据自行决定是否对准、下降或放弃。
     */
    uint8_t FetchLatest(VisionI2CFrame_t *out_frame, uint8_t clear_new_flag = 0U);

    /**
     * @brief 判断最近一次视觉结果是否仍然新鲜。
     * @param timeout_ms 允许的数据陈旧时间，单位毫秒。
     * @return uint8_t 1 表示最近帧仍在有效期内，0 表示已经超时或尚未收到过有效帧。
     *
     * 后续机械臂状态机应优先调用该接口，避免拿超时视觉偏差继续闭环。
     */
    uint8_t HasFreshFrame(uint32_t timeout_ms = VISION_I2C_TIMEOUT_MS);

    /**
     * @brief 读取当前 I2C 从机诊断计数。
     * @param out_counters 输出计数缓存，不能为空。
     * @retval None
     *
     * 该函数只复制计数，不清零。主循环可以周期打印它，用来判断 MaixCAM2 是否
     * 真的访问到了 ESP32 从机地址。
     */
    void FetchDebugCounters(VisionI2CDebugCounters_t *out_counters);

    /**
     * @brief 查询 I2C 从机是否完成初始化。
     * @return uint8_t 1 表示链路初始化成功，0 表示初始化失败。
     */
    uint8_t IsReady(void) const;
};

#endif
