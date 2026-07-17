#ifndef USER_APP_UART_COMMAND_H
#define USER_APP_UART_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief 启动 MP157 主链路的 DMA+空闲中断接收，并初始化同步资源。
 *
 * 该函数完成以下工作：
 * 1. 创建串口发送互斥锁；
 * 2. 创建串口接收二值信号量；
 * 3. 启动 MP157 主链路的 `HAL_UARTEx_ReceiveToIdle_DMA()`；
 * 4. 关闭DMA半传输中断，只保留空闲或满缓冲接收事件。
 */
void UartCommand_StartReceive(void);

/**
 * @brief 获取当前 MP157-F407 正式主链路串口句柄。
 * @return UART_HandleTypeDef* 返回当前用于接收 MP157 命令和发送二进制回包的 HAL 串口句柄。
 *
 * 该函数把主链路串口集中在 `uart_command.c` 内部配置，避免协议发送端再次硬编码
 * `&huart1` 或 `&huart2`，后续现场接线变化时只需要改一个宏。
 */
UART_HandleTypeDef *UartCommand_GetMp157Huart(void);

/**
 * @brief 从接收模块中取出一条完整命令。
 * @param command_buffer 用于接收命令文本的缓存区。
 * @param buffer_size 缓存区大小，必须大于0。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到命令，0表示当前没有新命令。
 *
 * 文本命令由 MP157 主链路原始字节环形缓存取出后补 `\0`，
 * 因此仍然只能用于 `GET/TARE/CAL` 这类 ASCII 维护命令。
 */
uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size, uint32_t timeout_ms);

/**
 * @brief 从接收模块中取出一批 MP157 主链路原始串口数据。
 * @param frame_buffer 调用者提供的原始字节输出缓存，不能为 NULL。
 * @param buffer_size 输出缓存大小，必须大于0。
 * @param frame_length 实际拷贝出来的原始字节长度输出参数，不能为 NULL。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到至少一个字节，0表示当前没有新数据或参数非法。
 *
 * 该接口保留二进制数据中的 `0x00`，用于 `A5 5A ... 6B` 这类带零字节的正式二进制帧；
 * 一次返回可能是半帧、整帧或多帧粘包，调用者必须继续交给流式解析器处理。
 */
uint8_t UartCommand_FetchRaw(uint8_t *frame_buffer,
                             uint16_t buffer_size,
                             uint16_t *frame_length,
                             uint32_t timeout_ms);

/**
 * @brief 串口线程安全格式化输出。
 * @param huart 目标串口句柄，不能为空。
 * @param format `printf` 风格格式串。
 * @return int 实际输出字符数；小于0表示格式化失败。
 *
 * 该接口统一管理串口发送路径，内部使用互斥锁保护，适合多个任务复用。
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief 串口线程安全原始字节发送。
 * @param huart 目标串口句柄，不能为空。
 * @param data 待发送的原始字节缓存，不能为 NULL。
 * @param length 待发送字节数，必须大于 0。
 * @param timeout_ms HAL_UART_Transmit 的阻塞发送超时时间，单位毫秒。
 * @return HAL_StatusTypeDef HAL 串口发送结果。
 *
 * 该接口用于二进制协议 ACK/NACK 等不能经过 `printf` 字符串格式化的帧。
 * 它和 `my_printf()` 共用同一把发送互斥锁，避免文本日志和二进制帧在同一串口上交叉。
 */
HAL_StatusTypeDef UartCommand_SendRaw(UART_HandleTypeDef *huart,
                                      const uint8_t *data,
                                      uint16_t length,
                                      uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
