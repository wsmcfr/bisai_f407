#ifndef USER_APP_UART_COMMAND_H
#define USER_APP_UART_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief 启动USART1的DMA+空闲中断接收，并初始化同步资源。
 *
 * 该函数完成以下工作：
 * 1. 创建串口发送互斥锁；
 * 2. 创建串口接收二值信号量；
 * 3. 启动 `HAL_UARTEx_ReceiveToIdle_DMA()`；
 * 4. 关闭DMA半传输中断，只保留完整帧接收事件。
 */
void UartCommand_StartReceive(void);

/**
 * @brief 从接收模块中取出一条完整命令。
 * @param command_buffer 用于接收命令文本的缓存区。
 * @param buffer_size 缓存区大小，必须大于0。
 * @param timeout_ms 等待命令的超时时间，单位毫秒。传0表示立即返回。
 * @return uint8_t 1表示成功取到命令，0表示当前没有新命令。
 *
 * 本项目当前只处理短文本命令，因此采用“单帧缓存 + 二值信号量”的轻量方案，
 * 不再引入环形缓冲区。
 */
uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size, uint32_t timeout_ms);

/**
 * @brief 串口线程安全格式化输出。
 * @param huart 目标串口句柄，不能为空。
 * @param format `printf` 风格格式串。
 * @return int 实际输出字符数；小于0表示格式化失败。
 *
 * 该接口统一管理串口发送路径，内部使用互斥锁保护，适合多个任务复用。
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
