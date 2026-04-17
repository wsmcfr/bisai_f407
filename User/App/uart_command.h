#ifndef USER_APP_UART_COMMAND_H
#define USER_APP_UART_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 启动USART1的DMA+空闲中断接收。
 *
 * 该函数会调用HAL的 `HAL_UARTEx_ReceiveToIdle_DMA()`，
 * 让串口在接收到一帧数据后通过空闲中断回调通知用户层。
 */
void UartCommand_StartReceive(void);

/**
 * @brief 从接收模块中取出一条待处理命令。
 * @param command_buffer 用于接收命令文本的缓存区。
 * @param buffer_size 缓存区大小，必须大于0。
 * @return uint8_t 1表示成功取到命令，0表示当前没有待处理命令。
 *
 * 该函数在任务上下文中调用。若ISR中已有新命令到达，本函数会在临界区内
 * 把命令复制到调用者缓存并清除待处理标志。
 */
uint8_t UartCommand_Fetch(char *command_buffer, uint16_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif
