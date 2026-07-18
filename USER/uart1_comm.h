#ifndef __UART1_COMM_H
#define __UART1_COMM_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>

/* 单条上位机指令最大长度，包括字符串结束符 */
#define UART1_RX_LINE_MAX_LEN        64U

/* USART1 RX DMA 环形缓存长度 */
#define UART1_RX_DMA_BUFFER_SIZE     128U

/* DMA 发送缓存长度，格式化输出时建议适当加大 */
#define UART1_TX_BUFFER_SIZE         256U

/**
 * @brief 启动 USART1 的 DMA 接收。
 *
 * 在 MX_USART1_UART_Init() 之后调用一次。
 *
 * 接收方式：
 * 1. USART1_RX 使用 DMA Circular 模式持续接收；
 * 2. USART1 IDLE 中断触发后，从 DMA 环形缓存中取出新数据；
 * 3. 检测 \r\n 后生成完整指令；
 * 4. 主循环通过 UART1_Comm_ReadLine() 读取完整指令。
 */
void UART1_Comm_Start(void);

/**
 * @brief 在 USART1_IRQHandler() 中调用。
 *
 * 作用：
 * 1. 检测 USART1 IDLE 中断；
 * 2. 清除 IDLE 标志；
 * 3. 从 RX DMA 环形缓存中取出新收到的数据。
 */
void UART1_Comm_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief 在 HAL_UART_RxCpltCallback() 或 HAL_UART_RxHalfCpltCallback() 中调用。
 *
 * DMA 环形缓存半满或满时，及时处理新接收的数据，
 * 避免数据量过大时等待 IDLE 中断导致缓存覆盖。
 */
void UART1_Comm_OnRxDmaEvent(UART_HandleTypeDef *huart);

/**
 * @brief 在 HAL_UART_TxCpltCallback() 中调用。
 *
 * DMA 发送完成后，释放发送忙标志。
 */
void UART1_Comm_OnTxCplt(UART_HandleTypeDef *huart);

/**
 * @brief 在 HAL_UART_ErrorCallback() 中调用。
 *
 * UART 出现错误后，重新启动 DMA 接收。
 */
void UART1_Comm_OnError(UART_HandleTypeDef *huart);

/**
 * @brief 从接收模块读取一条完整命令。
 *
 * @param dst      用户缓存
 * @param dst_size 用户缓存长度
 *
 * @return true  读取到一条完整命令
 * @return false 当前没有完整命令
 */
bool UART1_Comm_ReadLine(char *dst, size_t dst_size);

/**
 * @brief 使用 USART1 DMA 格式化发送数据。
 *
 * @param format printf 风格格式字符串
 *
 * @return true  DMA 已成功启动
 * @return false 上一次 DMA 尚未完成，或参数错误，或格式化失败
 *
 * 使用示例：
 * UART1_SendString_DMA("Hello\r\n");
 * UART1_SendString_DMA("C=%d\r\n", current);
 * UART1_SendString_DMA("T=%.3f\r\n", temperature);
 *
 * 注意：
 * 1. 本函数内部使用静态发送缓存；
 * 2. DMA 发送完成前不能再次覆盖该缓存；
 * 3. 因此如果上一次 DMA 未完成，本函数会直接返回 false；
 * 4. 不建议在中断中调用本函数。
 */
bool UART1_SendString_DMA(const char *format, ...);

/**
 * @brief 查询 DMA 是否正在发送。
 */
bool UART1_Comm_IsTxBusy(void);

#endif