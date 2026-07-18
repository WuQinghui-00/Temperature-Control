#include "uart1_comm.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USART1 RX DMA 环形缓存 */
static uint8_t s_rx_dma_buffer[UART1_RX_DMA_BUFFER_SIZE];

/* 上一次已经处理到的 DMA 写入位置 */
static volatile uint16_t s_rx_dma_old_pos;

/* 正在拼接的一行指令 */
static char s_rx_build_buffer[UART1_RX_LINE_MAX_LEN];
static uint16_t s_rx_build_index;

/* 已经接收完成、等待主循环处理的一行指令 */
static char s_rx_ready_line[UART1_RX_LINE_MAX_LEN];
static volatile uint8_t s_rx_line_ready;

/* 当前行是否因长度超限而被丢弃 */
static uint8_t s_rx_overflow;

/*
 * DMA 发送缓冲。
 * 必须是静态变量，DMA 传输完成前不能被释放或覆盖。
 */
static uint8_t s_tx_buffer[UART1_TX_BUFFER_SIZE];
static volatile uint8_t s_tx_busy;

/**
 * @brief 获取字符串长度，同时限制最大扫描长度。
 */
static size_t UART1_LimitedStrLen(const char *text, size_t max_len)
{
    size_t len = 0U;

    while ((len < max_len) && (text[len] != '\0'))
    {
        len++;
    }

    return len;
}

/**
 * @brief 将一个接收到的字节放入命令拼接状态机。
 *
 * 协议规定每条指令以 \r\n 结束。
 *
 * 处理规则：
 * 1. '\r' 直接忽略；
 * 2. '\n' 表示一条指令结束；
 * 3. 普通字符依次存入 s_rx_build_buffer；
 * 4. 超过最大长度后丢弃当前行，直到下一次 '\n'。
 */
static void UART1_Comm_PushRxByte(uint8_t ch)
{
    if (ch == '\r')
    {
        /* 忽略回车符 */
    }
    else if (ch == '\n')
    {
        /*
         * 收到换行符，代表命令结束。
         * 简化设计：当前仅缓存一条完整命令。
         * 上位机发送下一条指令前，建议等待上一条回复。
         */
        if ((s_rx_build_index > 0U) && (s_rx_overflow == 0U))
        {
            if (s_rx_line_ready == 0U)
            {
                s_rx_build_buffer[s_rx_build_index] = '\0';

                memcpy(s_rx_ready_line,
                       s_rx_build_buffer,
                       s_rx_build_index + 1U);

                s_rx_line_ready = 1U;
            }
        }

        s_rx_build_index = 0U;
        s_rx_overflow = 0U;
    }
    else
    {
        if (s_rx_overflow == 0U)
        {
            if (s_rx_build_index < (UART1_RX_LINE_MAX_LEN - 1U))
            {
                s_rx_build_buffer[s_rx_build_index] = (char)ch;
                s_rx_build_index++;
            }
            else
            {
                /*
                 * 指令太长：丢弃当前行，
                 * 直到收到下一个 '\n' 后恢复。
                 */
                s_rx_overflow = 1U;
                s_rx_build_index = 0U;
            }
        }
    }
}

/**
 * @brief 从 USART1 RX DMA 环形缓存中取出新收到的数据。
 *
 * DMA 使用 Circular 模式后，硬件会一直向 s_rx_dma_buffer 写数据。
 * 本函数通过 DMA 剩余计数器计算当前写入位置，
 * 再把上次处理位置到当前写入位置之间的新数据取出来。
 */
static void UART1_Comm_ProcessRxDmaBuffer(void)
{
    uint16_t pos;
    uint16_t i;

    if (huart1.hdmarx == NULL)
    {
        return;
    }

    /*
     * 当前 DMA 写入位置：
     * 已接收字节数 = 缓存总长度 - DMA 剩余传输数量。
     */
    pos = (uint16_t)(UART1_RX_DMA_BUFFER_SIZE -
                     __HAL_DMA_GET_COUNTER(huart1.hdmarx));

    if (pos == s_rx_dma_old_pos)
    {
        return;
    }

    if (pos > s_rx_dma_old_pos)
    {
        /*
         * 没有发生环绕：
         * 处理 [old_pos, pos) 区间。
         */
        for (i = s_rx_dma_old_pos; i < pos; i++)
        {
            UART1_Comm_PushRxByte(s_rx_dma_buffer[i]);
        }
    }
    else
    {
        /*
         * 发生环绕：
         * 先处理 [old_pos, buffer_end)，
         * 再处理 [0, pos)。
         */
        for (i = s_rx_dma_old_pos; i < UART1_RX_DMA_BUFFER_SIZE; i++)
        {
            UART1_Comm_PushRxByte(s_rx_dma_buffer[i]);
        }

        for (i = 0U; i < pos; i++)
        {
            UART1_Comm_PushRxByte(s_rx_dma_buffer[i]);
        }
    }

    s_rx_dma_old_pos = pos;
}

/**
 * @brief 启动 USART1 DMA 接收。
 */
void UART1_Comm_Start(void)
{
    s_rx_dma_old_pos = 0U;

    s_rx_build_index = 0U;
    s_rx_line_ready = 0U;
    s_rx_overflow = 0U;

    s_tx_busy = 0U;

    memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));
    memset(s_rx_build_buffer, 0, sizeof(s_rx_build_buffer));
    memset(s_rx_ready_line, 0, sizeof(s_rx_ready_line));

    /*
     * 启动 USART1_RX DMA Circular 接收。
     * CubeMX 中 RX DMA 必须配置为 Circular 模式。
     */
    (void)HAL_UART_Receive_DMA(&huart1,
                               s_rx_dma_buffer,
                               UART1_RX_DMA_BUFFER_SIZE);

    /*
     * 可选：关闭半传输中断，减少中断次数。
     * 如果你希望半满时也及时处理数据，可以注释掉这一行，
     * 并保留 HAL_UART_RxHalfCpltCallback() 中的处理。
     */
    /* __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); */

    /*
     * 清除并开启 IDLE 中断。
     * IDLE 中断用于判断一段串口数据已经接收结束。
     */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/**
 * @brief USART1 中断处理。
 */
void UART1_Comm_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    /*
     * 检测 USART IDLE 中断。
     * 上位机发送 "SETTEMPERATURE=25001\r\n" 后，
     * 总线上短暂空闲，USART 会置位 IDLE 标志。
     */
    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        UART1_Comm_ProcessRxDmaBuffer();
    }
}

/**
 * @brief DMA RX 半满或满事件处理。
 */
void UART1_Comm_OnRxDmaEvent(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART1_Comm_ProcessRxDmaBuffer();
    }
}

/**
 * @brief DMA 发送完成后的处理。
 */
void UART1_Comm_OnTxCplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_tx_busy = 0U;
    }
}

/**
 * @brief UART 错误处理。
 */
void UART1_Comm_OnError(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /*
         * 只重启接收部分。
         * 如果 HAL_UART_AbortReceive() 在你的 HAL 版本中不可用，
         * 可改为 HAL_UART_DMAStop()，但要注意它可能影响正在进行的 TX DMA。
         */
        (void)HAL_UART_AbortReceive(huart);

        s_rx_dma_old_pos = 0U;
        s_rx_build_index = 0U;
        s_rx_line_ready = 0U;
        s_rx_overflow = 0U;

        memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));

        (void)HAL_UART_Receive_DMA(&huart1,
                                   s_rx_dma_buffer,
                                   UART1_RX_DMA_BUFFER_SIZE);

        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
}

/**
 * @brief 读取一条完整命令。
 */
bool UART1_Comm_ReadLine(char *dst, size_t dst_size)
{
    size_t line_len;
    bool result = false;

    if ((dst == NULL) || (dst_size == 0U))
    {
        return false;
    }

    __disable_irq();

    if (s_rx_line_ready != 0U)
    {
        line_len = UART1_LimitedStrLen(s_rx_ready_line,
                                       UART1_RX_LINE_MAX_LEN);

        if ((line_len + 1U) <= dst_size)
        {
            memcpy(dst, s_rx_ready_line, line_len + 1U);

            s_rx_line_ready = 0U;
            result = true;
        }
    }

    __enable_irq();

    return result;
}

/**
 * @brief 使用 USART1 DMA 格式化发送数据。
 *
 * 本函数既可以直接发送普通字符串，也可以像 printf 一样格式化输出。
 *
 * 示例：
 * UART1_SendString_DMA("Hello\r\n");
 * UART1_SendString_DMA("A=%d\r\n", a);
 * UART1_SendString_DMA("T=%.3f\r\n", temperature);
 */
bool UART1_SendString_DMA(const char *format, ...)
{
    va_list args;
    int len;
    bool can_start = false;
    HAL_StatusTypeDef status;

    if (format == NULL)
    {
        return false;
    }

    /*
     * 防止 DMA 发送期间覆盖 s_tx_buffer。
     * 当前实现为单发送缓存，适合命令-应答式通信。
     */
    __disable_irq();

    if (s_tx_busy == 0U)
    {
        s_tx_busy = 1U;
        can_start = true;
    }

    __enable_irq();

    if (!can_start)
    {
        return false;
    }

    va_start(args, format);

    len = vsnprintf((char *)s_tx_buffer,
                    sizeof(s_tx_buffer),
                    format,
                    args);

    va_end(args);

    if ((len <= 0) || (len >= (int)sizeof(s_tx_buffer)))
    {
        s_tx_busy = 0U;
        return false;
    }

    status = HAL_UART_Transmit_DMA(&huart1,
                                   s_tx_buffer,
                                   (uint16_t)len);

    if (status != HAL_OK)
    {
        s_tx_busy = 0U;
        return false;
    }

    return true;
}

/**
 * @brief 查询 DMA 发送忙状态。
 */
bool UART1_Comm_IsTxBusy(void)
{
    return (s_tx_busy != 0U);
}