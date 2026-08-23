/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @brief   UART 底层驱动实现（BSP 层）
 * @note    职责边界：
 *            - 维护各 UART 端口的 DMA 接收缓冲区
 *            - 在数据就绪时通过 __weak 回调通知上层，不直接调用上层函数
 *          解耦说明：
 *            原代码在 HAL 中断回调中直接调用 VTM_to_UART / DT7_to_UART 等
 *            上层函数，造成 BSP 层对上层产生依赖（层级倒置）。
 *            重构后改为 __weak 回调：上层协议模块覆盖对应 weak 函数即可，
 *            BSP 层对上层完全透明。
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "bsp_uart.h"

/* ========================================================================== */
/*                            原始接收缓冲区                                    */
/* ========================================================================== */

uint8_t g_usart1_rx_buf[VTM_DATA_LENGTH]; /**< UART1 接收缓冲：视觉模块 */
uint8_t g_usart3_rx_buf[DT7_DATA_LENGTH]; /**< UART3 接收缓冲：DT7 遥控器 */
uint8_t g_usart6_rx_buf[RSI_DATA_LENGTH]; /**< UART6 接收缓冲：裁判系统 */

/* ========================================================================== */
/*                      __weak 回调默认空实现                                   */
/* ========================================================================== */

/**
 * @brief  以下三个函数为弱符号，默认为空实现。
 *         上层协议解析模块在各自的 .c 文件中覆盖对应函数即可接入数据。
 *
 *  示例（在 vtm_driver.c 中）：
 *    void BSP_UART_VTM_DataReadyCallback(uint8_t *buf, uint16_t size)
 *    {
 *        VTM_ParseFrame(buf);  // 调用图传模块协议解析
 *    }
 */
__weak void BSP_UART_VTM_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    (void)buf;
    (void)size;
}

__weak void BSP_UART_DT7_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    (void)buf;
    (void)size;
}

__weak void BSP_UART_RSI_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    (void)buf;
    (void)size;
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  通过 DMA 方式发送 UART 数据
 */
void BSP_UART_TxData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(huart, data, size);
}

/**
 * @brief  启动所有 UART 端口的 DMA 空闲帧接收
 */
void BSP_UART_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_usart1_rx_buf, VTM_DATA_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_usart3_rx_buf, DT7_DATA_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, g_usart6_rx_buf, RSI_DATA_LENGTH);
}

/* ========================================================================== */
/*                              HAL 回调实现                                    */
/* ========================================================================== */

/**
 * @brief  UART DMA 空闲帧接收完成回调（HAL 弱函数覆盖）
 * @note   接收完成后：
 *           1. 调用对应的上层 weak 回调（通知数据就绪）
 *           2. 立即重启本端口 DMA 接收，保证连续接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart1)
    {
        BSP_UART_VTM_DataReadyCallback(g_usart1_rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_usart1_rx_buf, VTM_DATA_LENGTH);
    }
    else if (huart == &huart3)
    {
        BSP_UART_DT7_DataReadyCallback(g_usart3_rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_usart3_rx_buf, DT7_DATA_LENGTH);
    }
    else if (huart == &huart6)
    {
        BSP_UART_RSI_DataReadyCallback(g_usart6_rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, g_usart6_rx_buf, RSI_DATA_LENGTH);
    }
}
