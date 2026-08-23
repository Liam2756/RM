/**
 ******************************************************************************
 * @file    bsp_uart.h
 * @brief   UART 底层驱动头文件（BSP 层）
 * @note    层级说明：
 *            本文件属于【BSP 层】。
 *            职责：管理 UART DMA 收发，维护原始缓冲区。
 *            通过 __weak 回调向上层（协议解析层）通知数据就绪，
 *            上层覆盖对应 weak 函数即可接入，BSP 层不感知上层实现。
 *          数据流向：
 *            DMA 中断 → bsp_uart（存入 rx buf，触发 weak 回调）
 *                     → 协议解析层（覆盖 weak 函数，解析数据）
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* ========================================================================== */
/*                          接收缓冲区长度宏定义                                */
/* ========================================================================== */

#define VTM_DATA_LENGTH   (13)
#define DT7_DATA_LENGTH   (18)
#define RSI_DATA_LENGTH   (26)

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  通过 DMA 方式发送 UART 数据
 * @param  huart  目标 UART 句柄指针
 * @param  data   待发送数据指针
 * @param  size   发送字节数
 * @retval 无
 */
void BSP_UART_TxData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

/**
 * @brief  启动所有 UART 端口的 DMA 空闲帧接收
 * @note   在 BSP_Init() 中调用，完成后各端口持续后台接收
 * @retval 无
 */
void BSP_UART_StartReceive(void);

#endif /* BSP_UART_H */
