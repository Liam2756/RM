/**
 ******************************************************************************
 * @file    bsp_debug.h
 * @brief   调试工具头文件（BSP 层）
 * @note    提供 SerialPlot 上位机可视化调试接口。
 *          仅用于开发调试阶段，正式发布时可通过宏关闭。
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef BSP_DEBUG_H
#define BSP_DEBUG_H

#include "main.h"
#include <stdint.h>
#include "bsp_uart.h"

/* ========================================================================== */
/*                          SerialPlot 协议参数                                 */
/* ========================================================================== */

#define SERIAL_PLOT_FRAME_HEADER    0xABU   /**< 帧头标识字节 */
#define SERIAL_PLOT_MAX_CHANNELS    12U     /**< 最多支持同时显示的通道数 */
#define SERIAL_PLOT_FLOAT_BYTES     4U      /**< 每个 float 占用字节数（= sizeof(float)）*/

/** 发送缓冲区长度 = 1字节帧头 + N通道 × 4字节/float */
#define SERIAL_PLOT_BUF_SIZE        (SERIAL_PLOT_MAX_CHANNELS * SERIAL_PLOT_FLOAT_BYTES + 1U)

/* ========================================================================== */
/*                          LED 调试接口参数                                    */
/* ========================================================================== */

/** LED 逻辑电平。若实测板载 LED 为低电平点亮，只需对调这两个宏。 */
#define BSP_DEBUG_LED_ON_LEVEL      GPIO_PIN_SET
#define BSP_DEBUG_LED_OFF_LEVEL     GPIO_PIN_RESET

/**
 * @brief  板载 LED 选择
 */
typedef enum
{
    BSP_DEBUG_LED_R = 0U,  /**< PH12 */
    BSP_DEBUG_LED_G,       /**< PH11 */
    BSP_DEBUG_LED_B        /**< PH10 */
} BSP_DebugLed_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  通过 UART 向 SerialPlot 上位机发送浮点数据帧
 * @note   数据帧格式：[0xAB][float0低字节...高字节][float1...]...
 *         numChannels 不得超过 SERIAL_PLOT_MAX_CHANNELS，
 *         否则仅发送前 SERIAL_PLOT_MAX_CHANNELS 个通道。
 * @param  huart        目标 UART 句柄指针
 * @param  data         指向浮点数组的指针
 * @param  numChannels  通道数（数组元素个数）
 * @retval 无
 */
void BSP_Debug_SerialPlot(UART_HandleTypeDef *huart, float *data, uint16_t numChannels);

/**
 * @brief  设置板载 LED 亮灭
 * @param  led  LED 选择：BSP_DEBUG_LED_R / G / B
 * @param  on   1=点亮，0=熄灭
 * @retval 无
 */
void BSP_Debug_LEDSet(BSP_DebugLed_t led, uint8_t on);

/**
 * @brief  翻转板载 LED
 * @param  led  LED 选择：BSP_DEBUG_LED_R / G / B
 * @retval 无
 */
void BSP_Debug_LEDToggle(BSP_DebugLed_t led);

#endif /* BSP_DEBUG_H */
