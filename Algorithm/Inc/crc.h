/**
 ******************************************************************************
 * @file    crc.h
 * @brief   CRC8 / CRC16 校验算法头文件（算法层）
 * @note    层级说明：
 *            本文件属于【算法层】，纯查表算法，无任何硬件依赖，
 *            可在任意平台复用。
 *
 *          算法规格：
 *            CRC8  ：生成多项式 G(x) = x⁸+x⁵+x⁴+1，初始值 0xFF
 *            CRC16 ：生成多项式 G(x) = x¹⁶+x¹²+x⁵+1（CRC-16/IBM），初始值 0xFFFF
 *
 *          数据来源：RM 裁判系统串口协议使用上述两种 CRC 校验，
 *                    本模块提供计算、验证、追加三套接口，供协议层调用。
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#ifndef CRC_H
#define CRC_H

#include "main.h"
#include <stdint.h>

/* ========================================================================== */
/*                              CRC 初始值                                      */
/* ========================================================================== */

#define CRC8_INIT_VALUE     0xFFU   /**< CRC8  初始值  */
#define CRC16_INIT_VALUE    0xFFFFU /**< CRC16 初始值  */

/* ========================================================================== */
/*                              CRC8 接口                                       */
/* ========================================================================== */

/**
 * @brief  计算数据流的 CRC8 校验值
 * @param  data    数据指针
 * @param  length  数据长度（字节）
 * @param  init    CRC 初始值，通常传入 CRC8_INIT_VALUE
 * @retval CRC8 校验值
 */
uint8_t CRC8_Calc(const uint8_t *data, uint32_t length, uint8_t init);

/**
 * @brief  验证数据流末尾的 CRC8 校验字节
 * @param  data    数据指针（含末尾 CRC 字节）
 * @param  length  总长度（数据 + 1 字节 CRC）
 * @retval 1: 校验通过, 0: 校验失败
 */
uint32_t CRC8_Verify(const uint8_t *data, uint32_t length);

/**
 * @brief  计算 CRC8 并追加到数据流末尾
 * @param  data    数据指针（末尾预留 1 字节用于写入 CRC）
 * @param  length  总长度（数据 + 1 字节 CRC）
 * @retval 无
 */
void CRC8_Append(uint8_t *data, uint32_t length);

/* ========================================================================== */
/*                              CRC16 接口                                      */
/* ========================================================================== */

/**
 * @brief  计算数据流的 CRC16 校验值
 * @param  data    数据指针
 * @param  length  数据长度（字节）
 * @param  init    CRC 初始值，通常传入 CRC16_INIT_VALUE
 * @retval CRC16 校验值（小端，低字节在前）
 */
uint16_t CRC16_Calc(const uint8_t *data, uint32_t length, uint16_t init);

/**
 * @brief  验证数据流末尾的 CRC16 校验字节（小端 2 字节）
 * @param  data    数据指针（含末尾 2 字节 CRC）
 * @param  length  总长度（数据 + 2 字节 CRC）
 * @retval 1: 校验通过, 0: 校验失败
 */
uint32_t CRC16_Verify(const uint8_t *data, uint32_t length);

/**
 * @brief  计算 CRC16 并追加到数据流末尾（小端 2 字节）
 * @param  data    数据指针（末尾预留 2 字节用于写入 CRC）
 * @param  length  总长度（数据 + 2 字节 CRC）
 * @retval 无
 */
void CRC16_Append(uint8_t *data, uint32_t length);

#endif /* CRC_H */


