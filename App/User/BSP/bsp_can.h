/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   CAN 总线底层驱动头文件（BSP 层）
 * @note    只封装 CAN 原始帧收发，不保存任何机器人专属电机映射。
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <can.h>
#include <stdint.h>

#define CAN_STD_DLC 8U

/* ── 小云台 CAN1：GM6020 反馈 ID ── */
#define CAN1_YAW_GM6020_ID      0x205U
#define CAN1_PITCH_GM6020_ID    0x206U  /* GM6020 ID2 */

/** CAN 收发诊断计数。 */
typedef struct
{
    volatile uint32_t tx_error_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t invalid_rx_count;
} BSP_CAN_Diagnostics_t;

extern BSP_CAN_Diagnostics_t g_bsp_can1_diagnostics;

/**
 * @brief  发送一帧 8 字节标准数据帧。
 * @retval HAL_OK: 已写入发送邮箱；其他值: 发送失败且错误计数已增加。
 */
HAL_StatusTypeDef BSP_CAN_TxData(CAN_HandleTypeDef *hcan,
                                 uint32_t std_id,
                                 const uint8_t data[CAN_STD_DLC]);

/** 由设备驱动层覆盖，用于接收完整合法的 8 字节标准数据帧。 */
void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t data[CAN_STD_DLC]);

#endif /* BSP_CAN_H */
