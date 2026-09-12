/**
 ******************************************************************************
 * @file    motor_tx.h
 * @brief   共享电机控制帧分组打包接口（设备驱动层）
 * @note    每个实例只描述一条 CAN 总线上的一个标准控制帧。
 * @version 1.0
 * @date    2026-8-11
 ******************************************************************************
 */

#ifndef MOTOR_TX_H
#define MOTOR_TX_H

#include <bsp_can.h>

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint32_t std_id;
    uint8_t data[CAN_STD_DLC];
    uint32_t flush_error_count;
} MotorTxGroup_t;

HAL_StatusTypeDef MotorTx_Init(MotorTxGroup_t *group,
                               CAN_HandleTypeDef *hcan,
                               uint32_t std_id);
HAL_StatusTypeDef MotorTx_SetSlot(MotorTxGroup_t *group,
                                  uint8_t byte_offset,
                                  int16_t value);
void MotorTx_Clear(MotorTxGroup_t *group);
HAL_StatusTypeDef MotorTx_Flush(MotorTxGroup_t *groups, uint8_t group_count);

#endif /* MOTOR_TX_H */
