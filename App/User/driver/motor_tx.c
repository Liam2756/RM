/**
 ******************************************************************************
 * @file    motor_tx.c
 * @brief   共享电机控制帧分组打包实现（设备驱动层）
 * @version 1.0
 * @date    2026-8-11
 ******************************************************************************
 */

#include <motor_tx.h>

HAL_StatusTypeDef MotorTx_Init(MotorTxGroup_t *group,
                               CAN_HandleTypeDef *hcan,
                               uint32_t std_id)
{
    if ((group == 0) || (hcan == 0) || (std_id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    group->hcan = hcan;
    group->std_id = std_id;
    group->flush_error_count = 0U;
    MotorTx_Clear(group);
    return HAL_OK;
}

HAL_StatusTypeDef MotorTx_SetSlot(MotorTxGroup_t *group,
                                  uint8_t byte_offset,
                                  int16_t value)
{
    uint16_t raw;

    if ((group == 0) || (byte_offset > 6U) || ((byte_offset & 1U) != 0U))
    {
        return HAL_ERROR;
    }

    raw = (uint16_t)value;
    group->data[byte_offset] = (uint8_t)(raw >> 8);
    group->data[byte_offset + 1U] = (uint8_t)raw;
    return HAL_OK;
}

void MotorTx_Clear(MotorTxGroup_t *group)
{
    uint8_t i;

    if (group == 0)
    {
        return;
    }

    for (i = 0U; i < CAN_STD_DLC; i++)
    {
        group->data[i] = 0U;
    }
}

HAL_StatusTypeDef MotorTx_Flush(MotorTxGroup_t *groups, uint8_t group_count)
{
    HAL_StatusTypeDef overall_status = HAL_OK;
    uint8_t i;

    if ((groups == 0) || (group_count == 0U))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < group_count; i++)
    {
        if (BSP_CAN_TxData(groups[i].hcan,
                           groups[i].std_id,
                           groups[i].data) != HAL_OK)
        {
            groups[i].flush_error_count++;
            overall_status = HAL_ERROR;
        }
    }
    return overall_status;
}
