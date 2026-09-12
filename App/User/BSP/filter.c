/**
 ******************************************************************************
 * @file    filter.c
 * @brief   双 CAN 过滤器和接收中断初始化实现（BSP 层）
 * @note    CAN1 使用 FilterBank 0 接收标准帧。
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#include <filter.h>

HAL_StatusTypeDef can_filter_init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    filter.FilterBank = 0U;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
