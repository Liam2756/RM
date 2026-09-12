/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   CAN 总线底层驱动实现（BSP 层）
 * @note    发送只封装 HAL；接收只校验帧格式并向设备驱动层分发。
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#include <bsp_can.h>

BSP_CAN_Diagnostics_t g_bsp_can1_diagnostics;

static BSP_CAN_Diagnostics_t *BSP_CAN_GetDiagnostics(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        return &g_bsp_can1_diagnostics;
    }
    return 0;
}

__weak void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                                     uint32_t std_id,
                                     const uint8_t data[CAN_STD_DLC])
{
    (void)hcan;
    (void)std_id;
    (void)data;
}

HAL_StatusTypeDef BSP_CAN_TxData(CAN_HandleTypeDef *hcan,
                                 uint32_t std_id,
                                 const uint8_t data[CAN_STD_DLC])
{
    CAN_TxHeaderTypeDef tx_header;
    BSP_CAN_Diagnostics_t *diagnostics;
    HAL_StatusTypeDef status;
    uint32_t tx_mailbox;

    if ((hcan == 0) || (data == 0) || (std_id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = CAN_STD_DLC;
    tx_header.TransmitGlobalTime = DISABLE;

    status = HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
    if (status != HAL_OK)
    {
        diagnostics = BSP_CAN_GetDiagnostics(hcan);
        if (diagnostics != 0)
        {
            diagnostics->tx_error_count++;
        }
    }
    return status;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    BSP_CAN_Diagnostics_t *diagnostics;
    uint8_t rx_data[CAN_STD_DLC];

    diagnostics = BSP_CAN_GetDiagnostics(hcan);
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        if (diagnostics != 0)
        {
            diagnostics->rx_error_count++;
        }
        return;
    }

    if ((rx_header.IDE != CAN_ID_STD) ||
        (rx_header.RTR != CAN_RTR_DATA) ||
        (rx_header.DLC != CAN_STD_DLC))
    {
        if (diagnostics != 0)
        {
            diagnostics->invalid_rx_count++;
        }
        return;
    }

    BSP_CAN_RxFrameCallback(hcan, rx_header.StdId, rx_data);
}
