#include "bsp_uart.h"

static BSP_UART_PortConfig_t g_uart_ports[BSP_UART_MAX_PORTS];
static uint8_t g_uart_port_count = 0U;

static BSP_UART_PortConfig_t *BSP_UART_FindPort(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0U; i < g_uart_port_count; i++)
    {
        if (g_uart_ports[i].huart == huart)
            return &g_uart_ports[i];
    }
    return NULL;
}

void BSP_UART_Register(const BSP_UART_PortConfig_t *port)
{
    if ((port == NULL) || (g_uart_port_count >= BSP_UART_MAX_PORTS))
        return;

    g_uart_ports[g_uart_port_count] = *port;
    g_uart_port_count++;
}

void BSP_UART_StartReceive(UART_HandleTypeDef *huart)
{
    BSP_UART_PortConfig_t *port = BSP_UART_FindPort(huart);
    if (port != NULL)
        HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->rx_buf, port->rx_size);
}

void BSP_UART_StartReceiveAll(void)
{
    for (uint8_t i = 0U; i < g_uart_port_count; i++)
        BSP_UART_StartReceive(g_uart_ports[i].huart);
}

void BSP_UART_TxData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(huart, data, size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;
    BSP_UART_StartReceive(huart);
}
