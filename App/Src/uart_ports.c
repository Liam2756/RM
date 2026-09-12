#include "uart_ports.h"

#include "bsp_uart.h"
#include "usart.h"

uint8_t g_uart1_rx_buf[UART1_RX_BUF_SIZE];
uint8_t g_uart3_rx_buf[UART3_RX_BUF_SIZE];
uint8_t g_uart6_rx_buf[UART6_RX_BUF_SIZE];

void UART_Ports_Init(void)
{
    BSP_UART_Register(&(BSP_UART_PortConfig_t){ &huart1, g_uart1_rx_buf, UART1_RX_BUF_SIZE });
    BSP_UART_Register(&(BSP_UART_PortConfig_t){ &huart3, g_uart3_rx_buf, UART3_RX_BUF_SIZE });
    BSP_UART_Register(&(BSP_UART_PortConfig_t){ &huart6, g_uart6_rx_buf, UART6_RX_BUF_SIZE });

    BSP_UART_StartReceiveAll();
}
