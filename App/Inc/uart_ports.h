#ifndef UART_PORTS_H
#define UART_PORTS_H

#include <stdint.h>

#define UART1_RX_BUF_SIZE  13U
#define UART3_RX_BUF_SIZE  18U
#define UART6_RX_BUF_SIZE  26U

extern uint8_t g_uart1_rx_buf[UART1_RX_BUF_SIZE];
extern uint8_t g_uart3_rx_buf[UART3_RX_BUF_SIZE];
extern uint8_t g_uart6_rx_buf[UART6_RX_BUF_SIZE];

void UART_Ports_Init(void);

#endif /* UART_PORTS_H */
