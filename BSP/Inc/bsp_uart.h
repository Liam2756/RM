#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>

#define BSP_UART_MAX_PORTS  8U

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_buf;
    uint16_t rx_size;
} BSP_UART_PortConfig_t;

void BSP_UART_Register(const BSP_UART_PortConfig_t *port);
void BSP_UART_StartReceive(UART_HandleTypeDef *huart);
void BSP_UART_StartReceiveAll(void);
void BSP_UART_TxData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

#endif /* BSP_UART_H */
