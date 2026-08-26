#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdint.h>

#define VISION_UART_FRAME_HEADER  0xA5U
#define VISION_UART_FRAME_SIZE    16U

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} Vision_Target_t;

void Vision_UART_Init(void);
void Vision_UART_DataReadyCallback(uint8_t *buf, uint16_t size);
uint8_t Vision_UART_GetNewTarget(Vision_Target_t *target);

#endif /* VISION_UART_H */
