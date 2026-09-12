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

typedef struct
{
    volatile uint32_t rx_byte_count;      /**< 累计接收字节数 */
    volatile uint32_t frame_count;        /**< 累计收到完整帧数（含CRC失败） */
    volatile uint32_t crc_ok_count;       /**< CRC校验通过帧数 */
    volatile uint32_t crc_error_count;    /**< CRC校验失败帧数 */
    volatile uint32_t uart_error_count;   /**< UART硬件错误次数 */
    volatile uint32_t last_uart_error_code; /**< 最近一次错误码(HAL ErrorCode) */
    volatile uint16_t last_rx_size;       /**< 最近一次接收字节数 */
    volatile uint8_t current_frame_size;  /**< 当前分帧缓冲已收字节数 */
} Vision_UART_Stats_t;

extern Vision_UART_Stats_t vision_uart_stats;

void Vision_UART_Init(void);
void Vision_UART_DataReadyCallback(uint8_t *buf, uint16_t size);
void Vision_UART_ResetFraming(void);
void Vision_UART_RecordError(uint32_t error_code);
uint8_t Vision_UART_GetNewTarget(Vision_Target_t *target);
uint8_t Vision_UART_GetLatestTarget(Vision_Target_t *target);

#endif /* VISION_UART_H */
