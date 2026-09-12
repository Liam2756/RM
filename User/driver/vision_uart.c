#include "vision_uart.h"

#include "crc.h"
#include <string.h>

static uint8_t s_frame[VISION_UART_FRAME_SIZE];
static uint8_t s_frame_size;
static Vision_Target_t s_target;
static uint8_t s_target_pending;
static uint8_t s_target_valid;

Vision_UART_Stats_t vision_uart_stats;

void BSP_UART_Vision_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    Vision_UART_DataReadyCallback(buf, size);
}

void BSP_UART_Vision_ResetFraming(void)
{
    Vision_UART_ResetFraming();
}

void BSP_UART_Vision_RecordError(uint32_t error_code)
{
    Vision_UART_RecordError(error_code);
}

static void Vision_UART_SubmitFrame(const uint8_t *frame)
{
    Vision_Target_t target;
    uint32_t primask;
    uint8_t crc_ok;

    vision_uart_stats.frame_count++;

    if ((frame[0] != VISION_UART_FRAME_HEADER) ||
        ((frame[1] != 0U) && (frame[1] != 1U)))
    {
        return;
    }

    crc_ok = CRC16_Verify(frame, VISION_UART_FRAME_SIZE);
    if (crc_ok == 0U)
    {
        vision_uart_stats.crc_error_count++;
        return;
    }

    vision_uart_stats.crc_ok_count++;

    if (frame[1] == 0U)
    {
        return;
    }

    memcpy(&target.x_mm, &frame[2], sizeof(float));
    memcpy(&target.y_mm, &frame[6], sizeof(float));
    memcpy(&target.z_mm, &frame[10], sizeof(float));

    primask = __get_PRIMASK();
    __disable_irq();
    s_target = target;
    s_target_pending = 1U;
    s_target_valid = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Vision_UART_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    memset(s_frame, 0, sizeof(s_frame));
    s_frame_size = 0U;
    memset(&s_target, 0, sizeof(s_target));
    s_target_pending = 0U;
    s_target_valid = 0U;
    memset((void *)&vision_uart_stats, 0, sizeof(vision_uart_stats));
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Vision_UART_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    uint16_t index;

    if (buf == 0)
    {
        return;
    }

    vision_uart_stats.rx_byte_count += size;
    vision_uart_stats.last_rx_size = size;

    for (index = 0U; index < size; index++)
    {
        if (s_frame_size == 0U)
        {
            if (buf[index] != VISION_UART_FRAME_HEADER)
            {
                continue;
            }
        }

        s_frame[s_frame_size++] = buf[index];
        if (s_frame_size == VISION_UART_FRAME_SIZE)
        {
            Vision_UART_SubmitFrame(s_frame);
            s_frame_size = 0U;
        }
    }

    vision_uart_stats.current_frame_size = s_frame_size;
}

void Vision_UART_ResetFraming(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_frame_size = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Vision_UART_RecordError(uint32_t error_code)
{
    vision_uart_stats.uart_error_count++;
    vision_uart_stats.last_uart_error_code = error_code;
}

uint8_t Vision_UART_GetNewTarget(Vision_Target_t *target)
{
    uint32_t primask;

    if (target == 0)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_target_pending == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *target = s_target;
    s_target_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

uint8_t Vision_UART_GetLatestTarget(Vision_Target_t *target)
{
    uint32_t primask;

    if (target == 0)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_target_valid == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *target = s_target;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}
