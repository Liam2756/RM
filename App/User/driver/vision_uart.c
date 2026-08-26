#include "vision_uart.h"

#include "crc.h"
#include <string.h>

static uint8_t s_frame[VISION_UART_FRAME_SIZE];
static uint8_t s_frame_size;
static Vision_Target_t s_target;
static uint8_t s_target_pending;

static void Vision_UART_SubmitFrame(const uint8_t *frame)
{
    Vision_Target_t target;
    uint32_t primask;

    if ((frame[0] != VISION_UART_FRAME_HEADER) ||
        ((frame[1] != 0U) && (frame[1] != 1U)) ||
        (CRC16_Verify(frame, VISION_UART_FRAME_SIZE) == 0U) ||
        (frame[1] == 0U))
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
