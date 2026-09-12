/**
 ******************************************************************************
 * @file    vision_test.c
 * @brief   视觉链路自注入测试模块实现（临时件，验证完删除）
 * @note    ========================= VISION LINK TEST (deletable) =========================
 *          删除步骤见 vision_test.h 头部。本文件只依赖公开接口，可整体删除。
 *
 *          扫描相位 s_phase_ms 在 [0, SWEEP_MS] 间往返推进：
 *            frac = phase / SWEEP_MS ∈ [0,1]，线性插值 Y/Z = START + frac*(END-START)。
 *          折返模式下 phase 到达 SWEEP_MS 后反向递减，形成三角波往复。
 * @version 2.0
 * @date    2026-9-8
 ******************************************************************************
 */

#include "vision_test.h"

#if VISION_TEST_INJECT_ENABLE

#include "vision_uart.h"   /* 复用真实解析入口 Vision_UART_DataReadyCallback */
#include "crc.h"           /* CRC16_Append：与设备端同一套 CRC-16，现算不硬编码 */
#include <string.h>
#include <stdint.h>

/* 帧格式（与 vision_uart 一致）：[0]=0xA5 头，[1]=0x01 有目标标志，
 * [2..5]=x float 小端，[6..9]=y，[10..13]=z，[14..15]=CRC16 小端（覆盖前 14 字节）。*/
#define VT_FRAME_SIZE     16U
#define VT_FRAME_HEADER   0xA5U
#define VT_TARGET_VALID   0x01U

static uint16_t s_phase_ms;     /* 当前扫描相位 [0, SWEEP_MS] */
static int8_t   s_dir;          /* 相位推进方向：+1 前进，-1 折返 */
static uint16_t s_period_ms;    /* 注入分频计数 */

static void VisionTest_BuildFrame(uint8_t *frame, float x, float y, float z)
{
    frame[0] = VT_FRAME_HEADER;
    frame[1] = VT_TARGET_VALID;
    memcpy(&frame[2],  &x, sizeof(float));
    memcpy(&frame[6],  &y, sizeof(float));
    memcpy(&frame[10], &z, sizeof(float));
    /* 末 2 字节 CRC16（小端）现算，保证与设备端 CRC16_Verify 一致 */
    CRC16_Append(frame, VT_FRAME_SIZE);
}

void VisionTest_Init(void)
{
    s_phase_ms  = 0U;
    s_dir       = 1;
    s_period_ms = 0U;
}

void VisionTest_Tick(void)
{
    float frac;
    float y;
    float z;
    uint8_t frame[VT_FRAME_SIZE];

    /* ── 每 1ms 推进扫描相位（线性三角波）── */
    if (s_dir > 0)
    {
        if (s_phase_ms >= VISION_TEST_SWEEP_MS)
        {
#if VISION_TEST_PINGPONG
            s_dir = -1;                       /* 到 END：折返 */
#endif
        }
        else
        {
            s_phase_ms++;
        }
    }
    else /* s_dir < 0：折返递减 */
    {
        if (s_phase_ms == 0U)
        {
            s_dir = 1;                        /* 回到 START：再正向 */
        }
        else
        {
            s_phase_ms--;
        }
    }

    /* ── 注入分频：每满 PERIOD_MS 才喂一帧 ── */
    if (++s_period_ms < VISION_TEST_PERIOD_MS)
    {
        return;
    }
    s_period_ms = 0U;

    /* ── 按当前相位线性插值 Y/Z，X 固定 ── */
    frac = (float)s_phase_ms / (float)VISION_TEST_SWEEP_MS;   /* [0,1] */
    y = VISION_TEST_Y_START_MM + frac * (VISION_TEST_Y_END_MM - VISION_TEST_Y_START_MM);
    z = VISION_TEST_Z_START_MM + frac * (VISION_TEST_Z_END_MM - VISION_TEST_Z_START_MM);

    VisionTest_BuildFrame(frame, VISION_TEST_X_MM, y, z);

    /* 假装 UART6 刚收到完整一帧：走与真实视觉完全相同的解析入口 */
    Vision_UART_DataReadyCallback(frame, VT_FRAME_SIZE);
}

#else  /* VISION_TEST_INJECT_ENABLE == 0：空实现，零 footprint */

void VisionTest_Init(void) {}
void VisionTest_Tick(void) {}

#endif /* VISION_TEST_INJECT_ENABLE */
