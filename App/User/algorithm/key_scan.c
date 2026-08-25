/**
 ******************************************************************************
 * @file    key_scan.c
 * @brief   软件按键消抖与状态机实现（算法层）
 * @note    职责边界：
 *            纯状态机逻辑，无任何 GPIO / HAL 调用，
 *            输入电平由调用方从 BSP 层读取后传入。
 * @version 1.0
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#include "key_scan.h"

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  按键扫描状态机（Toggle 型）
 */
void Key_Scan(Key_t *key, uint8_t key_level, uint8_t *out_state)
{
    switch (key->state)
    {
        /* ── 空闲：检测按下沿 ── */
        case KEY_STATE_IDLE:
            if (key_level == KEY_LEVEL_PRESSED)
            {
                key->state = KEY_STATE_PRESS_DEBOUNCE;
                key->cnt   = 0U;
            }
            break;

        /* ── 按下消抖：计数到阈值后确认 ── */
        case KEY_STATE_PRESS_DEBOUNCE:
            if (key->cnt++ >= KEY_DEBOUNCE_CNT)
            {
                if (key_level == KEY_LEVEL_RELEASED)
                {
                    /* 抖动，未真正按下，回到空闲 */
                    key->state = KEY_STATE_IDLE;
                }
                else
                {
                    /* 确认按下 */
                    key->state = KEY_STATE_HELD;
                }
            }
            break;

        /* ── 已按下：等待松开沿 ── */
        case KEY_STATE_HELD:
            if (key_level == KEY_LEVEL_RELEASED)
            {
                key->state = KEY_STATE_RELEASE_DEBOUNCE;
                key->cnt   = 0U;
            }
            break;

        /* ── 松开消抖：计数到阈值后触发翻转 ── */
        case KEY_STATE_RELEASE_DEBOUNCE:
            if (key->cnt++ >= KEY_DEBOUNCE_CNT)
            {
                if (key_level == KEY_LEVEL_PRESSED)
                {
                    /* 抖动，未真正松开，回到已按下 */
                    key->state = KEY_STATE_HELD;
                }
                else
                {
                    /* 确认完整按下+松开，翻转输出状态 */
                    key->state  = KEY_STATE_IDLE;
                    *out_state  = (uint8_t)!(*out_state);
                }
            }
            break;

        default:
            /* 非法状态，强制复位 */
            key->state = KEY_STATE_IDLE;
            key->cnt   = 0U;
            break;
    }
}
