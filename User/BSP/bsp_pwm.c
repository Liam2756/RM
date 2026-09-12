/**
 ******************************************************************************
 * @file    bsp_pwm.c
 * @brief   PWM 底层驱动实现（BSP 层）
 * @note    职责边界：
 *            封装 HAL TIM PWM 原语，屏蔽定时器寄存器细节。
 *            【禁止】在本文件中出现任何业务概念（如"摩擦轮"、"发射"）。
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#include "bsp_pwm.h"

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  启动单个 PWM 通道输出
 */
void BSP_PWM_Start(const BSP_PWM_Channel_t *ch)
{
    HAL_TIM_PWM_Start(ch->htim, ch->channel);
}

/**
 * @brief  设置单个 PWM 通道的比较值
 */
void BSP_PWM_SetPulse(const BSP_PWM_Channel_t *ch, uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(ch->htim, ch->channel, pulse);
}

/**
 * @brief  启动高级定时器主输出使能（MOE）
 */
void BSP_PWM_EnableMOE(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_MOE_ENABLE(htim);
}
