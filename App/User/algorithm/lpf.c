/**
 ******************************************************************************
 * @file    lpf.c
 * @brief   一阶低通滤波器实现（算法层）
 * @note    职责边界：
 *            纯数学运算，无任何外设/HAL 调用。
 *            任何需要平滑信号的模块均可直接使用本文件，无需修改。
 * @version 1.0
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#include "lpf.h"

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  初始化低通滤波器
 */
void LPF_Init(LowPassFilter_t *lpf, float dt, float fc)
{
    lpf->dt = dt;
    lpf->fc = fc;

    float RC      = 1.0f / (2.0f * LPF_PI * fc);
    lpf->a_coef   = dt / (RC + dt);          /* α：本拍输入权重   */
    lpf->b_coef   = 1.0f - lpf->a_coef;      /* 1-α：历史输出权重 */

    lpf->input       = 0.0f;
    lpf->output      = 0.0f;
    lpf->last_output = 0.0f;
}

/**
 * @brief  更新滤波器并返回当前输出
 */
float LPF_Update(LowPassFilter_t *lpf, float input)
{
    lpf->input       = input;
    lpf->output      = lpf->a_coef * lpf->input + lpf->b_coef * lpf->last_output;
    lpf->last_output = lpf->output;  /* 保存本拍输出，供下一拍使用 */
    return lpf->output;
}
