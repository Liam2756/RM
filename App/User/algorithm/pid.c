/**
 ******************************************************************************
 * @file    pid.c
 * @brief   PID 控制器实现（算法层）
 * @note    职责边界：
 *            纯数学运算，无任何外设 / HAL 调用，可跨平台直接移植。
 * @version 1.0
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#include "pid.h"

/* ========================================================================== */
/*                              内部辅助宏                                      */
/* ========================================================================== */

/** 将值限幅到 [-limit, +limit] */
#define CLAMP_SYM(val, limit)  \
    ((val) >  (limit) ? (limit) : ((val) < -(limit) ? -(limit) : (val)))

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  PID 控制器初始化
 */
void PID_Init(PID_t *pid, const PID_Config_t *cfg)
{
    if ((pid == 0) || (cfg == 0))
        return;

    /* 写入增益与限制参数 */
    pid->kp      = cfg->kp;
    pid->ki      = cfg->ki;
    pid->kd      = cfg->kd;
    pid->i_max   = cfg->i_max;
    pid->out_max = cfg->out_max;
    pid->i_gap   = cfg->i_gap;

    /* 清零所有状态量 */
    pid->target   = 0.0f;
    pid->measure  = 0.0f;
    pid->last_out = 0.0f;
    for (int i = 0; i < 3; i++)
        pid->err[i] = 0.0f;
}

/**
 * @brief  PID 单步计算
 */
float PID_Calculate(PID_t *pid, float target, float measure)
{
    pid->target  = target;
    pid->measure = measure;

    /* ── Step 1：计算当前误差 e(k) ── */
    pid->err[0] = target - measure;

    /* ── Step 2：比例项 P ── */
    float pout = pid->kp * pid->err[0];

    /* ── Step 3：积分项 I（含积分分离与限幅）── */
    float iout;
    if (fabsf(pid->err[0]) < pid->i_gap)
    {
        /* 误差在分离阈值内：正常累积积分 */
        pid->err[2] += pid->err[0];

        /* 积分限幅（Anti-Windup）：限制积分累计量，等效限制 iout 幅度 */
        if (pid->ki != 0.0f)
        {
            pid->err[2] = CLAMP_SYM(pid->err[2], pid->i_max / pid->ki);
        }
        iout = pid->ki * pid->err[2];
    }
    else
    {
        /* 误差超出阈值：清零积分，防止大误差阶段积分饱和导致超调 */
        pid->err[2] = 0.0f;
        iout = 0.0f;
    }

    /* ── Step 4：微分项 D（基于误差差分，隐含 dt 于 kd 参数）── */
    float dout = pid->kd * (pid->err[0] - pid->err[1]);

    /* ── Step 5：合并输出并总限幅 ── */
    float out = CLAMP_SYM(pout + iout + dout, pid->out_max);

    /* ── Step 6：更新状态，为下一帧准备 ── */
    pid->err[1]  = pid->err[0];  /* e(k-1) ← e(k) */
    pid->last_out = out;

    return out;
}
