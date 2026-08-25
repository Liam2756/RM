/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID 控制器头文件（算法层）
 * @note    层级说明：
 *            本文件属于【算法层】，无任何硬件依赖，可跨平台复用。
 *            仅依赖标准库 <stdint.h> 和 <math.h>。
 *
 *          算法特性：
 *            - 位置式 PID（非增量式）
 *            - 积分分离（Anti-Windup by Zone Separation）：
 *                误差绝对值超过 i_gap 时，停止积分累积并清零积分项，
 *                避免大误差阶段的积分饱和导致超调。
 *            - 积分限幅：防止积分项无界增长。
 *            - 输出总限幅：保护执行器不超量程。
 *
 *          err 数组索引约定：
 *            err[0] : 当前帧误差   e(k)
 *            err[1] : 上一帧误差   e(k-1)，用于微分计算
 *            err[2] : 积分累计量   Σe，用于积分计算
 *
 * @version 1.0
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>  /* fabsf */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  PID 控制器实例结构体
 * @note   使用前必须调用 PID_Init() 初始化，禁止直接赋值内部状态量。
 */
typedef struct
{
    /* ── PID 增益参数 ── */
    float kp;       /**< 比例增益                                     */
    float ki;       /**< 积分增益                                     */
    float kd;       /**< 微分增益                                     */

    /* ── 限制参数 ── */
    float i_gap;    /**< 积分分离阈值：|err| > i_gap 时停止积分       */
    float i_max;    /**< 积分输出限幅：|ki * Σe| ≤ i_max             */
    float out_max;  /**< 输出总限幅：|output| ≤ out_max               */

    /* ── 状态量（每帧由 PID_Calculate 更新，外部只读）── */
    float target;   /**< 当前目标值                                   */
    float measure;  /**< 当前测量值                                   */
    float err[3];   /**< [0]=当前误差, [1]=上帧误差, [2]=积分累计量  */
    float last_out; /**< 上一帧输出值                                 */
} PID_t;

/**
 * @brief  PID 增益与限幅配置（传参用）
 * @note   只承载"可调参数"，不含运行状态；由各控制模块以 const 实例
 *         （通常来自模块头文件的参数宏）经 PID_Init 传入。
 *         多个同规格电机可共用同一份 config。
 */
typedef struct
{
    float kp;       /**< 比例增益                                     */
    float ki;       /**< 积分增益                                     */
    float kd;       /**< 微分增益                                     */
    float i_max;    /**< 积分输出限幅（正值）                         */
    float out_max;  /**< 输出总限幅（正值）                           */
    float i_gap;    /**< 积分分离阈值（正值），极大值可关闭积分分离   */
} PID_Config_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  PID 控制器初始化
 * @note   把 cfg 中的增益/限幅拷入实例，并清零所有内部状态量
 *         （err[]、last_out），可用于运行时重置控制器（如模式切换
 *         或电机离线后消除历史积分）。cfg 只读，可被多个实例共用。
 * @param  pid  PID 实例指针
 * @param  cfg  增益与限幅配置（只读）
 * @retval 无
 */
void PID_Init(PID_t *pid, const PID_Config_t *cfg);

/**
 * @brief  PID 单步计算
 * @note   须以固定周期调用，dt 隐含在 kd 参数中（kd = Kd_实际 / dt）。
 *         若采样周期变化，需重新标定 kd。
 * @param  pid      PID 实例指针
 * @param  target   目标设定值
 * @param  measure  当前测量值
 * @retval 限幅后的控制输出
 */
float PID_Calculate(PID_t *pid, float target, float measure);

#endif /* PID_H */
