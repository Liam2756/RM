/**
 ******************************************************************************
 * @file    freshness.h
 * @brief   通用新鲜度/在线看门狗（算法层）
 * @note    层级说明：
 *            本文件属于【算法层】，无任何硬件依赖，可跨平台复用。
 *            仅依赖标准库 <stdint.h>；禁止 include 任何 BSP / HAL 头文件。
 *            时间戳 now（ms 计的单调计数）由调用方传入，本模块不读时钟。
 *
 *          用途：把「最近一次内容合法更新 + 超时判离线 + 上线去抖」统一成
 *          一份逻辑，供 DT7 / IMU / 裁判 UART 等各数据源以各自参数复用。
 *          各源的超时/去抖参数放在各自驱动头文件里（宏），经 Freshness_Init
 *          以 const 配置传入，用法与 PID_Config_t + PID_Init 一致。
 *
 *          两条安全约定由调用方保证：
 *            ① 只在「内容合法」的更新到达时调用 Freshness_FeedGood 刷新计时；
 *            ② 判离线后应把被看护的数据复位为安全中立态。
 *
 *          典型接线（1kHz 控制环 + ~14ms 一帧的 DT7）：
 *            - 收到合法帧的回调里：Freshness_FeedGood(&f, now);
 *            - 控制环每周期：if (!Freshness_IsOnline(&f, now)) { 复位为安全态 }
 * @version 1.0
 * @date    2026-8-16
 ******************************************************************************
 */

#ifndef FRESHNESS_H
#define FRESHNESS_H

#include <stdint.h>

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  新鲜度看门狗配置（传参用）
 * @note   只承载「可调参数」，不含运行状态；由各数据源以 const 实例
 *         （通常来自驱动头文件的参数宏）经 Freshness_Init 传入。
 *         多个同类源可共用同一份 config。
 */
typedef struct
{
    uint32_t timeout_ms;  /**< 离线判定超时（ms）：距上次 FeedGood 超此值判离线 */
    uint8_t  online_need; /**< 上线去抖：需连续多少合法帧才判在线（0 视作 1） */
} Freshness_Config_t;

/**
 * @brief  新鲜度看门狗实例
 * @note   使用前必须调用 Freshness_Init() 初始化，禁止直接赋值内部状态量。
 */
typedef struct
{
    /* ── 配置（Init 拷入，运行期只读）── */
    uint32_t timeout_ms;     /**< 离线超时（ms）                            */
    uint8_t  online_need;    /**< 上线所需连续合法帧数（≥1）                */

    /* ── 状态量（由 FeedGood / IsOnline 更新，外部只读）── */
    uint32_t last_good_tick; /**< 上次收到合法更新时的 now                  */
    uint8_t  online;         /**< 当前在线标志：1 在线，0 离线              */
    uint8_t  online_cnt;     /**< 离线态下已累计的连续合法帧数              */
} Freshness_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  初始化新鲜度看门狗：拷入配置、状态清零、初始判为离线
 * @param  f    实例指针
 * @param  cfg  配置（只读，可被多个实例共用）；online_need 为 0 时按 1 处理
 * @retval 无
 * @note   初始 online=0，需连续 online_need 帧合法更新后才上线，符合
 *         「上电默认离线、收到有效数据才放行」的安全约定。
 */
void Freshness_Init(Freshness_t *f, const Freshness_Config_t *cfg);

/**
 * @brief  喂入一次「内容合法」的更新，刷新计时并推进上线去抖
 * @param  f    实例指针
 * @param  now  当前时间戳（ms 单调计数），由调用方传入
 * @retval 无
 * @note   仅应在内容校验通过后调用；坏帧不喂，以免用坏数据维持在线。
 *         online_need==1 时首帧即上线。
 */
void Freshness_FeedGood(Freshness_t *f, uint32_t now);

/**
 * @brief  查询是否在线；超时则判离线并清空上线去抖计数
 * @param  f    实例指针
 * @param  now  当前时间戳（ms 单调计数），由调用方传入
 * @retval 1 在线，0 离线
 * @note   须周期调用（如每个控制周期）：距上次 FeedGood 超 timeout_ms 即判
 *         离线；离线态下的超时会打断「连续帧」计数，保证 online_need 语义为
 *         真正的连续帧。本函数不复位被看护数据，复位动作由调用方按业务执行。
 */
uint8_t Freshness_IsOnline(Freshness_t *f, uint32_t now);

#endif /* FRESHNESS_H */
