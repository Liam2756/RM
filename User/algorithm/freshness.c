/**
 ******************************************************************************
 * @file    freshness.c
 * @brief   通用新鲜度/在线看门狗实现（算法层）
 * @note    纯算法、无硬件依赖；时间戳由调用方传入。设计说明见 freshness.h。
 * @version 1.0
 * @date    2026-8-16
 ******************************************************************************
 */

#include <freshness.h>

void Freshness_Init(Freshness_t *f, const Freshness_Config_t *cfg)
{
    if ((f == 0) || (cfg == 0))
    {
        return;
    }

    f->timeout_ms  = cfg->timeout_ms;
    f->online_need = (cfg->online_need == 0U) ? 1U : cfg->online_need;

    f->last_good_tick = 0U;
    f->online         = 0U;
    f->online_cnt     = 0U;
}

void Freshness_FeedGood(Freshness_t *f, uint32_t now)
{
    if (f == 0)
    {
        return;
    }

    f->last_good_tick = now;

    /* 已在线则无需再累计去抖；离线态下连续合法帧累计到阈值即上线。 */
    if (f->online == 0U)
    {
        if (f->online_cnt < f->online_need)
        {
            f->online_cnt++;
        }
        if (f->online_cnt >= f->online_need)
        {
            f->online     = 1U;
            f->online_cnt = 0U;
        }
    }
}

uint8_t Freshness_IsOnline(Freshness_t *f, uint32_t now)
{
    if (f == 0)
    {
        return 0U;
    }

    /* 超时：判离线并清去抖计数。离线态下的超时同样清零，从而打断「连续帧」
       计数——保证 online_need 表示的是真正连续到达的合法帧，而非零散累计。 */
    if ((now - f->last_good_tick) > f->timeout_ms)
    {
        f->online     = 0U;
        f->online_cnt = 0U;
    }

    return f->online;
}
