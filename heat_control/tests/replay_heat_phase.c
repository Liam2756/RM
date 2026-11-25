/* 主机离线回放。参考时钟使用绝对毫秒刻度，与估算器的余数累积实现独立。 */
#include "heat_estimate.h"
#include "heat_rules_2027.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>

enum { DURATION_MS = 300 };

typedef struct {
    const char *name;
    unsigned initial_heat;
    unsigned limit;
    unsigned base_rate;
    unsigned boosted_from_ms;  /* 0 表示本段无增益 */
    unsigned event_ms[3];      /* 0 表示无事件，其余时间各新增一个计数 */
} Trace;

typedef struct {
    int64_t over;
    int64_t under;
    bool false_alarm;
    bool missed_alarm;
} Differences;

static void compare_phase(const Trace *trace, unsigned first_tick, Differences *diff)
{
    const unsigned period = RMUL2027_HEAT_COOLING_PERIOD_MS;
    HeatEstimate local = heat_estimate_begin(trace->initial_heat,
                                             RMUL2027_17MM_HEAT_PER_EVENT, period);
    /* 参考值以十分之一热量表示，能精确表示本组规则参数的单次冷却量。 */
    int reference = (int)trace->initial_heat * 10;
    unsigned next_tick = first_tick;
    for (unsigned ms = 1; ms <= DURATION_MS; ++ms) {
        unsigned rate = trace->base_rate;
        if (trace->boosted_from_ms && ms >= trace->boosted_from_ms) {
            rate = rate * 3 / 2;
        }
        unsigned events = 0;
        for (unsigned i = 0; i < 3; ++i) {
            if (trace->event_ms[i] == ms) ++events;
        }
        /* 同毫秒都先结算冷却再计数，确保两侧只改变时钟相位。 */
        if (ms == next_tick) {
            reference -= (int)rate;
            if (reference < 0) reference = 0;
            next_tick += period;
        }
        reference += (int)events * RMUL2027_17MM_HEAT_PER_EVENT * 10;

        bool updated = heat_estimate_step(&local, events, 1, rate);
        uint64_t value = 0;
        bool available = heat_estimate_read(&local, &value);
        assert(updated && available);
        int64_t error = (int64_t)value - (int64_t)reference * 100;
        if (error > diff->over) diff->over = error;
        if (-error > diff->under) diff->under = -error;
        bool local_above = value > (uint64_t)trace->limit * 1000;
        bool reference_above = reference > (int)trace->limit * 10;
        diff->false_alarm |= local_above && !reference_above;
        diff->missed_alarm |= reference_above && !local_above;
    }
}

int main(void)
{
    const unsigned burst_limit = RMUL2027_INFANTRY_BURST_HEAT_LIMIT;
    const unsigned burst_rate = RMUL2027_INFANTRY_BURST_COOLING_PER_S;
    const unsigned cool_limit = RMUL2027_INFANTRY_COOL_HEAT_LIMIT;
    const unsigned cool_rate = RMUL2027_INFANTRY_COOL_COOLING_PER_S;
    const Trace traces[] = {
        {"idle_at_limit", cool_limit, cool_limit, cool_rate, 0, {0}},
        {"burst_boundary", burst_limit - 9, burst_limit, burst_rate, 0, {99, 100, 101}},
        {"cool_boundary", cool_limit - 9, cool_limit, cool_rate, 0, {99, 100, 101}},
        {"restart_from_zero", 0, cool_limit, cool_rate, 0, {99}},
        {"boost_transition", cool_limit + 3, cool_limit, cool_rate, 75, {0}},
    };
    for (unsigned i = 0; i < sizeof traces / sizeof traces[0]; ++i) {
        Differences aligned = {0};
        compare_phase(&traces[i], RMUL2027_HEAT_COOLING_PERIOD_MS, &aligned);
        assert(!aligned.over && !aligned.under && !aligned.false_alarm && !aligned.missed_alarm);
        Differences shifted = {0};
        for (unsigned first = 1; first < RMUL2027_HEAT_COOLING_PERIOD_MS; ++first) {
            compare_phase(&traces[i], first, &shifted);
        }
        printf("%s,%" PRId64 ",%" PRId64 ",%d,%d\n", traces[i].name,
               shifted.over, shifted.under, shifted.false_alarm, shifted.missed_alarm);
    }
    return 0;
}
