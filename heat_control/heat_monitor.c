#include "heat_monitor.h"

static bool expired(HeatReading reading, uint32_t now_ms, uint32_t max_age_ms)
{
    return now_ms - reading.received_ms > max_age_ms;
}

static HeatStatus compare_values(uint64_t heat, uint64_t limit)
{
    if (heat == limit) return HEAT_AT_LIMIT;
    return heat < limit ? HEAT_BELOW_LIMIT : HEAT_ABOVE_LIMIT;
}

HeatStatus heat_monitor_compare(HeatReading heat, HeatReading limit,
                                uint32_t now_ms, uint32_t heat_max_age_ms,
                                uint32_t limit_max_age_ms)
{
    if (!heat.received || !limit.received) {
        return HEAT_DATA_MISSING;
    }
    /* 同一毫秒时钟上的无符号差值允许跨越一次计数器零点。 */
    if (expired(heat, now_ms, heat_max_age_ms) ||
        expired(limit, now_ms, limit_max_age_ms)) {
        return HEAT_DATA_STALE;
    }
    return compare_values(heat.value, limit.value);
}

HeatReport heat_monitor_report(const uint64_t *estimate_milliheat,
                               HeatReading referee, HeatReading limit,
                               uint32_t now_ms, uint32_t referee_max_age_ms,
                               uint32_t limit_max_age_ms)
{
    HeatReport report;
    report.referee_status = heat_monitor_compare(referee, limit, now_ms,
                                                 referee_max_age_ms, limit_max_age_ms);
    if (!estimate_milliheat || !limit.received) {
        report.estimated_status = HEAT_DATA_MISSING;
    } else if (expired(limit, now_ms, limit_max_age_ms)) {
        report.estimated_status = HEAT_DATA_STALE;
    } else {
        /* 上限换为同一单位再比较，保留估值的小数部分。 */
        report.estimated_status = compare_values(*estimate_milliheat,
                                                 (uint64_t)limit.value * 1000U);
    }
    return report;
}
