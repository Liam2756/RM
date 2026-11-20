#include "heat_monitor.h"

HeatStatus heat_monitor_compare(HeatReading heat, HeatReading limit,
                                uint32_t now_ms, uint32_t heat_max_age_ms,
                                uint32_t limit_max_age_ms)
{
    if (!heat.received || !limit.received) {
        return HEAT_DATA_MISSING;
    }
    /* 同一毫秒时钟上的无符号差值允许跨越一次计数器零点。 */
    if (now_ms - heat.received_ms > heat_max_age_ms ||
        now_ms - limit.received_ms > limit_max_age_ms) {
        return HEAT_DATA_STALE;
    }
    if (heat.value == limit.value) {
        return HEAT_AT_LIMIT;
    }
    return heat.value < limit.value ? HEAT_BELOW_LIMIT : HEAT_ABOVE_LIMIT;
}
