#include "heat_monitor.h"
#include "heat_rules_2027.h"
#include <assert.h>

int main(void)
{
    HeatReading heat = {99, 100, true};
    HeatReading limit = {100, 90, true};
    assert(heat_monitor_compare(heat, limit, 110, 10, 20) == HEAT_BELOW_LIMIT);
    assert(heat_monitor_compare(heat, limit, 110, 9, 20) == HEAT_DATA_STALE);
    assert(heat_monitor_compare(heat, limit, 110, 10, 19) == HEAT_DATA_STALE);
    heat.value = 100;
    assert(heat_monitor_compare(heat, limit, 110, 10, 20) == HEAT_AT_LIMIT);
    heat.value = 101;
    assert(heat_monitor_compare(heat, limit, 110, 10, 20) == HEAT_ABOVE_LIMIT);
    heat.received = false;
    assert(heat_monitor_compare(heat, limit, 110, 0, 0) == HEAT_DATA_MISSING);
    heat.received = true;
    limit.received = false;
    assert(heat_monitor_compare(heat, limit, 110, 0, 0) == HEAT_DATA_MISSING);
    limit.received = true;
    heat.received_ms = UINT32_MAX - 2;
    limit.received_ms = 1;
    assert(heat_monitor_compare(heat, limit, 2, 5, 1) == HEAT_ABOVE_LIMIT);
    heat = (HeatReading){RMUL2027_INFANTRY_BURST_HEAT_LIMIT, 0, true};
    limit = heat;
    assert(heat_monitor_compare(heat, limit, 0, 0, 0) == HEAT_AT_LIMIT);
    heat.value += RMUL2027_17MM_HEAT_PER_EVENT;
    assert(heat_monitor_compare(heat, limit, 0, 0, 0) == HEAT_ABOVE_LIMIT);
    heat.value = 80;
    limit.value = RMUL2027_INFANTRY_COOL_HEAT_LIMIT;
    assert(heat_monitor_compare(heat, limit, 0, 0, 0) == HEAT_BELOW_LIMIT);
    heat.value += RMUL2027_17MM_HEAT_PER_EVENT;
    assert(heat_monitor_compare(heat, limit, 0, 0, 0) == HEAT_ABOVE_LIMIT);
    return 0;
}
