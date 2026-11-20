#include "heat_estimate.h"

#define MILLIHEAT_PER_UNIT UINT64_C(1000)

HeatEstimate heat_estimate_begin(uint16_t heat, uint32_t count, uint32_t at_ms,
                                 uint16_t heat_per_event, uint16_t cooling_per_s)
{
    return (HeatEstimate){
        .milliheat = heat * MILLIHEAT_PER_UNIT,
        .at_ms = at_ms,
        .confirmed_count = count,
        .heat_per_event = heat_per_event,
        .cooling_per_s = cooling_per_s,
        .valid = heat_per_event != 0
    };
}

bool heat_estimate_read(const HeatEstimate *estimate, uint32_t at_ms,
                        uint64_t *milliheat)
{
    uint32_t elapsed_ms = at_ms - estimate->at_ms;
    if (!estimate->valid || elapsed_ms > INT32_MAX) {
        return false;
    }
    /* 每秒一个热量单位等价于每毫秒一个千分单位。 */
    uint64_t cooled = (uint64_t)elapsed_ms * estimate->cooling_per_s;
    *milliheat = cooled < estimate->milliheat ? estimate->milliheat - cooled : 0;
    return true;
}

bool heat_estimate_observe(HeatEstimate *estimate, uint32_t count,
                           uint32_t at_ms, uint16_t cooling_per_s_after)
{
    uint32_t new_events = count - estimate->confirmed_count;
    uint64_t heat;
    /* 跳号无法还原各事件时刻；特别是在中间冷却到零时，不能统一补记。 */
    if (new_events > 1 || !heat_estimate_read(estimate, at_ms, &heat)) {
        estimate->valid = false;
        return false;
    }
    uint64_t added = new_events * MILLIHEAT_PER_UNIT * estimate->heat_per_event;
    if (heat > UINT64_MAX - added) {
        estimate->valid = false;
        return false;
    }
    estimate->milliheat = heat + added;
    estimate->confirmed_count = count;
    estimate->at_ms = at_ms;
    estimate->cooling_per_s = cooling_per_s_after;
    return true;
}
