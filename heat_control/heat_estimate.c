#include "heat_estimate.h"

#define MILLIHEAT_PER_UNIT UINT64_C(1000)

HeatEstimate heat_estimate_begin(uint16_t heat, uint16_t heat_per_event,
                                 uint16_t cooling_period_ms)
{
    return (HeatEstimate){
        .milliheat = heat * MILLIHEAT_PER_UNIT,
        .heat_per_event = heat_per_event,
        .cooling_period_ms = cooling_period_ms,
        .cooling_elapsed_ms = 0,
        .valid = heat_per_event != 0 && cooling_period_ms != 0
    };
}

bool heat_estimate_read(const HeatEstimate *estimate, uint64_t *milliheat)
{
    if (!estimate->valid) {
        return false;
    }
    *milliheat = estimate->milliheat;
    return true;
}

bool heat_estimate_step(HeatEstimate *estimate, uint32_t new_events,
                        uint32_t elapsed_ms, uint16_t cooling_per_s)
{
    if (!estimate->valid) {
        return false;
    }
    uint64_t elapsed = (uint64_t)estimate->cooling_elapsed_ms + elapsed_ms;
    uint64_t cooling_ticks = elapsed / estimate->cooling_period_ms;
    /* 热量/秒 × 结算周期毫秒数，得到每次结算的千分热量。 */
    uint64_t cooled = cooling_ticks * estimate->cooling_period_ms * cooling_per_s;
    uint64_t heat = cooled < estimate->milliheat ? estimate->milliheat - cooled : 0;
    uint64_t added = new_events * MILLIHEAT_PER_UNIT * estimate->heat_per_event;
    if (heat > UINT64_MAX - added) {
        estimate->valid = false;
        return false;
    }
    estimate->milliheat = heat + added;
    estimate->cooling_elapsed_ms = (uint16_t)(elapsed % estimate->cooling_period_ms);
    return true;
}
