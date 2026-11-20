#include "heat_estimate.h"
#include "heat_monitor.h"
#include <assert.h>

static void expect_heat(const HeatEstimate *estimate, uint32_t at_ms, uint64_t expected)
{
    uint64_t value;
    assert(heat_estimate_read(estimate, at_ms, &value));
    assert(value == expected);
}

static void confirmed_events_and_late_report(void)
{
    /* 人工测试参数；第 40 次通过已包含在起点内。 */
    HeatEstimate estimate = heat_estimate_begin(3, 40, 100, 4, 2);
    assert(heat_estimate_observe(&estimate, 40, 100, 2));
    expect_heat(&estimate, 100, 3000);
    assert(heat_estimate_observe(&estimate, 41, 200, 2));
    assert(heat_estimate_observe(&estimate, 41, 200, 2));
    expect_heat(&estimate, 200, 6800);
    assert(heat_estimate_observe(&estimate, 42, 200, 2));
    expect_heat(&estimate, 200, 10800);

    HeatReading report = {3, 210, true};
    HeatReading limit = {12, 210, true};
    assert(heat_monitor_compare(report, limit, 210, 0, 0) == HEAT_BELOW_LIMIT);
    expect_heat(&estimate, 210, 10780);
    /* 显示刷新不推进事件处理时钟，接下来仍可处理时间戳 205 的事件。 */
    assert(heat_estimate_observe(&estimate, 43, 205, 2));
    expect_heat(&estimate, 210, 14780);
}

static void cooling_and_wrap(void)
{
    HeatEstimate estimate = heat_estimate_begin(1, UINT32_MAX, UINT32_MAX - 2, 4, 3);
    assert(heat_estimate_observe(&estimate, 0, 1, 3));
    expect_heat(&estimate, 1, 4988);
    assert(heat_estimate_observe(&estimate, 0, 101, 5));
    expect_heat(&estimate, 201, 4188);
    expect_heat(&estimate, 5000, 0);
    assert(heat_estimate_observe(&estimate, 1, 5000, 0));
    expect_heat(&estimate, 5500, 4000);
}

static void invalid_history_and_new_baseline(void)
{
    HeatEstimate estimate = {0};
    uint64_t untouched = 123;
    assert(!heat_estimate_read(&estimate, 0, &untouched));
    assert(untouched == 123);
    estimate = heat_estimate_begin(0, 12, 50, 4, 2);
    assert(!heat_estimate_observe(&estimate, 14, 100, 2));
    assert(!heat_estimate_read(&estimate, 100, &untouched));
    assert(!heat_estimate_observe(&estimate, 13, 110, 2));
    estimate = heat_estimate_begin(0, 0, 500, 4, 2);
    assert(heat_estimate_observe(&estimate, 1, 501, 2));
    assert(!heat_estimate_observe(&estimate, 2, 500, 2));
    assert(!heat_estimate_read(&estimate, 505, &untouched));
    estimate = heat_estimate_begin(2, 0, 0, 4, UINT16_MAX);
    expect_heat(&estimate, INT32_MAX, 0);
    assert(!heat_estimate_read(&estimate, (uint32_t)INT32_MAX + 1, &untouched));
    assert(untouched == 123);
}

int main(void)
{
    confirmed_events_and_late_report();
    cooling_and_wrap();
    invalid_history_and_new_baseline();
    return 0;
}
