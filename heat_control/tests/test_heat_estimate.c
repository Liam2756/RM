#include "heat_estimate.h"
#include "heat_rules_2027.h"
#include <assert.h>

static void expect_heat(const HeatEstimate *estimate, uint64_t expected)
{
    uint64_t value;
    assert(heat_estimate_read(estimate, &value));
    assert(value == expected);
}

static void burst_cooling_boundary(void)
{
    HeatEstimate estimate = heat_estimate_begin(
        30, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    assert(heat_estimate_step(&estimate, 0, 99, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    expect_heat(&estimate, 30000);
    assert(heat_estimate_step(&estimate, 0, 1, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    expect_heat(&estimate, 28600);
    assert(heat_estimate_step(&estimate, 0, 250, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    expect_heat(&estimate, 25800);
    assert(estimate.cooling_elapsed_ms == 50);
    assert(heat_estimate_step(&estimate, 0, 50, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    expect_heat(&estimate, 24400);
}

static void cool_profile_and_batch_count(void)
{
    HeatEstimate estimate = heat_estimate_begin(
        0, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    assert(heat_estimate_step(&estimate, 3, 1, RMUL2027_INFANTRY_COOL_COOLING_PER_S));
    expect_heat(&estimate, 30000);
    assert(heat_estimate_step(&estimate, 0, 98, RMUL2027_INFANTRY_COOL_COOLING_PER_S));
    expect_heat(&estimate, 30000);
    assert(heat_estimate_step(&estimate, 0, 1, RMUL2027_INFANTRY_COOL_COOLING_PER_S));
    expect_heat(&estimate, 27600);
    expect_heat(&estimate, 27600);
}

static void cooling_boost_and_idle(void)
{
    HeatEstimate estimate = heat_estimate_begin(
        30, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    /* 参数由使用方传入；此处分别验证 1.5 倍增益后的实际冷却数值。 */
    uint16_t burst_boosted = RMUL2027_INFANTRY_BURST_COOLING_PER_S * 3U / 2U;
    uint16_t cool_boosted = RMUL2027_INFANTRY_COOL_COOLING_PER_S * 3U / 2U;
    assert(heat_estimate_step(&estimate, 0, 100, burst_boosted));
    expect_heat(&estimate, 27900);
    assert(heat_estimate_step(&estimate, 0, 100, cool_boosted));
    expect_heat(&estimate, 24300);
    assert(heat_estimate_step(&estimate, 2, 5000, cool_boosted));
    expect_heat(&estimate, 20000);
    assert(heat_estimate_step(&estimate, 0, 200, 0));
    expect_heat(&estimate, 20000);
}

static void one_ms_calls_and_reset(void)
{
    HeatEstimate split = heat_estimate_begin(
        30, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    HeatEstimate single = split;
    assert(heat_estimate_step(&single, 0, 1000, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    for (unsigned tick = 0; tick < 1000; ++tick) {
        assert(heat_estimate_step(&split, 0, 1, RMUL2027_INFANTRY_BURST_COOLING_PER_S));
    }
    expect_heat(&single, 16000);
    expect_heat(&split, 16000);
    split = heat_estimate_begin(
        0, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    expect_heat(&split, 0);
    assert(split.cooling_elapsed_ms == 0);
}

static void invalid_state_and_numeric_limits(void)
{
    uint64_t untouched = 123;
    HeatEstimate estimate = {0};
    assert(!heat_estimate_read(&estimate, &untouched));
    assert(!heat_estimate_step(&estimate, 1, 1, 1));
    assert(untouched == 123);
    estimate = heat_estimate_begin(0, 0, RMUL2027_HEAT_COOLING_PERIOD_MS);
    assert(!heat_estimate_step(&estimate, 1, 1, 1));
    estimate = heat_estimate_begin(0, RMUL2027_17MM_HEAT_PER_EVENT, 0);
    assert(!heat_estimate_step(&estimate, 1, 1, 1));
    estimate = heat_estimate_begin(
        0, RMUL2027_17MM_HEAT_PER_EVENT, RMUL2027_HEAT_COOLING_PERIOD_MS);
    assert(heat_estimate_step(&estimate, UINT32_MAX, 99, 0));
    expect_heat(&estimate, (uint64_t)UINT32_MAX * RMUL2027_17MM_HEAT_PER_EVENT * 1000);
    assert(heat_estimate_step(&estimate, 0, UINT32_MAX, UINT16_MAX));
    assert(estimate.cooling_elapsed_ms == ((uint64_t)99 + UINT32_MAX) % 100);
    estimate.milliheat = UINT64_MAX;
    assert(!heat_estimate_step(&estimate, 1, 0, 0));
    assert(!heat_estimate_read(&estimate, &untouched));
    assert(untouched == 123);
}

int main(void)
{
    burst_cooling_boundary();
    cool_profile_and_batch_count();
    cooling_boost_and_idle();
    one_ms_calls_and_reset();
    invalid_state_and_numeric_limits();
    return 0;
}
