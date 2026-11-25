#ifndef RM_HEAT_ESTIMATE_H
#define RM_HEAT_ESTIMATE_H

#include <stdbool.h>
#include <stdint.h>

/* 每个热量通道持有一个实例。零初始化表示尚无已知热量起点。 */
typedef struct {
    uint64_t milliheat;
    uint16_t heat_per_event;
    uint16_t cooling_period_ms;
    uint16_t cooling_elapsed_ms;
    bool valid;
} HeatEstimate;

/*
 * heat 是开始统计新增数量之前的已知热量，cooling_period_ms 为冷却结算周期。
 * 初始化时从本地结算周期起点开始，尚未对齐裁判时钟。
 */
HeatEstimate heat_estimate_begin(uint16_t heat, uint16_t heat_per_event,
                                 uint16_t cooling_period_ms);

/*
 * 定时调用，每个区间只提交一次新增计数，允许多发。
 * elapsed_ms 为本区间时长。达到结算周期时才冷却，保留不足一周期的余量。
 * cooling_per_s 为本次结算采用的冷却速率，单位为热量/秒。
 * 先结算上一拍热量的冷却，再将本区间新增量统一计在末尾。
 */
bool heat_estimate_step(HeatEstimate *estimate, uint32_t new_events,
                        uint32_t elapsed_ms, uint16_t cooling_per_s);

/* 读取最近一次计算后的估值，单位为千分之一热量。失败时保留输出。 */
bool heat_estimate_read(const HeatEstimate *estimate, uint64_t *milliheat);

#endif
