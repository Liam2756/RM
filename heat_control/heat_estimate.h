#ifndef RM_HEAT_ESTIMATE_H
#define RM_HEAT_ESTIMATE_H

#include <stdbool.h>
#include <stdint.h>

/* 每条日志时间线持有一个实例。零初始化表示尚无已对齐的起点。 */
typedef struct {
    uint64_t milliheat;
    uint32_t at_ms;
    uint32_t confirmed_count;
    uint16_t heat_per_event;
    uint16_t cooling_per_s;
    bool valid;
} HeatEstimate;

/* heat 和 count 必须描述同一时刻、同一通道，heat 已包含 count。 */
HeatEstimate heat_estimate_begin(uint16_t heat, uint32_t count, uint32_t at_ms,
                                 uint16_t heat_per_event, uint16_t cooling_per_s);

/* 按发生顺序提交一次已确认事件或冷却速率变更。新速率从 at_ms 起生效。 */
bool heat_estimate_observe(HeatEstimate *estimate, uint32_t count,
                           uint32_t at_ms, uint16_t cooling_per_s_after);

/* 按已知事件读取指定时刻的估值，单位为千分之一热量。失败时保留输出。 */
bool heat_estimate_read(const HeatEstimate *estimate, uint32_t at_ms,
                        uint64_t *milliheat);

#endif
