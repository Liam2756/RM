#ifndef RM_HEAT_MONITOR_H
#define RM_HEAT_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/* 裁判回传的规则数值及车端接收时刻。未收到字段时 received 为 false。 */
typedef struct {
    uint16_t value;
    uint32_t received_ms;
    bool received;
} HeatReading;

typedef enum {
    HEAT_DATA_MISSING = 0,
    HEAT_DATA_STALE,
    HEAT_BELOW_LIMIT,
    HEAT_AT_LIMIT,
    HEAT_ABOVE_LIMIT
} HeatStatus;

typedef struct {
    HeatStatus estimated_status;
    HeatStatus referee_status;
} HeatReport;

/* 用于显示和日志。两项读数可以来自不同报文，各自携带接收时间。 */
HeatStatus heat_monitor_compare(HeatReading heat, HeatReading limit,
                                uint32_t now_ms, uint32_t heat_max_age_ms,
                                uint32_t limit_max_age_ms);

/* 日志/显示用。估值单位为千分之一热量；传空指针表示无有效估值。 */
HeatReport heat_monitor_report(const uint64_t *estimate_milliheat,
                               HeatReading referee, HeatReading limit,
                               uint32_t now_ms, uint32_t referee_max_age_ms,
                               uint32_t limit_max_age_ms);

#endif
