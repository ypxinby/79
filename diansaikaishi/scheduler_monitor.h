#ifndef SCHEDULER_MONITOR_H
#define SCHEDULER_MONITOR_H

#include <stdint.h>

typedef struct {
    uint32_t gimbal_5ms_missed_count;
    uint32_t gimbal_5ms_drop_count;
    uint32_t gimbal_5ms_overrun_count;
    uint32_t tracker_10ms_missed_count;
    uint32_t tracker_10ms_drop_count;
    uint32_t tracker_10ms_overrun_count;
    uint32_t app_20ms_missed_count;
    uint32_t app_20ms_drop_count;
    uint32_t app_20ms_overrun_count;
    uint32_t app_dt_clamp_count;
    uint32_t app_last_elapsed_ms;
} SchedulerStats;

uint32_t SystemTime_GetMs(void);
void SchedulerMonitor_GetStats(SchedulerStats *stats);

#endif
