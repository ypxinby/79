#ifndef WATCHDOG_MONITOR_H
#define WATCHDOG_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool heartbeat_seen;
    bool tripped;
    bool hardware_watchdog_enabled;
    uint32_t last_heartbeat_ms;
    uint32_t heartbeat_age_ms;
    uint32_t timeout_ms;
    uint32_t trip_count;
} WatchdogMonitorStatus;

void WatchdogMonitor_Init(uint32_t now_ms);
void WatchdogMonitor_Tick1msFromIsr(uint32_t now_ms);
void WatchdogMonitor_NotifyControlCycleComplete(uint32_t now_ms);
void WatchdogMonitor_ApplyFaultIfNeeded(uint32_t now_ms);
void WatchdogMonitor_Reset(void);
bool WatchdogMonitor_HasTripped(void);
const WatchdogMonitorStatus *WatchdogMonitor_GetStatus(void);

#endif
