#include "watchdog_monitor.h"

#include "app_features.h"
#include "car_controller.h"
#include "fault.h"
#include "mission_manager.h"
#include "motor.h"

#define APP_HEARTBEAT_TIMEOUT_MS    (100U)

#if FEATURE_HARDWARE_WATCHDOG
#error FEATURE_HARDWARE_WATCHDOG requires a reviewed SysConfig WWDT instance
#endif

static volatile bool g_faultPending;
static bool g_faultApplied;
static volatile uint32_t g_lastTickMs;
static WatchdogMonitorStatus g_status;

static bool watchdog_hardware_init(void)
{
    /*
     * P1 skeleton only. No WWDT instance exists in empty.syscfg yet, so it is
     * unsafe to claim hardware reset coverage. P2 entry review must allocate
     * and validate the WWDT clock/reset/debug policy before enabling this.
     */
    return false;
}

static void watchdog_hardware_feed(void)
{
    /* Intentionally empty until a SysConfig-backed WWDT is allocated. */
}

void WatchdogMonitor_Init(uint32_t now_ms)
{
    g_status.heartbeat_seen = false;
    g_status.tripped = false;
    g_status.hardware_watchdog_enabled = watchdog_hardware_init();
    g_status.last_heartbeat_ms = now_ms;
    g_status.heartbeat_age_ms = 0U;
    g_status.timeout_ms = APP_HEARTBEAT_TIMEOUT_MS;
    g_status.trip_count = 0U;
    g_faultPending = false;
    g_faultApplied = false;
    g_lastTickMs = now_ms;
}

void WatchdogMonitor_Tick1msFromIsr(uint32_t now_ms)
{
    uint32_t age;

    g_lastTickMs = now_ms;
    if (g_status.tripped || !g_status.heartbeat_seen) {
        return;
    }

    age = now_ms - g_status.last_heartbeat_ms;
    g_status.heartbeat_age_ms = age;
    if (age < g_status.timeout_ms) {
        return;
    }

    g_status.tripped = true;
    if (g_status.trip_count < UINT32_MAX) {
        g_status.trip_count++;
    }
    g_faultPending = true;
    g_faultApplied = false;

    /* The main loop may be stalled, so final output must be stopped here. */
    Motor_Stop();
}

void WatchdogMonitor_NotifyControlCycleComplete(uint32_t now_ms)
{
    if (g_status.tripped) {
        return;
    }

    g_status.heartbeat_seen = true;
    g_status.last_heartbeat_ms = now_ms;
    g_status.heartbeat_age_ms = 0U;
    if (g_status.hardware_watchdog_enabled) {
        watchdog_hardware_feed();
    }
}

void WatchdogMonitor_ApplyFaultIfNeeded(uint32_t now_ms)
{
    if (g_faultApplied || (!g_faultPending && !g_status.tripped)) {
        return;
    }

    g_faultPending = false;
    CarController_SetSafetyHold(true);
    MissionManager_ReportExternalFailure(
        (uint16_t)FAULT_CODE_APP_HEARTBEAT_TIMEOUT);
    Fault_Raise(FAULT_CODE_APP_HEARTBEAT_TIMEOUT, 0U, 0U, now_ms);
    g_faultApplied = true;
}

void WatchdogMonitor_Reset(void)
{
    g_status.heartbeat_seen = false;
    g_status.tripped = false;
    g_status.last_heartbeat_ms = g_lastTickMs;
    g_status.heartbeat_age_ms = 0U;
    g_faultPending = false;
    g_faultApplied = false;
}

bool WatchdogMonitor_HasTripped(void)
{
    return g_status.tripped;
}

const WatchdogMonitorStatus *WatchdogMonitor_GetStatus(void)
{
    return &g_status;
}
