#include "emergency_stop.h"

#include "car_controller.h"
#include "fault.h"
#include "mission_manager.h"
#include "motor.h"
#include "obstacle_avoidance.h"
#include "obstacle_safety.h"
#include "scheduler_monitor.h"
#include "watchdog_monitor.h"

static volatile bool g_emergencyStopActive;

void EmergencyStop_Init(void)
{
    g_emergencyStopActive = false;
}

void EmergencyStop_Trigger(void)
{
    g_emergencyStopActive = true;
    Motor_Stop();
    CarController_SetSafetyHold(true);
    MissionManager_ReportExternalFailure(
        (uint16_t)FAULT_CODE_SOFTWARE_EMERGENCY_STOP);
    Fault_Raise(FAULT_CODE_SOFTWARE_EMERGENCY_STOP, 0U, 0U,
        SystemTime_GetMs());
}

void EmergencyStop_Enforce(void)
{
    if (!g_emergencyStopActive) {
        return;
    }

    Motor_Stop();
    CarController_SetSafetyHold(true);
    MissionManager_SetExternalHold(true);
}

void EmergencyStop_Reset(void)
{
    if (!g_emergencyStopActive) {
        return;
    }

    Motor_Stop();
    g_emergencyStopActive = false;
    WatchdogMonitor_Reset();
    ObstacleAvoidance_Init();
    CarController_SetSafetyHold(false);
    MissionManager_SetExternalHold(false);
    CarController_ResetRuntime();
    MissionManager_Reset();
    ObstacleSafety_Init();
    Fault_Clear();
}

bool EmergencyStop_IsActive(void)
{
    return g_emergencyStopActive;
}
