#include "obstacle_safety.h"

#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "mission_manager.h"
#include "obstacle_avoidance.h"
#include "obstacle_monitor.h"
#include "ultrasonic.h"
#include "watchdog_monitor.h"

#define OBSTACLE_EMERGENCY_DISTANCE_CM  (10U)

static bool g_obstacleSafetyHolding;

static bool obstacle_safety_can_hold_mode(TrackRunMode mode)
{
    return (mode == TRACK_MODE_SEEK_LINE) ||
        (mode == TRACK_MODE_FOLLOW_LINE) ||
        (mode == TRACK_MODE_LOST_RECOVER);
}

static void obstacle_safety_apply_hold(bool enable)
{
    g_obstacleSafetyHolding = enable;
    CarController_SetSafetyHold(enable);
    MissionManager_SetExternalHold(enable);
}

static bool obstacle_safety_emergency_blocked(void)
{
    const UltrasonicFeedback *ultrasonic = Ultrasonic_GetFeedback();

    return ultrasonic->measurement_valid &&
        (ultrasonic->distance_cm < OBSTACLE_EMERGENCY_DISTANCE_CM);
}

void ObstacleSafety_Init(void)
{
    obstacle_safety_apply_hold(false);
}

void ObstacleSafety_Update_20ms(void)
{
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
    TrackRunMode mode = CarController_GetRunMode();
    bool canHold = obstacle_safety_can_hold_mode(mode);

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped() ||
        ObstacleAvoidance_IsFailed()) {
        obstacle_safety_apply_hold(true);
        return;
    }

    if (CarState_Get() != CAR_STATE_RUNNING) {
        obstacle_safety_apply_hold(false);
        return;
    }

    if (ObstacleAvoidance_IsActive()) {
        const ObstacleAvoidanceFeedback *avoid =
            ObstacleAvoidance_GetFeedback();

        if (avoid->state == AVOID_STATE_WAIT_AFTER_OBSTACLE) {
            obstacle_safety_apply_hold(true);
            return;
        }

        if (obstacle_safety_emergency_blocked()) {
            obstacle_safety_apply_hold(true);
            return;
        }

        g_obstacleSafetyHolding = false;
        CarController_SetSafetyHold(false);
        MissionManager_SetExternalHold(true);
        return;
    }

    if (ObstacleAvoidance_IsResumeGraceActive()) {
        if (obstacle_safety_emergency_blocked()) {
            obstacle_safety_apply_hold(true);
            return;
        }

        g_obstacleSafetyHolding = false;
        CarController_SetSafetyHold(false);
        MissionManager_SetExternalHold(false);
        return;
    }

    if (obstacle->blocked && canHold) {
        obstacle_safety_apply_hold(true);
        return;
    }

    if (!obstacle->blocked) {
        obstacle_safety_apply_hold(false);
    }
}

bool ObstacleSafety_IsHolding(void)
{
    return g_obstacleSafetyHolding;
}
