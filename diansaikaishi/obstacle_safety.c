#include "obstacle_safety.h"

#include "car_controller.h"
#include "car_state.h"
#include "mission_manager.h"
#include "obstacle_monitor.h"

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

void ObstacleSafety_Init(void)
{
    obstacle_safety_apply_hold(false);
}

void ObstacleSafety_Update_20ms(void)
{
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
    TrackRunMode mode = CarController_GetRunMode();
    bool canHold = obstacle_safety_can_hold_mode(mode);

    if (CarState_Get() != CAR_STATE_RUNNING) {
        obstacle_safety_apply_hold(false);
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
