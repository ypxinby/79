#include "obstacle_avoidance.h"

#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "imu.h"
#include "mission_manager.h"
#include "motion_action.h"
#include "motion_types.h"
#include "obstacle_monitor.h"
#include "scheduler_monitor.h"
#include "watchdog_monitor.h"

#define AVOID_CENTER_CONFIRM_COUNT      (3U)
static ObstacleAvoidanceFeedback g_avoidFeedback;
static bool g_stateStarted;
static uint16_t g_resumeGraceMs;

static float avoid_direction_angle(float right_angle_deg)
{
    if (g_avoidFeedback.direction == BYPASS_DIRECTION_LEFT) {
        return -right_angle_deg;
    }

    return right_angle_deg;
}

static void avoid_set_state(ObstacleAvoidState state)
{
    g_avoidFeedback.state = state;
    g_avoidFeedback.state_elapsed_ms = 0U;
    g_stateStarted = false;
}

static uint16_t avoid_detail_from_feedback(
    const CarControllerFeedback *feedback)
{
    return (feedback == (const CarControllerFeedback *)0) ? 0U :
        (uint16_t)feedback->error_code;
}

static FaultCode avoid_fault_for_state(ObstacleAvoidState state)
{
    switch (state) {
        case AVOID_STATE_TURN_OUT:
            return FAULT_CODE_AVOID_TURN_OUT;
        case AVOID_STATE_DRIVE_OUT:
            return FAULT_CODE_AVOID_DRIVE_OUT;
        case AVOID_STATE_TURN_TO_LINE:
            return FAULT_CODE_AVOID_TURN_TO_LINE;
        case AVOID_STATE_REACQUIRE_SEARCH:
            return FAULT_CODE_AVOID_REACQUIRE_SEARCH;
        case AVOID_STATE_REACQUIRE_SETTLE:
            return FAULT_CODE_AVOID_REACQUIRE_SETTLE;
        default:
            return FAULT_CODE_AVOID_INVALID_STATE;
    }
}

static bool avoid_can_start(void)
{
    const MotionActionRuntime *runtime = MotionAction_GetRuntime();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();
    TrackRunMode mode = CarController_GetRunMode();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        return false;
    }
    if (!obstacle->blocked) {
        return false;
    }

    if ((runtime->action != (const MotionAction *)0) &&
        runtime->started &&
        (runtime->action->type == MOTION_ACTION_FOLLOW_LINE)) {
        return MotionAction_GetCurrentObstaclePolicy() ==
            OBSTACLE_POLICY_FIXED_BYPASS;
    }

    if (mode == TRACK_MODE_FOLLOW_LINE) {
        return MotionAction_GetCurrentObstaclePolicy() ==
            OBSTACLE_POLICY_FIXED_BYPASS;
    }

    return false;
}

static void avoid_fail(FaultCode code, uint16_t detail)
{
    ObstacleAvoidState failedStage = g_avoidFeedback.state;

    CarController_Stop();
    CarController_SetSafetyHold(true);
    g_avoidFeedback.active = false;
    g_avoidFeedback.failed = true;
    g_avoidFeedback.failure_code = code;
    g_avoidFeedback.failure_stage = failedStage;
    g_avoidFeedback.failure_detail = detail;
    MissionManager_SetExternalHold(true);
    avoid_set_state(AVOID_STATE_FAILED);
    MissionManager_ReportExternalFailure((uint16_t)code);
    Fault_Raise(code, detail, (uint16_t)failedStage, SystemTime_GetMs());
}

static void avoid_start(void)
{
    g_avoidFeedback.active = true;
    g_avoidFeedback.failed = false;
    g_avoidFeedback.direction = MotionAction_GetCurrentBypassDirection();
    g_avoidFeedback.center_count = 0U;
    g_avoidFeedback.wait_ms = 0U;
    g_avoidFeedback.settle_ms = 0U;
    g_avoidFeedback.failure_code = FAULT_CODE_NONE;
    g_avoidFeedback.failure_stage = AVOID_STATE_IDLE;
    g_avoidFeedback.failure_detail = 0U;
    MissionManager_SetExternalHold(true);
    avoid_set_state(AVOID_STATE_WAIT_AFTER_OBSTACLE);
}

static bool avoid_feedback_failed(const CarControllerFeedback *feedback)
{
    return (CarState_Get() == CAR_STATE_ERROR) ||
        feedback->operation_failed ||
        !Imu_IsReady();
}

void ObstacleAvoidance_Init(void)
{
    g_avoidFeedback.state = AVOID_STATE_IDLE;
    g_avoidFeedback.active = false;
    g_avoidFeedback.failed = false;
    g_avoidFeedback.direction = BYPASS_DIRECTION_RIGHT;
    g_avoidFeedback.wait_ms = 0U;
    g_avoidFeedback.settle_ms = 0U;
    g_avoidFeedback.center_count = 0U;
    g_avoidFeedback.state_elapsed_ms = 0U;
    g_avoidFeedback.failure_code = FAULT_CODE_NONE;
    g_avoidFeedback.failure_stage = AVOID_STATE_IDLE;
    g_avoidFeedback.failure_detail = 0U;
    g_stateStarted = false;
    g_resumeGraceMs = 0U;
}

void ObstacleAvoidance_Update_20ms(uint32_t elapsed_ms)
{
    const CarControllerFeedback *feedback = CarController_GetFeedback();

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        return;
    }

    if (CarState_Get() != CAR_STATE_RUNNING) {
        if (g_avoidFeedback.active &&
            (CarState_Get() == CAR_STATE_ERROR)) {
            avoid_fail(avoid_fault_for_state(g_avoidFeedback.state),
                avoid_detail_from_feedback(feedback));
        } else if (!g_avoidFeedback.failed) {
            ObstacleAvoidance_Init();
        }
        return;
    }

    if (g_resumeGraceMs >= elapsed_ms) {
        g_resumeGraceMs = (uint16_t)(g_resumeGraceMs - elapsed_ms);
    } else {
        g_resumeGraceMs = 0U;
    }

    if (!g_avoidFeedback.active) {
        if (g_avoidFeedback.failed) {
            return;
        }
        if (!avoid_can_start()) {
            return;
        }
        avoid_start();
    }

    MissionManager_SetExternalHold(true);

    if (g_avoidFeedback.state_elapsed_ms > UINT32_MAX - elapsed_ms) {
        g_avoidFeedback.state_elapsed_ms = UINT32_MAX;
    } else {
        g_avoidFeedback.state_elapsed_ms += elapsed_ms;
    }

    if (avoid_feedback_failed(feedback)) {
        avoid_fail(avoid_fault_for_state(g_avoidFeedback.state),
            avoid_detail_from_feedback(feedback));
        return;
    }

    switch (g_avoidFeedback.state) {
        case AVOID_STATE_WAIT_AFTER_OBSTACLE:
            if (elapsed_ms >= UINT16_MAX) {
                g_avoidFeedback.wait_ms = UINT16_MAX;
            } else if (g_avoidFeedback.wait_ms >
                (uint16_t)(UINT16_MAX - (uint16_t)elapsed_ms)) {
                g_avoidFeedback.wait_ms = UINT16_MAX;
            } else {
                g_avoidFeedback.wait_ms = (uint16_t)(
                    g_avoidFeedback.wait_ms + (uint16_t)elapsed_ms);
            }
            if (g_avoidFeedback.wait_ms >=
                g_appConfig.avoid_wait_before_ms) {
                avoid_set_state(AVOID_STATE_TURN_OUT);
            }
            break;

        case AVOID_STATE_TURN_OUT:
            if (!g_stateStarted) {
                CarController_StartTurnToYawRelative(
                    avoid_direction_angle(
                        (float)g_appConfig.avoid_turn_out_deg),
                    g_appConfig.yaw_turn_timeout_ms);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_DRIVE_OUT);
            }
            break;

        case AVOID_STATE_DRIVE_OUT:
            if (!g_stateStarted) {
                CarController_StartDriveHeading(
                    g_appConfig.avoid_drive_out_ms);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_TURN_TO_LINE);
            }
            break;

        case AVOID_STATE_TURN_TO_LINE:
            if (!g_stateStarted) {
                CarController_StartTurnToYawRelative(
                    avoid_direction_angle(
                        (float)g_appConfig.avoid_turn_to_line_deg),
                    g_appConfig.yaw_turn_timeout_ms);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_REACQUIRE_SEARCH);
            }
            break;

        case AVOID_STATE_REACQUIRE_SEARCH:
            if (g_avoidFeedback.state_elapsed_ms >=
                g_appConfig.avoid_reacquire_timeout_ms) {
                avoid_fail(FAULT_CODE_AVOID_REACQUIRE_SEARCH, 0U);
                break;
            }
            if (!g_stateStarted) {
                g_avoidFeedback.center_count = 0U;
                CarController_StartSeekLine(Imu_GetYaw());
                g_stateStarted = true;
                break;
            }
            if (feedback->center_detected) {
                if (g_avoidFeedback.center_count < UINT8_MAX) {
                    g_avoidFeedback.center_count++;
                }
            } else {
                g_avoidFeedback.center_count = 0U;
            }
            if (g_avoidFeedback.center_count >=
                AVOID_CENTER_CONFIRM_COUNT) {
                avoid_set_state(AVOID_STATE_REACQUIRE_SETTLE);
            }
            break;

        case AVOID_STATE_REACQUIRE_SETTLE:
            if (!g_stateStarted) {
                g_avoidFeedback.center_count = 0U;
                g_avoidFeedback.settle_ms = 0U;
                CarController_StartFollowLine(CAR_TURN_POLICY_IGNORE);
                g_stateStarted = true;
                break;
            }
            if (feedback->line_lost) {
                avoid_fail(FAULT_CODE_AVOID_REACQUIRE_SETTLE, 0U);
                break;
            }
            if (elapsed_ms >= UINT16_MAX) {
                g_avoidFeedback.settle_ms = UINT16_MAX;
            } else if (g_avoidFeedback.settle_ms >
                (uint16_t)(UINT16_MAX - (uint16_t)elapsed_ms)) {
                g_avoidFeedback.settle_ms = UINT16_MAX;
            } else {
                g_avoidFeedback.settle_ms = (uint16_t)(
                    g_avoidFeedback.settle_ms +
                    (uint16_t)elapsed_ms);
            }
            if (g_avoidFeedback.settle_ms >=
                g_appConfig.avoid_reacquire_settle_ms) {
                avoid_set_state(AVOID_STATE_COMPLETE);
            }
            break;

        case AVOID_STATE_COMPLETE:
            if (!MotionAction_ReapplyControllerTarget()) {
                CarController_StartFollowLine(CAR_TURN_POLICY_AUTO);
            }
            g_avoidFeedback.active = false;
            g_avoidFeedback.failed = false;
            g_resumeGraceMs = g_appConfig.avoid_resume_grace_ms;
            MissionManager_SetExternalHold(false);
            avoid_set_state(AVOID_STATE_IDLE);
            break;

        case AVOID_STATE_FAILED:
            g_avoidFeedback.active = false;
            MissionManager_SetExternalHold(true);
            break;

        case AVOID_STATE_IDLE:
            g_avoidFeedback.active = false;
            break;

        default:
            avoid_fail(FAULT_CODE_AVOID_INVALID_STATE, 0U);
            break;
    }
}

bool ObstacleAvoidance_IsActive(void)
{
    return g_avoidFeedback.active;
}

bool ObstacleAvoidance_IsFailed(void)
{
    return g_avoidFeedback.failed;
}

bool ObstacleAvoidance_IsResumeGraceActive(void)
{
    return g_resumeGraceMs > 0U;
}

const ObstacleAvoidanceFeedback *ObstacleAvoidance_GetFeedback(void)
{
    return &g_avoidFeedback;
}
