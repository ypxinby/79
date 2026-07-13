#include "obstacle_avoidance.h"

#include "car_controller.h"
#include "car_state.h"
#include "imu.h"
#include "mission_manager.h"
#include "motion_action.h"
#include "motion_types.h"
#include "obstacle_monitor.h"

#define AVOID_TURN_OUT_DEG              (35.0f)
#define AVOID_DRIVE_OUT_MS              (1400U)
#define AVOID_TURN_TO_LINE_DEG          (-55.0f)
#define AVOID_WAIT_AFTER_OBSTACLE_MS    (2000U)
#define AVOID_CENTER_CONFIRM_COUNT      (3U)
#define AVOID_REACQUIRE_SETTLE_MS       (300U)
#define AVOID_RESUME_GRACE_MS           (600U)
#define AVOID_PERIOD_MS                 (20U)

static ObstacleAvoidanceFeedback g_avoidFeedback;
static bool g_stateStarted;
static uint16_t g_resumeGraceMs;

static void avoid_set_state(ObstacleAvoidState state)
{
    g_avoidFeedback.state = state;
    g_stateStarted = false;
}

static bool avoid_can_start(void)
{
    const MotionActionRuntime *runtime = MotionAction_GetRuntime();
    const ObstacleFeedback *obstacle = ObstacleMonitor_GetFeedback();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        return false;
    }
    if (!obstacle->blocked) {
        return false;
    }
    if ((runtime->action == (const MotionAction *)0) ||
        !runtime->started) {
        return false;
    }
    if (runtime->action->type != MOTION_ACTION_FOLLOW_LINE) {
        return false;
    }

    return true;
}

static void avoid_fail(void)
{
    CarController_Stop();
    g_avoidFeedback.active = false;
    g_avoidFeedback.failed = true;
    MissionManager_SetExternalHold(true);
    avoid_set_state(AVOID_STATE_FAILED);
}

static void avoid_start(void)
{
    g_avoidFeedback.active = true;
    g_avoidFeedback.failed = false;
    g_avoidFeedback.center_count = 0U;
    g_avoidFeedback.wait_ms = 0U;
    g_avoidFeedback.settle_ms = 0U;
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
    g_avoidFeedback.wait_ms = 0U;
    g_avoidFeedback.settle_ms = 0U;
    g_avoidFeedback.center_count = 0U;
    g_stateStarted = false;
    g_resumeGraceMs = 0U;
}

void ObstacleAvoidance_Update_20ms(void)
{
    const CarControllerFeedback *feedback = CarController_GetFeedback();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        ObstacleAvoidance_Init();
        return;
    }

    if (g_resumeGraceMs >= AVOID_PERIOD_MS) {
        g_resumeGraceMs = (uint16_t)(g_resumeGraceMs - AVOID_PERIOD_MS);
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

    if (avoid_feedback_failed(feedback)) {
        avoid_fail();
        return;
    }

    switch (g_avoidFeedback.state) {
        case AVOID_STATE_WAIT_AFTER_OBSTACLE:
            if (g_avoidFeedback.wait_ms <
                UINT16_MAX - AVOID_PERIOD_MS) {
                g_avoidFeedback.wait_ms =
                    (uint16_t)(g_avoidFeedback.wait_ms + AVOID_PERIOD_MS);
            }
            if (g_avoidFeedback.wait_ms >= AVOID_WAIT_AFTER_OBSTACLE_MS) {
                avoid_set_state(AVOID_STATE_TURN_OUT);
            }
            break;

        case AVOID_STATE_TURN_OUT:
            if (!g_stateStarted) {
                CarController_StartTurnToYawRelative(AVOID_TURN_OUT_DEG);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_DRIVE_OUT);
            }
            break;

        case AVOID_STATE_DRIVE_OUT:
            if (!g_stateStarted) {
                CarController_StartDriveHeading(AVOID_DRIVE_OUT_MS);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_TURN_TO_LINE);
            }
            break;

        case AVOID_STATE_TURN_TO_LINE:
            if (!g_stateStarted) {
                CarController_StartTurnToYawRelative(AVOID_TURN_TO_LINE_DEG);
                g_stateStarted = true;
                break;
            }
            if (feedback->turn_completed) {
                avoid_set_state(AVOID_STATE_REACQUIRE_SEARCH);
            }
            break;

        case AVOID_STATE_REACQUIRE_SEARCH:
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
                avoid_fail();
                break;
            }
            if (g_avoidFeedback.settle_ms < UINT16_MAX - AVOID_PERIOD_MS) {
                g_avoidFeedback.settle_ms =
                    (uint16_t)(g_avoidFeedback.settle_ms + AVOID_PERIOD_MS);
            }
            if (g_avoidFeedback.settle_ms >= AVOID_REACQUIRE_SETTLE_MS) {
                avoid_set_state(AVOID_STATE_COMPLETE);
            }
            break;

        case AVOID_STATE_COMPLETE:
            if (!MotionAction_ReapplyControllerTarget()) {
                avoid_fail();
                break;
            }
            g_avoidFeedback.active = false;
            g_avoidFeedback.failed = false;
            g_resumeGraceMs = AVOID_RESUME_GRACE_MS;
            MissionManager_SetExternalHold(false);
            avoid_set_state(AVOID_STATE_IDLE);
            break;

        case AVOID_STATE_FAILED:
            g_avoidFeedback.active = false;
            MissionManager_SetExternalHold(true);
            break;

        case AVOID_STATE_IDLE:
        default:
            g_avoidFeedback.active = false;
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
