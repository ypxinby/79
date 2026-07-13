#include "motion_action.h"

#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "imu.h"

#define MOTION_ACTION_PERIOD_MS      (20U)
#define REACQUIRE_CENTER_CONFIRM_COUNT  (3U)
#define REACQUIRE_SETTLE_MS             (300U)

typedef enum {
    REACQUIRE_PHASE_SEARCH = 0,
    REACQUIRE_PHASE_SETTLE
} ReacquirePhase;

static MotionActionRuntime g_motionActionRuntime;
static float g_motionActionStartYawDeg;

static void motion_action_set_result(MotionActionResult result,
    MotionErrorCode error_code)
{
    g_motionActionRuntime.result = result;
    g_motionActionRuntime.error_code = (uint16_t)error_code;
}

static void motion_action_stop_car(void)
{
    CarController_Stop();
}

static CarTurnHandlingPolicy motion_action_map_turn_policy(
    TurnHandlingPolicy policy)
{
    switch (policy) {
        case TURN_POLICY_REPORT_ONLY:
            return CAR_TURN_POLICY_REPORT_ONLY;
        case TURN_POLICY_IGNORE:
            return CAR_TURN_POLICY_IGNORE;
        case TURN_POLICY_AUTO:
        default:
            return CAR_TURN_POLICY_AUTO;
    }
}

static bool motion_action_car_is_error(void)
{
    return CarState_Get() == CAR_STATE_ERROR;
}

static float motion_action_resolve_yaw_target(const MotionYawTarget *target,
    float mission_start_yaw_deg)
{
    float resolved_yaw;

    switch (target->reference) {
        case YAW_REFERENCE_CURRENT:
            resolved_yaw = g_motionActionStartYawDeg + target->angle_deg;
            break;
        case YAW_REFERENCE_MISSION_START:
            resolved_yaw = mission_start_yaw_deg + target->angle_deg;
            break;
        case YAW_REFERENCE_SECOND_SEEK_CONFIG:
            resolved_yaw =
                mission_start_yaw_deg +
                (float)g_appConfig.second_seek_angle_deg +
                target->angle_deg;
            break;
        case YAW_REFERENCE_ABSOLUTE:
        default:
            resolved_yaw = target->angle_deg;
            break;
    }

    return resolved_yaw + (float)g_appConfig.seek_heading_offset_deg;
}

static bool motion_action_check_timeout(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) || (action->timeout_ms == 0U)) {
        return false;
    }

    if (g_motionActionRuntime.elapsed_ms < action->timeout_ms) {
        return false;
    }

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_SEEK_TIMEOUT);
            break;
        case MOTION_ACTION_FOLLOW_LINE:
        case MOTION_ACTION_REACQUIRE_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_FOLLOW_TIMEOUT);
            break;
        case MOTION_ACTION_DRIVE_HEADING_TIME:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_FOLLOW_TIMEOUT);
            break;
        case MOTION_ACTION_TURN_LEFT_90:
        case MOTION_ACTION_TURN_RIGHT_90:
        case MOTION_ACTION_TURN_TO_YAW:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_TURN_TIMEOUT);
            break;
        case MOTION_ACTION_WAIT:
        case MOTION_ACTION_STOP:
            return false;
        default:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_INVALID_ACTION);
            break;
    }

    motion_action_stop_car();
    return true;
}

void MotionAction_Init(void)
{
    g_motionActionRuntime.action = (const MotionAction *)0;
    g_motionActionRuntime.result = MOTION_RESULT_IDLE;
    g_motionActionRuntime.elapsed_ms = 0U;
    g_motionActionRuntime.error_code = (uint16_t)MOTION_ERROR_NONE;
    g_motionActionRuntime.reacquire_settle_ms = 0U;
    g_motionActionRuntime.reacquire_center_count = 0U;
    g_motionActionRuntime.reacquire_phase = (uint8_t)REACQUIRE_PHASE_SEARCH;
    g_motionActionRuntime.started = false;
    g_motionActionStartYawDeg = 0.0f;
}

bool MotionAction_Start(const MotionAction *action,
    float mission_start_yaw_deg)
{
    MotionAction_Init();

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return false;
    }

    g_motionActionRuntime.action = action;
    g_motionActionRuntime.started = true;
    g_motionActionStartYawDeg = Imu_GetYaw();

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            CarController_StartSeekLine(
                motion_action_resolve_yaw_target(
                    &action->params.seek_line.yaw,
                    mission_start_yaw_deg));
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_FOLLOW_LINE:
            CarController_StartFollowLine(
                motion_action_map_turn_policy(
                    action->params.follow_line.turn_policy));
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_TURN_LEFT_90:
            CarController_StartTurnLeft90();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_TURN_RIGHT_90:
            CarController_StartTurnRight90();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_TURN_TO_YAW:
            if (!Imu_IsReady()) {
                motion_action_stop_car();
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                return false;
            }
            CarController_StartTurnToYawRelative(
                action->params.turn_to_yaw.angle_deg);
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_DRIVE_HEADING_TIME:
            if (!Imu_IsReady()) {
                motion_action_stop_car();
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                return false;
            }
            CarController_StartDriveHeading(
                action->params.drive_heading_time.duration_ms);
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_REACQUIRE_LINE:
            if (!Imu_IsReady()) {
                motion_action_stop_car();
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                return false;
            }
            g_motionActionRuntime.reacquire_phase =
                (uint8_t)REACQUIRE_PHASE_SEARCH;
            g_motionActionRuntime.reacquire_center_count = 0U;
            g_motionActionRuntime.reacquire_settle_ms = 0U;
            CarController_StartSeekLine(Imu_GetYaw());
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_STOP:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_SUCCESS,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_WAIT:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        default:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_FAILED,
                MOTION_ERROR_INVALID_ACTION);
            return false;
    }
}

MotionActionResult MotionAction_Update_20ms(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if (!g_motionActionRuntime.started) {
        return MOTION_RESULT_IDLE;
    }

    if (g_motionActionRuntime.result != MOTION_RESULT_RUNNING) {
        return g_motionActionRuntime.result;
    }

    if (g_motionActionRuntime.elapsed_ms <
        UINT32_MAX - MOTION_ACTION_PERIOD_MS) {
        g_motionActionRuntime.elapsed_ms += MOTION_ACTION_PERIOD_MS;
    }

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return g_motionActionRuntime.result;
    }

    if (motion_action_check_timeout()) {
        return g_motionActionRuntime.result;
    }

    switch (action->type) {
        case MOTION_ACTION_SEEK_LINE:
            if (motion_action_car_is_error()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (CarController_GetRunMode() != TRACK_MODE_SEEK_LINE) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        case MOTION_ACTION_FOLLOW_LINE:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if ((action->params.follow_line.end_condition ==
                    FOLLOW_END_LINE_LOST) &&
                feedback->line_lost) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
                break;
            }
            if ((action->params.follow_line.end_condition ==
                    FOLLOW_END_LEFT_90_DETECTED) &&
                (feedback->detected_turn == TRACK_TURN_LEFT_90)) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
                break;
            }
            if ((action->params.follow_line.end_condition ==
                    FOLLOW_END_RIGHT_90_DETECTED) &&
                (feedback->detected_turn == TRACK_TURN_RIGHT_90)) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
                break;
            }
            if ((action->params.follow_line.end_condition ==
                    FOLLOW_END_ANY_90_DETECTED) &&
                (feedback->detected_turn != TRACK_TURN_NONE)) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
                break;
            }
            if (motion_action_car_is_error()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (action->params.follow_line.end_condition ==
                FOLLOW_END_DURATION) {
                if (g_motionActionRuntime.elapsed_ms >=
                    action->params.follow_line.duration_ms) {
                    motion_action_set_result(MOTION_RESULT_SUCCESS,
                        MOTION_ERROR_NONE);
                }
            }
            break;
        }

        case MOTION_ACTION_TURN_LEFT_90:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (feedback->turn_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_TURN_RIGHT_90:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (feedback->turn_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_TURN_TO_YAW:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (!Imu_IsReady()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                break;
            }
            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_TURN_TIMEOUT);
                break;
            }
            if (feedback->turn_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_DRIVE_HEADING_TIME:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (!Imu_IsReady()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                break;
            }
            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_FOLLOW_TIMEOUT);
                break;
            }
            if (feedback->turn_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_REACQUIRE_LINE:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (!Imu_IsReady()) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_IMU_NOT_READY);
                break;
            }
            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }

            if (g_motionActionRuntime.reacquire_phase ==
                (uint8_t)REACQUIRE_PHASE_SEARCH) {
                if (feedback->center_detected) {
                    if (g_motionActionRuntime.reacquire_center_count <
                        UINT8_MAX) {
                        g_motionActionRuntime.reacquire_center_count++;
                    }
                } else {
                    g_motionActionRuntime.reacquire_center_count = 0U;
                }

                if (g_motionActionRuntime.reacquire_center_count >=
                    REACQUIRE_CENTER_CONFIRM_COUNT) {
                    g_motionActionRuntime.reacquire_phase =
                        (uint8_t)REACQUIRE_PHASE_SETTLE;
                    g_motionActionRuntime.reacquire_center_count = 0U;
                    g_motionActionRuntime.reacquire_settle_ms = 0U;
                    CarController_StartFollowLine(CAR_TURN_POLICY_IGNORE);
                }
                break;
            }

            if (feedback->line_lost) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    MOTION_ERROR_LINE_LOST);
                break;
            }
            if (g_motionActionRuntime.reacquire_settle_ms <
                UINT16_MAX - MOTION_ACTION_PERIOD_MS) {
                g_motionActionRuntime.reacquire_settle_ms =
                    (uint16_t)(g_motionActionRuntime.reacquire_settle_ms +
                        MOTION_ACTION_PERIOD_MS);
            }
            if (feedback->center_detected &&
                (g_motionActionRuntime.reacquire_center_count < UINT8_MAX)) {
                g_motionActionRuntime.reacquire_center_count++;
            }
            if ((g_motionActionRuntime.reacquire_settle_ms >=
                    REACQUIRE_SETTLE_MS) &&
                (g_motionActionRuntime.reacquire_center_count > 0U)) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_WAIT:
            motion_action_stop_car();
            if (g_motionActionRuntime.elapsed_ms >=
                action->params.wait.wait_ms) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;

        default:
            motion_action_stop_car();
            motion_action_set_result(MOTION_RESULT_FAILED,
                MOTION_ERROR_INVALID_ACTION);
            break;
    }

    return g_motionActionRuntime.result;
}

void MotionAction_Cancel(void)
{
    motion_action_stop_car();
    motion_action_set_result(MOTION_RESULT_CANCELLED, MOTION_ERROR_NONE);
    g_motionActionRuntime.started = false;
}

bool MotionAction_ReapplyControllerTarget(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) ||
        !g_motionActionRuntime.started) {
        return false;
    }

    if (action->type != MOTION_ACTION_FOLLOW_LINE) {
        return false;
    }

    CarController_StartFollowLine(
        motion_action_map_turn_policy(action->params.follow_line.turn_policy));
    return true;
}

ObstaclePolicy MotionAction_GetCurrentObstaclePolicy(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) ||
        !g_motionActionRuntime.started ||
        (action->type != MOTION_ACTION_FOLLOW_LINE)) {
        return OBSTACLE_POLICY_STOP_ONLY;
    }

    return action->params.follow_line.obstacle_policy;
}

BypassDirection MotionAction_GetCurrentBypassDirection(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) ||
        !g_motionActionRuntime.started ||
        (action->type != MOTION_ACTION_FOLLOW_LINE)) {
        return BYPASS_DIRECTION_RIGHT;
    }

    return action->params.follow_line.bypass_direction;
}

const MotionActionRuntime *MotionAction_GetRuntime(void)
{
    return &g_motionActionRuntime;
}
