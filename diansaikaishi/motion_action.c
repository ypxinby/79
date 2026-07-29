#include "motion_action.h"

#include "app_config.h"
#include "app_features.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "imu.h"
#include "scheduler_monitor.h"
#include "watchdog_monitor.h"

static MotionActionRuntime g_motionActionRuntime;

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

static uint16_t motion_action_add_elapsed_u16(uint16_t value,
    uint32_t elapsed_ms)
{
    if ((elapsed_ms >= UINT16_MAX) ||
        (value > (uint16_t)(UINT16_MAX - (uint16_t)elapsed_ms))) {
        return UINT16_MAX;
    }
    return (uint16_t)(value + (uint16_t)elapsed_ms);
}

static bool motion_action_is_imu_dependent(MotionActionType type)
{
    return (type == MOTION_ACTION_TURN_TO_YAW) ||
        (type == MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) ||
        (type == MOTION_ACTION_DRIVE_DISTANCE_HEADING)
#if FEATURE_LEGACY_MOTION_CONTROL
        || (type == MOTION_ACTION_DRIVE_HEADING_TIME)
#endif
        ;
}

static void motion_action_start_imu_controller(const MotionAction *action)
{
    if (action->type == MOTION_ACTION_TURN_TO_YAW) {
        CarController_StartTurnToYawRelative(
            action->params.turn_to_yaw.angle_deg,
            action->timeout_ms);
    }
#if FEATURE_LEGACY_MOTION_CONTROL
    else if (action->type == MOTION_ACTION_DRIVE_HEADING_TIME) {
        CarController_StartDriveHeading(
            action->params.drive_heading_time.target_yaw_deg,
            action->params.drive_heading_time.duration_ms,
            (action->params.drive_heading_time.speed_override ==
                MOTION_USE_GLOBAL_SPEED) ? g_appConfig.search_speed :
                action->params.drive_heading_time.speed_override);
    }
#endif
    else if (action->type ==
        MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) {
        CarController_StartDriveHeading(
            action->params.drive_heading_until_line.target_yaw_deg,
            action->timeout_ms,
            action->params.drive_heading_until_line.normalized_command);
    } else {
        CarController_StartDriveDistanceAtYaw(
            action->params.drive_distance_heading.distance_cm,
            action->params.drive_distance_heading.lock_current_yaw_on_start ?
                Imu_GetYaw() :
                action->params.drive_distance_heading.target_yaw_deg,
            action->params.drive_distance_heading.normalized_command);
    }

    g_motionActionRuntime.controller_started = true;
    g_motionActionRuntime.waiting_for_imu = false;
    g_motionActionRuntime.imu_wait_elapsed_ms = 0U;
}

static bool motion_action_update_imu_start_wait(uint32_t elapsed_ms)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) ||
        !motion_action_is_imu_dependent(action->type) ||
        g_motionActionRuntime.controller_started) {
        return false;
    }

    motion_action_stop_car();
    g_motionActionRuntime.waiting_for_imu = true;
    if (Imu_IsReady()) {
        motion_action_start_imu_controller(action);
        g_motionActionRuntime.elapsed_ms = 0U;
        return false;
    }

    g_motionActionRuntime.imu_wait_elapsed_ms =
        motion_action_add_elapsed_u16(
            g_motionActionRuntime.imu_wait_elapsed_ms, elapsed_ms);
    if (g_motionActionRuntime.imu_wait_elapsed_ms <
        g_appConfig.heading_imu_invalid_grace_ms) {
        return true;
    }

    motion_action_set_result(MOTION_RESULT_FAILED,
        MOTION_ERROR_IMU_NOT_READY);
    Fault_Raise(FAULT_CODE_TURN_YAW_IMU_NOT_READY,
        g_motionActionRuntime.imu_wait_elapsed_ms,
        (uint16_t)action->type, SystemTime_GetMs());
    CarState_Set(CAR_STATE_ERROR);
    return true;
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

static bool motion_action_check_timeout(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) || (action->timeout_ms == 0U)) {
        return false;
    }

    /* TURN_TO_YAW owns the same timeout in CarController so it can latch
     * zero output and a precise controller-side timeout result first. */
    if (action->type == MOTION_ACTION_TURN_TO_YAW) {
        return false;
    }

    if (g_motionActionRuntime.elapsed_ms < action->timeout_ms) {
        return false;
    }

    switch (action->type) {
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_SEEK_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_SEEK_TIMEOUT);
            break;
#endif
        case MOTION_ACTION_FOLLOW_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_FOLLOW_TIMEOUT);
            break;
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_DRIVE_HEADING_TIME:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_FOLLOW_TIMEOUT);
            break;
#endif
        case MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_SEEK_TIMEOUT);
            break;
        case MOTION_ACTION_DRIVE_DISTANCE:
        case MOTION_ACTION_DRIVE_DISTANCE_HEADING:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_DISTANCE_TIMEOUT);
            break;
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_TURN_LEFT_90:
        case MOTION_ACTION_TURN_RIGHT_90:
            motion_action_set_result(MOTION_RESULT_TIMEOUT,
                MOTION_ERROR_TURN_TIMEOUT);
            break;
#endif
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
    g_motionActionRuntime.imu_wait_elapsed_ms = 0U;
    g_motionActionRuntime.error_code = (uint16_t)MOTION_ERROR_NONE;
    g_motionActionRuntime.started = false;
    g_motionActionRuntime.controller_started = false;
    g_motionActionRuntime.waiting_for_imu = false;
}

static bool motion_action_start_internal(const MotionAction *action,
    bool resume_current_action)
{
    MotionAction_Init();

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        motion_action_stop_car();
        return false;
    }

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return false;
    }

    g_motionActionRuntime.action = action;
    g_motionActionRuntime.started = true;

    if ((action->type == MOTION_ACTION_TURN_TO_YAW) &&
        ((action->params.turn_to_yaw.angle_deg !=
            action->params.turn_to_yaw.angle_deg) ||
         (action->params.turn_to_yaw.angle_deg < -180.0f) ||
         (action->params.turn_to_yaw.angle_deg > 180.0f))) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        motion_action_stop_car();
        return false;
    }
    if ((action->type == MOTION_ACTION_DRIVE_DISTANCE) &&
        ((action->params.drive_distance.distance_cm !=
            action->params.drive_distance.distance_cm) ||
         (action->params.drive_distance.distance_cm == 0.0f) ||
         (action->params.drive_distance.normalized_command <= 0) ||
         (action->params.drive_distance.normalized_command >
            MOTION_NORMALIZED_COMMAND_MAX))) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        motion_action_stop_car();
        return false;
    }
    if ((action->type == MOTION_ACTION_DRIVE_DISTANCE_HEADING) &&
        ((action->params.drive_distance_heading.distance_cm !=
            action->params.drive_distance_heading.distance_cm) ||
         (action->params.drive_distance_heading.distance_cm == 0.0f) ||
         (!action->params.drive_distance_heading.lock_current_yaw_on_start &&
          ((action->params.drive_distance_heading.target_yaw_deg !=
                action->params.drive_distance_heading.target_yaw_deg) ||
           (action->params.drive_distance_heading.target_yaw_deg < -180.0f) ||
           (action->params.drive_distance_heading.target_yaw_deg > 180.0f))) ||
         (action->params.drive_distance_heading.normalized_command <= 0) ||
         (action->params.drive_distance_heading.normalized_command >
            MOTION_NORMALIZED_COMMAND_MAX))) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        motion_action_stop_car();
        return false;
    }
    if ((action->type == MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE) &&
        ((action->params.drive_heading_until_line.target_yaw_deg !=
            action->params.drive_heading_until_line.target_yaw_deg) ||
         (action->params.drive_heading_until_line.target_yaw_deg < -180.0f) ||
         (action->params.drive_heading_until_line.target_yaw_deg > 180.0f) ||
         (action->params.drive_heading_until_line.normalized_command <= 0) ||
         (action->params.drive_heading_until_line.normalized_command >
            MOTION_NORMALIZED_COMMAND_MAX))) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        motion_action_stop_car();
        return false;
    }

    switch (action->type) {
#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_SEEK_LINE:
            CarController_StartSeekLine();
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;
#endif

        case MOTION_ACTION_FOLLOW_LINE:
            if (resume_current_action) {
                CarController_ResumeFollowLine(
                    motion_action_map_turn_policy(
                        action->params.follow_line.turn_policy));
            } else {
                CarController_StartFollowLine(
                    motion_action_map_turn_policy(
                        action->params.follow_line.turn_policy));
            }
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

#if FEATURE_LEGACY_MOTION_CONTROL
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

#endif

        case MOTION_ACTION_TURN_TO_YAW:
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            if (Imu_IsReady()) {
                motion_action_start_imu_controller(action);
            } else {
                motion_action_stop_car();
                g_motionActionRuntime.waiting_for_imu = true;
            }
            return true;

#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_DRIVE_HEADING_TIME:
#endif
        case MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE:
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            if (Imu_IsReady()) {
                motion_action_start_imu_controller(action);
            } else {
                motion_action_stop_car();
                g_motionActionRuntime.waiting_for_imu = true;
            }
            return true;

        case MOTION_ACTION_DRIVE_DISTANCE:
            CarController_StartDriveDistance(
                action->params.drive_distance.distance_cm,
                action->params.drive_distance.normalized_command);
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            return true;

        case MOTION_ACTION_DRIVE_DISTANCE_HEADING:
            motion_action_set_result(MOTION_RESULT_RUNNING,
                MOTION_ERROR_NONE);
            if (Imu_IsReady()) {
                motion_action_start_imu_controller(action);
            } else {
                motion_action_stop_car();
                g_motionActionRuntime.waiting_for_imu = true;
            }
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

bool MotionAction_Start(const MotionAction *action)
{
    return motion_action_start_internal(action, false);
}

bool MotionAction_Resume(const MotionAction *action)
{
    return motion_action_start_internal(action, true);
}

MotionActionResult MotionAction_Update_20ms(uint32_t elapsed_ms)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if (!g_motionActionRuntime.started) {
        return MOTION_RESULT_IDLE;
    }

    if (g_motionActionRuntime.result != MOTION_RESULT_RUNNING) {
        return g_motionActionRuntime.result;
    }

    if (g_motionActionRuntime.elapsed_ms > UINT32_MAX - elapsed_ms) {
        g_motionActionRuntime.elapsed_ms = UINT32_MAX;
    } else {
        g_motionActionRuntime.elapsed_ms += elapsed_ms;
    }

    if (action == (const MotionAction *)0) {
        motion_action_set_result(MOTION_RESULT_FAILED,
            MOTION_ERROR_INVALID_ACTION);
        return g_motionActionRuntime.result;
    }

    if (motion_action_update_imu_start_wait(elapsed_ms)) {
        return g_motionActionRuntime.result;
    }

    if (motion_action_check_timeout()) {
        return g_motionActionRuntime.result;
    }

    switch (action->type) {
#if FEATURE_LEGACY_MOTION_CONTROL
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
#endif

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

#if FEATURE_LEGACY_MOTION_CONTROL
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
#endif

        case MOTION_ACTION_TURN_TO_YAW:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (feedback->operation_failed) {
                if (feedback->error_code ==
                    CAR_CONTROLLER_ERROR_YAW_TURN_TIMEOUT) {
                    motion_action_set_result(MOTION_RESULT_TIMEOUT,
                        MOTION_ERROR_TURN_TIMEOUT);
                } else {
                    motion_action_set_result(MOTION_RESULT_FAILED,
                        MOTION_ERROR_IMU_NOT_READY);
                }
                break;
            }
            if (motion_action_car_is_error()) {
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

#if FEATURE_LEGACY_MOTION_CONTROL
        case MOTION_ACTION_DRIVE_HEADING_TIME:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    (feedback->error_code ==
                        CAR_CONTROLLER_ERROR_IMU_NOT_READY) ?
                        MOTION_ERROR_IMU_NOT_READY :
                        MOTION_ERROR_FOLLOW_TIMEOUT);
                break;
            }
            if (feedback->turn_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }
#endif

        case MOTION_ACTION_DRIVE_HEADING_UNTIL_LINE:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    (feedback->error_code ==
                        CAR_CONTROLLER_ERROR_IMU_NOT_READY) ?
                        MOTION_ERROR_IMU_NOT_READY :
                        MOTION_ERROR_INVALID_ACTION);
                break;
            }
            if (feedback->line_found) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_DRIVE_DISTANCE:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                motion_action_set_result(MOTION_RESULT_FAILED,
                    (feedback->error_code ==
                        CAR_CONTROLLER_ERROR_ENCODER_NOT_READY) ?
                        MOTION_ERROR_ENCODER_NOT_READY :
                        MOTION_ERROR_INVALID_ACTION);
                break;
            }
            if (feedback->distance_completed) {
                motion_action_set_result(MOTION_RESULT_SUCCESS,
                    MOTION_ERROR_NONE);
            }
            break;
        }

        case MOTION_ACTION_DRIVE_DISTANCE_HEADING:
        {
            const CarControllerFeedback *feedback =
                CarController_GetFeedback();

            if (motion_action_car_is_error() || feedback->operation_failed) {
                MotionErrorCode error = MOTION_ERROR_INVALID_ACTION;

                if (feedback->error_code ==
                    CAR_CONTROLLER_ERROR_ENCODER_NOT_READY) {
                    error = MOTION_ERROR_ENCODER_NOT_READY;
                } else if (feedback->error_code ==
                    CAR_CONTROLLER_ERROR_IMU_NOT_READY) {
                    error = MOTION_ERROR_IMU_NOT_READY;
                } else if (feedback->error_code ==
                    CAR_CONTROLLER_ERROR_HEADING_START_MISMATCH) {
                    error = MOTION_ERROR_HEADING_START_MISMATCH;
                }
                motion_action_set_result(MOTION_RESULT_FAILED, error);
                break;
            }
            if (feedback->distance_completed) {
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
    g_motionActionRuntime.controller_started = false;
    g_motionActionRuntime.waiting_for_imu = false;
    g_motionActionRuntime.imu_wait_elapsed_ms = 0U;
}

bool MotionAction_ReapplyControllerTarget(void)
{
    const MotionAction *action = g_motionActionRuntime.action;

    if ((action == (const MotionAction *)0) ||
        !g_motionActionRuntime.started || EmergencyStop_IsActive() ||
        WatchdogMonitor_HasTripped()) {
        return false;
    }

    if (action->type != MOTION_ACTION_FOLLOW_LINE) {
        return false;
    }

    CarController_ResumeFollowLine(
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
