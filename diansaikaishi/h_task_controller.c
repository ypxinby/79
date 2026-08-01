#include "h_task_controller.h"

#include <math.h>

#include "app_config.h"
#include "app_features.h"
#include "balance_ball_control.h"
#include "balance_position_control.h"
#include "balance_soft_limits.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "line_controller.h"
#include "motor_control.h"
#include "scheduler_monitor.h"
#include "vision_receiver.h"
#include "watchdog_monitor.h"
#include "wheel_speed_estimator.h"

#define H_TASK_CENTER_TARGET_MM             BALANCE_BALL_DEFAULT_TARGET_MM
/* Scoring targets are absolute K230 coordinates relative to physical O.
 * Mechanical neutral correction belongs to BIAS, never to target_mm. */
/* The installed K230 pipe coordinate is opposite to the physical scoring
 * labels observed on the vehicle: negative protocol position is physical
 * +5 cm, positive protocol position is physical -5 cm.  Drive 10 mm beyond
 * each scoring point so static friction and the old +/-10 mm arrival window
 * cannot stop a leg at only 3-4 cm.  Stage changes still use the real K230
 * position crossing the required +/-50 mm point. */
#define H_TASK_H3_FIRST_DRIVE_TARGET_MM      (-60)
#define H_TASK_H3_FIRST_REACH_MM             (-50)
#define H_TASK_H3_FINAL_DRIVE_TARGET_MM      (60)
#define H_TASK_H3_FINAL_REACH_MM             (50)
#define H_TASK_H3_FINAL_HOLD_TARGET_MM       (50)
#define H_TASK_LINE_NORMAL_COMMAND          (500)
#define H_TASK_LINE_LOW_COMMAND             (330)
#define H_TASK_LINE_START_COMMAND           (150)
#define H_TASK_LINE_FF_GAIN                  (0.60f)
#define H_TASK_LINE_KP                       (0.60f)
#define H_TASK_LINE_KD                       (0.010f)
#define H_TASK_LINE_MAX_CORRECTION           (260)
/* H4-H6 carry the ball while the chassis starts and changes speed.  Use a
 * task-local wheel-speed ramp; Task1 keeps its faster competition ramp. */
#define H_TASK_WHEEL_MAX_ACCEL_CMPS2          (25.0f)
#define H_TASK_WHEEL_MAX_DECEL_CMPS2          (25.0f)
#define H_TASK_WHEEL_LAUNCH_ACCEL_CMPS2       (12.5f)
#define H_TASK_LAUNCH_STABLE_ERROR_MM         (15)
#define H_TASK_LAUNCH_RESET_ERROR_MM          (30)
#define H_TASK_LAUNCH_SETTLE_MS               (300U)
#define H_TASK_LAUNCH_COMMAND_RATE_PER_S       (200U)
/* Longitudinal acceleration feedforward:
 * theta_ff = -atan(a_parallel / g).  The calibrated endpoint counts are
 * treated as approximately +/-10 degrees, matching the measured mechanism.
 * DIRECTION is the only installation-sign switch; GAIN is reserved for the
 * later real-vehicle fine adjustment. */
#define H_TASK_GRAVITY_CMPS2                    (980.665f)
#define H_TASK_PIPE_ENDPOINT_ANGLE_RAD          (0.174532925f)
#define H_TASK_ACCEL_FF_GAIN                    (1.00f)
#define H_TASK_ACCEL_FF_DIRECTION               (1)
#define H_TASK_ACCEL_FF_MIN_ACCEL_CMPS2          (1.0f)
#define H_TASK_ACCEL_FF_MAX_ACCEL_CMPS2          (25.0f)
#define H_TASK_ACCEL_FF_MAX_ANGLE_RAD            (0.026179939f)
#define H_TASK_ACCEL_FF_MIN_CAL_ROOM_COUNT       (200.0f)
#define H_TASK_ACCEL_FF_MAX_CAL_ROOM_COUNT       (600.0f)
#define H_TASK_ACCEL_FF_MAX_COUNT                (45)
#define H_TASK_H3_TRANSFER_MIN_COUNT            (90U)
#define H_TASK_H3_TRANSFER_MAX_COUNT            (90U)
#define H_TASK_H3_TRANSFER_EXIT_ERROR_MM       (20U)
#define H_TASK_H3_TRANSFER_MAX_BALL_SPEED_MM_S (35U)
#define H_TASK_START_LINK_MAX_AGE_MS        (500U)
#define H_TASK_START_VERIFY_TIMEOUT_MS      (2000U)
#define H_TASK_BALL_ACTIVE_TIMEOUT_MS       (2000U)
#define H_TASK_START_CONFIRM_FRAMES         (3U)
#define H_TASK_START_CENTER_TOLERANCE_MM    (20)
#define H_TASK_START_MAX_SPEED_MM_S         \
    BALANCE_BALL_START_MAX_SPEED_MM_S
#define H_TASK_LOCK_SPREAD_MM               (10)
#define H_TASK_H3_POSITIVE_SPEED_MM_S       (100)
#define H_TASK_H3_POSITIVE_CONFIRM_FRAMES   (3U)
#define H_TASK_H3_FINAL_SPEED_MM_S          (80)
#define H_TASK_H3_FINAL_CONFIRM_FRAMES      (5U)
#define H_TASK_H3_SCORE_TIME_MS             (5000U)
#define H_TASK_H3_SAFETY_TIMEOUT_MS         (10000U)
#define H_TASK_H4_SCORE_TIME_MS             (8000U)
#define H_TASK_FULL_TRACK_SCORE_TIME_MS     (30000U)
#define H_TASK_B_ESTIMATE_DISTANCE_CM       (150.0f)
#define H_TASK_VISION_HOLD_TIMEOUT_MS       (3000U)
#define H_TASK_RECOVERY_CONFIRM_FRAMES      (3U)
#define H_TASK_RECOVERY_ERROR_MM            (25)
#define H_TASK_LOW_ENTER_ERROR_MM           (30)
#define H_TASK_LOW_EXIT_ERROR_MM            (15)
#define H_TASK_STOP_ENTER_ERROR_MM          (100)
#define H_TASK_LOW_ENTER_MS                 (100U)
#define H_TASK_LOW_EXIT_MS                  (300U)
#define H_TASK_STOP_ENTER_MS                (200U)

typedef enum {
    H_MEASUREMENT_NONE = 0,
    H_MEASUREMENT_VALID,
    H_MEASUREMENT_INVALID
} HMeasurementEvent;

static HTaskRuntime g_runtime;
static HTaskChassisTuning g_chassisTuning;
static uint32_t g_lastObservationUpdateCount;
static uint32_t g_lowConditionMs;
static uint32_t g_normalConditionMs;
static uint32_t g_stopConditionMs;
static uint8_t g_positiveStableCount;
static uint8_t g_finalStableCount;
static int16_t g_verifyPositions[H_TASK_START_CONFIRM_FRAMES];
static HTaskState g_resumeState;
static float g_startCenterDistanceCm;
static float g_savedWheelMaxAccelCmps2;
static float g_savedWheelMaxDecelCmps2;
static uint8_t g_wheelRampOverrideActive;
static uint8_t g_vehicleLaunchStage;
static uint32_t g_vehicleLaunchStableMs;
static uint32_t g_vehicleLaunchRampRemainder;
static int16_t g_vehicleLaunchCommand;

static void h_load_default_chassis_tuning(void)
{
    g_chassisTuning.normal_command = H_TASK_LINE_NORMAL_COMMAND;
    g_chassisTuning.feedforward_x100 =
        (int16_t)(H_TASK_LINE_FF_GAIN * 100.0f + 0.5f);
    g_chassisTuning.line_kp_x100 =
        (int16_t)(H_TASK_LINE_KP * 100.0f + 0.5f);
    g_chassisTuning.line_kd_x1000 =
        (int16_t)(H_TASK_LINE_KD * 1000.0f + 0.5f);
    g_chassisTuning.max_correction = H_TASK_LINE_MAX_CORRECTION;
}

static int16_t h_low_line_command(void)
{
    return (g_chassisTuning.normal_command < H_TASK_LINE_LOW_COMMAND) ?
        g_chassisTuning.normal_command : H_TASK_LINE_LOW_COMMAND;
}

static int32_t h_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float h_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t h_acceleration_feedforward_count(
    float acceleration_cmps2)
{
    BalanceSoftLimitsRuntime limits;
    float thetaRad;
    float availableCount;
    float requestedCount;

    BalanceSoftLimits_GetSnapshot(&limits);
    if ((limits.zero_valid == 0U) || (limits.limits_valid == 0U) ||
        (H_TASK_PIPE_ENDPOINT_ANGLE_RAD <= 0.0f) ||
        (acceleration_cmps2 < H_TASK_ACCEL_FF_MIN_ACCEL_CMPS2) ||
        (acceleration_cmps2 > H_TASK_ACCEL_FF_MAX_ACCEL_CMPS2)) {
        return 0;
    }

    thetaRad = -atanf(acceleration_cmps2 / H_TASK_GRAVITY_CMPS2);
    thetaRad *= H_TASK_ACCEL_FF_GAIN *
        (float)H_TASK_ACCEL_FF_DIRECTION;
    if (thetaRad > H_TASK_ACCEL_FF_MAX_ANGLE_RAD) {
        thetaRad = H_TASK_ACCEL_FF_MAX_ANGLE_RAD;
    } else if (thetaRad < -H_TASK_ACCEL_FF_MAX_ANGLE_RAD) {
        thetaRad = -H_TASK_ACCEL_FF_MAX_ANGLE_RAD;
    }
    availableCount = (thetaRad >= 0.0f) ?
        (float)limits.maximum_logical_count :
        (float)(-limits.minimum_logical_count);
    if ((availableCount < H_TASK_ACCEL_FF_MIN_CAL_ROOM_COUNT) ||
        (availableCount > H_TASK_ACCEL_FF_MAX_CAL_ROOM_COUNT)) {
        return 0;
    }
    requestedCount = thetaRad * availableCount /
        H_TASK_PIPE_ENDPOINT_ANGLE_RAD;
    if (requestedCount > H_TASK_ACCEL_FF_MAX_COUNT) {
        requestedCount = H_TASK_ACCEL_FF_MAX_COUNT;
    } else if (requestedCount < -H_TASK_ACCEL_FF_MAX_COUNT) {
        requestedCount = -H_TASK_ACCEL_FF_MAX_COUNT;
    }
    return (int16_t)((requestedCount >= 0.0f) ?
        (requestedCount + 0.5f) : (requestedCount - 0.5f));
}

static uint32_t h_add_u32(uint32_t value, uint32_t increment)
{
    return (value > UINT32_MAX - increment) ?
        UINT32_MAX : value + increment;
}

static int16_t h_median3(int16_t a, int16_t b, int16_t c)
{
    if (a > b) {
        int16_t swap = a;
        a = b;
        b = swap;
    }
    if (b > c) {
        int16_t swap = b;
        b = c;
        c = swap;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

static uint8_t h_is_car_task(HTaskId task_id)
{
    return (task_id == H_TASK_ID_H4) ||
        (task_id == H_TASK_ID_H5) ||
        (task_id == H_TASK_ID_H6);
}

static void h_set_state(HTaskState state)
{
    g_runtime.state = state;
    g_runtime.stage_elapsed_ms = 0U;
}

static void h_stop_car(void)
{
    CarController_Stop();
    g_runtime.vehicle_level = H_TASK_VEHICLE_STOP;
}

static uint8_t h_apply_chassis_profile(void)
{
    float lineKp = (float)g_chassisTuning.line_kp_x100 / 100.0f;
    float lineKd = (float)g_chassisTuning.line_kd_x1000 / 1000.0f;
    float feedforward =
        (float)g_chassisTuning.feedforward_x100 / 100.0f;

    if (LineController_SetProfileOverride(lineKp, lineKd,
            g_chassisTuning.max_correction) == 0U) {
        return 0U;
    }
    if (MotorControl_SetFeedforwardOverride(feedforward,
            feedforward) == 0U) {
        LineController_ClearProfileOverride();
        return 0U;
    }
    if (g_wheelRampOverrideActive == 0U) {
        g_savedWheelMaxAccelCmps2 =
            g_appConfig.wheel_control_max_accel_cmps2;
        g_savedWheelMaxDecelCmps2 =
            g_appConfig.wheel_control_max_decel_cmps2;
        g_wheelRampOverrideActive = 1U;
    }
    g_appConfig.wheel_control_max_accel_cmps2 =
        H_TASK_WHEEL_LAUNCH_ACCEL_CMPS2;
    g_appConfig.wheel_control_max_decel_cmps2 =
        H_TASK_WHEEL_MAX_DECEL_CMPS2;
    LineController_ResetControlState();
    MotorControl_Reset();
    return 1U;
}

static void h_release_chassis_profile(void)
{
    (void)BalanceBallControl_SetTransferAssist(0U, 0U, 0U, 0U, 0U);
    BalanceBallControl_SetAngleFeedforwardCount(0);
    g_vehicleLaunchStage = 0U;
    g_vehicleLaunchStableMs = 0U;
    g_vehicleLaunchRampRemainder = 0U;
    g_vehicleLaunchCommand = 0;
    if (g_wheelRampOverrideActive != 0U) {
        g_appConfig.wheel_control_max_accel_cmps2 =
            g_savedWheelMaxAccelCmps2;
        g_appConfig.wheel_control_max_decel_cmps2 =
            g_savedWheelMaxDecelCmps2;
        g_wheelRampOverrideActive = 0U;
    }
    LineController_ClearProfileOverride();
    MotorControl_ClearFeedforwardOverride();
    LineController_ResetControlState();
    MotorControl_Reset();
}

static void h_fault(HTaskFault fault)
{
    h_stop_car();
    h_release_chassis_profile();
    BalanceBallControl_ForceStop();
    g_runtime.fault = fault;
    h_set_state(H_TASK_STATE_FAULT);
}

static uint8_t h_task_id_valid(HTaskId task_id)
{
    return (task_id >= H_TASK_ID_H3) && (task_id <= H_TASK_ID_H6);
}

static uint8_t h_axis_ready(void)
{
    BalanceSoftLimitsRuntime limits;

    BalanceSoftLimits_GetSnapshot(&limits);
    return (limits.zero_valid != 0U) &&
        (limits.limits_valid != 0U) &&
        (limits.calibration_active == 0U) &&
        (limits.recalibration_armed == 0U) &&
        (limits.test_active == 0U) &&
        (limits.oscillation_active == 0U) &&
        (BalancePositionControl_IsBusy() == 0U) &&
        (BalancePositionControl_HasFault() == 0U);
}

static uint8_t h_vision_link_recent(uint32_t now_ms)
{
    const VisionBallPositionObservation *observation =
        VisionReceiver_GetBallPositionObservation();

    return (observation !=
            (const VisionBallPositionObservation *)0) &&
        (observation->available != 0U) &&
        ((now_ms - observation->local_receive_timestamp_ms) <=
            H_TASK_START_LINK_MAX_AGE_MS);
}

static HMeasurementEvent h_get_new_measurement(int16_t *position_mm)
{
    const VisionBallPositionObservation *observation =
        VisionReceiver_GetBallPositionObservation();

    if ((observation == (const VisionBallPositionObservation *)0) ||
        (observation->update_count == g_lastObservationUpdateCount)) {
        return H_MEASUREMENT_NONE;
    }

    g_lastObservationUpdateCount = observation->update_count;
    if ((observation->available == 0U) ||
        (observation->target_valid == 0U) ||
        (observation->measured == 0U)) {
        return H_MEASUREMENT_INVALID;
    }

    if (position_mm != (int16_t *)0) {
        *position_mm = observation->position_mm;
    }
    return H_MEASUREMENT_VALID;
}

static void h_start_car(void)
{
    const volatile WheelSpeedEstimatorRuntime *wheel =
        WheelSpeedEstimator_GetRuntime();

    if ((wheel == (const volatile WheelSpeedEstimatorRuntime *)0) ||
        !wheel->valid || wheel->stale || wheel->overflow) {
        h_fault(H_TASK_FAULT_CAR_CONTROL);
        return;
    }

    g_startCenterDistanceCm = wheel->center_distance_cm;
    g_vehicleLaunchStage = 1U;
    g_vehicleLaunchStableMs = 0U;
    g_vehicleLaunchRampRemainder = 0U;
    g_vehicleLaunchCommand = H_TASK_LINE_START_COMMAND;
    BalanceBallControl_SetAngleFeedforwardCount(0);
    CarController_StartFollowLineAtCommand(CAR_TURN_POLICY_IGNORE,
        g_vehicleLaunchCommand);
    g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
    h_set_state(H_TASK_STATE_CAR_RUNNING);
}

static void h_mark_done(void)
{
    (void)BalanceBallControl_SetTransferAssist(0U, 0U, 0U, 0U, 0U);
    g_runtime.finish_latched = 1U;
    g_runtime.completion_time_ms = g_runtime.task_elapsed_ms;
    h_set_state(H_TASK_STATE_DONE);
}

static void h_enter_vision_hold(void)
{
    if (g_runtime.state == H_TASK_STATE_VISION_HOLD) {
        return;
    }
    g_resumeState = g_runtime.state;
    g_runtime.recovery_count = 0U;
    if (h_is_car_task(g_runtime.task_id) != 0U) {
        h_stop_car();
    }
    h_set_state(H_TASK_STATE_VISION_HOLD);
}

static void h_update_start_verify(HMeasurementEvent event,
    int16_t position_mm, const BalanceBallControlRuntime *ball)
{
    if (event == H_MEASUREMENT_INVALID) {
        g_runtime.valid_confirm_count = 0U;
        return;
    }
    if (event != H_MEASUREMENT_VALID) {
        return;
    }
    if (h_abs_i32(ball->velocity_mm_s) >
        H_TASK_START_MAX_SPEED_MM_S) {
        g_runtime.valid_confirm_count = 0U;
        return;
    }

    if (g_runtime.task_id == H_TASK_ID_H6) {
        int16_t minimum;
        int16_t maximum;

        if (g_runtime.valid_confirm_count == 0U) {
            /* Hold the first admitted post-start real position with the
             * controller's weak authority while the remaining lock frames
             * are collected. This target is only temporary. */
            (void)BalanceBallControl_SetTargetMm(position_mm);
        }
        g_verifyPositions[g_runtime.valid_confirm_count] = position_mm;
        g_runtime.valid_confirm_count++;
        minimum = g_verifyPositions[0];
        maximum = g_verifyPositions[0];
        for (uint8_t i = 1U; i < g_runtime.valid_confirm_count; i++) {
            if (g_verifyPositions[i] < minimum) {
                minimum = g_verifyPositions[i];
            }
            if (g_verifyPositions[i] > maximum) {
                maximum = g_verifyPositions[i];
            }
        }
        if ((maximum - minimum) > H_TASK_LOCK_SPREAD_MM) {
            g_verifyPositions[0] = position_mm;
            g_runtime.valid_confirm_count = 1U;
            return;
        }
    } else {
        if (h_abs_i32((int32_t)position_mm -
                H_TASK_CENTER_TARGET_MM) >
            H_TASK_START_CENTER_TOLERANCE_MM) {
            g_runtime.valid_confirm_count = 0U;
            return;
        }
        g_runtime.valid_confirm_count++;
    }

    if (g_runtime.valid_confirm_count < H_TASK_START_CONFIRM_FRAMES) {
        return;
    }

    if (g_runtime.task_id == H_TASK_ID_H3) {
        g_runtime.target_mm = H_TASK_H3_FIRST_DRIVE_TARGET_MM;
        if (BalanceBallControl_SetTargetMm(g_runtime.target_mm) == 0U) {
            h_fault(H_TASK_FAULT_TARGET_INVALID);
            return;
        }
        if (BalanceBallControl_SetTransferAssist(1U,
                H_TASK_H3_TRANSFER_MIN_COUNT,
                H_TASK_H3_TRANSFER_MAX_COUNT,
                H_TASK_H3_TRANSFER_EXIT_ERROR_MM,
                H_TASK_H3_TRANSFER_MAX_BALL_SPEED_MM_S) == 0U) {
            h_fault(H_TASK_FAULT_BALL_CONTROL);
            return;
        }
        h_set_state(H_TASK_STATE_BALL_POSITIVE);
    } else if (g_runtime.task_id == H_TASK_ID_H6) {
        g_runtime.target_lock_mm = h_median3(g_verifyPositions[0],
            g_verifyPositions[1], g_verifyPositions[2]);
        g_runtime.target_lock_valid = 1U;
        g_runtime.target_mm = g_runtime.target_lock_mm;
        if (BalanceBallControl_SetTargetMm(g_runtime.target_mm) == 0U) {
            h_fault(H_TASK_FAULT_TARGET_INVALID);
            return;
        }
        h_set_state(H_TASK_STATE_WAIT_BALL_ACTIVE);
    } else {
        g_runtime.target_mm = H_TASK_CENTER_TARGET_MM;
        h_set_state(H_TASK_STATE_WAIT_BALL_ACTIVE);
    }
}

static void h_update_h3(HMeasurementEvent event, int16_t position_mm,
    const BalanceBallControlRuntime *ball)
{
    if (g_runtime.state == H_TASK_STATE_BALL_POSITIVE) {
        if ((event == H_MEASUREMENT_VALID) &&
            (position_mm <= H_TASK_H3_FIRST_REACH_MM) &&
            (h_abs_i32(ball->velocity_mm_s) <=
                H_TASK_H3_POSITIVE_SPEED_MM_S)) {
            if (g_positiveStableCount < UINT8_MAX) {
                g_positiveStableCount++;
            }
        } else if (event != H_MEASUREMENT_NONE) {
            g_positiveStableCount = 0U;
        }
        if (g_positiveStableCount >=
            H_TASK_H3_POSITIVE_CONFIRM_FRAMES) {
            g_runtime.target_mm = H_TASK_H3_FINAL_DRIVE_TARGET_MM;
            if (BalanceBallControl_SetTargetMm(g_runtime.target_mm) == 0U) {
                h_fault(H_TASK_FAULT_TARGET_INVALID);
                return;
            }
            g_positiveStableCount = 0U;
            g_finalStableCount = 0U;
            h_set_state(H_TASK_STATE_BALL_NEGATIVE);
        }
    } else if (g_runtime.state == H_TASK_STATE_BALL_NEGATIVE) {
        if ((event == H_MEASUREMENT_VALID) &&
            (position_mm >= H_TASK_H3_FINAL_REACH_MM) &&
            (h_abs_i32(ball->velocity_mm_s) <=
                H_TASK_H3_FINAL_SPEED_MM_S)) {
            if (g_finalStableCount < UINT8_MAX) {
                g_finalStableCount++;
            }
        } else if (event != H_MEASUREMENT_NONE) {
            g_finalStableCount = 0U;
        }

        if (g_finalStableCount >= H_TASK_H3_FINAL_CONFIRM_FRAMES) {
            /* Finish against the exact scoring position instead of keeping
             * the 10 mm transfer overdrive after the timer is latched. */
            g_runtime.target_mm = H_TASK_H3_FINAL_HOLD_TARGET_MM;
            if (BalanceBallControl_SetTargetMm(g_runtime.target_mm) == 0U) {
                h_fault(H_TASK_FAULT_TARGET_INVALID);
                return;
            }
            h_mark_done();
        }
    }
}

static void h_update_vehicle_supervision(uint32_t elapsed_ms,
    uint8_t vision_degraded, uint8_t ball_ready)
{
    int32_t errorMagnitude = h_abs_i32(g_runtime.error_mm);

    if (g_vehicleLaunchStage != 0U) {
        uint32_t rampBudget = g_vehicleLaunchRampRemainder +
            H_TASK_LAUNCH_COMMAND_RATE_PER_S * elapsed_ms;
        int16_t commandDelta = (int16_t)(rampBudget / 1000U);
        uint8_t accelerationCommanded = 0U;

        g_vehicleLaunchRampRemainder = rampBudget % 1000U;
        if ((vision_degraded != 0U) || (ball_ready == 0U) ||
            (errorMagnitude > H_TASK_LAUNCH_RESET_ERROR_MM)) {
            g_vehicleLaunchStableMs = 0U;
            g_vehicleLaunchStage = 1U;
            if ((commandDelta > 0) &&
                (g_vehicleLaunchCommand > H_TASK_LINE_START_COMMAND)) {
                int16_t nextCommand = g_vehicleLaunchCommand - commandDelta;

                if (nextCommand < H_TASK_LINE_START_COMMAND) {
                    nextCommand = H_TASK_LINE_START_COMMAND;
                }
                if (!CarController_SetFollowLineBaseCommand(
                        nextCommand)) {
                    h_fault(H_TASK_FAULT_CAR_CONTROL);
                    return;
                }
                g_vehicleLaunchCommand = nextCommand;
            }
            g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
            BalanceBallControl_SetAngleFeedforwardCount(0);
            return;
        }

        if (g_vehicleLaunchStage == 1U) {
            if ((g_vehicleLaunchCommand == H_TASK_LINE_START_COMMAND) &&
                (errorMagnitude <= H_TASK_LAUNCH_STABLE_ERROR_MM)) {
                g_vehicleLaunchStableMs = h_add_u32(
                    g_vehicleLaunchStableMs, elapsed_ms);
            } else {
                g_vehicleLaunchStableMs = 0U;
            }
            if (g_vehicleLaunchStableMs >= H_TASK_LAUNCH_SETTLE_MS) {
                g_vehicleLaunchStage = 2U;
                g_vehicleLaunchStableMs = 0U;
                g_vehicleLaunchRampRemainder = 0U;
            }
            BalanceBallControl_SetAngleFeedforwardCount(0);
            return;
        }

        /* Stage 2 is a continuous speed-target ramp.  The wheel PI loop owns
         * speed regulation while the ball error gates whether the ramp may
         * advance.  Constant positive target acceleration receives a
         * theta_ff=-atan(a/g) physical tilt feedforward. */
        if (errorMagnitude <= H_TASK_LAUNCH_STABLE_ERROR_MM) {
            if ((commandDelta > 0) &&
                (g_vehicleLaunchCommand <
                    g_chassisTuning.normal_command)) {
                int16_t nextCommand = g_vehicleLaunchCommand + commandDelta;

                if (nextCommand > g_chassisTuning.normal_command) {
                    nextCommand = g_chassisTuning.normal_command;
                }
                if (!CarController_SetFollowLineBaseCommand(nextCommand)) {
                    h_fault(H_TASK_FAULT_CAR_CONTROL);
                    return;
                }
                g_vehicleLaunchCommand = nextCommand;
                accelerationCommanded = 1U;
            }
            if (accelerationCommanded != 0U) {
                BalanceBallControl_SetAngleFeedforwardCount(
                    h_acceleration_feedforward_count(
                        H_TASK_WHEEL_LAUNCH_ACCEL_CMPS2));
            } else {
                BalanceBallControl_SetAngleFeedforwardCount(0);
            }
        } else {
            BalanceBallControl_SetAngleFeedforwardCount(0);
        }

        if (g_vehicleLaunchCommand >= g_chassisTuning.normal_command) {
            g_vehicleLaunchStage = 0U;
            g_vehicleLaunchStableMs = 0U;
            g_vehicleLaunchRampRemainder = 0U;
            g_runtime.vehicle_level = H_TASK_VEHICLE_NORMAL;
            g_appConfig.wheel_control_max_accel_cmps2 =
                H_TASK_WHEEL_MAX_ACCEL_CMPS2;
            BalanceBallControl_SetAngleFeedforwardCount(0);
        }
        return;
    }

    if (errorMagnitude >= H_TASK_STOP_ENTER_ERROR_MM) {
        g_stopConditionMs = h_add_u32(g_stopConditionMs, elapsed_ms);
    } else {
        g_stopConditionMs = 0U;
    }
    if (g_stopConditionMs >= H_TASK_STOP_ENTER_MS) {
        /* A large but still measured ball displacement is not a lost-vision
         * event.  An immediate chassis stop throws the ball even farther.
         * Stay on the line at LOW and let the encoder speed loop decelerate
         * with the H-task ramp.  Real VISION_LOST and hard faults still stop. */
        g_lowConditionMs = 0U;
        g_normalConditionMs = 0U;
        if ((g_runtime.vehicle_level == H_TASK_VEHICLE_NORMAL) &&
            CarController_SetFollowLineBaseCommand(
                h_low_line_command())) {
            g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
        }
        g_stopConditionMs = H_TASK_STOP_ENTER_MS;
        return;
    }

    if (vision_degraded != 0U) {
        g_lowConditionMs = 0U;
        g_normalConditionMs = 0U;
        if ((g_runtime.vehicle_level == H_TASK_VEHICLE_NORMAL) &&
            CarController_SetFollowLineBaseCommand(
                h_low_line_command())) {
            g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
        }
        return;
    }

    if (errorMagnitude >= H_TASK_LOW_ENTER_ERROR_MM) {
        g_lowConditionMs = h_add_u32(g_lowConditionMs, elapsed_ms);
        g_normalConditionMs = 0U;
    } else if (errorMagnitude <= H_TASK_LOW_EXIT_ERROR_MM) {
        g_normalConditionMs = h_add_u32(g_normalConditionMs, elapsed_ms);
        g_lowConditionMs = 0U;
    } else {
        g_lowConditionMs = 0U;
        g_normalConditionMs = 0U;
    }

    if ((g_runtime.vehicle_level == H_TASK_VEHICLE_NORMAL) &&
        (g_lowConditionMs >= H_TASK_LOW_ENTER_MS)) {
        if (CarController_SetFollowLineBaseCommand(
                h_low_line_command())) {
            g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
        }
        g_lowConditionMs = 0U;
    } else if ((g_runtime.vehicle_level == H_TASK_VEHICLE_LOW) &&
        (g_normalConditionMs >= H_TASK_LOW_EXIT_MS)) {
        if (CarController_SetFollowLineBaseCommand(
                g_chassisTuning.normal_command)) {
            g_runtime.vehicle_level = H_TASK_VEHICLE_NORMAL;
        }
        g_normalConditionMs = 0U;
    }
}

static void h_update_vision_hold(HMeasurementEvent event)
{
    BalanceBallControlRuntime ball;

    BalanceBallControl_GetSnapshot(&ball);
    if (g_runtime.stage_elapsed_ms >= H_TASK_VISION_HOLD_TIMEOUT_MS) {
        h_fault(H_TASK_FAULT_VISION_NOT_READY);
        return;
    }
    if ((event == H_MEASUREMENT_VALID) &&
        (ball.state == BALANCE_BALL_STATE_ACTIVE) &&
        (ball.measurement_valid != 0U) &&
        (h_abs_i32(ball.error_mm) <= H_TASK_RECOVERY_ERROR_MM)) {
        if (g_runtime.recovery_count < UINT8_MAX) {
            g_runtime.recovery_count++;
        }
    } else if (event != H_MEASUREMENT_NONE) {
        g_runtime.recovery_count = 0U;
    }

    if (g_runtime.recovery_count < H_TASK_RECOVERY_CONFIRM_FRAMES) {
        return;
    }

    if (h_is_car_task(g_runtime.task_id) != 0U) {
        g_appConfig.wheel_control_max_accel_cmps2 =
            H_TASK_WHEEL_LAUNCH_ACCEL_CMPS2;
        CarController_StartFollowLineAtCommand(CAR_TURN_POLICY_IGNORE,
            H_TASK_LINE_START_COMMAND);
        g_vehicleLaunchStage = 1U;
        g_vehicleLaunchStableMs = 0U;
        g_vehicleLaunchRampRemainder = 0U;
        g_vehicleLaunchCommand = H_TASK_LINE_START_COMMAND;
        BalanceBallControl_SetAngleFeedforwardCount(0);
        g_runtime.vehicle_level = H_TASK_VEHICLE_LOW;
    }
    h_set_state(g_resumeState);
}

void HTaskController_Init(void)
{
    h_load_default_chassis_tuning();
    HTaskController_Reset();
}

uint8_t HTaskController_Start(HTaskId task_id)
{
    const VisionBallPositionObservation *observation =
        VisionReceiver_GetBallPositionObservation();
    uint32_t nowMs = SystemTime_GetMs();

    HTaskController_Reset();
    g_runtime.task_id = task_id;
    if (h_task_id_valid(task_id) == 0U) {
        h_fault(H_TASK_FAULT_INVALID_TASK);
        return 0U;
    }
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        h_fault(H_TASK_FAULT_ESTOP);
        return 0U;
    }
    if (h_axis_ready() == 0U) {
        h_fault(H_TASK_FAULT_AXIS_NOT_READY);
        return 0U;
    }
    if (h_vision_link_recent(nowMs) == 0U) {
        h_fault(H_TASK_FAULT_VISION_NOT_READY);
        return 0U;
    }
    if ((h_is_car_task(task_id) != 0U) &&
        !WheelSpeedEstimator_GetRuntime()->valid) {
        h_fault(H_TASK_FAULT_CAR_CONTROL);
        return 0U;
    }

    BalanceBallControl_ForceStop();
    h_stop_car();
    h_release_chassis_profile();
    g_runtime.task_id = task_id;
    g_runtime.fault = H_TASK_FAULT_NONE;
    g_runtime.start_observation_update_count = observation->update_count;
    g_runtime.start_sequence = observation->sequence;
    g_lastObservationUpdateCount = observation->update_count;
    g_runtime.target_mm = (task_id == H_TASK_ID_H6) ?
        observation->position_mm : H_TASK_CENTER_TARGET_MM;
    if (BalanceBallControl_Enable(g_runtime.target_mm) == 0U) {
        h_fault(H_TASK_FAULT_TARGET_INVALID);
        return 0U;
    }
    if ((h_is_car_task(task_id) != 0U) &&
        (h_apply_chassis_profile() == 0U)) {
        h_fault(H_TASK_FAULT_CAR_CONTROL);
        return 0U;
    }
    h_set_state(H_TASK_STATE_START_VERIFY);
    return 1U;
}

void HTaskController_Update20ms(uint32_t elapsed_ms)
{
    BalanceBallControlRuntime ball;
    HMeasurementEvent event;
    int16_t positionMm = 0;

    if ((g_runtime.state == H_TASK_STATE_IDLE) ||
        (g_runtime.state == H_TASK_STATE_FAULT) ||
        (g_runtime.state == H_TASK_STATE_ABORTED)) {
        return;
    }

    if (g_runtime.finish_latched == 0U) {
        g_runtime.task_elapsed_ms = h_add_u32(
            g_runtime.task_elapsed_ms, elapsed_ms);
    }
    g_runtime.stage_elapsed_ms = h_add_u32(
        g_runtime.stage_elapsed_ms, elapsed_ms);
    event = h_get_new_measurement(&positionMm);
    BalanceBallControl_GetSnapshot(&ball);
    if ((event == H_MEASUREMENT_VALID) &&
        (ball.last_observation_result !=
            BALANCE_BALL_OBSERVATION_ACCEPTED)) {
        /* Stage transitions must use the exact same admitted sample as the
         * cascade loop. A protocol-valid but rejected point cannot complete
         * +50/-50, startup verification, or LOST recovery. */
        event = H_MEASUREMENT_INVALID;
    } else if (event == H_MEASUREMENT_VALID) {
        positionMm = ball.position_mm;
    }
    g_runtime.position_mm = ball.position_mm;
    g_runtime.error_mm = ball.error_mm;

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        h_fault(H_TASK_FAULT_ESTOP);
        return;
    }
    if (BalancePositionControl_HasFault() != 0U) {
        h_fault(H_TASK_FAULT_POSITION_CONTROL);
        return;
    }
    if (ball.state == BALANCE_BALL_STATE_FAULT) {
        h_fault(H_TASK_FAULT_BALL_CONTROL);
        return;
    }
    if ((h_is_car_task(g_runtime.task_id) != 0U) &&
        (CarState_Get() == CAR_STATE_ERROR)) {
        h_fault(H_TASK_FAULT_CAR_CONTROL);
        return;
    }

    if (g_runtime.state == H_TASK_STATE_START_VERIFY) {
        if (g_runtime.stage_elapsed_ms >=
            H_TASK_START_VERIFY_TIMEOUT_MS) {
            h_fault(H_TASK_FAULT_START_TIMEOUT);
            return;
        }
        h_update_start_verify(event, positionMm, &ball);
        return;
    }

    if (g_runtime.state == H_TASK_STATE_WAIT_BALL_ACTIVE) {
        if (ball.state == BALANCE_BALL_STATE_ACTIVE) {
            h_start_car();
        } else if (g_runtime.stage_elapsed_ms >=
            H_TASK_BALL_ACTIVE_TIMEOUT_MS) {
            h_fault(H_TASK_FAULT_VISION_NOT_READY);
        }
        return;
    }

    if (g_runtime.state == H_TASK_STATE_VISION_HOLD) {
        h_update_vision_hold(event);
        return;
    }
    if (ball.state == BALANCE_BALL_STATE_VISION_LOST) {
        h_enter_vision_hold();
        return;
    }
    if (g_runtime.task_id == H_TASK_ID_H3) {
        if (g_runtime.task_elapsed_ms > H_TASK_H3_SCORE_TIME_MS) {
            g_runtime.overtime = 1U;
        }
        if ((g_runtime.finish_latched == 0U) &&
            (g_runtime.task_elapsed_ms >= H_TASK_H3_SAFETY_TIMEOUT_MS)) {
            h_fault(H_TASK_FAULT_TASK_TIMEOUT);
            return;
        }
        h_update_h3(event, positionMm, &ball);
        return;
    }

    if ((g_runtime.state == H_TASK_STATE_CAR_RUNNING) ||
        (g_runtime.state == H_TASK_STATE_DONE)) {
        const volatile WheelSpeedEstimatorRuntime *wheel =
            WheelSpeedEstimator_GetRuntime();

        if ((wheel == (const volatile WheelSpeedEstimatorRuntime *)0) ||
            !wheel->valid || wheel->stale || wheel->overflow) {
            h_fault(H_TASK_FAULT_CAR_CONTROL);
            return;
        }

        h_update_vehicle_supervision(elapsed_ms,
            (ball.state == BALANCE_BALL_STATE_HOLD_LAST) ? 1U : 0U,
            ((ball.state == BALANCE_BALL_STATE_ACTIVE) &&
             (ball.measurement_valid != 0U)) ? 1U : 0U);
        if ((g_runtime.state == H_TASK_STATE_VISION_HOLD) ||
            (g_runtime.state == H_TASK_STATE_FAULT)) {
            return;
        }
        if ((g_runtime.task_id == H_TASK_ID_H4) &&
            (g_runtime.b_passed_estimate == 0U) &&
            (h_abs_float(wheel->center_distance_cm -
                g_startCenterDistanceCm) >=
                H_TASK_B_ESTIMATE_DISTANCE_CM)) {
            g_runtime.b_passed_estimate = 1U;
            g_runtime.completion_time_ms = g_runtime.task_elapsed_ms;
        }
        if ((g_runtime.task_id == H_TASK_ID_H4) &&
            (g_runtime.b_passed_estimate == 0U) &&
            (g_runtime.task_elapsed_ms > H_TASK_H4_SCORE_TIME_MS)) {
            g_runtime.overtime = 1U;
        }
        if ((g_runtime.task_id == H_TASK_ID_H5) ||
            (g_runtime.task_id == H_TASK_ID_H6)) {
            if (g_runtime.task_elapsed_ms >
                H_TASK_FULL_TRACK_SCORE_TIME_MS) {
                g_runtime.overtime = 1U;
            }
        }
    }
}

void HTaskController_RequestNormalStop(void)
{
    h_stop_car();
    h_release_chassis_profile();
    BalanceBallControl_Disable();
    if (g_runtime.state != H_TASK_STATE_IDLE) {
        h_set_state(H_TASK_STATE_ABORTED);
    }
}

void HTaskController_Reset(void)
{
    h_release_chassis_profile();
    g_runtime = (HTaskRuntime){0};
    g_runtime.state = H_TASK_STATE_IDLE;
    g_runtime.vehicle_level = H_TASK_VEHICLE_STOP;
    g_lastObservationUpdateCount = 0U;
    g_lowConditionMs = 0U;
    g_normalConditionMs = 0U;
    g_stopConditionMs = 0U;
    g_positiveStableCount = 0U;
    g_finalStableCount = 0U;
    g_resumeState = H_TASK_STATE_IDLE;
    g_startCenterDistanceCm = 0.0f;
    g_vehicleLaunchStage = 0U;
    g_vehicleLaunchStableMs = 0U;
    g_vehicleLaunchRampRemainder = 0U;
    g_vehicleLaunchCommand = 0;
    for (uint8_t i = 0U; i < H_TASK_START_CONFIRM_FRAMES; i++) {
        g_verifyPositions[i] = 0;
    }
}

uint8_t HTaskController_IsActive(void)
{
    return (g_runtime.state != H_TASK_STATE_IDLE) &&
        (g_runtime.state != H_TASK_STATE_FAULT) &&
        (g_runtime.state != H_TASK_STATE_ABORTED);
}

uint8_t HTaskController_HasFault(void)
{
    return g_runtime.state == H_TASK_STATE_FAULT;
}

void HTaskController_GetSnapshot(HTaskRuntime *snapshot)
{
    if (snapshot != (HTaskRuntime *)0) {
        *snapshot = g_runtime;
    }
}

void HTaskController_GetChassisTuning(HTaskChassisTuning *tuning)
{
    if (tuning != (HTaskChassisTuning *)0) {
        *tuning = g_chassisTuning;
    }
}

uint8_t HTaskController_SetChassisTuning(
    const HTaskChassisTuning *tuning)
{
    if ((tuning == (const HTaskChassisTuning *)0) ||
        (HTaskController_IsActive() != 0U) ||
        (tuning->normal_command < 100) ||
        (tuning->normal_command > 500) ||
        (tuning->feedforward_x100 < 50) ||
        (tuning->feedforward_x100 > 80) ||
        (tuning->line_kp_x100 < 0) ||
        (tuning->line_kp_x100 > 200) ||
        (tuning->line_kd_x1000 < 0) ||
        (tuning->line_kd_x1000 > 100) ||
        (tuning->max_correction < 0) ||
        (tuning->max_correction > 1000)) {
        return 0U;
    }
    g_chassisTuning = *tuning;
    return 1U;
}

const char *HTaskController_StateToString(HTaskState state)
{
    switch (state) {
        case H_TASK_STATE_START_VERIFY: return "VERIFY";
        case H_TASK_STATE_WAIT_BALL_ACTIVE: return "B-WAIT";
        case H_TASK_STATE_BALL_POSITIVE: return "+50";
        case H_TASK_STATE_BALL_NEGATIVE: return "-50";
        case H_TASK_STATE_CAR_RUNNING: return "RUN";
        case H_TASK_STATE_VISION_HOLD: return "HOLD";
        case H_TASK_STATE_DONE: return "DONE";
        case H_TASK_STATE_FAULT: return "FAULT";
        case H_TASK_STATE_ABORTED: return "STOP";
        case H_TASK_STATE_IDLE:
        default: return "IDLE";
    }
}

const char *HTaskController_VehicleToString(HTaskVehicleLevel level)
{
    switch (level) {
        case H_TASK_VEHICLE_NORMAL: return "N";
        case H_TASK_VEHICLE_LOW: return "L";
        case H_TASK_VEHICLE_STOP:
        default: return "S";
    }
}
