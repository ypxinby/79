#include "balance_ball_control.h"

#include <limits.h>

#include "app_features.h"
#include "balance_position_control.h"
#include "balance_soft_limits.h"
#include "emergency_stop.h"
#include "vision_receiver.h"
#include "watchdog_monitor.h"

#if FEATURE_BALANCE_BALL_PD_CONTROL

static BalanceBallControlRuntime g_runtime;
static BalanceBallControlConfig g_config;
static uint32_t g_lastObservationUpdateCount;
static uint32_t g_lastSessionId;
static int16_t g_lastPositionMm;
static uint8_t g_positionInitialized;
static uint8_t g_hadMeasurementGap;
static uint8_t g_jumpCandidateActive;
static uint8_t g_jumpCandidateCount;
static int16_t g_jumpCandidatePositionMm;
static uint32_t g_jumpCandidateTimeMs;

static void load_default_config(void)
{
    g_config.kp_x100 =
        BALANCE_BALL_PD_KP_COUNTS_PER_MM_X100;
    g_config.kd_x100 =
        BALANCE_BALL_PD_KD_COUNTS_PER_MM_S_X100;
    g_config.maximum_target_velocity_mm_s =
        BALANCE_BALL_MAX_TARGET_VELOCITY_MM_S;
    g_config.neutral_bias_count =
        BALANCE_BALL_NEUTRAL_BIAS_COUNT;
    g_config.positive_brake_accel_mm_s2 =
        BALANCE_BALL_BRAKE_ACCEL_POS_MM_S2;
    g_config.negative_brake_accel_mm_s2 =
        BALANCE_BALL_BRAKE_ACCEL_NEG_MM_S2;
    g_config.brake_delay_ms = BALANCE_BALL_BRAKE_DELAY_MS;
    g_config.brake_margin_mm = BALANCE_BALL_BRAKE_MARGIN_MM;
    g_config.approach_velocity_mm_s =
        BALANCE_BALL_APPROACH_VELOCITY_MM_S;
    g_config.maximum_offset_count =
        BALANCE_BALL_PD_MAX_OFFSET_COUNTS;
    g_config.target_slew_count_per_20ms =
        BALANCE_BALL_PD_TARGET_SLEW_COUNTS_PER_20MS;
    g_config.tracking_deadband_count =
        BALANCE_POSITION_TRACKING_DEADBAND_COUNTS;
    g_config.tracking_reengage_count =
        BALANCE_POSITION_TRACKING_REENGAGE_COUNTS;
    g_config.tilt_sign = BALANCE_BALL_PD_TILT_SIGN;
}

static uint8_t config_is_valid(
    const BalanceBallControlConfig *config)
{
    return ((config != (const BalanceBallControlConfig *)0) &&
        (config->kp_x100 >= 0) && (config->kp_x100 <= 2000) &&
        (config->kd_x100 >= 0) && (config->kd_x100 <= 500) &&
        (config->maximum_target_velocity_mm_s >= 10) &&
        (config->maximum_target_velocity_mm_s <= 1000) &&
        (config->neutral_bias_count >= -256) &&
        (config->neutral_bias_count <= 256) &&
        (config->positive_brake_accel_mm_s2 >= 50U) &&
        (config->positive_brake_accel_mm_s2 <= 5000U) &&
        (config->negative_brake_accel_mm_s2 >= 50U) &&
        (config->negative_brake_accel_mm_s2 <= 5000U) &&
        (config->brake_delay_ms <= 500U) &&
        (config->brake_margin_mm <= 50U) &&
        (config->approach_velocity_mm_s > 0U) &&
        (config->approach_velocity_mm_s <=
            (uint16_t)config->maximum_target_velocity_mm_s) &&
        (config->maximum_offset_count >= 8) &&
        (config->maximum_offset_count <= 1024) &&
        (config->maximum_offset_count >=
            config->tracking_reengage_count) &&
        (config->target_slew_count_per_20ms >= 1) &&
        (config->target_slew_count_per_20ms <= 128) &&
        (config->tracking_deadband_count <= 32U) &&
        (config->tracking_reengage_count >
            config->tracking_deadband_count) &&
        (config->tracking_reengage_count <= 64U) &&
        ((config->tilt_sign == 1) ||
            (config->tilt_sign == -1))) ? 1U : 0U;
}

static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static int16_t clamp_i32_to_i16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static uint32_t magnitude_i32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }
    return (uint32_t)(-(value + 1)) + 1U;
}

static int32_t clamp_symmetric(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int32_t slew_toward(int32_t current, int32_t target,
    int32_t maximum_delta)
{
    int64_t delta = (int64_t)target - current;

    if (delta > maximum_delta) {
        return current + maximum_delta;
    }
    if (delta < -maximum_delta) {
        return current - maximum_delta;
    }
    return target;
}

static uint8_t axis_is_ready(const BalanceSoftLimitsRuntime *limits,
    const BalancePositionRuntime *position)
{
    return ((limits->zero_valid != 0U) &&
        (limits->limits_valid != 0U) &&
        (limits->calibration_active == 0U) &&
        (limits->recalibration_armed == 0U) &&
        (limits->test_active == 0U) &&
        (limits->oscillation_active == 0U) &&
        (position->fault == BALANCE_POSITION_FAULT_NONE)) ? 1U : 0U;
}

static int32_t maximum_offset_count(
    const BalanceSoftLimitsRuntime *limits)
{
    uint32_t negative_room = magnitude_i32(
        limits->minimum_logical_count);
    uint32_t positive_room = magnitude_i32(
        limits->maximum_logical_count);
    uint32_t room = (negative_room < positive_room) ?
        negative_room : positive_room;
    uint32_t safe_room = (room > BALANCE_POSITION_LIMIT_MARGIN_COUNTS) ?
        (room - BALANCE_POSITION_LIMIT_MARGIN_COUNTS) : 0U;
    uint32_t offset = (uint32_t)g_config.maximum_offset_count;

    /* MAX is the operator-visible control-authority limit.  The calibrated
     * software limits and their safety margin remain the final hard clamp;
     * there is no additional hidden percentage cap. */
    if (offset > safe_room) {
        offset = safe_room;
    }
    return (int32_t)offset;
}

static void clear_jump_candidate(void)
{
    g_jumpCandidateActive = 0U;
    g_jumpCandidateCount = 0U;
    g_jumpCandidatePositionMm = 0;
    g_jumpCandidateTimeMs = 0U;
}

static void reset_measurement_history(void)
{
    g_runtime.raw_vision_valid = 0U;
    g_runtime.measurement_valid = 0U;
    g_runtime.valid_streak = 0U;
    g_runtime.velocity_mm_s = 0;
    g_runtime.raw_velocity_mm_s = 0;
    g_runtime.velocity_sample_dt_ms = 0U;
    g_runtime.velocity_filter_alpha_x1000 = 0U;
    g_runtime.velocity_reversal_fast = 0U;
    g_runtime.last_measurement_time_ms = 0U;
    g_runtime.startup_discard_remaining =
        BALANCE_BALL_START_DISCARD_FRAMES;
    g_positionInitialized = 0U;
    g_hadMeasurementGap = 1U;
    clear_jump_candidate();
}

static void process_invalid_observation(
    BalanceBallObservationResult result)
{
    g_runtime.raw_vision_valid = 0U;
    g_runtime.last_observation_result = result;
    g_hadMeasurementGap = 1U;
    clear_jump_candidate();
    if (result == BALANCE_BALL_OBSERVATION_TARGET_INVALID) {
        g_runtime.target_invalid_count++;
    } else if (result == BALANCE_BALL_OBSERVATION_NOT_MEASURED) {
        g_runtime.not_measured_count++;
    } else if (result == BALANCE_BALL_OBSERVATION_STALE) {
        g_runtime.stale_count++;
    }

    /* Once control is active, a single K230 invalid frame must not make the
     * axis return toward ZERO.  The update loop keeps the last valid sample
     * until BALANCE_VISION_STALE_TIMEOUT_MS expires. */
    if (g_runtime.measurement_valid != 0U) {
        g_runtime.held_invalid_count++;
        return;
    }

    /* During initial acquisition or recovery, validity must still be truly
     * consecutive before the actuator is allowed to re-enter ACTIVE. */
    g_runtime.valid_streak = 0U;
    g_runtime.velocity_mm_s = 0;
    g_runtime.raw_velocity_mm_s = 0;
    g_runtime.velocity_sample_dt_ms = 0U;
    g_runtime.velocity_filter_alpha_x1000 = 0U;
    g_runtime.velocity_reversal_fast = 0U;
}

static void admit_measurement(
    const VisionBallPositionObservation *ball,
    uint8_t force_zero_velocity,
    BalanceBallObservationResult result)
{
    int32_t reportedVelocity = ball->reported_velocity_mm_s;
    int32_t controlVelocity = reportedVelocity;

    if (force_zero_velocity != 0U) {
        controlVelocity = 0;
        g_runtime.velocity_gap_reset_count++;
    } else {
        controlVelocity = clamp_symmetric(controlVelocity,
            BALANCE_BALL_CONTROL_SPEED_LIMIT_MM_S);
        if (controlVelocity != reportedVelocity) {
            g_runtime.speed_clamp_count++;
        }
    }

    g_lastPositionMm = ball->position_mm;
    g_positionInitialized = 1U;
    g_hadMeasurementGap = 0U;
    g_runtime.position_mm = ball->position_mm;
    g_runtime.raw_velocity_mm_s = ball->reported_velocity_mm_s;
    g_runtime.velocity_mm_s = clamp_i32_to_i16(controlVelocity);
    g_runtime.velocity_sample_dt_ms = 0U;
    g_runtime.velocity_filter_alpha_x1000 = 1000U;
    g_runtime.velocity_reversal_fast = 0U;
    g_runtime.axis_span_mm = ball->axis_span_mm;
    g_runtime.confidence = ball->confidence;
    g_runtime.last_measurement_time_ms =
        ball->local_receive_timestamp_ms;
    g_runtime.accepted_measurement_count++;
    g_runtime.last_observation_result = result;
    g_runtime.raw_vision_valid = 1U;
    if (g_runtime.valid_streak <
        BALANCE_BALL_PD_VALID_FRAME_COUNT) {
        g_runtime.valid_streak++;
    }
    g_runtime.measurement_valid =
        (g_runtime.valid_streak >=
            BALANCE_BALL_PD_VALID_FRAME_COUNT) ? 1U : 0U;
}

static void process_observation(uint32_t now_ms)
{
    const VisionBallPositionObservation *ball =
        VisionReceiver_GetBallPositionObservation();
    int32_t delta_mm;
    uint8_t rebase = 0U;
    uint8_t hadPositionBaseline = g_positionInitialized;

    if (ball->available == 0U) {
        return;
    }
    if (ball->update_count == g_lastObservationUpdateCount) {
        return;
    }
    g_lastObservationUpdateCount = ball->update_count;
    g_runtime.last_observation_time_ms = now_ms;

    if (ball->session_id != g_lastSessionId) {
        g_lastSessionId = ball->session_id;
        reset_measurement_history();
    }

    /* Only a new frame that contains a real K230 measurement may update
     * position, velocity history, or the consecutive acquisition count.
     * HOLD/prediction frames remain available in VisionReceiver for
     * diagnostics, but the controller treats them as missing measurements
     * and relies on the bounded last-command hold window instead. */
    if (ball->target_valid == 0U) {
        process_invalid_observation(
            BALANCE_BALL_OBSERVATION_TARGET_INVALID);
        return;
    }
    if (ball->measured == 0U) {
        process_invalid_observation(
            BALANCE_BALL_OBSERVATION_NOT_MEASURED);
        return;
    }
    if ((now_ms - ball->local_receive_timestamp_ms) >
        BALANCE_VISION_STALE_TIMEOUT_MS) {
        process_invalid_observation(
            BALANCE_BALL_OBSERVATION_STALE);
        return;
    }

    if (g_runtime.startup_discard_remaining != 0U) {
        g_runtime.startup_discard_remaining--;
        g_runtime.last_observation_result =
            BALANCE_BALL_OBSERVATION_START_DISCARDED;
        g_runtime.raw_vision_valid = 0U;
        g_runtime.measurement_valid = 0U;
        g_runtime.valid_streak = 0U;
        g_positionInitialized = 0U;
        g_hadMeasurementGap = 1U;
        clear_jump_candidate();
        return;
    }

    if (g_positionInitialized != 0U) {
        delta_mm = (int32_t)ball->position_mm - g_lastPositionMm;
        if (magnitude_i32(delta_mm) >
            BALANCE_BALL_PD_MAX_JUMP_MM) {
            g_runtime.rejected_jump_count++;
            g_runtime.jump_candidate_count++;
            g_hadMeasurementGap = 1U;
            if ((g_jumpCandidateActive != 0U) &&
                ((ball->local_receive_timestamp_ms -
                    g_jumpCandidateTimeMs) <=
                    BALANCE_BALL_JUMP_CONFIRM_TIMEOUT_MS) &&
                (magnitude_i32((int32_t)ball->position_mm -
                    g_jumpCandidatePositionMm) <=
                    BALANCE_BALL_JUMP_CONFIRM_TOLERANCE_MM)) {
                if (g_jumpCandidateCount < UINT8_MAX) {
                    g_jumpCandidateCount++;
                }
                g_jumpCandidatePositionMm = ball->position_mm;
                g_jumpCandidateTimeMs =
                    ball->local_receive_timestamp_ms;
            } else {
                g_jumpCandidateActive = 1U;
                g_jumpCandidateCount = 1U;
                g_jumpCandidatePositionMm = ball->position_mm;
                g_jumpCandidateTimeMs =
                    ball->local_receive_timestamp_ms;
            }
            g_runtime.raw_vision_valid = 0U;
            g_runtime.last_observation_result =
                BALANCE_BALL_OBSERVATION_JUMP_CANDIDATE;
            if (g_jumpCandidateCount <
                BALANCE_BALL_JUMP_CONFIRM_FRAMES) {
                return;
            }
            rebase = 1U;
        }
    }
    if ((g_positionInitialized == 0U) ||
        (g_hadMeasurementGap != 0U)) {
        rebase = 1U;
    }
    if ((rebase != 0U) && (hadPositionBaseline != 0U)) {
        g_runtime.rebase_count++;
    }
    clear_jump_candidate();
    admit_measurement(ball, rebase,
        (rebase != 0U) ?
            BALANCE_BALL_OBSERVATION_POSITION_REBASED :
            BALANCE_BALL_OBSERVATION_ACCEPTED);
}

static uint8_t ensure_tracking_started(
    const BalanceSoftLimitsRuntime *limits,
    const BalancePositionRuntime *position)
{
    if (g_runtime.tracking_owned != 0U) {
        if (position->tracking_enabled != 0U) {
            return 1U;
        }
        g_runtime.tracking_owned = 0U;
    }
    if (position->busy != 0U) {
        return 0U;
    }
    if (BalancePositionControl_StartTracking(0,
        limits->current_logical_count,
        limits->minimum_logical_count,
        limits->maximum_logical_count,
        limits->clamp_count) == 0U) {
        return 0U;
    }
    g_runtime.tracking_owned = 1U;
    g_runtime.commanded_offset_count = 0;
    g_runtime.actuator_target_count = 0;
    return 1U;
}

static uint8_t command_offset_limited(
    const BalanceSoftLimitsRuntime *limits,
    int32_t requested_offset, int32_t requested_maximum,
    int32_t requested_slew);

static uint8_t command_offset(const BalanceSoftLimitsRuntime *limits,
    int32_t requested_offset)
{
    return command_offset_limited(limits, requested_offset,
        maximum_offset_count(limits),
        g_config.target_slew_count_per_20ms);
}

static uint8_t command_offset_limited(
    const BalanceSoftLimitsRuntime *limits,
    int32_t requested_offset, int32_t requested_maximum,
    int32_t requested_slew)
{
    int32_t maximum_offset = maximum_offset_count(limits);
    int32_t target_count;

    if (requested_maximum < maximum_offset) {
        maximum_offset = requested_maximum;
    }
    if (requested_offset > maximum_offset) {
        requested_offset = maximum_offset;
    } else if (requested_offset < -maximum_offset) {
        requested_offset = -maximum_offset;
    }
    g_runtime.commanded_offset_count = slew_toward(
        g_runtime.commanded_offset_count, requested_offset,
        requested_slew);
    target_count = g_runtime.commanded_offset_count;
    g_runtime.actuator_target_count = target_count;
    return BalancePositionControl_SetTrackingTarget(target_count);
}

static int32_t calculate_cascade_request(uint8_t weak_control)
{
    int32_t error;
    int32_t velocity;
    int32_t targetVelocity;
    int32_t velocityError;
    int32_t approachVelocity;
    uint32_t stoppingDistance = 0U;
    uint32_t acceleration;
    uint8_t movingToward;
    int32_t targetComponent;
    int32_t velocityComponent;
    int32_t requestedOffset;

    error = (int32_t)g_runtime.target_mm - g_runtime.position_mm;
    velocity = g_runtime.velocity_mm_s;
    targetVelocity = clamp_i64_to_i32(
        ((int64_t)g_config.kp_x100 * error) / 100);
    targetVelocity = clamp_symmetric(targetVelocity,
        g_config.maximum_target_velocity_mm_s);

    movingToward = (((error > 0) && (velocity > 0)) ||
        ((error < 0) && (velocity < 0))) ? 1U : 0U;
    if (movingToward != 0U) {
        acceleration = (velocity > 0) ?
            g_config.positive_brake_accel_mm_s2 :
            g_config.negative_brake_accel_mm_s2;
        stoppingDistance =
            (magnitude_i32(velocity) * g_config.brake_delay_ms) /
                1000U;
        stoppingDistance += (uint32_t)(((int64_t)velocity * velocity) /
            (2 * (int64_t)acceleration));
        stoppingDistance += g_config.brake_margin_mm;
        if (stoppingDistance > UINT16_MAX) {
            stoppingDistance = UINT16_MAX;
        }
        if (magnitude_i32(error) <= stoppingDistance) {
            approachVelocity = (int32_t)
                g_config.approach_velocity_mm_s;
            if (error < 0) {
                approachVelocity = -approachVelocity;
            }
            if ((error == 0) ||
                (magnitude_i32(targetVelocity) >
                    g_config.approach_velocity_mm_s)) {
                targetVelocity = (error == 0) ? 0 : approachVelocity;
            }
            g_runtime.braking_active = 1U;
        } else {
            g_runtime.braking_active = 0U;
        }
    } else {
        g_runtime.braking_active = 0U;
    }

    velocityError = targetVelocity - velocity;
    targetComponent = clamp_i64_to_i32(
        ((int64_t)g_config.kd_x100 * targetVelocity) / 100);
    velocityComponent = clamp_i64_to_i32(
        -((int64_t)g_config.kd_x100 * velocity) / 100);
    requestedOffset = clamp_i64_to_i32(
        (int64_t)g_config.neutral_bias_count +
        targetComponent + velocityComponent);
    requestedOffset *= g_config.tilt_sign;

    g_runtime.error_mm = clamp_i32_to_i16(error);
    g_runtime.target_velocity_mm_s =
        clamp_i32_to_i16(targetVelocity);
    g_runtime.velocity_error_mm_s =
        clamp_i32_to_i16(velocityError);
    g_runtime.braking_distance_mm = (uint16_t)stoppingDistance;
    g_runtime.p_output_count = targetComponent * g_config.tilt_sign;
    g_runtime.d_output_count = velocityComponent * g_config.tilt_sign;
    g_runtime.pd_output_count = requestedOffset;
    g_runtime.weak_control_active = weak_control;
    return requestedOffset;
}

static void return_toward_zero(
    const BalanceSoftLimitsRuntime *limits, uint8_t cancel_at_zero)
{
    BalancePositionRuntime position;

    BalancePositionControl_GetSnapshot(&position);
    if (ensure_tracking_started(limits, &position) == 0U) {
        return;
    }
    if (command_offset(limits, 0) == 0U) {
        g_runtime.state = BALANCE_BALL_STATE_FAULT;
        return;
    }
    if ((cancel_at_zero != 0U) &&
        (g_runtime.commanded_offset_count == 0) &&
        (magnitude_i32(limits->current_logical_count) <=
            g_config.tracking_reengage_count)) {
        BalancePositionControl_Cancel();
        g_runtime.tracking_owned = 0U;
        g_runtime.state = BALANCE_BALL_STATE_DISABLED;
    }
}

void BalanceBallControl_Init(void)
{
    g_runtime = (BalanceBallControlRuntime){0};
    load_default_config();
    (void)BalancePositionControl_SetTrackingThresholds(
        g_config.tracking_deadband_count,
        g_config.tracking_reengage_count);
    g_runtime.state = BALANCE_BALL_STATE_DISABLED;
    g_lastObservationUpdateCount = 0U;
    g_lastSessionId = 0U;
    g_lastPositionMm = 0;
    g_positionInitialized = 0U;
    g_hadMeasurementGap = 1U;
    clear_jump_candidate();
}

uint8_t BalanceBallControl_Enable(int16_t target_mm)
{
    BalanceSoftLimitsRuntime limits;
    BalancePositionRuntime position;

    if (magnitude_i32(target_mm) >
        BALANCE_BALL_PD_MAX_TARGET_ABS_MM ||
        ((g_runtime.axis_span_mm != 0U) &&
         (magnitude_i32(target_mm) * 2U >
            g_runtime.axis_span_mm))) {
        g_runtime.start_result = BALANCE_BALL_START_TARGET_INVALID;
        return 0U;
    }
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        g_runtime.start_result = BALANCE_BALL_START_ESTOP;
        return 0U;
    }

    BalanceSoftLimits_GetSnapshot(&limits);
    if (limits.zero_valid == 0U) {
        g_runtime.start_result = BALANCE_BALL_START_NO_ZERO;
        return 0U;
    }
    if (limits.limits_valid == 0U) {
        g_runtime.start_result = BALANCE_BALL_START_LIMIT_INVALID;
        return 0U;
    }
    if ((limits.calibration_active != 0U) ||
        (limits.recalibration_armed != 0U) ||
        (limits.test_active != 0U) ||
        (limits.oscillation_active != 0U)) {
        g_runtime.start_result = BALANCE_BALL_START_BUSY;
        return 0U;
    }

    /* A fresh Task5/RUN request owns the axis from this point onward.  Stop
     * stale test/tracking work first, then clear only a stopped, recoverable
     * position-loop latch.  If the physical problem remains it will trip
     * again normally after motion restarts. */
    BalanceBallControl_ForceStop();
    BalanceSoftLimits_StopMotion();
    BalancePositionControl_GetSnapshot(&position);
    if (position.fault != BALANCE_POSITION_FAULT_NONE) {
        if (BalanceSoftLimits_ResetPositionFault() == 0U) {
            g_runtime.start_result = BALANCE_BALL_START_POSITION_FAULT;
            return 0U;
        }
    }

    g_runtime.target_mm = target_mm;
    g_runtime.measurement_valid = 0U;
    g_runtime.raw_vision_valid = 0U;
    g_runtime.valid_streak = 0U;
    g_runtime.velocity_mm_s = 0;
    g_runtime.raw_velocity_mm_s = 0;
    g_runtime.velocity_sample_dt_ms = 0U;
    g_runtime.velocity_filter_alpha_x1000 = 0U;
    g_runtime.velocity_reversal_fast = 0U;
    g_runtime.last_measurement_time_ms = 0U;
    g_runtime.target_velocity_mm_s = 0;
    g_runtime.velocity_error_mm_s = 0;
    g_runtime.braking_distance_mm = 0U;
    g_runtime.braking_active = 0U;
    g_runtime.weak_control_active = 0U;
    g_runtime.startup_discard_remaining =
        BALANCE_BALL_START_DISCARD_FRAMES;
    g_positionInitialized = 0U;
    g_hadMeasurementGap = 1U;
    clear_jump_candidate();
    g_runtime.start_result = BALANCE_BALL_START_OK;
    g_runtime.enable_requested = 1U;
    g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
    return 1U;
}

void BalanceBallControl_Disable(void)
{
    g_runtime.enable_requested = 0U;
    g_runtime.state = (g_runtime.tracking_owned != 0U) ?
        BALANCE_BALL_STATE_RETURN_ZERO :
        BALANCE_BALL_STATE_DISABLED;
}

void BalanceBallControl_ForceStop(void)
{
    BalancePositionControl_Cancel();
    g_runtime.enable_requested = 0U;
    g_runtime.tracking_owned = 0U;
    g_runtime.commanded_offset_count = 0;
    g_runtime.actuator_target_count = 0;
    g_runtime.weak_control_active = 0U;
    g_runtime.state = BALANCE_BALL_STATE_DISABLED;
}

uint8_t BalanceBallControl_SetTargetMm(int16_t target_mm)
{
    if (magnitude_i32(target_mm) >
        BALANCE_BALL_PD_MAX_TARGET_ABS_MM ||
        ((g_runtime.axis_span_mm != 0U) &&
         (magnitude_i32(target_mm) * 2U >
            g_runtime.axis_span_mm))) {
        return 0U;
    }
    g_runtime.target_mm = target_mm;
    return 1U;
}

void BalanceBallControl_Update20ms(uint32_t now_ms,
    uint32_t elapsed_ms)
{
    BalanceSoftLimitsRuntime limits;
    BalancePositionRuntime position;
    uint32_t measurement_age_ms;
    int32_t requested_offset;

    (void)elapsed_ms;
    process_observation(now_ms);

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        g_runtime.start_result = BALANCE_BALL_START_ESTOP;
        BalanceBallControl_ForceStop();
        return;
    }

    BalanceSoftLimits_GetSnapshot(&limits);
    BalancePositionControl_GetSnapshot(&position);
    if (position.fault != BALANCE_POSITION_FAULT_NONE) {
        g_runtime.start_result = BALANCE_BALL_START_POSITION_FAULT;
        g_runtime.enable_requested = 0U;
        g_runtime.tracking_owned = 0U;
        g_runtime.state = BALANCE_BALL_STATE_FAULT;
        return;
    }

    if (g_runtime.enable_requested == 0U) {
        if (g_runtime.tracking_owned != 0U) {
            if (axis_is_ready(&limits, &position) == 0U) {
                BalancePositionControl_Cancel();
                g_runtime.tracking_owned = 0U;
                g_runtime.state = BALANCE_BALL_STATE_DISABLED;
                return;
            }
            g_runtime.state = BALANCE_BALL_STATE_RETURN_ZERO;
            return_toward_zero(&limits, 1U);
        } else {
            g_runtime.state = BALANCE_BALL_STATE_DISABLED;
        }
        return;
    }

    if (axis_is_ready(&limits, &position) == 0U) {
        if (g_runtime.tracking_owned != 0U) {
            BalancePositionControl_Cancel();
            g_runtime.tracking_owned = 0U;
        }
        if (limits.zero_valid == 0U) {
            g_runtime.start_result = BALANCE_BALL_START_NO_ZERO;
        } else if (limits.limits_valid == 0U) {
            g_runtime.start_result = BALANCE_BALL_START_LIMIT_INVALID;
        } else if (position.fault != BALANCE_POSITION_FAULT_NONE) {
            g_runtime.start_result = BALANCE_BALL_START_POSITION_FAULT;
        } else {
            g_runtime.start_result = BALANCE_BALL_START_BUSY;
        }
        g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
        return;
    }
    g_runtime.start_result = BALANCE_BALL_START_OK;

    if (g_runtime.last_measurement_time_ms == 0U) {
        measurement_age_ms = UINT32_MAX;
    } else {
        measurement_age_ms = now_ms -
            g_runtime.last_measurement_time_ms;
    }
    if (g_runtime.measurement_valid == 0U) {
        g_runtime.state = (measurement_age_ms >
            BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS) ?
            BALANCE_BALL_STATE_VISION_LOST :
            BALANCE_BALL_STATE_WAIT_VISION;
        if ((g_runtime.valid_streak != 0U) &&
            (measurement_age_ms <= BALANCE_VISION_STALE_TIMEOUT_MS) &&
            (g_runtime.axis_span_mm != 0U)) {
            if (ensure_tracking_started(&limits, &position) == 0U) {
                g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
                return;
            }
            requested_offset = calculate_cascade_request(1U);
            if (command_offset_limited(&limits, requested_offset,
                    BALANCE_BALL_START_WEAK_MAX_OFFSET_COUNTS,
                    BALANCE_BALL_START_WEAK_SLEW_COUNTS_PER_20MS) == 0U) {
                g_runtime.state = BALANCE_BALL_STATE_FAULT;
            }
        } else if (g_runtime.tracking_owned != 0U) {
            g_runtime.weak_control_active = 0U;
            return_toward_zero(&limits, 0U);
        }
        return;
    }

    if (measurement_age_ms >
        BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS) {
        g_runtime.raw_vision_valid = 0U;
        g_runtime.measurement_valid = 0U;
        g_runtime.valid_streak = 0U;
        g_runtime.velocity_mm_s = 0;
        g_runtime.raw_velocity_mm_s = 0;
        g_runtime.velocity_sample_dt_ms = 0U;
        g_runtime.velocity_filter_alpha_x1000 = 0U;
        g_runtime.velocity_reversal_fast = 0U;
        g_hadMeasurementGap = 1U;
        clear_jump_candidate();
        g_runtime.weak_control_active = 0U;
        if (g_runtime.state != BALANCE_BALL_STATE_VISION_LOST) {
            g_runtime.vision_lost_count++;
        }
        g_runtime.state = BALANCE_BALL_STATE_VISION_LOST;
        if (g_runtime.tracking_owned != 0U) {
            return_toward_zero(&limits, 0U);
        }
        return;
    }

    if (measurement_age_ms > BALANCE_VISION_STALE_TIMEOUT_MS) {
        /* Bounded sample-and-hold: a rejected or missing K230 frame must not
         * erase the last real ball position. Freeze the last cascade request
         * request and keep advancing the slew-limited axis target.  This
         * lets the stepper finish the intended tilt through short corrupt
         * bursts, while the hard timeout above prevents indefinite motion
         * on stale vision. */
        if (ensure_tracking_started(&limits, &position) == 0U) {
            g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
            return;
        }
        if (command_offset(&limits,
            g_runtime.pd_output_count) == 0U) {
            BalancePositionControl_Cancel();
            g_runtime.start_result = BALANCE_BALL_START_POSITION_FAULT;
            g_runtime.enable_requested = 0U;
            g_runtime.tracking_owned = 0U;
            g_runtime.state = BALANCE_BALL_STATE_FAULT;
            return;
        }
        g_runtime.state = BALANCE_BALL_STATE_HOLD_LAST;
        return;
    }

    if ((g_runtime.axis_span_mm == 0U) ||
        (magnitude_i32(g_runtime.target_mm) * 2U >
            g_runtime.axis_span_mm)) {
        g_runtime.start_result = BALANCE_BALL_START_DATA_INVALID;
        BalancePositionControl_Cancel();
        g_runtime.enable_requested = 0U;
        g_runtime.tracking_owned = 0U;
        g_runtime.state = BALANCE_BALL_STATE_FAULT;
        return;
    }

    if (ensure_tracking_started(&limits, &position) == 0U) {
        g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
        return;
    }

    requested_offset = calculate_cascade_request(0U);
    if (command_offset(&limits, requested_offset) == 0U) {
        BalancePositionControl_Cancel();
        g_runtime.start_result = BALANCE_BALL_START_POSITION_FAULT;
        g_runtime.enable_requested = 0U;
        g_runtime.tracking_owned = 0U;
        g_runtime.state = BALANCE_BALL_STATE_FAULT;
        return;
    }
    g_runtime.state = BALANCE_BALL_STATE_ACTIVE;
}

void BalanceBallControl_GetSnapshot(BalanceBallControlRuntime *snapshot)
{
    if (snapshot != (BalanceBallControlRuntime *)0) {
        *snapshot = g_runtime;
    }
}

void BalanceBallControl_GetConfig(BalanceBallControlConfig *config)
{
    if (config != (BalanceBallControlConfig *)0) {
        *config = g_config;
    }
}

uint8_t BalanceBallControl_SetConfig(
    const BalanceBallControlConfig *config)
{
    uint8_t active;

    if (config_is_valid(config) == 0U) {
        return 0U;
    }
    active = ((g_runtime.enable_requested != 0U) ||
        (g_runtime.tracking_owned != 0U)) ? 1U : 0U;
    if (active != 0U) {
        if ((config->maximum_offset_count !=
                g_config.maximum_offset_count) ||
            (config->target_slew_count_per_20ms !=
                g_config.target_slew_count_per_20ms) ||
            (config->tracking_deadband_count !=
                g_config.tracking_deadband_count) ||
            (config->tracking_reengage_count !=
                g_config.tracking_reengage_count) ||
            (config->tilt_sign != g_config.tilt_sign)) {
            return 0U;
        }
        g_config.kp_x100 = config->kp_x100;
        g_config.kd_x100 = config->kd_x100;
        g_config.maximum_target_velocity_mm_s =
            config->maximum_target_velocity_mm_s;
        g_config.neutral_bias_count = config->neutral_bias_count;
        g_config.positive_brake_accel_mm_s2 =
            config->positive_brake_accel_mm_s2;
        g_config.negative_brake_accel_mm_s2 =
            config->negative_brake_accel_mm_s2;
        g_config.brake_delay_ms = config->brake_delay_ms;
        g_config.brake_margin_mm = config->brake_margin_mm;
        g_config.approach_velocity_mm_s =
            config->approach_velocity_mm_s;
        return 1U;
    }
    if (BalancePositionControl_SetTrackingThresholds(
            config->tracking_deadband_count,
            config->tracking_reengage_count) == 0U) {
        return 0U;
    }
    g_config = *config;
    return 1U;
}

void BalanceBallControl_ResetConfig(void)
{
    BalanceBallControlConfig config;

    load_default_config();
    config = g_config;
    (void)BalancePositionControl_SetTrackingThresholds(
        config.tracking_deadband_count,
        config.tracking_reengage_count);
}

#else

static BalanceBallControlRuntime g_runtime;
static BalanceBallControlConfig g_config;

void BalanceBallControl_Init(void)
{
    g_runtime = (BalanceBallControlRuntime){0};
    g_config = (BalanceBallControlConfig){0};
    g_runtime.state = BALANCE_BALL_STATE_DISABLED;
}

uint8_t BalanceBallControl_Enable(int16_t target_mm)
{
    (void)target_mm;
    return 0U;
}

void BalanceBallControl_Disable(void)
{
}

void BalanceBallControl_ForceStop(void)
{
}

uint8_t BalanceBallControl_SetTargetMm(int16_t target_mm)
{
    (void)target_mm;
    return 0U;
}

void BalanceBallControl_Update20ms(uint32_t now_ms,
    uint32_t elapsed_ms)
{
    (void)now_ms;
    (void)elapsed_ms;
}

void BalanceBallControl_GetSnapshot(BalanceBallControlRuntime *snapshot)
{
    if (snapshot != (BalanceBallControlRuntime *)0) {
        *snapshot = g_runtime;
    }
}

void BalanceBallControl_GetConfig(BalanceBallControlConfig *config)
{
    if (config != (BalanceBallControlConfig *)0) {
        *config = g_config;
    }
}

uint8_t BalanceBallControl_SetConfig(
    const BalanceBallControlConfig *config)
{
    (void)config;
    return 0U;
}

void BalanceBallControl_ResetConfig(void)
{
}

#endif
