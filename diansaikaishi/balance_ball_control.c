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
static uint32_t g_lastPositionTimeMs;
static uint8_t g_velocityInitialized;

static void load_default_config(void)
{
    g_config.kp_x100 =
        BALANCE_BALL_PD_KP_COUNTS_PER_MM_X100;
    g_config.kd_x100 =
        BALANCE_BALL_PD_KD_COUNTS_PER_MM_S_X100;
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
        (config->kp_x100 >= 0) && (config->kp_x100 <= 1000) &&
        (config->kd_x100 >= 0) && (config->kd_x100 <= 100) &&
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

static void reset_measurement_history(void)
{
    g_runtime.raw_vision_valid = 0U;
    g_runtime.measurement_valid = 0U;
    g_runtime.valid_streak = 0U;
    g_runtime.velocity_mm_s = 0;
    g_runtime.last_measurement_time_ms = 0U;
    g_velocityInitialized = 0U;
    g_lastPositionTimeMs = 0U;
}

static void process_invalid_observation(void)
{
    g_runtime.raw_vision_valid = 0U;

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
    g_velocityInitialized = 0U;
    g_lastPositionTimeMs = 0U;
}

static void process_observation(uint32_t now_ms)
{
    const VisionBallPositionObservation *ball =
        VisionReceiver_GetBallPositionObservation();
    uint32_t delta_ms;
    int32_t delta_mm;
    int32_t raw_velocity;

    if (ball->available == 0U) {
        return;
    }
    if (ball->update_count == g_lastObservationUpdateCount) {
        return;
    }
    g_lastObservationUpdateCount = ball->update_count;

    if (ball->session_id != g_lastSessionId) {
        g_lastSessionId = ball->session_id;
        reset_measurement_history();
    }

    if ((ball->target_valid == 0U) ||
        (ball->confidence < BALANCE_BALL_PD_MIN_CONFIDENCE) ||
        ((now_ms - ball->local_receive_timestamp_ms) >
            BALANCE_VISION_STALE_TIMEOUT_MS)) {
        process_invalid_observation();
        return;
    }

    if (g_velocityInitialized != 0U) {
        delta_mm = (int32_t)ball->position_mm - g_lastPositionMm;
        if (magnitude_i32(delta_mm) >
            BALANCE_BALL_PD_MAX_JUMP_MM) {
            g_runtime.rejected_jump_count++;
            process_invalid_observation();
            return;
        }
        delta_ms = ball->local_receive_timestamp_ms -
            g_lastPositionTimeMs;
        if (delta_ms != 0U) {
            raw_velocity = (delta_mm * 1000) / (int32_t)delta_ms;
            g_runtime.velocity_mm_s = clamp_i32_to_i16(
                ((int32_t)g_runtime.velocity_mm_s * 3 +
                    raw_velocity) / 4);
        }
    } else {
        g_velocityInitialized = 1U;
        g_runtime.velocity_mm_s = 0;
    }

    g_lastPositionMm = ball->position_mm;
    g_lastPositionTimeMs = ball->local_receive_timestamp_ms;
    g_runtime.position_mm = ball->position_mm;
    g_runtime.axis_span_mm = ball->axis_span_mm;
    g_runtime.confidence = ball->confidence;
    g_runtime.last_measurement_time_ms =
        ball->local_receive_timestamp_ms;
    g_runtime.accepted_measurement_count++;
    g_runtime.raw_vision_valid = 1U;
    if (g_runtime.valid_streak <
        BALANCE_BALL_PD_VALID_FRAME_COUNT) {
        g_runtime.valid_streak++;
    }
    g_runtime.measurement_valid =
        (g_runtime.valid_streak >=
            BALANCE_BALL_PD_VALID_FRAME_COUNT) ? 1U : 0U;
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

static uint8_t command_offset(const BalanceSoftLimitsRuntime *limits,
    int32_t requested_offset)
{
    int32_t maximum_offset = maximum_offset_count(limits);
    int32_t target_count;

    if (requested_offset > maximum_offset) {
        requested_offset = maximum_offset;
    } else if (requested_offset < -maximum_offset) {
        requested_offset = -maximum_offset;
    }
    g_runtime.commanded_offset_count = slew_toward(
        g_runtime.commanded_offset_count, requested_offset,
        g_config.target_slew_count_per_20ms);
    target_count = g_runtime.commanded_offset_count;
    g_runtime.actuator_target_count = target_count;
    return BalancePositionControl_SetTrackingTarget(target_count);
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
    g_lastPositionTimeMs = 0U;
    g_velocityInitialized = 0U;
}

uint8_t BalanceBallControl_Enable(int16_t target_mm)
{
    if (magnitude_i32(target_mm) >
        BALANCE_BALL_PD_MAX_TARGET_ABS_MM ||
        ((g_runtime.axis_span_mm != 0U) &&
         (magnitude_i32(target_mm) * 2U >
            g_runtime.axis_span_mm))) {
        return 0U;
    }
    g_runtime.target_mm = target_mm;
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
    int64_t pd_scaled;
    int32_t requested_offset;

    (void)elapsed_ms;
    process_observation(now_ms);

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        BalanceBallControl_ForceStop();
        return;
    }

    BalanceSoftLimits_GetSnapshot(&limits);
    BalancePositionControl_GetSnapshot(&position);
    if (position.fault != BALANCE_POSITION_FAULT_NONE) {
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
        g_runtime.state = BALANCE_BALL_STATE_WAIT_AXIS;
        return;
    }

    if (g_runtime.last_measurement_time_ms == 0U) {
        measurement_age_ms = UINT32_MAX;
    } else {
        measurement_age_ms = now_ms -
            g_runtime.last_measurement_time_ms;
    }
    if ((g_runtime.measurement_valid == 0U) ||
        (measurement_age_ms > BALANCE_VISION_STALE_TIMEOUT_MS)) {
        if (measurement_age_ms > BALANCE_VISION_STALE_TIMEOUT_MS) {
            g_runtime.raw_vision_valid = 0U;
            g_runtime.measurement_valid = 0U;
            g_runtime.valid_streak = 0U;
            g_runtime.velocity_mm_s = 0;
            g_velocityInitialized = 0U;
            g_lastPositionTimeMs = 0U;
        }
        if (measurement_age_ms >
            BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS) {
            if (g_runtime.state != BALANCE_BALL_STATE_VISION_LOST) {
                g_runtime.vision_lost_count++;
            }
            g_runtime.state = BALANCE_BALL_STATE_VISION_LOST;
        } else {
            g_runtime.state = BALANCE_BALL_STATE_WAIT_VISION;
        }
        if (g_runtime.tracking_owned != 0U) {
            return_toward_zero(&limits, 0U);
        }
        return;
    }

    if ((g_runtime.axis_span_mm == 0U) ||
        (magnitude_i32(g_runtime.target_mm) * 2U >
            g_runtime.axis_span_mm)) {
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

    g_runtime.error_mm = clamp_i32_to_i16(
        (int32_t)g_runtime.target_mm - g_runtime.position_mm);
    g_runtime.p_output_count = clamp_i64_to_i32(
        (((int64_t)g_config.kp_x100 * g_runtime.error_mm) / 100) *
            g_config.tilt_sign);
    g_runtime.d_output_count = clamp_i64_to_i32(
        -((int64_t)g_config.kd_x100 *
            g_runtime.velocity_mm_s) / 100 * g_config.tilt_sign);
    pd_scaled =
        (int64_t)g_config.kp_x100 * g_runtime.error_mm -
        (int64_t)g_config.kd_x100 * g_runtime.velocity_mm_s;
    requested_offset = clamp_i64_to_i32(pd_scaled / 100);
    requested_offset *= g_config.tilt_sign;
    g_runtime.pd_output_count = requested_offset;
    if (command_offset(&limits, requested_offset) == 0U) {
        BalancePositionControl_Cancel();
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
