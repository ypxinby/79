#include "gimbal_tracker.h"

#include "gimbal.h"

#define GIMBAL_TRACKER_DEFAULT_DT_S          (0.010f)
#define GIMBAL_TRACKER_MIN_DT_S              (0.001f)
#define GIMBAL_TRACKER_MAX_DT_S              (0.050f)
#define GIMBAL_TRACKER_FILTER_ALPHA          (0.35f)
#define GIMBAL_TRACKER_DEADBAND_PX           (8.0f)
#define GIMBAL_TRACKER_YAW_KP_DEG_S_PER_PX   (0.08f)
#define GIMBAL_TRACKER_PITCH_KP_DEG_S_PER_PX (0.06f)
#define GIMBAL_TRACKER_MAX_YAW_SPEED_DEG_S   (30.0f)
#define GIMBAL_TRACKER_MAX_PITCH_SPEED_DEG_S (20.0f)
#define GIMBAL_TRACKER_MAX_YAW_DELTA_DEG     (1.0f)
#define GIMBAL_TRACKER_MAX_PITCH_DELTA_DEG   (1.0f)
#define GIMBAL_TRACKER_SIM_OBSERVATION_UPDATES (30U)

/*
 * Direction mapping is intentionally centralized here for P23实测校正.
 * error_x_px > 0 means target is to the right of image center.
 * error_y_px > 0 means target is below image center.
 * If an axis moves in the opposite direction during validation, flip the
 * corresponding sign only; do not change lower-level stepper polarity first.
 */
#define GIMBAL_TRACKER_YAW_ERROR_SIGN        (1.0f)
#define GIMBAL_TRACKER_PITCH_ERROR_SIGN      (1.0f)

static GimbalTargetObservation g_observation;
static GimbalTrackerFeedback g_feedback;
static float g_filteredErrorX;
static float g_filteredErrorY;
static float g_yawTargetWrappedDeg;
static float g_pitchTargetDeg;
static uint8_t g_enabled;
static uint8_t g_targetValid;
static uint8_t g_trackingActive;
static uint8_t g_filterReady;
static uint8_t g_targetsSynced;
static uint8_t g_simObservationUpdatesRemaining;

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float wrap_deg_360(float deg)
{
    while (deg >= 360.0f) {
        deg -= 360.0f;
    }
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

static int16_t float_to_x10(float value)
{
    float raw = value * 10.0f;

    if (raw > 32767.0f) {
        return 32767;
    }
    if (raw < -32768.0f) {
        return -32768;
    }
    if (raw >= 0.0f) {
        return (int16_t)(raw + 0.5f);
    }
    return (int16_t)(raw - 0.5f);
}

static int16_t float_to_i16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static float apply_deadband(float value, float deadband, uint8_t *deadbanded)
{
    if (abs_f32(value) <= deadband) {
        if (deadbanded != 0) {
            *deadbanded = 1U;
        }
        return 0.0f;
    }
    if (deadbanded != 0) {
        *deadbanded = 0U;
    }
    return value;
}

static void sync_targets_from_gimbal(void)
{
    const GimbalFeedback *yaw = Gimbal_YawGetFeedback();
    const GimbalFeedback *pitch = Gimbal_PitchGetFeedback();

    g_yawTargetWrappedDeg =
        wrap_deg_360((float)yaw->wrapped_deg_x10 / 10.0f);
    g_pitchTargetDeg = (float)pitch->continuous_deg_x10 / 10.0f;
    g_targetsSynced = 1U;
}

static void stop_tracking_hold(void)
{
    if (g_trackingActive != 0U) {
        Gimbal_YawStopHold();
        Gimbal_PitchStopHold();
    }
    g_trackingActive = 0U;
    g_targetsSynced = 0U;
    g_feedback.yaw_deadbanded = 0U;
    g_feedback.pitch_deadbanded = 0U;
    g_feedback.command_limited = 0U;
}

static void update_feedback(float yaw_speed, float pitch_speed,
    float yaw_delta, float pitch_delta)
{
    g_feedback.enabled = g_enabled;
    g_feedback.target_valid = g_targetValid;
    g_feedback.tracking_active = g_trackingActive;
    g_feedback.filter_ready = g_filterReady;
    g_feedback.observation_sequence = g_observation.sequence;
    g_feedback.observation_timestamp_ms = g_observation.timestamp_ms;
    g_feedback.raw_error_x_px = g_observation.error_x_px;
    g_feedback.raw_error_y_px = g_observation.error_y_px;
    g_feedback.filtered_error_x_px = float_to_i16(g_filteredErrorX);
    g_feedback.filtered_error_y_px = float_to_i16(g_filteredErrorY);
    g_feedback.yaw_speed_deg_s_x10 = float_to_x10(yaw_speed);
    g_feedback.pitch_speed_deg_s_x10 = float_to_x10(pitch_speed);
    g_feedback.yaw_delta_deg_x10 = float_to_x10(yaw_delta);
    g_feedback.pitch_delta_deg_x10 = float_to_x10(pitch_delta);
    g_feedback.yaw_target_deg_x10 = float_to_x10(g_yawTargetWrappedDeg);
    g_feedback.pitch_target_deg_x10 = float_to_x10(g_pitchTargetDeg);
}

void GimbalTracker_Init(void)
{
    g_observation = (GimbalTargetObservation){0};
    g_feedback = (GimbalTrackerFeedback){0};
    g_filteredErrorX = 0.0f;
    g_filteredErrorY = 0.0f;
    g_yawTargetWrappedDeg = 0.0f;
    g_pitchTargetDeg = 0.0f;
    g_enabled = 0U;
    g_targetValid = 0U;
    g_trackingActive = 0U;
    g_filterReady = 0U;
    g_targetsSynced = 0U;
    g_simObservationUpdatesRemaining = 0U;
    sync_targets_from_gimbal();
    update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
}

void GimbalTracker_Enable(uint8_t enable)
{
    g_enabled = (enable != 0U) ? 1U : 0U;
    if (g_enabled == 0U) {
        stop_tracking_hold();
        g_filterReady = 0U;
    } else {
        sync_targets_from_gimbal();
    }
    update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
}

void GimbalTracker_PushObservation(
    const GimbalTargetObservation *observation)
{
    if (observation == 0) {
        GimbalTracker_ClearObservation();
        return;
    }

    g_observation = *observation;
    g_observation.valid = (observation->valid != 0U) ? 1U : 0U;
    g_targetValid = g_observation.valid;
    if (g_targetValid == 0U) {
        g_filterReady = 0U;
        g_simObservationUpdatesRemaining = 0U;
        g_feedback.yaw_deadbanded = 0U;
        g_feedback.pitch_deadbanded = 0U;
        g_feedback.command_limited = 0U;
    } else if (g_observation.timestamp_ms == 0U) {
        /*
         * P23按键模拟输入没有真实时间戳。为了避免短按一次后
         * 固定error被Tracker永久复用，timestamp=0的模拟Observation
         * 只保持约300ms；真实K230观测后续应填入非0时间戳并连续刷新。
         */
        g_simObservationUpdatesRemaining =
            GIMBAL_TRACKER_SIM_OBSERVATION_UPDATES;
    } else {
        g_simObservationUpdatesRemaining = 0U;
    }
    update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
}

void GimbalTracker_ClearObservation(void)
{
    g_observation.valid = 0U;
    g_targetValid = 0U;
    g_filterReady = 0U;
    g_simObservationUpdatesRemaining = 0U;
    g_feedback.yaw_deadbanded = 0U;
    g_feedback.pitch_deadbanded = 0U;
    g_feedback.command_limited = 0U;
    update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
}

void GimbalTracker_Update(float dt_s)
{
    float error_x;
    float error_y;
    float yaw_speed;
    float pitch_speed;
    float yaw_delta;
    float pitch_delta;
    uint8_t yaw_deadbanded;
    uint8_t pitch_deadbanded;
    uint8_t command_limited = 0U;

    if ((dt_s < GIMBAL_TRACKER_MIN_DT_S) ||
        (dt_s > GIMBAL_TRACKER_MAX_DT_S)) {
        dt_s = GIMBAL_TRACKER_DEFAULT_DT_S;
    }

    g_feedback.update_count_10ms++;

    if ((g_enabled == 0U) || (g_targetValid == 0U)) {
        stop_tracking_hold();
        update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (g_targetsSynced == 0U) {
        sync_targets_from_gimbal();
    }

    if (g_filterReady == 0U) {
        g_filteredErrorX = (float)g_observation.error_x_px;
        g_filteredErrorY = (float)g_observation.error_y_px;
        g_filterReady = 1U;
    } else {
        g_filteredErrorX +=
            GIMBAL_TRACKER_FILTER_ALPHA *
            ((float)g_observation.error_x_px - g_filteredErrorX);
        g_filteredErrorY +=
            GIMBAL_TRACKER_FILTER_ALPHA *
            ((float)g_observation.error_y_px - g_filteredErrorY);
    }

    error_x = apply_deadband(g_filteredErrorX,
        GIMBAL_TRACKER_DEADBAND_PX, &yaw_deadbanded);
    error_y = apply_deadband(g_filteredErrorY,
        GIMBAL_TRACKER_DEADBAND_PX, &pitch_deadbanded);

    yaw_speed =
        GIMBAL_TRACKER_YAW_ERROR_SIGN *
        GIMBAL_TRACKER_YAW_KP_DEG_S_PER_PX * error_x;
    pitch_speed =
        GIMBAL_TRACKER_PITCH_ERROR_SIGN *
        GIMBAL_TRACKER_PITCH_KP_DEG_S_PER_PX * error_y;

    if (yaw_speed > GIMBAL_TRACKER_MAX_YAW_SPEED_DEG_S) {
        yaw_speed = GIMBAL_TRACKER_MAX_YAW_SPEED_DEG_S;
        command_limited = 1U;
    } else if (yaw_speed < -GIMBAL_TRACKER_MAX_YAW_SPEED_DEG_S) {
        yaw_speed = -GIMBAL_TRACKER_MAX_YAW_SPEED_DEG_S;
        command_limited = 1U;
    }

    if (pitch_speed > GIMBAL_TRACKER_MAX_PITCH_SPEED_DEG_S) {
        pitch_speed = GIMBAL_TRACKER_MAX_PITCH_SPEED_DEG_S;
        command_limited = 1U;
    } else if (pitch_speed < -GIMBAL_TRACKER_MAX_PITCH_SPEED_DEG_S) {
        pitch_speed = -GIMBAL_TRACKER_MAX_PITCH_SPEED_DEG_S;
        command_limited = 1U;
    }

    yaw_delta = clamp_f32(yaw_speed * dt_s,
        -GIMBAL_TRACKER_MAX_YAW_DELTA_DEG,
        GIMBAL_TRACKER_MAX_YAW_DELTA_DEG);
    pitch_delta = clamp_f32(pitch_speed * dt_s,
        -GIMBAL_TRACKER_MAX_PITCH_DELTA_DEG,
        GIMBAL_TRACKER_MAX_PITCH_DELTA_DEG);
    if ((yaw_delta != (yaw_speed * dt_s)) ||
        (pitch_delta != (pitch_speed * dt_s))) {
        command_limited = 1U;
    }

    g_yawTargetWrappedDeg = wrap_deg_360(g_yawTargetWrappedDeg + yaw_delta);
    g_pitchTargetDeg += pitch_delta;

    Gimbal_YawSetWrappedTargetDeg(g_yawTargetWrappedDeg);
    Gimbal_PitchSetTargetDeg(g_pitchTargetDeg);

    {
        const GimbalFeedback *pitch = Gimbal_PitchGetFeedback();

        g_pitchTargetDeg = (float)pitch->target_deg_x10 / 10.0f;
    }

    g_trackingActive = 1U;
    g_feedback.yaw_deadbanded = yaw_deadbanded;
    g_feedback.pitch_deadbanded = pitch_deadbanded;
    g_feedback.command_limited = command_limited;
    update_feedback(yaw_speed, pitch_speed, yaw_delta, pitch_delta);

    if (g_simObservationUpdatesRemaining != 0U) {
        g_simObservationUpdatesRemaining--;
        if (g_simObservationUpdatesRemaining == 0U) {
            g_observation.valid = 0U;
            g_targetValid = 0U;
            g_filterReady = 0U;
            stop_tracking_hold();
            update_feedback(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

const GimbalTrackerFeedback *GimbalTracker_GetFeedback(void)
{
    update_feedback(g_feedback.yaw_speed_deg_s_x10 / 10.0f,
        g_feedback.pitch_speed_deg_s_x10 / 10.0f,
        g_feedback.yaw_delta_deg_x10 / 10.0f,
        g_feedback.pitch_delta_deg_x10 / 10.0f);
    return &g_feedback;
}
