#include "gimbal_vision_yaw_tracker.h"

#include "gimbal.h"
#include "gimbal_tracker.h"
#include "gimbal_vision_adapter.h"
#include "vision_yaw_tuning.h"

#define VISION_YAW_MAX_DELTA_DEG           (0.30f)
#define VISION_YAW_ERROR_SIGN              (1.0f)
#define VISION_YAW_UPDATE_DT_S             (0.010f)

static GimbalVisionYawFeedback g_feedback;
static uint8_t g_enabled;
static uint8_t g_trackingActive;
static uint8_t g_targetSynced;
static uint8_t g_sessionInitialized;
static uint32_t g_sessionId;
static float g_targetWrappedDeg;

static float clamp_f32(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static float wrap_deg_360(float value)
{
    while (value >= 360.0f) {
        value -= 360.0f;
    }
    while (value < 0.0f) {
        value += 360.0f;
    }
    return value;
}

static int16_t float_to_scaled_i16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    if (scaled >= 0.0f) {
        return (int16_t)(scaled + 0.5f);
    }
    return (int16_t)(scaled - 0.5f);
}

static void refresh_yaw_feedback(void)
{
    const GimbalFeedback *yaw = Gimbal_YawGetFeedback();

    g_feedback.position_valid = yaw->position_valid;
    g_feedback.world_lock_enabled = yaw->world_lock_enabled;
    g_feedback.current_wrapped_deg_x10 = yaw->wrapped_deg_x10;
    g_feedback.limit_direction = yaw->limit_direction;
    g_feedback.positive_limit = yaw->positive_limit;
    g_feedback.negative_limit = yaw->negative_limit;
}

static void sync_target_from_yaw(void)
{
    const GimbalFeedback *yaw = Gimbal_YawGetFeedback();

    g_targetWrappedDeg =
        wrap_deg_360((float)yaw->wrapped_deg_x10 / 10.0f);
    g_targetSynced = 1U;
    g_feedback.target_wrapped_deg_x10 =
        float_to_scaled_i16(g_targetWrappedDeg, 10.0f);
}

static void clear_command_feedback(void)
{
    g_feedback.tracking_active = 0U;
    g_feedback.command_speed_deg_s_x10 = 0;
    g_feedback.command_delta_deg_x1000 = 0;
}

static void stop_owned_tracking(void)
{
    if (g_trackingActive != 0U) {
        Gimbal_YawStopHold();
    }
    g_trackingActive = 0U;
    g_targetSynced = 0U;
    clear_command_feedback();
    refresh_yaw_feedback();
    sync_target_from_yaw();
}

static void relinquish_without_stop(void)
{
    g_trackingActive = 0U;
    g_targetSynced = 0U;
    clear_command_feedback();
    refresh_yaw_feedback();
    g_feedback.target_wrapped_deg_x10 =
        g_feedback.current_wrapped_deg_x10;
}

static uint8_t speed_points_toward_limit(float speedDegS,
    int8_t limitDirection)
{
    if ((limitDirection > 0) && (speedDegS > 0.0f)) {
        return 1U;
    }
    if ((limitDirection < 0) && (speedDegS < 0.0f)) {
        return 1U;
    }
    return 0U;
}

void GimbalVisionYawTracker_Init(void)
{
    g_feedback = (GimbalVisionYawFeedback){0};
    g_feedback.state = GIMBAL_VISION_YAW_DISABLED;
    g_enabled = 0U;
    g_trackingActive = 0U;
    g_targetSynced = 0U;
    g_sessionInitialized = 0U;
    g_sessionId = 0U;
    g_targetWrappedDeg = 0.0f;
    refresh_yaw_feedback();
    sync_target_from_yaw();
}

uint8_t GimbalVisionYawTracker_Enable(uint8_t enable)
{
    refresh_yaw_feedback();

    if (enable == 0U) {
        if (g_feedback.world_lock_enabled == 0U) {
            stop_owned_tracking();
        } else {
            relinquish_without_stop();
        }
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_DISABLED;
        g_sessionInitialized = 0U;
        return 1U;
    }

    if (g_feedback.position_valid == 0U) {
        relinquish_without_stop();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_WAIT_ZERO;
        return 0U;
    }

    if (g_feedback.world_lock_enabled != 0U) {
        relinquish_without_stop();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_WORLD_LOCKED;
        return 0U;
    }

    /* The legacy P23 tracker and YVT must never own YAW simultaneously. */
    GimbalTracker_Enable(0U);
    Gimbal_YawStopHold();
    g_enabled = 1U;
    g_trackingActive = 0U;
    g_targetSynced = 0U;
    g_sessionInitialized = 0U;
    g_feedback.enabled = 1U;
    g_feedback.target_valid = 0U;
    g_feedback.deadbanded = 0U;
    clear_command_feedback();
    sync_target_from_yaw();
    g_feedback.state = GIMBAL_VISION_YAW_WAIT_OBSERVATION;
    return 1U;
}

void GimbalVisionYawTracker_Update10ms(uint32_t localTimeMs)
{
    const GimbalVisionAdapterFeedback *adapter =
        GimbalVisionAdapter_GetFeedback();
    const GimbalTargetObservation *observation =
        GimbalVisionAdapter_GetObservation();
    const GimbalTrackerFeedback *legacyTracker =
        GimbalTracker_GetFeedback();
    VisionYawTuningParams tuning;
    uint32_t observationAgeMs = 0U;
    float speedDegS;
    float deltaDeg;

    VisionYawTuning_GetSnapshot(&tuning);
    g_feedback.update_count_10ms++;
    refresh_yaw_feedback();
    g_feedback.observation_available = adapter->output_available;
    g_feedback.observation_sequence = observation->sequence;
    g_feedback.session_id = adapter->session_id;
    g_feedback.error_x_px = observation->error_x_px;

    if (adapter->output_available != 0U) {
        observationAgeMs =
            localTimeMs - adapter->local_receive_timestamp_ms;
    }
    g_feedback.observation_age_ms = observationAgeMs;

    if (g_enabled == 0U) {
        g_feedback.enabled = 0U;
        return;
    }

    g_feedback.enabled = 1U;
    if (g_feedback.position_valid == 0U) {
        relinquish_without_stop();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_WAIT_ZERO;
        return;
    }

    if (g_feedback.world_lock_enabled != 0U) {
        relinquish_without_stop();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_WORLD_LOCKED;
        return;
    }

    if (legacyTracker->enabled != 0U) {
        stop_owned_tracking();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_PREEMPTED;
        return;
    }

    if (adapter->output_available == 0U) {
        stop_owned_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_WAIT_OBSERVATION;
        return;
    }

    if (observationAgeMs > tuning.observation_timeout_ms) {
        stop_owned_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_COMM_TIMEOUT;
        return;
    }

    if (observation->valid == 0U) {
        stop_owned_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_TARGET_LOST;
        return;
    }

    if ((g_sessionInitialized == 0U) ||
        (adapter->session_id != g_sessionId)) {
        if (g_sessionInitialized != 0U) {
            stop_owned_tracking();
        }
        g_sessionId = adapter->session_id;
        g_sessionInitialized = 1U;
        g_targetSynced = 0U;
        sync_target_from_yaw();
    }

    g_feedback.target_valid = 1U;
    if ((observation->error_x_px >= -(int16_t)tuning.deadband_px) &&
        (observation->error_x_px <= (int16_t)tuning.deadband_px)) {
        stop_owned_tracking();
        g_feedback.deadbanded = 1U;
        g_feedback.state = GIMBAL_VISION_YAW_CENTERED;
        return;
    }

    if (g_targetSynced == 0U) {
        sync_target_from_yaw();
    }

    speedDegS = VISION_YAW_ERROR_SIGN *
        ((float)tuning.kp_x1000 / 1000.0f) *
        (float)observation->error_x_px;
    speedDegS = clamp_f32(speedDegS,
        -((float)tuning.max_speed_deg_s_x10 / 10.0f),
        (float)tuning.max_speed_deg_s_x10 / 10.0f);

    if (speed_points_toward_limit(speedDegS,
            g_feedback.limit_direction) != 0U) {
        g_trackingActive = 0U;
        clear_command_feedback();
        sync_target_from_yaw();
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_LIMITED;
        return;
    }

    deltaDeg = clamp_f32(speedDegS * VISION_YAW_UPDATE_DT_S,
        -VISION_YAW_MAX_DELTA_DEG, VISION_YAW_MAX_DELTA_DEG);
    g_targetWrappedDeg = wrap_deg_360(g_targetWrappedDeg + deltaDeg);
    Gimbal_YawSetWrappedTargetDeg(g_targetWrappedDeg);

    refresh_yaw_feedback();
    if (g_feedback.limit_direction != 0) {
        g_trackingActive = 0U;
        clear_command_feedback();
        sync_target_from_yaw();
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_YAW_LIMITED;
        return;
    }

    g_trackingActive = 1U;
    g_feedback.tracking_active = 1U;
    g_feedback.deadbanded = 0U;
    g_feedback.command_speed_deg_s_x10 =
        float_to_scaled_i16(speedDegS, 10.0f);
    g_feedback.command_delta_deg_x1000 =
        float_to_scaled_i16(deltaDeg, 1000.0f);
    g_feedback.target_wrapped_deg_x10 =
        float_to_scaled_i16(g_targetWrappedDeg, 10.0f);
    g_feedback.state = GIMBAL_VISION_YAW_TRACKING;
}

const GimbalVisionYawFeedback *GimbalVisionYawTracker_GetFeedback(void)
{
    refresh_yaw_feedback();
    return &g_feedback;
}
