#include "gimbal_vision_pitch_tracker.h"

#include "gimbal.h"
#include "gimbal_tracker.h"
#include "gimbal_vision_adapter.h"
#include "vision_pitch_tuning.h"

#define VISION_PITCH_MIN_SPEED_DEG_S         (3.0f)
#define VISION_PITCH_ERROR_SIGN              (1.0f)

static GimbalVisionPitchFeedback g_feedback;
static uint8_t g_enabled;
static uint8_t g_trackingActive;

static int16_t speed_to_x10(float speedDegS)
{
    float value = speedDegS * 10.0f;

    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static float clamp_speed(float speedDegS, float maxSpeedDegS)
{
    if (speedDegS > maxSpeedDegS) {
        return maxSpeedDegS;
    }
    if (speedDegS < -maxSpeedDegS) {
        return -maxSpeedDegS;
    }
    if ((speedDegS > 0.0f) &&
        (speedDegS < VISION_PITCH_MIN_SPEED_DEG_S)) {
        return VISION_PITCH_MIN_SPEED_DEG_S;
    }
    if ((speedDegS < 0.0f) &&
        (speedDegS > -VISION_PITCH_MIN_SPEED_DEG_S)) {
        return -VISION_PITCH_MIN_SPEED_DEG_S;
    }
    return speedDegS;
}

static uint8_t error_is_deadbanded(int16_t errorY, uint16_t deadbandPx)
{
    int32_t deadband = (int32_t)deadbandPx;

    return (((int32_t)errorY >= -deadband) &&
        ((int32_t)errorY <= deadband)) ? 1U : 0U;
}

static void stop_tracking(void)
{
    if (g_trackingActive != 0U) {
        Gimbal_PitchStopTrackingHold();
    }
    g_trackingActive = 0U;
    g_feedback.tracking_active = 0U;
    g_feedback.command_speed_deg_s_x10 = 0;
}

static void refresh_pitch_feedback(void)
{
    const GimbalFeedback *pitch = Gimbal_PitchGetFeedback();

    g_feedback.position_valid = pitch->position_valid;
    g_feedback.pitch_angle_deg_x10 = pitch->continuous_deg_x10;
    g_feedback.limit_direction = pitch->limit_direction;
}

void GimbalVisionPitchTracker_Init(void)
{
    g_feedback = (GimbalVisionPitchFeedback){0};
    g_feedback.state = GIMBAL_VISION_PITCH_DISABLED;
    g_enabled = 0U;
    g_trackingActive = 0U;
    refresh_pitch_feedback();
}

uint8_t GimbalVisionPitchTracker_Enable(uint8_t enable)
{
    if (enable == 0U) {
        stop_tracking();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_DISABLED;
        refresh_pitch_feedback();
        return 1U;
    }

    refresh_pitch_feedback();
    if (g_feedback.position_valid == 0U) {
        stop_tracking();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_WAIT_ZERO;
        return 0U;
    }

    /* Visual PITCH owns only the pitch speed path; stop the legacy tracker. */
    GimbalTracker_Enable(0U);
    Gimbal_PitchStopTrackingHold();
    g_enabled = 1U;
    g_trackingActive = 0U;
    g_feedback.enabled = 1U;
    g_feedback.target_valid = 0U;
    g_feedback.deadbanded = 0U;
    g_feedback.command_speed_deg_s_x10 = 0;
    g_feedback.state = GIMBAL_VISION_PITCH_WAIT_OBSERVATION;
    return 1U;
}

void GimbalVisionPitchTracker_Update10ms(uint32_t localTimeMs)
{
    const GimbalVisionAdapterFeedback *adapter =
        GimbalVisionAdapter_GetFeedback();
    const GimbalTargetObservation *observation =
        GimbalVisionAdapter_GetObservation();
    const GimbalTrackerFeedback *legacyTracker =
        GimbalTracker_GetFeedback();
    VisionPitchTuningParams tuning;
    uint32_t observationAgeMs = 0U;

    VisionPitchTuning_GetSnapshot(&tuning);
    g_feedback.update_count_10ms++;
    refresh_pitch_feedback();
    g_feedback.observation_available = adapter->output_available;
    g_feedback.observation_sequence = observation->sequence;
    g_feedback.error_y_px = observation->error_y_px;

    if (adapter->output_available != 0U) {
        observationAgeMs =
            localTimeMs - adapter->local_receive_timestamp_ms;
    }
    g_feedback.observation_age_ms = observationAgeMs;

    if (g_enabled == 0U) {
        g_feedback.enabled = 0U;
        if (g_feedback.state != GIMBAL_VISION_PITCH_WAIT_ZERO) {
            g_feedback.state = GIMBAL_VISION_PITCH_DISABLED;
        }
        return;
    }

    if (legacyTracker->enabled != 0U) {
        stop_tracking();
        g_enabled = 0U;
        g_feedback.enabled = 0U;
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_PREEMPTED;
        return;
    }

    g_feedback.enabled = 1U;
    if (g_feedback.position_valid == 0U) {
        stop_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_WAIT_ZERO;
        return;
    }

    if (adapter->output_available == 0U) {
        stop_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_WAIT_OBSERVATION;
        return;
    }

    if (observationAgeMs > tuning.observation_timeout_ms) {
        stop_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_STALE;
        return;
    }

    if (observation->valid == 0U) {
        stop_tracking();
        g_feedback.target_valid = 0U;
        g_feedback.deadbanded = 0U;
        g_feedback.state = GIMBAL_VISION_PITCH_TARGET_LOST;
        return;
    }

    g_feedback.target_valid = 1U;
    if (error_is_deadbanded(observation->error_y_px,
            tuning.deadband_px) != 0U) {
        stop_tracking();
        g_feedback.deadbanded = 1U;
        g_feedback.state = GIMBAL_VISION_PITCH_CENTERED;
        return;
    }

    {
        float kpDegSPerPx = (float)tuning.kp_x1000 / 1000.0f;
        float maxSpeedDegS =
            (float)tuning.max_speed_deg_s_x10 / 10.0f;
        float speedDegS = clamp_speed(VISION_PITCH_ERROR_SIGN *
            kpDegSPerPx * (float)observation->error_y_px,
            maxSpeedDegS);

        g_feedback.deadbanded = 0U;
        Gimbal_PitchSetTrackingSpeedDegS(speedDegS);
        g_trackingActive = 1U;
        g_feedback.tracking_active = 1U;
        g_feedback.command_speed_deg_s_x10 = speed_to_x10(speedDegS);
        refresh_pitch_feedback();
        if (g_feedback.limit_direction != 0) {
            g_feedback.state = GIMBAL_VISION_PITCH_LIMITED;
        } else {
            g_feedback.state = GIMBAL_VISION_PITCH_TRACKING;
        }
    }
}

const GimbalVisionPitchFeedback *GimbalVisionPitchTracker_GetFeedback(void)
{
    refresh_pitch_feedback();
    return &g_feedback;
}
