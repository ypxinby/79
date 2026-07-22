#include "wheel_speed_estimator.h"

#include <limits.h>

#include "app_config.h"
#include "encoder.h"

#define WHEEL_PI                       (3.14159265358979323846f)
#define WHEEL_SPEED_FILTER_ALPHA       (0.35f)
#define WHEEL_ESTIMATOR_DT_MAX_MS      (100U)
#define WHEEL_ESTIMATOR_STALE_MS       (60U)
#define WHEEL_DIMENSION_MAX_CM         (100.0f)

#define WHEEL_ESTIMATOR_OVERFLOW_MASK  \
    (WHEEL_ESTIMATOR_ERROR_LEFT_ENCODER_OVERFLOW | \
        WHEEL_ESTIMATOR_ERROR_RIGHT_ENCODER_OVERFLOW | \
        WHEEL_ESTIMATOR_ERROR_LEFT_DIRECTION_OVERFLOW | \
        WHEEL_ESTIMATOR_ERROR_RIGHT_DIRECTION_OVERFLOW | \
        WHEEL_ESTIMATOR_ERROR_LEFT_TOTAL_OVERFLOW | \
        WHEEL_ESTIMATOR_ERROR_RIGHT_TOTAL_OVERFLOW)

#define WHEEL_ESTIMATOR_METRIC_CONFIG_ERROR_MASK  \
    (WHEEL_ESTIMATOR_ERROR_INVALID_PPR | \
        WHEEL_ESTIMATOR_ERROR_INVALID_WHEEL_DIAMETER | \
        WHEEL_ESTIMATOR_ERROR_INVALID_LEFT_DIRECTION | \
        WHEEL_ESTIMATOR_ERROR_INVALID_RIGHT_DIRECTION)

volatile WheelSpeedEstimatorRuntime g_wheelSpeedEstimatorRuntime;

static bool g_leftFilterInitialized;
static bool g_rightFilterInitialized;
static uint32_t g_latchedErrorFlags;

static bool direction_is_valid(int8_t direction)
{
    return (direction == 1) || (direction == -1);
}

static uint32_t get_config_error_flags(void)
{
    uint32_t errors = WHEEL_ESTIMATOR_ERROR_NONE;

    if (g_appConfig.encoder_ppr_x2 == 0U) {
        errors |= WHEEL_ESTIMATOR_ERROR_INVALID_PPR;
    }
    if (!((g_appConfig.wheel_diameter_cm > 0.0f) &&
        (g_appConfig.wheel_diameter_cm <= WHEEL_DIMENSION_MAX_CM))) {
        errors |= WHEEL_ESTIMATOR_ERROR_INVALID_WHEEL_DIAMETER;
    }
    if (!((g_appConfig.wheel_track_cm > 0.0f) &&
        (g_appConfig.wheel_track_cm <= WHEEL_DIMENSION_MAX_CM))) {
        errors |= WHEEL_ESTIMATOR_ERROR_INVALID_WHEEL_TRACK;
    }
    if (!direction_is_valid(g_appConfig.left_encoder_direction)) {
        errors |= WHEEL_ESTIMATOR_ERROR_INVALID_LEFT_DIRECTION;
    }
    if (!direction_is_valid(g_appConfig.right_encoder_direction)) {
        errors |= WHEEL_ESTIMATOR_ERROR_INVALID_RIGHT_DIRECTION;
    }

    return errors;
}

static void update_config_snapshot(void)
{
    g_wheelSpeedEstimatorRuntime.encoder_ppr_x2 =
        g_appConfig.encoder_ppr_x2;
    g_wheelSpeedEstimatorRuntime.wheel_diameter_cm =
        g_appConfig.wheel_diameter_cm;
    g_wheelSpeedEstimatorRuntime.wheel_track_cm =
        g_appConfig.wheel_track_cm;
    g_wheelSpeedEstimatorRuntime.left_encoder_direction =
        g_appConfig.left_encoder_direction;
    g_wheelSpeedEstimatorRuntime.right_encoder_direction =
        g_appConfig.right_encoder_direction;
}

static bool apply_direction(
    int32_t rawDelta, int8_t direction, int32_t *correctedDelta)
{
    if (!direction_is_valid(direction)) {
        *correctedDelta = rawDelta;
        return false;
    }
    if ((direction < 0) && (rawDelta == INT32_MIN)) {
        *correctedDelta = INT32_MAX;
        return false;
    }

    *correctedDelta = (direction < 0) ? -rawDelta : rawDelta;
    return true;
}

static bool add_total_pulse(volatile int64_t *total, int32_t delta)
{
    int64_t current = *total;
    int64_t delta64 = (int64_t)delta;

    if ((delta64 > 0) && (current > (INT64_MAX - delta64))) {
        *total = INT64_MAX;
        return false;
    }
    if ((delta64 < 0) && (current < (INT64_MIN - delta64))) {
        *total = INT64_MIN;
        return false;
    }

    *total = current + delta64;
    return true;
}

static float filter_speed(float rawSpeed, bool *initialized, float previous)
{
    if (!*initialized) {
        *initialized = true;
        return rawSpeed;
    }

    return previous + WHEEL_SPEED_FILTER_ALPHA * (rawSpeed - previous);
}

void WheelSpeedEstimator_Init(void)
{
    WheelSpeedEstimatorRuntime initial = {0};

    g_wheelSpeedEstimatorRuntime = initial;
    g_leftFilterInitialized = false;
    g_rightFilterInitialized = false;
    g_latchedErrorFlags = WHEEL_ESTIMATOR_ERROR_NONE;
    update_config_snapshot();
    g_wheelSpeedEstimatorRuntime.stale = true;
    g_wheelSpeedEstimatorRuntime.error_flags = get_config_error_flags();
}

void WheelSpeedEstimator_Update(uint32_t elapsed_ms)
{
    int32_t rawLeftDelta;
    int32_t rawRightDelta;
    int32_t correctedLeftDelta;
    int32_t correctedRightDelta;
    uint8_t encoderErrors;
    uint32_t currentErrors;
    bool leftDirectionValid;
    bool rightDirectionValid;
    bool leftSampleValid;
    bool rightSampleValid;
    bool dtValid;
    bool metricConfigValid;
    float distancePerPulseCm = 0.0f;

    encoderErrors = Encoder_GetAndClearPulseDeltas(
        &rawLeftDelta, &rawRightDelta);
    update_config_snapshot();
    g_wheelSpeedEstimatorRuntime.sample_dt_ms = elapsed_ms;
    g_wheelSpeedEstimatorRuntime.update_count++;

    currentErrors = get_config_error_flags();
    if ((encoderErrors & ENCODER_READ_ERROR_MOTOR_A_OVERFLOW) != 0U) {
        g_latchedErrorFlags |=
            WHEEL_ESTIMATOR_ERROR_LEFT_ENCODER_OVERFLOW;
    }
    if ((encoderErrors & ENCODER_READ_ERROR_MOTOR_B_OVERFLOW) != 0U) {
        g_latchedErrorFlags |=
            WHEEL_ESTIMATOR_ERROR_RIGHT_ENCODER_OVERFLOW;
    }

    leftDirectionValid = apply_direction(rawLeftDelta,
        g_appConfig.left_encoder_direction,
        &correctedLeftDelta);
    rightDirectionValid = apply_direction(rawRightDelta,
        g_appConfig.right_encoder_direction,
        &correctedRightDelta);
    g_wheelSpeedEstimatorRuntime.left_delta_pulse = correctedLeftDelta;
    g_wheelSpeedEstimatorRuntime.right_delta_pulse = correctedRightDelta;
    if (!leftDirectionValid &&
        direction_is_valid(g_appConfig.left_encoder_direction)) {
        g_latchedErrorFlags |=
            WHEEL_ESTIMATOR_ERROR_LEFT_DIRECTION_OVERFLOW;
    }
    if (!rightDirectionValid &&
        direction_is_valid(g_appConfig.right_encoder_direction)) {
        g_latchedErrorFlags |=
            WHEEL_ESTIMATOR_ERROR_RIGHT_DIRECTION_OVERFLOW;
    }

    leftSampleValid = leftDirectionValid &&
        ((encoderErrors & ENCODER_READ_ERROR_MOTOR_A_OVERFLOW) == 0U);
    rightSampleValid = rightDirectionValid &&
        ((encoderErrors & ENCODER_READ_ERROR_MOTOR_B_OVERFLOW) == 0U);

    metricConfigValid = (currentErrors &
        WHEEL_ESTIMATOR_METRIC_CONFIG_ERROR_MASK) == 0U;

    if (leftSampleValid && !add_total_pulse(
        &g_wheelSpeedEstimatorRuntime.left_total_pulse,
        g_wheelSpeedEstimatorRuntime.left_delta_pulse)) {
        g_latchedErrorFlags |= WHEEL_ESTIMATOR_ERROR_LEFT_TOTAL_OVERFLOW;
    }
    if (rightSampleValid && !add_total_pulse(
        &g_wheelSpeedEstimatorRuntime.right_total_pulse,
        g_wheelSpeedEstimatorRuntime.right_delta_pulse)) {
        g_latchedErrorFlags |= WHEEL_ESTIMATOR_ERROR_RIGHT_TOTAL_OVERFLOW;
    }

    dtValid = (elapsed_ms > 0U) &&
        (elapsed_ms <= WHEEL_ESTIMATOR_DT_MAX_MS);
    if (!dtValid) {
        currentErrors |= WHEEL_ESTIMATOR_ERROR_INVALID_DT;
    }

    if (metricConfigValid) {
        distancePerPulseCm =
            (WHEEL_PI * g_appConfig.wheel_diameter_cm) /
            (float)g_appConfig.encoder_ppr_x2;
        g_wheelSpeedEstimatorRuntime.left_total_distance_cm =
            (float)g_wheelSpeedEstimatorRuntime.left_total_pulse *
            distancePerPulseCm;
        g_wheelSpeedEstimatorRuntime.right_total_distance_cm =
            (float)g_wheelSpeedEstimatorRuntime.right_total_pulse *
            distancePerPulseCm;
        g_wheelSpeedEstimatorRuntime.center_distance_cm =
            (g_wheelSpeedEstimatorRuntime.left_total_distance_cm +
                g_wheelSpeedEstimatorRuntime.right_total_distance_cm) * 0.5f;
    } else {
        g_wheelSpeedEstimatorRuntime.left_total_distance_cm = 0.0f;
        g_wheelSpeedEstimatorRuntime.right_total_distance_cm = 0.0f;
        g_wheelSpeedEstimatorRuntime.center_distance_cm = 0.0f;
    }

    if (metricConfigValid && dtValid && leftSampleValid) {
        g_wheelSpeedEstimatorRuntime.left_raw_speed_cmps =
            (float)g_wheelSpeedEstimatorRuntime.left_delta_pulse *
            distancePerPulseCm * 1000.0f / (float)elapsed_ms;
        g_wheelSpeedEstimatorRuntime.left_speed_cmps = filter_speed(
            g_wheelSpeedEstimatorRuntime.left_raw_speed_cmps,
            &g_leftFilterInitialized,
            g_wheelSpeedEstimatorRuntime.left_speed_cmps);
    } else {
        g_wheelSpeedEstimatorRuntime.left_raw_speed_cmps = 0.0f;
    }
    if (metricConfigValid && dtValid && rightSampleValid) {
        g_wheelSpeedEstimatorRuntime.right_raw_speed_cmps =
            (float)g_wheelSpeedEstimatorRuntime.right_delta_pulse *
            distancePerPulseCm * 1000.0f / (float)elapsed_ms;
        g_wheelSpeedEstimatorRuntime.right_speed_cmps = filter_speed(
            g_wheelSpeedEstimatorRuntime.right_raw_speed_cmps,
            &g_rightFilterInitialized,
            g_wheelSpeedEstimatorRuntime.right_speed_cmps);
    } else {
        g_wheelSpeedEstimatorRuntime.right_raw_speed_cmps = 0.0f;
    }

    g_wheelSpeedEstimatorRuntime.stale = !dtValid ||
        (elapsed_ms > WHEEL_ESTIMATOR_STALE_MS);
    g_wheelSpeedEstimatorRuntime.error_flags =
        currentErrors | g_latchedErrorFlags;
    g_wheelSpeedEstimatorRuntime.overflow =
        (g_wheelSpeedEstimatorRuntime.error_flags &
            WHEEL_ESTIMATOR_OVERFLOW_MASK) != 0U;
    g_wheelSpeedEstimatorRuntime.valid =
        (g_wheelSpeedEstimatorRuntime.error_flags ==
            WHEEL_ESTIMATOR_ERROR_NONE) &&
        !g_wheelSpeedEstimatorRuntime.stale;
}

const volatile WheelSpeedEstimatorRuntime *WheelSpeedEstimator_GetRuntime(void)
{
    return &g_wheelSpeedEstimatorRuntime;
}
