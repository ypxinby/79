#ifndef WHEEL_SPEED_ESTIMATOR_H
#define WHEEL_SPEED_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WHEEL_ESTIMATOR_ERROR_NONE = 0U,
    WHEEL_ESTIMATOR_ERROR_INVALID_PPR = (1UL << 0),
    WHEEL_ESTIMATOR_ERROR_INVALID_WHEEL_DIAMETER = (1UL << 1),
    WHEEL_ESTIMATOR_ERROR_INVALID_WHEEL_TRACK = (1UL << 2),
    WHEEL_ESTIMATOR_ERROR_INVALID_LEFT_DIRECTION = (1UL << 3),
    WHEEL_ESTIMATOR_ERROR_INVALID_RIGHT_DIRECTION = (1UL << 4),
    WHEEL_ESTIMATOR_ERROR_INVALID_DT = (1UL << 5),
    WHEEL_ESTIMATOR_ERROR_LEFT_ENCODER_OVERFLOW = (1UL << 6),
    WHEEL_ESTIMATOR_ERROR_RIGHT_ENCODER_OVERFLOW = (1UL << 7),
    WHEEL_ESTIMATOR_ERROR_LEFT_DIRECTION_OVERFLOW = (1UL << 8),
    WHEEL_ESTIMATOR_ERROR_RIGHT_DIRECTION_OVERFLOW = (1UL << 9),
    WHEEL_ESTIMATOR_ERROR_LEFT_TOTAL_OVERFLOW = (1UL << 10),
    WHEEL_ESTIMATOR_ERROR_RIGHT_TOTAL_OVERFLOW = (1UL << 11)
} WheelSpeedEstimatorError;

typedef struct {
    int32_t left_delta_pulse;
    int32_t right_delta_pulse;
    int64_t left_total_pulse;
    int64_t right_total_pulse;
    float left_raw_speed_cmps;
    float right_raw_speed_cmps;
    float left_speed_cmps;
    float right_speed_cmps;
    float left_total_distance_cm;
    float right_total_distance_cm;
    float center_distance_cm;
    uint32_t sample_dt_ms;
    uint32_t update_count;
    uint32_t encoder_ppr_x2;
    float wheel_diameter_cm;
    float wheel_track_cm;
    int8_t left_encoder_direction;
    int8_t right_encoder_direction;
    bool valid;
    bool stale;
    bool overflow;
    uint32_t error_flags;
} WheelSpeedEstimatorRuntime;

extern volatile WheelSpeedEstimatorRuntime g_wheelSpeedEstimatorRuntime;

void WheelSpeedEstimator_Init(void);
void WheelSpeedEstimator_Update(uint32_t elapsed_ms);
const volatile WheelSpeedEstimatorRuntime *WheelSpeedEstimator_GetRuntime(void);

#endif
