#ifndef GIMBAL_TRACKER_H
#define GIMBAL_TRACKER_H

#include <stdint.h>

typedef struct {
    int16_t error_x_px;
    int16_t error_y_px;
    uint8_t valid;
    uint16_t sequence;
    uint32_t timestamp_ms;
} GimbalTargetObservation;

typedef struct {
    uint8_t enabled;
    uint8_t target_valid;
    uint8_t tracking_active;
    uint8_t filter_ready;
    uint8_t yaw_deadbanded;
    uint8_t pitch_deadbanded;
    uint8_t command_limited;
    uint16_t observation_sequence;
    uint32_t observation_timestamp_ms;
    uint32_t update_count_10ms;
    int16_t raw_error_x_px;
    int16_t raw_error_y_px;
    int16_t filtered_error_x_px;
    int16_t filtered_error_y_px;
    int16_t yaw_speed_deg_s_x10;
    int16_t pitch_speed_deg_s_x10;
    int16_t yaw_delta_deg_x10;
    int16_t pitch_delta_deg_x10;
    int16_t yaw_target_deg_x10;
    int16_t pitch_target_deg_x10;
} GimbalTrackerFeedback;

void GimbalTracker_Init(void);
void GimbalTracker_Enable(uint8_t enable);
void GimbalTracker_PushObservation(
    const GimbalTargetObservation *observation);
void GimbalTracker_ClearObservation(void);
void GimbalTracker_Update(float dt_s);
const GimbalTrackerFeedback *GimbalTracker_GetFeedback(void);

#endif
