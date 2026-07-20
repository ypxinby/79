#ifndef GIMBAL_VISION_YAW_TRACKER_H
#define GIMBAL_VISION_YAW_TRACKER_H

#include <stdint.h>

typedef enum {
    GIMBAL_VISION_YAW_DISABLED = 0,
    GIMBAL_VISION_YAW_WAIT_ZERO,
    GIMBAL_VISION_YAW_WAIT_OBSERVATION,
    GIMBAL_VISION_YAW_TRACKING,
    GIMBAL_VISION_YAW_CENTERED,
    GIMBAL_VISION_YAW_TARGET_LOST,
    GIMBAL_VISION_YAW_COMM_TIMEOUT,
    GIMBAL_VISION_YAW_LIMITED,
    GIMBAL_VISION_YAW_WORLD_LOCKED,
    GIMBAL_VISION_YAW_PREEMPTED
} GimbalVisionYawState;

typedef struct {
    uint8_t enabled;
    uint8_t target_valid;
    uint8_t tracking_active;
    uint8_t deadbanded;
    uint8_t position_valid;
    uint8_t world_lock_enabled;
    uint8_t observation_available;
    uint8_t positive_limit;
    uint8_t negative_limit;
    uint16_t observation_sequence;
    uint32_t observation_age_ms;
    uint32_t session_id;
    uint32_t update_count_10ms;
    int16_t error_x_px;
    int16_t command_speed_deg_s_x10;
    int16_t command_delta_deg_x1000;
    int16_t target_wrapped_deg_x10;
    int16_t current_wrapped_deg_x10;
    int8_t limit_direction;
    GimbalVisionYawState state;
} GimbalVisionYawFeedback;

void GimbalVisionYawTracker_Init(void);
uint8_t GimbalVisionYawTracker_Enable(uint8_t enable);
void GimbalVisionYawTracker_Update10ms(uint32_t localTimeMs);
const GimbalVisionYawFeedback *GimbalVisionYawTracker_GetFeedback(void);

#endif
