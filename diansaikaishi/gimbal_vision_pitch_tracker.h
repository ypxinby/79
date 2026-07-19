#ifndef GIMBAL_VISION_PITCH_TRACKER_H
#define GIMBAL_VISION_PITCH_TRACKER_H

#include <stdint.h>

typedef enum {
    GIMBAL_VISION_PITCH_DISABLED = 0,
    GIMBAL_VISION_PITCH_WAIT_ZERO,
    GIMBAL_VISION_PITCH_WAIT_OBSERVATION,
    GIMBAL_VISION_PITCH_TRACKING,
    GIMBAL_VISION_PITCH_CENTERED,
    GIMBAL_VISION_PITCH_TARGET_LOST,
    GIMBAL_VISION_PITCH_STALE,
    GIMBAL_VISION_PITCH_LIMITED,
    GIMBAL_VISION_PITCH_PREEMPTED
} GimbalVisionPitchState;

typedef struct {
    uint8_t enabled;
    uint8_t target_valid;
    uint8_t tracking_active;
    uint8_t deadbanded;
    uint8_t position_valid;
    uint8_t observation_available;
    uint16_t observation_sequence;
    uint32_t observation_age_ms;
    uint32_t update_count_10ms;
    int16_t error_y_px;
    int16_t command_speed_deg_s_x10;
    int16_t pitch_angle_deg_x10;
    int8_t limit_direction;
    GimbalVisionPitchState state;
} GimbalVisionPitchFeedback;

void GimbalVisionPitchTracker_Init(void);
uint8_t GimbalVisionPitchTracker_Enable(uint8_t enable);
void GimbalVisionPitchTracker_Update10ms(uint32_t localTimeMs);
const GimbalVisionPitchFeedback *GimbalVisionPitchTracker_GetFeedback(void);

#endif
