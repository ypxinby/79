#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum {
    GIMBAL_MODE_RELEASED = 0,
    GIMBAL_MODE_HOLDING,
    GIMBAL_MODE_MOVING
} GimbalMode;

typedef struct {
    int64_t estimated_steps;
    int32_t target_steps;
    int32_t completed_steps;
    int16_t target_deg_x10;
    int16_t completed_deg_x10;
    int16_t continuous_deg_x10;
    int16_t wrapped_deg_x10;
    int16_t target_rpm_x10;
    int16_t commanded_rpm_x10;
    int32_t turn_count;
    uint16_t step_half_period_ticks;
    int16_t min_limit_deg_x10;
    int16_t max_limit_deg_x10;
    uint32_t control_tick_5ms;
    GimbalMode mode;
    int8_t direction;
    uint8_t enabled;
    uint8_t running;
    uint8_t target_reached;
    uint8_t limit_clamped;
    uint8_t position_valid;
} GimbalFeedback;

/* P11 public yaw-axis interface. New business code should use this group. */
void Gimbal_YawInit(void);
void Gimbal_YawTick100us(void);
void Gimbal_YawUpdate5ms(void);
void Gimbal_YawMoveToDeg(float target_deg);
void Gimbal_YawMoveWrappedDeg(float target_deg);
void Gimbal_YawMoveRelativeDeg(float delta_deg);
void Gimbal_YawStopHold(void);
void Gimbal_YawRelease(void);
const GimbalFeedback *Gimbal_YawGetFeedback(void);

/* Compatibility wrappers kept for early single-axis test code. */
void Gimbal_Init(void);
void Gimbal_Tick100us(void);
void Gimbal_Update5ms(void);
void Gimbal_MoveToDeg(float target_deg);
void Gimbal_MoveWrappedDeg(float target_deg);
void Gimbal_MoveRelativeDeg(float delta_deg);
void Gimbal_StopHold(void);
void Gimbal_Release(void);
const GimbalFeedback *Gimbal_GetFeedback(void);

#endif
