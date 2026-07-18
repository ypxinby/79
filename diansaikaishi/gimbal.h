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
    int16_t car_yaw_deg_x10;
    int16_t locked_world_yaw_deg_x10;
    int16_t world_target_deg_x10;
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
    uint8_t world_lock_enabled;
} GimbalFeedback;

/* P11 public yaw-axis interface. New business code should use this group. */
void Gimbal_YawInit(void);
void Gimbal_YawTick100us(void);
void Gimbal_YawUpdate5ms(void);
void Gimbal_YawMoveToDeg(float target_deg);
void Gimbal_YawMoveWrappedDeg(float target_deg);
void Gimbal_YawSetWrappedTargetDeg(float target_deg);
void Gimbal_YawMoveRelativeDeg(float delta_deg);
void Gimbal_YawEnableWorldLock(void);
void Gimbal_YawDisableWorldLock(void);
void Gimbal_YawToggleWorldLock(void);
void Gimbal_YawStopHold(void);
void Gimbal_YawRelease(void);
const GimbalFeedback *Gimbal_YawGetFeedback(void);

/*
 * P18 pitch-axis interface for the second motor.
 * PB5/PB6/PB7 drive STEP/DIR/EN through the low-level implementation.
 */
void Gimbal_PitchInit(void);
void Gimbal_PitchTick100us(void);
void Gimbal_PitchUpdate5ms(void);
void Gimbal_PitchMoveToDeg(float target_deg);
void Gimbal_PitchSetTargetDeg(float target_deg);
void Gimbal_PitchMoveRelativeDeg(float delta_deg);
void Gimbal_PitchStopHold(void);
void Gimbal_PitchRelease(void);
const GimbalFeedback *Gimbal_PitchGetFeedback(void);

/* P19 unified two-axis interface for future vision tracking code. */
void Gimbal_Init(void);
void Gimbal_Tick100us(void);
void Gimbal_Update5ms(void);
void Gimbal_MoveToYawPitchDeg(float yaw_deg, float pitch_deg);
void Gimbal_MoveWrappedYawPitchDeg(float yaw_wrapped_deg,
    float pitch_deg);
void Gimbal_SetWrappedYawPitchTargetDeg(float yaw_wrapped_deg,
    float pitch_deg);
void Gimbal_MoveRelativeYawPitchDeg(float yaw_delta_deg,
    float pitch_delta_deg);
void Gimbal_StopHoldAll(void);
void Gimbal_ReleaseAll(void);

/* Compatibility wrappers kept for early single-axis yaw test code. */
void Gimbal_MoveToDeg(float target_deg);
void Gimbal_MoveWrappedDeg(float target_deg);
void Gimbal_SetWrappedTargetDeg(float target_deg);
void Gimbal_MoveRelativeDeg(float delta_deg);
void Gimbal_EnableWorldLock(void);
void Gimbal_DisableWorldLock(void);
void Gimbal_ToggleWorldLock(void);
void Gimbal_StopHold(void);
void Gimbal_Release(void);
const GimbalFeedback *Gimbal_GetFeedback(void);

#endif
