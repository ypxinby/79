#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef enum {
    GIMBAL_MODE_RELEASED = 0,
    GIMBAL_MODE_HOLDING,
    GIMBAL_MODE_MOVING
} GimbalMode;

typedef struct {
    int32_t target_steps;
    int32_t completed_steps;
    int16_t target_deg_x10;
    int16_t completed_deg_x10;
    uint32_t control_tick_5ms;
    GimbalMode mode;
    int8_t direction;
    uint8_t enabled;
    uint8_t running;
    uint8_t target_reached;
} GimbalFeedback;

/* P8 public pitch-axis interface. New business code should use this group. */
void Gimbal_PitchInit(void);
void Gimbal_PitchTick100us(void);
void Gimbal_PitchUpdate5ms(void);
void Gimbal_PitchMoveRelativeDeg(float delta_deg);
void Gimbal_PitchStopHold(void);
void Gimbal_PitchRelease(void);
const GimbalFeedback *Gimbal_PitchGetFeedback(void);

/* Compatibility wrappers kept for early test code. */
void Gimbal_Init(void);
void Gimbal_Tick100us(void);
void Gimbal_Update5ms(void);
void Gimbal_MoveRelativeDeg(float delta_deg);
void Gimbal_StopHold(void);
void Gimbal_Release(void);
const GimbalFeedback *Gimbal_GetFeedback(void);

#endif
