#ifndef GIMBAL_STEPPER_H
#define GIMBAL_STEPPER_H

#include <stdint.h>

typedef struct {
    int64_t estimated_steps;
    int32_t target_steps;
    int32_t completed_steps;
    uint16_t step_half_period_ticks;
    int8_t direction;
    uint8_t enabled;
    uint8_t running;
    uint8_t target_reached;
} GimbalStepperFeedback;

void GimbalStepper_Init(void);
void GimbalStepper_Tick100us(void);
void GimbalStepper_SetStepHalfPeriodTicks(uint16_t half_period_ticks);
void GimbalStepper_MoveToEstimatedSteps(int64_t target_estimated_steps);
void GimbalStepper_MoveRelativeDeg(float delta_deg);
uint8_t GimbalStepper_ConfirmZero(void);
void GimbalStepper_StopHold(void);
void GimbalStepper_Release(void);
const GimbalStepperFeedback *GimbalStepper_GetFeedback(void);

#endif
