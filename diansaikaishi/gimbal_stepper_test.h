#ifndef GIMBAL_STEPPER_TEST_H
#define GIMBAL_STEPPER_TEST_H

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
} GimbalStepperTestFeedback;

void GimbalStepperTest_Init(void);
void GimbalStepperTest_Tick100us(void);
void GimbalStepperTest_SetStepHalfPeriodTicks(uint16_t half_period_ticks);
void GimbalStepperTest_MoveToEstimatedSteps(int64_t target_estimated_steps);
void GimbalStepperTest_MoveRelativeDeg(float delta_deg);
void GimbalStepperTest_StopHold(void);
void GimbalStepperTest_Release(void);
const GimbalStepperTestFeedback *GimbalStepperTest_GetFeedback(void);

#endif
