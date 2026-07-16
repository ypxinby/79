#ifndef GIMBAL_STEPPER_TEST_H
#define GIMBAL_STEPPER_TEST_H

#include <stdint.h>

typedef struct {
    int32_t target_steps;
    int32_t completed_steps;
    uint8_t running;
    uint8_t target_reached;
} GimbalStepperTestFeedback;

void GimbalStepperTest_Init(void);
void GimbalStepperTest_Tick100us(void);
const GimbalStepperTestFeedback *GimbalStepperTest_GetFeedback(void);

#endif
