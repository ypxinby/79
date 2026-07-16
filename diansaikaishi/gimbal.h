#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef struct {
    int32_t target_steps;
    int32_t completed_steps;
    int16_t target_deg_x10;
    int16_t completed_deg_x10;
    uint32_t control_tick_5ms;
    int8_t direction;
    uint8_t running;
    uint8_t target_reached;
} GimbalFeedback;

void Gimbal_Init(void);
void Gimbal_Tick100us(void);
void Gimbal_Update5ms(void);
void Gimbal_MoveRelativeDeg(float delta_deg);
const GimbalFeedback *Gimbal_GetFeedback(void);

#endif
