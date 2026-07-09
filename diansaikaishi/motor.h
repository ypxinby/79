#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_MAX_DUTY          (1000)

void Motor_Init(void);
void Motor_SetSpeed(int16_t motorA, int16_t motorB);
void Motor_Stop(void);
void Motor_PwmTick100us(void);

#endif
