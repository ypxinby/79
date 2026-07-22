#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_DUTY          (1000)

typedef struct {
    int16_t requested_motor_a;
    int16_t requested_motor_b;
    int16_t applied_motor_a;
    int16_t applied_motor_b;
    uint16_t pwm_compare_a;
    uint16_t pwm_compare_b;
    bool hardware_pwm_enabled;
    int8_t direction_a;
    int8_t direction_b;
} MotorRuntime;

void Motor_Init(void);
void Motor_SetSpeed(int16_t motorA, int16_t motorB);
void Motor_Stop(void);
void Motor_PwmTick100us(void);
const volatile MotorRuntime *Motor_GetRuntime(void);

#endif
