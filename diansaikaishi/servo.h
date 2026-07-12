#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t target_angle_deg;
    uint8_t pulse_ticks_100us;
    bool output_high;
} ServoFeedback;

void Servo_Init(void);
void Servo_SetAngleDeg(int16_t angle_deg);
void Servo_Tick100us(void);
const ServoFeedback *Servo_GetFeedback(void);

#endif
