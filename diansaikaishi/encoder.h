#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENCODER_READ_ERROR_NONE = 0U,
    ENCODER_READ_ERROR_MOTOR_A_OVERFLOW = (1U << 0),
    ENCODER_READ_ERROR_MOTOR_B_OVERFLOW = (1U << 1),
    ENCODER_READ_ERROR_INVALID_ARGUMENT = (1U << 2)
} EncoderReadError;

void Encoder_Reset(void);
void Encoder_HandleGpioInterrupt(void);
uint8_t Encoder_GetAndClearPulseDeltas(int32_t *motorA, int32_t *motorB);

#endif
