#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Reset(void);
void Encoder_HandleGpioInterrupt(void);
void Encoder_GetAndClearPulseDeltas(int32_t *motorA, int32_t *motorB);

#endif
