#ifndef BALANCE_ENCODER_H
#define BALANCE_ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t count;
    uint32_t valid_transition_count;
    uint32_t invalid_transition_count;
    uint32_t overflow_count;
    int32_t speed_sample_delta_count;
    uint32_t speed_sample_count;
    uint8_t ab_state;
    int8_t direction;
    uint8_t initialized;
} BalanceEncoderRuntime;

void BalanceEncoder_Init(void);
void BalanceEncoder_Reset(void);
void BalanceEncoder_HandleGpioInterrupt(void);
void BalanceEncoder_SampleSpeed10msFromIsr(void);
void BalanceEncoder_GetSnapshot(BalanceEncoderRuntime *snapshot);
uint8_t BalanceEncoder_CalculateFollowError(int64_t step_count,
    int32_t encoder_count, int32_t *error_count);
int32_t BalanceEncoder_CalculateSpeedRpmX10(int32_t sample_delta_count);

#endif
