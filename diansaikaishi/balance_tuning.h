#ifndef BALANCE_TUNING_H
#define BALANCE_TUNING_H

#include <stdint.h>

typedef struct {
    uint32_t telemetry_frame_count;
    uint32_t telemetry_drop_count;
    uint32_t rx_byte_count;
    uint32_t rx_line_count;
    uint32_t command_error_count;
    uint32_t rx_overflow_count;
    uint32_t tx_overflow_count;
} BalanceTuningStatus;

void BalanceTuning_Init(void);
void BalanceTuning_Process(void);
void BalanceTuning_Update20ms(uint32_t now_ms);
void BalanceTuning_GetStatus(BalanceTuningStatus *status);
void BalanceTuning_AbortCalibrationSession(void);

#endif
