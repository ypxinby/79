#ifndef BALANCE_SOFT_LIMITS_H
#define BALANCE_SOFT_LIMITS_H

#include <stdint.h>

typedef enum {
    BALANCE_SOFT_LIMIT_CAL_IDLE = 0,
    BALANCE_SOFT_LIMIT_CAL_LOW,
    BALANCE_SOFT_LIMIT_CAL_HIGH,
    BALANCE_SOFT_LIMIT_CAL_COMPLETE
} BalanceSoftLimitCalibrationStage;

typedef enum {
    BALANCE_SOFT_LIMIT_ERROR_NONE = 0,
    BALANCE_SOFT_LIMIT_ERROR_RUNNING,
    BALANCE_SOFT_LIMIT_ERROR_RANGE
} BalanceSoftLimitError;

typedef struct {
    int32_t zero_raw_count;
    int32_t current_logical_count;
    int32_t low_logical_count;
    int32_t high_logical_count;
    int32_t minimum_logical_count;
    int32_t maximum_logical_count;
    uint32_t clamp_count;
    int8_t last_clamp_direction;
    uint8_t zero_valid;
    uint8_t low_valid;
    uint8_t high_valid;
    uint8_t limits_valid;
    uint8_t calibration_active;
    BalanceSoftLimitCalibrationStage calibration_stage;
    BalanceSoftLimitError error;
} BalanceSoftLimitsRuntime;

void BalanceSoftLimits_Init(void);
uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void);
uint8_t BalanceSoftLimits_CaptureCurrentStage(void);
void BalanceSoftLimits_AbortCalibration(void);
uint8_t BalanceSoftLimits_IsCalibrationActive(void);
void BalanceSoftLimits_Enforce100usFromIsr(void);
void BalanceSoftLimits_GetSnapshot(BalanceSoftLimitsRuntime *snapshot);

#endif
