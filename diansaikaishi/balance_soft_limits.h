#ifndef BALANCE_SOFT_LIMITS_H
#define BALANCE_SOFT_LIMITS_H

#include <stdint.h>

#include "balance_calibration_store.h"

typedef enum {
    BALANCE_SOFT_LIMIT_CAL_IDLE = 0,
    BALANCE_SOFT_LIMIT_CAL_LOW,
    BALANCE_SOFT_LIMIT_CAL_HIGH,
    BALANCE_SOFT_LIMIT_CAL_COMPLETE
} BalanceSoftLimitCalibrationStage;

typedef enum {
    BALANCE_SOFT_LIMIT_ERROR_NONE = 0,
    BALANCE_SOFT_LIMIT_ERROR_RUNNING,
    BALANCE_SOFT_LIMIT_ERROR_RANGE,
    BALANCE_SOFT_LIMIT_ERROR_FLASH
} BalanceSoftLimitError;

typedef enum {
    BALANCE_SOFT_LIMIT_TEST_IDLE = 0,
    BALANCE_SOFT_LIMIT_TEST_TO_LOW,
    BALANCE_SOFT_LIMIT_TEST_TO_HIGH,
    BALANCE_SOFT_LIMIT_TEST_TO_ZERO,
    BALANCE_SOFT_LIMIT_TEST_DONE,
    BALANCE_SOFT_LIMIT_TEST_ERROR
} BalanceSoftLimitTestStage;

typedef enum {
    BALANCE_SOFT_LIMIT_TEST_ERROR_NONE = 0,
    BALANCE_SOFT_LIMIT_TEST_ERROR_NOT_READY,
    BALANCE_SOFT_LIMIT_TEST_ERROR_RANGE,
    BALANCE_SOFT_LIMIT_TEST_ERROR_TIMEOUT,
    BALANCE_SOFT_LIMIT_TEST_ERROR_POSITION,
    BALANCE_SOFT_LIMIT_TEST_ERROR_CLAMPED,
    BALANCE_SOFT_LIMIT_TEST_ERROR_CANCELLED
} BalanceSoftLimitTestError;

typedef struct {
    int32_t zero_raw_count;
    int32_t current_logical_count;
    int32_t low_logical_count;
    int32_t high_logical_count;
    int32_t minimum_logical_count;
    int32_t maximum_logical_count;
    uint32_t clamp_count;
    uint32_t flash_sequence;
    uint32_t test_elapsed_ms;
    uint32_t test_timeout_ms;
    int32_t test_target_logical_count;
    int32_t test_position_error_count;
    int8_t last_clamp_direction;
    uint8_t zero_valid;
    uint8_t low_valid;
    uint8_t high_valid;
    uint8_t limits_valid;
    uint8_t calibration_active;
    uint8_t flash_record_valid;
    uint8_t zero_confirmation_required;
    uint8_t recalibration_armed;
    uint8_t test_active;
    BalanceSoftLimitCalibrationStage calibration_stage;
    BalanceSoftLimitError error;
    BalanceCalibrationFlashStatus flash_status;
    BalanceSoftLimitTestStage test_stage;
    BalanceSoftLimitTestError test_error;
} BalanceSoftLimitsRuntime;

void BalanceSoftLimits_Init(void);
uint8_t BalanceSoftLimits_BeginAtCurrentAsZero(void);
uint8_t BalanceSoftLimits_CaptureCurrentStage(void);
void BalanceSoftLimits_AbortCalibration(void);
uint8_t BalanceSoftLimits_IsCalibrationActive(void);
uint8_t BalanceSoftLimits_ArmRecalibration(void);
void BalanceSoftLimits_CancelRecalibration(void);
uint8_t BalanceSoftLimits_StartTravelTest(void);
void BalanceSoftLimits_CancelTravelTest(void);
uint8_t BalanceSoftLimits_IsTravelTestActive(void);
void BalanceSoftLimits_Update20ms(uint32_t elapsed_ms);
void BalanceSoftLimits_Enforce100usFromIsr(void);
void BalanceSoftLimits_GetSnapshot(BalanceSoftLimitsRuntime *snapshot);

#endif
