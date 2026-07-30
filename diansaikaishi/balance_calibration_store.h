#ifndef BALANCE_CALIBRATION_STORE_H
#define BALANCE_CALIBRATION_STORE_H

#include <stdint.h>

typedef enum {
    BALANCE_CALIBRATION_FLASH_EMPTY = 0,
    BALANCE_CALIBRATION_FLASH_VALID,
    BALANCE_CALIBRATION_FLASH_INVALID,
    BALANCE_CALIBRATION_FLASH_WRITE_ERROR
} BalanceCalibrationFlashStatus;

typedef struct {
    int32_t low_offset_count;
    int32_t high_offset_count;
    uint32_t sequence;
} BalanceCalibrationStoredLimits;

BalanceCalibrationFlashStatus BalanceCalibrationStore_Load(
    BalanceCalibrationStoredLimits *limits);
BalanceCalibrationFlashStatus BalanceCalibrationStore_Save(
    int32_t low_offset_count, int32_t high_offset_count,
    BalanceCalibrationStoredLimits *saved_limits);

#endif
