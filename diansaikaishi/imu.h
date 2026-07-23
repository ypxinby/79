#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t raw_gyro_z;
    float gyro_z_dps;
    float corrected_gyro_z_dps;
    float gyro_bias_dps;
    float yaw_deg;

    bool initialized;
    bool calibrated;
    bool valid;
    bool stale;
    bool dt_valid;
    bool short_gap_compensating;

    uint32_t update_count;
    uint32_t sample_dt_ms;
    float sample_dt_s;
    uint32_t last_success_age_ms;
    uint32_t read_fail_count;
    uint32_t consecutive_read_fail_count;
    uint32_t read_error_count;
    uint32_t gyro_range_error_count;

    uint8_t i2c_addr;
    uint8_t last_who_am_i;
    uint8_t last_error_code;
    uint8_t bus_state;
    uint8_t drive_state;
} ImuRuntime;

bool Imu_Init(void);
bool Imu_ReadRawGyroZ(int16_t *raw_gyro_z);
/* Blocking bias calibration; the caller must keep the vehicle stationary. */
bool Imu_CalibrateGyroBias(uint16_t sample_count);
void Imu_Update(uint32_t elapsed_ms);
void Imu_ResetYaw(void);
void Imu_SetYaw(float yaw_deg);
float Imu_GetYaw(void);
float Imu_GetGyroZDps(void);
float Imu_GetCorrectedGyroZDps(void);
float Imu_GetGyroBiasDps(void);
bool Imu_IsReady(void);
const ImuRuntime *Imu_GetRuntime(void);

#endif
