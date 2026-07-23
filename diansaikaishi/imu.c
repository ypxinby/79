#include "imu.h"

#include "angle_utils.h"
#include "app_config.h"
#include "app_features.h"
#include "ti_msp_dl_config.h"

#define MPU6050_I2C_ADDR_AD0_LOW       (0x68U)
#define MPU6050_I2C_ADDR_AD0_HIGH      (0x69U)
#define MPU6050_REG_SMPLRT_DIV         (0x19U)
#define MPU6050_REG_CONFIG             (0x1AU)
#define MPU6050_REG_GYRO_CONFIG        (0x1BU)
#define MPU6050_REG_GYRO_ZOUT_H        (0x47U)
#define MPU6050_REG_PWR_MGMT_1         (0x6BU)
#define MPU6050_REG_WHO_AM_I           (0x75U)
#define MPU6050_WHO_AM_I_VALUE         (0x68U)
#define MPU6050_WHO_AM_I_COMPAT_VALUE  (0x70U)
#define MPU6050_CONFIG_DLPF_44HZ       (0x03U)
#define MPU6050_GYRO_CONFIG_500DPS     (0x08U)
#define MPU6050_GYRO_FS_SEL_MASK       (0x18U)
#define MPU6050_GYRO_FS_SEL_SHIFT      (3U)
#define MPU6050_GYRO_FS_SEL_REQUIRED   (1U)
#define MPU6050_PWR_MGMT_WAKE          (0x00U)
#define MPU6050_SMPLRT_DIV_1KHZ_125HZ  (0x07U)

#define MPU6050_GYRO_SCALE_250DPS      (131.0f)
#define MPU6050_GYRO_SCALE_500DPS      (65.5f)
#define MPU6050_GYRO_SCALE_1000DPS     (32.8f)
#define MPU6050_GYRO_SCALE_2000DPS     (16.4f)

#define IMU_I2C_DELAY_CYCLES           (160U)
#define IMU_STARTUP_DELAY_CYCLES       (3200000U)
#define IMU_WHO_AM_I_RETRY_COUNT       (20U)
#define IMU_WHO_AM_I_RETRY_DELAY       (160000U)
#define IMU_CONFIG_SETTLE_DELAY        (16000000U)
#define IMU_CALIBRATION_DISCARD_COUNT  (50U)
#define IMU_CALIBRATION_SAMPLE_DELAY   (160000U)

#define IMU_ERROR_NONE                 (0U)
#define IMU_ERROR_WHO_READ_68          (1U)
#define IMU_ERROR_WHO_READ_69          (2U)
#define IMU_ERROR_WHO_VALUE            (3U)
#define IMU_ERROR_PWR_WRITE            (4U)
#define IMU_ERROR_SAMPLE_WRITE         (5U)
#define IMU_ERROR_CONFIG_WRITE         (6U)
#define IMU_ERROR_GYRO_CONFIG_WRITE    (7U)
#define IMU_ERROR_NOT_INITIALIZED      (8U)
#define IMU_ERROR_GYRO_READ            (9U)
#define IMU_ERROR_NULL_POINTER         (10U)
#define IMU_ERROR_CALIBRATION          (11U)
#define IMU_ERROR_GYRO_RANGE           (12U)
#define IMU_ERROR_DT_RANGE             (13U)
#define IMU_ERROR_GYRO_CONFIG_READ     (14U)
#define IMU_ERROR_GYRO_CONFIG_MISMATCH (15U)
#define IMU_CALIBRATION_MIN_VALID_DIV  (10U)

#ifndef GPIO_IMU_PORT
#define GPIO_IMU_PORT                  (GPIOA)
#define GPIO_IMU_SDA_PIN               (DL_GPIO_PIN_10)
#define GPIO_IMU_SCL_PIN               (DL_GPIO_PIN_11)
#endif

#ifndef GPIO_IMU_SDA_PIN
#ifdef GPIO_IMU_IMU_SDA_PIN
#define GPIO_IMU_SDA_PIN               GPIO_IMU_IMU_SDA_PIN
#endif
#endif

#ifndef GPIO_IMU_SCL_PIN
#ifdef GPIO_IMU_IMU_SCL_PIN
#define GPIO_IMU_SCL_PIN               GPIO_IMU_IMU_SCL_PIN
#endif
#endif

#ifndef GPIO_IMU_SDA_IOMUX
#ifdef GPIO_IMU_IMU_SDA_IOMUX
#define GPIO_IMU_SDA_IOMUX             GPIO_IMU_IMU_SDA_IOMUX
#endif
#endif

#ifndef GPIO_IMU_SCL_IOMUX
#ifdef GPIO_IMU_IMU_SCL_IOMUX
#define GPIO_IMU_SCL_IOMUX             GPIO_IMU_IMU_SCL_IOMUX
#endif
#endif

ImuRuntime g_imuRuntime;
static float g_previousValidCorrectedGyroZDps;
static uint32_t g_elapsedSinceValidSampleMs;
static bool g_integrationHistoryValid;
static bool g_shortGapPending;

static void imu_delay(void)
{
    delay_cycles(IMU_I2C_DELAY_CYCLES);
}

static void imu_delay_cycles(uint32_t cycles)
{
    delay_cycles(cycles);
}

static void imu_scl_high(void)
{
    DL_GPIO_disableOutput(GPIO_IMU_PORT, GPIO_IMU_SCL_PIN);
    imu_delay();
}

static void imu_scl_low(void)
{
    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_IMU_PORT, GPIO_IMU_SCL_PIN);
    imu_delay();
}

static void imu_sda_high(void)
{
    DL_GPIO_disableOutput(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN);
    imu_delay();
}

static void imu_sda_low(void)
{
    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN);
    imu_delay();
}

static bool imu_sda_read(void)
{
    imu_sda_high();
    return (DL_GPIO_readPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN) != 0U);
}

static bool imu_scl_read(void)
{
    imu_scl_high();
    return (DL_GPIO_readPins(GPIO_IMU_PORT, GPIO_IMU_SCL_PIN) != 0U);
}

static void imu_update_bus_state(void)
{
    uint8_t state = 0;

    if (imu_sda_read()) {
        state |= 1U;
    }
    if (imu_scl_read()) {
        state |= 2U;
    }
    g_imuRuntime.bus_state = state;
}

static void imu_update_drive_state(void)
{
    uint8_t state = 0;

    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN | GPIO_IMU_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN | GPIO_IMU_SCL_PIN);
    imu_delay();

    if ((DL_GPIO_readPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN) != 0U)) {
        state |= 1U;
    }
    if ((DL_GPIO_readPins(GPIO_IMU_PORT, GPIO_IMU_SCL_PIN) != 0U)) {
        state |= 2U;
    }

    g_imuRuntime.drive_state = state;
    imu_sda_high();
    imu_scl_high();
}

static void imu_i2c_init_pins(void)
{
#ifdef GPIO_IMU_SDA_IOMUX
    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
#endif
#ifdef GPIO_IMU_SCL_IOMUX
    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
#endif

    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN | GPIO_IMU_SCL_PIN);
    imu_sda_high();
    imu_scl_high();
    imu_update_bus_state();
    imu_update_drive_state();
}

static void imu_i2c_start(void)
{
    imu_sda_high();
    imu_scl_high();
    imu_sda_low();
    imu_scl_low();
}

static void imu_i2c_stop(void)
{
    imu_sda_low();
    imu_scl_high();
    imu_sda_high();
}

static void imu_i2c_recover_bus(void)
{
    imu_sda_high();
    for (uint8_t i = 0; i < 9U; i++) {
        imu_scl_high();
        imu_scl_low();
    }
    imu_i2c_stop();
    imu_update_bus_state();
}

static bool imu_i2c_write_byte(uint8_t value)
{
    bool nack;

    for (uint8_t i = 0; i < 8U; i++) {
        if ((value & 0x80U) != 0U) {
            imu_sda_high();
        } else {
            imu_sda_low();
        }

        imu_scl_high();
        imu_scl_low();
        value <<= 1;
    }

    imu_sda_high();
    imu_scl_high();
    nack = imu_sda_read();
    imu_scl_low();

    return !nack;
}

static uint8_t imu_i2c_read_byte(bool ack)
{
    uint8_t value = 0;

    imu_sda_high();
    for (uint8_t i = 0; i < 8U; i++) {
        value <<= 1;
        imu_scl_high();
        if (imu_sda_read()) {
            value |= 1U;
        }
        imu_scl_low();
    }

    if (ack) {
        imu_sda_low();
    } else {
        imu_sda_high();
    }
    imu_scl_high();
    imu_scl_low();
    imu_sda_high();

    return value;
}

static bool mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    bool ok;

    imu_i2c_start();
    ok = imu_i2c_write_byte((uint8_t)(g_imuRuntime.i2c_addr << 1));
    ok = ok && imu_i2c_write_byte(reg);
    ok = ok && imu_i2c_write_byte(value);
    imu_i2c_stop();

    return ok;
}

static bool mpu6050_read_regs(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    bool ok;

    if ((buffer == (uint8_t *)0) || (length == 0U)) {
        return false;
    }

    imu_i2c_start();
    ok = imu_i2c_write_byte((uint8_t)(g_imuRuntime.i2c_addr << 1));
    ok = ok && imu_i2c_write_byte(reg);

    if (!ok) {
        imu_i2c_stop();
        return false;
    }

    imu_i2c_start();
    ok = imu_i2c_write_byte((uint8_t)((g_imuRuntime.i2c_addr << 1) | 1U));
    if (!ok) {
        imu_i2c_stop();
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = imu_i2c_read_byte((i + 1U) < length);
    }

    imu_i2c_stop();
    return true;
}

static bool mpu6050_read_reg(uint8_t reg, uint8_t *value)
{
    return mpu6050_read_regs(reg, value, 1U);
}

static bool mpu6050_read_who_am_i_retry(uint8_t *who_am_i)
{
    for (uint8_t i = 0; i < IMU_WHO_AM_I_RETRY_COUNT; i++) {
        if (mpu6050_read_reg(MPU6050_REG_WHO_AM_I, who_am_i)) {
            return true;
        }
        g_imuRuntime.read_error_count++;
        imu_i2c_recover_bus();
        imu_delay_cycles(IMU_WHO_AM_I_RETRY_DELAY);
    }

    return false;
}

static void imu_set_gyro_scale_from_config(uint8_t gyro_config)
{
    uint8_t fs_sel = (uint8_t)((gyro_config & MPU6050_GYRO_FS_SEL_MASK) >>
        MPU6050_GYRO_FS_SEL_SHIFT);

    g_imuRuntime.gyro_config_readback = gyro_config;
    g_imuRuntime.gyro_fs_sel = fs_sel;

    switch (fs_sel) {
        case 0U:
            g_imuRuntime.gyro_sensitivity_lsb_per_dps =
                MPU6050_GYRO_SCALE_250DPS;
            g_imuRuntime.gyro_full_scale_dps = 250U;
            break;
        case 1U:
            g_imuRuntime.gyro_sensitivity_lsb_per_dps =
                MPU6050_GYRO_SCALE_500DPS;
            g_imuRuntime.gyro_full_scale_dps = 500U;
            break;
        case 2U:
            g_imuRuntime.gyro_sensitivity_lsb_per_dps =
                MPU6050_GYRO_SCALE_1000DPS;
            g_imuRuntime.gyro_full_scale_dps = 1000U;
            break;
        case 3U:
        default:
            g_imuRuntime.gyro_sensitivity_lsb_per_dps =
                MPU6050_GYRO_SCALE_2000DPS;
            g_imuRuntime.gyro_full_scale_dps = 2000U;
            break;
    }
}

static float mpu6050_raw_gyro_to_dps(int16_t raw_gyro)
{
    if (g_imuRuntime.gyro_sensitivity_lsb_per_dps <= 0.0f) {
        return 0.0f;
    }
    return ((float)raw_gyro) /
        g_imuRuntime.gyro_sensitivity_lsb_per_dps;
}

static float imu_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t imu_add_elapsed_u32(uint32_t value, uint32_t elapsed_ms)
{
    if (value > UINT32_MAX - elapsed_ms) {
        return UINT32_MAX;
    }
    return value + elapsed_ms;
}

static void imu_increment_u32(uint32_t *value)
{
    if (*value < UINT32_MAX) {
        (*value)++;
    }
}

static void imu_clear_integration_history(void)
{
    g_previousValidCorrectedGyroZDps = 0.0f;
    g_elapsedSinceValidSampleMs = 0U;
    g_integrationHistoryValid = false;
    g_imuRuntime.integration_history_valid = false;
    g_shortGapPending = false;
    g_imuRuntime.short_gap_compensating = false;
    g_imuRuntime.angle_increment_deg = 0.0f;
    g_imuRuntime.integration_applied = false;
}

static void imu_handle_unusable_sample(bool read_failed)
{
    if (read_failed) {
        if (g_imuRuntime.read_fail_count < UINT32_MAX) {
            g_imuRuntime.read_fail_count++;
        }
    }
    if (g_imuRuntime.consecutive_read_fail_count < UINT32_MAX) {
        g_imuRuntime.consecutive_read_fail_count++;
    }

    g_imuRuntime.short_gap_compensating = false;
    if (g_imuRuntime.last_success_age_ms >=
        g_appConfig.imu_stale_timeout_ms) {
        g_imuRuntime.stale = true;
        g_imuRuntime.valid = false;
        imu_clear_integration_history();
        return;
    }

    if (!g_integrationHistoryValid) {
        g_imuRuntime.valid = false;
        return;
    }

    if (g_elapsedSinceValidSampleMs >=
        g_appConfig.imu_short_gap_max_ms) {
        g_imuRuntime.valid = false;
        imu_clear_integration_history();
        return;
    }

    g_shortGapPending = true;
    g_imuRuntime.short_gap_compensating = true;
    g_imuRuntime.valid = g_imuRuntime.calibrated;
}

static void imu_reset_runtime(void)
{
    g_imuRuntime.raw_gyro_z = 0;
    g_imuRuntime.gyro_z_before_bias_dps = 0.0f;
    g_imuRuntime.gyro_z_after_bias_dps = 0.0f;
    g_imuRuntime.gyro_z_dps = 0.0f;
    g_imuRuntime.corrected_gyro_z_dps = 0.0f;
    g_imuRuntime.gyro_bias_dps = 0.0f;
    g_imuRuntime.yaw_deg = 0.0f;
    g_imuRuntime.angle_increment_deg = 0.0f;
    g_imuRuntime.gyro_sensitivity_lsb_per_dps = 0.0f;
    g_imuRuntime.initialized = false;
    g_imuRuntime.calibrated = false;
    g_imuRuntime.valid = false;
    g_imuRuntime.stale = false;
    g_imuRuntime.dt_valid = false;
    g_imuRuntime.short_gap_compensating = false;
    g_imuRuntime.integration_applied = false;
    g_imuRuntime.integration_history_valid = false;
    g_imuRuntime.update_count = 0;
    g_imuRuntime.successful_read_count = 0U;
    g_imuRuntime.integration_count = 0U;
    g_imuRuntime.integration_skip_count = 0U;
    g_imuRuntime.history_rebuild_count = 0U;
    g_imuRuntime.dt_invalid_skip_count = 0U;
    g_imuRuntime.read_fail_skip_count = 0U;
    g_imuRuntime.gyro_invalid_skip_count = 0U;
    g_imuRuntime.yaw_reset_count = 0U;
    g_imuRuntime.sample_dt_ms = 0U;
    g_imuRuntime.sample_dt_s = 0.0f;
    g_imuRuntime.last_success_age_ms = 0U;
    g_imuRuntime.read_fail_count = 0U;
    g_imuRuntime.consecutive_read_fail_count = 0U;
    g_imuRuntime.read_error_count = 0;
    g_imuRuntime.gyro_range_error_count = 0U;
    g_imuRuntime.cumulative_elapsed_ms = 0U;
    g_imuRuntime.cumulative_integrated_dt_ms = 0U;
    g_imuRuntime.cumulative_angle_increment_deg = 0.0f;
    g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
    g_imuRuntime.last_who_am_i = 0;
    g_imuRuntime.gyro_config_readback = 0xFFU;
    g_imuRuntime.gyro_fs_sel = 0xFFU;
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
    g_imuRuntime.bus_state = 0;
    g_imuRuntime.drive_state = 0;
    g_imuRuntime.gyro_full_scale_dps = 0U;
    g_imuRuntime.yaw_axis_sign = IMU_YAW_AXIS_SIGN_DEFAULT;
    imu_clear_integration_history();
}

bool Imu_Init(void)
{
    uint8_t who_am_i;
    uint8_t gyro_config;

    imu_reset_runtime();

#if ENABLE_IMU
    imu_i2c_init_pins();
    imu_delay_cycles(IMU_STARTUP_DELAY_CYCLES);
    imu_i2c_recover_bus();

    g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
    if (!mpu6050_read_who_am_i_retry(&who_am_i)) {
        g_imuRuntime.last_error_code = IMU_ERROR_WHO_READ_68;
        g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
        return false;
    }

    g_imuRuntime.last_who_am_i = who_am_i;
    if ((who_am_i != MPU6050_WHO_AM_I_VALUE) &&
        (who_am_i != MPU6050_WHO_AM_I_COMPAT_VALUE)) {
        g_imuRuntime.last_error_code = IMU_ERROR_WHO_VALUE;
        g_imuRuntime.read_error_count++;
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_WAKE)) {
        g_imuRuntime.last_error_code = IMU_ERROR_PWR_WRITE;
        g_imuRuntime.read_error_count++;
        return false;
    }
    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV,
            MPU6050_SMPLRT_DIV_1KHZ_125HZ)) {
        g_imuRuntime.last_error_code = IMU_ERROR_SAMPLE_WRITE;
        g_imuRuntime.read_error_count++;
        return false;
    }
    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, MPU6050_CONFIG_DLPF_44HZ)) {
        g_imuRuntime.last_error_code = IMU_ERROR_CONFIG_WRITE;
        g_imuRuntime.read_error_count++;
        return false;
    }
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,
            MPU6050_GYRO_CONFIG_500DPS)) {
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_CONFIG_WRITE;
        g_imuRuntime.read_error_count++;
        return false;
    }
    if (!mpu6050_read_reg(MPU6050_REG_GYRO_CONFIG, &gyro_config)) {
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_CONFIG_READ;
        g_imuRuntime.read_error_count++;
        return false;
    }
    imu_set_gyro_scale_from_config(gyro_config);
    if (g_imuRuntime.gyro_fs_sel != MPU6050_GYRO_FS_SEL_REQUIRED) {
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_CONFIG_MISMATCH;
        return false;
    }

    g_imuRuntime.yaw_axis_sign = g_appConfig.imu_yaw_axis_sign;
    g_imuRuntime.initialized = true;
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
#endif

    return g_imuRuntime.initialized;
}

bool Imu_ReadRawGyroZ(int16_t *raw_gyro_z)
{
    uint8_t data[2];

    if (raw_gyro_z == (int16_t *)0) {
        g_imuRuntime.last_error_code = IMU_ERROR_NULL_POINTER;
        g_imuRuntime.read_error_count++;
        return false;
    }

#if ENABLE_IMU
    if (!g_imuRuntime.initialized) {
        g_imuRuntime.last_error_code = IMU_ERROR_NOT_INITIALIZED;
        g_imuRuntime.read_error_count++;
        return false;
    }

    if (!mpu6050_read_regs(MPU6050_REG_GYRO_ZOUT_H, data, 2U)) {
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_READ;
        g_imuRuntime.read_error_count++;
        return false;
    }

    *raw_gyro_z = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
    return true;
#else
    *raw_gyro_z = 0;
    g_imuRuntime.last_error_code = IMU_ERROR_NOT_INITIALIZED;
    g_imuRuntime.read_error_count++;
    return false;
#endif
}

bool Imu_CalibrateGyroBias(uint16_t sample_count)
{
    int64_t sum = 0;
    uint16_t valid_count = 0;
    uint16_t min_valid_count;

    if (!g_imuRuntime.initialized || (sample_count == 0U)) {
        g_imuRuntime.calibrated = false;
        g_imuRuntime.valid = false;
        imu_clear_integration_history();
        g_imuRuntime.last_error_code = IMU_ERROR_CALIBRATION;
        return false;
    }

    imu_delay_cycles(IMU_CONFIG_SETTLE_DELAY);

    for (uint16_t i = 0; i < IMU_CALIBRATION_DISCARD_COUNT; i++) {
        int16_t raw_gyro_z;

        (void)Imu_ReadRawGyroZ(&raw_gyro_z);
        imu_delay_cycles(IMU_CALIBRATION_SAMPLE_DELAY);
    }

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t raw_gyro_z;

        if (Imu_ReadRawGyroZ(&raw_gyro_z)) {
            sum += raw_gyro_z;
            valid_count++;
        }
        imu_delay_cycles(IMU_CALIBRATION_SAMPLE_DELAY);
    }

    min_valid_count =
        (uint16_t)(sample_count - (sample_count / IMU_CALIBRATION_MIN_VALID_DIV));
    if (valid_count < min_valid_count) {
        g_imuRuntime.calibrated = false;
        g_imuRuntime.valid = false;
        imu_clear_integration_history();
        g_imuRuntime.last_error_code = IMU_ERROR_CALIBRATION;
        return false;
    }

    g_imuRuntime.gyro_bias_dps =
        mpu6050_raw_gyro_to_dps((int16_t)(sum / valid_count));
    g_imuRuntime.calibrated = true;
    g_imuRuntime.valid = false;
    g_imuRuntime.stale = false;
    g_imuRuntime.last_success_age_ms = 0U;
    g_imuRuntime.consecutive_read_fail_count = 0U;
    imu_clear_integration_history();
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;

    return true;
}

void Imu_Update(uint32_t elapsed_ms)
{
    int16_t raw_gyro_z;
    float gyro_z_dps;
    float corrected_gyro_z;
    float integration_dt_s;
    float angle_increment_deg;

    imu_increment_u32(&g_imuRuntime.update_count);
    g_imuRuntime.cumulative_elapsed_ms = imu_add_elapsed_u32(
        g_imuRuntime.cumulative_elapsed_ms, elapsed_ms);
    g_imuRuntime.sample_dt_ms = elapsed_ms;
    g_imuRuntime.sample_dt_s = (float)elapsed_ms / 1000.0f;
    g_imuRuntime.angle_increment_deg = 0.0f;
    g_imuRuntime.integration_applied = false;
    g_imuRuntime.dt_valid =
        (elapsed_ms >= g_appConfig.imu_dt_min_ms) &&
        (elapsed_ms <= g_appConfig.imu_dt_max_ms);
    g_imuRuntime.last_success_age_ms = imu_add_elapsed_u32(
        g_imuRuntime.last_success_age_ms, elapsed_ms);
    if (g_integrationHistoryValid) {
        g_elapsedSinceValidSampleMs = imu_add_elapsed_u32(
            g_elapsedSinceValidSampleMs, elapsed_ms);
    }

    if (!g_imuRuntime.initialized) {
        imu_increment_u32(&g_imuRuntime.integration_skip_count);
        g_imuRuntime.valid = false;
        g_imuRuntime.stale = true;
        imu_clear_integration_history();
        imu_update_bus_state();
        imu_update_drive_state();
        return;
    }

    if (!Imu_ReadRawGyroZ(&raw_gyro_z)) {
        imu_increment_u32(&g_imuRuntime.integration_skip_count);
        imu_increment_u32(&g_imuRuntime.read_fail_skip_count);
        imu_handle_unusable_sample(true);
        return;
    }
    imu_increment_u32(&g_imuRuntime.successful_read_count);

    gyro_z_dps = mpu6050_raw_gyro_to_dps(raw_gyro_z);
    g_imuRuntime.raw_gyro_z = raw_gyro_z;
    g_imuRuntime.gyro_z_before_bias_dps = gyro_z_dps;
    g_imuRuntime.gyro_z_dps = gyro_z_dps;
    corrected_gyro_z = gyro_z_dps - g_imuRuntime.gyro_bias_dps;
    g_imuRuntime.gyro_z_after_bias_dps = corrected_gyro_z;
    if (imu_abs_float(gyro_z_dps) >
        g_appConfig.imu_max_abs_gyro_dps) {
        imu_increment_u32(&g_imuRuntime.integration_skip_count);
        imu_increment_u32(&g_imuRuntime.gyro_invalid_skip_count);
        if (g_imuRuntime.gyro_range_error_count < UINT32_MAX) {
            g_imuRuntime.gyro_range_error_count++;
        }
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_RANGE;
        imu_handle_unusable_sample(false);
        return;
    }

    if (imu_abs_float(corrected_gyro_z) < g_appConfig.gyro_deadband_dps) {
        corrected_gyro_z = 0.0f;
    }
    corrected_gyro_z *= (float)g_imuRuntime.yaw_axis_sign;
    g_imuRuntime.corrected_gyro_z_dps = corrected_gyro_z;

    g_imuRuntime.last_success_age_ms = 0U;
    g_imuRuntime.consecutive_read_fail_count = 0U;
    g_imuRuntime.stale = false;
    if (!g_imuRuntime.dt_valid) {
        imu_increment_u32(&g_imuRuntime.integration_skip_count);
        imu_increment_u32(&g_imuRuntime.dt_invalid_skip_count);
        g_imuRuntime.valid = false;
        g_imuRuntime.last_error_code = IMU_ERROR_DT_RANGE;
        imu_clear_integration_history();
        return;
    }

    g_imuRuntime.short_gap_compensating = false;
    /*
     * A successful sample may legitimately arrive after a 61..100 ms control
     * tick. Apply the short-gap bound only when an I2C miss is being bridged.
     */
    if (g_imuRuntime.calibrated && g_integrationHistoryValid &&
        (!g_shortGapPending ||
            (g_elapsedSinceValidSampleMs <=
                g_appConfig.imu_short_gap_max_ms))) {
        integration_dt_s =
            (float)g_elapsedSinceValidSampleMs / 1000.0f;
        angle_increment_deg =
            0.5f * (g_previousValidCorrectedGyroZDps +
                corrected_gyro_z) * integration_dt_s;
        g_imuRuntime.yaw_deg = Angle_Normalize180(
            g_imuRuntime.yaw_deg + angle_increment_deg);
        g_imuRuntime.angle_increment_deg = angle_increment_deg;
        g_imuRuntime.integration_applied = true;
        imu_increment_u32(&g_imuRuntime.integration_count);
        g_imuRuntime.cumulative_integrated_dt_ms = imu_add_elapsed_u32(
            g_imuRuntime.cumulative_integrated_dt_ms,
            g_elapsedSinceValidSampleMs);
        g_imuRuntime.cumulative_angle_increment_deg += angle_increment_deg;
        g_imuRuntime.short_gap_compensating = g_shortGapPending;
    } else {
        imu_increment_u32(&g_imuRuntime.integration_skip_count);
        if (g_imuRuntime.calibrated) {
            imu_increment_u32(&g_imuRuntime.history_rebuild_count);
        }
    }

    g_previousValidCorrectedGyroZDps = corrected_gyro_z;
    g_elapsedSinceValidSampleMs = 0U;
    g_integrationHistoryValid = g_imuRuntime.calibrated;
    g_imuRuntime.integration_history_valid =
        g_integrationHistoryValid;
    g_shortGapPending = false;
    g_imuRuntime.valid = g_imuRuntime.calibrated;
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
}

void Imu_ResetYaw(void)
{
    g_imuRuntime.yaw_deg = 0.0f;
    g_imuRuntime.cumulative_elapsed_ms = 0U;
    g_imuRuntime.cumulative_integrated_dt_ms = 0U;
    g_imuRuntime.cumulative_angle_increment_deg = 0.0f;
    imu_increment_u32(&g_imuRuntime.yaw_reset_count);
    imu_clear_integration_history();
}

void Imu_SetYaw(float yaw_deg)
{
    g_imuRuntime.yaw_deg = Angle_Normalize180(yaw_deg);
    g_imuRuntime.cumulative_elapsed_ms = 0U;
    g_imuRuntime.cumulative_integrated_dt_ms = 0U;
    g_imuRuntime.cumulative_angle_increment_deg = 0.0f;
    imu_increment_u32(&g_imuRuntime.yaw_reset_count);
    imu_clear_integration_history();
}

float Imu_GetYaw(void)
{
    return g_imuRuntime.yaw_deg;
}

float Imu_GetGyroZDps(void)
{
    return g_imuRuntime.gyro_z_dps;
}

float Imu_GetCorrectedGyroZDps(void)
{
    return g_imuRuntime.corrected_gyro_z_dps;
}

float Imu_GetGyroBiasDps(void)
{
    return g_imuRuntime.gyro_bias_dps;
}

bool Imu_IsReady(void)
{
    return (g_imuRuntime.initialized && g_imuRuntime.calibrated &&
        g_imuRuntime.valid && !g_imuRuntime.stale);
}

const ImuRuntime *Imu_GetRuntime(void)
{
    return &g_imuRuntime;
}
