#include "imu.h"

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
#define MPU6050_GYRO_250DPS_SCALE      (131.0f)
#define MPU6050_CONFIG_DLPF_44HZ       (0x03U)
#define MPU6050_GYRO_CONFIG_250DPS     (0x00U)
#define MPU6050_PWR_MGMT_WAKE          (0x00U)
#define MPU6050_SMPLRT_DIV_1KHZ_125HZ  (0x07U)

#define IMU_I2C_DELAY_CYCLES           (36U)

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

static void imu_delay(void)
{
    delay_cycles(IMU_I2C_DELAY_CYCLES);
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

static void imu_i2c_init_pins(void)
{
#ifdef GPIO_IMU_SDA_IOMUX
    DL_GPIO_initDigitalOutput(GPIO_IMU_SDA_IOMUX);
#endif
#ifdef GPIO_IMU_SCL_IOMUX
    DL_GPIO_initDigitalOutput(GPIO_IMU_SCL_IOMUX);
#endif

    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_SDA_PIN | GPIO_IMU_SCL_PIN);
    imu_sda_high();
    imu_scl_high();
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

static float mpu6050_raw_gyro_to_dps(int16_t raw_gyro)
{
    return ((float)raw_gyro) / MPU6050_GYRO_250DPS_SCALE;
}

static void imu_reset_runtime(void)
{
    g_imuRuntime.raw_gyro_z = 0;
    g_imuRuntime.gyro_z_dps = 0.0f;
    g_imuRuntime.gyro_bias_dps = 0.0f;
    g_imuRuntime.yaw_deg = 0.0f;
    g_imuRuntime.initialized = false;
    g_imuRuntime.calibrated = false;
    g_imuRuntime.data_valid = false;
    g_imuRuntime.update_count = 0;
    g_imuRuntime.read_error_count = 0;
    g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
    g_imuRuntime.last_who_am_i = 0;
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
}

bool Imu_Init(void)
{
    uint8_t who_am_i;

    imu_reset_runtime();

#if ENABLE_IMU
    imu_i2c_init_pins();

    g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
    if (!mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i)) {
        g_imuRuntime.last_error_code = IMU_ERROR_WHO_READ_68;
        g_imuRuntime.read_error_count++;

        g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_HIGH;
        if (!mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i)) {
            g_imuRuntime.last_error_code = IMU_ERROR_WHO_READ_69;
            g_imuRuntime.read_error_count++;
            g_imuRuntime.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;
            return false;
        }
    }

    g_imuRuntime.last_who_am_i = who_am_i;
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
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
            MPU6050_GYRO_CONFIG_250DPS)) {
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_CONFIG_WRITE;
        g_imuRuntime.read_error_count++;
        return false;
    }

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
        g_imuRuntime.data_valid = false;
        g_imuRuntime.last_error_code = IMU_ERROR_NOT_INITIALIZED;
        g_imuRuntime.read_error_count++;
        return false;
    }

    if (!mpu6050_read_regs(MPU6050_REG_GYRO_ZOUT_H, data, 2U)) {
        g_imuRuntime.data_valid = false;
        g_imuRuntime.last_error_code = IMU_ERROR_GYRO_READ;
        g_imuRuntime.read_error_count++;
        return false;
    }

    *raw_gyro_z = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_imuRuntime.last_error_code = IMU_ERROR_NONE;
    return true;
#else
    *raw_gyro_z = 0;
    g_imuRuntime.data_valid = false;
    g_imuRuntime.last_error_code = IMU_ERROR_NOT_INITIALIZED;
    g_imuRuntime.read_error_count++;
    return false;
#endif
}

bool Imu_CalibrateGyroBias(uint16_t sample_count)
{
    (void)sample_count;

    g_imuRuntime.calibrated = false;
    g_imuRuntime.gyro_bias_dps = 0.0f;
    return false;
}

void Imu_Update(float dt_s)
{
    int16_t raw_gyro_z;

    (void)dt_s;
    g_imuRuntime.update_count++;

    if (!Imu_ReadRawGyroZ(&raw_gyro_z)) {
        return;
    }

    g_imuRuntime.raw_gyro_z = raw_gyro_z;
    g_imuRuntime.gyro_z_dps = mpu6050_raw_gyro_to_dps(raw_gyro_z);
    g_imuRuntime.data_valid = true;
}

void Imu_ResetYaw(void)
{
    g_imuRuntime.yaw_deg = 0.0f;
}

void Imu_SetYaw(float yaw_deg)
{
    g_imuRuntime.yaw_deg = yaw_deg;
}

float Imu_GetYaw(void)
{
    return g_imuRuntime.yaw_deg;
}

float Imu_GetGyroZDps(void)
{
    return g_imuRuntime.gyro_z_dps;
}

float Imu_GetGyroBiasDps(void)
{
    return g_imuRuntime.gyro_bias_dps;
}

bool Imu_IsReady(void)
{
    return (g_imuRuntime.initialized && g_imuRuntime.calibrated &&
        g_imuRuntime.data_valid);
}

const ImuRuntime *Imu_GetRuntime(void)
{
    return &g_imuRuntime;
}
