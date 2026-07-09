#include "car_controller.h"

#include "app_config.h"
#include "car_state.h"
#include "motor.h"
#include "track_sensor.h"

#ifndef TRACK_REVERSE_CORRECTION
#define TRACK_REVERSE_CORRECTION    (0)
#endif

AppRuntime g_appRuntime;

static int16_t clamp_i16(int32_t value, int16_t minValue, int16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return (int16_t)value;
}

static void update_sensor_runtime(void)
{
    g_appRuntime.sensor_raw = TrackSensor_ReadRaw();
    g_appRuntime.black_count =
        TrackSensor_CountBlack(g_appRuntime.sensor_raw);
    g_appRuntime.line_error =
        TrackSensor_GetErrorFromRaw(g_appRuntime.sensor_raw);
}

static void update_tracking(void)
{
    int16_t error;
    int16_t derivative;
    int32_t correction;

    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw)) {
        if (g_appRuntime.lost_count < UINT8_MAX) {
            g_appRuntime.lost_count++;
        }

        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;

        if (g_appRuntime.lost_count >= g_appConfig.lost_line_threshold) {
            CarState_Set(CAR_STATE_ERROR);
        }
        return;
    }

    g_appRuntime.lost_count = 0;

    error = g_appRuntime.line_error;
    derivative = (int16_t)(error - g_appRuntime.last_error);

    correction =
        ((int32_t)g_appConfig.track_kp * error +
            (int32_t)g_appConfig.track_kd * derivative) /
        g_appConfig.track_scale;

    correction = clamp_i16(correction,
        (int16_t)-g_appConfig.max_correction,
        g_appConfig.max_correction);

    if (TRACK_REVERSE_CORRECTION) {
        correction = -correction;
    }

    g_appRuntime.correction = (int16_t)correction;
    g_appRuntime.left_speed = clamp_i16(
        (int32_t)g_appConfig.base_speed + g_appRuntime.correction,
        -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    g_appRuntime.right_speed = clamp_i16(
        (int32_t)g_appConfig.base_speed - g_appRuntime.correction,
        -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    Motor_SetSpeed(g_appRuntime.left_speed, g_appRuntime.right_speed);
    g_appRuntime.last_error = error;
}

void CarController_Init(void)
{
    CarController_ResetRuntime();
}

void CarController_ResetRuntime(void)
{
    g_appRuntime.current_lap = 0;
    g_appRuntime.sensor_raw = 0;
    g_appRuntime.black_count = 0;
    g_appRuntime.line_error = 0;
    g_appRuntime.last_error = 0;
    g_appRuntime.correction = 0;
    g_appRuntime.left_speed = 0;
    g_appRuntime.right_speed = 0;
    g_appRuntime.lost_count = 0;
    g_appRuntime.lap_cooldown_ms = 0;
    Motor_Stop();
}

void CarController_Update_20ms(void)
{
    update_sensor_runtime();

    if (CarState_Get() != CAR_STATE_RUNNING) {
        Motor_Stop();
        g_appRuntime.left_speed = 0;
        g_appRuntime.right_speed = 0;
        g_appRuntime.correction = 0;
        return;
    }

    update_tracking();
}
