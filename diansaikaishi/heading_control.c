#include "heading_control.h"

#include "angle_utils.h"
#include "app_config.h"

HeadingControlRuntime g_headingRuntime;

static int16_t clamp_i16_from_float(float value, int16_t minValue,
    int16_t maxValue)
{
    if (value > (float)maxValue) {
        return maxValue;
    }
    if (value < (float)minValue) {
        return minValue;
    }
    return (int16_t)value;
}

void HeadingControl_Init(void)
{
    HeadingControl_Reset();
}

void HeadingControl_Reset(void)
{
    g_headingRuntime.target_yaw_deg = 0.0f;
    g_headingRuntime.heading_error_deg = 0.0f;
    g_headingRuntime.last_heading_error_deg = 0.0f;
    g_headingRuntime.heading_derivative = 0.0f;
    g_headingRuntime.correction = 0;
    g_headingRuntime.enabled = false;
    g_headingRuntime.target_locked = false;
}

void HeadingControl_Enable(bool enable)
{
    g_headingRuntime.enabled = enable;
    if (!enable) {
        g_headingRuntime.correction = 0;
    }
}

void HeadingControl_LockCurrentYaw(float current_yaw_deg)
{
    HeadingControl_SetTargetYaw(current_yaw_deg);
}

void HeadingControl_SetTargetYaw(float target_yaw_deg)
{
    g_headingRuntime.target_yaw_deg = target_yaw_deg;
    g_headingRuntime.heading_error_deg = 0.0f;
    g_headingRuntime.last_heading_error_deg = 0.0f;
    g_headingRuntime.heading_derivative = 0.0f;
    g_headingRuntime.correction = 0;
    g_headingRuntime.target_locked = true;
}

int16_t HeadingControl_Update(float current_yaw_deg, float gyro_z_dps,
    float dt_s)
{
    float output;

    (void)dt_s;

    if (!g_headingRuntime.enabled || !g_headingRuntime.target_locked) {
        g_headingRuntime.correction = 0;
        return 0;
    }

    g_headingRuntime.last_heading_error_deg =
        g_headingRuntime.heading_error_deg;
    g_headingRuntime.heading_error_deg =
        Angle_Normalize180(g_headingRuntime.target_yaw_deg -
            current_yaw_deg);
    g_headingRuntime.heading_derivative = -gyro_z_dps;

    output =
        (float)g_appConfig.heading_kp *
            g_headingRuntime.heading_error_deg +
        (float)g_appConfig.heading_kd *
            g_headingRuntime.heading_derivative;
    output /= (float)g_appConfig.heading_scale;

    g_headingRuntime.correction =
        clamp_i16_from_float(output,
            (int16_t)-g_appConfig.heading_max_correction,
            g_appConfig.heading_max_correction);

    return g_headingRuntime.correction;
}

const HeadingControlRuntime *HeadingControl_GetRuntime(void)
{
    return &g_headingRuntime;
}
