#include "oled_ui.h"
#include "app.h"
#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "heading_control.h"
#include "imu.h"
#include "menu.h"
#include "oled.h"
#include "track_sensor.h"

static int16_t clamp_display_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static void print_status_page(uint8_t raw, int16_t error, uint8_t keyEvent)
{
    (void)error;

    OLED_SetCursor(0, 0);
    OLED_PrintString("ST:");
    OLED_PrintString(CarState_ToString(CarState_Get()));
    OLED_PrintString(" M:");
    OLED_PrintString(CarController_RunModeToString(CarController_GetRunMode()));

    OLED_SetCursor(2, 0);
    OLED_PrintString("LAP:");
    OLED_PrintInt16((int16_t)g_appConfig.target_laps);
    OLED_PrintString(" K:");
    OLED_PrintInt16((int16_t)keyEvent);

    OLED_SetCursor(4, 0);
    OLED_PrintString("RAW:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));

    OLED_SetCursor(6, 0);
    OLED_PrintString("L:");
    OLED_PrintInt16(g_appRuntime.left_speed);
    OLED_PrintString(" R:");
    OLED_PrintInt16(g_appRuntime.right_speed);
}

static void print_param_page(uint8_t keyEvent)
{
    ParamItem item = Menu_GetParamItem();

    OLED_SetCursor(0, 0);
    OLED_PrintString("PARAM");

    OLED_SetCursor(2, 0);
    OLED_PrintChar('>');
    OLED_PrintString(Menu_ParamItemToString(item));

    OLED_SetCursor(4, 0);
    OLED_PrintString("VAL:");
    OLED_PrintInt16(Menu_GetParamValue(item));

    OLED_SetCursor(6, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(g_appRuntime.correction);
    OLED_PrintString(" K:");
    OLED_PrintInt16((int16_t)keyEvent);
}

static void print_sensor_page(uint8_t raw, uint8_t blackCount, int16_t error)
{
    OLED_SetCursor(0, 0);
    OLED_PrintString("SENSOR");
    OLED_PrintString(" M:");
    OLED_PrintString(CarController_RunModeToString(CarController_GetRunMode()));

    OLED_SetCursor(2, 0);
    OLED_PrintString("RAW:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));

    OLED_SetCursor(4, 0);
    OLED_PrintString("CNT:");
    OLED_PrintInt16((int16_t)blackCount);

    OLED_SetCursor(6, 0);
    OLED_PrintString("LOST:");
    OLED_PrintInt16((int16_t)g_appRuntime.lost_count);
    OLED_PrintString(" T:");
    OLED_PrintInt16((int16_t)g_trackTurnDebug);
    OLED_PrintString(" E:");
    OLED_PrintInt16(error);
}

static void print_imu_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    int16_t gyroDpsX10 =
        clamp_display_i16((int32_t)(imu->corrected_gyro_z_dps * 10.0f));
    int16_t biasDpsX10 =
        clamp_display_i16((int32_t)(imu->gyro_bias_dps * 10.0f));
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);

    OLED_SetCursor(0, 0);
    OLED_PrintString("IMU:");
    if (imu->initialized) {
        OLED_PrintString("OK");
    } else {
        OLED_PrintString("ERR");
    }
    OLED_PrintString(" C:");
    OLED_PrintInt16((int16_t)imu->last_error_code);
    OLED_PrintString(" b:");
    OLED_PrintInt16((int16_t)imu->bus_state);

    OLED_SetCursor(2, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" CAL:");
    OLED_PrintInt16((int16_t)imu->calibrated);

    OLED_SetCursor(4, 0);
    OLED_PrintString("R:");
    OLED_PrintInt16(imu->raw_gyro_z);
    OLED_PrintString(" G:");
    OLED_PrintInt16(gyroDpsX10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("BI:");
    OLED_PrintInt16(biasDpsX10);
    OLED_PrintString(" ID:");
    OLED_PrintInt16((int16_t)imu->last_who_am_i);
}

static void print_heading_page(void)
{
    const ImuRuntime *imu = Imu_GetRuntime();
    const HeadingControlRuntime *heading = HeadingControl_GetRuntime();
    int16_t yawDeg = clamp_display_i16((int32_t)imu->yaw_deg);
    int16_t targetDeg =
        clamp_display_i16((int32_t)heading->target_yaw_deg);
    int16_t errorX10 =
        clamp_display_i16((int32_t)(heading->heading_error_deg * 10.0f));
    int16_t derivativeX10 =
        clamp_display_i16((int32_t)(heading->heading_derivative * 10.0f));

    OLED_SetCursor(0, 0);
    OLED_PrintString("HEAD OBS");

    OLED_SetCursor(2, 0);
    OLED_PrintString("Y:");
    OLED_PrintInt16(yawDeg);
    OLED_PrintString(" T:");
    OLED_PrintInt16(targetDeg);

    OLED_SetCursor(4, 0);
    OLED_PrintString("E10:");
    OLED_PrintInt16(errorX10);
    OLED_PrintString(" D10:");
    OLED_PrintInt16(derivativeX10);

    OLED_SetCursor(6, 0);
    OLED_PrintString("C:");
    OLED_PrintInt16(heading->correction);
    OLED_PrintString(" LK:");
    OLED_PrintInt16((int16_t)heading->target_locked);
}

void OledUi_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

void OledUi_Update_20ms(uint8_t raw, uint8_t blackCount, int16_t error,
    uint8_t keyEvent)
{
    static uint8_t refreshDivider;

    refreshDivider++;
    if (refreshDivider < 5U) {
        return;
    }
    refreshDivider = 0;

    OLED_Clear();

    switch (Menu_GetPage()) {
        case OLED_PAGE_PARAM:
            print_param_page(keyEvent);
            break;
        case OLED_PAGE_SENSOR:
            print_sensor_page(raw, blackCount, error);
            break;
        case OLED_PAGE_IMU:
            print_imu_page();
            break;
        case OLED_PAGE_HEADING:
            print_heading_page();
            break;
        case OLED_PAGE_STATUS:
        default:
            print_status_page(raw, error, keyEvent);
            break;
    }
}
