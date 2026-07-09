#include "oled_ui.h"
#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "menu.h"
#include "oled.h"
#include "track_sensor.h"

static void print_status_page(uint8_t raw, int16_t error, uint8_t keyEvent)
{
    (void)error;

    OLED_SetCursor(0, 0);
    OLED_PrintString("ST:");
    OLED_PrintString(CarState_ToString(CarState_Get()));

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

    OLED_SetCursor(2, 0);
    OLED_PrintString("RAW:");
    OLED_PrintBinary7((uint8_t)(raw & TRACK_RAW_VALID_MASK));

    OLED_SetCursor(4, 0);
    OLED_PrintString("CNT:");
    OLED_PrintInt16((int16_t)blackCount);

    OLED_SetCursor(6, 0);
    OLED_PrintString("LOST:");
    OLED_PrintInt16((int16_t)g_appRuntime.lost_count);
    OLED_PrintString(" E:");
    OLED_PrintInt16(error);
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
        case OLED_PAGE_STATUS:
        default:
            print_status_page(raw, error, keyEvent);
            break;
    }
}
