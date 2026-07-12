#include "menu.h"
#include "app_config.h"
#include "car_controller.h"
#include "car_state.h"
#include "mission_manager.h"

static OledPage g_oledPage;
static ParamItem g_paramItem;

static void menu_toggle_status_sensor_page(void)
{
    if (g_oledPage == OLED_PAGE_STATUS) {
        g_oledPage = OLED_PAGE_HEADING;
    } else {
        g_oledPage = OLED_PAGE_STATUS;
    }
}

static void menu_next_param(void)
{
    g_paramItem = (ParamItem)(g_paramItem + 1);
    if (g_paramItem >= PARAM_COUNT) {
        g_paramItem = PARAM_TASK;
    }
}

static void menu_adjust_param(int8_t direction)
{
    switch (g_paramItem) {
        case PARAM_TASK:
            if (direction > 0) {
                (void)MissionManager_SelectNext();
            } else {
                (void)MissionManager_SelectPrevious();
            }
            break;
        case PARAM_BASE_SPEED:
            g_appConfig.base_speed += (int16_t)(direction * 10);
            break;
        case PARAM_SEEK_HEADING_OFFSET:
            g_appConfig.seek_heading_offset_deg += direction;
            break;
        case PARAM_SECOND_SEEK_ANGLE:
            g_appConfig.second_seek_angle_deg += (int16_t)(direction * 5);
            break;
        case PARAM_KP:
            g_appConfig.track_kp += direction;
            break;
        case PARAM_KD:
            g_appConfig.track_kd += direction;
            break;
        case PARAM_MAX_CORRECTION:
            g_appConfig.max_correction += (int16_t)(direction * 10);
            break;
        case PARAM_SERVO_ANGLE:
            g_appConfig.servo_angle_deg += (int16_t)(direction * 15);
            break;
        default:
            break;
    }

    AppConfig_LimitAll();
}

static void menu_handle_status_key(KeyEvent event)
{
    CarState state = CarState_Get();

    switch (event) {
        case KEY1_SHORT:
            menu_toggle_status_sensor_page();
            break;
        case KEY1_LONG:
            g_oledPage = OLED_PAGE_PARAM;
            CarState_Set(CAR_STATE_MENU);
            break;
        case KEY2_SHORT:
            switch (state) {
                case CAR_STATE_READY:
                    (void)MissionManager_Start();
                    break;
                case CAR_STATE_RUNNING:
                    MissionManager_Pause();
                    break;
                case CAR_STATE_PAUSED:
                    MissionManager_Resume();
                    break;
                default:
                    break;
            }
            break;
        case KEY2_LONG:
            if (state == CAR_STATE_PAUSED) {
                MissionManager_Resume();
            }
            break;
        case KEY3_SHORT:
            MissionManager_Cancel();
            break;
        case KEY3_LONG:
            CarController_ResetRuntime();
            MissionManager_Reset();
            g_oledPage = OLED_PAGE_STATUS;
            break;
        default:
            break;
    }
}

static void menu_handle_param_key(KeyEvent event)
{
    switch (event) {
        case KEY1_SHORT:
            menu_next_param();
            break;
        case KEY1_LONG:
            g_oledPage = OLED_PAGE_STATUS;
            CarState_Set(CAR_STATE_READY);
            break;
        case KEY2_SHORT:
        case KEY2_LONG:
            menu_adjust_param(1);
            break;
        case KEY3_SHORT:
        case KEY3_LONG:
            menu_adjust_param(-1);
            break;
        default:
            break;
    }
}

void Menu_Init(void)
{
    g_oledPage = OLED_PAGE_STATUS;
    g_paramItem = PARAM_TASK;
}

void Menu_HandleKeyEvent(KeyEvent event)
{
    if (event == KEY_EVENT_NONE) {
        return;
    }

    if (g_oledPage == OLED_PAGE_PARAM) {
        menu_handle_param_key(event);
    } else {
        menu_handle_status_key(event);
    }
}

OledPage Menu_GetPage(void)
{
    return g_oledPage;
}

ParamItem Menu_GetParamItem(void)
{
    return g_paramItem;
}

const char *Menu_ParamItemToString(ParamItem item)
{
    switch (item) {
        case PARAM_TASK:
            return "TASK";
        case PARAM_BASE_SPEED:
            return "SPD";
        case PARAM_SEEK_HEADING_OFFSET:
            return "YAW";
        case PARAM_SECOND_SEEK_ANGLE:
            return "REV";
        case PARAM_KP:
            return "KP";
        case PARAM_KD:
            return "KD";
        case PARAM_MAX_CORRECTION:
            return "MAX";
        case PARAM_SERVO_ANGLE:
            return "SV";
        default:
            return "ERR";
    }
}

int16_t Menu_GetParamValue(ParamItem item)
{
    switch (item) {
        case PARAM_TASK:
            return (int16_t)MissionManager_GetSelectedMissionId();
        case PARAM_BASE_SPEED:
            return g_appConfig.base_speed;
        case PARAM_SEEK_HEADING_OFFSET:
            return g_appConfig.seek_heading_offset_deg;
        case PARAM_SECOND_SEEK_ANGLE:
            return g_appConfig.second_seek_angle_deg;
        case PARAM_KP:
            return g_appConfig.track_kp;
        case PARAM_KD:
            return g_appConfig.track_kd;
        case PARAM_MAX_CORRECTION:
            return g_appConfig.max_correction;
        case PARAM_SERVO_ANGLE:
            return g_appConfig.servo_angle_deg;
        default:
            return 0;
    }
}
