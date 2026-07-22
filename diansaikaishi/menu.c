#include "menu.h"
#include "app_config.h"
#include "app_features.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "gimbal.h"
#include "gimbal_tracker.h"
#include "gimbal_vision_pitch_tracker.h"
#include "gimbal_vision_yaw_tracker.h"
#include "mission_manager.h"
#include "obstacle_avoidance.h"
#include "watchdog_monitor.h"

static OledPage g_oledPage;
static OledPage g_paramReturnPage;
static ParamItem g_paramItem;

static void menu_enter_param_page(ParamItem item, uint8_t select_item)
{
    g_paramReturnPage = g_oledPage;
    if (select_item != 0U) {
        g_paramItem = item;
    }
    g_oledPage = OLED_PAGE_PARAM;
    CarState_Set(CAR_STATE_MENU);
}

static void menu_next_main_page(void)
{
    if (g_oledPage == OLED_PAGE_STATUS) {
        g_oledPage = OLED_PAGE_HEADING;
    } else if (g_oledPage == OLED_PAGE_HEADING) {
        g_oledPage = OLED_PAGE_OBSTACLE;
    } else if (g_oledPage == OLED_PAGE_OBSTACLE) {
        g_oledPage = OLED_PAGE_ENCODER;
    } else {
        g_oledPage = OLED_PAGE_STATUS;
    }
}

#if FEATURE_GIMBAL_OLED_TEST
static uint8_t menu_is_debug_page(OledPage page)
{
    return ((page == OLED_PAGE_VISION_RECEIVER) ||
        (page == OLED_PAGE_GIMBAL_VISION_ADAPTER) ||
        (page == OLED_PAGE_GIMBAL_VISION_YAW) ||
        (page == OLED_PAGE_GIMBAL_TRACKER) ||
        (page == OLED_PAGE_GIMBAL_TRACKER_PITCH) ||
        (page == OLED_PAGE_IMU) ||
        (page == OLED_PAGE_SENSOR)) ? 1U : 0U;
}

static void menu_next_debug_page(void)
{
    switch (g_oledPage) {
        case OLED_PAGE_VISION_RECEIVER:
            g_oledPage = OLED_PAGE_GIMBAL_VISION_ADAPTER;
            break;
        case OLED_PAGE_GIMBAL_VISION_ADAPTER:
            g_oledPage = OLED_PAGE_GIMBAL_VISION_YAW;
            break;
        case OLED_PAGE_GIMBAL_VISION_YAW:
            g_oledPage = OLED_PAGE_GIMBAL_TRACKER;
            break;
        case OLED_PAGE_GIMBAL_TRACKER:
            g_oledPage = OLED_PAGE_GIMBAL_TRACKER_PITCH;
            break;
        case OLED_PAGE_GIMBAL_TRACKER_PITCH:
            g_oledPage = OLED_PAGE_IMU;
            break;
        case OLED_PAGE_IMU:
            g_oledPage = OLED_PAGE_SENSOR;
            break;
        case OLED_PAGE_SENSOR:
        default:
            g_oledPage = OLED_PAGE_VISION_RECEIVER;
            break;
    }
}
#endif

static void menu_next_param(void)
{
    g_paramItem = (ParamItem)(g_paramItem + 1);
    if (g_paramItem >= PARAM_COUNT) {
        g_paramItem = PARAM_TASK;
    }
}

#if FEATURE_GIMBAL_OLED_TEST
static void menu_enable_dual_vision_tracking(void)
{
    const GimbalFeedback *yaw;
    const GimbalFeedback *pitch;
    uint8_t yawEnabled;
    uint8_t pitchEnabled;

    GimbalTracker_Enable(0U);
    yaw = Gimbal_YawGetFeedback();
    pitch = Gimbal_PitchGetFeedback();

    if ((yaw->position_valid == 0U) ||
        (pitch->position_valid == 0U) ||
        (yaw->world_lock_enabled != 0U)) {
        (void)GimbalVisionYawTracker_Enable(0U);
        (void)GimbalVisionPitchTracker_Enable(0U);
        return;
    }

    yawEnabled = GimbalVisionYawTracker_Enable(1U);
    pitchEnabled = GimbalVisionPitchTracker_Enable(1U);
    if ((yawEnabled == 0U) || (pitchEnabled == 0U)) {
        (void)GimbalVisionYawTracker_Enable(0U);
        (void)GimbalVisionPitchTracker_Enable(0U);
    }
}

static void menu_disable_dual_vision_tracking(void)
{
    (void)GimbalVisionYawTracker_Enable(0U);
    (void)GimbalVisionPitchTracker_Enable(0U);
}
#endif

static void menu_adjust_param(int8_t direction, uint8_t fast)
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
            if (fast != 0U) {
                g_appConfig.servo_angle_deg += (int16_t)(direction * 30);
            } else {
                g_appConfig.servo_angle_deg += (int16_t)(direction * 5);
            }
            break;
        case PARAM_GIMBAL_WORLD_LOCK:
            if (direction > 0) {
                Gimbal_YawEnableWorldLock();
            } else {
                Gimbal_YawDisableWorldLock();
            }
            break;
        default:
            break;
    }

    AppConfig_LimitAll();
}

static void menu_handle_status_key(KeyEvent event)
{
    CarState state = CarState_Get();

#if FEATURE_GIMBAL_OLED_TEST
    if (menu_is_debug_page(g_oledPage) != 0U) {
        if (event == KEY1_SHORT) {
            menu_next_debug_page();
            return;
        }
        if (event == KEY1_LONG) {
            g_oledPage = OLED_PAGE_VISION_PITCH_TUNING;
            return;
        }
        if ((g_oledPage != OLED_PAGE_GIMBAL_TRACKER) &&
            (g_oledPage != OLED_PAGE_GIMBAL_TRACKER_PITCH) &&
            (g_oledPage != OLED_PAGE_GIMBAL_VISION_YAW)) {
            return;
        }
    }

    if (g_oledPage == OLED_PAGE_GIMBAL_VISION_YAW) {
        if (event == KEY2_SHORT) {
            (void)GimbalVisionYawTracker_Enable(1U);
        } else if (event == KEY3_SHORT) {
            (void)GimbalVisionYawTracker_Enable(0U);
        }
        return;
    }

    if (g_oledPage == OLED_PAGE_VISION_PITCH_TUNING) {
        switch (event) {
            case KEY1_SHORT:
                menu_next_main_page();
                break;
            case KEY1_LONG:
                menu_enter_param_page(g_paramItem, 0U);
                break;
            case KEY2_SHORT:
                g_oledPage = OLED_PAGE_VISION_RECEIVER;
                break;
            default:
                break;
        }
        return;
    }

    if (g_oledPage == OLED_PAGE_GIMBAL_VISION_DUAL) {
        switch (event) {
            case KEY1_SHORT:
                menu_next_main_page();
                break;
            case KEY1_LONG:
                menu_enter_param_page(g_paramItem, 0U);
                break;
            case KEY2_SHORT:
                menu_enable_dual_vision_tracking();
                break;
            case KEY3_SHORT:
                menu_disable_dual_vision_tracking();
                break;
            default:
                break;
        }
        return;
    }

    if (g_oledPage == OLED_PAGE_GIMBAL_VISION_PITCH) {
        switch (event) {
            case KEY1_SHORT:
                menu_next_main_page();
                break;
            case KEY1_LONG:
                menu_enter_param_page(g_paramItem, 0U);
                break;
            case KEY2_SHORT:
                (void)GimbalVisionPitchTracker_Enable(1U);
                break;
            case KEY3_SHORT:
                (void)GimbalVisionPitchTracker_Enable(0U);
                break;
            default:
                break;
        }
        return;
    }

    if ((g_oledPage == OLED_PAGE_GIMBAL_TRACKER) ||
        (g_oledPage == OLED_PAGE_GIMBAL_TRACKER_PITCH)) {
        static uint16_t trackerSequence;

        switch (event) {
            case KEY2_SHORT:
            {
                GimbalTargetObservation observation = {
                    .error_x_px =
                        (g_oledPage == OLED_PAGE_GIMBAL_TRACKER) ?
                            80 : 0,
                    .error_y_px =
                        (g_oledPage == OLED_PAGE_GIMBAL_TRACKER_PITCH) ?
                            240 : 0,
                    .valid = 1U,
                    .sequence = ++trackerSequence,
                    .timestamp_ms = 0U
                };

                GimbalTracker_PushObservation(&observation);
                break;
            }
            case KEY2_LONG:
            {
                const GimbalTrackerFeedback *tracker =
                    GimbalTracker_GetFeedback();

                GimbalTracker_Enable(
                    (tracker->enabled == 0U) ? 1U : 0U);
                break;
            }
            case KEY3_SHORT:
            {
                GimbalTargetObservation observation = {
                    .error_x_px =
                        (g_oledPage == OLED_PAGE_GIMBAL_TRACKER) ?
                            -80 : 0,
                    .error_y_px =
                        (g_oledPage == OLED_PAGE_GIMBAL_TRACKER_PITCH) ?
                            -240 : 0,
                    .valid = 1U,
                    .sequence = ++trackerSequence,
                    .timestamp_ms = 0U
                };

                GimbalTracker_PushObservation(&observation);
                break;
            }
            case KEY3_LONG:
                GimbalTracker_ClearObservation();
                break;
            default:
                break;
        }
        return;
    }

    if ((g_oledPage == OLED_PAGE_GIMBAL) ||
        (g_oledPage == OLED_PAGE_GIMBAL_PITCH)) {
        switch (event) {
            case KEY1_SHORT:
                menu_next_main_page();
                break;
            case KEY1_LONG:
                if (g_oledPage == OLED_PAGE_GIMBAL) {
                    menu_enter_param_page(PARAM_GIMBAL_WORLD_LOCK, 1U);
                } else {
                    menu_enter_param_page(g_paramItem, 0U);
                }
                break;
            case KEY2_SHORT:
                if (g_oledPage == OLED_PAGE_GIMBAL_PITCH) {
                    Gimbal_PitchMoveRelativeDeg(10.0f);
                } else {
                    Gimbal_YawMoveRelativeDeg(30.0f);
                }
                break;
            case KEY2_LONG:
                if (g_oledPage == OLED_PAGE_GIMBAL_PITCH) {
                    const GimbalFeedback *pitch = Gimbal_PitchGetFeedback();

                    if (pitch->running != 0U) {
                        Gimbal_PitchStopHold();
                    } else if (pitch->position_valid == 0U) {
                        (void)Gimbal_PitchConfirmZero();
                        Gimbal_PitchStopHold();
                    } else {
                        Gimbal_PitchStopHold();
                    }
                } else {
                    const GimbalFeedback *yaw = Gimbal_YawGetFeedback();

                    if (yaw->running != 0U) {
                        Gimbal_YawStopHold();
                    } else if (yaw->position_valid == 0U) {
                        (void)Gimbal_YawConfirmZero();
                        Gimbal_YawStopHold();
                    } else {
                        Gimbal_YawStopHold();
                    }
                }
                break;
            case KEY3_SHORT:
                if (g_oledPage == OLED_PAGE_GIMBAL_PITCH) {
                    Gimbal_PitchMoveRelativeDeg(-10.0f);
                } else {
                    Gimbal_YawMoveRelativeDeg(-30.0f);
                }
                break;
            case KEY3_LONG:
                if (g_oledPage == OLED_PAGE_GIMBAL_PITCH) {
                    Gimbal_PitchRelease();
                } else {
                    Gimbal_YawRelease();
                }
                break;
            default:
                break;
        }
        return;
    }
#endif

    switch (event) {
        case KEY1_SHORT:
            menu_next_main_page();
            break;
        case KEY1_LONG:
            menu_enter_param_page(g_paramItem, 0U);
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
        case KEY3_SHORT:
            MissionManager_Cancel();
            break;
        case KEY3_LONG:
            Fault_Clear();
            WatchdogMonitor_Reset();
            ObstacleAvoidance_Init();
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
            g_oledPage = g_paramReturnPage;
            if (g_oledPage == OLED_PAGE_PARAM) {
                g_oledPage = OLED_PAGE_STATUS;
            }
            CarState_Set(CAR_STATE_READY);
            break;
        case KEY2_SHORT:
            menu_adjust_param(1, 0U);
            break;
        case KEY2_LONG:
            menu_adjust_param(1, 1U);
            break;
        case KEY3_SHORT:
            menu_adjust_param(-1, 0U);
            break;
        case KEY3_LONG:
            menu_adjust_param(-1, 1U);
            break;
        default:
            break;
    }
}

void Menu_Init(void)
{
    g_oledPage = OLED_PAGE_STATUS;
    g_paramReturnPage = OLED_PAGE_STATUS;
    g_paramItem = PARAM_TASK;
}

void Menu_HandleKeyEvent(KeyEvent event)
{
    if (event == KEY_EVENT_NONE) {
        return;
    }

    if (EmergencyStop_IsActive()) {
        if (event == KEY3_LONG) {
            EmergencyStop_Reset();
            g_oledPage = OLED_PAGE_STATUS;
        }
        return;
    }

    if (event == KEY2_LONG) {
        EmergencyStop_Trigger();
        g_oledPage = OLED_PAGE_STATUS;
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
        case PARAM_GIMBAL_WORLD_LOCK:
            return "WLK";
        default:
            return "ERR";
    }
}

int16_t Menu_GetParamValue(ParamItem item)
{
    switch (item) {
        case PARAM_TASK:
            return (int16_t)(MissionManager_GetSelectedMissionIndex() + 1U);
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
        case PARAM_GIMBAL_WORLD_LOCK:
            return (int16_t)Gimbal_YawGetFeedback()->world_lock_enabled;
        default:
            return 0;
    }
}
