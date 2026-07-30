#include "menu.h"

#include "app.h"
#include "app_config.h"
#include "app_features.h"
#include "balance_encoder.h"
#include "balance_position_control.h"
#include "balance_soft_limits.h"
#include "car_controller.h"
#include "car_state.h"
#include "emergency_stop.h"
#include "fault.h"
#include "gimbal.h"
#include "gimbal_stepper.h"
#include "gimbal_tracker.h"
#include "gimbal_vision_pitch_tracker.h"
#include "gimbal_vision_yaw_tracker.h"
#include "line_controller.h"
#include "mission_manager.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "watchdog_monitor.h"

static OledPage g_oledPage;
static OledPage g_paramReturnPage;
static ParamItem g_paramItem;

#define MENU_LINE_BASE_MIN              (100)
#define MENU_LINE_BASE_MAX              (500)
#define MENU_LINE_KP_X100_MIN           (0)
#define MENU_LINE_KP_X100_MAX           (200)
#define MENU_LINE_KD_X1000_MIN          (0)
#define MENU_LINE_KD_X1000_MAX          (100)
#define MENU_WHEEL_FF_X100_MIN          (50)
#define MENU_WHEEL_FF_X100_MAX          (80)

static int16_t menu_clamp_i16(int16_t value, int16_t minimum,
    int16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t menu_float_to_scaled(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    return (int16_t)scaled;
}

static int16_t menu_common_feedforward_x100(void)
{
    float average = (g_appConfig.wheel_control_left_feedforward_gain +
        g_appConfig.wheel_control_right_feedforward_gain) * 0.5f;

    return menu_float_to_scaled(average, 100.0f);
}

static void menu_limit_line_tuning(void)
{
    int16_t kp_x100;
    int16_t kd_x1000;

    g_appConfig.line_control_v2_base_command = menu_clamp_i16(
        g_appConfig.line_control_v2_base_command,
        MENU_LINE_BASE_MIN, MENU_LINE_BASE_MAX);
    g_appConfig.line_control_v2_max_correction = menu_clamp_i16(
        g_appConfig.line_control_v2_max_correction, 0,
        g_appConfig.line_control_v2_base_command);

    kp_x100 = menu_clamp_i16(menu_float_to_scaled(
        g_appConfig.line_control_v2_kp, 100.0f),
        MENU_LINE_KP_X100_MIN, MENU_LINE_KP_X100_MAX);
    kd_x1000 = menu_clamp_i16(menu_float_to_scaled(
        g_appConfig.line_control_v2_kd, 1000.0f),
        MENU_LINE_KD_X1000_MIN, MENU_LINE_KD_X1000_MAX);
    g_appConfig.line_control_v2_kp = (float)kp_x100 / 100.0f;
    g_appConfig.line_control_v2_kd = (float)kd_x1000 / 1000.0f;
}

static void menu_enter_param_page(ParamItem item, uint8_t select_item)
{
    if (CarState_Get() != CAR_STATE_READY) {
        return;
    }
    g_paramReturnPage = g_oledPage;
    if (select_item != 0U) {
        g_paramItem = item;
    }
    g_oledPage = OLED_PAGE_PARAM;
    CarState_Set(CAR_STATE_MENU);
}

static void menu_next_main_page(void)
{
#if FEATURE_OLED_LEGACY_DIAG_PAGES
    if (g_oledPage == OLED_PAGE_STATUS) {
        g_oledPage = OLED_PAGE_SENSOR;
    } else if (g_oledPage == OLED_PAGE_SENSOR) {
        g_oledPage = OLED_PAGE_IMU;
    } else if (g_oledPage == OLED_PAGE_IMU) {
        g_oledPage = OLED_PAGE_IMU_DETAIL;
    } else if (g_oledPage == OLED_PAGE_IMU_DETAIL) {
        g_oledPage = OLED_PAGE_IMU_COUNTERS;
    } else if (g_oledPage == OLED_PAGE_IMU_COUNTERS) {
        g_oledPage = OLED_PAGE_HEADING;
    } else if (g_oledPage == OLED_PAGE_HEADING) {
        g_oledPage = OLED_PAGE_DISTANCE;
    } else if (g_oledPage == OLED_PAGE_DISTANCE) {
        g_oledPage = OLED_PAGE_OBSTACLE;
    } else if (g_oledPage == OLED_PAGE_OBSTACLE) {
        g_oledPage = OLED_PAGE_ENCODER;
    } else if (g_oledPage == OLED_PAGE_ENCODER) {
        g_oledPage = OLED_PAGE_MOTOR_CONTROL;
    } else if (g_oledPage == OLED_PAGE_MOTOR_CONTROL) {
        g_oledPage = OLED_PAGE_MOTOR_CONTROL_DETAIL;
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_MOTOR_CONTROL_DETAIL) {
        g_oledPage = OLED_PAGE_BALANCE_STEPPER_TEST;
#endif
#if FEATURE_BALANCE_VISION_MONITOR
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_BALANCE_STEPPER_TEST) {
#else
    } else if (g_oledPage == OLED_PAGE_MOTOR_CONTROL_DETAIL) {
#endif
        g_oledPage = OLED_PAGE_BALANCE_VISION;
#endif
#if FEATURE_BLUETOOTH_UART
#if FEATURE_BALANCE_VISION_MONITOR
    } else if (g_oledPage == OLED_PAGE_BALANCE_VISION) {
#elif FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_BALANCE_STEPPER_TEST) {
#else
    } else if (g_oledPage == OLED_PAGE_MOTOR_CONTROL_DETAIL) {
#endif
        g_oledPage = OLED_PAGE_BLUETOOTH;
#endif
    } else {
        g_oledPage = OLED_PAGE_STATUS;
    }
#else
    if (g_oledPage == OLED_PAGE_STATUS) {
        g_oledPage = OLED_PAGE_SENSOR;
    } else if (g_oledPage == OLED_PAGE_SENSOR) {
        g_oledPage = OLED_PAGE_HEADING;
    } else if (g_oledPage == OLED_PAGE_HEADING) {
        g_oledPage = OLED_PAGE_DISTANCE;
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_DISTANCE) {
        g_oledPage = OLED_PAGE_BALANCE_STEPPER_TEST;
#endif
#if FEATURE_BALANCE_VISION_MONITOR
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_BALANCE_STEPPER_TEST) {
#else
    } else if (g_oledPage == OLED_PAGE_DISTANCE) {
#endif
        g_oledPage = OLED_PAGE_BALANCE_VISION;
#endif
#if FEATURE_BLUETOOTH_UART
#if FEATURE_BALANCE_VISION_MONITOR
    } else if (g_oledPage == OLED_PAGE_BALANCE_VISION) {
#elif FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    } else if (g_oledPage == OLED_PAGE_BALANCE_STEPPER_TEST) {
#else
    } else if (g_oledPage == OLED_PAGE_DISTANCE) {
#endif
        g_oledPage = OLED_PAGE_BLUETOOTH;
#endif
    } else {
        g_oledPage = OLED_PAGE_STATUS;
    }
#endif
}

#if FEATURE_GIMBAL_OLED_TEST
static uint8_t menu_is_debug_page(OledPage page)
{
    return ((page == OLED_PAGE_VISION_RECEIVER) ||
        (page == OLED_PAGE_GIMBAL_VISION_ADAPTER) ||
        (page == OLED_PAGE_GIMBAL_VISION_YAW) ||
        (page == OLED_PAGE_GIMBAL_TRACKER) ||
        (page == OLED_PAGE_GIMBAL_TRACKER_PITCH)) ? 1U : 0U;
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
    uint8_t line_tuning_changed = 0U;
    uint8_t motor_tuning_changed = 0U;

    switch (g_paramItem) {
        case PARAM_TASK:
            if (direction > 0) {
                (void)MissionManager_SelectNext();
            } else {
                (void)MissionManager_SelectPrevious();
            }
            break;
        case PARAM_BASE_SPEED:
#if FEATURE_LINE_CONTROL_V2
            g_appConfig.line_control_v2_base_command +=
                (int16_t)(direction * 10);
            line_tuning_changed = 1U;
#else
            g_appConfig.base_speed += (int16_t)(direction * 10);
#endif
            break;
        case PARAM_WHEEL_FEEDFORWARD:
        {
            int16_t feedforward_x100 = menu_common_feedforward_x100();

            feedforward_x100 += (int16_t)(direction * 5);
            feedforward_x100 = menu_clamp_i16(feedforward_x100,
                MENU_WHEEL_FF_X100_MIN, MENU_WHEEL_FF_X100_MAX);
            g_appConfig.wheel_control_left_feedforward_gain =
                (float)feedforward_x100 / 100.0f;
            g_appConfig.wheel_control_right_feedforward_gain =
                (float)feedforward_x100 / 100.0f;
            motor_tuning_changed = 1U;
            break;
        }
        case PARAM_KP:
#if FEATURE_LINE_CONTROL_V2
        {
            int16_t kp_x100 = menu_float_to_scaled(
                g_appConfig.line_control_v2_kp, 100.0f);

            kp_x100 += (int16_t)(direction * 5);
            kp_x100 = menu_clamp_i16(kp_x100,
                MENU_LINE_KP_X100_MIN, MENU_LINE_KP_X100_MAX);
            g_appConfig.line_control_v2_kp = (float)kp_x100 / 100.0f;
            line_tuning_changed = 1U;
            break;
        }
#else
            g_appConfig.track_kp += direction;
            break;
#endif
        case PARAM_KD:
#if FEATURE_LINE_CONTROL_V2
        {
            int16_t kd_x1000 = menu_float_to_scaled(
                g_appConfig.line_control_v2_kd, 1000.0f);

            kd_x1000 += direction;
            kd_x1000 = menu_clamp_i16(kd_x1000,
                MENU_LINE_KD_X1000_MIN, MENU_LINE_KD_X1000_MAX);
            g_appConfig.line_control_v2_kd =
                (float)kd_x1000 / 1000.0f;
            line_tuning_changed = 1U;
            break;
        }
#else
            g_appConfig.track_kd += direction;
            break;
#endif
        case PARAM_MAX_CORRECTION:
#if FEATURE_LINE_CONTROL_V2
            g_appConfig.line_control_v2_max_correction +=
                (int16_t)(direction * 10);
            line_tuning_changed = 1U;
#else
            g_appConfig.max_correction += (int16_t)(direction * 10);
#endif
            break;
        case PARAM_SERVO_ANGLE:
            if (fast != 0U) {
                g_appConfig.servo_angle_deg += (int16_t)(direction * 30);
            } else {
                g_appConfig.servo_angle_deg += (int16_t)(direction * 5);
            }
            break;
        case PARAM_GIMBAL_WORLD_LOCK:
#if FEATURE_GIMBAL_MOTION_CONTROL
            if (direction > 0) {
                Gimbal_YawEnableWorldLock();
            } else {
                Gimbal_YawDisableWorldLock();
            }
#endif
            break;
        default:
            break;
    }

    AppConfig_LimitAll();
#if FEATURE_LINE_CONTROL_V2
    if (line_tuning_changed != 0U) {
        menu_limit_line_tuning();
        LineController_ResetControlState();
    }
#else
    (void)line_tuning_changed;
#endif
#if FEATURE_WHEEL_SPEED_CONTROL
    if (motor_tuning_changed != 0U) {
        MotorControl_Reset();
    }
#else
    (void)motor_tuning_changed;
#endif
    (void)fast;
}

static void menu_handle_status_key(KeyEvent event)
{
    CarState state = CarState_Get();

#if FEATURE_BALANCE_VISION_MONITOR
    if (g_oledPage == OLED_PAGE_BALANCE_VISION) {
        if (event == KEY1_SHORT) {
            menu_next_main_page();
        }
        return;
    }
#endif

#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    if (g_oledPage == OLED_PAGE_BALANCE_STEPPER_TEST) {
#if FEATURE_BALANCE_SOFT_LIMITS
        BalanceSoftLimitsRuntime limits;
        BalancePositionRuntime position;

        BalanceSoftLimits_GetSnapshot(&limits);
        BalancePositionControl_GetSnapshot(&position);
        if (position.fault != BALANCE_POSITION_FAULT_NONE) {
            if (event == KEY3_LONG) {
                (void)BalanceSoftLimits_ResetPositionFault();
            }
            return;
        }
        if (BalanceSoftLimits_IsCalibrationActive() != 0U) {
            switch (event) {
                case KEY1_SHORT:
                    break;
                case KEY1_LONG:
                    if (GimbalStepper_GetFeedback()->running != 0U) {
                        GimbalStepper_StopHold();
                    } else {
                        (void)BalanceSoftLimits_CaptureCurrentStage();
                    }
                    break;
                case KEY2_SHORT:
                    GimbalStepper_SetStepHalfPeriodTicks(
                        BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
                    GimbalStepper_MoveRelativeSteps(
                        BALANCE_STEPPER_CAL_JOG_STEPS);
                    break;
                case KEY3_SHORT:
                    GimbalStepper_SetStepHalfPeriodTicks(
                        BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
                    GimbalStepper_MoveRelativeSteps(
                        -BALANCE_STEPPER_CAL_JOG_STEPS);
                    break;
                case KEY3_LONG:
                    GimbalStepper_StopHold();
                    BalanceSoftLimits_AbortCalibration();
                    break;
                default:
                    break;
            }
            return;
        }

        if (BalanceSoftLimits_IsTravelTestActive() != 0U) {
            if ((event == KEY1_LONG) || (event == KEY3_LONG)) {
                BalanceSoftLimits_CancelTravelTest();
            }
            return;
        }

        if (BalanceSoftLimits_IsOscillationTestActive() != 0U) {
            if ((event == KEY1_LONG) || (event == KEY3_LONG)) {
                BalanceSoftLimits_CancelOscillationTest();
            }
            return;
        }

        if (BalanceSoftLimits_IsPositionMoveActive() != 0U) {
            if ((event == KEY1_LONG) || (event == KEY3_LONG)) {
                BalanceSoftLimits_CancelPositionMove();
            }
            return;
        }
#endif
        switch (event) {
            case KEY1_SHORT:
#if FEATURE_BALANCE_SOFT_LIMITS
                BalanceSoftLimits_CancelPositionMove();
#endif
                GimbalStepper_StopHold();
                menu_next_main_page();
                break;
            case KEY1_LONG:
                if (GimbalStepper_GetFeedback()->running != 0U) {
                    GimbalStepper_StopHold();
                } else {
#if FEATURE_BALANCE_SOFT_LIMITS
                    if (limits.zero_confirmation_required != 0U) {
                        (void)BalanceSoftLimits_BeginAtCurrentAsZero();
                    } else {
                        (void)BalanceSoftLimits_StartOscillationTest();
                    }
#else
                    (void)GimbalStepper_ConfirmZero();
                    BalanceEncoder_Reset();
#endif
                }
                break;
            case KEY2_SHORT:
#if FEATURE_BALANCE_SOFT_LIMITS
                (void)BalanceSoftLimits_StartRelativePositionMoveSteps(
                    BALANCE_STEPPER_TEST_DELTA_STEPS);
#else
                GimbalStepper_SetStepHalfPeriodTicks(
                    BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
                GimbalStepper_MoveRelativeSteps(
                    BALANCE_STEPPER_TEST_DELTA_STEPS);
#endif
                break;
            case KEY3_SHORT:
#if FEATURE_BALANCE_SOFT_LIMITS
                (void)BalanceSoftLimits_StartRelativePositionMoveSteps(
                    -BALANCE_STEPPER_TEST_DELTA_STEPS);
#else
                GimbalStepper_SetStepHalfPeriodTicks(
                    BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
                GimbalStepper_MoveRelativeSteps(
                    -BALANCE_STEPPER_TEST_DELTA_STEPS);
#endif
                break;
            case KEY3_LONG:
#if FEATURE_BALANCE_SOFT_LIMITS
                if (limits.recalibration_armed != 0U) {
                    BalanceSoftLimits_CancelRecalibration();
                } else if ((limits.zero_valid != 0U) &&
                    (limits.limits_valid != 0U)) {
                    (void)BalanceSoftLimits_ArmRecalibration();
                } else {
                    GimbalStepper_Release();
                }
#else
                GimbalStepper_Release();
#endif
                break;
            default:
                break;
        }
        return;
    }
#endif

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
            (void)App_ResetToReady();
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
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
            GimbalStepper_Release();
#endif
#if FEATURE_BALANCE_SOFT_LIMITS
            BalanceSoftLimits_AbortCalibration();
#endif
            (void)App_ResetToReady();
            g_oledPage = OLED_PAGE_STATUS;
        }
        return;
    }

    if (event == KEY2_LONG) {
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
        GimbalStepper_StopHold();
#endif
#if FEATURE_BALANCE_SOFT_LIMITS
        BalanceSoftLimits_AbortCalibration();
#endif
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
        case PARAM_WHEEL_FEEDFORWARD:
            return "FF";
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
#if FEATURE_LINE_CONTROL_V2
            return g_appConfig.line_control_v2_base_command;
#else
            return g_appConfig.base_speed;
#endif
        case PARAM_WHEEL_FEEDFORWARD:
            return menu_common_feedforward_x100();
        case PARAM_KP:
#if FEATURE_LINE_CONTROL_V2
            return menu_float_to_scaled(g_appConfig.line_control_v2_kp,
                100.0f);
#else
            return g_appConfig.track_kp;
#endif
        case PARAM_KD:
#if FEATURE_LINE_CONTROL_V2
            return menu_float_to_scaled(g_appConfig.line_control_v2_kd,
                1000.0f);
#else
            return g_appConfig.track_kd;
#endif
        case PARAM_MAX_CORRECTION:
#if FEATURE_LINE_CONTROL_V2
            return g_appConfig.line_control_v2_max_correction;
#else
            return g_appConfig.max_correction;
#endif
        case PARAM_SERVO_ANGLE:
            return g_appConfig.servo_angle_deg;
        case PARAM_GIMBAL_WORLD_LOCK:
#if FEATURE_GIMBAL_MOTION_CONTROL
            return (int16_t)Gimbal_YawGetFeedback()->world_lock_enabled;
#else
            return 0;
#endif
        default:
            return 0;
    }
}
