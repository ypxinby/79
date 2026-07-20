#include "app.h"

#include "app_config.h"
#include "app_features.h"
#include "car_controller.h"
#include "car_state.h"
#include "debug_telemetry.h"
#include "emergency_stop.h"
#include "encoder.h"
#include "fault.h"
#include "gimbal.h"
#include "gimbal_tracker.h"
#include "heading_control.h"
#include "imu.h"
#include "key.h"
#include "menu.h"
#include "mission_manager.h"
#include "motor.h"
#include "obstacle_avoidance.h"
#include "obstacle_monitor.h"
#include "obstacle_scanner.h"
#include "obstacle_safety.h"
#include "oled_ui.h"
#include "runtime_snapshot.h"
#include "scheduler_monitor.h"
#include "servo.h"
#include "track_sensor.h"
#include "ultrasonic.h"
#include "watchdog_monitor.h"

volatile uint8_t g_trackRaw;
volatile uint8_t g_trackBlackCount;
volatile int16_t g_trackError;
volatile uint8_t g_keyEvent;
volatile uint8_t g_carStateDebug;
volatile uint8_t g_oledPageDebug;
volatile uint8_t g_paramItemDebug;
volatile uint8_t g_trackModeDebug;
volatile uint8_t g_trackTurnDebug;

static void App_UpdateHeadingObserver(uint32_t elapsed_ms)
{
#if ENABLE_IMU && ENABLE_HEADING_OBSERVER && !ENABLE_HEADING_CONTROL
    const HeadingControlRuntime *heading = HeadingControl_GetRuntime();

    if (!Imu_IsReady()) {
        HeadingControl_Enable(false);
        return;
    }

    if (!heading->target_locked) {
        HeadingControl_LockCurrentYaw(Imu_GetYaw());
        HeadingControl_Enable(true);
    }

    (void)HeadingControl_Update(Imu_GetYaw(), Imu_GetCorrectedGyroZDps(),
        (float)elapsed_ms / 1000.0f);
#else
    (void)elapsed_ms;
#endif
}

static void App_UpdateDebugAndUi(uint32_t timestamp_ms)
{
    g_trackRaw = g_appRuntime.sensor_raw;
    g_trackBlackCount = g_appRuntime.black_count;
    g_trackError = g_appRuntime.line_error;
    g_carStateDebug = (uint8_t)CarState_Get();
    g_oledPageDebug = (uint8_t)Menu_GetPage();
    g_paramItemDebug = (uint8_t)Menu_GetParamItem();
    g_trackModeDebug = (uint8_t)CarController_GetRunMode();
    g_trackTurnDebug =
        (uint8_t)TrackSensor_DetectTurn(g_trackRaw, g_trackError);

    RuntimeSnapshot_Update(timestamp_ms);
    DebugTelemetry_Update(timestamp_ms);
    OledUi_Update_20ms(g_trackRaw, g_trackBlackCount, g_trackError,
        g_keyEvent);
}

void App_Init(void)
{
    Fault_Init();
    EmergencyStop_Init();
    WatchdogMonitor_Init(SystemTime_GetMs());
    RuntimeSnapshot_Init();
    DebugTelemetry_Init();
    Motor_Init();
    Gimbal_Init();
    GimbalTracker_Init();
    Encoder_Reset();
    AppConfig_InitDefault();
#if ENABLE_IMU
    if (Imu_Init()) {
        (void)Imu_CalibrateGyroBias(200U);
        Imu_ResetYaw();
    }
#endif
    HeadingControl_Init();
    CarState_Init();
    Menu_Init();
    MissionManager_Init();
    TrackSensor_Init();
    Ultrasonic_Init();
    ObstacleMonitor_Init();
    ObstacleAvoidance_Init();
    ObstacleSafety_Init();
#if FEATURE_OBSTACLE_SCANNER
    ObstacleScanner_Init();
#endif
    Servo_Init();
    CarController_Init();
    Key_Init();
    OledUi_Init();
    Motor_Stop();
}

void App_Update_20ms(uint32_t elapsed_ms)
{
    uint32_t timestamp_ms = SystemTime_GetMs();

    WatchdogMonitor_ApplyFaultIfNeeded(timestamp_ms);
#if ENABLE_IMU
    Imu_Update((float)elapsed_ms / 1000.0f);
#endif
    App_UpdateHeadingObserver(elapsed_ms);

    Ultrasonic_Update_20ms();
    ObstacleMonitor_Update_20ms();
#if FEATURE_OBSTACLE_SCANNER
    {
        const ObstacleScanFeedback *scan = ObstacleScanner_GetFeedback();

        if (!scan->active && !scan->complete) {
            Servo_SetAngleDeg(g_appConfig.servo_angle_deg);
        }
    }
#else
    Servo_SetAngleDeg(g_appConfig.servo_angle_deg);
#endif

    Key_Update_20ms();
    {
        KeyEvent event = Key_GetEvent();
        if (event != KEY_EVENT_NONE) {
            g_keyEvent = (uint8_t)event;
            Menu_HandleKeyEvent(event);
        }
    }

    WatchdogMonitor_ApplyFaultIfNeeded(SystemTime_GetMs());
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        EmergencyStop_Enforce();
        App_UpdateDebugAndUi(SystemTime_GetMs());
        return;
    }

    MissionManager_Update_20ms(elapsed_ms);
    ObstacleSafety_Update_20ms();
    ObstacleAvoidance_Update_20ms(elapsed_ms);
    CarController_Update_20ms(elapsed_ms);

    App_UpdateDebugAndUi(SystemTime_GetMs());
}
