#include "app.h"

#include <string.h>

#include "app_config.h"
#include "app_features.h"
#include "bluetooth_uart.h"
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
#include "motor_control.h"
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
#include "wheel_speed_estimator.h"

volatile uint8_t g_trackRaw;
volatile uint8_t g_trackBlackCount;
volatile int16_t g_trackError;
volatile uint8_t g_keyEvent;
volatile uint8_t g_carStateDebug;
volatile uint8_t g_oledPageDebug;
volatile uint8_t g_paramItemDebug;
volatile uint8_t g_trackModeDebug;
volatile uint8_t g_trackTurnDebug;

#if FEATURE_BLUETOOTH_UART
#define APP_REMOTE_LINE_MAX             (63U)
#define APP_REMOTE_TOKEN_MAX            (4U)
#define APP_REMOTE_RX_BUDGET            (64U)
#define APP_REMOTE_DISTANCE_MAX_CM      (1000)
#define APP_REMOTE_TIMEOUT_MAX_MS       (120000U)
#define APP_REMOTE_TURN_BASE_TIMEOUT_MS (4000U)
#define APP_REMOTE_TURN_PER_DEG_MS      (40U)
/* Current +90 degree vehicle convention is treated as clockwise. */
#define APP_REMOTE_CLOCKWISE_180_DEG    (180.0f)

typedef enum {
    APP_MOTION_GEAR_LOW = 0,
    APP_MOTION_GEAR_HIGH
} AppMotionGear;

typedef enum {
    APP_REMOTE_ACTION_NONE = 0,
    APP_REMOTE_ACTION_MOVE,
    APP_REMOTE_ACTION_TURN
} AppRemoteAction;

static char g_remoteLine[APP_REMOTE_LINE_MAX + 1U];
static uint8_t g_remoteLineLength;
static bool g_remoteDiscardLine;
static AppMotionGear g_remoteGear = APP_MOTION_GEAR_LOW;
static AppRemoteAction g_remoteAction;

static const char *app_remote_action_name(AppRemoteAction action)
{
    if (action == APP_REMOTE_ACTION_MOVE) {
        return "MOVE";
    }
    if (action == APP_REMOTE_ACTION_TURN) {
        return "TURN";
    }
    return "NONE";
}

static const char *app_remote_gear_name(AppMotionGear gear)
{
    return (gear == APP_MOTION_GEAR_HIGH) ? "HIGH" : "LOW";
}

static void app_remote_send(const char *text)
{
    (void)BluetoothUart_TryWriteString(text);
}

static uint16_t app_remote_append_text(char *buffer, uint16_t length,
    uint16_t capacity, const char *text)
{
    while ((*text != '\0') && ((uint16_t)(length + 1U) < capacity)) {
        buffer[length++] = *text++;
    }
    buffer[length] = '\0';
    return length;
}

static uint16_t app_remote_append_u32(char *buffer, uint16_t length,
    uint16_t capacity, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));

    while ((count != 0U) && ((uint16_t)(length + 1U) < capacity)) {
        buffer[length++] = digits[--count];
    }
    buffer[length] = '\0';
    return length;
}

static uint16_t app_remote_append_float_tenth(char *buffer,
    uint16_t length, uint16_t capacity, float value)
{
    char fraction[3] = {'.', '0', '\0'};
    uint32_t scaled;

    if ((value != value) || (value > 1000000.0f) ||
        (value < -1000000.0f)) {
        return app_remote_append_text(buffer, length, capacity, "NA");
    }
    if (value < 0.0f) {
        length = app_remote_append_text(buffer, length, capacity, "-");
        value = -value;
    }
    scaled = (uint32_t)(value * 10.0f + 0.5f);
    length = app_remote_append_u32(buffer, length, capacity, scaled / 10U);
    fraction[1] = (char)('0' + (scaled % 10U));
    return app_remote_append_text(buffer, length, capacity, fraction);
}

static char app_remote_upper(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char)(value - ('a' - 'A'));
    }
    return value;
}

static char *app_remote_trim(char *text)
{
    char *end;

    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }
    end = text + strlen(text);
    while ((end > text) &&
        ((end[-1] == ' ') || (end[-1] == '\t'))) {
        *--end = '\0';
    }
    return text;
}

static uint8_t app_remote_split(char *line, char **tokens)
{
    uint8_t count = 0U;
    char *cursor = line;

    tokens[count++] = cursor;
    while (*cursor != '\0') {
        if (*cursor == ',') {
            *cursor = '\0';
            if (count >= APP_REMOTE_TOKEN_MAX) {
                return 0U;
            }
            tokens[count++] = cursor + 1;
        }
        cursor++;
    }
    for (uint8_t i = 0U; i < count; i++) {
        tokens[i] = app_remote_trim(tokens[i]);
        if (*tokens[i] == '\0') {
            return 0U;
        }
    }
    return count;
}

static bool app_remote_parse_i32(const char *text, int32_t *value)
{
    bool negative = false;
    uint32_t magnitude = 0U;

    if ((text == (const char *)0) || (value == (int32_t *)0)) {
        return false;
    }
    if ((*text == '+') || (*text == '-')) {
        negative = *text == '-';
        text++;
    }
    if ((*text < '0') || (*text > '9')) {
        return false;
    }
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t)(*text - '0');

        if (magnitude > (2147483647U - digit) / 10U) {
            return false;
        }
        magnitude = magnitude * 10U + digit;
        text++;
    }
    if (*text != '\0') {
        return false;
    }
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}

static bool app_remote_parse_gear(const char *text, AppMotionGear *gear)
{
    if ((strcmp(text, "LOW") == 0) || (strcmp(text, "25") == 0)) {
        *gear = APP_MOTION_GEAR_LOW;
        return true;
    }
    if ((strcmp(text, "HIGH") == 0) || (strcmp(text, "45") == 0)) {
        *gear = APP_MOTION_GEAR_HIGH;
        return true;
    }
    return false;
}

static float app_remote_gear_speed_cmps(AppMotionGear gear)
{
    return (gear == APP_MOTION_GEAR_HIGH) ?
        g_appConfig.motion_high_speed_cmps :
        g_appConfig.motion_low_speed_cmps;
}

static int16_t app_remote_speed_command(AppMotionGear gear)
{
    float maximum = g_appConfig.wheel_control_max_speed_cmps;
    float speed = app_remote_gear_speed_cmps(gear);
    float command;

    if (!(maximum > 0.0f) || !(speed > 0.0f) || (speed > maximum)) {
        return 0;
    }
    command = speed * (float)MOTION_NORMALIZED_COMMAND_MAX / maximum;
    command += 0.5f;
    if (command > (float)MOTION_NORMALIZED_COMMAND_MAX) {
        command = (float)MOTION_NORMALIZED_COMMAND_MAX;
    }
    return (int16_t)command;
}

static uint32_t app_remote_move_timeout_ms(int32_t distance_cm,
    AppMotionGear gear)
{
    uint32_t distance = (uint32_t)((distance_cm < 0) ?
        -distance_cm : distance_cm);
    uint32_t speed = (uint32_t)app_remote_gear_speed_cmps(gear);
    uint32_t expected = (distance * 1000U) / speed;
    uint32_t timeout;

    if (expected > (APP_REMOTE_TIMEOUT_MAX_MS - 2000U) / 2U) {
        return APP_REMOTE_TIMEOUT_MAX_MS;
    }
    timeout = expected * 2U + 2000U;
    return (timeout < 3000U) ? 3000U : timeout;
}

static uint32_t app_remote_turn_timeout_ms(int32_t magnitude_deg)
{
    uint32_t timeout = APP_REMOTE_TURN_BASE_TIMEOUT_MS +
        (uint32_t)magnitude_deg * APP_REMOTE_TURN_PER_DEG_MS;

    if (timeout < g_appConfig.yaw_turn_timeout_ms) {
        timeout = g_appConfig.yaw_turn_timeout_ms;
    }
    return timeout;
}

static bool app_remote_start_action(const MotionAction *action,
    AppRemoteAction remote_action)
{
    const MissionRuntime *mission = MissionManager_GetRuntime();

    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped() ||
        (mission->status == MISSION_STATUS_ERROR)) {
        app_remote_send("ERR,RESET_REQUIRED\r\n");
        return false;
    }
    if ((mission->status == MISSION_STATUS_RUNNING) ||
        (mission->status == MISSION_STATUS_PAUSED)) {
        app_remote_send("BUSY\r\n");
        return false;
    }
    if (!MissionManager_StartTransientAction(action)) {
        app_remote_send("ERR,ACTION_REJECTED\r\n");
        return false;
    }
    g_remoteAction = remote_action;
    return true;
}

static void app_remote_send_status(void)
{
    char response[80];
    uint16_t length = 0U;
    const FaultRecord *fault = Fault_GetRecord();

    response[0] = '\0';
    length = app_remote_append_text(response, length, sizeof(response),
        "STATUS,");
    length = app_remote_append_text(response, length, sizeof(response),
        CarState_ToString(CarState_Get()));
    length = app_remote_append_text(response, length, sizeof(response), ",");
    length = app_remote_append_text(response, length, sizeof(response),
        CarController_RunModeToString(CarController_GetRunMode()));
    length = app_remote_append_text(response, length, sizeof(response), ",");
    length = app_remote_append_u32(response, length, sizeof(response),
        (uint32_t)fault->code);
    length = app_remote_append_text(response, length, sizeof(response), ",");
    length = app_remote_append_text(response, length, sizeof(response),
        app_remote_gear_name(g_remoteGear));
    (void)app_remote_append_text(response, length, sizeof(response), "\r\n");
    app_remote_send(response);
}

static void app_remote_handle_move(char **tokens, uint8_t count)
{
    MotionAction action = {0};
    AppMotionGear gear = g_remoteGear;
    int32_t distance_cm;
    int16_t command;

    if ((count < 2U) || (count > 3U) ||
        !app_remote_parse_i32(tokens[1], &distance_cm) ||
        (distance_cm == 0) ||
        (distance_cm < -APP_REMOTE_DISTANCE_MAX_CM) ||
        (distance_cm > APP_REMOTE_DISTANCE_MAX_CM) ||
        ((count == 3U) && !app_remote_parse_gear(tokens[2], &gear))) {
        app_remote_send("ERR,MOVE,PARAM\r\n");
        return;
    }
    command = app_remote_speed_command(gear);
    if (command <= 0) {
        app_remote_send("ERR,MOVE,SPEED_CONFIG\r\n");
        return;
    }

    action.type = MOTION_ACTION_DRIVE_DISTANCE_HEADING;
    action.timeout_ms = app_remote_move_timeout_ms(distance_cm, gear);
    action.max_retries = 0U;
    action.params.drive_distance_heading.distance_cm = (float)distance_cm;
    action.params.drive_distance_heading.target_yaw_deg = 0.0f;
    action.params.drive_distance_heading.normalized_command = command;
    action.params.drive_distance_heading.lock_current_yaw_on_start = true;
    if (app_remote_start_action(&action, APP_REMOTE_ACTION_MOVE)) {
        g_remoteGear = gear;
        app_remote_send((gear == APP_MOTION_GEAR_HIGH) ?
            "ACK,MOVE,HIGH\r\n" : "ACK,MOVE,LOW\r\n");
    }
}

static void app_remote_handle_turn(char **tokens, uint8_t count)
{
    MotionAction action = {0};
    int32_t angle;
    int32_t magnitude;

    if ((count != 2U) || !app_remote_parse_i32(tokens[1], &angle)) {
        app_remote_send("ERR,TURN,PARAM\r\n");
        return;
    }
    magnitude = (angle < 0) ? -angle : angle;
    if ((magnitude != 45) && (magnitude != 90) && (magnitude != 180)) {
        app_remote_send("ERR,TURN,ANGLE\r\n");
        return;
    }

    action.type = MOTION_ACTION_TURN_TO_YAW;
    action.timeout_ms = app_remote_turn_timeout_ms(magnitude);
    action.max_retries = 0U;
    action.params.turn_to_yaw.angle_deg = (magnitude == 180) ?
        APP_REMOTE_CLOCKWISE_180_DEG : (float)angle;
    action.params.turn_to_yaw.speed_override = MOTION_USE_GLOBAL_SPEED;
    if (app_remote_start_action(&action, APP_REMOTE_ACTION_TURN)) {
        app_remote_send("ACK,TURN\r\n");
    }
}

static void app_remote_handle_line(char *line)
{
    char *tokens[APP_REMOTE_TOKEN_MAX];
    uint8_t count;

    for (char *cursor = line; *cursor != '\0'; cursor++) {
        *cursor = app_remote_upper(*cursor);
    }
    count = app_remote_split(line, tokens);
    if (count == 0U) {
        app_remote_send("ERR,FORMAT\r\n");
        return;
    }

    if ((strcmp(tokens[0], "PING") == 0) && (count == 1U)) {
        app_remote_send("PONG\r\n");
    } else if ((strcmp(tokens[0], "STATUS") == 0) && (count == 1U)) {
        app_remote_send_status();
    } else if (strcmp(tokens[0], "MOVE") == 0) {
        app_remote_handle_move(tokens, count);
    } else if (strcmp(tokens[0], "TURN") == 0) {
        app_remote_handle_turn(tokens, count);
    } else if ((strcmp(tokens[0], "SPEED") == 0) && (count == 2U)) {
        AppMotionGear gear;

        if (!app_remote_parse_gear(tokens[1], &gear)) {
            app_remote_send("ERR,SPEED,PARAM\r\n");
        } else {
            g_remoteGear = gear;
            app_remote_send((gear == APP_MOTION_GEAR_HIGH) ?
                "ACK,SPEED,HIGH\r\n" : "ACK,SPEED,LOW\r\n");
        }
    } else if ((strcmp(tokens[0], "STOP") == 0) && (count == 1U)) {
        g_remoteAction = APP_REMOTE_ACTION_NONE;
        CarController_Stop();
        MissionManager_Cancel();
        app_remote_send("ACK,STOP\r\n");
    } else if ((strcmp(tokens[0], "ESTOP") == 0) && (count == 1U)) {
        g_remoteAction = APP_REMOTE_ACTION_NONE;
        EmergencyStop_Trigger();
        app_remote_send("ACK,ESTOP\r\n");
    } else if ((strcmp(tokens[0], "RESET") == 0) && (count == 1U)) {
        g_remoteAction = APP_REMOTE_ACTION_NONE;
        if (App_ResetToReady()) {
            app_remote_send("ACK,RESET\r\n");
        } else {
            app_remote_send("ERR,RESET\r\n");
        }
    } else if ((strcmp(tokens[0], "MAGNET") == 0) && (count == 2U) &&
        ((strcmp(tokens[1], "ON") == 0) ||
         (strcmp(tokens[1], "OFF") == 0))) {
        app_remote_send("ERR,MAGNET,NOT_IMPLEMENTED\r\n");
    } else {
        app_remote_send("ERR,UNKNOWN\r\n");
    }
}

static void app_remote_process_rx(void)
{
    uint8_t byte;
    uint8_t processed = 0U;

    while ((processed < APP_REMOTE_RX_BUDGET) &&
        BluetoothUart_TryReadByte(&byte)) {
        processed++;
        if ((byte == '\r') || (byte == '\n')) {
            if (g_remoteDiscardLine) {
                g_remoteDiscardLine = false;
                g_remoteLineLength = 0U;
                app_remote_send("ERR,FRAME_TOO_LONG\r\n");
            } else if (g_remoteLineLength != 0U) {
                g_remoteLine[g_remoteLineLength] = '\0';
                app_remote_handle_line(g_remoteLine);
                g_remoteLineLength = 0U;
            }
            continue;
        }
        if (g_remoteDiscardLine) {
            continue;
        }
        if (g_remoteLineLength >= APP_REMOTE_LINE_MAX) {
            g_remoteDiscardLine = true;
            g_remoteLineLength = 0U;
            continue;
        }
        g_remoteLine[g_remoteLineLength++] = (char)byte;
    }
}

static void app_remote_update_result(void)
{
    const MissionRuntime *mission;
    char response[96];
    uint16_t length = 0U;

    if (g_remoteAction == APP_REMOTE_ACTION_NONE) {
        return;
    }
    mission = MissionManager_GetRuntime();
    if (mission->status == MISSION_STATUS_DONE) {
        length = app_remote_append_text(response, length, sizeof(response),
            "DONE,");
        length = app_remote_append_text(response, length, sizeof(response),
            app_remote_action_name(g_remoteAction));
        (void)app_remote_append_text(response, length, sizeof(response),
            "\r\n");
        app_remote_send(response);
        g_remoteAction = APP_REMOTE_ACTION_NONE;
    } else if (mission->status == MISSION_STATUS_ERROR) {
        length = app_remote_append_text(response, length, sizeof(response),
            "ERR,");
        length = app_remote_append_text(response, length, sizeof(response),
            app_remote_action_name(g_remoteAction));
        length = app_remote_append_text(response, length, sizeof(response),
            ",");
        length = app_remote_append_u32(response, length, sizeof(response),
            mission->last_error_code);
        if ((g_remoteAction == APP_REMOTE_ACTION_TURN) &&
            (mission->last_error_code ==
                (uint16_t)MOTION_ERROR_TURN_TIMEOUT)) {
            length = app_remote_append_text(response, length,
                sizeof(response), ",E=");
            length = app_remote_append_float_tenth(response, length,
                sizeof(response), g_appRuntime.yaw_turn_error_deg);
            length = app_remote_append_text(response, length,
                sizeof(response), ",Y=");
            length = app_remote_append_float_tenth(response, length,
                sizeof(response), Imu_GetYaw());
            length = app_remote_append_text(response, length,
                sizeof(response), ",G=");
            length = app_remote_append_float_tenth(response, length,
                sizeof(response), Imu_GetCorrectedGyroZDps());
            length = app_remote_append_text(response, length,
                sizeof(response), ",MS=");
            length = app_remote_append_u32(response, length,
                sizeof(response), mission->action_elapsed_ms);
        }
        (void)app_remote_append_text(response, length, sizeof(response),
            "\r\n");
        app_remote_send(response);
        g_remoteAction = APP_REMOTE_ACTION_NONE;
    } else if (!MissionManager_IsTransientAction()) {
        app_remote_send("CANCELLED\r\n");
        g_remoteAction = APP_REMOTE_ACTION_NONE;
    }
}
#endif

#if FEATURE_WHEEL_SPEED_TEST
static void App_UpdateWheelSpeedTest(uint32_t elapsed_ms)
{
    ObstacleSafety_Update_20ms();
    if ((CarState_Get() == CAR_STATE_RUNNING) &&
        !CarController_IsSafetyHoldActive()) {
        MotorControl_SetSpeedTargetCmps(
            (float)WHEEL_SPEED_TEST_TARGET_CMPS,
            (float)WHEEL_SPEED_TEST_TARGET_CMPS);
    } else {
        MotorControl_Stop();
    }
    MotorControl_Update(elapsed_ms);
}
#endif

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
#if FEATURE_GIMBAL_MOTION_CONTROL
    Gimbal_Init();
    GimbalTracker_Init();
#endif
    Encoder_Reset();
    AppConfig_InitDefault();
    WheelSpeedEstimator_Init();
    MotorControl_Init();
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
#if FEATURE_BLUETOOTH_UART
    g_remoteLineLength = 0U;
    g_remoteDiscardLine = false;
    g_remoteGear = APP_MOTION_GEAR_LOW;
    g_remoteAction = APP_REMOTE_ACTION_NONE;
#endif
}

bool App_ResetToReady(void)
{
    Motor_Stop();
    if (EmergencyStop_IsActive()) {
        EmergencyStop_Reset();
    } else {
        Fault_Clear();
        WatchdogMonitor_Reset();
        ObstacleAvoidance_Init();
        CarController_ResetRuntime();
        MissionManager_Reset();
        ObstacleSafety_Init();
#if FEATURE_WHEEL_SPEED_CONTROL
        MotorControl_Reset();
#endif
    }
#if FEATURE_BLUETOOTH_UART
    g_remoteAction = APP_REMOTE_ACTION_NONE;
    g_remoteGear = APP_MOTION_GEAR_LOW;
#endif
    return !EmergencyStop_IsActive() && !WatchdogMonitor_HasTripped() &&
        (Fault_GetRecord()->code == FAULT_CODE_NONE) &&
        (CarState_Get() == CAR_STATE_READY);
}

void App_Update_20ms(uint32_t elapsed_ms)
{
    uint32_t timestamp_ms = SystemTime_GetMs();

#if FEATURE_WHEEL_SPEED_ESTIMATOR
    WheelSpeedEstimator_Update(elapsed_ms);
#endif
    WatchdogMonitor_ApplyFaultIfNeeded(timestamp_ms);
#if ENABLE_IMU
    Imu_Update(elapsed_ms);
#endif
#if FEATURE_BLUETOOTH_UART
    app_remote_process_rx();
#endif

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
#if FEATURE_WHEEL_SPEED_CONTROL
        MotorControl_Update(elapsed_ms);
#endif
        EmergencyStop_Enforce();
#if FEATURE_BLUETOOTH_UART
        app_remote_update_result();
#endif
        App_UpdateDebugAndUi(SystemTime_GetMs());
        return;
    }

#if FEATURE_WHEEL_SPEED_TEST
    App_UpdateWheelSpeedTest(elapsed_ms);
#else
    MissionManager_Update_20ms(elapsed_ms);
    ObstacleSafety_Update_20ms();
    ObstacleAvoidance_Update_20ms(elapsed_ms);
    CarController_Update_20ms(elapsed_ms);
#if FEATURE_WHEEL_SPEED_CONTROL
    MotorControl_Update(elapsed_ms);
#endif
#endif

#if FEATURE_BLUETOOTH_UART
    app_remote_update_result();
#endif

    App_UpdateDebugAndUi(SystemTime_GetMs());
}
