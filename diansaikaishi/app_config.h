#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* P3 measured chassis and current ENCA both-edge x2 decoder calibration. */
#define WHEEL_ENCODER_PPR_X2_DEFAULT       (734U)
#define WHEEL_DIAMETER_CM_DEFAULT          (6.5f)
#define WHEEL_TRACK_CM_DEFAULT             (11.25f)
#define LEFT_ENCODER_DIRECTION_DEFAULT     (-1)
#define RIGHT_ENCODER_DIRECTION_DEFAULT    (1)

/* P4 control values; fast-settling additions still require vehicle retest. */
#define WHEEL_CONTROL_MAX_SPEED_CMPS_DEFAULT      (60.0f)
#define WHEEL_CONTROL_LEFT_KP_DEFAULT             (4.0f)
#define WHEEL_CONTROL_LEFT_KI_DEFAULT             (1.0f)
#define WHEEL_CONTROL_RIGHT_KP_DEFAULT            (4.0f)
#define WHEEL_CONTROL_RIGHT_KI_DEFAULT            (1.0f)
#define WHEEL_CONTROL_LEFT_KP_OVERSPEED_DEFAULT   (5.0f)
#define WHEEL_CONTROL_RIGHT_KP_OVERSPEED_DEFAULT  (5.0f)
#define WHEEL_CONTROL_LEFT_FF_GAIN_DEFAULT        (0.5f)
#define WHEEL_CONTROL_RIGHT_FF_GAIN_DEFAULT       (0.5f)
#define WHEEL_CONTROL_LEFT_FF_GAIN_LEGACY         (1.0f)
#define WHEEL_CONTROL_RIGHT_FF_GAIN_LEGACY        (1.0f)
#define WHEEL_CONTROL_INTEGRAL_LIMIT_DEFAULT      (300.0f)
#define WHEEL_CONTROL_INTEGRAL_RELEASE_MULTIPLIER_DEFAULT (4.0f)
#define WHEEL_CONTROL_MAX_ACCEL_CMPS2_DEFAULT     (60.0f)
#define WHEEL_CONTROL_MAX_DECEL_CMPS2_DEFAULT     (120.0f)
#define WHEEL_CONTROL_TARGET_TIMEOUT_MS_DEFAULT   (100U)

/*
 * P5.1 conservative FOLLOW values. Commands remain normalized -1000..1000;
 * these are initial vehicle-test values and are intentionally independent of
 * the validated legacy track_kp/track_kd settings.
 */
#define LINE_CONTROL_V2_ERROR_FILTER_ALPHA_DEFAULT       (0.35f)
#define LINE_CONTROL_V2_DERIV_FILTER_ALPHA_DEFAULT       (0.20f)
#define LINE_CONTROL_V2_KP_DEFAULT                       (0.35f)
#define LINE_CONTROL_V2_KD_DEFAULT                       (0.010f)
#define LINE_CONTROL_V2_MAX_CORRECTION_DEFAULT           (120)
#define LINE_CONTROL_V2_MIN_RUNNING_COMMAND_DEFAULT      (50)
#define LINE_CONTROL_V2_MIN_DT_MS_DEFAULT                 (5U)
#define LINE_CONTROL_V2_MAX_DT_MS_DEFAULT                 (100U)

typedef struct {
    uint8_t target_laps;
    uint8_t min_target_laps;
    uint8_t max_target_laps;

    int16_t base_speed;
    int16_t min_base_speed;
    int16_t max_base_speed;

    int16_t search_speed;
    int16_t min_search_speed;
    int16_t max_search_speed;

    int16_t recover_speed;
    int16_t min_recover_speed;
    int16_t max_recover_speed;

    int16_t turn_speed;
    int16_t min_turn_speed;
    int16_t max_turn_speed;

    int16_t max_correction;
    int16_t min_max_correction;
    int16_t max_max_correction;

    int16_t track_kp;
    int16_t track_kd;
    int16_t track_scale;

    uint8_t start_line_threshold;
    uint8_t lost_line_threshold;
    uint16_t lap_cooldown_ms;
    uint16_t lost_recover_max_ms;
    uint16_t turn_min_ms;
    uint16_t turn_max_ms;
    uint32_t yaw_turn_timeout_ms;

    float gyro_deadband_dps;

    int16_t heading_kp;
    int16_t heading_kd;
    int16_t heading_scale;
    int16_t heading_max_correction;
    int16_t heading_enable_error;
    int16_t heading_enable_derivative;
    uint16_t heading_lock_delay_ms;
    int16_t seek_heading_offset_deg;
    int16_t second_seek_angle_deg;

    int16_t servo_angle_deg;
    int16_t min_servo_angle_deg;
    int16_t max_servo_angle_deg;

    int16_t avoid_turn_out_deg;
    uint16_t avoid_drive_out_ms;
    int16_t avoid_turn_to_line_deg;
    uint16_t avoid_wait_before_ms;
    uint16_t avoid_resume_grace_ms;
    uint16_t avoid_reacquire_settle_ms;
    uint32_t avoid_reacquire_timeout_ms;

    /* Pulses per wheel revolution after the current ENCA both-edge x2 decode. */
    uint32_t encoder_ppr_x2;
    float wheel_diameter_cm;
    float wheel_track_cm;
    int8_t left_encoder_direction;
    int8_t right_encoder_direction;

    float wheel_control_max_speed_cmps;
    float wheel_control_left_kp;
    float wheel_control_left_ki;
    float wheel_control_right_kp;
    float wheel_control_right_ki;
    float wheel_control_left_kp_overspeed;
    float wheel_control_right_kp_overspeed;
    float wheel_control_left_feedforward_gain;
    float wheel_control_right_feedforward_gain;
    float wheel_control_integral_limit;
    float wheel_control_integral_release_multiplier;
    float wheel_control_max_accel_cmps2;
    float wheel_control_max_decel_cmps2;
    uint32_t wheel_control_target_timeout_ms;

    float line_control_v2_error_filter_alpha;
    float line_control_v2_derivative_filter_alpha;
    float line_control_v2_kp;
    float line_control_v2_kd;
    int16_t line_control_v2_max_correction;
    int16_t line_control_v2_min_running_command;
    uint16_t line_control_v2_min_dt_ms;
    uint16_t line_control_v2_max_dt_ms;
} AppConfig;

extern AppConfig g_appConfig;

void AppConfig_InitDefault(void);
void AppConfig_LimitAll(void);
void AppConfig_IncreaseTargetLap(void);
void AppConfig_DecreaseTargetLap(void);

#endif
