#ifndef APP_FEATURES_H
#define APP_FEATURES_H

/*
 * Feature gates for staged development.
 * Keep IMU-related control disabled until the sensor data is verified.
 */
#define APP_PROFILE_DEVELOPMENT    (0)
#define APP_PROFILE_COMPETITION    (1)

#define APP_PROFILE                APP_PROFILE_DEVELOPMENT

#define FEATURE_OBSTACLE_SCANNER   (0)
#define FEATURE_GIMBAL_TEST_AUTO_RUN (0)
/* The legacy two-axis gimbal remains frozen. The new single balance axis must
 * use its own feature gate and pin group instead of enabling this switch. */
#define FEATURE_GIMBAL_MOTION_CONTROL (0)
#define FEATURE_GIMBAL_OLED_TEST   (0)
/* Stage 1 balance-axis regression: reuse the proven PB4/PA1/PA2 open-loop
 * stepper pulse generator and expose only a guarded OLED/key smoke test. */
#define FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST (1)
#define BALANCE_STEPPER_TEST_DELTA_STEPS        (200)
#define BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS  (25U)
/* Stage 2 capture only: the MT6816 AB output is 1024 lines/rev and is
 * decoded at x4 on PB6/PB7, giving 4096 counts/rev. */
#define FEATURE_BALANCE_ENCODER_CAPTURE         (1)
#define BALANCE_STEPPER_COMMAND_STEPS_PER_REV   (3200)
#define BALANCE_ENCODER_COUNTS_PER_REV          (4096)
#define BALANCE_ENCODER_DIRECTION_SIGN          (1)
#define BALANCE_ENCODER_SPEED_SAMPLE_MS         (10U)
/* No physical switches are fitted. Flash keeps LOW/HIGH offsets relative to
 * ZERO; every power-up still requires one manual physical-ZERO confirmation. */
#define FEATURE_BALANCE_SOFT_LIMITS              (1)
#define BALANCE_STEPPER_CAL_JOG_STEPS             (20)
#define BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS        (64)
#define BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT     (10)
#define BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS  (16)
/* Encoder position loop: proportional pulse-rate scheduling with latched
 * no-feedback, persistent following-error and direction checks. */
#define FEATURE_BALANCE_POSITION_CONTROL          (1)
#define BALANCE_POSITION_DEADBAND_COUNTS           (4)
#define BALANCE_POSITION_REENGAGE_COUNTS           (8)
#define BALANCE_POSITION_LIMIT_MARGIN_COUNTS       (8)
#define BALANCE_POSITION_SETTLE_MS                 (100U)
#define BALANCE_POSITION_MIN_STEP_RATE_HZ          (40U)
#define BALANCE_POSITION_MAX_STEP_RATE_HZ          (200U)
#define BALANCE_POSITION_STEP_RATE_KP              (1U)
#define BALANCE_POSITION_NO_FEEDBACK_STEP_THRESHOLD (32U)
#define BALANCE_POSITION_FOLLOW_ERROR_COUNTS       (64)
#define BALANCE_POSITION_FOLLOW_ERROR_DURATION_MS  (200U)
#define BALANCE_POSITION_DIRECTION_ERROR_DURATION_MS (100U)
#define BALANCE_POSITION_TIMEOUT_MULTIPLIER        (3U)
#define BALANCE_POSITION_TIMEOUT_MIN_MS            (3000U)
#define BALANCE_POSITION_TIMEOUT_MARGIN_MS         (3000U)
#define BALANCE_POSITION_TIMEOUT_MAX_MS            (60000U)
/* Continuous actuator-bandwidth test. The generated target completes one
 * smooth LOW->HIGH->LOW cycle per second and uses 60% of each calibrated
 * side around logical ZERO. The normal position-loop speed limit is kept so
 * target tracking error exposes insufficient mechanical bandwidth. */
#define FEATURE_BALANCE_OSCILLATION_TEST            (1)
#define BALANCE_OSCILLATION_RANGE_PERCENT           (60)
#define BALANCE_OSCILLATION_PERIOD_MS               (1000U)
/* K230 receive-only bring-up. The existing robust 40-byte receiver is reused,
 * while the balance profile accepts one meaningful pipe-axis coordinate:
 * frame_height=1, target_center_y=0, target_center_x=axis position. */
#define FEATURE_BALANCE_VISION_MONITOR               (1)
/* Keep the last valid position through roughly 2-3 missing K230 frames.
 * Only after this grace period may the ball controller return toward ZERO. */
#define BALANCE_VISION_STALE_TIMEOUT_MS              (150U)
#define BALANCE_BALL_AXIS_SPAN_MM                     (300U)
#define BALANCE_BALL_MAX_REPORTED_SPEED_MM_S          (3000U)
/* Static ball controller: K230 reports signed millimetres relative to O.
 * The outer PD generates a small encoder-count offset around horizontal and
 * the proven position loop remains the actuator inner loop. It never starts
 * automatically; the BALL page K2 command is still required. */
#define FEATURE_BALANCE_BALL_PD_CONTROL               (1)
#define FEATURE_BALANCE_SERIAL_TUNING                  (1)
#define BALANCE_BALL_PD_MIN_CONFIDENCE                (200U)
#define BALANCE_BALL_PD_VALID_FRAME_COUNT             (3U)
#define BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS        (400U)
#define BALANCE_BALL_PD_MAX_JUMP_MM                   (60U)
#define BALANCE_BALL_PD_MAX_TARGET_ABS_MM             (500U)
#define BALANCE_BALL_PD_KP_COUNTS_PER_MM_X100         (30)
#define BALANCE_BALL_PD_KD_COUNTS_PER_MM_S_X100       (3)
#define BALANCE_BALL_PD_TILT_SIGN                     (1)
#define BALANCE_BALL_PD_MAX_OFFSET_PERCENT            (15U)
#define BALANCE_BALL_PD_MAX_OFFSET_COUNTS             (64U)
#define BALANCE_BALL_PD_TARGET_SLEW_COUNTS_PER_20MS   (4)
#define BALANCE_POSITION_TRACKING_DEADBAND_COUNTS      (2)
#define BALANCE_POSITION_TRACKING_REENGAGE_COUNTS      (3)
/* Do not multiplex the retired $VPT/$VYT ASCII tuning console onto the K230
 * binary stream; accidental prefix matches must never generate UART replies. */
#define FEATURE_VISION_TUNING_CONSOLE                (0)
/* HC-06 is retired. PB6/PB7 now belong to the balance-axis encoder;
 * PB2/PB3 UART_VISION remains the future K230/Raspberry Pi host link. */
#define FEATURE_BLUETOOTH_UART     (0)
/* PB5 drives a 3.3 V high-level-trigger relay module for the 5 V magnet. */
#define FEATURE_MAGNET_RELAY       (1)
/* Bring-up aid: echo every received byte through HC-06. Disable this when the
 * application protocol takes ownership of Bluetooth TX. */
#define FEATURE_BLUETOOTH_RX_ECHO  (0)
/* Keep off until a receiver is confirmed safe for unsolicited $DBG frames. */
#define FEATURE_DEBUG_TELEMETRY_VISION_UART (0)
/* P1 provides the wrapper/heartbeat only; empty.syscfg has no WWDT yet. */
#define FEATURE_HARDWARE_WATCHDOG  (0)
/* Keep software PWM as the default until TIMG8 PWM is verified on hardware. */
#define FEATURE_HW_MOTOR_PWM       (1)
/* P3 observation only; no wheel-speed feedback is connected to control. */
#define FEATURE_WHEEL_SPEED_ESTIMATOR (1)
/* P4A closed-loop control is enabled after hardware validation. */
#define FEATURE_WHEEL_SPEED_CONTROL (1)
/* Set to 0 to restore the validated P4A PI/feedforward behavior. */
#define FEATURE_MOTOR_CONTROL_FAST_SETTLING (1)
/* Minimal closed-loop test: select 10/20/30 cm/s and start with K2 short. */
#define FEATURE_WHEEL_SPEED_TEST    (0)
#define WHEEL_SPEED_TEST_TARGET_CMPS (30)
/*
 * Frozen new-base build: old seek-line, sensor-triggered 90-degree turn and
 * legacy lost-recover modes remain in source for reference but cannot own the
 * motion output when this switch is 0.
 */
#define FEATURE_LEGACY_MOTION_CONTROL (0)
/* P5 FOLLOW path. Legacy rollback requires enabling the legacy switch first. */
#define FEATURE_LINE_CONTROL_V2     (1)
/* Keep the normal K1 page loop compact. Set to 1 to restore P1-P6 detail pages. */
#define FEATURE_OLED_LEGACY_DIAG_PAGES (0)

#if FEATURE_WHEEL_SPEED_CONTROL && !FEATURE_WHEEL_SPEED_ESTIMATOR
#error FEATURE_WHEEL_SPEED_CONTROL requires FEATURE_WHEEL_SPEED_ESTIMATOR
#endif

#if FEATURE_WHEEL_SPEED_TEST && !FEATURE_WHEEL_SPEED_CONTROL
#error FEATURE_WHEEL_SPEED_TEST requires FEATURE_WHEEL_SPEED_CONTROL
#endif

#if FEATURE_LINE_CONTROL_V2 && !FEATURE_WHEEL_SPEED_CONTROL
#error FEATURE_LINE_CONTROL_V2 requires FEATURE_WHEEL_SPEED_CONTROL
#endif

#if !FEATURE_LEGACY_MOTION_CONTROL && !FEATURE_LINE_CONTROL_V2
#error Disabling legacy motion control requires FEATURE_LINE_CONTROL_V2
#endif

#if !FEATURE_LEGACY_MOTION_CONTROL && !FEATURE_WHEEL_SPEED_CONTROL
#error Disabling legacy motion control requires P4 wheel speed control
#endif

#if FEATURE_WHEEL_SPEED_TEST && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 10) && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 20) && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 30)
#error WHEEL_SPEED_TEST_TARGET_CMPS must be 10, 20, or 30
#endif

#define ENABLE_IMU                 (1)
#define ENABLE_HEADING_CONTROL     (1)
#define ENABLE_IMU_ANGLE_TURN      (0)

#if FEATURE_GIMBAL_OLED_TEST && !FEATURE_GIMBAL_MOTION_CONTROL
#error Gimbal OLED test requires gimbal motion control and dedicated pins
#endif

#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST && FEATURE_GIMBAL_MOTION_CONTROL
#error Balance stepper test and legacy gimbal cannot own PB4/PA1/PA2 together
#endif

#if FEATURE_BALANCE_ENCODER_CAPTURE && FEATURE_BLUETOOTH_UART
#error Balance encoder and Bluetooth UART cannot own PB6/PB7 together
#endif

#if FEATURE_BALANCE_SOFT_LIMITS && \
    (!FEATURE_BALANCE_ENCODER_CAPTURE || \
     !FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST)
#error Balance software limits require the balance encoder and stepper
#endif

#if FEATURE_BALANCE_POSITION_CONTROL && !FEATURE_BALANCE_SOFT_LIMITS
#error Balance position control requires calibrated software limits
#endif

#if FEATURE_BALANCE_OSCILLATION_TEST && \
    !FEATURE_BALANCE_POSITION_CONTROL
#error Balance oscillation test requires position control
#endif

#if BALANCE_STEPPER_CAL_JOG_STEPS <= 0
#error BALANCE_STEPPER_CAL_JOG_STEPS must be positive
#endif

#if BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS <= 0
#error BALANCE_SOFT_LIMIT_MIN_SPAN_COUNTS must be positive
#endif

#if (BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT <= 0) || \
    (BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT >= 50)
#error BALANCE_SOFT_LIMIT_TEST_INSET_PERCENT must be in 1..49
#endif

#if BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS <= 0
#error BALANCE_SOFT_LIMIT_TEST_MIN_INSET_COUNTS must be positive
#endif

#if (BALANCE_POSITION_DEADBAND_COUNTS < 0) || \
    (BALANCE_POSITION_REENGAGE_COUNTS <= BALANCE_POSITION_DEADBAND_COUNTS)
#error Balance position deadband/reengage configuration is invalid
#endif

#if BALANCE_POSITION_LIMIT_MARGIN_COUNTS < \
    BALANCE_POSITION_DEADBAND_COUNTS
#error Balance position limit margin must cover the position deadband
#endif

#if (BALANCE_POSITION_MIN_STEP_RATE_HZ == 0) || \
    (BALANCE_POSITION_MAX_STEP_RATE_HZ < \
        BALANCE_POSITION_MIN_STEP_RATE_HZ)
#error Balance position step-rate range is invalid
#endif

#if (BALANCE_POSITION_NO_FEEDBACK_STEP_THRESHOLD == 0) || \
    (BALANCE_POSITION_FOLLOW_ERROR_COUNTS <= 0)
#error Balance position fault thresholds must be positive
#endif

#if (BALANCE_OSCILLATION_RANGE_PERCENT <= 0) || \
    (BALANCE_OSCILLATION_RANGE_PERCENT >= 100)
#error Balance oscillation range must be in 1..99 percent
#endif

#if (BALANCE_OSCILLATION_PERIOD_MS < 200U) || \
    ((BALANCE_OSCILLATION_PERIOD_MS % 40U) != 0U)
#error Balance oscillation period must be >=200 ms and divisible by 40 ms
#endif

#if FEATURE_BALANCE_VISION_MONITOR && \
    (BALANCE_VISION_STALE_TIMEOUT_MS == 0U)
#error Balance vision stale timeout must be positive
#endif

#if FEATURE_BALANCE_VISION_MONITOR && \
    ((BALANCE_BALL_AXIS_SPAN_MM == 0U) || \
     (BALANCE_BALL_AXIS_SPAN_MM > 5000U) || \
     (BALANCE_BALL_MAX_REPORTED_SPEED_MM_S == 0U))
#error Balance ball ASCII protocol ranges are invalid
#endif

#if FEATURE_BALANCE_BALL_PD_CONTROL && \
    (!FEATURE_BALANCE_VISION_MONITOR || \
     !FEATURE_BALANCE_POSITION_CONTROL || \
     !FEATURE_BALANCE_SOFT_LIMITS)
#error Balance ball PD requires vision, position control and soft limits
#endif

#if FEATURE_BALANCE_SERIAL_TUNING && !FEATURE_BALANCE_BALL_PD_CONTROL
#error Balance serial tuning requires balance ball PD control
#endif

#if (BALANCE_POSITION_TRACKING_REENGAGE_COUNTS <= \
        BALANCE_POSITION_TRACKING_DEADBAND_COUNTS) || \
    (BALANCE_POSITION_TRACKING_REENGAGE_COUNTS > \
        BALANCE_POSITION_FOLLOW_ERROR_COUNTS)
#error Balance tracking deadband/reengage configuration is invalid
#endif

#if FEATURE_BALANCE_BALL_PD_CONTROL && \
    ((BALANCE_BALL_PD_VALID_FRAME_COUNT == 0U) || \
     (BALANCE_BALL_PD_VISION_LOST_TIMEOUT_MS <= \
        BALANCE_VISION_STALE_TIMEOUT_MS) || \
     (BALANCE_BALL_PD_MAX_OFFSET_PERCENT == 0U) || \
     (BALANCE_BALL_PD_MAX_OFFSET_PERCENT >= 50U))
#error Balance ball PD safety configuration is invalid
#endif

#if FEATURE_BALANCE_BALL_PD_CONTROL && \
    (BALANCE_BALL_PD_TILT_SIGN != 1) && \
    (BALANCE_BALL_PD_TILT_SIGN != -1)
#error Balance ball PD tilt sign must be 1 or -1
#endif

#if (BALANCE_ENCODER_DIRECTION_SIGN != 1) && \
    (BALANCE_ENCODER_DIRECTION_SIGN != -1)
#error BALANCE_ENCODER_DIRECTION_SIGN must be 1 or -1
#endif

#if BALANCE_STEPPER_COMMAND_STEPS_PER_REV <= 0
#error BALANCE_STEPPER_COMMAND_STEPS_PER_REV must be positive
#endif

#if BALANCE_ENCODER_COUNTS_PER_REV < 0
#error BALANCE_ENCODER_COUNTS_PER_REV cannot be negative
#endif

#if (BALANCE_ENCODER_SPEED_SAMPLE_MS == 0) || \
    (BALANCE_ENCODER_SPEED_SAMPLE_MS > 255)
#error BALANCE_ENCODER_SPEED_SAMPLE_MS must be in 1..255 ms
#endif

#if FEATURE_GIMBAL_MOTION_CONTROL && FEATURE_BLUETOOTH_UART
#error PB6/PB7 cannot be shared by gimbal motion and Bluetooth UART
#endif

#if FEATURE_GIMBAL_MOTION_CONTROL && FEATURE_MAGNET_RELAY
#error PB5 cannot be shared by gimbal pitch STEP and the magnet relay
#endif

#if FEATURE_BLUETOOTH_RX_ECHO && !FEATURE_BLUETOOTH_UART
#error Bluetooth RX echo requires Bluetooth UART
#endif

#endif
