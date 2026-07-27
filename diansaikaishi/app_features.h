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
/* Gimbal motion is frozen and its PB6/PB7 pitch pins are reassigned to the
 * HC-06 Bluetooth UART. Set back to 1 only after moving the gimbal pins. */
#define FEATURE_GIMBAL_MOTION_CONTROL (0)
#define FEATURE_GIMBAL_OLED_TEST   (0)
/* HC-06 on UART1: PB6 MCU TX -> module RXD; PB7 MCU RX <- module TXD. */
#define FEATURE_BLUETOOTH_UART     (1)
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
