#ifndef APP_FEATURES_H
#define APP_FEATURES_H

/*
 * Feature gates for staged development.
 * Keep IMU-related control disabled until the sensor data is verified.
 */
#define APP_PROFILE_DEVELOPMENT    (0)
#define APP_PROFILE_COMPETITION    (1)

#define APP_PROFILE                APP_PROFILE_COMPETITION

#define FEATURE_OBSTACLE_SCANNER   (0)
#define FEATURE_GIMBAL_TEST_AUTO_RUN (0)
#define FEATURE_GIMBAL_OLED_TEST   (1)
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
/* Minimal closed-loop test: select 10/20/30 cm/s and start with K2 short. */
#define FEATURE_WHEEL_SPEED_TEST    (1)
#define WHEEL_SPEED_TEST_TARGET_CMPS (30)

#if FEATURE_WHEEL_SPEED_CONTROL && !FEATURE_WHEEL_SPEED_ESTIMATOR
#error FEATURE_WHEEL_SPEED_CONTROL requires FEATURE_WHEEL_SPEED_ESTIMATOR
#endif

#if FEATURE_WHEEL_SPEED_TEST && !FEATURE_WHEEL_SPEED_CONTROL
#error FEATURE_WHEEL_SPEED_TEST requires FEATURE_WHEEL_SPEED_CONTROL
#endif

#if FEATURE_WHEEL_SPEED_TEST && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 10) && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 20) && \
    (WHEEL_SPEED_TEST_TARGET_CMPS != 30)
#error WHEEL_SPEED_TEST_TARGET_CMPS must be 10, 20, or 30
#endif

#define ENABLE_IMU                 (1)
#define ENABLE_HEADING_OBSERVER    (1)
#define ENABLE_HEADING_CONTROL     (1)
#define ENABLE_IMU_ANGLE_TURN      (0)

#endif
