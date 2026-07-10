#ifndef APP_FEATURES_H
#define APP_FEATURES_H

/*
 * Feature gates for staged development.
 * Keep IMU-related control disabled until the sensor data is verified.
 */
#define ENABLE_IMU                 (1)
#define ENABLE_HEADING_OBSERVER    (1)
#define ENABLE_HEADING_CONTROL     (0)
#define ENABLE_IMU_ANGLE_TURN      (0)

#endif
