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

#define ENABLE_IMU                 (1)
#define ENABLE_HEADING_OBSERVER    (1)
#define ENABLE_HEADING_CONTROL     (1)
#define ENABLE_IMU_ANGLE_TURN      (0)

#endif
