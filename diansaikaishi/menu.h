#ifndef MENU_H
#define MENU_H

#include <stdint.h>

#include "key.h"

typedef enum {
    OLED_PAGE_STATUS = 0,
    OLED_PAGE_PARAM,
    OLED_PAGE_SENSOR,
    OLED_PAGE_IMU,
    OLED_PAGE_HEADING,
    OLED_PAGE_OBSTACLE,
    OLED_PAGE_GIMBAL,
    OLED_PAGE_GIMBAL_PITCH,
    OLED_PAGE_GIMBAL_TRACKER,
    OLED_PAGE_GIMBAL_TRACKER_PITCH,
    OLED_PAGE_VISION_RECEIVER,
    OLED_PAGE_GIMBAL_VISION_ADAPTER,
    OLED_PAGE_GIMBAL_VISION_PITCH
} OledPage;

typedef enum {
    PARAM_TASK = 0,
    PARAM_BASE_SPEED,
    PARAM_SEEK_HEADING_OFFSET,
    PARAM_SECOND_SEEK_ANGLE,
    PARAM_KP,
    PARAM_KD,
    PARAM_MAX_CORRECTION,
    PARAM_SERVO_ANGLE,
    PARAM_GIMBAL_WORLD_LOCK,
    PARAM_COUNT
} ParamItem;

void Menu_Init(void);
void Menu_HandleKeyEvent(KeyEvent event);
OledPage Menu_GetPage(void);
ParamItem Menu_GetParamItem(void);
const char *Menu_ParamItemToString(ParamItem item);
int16_t Menu_GetParamValue(ParamItem item);

#endif
