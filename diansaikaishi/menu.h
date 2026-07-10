#ifndef MENU_H
#define MENU_H

#include <stdint.h>

#include "key.h"

typedef enum {
    OLED_PAGE_STATUS = 0,
    OLED_PAGE_PARAM,
    OLED_PAGE_SENSOR,
    OLED_PAGE_IMU,
    OLED_PAGE_HEADING
} OledPage;

typedef enum {
    PARAM_TARGET_LAPS = 0,
    PARAM_KP,
    PARAM_KD,
    PARAM_BASE_SPEED,
    PARAM_MAX_CORRECTION,
    PARAM_START_LINE_THRESHOLD,
    PARAM_LOST_LINE_THRESHOLD,
    PARAM_SEEK_HEADING_OFFSET,
    PARAM_SEEK_TASK_MODE,
    PARAM_COUNT
} ParamItem;

void Menu_Init(void);
void Menu_HandleKeyEvent(KeyEvent event);
OledPage Menu_GetPage(void);
ParamItem Menu_GetParamItem(void);
const char *Menu_ParamItemToString(ParamItem item);
int16_t Menu_GetParamValue(ParamItem item);

#endif
