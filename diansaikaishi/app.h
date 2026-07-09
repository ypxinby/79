#ifndef APP_H
#define APP_H

#include <stdint.h>

extern volatile uint8_t g_trackRaw;
extern volatile uint8_t g_trackBlackCount;
extern volatile int16_t g_trackError;
extern volatile uint8_t g_keyEvent;
extern volatile uint8_t g_carStateDebug;
extern volatile uint8_t g_oledPageDebug;
extern volatile uint8_t g_paramItemDebug;

void App_Init(void);
void App_Update_20ms(void);

#endif
