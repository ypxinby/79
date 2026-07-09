#ifndef OLED_UI_H
#define OLED_UI_H

#include <stdint.h>

void OledUi_Init(void);
void OledUi_Update_20ms(uint8_t raw, uint8_t blackCount, int16_t error,
    uint8_t keyEvent);

#endif
