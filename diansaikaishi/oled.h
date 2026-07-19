#ifndef OLED_H
#define OLED_H

#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t column);
void OLED_PrintChar(char ch);
void OLED_PrintString(const char *text);
void OLED_PrintInt16(int16_t value);
void OLED_PrintUInt16(uint16_t value);
void OLED_PrintBinary7(uint8_t value);
void OLED_PrintBinary8(uint8_t value);

#endif
