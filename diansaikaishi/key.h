#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#ifndef KEY_PRESSED_LEVEL
#define KEY_PRESSED_LEVEL       (0)
#endif

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY1_SHORT,
    KEY1_LONG,
    KEY2_SHORT,
    KEY2_LONG,
    KEY3_SHORT,
    KEY3_LONG
} KeyEvent;

void Key_Init(void);
void Key_Update_20ms(void);
KeyEvent Key_GetEvent(void);

#endif
