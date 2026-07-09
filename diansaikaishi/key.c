#include "key.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define KEY_DEBOUNCE_TICKS      (2U)
#define KEY_LONG_TICKS          (40U)

typedef struct {
    bool stablePressed;
    bool lastRawPressed;
    bool longReported;
    uint8_t debounceTicks;
    uint16_t holdTicks;
} KeyState;

static KeyState g_key1;
static KeyState g_key2;
static KeyState g_key3;
static KeyEvent g_pendingEvent;

static bool key_level_is_pressed(uint32_t level)
{
#if KEY_PRESSED_LEVEL
    return level != 0U;
#else
    return level == 0U;
#endif
}

static bool read_key1_raw(void)
{
    return key_level_is_pressed(DL_GPIO_readPins(GPIO_KEYS_PORT,
        GPIO_KEYS_K1_PIN));
}

static bool read_key2_raw(void)
{
    return key_level_is_pressed(DL_GPIO_readPins(GPIO_KEYS_PORT,
        GPIO_KEYS_K2_PIN));
}

static bool read_key3_raw(void)
{
    return key_level_is_pressed(DL_GPIO_readPins(GPIO_KEYS_PORT,
        GPIO_KEYS_K3_PIN));
}

static void post_event(KeyEvent event)
{
    if (g_pendingEvent == KEY_EVENT_NONE) {
        g_pendingEvent = event;
    }
}

static void key_update_one(KeyState *key, bool rawPressed,
    KeyEvent shortEvent, KeyEvent longEvent)
{
    if (rawPressed != key->lastRawPressed) {
        key->lastRawPressed = rawPressed;
        key->debounceTicks = 0;
        return;
    }

    if (key->debounceTicks < KEY_DEBOUNCE_TICKS) {
        key->debounceTicks++;
        return;
    }

    if (rawPressed != key->stablePressed) {
        bool wasLongReported = key->longReported;

        key->stablePressed = rawPressed;
        key->holdTicks = 0;
        key->longReported = false;

        if (!rawPressed && !wasLongReported) {
            post_event(shortEvent);
        }
        return;
    }

    if (!key->stablePressed) {
        return;
    }

    if (key->holdTicks < UINT16_MAX) {
        key->holdTicks++;
    }

    if (!key->longReported) {
        if (key->holdTicks >= KEY_LONG_TICKS) {
            key->longReported = true;
            post_event(longEvent);
        }
    }
}

void Key_Init(void)
{
    g_key1 = (KeyState){0};
    g_key2 = (KeyState){0};
    g_key3 = (KeyState){0};
    g_pendingEvent = KEY_EVENT_NONE;
}

void Key_Update_20ms(void)
{
    key_update_one(&g_key1, read_key1_raw(), KEY1_SHORT, KEY1_LONG);
    key_update_one(&g_key2, read_key2_raw(), KEY2_SHORT, KEY2_LONG);
    key_update_one(&g_key3, read_key3_raw(), KEY3_SHORT, KEY3_LONG);
}

KeyEvent Key_GetEvent(void)
{
    KeyEvent event = g_pendingEvent;

    g_pendingEvent = KEY_EVENT_NONE;
    return event;
}
