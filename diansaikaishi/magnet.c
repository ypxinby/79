#include "magnet.h"

#include "app_features.h"
#include "ti_msp_dl_config.h"

#if FEATURE_MAGNET_RELAY
#ifndef GPIO_MAGNET_RELAY_RELAY_CTRL_PIN
#error FEATURE_MAGNET_RELAY requires GPIO_MAGNET_RELAY in empty.syscfg
#endif
#endif

static MagnetRuntime g_runtime;

static void magnet_increment(volatile uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

void Magnet_Init(void)
{
#if FEATURE_MAGNET_RELAY
    /* The selected relay is high-level-triggered; low is always OFF. */
    DL_GPIO_clearPins(GPIO_MAGNET_RELAY_PORT,
        GPIO_MAGNET_RELAY_RELAY_CTRL_PIN);
#endif
    g_runtime.initialized = true;
    g_runtime.enabled = false;
    g_runtime.enable_count = 0U;
    g_runtime.disable_count = 0U;
}

bool Magnet_Set(bool enable)
{
    if (!g_runtime.initialized) {
        return false;
    }

    if (!enable) {
        Magnet_ForceOff();
        return true;
    }

#if FEATURE_MAGNET_RELAY
    DL_GPIO_setPins(GPIO_MAGNET_RELAY_PORT,
        GPIO_MAGNET_RELAY_RELAY_CTRL_PIN);
    if (!g_runtime.enabled) {
        magnet_increment(&g_runtime.enable_count);
    }
    g_runtime.enabled = true;
    return true;
#else
    return false;
#endif
}

void Magnet_ForceOff(void)
{
#if FEATURE_MAGNET_RELAY
    DL_GPIO_clearPins(GPIO_MAGNET_RELAY_PORT,
        GPIO_MAGNET_RELAY_RELAY_CTRL_PIN);
#endif
    if (g_runtime.enabled) {
        magnet_increment(&g_runtime.disable_count);
    }
    g_runtime.enabled = false;
}

bool Magnet_IsOn(void)
{
    return g_runtime.enabled;
}

const volatile MagnetRuntime *Magnet_GetRuntime(void)
{
    return &g_runtime;
}
