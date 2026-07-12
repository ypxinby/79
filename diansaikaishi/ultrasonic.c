#include "ultrasonic.h"

#include "ti_msp_dl_config.h"

#ifndef GPIO_ULTRASONIC_PORT
#define GPIO_ULTRASONIC_PORT             (GPIOA)
#endif

#ifndef GPIO_ULTRASONIC_HC_TRIG_PIN
#define GPIO_ULTRASONIC_HC_TRIG_PIN      (DL_GPIO_PIN_8)
#endif

#ifndef GPIO_ULTRASONIC_HC_ECHO_PIN
#define GPIO_ULTRASONIC_HC_ECHO_PIN      (DL_GPIO_PIN_9)
#endif

#define ULTRASONIC_TRIGGER_PERIOD_TICKS  (5U)
#define ULTRASONIC_TRIG_PULSE_TICKS      (1U)
#define ULTRASONIC_ECHO_TIMEOUT_TICKS    (300U)

typedef enum {
    ULTRASONIC_STATE_IDLE = 0,
    ULTRASONIC_STATE_TRIGGER,
    ULTRASONIC_STATE_WAIT_RISING,
    ULTRASONIC_STATE_WAIT_FALLING,
    ULTRASONIC_STATE_DONE,
    ULTRASONIC_STATE_TIMEOUT
} UltrasonicState;

static UltrasonicFeedback g_ultrasonicFeedback;
static volatile bool g_triggerRequest;
static UltrasonicState g_ultrasonicState;
static uint16_t g_stateTicks;
static uint16_t g_echoTicks;

static bool ultrasonic_echo_is_high(void)
{
    return (DL_GPIO_readPins(GPIO_ULTRASONIC_PORT,
        GPIO_ULTRASONIC_HC_ECHO_PIN) != 0U);
}

static void ultrasonic_start_trigger(void)
{
    g_triggerRequest = false;
    g_ultrasonicState = ULTRASONIC_STATE_TRIGGER;
    g_stateTicks = 0U;
    g_echoTicks = 0U;
    g_ultrasonicFeedback.measurement_valid = false;
    DL_GPIO_setPins(GPIO_ULTRASONIC_PORT, GPIO_ULTRASONIC_HC_TRIG_PIN);

    if (g_ultrasonicFeedback.trigger_count < UINT16_MAX) {
        g_ultrasonicFeedback.trigger_count++;
    }
}

static void ultrasonic_finish_measurement(uint16_t echoTicks)
{
    g_ultrasonicFeedback.echo_ticks_100us = echoTicks;
    g_ultrasonicFeedback.distance_cm =
        (uint16_t)(((uint32_t)echoTicks * 100U + 29U) / 58U);
    g_ultrasonicFeedback.measurement_valid = true;
    g_ultrasonicState = ULTRASONIC_STATE_DONE;
    g_stateTicks = 0U;
}

static void ultrasonic_finish_timeout(void)
{
    g_ultrasonicFeedback.measurement_valid = false;
    g_ultrasonicFeedback.echo_ticks_100us = 0U;
    g_ultrasonicFeedback.distance_cm = 0U;
    g_ultrasonicState = ULTRASONIC_STATE_TIMEOUT;
    g_stateTicks = 0U;
}

static void ultrasonic_request_trigger(void)
{
    if ((g_ultrasonicState == ULTRASONIC_STATE_IDLE) ||
        (g_ultrasonicState == ULTRASONIC_STATE_DONE) ||
        (g_ultrasonicState == ULTRASONIC_STATE_TIMEOUT)) {
        g_triggerRequest = true;
    }
}

void Ultrasonic_Init(void)
{
    g_ultrasonicFeedback.echo_high = false;
    g_ultrasonicFeedback.measurement_valid = false;
    g_ultrasonicFeedback.distance_cm = 0U;
    g_ultrasonicFeedback.echo_ticks_100us = 0U;
    g_ultrasonicFeedback.trigger_count = 0U;
    g_ultrasonicFeedback.update_count = 0U;
    g_ultrasonicFeedback.state = (uint8_t)ULTRASONIC_STATE_IDLE;
    g_triggerRequest = false;
    g_ultrasonicState = ULTRASONIC_STATE_IDLE;
    g_stateTicks = 0U;
    g_echoTicks = 0U;

    DL_GPIO_clearPins(GPIO_ULTRASONIC_PORT, GPIO_ULTRASONIC_HC_TRIG_PIN);
    DL_GPIO_enableOutput(GPIO_ULTRASONIC_PORT,
        GPIO_ULTRASONIC_HC_TRIG_PIN);
}

void Ultrasonic_Tick100us(void)
{
    bool echoHigh = ultrasonic_echo_is_high();

    g_ultrasonicFeedback.echo_high = echoHigh;
    g_ultrasonicFeedback.state = (uint8_t)g_ultrasonicState;

    if (g_triggerRequest) {
        ultrasonic_start_trigger();
        g_ultrasonicFeedback.state = (uint8_t)g_ultrasonicState;
        return;
    }

    switch (g_ultrasonicState) {
        case ULTRASONIC_STATE_TRIGGER:
            g_stateTicks++;
            if (g_stateTicks >= ULTRASONIC_TRIG_PULSE_TICKS) {
                DL_GPIO_clearPins(GPIO_ULTRASONIC_PORT,
                    GPIO_ULTRASONIC_HC_TRIG_PIN);
                g_ultrasonicState = ULTRASONIC_STATE_WAIT_RISING;
                g_stateTicks = 0U;
            }
            break;

        case ULTRASONIC_STATE_WAIT_RISING:
            if (echoHigh) {
                g_ultrasonicState = ULTRASONIC_STATE_WAIT_FALLING;
                g_stateTicks = 0U;
                g_echoTicks = 0U;
            } else {
                g_stateTicks++;
                if (g_stateTicks >= ULTRASONIC_ECHO_TIMEOUT_TICKS) {
                    ultrasonic_finish_timeout();
                }
            }
            break;

        case ULTRASONIC_STATE_WAIT_FALLING:
            if (echoHigh) {
                g_echoTicks++;
                if (g_echoTicks >= ULTRASONIC_ECHO_TIMEOUT_TICKS) {
                    ultrasonic_finish_timeout();
                }
            } else {
                ultrasonic_finish_measurement(g_echoTicks);
            }
            break;

        case ULTRASONIC_STATE_IDLE:
        case ULTRASONIC_STATE_DONE:
        case ULTRASONIC_STATE_TIMEOUT:
        default:
            break;
    }

    g_ultrasonicFeedback.state = (uint8_t)g_ultrasonicState;
}

void Ultrasonic_Update_20ms(void)
{
    static uint8_t triggerDivider;

    g_ultrasonicFeedback.echo_high = ultrasonic_echo_is_high();

    if (g_ultrasonicFeedback.update_count < UINT16_MAX) {
        g_ultrasonicFeedback.update_count++;
    }

    triggerDivider++;
    if (triggerDivider >= ULTRASONIC_TRIGGER_PERIOD_TICKS) {
        triggerDivider = 0U;
        ultrasonic_request_trigger();
    }
}

const UltrasonicFeedback *Ultrasonic_GetFeedback(void)
{
    return &g_ultrasonicFeedback;
}
