/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "encoder.h"
#include "gimbal.h"
#include "gimbal_tracker.h"
#include "motor.h"
#include "servo.h"
#include "ti_msp_dl_config.h"
#include "ultrasonic.h"
#include "vision_receiver.h"
#include "vision_uart.h"

#define APP_TICK_HZ             (10000U)
#define APP_TICKS_PER_MS        (APP_TICK_HZ / 1000U)
#define GIMBAL_UPDATE_PERIOD_MS (5U)
#define GIMBAL_UPDATE_PENDING_MAX (2U)
#define TRACKER_UPDATE_PERIOD_MS (10U)
#define TRACKER_UPDATE_PENDING_MAX (2U)
#define APP_UPDATE_PERIOD_MS    (20U)
#define VISION_RX_PROCESS_BUDGET (64U)

static volatile bool g_appUpdateDue;
static volatile uint8_t g_gimbalUpdatePending;
static volatile uint8_t g_trackerUpdatePending;
static volatile uint32_t g_localTimeMs;

int main(void)
{
    SYSCFG_DL_init();

    App_Init();
    VisionReceiver_Init();
    VisionUart_Init();

    SysTick_Config(CPUCLK_FREQ / APP_TICK_HZ);
    NVIC_EnableIRQ(GPIO_ENCODERS_INT_IRQN);
    __enable_irq();

    while (1) {
        bool gimbalUpdateDue;
        bool trackerUpdateDue;

        (void)VisionReceiver_Process(g_localTimeMs,
            VISION_RX_PROCESS_BUDGET);

        do {
            __disable_irq();
            if (g_trackerUpdatePending != 0U) {
                g_trackerUpdatePending--;
                trackerUpdateDue = true;
            } else {
                trackerUpdateDue = false;
            }
            __enable_irq();

            if (trackerUpdateDue) {
                GimbalTracker_Update(0.010f);
            }
        } while (trackerUpdateDue);

        do {
            __disable_irq();
            if (g_gimbalUpdatePending != 0U) {
                g_gimbalUpdatePending--;
                gimbalUpdateDue = true;
            } else {
                gimbalUpdateDue = false;
            }
            __enable_irq();

            if (gimbalUpdateDue) {
                Gimbal_Update5ms();
            }
        } while (gimbalUpdateDue);

        if (g_appUpdateDue) {
            g_appUpdateDue = false;
            App_Update_20ms();
        }
    }
}

void SysTick_Handler(void)
{
    static uint8_t tick100usCount;
    static uint8_t controlMsCount;
    static uint8_t gimbalMsCount;
    static uint8_t trackerMsCount;

    Motor_PwmTick100us();
    Gimbal_Tick100us();
    Servo_Tick100us();
    Ultrasonic_Tick100us();

    tick100usCount++;
    if (tick100usCount >= APP_TICKS_PER_MS) {
        tick100usCount = 0;

        g_localTimeMs++;

        controlMsCount++;
        gimbalMsCount++;
        trackerMsCount++;
        if (gimbalMsCount >= GIMBAL_UPDATE_PERIOD_MS) {
            gimbalMsCount = 0;
            if (g_gimbalUpdatePending < GIMBAL_UPDATE_PENDING_MAX) {
                g_gimbalUpdatePending++;
            }
        }

        if (trackerMsCount >= TRACKER_UPDATE_PERIOD_MS) {
            trackerMsCount = 0;
            if (g_trackerUpdatePending < TRACKER_UPDATE_PENDING_MAX) {
                g_trackerUpdatePending++;
            }
        }

        if (controlMsCount >= APP_UPDATE_PERIOD_MS) {
            controlMsCount = 0;
            g_appUpdateDue = true;
        }
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_ENCODERS_INT_IIDX:
            Encoder_HandleGpioInterrupt();
            break;
        default:
            break;
    }
}
