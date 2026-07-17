/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "encoder.h"
#include "gimbal.h"
#include "motor.h"
#include "servo.h"
#include "ti_msp_dl_config.h"
#include "ultrasonic.h"

#define APP_TICK_HZ             (10000U)
#define APP_TICKS_PER_MS        (APP_TICK_HZ / 1000U)
#define GIMBAL_UPDATE_PERIOD_MS (5U)
#define GIMBAL_UPDATE_PENDING_MAX (2U)
#define APP_UPDATE_PERIOD_MS    (20U)

static volatile bool g_appUpdateDue;
static volatile uint8_t g_gimbalUpdatePending;

int main(void)
{
    SYSCFG_DL_init();

    App_Init();

    SysTick_Config(CPUCLK_FREQ / APP_TICK_HZ);
    NVIC_EnableIRQ(GPIO_ENCODERS_INT_IRQN);
    __enable_irq();

    while (1) {
        bool gimbalUpdateDue;

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
                Gimbal_YawUpdate5ms();
                Gimbal_PitchUpdate5ms();
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

    Motor_PwmTick100us();
    Gimbal_YawTick100us();
    Gimbal_PitchTick100us();
    Servo_Tick100us();
    Ultrasonic_Tick100us();

    tick100usCount++;
    if (tick100usCount >= APP_TICKS_PER_MS) {
        tick100usCount = 0;

        controlMsCount++;
        gimbalMsCount++;
        if (gimbalMsCount >= GIMBAL_UPDATE_PERIOD_MS) {
            gimbalMsCount = 0;
            if (g_gimbalUpdatePending < GIMBAL_UPDATE_PENDING_MAX) {
                g_gimbalUpdatePending++;
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
