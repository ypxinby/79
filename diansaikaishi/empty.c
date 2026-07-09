/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "encoder.h"
#include "motor.h"
#include "straight_control.h"
#include "ti_msp_dl_config.h"

#define APP_TICK_HZ             (10000U)
#define APP_TICKS_PER_MS        (APP_TICK_HZ / 1000U)

static volatile bool g_straightControlDue;

int main(void)
{
    SYSCFG_DL_init();

    App_Init();

    SysTick_Config(CPUCLK_FREQ / APP_TICK_HZ);
    NVIC_EnableIRQ(GPIO_ENCODERS_INT_IRQN);
    __enable_irq();

    while (1) {
        if (g_straightControlDue) {
            g_straightControlDue = false;
            App_Update_20ms();
        }
    }
}

void SysTick_Handler(void)
{
    static uint8_t tick100usCount;
    static uint8_t controlMsCount;

    Motor_PwmTick100us();

    tick100usCount++;
    if (tick100usCount >= APP_TICKS_PER_MS) {
        tick100usCount = 0;

        controlMsCount++;
        if (controlMsCount >= STRAIGHT_CONTROL_PERIOD_MS) {
            controlMsCount = 0;
            g_straightControlDue = true;
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
