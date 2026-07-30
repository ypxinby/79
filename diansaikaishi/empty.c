/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "app_features.h"
#include "balance_encoder.h"
#include "balance_soft_limits.h"
#include "bluetooth_uart.h"
#include "encoder.h"
#include "emergency_stop.h"
#include "gimbal.h"
#include "gimbal_stepper.h"
#include "gimbal_tracker.h"
#include "gimbal_vision_adapter.h"
#include "gimbal_vision_pitch_tracker.h"
#include "gimbal_vision_yaw_tracker.h"
#include "motor.h"
#include "servo.h"
#include "scheduler_monitor.h"
#include "ti_msp_dl_config.h"
#include "ultrasonic.h"
#include "vision_receiver.h"
#include "vision_pitch_tuning.h"
#include "vision_yaw_tuning.h"
#include "vision_tuning_console.h"
#include "vision_uart.h"
#include "watchdog_monitor.h"

#define APP_TICK_HZ             (10000U)
#define APP_TICKS_PER_MS        (APP_TICK_HZ / 1000U)
#define GIMBAL_UPDATE_PERIOD_MS (5U)
#define GIMBAL_UPDATE_PENDING_MAX (2U)
#define TRACKER_UPDATE_PERIOD_MS (10U)
#define TRACKER_UPDATE_PENDING_MAX (2U)
#define APP_UPDATE_PERIOD_MS    (20U)
#define APP_UPDATE_PENDING_MAX  (8U)
#define APP_CONTROL_DT_MAX_MS   (100U)
#define VISION_RX_PROCESS_BUDGET (64U)

static volatile uint8_t g_appUpdatePending;
#if FEATURE_GIMBAL_MOTION_CONTROL
static volatile uint8_t g_gimbalUpdatePending;
static volatile uint8_t g_trackerUpdatePending;
#endif
static volatile uint32_t g_localTimeMs;
static volatile SchedulerStats g_schedulerStats;

uint32_t SystemTime_GetMs(void)
{
    return g_localTimeMs;
}

void SchedulerMonitor_GetStats(SchedulerStats *stats)
{
    if (stats == (SchedulerStats *)0) {
        return;
    }

    __disable_irq();
    *stats = g_schedulerStats;
    __enable_irq();
}

int main(void)
{
    SYSCFG_DL_init();

    App_Init();
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    GimbalStepper_Init();
    GimbalStepper_SetStepHalfPeriodTicks(
        BALANCE_STEPPER_TEST_HALF_PERIOD_TICKS);
#endif
#if FEATURE_BALANCE_ENCODER_CAPTURE
    BalanceEncoder_Init();
#endif
#if FEATURE_BALANCE_SOFT_LIMITS
    BalanceSoftLimits_Init();
#endif
#if FEATURE_BLUETOOTH_UART
    BluetoothUart_Init();
#endif
    VisionReceiver_Init();
#if FEATURE_GIMBAL_MOTION_CONTROL
    GimbalVisionAdapter_Init();
#endif
#if FEATURE_VISION_TUNING_CONSOLE
    VisionPitchTuning_Init();
    VisionYawTuning_Init();
#endif
#if FEATURE_GIMBAL_MOTION_CONTROL
    GimbalVisionPitchTracker_Init();
    GimbalVisionYawTracker_Init();
#endif
#if FEATURE_VISION_TUNING_CONSOLE
    VisionTuningConsole_Init();
#endif
    VisionUart_Init();

    SysTick_Config(CPUCLK_FREQ / APP_TICK_HZ);
    NVIC_EnableIRQ(GPIO_ENCODERS_INT_IRQN);
#if FEATURE_BALANCE_ENCODER_CAPTURE
    NVIC_EnableIRQ(GPIO_BALANCE_ENCODER_INT_IRQN);
#endif
    __enable_irq();

    while (1) {
#if FEATURE_GIMBAL_MOTION_CONTROL
        bool gimbalUpdateDue;
        bool trackerUpdateDue;
#endif
        uint8_t appUpdatePending;
        static uint32_t lastAppUpdateMs;

#if FEATURE_BLUETOOTH_UART
        BluetoothUart_Process();
#endif
        VisionUart_Process();
        (void)VisionReceiver_Process(g_localTimeMs,
            VISION_RX_PROCESS_BUDGET);
#if FEATURE_GIMBAL_MOTION_CONTROL
        GimbalVisionAdapter_Update();

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
                uint32_t startMs = g_localTimeMs;

                GimbalVisionYawTracker_Update10ms(g_localTimeMs);
                GimbalVisionPitchTracker_Update10ms(g_localTimeMs);
                GimbalTracker_Update(0.010f);
                if ((g_localTimeMs - startMs) >
                    TRACKER_UPDATE_PERIOD_MS) {
                    g_schedulerStats.tracker_10ms_overrun_count++;
                }
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
                uint32_t startMs = g_localTimeMs;

                Gimbal_Update5ms();
                if ((g_localTimeMs - startMs) >
                    GIMBAL_UPDATE_PERIOD_MS) {
                    g_schedulerStats.gimbal_5ms_overrun_count++;
                }
            }
        } while (gimbalUpdateDue);
#endif

        __disable_irq();
        appUpdatePending = g_appUpdatePending;
        g_appUpdatePending = 0U;
        if (appUpdatePending > 1U) {
            g_schedulerStats.app_20ms_drop_count +=
                (uint32_t)(appUpdatePending - 1U);
        }
        __enable_irq();

        if (appUpdatePending != 0U) {
            uint32_t startMs = g_localTimeMs;
            uint32_t elapsedMs = (lastAppUpdateMs == 0U) ?
                APP_UPDATE_PERIOD_MS : startMs - lastAppUpdateMs;

            lastAppUpdateMs = startMs;
            g_schedulerStats.app_last_elapsed_ms = elapsedMs;
            if (elapsedMs > APP_CONTROL_DT_MAX_MS) {
                elapsedMs = APP_CONTROL_DT_MAX_MS;
                g_schedulerStats.app_dt_clamp_count++;
            }

            App_Update_20ms(elapsedMs);
            if ((g_localTimeMs - startMs) > APP_UPDATE_PERIOD_MS) {
                g_schedulerStats.app_20ms_overrun_count++;
            }
            WatchdogMonitor_NotifyControlCycleComplete(g_localTimeMs);
        }
    }
}

void SysTick_Handler(void)
{
    static uint8_t tick100usCount;
    static uint8_t controlMsCount;
#if FEATURE_BALANCE_ENCODER_CAPTURE
    static uint8_t balanceEncoderSpeedMsCount;
#endif
#if FEATURE_GIMBAL_MOTION_CONTROL
    static uint8_t gimbalMsCount;
    static uint8_t trackerMsCount;
#endif

#if !FEATURE_HW_MOTOR_PWM
    Motor_PwmTick100us();
#endif
#if FEATURE_GIMBAL_MOTION_CONTROL
    Gimbal_Tick100us();
#endif
#if FEATURE_BALANCE_STEPPER_OPEN_LOOP_TEST
    if (EmergencyStop_IsActive() || WatchdogMonitor_HasTripped()) {
        GimbalStepper_StopHold();
    } else {
#if FEATURE_BALANCE_SOFT_LIMITS
        BalanceSoftLimits_Enforce100usFromIsr();
#endif
        GimbalStepper_Tick100us();
    }
#endif
    Servo_Tick100us();
    Ultrasonic_Tick100us();

    tick100usCount++;
    if (tick100usCount >= APP_TICKS_PER_MS) {
        tick100usCount = 0;

        g_localTimeMs++;
        WatchdogMonitor_Tick1msFromIsr(g_localTimeMs);

        controlMsCount++;
#if FEATURE_BALANCE_ENCODER_CAPTURE
        balanceEncoderSpeedMsCount++;
        if (balanceEncoderSpeedMsCount >=
            BALANCE_ENCODER_SPEED_SAMPLE_MS) {
            balanceEncoderSpeedMsCount = 0U;
            BalanceEncoder_SampleSpeed10msFromIsr();
        }
#endif
#if FEATURE_GIMBAL_MOTION_CONTROL
        gimbalMsCount++;
        trackerMsCount++;
        if (gimbalMsCount >= GIMBAL_UPDATE_PERIOD_MS) {
            gimbalMsCount = 0;
            if (g_gimbalUpdatePending != 0U) {
                g_schedulerStats.gimbal_5ms_missed_count++;
            }
            if (g_gimbalUpdatePending < GIMBAL_UPDATE_PENDING_MAX) {
                g_gimbalUpdatePending++;
            } else {
                g_schedulerStats.gimbal_5ms_drop_count++;
            }
        }

        if (trackerMsCount >= TRACKER_UPDATE_PERIOD_MS) {
            trackerMsCount = 0;
            if (g_trackerUpdatePending != 0U) {
                g_schedulerStats.tracker_10ms_missed_count++;
            }
            if (g_trackerUpdatePending < TRACKER_UPDATE_PENDING_MAX) {
                g_trackerUpdatePending++;
            } else {
                g_schedulerStats.tracker_10ms_drop_count++;
            }
        }
#endif

        if (controlMsCount >= APP_UPDATE_PERIOD_MS) {
            controlMsCount = 0;
            if (g_appUpdatePending != 0U) {
                g_schedulerStats.app_20ms_missed_count++;
            }
            if (g_appUpdatePending < APP_UPDATE_PENDING_MAX) {
                g_appUpdatePending++;
            } else {
                g_schedulerStats.app_20ms_drop_count++;
            }
        }
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_ENCODERS_INT_IIDX:
            Encoder_HandleGpioInterrupt();
            break;
#if FEATURE_BALANCE_ENCODER_CAPTURE
        case GPIO_BALANCE_ENCODER_INT_IIDX:
            BalanceEncoder_HandleGpioInterrupt();
            break;
#endif
        default:
            break;
    }
}
