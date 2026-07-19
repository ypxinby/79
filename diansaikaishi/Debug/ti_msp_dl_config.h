/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for UART_VISION */
#define UART_VISION_INST                                                   UART3
#define UART_VISION_INST_FREQUENCY                                      32000000
#define UART_VISION_INST_IRQHandler                             UART3_IRQHandler
#define UART_VISION_INST_INT_IRQN                                 UART3_INT_IRQn
#define GPIO_UART_VISION_RX_PORT                                           GPIOB
#define GPIO_UART_VISION_TX_PORT                                           GPIOB
#define GPIO_UART_VISION_RX_PIN                                    DL_GPIO_PIN_3
#define GPIO_UART_VISION_TX_PIN                                    DL_GPIO_PIN_2
#define GPIO_UART_VISION_IOMUX_RX                                (IOMUX_PINCM16)
#define GPIO_UART_VISION_IOMUX_TX                                (IOMUX_PINCM15)
#define GPIO_UART_VISION_IOMUX_RX_FUNC                 IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_VISION_IOMUX_TX_FUNC                 IOMUX_PINCM15_PF_UART3_TX
#define UART_VISION_BAUD_RATE                                           (115200)
#define UART_VISION_IBRD_32_MHZ_115200_BAUD                                 (17)
#define UART_VISION_FBRD_32_MHZ_115200_BAUD                                 (23)





/* Port definition for Pin Group GPIO_TB6612_A */
#define GPIO_TB6612_A_PORT                                               (GPIOA)

/* Defines for AIN2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_TB6612_A_AIN2_PIN                                  (DL_GPIO_PIN_12)
#define GPIO_TB6612_A_AIN2_IOMUX                                 (IOMUX_PINCM34)
/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_TB6612_A_AIN1_PIN                                  (DL_GPIO_PIN_13)
#define GPIO_TB6612_A_AIN1_IOMUX                                 (IOMUX_PINCM35)
/* Port definition for Pin Group GPIO_TB6612_B */
#define GPIO_TB6612_B_PORT                                               (GPIOB)

/* Defines for PWMA: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_TB6612_B_PWMA_PIN                                  (DL_GPIO_PIN_15)
#define GPIO_TB6612_B_PWMA_IOMUX                                 (IOMUX_PINCM32)
/* Defines for BIN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_TB6612_B_BIN1_PIN                                   (DL_GPIO_PIN_0)
#define GPIO_TB6612_B_BIN1_IOMUX                                 (IOMUX_PINCM12)
/* Defines for BIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_TB6612_B_BIN2_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_TB6612_B_BIN2_IOMUX                                 (IOMUX_PINCM13)
/* Defines for PWMB: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_TB6612_B_PWMB_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_TB6612_B_PWMB_IOMUX                                 (IOMUX_PINCM33)
/* Port definition for Pin Group GPIO_ENCODERS */
#define GPIO_ENCODERS_PORT                                               (GPIOA)

/* Defines for MOTOR_A_ENCA: GPIOA.15 with pinCMx 37 on package pin 8 */
// pins affected by this interrupt request:["MOTOR_A_ENCA","MOTOR_B_ENCA"]
#define GPIO_ENCODERS_INT_IRQN                                  (GPIOA_INT_IRQn)
#define GPIO_ENCODERS_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODERS_MOTOR_A_ENCA_IIDX                     (DL_GPIO_IIDX_DIO15)
#define GPIO_ENCODERS_MOTOR_A_ENCA_PIN                          (DL_GPIO_PIN_15)
#define GPIO_ENCODERS_MOTOR_A_ENCA_IOMUX                         (IOMUX_PINCM37)
/* Defines for MOTOR_A_ENCB: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_ENCODERS_MOTOR_A_ENCB_PIN                          (DL_GPIO_PIN_16)
#define GPIO_ENCODERS_MOTOR_A_ENCB_IOMUX                         (IOMUX_PINCM38)
/* Defines for MOTOR_B_ENCA: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_ENCODERS_MOTOR_B_ENCA_IIDX                     (DL_GPIO_IIDX_DIO17)
#define GPIO_ENCODERS_MOTOR_B_ENCA_PIN                          (DL_GPIO_PIN_17)
#define GPIO_ENCODERS_MOTOR_B_ENCA_IOMUX                         (IOMUX_PINCM39)
/* Defines for MOTOR_B_ENCB: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_ENCODERS_MOTOR_B_ENCB_PIN                          (DL_GPIO_PIN_24)
#define GPIO_ENCODERS_MOTOR_B_ENCB_IOMUX                         (IOMUX_PINCM54)
/* Port definition for Pin Group GPIO_TRACK_A */
#define GPIO_TRACK_A_PORT                                                (GPIOA)

/* Defines for S4: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_TRACK_A_S4_PIN                                     (DL_GPIO_PIN_14)
#define GPIO_TRACK_A_S4_IOMUX                                    (IOMUX_PINCM36)
/* Defines for S8: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_TRACK_A_S8_PIN                                      (DL_GPIO_PIN_7)
#define GPIO_TRACK_A_S8_IOMUX                                    (IOMUX_PINCM14)
/* Port definition for Pin Group GPIO_TRACK_B */
#define GPIO_TRACK_B_PORT                                                (GPIOB)

/* Defines for S1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_TRACK_B_S1_PIN                                     (DL_GPIO_PIN_25)
#define GPIO_TRACK_B_S1_IOMUX                                    (IOMUX_PINCM56)
/* Defines for S2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_TRACK_B_S2_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_TRACK_B_S2_IOMUX                                    (IOMUX_PINCM52)
/* Defines for S3: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_TRACK_B_S3_PIN                                     (DL_GPIO_PIN_20)
#define GPIO_TRACK_B_S3_IOMUX                                    (IOMUX_PINCM48)
/* Defines for S5: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_TRACK_B_S5_PIN                                     (DL_GPIO_PIN_18)
#define GPIO_TRACK_B_S5_IOMUX                                    (IOMUX_PINCM44)
/* Defines for S6: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_TRACK_B_S6_PIN                                     (DL_GPIO_PIN_19)
#define GPIO_TRACK_B_S6_IOMUX                                    (IOMUX_PINCM45)
/* Defines for S7: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_TRACK_B_S7_PIN                                     (DL_GPIO_PIN_10)
#define GPIO_TRACK_B_S7_IOMUX                                    (IOMUX_PINCM27)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for SDA: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_OLED_SDA_PIN                                       (DL_GPIO_PIN_28)
#define GPIO_OLED_SDA_IOMUX                                       (IOMUX_PINCM3)
/* Defines for SCL: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_OLED_SCL_PIN                                       (DL_GPIO_PIN_31)
#define GPIO_OLED_SCL_IOMUX                                       (IOMUX_PINCM6)
/* Port definition for Pin Group GPIO_KEYS */
#define GPIO_KEYS_PORT                                                   (GPIOA)

/* Defines for K1: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_KEYS_K1_PIN                                        (DL_GPIO_PIN_26)
#define GPIO_KEYS_K1_IOMUX                                       (IOMUX_PINCM59)
/* Defines for K2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_KEYS_K2_PIN                                        (DL_GPIO_PIN_25)
#define GPIO_KEYS_K2_IOMUX                                       (IOMUX_PINCM55)
/* Defines for K3: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_KEYS_K3_PIN                                        (DL_GPIO_PIN_27)
#define GPIO_KEYS_K3_IOMUX                                       (IOMUX_PINCM60)
/* Port definition for Pin Group GPIO_IMU */
#define GPIO_IMU_PORT                                                    (GPIOA)

/* Defines for IMU_SDA: GPIOA.10 with pinCMx 21 on package pin 56 */
#define GPIO_IMU_IMU_SDA_PIN                                    (DL_GPIO_PIN_10)
#define GPIO_IMU_IMU_SDA_IOMUX                                   (IOMUX_PINCM21)
/* Defines for IMU_SCL: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_IMU_IMU_SCL_PIN                                    (DL_GPIO_PIN_11)
#define GPIO_IMU_IMU_SCL_IOMUX                                   (IOMUX_PINCM22)
/* Port definition for Pin Group GPIO_ULTRASONIC */
#define GPIO_ULTRASONIC_PORT                                             (GPIOA)

/* Defines for HC_TRIG: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_ULTRASONIC_HC_TRIG_PIN                              (DL_GPIO_PIN_8)
#define GPIO_ULTRASONIC_HC_TRIG_IOMUX                            (IOMUX_PINCM19)
/* Defines for HC_ECHO: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_ULTRASONIC_HC_ECHO_PIN                              (DL_GPIO_PIN_9)
#define GPIO_ULTRASONIC_HC_ECHO_IOMUX                            (IOMUX_PINCM20)
/* Port definition for Pin Group GPIO_SERVO */
#define GPIO_SERVO_PORT                                                  (GPIOB)

/* Defines for SERVO: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_SERVO_SERVO_PIN                                     (DL_GPIO_PIN_9)
#define GPIO_SERVO_SERVO_IOMUX                                   (IOMUX_PINCM26)
/* Port definition for Pin Group GPIO_GIMBAL_A */
#define GPIO_GIMBAL_A_PORT                                               (GPIOA)

/* Defines for PITCH_DIR: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GPIO_GIMBAL_A_PITCH_DIR_PIN                              (DL_GPIO_PIN_1)
#define GPIO_GIMBAL_A_PITCH_DIR_IOMUX                             (IOMUX_PINCM2)
/* Defines for PITCH_EN: GPIOA.2 with pinCMx 7 on package pin 42 */
#define GPIO_GIMBAL_A_PITCH_EN_PIN                               (DL_GPIO_PIN_2)
#define GPIO_GIMBAL_A_PITCH_EN_IOMUX                              (IOMUX_PINCM7)
/* Port definition for Pin Group GPIO_GIMBAL_B */
#define GPIO_GIMBAL_B_PORT                                               (GPIOB)

/* Defines for PITCH_STEP: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_GIMBAL_B_PITCH_STEP_PIN                             (DL_GPIO_PIN_4)
#define GPIO_GIMBAL_B_PITCH_STEP_IOMUX                           (IOMUX_PINCM17)
/* Port definition for Pin Group GPIO_GIMBAL_PITCH */
#define GPIO_GIMBAL_PITCH_PORT                                           (GPIOB)

/* Defines for STEP: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_GIMBAL_PITCH_STEP_PIN                               (DL_GPIO_PIN_5)
#define GPIO_GIMBAL_PITCH_STEP_IOMUX                             (IOMUX_PINCM18)
/* Defines for DIR: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_GIMBAL_PITCH_DIR_PIN                                (DL_GPIO_PIN_6)
#define GPIO_GIMBAL_PITCH_DIR_IOMUX                              (IOMUX_PINCM23)
/* Defines for EN: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_GIMBAL_PITCH_EN_PIN                                 (DL_GPIO_PIN_7)
#define GPIO_GIMBAL_PITCH_EN_IOMUX                               (IOMUX_PINCM24)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_VISION_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
