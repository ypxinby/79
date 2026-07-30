#include "vision_uart.h"

#include "app_features.h"
#include "debug_telemetry.h"
#include "ti_msp_dl_config.h"
#include "vision_receiver.h"
#include "vision_tuning_console.h"

#define VISION_TUNING_RX_PROCESS_BUDGET (64U)
#define VISION_TUNING_TX_PROCESS_BUDGET (16U)
#define VISION_UART_POLL_RX_BUDGET       (64U)

void VisionUart_Init(void)
{
    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
}

void VisionUart_Process(void)
{
    uint8_t byte;
    uint8_t received = 0U;
#if FEATURE_VISION_TUNING_CONSOLE || \
    FEATURE_DEBUG_TELEMETRY_VISION_UART
    uint8_t transmitted = 0U;
#endif

    /* Normal reception is interrupt-driven. Polling is retained as a safe
     * fallback so a vector/NVIC integration issue cannot make the wired K230
     * link appear completely dead during competition bring-up. Reading the
     * data register removes the byte, so ISR and polling cannot duplicate it. */
    while ((received < VISION_UART_POLL_RX_BUDGET) &&
        !DL_UART_Main_isRXFIFOEmpty(UART_VISION_INST)) {
        byte = DL_UART_Main_receiveData(UART_VISION_INST);
        VisionReceiver_PushByteFromIsr(byte);
#if FEATURE_VISION_TUNING_CONSOLE
        VisionTuningConsole_PushByteFromIsr(byte);
#endif
        received++;
    }

#if FEATURE_VISION_TUNING_CONSOLE
    (void)VisionTuningConsole_Process(
        VISION_TUNING_RX_PROCESS_BUDGET);

    while ((transmitted < VISION_TUNING_TX_PROCESS_BUDGET) &&
        !DL_UART_Main_isTXFIFOFull(UART_VISION_INST)) {
        if (VisionTuningConsole_TryPopTxByte(&byte) == 0U) {
            break;
        }
        DL_UART_Main_transmitData(UART_VISION_INST, byte);
        transmitted++;
    }
#endif

#if FEATURE_DEBUG_TELEMETRY_VISION_UART
    while ((transmitted < VISION_TUNING_TX_PROCESS_BUDGET) &&
        !DL_UART_Main_isTXFIFOFull(UART_VISION_INST)) {
        if (DebugTelemetry_TryPopTxByte(&byte) == 0U) {
            break;
        }
        DL_UART_Main_transmitData(UART_VISION_INST, byte);
        transmitted++;
    }
#endif
}

void UART_VISION_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VISION_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_VISION_INST)) {
                uint8_t byte =
                    DL_UART_Main_receiveData(UART_VISION_INST);

                VisionReceiver_PushByteFromIsr(byte);
#if FEATURE_VISION_TUNING_CONSOLE
                VisionTuningConsole_PushByteFromIsr(byte);
#endif
            }
            break;
        default:
            break;
    }
}
