#include "vision_uart.h"

#include "app_features.h"
#include "debug_telemetry.h"
#include "ti_msp_dl_config.h"
#include "vision_receiver.h"
#include "vision_tuning_console.h"

#define VISION_TUNING_RX_PROCESS_BUDGET (64U)
#define VISION_TUNING_TX_PROCESS_BUDGET (16U)

void VisionUart_Init(void)
{
    /* Keep the receiver single-producer: only this UART ISR may push bytes
     * into VisionReceiver. FIFO absorbs short higher-priority ISR delays. */
    DL_UART_Main_enableFIFOs(UART_VISION_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_VISION_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_VISION_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
}

void VisionUart_Process(void)
{
#if FEATURE_VISION_TUNING_CONSOLE || \
    FEATURE_DEBUG_TELEMETRY_VISION_UART
    uint8_t byte;
    uint8_t transmitted = 0U;
#endif

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
    for (;;) {
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
            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
                VisionReceiver_RecordUartOverrunFromIsr();
                DL_UART_Main_clearInterruptStatus(UART_VISION_INST,
                    DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
                break;
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
                VisionReceiver_RecordUartFramingErrorFromIsr();
                DL_UART_Main_clearInterruptStatus(UART_VISION_INST,
                    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
                break;
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
                VisionReceiver_RecordUartParityErrorFromIsr();
                DL_UART_Main_clearInterruptStatus(UART_VISION_INST,
                    DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
                break;
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
                VisionReceiver_RecordUartBreakErrorFromIsr();
                DL_UART_Main_clearInterruptStatus(UART_VISION_INST,
                    DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
                break;
            default:
                return;
        }
    }
}
