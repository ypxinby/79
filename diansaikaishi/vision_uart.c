#include "vision_uart.h"

#include "ti_msp_dl_config.h"
#include "vision_receiver.h"
#include "vision_tuning_console.h"

#define VISION_TUNING_RX_PROCESS_BUDGET (64U)
#define VISION_TUNING_TX_PROCESS_BUDGET (16U)

void VisionUart_Init(void)
{
    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
}

void VisionUart_Process(void)
{
    uint8_t byte;
    uint8_t transmitted = 0U;

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
}

void UART_VISION_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VISION_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_VISION_INST)) {
                uint8_t byte =
                    DL_UART_Main_receiveData(UART_VISION_INST);

                VisionReceiver_PushByteFromIsr(byte);
                VisionTuningConsole_PushByteFromIsr(byte);
            }
            break;
        default:
            break;
    }
}
