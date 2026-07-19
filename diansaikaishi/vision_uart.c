#include "vision_uart.h"

#include "ti_msp_dl_config.h"
#include "vision_receiver.h"

void VisionUart_Init(void)
{
    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
}

void UART_VISION_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VISION_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_VISION_INST)) {
                VisionReceiver_PushByteFromIsr(
                    DL_UART_Main_receiveData(UART_VISION_INST));
            }
            break;
        default:
            break;
    }
}
