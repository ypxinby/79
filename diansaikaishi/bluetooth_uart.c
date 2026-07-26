#include "bluetooth_uart.h"

#include "app_features.h"

#if FEATURE_BLUETOOTH_UART

#include "ti_msp_dl_config.h"

#define BLUETOOTH_UART_RX_RING_SIZE (128U)
#define BLUETOOTH_UART_RX_RING_MASK (BLUETOOTH_UART_RX_RING_SIZE - 1U)

#if ((BLUETOOTH_UART_RX_RING_SIZE & BLUETOOTH_UART_RX_RING_MASK) != 0U)
#error BLUETOOTH_UART_RX_RING_SIZE must be a power of two
#endif

#ifndef UART_BLUETOOTH_INST
#error FEATURE_BLUETOOTH_UART requires UART_BLUETOOTH in empty.syscfg
#endif

static volatile uint8_t g_rxRing[BLUETOOTH_UART_RX_RING_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static BluetoothUartRuntime g_runtime;

void BluetoothUart_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_runtime.initialized = true;
    g_runtime.rx_byte_count = 0U;
    g_runtime.rx_overflow_count = 0U;
    g_runtime.last_rx_byte = 0U;
    g_runtime.tx_byte_count = 0U;

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

bool BluetoothUart_TryReadByte(uint8_t *byte)
{
    uint16_t tail;

    if (byte == (uint8_t *)0) {
        return false;
    }

    tail = g_rxTail;
    if (tail == g_rxHead) {
        return false;
    }

    *byte = g_rxRing[tail];
    g_rxTail = (uint16_t)((tail + 1U) & BLUETOOTH_UART_RX_RING_MASK);
    return true;
}

bool BluetoothUart_TryWriteByte(uint8_t byte)
{
    if (!g_runtime.initialized ||
        DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
        return false;
    }

    DL_UART_Main_transmitData(UART_BLUETOOTH_INST, byte);
    if (g_runtime.tx_byte_count < UINT32_MAX) {
        g_runtime.tx_byte_count++;
    }
    return true;
}

const volatile BluetoothUartRuntime *BluetoothUart_GetRuntime(void)
{
    return &g_runtime;
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
                uint8_t byte =
                    DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
                uint16_t head = g_rxHead;
                uint16_t next = (uint16_t)((head + 1U) &
                    BLUETOOTH_UART_RX_RING_MASK);

                if (g_runtime.rx_byte_count < UINT32_MAX) {
                    g_runtime.rx_byte_count++;
                }
                g_runtime.last_rx_byte = byte;

                if (next == g_rxTail) {
                    if (g_runtime.rx_overflow_count < UINT32_MAX) {
                        g_runtime.rx_overflow_count++;
                    }
                    continue;
                }

                g_rxRing[head] = byte;
                g_rxHead = next;
            }
            break;
        default:
            break;
    }
}

#else

static BluetoothUartRuntime g_runtime;

void BluetoothUart_Init(void)
{
    g_runtime.initialized = false;
}

bool BluetoothUart_TryReadByte(uint8_t *byte)
{
    (void)byte;
    return false;
}

bool BluetoothUart_TryWriteByte(uint8_t byte)
{
    (void)byte;
    return false;
}

const volatile BluetoothUartRuntime *BluetoothUart_GetRuntime(void)
{
    return &g_runtime;
}

#endif
