#include "bluetooth_uart.h"

#include "app_features.h"
#include <string.h>

#if FEATURE_BLUETOOTH_UART

#include "ti_msp_dl_config.h"

#define BLUETOOTH_UART_RX_RING_SIZE (128U)
#define BLUETOOTH_UART_RX_RING_MASK (BLUETOOTH_UART_RX_RING_SIZE - 1U)
#define BLUETOOTH_UART_TX_RING_SIZE (256U)
#define BLUETOOTH_UART_TX_RING_MASK (BLUETOOTH_UART_TX_RING_SIZE - 1U)

#if ((BLUETOOTH_UART_RX_RING_SIZE & BLUETOOTH_UART_RX_RING_MASK) != 0U)
#error BLUETOOTH_UART_RX_RING_SIZE must be a power of two
#endif
#if ((BLUETOOTH_UART_TX_RING_SIZE & BLUETOOTH_UART_TX_RING_MASK) != 0U)
#error BLUETOOTH_UART_TX_RING_SIZE must be a power of two
#endif

#ifndef UART_BLUETOOTH_INST
#error FEATURE_BLUETOOTH_UART requires UART_BLUETOOTH in empty.syscfg
#endif

static volatile uint8_t g_rxRing[BLUETOOTH_UART_RX_RING_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static volatile uint8_t g_txRing[BLUETOOTH_UART_TX_RING_SIZE];
static volatile uint16_t g_txHead;
static volatile uint16_t g_txTail;
static BluetoothUartRuntime g_runtime;

static bool bluetooth_uart_pinmux_is_valid(void)
{
    uint32_t txConfig =
        IOMUX->SECCFG.PINCM[GPIO_UART_BLUETOOTH_IOMUX_TX];
    uint32_t rxConfig =
        IOMUX->SECCFG.PINCM[GPIO_UART_BLUETOOTH_IOMUX_RX];

    return (((txConfig & IOMUX_PINCM_PF_MASK) ==
                GPIO_UART_BLUETOOTH_IOMUX_TX_FUNC) &&
        ((txConfig & IOMUX_PINCM_PC_CONNECTED) != 0U) &&
        ((rxConfig & IOMUX_PINCM_PF_MASK) ==
            GPIO_UART_BLUETOOTH_IOMUX_RX_FUNC) &&
        ((rxConfig & IOMUX_PINCM_PC_CONNECTED) != 0U) &&
        ((rxConfig & IOMUX_PINCM_INENA_ENABLE) != 0U));
}

static void bluetooth_uart_increment(volatile uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void bluetooth_uart_store_byte(uint8_t byte, bool polled)
{
    uint16_t head = g_rxHead;
    uint16_t next = (uint16_t)((head + 1U) &
        BLUETOOTH_UART_RX_RING_MASK);

    bluetooth_uart_increment(&g_runtime.rx_byte_count);
    if (polled) {
        bluetooth_uart_increment(&g_runtime.polled_rx_byte_count);
    }
    g_runtime.last_rx_byte = byte;

    if (next == g_rxTail) {
        bluetooth_uart_increment(&g_runtime.rx_overflow_count);
    } else {
        g_rxRing[head] = byte;
        g_rxHead = next;
    }

#if FEATURE_BLUETOOTH_RX_ECHO
    (void)BluetoothUart_TryWriteByte(byte);
#endif
}

static void bluetooth_uart_drain_rx(bool polled)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
        bluetooth_uart_store_byte(
            DL_UART_Main_receiveData(UART_BLUETOOTH_INST), polled);
    }
}

static void bluetooth_uart_drain_tx(void)
{
    while ((g_txTail != g_txHead) &&
        !DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
        DL_UART_Main_transmitData(UART_BLUETOOTH_INST,
            g_txRing[g_txTail]);
        g_txTail = (uint16_t)((g_txTail + 1U) &
            BLUETOOTH_UART_TX_RING_MASK);
        bluetooth_uart_increment(&g_runtime.tx_byte_count);
    }
}

static uint16_t bluetooth_uart_tx_free(void)
{
    uint16_t head = g_txHead;
    uint16_t tail = g_txTail;

    if (head >= tail) {
        return (uint16_t)(BLUETOOTH_UART_TX_RING_SIZE -
            (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

void BluetoothUart_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    g_runtime.initialized = true;
    g_runtime.pinmux_valid = bluetooth_uart_pinmux_is_valid();
    g_runtime.rx_pin_high = false;
    g_runtime.irq_count = 0U;
    g_runtime.polled_rx_byte_count = 0U;
    g_runtime.rx_byte_count = 0U;
    g_runtime.rx_overflow_count = 0U;
    g_runtime.last_rx_byte = 0U;
    g_runtime.tx_byte_count = 0U;
    g_runtime.tx_drop_count = 0U;

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

void BluetoothUart_Process(void)
{
    uint32_t interruptState;

    if (!g_runtime.initialized) {
        return;
    }

    g_runtime.pinmux_valid = bluetooth_uart_pinmux_is_valid();
    g_runtime.rx_pin_high =
        (DL_GPIO_readPins(GPIO_UART_BLUETOOTH_RX_PORT,
            GPIO_UART_BLUETOOTH_RX_PIN) != 0U);

    interruptState = __get_PRIMASK();
    __disable_irq();
    bluetooth_uart_drain_tx();
    if (interruptState == 0U) {
        __enable_irq();
    }

    if (DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    bluetooth_uart_drain_rx(true);
    if (interruptState == 0U) {
        __enable_irq();
    }
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
    return BluetoothUart_TryWriteBuffer(&byte, 1U);
}

bool BluetoothUart_TryWriteBuffer(const uint8_t *data, uint16_t length)
{
    uint32_t interruptState;
    uint16_t i;

    if (!g_runtime.initialized ||
        ((data == (const uint8_t *)0) && (length != 0U))) {
        return false;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    if (length > bluetooth_uart_tx_free()) {
        for (i = 0U; i < length; i++) {
            bluetooth_uart_increment(&g_runtime.tx_drop_count);
        }
        if (interruptState == 0U) {
            __enable_irq();
        }
        return false;
    }

    for (i = 0U; i < length; i++) {
        g_txRing[g_txHead] = data[i];
        g_txHead = (uint16_t)((g_txHead + 1U) &
            BLUETOOTH_UART_TX_RING_MASK);
    }
    bluetooth_uart_drain_tx();
    if (interruptState == 0U) {
        __enable_irq();
    }
    return true;
}

bool BluetoothUart_TryWriteString(const char *text)
{
    size_t length;

    if (text == (const char *)0) {
        return false;
    }
    length = strlen(text);
    if (length > UINT16_MAX) {
        return false;
    }
    return BluetoothUart_TryWriteBuffer((const uint8_t *)text,
        (uint16_t)length);
}

const volatile BluetoothUartRuntime *BluetoothUart_GetRuntime(void)
{
    return &g_runtime;
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    bluetooth_uart_increment(&g_runtime.irq_count);

    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            bluetooth_uart_drain_rx(false);
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

void BluetoothUart_Process(void)
{
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

bool BluetoothUart_TryWriteBuffer(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return false;
}

bool BluetoothUart_TryWriteString(const char *text)
{
    (void)text;
    return false;
}

const volatile BluetoothUartRuntime *BluetoothUart_GetRuntime(void)
{
    return &g_runtime;
}

#endif
