#ifndef BLUETOOTH_UART_H
#define BLUETOOTH_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    volatile bool pinmux_valid;
    volatile bool rx_pin_high;
    volatile uint32_t irq_count;
    volatile uint32_t polled_rx_byte_count;
    volatile uint32_t rx_byte_count;
    volatile uint32_t rx_overflow_count;
    volatile uint8_t last_rx_byte;
    volatile uint32_t tx_byte_count;
    volatile uint32_t tx_drop_count;
} BluetoothUartRuntime;

void BluetoothUart_Init(void);
void BluetoothUart_Process(void);
bool BluetoothUart_TryReadByte(uint8_t *byte);
bool BluetoothUart_TryWriteByte(uint8_t byte);
const volatile BluetoothUartRuntime *BluetoothUart_GetRuntime(void);

#endif
