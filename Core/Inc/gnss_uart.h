#ifndef GNSS_UART_H
#define GNSS_UART_H

#include <stdbool.h>
#include <stdint.h>

// Start the circular DMA reception. False if the HAL refused.
bool gnss_uart_start(void);

// call from HAL_UARTEx_RxEventCallback
void gnss_uart_on_receive_event(uint16_t dma_write_position);

// HAL_UART_ErrorCallback
void gnss_uart_on_error(void);

// Main loop side: copy out bytes that arrived since the last call.
// Returns how many were copied (0 .. maximum_bytes).
uint32_t gnss_uart_read_new_bytes(uint8_t *destination, uint32_t maximum_bytes);

uint64_t gnss_uart_get_latest_receive_tick(void);

// Change the UART speed
bool gnss_uart_set_baud_rate(uint32_t baud_rate);

// Blocking send to the receiver
void gnss_uart_send(const uint8_t *data, uint16_t length);


// Counters for the debugger
uint32_t gnss_uart_get_total_bytes_received(void);
uint32_t gnss_uart_get_error_count(void);

#endif