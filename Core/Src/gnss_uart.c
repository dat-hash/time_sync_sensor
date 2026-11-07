#include "gnss_uart.h"
#include "dma_buffer.h"
#include "usart.h"
#include "timebase.h"

#define GNSS_RECEIVE_BUFFER_LENGTH 2048u

static uint8_t gnss_receive_buffer[GNSS_RECEIVE_BUFFER_LENGTH] DMA_BUFFER;

// time :)
static volatile uint64_t latest_receive_tick = 0u;

// how far into the buffer the DMA has written
static volatile uint16_t dma_write_position = 0u;

// how far we've read.
static uint16_t main_loop_read_position = 0u;

static volatile uint32_t total_bytes_received = 0u;
static volatile uint32_t error_count = 0u;

bool gnss_uart_start(void)
{
    dma_write_position = 0u;
    main_loop_read_position = 0u;

    // HAL calls HAL_UARTEx_RxEventCallback with the DMA position when:
    // - the RX line has been idle (aka sentence end)
    // - the DMA reaches the half-way point of the buffer (idk why they even have this absolute garbage)
    // - the DMA reaches the end (and wraps to the start)
    
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gnss_receive_buffer, GNSS_RECEIVE_BUFFER_LENGTH) != HAL_OK) {
        return false;
    }
    else {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // turn off half time as who care about this
        return true;
    }
}

// interrupt
void gnss_uart_on_receive_event(uint16_t new_dma_write_position)
{
    // so when idle at 0 it actually report the max 2048 which is wrong
    // also to prvent any weird out of bound index
    if (new_dma_write_position >= GNSS_RECEIVE_BUFFER_LENGTH) {
        dma_write_position = 0u;
    }
    else {
        dma_write_position = new_dma_write_position;
    }
}

void gnss_uart_on_error(void)
{
    error_count = error_count + 1u;
    HAL_UART_AbortReceive(&huart2);
    (void)gnss_uart_start();
}


// IN MAIN
// Copy buffer and returns how many were copied 
// (WARNING PASS IN REAL END AKA REAL WRITE POSITION AND NOT WHERE THE READ END)
static uint32_t copy_range(uint8_t *destination, uint32_t maximum_bytes, uint16_t start, uint16_t end)
{
    uint32_t bytes_copied = 0u;

    for (uint16_t position = start; position < end; position++){
        if (bytes_copied >= maximum_bytes) {
            break; // the destination is full :(
        }
        else {
            destination[bytes_copied] = gnss_receive_buffer[position];
            bytes_copied = bytes_copied + 1u;
        }
    }
    return bytes_copied;
}

// get byte out
uint32_t gnss_uart_read_new_bytes(uint8_t *destination, uint32_t maximum_bytes)
{
    uint16_t write_position = dma_write_position;
    uint16_t read_position = main_loop_read_position;
    uint32_t bytes_copied  = 0u;

    /* Three cases (as buffer is a circle):
        1. write > read -> new bytes are read so read until write-1
        2. write < read : the DMA wrapped past the end -> new bytes are read until end-1, then 0 until write-1
        3. write == read: nothing new */

    if (write_position > read_position) {
        bytes_copied = copy_range(destination, maximum_bytes, read_position, write_position);
        read_position = (uint16_t)(read_position + bytes_copied);
    }
    else if (write_position < read_position) {
        // first portion
        uint32_t first_chunk_copied = copy_range(destination, maximum_bytes, read_position, GNSS_RECEIVE_BUFFER_LENGTH);
        bytes_copied = first_chunk_copied;
        read_position = (uint16_t)(read_position + first_chunk_copied);

        // the rest
        if (read_position >= GNSS_RECEIVE_BUFFER_LENGTH) {
            read_position = 0u;
            uint32_t second_chunk_copied = copy_range(&destination[bytes_copied], maximum_bytes - bytes_copied, read_position, write_position);
            bytes_copied = bytes_copied + second_chunk_copied;
            read_position = (uint16_t)second_chunk_copied;
        }
    }
    else {
        // nothing new, yup just move on
    }

    main_loop_read_position = read_position;
    total_bytes_received = total_bytes_received + bytes_copied;
    return bytes_copied;
}

uint64_t gnss_uart_get_latest_receive_tick(void)
{
    uint32_t saved_interrupt_mask = __get_PRIMASK();
    __disable_irq();
    uint64_t tick = latest_receive_tick;
    __set_PRIMASK(saved_interrupt_mask);
    return tick;
}

// baud finder
bool gnss_uart_set_baud_rate(uint32_t baud_rate)
{
    HAL_UART_AbortReceive(&huart2);
    huart2.Init.BaudRate = baud_rate;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return false;
    }
    else {
        return gnss_uart_start();
    }
}

// config gnss
void gnss_uart_send(const uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, length, 100u);
}

// info
uint32_t gnss_uart_get_total_bytes_received(void)
{
    return total_bytes_received;
}

uint32_t gnss_uart_get_error_count(void)
{
    return error_count;
}