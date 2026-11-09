#include <stdint.h>
#include <stdio.h>
#include "app_main.h"
#include "main.h"
#include "timebase.h"
#include "tim.h"
#include "gnss_uart.h"
#include "usart.h"
#include "debug_print.h"
#include "pps_capture.h"
#include "gnss_ubx.h"

// debug timebase
volatile uint64_t debug_now_ticks = 0u;

// debug gnss
volatile uint32_t debug_gnss_total_bytes = 0u;
volatile uint32_t debug_gnss_errors = 0u;

// debug
static uint32_t last_report_time_ms = 0u;

void app_init(void)
{
    debug_print_init();
    printf("\r\n---time sync board start ---\r\n");

    if (!timebase_start()) {
        Error_Handler();
    }

    if (!gnss_uart_start()) {
        Error_Handler();
    }

    if (gnss_ubx_configure_receiver()){
        printf("gnss: configured at %lu baud (10 Hz, NMEA RMC + GGA)\r\n", (unsigned long)gnss_ubx_get_baud_rate_found());
    }
    else {
        printf("gnss: NOT configured (receiver found at %lu baud, 0 = no answer)\r\n", (unsigned long)gnss_ubx_get_baud_rate_found());
    }

    if(!pps_capture_start()){
        Error_Handler();
    }
}

// to print debug per sec
static void print_status(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_report_time_ms) >= 1000u) {
        last_report_time_ms = now_ms;

        // gnss report
        uint64_t now_ticks = timebase_read_now_ticks();
        uint32_t seconds = (uint32_t)(now_ticks / TIMEBASE_NOMINAL_TICKS_PER_SECOND);
        uint32_t remain_ticks = (uint32_t)(now_ticks % TIMEBASE_NOMINAL_TICKS_PER_SECOND);
        uint32_t nanoseconds = remain_ticks * TIMEBASE_NANOSECONDS_PER_TICK;

        printf("[STATUS] tim2 %lu.%09lu s | gnss bytes %lu errors %lu\r\n", (unsigned long)seconds, (unsigned long)nanoseconds,
                (unsigned long)gnss_uart_get_total_bytes_received(), (unsigned long)gnss_uart_get_error_count());
        
        // pps report
        pps_status_t pps_status;
        pps_capture_status(&pps_status);

        if (pps_status.last_interval_is_valid) {
            int32_t clock_error = pps_status.last_clock_error;
            int64_t total_clock_error = pps_status.total_clock_error;

            char clock_error_sign = (clock_error < 0) ? '-':'+';
            char total_clock_error_sign = (total_clock_error < 0) ? '-':'+';

            uint32_t error_magnitude = (clock_error < 0) ? (uint32_t)(-clock_error) : (uint32_t)(clock_error);
            uint32_t total_error_magnitude = (total_clock_error < 0) ? (uint32_t)(-total_clock_error) : (uint32_t)(total_clock_error);

            printf("[pps] interval %lu ticks | clock error %c%lu ticks | Total clock drift %c%lu ticks | good %lu bad %lu lost %lu\r\n",
                    (unsigned long)pps_status.last_interval_ticks, clock_error_sign, (unsigned long)error_magnitude, 
                    total_clock_error_sign, (unsigned long)total_error_magnitude,(unsigned long)pps_status.good_interval_count, 
                    (unsigned long)pps_status.bad_interval_count, (unsigned long)pps_status.lost_pulse_count);
    
        }
        else {
            printf("[pps] no valid interval yet | seen %lu good %lu bad %lu lost %lu | last interval %lu ticks\r\n",
                    (unsigned long)pps_status.pulse_count, (unsigned long)pps_status.good_interval_count,
                    (unsigned long)pps_status.bad_interval_count, (unsigned long)pps_status.lost_pulse_count,
                    (unsigned long)pps_status.last_raw_interval_ticks);
        }
    }
}

void app_loop(void)
{
    // timebase
    debug_now_ticks = timebase_read_now_ticks();

    // gnss
    static uint8_t gnss_new_bytes[256];
    uint32_t new_byte_count = gnss_uart_read_new_bytes(gnss_new_bytes, sizeof(gnss_new_bytes));
    if (new_byte_count > 0u) {
        // fwrite(gnss_new_bytes, 1, new_byte_count, stdout);
    }
    debug_gnss_total_bytes = gnss_uart_get_total_bytes_received();
    debug_gnss_errors = gnss_uart_get_error_count();
    
    // pps
    pps_capture_process();

    // status debug per second
    print_status();
}

// Callbacks
// tim2 overflow
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer_handle)
{
    if (timer_handle->Instance == TIM2) {
        timebase_on_timer_overflow();
    }
}

// gnss uart
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart_handle, uint16_t size)
{
    if (uart_handle->Instance == USART2) {
        gnss_uart_on_receive_event(size);
    }
}

// error uart gnss
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle->Instance == USART2) {
        gnss_uart_on_error();
    }
}

// tim2 channel interupt
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *timer_handle)
{
    if (timer_handle->Instance == TIM2) {
        if (timer_handle->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            uint32_t captured_value = HAL_TIM_ReadCapturedValue(timer_handle, TIM_CHANNEL_3);
            pps_capture_get_edge(captured_value);
        }
        else {
            // channel 4 = IMU data-ready, later
        }
    }
}