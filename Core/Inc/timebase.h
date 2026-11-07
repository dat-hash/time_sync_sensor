#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

#define TIMEBASE_NOMINAL_TICKS_PER_SECOND 200000000u // APB1 timer clock
#define TIMEBASE_NANOSECONDS_PER_TICK 5u // the timer use 200MHZ -> 5ns per tick

// Start TIM2 counting with its overflow interrupt. False if the HAL refused.
bool timebase_start(void);

// Call from HAL_TIM_wrap_callback when TIM2 wraps.
void timebase_on_timer_overflow(void);

// read now tick
uint64_t timebase_read_now_ticks(void);

uint64_t timebase_convert_captured_value(uint32_t captured_counter_value);

#endif