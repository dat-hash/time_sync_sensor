#include "pps_capture.h"
#include "timebase.h"
#include "tim.h" 
#include <stdint.h>

#define PULSE_QUEUE_LENGTH 8u
#define MAXIMUM_INTERVAL_DEVIATION_TICKS 3000000

static volatile uint64_t pulse_queue[PULSE_QUEUE_LENGTH];
static volatile uint32_t pulse_queue_write_index = 0u;
static volatile uint32_t pulse_queue_read_index = 0u;

static volatile uint32_t pulse_count = 0u;
static volatile uint32_t lost_pulse_count = 0u;

// Main state
static bool previous_pulse_is_available = false;
static uint64_t previous_pulse_tick = 0u;
static uint32_t good_interval_count = 0u;
static uint32_t bad_interval_count = 0u;
static bool last_interval_is_valid = false;
static uint32_t last_interval_ticks = 0u;
static int32_t last_clock_error= 0;
static int64_t total_clock_error = 0;
static uint64_t last_pulse_tick = 0u;
static uint32_t last_raw_interval_ticks = 0u;


// init
bool pps_capture_start(void){
        if ((HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3)) != HAL_OK) {
        return false;
    }
    else {
        return true;
    }
}

// interupt
void pps_capture_get_edge(uint32_t captured_counter_value)
{
    uint64_t pulse_tick = timebase_convert_captured_value(captured_counter_value);
    pulse_count = pulse_count + 1u;

    uint32_t pulses_waiting = pulse_queue_write_index - pulse_queue_read_index;

    if (pulses_waiting >= PULSE_QUEUE_LENGTH) {
        lost_pulse_count = lost_pulse_count + 1u; // the queue is full just drop it
    }
    else {
        // insert the stamp into queue
        uint32_t slot = pulse_queue_write_index % PULSE_QUEUE_LENGTH;
        pulse_queue[slot] = pulse_tick;

        // make sure the timestamp is really stored before the index advanced (arm cortex M7 stuff whyyyyyyy)
        // the chip will try to optimize so this prevent it
        __DMB();
        pulse_queue_write_index = pulse_queue_write_index + 1u;
    }
}

// main loop
static bool take_pulse_from_queue(uint64_t *pulse_tick)
{
    if (pulse_queue_read_index == pulse_queue_write_index){
        return false; // there nothing to be read
    }
    else {
        uint32_t slot = pulse_queue_read_index % PULSE_QUEUE_LENGTH;
        *pulse_tick = pulse_queue[slot];
        __DMB(); // again ensure finish reading
        pulse_queue_read_index = pulse_queue_read_index + 1u;
        return true;
    }
}

static void measure_interval(uint64_t pulse_tick)
{
    if (previous_pulse_is_available) {
        uint64_t interval_ticks = pulse_tick - previous_pulse_tick;
        last_raw_interval_ticks = (uint32_t)interval_ticks;
        int64_t deviation_ticks = (int64_t)interval_ticks - (int64_t)TIMEBASE_NOMINAL_TICKS_PER_SECOND;

        // inbound check
        if (deviation_ticks <= MAXIMUM_INTERVAL_DEVIATION_TICKS && deviation_ticks >= -MAXIMUM_INTERVAL_DEVIATION_TICKS) {

            last_clock_error = deviation_ticks; // for debug and clock performance measurement
            total_clock_error = total_clock_error + deviation_ticks;
            last_interval_ticks = (uint32_t)interval_ticks;
            last_interval_is_valid = true;
            good_interval_count = good_interval_count + 1u;
        }
        else {
            bad_interval_count = bad_interval_count + 1u;
        }
    }
    previous_pulse_tick = pulse_tick;
    previous_pulse_is_available = true;
    last_pulse_tick = pulse_tick;
}

void pps_capture_process(void)
{
    uint64_t pulse_tick = 0u;
    while (take_pulse_from_queue(&pulse_tick)) {
        measure_interval(pulse_tick);
    }
}

// debug
void pps_capture_status(pps_status_t *status)
{
    status->pulse_count = pulse_count;
    status->good_interval_count = good_interval_count;
    status->bad_interval_count = bad_interval_count;
    status->lost_pulse_count = lost_pulse_count;
    status->last_interval_is_valid = last_interval_is_valid;
    status->last_interval_ticks = last_interval_ticks;
    status->last_clock_error = last_clock_error;
    status->last_pulse_tick = last_pulse_tick;
    status->last_raw_interval_ticks = last_raw_interval_ticks;
    status->total_clock_error = total_clock_error;
}













