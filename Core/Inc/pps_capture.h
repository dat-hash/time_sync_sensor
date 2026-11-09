#ifndef PPS_CAPTURE_H
#define PPS_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t pulse_count; // total pulse since start
    uint32_t good_interval_count; // pulse interval that about 1s
    uint32_t bad_interval_count; // noise, weird stuff
    uint32_t lost_pulse_count; // main too slow, miss
    bool last_interval_is_valid;
    uint32_t last_interval_ticks;
    int32_t last_clock_error; // how off is our internal clock
    uint64_t last_pulse_tick; // latest 64bit stamp
    uint32_t last_raw_interval_ticks;
    int64_t total_clock_error;
} pps_status_t;

bool pps_capture_start(void);

//for call back
void pps_capture_get_edge(uint32_t captured_counter_value);

// for main loop
void pps_capture_process(void);

// debug
void pps_capture_status(pps_status_t *status);

#endif
