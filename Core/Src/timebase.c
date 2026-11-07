#include "timebase.h"
#include "tim.h"
#include <stdint.h>

static volatile uint32_t timer_overflow_count = 0u;

// init
bool timebase_start(void) 
{
    timer_overflow_count = 0u;

    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    // enable interupt
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        return false;
    }
    else {
        return true;
    }
}


void timebase_on_timer_overflow(void)
{
    timer_overflow_count = timer_overflow_count + 1u;
}

uint64_t timebase_read_now_ticks(void)
{
    uint32_t saved_interrupt_mask = __get_PRIMASK(); //  get primask when it in enable state (usually 0)
    __disable_irq(); // disable interupt

    uint32_t counter_value = TIM2->CNT;
    uint32_t overflow_count_now = timer_overflow_count;

    if ((TIM2->SR & TIM_SR_UIF) != 0u) // get update interrupt flag (set to 1 when wrap happen)
    {
        counter_value = TIM2->CNT; // re-read: definitely after the wrap
        overflow_count_now = overflow_count_now + 1u; // count up the missing wrap
    }

    __set_PRIMASK(saved_interrupt_mask); // renable the interupt

    // convert to full tick then send back
    const uint64_t ticks_per_wrap = 4294967296ull;
    uint64_t full_tick = (uint64_t)overflow_count_now * ticks_per_wrap + (uint64_t)counter_value;
    return full_tick;
}

// for interupt
uint64_t timebase_convert_captured_value(uint32_t captured_counter_value){
    uint64_t time_now = timebase_read_now_ticks();
    uint32_t low_32_ticks = (uint32_t)time_now;
    uint32_t time_pass = low_32_ticks - captured_counter_value;
    uint64_t time_captured = time_now - (uint64_t)time_pass;
    return time_captured;
}








