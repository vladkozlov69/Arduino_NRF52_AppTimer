#ifndef APP_TIMER_HPP__
#define APP_TIMER_HPP__

#include "app_timer_internal.h"

extern "C" {
uint32_t app_timer_start(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context);
uint32_t app_timer_init(uint32_t                      prescaler, 
                        uint8_t                       max_timers,
                        uint8_t                       op_queues_size,
                        void *                        p_buffer,
                        app_timer_evt_schedule_func_t evt_schedule_func);
uint32_t app_timer_create(app_timer_id_t *            p_timer_id,
                          app_timer_mode_t            mode,
                          app_timer_timeout_handler_t timeout_handler);
}

#endif