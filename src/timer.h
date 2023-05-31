// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      timer.h
*@brief     Timer LL Driver based on STM32 HAL library
*@author    Ziga Miklosic
*@email     ziga.miklosic@gmail.si
*@date      25.04.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup TIMER_API
* @{ <!-- BEGIN GROUP -->
*
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __TIMER_H
#define __TIMER_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../timer_cfg.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Module version
 */
#define TIMER_VER_MAJOR          ( 0 )
#define TIMER_VER_MINOR          ( 1 )
#define TIMER_VER_DEVELOP        ( 0 )

/**
 *  Timer status
 */
typedef enum
{
    eTIMER_OK        = 0x00U,    /**<Normal operation */
    eTIMER_ERROR     = 0x01U,    /**<General error code */
} timer_status_t;

/**
 *  Float32 definition
 */
typedef float float32_t;

/**
 *  Timer callbacks
 */
typedef void(*pf_timer_cb)(void);


////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_init           (const timer_inst_t tim_inst);
timer_status_t timer_deinit         (const timer_inst_t tim_inst);
timer_status_t timer_is_init        (const timer_inst_t tim_inst, bool * const p_is_init);
timer_status_t timer_start          (const timer_inst_t tim_inst);
timer_status_t timer_stop           (const timer_inst_t tim_inst);
timer_status_t timer_cnt_set        (const timer_inst_t tim_inst, const uint32_t counter);
timer_status_t timer_cnt_reset      (const timer_inst_t tim_inst);
timer_status_t timer_cnt_get        (const timer_inst_t tim_inst, uint32_t * const p_counter);


// TODO: PWM Generation
timer_status_t timer_set_pwm       (const timer_ch_t tim_ch, const float32_t duty);
timer_status_t timer_read_pwm      (const timer_ch_t tim_ch, float32_t * const p_duty);

// TODO: Timer expire + callback: StopWatch
timer_status_t timer_start_stopwatch(const timer_inst_t tim_inst, const float32_t expire_time, pf_timer_cb cb);


// TODO: CLean up timer 1 settings
typedef struct
{
    float32_t freq;         /**<Switching frequency in Hz */
    float32_t toff_min;     /**<Minimum OFF pulse time in us */
    float32_t deadtime;     /**<Dead time in us */
    float32_t sample_delay; /**<ADC trigged delay from low PWM center cycle */
} timer_1_cfg_t;


timer_status_t timer_1_init     (const timer_1_cfg_t * const p_cfg);
timer_status_t timer_1_set_pwm  (const float32_t duty_u, const float32_t duty_v, const float32_t duty_w);
timer_status_t timer_1_pwm_en   (const bool en);


// TODO: CLean up timer 4 settings
timer_status_t timer_4_init     (void);
timer_status_t timer_4_cnt_get  (uint32_t * const p_counter);


#endif // __TIMER_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
