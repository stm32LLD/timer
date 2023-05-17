# **STM32 Timer Low Level Driver**
Following repository constains STM32 Timer Low Level (LL) driver C implementation based on STM32 HAL library.

STM32 Timer LL driver is supporting following STM32 device family:
- STM32G0:      Has TIM1/2/3/4/6/7/14/15/16/17
- STM32L4/L4+:  Has TIM1/2/3/4/5/6/7/8/15/16/17
- STM32H7:      Has TIM1/2/3/4/5/6/7/8/12/13/14/15/16/17/23/24


## **Dependencies**

### **1. STM32 HAL library**
STM32 Timer LL driver module uses STM32 HAL library.


## **Timer Profiles**

TODO: Describe supported timer profiles...

```C
/**
 *  Timer profile
 */
typedef enum
{
    eTIM_PROFILE_FREERUNNING = 0,   /**<Free-running timer */
    eTIM_PROFILE_STOPWATCH,         /**<Stopwatch timer profile */
    eTIM_PROFILE_PWM_OUT,           /**<PWM generation timer profile */
    eTIM_PROFILE_PWM_IN,            /**<PWM signal decoding/reading timer profile */

    eTIM_PROFILE_NUM_OF,            /**<Number of all timer profiles */
} tim_profile_t;
```


## **API**
| API Functions | Description | Prototype |
| --- | ----------- | ----- |
| **timer_init** | Initialization of Timer module | timer_status_t timer_init(const timer_inst_t tim_inst) |
| **timer_deinit** | De-initialization of Timer module | timer_status_t timer_deinit(const timer_inst_t tim_inst) |
| **timer_is_init** | Get initialization state of Timer module| timer_status_t timer_is_init(const timer_inst_t tim_inst, bool * const p_is_init) |
| **timer_start** | Start Timer instance| timer_status_t timer_start(const timer_inst_t tim_inst) |
| **timer_start** | Stop Timer instance| timer_status_t timer_stop(const timer_inst_t tim_inst) |
| **timer_cnt_set** | Set Timer instance counter| timer_status_t timer_cnt_set(const timer_inst_t tim_inst, const uint32_t counter) |
| **timer_cnt_reset** | Reset (set to 0) Timer instance counter| timer_status_t timer_cnt_reset(const timer_inst_t tim_inst) |
| **timer_cnt_get** | Get Timer instance counter| timer_status_t timer_cnt_get(const timer_inst_t tim_inst, uint32_t * const p_counter) |

TODO: Add more here after all timer profiles are being created...


## **Usage**

**GENERAL NOTICE: Put all user code between sections: USER CODE BEGIN & USER CODE END!**

**1. Copy template files to root directory of the module**

Copy configuration file *timer_cfg* to root directory and replace file extension (.htmp/.ctmp -> .h/.c).

**2. Change default HAL library include to target microprocessor inside ***timer_cfg.h***:**

Following example shows HAL library include for STM32L4 family:
```C
// USER INCLUDE BEGIN...

#include "stm32l4xx_hal.h"

// USER INCLUDE END...
```

**3. Configure UART module for application needs by changing ***timer_cfg.h***. Configuration options are following:**

| Configuration | Description |
| --- | --- |
| **TIMER_CFG_ASSERT_EN** 		        | Enable/Disable assertions |
| **TIMER_ASSERT** 		                | Assert definition |


**4. List all needed Timer instances inside ***timer_cfg.h***:**
```C
/**
 *  Timer instances
 *
 *  @note   Must start with enumeration of 0!
 */
typedef enum
{
    // USER CODE BEGIN...

    eTIM2_RTOS = 0,         /**<RTOS diagnostics timer */


    // USER CODE END...

    eTIM_INST_NUM_OF,       /**<Number of all timer instances */

} timer_inst_t;
```

**5. Configure all needed Timer instances inside ***timer_cfg.c***:**
```C
/**
 *      Timer Instance Configuration
 */
static const tim_inst_cfg_t g_tim_inst_cfg[eTIM_INST_NUM_OF] =
{
    // USER CODE BEGIN...

    /**
     *      TIM2 is being used for RTOS diagnostics purposes as free running timer
     *
     *  Timer input clock frequency:    80 MHz
     *  Timer base clock frequency:     100 kHz = 80MHz/800 (psc), resolution@100kHz = 10us
     */
    [eTIM2_RTOS]    = { .p_instance = TIM2, .psc = 800U, .per = UINT32_MAX, .mode = TIM_COUNTERMODE_UP, .start = true },


    // USER CODE END...
};
```

**6. Initialize Timer module**
```C
// Init timer instance TIM2
if ( eTIMER_OK != timer_init( eTIM2_RTOS ))
{
    // Initialization failed...

    // Further actions here...
}
```

**7. Get ticks from timer**
```C
uint32_t freertos_timer_get_tick(void)
{
    uint32_t ticks = 0U;

    (void) timer_cnt_get( eTIM2_RTOS, &ticks );

    return ticks;
}
```