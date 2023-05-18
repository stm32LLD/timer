// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      timer.c
*@brief     Timer LL Driver based on STM32 HAL library
*@author    Ziga Miklosic
*@email     ziga.miklosic@gmail.si
*@date      25.04.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/*!
* @addtogroup TIMER
* @{ <!-- BEGIN GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "timer.h"
#include "../../timer_cfg.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Timer control
 */
typedef struct
{
    TIM_HandleTypeDef   handle;         /**<UART handler */
    bool                is_init;        /**<Initialization flag */
} timer_ctrl_t;


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/**
 *  Timer control block
 */
static timer_ctrl_t g_tim_inst[eTIM_INST_NUM_OF] = {0};

////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
static void timer_enable_clock  (const TIM_TypeDef * p_inst);
static void timer_disable_clock (const TIM_TypeDef * p_inst);



////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Enable Timer clock
*
* @param[in]    p_inst  - Timer instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void timer_enable_clock(const TIM_TypeDef * p_inst)
{
#if defined(TIM1)
    if ( TIM1 == p_inst )
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
#endif

#if defined(TIM2)
    if ( TIM2 == p_inst )
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
#endif

#if defined(TIM3)
    if ( TIM3 == p_inst )
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
#endif

#if defined(TIM4)
    if ( TIM4 == p_inst )
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
#endif

#if defined(TIM5)
    if ( TIM5 == p_inst )
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
    }
#endif

#if defined(TIM6)
    if ( TIM6 == p_inst )
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
    }
#endif

#if defined(TIM7)
    if ( TIM7 == p_inst )
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
    }
#endif

#if defined(TIM8)
    if ( TIM8 == p_inst )
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
#endif

#if defined(TIM15)
    if ( TIM15 == p_inst )
    {
        __HAL_RCC_TIM15_CLK_ENABLE();
    }
#endif

#if defined(TIM16)
    if ( TIM16 == p_inst )
    {
        __HAL_RCC_TIM16_CLK_ENABLE();
    }
#endif

#if defined(TIM17)
    if ( TIM17 == p_inst )
    {
        __HAL_RCC_TIM17_CLK_ENABLE();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Disable Timer clock
*
* @param[in]    p_inst  - Timer instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void timer_disable_clock(const TIM_TypeDef * p_inst)
{
#if defined(TIM1)
    if ( TIM1 == p_inst )
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
#endif

#if defined(TIM2)
    if ( TIM2 == p_inst )
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
#endif

#if defined(TIM3)
    if ( TIM3 == p_inst )
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
#endif

#if defined(TIM4)
    if ( TIM4 == p_inst )
    {
        __HAL_RCC_TIM4_CLK_DISABLE();
    }
#endif

#if defined(TIM5)
    if ( TIM5 == p_inst )
    {
        __HAL_RCC_TIM5_CLK_DISABLE();
    }
#endif

#if defined(TIM6)
    if ( TIM6 == p_inst )
    {
        __HAL_RCC_TIM6_CLK_DISABLE();
    }
#endif

#if defined(TIM7)
    if ( TIM7 == p_inst )
    {
        __HAL_RCC_TIM7_CLK_DISABLE();
    }
#endif

#if defined(TIM8)
    if ( TIM8 == p_inst )
    {
        __HAL_RCC_TIM8_CLK_DISABLE();
    }
#endif

#if defined(TIM15)
    if ( TIM15 == p_inst )
    {
        __HAL_RCC_TIM15_CLK_DISABLE();
    }
#endif

#if defined(TIM16)
    if ( TIM16 == p_inst )
    {
        __HAL_RCC_TIM16_CLK_DISABLE();
    }
#endif

#if defined(TIM17)
    if ( TIM17 == p_inst )
    {
        __HAL_RCC_TIM17_CLK_DISABLE();
    }
#endif
}

#if defined(TIM1) || defined(TIM15)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM1 BRK and TIM15 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM1_BRK_TIM15_IRQHandler(void)
    {
        // TODO: Process TIM1 break or TIM15 IRQ
    }
#endif

#if defined(TIM1) || defined(TIM16)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM1 UP and TIM16 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM1_UP_TIM16_IRQHandler(void)
    {
        // TODO: Process TIM1 UP or TIM16 IRQ
    }
#endif

#if defined(TIM1) || defined(TIM17)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM1 TRG, COM and TIM17 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM1_TRG_COM_TIM17_IRQHandler(void)
    {
        // TODO: Process TIM1 TRG,COM or TIM17 IRQ
    }
#endif

#if defined(TIM1)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM1 CC IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM1_CC_IRQHandler(void)
    {
        // TODO: Process TIM1 CC IRQ
    }
#endif

#if defined(TIM2)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM2 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM2_IRQHandler(void)
    {
        // TODO: Process TIM2 IRQ
    }
#endif

#if defined(TIM3)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM3 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM3_IRQHandler(void)
    {
        // TODO: Process TIM3 IRQ
    }
#endif

#if defined(TIM4)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM4 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM4_IRQHandler(void)
    {
        // TODO: Process TIM4 IRQ
    }
#endif

#if defined(TIM5)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM5 IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM5_IRQHandler(void)
    {
        // TODO: Process TIM5 IRQ
    }
#endif

#if defined(TIM8)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM8 BRK IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM8_BRK_IRQHandler(void)
    {
        // TODO: Process TIM8 BRK IRQ
    }

    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM8 UP IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM8_UP_IRQHandler(void)
    {
        // TODO: Process TIM8 UP IRQ
    }

    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM8 TRG,COM IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM8_TRG_COM_IRQHandler(void)
    {
        // TODO: Process TIM8 TRG,COM IRQ
    }

    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        TIM8 CC IRQ
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void TIM8_CC_IRQHandler(void)
    {
        // TODO: Process TIM8 CC IRQ
    }
#endif


////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup TIMER_API
* @{ <!-- BEGIN GROUP -->
*
* 	Following function are part of Timer LL Driver API.
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Initialize timer
*
* @param[in]    tim_inst    - Timer instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_init(const timer_inst_t tim_inst)
{
    timer_status_t          status              = eTIMER_OK;
    TIM_ClockConfigTypeDef  sClockSourceConfig  = {0};

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );

    if ( tim_inst < eTIM_INST_NUM_OF )
    {
        if ( false == g_tim_inst[tim_inst].is_init )
        {
            // Get timer configurations
            const tim_inst_cfg_t * p_cfg = timer_cfg_get_inst( tim_inst );

            // Init timer clock
            timer_enable_clock( p_cfg->p_instance );

            // Prepare HAL init structure
            g_tim_inst[tim_inst].handle.Instance                = p_cfg->p_instance;
            g_tim_inst[tim_inst].handle.Init.Prescaler          = p_cfg->psc;
            g_tim_inst[tim_inst].handle.Init.CounterMode        = p_cfg->mode;
            g_tim_inst[tim_inst].handle.Init.Period             = p_cfg->per;
            g_tim_inst[tim_inst].handle.Init.ClockDivision      = TIM_CLOCKDIVISION_DIV1;
            g_tim_inst[tim_inst].handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;

            // Init timer
            if ( HAL_OK != HAL_TIM_Base_Init( &g_tim_inst[tim_inst].handle ))
            {
                status |= eTIMER_ERROR;
                TIMER_ASSERT( 0 );
            }

            // Internal clock source
            sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

            // Init timer clock source
            if ( HAL_OK != HAL_TIM_ConfigClockSource( &g_tim_inst[tim_inst].handle, &sClockSourceConfig ))
            {
                status |= eTIMER_ERROR;
                TIMER_ASSERT( 0 );
            }

            // Init success
            if ( eTIMER_OK == status )
            {
                // TODO: Enable interrupt that are needed by specific timer profile
                //__HAL_TIM_ENABLE_IT( &g_tim_inst[tim_inst].handle, TIM_IT_UPDATE );
                //__HAL_TIM_ENABLE_IT( &g_tim_inst[tim_inst].handle, TIM_IT_CC2 );

                // TODO:  Setup timer interrupt priority and enable it
                //NVIC_SetPriority( p_cfg->irq_num, p_cfg->irq_prio );
                //NVIC_EnableIRQ( p_cfg->irq_num );

                // Init success
                g_tim_inst[tim_inst].is_init = true;

                // Start timer
                if ( true == p_cfg->start )
                {
                    status |= timer_start( tim_inst );
                }
            }
        }
    }
    else
    {
        status = eTIMER_ERROR;
    }


    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        De-Initialize timer
*
* @param[in]    tim_inst    - Timer instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_deinit(const timer_inst_t tim_inst)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );

    if ( tim_inst < eTIM_INST_NUM_OF )
    {
        if ( true == g_tim_inst[tim_inst].is_init )
        {
            // De-initilize timer
            HAL_TIM_Base_DeInit( &g_tim_inst[tim_inst].handle );

            // Disable timer clock
            timer_disable_clock( g_tim_inst[tim_inst].handle.Instance );

            // Init success
            if ( eTIMER_OK == status )
            {
                g_tim_inst[tim_inst].is_init = false;
            }
        }
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Get Timer initialization flag
*
* @param[in]    tim_inst    - Timer instance
* @param[out]   p_is_init   - Pointer to init flag
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_is_init(const timer_inst_t tim_inst, bool * const p_is_init)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );
    TIMER_ASSERT( NULL != p_is_init );

    if  (   ( tim_inst < eTIM_INST_NUM_OF )
        &&  ( NULL != p_is_init ))
    {
        *p_is_init = g_tim_inst[tim_inst].is_init;
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Start timer
*
* @param[in]    tim_inst    - Timer instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_start(const timer_inst_t tim_inst)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );
    TIMER_ASSERT( true == g_tim_inst[tim_inst].is_init );

    if  (   ( tim_inst < eTIM_INST_NUM_OF )
        &&  ( true == g_tim_inst[tim_inst].is_init ))
    {
        if ( HAL_OK != HAL_TIM_Base_Start( &g_tim_inst[tim_inst].handle ))
        {
            status = eTIMER_ERROR;
        }
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Stop timer
*
* @param[in]    tim_inst    - Timer instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_stop(const timer_inst_t tim_inst)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );
    TIMER_ASSERT( true == g_tim_inst[tim_inst].is_init );

    if  (   ( tim_inst < eTIM_INST_NUM_OF )
        &&  ( true == g_tim_inst[tim_inst].is_init ))
    {
        if ( HAL_OK != HAL_TIM_Base_Stop( &g_tim_inst[tim_inst].handle ))
        {
            status = eTIMER_ERROR;
        }
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Set timer counter
*
* @param[in]    tim_inst    - Timer instance
* @param[in]    counter     - Timer counter value
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_cnt_set(const timer_inst_t tim_inst, const uint32_t counter)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );
    TIMER_ASSERT( true == g_tim_inst[tim_inst].is_init );

    if  (   ( tim_inst < eTIM_INST_NUM_OF )
        &&  ( true == g_tim_inst[tim_inst].is_init ))
    {
        __HAL_TIM_SET_COUNTER( &g_tim_inst[tim_inst].handle, counter );
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Set timer counter to 0
*
* @param[in]    tim_inst    - Timer instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_cnt_reset(const timer_inst_t tim_inst)
{
    timer_status_t status = eTIMER_OK;

    status = timer_cnt_set( tim_inst, 0U );

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Get timer counter
*
* @param[in]    tim_inst    - Timer instance
* @param[out]   p_counter   - Pointer to counter value
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
timer_status_t timer_cnt_get(const timer_inst_t tim_inst, uint32_t * const p_counter)
{
    timer_status_t status = eTIMER_OK;

    TIMER_ASSERT( tim_inst < eTIM_INST_NUM_OF );
    TIMER_ASSERT( true == g_tim_inst[tim_inst].is_init );
    TIMER_ASSERT( NULL != p_counter );

    if  (   ( tim_inst < eTIM_INST_NUM_OF )
        &&  ( true == g_tim_inst[tim_inst].is_init )
        &&  ( NULL != p_counter ))
    {
        *p_counter = __HAL_TIM_GET_COUNTER( &g_tim_inst[tim_inst].handle );
    }
    else
    {
        status = eTIMER_ERROR;
    }

    return status;
}




static TIM_HandleTypeDef gh_tim1 = {0};




static void timer_1_init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**TIM1 GPIO Configuration
    PC13     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */

    // TODO: Check out that pins!

    GPIO_InitStruct.Pin         = GPIO_PIN_13;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull        = GPIO_NOPULL;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate   = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin         = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull        = GPIO_NOPULL;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate   = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin         = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull        = GPIO_NOPULL;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate   = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}




timer_status_t timer_1_init(const timer_1_cfg_t * const p_cfg)
{
    timer_status_t status = eTIMER_OK;

    TIM_ClockConfigTypeDef          sClockSourceConfig      = {0};
    //TIM_MasterConfigTypeDef         sMasterConfig           = {0};
    TIM_OC_InitTypeDef              sConfigOC               = {0};
    TIM_BreakDeadTimeConfigTypeDef  sBreakDeadTimeConfig    = {0};

    // Init Timer PWM gpios
    timer_1_init_gpio();

    // Enable timer clock
    __HAL_RCC_TIM1_CLK_ENABLE();


    // Input clock is 150 MHz
    // Frequency times 2 as it is up/down counter
    const uint32_t period = (uint32_t)( 150e6 / ( 2 * p_cfg->freq ));

    gh_tim1.Instance                  = TIM1;
    gh_tim1.Init.Prescaler            = 0;
    gh_tim1.Init.CounterMode          = TIM_COUNTERMODE_CENTERALIGNED1;
    gh_tim1.Init.Period               = period;
    gh_tim1.Init.ClockDivision        = TIM_CLOCKDIVISION_DIV1;
    gh_tim1.Init.RepetitionCounter    = 0;
    gh_tim1.Init.AutoReloadPreload    = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if ( HAL_OK != HAL_TIM_Base_Init( &gh_tim1 ))
    {
        status = eTIMER_ERROR;
    }


    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if ( HAL_OK != HAL_TIM_ConfigClockSource( &gh_tim1, &sClockSourceConfig ))
    {
        status = eTIMER_ERROR;
    }


    if ( HAL_OK != HAL_TIM_PWM_Init( &gh_tim1 ))
    {
        status = eTIMER_ERROR;
    }


    // TODO: This is important for ADC triggering
/*    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&gh_tim1, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }*/


    sConfigOC.OCMode        = TIM_OCMODE_PWM1;
    sConfigOC.Pulse         = 0;
    sConfigOC.OCPolarity    = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode    = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState   = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState  = TIM_OCNIDLESTATE_RESET;

    if ( HAL_OK != HAL_TIM_PWM_ConfigChannel( &gh_tim1, &sConfigOC, TIM_CHANNEL_1 ))
    {
        status = eTIMER_ERROR;
    }

    if ( HAL_OK != HAL_TIM_PWM_ConfigChannel( &gh_tim1, &sConfigOC, TIM_CHANNEL_2 ))
    {
        status = eTIMER_ERROR;
    }

    if ( HAL_OK != HAL_TIM_PWM_ConfigChannel( &gh_tim1, &sConfigOC, TIM_CHANNEL_3 ))
    {
      status = eTIMER_ERROR;
    }


    // Calculate deadtime
    // p_cfg->deadtime [us] * timer_freq [MHz]
    const uint32_t deadtime = (uint32_t) ( p_cfg->deadtime * 150e6 );

    sBreakDeadTimeConfig.OffStateRunMode    = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode   = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel          = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime           = deadtime;
    sBreakDeadTimeConfig.BreakState         = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity      = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter        = 0;
    sBreakDeadTimeConfig.BreakAFMode        = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State        = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity     = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter       = 0;
    sBreakDeadTimeConfig.Break2AFMode       = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput    = TIM_AUTOMATICOUTPUT_DISABLE;


    if ( HAL_OK != HAL_TIMEx_ConfigBreakDeadTime( &gh_tim1, &sBreakDeadTimeConfig ))
    {
        status = eTIMER_ERROR;
    }

    return status;
}

timer_status_t timer_1_set_pwm(const float32_t duty_u, const float32_t duty_v, const float32_t duty_w)
{
    timer_status_t status = eTIMER_OK;

    (void) duty_u;
    (void) duty_v;
    (void) duty_w;

    return status;
}



////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
