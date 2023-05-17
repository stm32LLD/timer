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

#ifdef STM32L4
    #include "stm32l4xx_hal.h"
#endif

#ifdef STM32G0
    #include "stm32g0xx_hal.h"
#endif

#ifdef STM32H7
    #include "stm32h7xx_hal.h"
#endif

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

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
