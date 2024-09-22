/**************************************************************************//**
* @file     pan_hal_wdt.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series WDT (Watchdog Timer) HAL (Hardware Abstraction Layer) header file.
*
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __PAN_HAL_TIMER_H
#define __PAN_HAL_TIMER_H

#include "pan_hal_def.h"

/**
 * @brief TIMER HAL Interface
 * @defgroup timer_hal_interface Timer HAL Interface
 * @{
 */


/**
 * @brief Enumeration of timer callback flags.
 *
 * These flags are used to identify different timer events that can trigger callbacks.
 */
typedef enum
{
    TIMER_CB_FLAG_CNT = 0X01, /**< Callback flag for timer count event. */
    TIMER_CB_FLAG_CAP = 0X02, /**< Callback flag for timer capture event. */
    TIMER_CB_FLAG_WK = 0X03   /**< Callback flag for timer wake-up event. */
} TIMER_Cb_Flag_Opt;

/**
 * @brief Enumeration of timer callback flags.
 *
 * These flags are used to identify different timer events that can trigger callbacks.
 */
typedef void (*TIMER_CallbackFunc)(TIMER_Cb_Flag_Opt); /**< Callback function type for timer events. */

/**
 * @brief Enumeration of timer modes.
 *
 * This enumeration defines various timer operation modes.
 */
typedef enum
{
    TIMER_MODE_BASECNT,    /**< Timer mode: Base counter. */
    TIMER_MODE_EVENTCNT,   /**< Timer mode: Event counter. */
    TIMER_MODE_INCAP,      /**< Timer mode: Input capture. */
    TIMER_MODE_WAKEUP      /**< Timer mode: Wake-up. */
} TIMER_Mode_Opt;


typedef struct
{
    uint8_t pass; /**< Placeholder field in TIMER_Interrupt_Opt structure. */
} TIMER_Interrupt_Opt;

/**
 * @brief Structure containing timer initialization options.
 *
 * This structure holds configuration parameters for initializing a timer.
 */
typedef struct 
{
    uint32_t Freq;               /**< Timer frequency. */
    TIMER_Mode_Opt Mode;         /**< Timer mode. */
    TIMER_CntModeDef CntMode;   /**< Timer count mode definition. */
    TIMER_CapModeDef CapMode;   /**< Timer capture mode definition. */
    uint32_t CmpValue;           /**< Timer comparison value. */
    TIMER_CapEdgeDef CapEdge;   /**< Timer capture edge definition. */
    TIMER_EvtCntEdgeDef EvtCntEdge; /**< Timer event count edge definition. */
    TIMER_CapSrcDef CapSrc;     /**< Timer capture source definition. */
    uint32_t Prescale;           /**< Timer prescaler value. */
} TIMER_Init_Opt;


typedef struct
{
    TIMER_T *TIMERx; /*!< Timer registers base address. */

    TIMER_Init_Opt InitObj; /*!< Timer initialization parameters. */

    TIMER_Interrupt_Opt InterruptObj;

    IRQn_Type IRQn; /*!< Timer IRQ number. */

    TIMER_CallbackFunc Callback; /*!< Timer callback function. */

} TIMER_HandleTypeDef;

#define TIMER0_OBJ TIMER_Handle_Array[0] /**< Reference to the TIMER0 handle object. */
#define TIMER1_OBJ TIMER_Handle_Array[1] /**< Reference to the TIMER1 handle object. */
#define TIMER2_OBJ TIMER_Handle_Array[2] /**< Reference to the TIMER2 handle object. */
extern TIMER_HandleTypeDef TIMER_Handle_Array[3]; /**< Array of TIMER handle structures. */

/**
  * @brief  Initialize the TIMER peripheral.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval None
  */
void HAL_TIMER_Init(TIMER_HandleTypeDef *timer);

/**
  * @brief  DeInitialize the TIMER peripheral.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval None
  */
void HAL_TIMER_DeInit(TIMER_HandleTypeDef *timer);

/**
  * @brief  Initialize the TIMER peripheral with interrupt support.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval None
  */
void HAL_TIMER_Init_INT(TIMER_HandleTypeDef *timer);

/**
  * @brief  Initialize the TIMER peripheral with interrupt support.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval None
  */
void HAL_TIMER_DeInit_INT(TIMER_HandleTypeDef *timer);

/**
  * @brief  Start the TIMER peripheral.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval None
  */
void HAL_TIMER_Start(TIMER_HandleTypeDef *timer);

/**
  * @brief  Get the real frequency of the TIMER peripheral.
  * @param  timer: Pointer to a TIMER_HandleTypeDef structure that contains
  *                the configuration information for the TIMER peripheral.
  * @retval The real frequency of the TIMER peripheral.
  */
uint32_t HAL_TIMER_GetRealFreq(TIMER_HandleTypeDef *timer);

/** @} */ // end of group


#endif /* __PAN_HAL_PWM_H */
