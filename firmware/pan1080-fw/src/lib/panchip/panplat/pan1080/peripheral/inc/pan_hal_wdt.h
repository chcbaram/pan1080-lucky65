/**************************************************************************//**
* @file     pan_hal_wdt.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series Watchdog Timer (WDT) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __PAN_HAL_WDT_H
#define __PAN_HAL_WDT_H

#include "pan_hal_def.h"

/**
 * @defgroup pan_hal_wdt WDT driver
 * @{
 * @ingroup pan_wdt
 * @brief   Watchdog Timer (WDT) driver group description.
 */

/**
 * @brief  Structure definition for initializing the Watchdog Timer.
 */
typedef struct 
{
    WDT_TimeoutDef Timeout;      /**< @ref WDT_TimeoutDef: Timeout duration for the WDT. */
    WDT_ResetDelayDef ResetDelay;/**< @ref WDT_ResetDelayDef: Delay before the WDT initiates a reset after timeout. */
    uint32_t  ClockSource;       /**< Clock source for the WDT. */
    uint32_t ResetSwitch;        /**< Enable or disable the reset functionality of the WDT. */
    uint32_t WakeupSwitch;       /**< Enable or disable the wakeup functionality of the WDT. */
}WDT_Init_Opt;

/**
 * @brief  Type definition for the Watchdog Timer callback function.
 */
typedef void (*WDT_CallbackFunc)(void);

/**
 * @brief  Structure to manage the Watchdog Timer.
 */
typedef struct
{
    WDT_Init_Opt InitObj;        /**< @ref WDT_Init_Opt: Initialization parameters for the WDT. */
    WDT_CallbackFunc Callback;   /**< @ref WDT_CallbackFunc: Callback function for WDT events. */
} WDT_HandleTypeDef;

/**
 * @brief  Global array of Watchdog Timer handle structures.
 */
extern WDT_HandleTypeDef WDT_Handle_Array[1];
#define WDT_OBJ WDT_Handle_Array[0]

/**
 * @brief  Initialize the Watchdog Timer.
 * 
 * @param  wdt Pointer to a @ref WDT_HandleTypeDef structure that contains
 *         the configuration information for the specified Watchdog Timer.
 */
void HAL_WDT_Init(WDT_HandleTypeDef *wdt);

/**
 * @brief  DeInitialize the specified Watchdog Timer.
 * 
 * @param  wdt Pointer to a @ref WDT_HandleTypeDef structure that contains
 *         the configuration information for the specified Watchdog Timer.
 */
void HAL_WDT_DeInit(WDT_HandleTypeDef *wdt);

/**
 * @brief  Reset the counter of the specified Watchdog Timer.
 * 
 * @param  wdt Pointer to a @ref WDT_HandleTypeDef structure representing
 *         the Watchdog Timer instance.
 */
void HAL_WDT_Feed(WDT_HandleTypeDef *wdt);

/**
 * @brief  Initialize the specified Watchdog Timer and enable its interrupts.
 * 
 * @param  wdt Pointer to a @ref WDT_HandleTypeDef structure that contains
 *         the configuration information for the specified Watchdog Timer.
 */
void HAL_WDT_Init_INT(WDT_HandleTypeDef *wdt);

/**
 * @brief  DeInitialize the specified Watchdog Timer and disable its interrupts.
 * 
 * @param  wdt Pointer to a @ref WDT_HandleTypeDef structure that contains
 *         the configuration information for the specified Watchdog Timer.
 */
void HAL_WDT_DeInit_INT(WDT_HandleTypeDef *wdt);

/**
 * @brief  Interrupt handler for the Watchdog Timer.
 */
void WDT_IRQHandler(void);
/** @} */ // end of group
#endif /* __PAN_HAL_WDT_H */
