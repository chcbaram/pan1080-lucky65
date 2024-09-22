/**************************************************************************//**
* @file     pan_hal_pwm.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series PWM (Pulse Width Modulation) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PAN_HAL_PWM_H
#define __PAN_HAL_PWM_H
#include "pan_hal_def.h"

/**
 * @brief PWM HAL Interface
 * @defgroup pwm_hal_interface Pwm HAL Interface
 * @{
 */

/**
 * @brief Enumeration of PWM channel identifiers.
 *
 * These values represent different PWM channels available.
 */
typedef enum
{
     PWM0_0 = 0x001,
     PWM0_1 = 0x002,
     PWM0_2 = 0x004,
     PWM0_3 = 0x008,
     PWM0_4 = 0x010,
     PWM0_5 = 0x020,
     PWM0_6 = 0x040,
     PWM0_7 = 0x080,
     PWM1_0 = 0x701,
     PWM1_1 = 0x702,
     PWM1_2 = 0x704,
     PWM1_3 = 0x708,
     PWM1_4 = 0x710,
     PWM1_5 = 0x720,
     PWM1_6 = 0x740,
     PWM1_7 = 0x780,
     PWM2_0 = 0x801,
     PWM2_1 = 0x802,
     PWM2_2 = 0x804,
     PWM2_3 = 0x808,
     PWM2_4 = 0x810,
     PWM2_5 = 0x820,
     PWM2_6 = 0x840,
     PWM2_7 = 0x880,
 } PWM_Ch_Id;

/**
 * @brief Enumeration of PWM operating modes.
 *
 * These values define different PWM operating modes.
 */
typedef enum
{
    PWM_MODE_INDEPENDENT   = 0x00,  /*!< Independent mode */
    PWM_MODE_COMPLEMENTARY = 0x01,  /*!< Complementary mode */
    PWM_MODE_SYNCHRONIZED  = 0x10,   /*!< Synchronized mode */
    PWM_MODE_GROUP = 0X100         /*!< Group mode */
} PWM_Mode_Opt;

/**
 * @brief Structure defining PWM initialization parameters.
 *
 * This structure holds the parameters required to initialize a PWM channel.
 */
typedef struct
{
    uint32_t Frequency;        /*!< PWM frequency in Hz */
    uint32_t DutyCycle;        /*!< Initial duty cycle value */
    PWM_OperateTypeDef OperateType; /*!< PWM operating type */
    PWM_Mode_Opt Mode;         /*!< PWM operating mode */
    bool Inverter;             /*!< Inverter control */
    uint32_t DZDuration;       /*!< Dead Zone Duration */
} PWM_Init_Opt;

/**
 * @brief Initialize a PWM channel with the specified configuration.
 *
 * @param ChID     PWM channel ID to initialize.
 * @param InitObj  Configuration parameters for PWM initialization.
 *
 * This function initializes a PWM channel with the provided configuration.
 */
void HAL_PWM_Init(PWM_Ch_Id ChID,PWM_Init_Opt InitObj);

/**
 * @brief DeInitialize a PWM channel.
 *
 * @param ChID   PWM channel ID to deinitialize.
 */
void HAL_PWM_DeInit(PWM_Ch_Id ChID);

/**
 * @brief Start a PWM channel.
 *
 * @param ChID   PWM channel ID to start.
 *
 * This function enables the output of the specified PWM channel and starts its operation.
 */
void HAL_PWM_Start(PWM_Ch_Id ChID);

/**
 * @brief Stop a PWM channel.
 *
 * @param ChID   PWM channel ID to stop.
 * @param force  Flag to force stop the channel.
 *
 * This function stops the specified PWM channel's operation. If 'force' is true, it forcefully stops the channel.
 * Otherwise, it stops the channel gracefully.
 */
void HAL_PWM_Stop(PWM_Ch_Id ChID,bool force);

/**
 * @brief Stop a PWM channel.
 *
 * @param ChID   PWM channel ID to stop.
 * @param force  Flag to force stop the channel.
 *
 * This function stops the specified PWM channel's operation. If 'force' is true, it forcefully stops the channel.
 * Otherwise, it stops the channel gracefully.
 */
void HAL_PWM_SetFreqAndDuty(PWM_Ch_Id ChID,uint32_t Frequency, uint32_t DutyCycle);

/** @} */ // end of group
#endif /* __PAN_HAL_PWM_H */

