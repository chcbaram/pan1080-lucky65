/**************************************************************************//**
* @file     pan_hal_gpio.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series GPIO (General Purpose Input/Output) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PAN_HAL_GPIO_H__
#define __PAN_HAL_GPIO_H__

#include "pan_hal_def.h"

/**
 * @brief GPIO HAL Interface
 * @defgroup gpio_hal_interface Gpio HAL Interface
 * @{
 */

#define GPIO_INTEN_MASK_BASE                0x00010001UL
#define GPIO_INTTYPE_MASK_BASE              0x00000001UL

/* PIN ID Definitions */
typedef enum {
    P0_0 = 0x00,
    P0_1 = 0x01,
    P0_2 = 0x02,
    P0_3 = 0x03,
    P0_4 = 0x04,
    P0_5 = 0x05,
    P0_6 = 0x06,
    P0_7 = 0x07,
    P1_0 = 0x10,
    P1_1 = 0x11,
    P1_2 = 0x12,
    P1_3 = 0x13,
    P1_4 = 0x14,
    P1_5 = 0x15,
    P1_6 = 0x16,
    P1_7 = 0x17,
    P2_0 = 0x20,
    P2_1 = 0x21,
    P2_2 = 0x22,
    P2_3 = 0x23,
    P2_4 = 0x24,
    P2_5 = 0x25,
    P2_6 = 0x26,
    P2_7 = 0x27,
    P3_0 = 0x30,
    P3_1 = 0x31,
    P3_2 = 0x32,
    P3_3 = 0x33,
    P3_4 = 0x34,
    P3_5 = 0x35,
    P3_6 = 0x36,
    P3_7 = 0x37,
    P4_0 = 0x40,
    P4_1 = 0x41,
    P4_2 = 0x42,
    P4_3 = 0x43,
    P4_4 = 0x44,
    P4_5 = 0x45,
    P4_6 = 0x46,
    P4_7 = 0x47,
    P5_0 = 0x50,
    P5_1 = 0x51,
    P5_2 = 0x52,
    P5_3 = 0x53,
    P5_4 = 0x54,
    P5_5 = 0x55,
    P5_6 = 0x56,
    P5_7 = 0x57
} GPIO_Pin_Id;

#define WAKEUP_PIN      P0_4


/** 
  * @brief   Typedef for GPIO callback function pointer.
  */
typedef void (*GPIO_CallbackFunc)(void);

/** 
  * @brief   Global array to store GPIO callback functions.
  *          Supports up to 48 GPIO pins.
  */
extern GPIO_CallbackFunc GPIO_CallbackFunc_Array[48];

/**
  * @brief  MODE Constant Definitions     
  */
typedef enum _GPIO_Mode_Opt
{
    HAL_GPIO_MODE_INPUT_DIGITAL = 0x00000,  /*!< Input Digital Mode */
    HAL_GPIO_MODE_INPUT_ANALOG  = 0x10000,  /*!< Input Analog Mode */

    HAL_GPIO_MODE_OUTPUT        = 0x00001,  /*!< Output Mode */
    HAL_GPIO_MODE_OPEN_DRAIN    = 0x00002,  /*!< Open-Drain Mode */
    HAL_GPIO_MODE_QUASI         = 0x00003   /*!< Quasi-bidirectional Mode */

} GPIO_Mode_Opt;

/**
  * @brief  MODE Constant Definitions for GPIO Pull Options.
  */
typedef enum 
{
  HAL_PULL_DISABLE = 0U,    /**< No pull-up or pull-down. */
  HAL_PULL_UP,              /**< Enable pull-up. */
  HAL_PULL_DOWN             /**< Enable pull-down. */
} GPIO_Pull_Opt;

/**
  * @brief  GPIO Bit SET and Bit RESET enumeration
  */
typedef enum 
{
  HAL_GPIO_PIN_RESET = 0,  /**< Pin is in reset state. */
  HAL_GPIO_PIN_SET         /**< Pin is in set. */
} GPIO_Pinstate_Opt;

/**
  * @brief  GPIO Interrupt Type Constant Definitions   
  */
typedef enum
{
  HAL_GPIO_INT_RISING = 0x00010000UL,    /*!< Interrupt enable by Input Rising Edge */
  HAL_GPIO_INT_FALLING = 0x00000001UL,   /*!< Interrupt enable by Input Falling Edge */
  HAL_GPIO_INT_BOTH_EDGE = 0x00010001UL, /*!< Interrupt enable by both Rising Edge and Falling Edge */
  HAL_GPIO_INT_HIGH = 0x01010000UL,      /*!< Interrupt enable by Level-High */
  HAL_GPIO_INT_LOW = 0x01000001UL,       /*!< Interrupt enable by Level-Low */

  HAL_GPIO_INT_DISABLE = 0x00000000UL /*!< Interrupt disenable */
} GPIO_Interrupt_Mode_Opt;

/**
  * @brief  GPIO Interrupt Type Constant Definitions   
  */
typedef enum 
{
  Debounce_Time_1
} Debounce_Time_Opt;

/** 
  * @brief   GPIO Init structure definition  
  */ 
typedef struct
{   
    GPIO_Mode_Opt Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */
    GPIO_Pull_Opt Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */
    FunctionalState Debounce;/*!< Specifies the debounce state for the selected pins. */

}GPIO_Init_Opt;



/**
 * @struct GPIO_Interrupt_Opt
 * @brief  GPIO interrupt configuration structure definition.
 */
typedef struct
{   
    GPIO_Interrupt_Mode_Opt Mode;     
    GPIO_CallbackFunc CallbackFunc;
}GPIO_Interrupt_Opt;

extern uint8_t HAL_GPIO_SetDebounceTime(Debounce_Time_Opt time);
/**
  * @brief  Initializes a GPIO pin.
  * @param  PinID: Identifier of the GPIO pin to be initialized.
  * @param  GPIO_InitObj: Pointer to a structure containing GPIO initialization options.
  * @retval None
  */
extern void HAL_GPIO_Init(GPIO_Pin_Id PinID, GPIO_Init_Opt *GPIO_InitObj);


/**
  * @brief  Writes a digital state to a GPIO pin.
  * @param  PinID: Identifier of the GPIO pin to write to.
  * @param  PinState: The digital state to write to the GPIO pin (HAL_GPIO_PIN_SET or HAL_GPIO_PIN_RESET).
  * @retval None
  */
extern void HAL_GPIO_WritePin(GPIO_Pin_Id PinID ,GPIO_Pinstate_Opt PinState);

/**
  * @brief  Reads the digital state of a GPIO pin.
  * @param  PinID: Identifier of the GPIO pin to read.
  * @retval GPIO_Pinstate_Opt: The digital state of the GPIO pin (HAL_GPIO_PIN_SET or HAL_GPIO_PIN_RESET).
  */
extern GPIO_Pinstate_Opt HAL_GPIO_ReadPin(GPIO_Pin_Id PinID);

/**
  * @brief  Deinitializes a GPIO pin.
  * @param  PinID: Identifier of the GPIO pin to be deinitialized.
  */
extern void HAL_GPIO_DeInit(GPIO_Pin_Id PinID);

/**
  * @brief  Initializes a GPIO pin for interrupt handling.
  * @param  PinID: Identifier of the GPIO pin to be initialized for interrupts.
  * @param  GPIO_InterruptObj: Pointer to a structure containing GPIO interrupt configuration.
  * @retval None
  */
extern void HAL_GPIO_Init_INT(GPIO_Pin_Id PinID,GPIO_Interrupt_Opt *GPIO_InterruptObj);

/** @} */ // end of group
#endif
