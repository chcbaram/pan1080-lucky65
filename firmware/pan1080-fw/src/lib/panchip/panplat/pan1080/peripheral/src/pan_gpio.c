/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 19/11/18 18:33 $
 * @brief    Panchip series GPIO driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_gpio.h"


/** @addtogroup Panchip_Device_Driver Panchip Device Driver
  @{
*/

/** @addtogroup Panchip_GPIO_Driver GPIO Driver
  @{
*/


/** @addtogroup Panchip_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 * @param[in]   u32Mode     Operation mode.
 *                          - \ref GPIO_MODE_INPUT,
 *                          - \ref GPIO_MODE_OUTPUT,
 *                          - \ref GPIO_MODE_OPEN_DRAIN,
 *                          - \ref GPIO_MODE_QUASI
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, GPIO_ModeDef Mode)
{
    uint32_t i;

    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            gpio->MODE = (gpio->MODE & ~(0x3 << (i << 1))) | (Mode << (i << 1));
        }
    }
    if((GPIO_MODE_INPUT==Mode)|(GPIO_MODE_QUASI==Mode))
        gpio->DINOFF  &= ~ ( u32PinMask<<16 );
}


/*@}*/ /* end of group Panchip_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Panchip_GPIO_Driver */

/*@}*/ /* end of group Panchip_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
