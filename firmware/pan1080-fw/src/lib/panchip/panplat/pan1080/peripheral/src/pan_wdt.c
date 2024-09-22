/**************************************************************************//**
 * @file     wdt.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/24 17:25 $
 * @brief    Panchip series WDT driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_wdt.h"


/**
 * @brief This function make WDT module start counting with different time-out interval
 * @param[in] timeoutInterval  Time-out interval period of WDT module. Valid values are:
 *                - \ref WDT_TIMEOUT_2POW4
 *                - \ref WDT_TIMEOUT_2POW6
 *                - \ref WDT_TIMEOUT_2POW8
 *                - \ref WDT_TIMEOUT_2POW10
 *                - \ref WDT_TIMEOUT_2POW12
 *                - \ref WDT_TIMEOUT_2POW14
 *                - \ref WDT_TIMEOUT_2POW16
 *                - \ref WDT_TIMEOUT_2POW18
 * @param[in] resetDelay Reset delay period while WDT time-out happened. Valid values are:
 *                - \ref WDT_RESET_DELAY_2CLK
 *                - \ref WDT_RESET_DELAY_17CLK
 *                - \ref WDT_RESET_DELAY_129CLK
 *                - \ref WDT_RESET_DELAY_1025CLK
 * @param[in] u32EnableReset Enable WDT reset system function. Valid values are TRUE and FALSE
 * @param[in] u32EnableWakeup Enable WDT wake-up system function. Valid values are TRUE and FALSE
 * @return None
 */
void  WDT_Open(WDT_TimeoutDef timeoutInterval,
               WDT_ResetDelayDef resetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup)
{

    WDT->CTL = timeoutInterval | WDT_CTL_WDTEN_Msk |
               (u32EnableReset << WDT_CTL_RSTEN_Pos) |
               (u32EnableWakeup << WDT_CTL_WKEN_Pos);
    WDT->ALTCTL = resetDelay;
    return;
}

/**
 * @brief This function stops WDT counting and disable WDT module
 * @param None
 * @return None
 */
void WDT_Close(void)
{
    WDT->CTL = 0;
    WDT->CTL |= (WDT_CTL_TOF_Msk | WDT_CTL_IF_Msk | WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk);
    return;
}

/**
 * @brief This function enables the WDT time-out interrupt
 * @param None
 * @return None
 */
void WDT_EnableInt(void)
{
    WDT->CTL |= WDT_CTL_INTEN_Msk;
    return;
}

/**
 * @brief This function disables the WDT time-out interrupt
 * @param None
 * @return None
 */
void WDT_DisableInt(void)
{
    WDT->CTL &= ~WDT_CTL_INTEN_Msk;
    return;
}


/*@}*/ /* end of group PN501_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN501_WDT_Driver */

/*@}*/ /* end of group PN501_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
