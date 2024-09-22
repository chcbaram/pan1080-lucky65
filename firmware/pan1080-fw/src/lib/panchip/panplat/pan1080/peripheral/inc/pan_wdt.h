/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series wdt driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef __PAN_WDT_H__
#define __PAN_WDT_H__

/**
 * @brief Wdt Interface
 * @defgroup wdt_interface Wdt Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif

/**@defgroup WDT_TIMEOUT_FLAG Wdt timeout count
 * @brief       Wdt timeout count definition
 * @{ */
typedef enum _WDT_Timeout
{
    WDT_TIMEOUT_2POW4   = (0UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^4 * WDT clocks */
    WDT_TIMEOUT_2POW6   = (1UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^6 * WDT clocks */
    WDT_TIMEOUT_2POW8   = (2UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^8 * WDT clocks */
    WDT_TIMEOUT_2POW10  = (3UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^10 * WDT clocks */
    WDT_TIMEOUT_2POW12  = (4UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^12 * WDT clocks */
    WDT_TIMEOUT_2POW14  = (5UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^14 * WDT clocks */
    WDT_TIMEOUT_2POW16  = (6UL << WDT_CTL_TOUTSEL_Pos), /*!< WDT setting for timeout interval = 2^16 * WDT clocks */
    WDT_TIMEOUT_2POW18  = (7UL << WDT_CTL_TOUTSEL_Pos)  /*!< WDT setting for timeout interval = 2^18 * WDT clocks */
} WDT_TimeoutDef;
/**@} */

/**@defgroup WDT_RESET_DELAY_FLAG Wdt reset delay time
 * @brief       Wdt reset delay time definition
 * @{ */
typedef enum _WDT_ResetDelay
{
    WDT_RESET_DELAY_2CLK    = (3UL << WDT_ALTCTL_RSTDSEL_Pos),  /*!< WDT setting reset delay to 2 WDT clocks */
    WDT_RESET_DELAY_17CLK   = (2UL << WDT_ALTCTL_RSTDSEL_Pos),  /*!< WDT setting reset delay to 17 WDT clocks */
    WDT_RESET_DELAY_129CLK  = (1UL << WDT_ALTCTL_RSTDSEL_Pos),  /*!< WDT setting reset delay to 129 WDT clocks */
    WDT_RESET_DELAY_1025CLK = (0UL << WDT_ALTCTL_RSTDSEL_Pos)   /*!< WDT setting reset delay to 1025 WDT clocks */
} WDT_ResetDelayDef;
/**@} */


/** 
  * @brief This function clear WDT time-out reset system flag. 
  * @return None  
  */
__STATIC_INLINE void WDT_ClearResetFlag(void)
{
    WDT->CTL |= WDT_CTL_RSTF_Msk;
}

/** 
  * @brief This function clear WDT time-out flag. 
  * @return None 
  */
__STATIC_INLINE void WDT_ClearTimeoutFlag(void)
{
    WDT->CTL |= WDT_CTL_TOF_Msk;
}

/** 
  * @brief This function clear WDT time-out interrupt flag. 
  * @return None 
  */
__STATIC_INLINE void WDT_ClearTimeoutIntFlag(void)
{
    WDT->CTL |= WDT_CTL_IF_Msk;
}

/** 
  * @brief This function clear WDT time-out wake-up system flag. 
  * @return None 
  */
__STATIC_INLINE void WDT_ClearTimeoutWakeupFlag(void)
{
    WDT->CTL |= WDT_CTL_WKF_Msk;
}

/** 
  * @brief This function indicate WDT time-out to reset system or not.
  * @return WDT reset system or not
  * @retval false WDT did not cause system reset
  * @retval true  WDT caused system reset 
  */
__STATIC_INLINE bool WDT_GetResetFlag(void)
{
    return (WDT->CTL & WDT_CTL_RSTF_Msk) ? true : false;
}

/** 
  * @brief This function indicate WDT time-out evnet occurred or not.
  * @return WDT time-out evnet occurred or not
  * @retval false WDT time-out evnet did not occur
  * @retval true  WDT time-out evnet occurred 
  */
__STATIC_INLINE bool WDT_GetTimeoutFlag(void)
{
    return (WDT->CTL & WDT_CTL_TOF_Msk) ? true : false;
}

/** 
  * @brief This function indicate WDT time-out interrupt occurred or not.
  * @return WDT time-out interrupt occurred or not
  * @retval false WDT time-out interrupt did not occur
  * @retval true  WDT time-out interrupt occurred 
  */
__STATIC_INLINE bool WDT_GetTimeoutIntFlag(void)
{
    return (WDT->CTL & WDT_CTL_IF_Msk) ? true : false;
}

/** 
  * @brief This function indicate WDT time-out waked system up or not
  * @return WDT time-out waked system up or not
  * @retval false WDT did not wake up system
  * @retval true  WDT waked up system
  */
__STATIC_INLINE bool WDT_GetTimeoutWakeupFlag(void)
{
    return (WDT->CTL & WDT_CTL_WKF_Msk) ? true : false;
}

/** 
  * @brief  This function is used to reset 18-bit WDT counter.
  * @return None   
  * @details If WDT is activated and enabled to reset system, software must reset WDT counter 
  *  before WDT time-out plus reset delay reached. Or WDT generate a reset signal.
  */
__STATIC_INLINE void WDT_ResetCounter(void)
{
    WDT->CTL |= WDT_CTL_RSTCNT_Msk;
}

/**
 * @brief This function make WDT module start counting with different time-out interval
 * @param[in] u32TimeoutInterval  Time-out interval period of WDT module. Valid values are:
 *                - \ref WDT_TIMEOUT_2POW4
 *                - \ref WDT_TIMEOUT_2POW6
 *                - \ref WDT_TIMEOUT_2POW8
 *                - \ref WDT_TIMEOUT_2POW10
 *                - \ref WDT_TIMEOUT_2POW12
 *                - \ref WDT_TIMEOUT_2POW14
 *                - \ref WDT_TIMEOUT_2POW16
 *                - \ref WDT_TIMEOUT_2POW18
 * @param[in] u32ResetDelay Reset delay period while WDT time-out happened. Valid values are:
 *                - \ref WDT_RESET_DELAY_2CLK
 *                - \ref WDT_RESET_DELAY_17CLK
 *                - \ref WDT_RESET_DELAY_129CLK
 *                - \ref WDT_RESET_DELAY_1025CLK
 * @param[in] u32EnableReset Enable WDT reset system function. Valid values are TRUE and FALSE
 * @param[in] u32EnableWakeup Enable WDT wake-up system function. Valid values are TRUE and FALSE
 * @return None
 *
 * @code
 *
 *	CLK_SetWdtClkSrc(WDT_CLK_SRC_SEL_APBDIV);
 *	WDT_Open(WDT_TIMEOUT_2POW16, WDT_RESET_DELAY_1025CLK, FALSE, FALSE);
 *
 * @endcode
 */
void  WDT_Open(WDT_TimeoutDef u32TimeoutInterval,
                  WDT_ResetDelayDef u32ResetDelay,
                  uint32_t u32EnableReset,
                  uint32_t u32EnableWakeup);  
           
/**
 * @brief This function stops WDT counting and disable WDT module
 * @return None
 */
void WDT_Close(void);

/**
 * @brief This function enables the WDT time-out interrupt
 * @return None
 */
void WDT_EnableInt(void);

/**
 * @brief This function disables the WDT time-out interrupt
 * @return None
 */
void WDT_DisableInt(void);


/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_WDT_H__ */
