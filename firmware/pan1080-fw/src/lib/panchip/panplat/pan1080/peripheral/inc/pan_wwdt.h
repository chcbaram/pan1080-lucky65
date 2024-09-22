/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series wwdt driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef __PAN_WWDT_H__
#define __PAN_WWDT_H__

/**
 * @brief Wwdt Interface
 * @defgroup wwdt_interface Wwdt Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif


/**@defgroup WWDT_PRESCARE_FLAG Wdt prescare 
 * @brief       Wdt prescare definition
 * @{ */
typedef enum _WWDT_Prescale
{
    WWDT_PRESCALER_1        = (0UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 1     \hideinitializer */
    WWDT_PRESCALER_2        = (1UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 2     \hideinitializer */
    WWDT_PRESCALER_4        = (2UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 4     \hideinitializer */
    WWDT_PRESCALER_8        = (3UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 8     \hideinitializer */
    WWDT_PRESCALER_16       = (4UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 16    \hideinitializer */
    WWDT_PRESCALER_32       = (5UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 32    \hideinitializer */
    WWDT_PRESCALER_64       = (6UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 64    \hideinitializer */
    WWDT_PRESCALER_128      = (7UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 128   \hideinitializer */
    WWDT_PRESCALER_192      = (8UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 192   \hideinitializer */
    WWDT_PRESCALER_256      = (9UL << WWDT_CTL_PSCSEL_Pos),   /*!< WWDT setting prescaler to 256   \hideinitializer */
    WWDT_PRESCALER_384      = (0xAUL << WWDT_CTL_PSCSEL_Pos), /*!< WWDT setting prescaler to 384   \hideinitializer */
    WWDT_PRESCALER_512      = (0xBUL << WWDT_CTL_PSCSEL_Pos), /*!< WWDT setting prescaler to 512   \hideinitializer */
    WWDT_PRESCALER_768      = (0xCUL << WWDT_CTL_PSCSEL_Pos), /*!< WWDT setting prescaler to 768   \hideinitializer */
    WWDT_PRESCALER_1024     = (0xDUL << WWDT_CTL_PSCSEL_Pos), /*!< WWDT setting prescaler to 1024  \hideinitializer */
    WWDT_PRESCALER_1536     = (0xEUL << WWDT_CTL_PSCSEL_Pos), /*!< WWDT setting prescaler to 1536  \hideinitializer */
    WWDT_PRESCALER_2048     = (0xFUL << WWDT_CTL_PSCSEL_Pos)  /*!< WWDT setting prescaler to 2048  \hideinitializer */
} WWDT_PrescaleDef;
/**@} */

#define WWDT_RELOAD_WORD          (0x00005AA5)                     /*!< Fill this value to RLD register to reload WWDT counter  \hideinitializer */


/**
  * @brief This function clear WWDT time-out reset system flag.
  * @return None
  */
__STATIC_INLINE void WWDT_ClearResetFlag(void)
{
    WWDT->STATUS = WWDT_STATUS_WWDTRF_Msk;
}

/**
  * @brief This function clear WWDT compare match interrupt flag.
  * @return None
  */
__STATIC_INLINE void WWDT_ClearIntFlag(void)
{
    WWDT->STATUS = WWDT_STATUS_WWDTIF_Msk;
}

/**
  * @brief This function clear WWDT compare match event flag.
  * @return None
  */
__STATIC_INLINE void WWDT_ClearWWDTFFlag(void)
{
    WWDT->STATUS = WWDT_STATUS_WWDTF_Msk;
}

/**
  * @brief This function is use to get WWDT time-out reset system flag.
  * @return WWDT reset system or not
  * @retval false WWDT did not cause system reset
  * @retval true  WWDT caused system reset
  */
__STATIC_INLINE bool WWDT_GetResetFlag(void)
{
    return (WWDT->STATUS & WWDT_STATUS_WWDTRF_Msk) ? true : false;
}

/**
  * @brief This function is used to indicate WWDT compare match interrupt flag.
  * @return WWDT compare match interrupt occurred or not
  * @retval false WWDT compare match interrupt did not occur
  * @retval true  WWDT compare match interrupt occurred
  */
__STATIC_INLINE bool WWDT_GetIntFlag(void)
{
    return (WWDT->STATUS & WWDT_STATUS_WWDTIF_Msk) ? true : false;
}

/**
  * @brief This function is used to indicate WWDT compare match event flag.
  * @return WWDT compare match interrupt occurred or not
  * @retval false WWDT compare match interrupt did not occur
  * @retval true  WWDT compare match interrupt occurred
  */
__STATIC_INLINE bool WWDT_GetWWDTFFlag(void)
{
    return (WWDT->STATUS & WWDT_STATUS_WWDTF_Msk) ? true : false;
}

/**
  * @brief This function to reflects current WWDT counter value
  * @return Return current WWDT counter value
  */
__STATIC_INLINE uint32_t WWDT_GetCounter(void)
{
    return WWDT->CNT;
}

/**
  * @brief This function reflects current WWDT compare value (CMPDAT)
  * @return Return current WWDT compare value
  */
__STATIC_INLINE uint32_t WWDT_GetCompareValue(void)
{
    return (WWDT->CTL & WWDT_CTL_CMPDAT_Msk) >> WWDT_CTL_CMPDAT_Pos;
}

/**
  * @brief This function is used to reload the WWDT counter value to 0x3F.
  * @return None
  * @details After WWDT enabled, application must reload WWDT counter while
  *          current counter is less than compare value and larger than 0,
  *          otherwise WWDT will cause system reset.
  */
__STATIC_INLINE void WWDT_ReloadCounter(void)
{
    WWDT->RLDCNT  = WWDT_RELOAD_WORD;
}

/**
 * @brief This function make WWDT module start counting with different counter period and compared window value
 * @param[in] preScale  Prescale period for the WWDT counter period. Valid values are:
 *              - \ref WWDT_PRESCALER_1
 *              - \ref WWDT_PRESCALER_2
 *              - \ref WWDT_PRESCALER_4
 *              - \ref WWDT_PRESCALER_8
 *              - \ref WWDT_PRESCALER_16
 *              - \ref WWDT_PRESCALER_32
 *              - \ref WWDT_PRESCALER_64
 *              - \ref WWDT_PRESCALER_128
 *              - \ref WWDT_PRESCALER_192
 *              - \ref WWDT_PRESCALER_256
 *              - \ref WWDT_PRESCALER_384
 *              - \ref WWDT_PRESCALER_512
 *              - \ref WWDT_PRESCALER_768
 *              - \ref WWDT_PRESCALER_1024
 *              - \ref WWDT_PRESCALER_1536
 *              - \ref WWDT_PRESCALER_2048
 * @param[in] u32CmpValue Window compared value. Valid values are between 0x0 to 0x3F
 * @param[in] u32EnableInt Enable WWDT interrupt or not. Valid values are TRUE and  FALSE
 * @return None
 * @note Application can call this function can only once after boot up
 */
void WWDT_Open(WWDT_PrescaleDef preScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

/**
 * @brief This function stops WWDT counting 
 * @return None
 */
void WWDT_Close(void);

/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_WWDT_H__ */

