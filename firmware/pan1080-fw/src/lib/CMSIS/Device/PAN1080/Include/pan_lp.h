/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series lowpower driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */

#ifndef		__PAN_LP_H__
#define		__PAN_LP_H__

/**
 * @brief Lowpower Interface
 * @defgroup lowpower_interface Lowpower Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif


#define	LP_EXT_P56_WK_EN								(1)		/*!< Gpio p56 wake up enable */
#define	LP_EXT_P56_WK_DISABLE							(0)		/*!< Gpio p56 wake up disable */

/**@defgroup LP_GPIO_WK_EDGE_FLAG Lowpower gpio wake up edge 
 * @brief       Lowpower gpio wake up edge select definitions
 * @{ */
#define	LP_EXT_P56_WK_EDGE_LOW							(0)		/*!< Gpio p56 wake up by falling edge */
#define	LP_EXT_P56_WK_EDGE_HIGH							(1)		/*!< Gpio p56 wake up by rising edge */
/**@} */

/**@defgroup LP_WK_SOURCE_FLAG Lowpower wakeup source select
 * @brief       Lowpower wakeup source select definitions
 * @{ */
#define LP_WKUP_MODE_SEL_GPIO_P56                       (0)		/*!< Wake up by gpio p56 */
#define LP_WKUP_MODE_SEL_32K                            (1)		/*!< Wake up by gpio rcl */
#define LP_WKUP_MODE_SEL_GPIO                       	(2)		/*!< Wake up by gpio gpio */
#define LP_WKUP_MODE_SEL_WDT                            (3)		/*!< Wake up by gpio wdt */
#define LP_WKUP_MODE_SEL_TIMER                      	(4)		/*!< Wake up by gpio timer */
#define LP_WKUP_MODE_SEL_KSCAN                      	(5)		/*!< Wake up by gpio kscan */
#define LP_WKUP_MODE_SEL_QDEC	                      	(6)		/*!< Wake up by gpio qdec */
/**@} */

/**
  * @brief  This function enable lowpower intterrupt
  * @param[in]  ana: where anac is anac module base address
  * @param[in]  u32Mask: intterrupt bit need to be enabled
  *					\ref ANAC_INT_STANDBY_INT_EN_Msk
  *					\ref ANAC_INT_SLEEP_INT_EN_Msk
  * @return   none
  */
__STATIC_INLINE void LP_EnableInt(ANA_T *ana,uint32_t u32Mask)
{
	ana->LP_INT_CTRL |= u32Mask;
}

/**
  * @brief  This function wait to clear wake up flag
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  u32Mask: bit need to be cleared
  * @return   none
  */
__STATIC_INLINE void LP_ClearWakeFlag(ANA_T *ana,uint32_t u32Mask)
{
    ana->LP_INT_CTRL = (ana->LP_INT_CTRL & (~(ANAC_INT_STANDBY_INT_FLG_Msk |
                        ANAC_INT_SLEEP_INT_FLG_Msk | ANAC_INT_P56_WKUP_FLG_Msk |
                        ANAC_INT_TMR32K_WKUP_FLG_Msk | ANAC_SRAM_RETENTION_FLG_Msk))) | u32Mask;
}

/**
  * @brief  This function request to enter deepsleep
  * @param[in]  ana: where anac is anac module base address
  * @return   none
  */
__STATIC_INLINE void LP_RequestDeepSleep(ANA_T *ana)
{
	ana->LP_FL_CTRL |= ANAC_FL_DEEPSLEEP_REQUEST_Msk;
}

/**
  * @brief  This function request to reset digital
  * @param[in]  ana: where anac is anac module base address
  * @param  NewState: new state of enabling state
  * @return   none
*/
__STATIC_INLINE void LP_ResetDigital(ANA_T *ana,FunctionalState NewState)
{
	(NewState)?(ana->LP_RST_CTRL |= ANAC_DIG_RST_EN_Msk):(ana->LP_RST_CTRL &= ~ANAC_DIG_RST_EN_Msk);
}

/**
  * @brief  This function enable gpio p56 wake up 
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  NewState: enable or disable
  * @param[in]  WkEdge: p56 wake up edge select,0-->low,1-->high 
  * @return   none
  */
void LP_SetExternalWake(ANA_T *ana,FunctionalState NewState,uint8_t WkEdge);

/**
  * @brief  This function set sleep time
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  u32ClkCnt: where u32ClkCnt is 32k clock cnt num
  * @return   none
  */
void LP_SetSleepTime(ANA_T *ana,uint32_t u32ClkCnt);

/**
  * @brief  This function used to set digital reset time
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  u8Clk32Cnt: where u8Clk32Cnt is 32k clock period cnt
  * @return   none
  */
void LP_SetDigitalDelay(ANA_T *ana,uint16_t u16Clk32Cnt);
	
/**
  * @brief  This function set sleep mode config
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  wkMode: where wkMode determine which wakeup mode to choose
  * @param[in]  enterCyclically: where enterCyclically determine
  *			  whether to cycle into low power	
  * @return   none
  */
void LP_SetSleepModeConfig(ANA_T *ana,uint32_t wkMode,bool enterCyclically);
/**
  * @brief  This function set deep sleep mode config
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  wkMode: where wkMode determine which wakeup mode to choose
  * @param[in]  enterCyclically: where enterCyclically determine
  *			  whether to cycle into low power
  * @return   none
  */
void LP_SetDeepSleepConfig(ANA_T *ana,uint32_t wkMode,bool enterCyclically);
/**
  * @brief  This function set standby mode config
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  wkMode: where wkMode determine which wake source selected
  * @param[in]  PowerCtrl: where PowerCtrl determine which module should be powered
  * @return   none
  */
void LP_SetStandbyConfig(ANA_T *ana,uint32_t wkMode, uint8_t PowerCtrl);

/**
  * @brief  This function set standby mode0 config
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  wkMode: where wkMode determine which wake source selected
  * @return   none
  */
void LP_SetStandbyMode0Config(ANA_T *ana,uint32_t wkMode);

/**
  * @brief  This function set standby mode1 config
  * @param[in]  ana: where ana is analog module base address
  * @param[in]  wkMode: where wkMode determine which wake source selected
  * @return   none
  */
void LP_SetStandbyMode1Config(ANA_T *ana,uint32_t wkMode);

/**@} */
#ifdef __cplusplus
}
#endif

#endif /* __PAN_LP_H__ */

