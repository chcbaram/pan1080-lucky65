/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Date: 16/02/22 9:39a $
 * @brief    Panchip series CLK driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
 
#include "PanSeries.h"
#include "pan_clk.h"

/** @addtogroup Panchip_Device_Driver Panchip Device Driver
  @{
*/

/** @addtogroup Panchip_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup Panchip_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


 /**
   * @brief  This function get CPU frequency. The frequency unit is Hz.
   * @return CPU frequency
   */
 uint32_t CLK_GetCPUFreq(void)
 {
     SystemCoreClockUpdate();
     return SystemCoreClock;
 }

/**
  * @brief  This function get APB1 frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t ahbclock,apb1_clock;
    uint32_t apb1_div;
    
    // update ahb clock
    SystemCoreClockUpdate();
    ahbclock =  SystemCoreClock;
    
    apb1_div = (CLK->CLK_TOP_CTRL & CLK_TOPCTL_APB1_DIV_Msk) >> CLK_TOPCTL_APB1_DIV_Pos;
    if(apb1_div)
			apb1_clock = ahbclock / (2 * apb1_div);
		else
			apb1_clock = ahbclock;
    
    return apb1_clock;
}
/**
  * @brief  This function get APB2 frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetPCLK2Freq(void)
{
    uint32_t ahbclock,apb2_clock;
    uint32_t apb2_div;
    
    // update ahb clock
    SystemCoreClockUpdate();
    ahbclock =  SystemCoreClock;
    
    apb2_div = (CLK->CLK_TOP_CTRL & CLK_TOPCTL_APB2_DIV_Msk) >> CLK_TOPCTL_APB2_DIV_Pos;
    if(apb2_div)
			apb2_clock = ahbclock / (2 * apb2_div);
		else
			apb2_clock = ahbclock;
    
    return apb2_clock;
}
 /**
    * @brief  This function set 16M ref clock source
    * @param[in]  u32ClkSrc is HCLK clock source. Including :
    *                  - \ref CLK_SYS_SRCSEL_RCH 
    *                  - \ref CLK_SYS_SRCSEL_XTH 
    *                  - \ref CLK_SYS_SRCSEL_RCL 
    *                  - \ref CLK_SYS_SRCSEL_XTL 
    *                  - \ref CLK_SYS_SRCSEL_DPLL
    * @return None
    */
void CLK_RefClkSrcConfig(uint32_t u32ClkSrc)
{
//    uint32_t tmpreg;
    CLK->CLK_TOP_CTRL &= ~CLK_TOPCTL_SYS_CLK_SEL_Msk;
    switch(u32ClkSrc)
    {
        case CLK_SYS_SRCSEL_RCH: 
            CLK->CLK_TOP_CTRL |= CLK_TOPCTL_RCH_EN_Msk;
#ifdef SYNC_3V_REG_MANUALLY
            CLK_Wait3vSyncReady();
#endif
			CLK_WaitClockReady(u32ClkSrc);
            CLK->CLK_TOP_CTRL |= CLK_SYS_SRCSEL_RCH;
            break;
        case CLK_SYS_SRCSEL_XTH: 
			CLK_XthStartupConfig();
            CLK->CLK_TOP_CTRL |= CLK_TOPCTL_XTH_EN_Msk;
#ifdef SYNC_3V_REG_MANUALLY
            CLK_Wait3vSyncReady();
#endif
			CLK_WaitClockReady(u32ClkSrc);
            CLK->CLK_TOP_CTRL |= CLK_SYS_SRCSEL_XTH;
            break;
//        case CLK_SYS_SRCSEL_RCL: 
//            CLK->CLK_TOP_CTRL |= (CLK_SYS_SRCSEL_RCL | CLK_TOPCTL_RCL_EN_Msk);
//            break;
//        case CLK_SYS_SRCSEL_XTL: 
//            CLK->CLK_TOP_CTRL |= (CLK_SYS_SRCSEL_XTL | CLK_TOPCTL_XTL_EN_Msk); 
//            break;
        case CLK_SYS_SRCSEL_DPLL: 
            CLK->CLK_TOP_CTRL |= CLK_TOPCTL_DPLL_EN_Msk;
#ifdef SYNC_3V_REG_MANUALLY
            CLK_Wait3vSyncReady();
#endif
			CLK_WaitClockReady(u32ClkSrc);
            CLK->CLK_TOP_CTRL |= CLK_SYS_SRCSEL_DPLL;
            break;
        default: break;
    }

    //wait xtal clock stable
//    CLK_WaitClockReady(u32ClkSrc);
	SystemCoreClockUpdate();
}
/**
* @brief  This function set HCLK clock source
* @param[in]  u32ClkSrc is HCLK clock source. Including :
*                  - \ref CLK_DPLL_REF_CLKSEL_RCH
*                  - \ref CLK_DPLL_REF_CLKSEL_XTH
* @return None
*/
void CLK_SYSCLKConfig(uint32_t u32ClkSrc,uint32_t freq_out)
{
    if(u32ClkSrc == CLK_DPLL_REF_CLKSEL_RCH){
        CLK->DPLL_CTRL &= ~CLK_DPLLCTL_RCLK_SEL_Msk;
    }
    else if(u32ClkSrc == CLK_DPLL_REF_CLKSEL_XTH){
        CLK->DPLL_CTRL |= CLK_DPLLCTL_RCLK_SEL_Msk;
    }
    
    /* Config MCU DPLL*/         
	CLK_SetDpllOutputFreq(freq_out);
    //enable dpll clock
    CLK->CLK_TOP_CTRL |=  CLK_TOPCTL_DPLL_EN_Msk;
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
    CLK_WaitClockReady(CLK_SYS_SRCSEL_DPLL);

    /* Update System Core Clock */
//    SystemCoreClockUpdate();
}



//void CLK_SetClkDivide(uint32_t u32ClkSel,uint32_t u32ClkDiv)
//{
//    switch(u32ClkSel){
//        case CLK_DivideSource_Apb1:
//            CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_APB1_DIV_Msk))| u32ClkDiv;
//            break;
//        case CLK_DivideSource_Apb2:
//            CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_APB2_DIV_Msk))| u32ClkDiv;
//            break;
//        default: break;
//    }
//}
/**
  * @brief  Enables or disables the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @param  CLK_AHBPeriph: specifies the AHB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CLK_AHBPeriph_DMAC   
  *            @arg CLK_AHBPeriph_GPIO   
  *            @arg CLK_AHBPeriph_SYSTICK
  *            @arg CLK_AHBPeriph_APB1   
  *            @arg CLK_AHBPeriph_APB2   
  *            @arg CLK_AHBPeriph_AHB    
  *            @arg CLK_AHBPeriph_RF 
  *            @arg CLK_AHBPeriph_All
  * @param  NewState    : new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_AHBPeriphClockCmd(uint32_t CLK_AHBPeriph, FunctionalState NewState)
{
    (NewState != DISABLE)?(CLK->AHB_CLK_CTRL |= CLK_AHBPeriph):(CLK->AHB_CLK_CTRL &= ~CLK_AHBPeriph);
}

/**
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @param  CLK_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CLK_APB1Periph_I2C0              
  *            @arg CLK_APB1Periph_SPI0       
  *            @arg CLK_APB1Periph_UART0     
  *            @arg CLK_APB1Periph_PWM01    
  *            @arg CLK_APB1Periph_PWM23      
  *            @arg CLK_APB1Periph_PWM45  
  *            @arg CLK_APB1Periph_PWM67   
  *            @arg CLK_APB1Periph_PWMEN    
  *            @arg CLK_APB1Periph_ADC    
  *            @arg CLK_APB1Periph_WDT   
  *            @arg CLK_APB1Periph_WWDT    
  *            @arg CLK_APB1Periph_TMR0    
  *            @arg CLK_APB1Periph_All
  * @param  NewState  : new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_APB1PeriphClockCmd(uint32_t CLK_APB1Periph, FunctionalState NewState)
{
  (NewState != DISABLE)?(CLK->APB1_CLK_CTRL |= CLK_APB1Periph):(CLK->APB1_CLK_CTRL &= ~CLK_APB1Periph);
}

/**
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  CLK_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CLK_APB2Periph_SPI1 
  *            @arg CLK_APB2Periph_UART1
  *            @arg CLK_APB2Periph_TMR1 
  *            @arg CLK_APB2Periph_TMR2 
  *            @arg CLK_APB2Periph_All  
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_APB2PeriphClockCmd(uint32_t CLK_APB2Periph, FunctionalState NewState)
{
    (NewState != DISABLE)?(CLK->APB2_CLK_CTRL |= CLK_APB2Periph):(CLK->APB2_CLK_CTRL &= ~CLK_APB2Periph); 
}

 /**
   * @brief  This function check selected clock source status
   * @param[in]  u32ClkMask is selected clock source. Including
   *                  - \ref CLK_RCL_SELECT
   *                  - \ref CLK_RCH_SELECT
   *                  - \ref CLK_XTL_SELECT
   *                  - \ref CLK_XTH_SELECT
   *                  - \ref CLK_DPLL_SELECT
   * @return   0  clock is not stable
   *           1  clock is stable
   *
   * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
   */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = 2160000;
    uint32_t u32ClkTmp = 0;
    while(1) 
    {
		switch(u32ClkMask){
				case CLK_SYS_SRCSEL_RCH :    u32ClkTmp = CLK->RCH_CTRL; break;
				case CLK_SYS_SRCSEL_XTH :    u32ClkTmp = CLK->XTH_CTRL; break;
//				case CLK_SYS_SRCSEL_RCL :    u32ClkTmp = CLK->RCL_CTRL; break;
//				case CLK_SYS_SRCSEL_XTL :    u32ClkTmp = CLK->XTL_CTRL; break;
				case CLK_SYS_SRCSEL_DPLL:    u32ClkTmp = CLK->DPLL_CTRL; break;
				default: break;
		}		
        if((u32ClkTmp & CLK_STABLE_STATUS_Msk) != CLK_STABLE_STATUS_Msk){
            if(i32TimeOutCnt-- <= 0)
             return 0;
        }else{
            break;
        }  
    }
    return 1;
}
 /**
   * @brief  This function wait sync 3v clock locale stable
   * @param[in]  none
   * @return   0  clock sync is not stable
   *           1  clock sync is stable
   *
   * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
   */
 uint32_t CLK_Wait3vSyncReady(void)
 {
     int32_t i32TimeOutCnt = 2160000;

    ANA->LP_REG_SYNC |= ANAC_LP_REG_SYNC_3V_Msk;
    while(ANA->LP_REG_SYNC & (ANAC_LP_REG_SYNC_3V_Msk | ANAC_LP_REG_SYNC_STS_Msk))
    {
         if(i32TimeOutCnt-- <= 0)
             return 0;
    }

     return 1;
 }
/**
* @brief  This function wait sync 3v clock locale stable by hardware
* @param[in]  none
* @return   none
*/
void CLK_Set3vSyncAuto(void)
{
    ANA->LP_REG_SYNC |= ANAC_LP_REG_AUTOSYNC_Msk;
}


/**
  * @brief This API is used to select wdt clock source
  * @param[in] u32clksel wdt clock selection. Could be 
  *             - \ref CLK_APB1_WDTSEL_MILLI_PULSE, \ref CLK_APB1_WDTSEL_RCL32K
  * @return none
  * @note This API is only used to select wdt clock source
  */
void CLK_SetWdtClkSrc(uint32_t u32clksel)
{
    CLK->APB1_CLK_CTRL = (CLK->APB1_CLK_CTRL & ~CLK_APB1CLK_WDTSRC_SEL_Msk) | u32clksel;
}

/**
  * @brief This API is used to select wwdt clock source
  * @param[in] wwdt The base address of wwdt module
  * @param[in] u32clksel wwdt clock selection. Could be 
  *             - \ref CLK_APB1_WDTSEL_MILLI_PULSE, \ref CLK_APB1_WDTSEL_RCL32K
  * @return none
  * @note This API is only used to select wwdt clock source
  */

void CLK_SetWwdtClkSrc(uint32_t u32clksel)
{
    CLK->APB1_CLK_CTRL = (CLK->APB1_CLK_CTRL & ~CLK_APB1CLK_WWDTSRC_SEL_Msk) | u32clksel;
}


/**
  * @brief This API is used to SELECT timer clock source
  * @param[in] timer The base address of Timer module
  * @param[in] u32clksel timer clock selection. Could be 
  *             - \ref CLK_APB_TMR0SEL_APB1CLK, \ref CLK_APB_TMR0SEL_LIRC,\ref CLK_APB_TMR0SEL_TM0
  *             - \ref CLK_APB_TMR1SEL_APB2CLK, \ref CLK_APB_TMR1SEL_LIRC,\ref CLK_APB_TMR0SEL_TM1
  *             - \ref CLK_APB_TMR2SEL_APB2CLK, \ref CLK_APB_TMR2SEL_LIRC,\ref CLK_APB_TMR0SEL_TM2
  * @return none
  * @note This API is only used to select timer clock source
  */

void CLK_SetTmrClkSrc(TIMER_T *timer, uint32_t u32clksel)
{
    if(timer == TIMER0) {        
        CLK->APB1_CLK_CTRL = (CLK->APB1_CLK_CTRL & ~CLK_APB1CLK_TMR0SRC_SEL_Msk) | u32clksel;
    }
    else if(timer == TIMER1){
        CLK->APB2_CLK_CTRL = (CLK->APB2_CLK_CTRL & ~CLK_APB2CLK_TMR1SRC_SEL_Msk) | u32clksel;
    }
    else if(timer == TIMER2){
        CLK->APB2_CLK_CTRL = (CLK->APB2_CLK_CTRL & ~CLK_APB2CLK_TMR2SRC_SEL_Msk) | u32clksel;
    }
}

uint32_t CLK_GetPeripheralFreq(void * Peripheral)
{
    uint8_t u8ApbDiv = 0;
	  uint32_t u32AhbClk,u32Pclk;

	/*get system clock frequency*/
    u32AhbClk = CLK_GetHCLKFreq(); 
    
    if((I2C0    == (I2C_T *)   Peripheral) 
     ||(UART0   == (UART_T *)  Peripheral)
     ||(SPI0    == (SPI_T *)   Peripheral)
     ||(TIMER0  == (TIMER_T *) Peripheral)
		 ||(I2S_M   == (I2S_T *)   Peripheral)
		 ||(I2S_S   == (I2S_T *)   Peripheral))
    {
        u8ApbDiv = (CLK->CLK_TOP_CTRL & CLK_TOPCTL_APB1_DIV_Msk) >> CLK_TOPCTL_APB1_DIV_Pos;
    }
    else if((SPI1    == (SPI_T *)    Peripheral) 
          ||(UART1   == (UART_T *)   Peripheral)
          ||(TIMER1  == (TIMER_T *)  Peripheral)
          ||(TIMER2  == (TIMER_T *)  Peripheral)
          ||(KSCAN == (KSCAN_T*) Peripheral)
          ||(QDEC    == (QDEC_T *)   Peripheral))
        u8ApbDiv = (CLK->CLK_TOP_CTRL & CLK_TOPCTL_APB2_DIV_Msk) >> CLK_TOPCTL_APB2_DIV_Pos;
    
    if(u8ApbDiv)
        u32Pclk = u32AhbClk / (2 * u8ApbDiv);
    else
        u32Pclk = u32AhbClk;
    return u32Pclk;
}

/*@}*/ /* end of group Panchip_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Panchip_CLK_Driver */

/*@}*/ /* end of group Panchip_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
