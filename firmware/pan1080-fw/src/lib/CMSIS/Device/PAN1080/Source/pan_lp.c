#include "PanSeries.h"
#include "pan_lp.h"


/**
* @brief  This function enable gpio p56 wake up 
* @param[in]  ana: where ana is analog module base address
* @param[in]  NewState: enable or disable
* @param[in]  WkEdge: p56 wake up edge select,0-->low,1-->high 
* @return   none
*/
void LP_SetExternalWake(ANA_T *ana,FunctionalState NewState,uint8_t WkEdge)
{
	if(ENABLE == NewState){
		ana->LP_FL_CTRL |= ANAC_FL_EXT_WAKEUP_EN_Msk;
		if(WkEdge){
			ana->LP_FL_CTRL |= ANAC_FL_EXT_WAKEUP_SEL_Msk;
		}else{
			ana->LP_FL_CTRL &= ~ANAC_FL_EXT_WAKEUP_SEL_Msk;
		}
	}else{
		ana->LP_FL_CTRL &= ~ANAC_FL_EXT_WAKEUP_EN_Msk;
	}
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
}

/**
* @brief  This function set sleep time
* @param[in]  ana: where ana is analog module base address
* @param[in]  u32ClkCnt: where u32ClkCnt is 32k clock cnt num
* @return   none
*/
void LP_SetSleepTime(ANA_T *ana,uint32_t u32ClkCnt)
{
	ana->LP_TMR_CTRL = u32ClkCnt;
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
}


/**
* @brief  This function used to set digital reset time
* @param[in]  ana: where ana is analog module base address
* @param[in]  u16Clk32Cnt: where u16Clk32Cnt is 32k clock period cnt
* @return   none
*/
void LP_SetDigitalDelay(ANA_T *ana,uint16_t u16Clk32Cnt)
{
	uint32_t tmp_reg;

	tmp_reg = ana->LP_RST_CTRL;
	tmp_reg &= ~(ANAC_DIG_DLY_TIME_Msk);
	tmp_reg |= (ANAC_CLK_MUX_RST_CTRL_Msk | (u16Clk32Cnt<<ANAC_DIG_DLY_TIME_Pos));
	ana->LP_RST_CTRL = tmp_reg;
}



 /**
* @brief  This function set sleep mode config
* @param[in]  ana: where ana is analog module base address
* @param[in]  wkMode: where wkMode determine which wakeup mode to choose
* @param[in]  enterCyclically: where enterCyclically determine
*			  whether to cycle into low power	
* @return   none
*/
void LP_SetSleepModeConfig(ANA_T *ana,uint32_t wkMode,bool enterCyclically)
{	
	LP_EnableInt(ana,ANAC_INT_SLEEP_INT_EN_Msk);
	ana->LP_FL_CTRL &= ~ANAC_FL_STANDBY_EN_Msk;
	if((LP_WKUP_MODE_SEL_GPIO_P56 != wkMode)&&(LP_WKUP_MODE_SEL_GPIO != wkMode)){
		ana->LP_FL_CTRL |= ANAC_FL_RC32K_EN_Msk;
		ana->LP_FL_CTRL |= ANAC_FL_SLEEP_CNT_EN_Msk;
	} else {
        ana->LP_FL_CTRL &= ~ANAC_FL_SLEEP_CNT_EN_Msk;
		ana->LP_FL_CTRL &= ~ANAC_FL_RC32K_EN_Msk;
    }
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	if(enterCyclically)
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	else
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
	__WFI();
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
}

void LP_SetDeepSleepConfig(ANA_T *ana,uint32_t wkMode,bool enterCyclically)
{
	LP_EnableInt(ana,ANAC_INT_SLEEP_INT_EN_Msk);
	ana->LP_FL_CTRL &= ~ANAC_FL_STANDBY_EN_Msk;
	if((LP_WKUP_MODE_SEL_GPIO_P56 != wkMode)&&(LP_WKUP_MODE_SEL_GPIO != wkMode)){
		ana->LP_FL_CTRL |= ANAC_FL_RC32K_EN_Msk;
		ana->LP_FL_CTRL |= ANAC_FL_SLEEP_CNT_EN_Msk;
	} else {
        ana->LP_FL_CTRL &= ~ANAC_FL_SLEEP_CNT_EN_Msk;
		ana->LP_FL_CTRL &= ~ANAC_FL_RC32K_EN_Msk;
    }
	ana->LP_FL_CTRL &= ~ANAC_FL_RCL_GATE_EN_Msk;
#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	if(enterCyclically)
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	else
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;	
	__WFI();
}

void LP_SetStandbyConfig(ANA_T *ana,uint32_t wkMode, uint8_t PowerCtrl)
{
	LP_EnableInt(ana,ANAC_INT_STANDBY_INT_EN_Msk);
	ana->LP_FL_CTRL |= ANAC_FL_STANDBY_EN_Msk;
	if(LP_WKUP_MODE_SEL_32K == wkMode){
		ana->LP_FL_CTRL |= ANAC_FL_RC32K_EN_Msk;
		ana->LP_FL_CTRL |= ANAC_FL_SLEEP_CNT_EN_Msk;
	} else {
		ana->LP_FL_CTRL &= ~ANAC_FL_SLEEP_CNT_EN_Msk;
		ana->LP_FL_CTRL &= ~ANAC_FL_RC32K_EN_Msk;
	}

	ana->LP_FL_CTRL &= ~ANAC_FL_RCL_GATE_EN_Msk;
	ana->LP_FL_CTRL = (PowerCtrl << 24u) | (ana->LP_FL_CTRL & 0x00FFFFFF);
	LP_ResetDigital(ANA,ENABLE);
	#ifdef SYNC_3V_REG_MANUALLY
	CLK_Wait3vSyncReady();
	#endif
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
}

void LP_SetStandbyMode0Config(ANA_T *ana,uint32_t wkMode)
{
    LP_SetStandbyConfig(ana, wkMode, 0xFF);
}

void LP_SetStandbyMode1Config(ANA_T *ana,uint32_t wkMode)
{
    // Retain all SRAM
    LP_SetStandbyConfig(ana, wkMode, 0x3F);
}
