/**************************************************************************//**
 * @file     PWM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 19/11/18 19:41 $
 * @brief    Panchip series PWM driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#include "PanSeries.h"
#include "pan_pwm.h"
#include "pan_clk.h"

/** @addtogroup Panchip_Device_Driver Panchip Device Driver
  @{
*/

/** @addtogroup Panchip_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup Panchip_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief This function config PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @param[in] OperateType  Target operation type.Valid value Edge-aligned,Center-Aligned,Precise Center-Aligned 
 * @return Nearest frequency clock in nano second
 * @note Since every two channels, (0 & 1), (2 & 3), (4 & 5),  (6 & 7) shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel (PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle,
																	PWM_OperateTypeDef OperateType)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16period = 0xFFFF;
    uint32_t u32Hclk = SystemCoreClock;
//    uint8_t u8HclkDiv;	
	
//    /*APB1 CLK = hclk\devider\2*/
//    u8HclkDiv = (CLK->CLK_DIV_2 >> CLK_DIV_2_APB1_DIV_Pos)&0xFF;
//    if(u8HclkDiv==0)
//        u8HclkDiv = 1;
//    else
//        u8HclkDiv *= 2;		//2n scale
//		
//    u32Hclk = SystemCoreClock/u8HclkDiv;
    u32Hclk = CLK_GetPCLK1Freq();
    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
				/*u32Frequency = SystemCoreClock /((u8Prescale+1)*(u8Divider))/( PERIODn+1) ==>
				(u8Prescale+1)*(PERIODn+1) = SystemCoreClock/u32Frequency/u8Divider					*/
        i = (u32Hclk / u32Frequency) / u8Divider;
			
        /* If target value is larger than (u16period+1)*(u8Prescale+1), need to use a larger divider*/
        if(i > (0x10000 * 0x100))
            continue;

        /* u16period = 0xFFFF + 1, get a prescaler that u16period value is below 0xFFFF
				 Must first determine a variable value, now we choose u16period*/
				if((pwm->CLKPSC >>((u32ChannelNum >> 1) * 8) & 0xff) > 0){
					u8Prescale = (pwm->CLKPSC >>((u32ChannelNum >> 1) * 8) & 0xff)+1;
				}
				else{
					u8Prescale = (i + 0xFFFF)/ 0x10000;
				}
        /*  u8Prescale must at least be 2, otherwise the output stop*/
        if(u8Prescale < 3)
            u8Prescale = 2;
				
				/*  (PERIODn+1) = i / u8Prescale */
        i /= u8Prescale;

				/*u16period must less than  0xFFFF*/
        if(i <= 0x10000) {
            if(i == 1){
                u16period = 1;     // Too fast, and PWM cannot generate expected frequency...
						}
            else{
								if(OperateType == OPERATION_EDGE_ALIGNED)
									u16period = i;
								else if(OperateType == OPERATION_CENTER_ALIGNED)
									u16period = i/2;
								else if(OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
									u16period = i;
						}
            break;
        }

    }
    /*  Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16period to the real value to fill into register*/
    i = u32Hclk / (u8Prescale * u8Divider * u16period);

    u8Prescale -= 1;
		if(OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
			u16period = u16period;
		else
			u16period -= 1;
    
    /* convert to real register value*/
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    /* every two channels share a prescaler*/
    pwm->CLKPSC = (pwm->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    pwm->CLKDIV = (pwm->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
		/*set cnt mode as auto reload*/
		/* PWM_SetCntMode(pwm,u32ChannelNum,PWM_CNTMODE_AUTO_RELOAD); */

		/*set cnt type as edge align*/
		PWM_SetAlignedType(pwm, u32ChannelNum, PWM_EDGE_ALIGNED);
	
		if((OperateType == OPERATION_CENTER_ALIGNED)||(OperateType == OPERATION_PRECISE_CENTER_ALIGNED)){
			/*set cnt type as center align*/
			PWM_SetAlignedType(pwm, u32ChannelNum, PWM_CENTER_ALIGNED);
			if(OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
				PWM_EnablePCA(pwm); //Precise center-aligned type Enabled.
            else
                PWM_DisablePCA(pwm);
		}
			
		/*update compare data register, Duty ratio = (CMPn+1)/(PERIODn+1).*/
    if(u32DutyCycle == 0){
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    }else{
			if(OperateType == OPERATION_EDGE_ALIGNED){
				/*set cnt type as edge align compare data*/
				*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u32DutyCycle * (u16period + 1) / 100 - 1;
			}
			else if(OperateType == OPERATION_CENTER_ALIGNED){
				/*set cnt type as center align compare data*/
				*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u16period-(u32DutyCycle * (u16period + 1) / 100) ;	
			}
			else if(OperateType == OPERATION_PRECISE_CENTER_ALIGNED){
				/*set cnt type as precise center align compare data*/
				*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = (u16period-(u16period*u32DutyCycle/100))/2-1 ;
			}
		}
		/*update Counter data register*/
    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = u16period;

    return(i);
}

/**
 * @brief This function configures PWM generator to set period/duty cycles and output level mode.
 * @param[in] pwm               The base address of PWM module
 * @param[in] u32ChannelNum     PWM channel number. Valid values are between 0~7
 * @param[in] u16PeriodCycle    Target period cycles to set
 * @param[in] u16PulseCycle     Target pulse (high level) cycles to set
 * @note IF u16PeriodCycle = 0 OR u16PulseCycle = 0, THEN the output is always low;
 *       ELSE IF u16PeriodCycle < u16PulseCycle, THEN the output is always high;
 *       ELSE the final output PWM waveform meets the following formulas:
 *          Actual period cycles        = u16PeriodCycle + 1
 *          Actual high-level cycles    = u16PulseCycle + 1
 *          Actual low-level cycles     = u16PeriodCycle - u16PulseCycle
 *          Actual duty cycle           = (u16PulseCycle + 1) / (u16PeriodCycle + 1)
 * @return None
 */
void PWM_SetPeriodAndDuty(PWM_T *pwm, uint32_t u32ChannelNum, uint16_t u16PeriodCycle, uint16_t u16PulseCycle)
{
    PWM_SetCNR(pwm, u32ChannelNum, u16PeriodCycle);
    PWM_SetCMR(pwm, u32ChannelNum, u16PulseCycle);
}

/**
 * @brief This function start PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }
    pwm->CTL |= u32Mask;
}

/**
 * @brief This function stop PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (i) * 4))) = 0;
        }
    }
}

/**
 * @brief This function stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }
    pwm->CTL &= ~u32Mask;
}

/**
 * @brief This function enable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * \hideinitializer
 */
void PWM_EnableOutputInverter(PWM_T *pwm, uint32_t u32ChannelMask)
{
    //pwm->CTL &= ~(PWM_CTL_PINV0_Msk|PWM_CTL_PINV1_Msk|PWM_CTL_PINV2_Msk|PWM_CTL_PINV3_Msk|PWM_CTL_PINV4_Msk|PWM_CTL_PINV5_Msk|PWM_CTL_PINV6_Msk|PWM_CTL_PINV7_Msk);
    uint32_t tmpCtlReg = pwm->CTL;
    for(size_t i = 0; i < 8; i++)
    {
        if((u32ChannelMask) & (1 << i))
            tmpCtlReg |= (1 << (PWM_CTL_PINV0_Pos + (i * 4)));
    }
    pwm->CTL = tmpCtlReg;
}

/**
 * @brief This function disable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of channels to disable. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * \hideinitializer
 */
void PWM_DisableOutputInverter(PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t tmpCtlReg = pwm->CTL;
    for(size_t i = 0; i < 8; i++)
    {
        if((u32ChannelMask) & (1 << i))
            tmpCtlReg &= ~(1 << (PWM_CTL_PINV0_Pos + (i * 4)));
    }
    pwm->CTL = tmpCtlReg;
}

#if 0
/**
 * @brief This function enable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition The condition to trigger ADC. Combination of following conditions:
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_0
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_D
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CNR
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_U
 * @return None
 */
void PWM_EnableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        pwm->ADCTCTL0 = (pwm->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum))) | (u32Condition << (8 * u32ChannelNum));
    } else {
        pwm->ADCTCTL1 = (pwm->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4)))) | (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function disable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum)
{
    if(u32ChannelNum < 4) {
        pwm->ADCTCTL0 = (pwm->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum)));
    } else {
        pwm->ADCTCTL1 = (pwm->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4))));
    }
}

/**
 * @brief This function clear selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be cleared.
 * @return None
 */
void PWM_ClearADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        pwm->ADCTSTS0 |= (u32Condition << (8 * u32ChannelNum));
    } else {
        pwm->ADCTSTS1 |= (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function get selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be selected.
 * @return Get status of the selected channel trigger ADC
 */
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        return(pwm->ADCTSTS0 & (u32Condition << (8 * u32ChannelNum)) ? 1 : 0);
    } else {
        return(pwm->ADCTSTS1 & (u32Condition << (8 * (u32ChannelNum - 4))) ? 1 : 0);
    }
}
#endif


/*@}*/ /* end of group Panchip_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Panchip_PWM_Driver */

/*@}*/ /* end of group Panchip_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
