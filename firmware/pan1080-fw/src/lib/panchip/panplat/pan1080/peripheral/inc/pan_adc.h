/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series adc driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
  
#ifndef __PAN_ADC_H__
#define __PAN_ADC_H__

/**
 * @brief Adc Interface
 * @defgroup adc_interface Adc Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif


#define ADC_INPUTRANGE_HIGH             (1UL)                       		/*!< ADC input range 0V~VDD */
#define ADC_INPUTRANGE_LOW              (0UL)                       		/*!< ADC input range 0V~1.2V */    

#define ADC_CH8_EXT                     (0UL)                       		/*!< Use external input pin as ADC channel 8 source */
#define ADC_CH8_BGP                     (ADC_CHEN_CH8SEL_Msk)       		/*!< Use internal band-gap voltage (VBG) as channel 8 source. */
#define ADC_CMP0_LESS_THAN              (0UL << ADC_CMP0_CMPCOND_Pos)	    /*!< ADC compare condition less than */
#define ADC_CMP1_LESS_THAN              (0UL << ADC_CMP1_CMPCOND_Pos)	    /*!< ADC compare condition less than */
#define ADC_CMP0_GREATER_OR_EQUAL_TO    (1ul << ADC_CMP0_CMPCOND_Pos)	    /*!< ADC compare condition greater or equal to */
#define ADC_CMP1_GREATER_OR_EQUAL_TO    (1ul << ADC_CMP1_CMPCOND_Pos)	    /*!< ADC compare condition greater or equal to */
#define ADC_TRIGGER_BY_EXT_PIN          (0UL << ADC_CTL_HWTRGSEL_Pos)       /*!< ADC trigger by STADC (P3.2) pin */
#define ADC_TRIGGER_BY_PWM              (ADC_CTL_HWTRGSEL_Msk)      		/*!< ADC trigger by PWM events */
#define ADC_FALLING_EDGE_TRIGGER        (0UL << ADC_CTL_HWTRGCOND_Pos)      /*!< External pin falling edge trigger ADC */
#define ADC_RISING_EDGE_TRIGGER         (ADC_CTL_HWTRGCOND_Msk)     		/*!< External pin rising edge trigger ADC */

/**@defgroup ADC_INT_FLAG Adc interrupt
 * @brief       Adc interrupt control 
 * @{ */
#define ADC_ADIF_INT                    (ADC_STATUS_ADIF_Msk)       		/*!< ADC convert complete interrupt */
#define ADC_CMP0_INT                    (ADC_STATUS_ADCMPIF0_Msk)    		/*!< ADC comparator 0 interrupt */
#define ADC_CMP1_INT                    (ADC_STATUS_ADCMPIF1_Msk)    		/*!< ADC comparator 1 interrupt */
#define ADC_FIFO_FULL_INT               (ADC_STATUS_INTFLG_FULL_Msk)		/*!< ADC fifo full interrupt */
#define ADC_FIFO_EMPTY_INT              (ADC_STATUS_INTFLG_EMPTY_Msk)		/*!< ADC fifo empty interrupt */
#define ADC_FIFO_OVER_INT               (ADC_STATUS_INTFLG_OVER_Msk)		/*!< ADC fifo overflow interrupt */
#define ADC_FIFO_HALF_INT               (ADC_STATUS_INTFLG_HALF_Msk)		/*!< ADC fifo half full interrupt */
/**@} */


/**@defgroup ADC_SEQMODE_FLAG Adc sequential mode
 * @brief       Adc sequential mode control 
 * @{ */
#define ADC_SEQMODE_TYPE_23SHUNT        (0UL)                       		/*!< ADC sequential mode 23-shunt type */
#define ADC_SEQMODE_TYPE_1SHUNT         (1UL)                       		/*!< ADC sequential mode 1-shunt type */
#define ADC_SEQMODE_MODESELECT_CH01     (0UL)                       		/*!< ADC channel 0 then channel 1 conversion */
#define ADC_SEQMODE_MODESELECT_CH12     (1UL)                       		/*!< ADC channel 1 then channel 2 conversion */
#define ADC_SEQMODE_MODESELECT_CH02     (2UL)                       		/*!< ADC channel 0 then channel 2 conversion */
#define ADC_SEQMODE_MODESELECT_ONE      (3UL)                       		/*!< ADC channel 0 then channel 2 conversion */
#define ADC_SEQMODE_PWM0_RISING		    (0UL)                       		/*!< ADC sequential mode PWM0 rising trigger ADC*/
#define ADC_SEQMODE_PWM0_CENTER		    (1UL)                       		/*!< ADC sequential mode PWM0 center trigger ADC*/
#define ADC_SEQMODE_PWM0_FALLING	    (2UL)                       		/*!< ADC sequential mode PWM0 falling trigger ADC*/
#define ADC_SEQMODE_PWM0_PERIOD 	    (3UL)                       		/*!< ADC sequential mode PWM0 period trigger ADC*/
#define ADC_SEQMODE_PWM2_RISING		    (4UL)                       		/*!< ADC sequential mode PWM2 rising trigger ADC*/
#define ADC_SEQMODE_PWM2_CENTER		    (5UL)                       		/*!< ADC sequential mode PWM2 center trigger ADC*/
#define ADC_SEQMODE_PWM2_FALLING	    (6UL)                       		/*!< ADC sequential mode PWM2 falling trigger ADC*/
#define ADC_SEQMODE_PWM2_PERIOD 	    (7UL)                       		/*!< ADC sequential mode PWM2 period trigger ADC*/
#define ADC_SEQMODE_PWM4_RISING		    (8UL)                       		/*!< ADC sequential mode PWM4 rising trigger ADC*/
#define ADC_SEQMODE_PWM4_CENTER		    (9UL)                       		/*!< ADC sequential mode PWM4 center trigger ADC*/
#define ADC_SEQMODE_PWM4_FALLING	    (10UL)                       		/*!< ADC sequential mode PWM4 falling trigger ADC*/
#define ADC_SEQMODE_PWM4_PERIOD 	    (11UL)                       		/*!< ADC sequential mode PWM4 period trigger ADC*/
#define ADC_SEQMODE_PWM6_RISING		    (12UL)                       		/*!< ADC sequential mode PWM4 rising trigger ADC*/
#define ADC_SEQMODE_PWM6_CENTER		    (13UL)                       		/*!< ADC sequential mode PWM4 center trigger ADC*/
#define ADC_SEQMODE_PWM6_FALLING	    (14UL)                       		/*!< ADC sequential mode PWM4 falling trigger ADC*/
#define ADC_SEQMODE_PWM6_PERIOD 	    (15UL)                       		/*!< ADC sequential mode PWM4 period trigger ADC*/
/**@} */

#define ADC_COMPARATOR_0                (0)	/*!< ADC comparator 0 selected */
#define ADC_COMPARATOR_1                (1)	/*!< ADC comparator 0 selected */

#define ADC_FIFO_TRIG_LEVEL_HALF        (0)	/*!< ADC half fifo threshold level setted  */
#define ADC_FIFO_TRIG_LEVEL_FULL        (1)	/*!< ADC full fifo threshold level setted  */

/**
  * @brief Get the latest ADC conversion data
  * @param[in] ADCx Base address of ADC module
  * @return  Latest ADC conversion data
  *
  * @code:
  *
  *	while(ADC_IsBusy(ADC)){}
	*	ADC_StartConvert(ADC);
	*	while(!ADC_IsDataValid(ADC)){}
	*	uint32_t AdcResult = ADC_GetConversionData(ADC);
  *
  * @endcode
  */
__STATIC_INLINE uint32_t ADC_GetConversionData(ADC_T *ADCx)
{
    return (ADCx->DAT & ADC_DAT_RESULT_Msk);
} 

/**
  * @brief Get raw status flag
  * @param[in] ADCx Base address of ADC module
  * @param[in] IntMask The combination of following status bits. Each bit corresponds to a status flag.
  *                     - \ref ADC_STATUS_ADCF_Msk      
  *                     - \ref ADC_STATUS_ADCMPF0_Msk   
  *                     - \ref ADC_STATUS_ADCMPF1_Msk   
  *                     - \ref ADC_STATUS_FLAG_FULL_Msk 
  *                     - \ref ADC_STATUS_FLAG_EMPTY_Msk
  *                     - \ref ADC_STATUS_FLAG_OVER_Msk 
  *                     - \ref ADC_STATUS_FLAG_HALF_Msk 
  * @return  True or false
  */
__STATIC_INLINE bool ADC_StatusFlag(ADC_T *ADCx,uint32_t IntMask)
{
    return (ADCx->STATUS & IntMask)?(true):(false);
}

/**
  * @brief Clear specified interrupt flag
  * @param[in] ADCx Base address of ADC module
  * @param[in] IntMask The combination of following status bits. Each bit corresponds to a status flag.
  *                     - \ref ADC_STATUS_ADCF_Msk      
  *                     - \ref ADC_STATUS_ADCMPF0_Msk   
  *                     - \ref ADC_STATUS_ADCMPF1_Msk   
  *                     - \ref ADC_STATUS_FLAG_FULL_Msk 
  *                     - \ref ADC_STATUS_FLAG_EMPTY_Msk
  *                     - \ref ADC_STATUS_FLAG_OVER_Msk 
  *                     - \ref ADC_STATUS_FLAG_HALF_Msk 
  * @return  None
  */
__STATIC_INLINE void ADC_ClearStatusFlag(ADC_T *ADCx,uint32_t IntMask)
{
    ADCx->STATUS |= IntMask;
}
/**
  * @brief Set interrupt mask,if masked,interrupt will not be happened
  * @param[in] ADCx Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt mask bits. Each bit corresponds to a interrupt mask flag.
  *                     - \ref ADC_STATUS_INTMSK_FULL_Msk  
  *                     - \ref ADC_STATUS_INTMSK_EMPTY_Msk 
  *                     - \ref ADC_STATUS_INTMSK_OVER_Msk  
  *                     - \ref ADC_STATUS_INTMSK_HALF_Msk  
  *                     - \ref ADC_STATUS_INTMSK_AD_Msk  	
  *                     - \ref ADC_STATUS_INTMSK_CMP0_Msk  
  *                     - \ref ADC_STATUS_INTMSK_CMP1_Msk 
  * @param[in] NewState: new state of adc interrupt mask
  * @return  none
  */
__STATIC_INLINE void ADC_IntMask(ADC_T *ADCx,uint32_t IntMask,FunctionalState NewState)
{
    (NewState == DISABLE)?(ADCx->STATUS |= IntMask):(ADCx->STATUS &= ~IntMask);
}

/**
  * @brief adjust the user-specified interrupt occured or not
  * @param[in] ADCx Base address of ADC module
  * @param[in] IntMask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT      
  *                     - \ref ADC_CMP0_INT      
  *                     - \ref ADC_CMP1_INT      
  *                     - \ref ADC_FIFO_FULL_INT 
  *                     - \ref ADC_FIFO_EMPTY_INT
  *                     - \ref ADC_FIFO_OVER_INT 
  *                     - \ref ADC_FIFO_HALF_INT 
  * @return  User specified interrupt flags
  */
__STATIC_INLINE bool ADC_IsIntOccured(ADC_T *ADCx,uint32_t IntMask)
{
    return (ADCx->STATUS & IntMask)?(true):(false);
}

/**
  * @brief This macro clear the selected interrupt status bits
  * @param[in] ADCx Base address of ADC module
  * @param[in] IntMask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT      
  *                     - \ref ADC_CMP0_INT      
  *                     - \ref ADC_CMP1_INT      
  *                     - \ref ADC_FIFO_FULL_INT 
  *                     - \ref ADC_FIFO_EMPTY_INT
  *                     - \ref ADC_FIFO_OVER_INT 
  *                     - \ref ADC_FIFO_HALF_INT 
  * @return  None
  */
__STATIC_INLINE void ADC_ClearIntFlag(ADC_T *ADCx,uint32_t IntMask)
{
    ADCx->STATUS = ADCx->STATUS | IntMask;
}                                                                       
/**
  * @brief Get the busy state of ADC
  * @param[in] ADCx Base address of ADC module
  * @return busy state of ADC
  * @retval 0 ADC is not busy
  * @retval 1 ADC is busy
  */
__STATIC_INLINE bool ADC_IsBusy(ADC_T *ADCx)
{
    return (ADCx->STATUS & ADC_STATUS_BUSY_Msk)?(true):(false);
}     
/**
  * @brief Check if the ADC conversion data is over written or not
  * @param[in] ADCx Base address of ADC module
  * @return Over run state of ADC data
  * @retval 0 ADC data is not overrun
  * @retval 1 ADC data is overrun
  */     
__STATIC_INLINE bool ADC_IsDataOverrun(ADC_T *ADCx)
{
    return (ADCx->STATUS & ADC_STATUS_OV_Msk)?(true):(false);
} 
/**
  * @brief Check if the ADC conversion data is valid or not
  * @param[in] ADCx Base address of ADC module
  * @return Valid state of ADC data
  * @retval 0 ADC data is not valid
  * @retval 1 ADC data us valid
  */  
__STATIC_INLINE bool ADC_IsDataValid(ADC_T *ADCx)
{
    return (ADCx->STATUS & ADC_STATUS_VALID_Msk)?(true):(false);
}
/**
  * @brief Power down ADC module
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_PowerDown(ADC_T *ADCx)
{
    ADCx->CTL = ADCx->CTL & ~ADC_CTL_ADCEN_Msk;
} 
/**
  * @brief Power on ADC module
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_PowerOn(ADC_T *ADCx)
{
    ADCx->CTL  = ADCx->CTL | ADC_CTL_ADCEN_Msk;
}
/**
  * @brief  ADC sequential mode Disabled
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_SequentialModeDisable(ADC_T *ADCx)
{
    ADCx->SEQCTL  = ADCx->SEQCTL & ~ADC_SEQCTL_SEQEN_Msk;
}
/**
  * @brief  TRG1CTL  select for 1-shunt sequential mode
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of adc sequence mode
  * @return None
  */
__STATIC_INLINE void ADC_Trigger2Select(ADC_T *ADCx,FunctionalState NewState)
{
    (NewState == ENABLE)?(ADCx->SEQCTL |= ADC_SEQCTL_TRG_SEL_Msk):(ADCx->SEQCTL &= ~ADC_SEQCTL_TRG_SEL_Msk);
}
																																	 
/**
  * @brief Disable comparator 0
  * @param[in] ADCx Base address of ADC module
  * @return None  
  */  
__STATIC_INLINE void ADC_DisableCompare0(ADC_T *ADCx)
{
    ADCx->CMP0  = 0;
}

/**
  * @brief Disable comparator 1
  * @param[in] ADCx Base address of ADC module
  * @return None  
  */                          
__STATIC_INLINE void ADC_DisableCompare1(ADC_T *ADCx)
{
    ADCx->CMP1  = 0;
}

/**
  * @brief Start the A/D conversion.
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_StartConvert(ADC_T *ADCx)
{
    ADCx->CTL  = ADCx->CTL | ADC_CTL_SWTRG_Msk;
}
/**
  * @brief Stop the A/D conversion.
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_StopConvert(ADC_T *ADCx)
{
    ADCx->CTL  &= ~ADC_CTL_SWTRG_Msk;
}

/**
  * @brief Enable the A/D test mode.
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_TestModeEnable(ADC_T *ADCx)
{
    ADCx->CTL2  |= ADC_CTL2_TESTMODE_Msk;
}
/**
  * @brief Disable the A/D test mode.
  * @param[in] ADCx Base address of ADC module
  * @return None
  */
__STATIC_INLINE void ADC_TestModeDisable(ADC_T *ADCx)
{
    ADCx->CTL2  &= ~ADC_CTL2_TESTMODE_Msk;
}
/**
  * @brief Enable the A/D dma mode.
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of clear mode in pwm sequence one channel mode
  * @return None
  */
__STATIC_INLINE void ADC_DmaModeEnable(ADC_T *ADCx,FunctionalState NewState)
{
    (NewState == ENABLE)?(ADCx->CTL2 |= ADC_CTL2_DMA_EN_Msk):(ADCx->CTL2 &= ~ADC_CTL2_DMA_EN_Msk);
}

/**
  * @brief Set the A/D clock division.
  * @param[in] ADCx Base address of ADC module
  * @param[in] Divider Adc clk divider  
  * @return None
  */
__STATIC_INLINE void ADC_SetClockDivider(ADC_T *ADCx,uint32_t Divider)
{
    ADCx->CTL2  = (ADCx->CTL2 & ~ADC_CTL2_CLKDIV_Msk) | (Divider << ADC_CTL2_CLKDIV_Pos);
}

/**
  * @brief This API configures ADC module to be ready for convert the input from selected channel
  * @param[in] ADCx Base address of ADC module
  * @param[in] ChMask Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1...
  * @return  None
  * @note Panchip series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not turn on ADC power nor does trigger ADC conversion
  */
__STATIC_INLINE void ADC_Open(ADC_T *ADCx,uint32_t ChMask)
{
    ADCx->CHEN  = (ADCx->CHEN & ~ADC_CHEN_ALL_Msk) | ChMask;
}
/**
  * @brief Select ADC range of input sample signal.
  * @param[in] ADCx Base address of ADC module
  * @param[in] EnableHigh	If EnableHigh is 1,adc input range is 0V~VDD;if u32EnableHigh is 0,adc input range is 0V~1.2V.
  * 0V~VDD & 0V~1.2V both is theoretical value,the real range is determined by bandgap voltage.
  * @return None
  */
__STATIC_INLINE void ADC_SelInputRange(ADC_T *ADCx,uint32_t EnableHigh)
{
    ADCx->CTL2 = (ADCx->CTL2 & ~ADC_SEL_VREF_Msk) | (EnableHigh << ADC_SEL_VREF_Pos);
}
/**
  * @brief Delay ADC start conversion time after PWM trigger
  * @param[in] ADCx Base address of ADC module
  * @param[in] Data for Delay time
  * @return None
  */
__STATIC_INLINE void ADC_TriggerDelay(ADC_T *ADCx,uint32_t Data)
{
	ADCx->TRGDLY = Data;
}
/**
  * @brief Set ADC sample time for designated channel.
  * @param[in] ADCx Base address of ADC module
  * @param[in] SampleTime ADC sample ADC time, valid values range is 0 ~ 0x3ff
  * @return None
  */
__STATIC_INLINE void ADC_SetExtraSampleTime(ADC_T *ADCx,uint32_t SampleTime)
{
    ADCx->EXTSMPT = (ADCx->EXTSMPT & ~ADC_EXTSMPT_EXTSMPT_Msk) | SampleTime;
}

/**
  * @brief adjust pwm sequence convert end or not in adc one channel
  * @param[in] ADCx Base address of ADC module
  * @return true of false
  * \hideinitializer
  */  
__STATIC_INLINE bool ADC_IsOneChConvertEnd(ADC_T *ADCx)
{
    return (ADCx->STATUS & ADC_STATUS_ONE_CH_FLAG_Msk)?(true):(false);
}

/**
  * @brief clear pwm sequence end flag
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of clear mode in pwm sequence one channel mode
  * @return None
  * \hideinitializer
  */  
__STATIC_INLINE void ADC_ClearByHw(ADC_T *ADCx,FunctionalState NewState)
{
	(NewState == ENABLE)?(ADCx->STATUS |= ADC_STATUS_ONE_CH_CLR_SEL_Msk):(ADCx->STATUS &= ~ADC_STATUS_ONE_CH_CLR_SEL_Msk);
}

/**
  * @brief enable left shift function,if enable,adc data {adc_output[11:0],4'b0 }
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of left shift function
  * @return None
  */  
__STATIC_INLINE void ADC_LeftShiftEn(ADC_T *ADCx,FunctionalState NewState)
{
	(NewState == ENABLE)?(ADCx->LS_CTL |= ADC_LS_CTL_LS_EN_Msk):(ADCx->LS_CTL &= ~ADC_LS_CTL_LS_EN_Msk);
}

/**
  * @brief get left shift data
  * @param[in] ADCx Base address of ADC module
  * @return data value
  */  
__STATIC_INLINE uint32_t ADC_GetLeftShiftData(ADC_T *ADCx)
{
	return ((ADCx->LS_CTL & ADC_LS_CTL_LS_DAT_Msk) >> ADC_LS_CTL_LS_DAT_Pos);
}
/**
  * @brief enable subtract bias function
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of subtract bias function
  * @return None
  */  
__STATIC_INLINE void ADC_SubtractBiasEn(ADC_T *ADCx,FunctionalState NewState)
{
	(NewState == ENABLE)?(ADCx->SUB_CTL |= ADC_SUB_CTL_BIAS_EN_Msk):(ADCx->SUB_CTL &= ~ADC_SUB_CTL_BIAS_EN_Msk);
}

/**
  * @brief set bias data
  * @param[in] ADCx Base address of ADC module
  * @param[in] BiasData data value
  * @return none
  */  
__STATIC_INLINE void ADC_SetBiasData(ADC_T *ADCx,uint32_t BiasData)
{
	 ADCx->SUB_CTL = (ADCx->SUB_CTL & ~ADC_LS_CTL_LS_DAT_Msk) | (BiasData << ADC_LS_CTL_LS_DAT_Pos);
}

/**
  * @brief get adc data after bias & left shift
  * @param[in] ADCx Base address of ADC module
  * @return data value
  */  
__STATIC_INLINE uint32_t ADC_GetLeftBiasData(ADC_T *ADCx)
{
	 return ADCx->DATA_SUB & ADC_DAT_SUB_LEFT_BIAS_Msk;
}

/**
  * @brief set adc fifo trig level
  * @param[in] ADCx Base address of ADC module
  * @param[in] Level dma request level
  *						\ref ADC_FIFO_TRIG_LEVEL_HALF
  *				    \ref ADC_FIFO_TRIG_LEVEL_FULL
  * @return none
  */  
__STATIC_INLINE void ADC_SetFifoTrigLevel(ADC_T *ADCx,uint8_t Level)
{
	 ADCx->FIFO_CTL = (ADCx->FIFO_CTL & ~ADC_FIFO_CTL_HALF_OR_FULL_Msk) | ((Level << ADC_FIFO_CTL_HALF_OR_FULL_Pos)|ADC_FIFO_CTL_EN_Msk);
}

/**
  * @brief get fifo pop data
  * @param[in] ADCx Base address of ADC module
  * @return data value
  */  
__STATIC_INLINE uint32_t ADC_GetFifoPopData(ADC_T *ADCx)
{
	 return ADCx->POP_DATA & ADC_FIFO_POP_DAT_Msk;
}

/**
  * @brief  pwm sequential enable in adc one channel mode
  * @param[in] ADCx Base address of ADC module
  * @param[in] NewState: new state of adc sequence mode
  * @return None
  */
__STATIC_INLINE void ADC_SeqModeOneChEn(ADC_T *ADCx,FunctionalState NewState)
{
    (NewState)?(ADCx->SEQCTL |= ADC_SEQCTL_ONE_CH_EN_Msk):(ADCx->SEQCTL &= ~ADC_SEQCTL_ONE_CH_EN_Msk);
}

/**
  * @brief  adc channel select in pwm sequential
  * @param[in] ADCx Base address of ADC module
  * @param[in] Ch: adc channel select
  * @return None
  */
__STATIC_INLINE void ADC_SeqModeChSelect(ADC_T *ADCx,uint8_t Ch)
{
    ADCx->SEQCTL = (ADCx->SEQCTL & ~ADC_SEQCTL_ONE_CH_SEL_Msk) | (Ch << ADC_SEQCTL_ONE_CH_SEL_Pos);
}

/**
  * @brief Init ADC config parameters
  * @param[in] adc Base address of ADC module
  * @return None
  */
bool ADC_Init(ADC_T *ADCx);

/**
  * @brief Convert adc code to voltage in uV
  * @param[in] adc Base address of ADC module
  * @param[in] adc_code Code sampled by ADC module
  * @return ADC sample voltage output in uV
  */

int32_t ADC_OutputVoltage(ADC_T *ADCx, uint32_t adc_code);

/**
  * @brief Convert adc code to voltage using piecewise function when ref src is  ADC-VBG
  * @param[in] adc Base address of ADC module
  * @param[in] adc_code Code sampled by ADC module
  * @return ADC sample voltage output in uV
  * @details The key points of this implementation is: when ref_src is ADC-VBG, piecewise
  *          function with several 2-points calibrated sub-functions are composed together.
  */
int32_t ADC_OutputVoltageRefVbgPiecewise(ADC_T *ADCx, uint32_t adc_code);

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] ADCx Base address of ADC module
  * @param[in] Source Decides the hardware trigger source. Valid values are:
  *                 - \ref ADC_TRIGGER_BY_EXT_PIN
  *                 - \ref ADC_TRIGGER_BY_PWM
  * @param[in] Param While ADC trigger by PWM, this parameter is used to set the delay between PWM
  *                     trigger and ADC conversion. Valid values are from 0 ~ 0xFF, and actual delay
  *                     time is (4 * u32Param * HCLK). While ADC trigger by external pin, this parameter
  *                     is used to set trigger condition. Valid values are:
  *                 - \ref ADC_FALLING_EDGE_TRIGGER
  *                 - \ref ADC_RISING_EDGE_TRIGGER
  * @return None
  */
void ADC_EnableHWTrigger(ADC_T *ADCx,
                         uint32_t Source,
                         uint32_t Param);
                         
/**
  * @brief Disable hardware trigger ADC function.
  * @param[in] ADCx Base address of ADC module
  * @return None
  */                         
void ADC_DisableHWTrigger(ADC_T *ADCx);

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] ADCx Base address of ADC module
  * @param[in] Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_EnableInt(ADC_T *ADCx, uint32_t Mask);

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] ADCx Base address of ADC module
  * @param[in] Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_DisableInt(ADC_T *ADCx, uint32_t Mask);
                   
/**
  * @brief ADC PWM Sequential Mode Control.
  * @param[in] ADCx Base address of ADC module
  * @param[in] SeqTYPE   This parameter decides which type will be selected.
  *                     - \ref ADC_SEQMODE_TYPE_23SHUNT
  *                     - \ref ADC_SEQMODE_TYPE_1SHUNT
  * @param[in] ModeSel  This parameter decides which mode will be selected.
  *                     - \ref ADC_SEQMODE_MODESELECT_CH01
  *                     - \ref ADC_SEQMODE_MODESELECT_CH12
  *                     - \ref ADC_SEQMODE_MODESELECT_CH02
  *                     - \ref ADC_SEQMODE_MODESELECT_ONE
  * @return None
  */
void ADC_SeqModeEnable(ADC_T *ADCx, uint32_t SeqTYPE, uint32_t ModeSel);

/**
  * @brief ADC PWM Sequential Mode PWM Trigger Source and type.
  * @param[in] ADCx Base address of ADC module
  * @param[in] SeqModeTriSrc  This parameter decides first PWM trigger source and type.
  *
  * @return None
  */
void ADC_SeqModeTriggerSrc(ADC_T *ADCx, uint32_t SeqModeTriSrc);

/**
  * @brief Configure the comparator 0 and enable it
  * @param[in] ADCx Base address of ADC module
  * @param[in] ChNum  Specifies the source channel, valid value are from 0 to 7
  * @param[in] CmpCondition Specifies the compare condition
  *                     - \ref ADC_CMP0_LESS_THAN
  *                     - \ref ADC_CMP0_GREATER_OR_EQUAL_TO
  * @param[in] CmpData Specifies the compare value. Valid value are between 0 ~ 0x3FF
  * @param[in] MatchCnt Specifies the match count setting, valid values are between 1~16
  * @param[in] CmpSelect comparator select,0 or 1
  * @return None
  * @details For example, ADC_CompareEnable(ADC, 5, ADC_CMP_GREATER_OR_EQUAL_TO, 0x800, 10,ADC_COMPARATOR_0);
  *          Means ADC will assert comparator 0 flag if channel 5 conversion result is 
  *          greater or equal to 0x800 for 10 times continuously.
  */ 
void ADC_CompareEnable(ADC_T *ADCx,
                       uint32_t ChNum,
                       uint32_t CmpCondition,
                       uint32_t CmpData,
                       uint32_t MatchCnt,
                       uint32_t CmpSelect);
                       
/**
  * @brief set ADC PWM one channel Sequential Mode configuration.
  * @param[in] ADCx Base address of ADC module
  * @param[in] Trig  This parameter decides first PWM trigger source and type.
  * @param[in] Level  This parameter decides fifo threshold value.
  * @param[in] DmaEn  This parameter decides dma is used or not.
  * @param[in] HwClrEN  This parameter decides PWM trigger flag cleared by hardware or software.
  * @param[in] AdcCh  This parameter decides which adc channel selected.
  * @return None
  *
  * @code:
  *
  * ADC_SeqOneChModeConfig(ADC,ADC_FALLING_EDGE_TRIGGER,ADC_FIFO_TRIG_LEVEL_HALF,1,1,5);
  *
  * @endcode
  */
void ADC_SeqOneChModeConfig(ADC_T *ADCx, 
							uint32_t Trig,
							uint8_t Level,
							uint8_t DmaEn,
							uint8_t HwClrEN,
							uint8_t AdcCh);
/**
  * @brief Convert adc code when ref src is  ADC-Vdd
  * @param[in] adc Base address of ADC module
  * @param[in] adc_ch adc channel num
  * @param[in] sample_cnt adc sample count
  * @param[in] drop_cnt the first drop_cnt data is discarded
  * @return ADC sample code
  */
int32_t ADC_ConvertRefVdd(ADC_T *ADCx, uint16_t adc_ch, uint16_t sample_cnt, uint8_t drop_cnt);
/**
  * @brief Calculate Linear Parameters according to saved VBG voltage value
  *         when ref src is  ADC-Vdd
  * @param[in] adc Base address of ADC module
  * @param[in] sample_cnt adc sample count
  * @param[in] drop_cnt the first drop_cnt data is discarded
  * @param[in] linear_param_k linear parameter k
  * @param[in] linear_param_b linear parameter b
  * @return none
  */
void ADC_CalculateCurLinearParamRefVdd(ADC_T *ADCx,
						uint16_t sample_cnt,
            uint8_t drop_cnt,
						int32_t *linear_param_k,
						int32_t *linear_param_b);
/**
  * @brief Convert adc code to voltage
  * @param[in] adc Base address of ADC module
  * @param[in] linear_param_k linear parameter k
  * @param[in] linear_param_b linear parameter b
  * @return adc voltage
  */
int32_t ADC_OutputVoltageRefVdd(uint32_t adc_code, int32_t linear_param_k, int32_t linear_param_b);
/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_ADC_H__ */
