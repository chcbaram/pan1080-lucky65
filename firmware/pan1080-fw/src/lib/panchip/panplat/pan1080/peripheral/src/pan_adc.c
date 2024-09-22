/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision:  2$
 * $Date: 16/02/25 15:53 $
 * @brief    Panchip series ADC driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_adc.h"

/** @addtogroup Panchip_Device_Driver Panchip Device Driver
  @{
*/

/** @addtogroup Panchip_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup Panchip_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief Init ADC config parameters
  * @param[in] adc Base address of ADC module
  * @return None
  */
bool ADC_Init(ADC_T *ADCx)
{
    uint32_t tmpreg;

    ADC->BV_CTL |= 0x1;

    if ((otp.m.chip_ver == 0x00) || (otp.m.chip_ver == 0xFF))
    {
        return false;
    }

    tmpreg = ADC->BV_CTL;
    tmpreg &= ~(0xFFu << 3);
    tmpreg |= otp.m.c_adcvbg << 3;
    ADC->BV_CTL = tmpreg;

    return true;
}


/**
  * @brief Convert adc code to voltage in uV
  * @param[in] adc Base address of ADC module
  * @param[in] adc_code Code sampled by ADC module
  * @return ADC sample voltage output in uV
  */
int32_t ADC_OutputVoltage(ADC_T *ADCx, uint32_t adc_code)
{
    int32_t voltage_uv;
    uint16_t k, b;

    if (ADCx->CTL2 & ADC_SEL_VREF_Msk)  // Ref src is Vbat
    {
        k = otp.m.adc_vdd_k;
        b = (otp.m.adc_vdd_b & BIT15) ? (~(otp.m.adc_vdd_b & 0x7FFFu) + 1) : otp.m.adc_vdd_b;
    }
    else    // Ref src is ADC VBG (1.2v)
    {
        k = otp.m.adc_vbg_k;
        b = (otp.m.adc_vbg_b & BIT15) ? (~(otp.m.adc_vbg_b & 0x7FFFu) + 1) : otp.m.adc_vbg_b;
    }

    voltage_uv = k * adc_code + ((otp.m.ft_version > 10) ? ((int16_t)b * 100) : (int16_t)b);

    return voltage_uv;
}

/**
  * @brief Convert adc code to voltage using piecewise function when ref src is  ADC-VBG
  * @param[in] adc Base address of ADC module
  * @param[in] adc_code Code sampled by ADC module
  * @return ADC sample voltage output in uV
  * @details The key points of this implementation is: when ref_src is ADC-VBG, piecewise
  *          function with several 2-points calibrated sub-functions are composed together.
  */
int32_t ADC_OutputVoltageRefVbgPiecewise(ADC_T *ADCx, uint32_t adc_code)
{
    int32_t voltage_uv, k, b;
    int32_t adc_data_code_head, adc_data_voltage_mv_head, adc_data_code_tail, adc_data_voltage_mv_tail;
    ADC_DATA_T *code_voltage_pairs = (ADC_DATA_T*)otp.m.adc_data_refvbg;
    size_t adc_data_cnt = sizeof(otp.m.adc_data_refvbg) / sizeof(ADC_DATA_T);
    
    (void)adc_data_voltage_mv_tail;

    for (size_t i = 0; i < adc_data_cnt - 1; i++) {
        adc_data_code_head = (code_voltage_pairs[i].m.c_adc_high_4 << 8) | code_voltage_pairs[i].m.c_adc_low_8;
        adc_data_code_tail = (code_voltage_pairs[i + 1].m.c_adc_high_4 << 8) | code_voltage_pairs[i + 1].m.c_adc_low_8;
        if ((adc_code < adc_data_code_tail) || (i == adc_data_cnt - 2)) {
            adc_data_voltage_mv_head = (code_voltage_pairs[i].m.v_adc_high_8 << 4) | code_voltage_pairs[i].m.v_adc_low_4;
            adc_data_voltage_mv_tail = (code_voltage_pairs[i + 1].m.v_adc_high_8 << 4) | code_voltage_pairs[i + 1].m.v_adc_low_4;
            break;
        }
    }

    /*
     * k * x1 + b = y1, k * x2 + b = y2
     * ==>
     * k = (y2 - y1) / (x2 - x1)
     * b = (y1 * x2 - y2 * x1) / (x2 - x1)
     */
    k = 1000 * (adc_data_voltage_mv_tail - adc_data_voltage_mv_head) / (adc_data_code_tail - adc_data_code_head);
    b = 1000 * (adc_data_voltage_mv_head * adc_data_code_tail - adc_data_voltage_mv_tail * adc_data_code_head) / (adc_data_code_tail - adc_data_code_head);

    voltage_uv = k * adc_code + b;

    return voltage_uv;
}

/**
  * @brief Convert adc code when ref src is  ADC-Vdd
  * @param[in] adc Base address of ADC module
  * @param[in] adc_ch adc channel num
  * @param[in] sample_cnt adc sample count
  * @param[in] drop_cnt the first drop_cnt data is discarded
  * @return ADC sample code
  */
int32_t ADC_ConvertRefVdd(ADC_T *ADCx, uint16_t adc_ch, uint16_t sample_cnt, uint8_t drop_cnt)
{
	int32_t avg_code = 0, cur_code;
	uint16_t cnt = sample_cnt + drop_cnt;

	/* Enable channel vbg */
	ADC_Open(ADC,adc_ch);
	ADC_SelInputRange(ADC,ADC_INPUTRANGE_HIGH);
	ADC_PowerOn(ADC);
	while(cnt--){
		ADC_StartConvert(ADC);
		while(!ADC_IsDataValid(ADC)){}
    if (cnt < sample_cnt) {
		  avg_code += ADC_GetConversionData(ADC);
    } else {
      cur_code = ADC_GetConversionData(ADC);
    }
	}

	ADC_PowerDown(ADC);

	return avg_code / sample_cnt;
}

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
						int32_t *linear_param_b)
{
	uint32_t tmpreg;
    int32_t voltage_vbg3f_uv, voltage_vbg0_uv;
	int32_t adc_code_vbg3f = 0, adc_code_vbg0 = 0;
	
	/* get stored voltage when vbg code set as 0 */
	voltage_vbg0_uv = otp.m.adc_data_vbgcode_00[2];
	voltage_vbg0_uv <<= 4;
	voltage_vbg0_uv |= ((otp.m.adc_data_vbgcode_00[1] & 0xf0) >> 4);
	tmpreg = ADC->BV_CTL;
	tmpreg &= ~(0xFFu << 3);
	tmpreg |= (0 << 3);
	ADC->BV_CTL = tmpreg;
	adc_code_vbg0 = ADC_ConvertRefVdd(ADCx, ADC_CHEN_CH8_VBG_Msk, sample_cnt, drop_cnt);

	/* get stored voltage when vbg code set as 0x3f */
	voltage_vbg3f_uv = otp.m.adc_data_vbgcode_3f[2];
	voltage_vbg3f_uv <<= 4;
	voltage_vbg3f_uv |= ((otp.m.adc_data_vbgcode_3f[1] & 0xf0) >> 4);
	tmpreg = ADC->BV_CTL;
	tmpreg &= ~(0xFFu << 3);
	tmpreg |= (0x3f << 3);
	ADC->BV_CTL = tmpreg;
	adc_code_vbg3f = ADC_ConvertRefVdd(ADCx, ADC_CHEN_CH8_VBG_Msk, sample_cnt, drop_cnt);
	/*
	 * k * x1 + b = y1, k * x2 + b = y2
	 * ==>
	 * k = (y2 - y1) / (x2 - x1)
	 * b = (y1 * x2 - y2 * x1) / (x2 - x1)
	 */

	*linear_param_k = 1000 * (voltage_vbg0_uv - voltage_vbg3f_uv) / (adc_code_vbg0 - adc_code_vbg3f);
  *linear_param_b = 1000 * (voltage_vbg3f_uv * adc_code_vbg0 - voltage_vbg0_uv * adc_code_vbg3f) / (adc_code_vbg0 - adc_code_vbg3f);

	tmpreg = ADC->BV_CTL;
	tmpreg &= ~(0xFFu << 3);
	tmpreg |= (otp.m.c_adcvbg << 3);
	ADC->BV_CTL = tmpreg;
}

/**
  * @brief Convert adc code to voltage
  * @param[in] adc Base address of ADC module
  * @param[in] linear_param_k linear parameter k
  * @param[in] linear_param_b linear parameter b
  * @return adc voltage
  */
int32_t ADC_OutputVoltageRefVdd(uint32_t adc_code, int32_t linear_param_k, int32_t linear_param_b)
{
	int32_t voltage_uv;

	voltage_uv = linear_param_k * adc_code + linear_param_b;

    return voltage_uv;
}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] adc Base address of ADC module
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                 - \ref ADC_TRIGGER_BY_EXT_PIN
  *                 - \ref ADC_TRIGGER_BY_PWM
  * @param[in] u32Param While ADC trigger by PWM, this parameter is used to set the delay between PWM
  *                     trigger and ADC conversion. Valid values are from 0 ~ 0xFF, and actual delay
  *                     time is (4 * u32Param * HCLK). While ADC trigger by external pin, this parameter
  *                     is used to set trigger condition. Valid values are:
  *                 - \ref ADC_FALLING_EDGE_TRIGGER
  *                 - \ref ADC_RISING_EDGE_TRIGGER
  * @return None
  */
void ADC_EnableHWTrigger(ADC_T *ADCx,uint32_t Source,uint32_t TrgCondition)
{
    ADCx->CTL &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_CTL_HWTRGEN_Msk);
    if(Source == ADC_TRIGGER_BY_EXT_PIN) {
        ADCx->CTL &= ~(ADC_CTL_HWTRGSEL_Msk | ADC_CTL_HWTRGCOND_Msk);
        ADCx->CTL |= (Source | TrgCondition);
    } else {
        ADCx->CTL |=  Source;
    }
    ADCx->CTL |=  ADC_CTL_HWTRGEN_Msk;
    return;
}

/**
  * @brief Disable hardware trigger ADC function.
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_DisableHWTrigger(ADC_T *ADCx)
{
    ADCx->CTL &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_CTL_HWTRGEN_Msk);
}

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_EnableInt(ADC_T *ADCx, uint32_t Mask)
{
    if(Mask & ADC_ADIF_INT)
        ADCx->CTL |= ADC_CTL_ADCIEN_Msk;
    if(Mask & ADC_CMP0_INT)
        ADCx->CMP0 |= ADC_CMP0_ADCMPIE_Msk;
    if(Mask & ADC_CMP1_INT)
        ADCx->CMP1 |= ADC_CMP1_ADCMPIE_Msk;

    return;
}

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_DisableInt(ADC_T *ADCx, uint32_t Mask)
{
    if(Mask & ADC_ADIF_INT)
        ADCx->CTL &= ~ADC_CTL_ADCIEN_Msk;
    if(Mask & ADC_CMP0_INT)
        ADCx->CMP0 &= ~ADC_CMP0_ADCMPIE_Msk;
    if(Mask & ADC_CMP1_INT)
        ADCx->CMP1 &= ~ADC_CMP1_ADCMPIE_Msk;

    return;
}

/**
  * @brief ADC PWM Sequential Mode Control.
  * @param[in] adc Base address of ADC module
  * @param[in] u32SeqTYPE   This parameter decides which type will be selected.
  *                     - \ref ADC_SEQMODE_TYPE_23SHUNT
  *                     - \ref ADC_SEQMODE_TYPE_1SHUNT
  * @param[in] u32ModeSel  This parameter decides which mode will be selected.
  *                     - \ref ADC_SEQMODE_MODESELECT_CH01
  *                     - \ref ADC_SEQMODE_MODESELECT_CH12
  *                     - \ref ADC_SEQMODE_MODESELECT_CH02
  *                     - \ref ADC_SEQMODE_MODESELECT_ONE
  * @return None
  */
void ADC_SeqModeEnable(ADC_T *ADCx, uint32_t SeqType, uint32_t ModeSel)
{
    // Enable ADC Sequential Mode
    ADCx->SEQCTL = ADCx->SEQCTL  | ADC_SEQCTL_SEQEN_Msk;
    // Select ADC Sequential Mode Type
    ADCx->SEQCTL = (ADCx->SEQCTL & ~(ADC_SEQCTL_SEQTYPE_Msk)) | (SeqType << ADC_SEQCTL_SEQTYPE_Pos);
    // Select ADC Sequential Mode Type
    ADCx->SEQCTL = (ADCx->SEQCTL & ~(ADC_SEQCTL_MODESEL_Msk)) | (ModeSel << ADC_SEQCTL_MODESEL_Pos);

    return;
}

/**
  * @brief ADC PWM Sequential Mode PWM Trigger Source and type.
  * @param[in] adc Base address of ADC module
  * @param[in] u32SeqModeTriSrc1  This parameter decides first PWM trigger source and type.
  *
  *
  * @return None
  */
void ADC_SeqModeTriggerSrc(ADC_T *ADCx, uint32_t SeqModeTriSrc)
{
    // Select PWM Trigger Source Selection for TRG1CTL or TRG2CTL
    if(ADCx->SEQCTL & ADC_SEQCTL_TRG_SEL_Msk)
        ADCx->SEQCTL = (ADCx->SEQCTL & ~(ADC_SEQCTL_TRG2CTL_Msk)) | (SeqModeTriSrc << ADC_SEQCTL_TRG2CTL_Pos);
    else
        ADCx->SEQCTL = (ADCx->SEQCTL & ~(ADC_SEQCTL_TRG1CTL_Msk)) | (SeqModeTriSrc << ADC_SEQCTL_TRG1CTL_Pos);
    return;
}
/**
  * @brief Configure the comparator 0 and enable it
  * @param[in] adc Base address of ADC module
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
  * \hideinitializer
  */ 
void ADC_CompareEnable(ADC_T *ADCx,
                       uint32_t ChNum,
                       uint32_t CmpCondition,
                       uint32_t CmpData,
                       uint32_t MatchCnt,
                       uint32_t CmpSelect)
{
    uint32_t TmpRegister;
    
    if(ADC_COMPARATOR_1 == CmpSelect)
        TmpRegister = ADC->CMP1;
    else
        TmpRegister = ADC->CMP0;
    
    TmpRegister |= (ChNum << ADC_CMP0_CMPCH_Pos); //select compare channel
    TmpRegister |= CmpCondition;    //select compare condition(less than or greater than )
    TmpRegister |= (CmpData << ADC_CMP0_CMPDAT_Pos);    //set expect compare data
    TmpRegister |= ((MatchCnt - 1) << ADC_CMP0_CMPMCNT_Pos); //set compare match count
    if(ADC_COMPARATOR_1 == CmpSelect){
        TmpRegister |= ADC_CMP1_ADCMPEN_Msk;    //comparator enable
        ADC->CMP1 = TmpRegister;
    }
    else{
        TmpRegister |= ADC_CMP0_ADCMPEN_Msk;    //comparator enable  
        ADC->CMP0 = TmpRegister;
    }        
}

/**
  * @brief set ADC PWM one channel Sequential Mode configuration.
  * @param[in] adc Base address of ADC module
  * @param[in] Trig  This parameter decides first PWM trigger source and type.
  * @param[in] Level  This parameter decides fifo threshold value.
  * @param[in] DmaEn  This parameter decides dma is used or not.
  * @param[in] HwClrEN  This parameter decides PWM trigger flag cleared by hardware or software.
  * @param[in] AdcCh  This parameter decides which adc channel selected.
  * @return None
  */
void ADC_SeqOneChModeConfig(ADC_T *ADCx, 
							uint32_t Trig,
							uint8_t Level,
							uint8_t DmaEn,
							uint8_t HwClrEN,
							uint8_t AdcCh)
{
    // ADC Hardware External trigger Enabled by PMW
    ADC_EnableHWTrigger(ADCx, ADC_TRIGGER_BY_PWM, Trig);
	ADC_SetFifoTrigLevel(ADCx,Level);
	if(DmaEn)
		ADC_DmaModeEnable(ADCx,ENABLE);
	else
		ADC_DmaModeEnable(ADCx,DISABLE);

	// ADC Sequential mode & type Enable
    ADC_SeqModeEnable(ADCx,ADC_SEQMODE_TYPE_23SHUNT,ADC_SEQMODE_MODESELECT_ONE);
	ADC_SeqModeOneChEn(ADCx,ENABLE);
	ADC_SeqModeChSelect(ADCx,AdcCh);
	if(HwClrEN)
			ADC_ClearByHw(ADCx,ENABLE);
	else
			ADC_ClearByHw(ADCx,DISABLE);		
}

/*@}*/ /* end of group Panchip_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Panchip_ADC_Driver */

/*@}*/ /* end of group Panchip_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
