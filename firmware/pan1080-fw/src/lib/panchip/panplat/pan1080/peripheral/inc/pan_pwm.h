/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series pwm driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef __PAN_PWM_H__
#define __PAN_PWM_H__

/**
 * @brief Pwm Interface
 * @defgroup pwm_interface Pwm Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif


#define PWM_CHANNEL_NUM                     (8)   /*!< PWM channel number */

/**@defgroup PWM_CHANNEL_SEL_FLAG Pwm channel number 
 * @brief       Pwm channel number constants definitions
 * @{ */ 
#define PWM_CH0                 0x0         /*!< PWM channel 0 */
#define PWM_CH1                 0x1         /*!< PWM channel 1 */
#define PWM_CH2                 0x2         /*!< PWM channel 2 */
#define PWM_CH3                 0x3         /*!< PWM channel 3 */
#define PWM_CH4                 0x4         /*!< PWM channel 4 */
#define PWM_CH5                 0x5         /*!< PWM channel 5 */
#define PWM_CH6                 0x6         /*!< PWM channel 4 */
#define PWM_CH7                 0x7         /*!< PWM channel 5 */
/**@} */

#if 0
#define PWM_TRIGGER_ADC_CNTR_IS_0           PWM_ADCTCTL0_ZPTRGEN0_Msk   /*!< PWM trigger ADC while counter matches 0 */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_D       PWM_ADCTCTL0_CDTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CMR during down count */
#define PWM_TRIGGER_ADC_CNTR_IS_CNR         PWM_ADCTCTL0_CPTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CNR */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_U       PWM_ADCTCTL0_CUTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CMR during up count  */
#endif

/**@defgroup PWM_CLK_DIV_FLAG Pwm clk divider 
 * @brief       Pwm clk divider constants definitions
 * @{ */ 
typedef enum _PWM_ClkDivDef
{
    PWM_CLK_DIV_1   = 4, /*!< PWM clock divide by 1 */
    PWM_CLK_DIV_2   = 0, /*!< PWM clock divide by 2 */
    PWM_CLK_DIV_4   = 1, /*!< PWM clock divide by 4 */
    PWM_CLK_DIV_8   = 2, /*!< PWM clock divide by 8 */
    PWM_CLK_DIV_16  = 3, /*!< PWM clock divide by 16 */
    PWM_CLK_APB     = 5  /*!< PWM clock directly use APB */
} PWM_ClkDivDef;
/**@} */

/**@defgroup PWM_ALIGNED_FLAG Pwm aligned type
 * @brief       Pwm aligned type constants definitions
 * @{ */ 
typedef enum _PWM_AlignedTypeDef
{
    PWM_EDGE_ALIGNED    = 0, /*!< PWM working in edge aligned type */
    PWM_CENTER_ALIGNED  = 1  /*!< PWM working in center aligned type */
} PWM_AlignedTypeDef;
/**@} */

/**@defgroup PWM_PERIOD_INT_FLAG Pwm period interrupt condition
 * @brief       Pwm period interrupt trigger condition constants definitions
 * @{ */
typedef enum _PWM_IntPeriodTypeDef
{
    PWM_PERIOD_INT_UNDERFLOW    = 0, /*!< PWM period interrupt trigger if counter underflow */
    PWM_PERIOD_INT_MATCH_CNR    = 1  /*!< PWM period interrupt trigger if counter match CNR */
} PWM_IntPeriodTypeDef;
/**@} */

/**@defgroup PWM_OPERATE_FLAG Pwm operate type
 * @brief       Pwm operate type constants definitions
 * @{ */
typedef enum _PWM_OperateTypeDef
{
    OPERATION_EDGE_ALIGNED              = 0x01, /*!< PWM working in edge aligned type */
    OPERATION_CENTER_ALIGNED            = 0x02, /*!< PWM working in center aligned type */
    OPERATION_PRECISE_CENTER_ALIGNED    = 0x04  /*!< PWM working in precise center aligned type */
} PWM_OperateTypeDef;
/**@} */

/**@defgroup PWM_CNT_MODE_FLAG Pwm count mode
 * @brief       Pwm count mode constants definitions
 * @{ */
typedef enum _PWM_CntModeDef
{
    PWM_CNTMODE_ONE_SHOT    = 0x00, /*!< PWM working in one shot mode */
    PWM_CNTMODE_AUTO_RELOAD = 0x01  /*!< PWM working in auto reload mode */
} PWM_CntModeDef;
/**@} */

/**
 * @brief This function enables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 */
__STATIC_INLINE void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
    pwm->POEN |= u32ChannelMask;
}

/**
 * @brief This function disables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 */
__STATIC_INLINE void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
    pwm->POEN &= ~u32ChannelMask;
}

/**
 * @brief This function starts specified channel of PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_StartChannel(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->CTL |= (PWM_CTL_CNTEN0_Msk << (u32ChannelNum * 4));
}

/**
 * @brief This function forcedly stops specified channel of PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_StopChannel(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->CTL &= ~(PWM_CTL_CNTEN0_Msk << (u32ChannelNum * 4));
}

/**
 * @brief This function enable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Duration Dead Zone length in PWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None
 */
__STATIC_INLINE void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // set duration
    pwm->DTCTL = (pwm->DTCTL & ~(PWM_DTCTL_DTI01_Msk << (8 * u32ChannelNum))) | (u32Duration << (8 * u32ChannelNum));
    // enable dead zone
    pwm->CTL2 |= (PWM_CTL2_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function disable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // enable dead zone
    pwm->CTL2 &= ~(PWM_CTL2_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function enable Compare_Down interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_EnableCMPDInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Compare_Down interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisableCMPDInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Compare_Down interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_ClearCMPDIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTSTS = (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Compare_Down interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Compare_Down interrupt flag of specified channel
 * @retval 0 Compare_Down interrupt did not occurred
 * @retval 1 Compare_Down interrupt occurred
 */
__STATIC_INLINE uint32_t PWM_GetCMPDIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(pwm->INTSTS & (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_PIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_PIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTSTS = (PWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occurred
 * @retval 1 Period interrupt occurred
 */
__STATIC_INLINE uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(pwm->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum) ? 1 : 0);
}



/**
 * @brief This function enable Zero interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_EnableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Zero interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Zero interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_ClearZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTSTS = (PWM_INTSTS_ZIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Zero interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Zero interrupt flag of specified channel
 * @retval 0 Zero interrupt did not occurred
 * @retval 1 Zero interrupt occurred
 */
__STATIC_INLINE uint32_t PWM_GetZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(pwm->INTSTS & (PWM_INTSTS_ZIF0_Msk << u32ChannelNum) ? 1 : 0);
}
//------------------------------------------------------
/**
 * @brief This function enable Compare_Up interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_EnableCMPUInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Compare_Up interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisableCMPUInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Compare_Up interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_ClearCMPUIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTSTS = (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Compare_Up interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Compare_Up interrupt flag of specified channel
 * @retval 0 Compare_Up interrupt did not occurred
 * @retval 1 Compare_Up interrupt occurred
 */
__STATIC_INLINE uint32_t PWM_GetCMPUIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    return (pwm->INTSTS & (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] IntPeriodType Period interrupt type, could be either
 *              - \ref PWM_PERIOD_INT_UNDERFLOW
 *              - \ref PWM_PERIOD_INT_MATCH_CNR
 * @return None
 * @note All channels share the same Central interrupt type setting.
 */
__STATIC_INLINE void PWM_EnableCenterInt(PWM_T *pwm, uint32_t u32ChannelNum, PWM_IntPeriodTypeDef IntPeriodType)
{
    pwm->CTL2 &= ~PWM_CTL2_PINTTYPE_Msk;
    pwm->CTL2 |= (IntPeriodType << PWM_CTL2_PINTTYPE_Pos);
    pwm->INTEN = (pwm->INTEN & ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum)) | (PWM_INTEN_PIEN0_Msk << u32ChannelNum) ;   
}

/**
 * @brief This function disable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_DisableCenterInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTEN &= ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief This function clear Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
__STATIC_INLINE void PWM_ClearCenterIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->INTSTS = (PWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Central interrupt flag of specified channel
 * @retval 0 Central interrupt did not occurred
 * @retval 1 Central interrupt occurred
 */
__STATIC_INLINE uint32_t PWM_GetCenterIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(pwm->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable independent mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnableIndependentMode(PWM_T *pwm)
{
    pwm->CTL2 &= ~PWM_CTL2_MODE_Msk;
}
/**
 * @brief This function enable complementary mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnableComplementaryMode(PWM_T *pwm)
{
    pwm->CTL2 = (pwm->CTL2 & ~PWM_CTL2_MODE_Msk) | (1UL << PWM_CTL2_MODE_Pos);
}

/**
 * @brief This function disable complementary mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_DisableComplementaryMode(PWM_T *pwm)
{
    pwm->CTL2 &= ~PWM_CTL2_MODE_Msk;
}

/**
 * @brief This function enable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnableGroupMode(PWM_T *pwm)
{
    pwm->CTL2 |= PWM_CTL2_GROUPEN_Msk;
}

/**
 * @brief This function disable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_DisableGroupMode(PWM_T *pwm)
{
    pwm->CTL2 &= ~PWM_CTL2_GROUPEN_Msk;
}

/**
 * @brief This function enable synchronous mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnableSyncMode(PWM_T *pwm)
{
    pwm->CTL2 = (pwm->CTL2 & ~PWM_CTL2_MODE_Msk) | (2UL << PWM_CTL2_MODE_Pos);
}

/**
 * @brief This function disable synchronous mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_DisableSyncMode(PWM_T *pwm)
{
    pwm->CTL2 &= ~PWM_CTL2_MODE_Msk;
}

/**
 * @brief This function set the prescaler of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Prescaler Clock prescaler of specified channel. Valid values are between 1 ~ 0xFF
 * @return None
 * @note Every even channel N, and channel (N + 1) share a prescaler. So if channel 0 prescaler changed, 
 *       channel 1 will also be affected.
 */
__STATIC_INLINE void PWM_SetPrescaler(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Prescaler)
{
    pwm->CLKPSC = (pwm->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u32Prescaler << ((u32ChannelNum >> 1) * 8));
}

/**
 * @brief This function reset the prescaler of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 * @note Every even channel N, and channel (N + 1) share a prescaler. So if channel 0 prescaler reset, 
 *       channel 1 will also be affected.
 */
__STATIC_INLINE void PWM_ResetPrescaler(PWM_T *pwm, uint32_t u32ChannelNum)
{
    pwm->CLKPSC &= ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8));
}

/**
 * @brief This function set the divider of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] Divider Clock divider of specified channel. Valid values are
 *              - \ref PWM_CLK_DIV_1
 *              - \ref PWM_CLK_DIV_2
 *              - \ref PWM_CLK_DIV_4
 *              - \ref PWM_CLK_DIV_8
 *              - \ref PWM_CLK_DIV_16 
 * @return None
 */
__STATIC_INLINE void PWM_SetDivider(PWM_T *pwm, uint32_t u32ChannelNum, PWM_ClkDivDef Divider)
{
    pwm->CLKDIV = (pwm->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << ((u32ChannelNum) * 4))) | (Divider << (u32ChannelNum * 4));
}

/**
 * @brief This function set the duty of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7 
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 */
__STATIC_INLINE void PWM_SetCMR(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32CMR)
{
    (*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4)))= (u32CMR & 0x0000FFFF)|(*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) & 0xFFFF0000));
}
/**
 * @brief This function set the period of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7 
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 * @note PWM counter will stop if period length set to 0
 */
__STATIC_INLINE void PWM_SetCNR(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32CNR)
{
    (*((__IO uint32_t *) ((((uint32_t)&((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = (u32CNR));
}

/**
 * @brief This function set the duty of the selected channel for PWM asymmetric Mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7 
 * @param[in] u32CMRD Down counter in Asymmetric Mode. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 */
__STATIC_INLINE void PWM_SetCMRD(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32CMRD)
{
    (*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + (u32ChannelNum) * 4)))= (u32CMRD << 16)|(*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + (u32ChannelNum) * 4))) & 0x0000FFFF));
}

/**
 * @brief This function set the PWM aligned type
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] AlignedType PWM aligned type, valid values are:
 *                  - \ref PWM_EDGE_ALIGNED
 *                  - \ref PWM_CENTER_ALIGNED
 * @return None
 */
__STATIC_INLINE void PWM_SetAlignedType(PWM_T *pwm, uint32_t u32ChannelMask, PWM_AlignedTypeDef AlignedType)
{
    pwm->CTL2 = (pwm->CTL2 & ~PWM_CTL2_CNTTYPE_Msk) | (AlignedType << PWM_CTL2_CNTTYPE_Pos);
}

/**
 * @brief This function enables PWM asymmetric mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnableAsymmetricMode(PWM_T *pwm)
{
    pwm->CTL |= PWM_CTL_ASYMEN_Msk;
}

/**
 * @brief This function disables PWM asymmetric mode
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_DisableAsymmetricMode(PWM_T *pwm)
{
    pwm->CTL &= ~PWM_CTL_ASYMEN_Msk;
}

/**
 * @brief This function enables PWM Precise Center-Aligned Type
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_EnablePCA(PWM_T *pwm)
{
    pwm->PCACTL |= PWM_PCACTL_PCAEN_Msk;
}

/**
 * @brief This function disables PWM Precise Center-Aligned Type
 * @param[in] pwm The base address of PWM module
 * @return None
 */
__STATIC_INLINE void PWM_DisablePCA(PWM_T *pwm)
{
    pwm->PCACTL &= ~PWM_PCACTL_PCAEN_Msk;
}

/**
 * @brief This function set the CntMode of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] CntMode PWM cntmode select. Valid value 0:one shot,1:auto reload
 * @return None
 */
/* 
 * __STATIC_INLINE void PWM_SetCntMode(PWM_T *pwm, uint32_t u32ChannelNum, PWM_CntModeDef CntMode)
 * {
 *     pwm->CTL = (pwm->CTL & ~(PWM_CTL_CNTMODE0_Msk << (u32ChannelNum * 4))) | ((CntMode << PWM_CTL_CNTMODE0_Pos) << ((u32ChannelNum) * 4));
 * }
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
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum, 
                                  uint32_t u32Frequency, 
                                  uint32_t u32DutyCycle,
                                  PWM_OperateTypeDef OperateType);
void PWM_SetPeriodAndDuty(PWM_T *pwm, uint32_t u32ChannelNum, uint16_t u16PeriodCycle, uint16_t u16PulseCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);

/**
 * @brief This function stop PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);

/**
 * @brief This function stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);

/**
 * @brief This function enable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 */
void PWM_EnableOutputInverter(PWM_T *pwm, uint32_t u32ChannelMask);

/**
 * @brief This function disable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of channels to disable. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 */
void PWM_DisableOutputInverter(PWM_T *pwm, uint32_t u32ChannelMask);


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
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
/**
 * @brief This function disable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
/**
 * @brief This function clear selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be cleared.
 * @return None
 */
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
/**
 * @brief This function get selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be selected.
 * @return Get status of the selected channel trigger ADC
 */
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
#endif



/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_PWM_H__ */
