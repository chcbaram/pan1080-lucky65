
/**************************************************************************//**
* @file     pan_hal_pwm.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series PWM (Pulse Width Modulation) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#include "pan_hal.h"

static PWM_AlignedTypeDef HAL_PWM_GetAlignedType(PWM_T *pwm, uint32_t u32ChannelMask){
    return  (PWM_AlignedTypeDef)(pwm->CTL2 & (PWM_CTL2_CNTTYPE_Msk<<PWM_CTL2_CNTTYPE_Pos));
}

static uint8_t HAL_PWM_GetPCA(PWM_T *pwm, uint32_t u32ChannelMask){
    return (pwm->PCACTL & PWM_PCACTL_PCAEN_Msk);
}

static PWM_OperateTypeDef HAL_PWM_GetOperateType(PWM_T *pwm, uint32_t u32ChannelMask){
    if(HAL_PWM_GetAlignedType(pwm,u32ChannelMask) == PWM_EDGE_ALIGNED)
    {
        return OPERATION_EDGE_ALIGNED;
    }
    else{
        if(HAL_PWM_GetPCA(pwm,u32ChannelMask) == 0x01){
            return OPERATION_PRECISE_CENTER_ALIGNED;
        }
        else{
            return OPERATION_CENTER_ALIGNED;
        }
    }
}

static uint32_t HAL_PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequency,
                                 uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t u8Divider = 1, u8Prescale = 0xFF;
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
    PWM_OperateTypeDef OperateType = HAL_PWM_GetOperateType(pwm,u32ChannelNum);
    u32Hclk = CLK_GetPCLK1Freq();
    for (; u8Divider < 17; u8Divider <<= 1)
    {   // clk divider could only be 1, 2, 4, 8, 16
        /*u32Frequency = SystemCoreClock /((u8Prescale+1)*(u8Divider))/( PERIODn+1) ==>
        (u8Prescale+1)*(PERIODn+1) = SystemCoreClock/u32Frequency/u8Divider					*/
        i = (u32Hclk / u32Frequency) / u8Divider;

        /* If target value is larger than (u16period+1)*(u8Prescale+1), need to use a larger divider*/
        if (i > (0x10000 * 0x100))
            continue;

        /* u16period = 0xFFFF + 1, get a prescaler that u16period value is below 0xFFFF
                 Must first determine a variable value, now we choose u16period*/
        if ((pwm->CLKPSC >> ((u32ChannelNum >> 1) * 8) & 0xff) > 0)
        {
            u8Prescale = (pwm->CLKPSC >> ((u32ChannelNum >> 1) * 8) & 0xff) + 1;
        }
        else
        {
            u8Prescale = (i + 0xFFFF) / 0x10000;
        }
        /*  u8Prescale must at least be 2, otherwise the output stop*/
        if (u8Prescale < 3)
            u8Prescale = 2;

        /*  (PERIODn+1) = i / u8Prescale */
        i /= u8Prescale;

        /*u16period must less than  0xFFFF*/
        if (i <= 0x10000)
        {
            if (i == 1)
            {
                u16period = 1; // Too fast, and PWM cannot generate expected frequency...
            }
            else
            {
                if (OperateType == OPERATION_EDGE_ALIGNED)
                    u16period = i;
                else if (OperateType == OPERATION_CENTER_ALIGNED)
                    u16period = i / 2;
                else if (OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
                    u16period = i;
            }
            break;
        }
    }
    /*  Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16period to the real value to fill into register*/
    i = u32Hclk / (u8Prescale * u8Divider * u16period);

    u8Prescale -= 1;
    if (OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
        u16period = u16period;
    else
        u16period -= 1;

    /* convert to real register value*/
    if (u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // /* every two channels share a prescaler*/
    // pwm->CLKPSC = (pwm->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    // pwm->CLKDIV = (pwm->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    // /*set cnt mode as auto reload*/
    // PWM_SetCntMode(pwm, u32ChannelNum, PWM_CNTMODE_AUTO_RELOAD);

    // /*set cnt type as edge align*/
    // PWM_SetAlignedType(pwm, u32ChannelNum, PWM_EDGE_ALIGNED);

    // if ((OperateType == OPERATION_CENTER_ALIGNED) || (OperateType == OPERATION_PRECISE_CENTER_ALIGNED))
    // {
    //     /*set cnt type as center align*/
    //     PWM_SetAlignedType(pwm, u32ChannelNum, PWM_CENTER_ALIGNED);
    //     if (OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
    //         PWM_EnablePCA(pwm); // Precise center-aligned type Enabled.
    //     else
    //         PWM_DisablePCA(pwm);
    // }

    /*update compare data register, Duty ratio = (CMPn+1)/(PERIODn+1).*/
    if (u32DutyCycle == 0)
    {
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    }
    else
    {
        if (OperateType == OPERATION_EDGE_ALIGNED)
        {
            /*set cnt type as edge align compare data*/
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u32DutyCycle * (u16period + 1) / 100 - 1;
        }
        else if (OperateType == OPERATION_CENTER_ALIGNED)
        {
            /*set cnt type as center align compare data*/
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u16period - (u32DutyCycle * (u16period + 1) / 100);
        }
        else if (OperateType == OPERATION_PRECISE_CENTER_ALIGNED)
        {
            /*set cnt type as precise center align compare data*/
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = (u16period - (u16period * u32DutyCycle / 100)) / 2 - 1;
        }
    }
    /*update Counter data register*/
    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum)*4))) = u16period;

    return (i);
}

static uint8_t findPositionOfOne(uint8_t value) {
    for (int i = 0; i < 8; i++) {
        if (value & (1 << i)) {
            return i; //
        }
    }

}



void HAL_PWM_Init(PWM_Ch_Id ChID,PWM_Init_Opt InitObj){
    uint8_t port = ChID >> 8;
    uint32_t ch_bit = (uint32_t)(ChID&(~0xf00));
    uint32_t ch_num = findPositionOfOne(ch_bit);
    PWM_T *PWMx = (PWM_T *)(PWM0_BASE + 0x00001000UL * port);

    PWM_ResetPrescaler(PWMx, ch_num);


    PWM_ConfigOutputChannel(PWMx, ch_num, InitObj.Frequency, InitObj.DutyCycle, InitObj.OperateType);
    if (InitObj.OperateType ==  OPERATION_CENTER_ALIGNED)
    {
        // Enable Center Loading Operation (support only in Center-Aligned Mode)
        PWMx->CTL |= 0x20; // CTL.HCUPDT = 1
    }
    // else if(InitObj.OperateType ==  OPERATION_EDGE_ALIGNED){
    //     // Set counting mode
    //     // Note: This API is not actually necessary here as the previous PWM_ConfigOutputChannel()
    //     //       sets the counting mode to Auto-Reload Mode internally.
    //     PWM_SetCntMode(TGT_PWM, TGT_PWM_CH, PWM_CNTMODE_AUTO_RELOAD);
    // }

// TODO: Optimize this section with direct register operations.
    if(InitObj.Inverter == true){
        PWM_EnableOutputInverter(PWMx, ch_bit);        
    }
    switch (InitObj.Mode)
    {
    case PWM_MODE_INDEPENDENT:
        PWM_EnableIndependentMode(PWMx);
        break;
    case PWM_MODE_COMPLEMENTARY:
        PWM_EnableComplementaryMode(PWMx);

        break;
    case PWM_MODE_SYNCHRONIZED:
        PWM_EnableSyncMode(PWMx);
        break;
    case PWM_MODE_GROUP:
        PWM_EnableGroupMode(PWMx);
        break;
    default:
        break;
    }
    if(InitObj.Mode == PWM_MODE_COMPLEMENTARY)
    {
        if(InitObj.DZDuration!=0)
        {
            PWM_EnableDeadZone(PWMx, ch_num, InitObj.DZDuration);
            SYS_TEST("DZDuration:%d",InitObj.DZDuration);
        }

    }
    else{
        PWM_DisableDeadZone(PWMx, ch_num);
    }
    // Enable output of TGT_PWM channel

//
}
void HAL_PWM_DeInit(PWM_Ch_Id ChID){
}



void HAL_PWM_Start(PWM_Ch_Id ChID){
    uint8_t port = ChID >> 8;
    uint32_t ch_bit = (uint32_t)(ChID&(~0xf00));
    uint32_t ch_num = findPositionOfOne(ch_bit);
    PWM_T *PWMx = (PWM_T *)(PWM0_BASE + 0x00001000UL * port);
    // // Enable output of TGT_PWM channel
    PWM_EnableOutput(PWMx, ch_bit);
    // Start
    PWM_Start(PWMx, ch_bit);
    
}

void HAL_PWM_Stop(PWM_Ch_Id ChID,bool force){
    uint8_t port = ChID >> 8;
    uint32_t ch_bit = (uint32_t)(ChID&(~0xf00));
    uint32_t ch_num = findPositionOfOne(ch_bit);
    PWM_T *PWMx = (PWM_T *)(PWM0_BASE + 0x00001000UL * port);
    if(force == true)
        PWM_ForceStop(PWMx, ch_bit);
    else{
        PWM_Stop(PWMx, ch_bit);
    }
    PWM_DisableDeadZone(PWMx, ch_num);
    PWM_DisableOutputInverter(PWMx, ch_num);
}

void HAL_PWM_SetFreqAndDuty(PWM_Ch_Id ChID,uint32_t Frequency, uint32_t DutyCycle){
    uint8_t port = ChID >> 8;
    uint32_t ch_bit = (uint32_t)(ChID&(~0xf00));
    uint32_t ch_num = findPositionOfOne(ch_bit);
    PWM_T *PWMx = (PWM_T *)(PWM0_BASE + 0x00001000UL * port);

    // PWM_OperateTypeDef OperateType = HAL_PWM_GetOperateType(PWMx,ch);
    // PWM_ConfigOutputChannel(PWMx,ch,Frequency,DutyCycle,OperateType);
    HAL_PWM_ConfigOutputChannel(PWMx,ch_num,Frequency,DutyCycle);
}
