/**
 * @file     pan_hal_adc.c
 * @version  V0.0.0
 * $Revision: 1 $
 * $Date:    23/09/10 $
 * @brief    Panchip series ADC (Analog-to-Digital Converter) HAL (Hardware Abstraction Layer) source file.
 * @note
 * Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#include "pan_hal.h"

ADC_HandleTypeDef ADC_Handle_Array[1] = {0};

void HAL_ADC_Init(ADC_Ch_Id ChID, ADC_Init_Opt InitObj)
{
    if (ChID == ADC_CH_VBG)
    {
        // en buffer(necessary)
        ADC->BV_CTL |= 0x2;
    }
    if (!ADC_Init(ADC))
    {
        SYS_WARN("Load ADC VBG init data failed, ADC_OutputVoltage() may work irregularly!\r\n");
    }
    ADC_SelInputRange(ADC, InitObj.InputRange);
    if (InitObj.SampleClock != 0)
    {
        ADC_SetExtraSampleTime(ADC, InitObj.SampleClock);
    }
    // Power on ADC
    ADC_PowerOn(ADC);
}
void HAL_ADC_DeInit()
{
    // TODO : if all channel off

    // Power on ADC
    ADC_PowerDown(ADC);
}

void HAL_ADC_Convert(ADC_Ch_Id ChID)
{
    uint32_t ch_bit = BIT(ChID);
    ADC_Open(ADC, ch_bit);
}

uint32_t HAL_ADC_GetValue(uint16_t Cnt)
{
    uint16_t tmp_cnt = 0;
    uint32_t tmp_sum = 0;
    uint32_t tmp_val;
    while (ADC_IsBusy(ADC))
    {
    }

    for (tmp_cnt = 0; tmp_cnt < Cnt; tmp_cnt++)
    {
        ADC_StartConvert(ADC);
        while (!ADC_IsDataValid(ADC))
            ;
        tmp_sum += ADC_GetConversionData(ADC);
    }
    return (uint32_t)(tmp_sum / tmp_cnt);
}

uint32_t HAL_ADC_Start_DMA(uint32_t *Buf, uint16_t Size, ADC_CallbackFunc Callback)
{
    ADC_OBJ.pRxBuffPtr = Buf;
    ADC_OBJ.RxXferSize = Size;

    DMAC_Init(DMA);
    NVIC_EnableIRQ(DMA_IRQn);

    /*get free dma channel;*/
    uint8_t gRxChNum = DMAC_AcquireChannel(DMA);
    /*enable dma transfer interrupt*/
    DMAC_ClrIntFlagMsk(DMA, gRxChNum, DMAC_FLAG_INDEX_TFR);
    // dma_uart2mem_config.PeripheralDst = uart->DmaSrc;
    DMAC_Channel_Array[gRxChNum].ConfigTmp = dma_adc2mem_config;
    DMAC_Channel_Array[gRxChNum].CallbackAdc = Callback;
    DMAC_Channel_Array[gRxChNum].PeriMode = DMAC_Peri_ADC;
    DMAC_Channel_Array[gRxChNum].pBuffPtr = ADC_OBJ.pRxBuffPtr;
    DMAC_Channel_Array[gRxChNum].XferSize = ADC_OBJ.RxXferSize;
    DMAC_SetChannelConfig(DMA, gRxChNum, &DMAC_Channel_Array[gRxChNum].ConfigTmp);
    /*start dma tx channel*/
    DMAC_StartChannel(DMA, gRxChNum, (void *)&(ADC->POP_DATA), ADC_OBJ.pRxBuffPtr, ADC_OBJ.RxXferSize);

    return gRxChNum;
}

void ADC_IRQHandler(void)
{
    if (ADC_IsIntOccured(ADC, ADC_ADIF_INT)) // Get ADC comparator interrupt flag
    {
        SYS_TEST("Convert end \r\n");
        ADC_ClearIntFlag(ADC, ADC_ADIF_INT);
    }
    else if (ADC_IsIntOccured(ADC, ADC_CMP0_INT)) // Get ADC comparator interrupt flag
    {
        SYS_TEST("Channel 1 input < 2000 \r\n");
        ADC_ClearIntFlag(ADC, ADC_CMP0_INT);
        // compare_finish_flag = true;
    }
    else if (ADC_IsIntOccured(ADC, ADC_CMP1_INT)) // Get ADC comparator interrupt flag
    {
        SYS_TEST("Channel 1 input >= 2000 \r\n");
        ADC_ClearIntFlag(ADC, ADC_CMP1_INT);
        // compare_finish_flag = true;
    }
}
