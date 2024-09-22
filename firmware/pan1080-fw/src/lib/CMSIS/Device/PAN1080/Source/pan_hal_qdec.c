/**************************************************************************//**
* @file     pan_hal_qdec.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series Quadrature Decoder (QDEC) HAL source file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#include "pan_hal.h"

QDEC_HandleTypeDef QDEC_Handle_Array[1];

void HAL_QDEC_Init(QDEC_HandleTypeDef *qdec)
{
    QDEC_SetPolarity(qdec->InitObj.Polarity);
    QDEC_SetCntResolution(qdec->InitObj.Resolution);

    QDEC_SetEventThreshold(qdec->InitObj.EventThrs); // in debounce test

    QDEC_SetFilterThreshold(qdec->InitObj.FilterPeriod);

    QDEC_ClearEventCnt(qdec->InitObj.CH);

    QDEC_ClearIntFlag(QDEC_FUNC_ALL);
    QDEC_ClearRawIntFlag(QDEC_FUNC_ALL);

    uint16_t base_mode = QDEC_EN_Msk | QDEC_FILTER_EN_Msk | (QDEC_CHANNEL_X_EN_Msk << QDEC_OBJ.InitObj.CH);
    QDEC_Enable(ENABLE, qdec->InitObj.Mode | base_mode);
}

void HAL_QDEC_Init_INT(QDEC_HandleTypeDef *qdec)
{
    QDEC_ClearIntMsk(qdec->InterruptObj.Mode, ENABLE);
}



void QDEC_IRQHandler(void)
{
    int16_t tmpCnt = 0;
    if(QDEC_IsIntOccured(QDEC_INT_INVALID_Msk))
    {
        QDEC_ClearIntFlag(QDEC_INT_INVALID_Msk);
//        SYS_TEST("qdec invalid input \n");
        QDEC_Enable(ENABLE,QDEC_EN_Msk);
    }
    if(QDEC_IsIntOccured(QDEC_INT_CNT_OVERFLOW_Msk)){
        tmpCnt = QDEC_GetEventCnt(QDEC_OBJ.InitObj.CH);
        QDEC_OBJ.Callback(QDEC_CB_OVERFLOW,tmpCnt);
        QDEC_ClearIntFlag(QDEC_INT_CNT_OVERFLOW_Msk);

    }
    if(QDEC_IsIntOccured(QDEC_INT_CNT_UNDERFLOW_Msk)){
        tmpCnt = QDEC_GetEventCnt(QDEC_OBJ.InitObj.CH);
        QDEC_OBJ.Callback(QDEC_CB_UNDERFLOW,tmpCnt);

        QDEC_ClearIntFlag(QDEC_INT_CNT_UNDERFLOW_Msk);
    }
    if(QDEC_IsIntOccured(QDEC_INT_INDEX_Msk)){
        tmpCnt = QDEC_GetEventCnt(QDEC_OBJ.InitObj.CH);
        QDEC_OBJ.Callback(QDEC_CB_INDEX,tmpCnt);        
        QDEC_ClearIntFlag(QDEC_INT_INDEX_Msk);
    }
    if(QDEC_IsIntOccured(QDEC_INT_FIFO_UNDERFLOW_Msk))
    {
        QDEC_ClearIntFlag(QDEC_INT_FIFO_UNDERFLOW_Msk);
//        SYS_TEST("qdec fifo empty \n");
    }    
}
