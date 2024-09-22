/**************************************************************************//**
 * @file     pan_qdec.c
 * @version  V1.00
 * $Date: 20/04/16 18:39p 
 * @brief    PAN1080 series qdec driver source file
 *
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
 
#include "PanSeries.h"
#include "pan_qdec.h"

/**
  * @brief  This function used to set count resolution
  * @param[in] resolution: count resolution value,including:
  *                          QDEC_CNT_RESOLUTION_1X
  *                          QDEC_CNT_RESOLUTION_2X
  *                          QDEC_CNT_RESOLUTION_4X
  * @retval NONE
  */
void QDEC_SetCntResolution(uint8_t resolution)
{
    uint32_t tmpReg;
    if(resolution > QDEC_MAX_CNT_RESOLUTION)
        resolution = QDEC_CNT_RESOLUTION_1X;
    tmpReg = QDEC->QDEC_CTL;
    tmpReg &= ~QDEC_CNT_RESOLUTION_Msk;
    tmpReg |= ((resolution << QDEC_CNT_RESOLUTION_Pos) & QDEC_CNT_RESOLUTION_Msk);
    QDEC->QDEC_CTL = tmpReg;
}

/**
  * @brief  This function used to set filter threshold
  * @param[in] threshold: filter threshold value,including:
  *                      QDEC_FILTER_PERIOD_1
  *                      QDEC_FILTER_PERIOD_2
  *                      QDEC_FILTER_PERIOD_3
  *                      QDEC_FILTER_PERIOD_4
  *                      QDEC_FILTER_PERIOD_5
  *                      QDEC_FILTER_PERIOD_6
  *                      QDEC_FILTER_PERIOD_7
  *                      QDEC_FILTER_PERIOD_8
  * @retval NONE
  */
void QDEC_SetFilterThreshold(uint8_t threshold)
{
    uint32_t tmpReg;
    if(threshold > QDEC_MAX_FILTER_NUM)
        threshold = QDEC_FILTER_PERIOD_1;
    tmpReg = QDEC->QDEC_CTL;
    tmpReg &= ~QDEC_FILTER_THRESHOLD_Msk;
    tmpReg |= ((threshold << QDEC_FILTER_THRESHOLD_Pos) & QDEC_FILTER_THRESHOLD_Msk);
    QDEC->QDEC_CTL = tmpReg;     
}

/**
  * @brief  This function used to set event threshold
  * @param[in] threshold: event threshold value     
  * @retval NONE
  */
void QDEC_SetEventThreshold(uint16_t threshold)
{
    uint32_t tmpReg;
    tmpReg = QDEC->QDEC_CTL;
    tmpReg &= ~QDEC_EVENT_THRESHOLD_Msk;
    tmpReg |= ((threshold << QDEC_EVENT_THRESHOLD_Pos) & QDEC_EVENT_THRESHOLD_Msk);
    QDEC->QDEC_CTL = tmpReg;    
}

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
