/**************************************************************************//**
 * @file     pan_kscan.c
 * @version  V1.00
 * $Date: 20/04/02 9:39a $
 * @brief    Panchip series key scan driver source file
 *
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#include "PanSeries.h"
#include "pan_kscan.h"

/**
  * @brief  This function is used to read key info
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  rowIdx: row number select (0~23) 
  * @retval row key infomation
  */
uint32_t KS_ReadKeyRowInfo(KSCAN_T *ks,uint8_t rowIdx)
{
    uint8_t regIdx = 0,bitIdx = 0;
    regIdx = rowIdx / 4;
    bitIdx = rowIdx % 4;
    
    if(rowIdx > 23)
        return 0xffffffff;
    
    uint32_t rowInfo = ((__I uint32_t*)&((ks)->KS_INFO0))[regIdx] & (0xff << (bitIdx << 3));
    return (rowInfo >> (bitIdx << 3));
}


/**
  * @brief  This function is used to read key info
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  rowIdx: row number select (0~17) 
  * @param  colIdx: column number select (0~7) 
  * @retval one key infomation
  */
bool KS_IsKeyPressed(KSCAN_T *ks,uint8_t rowIdx,uint8_t colIdx)
{
    uint8_t regIdx = 0,bitIdx = 0;
    regIdx = rowIdx / 4;
    bitIdx = rowIdx % 4;
    
    if((rowIdx > 23) || (colIdx > 7))
        return false;
    
    uint32_t rowInfo = ((__I uint32_t*)&((ks)->KS_INFO0))[regIdx] & (0xff << (bitIdx << 3));
    rowInfo = (rowInfo >> (bitIdx << 3));
    if(rowInfo & (0x1 << colIdx))
        return true;
    else
        return false;
}


/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
