/**************************************************************************//**
 * @file     acc.c
 * @version  V1.00
 * $Date: 20/12/14 9:39a $
 * @brief    Panchip acc driver source file
 *
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
 
#include "PanSeries.h"
#include "pan_acc.h"
/**
  * @brief  This function is used to start multiplier
  * @param  acc: where acc is a ACC peripheral base address
  * @param  left: data with calculations 
  * @param  right: data with calculations 
  * @param  num_words: The multiplier calculates the number of polls
  * @param  cycle: The multiplier reserves the computation period 
  * @retval none
  */

void ACC_StartMultCalc(ACC_T *acc,
                    const uint32_t *left,
                    const uint32_t *right,
                    uint8_t         num_words,
					uint8_t 		cycle)
{
    acc->ACC_R0 = 0;
    acc->ACC_R1 = 0;
    acc->ACC_R2 = 0;
    
    for (uint8_t i = 0; i < num_words; i++)
    {
        acc->ACC_MULT_LEFT[i]  = left[i];
        acc->ACC_MULT_RIGHT[i] = right[i];
    }
    
    acc->ACC_NUM_WORDS = num_words;
	ACC_SetMultCalcCycle(acc,cycle);
	ACC_EnableMultiplier(acc,ENABLE);
    acc->ACC_MULT_TRG = 1;
    while (acc->ACC_MULT_TRG);
}
/**
  * @brief  This function is used to get multiplier result
  * @param  acc: where acc is a ACC peripheral base address
  * @param  num_words: The multiplier calculates the number of polls
  * @param  result: data result array
  * @retval none
  */
void ACC_GetMultResult(ACC_T *acc,uint8_t num_words,uint32_t *result)
{
    for (int i = 0; i < num_words; i++)
    {
        result[i] = acc->ACC_MULT_RESULT[i];
        result[i+num_words] = acc->ACC_MULT_RESULT[16-num_words+i];
    }
    
    result[2*num_words-1] = acc->ACC_MULT_RESULT[15];	
}
/**
  * @brief  This function is used to start divider
  * @param  acc: where acc is a ACC peripheral base address
  * @param  divisor: divisor value
  * @param  dividend: dividend value
  * @param  cycle: The multiplier reserves the computation period 
  * @retval none
  */
uint32_t ACC_DivisionCalculate(ACC_T *acc,uint32_t divisor, uint32_t dividend,uint8_t cycle)
{
    acc->ACC_DIVISOR   = divisor;
    acc->ACC_DIVIDEND  = dividend;
	ACC_SetDivCalcCycle(acc,cycle);
	ACC_EnableDivider(acc,ENABLE);
    acc->ACC_DIV_TRG = 1;
    while (acc->ACC_DIV_TRG);
    return acc->ACC_DIV_RESULT;
}




/*** (C) COPYRIGHT 2020 Panchip Technology Corp. ***/
