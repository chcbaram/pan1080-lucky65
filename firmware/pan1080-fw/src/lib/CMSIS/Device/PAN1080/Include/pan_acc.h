/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series acc driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */

#ifndef __PAN_ACC_H__
#define __PAN_ACC_H__

/**
 * @brief Acc Interface
 * @defgroup acc_interface Acc Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif


/**
  * @brief  This function set acc divider calculate cycle cnt
  * @param  acc: where acc is a ACC peripheral base address
  * @param  cnt: cycle cnt value
  * @return None
  */
__STATIC_INLINE void ACC_SetDivCalcCycle(ACC_T *acc,uint32_t cnt)
{
    acc->ACC_MULT_DIV_SEL = ((acc->ACC_MULT_DIV_SEL & ~ACC_MULT_DIV_SEL_CALC_CYCLE_Msk) | (cnt << ACC_MULT_DIV_SEL_CALC_CYCLE_Pos));
}
/**
  * @brief  This function used to enable divider
  * @param  acc: where acc is a ACC peripheral base address
  * @param  NewState: new state of acc divider
  * @return None
  */
__STATIC_INLINE void ACC_EnableDivider(ACC_T *acc,FunctionalState NewState)
{
    (NewState != DISABLE)?(acc->ACC_MULT_DIV_SEL |= ACC_MULT_DIV_SEL_DIV_EN_Msk):(acc->ACC_MULT_DIV_SEL &= ~ACC_MULT_DIV_SEL_DIV_EN_Msk);
}

/**
  * @brief  This function set calculate cycle cnt 
  * @param  acc: where acc is a ACC peripheral base address
  * @param  cnt: cycle cnt value
  * @return None
  */
__STATIC_INLINE void ACC_SetMultCalcCycle(ACC_T *acc,uint32_t cnt)
{
    acc->ACC_LOGIC_SELECT = ((acc->ACC_LOGIC_SELECT & ~ACC_LOGIC_SEL_CALC_CYCLE_Msk) | (cnt << ACC_LOGIC_SEL_CALC_CYCLE_Pos));
}
/**
  * @brief  This function used to enable multiplying unit
  * @param  acc: where acc is a ACC peripheral base address
  * @param  NewState: new state of acc multiplier
  * @return None
  */
__STATIC_INLINE void ACC_EnableMultiplier(ACC_T *acc,FunctionalState NewState)
{
    (NewState != DISABLE)?(acc->ACC_LOGIC_SELECT |= ACC_LOGIC_SEL_MULT_EN_Msk):(acc->ACC_LOGIC_SELECT &= ~ACC_LOGIC_SEL_MULT_EN_Msk);
}

/**
  * @brief  This function used to enable acc interrupt
  * @param  acc: where acc is a ACC peripheral base address
  * @param  NewState: new state of acc interrupt
  * @return None
  */
__STATIC_INLINE void ACC_EnableInt(ACC_T *acc,FunctionalState NewState)
{
    (NewState != DISABLE)?(acc->ACC_INT |= ACC_INT_INT_EN_Msk):(acc->ACC_INT &= ~ACC_INT_INT_EN_Msk);
}

/**
  * @brief  This function used to check acc interrupt status
  * @param  acc: where acc is a ACC peripheral base address
  * @param  flag: interrupt status flag or polling status flag
  *							\ref ACC_INT_STATUS_FLAG_Msk	
  *							\ref ACC_INT_INT_FLAG_Msk
  * @return True or false
  */
__STATIC_INLINE bool ACC_StatusFlag(ACC_T *acc,uint32_t flag)
{
    return (ACC->ACC_INT & flag) ? (true) : (false);
}

/**
  * @brief  This function used to clear acc interrupt status
  * @param  acc: where acc is a ACC peripheral base address
  * @param  flag: interrupt status flag or polling status flag
  *							\ref ACC_INT_STATUS_FLAG_Msk	
  *							\ref ACC_INT_INT_FLAG_Msk
  * @return None
  */
__STATIC_INLINE void ACC_ClearStatus(ACC_T *acc,uint32_t flag)
{
    ACC->ACC_INT |= flag;
}

/**
  * @brief  This function is used to start multiplier
  * @param  acc: where acc is a ACC peripheral base address
  * @param  left: data with calculations 
  * @param  right: data with calculations 
  * @param  num_words: The multiplier calculates the number of polls
  * @param  cycle: The multiplier reserves the computation period 
  * @return None
  */
void ACC_StartMultCalc(ACC_T *acc,
                    const uint32_t *left,
                    const uint32_t *right,
                    uint8_t         num_words,
					uint8_t 		cycle);

/**
  * @brief  This function is used to get multiplier result
  * @param  acc: where acc is a ACC peripheral base address
  * @param  num_words: The multiplier calculates the number of polls
  * @param  result: data result array
  * @return None
  */
void ACC_GetMultResult(ACC_T *acc,uint8_t num_words,uint32_t *result);					

/**
  * @brief  This function is used to start divider
  * @param  acc: where acc is a ACC peripheral base address
  * @param  divisor: divisor value
  * @param  dividend: dividend value
  * @param  cycle: The multiplier reserves the computation period 
  * @return data result
  */
uint32_t ACC_DivisionCalculate(ACC_T *acc,uint32_t divisor, uint32_t dividend,uint8_t cycle);

/**@} */


#ifdef __cplusplus
}
#endif

#endif /* __PAN_ACC_H__ */

