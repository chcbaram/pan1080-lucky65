/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series clktrim driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */
#ifndef		_PAN_CLKTRIM_H_
#define		_PAN_CLKTRIM_H_

/**
 * @brief Clktrim Interface
 * @defgroup clktrim_interface Acc Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif

/**
* @brief  This function used to set a ideal count value,IDEA_CNT = CAL_CNT * (REF_CLK / CAL_CLK) -1
* @param[in]  trim: where trim is trim module base address
* @param[in]  cnt: where cnt is count value
* @return   none
*/
__STATIC_INLINE void TRIM_SetIdealCnt(TRIM_T *trim,uint32_t cnt)
{
	trim->TRIM_IDEAL_CNT = cnt;
}

/**
* @brief  This function used to read a ideal count value,IDEA_CNT = CAL_CNT * (REF_CLK / CAL_CLK) -1
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetIdealCnt(TRIM_T *trim)
{
	return trim->TRIM_IDEAL_CNT;
}

/**
* @brief  This function used to get a real count value
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetRealCnt(TRIM_T *trim)
{
	return trim->TRIM_REAL_CNT;
}


/**
* @brief  Adjust which clktrim interrupt occured 
* @param  trim: The base address of i2s module
* @param  msk: interrupt status:
*               \ref TRIM_INT_MEASURE_STOP_Msk
*               \ref TRIM_INT_CTUNE_STOP_Msk
*               \ref TRIM_INT_FTUNE_STOP_Msk
*               \ref TRIM_INT_PTUNE_STOP_Msk
*				\ref TRIM_INT_OVERFLOW_Msk
*				\ref TRIM_INT_HW_TRIG_Msk
*               \ref TRIM_FLAG_MEASURE_STOP_Msk
*               \ref TRIM_FLAG_CTUNE_STOP_Msk
*               \ref TRIM_FLAG_FTUNE_STOP_Msk
*               \ref TRIM_FLAG_PTUNE_STOP_Msk
*				\ref TRIM_FLAG_OVERFLOW_Msk
*				\ref TRIM_FLAG_HW_TRIG_Msk
* @return true or false
*/
__STATIC_INLINE bool TRIM_IsIntStatusOccured(TRIM_T *trim,uint32_t msk)
{
	return (trim->TRIM_INT & msk)?(true):(false);
}

/**
* @brief  Clear the specified interrupt 
* @param  trim: The base address of i2s module
* @param  msk: interrupt status:
*               \ref TRIM_INT_MEASURE_STOP_Msk
*               \ref TRIM_INT_CTUNE_STOP_Msk
*               \ref TRIM_INT_FTUNE_STOP_Msk
*               \ref TRIM_INT_PTUNE_STOP_Msk
*				\ref TRIM_INT_OVERFLOW_Msk
*				\ref TRIM_INT_HW_TRIG_Msk
*               \ref TRIM_FLAG_MEASURE_STOP_Msk
*               \ref TRIM_FLAG_CTUNE_STOP_Msk
*               \ref TRIM_FLAG_FTUNE_STOP_Msk
*               \ref TRIM_FLAG_PTUNE_STOP_Msk
*				\ref TRIM_FLAG_OVERFLOW_Msk
*				\ref TRIM_FLAG_HW_TRIG_Msk
* @return true or false
*/
__STATIC_INLINE void TRIM_ClearIntStatusMsk(TRIM_T *trim,uint32_t msk)
{
	 trim->TRIM_INT |= msk;
}

/**
* @brief  This function used to enable interrupt
* @param[in]  trim: where trim is trim module base address
* @param[in]  NewState: new state of the clk trim.This parameter can be: ENABLE or DISABLE.
* @return   none
*/
__STATIC_INLINE void TRIM_EnableInt(TRIM_T *trim,FunctionalState NewState)
{
	(NewState != DISABLE)?(trim->TRIM_INT |= TRIM_INT_EN_Msk):(trim->TRIM_INT &= ~TRIM_INT_EN_Msk);
}

/**
* @brief  This function used to set the relation ship between tuning code and frequency
* @param[in]  trim: where trim is trim module base address
* @param[in]  NewState: new state of the clk trim.This parameter can be: ENABLE or DISABLE.
* @return   none
*/
__STATIC_INLINE void TRIM_SetRelationIncrease(TRIM_T *trim,FunctionalState NewState)
{
	(NewState != DISABLE)?(trim->TRIM_CTRL &= ~TRIM_CLRL_RELATION_Msk):(trim->TRIM_CTRL |= TRIM_CLRL_RELATION_Msk);
}

/**
* @brief  This function used to enable early termination
* @param[in]  trim: where trim is trim module base address
* @param[in]  NewState: new state of the clk trim.This parameter can be: ENABLE or DISABLE.
* @return   none
*/
__STATIC_INLINE void TRIM_SetEarlyTerminatinEn(TRIM_T *trim,FunctionalState NewState)
{
	(NewState != ENABLE)?(trim->TRIM_CTRL &= ~TRIM_CLRL_EARLY_TERM_EN_Msk):(trim->TRIM_CTRL |= TRIM_CLRL_EARLY_TERM_EN_Msk);
}



/**
* @brief  This function used to set coarse code value
* @param[in]  trim: where trim is trim module base address
* @param[in]  code: coarse tuning code
* @return   none
*/
__STATIC_INLINE void TRIM_SetCoarseCode(TRIM_T *trim,uint32_t code)
{
	trim->TRIM_CODE = (trim->TRIM_CODE & ~TRIM_COARSE_CODE_Msk) | (code << TRIM_COARSE_CODE_Pos);
}

/**
* @brief  This function used to get coarse code value
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetCoarseCode(TRIM_T *trim)
{
	return ((trim->TRIM_CODE & TRIM_COARSE_CODE_Msk) >> TRIM_COARSE_CODE_Pos);
}
/**
* @brief  This function used to set fine code value
* @param[in]  trim: where trim is trim module base address
* @param[in]  code: fine tuning code
* @return   none
*/
__STATIC_INLINE void TRIM_SetFineCode(TRIM_T *trim,uint32_t code)
{
	trim->TRIM_CODE = (trim->TRIM_CODE & ~TRIM_FINE_CODE_Msk) | (code << TRIM_FINE_CODE_Pos);
}
/**
* @brief  This function used to get fine code value
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetFineCode(TRIM_T *trim)
{
	return ((trim->TRIM_CODE & TRIM_FINE_CODE_Msk) >> TRIM_FINE_CODE_Pos);
}
/**
* @brief  This function used to set precision code value
* @param[in]  trim: where trim is trim module base address
* @param[in]  code: precision tuning code
* @return   none
*/
__STATIC_INLINE void TRIM_SetPrecisionCode(TRIM_T *trim,uint32_t code)
{
	trim->TRIM_CODE = (trim->TRIM_CODE & ~TRIM_PRECISION_CODE_Msk) | (code << TRIM_PRECISION_CODE_Pos);
}
/**
* @brief  This function used to get precision code value
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetPrecisionCode(TRIM_T *trim)
{
	return ((trim->TRIM_CODE & TRIM_PRECISION_CODE_Msk) >> TRIM_PRECISION_CODE_Pos);
}
/**
* @brief  This function used to set bit width,bit_width = width + 1
* @param[in]  trim: where trim is trim module base address
* @param[in]  width: bit width
* @return   none
*/
__STATIC_INLINE void TRIM_SetBitWidth(TRIM_T *trim,uint32_t width)
{
	trim->TRIM_CODE = (trim->TRIM_CODE & ~TRIM_BIT_WIDTH_Msk) | (width << TRIM_BIT_WIDTH_Pos);
}

/**
* @brief  This function used to get bit width.
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
__STATIC_INLINE uint32_t TRIM_GetBitWidth(TRIM_T *trim)
{
	return ((trim->TRIM_CODE & TRIM_BIT_WIDTH_Msk) >> TRIM_BIT_WIDTH_Pos);
}
/**
* @brief  This function used to enable clk trim function
* @param[in]  trim: where trim is trim module base address
* @param[in]  function: tuning function,include:
*						\ref TRIM_MEASURE_TUNING_EN_Msk
*						\ref TRIM_COARSE_TUNING_EN_Msk
*						\ref TRIM_FINE_TUNING_EN_Msk
*						\ref TRIM_PRECISON_TUNING_EN_Msk
* @return   none
*/
__STATIC_INLINE void TRIM_StartTuning(TRIM_T *trim,uint32_t function)
{
	trim->TRIM_EN = (trim->TRIM_EN & ~0XF) | (function);
}

/**
* @brief  This function used to set calibration cnt number to make sure rc32k stable
* @param[in]  trim: where trim is trim module base address
* @param[in]  cnt: where cnt is count value
* @return   none
*/
void TRIM_SetCalWaitCnt(TRIM_T *trim,uint32_t cnt);

/**
* @brief  This function decide the number of calibration clock cycles you want count
* @param[in]  trim: where trim is trim module base address
* @param[in]  cnt: where cnt is count value
* @return   none
*/
void TRIM_SetCalCnt(TRIM_T *trim,uint32_t cnt);

/**
* @brief  This function get the number of calibration clock cycles
* @param[in]  trim: where trim is trim module base address
* @return   none
*/
uint32_t TRIM_GetCalCnt(TRIM_T *trim);

/**
* @brief  This function used to set range value
* @param[in]  trim: where trim is trim module base address
* @param[in]  deviation: frequency deviation,units of Hz
*				formula: deviation = 32k * clk_cnt / (HCLK / 32K) - 32k
* @return   none
*/
void TRIM_SetErrRange(TRIM_T *trim,uint32_t deviation);


/**@} */


#ifdef __cplusplus
}
#endif

#endif /* _PAN_CLKTRIM_H_ */

