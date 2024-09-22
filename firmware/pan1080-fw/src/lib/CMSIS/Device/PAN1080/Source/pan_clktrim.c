#include "PanSeries.h"
#include "pan_clk.h"
#include "pan_clktrim.h"

/**
* @brief  This function used to set calibration cnt number to make sure rc32k stable
* @param[in]  trim: where trim is trim module base address
* @param[in]  cnt: where cnt is count value
* @return   none
*/
void TRIM_SetCalWaitCnt(TRIM_T *trim,uint32_t cnt)
{
	if(cnt)
		trim->TRIM_CAL_CNT = (trim->TRIM_CAL_CNT & ~TRIM_CALC_WATI_CNT_Msk) | (cnt<<TRIM_CALC_WATI_CNT_Pos);
}

/**
* @brief  This function decide the number of calibration clock cycles you want count
* @param[in]  trim: where trim is trim module base address
* @param[in]  cnt: where cnt is count value
* @return   none
*/
void TRIM_SetCalCnt(TRIM_T *trim,uint32_t cnt)
{
	if(cnt)
		trim->TRIM_CAL_CNT = (trim->TRIM_CAL_CNT & ~TRIM_CALC_CNT_Msk) | (cnt<<TRIM_CALC_CNT_Pos);
}
 
uint32_t TRIM_GetCalCnt(TRIM_T *trim)
{
	return ((trim->TRIM_CAL_CNT & TRIM_CALC_CNT_Msk) >> TRIM_CALC_CNT_Pos);
}
/**
* @brief  This function used to set range value
* @param[in]  trim: where trim is trim module base address
* @param[in]  deviation: frequency deviation,units of Hz
*				   			HCLK/32k		 clk_cnt
*				formula:  -----------  =  ----------------	
*						     32k           32k+deviation
* @return   none
*/
void TRIM_SetErrRange(TRIM_T *trim,uint32_t deviation)
{
	uint32_t hclk,err_range;
	uint32_t calc_cnt,ideal_clk_cnt,calc_clk_cnt;
	
	hclk = FREQ_32MHZ;
	ideal_clk_cnt = hclk / 32000;
	calc_cnt = TRIM_GetCalCnt(trim);
	calc_clk_cnt = (32000+deviation)*ideal_clk_cnt/32;
	err_range = (calc_clk_cnt - ideal_clk_cnt*1000)*calc_cnt;
	err_range = (err_range / 1000) & 0xffff;
	trim->TRIM_CTRL = (trim->TRIM_CTRL & ~TRIM_CLRL_ERR_RANGE_Msk) | (err_range << TRIM_CLRL_ERR_RANGE_Pos);
}
