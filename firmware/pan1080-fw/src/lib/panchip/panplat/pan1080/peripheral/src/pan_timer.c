/**************************************************************************//**
 * @file     timer.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/24 15:37 $
 * @brief    Panchip series TIMER driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_timer.h"
#include "pan_clk.h"

/** @addtogroup Panchip_Device_Driver Panchip Device Driver
  @{
*/

/** @addtogroup Panchip_TIMER_Driver TIMER Driver
  @{
*/


/** @addtogroup Panchip_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This API is used to configure timer to operate in specified mode
  *        and frequency. If timer cannot work in target frequency, a closest
  *        frequency will be chose and returned.
  * @param[in] timer The base address of Timer module
  * @param[in] cntMode Operation mode. Possible options are
  *                 - \ref TIMER_ONESHOT_MODE
  *                 - \ref TIMER_PERIODIC_MODE
  *                 - \ref TIMER_TOGGLE_MODE
  *                 - \ref TIMER_CONTINUOUS_MODE
  * @param[in] u32Freq Timer Target working frequency
  * @return Real Timer working frequency
  * @note After calling this API, Timer is \b NOT running yet. But could start timer running be calling
  *       \ref TIMER_Start macro or program registers directly
  */
uint32_t TIMER_Open(TIMER_T *timer, TIMER_CntModeDef cntMode, uint32_t u32Freq)
{
	uint32_t u32Clk = CLK_GetPeripheralFreq(timer);
	uint32_t u32Prescale = 0;
	uint32_t u32RealFreq = u32Freq;

	if (u32RealFreq > u32Clk)
		u32RealFreq = u32Clk;
	else if (u32RealFreq == 0)
		u32RealFreq = 1;

	u32Prescale = u32Clk / u32RealFreq - 1;

	if (u32Prescale > 0xFF)
		u32Prescale = 0xFF; //Prescale is 8bit in reg

	timer->CTL = ((timer->CTL & ~TIMER_CTL_PSC_Msk) |u32Prescale);
	timer->CTL = (( timer->CTL& ~TIMER_CTL_OPMODE_Msk)|cntMode);

	// Calc real frequency and return
	u32RealFreq = u32Clk / (u32Prescale + 1);

	return u32RealFreq;
}

/**
  * @brief This API stops Timer counting and disable the Timer interrupt function
  * @param[in] timer The base address of Timer module
  * @return None
  */
void TIMER_Close(TIMER_T *timer)
{
	timer->CTL = 0;
	timer->EXTCTL = 0;

}

/**
  * @brief This API is used to create a delay loop for u32usec micro seconds
  * @param[in] timer The base address of Timer module
  * @param[in] u32Usec Delay period in micro seconds with 10 usec every step. Valid values are between 10~1000000 (10 micro second ~ 1 second)
  * @return None
  * @note This API overwrites the register setting of the timer used to count the delay time.
  * @note This API use polling mode. So there is no need to enable interrupt for the timer module used to generate delay
  */
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec)
{
	uint32_t u32Clk = CLK_GetPeripheralFreq(timer);
	uint32_t u32Prescale = 0, delay = SystemCoreClock / u32Clk;
	float fCmpr;

	// Clear current timer configuration
	timer->CTL = 0;
	timer->EXTCTL = 0;

	if(u32Clk == 10000) {         // min delay is 100us if timer clock source is LIRC 10k
		u32Usec = ((u32Usec + 99) / 100) * 100;
	} else {    // 10 usec every step
		u32Usec = ((u32Usec + 9) / 10) * 10;
	}

	if(u32Clk >= 0x2000000) {
		u32Prescale = 3;    // real prescaler value is 4
		u32Clk >>= 2;
	} else if(u32Clk >= 0x1000000) {
		u32Prescale = 1;    // real prescaler value is 2
		u32Clk >>= 1;
	}



	// u32Usec * u32Clk might overflow if using uint32_t
	fCmpr = (float)u32Usec * u32Clk / 1000000.0f;

	timer->CMP = (uint32_t)fCmpr;
	timer->CTL = TIMER_CTL_CNTEN_Msk | u32Prescale; // one shot mode

	// When system clock is faster than timer clock, it is possible timer active bit cannot set in time while we check it.
	// And the while loop below return immediately, so put a tiny delay here allowing timer start counting and raise active flag.
	for(; delay > 0; delay--) {
		__NOP();
	}

	while(timer->CTL & TIMER_CTL_ACTSTS_Msk);
}

/**
  * @brief This function is used to set new Timer compared value
  * @param[in] timer The base address of Timer module
  * @param[in] u32Value  Timer compare value. Valid values are between 2 to 0xFFFFFF
  * @return None
  */
void TIMER_SetCmpValue(TIMER_T *timer, uint32_t u32Value)
{
	uint32_t cnt_per_apb_clks = 0, clk_src = 0;
	uint8_t psc_store_value = timer->CTL & TIMER_CTL_PSC_Msk;
	uint8_t compare_compensate = 0, remainder = FIXED_DEVIATION;

	/*count freq = apbclock / (psc + 1), so 1 hw timer count = (psc + 1) apb clks */
	cnt_per_apb_clks = psc_store_value + 1;

	if (timer == TIMER0) {
		clk_src = CLK->APB1_CLK_CTRL & CLK_APB1CLK_TMR0SRC_SEL_Msk;
	} else if(timer == TIMER1){
		clk_src = CLK->APB2_CLK_CTRL & CLK_APB2CLK_TMR1SRC_SEL_Msk;
	} else if(timer == TIMER2){
		clk_src = CLK->APB2_CLK_CTRL & CLK_APB2CLK_TMR2SRC_SEL_Msk;
	}

	/*timer clk source is apb clk */
	if (clk_src == 0) {
		if (FIXED_DEVIATION >= cnt_per_apb_clks) {
			compare_compensate = FIXED_DEVIATION / cnt_per_apb_clks;
			remainder = FIXED_DEVIATION - compare_compensate * cnt_per_apb_clks;
		}

		if (remainder * 2 >= cnt_per_apb_clks) {
			compare_compensate += 1;
		}
	}

	/*here has 12 apb clk for hw timer sync operation*/
	timer->CMP = u32Value - compare_compensate;
}


/*@}*/ /* end of group Panchip_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Panchip_TIMER_Driver */

/*@}*/ /* end of group Panchip_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
