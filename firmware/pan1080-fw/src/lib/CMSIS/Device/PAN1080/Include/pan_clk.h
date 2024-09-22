/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series clk driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */

#ifndef __PAN_CLK_H__
#define __PAN_CLK_H__

/**
 * @brief Clk Interface
 * @defgroup clk_interface Clk Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

#define CLK_APB1_WDTSEL_MILLI_PULSE             (0x00000000UL)  /*!< Wdt clk source select apb_clk / 2048 */
#define CLK_APB1_WDTSEL_RCL32K                  (0x00010000UL)  /*!< Wdt clk source select rcl 32k */

#define CLK_APB1_WWDTSEL_MILLI_PULSE            (0x00000000UL)  /*!< Wwdt clk source select apb_clk / 2048 */
#define CLK_APB1_WWDTSEL_RCL32K                 (0x00020000UL)  /*!< Wwdt clk source select rcl 32k */

#define CLK_APB1_TMR0SEL_APB1CLK                (0x00000000UL)  /*!< Timer0 clk source select apb_clk */
#define CLK_APB1_TMR0SEL_RCL32K                 (0x00040000UL)  /*!< Timer0 clk source select rcl 32k */
#define CLK_APB1_TMR0SEL_TM0                    (0x00080000UL)  /*!< Timer0 clk source select externel input */

#define CLK_APB2_TMR1SEL_APB2CLK                (0x00000000UL)  /*!< Timer1 clk source select apb_clk */
#define CLK_APB2_TMR1SEL_RCL32K                 (0x00000100UL)  /*!< Timer1 clk source select rcl 32k */
#define CLK_APB2_TMR1SEL_TM1                    (0x00000200UL)  /*!< Timer1 clk source select externel input */
                                               
#define CLK_APB2_TMR2SEL_APB2CLK                (0x00000000UL)  /*!< Timer2 clk source select apb_clk */
#define CLK_APB2_TMR2SEL_RCL32K                 (0x00000400UL)  /*!< Timer2 clk source select rcl 32k */
#define CLK_APB2_TMR2SEL_TM2                    (0x00000800UL)  /*!< Timer2 clk source select externel input */

/**@defgroup CLK_FREQUENCT_FLAG Clk frequence
 * @brief       Clk frequence definitions
 * @{ */
 #define FREQ_1MHZ                            (1000000)         /*!< Clk frequence 1M Hz */
 #define FREQ_8MHZ                            (8000000)         /*!< Clk frequence 8M Hz */
 #define FREQ_16MHZ                           (16000000)        /*!< Clk frequence 16M Hz */
 #define FREQ_25MHZ                           (25000000)        /*!< Clk frequence 25M Hz */
 #define FREQ_32MHZ                           (32000000)        /*!< Clk frequence 32M Hz */
 #define FREQ_48MHZ                           (48000000)        /*!< Clk frequence 48M Hz */
 #define FREQ_64MHZ                           (64000000)        /*!< Clk frequence 64M Hz */
 #define FREQ_96MHZ                           (96000000)        /*!< Clk frequence 96M Hz */
 #define FREQ_50MHZ                           (50000000)        /*!< Clk frequence 50M Hz */
 #define FREQ_72MHZ                           (72000000)        /*!< Clk frequence 72M Hz */
 #define FREQ_100MHZ                          (100000000)       /*!< Clk frequence 100M Hz */
 #define FREQ_200MHZ                          (200000000)       /*!< Clk frequence 200M Hz */
 #define FREQ_250MHZ                          (250000000)       /*!< Clk frequence 250M Hz */
 #define FREQ_500MHZ                          (500000000)       /*!< Clk frequence 500M Hz */
/**@} */

/**@defgroup SYS_CLK_SOURCE_FLAG System clk source select
 * @brief       System clk source definitions
 * @{ */
#define CLK_SYS_SRCSEL_RCH         	          ((uint32_t)0x00000000)  /*!< System clk source frequence select as rch */
#define CLK_SYS_SRCSEL_XTH        	          ((uint32_t)0x00000100)  /*!< System clk source frequence select as xth */
#define CLK_SYS_SRCSEL_DPLL         	        ((uint32_t)0x00000200)  /*!< System clk source frequence select as dpll */
/**@} */

/**@defgroup DPLL_SOURCE_FLAG Dpll clk reference source select
 * @brief       Dpll clk reference source select definitions
 * @{ */
#define CLK_DPLL_REF_CLKSEL_RCH         	  ((uint32_t)0x00000000)    /*!< Dpll clk source select as rch */
#define CLK_DPLL_REF_CLKSEL_XTH         	  ((uint32_t)0x00000002)    /*!< Dpll clk source select as xth */
#define CLK_DPLL_OUT_48M       	            ((uint32_t)0x00000000)    /*!< Dpll clk output 48M Hz */
#define CLK_DPLL_OUT_64M        	          ((uint32_t)0x00000001)    /*!< Dpll clk output 64M Hz */
/**@} */


/**@defgroup APB2_CLK_DIV_FLAG Apb2 clk divisor
 * @brief       Apb2 clk divisor definitions
 * @{ */
#define CLK_HCLK_APB2_Div1                    (0)  /*!< Apb2 clk divisor set 1 */
#define CLK_HCLK_APB2_Div2                    (1)  /*!< Apb2 clk divisor set 2 */
#define CLK_HCLK_APB2_Div4                    (2)  /*!< Apb2 clk divisor set 4 */
#define CLK_HCLK_APB2_Div8                    (4)  /*!< Apb2 clk divisor set 8 */
#define CLK_HCLK_APB2_Div16                   (8)  /*!< Apb2 clk divisor set 16 */
/**@} */

/**@defgroup APB1_CLK_DIV_FLAG Apb1 clk divisor
 * @brief       Apb1 clk divisor definitions
 * @{ */
#define CLK_HCLK_APB1_Div1                    (0)  /*!< Apb1 clk divisor set 1 */
#define CLK_HCLK_APB1_Div2                    (1)  /*!< Apb1 clk divisor set 2 */
#define CLK_HCLK_APB1_Div4                    (2)  /*!< Apb1 clk divisor set 4 */
#define CLK_HCLK_APB1_Div8                    (4)  /*!< Apb1 clk divisor set 8 */
#define CLK_HCLK_APB1_Div16                   (8)  /*!< Apb1 clk divisor set 16 */
/**@} */

/**@defgroup APB1_PERIPHERAL_CLK_FLAG Apb1 clk enable
 * @brief       Apb1 clk enable definitions
 * @{ */
#define CLK_APB1Periph_I2C0               ((uint32_t)0x00000001)      /*!< Apb1 bus peripheral i2c clk enable */
#define CLK_APB1Periph_SPI0               ((uint32_t)0x00000002)      /*!< Apb1 bus peripheral spi0 clk enable */
#define CLK_APB1Periph_UART0              ((uint32_t)0x00000008)      /*!< Apb1 bus peripheral uart0 clk enable */
#define CLK_APB1Periph_PWM0_CH01          ((uint32_t)0x00000010)      /*!< Apb1 bus peripheral pwm0 ch0 & ch1 clk enable */
#define CLK_APB1Periph_PWM0_CH23          ((uint32_t)0x00000020)      /*!< Apb1 bus peripheral pwm0 ch2 & ch3 clk enable */
#define CLK_APB1Periph_PWM0_CH45          ((uint32_t)0x00000040)      /*!< Apb1 bus peripheral pwm0 ch4 & ch5 clk enable */
#define CLK_APB1Periph_PWM0_CH67          ((uint32_t)0x00000080)      /*!< Apb1 bus peripheral pwm0 ch6 & ch7 clk enable */
#define CLK_APB1Periph_PWM0_EN            ((uint32_t)0x00000100)      /*!< Apb1 bus peripheral pwm0 clk enable */
#define CLK_APB1Periph_ADC                ((uint32_t)0x00000200)      /*!< Apb1 bus peripheral adc clk enable */
#define CLK_APB1Periph_WDT                ((uint32_t)0x00000400)      /*!< Apb1 bus peripheral wdt clk enable */
#define CLK_APB1Periph_WWDT               ((uint32_t)0x00000800)      /*!< Apb1 bus peripheral wwdt clk enable */
#define CLK_APB1Periph_TMR0               ((uint32_t)0x00001000)      /*!< Apb1 bus peripheral timer0 clk enable */
#define CLK_APB1Periph_I2SS               ((uint32_t)0x00100000)      /*!< Apb1 bus peripheral i2s slave clk enable */
#define CLK_APB1Periph_I2SM               ((uint32_t)0x00200000)      /*!< Apb1 bus peripheral i2s master clk enable */
#define CLK_APB1Periph_PWM1_CH01          ((uint32_t)0x00400000)      /*!< Apb1 bus peripheral pwm1 ch0 & ch1 clk enable */
#define CLK_APB1Periph_PWM1_CH23          ((uint32_t)0x00800000)      /*!< Apb1 bus peripheral pwm1 ch2 & ch3 clk enable */
#define CLK_APB1Periph_PWM1_CH45          ((uint32_t)0x01000000)      /*!< Apb1 bus peripheral pwm1 ch4 & ch5 clk enable */
#define CLK_APB1Periph_PWM1_CH67          ((uint32_t)0x02000000)      /*!< Apb1 bus peripheral pwm1 ch6 & ch7 clk enable */
#define CLK_APB1Periph_PWM1_EN            ((uint32_t)0x04000000)      /*!< Apb1 bus peripheral pwm1 clk enable */
#define CLK_APB1Periph_PWM2_CH01          ((uint32_t)0x08000000)      /*!< Apb1 bus peripheral pwm2 ch0 & ch1 clk enable */
#define CLK_APB1Periph_PWM2_CH23          ((uint32_t)0x10000000)      /*!< Apb1 bus peripheral pwm2 ch2 & ch3 clk enable */
#define CLK_APB1Periph_PWM2_CH45          ((uint32_t)0x20000000)      /*!< Apb1 bus peripheral pwm2 ch4 & ch5 clk enable */
#define CLK_APB1Periph_PWM2_CH67          ((uint32_t)0x40000000)      /*!< Apb1 bus peripheral pwm2 ch6 & ch7 clk enable */
#define CLK_APB1Periph_PWM2_EN            ((uint32_t)0x80000000)      /*!< Apb1 bus peripheral pwm2 clk enable */
#define CLK_APB1Periph_All                ((uint32_t)0xfff01ffb)      /*!< Apb1 bus peripheral all clk enable */
/**@} */
  
/**@defgroup APB2_PERIPHERAL_CLK_FLAG Apb2 clk enable
 * @brief       Apb2 clk enable definitions
 * @{ */
#define CLK_APB2Periph_SPI1               ((uint32_t)0x00000002)      /*!< Apb2 bus peripheral spi1 clk enable */
#define CLK_APB2Periph_UART1              ((uint32_t)0x00000008)      /*!< Apb2 bus peripheral uart1 clk enable */
#define CLK_APB2Periph_TMR1               ((uint32_t)0x00000010)      /*!< Apb2 bus peripheral timer1 clk enable */
#define CLK_APB2Periph_TMR2               ((uint32_t)0x00000020)      /*!< Apb2 bus peripheral timer2 clk enable */
#define CLK_APB2Periph_KSCAN              ((uint32_t)0x00001000)      /*!< Apb2 bus peripheral kscan clk enable */
#define CLK_APB2Periph_QDEC               ((uint32_t)0x00400000)      /*!< Apb2 bus peripheral qdec clk enable */
#define CLK_APB2Periph_All                ((uint32_t)0x0040103a)      /*!< Apb2 bus peripheral all clk enable */
/**@} */

/**@defgroup AHB_PERIPHERAL_CLK_FLAG Ahb peripheral clk enable
 * @brief       Ahb peripheral clk enable definitions
 * @{ */
#define CLK_AHBPeriph_DMAC                ((uint32_t)0x00000001)      /*!< Ahb bus peripheral dma clk enable */
#define CLK_AHBPeriph_GPIO                ((uint32_t)0x00000002)      /*!< Ahb bus peripheral gpio clk enable */
#define CLK_AHBPeriph_SYSTICK             ((uint32_t)0x00000004)      /*!< Ahb bus peripheral systick clk enable */
#define CLK_AHBPeriph_APB1                ((uint32_t)0x00000008)      /*!< Ahb bus peripheral apb1 clk enable */
#define CLK_AHBPeriph_APB2                ((uint32_t)0x00000010)      /*!< Ahb bus peripheral apb2 clk enable */
#define CLK_AHBPeriph_AHB                 ((uint32_t)0x00000020)      /*!< Ahb bus peripheral ahb clk enable */
#define CLK_AHBPeriph_BLE_32M             ((uint32_t)0x00000040)      /*!< Ahb bus peripheral ble 32M clk enable */
#define CLK_AHBPeriph_BLE_32K             ((uint32_t)0x00000080)      /*!< Ahb bus peripheral ble 32K clk enable */
#define CLK_AHBPeriph_ROM                 ((uint32_t)0x00000400)      /*!< Ahb bus peripheral rom clk enable */
#define CLK_AHBPeriph_EFUSE               ((uint32_t)0x00000800)      /*!< Ahb bus peripheral efuse clk enable */
#define CLK_AHBPeriph_ACC                 ((uint32_t)0x00001000)      /*!< Ahb bus peripheral acc clk enable */
#define CLK_AHBPeriph_USB_AHB             ((uint32_t)0x00002000)      /*!< Ahb bus peripheral usb ahb clk enable */
#define CLK_AHBPeriph_USB_48M             ((uint32_t)0x00004000)      /*!< Ahb bus peripheral usb 48M clk enable */
#define CLK_AHBPeriph_All                 ((uint32_t)0x00007CFF)      /*!< Ahb bus peripheral all clk enable */
/**@} */

/**@defgroup CLK_SRC_FLAG Clk reference source
 * @brief       Clk reference source definitions
 * @{ */
#define CLK_RCL_SELECT                    (0)    /*!< Clk source reference set as rcl */
#define CLK_RCH_SELECT                    (1)    /*!< Clk source reference set as rch */
#define CLK_XTL_SELECT                    (2)    /*!< Clk source reference set as xtl */
#define CLK_XTH_SELECT                    (3)    /*!< Clk source reference set as xth */
#define CLK_DPLL_SELECT                   (4)    /*!< Clk source reference set as dpll */
/**@} */

#define CLK_STABLE_STATUS_Pos             (24)  /*!< Clk stable register position */
#define CLK_STABLE_STATUS_Msk             (0x1ul << CLK_STABLE_STATUS_Pos)    /*!< Clk stable register mask value */

#define CLKTRIM_CALC_CLK_SEL_32K		  (0)       /*!< Clktrim calibrate source selecet 32K */
#define CLKTRIM_CALC_CLK_SEL_32M		  (1)       /*!< Clktrim calibrate source selecet 32M */
#define CLKTRIM_CALC_CLK_SEL_EXT		  (3)       /*!< Clktrim calibrate source selecet external clk */

#define CLKTRIM_QDEC_CLK_SEL_APB		  (0)       /*!< Qdec clk source selecet apb clk */
#define CLKTRIM_QDEC_CLK_SEL_32K		  (1)       /*!< Qdec clk source selecet 32K */

#define CLKTRIM_KSCAN_CLK_SEL_APB		  (0)       /*!< Kscan clk source selecet apb clk */
#define CLKTRIM_KSCAN_CLK_SEL_32K		  (1)       /*!< Kscan clk source selecet 32K */


/**
  * @brief  Configures the xth clock.
  * @return None
  */
__STATIC_INLINE void CLK_XthStartupConfig(void)
{
    CLK->XTH_CTRL |= (CLK_XTHCTL_FSYN_EN_Msk | CLK_XTHCTL_START_FAST_Msk);
}

/**
  * @brief  Configures the Low Speed AHB clock (HCLK).
  * @param  u32ClkDiv: defines the AHB clock divisor. This clock is derived from 
  *         the AHB clock (HCLK).
  *          This parameter can be 0~15,hclk = hclk / (u32ClkDiv + 1):
  * @return None
  */
__STATIC_INLINE void CLK_HCLKConfig(uint32_t u32ClkDiv)
{
    CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_AHB_DIV_Msk)) | (u32ClkDiv << CLK_TOPCTL_AHB_DIV_Pos);
}

/**
  * @brief  Configures the Low Speed APB clock (PCLK1).
  * @param  u32ClkDiv: defines the APB1 clock divisor. This clock is derived from 
  *         the AHB clock (HCLK).
  * 		This parameter can be 0~15,PCLK1 = hclk / (u32ClkDiv * 2):
  * @return None
  */
__STATIC_INLINE void CLK_PCLK1Config(uint32_t u32ClkDiv )
{
    CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_APB1_DIV_Msk))| (u32ClkDiv << CLK_TOPCTL_APB1_DIV_Pos);
}

/**
  * @brief  Configures the High Speed APB clock (PCLK2).
  * @param  u32ClkDiv: defines the APB2 clock divisor. This clock is derived from 
  *         the AHB clock (HCLK).
  * 		This parameter can be 0~15,PCLK2 = hclk / (u32ClkDiv * 2):
  * @return None
  */
__STATIC_INLINE void CLK_PCLK2Config(uint32_t u32ClkDiv)
{
    CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_APB2_DIV_Msk))| (u32ClkDiv << CLK_TOPCTL_APB2_DIV_Pos);
}

/**
  * @brief  This function used to set qdec divisor
  * @param[in]  div is qdec sclk dividor ratio,9 bit width
  * @return   none
  *
  */
__STATIC_INLINE void CLK_SetQdecDiv(uint32_t div)
{
    CLK->APB2_CLK_CTRL = (CLK->APB2_CLK_CTRL & ~CLK_APB2CLK_QDEC_DIV_Msk) | (div << CLK_APB2CLK_QDEC_DIV_Pos);
}

/**
  * @brief  This function used to get qdec divisor
  * @return   qdec divisor value
  */
__STATIC_INLINE uint32_t CLK_GetQdecDiv(void)
{
    return ((CLK->APB2_CLK_CTRL & CLK_APB2CLK_QDEC_DIV_Msk) >> CLK_APB2CLK_QDEC_DIV_Pos);
}
/**
  * @brief  This function used to set qdec clk source
  * @param[in]  src is qdec sclk source,apb or 32k 
  * @return   none
  */
__STATIC_INLINE void CLK_SetQdecClkSrc(uint32_t src)
{
    CLK->APB2_CLK_CTRL1 = (CLK->APB2_CLK_CTRL1 & ~CLK_APB2CLK_QDEC_CLK_SEL_Msk) | (src << CLK_APB2CLK_QDEC_CLK_SEL_Pos);
}
/**
  * @brief  This function used to set keyscan divisor
  * @param[in]  div is keyscan sclk dividor ratio,9 bit width
  * @return   none
  */
__STATIC_INLINE void CLK_SetKeyscanDiv(uint32_t div)
{
    CLK->APB2_CLK_CTRL = (CLK->APB2_CLK_CTRL & ~CLK_APB2CLK_KEYSCAN_DIV_Msk) | (div << CLK_APB2CLK_KEYSCAN_DIV_Pos);
}
/**
  * @brief  This function used to get keyscan divisor
  * @return   kscan divisor value
  */
__STATIC_INLINE uint32_t CLK_GetKeyscanDiv(void)
{
    return ((CLK->APB2_CLK_CTRL & CLK_APB2CLK_KEYSCAN_DIV_Msk) >> CLK_APB2CLK_KEYSCAN_DIV_Pos);
}
/**
  * @brief  This function used to set keyscan clk source
  * @param[in]  src is keyscan sclk source,apb or 32k 
  * @return   none
  *
  */
__STATIC_INLINE void CLK_SetKeyscanClkSrc(uint32_t src)
{
    CLK->APB2_CLK_CTRL1 = (CLK->APB2_CLK_CTRL1 & ~CLK_APB2CLK_KSCAN_CLK_DIV_Msk) | (src << CLK_APB2CLK_KSCAN_CLK_DIV_Pos);
}


/**
  * @brief  This function set DPLL frequence
  * @param[in]  freq is target frequency,it could be:
  *							\ref CLK_DPLL_OUT_48M
  *							\ref CLK_DPLL_OUT_64M
  * @return   none
  *
  */
__STATIC_INLINE void CLK_SetDpllOutputFreq(uint32_t freq)
{
    CLK->DPLL_CTRL &= ~CLK_DPLLCTL_FREQ_OUT_Msk;
    CLK->DPLL_CTRL |= freq;
}

/**
  * @brief  This function set temperature gain value
  * @param[in]  gain is target gain value
  * @return   none
  *
  */
__STATIC_INLINE void CLK_SetTemperatureGain(uint32_t gain)
{
    ANA->ANA_TEMP = (ANA->ANA_TEMP & ~ANAC_TEMP_GAIN_CTL_Msk) | (gain<<ANAC_TEMP_GAIN_CTL_Pos);
}

/**
  * @brief  This function set temperature function enable
  * @return   none
  */
__STATIC_INLINE void CLK_EnableTempDetect(void)
{
		ANA->ANA_TEMP |= ANAC_TEMP_ENABLE_Msk;
}


/**
  * @brief  This function used to enable clktrim peripheral
  * @param[in]  NewState: new state of the clk.This parameter can be: ENABLE or DISABLE.
  * @return   none
  *
  */
__STATIC_INLINE void CLK_EnableClkTrim(FunctionalState NewState)
{
	(NewState != ENABLE)?(CLK->APB2_CLK_CTRL1 &= ~CLK_APB2CLK_TRIM_EN_Msk):(CLK->APB2_CLK_CTRL1 |= CLK_APB2CLK_TRIM_EN_Msk);
}

/**
  * @brief  This function used to enable clktrim calculate function
  * @param[in]  NewState: new state of the clk.This parameter can be: ENABLE or DISABLE.
  * @return   none
  *
  */
__STATIC_INLINE void CLK_EnableClkTrimCalc(FunctionalState NewState)
{
	(NewState != ENABLE)?(CLK->APB2_CLK_CTRL1 &= ~CLK_APB2CLK_TRIM_CALC_EN_Msk):(CLK->APB2_CLK_CTRL1 |= CLK_APB2CLK_TRIM_CALC_EN_Msk);
}

/**
  * @brief  This function used to select calculate clk source
  * @param[in]  src: clk source, including:
  *				\ref CLKTRIM_CALC_CLK_SEL_32K
  *				\ref CLKTRIM_CALC_CLK_SEL_32M
  *				\ref CLKTRIM_CALC_CLK_SEL_EXT
  * @return   none
  *
  */
__STATIC_INLINE void CLK_SelectClkTrimSrc(uint32_t src)
{
	CLK->APB2_CLK_CTRL1 = (CLK->APB2_CLK_CTRL1 & ~CLK_APB2CLK_CALC_CLK_SEL_Msk)| (src << CLK_APB2CLK_CALC_CLK_SEL_Pos);
}

/**
  * @brief  This function is used to set calculated high speed clock source (HSCK = RCH/EXT_CLK) divisor of CLKTRIM
  * @param[in]  div: clock divisor (9 bits),
  *                   0:      No division, cal_clk = HSCK
  *                   others: Divided by 2*div, cal_clk = HSCK / (2*div)
  * @return   none
  */
__STATIC_INLINE void CLK_SetClkTrimCalClkDiv(uint16_t div)
{
    uint32_t reg = CLK->APB2_CLK_CTRL1 & ~CLK_APB2CLK_CALC_CLK_DIV_Msk;
    CLK->APB2_CLK_CTRL1 = reg | ((div << CLK_APB2CLK_CALC_CLK_DIV_Pos) & CLK_APB2CLK_CALC_CLK_DIV_Msk);
}


enum
{
    CLK_FLASH_CLKDIV_1  = 0x00, /*!< Divide-by-1 (No Division) */
    CLK_FLASH_CLKDIV_2  = 0x01, /*!< Divide-by-2 */
    CLK_FLASH_CLKDIV_4  = 0x02, /*!< Divide-by-4 */
    CLK_FLASH_CLKDIV_8  = 0x04, /*!< Divide-by-8 */
    CLK_FLASH_CLKDIV_16 = 0x08, /*!< Divide-by-16 */
    CLK_FLASH_CLKDIV_32 = 0x10, /*!< Divide-by-32 */
};

/**
* @brief  This function is used to set flash clk divisor.
* @param[in]  div: clock divisor
* @return   none
*/
__STATIC_FORCEINLINE void CLK_SetFlashClkDiv(uint8_t div)
{
    uint32_t reg = CLK->AHB_CLK_CTRL & (~CLK_AHBCLK_SPI_FLASH_DIV_Msk);
    CLK->AHB_CLK_CTRL = reg | ((div << CLK_AHBCLK_SPI_FLASH_DIV_Pos) & CLK_AHBCLK_SPI_FLASH_DIV_Msk);
}

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void);

 /**
   * @brief  This function get CPU frequency. The frequency unit is Hz.
   * @return CPU frequency
   */
uint32_t CLK_GetCPUFreq(void);

/**
  * @brief  This function get APB1 frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetPCLK1Freq(void);

/**
  * @brief  This function get APB2 frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetPCLK2Freq(void);

 /**
    * @brief  This function set 16M ref clock source
    * @param[in]  u32ClkSrc is HCLK clock source. Including :
    *                  - \ref CLK_SYS_SRCSEL_RCH 
    *                  - \ref CLK_SYS_SRCSEL_XTH 
    *                  - \ref CLK_SYS_SRCSEL_DPLL
    * @return None
    */
void CLK_RefClkSrcConfig(uint32_t u32ClkSrc);
/**
  * @brief  This function set HCLK clock source
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *                  - \ref CLK_DPLL_REF_CLKSEL_RCH
  *                  - \ref CLK_DPLL_REF_CLKSEL_XTH
  * @param[in]  freq_out is output frequence clock
  * @return None
  */
void CLK_SYSCLKConfig(uint32_t u32ClkSrc,uint32_t freq_out);
/**
  * @brief  This function set CPU frequency divider. The frequency unit is Hz.
  * @param[in]  u32ClkDiv is ahb clock division
  * @return HCLK frequency
  */
void CLK_HCLKConfig(uint32_t u32ClkDiv);
/**
  * @brief  This function set APB1 frequency divider. The frequency unit is Hz.
  * @param[in]  u32ClkDiv is is APB1 clock division
  * @return HCLK frequency
  */
void CLK_PCLK1Config(uint32_t u32ClkDiv );
/**
  * @brief  This function set APB2 frequency divider. The frequency unit is Hz.
  * @param[in]  u32ClkDiv is is APB2 clock division  
  * @return HCLK frequency
  */
void CLK_PCLK2Config(uint32_t u32ClkDiv);

/**
  * @brief  Enables or disables the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @param  CLK_AHBPeriph: specifies the AHB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *             \ref AHB_PERIPHERAL_CLK_FLAG
  * @param  NewState    : new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_AHBPeriphClockCmd(uint32_t CLK_AHBPeriph, FunctionalState NewState);

/**
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @param  CLK_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            \ref APB1_PERIPHERAL_CLK_FLAG
  * @param  NewState  : new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_APB1PeriphClockCmd(uint32_t CLK_APB1Periph, FunctionalState NewState);

/**
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  CLK_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            \ref APB2_PERIPHERAL_CLK_FLAG 
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CLK_APB2PeriphClockCmd(uint32_t CLK_APB2Periph, FunctionalState NewState);

/**
  * @brief  This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including
  *                  - \ref CLK_RCL_SELECT
  *                  - \ref CLK_RCH_SELECT
  *                  - \ref CLK_DPLL_SELECT
  * @retval   0  clock is not stable
  * @retval   1  clock is stable
  *
  * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);

/**
  * @brief  This function wait sync 3v clock locale stable
  * @param[in]  none
  * @retval   0  clock sync is not stable
  * @retval   1  clock sync is stable
  *
  * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
  */
uint32_t CLK_Wait3vSyncReady(void);

/**
* @brief  This function wait sync 3v clock locale stable by hardware
* @return   none
*/
void CLK_Set3vSyncAuto(void);
/**
  * @brief This API is used to select wdt clock source
  * @param[in] u32clksel wdt clock selection. Could be 
  *             - \ref CLK_APB1_WDTSEL_MILLI_PULSE, \ref CLK_APB1_WDTSEL_RCL32K
  * @return none
  * @note This API is only used to select wdt clock source
  */
void CLK_SetWdtClkSrc(uint32_t u32clksel);

/**
  * @brief This API is used to select wwdt clock source
  * @param[in] u32clksel wwdt clock selection. Could be 
  *             - \ref CLK_APB1_WWDTSEL_MILLI_PULSE, \ref CLK_APB1_WWDTSEL_RCL32K
  * @return none
  * @note This API is only used to select wwdt clock source
  */
void CLK_SetWwdtClkSrc(uint32_t u32clksel);

/**
  * @brief This API is used to SELECT timer clock source
  * @param[in] timer The base address of Timer module
  * @param[in] u32clksel timer clock selection. Could be 
  *             - \ref CLK_APB_TMR0SEL_APB1CLK, \ref CLK_APB_TMR0SEL_LIRC,\ref CLK_APB_TMR0SEL_TM0
  *             - \ref CLK_APB_TMR1SEL_APB2CLK, \ref CLK_APB_TMR1SEL_LIRC,\ref CLK_APB_TMR0SEL_TM1
  *             - \ref CLK_APB_TMR2SEL_APB2CLK, \ref CLK_APB_TMR2SEL_LIRC,\ref CLK_APB_TMR0SEL_TM2
  * @return none
  * @note This API is only used to select timer clock source
  */
void CLK_SetTmrClkSrc(TIMER_T *timer, uint32_t u32clksel);

/**
  * @brief This API is used to get peripheral clk frequence
  * @param[in] Peripheral The base address of peripheral module
  * @return none
  * @note This API is only used to select timer clock source
  */
uint32_t CLK_GetPeripheralFreq(void * Peripheral);

/**
  * @brief This macro is used to disable Auto-sync function
  * @return none
  */
#define DISABLE_3V_AUTOSYNC       ANA->LP_REG_SYNC &= ~ANAC_LP_REG_AUTOSYNC_Msk
/**
  * @brief This macro is used to enable Auto-sync function
  * @return none
  */
#define ENABLE_3V_AUTOSYNC        ANA->LP_REG_SYNC |= ANAC_LP_REG_AUTOSYNC_Msk
/**
  * @brief This macro is used to start sync
  * @return none
  */
#define START_3V_SYNC_NBLOCK      ANA->LP_REG_SYNC |= ANAC_LP_REG_SYNC_3V_Msk

/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_CLK_H__ */
