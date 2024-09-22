/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series system driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */
#ifndef __PAN_SYS_H__
#define __PAN_SYS_H__


/**
 * @brief System Interface
 * @defgroup system_interface System Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif


/**@defgroup MODULE_RST_CTRL_FLAG Module reset control resister 
 * @brief       Module reset control resister constant definitions
 * @{ */
#define ADC_RST   ((0x4<<24) | SYS_IPRST1_ADCRST_Pos   ) /*!< ADC  reset is one of the SYS_ResetModule parameter */
#define ACMP_RST  ((0x4<<24) | SYS_IPRST1_ACMPRST_Pos  ) /*!< ACMP reset is one of the SYS_ResetModule parameter */
#define PWM0_RST  ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos  ) /*!< PWM  reset is one of the SYS_ResetModule parameter */
#define WWDT_RST  ((0x4<<24) | SYS_IPRST1_WWDTRST_Pos  ) /*!< WWDT  reset is one of the SYS_ResetModule parameter */
#define WDT_RST   ((0x4<<24) | SYS_IPRST1_WDTRST_Pos   ) /*!< WWDT  reset is one of the SYS_ResetModule parameter */
#define UART0_RST ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST ((0x4<<24) | SYS_IPRST1_UART1RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST  ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos  ) /*!< SPI0  reset is one of the SYS_ResetModule parameter */
#define SPI1_RST  ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos  ) /*!< SPI1  reset is one of the SYS_ResetModule parameter */
#define I2C0_RST  ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!< I2C0  reset is one of the SYS_ResetModule parameter */
#define I2C1_RST  ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos  ) /*!< I2C1  reset is one of the SYS_ResetModule parameter */
#define TMR2_RST  ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos  ) /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST  ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos  ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST  ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos  ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define GPIO_RST  ((0x4<<24) | SYS_IPRST1_GPIORST_Pos  ) /*!< GPIO reset is one of the SYS_ResetModule parameter */
/**@} */

/**@defgroup BROWN_OUT_DETECTOR_FLAG Brown out detector threshold voltage selection 
 * @brief       rown out detector threshold voltage selection constant definitions
 * @{ */
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)  /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_DISABLE              (3UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Disable */
#define SYS_BODCTL_BODVL_2_67V          (2UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 267V */
#define SYS_BODCTL_BODVL_2_4V           (1UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.40V */
#define SYS_BODCTL_BODVL_2_15V          (0UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.15V */
/**@} */

/**@defgroup MULTI_FUNC_FLAG Io function define
 * @brief       Io function define constant definitions
 * @{ */
#define SYS_MFP_TYPE_Msk(bit)       (1UL << ((bit) +16)) /*!< TYPE mask for Multiple Function Port */
#define SYS_MFP_ALT_Msk(bit)        (1UL << ((bit) + 8)) /*!< ALT mask for Multiple Function Port */
#define SYS_MFP_MFP_Msk(bit)        (1UL << ((bit)    )) /*!< MFP mask for Multiple Function Port */

#define SYS_MFP_GPIO                    0x00000000UL  /*!< Io function act as gpio */

#define SYS_MFP_P00_GPIO                0x00000000UL  /*!< Io p00 function act as gpio */
#define SYS_MFP_P00_UART0_TX            0x00000001UL  /*!< Io p00 function act as uart tx */
#define SYS_MFP_P00_I2C0_SCL            0x00000100UL  /*!< Io p00 function act as i2c scl */
#define SYS_MFP_P00_TMR0_OUT            0x00000101UL  /*!< Io p00 function act as timer out */
#define SYS_MFP_P00_PWM0_CH0            0x00010000UL  /*!< Io p00 function act as pwm0 ch0 */
#define SYS_MFP_P00_I2SM_DI             0x00010001UL  /*!< Io p00 function act as i2s master data in */
#define SYS_MFP_P00_KSCAN_O6            0x00010100UL  /*!< Io p00 function act as kscan output ch6 */
#define SYS_MFP_P00_I2SS_DI             0x00010101UL  /*!< Io p00 function act as i2s slave data in */
#define SYS_MFP_P00_Msk                 0x00010101UL  /*!< Io p00 function mask */

#define SYS_MFP_P01_GPIO                0x00000000UL  /*!< Io p01 function act as gpio */
#define SYS_MFP_P01_UART0_RX            0x00000002UL  /*!< Io p01 function act as uart rx */
#define SYS_MFP_P01_I2C0_SDA            0x00000200UL  /*!< Io p01 function act as i2c sda */
#define SYS_MFP_P01_I2SM_DO             0x00000202UL  /*!< Io p01 function act as i2s master data out */
#define SYS_MFP_P01_PWM0_CH1            0x00020000UL  /*!< Io p01 function act as pwm0 ch1 */
#define SYS_MFP_P01_I2SS_DO             0x00020002UL  /*!< Io p01 function act as i2s slave data out */
#define SYS_MFP_P01_KSCAN_O7            0x00020200UL  /*!< Io p01 function act as kscan output ch7 */
#define SYS_MFP_P01_LL_DBG01            0x00020202UL  /*!< Io p01 function act as linklayer debug */
#define SYS_MFP_P01_Msk                 0x00020202UL  /*!< Io p01 function mask */

#define SYS_MFP_P02_GPIO                0x00000000UL  /*!< Io p02 function act as gpio */
#define SYS_MFP_P02_UART0_CTS           0x00000004UL  /*!< Io p02 function act as uart cts */
#define SYS_MFP_P02_ANT_SW0             0x00000400UL  /*!< Io p02 function act as ant sw0 */
#define SYS_MFP_P02_SPI0_CS             0x00000404UL  /*!< Io p02 function act as spi0 cs */
#define SYS_MFP_P02_USB_DM              0x00040000UL  /*!< Io p02 function act as pwm ch0 */
#define SYS_MFP_P02_PWM0_CH2            0x00040004UL  /*!< Io p02 function act as pwm0 ch2 */
#define SYS_MFP_P02_TADC_DATL           0x00040400UL  /*!< Io p02 function act as test adc data low */
#define SYS_MFP_P02_KSCAN_I0            0x00040404UL  /*!< Io p02 function act as kscan input ch0 */
#define SYS_MFP_P02_Msk                 0x00040404UL  /*!< Io p02 function mask */

#define SYS_MFP_P03_GPIO                0x00000000UL  /*!< Io p03 function act as gpio */
#define SYS_MFP_P03_UART0_RTS           0x00000008UL  /*!< Io p03 function act as uart rts */
#define SYS_MFP_P03_ANT_SW1             0x00000800UL  /*!< Io p03 function act as ant sw1 */
#define SYS_MFP_P03_SPI0_CLK            0x00000808UL  /*!< Io p03 function act as spi0 clk */
#define SYS_MFP_P03_USB_DP              0x00080000UL  /*!< Io p03 function act as usb dp */
#define SYS_MFP_P03_PWM0_CH3            0x00080008UL  /*!< Io p03 function act as pwm0 ch3 */
#define SYS_MFP_P03_TADC_VLD            0x00080800UL  /*!< Io p03 function act as test adc valid flg */
#define SYS_MFP_P03_KSCAN_I1            0x00080808UL  /*!< Io p03 function act as kscan input ch1 */
#define SYS_MFP_P03_Msk                 0x00080808UL  /*!< Io p03 function mask */

#define SYS_MFP_P04_GPIO                0x00000000UL  /*!< Io p04 function act as gpio */
#define SYS_MFP_P04_I2SS_DI             0x00000010UL  /*!< Io p04 function act as i2s slave data in */
#define SYS_MFP_P04_I2SM_DI             0x00001000UL  /*!< Io p04 function act as i2s master data in */
#define SYS_MFP_P04_TMR2_OUT            0x00001010UL  /*!< Io p04 function act as timer2 out */
#define SYS_MFP_P04_PWM1_CH0            0x00100000UL  /*!< Io p04 function act as pwm1 ch0 */
#define SYS_MFP_P04_KSCAN_O10           0x00100010UL  /*!< Io p04 function act as kscan output ch10 */
#define SYS_MFP_P04_QDEC_X0             0x00101000UL  /*!< Io p04 function act as qdec ch x0 */
#define SYS_MFP_P04_LL_DBG10            0x00101010UL  /*!< Io p04 function act as linklayer debug */
#define SYS_MFP_P04_Msk                 0x00101010UL  /*!< Io p04 function mask */

#define SYS_MFP_P05_GPIO                0x00000000UL  /*!< Io p05 function act as gpio */
#define SYS_MFP_P05_I2SS_DO             0x00000020UL  /*!< Io p05 function act as i2s slave data out */
#define SYS_MFP_P05_I2SM_DO             0x00002000UL  /*!< Io p05 function act as i2s master data out */
#define SYS_MFP_P05_ANT_SW2             0x00002020UL  /*!< Io p05 function act as ant sw2 */
#define SYS_MFP_P05_PWM1_CH1            0x00200000UL  /*!< Io p05 function act as pwm1 ch1 */
#define SYS_MFP_P05_KSCAN_O11           0x00200020UL  /*!< Io p05 function act as kscan output ch11 */
#define SYS_MFP_P05_QDEC_X1             0x00202000UL  /*!< Io p05 function act as qdec ch x1 */
#define SYS_MFP_P05_EXT_STADC           0x00202020UL  /*!< Io p05 function act as externel adc input */
#define SYS_MFP_P05_Msk                 0x00202020UL  /*!< Io p05 function mask */

#define SYS_MFP_P06_GPIO                0x00000000UL  /*!< Io p06 function act as gpio */
#define SYS_MFP_P06_SPI1_CS             0x00000040UL  /*!< Io p06 function act as spi1 cs */
#define SYS_MFP_P06_UART1_TX            0x00004000UL  /*!< Io p06 function act as uart1 tx */
#define SYS_MFP_P06_ANT_SW3             0x00004040UL  /*!< Io p06 function act as ant sw3 */
#define SYS_MFP_P06_PWM1_CH2            0x00400000UL  /*!< Io p06 function act as pwm1 ch2 */
#define SYS_MFP_P06_KSCAN_O12           0x00400040UL  /*!< Io p06 function act as kscan output ch12 */
#define SYS_MFP_P06_QDEC_Y0             0x00404000UL  /*!< Io p06 function act as qdec ch y0 */
#define SYS_MFP_P06_I2S_MCLK            0x00404040UL  /*!< Io p06 function act as i2s mclk */
#define SYS_MFP_P06_Msk                 0x00404040UL  /*!< Io p06 function mask */

#define SYS_MFP_P07_GPIO                0x00000000UL  /*!< Io p07 function act as gpio */
#define SYS_MFP_P07_SPI1_CLK            0x00000080UL  /*!< Io p07 function act as spi1 clk */
#define SYS_MFP_P07_UART1_RX            0x00008000UL  /*!< Io p07 function act as uart1 rx */
#define SYS_MFP_P07_TMR0_EXT            0x00008080UL  /*!< Io p07 function act as timer0 externel input */
#define SYS_MFP_P07_PWM1_CH3            0x00800000UL  /*!< Io p07 function act as pwm1 ch3 */
#define SYS_MFP_P07_KSCAN_O13           0x00800080UL  /*!< Io p07 function act as kscan output ch13 */
#define SYS_MFP_P07_QDEC_Y1             0x00808000UL  /*!< Io p07 function act as qdec ch y1 */
#define SYS_MFP_P07_ANT_SW5             0x00808080UL  /*!< Io p07 function act as ant sw5 */
#define SYS_MFP_P07_Msk                 0x00808080UL  /*!< Io p07 function mask */

#define SYS_MFP_P10_GPIO                0x00000000UL  /*!< Io p10 function act as gpio */
#define SYS_MFP_P10_SPI0_MOSI           0x00000001UL  /*!< Io p10 function act as spi0 mosi */
#define SYS_MFP_P10_TMR0_EXT            0x00000100UL  /*!< Io p10 function act as timer0 externel input */
#define SYS_MFP_P10_ADC_CH5             0x00000101UL  /*!< Io p10 function act as adc ch5 */
#define SYS_MFP_P10_PWM0_CH4            0x00010000UL  /*!< Io p10 function act as pwm0 ch4 */
#define SYS_MFP_P10_KSCAN_O4            0x00010001UL  /*!< Io p10 function act as kscan output ch4 */
#define SYS_MFP_P10_32K_CLK             0x00010100UL  /*!< Io p10 function act as 32k clk */
#define SYS_MFP_P10_LL_DBG14            0x00010101UL  /*!< Io p10 function act as linklayer debug */
#define SYS_MFP_P10_Msk                 0x00010101UL  /*!< Io p10 function mask */

#define SYS_MFP_P11_GPIO                0x00000000UL  /*!< Io p11 function act as gpio */
#define SYS_MFP_P11_SPI0_MISO           0x00000002UL  /*!< Io p11 function act as spi0 miso */
#define SYS_MFP_P11_CALIB_CLK           0x00000200UL  /*!< Io p11 function act as calibrate clk */
#define SYS_MFP_P11_ADC_CH4             0x00000202UL  /*!< Io p11 function act as adc ch4 */
#define SYS_MFP_P11_PWM0_CH5            0x00020000UL  /*!< Io p11 function act as pwm0 ch5 */
#define SYS_MFP_P11_KSCAN_O5            0x00020002UL  /*!< Io p11 function act as kscan output ch5 */
#define SYS_MFP_P11_AHB_CLK             0x00020200UL  /*!< Io p11 function act as ahb clk */
#define SYS_MFP_P11_I2S_MCLK            0x00020202UL  /*!< Io p11 function act as i2s mclk */
#define SYS_MFP_P11_Msk                 0x00020202UL  /*!< Io p11 function mask */

#define SYS_MFP_P12_GPIO                0x00000000UL  /*!< Io p12 function act as gpio */
#define SYS_MFP_P12_I2C0_SCL            0x00000004UL  /*!< Io p12 function act as i2c scl */
#define SYS_MFP_P12_I2SS_CLK            0x00000400UL  /*!< Io p12 function act as i2s slave clk */
#define SYS_MFP_P12_I2SM_CLK            0x00000404UL  /*!< Io p12 function act as i2s master clk */
#define SYS_MFP_P12_PWM0_CH6            0x00040000UL  /*!< Io p12 function act as pwm0 ch6 */
#define SYS_MFP_P12_TADC_CLK            0x00040004UL  /*!< Io p12 function act as test adc clk */
#define SYS_MFP_P12_DPLL                0x00040400UL  /*!< Io p12 function act as dpll clk */
#define SYS_MFP_P12_LL_DBG15            0x00040404UL  /*!< Io p12 function act as linklayer debug */
#define SYS_MFP_P12_Msk                 0x00040404UL  /*!< Io p12 function mask */

#define SYS_MFP_P13_GPIO                0x00000000UL  /*!< Io p13 function act as gpio */
#define SYS_MFP_P13_I2C0_SDA            0x00000008UL  /*!< Io p13 function act as i2c sda tx */
#define SYS_MFP_P13_I2SS_WS             0x00000800UL  /*!< Io p13 function act as i2s slave ws */
#define SYS_MFP_P13_I2SM_WS             0x00000808UL  /*!< Io p13 function act as i2s master ws */
#define SYS_MFP_P13_PWM0_CH7            0x00080000UL  /*!< Io p13 function act as pwm0 ch7 */
#define SYS_MFP_P13_TADC_DATH           0x00080008UL  /*!< Io p13 function act as test adc data high */
#define SYS_MFP_P13_RCH                 0x00080800UL  /*!< Io p13 function act as rch clk */
#define SYS_MFP_P13_ANT_SW4             0x00080808UL  /*!< Io p13 function act as ant sw4 */
#define SYS_MFP_P13_Msk                 0x00080808UL  /*!< Io p13 function mask */

#define SYS_MFP_P14_GPIO                0x00000000UL  /*!< Io p14 function act as gpio */
#define SYS_MFP_P14_UART1_TX            0x00000010UL  /*!< Io p14 function act as uart1 tx */
#define SYS_MFP_P14_I2C0_SDA            0x00001000UL  /*!< Io p14 function act as i2c sda */
#define SYS_MFP_P14_ANT_SW6             0x00001010UL  /*!< Io p14 function act as ant sw6 */
#define SYS_MFP_P14_PWM0_CH4            0x00100000UL  /*!< Io p14 function act as pwm0 ch4 */
#define SYS_MFP_P14_KSCAN_I4            0x00100010UL  /*!< Io p14 function act as kscan input ch4 */
#define SYS_MFP_P14_QDEC_X0             0x00101000UL  /*!< Io p14 function act as qdecc ch x0 */
#define SYS_MFP_P14_MDM_DBG18           0x00101010UL  /*!< Io p14 function act as i2s slave data in */
#define SYS_MFP_P14_Msk                 0x00101010UL  /*!< Io p14 function mask */

#define SYS_MFP_P15_GPIO                0x00000000UL  /*!< Io p15 function act as gpio */
#define SYS_MFP_P15_UART1_RX            0x00000020UL  /*!< Io p15 function act as uart1 rx */
#define SYS_MFP_P15_ANT_SW7             0x00002000UL  /*!< Io p15 function act as ant sw7 */
#define SYS_MFP_P15_I2C0_SCL            0x00002020UL  /*!< Io p15 function act as i2c scl */
#define SYS_MFP_P15_PWM0_CH5            0x00200000UL  /*!< Io p15 function act as pwm0 ch5 */
#define SYS_MFP_P15_KSCAN_I5            0x00200020UL  /*!< Io p15 function act as kscan input ch5 */
#define SYS_MFP_P15_QDEC_X1             0x00202000UL  /*!< Io p15 function act as qdec ch x1 */
#define SYS_MFP_P15_LL_DBG04            0x00202020UL  /*!< Io p15 function act as linklayer debug */
#define SYS_MFP_P15_Msk                 0x00202020UL  /*!< Io p15 function mask */

#define SYS_MFP_P16_GPIO                0x00000000UL  /*!< Io p16 function act as gpio */
#define SYS_MFP_P16_UART1_CTS           0x00000040UL  /*!< Io p16 function act as uart1 cts */
#define SYS_MFP_P16_ANT_SW4             0x00004000UL  /*!< Io p16 function act as ant sw4 */
#define SYS_MFP_P16_TMR0_OUT            0x00004040UL  /*!< Io p16 function act as timer0 out */
#define SYS_MFP_P16_PWM0_CH6            0x00400000UL  /*!< Io p16 function act as pwm0 ch6 */
#define SYS_MFP_P16_KSCAN_I6            0x00400040UL  /*!< Io p16 function act as kscan input ch6 */
#define SYS_MFP_P16_QDEC_Y0             0x00404000UL  /*!< Io p16 function act as qdec ch y0 */
#define SYS_MFP_P16_LL_DBG13            0x00404040UL  /*!< Io p16 function act as linklayer debug */
#define SYS_MFP_P16_Msk                 0x00404040UL  /*!< Io p16 function mask */

#define SYS_MFP_P17_GPIO                0x00000000UL  /*!< Io p17 function act as gpio */
#define SYS_MFP_P17_UART1_RTS           0x00000080UL  /*!< Io p17 function act as uart1 rts */
#define SYS_MFP_P17_MDM_DBG03           0x00008000UL  /*!< Io p17 function act as modem debug */
#define SYS_MFP_P17_TMR1_OUT            0x00008080UL  /*!< Io p17 function act as timer1 out */
#define SYS_MFP_P17_PWM0_CH7            0x00800000UL  /*!< Io p17 function act as pwm0 ch7 */
#define SYS_MFP_P17_KSCAN_I7            0x00800080UL  /*!< Io p17 function act as kscan input ch7 */
#define SYS_MFP_P17_QDEC_Y1             0x00808000UL  /*!< Io p17 function act as qdec ch y1 */
#define SYS_MFP_P17_LL_DBG07            0x00808080UL  /*!< Io p17 function act as linklayer debug */
#define SYS_MFP_P17_Msk                 0x00808080UL  /*!< Io p17 function mask */

#define SYS_MFP_P20_GPIO                0x00000000UL  /*!< Io p20 function act as gpio */
#define SYS_MFP_P20_SPI1_CS             0x00000001UL  /*!< Io p20 function act as spi1 cs */
#define SYS_MFP_P20_TMR1_EXT            0x00000100UL  /*!< Io p20 function act as timer1 externel input */
#define SYS_MFP_P20_INT2                0x00000101UL  /*!< Io p20 function act as externel int */
#define SYS_MFP_P20_ADC_CH6             0x00010000UL  /*!< Io p20 function act as adc ch6 */
#define SYS_MFP_P20_KSCAN_I0            0x00010001UL  /*!< Io p20 function act as kscan input ch0 */
#define SYS_MFP_P20_QDEC_Z0             0x00010100UL  /*!< Io p20 function act as qdec ch z0 */
#define SYS_MFP_P20_LL_DBG08            0x00010101UL  /*!< Io p20 function act as linklayer debug */
#define SYS_MFP_P20_Msk                 0x00010101UL  /*!< Io p20 function mask */

#define SYS_MFP_P21_GPIO                0x00000000UL  /*!< Io p21 function act as gpio */
#define SYS_MFP_P21_SPI1_CLK            0x00000002UL  /*!< Io p21 function act as spi1 clk */
#define SYS_MFP_P21_TMR2_EXT            0x00000200UL  /*!< Io p21 function act as timer2 externel input */
#define SYS_MFP_P21_MDM_DBG04           0x00000202UL  /*!< Io p21 function act as modem debug */
#define SYS_MFP_P21_ADC_CH7             0x00020000UL  /*!< Io p21 function act as adc ch7 */
#define SYS_MFP_P21_KSCAN_I1            0x00020002UL  /*!< Io p21 function act as kscan input ch1 */
#define SYS_MFP_P21_QDEC_Z1             0x00020200UL  /*!< Io p21 function act as qdec ch z1 */
#define SYS_MFP_P21_LL_DBG09            0x00020202UL  /*!< Io p21 function act as linklayer debug */
#define SYS_MFP_P21_Msk                 0x00020202UL  /*!< Io p21 function mask */

#define SYS_MFP_P22_GPIO                0x00000000UL  /*!< Io p22 function act as gpio */
#define SYS_MFP_P22_QDEC_X_IDX          0x00000004UL  /*!< Io p22 function act as qdec ch x index */
#define SYS_MFP_P22_PWM1_CH4            0x00000400UL  /*!< Io p22 function act as pwm1 ch4 */
#define SYS_MFP_P22_I2SS_CLK            0x00000404UL  /*!< Io p22 function act as i2s slave clk */
#define SYS_MFP_P22_SPI1_MISO           0x00040000UL  /*!< Io p22 function act as spi1 miso */
#define SYS_MFP_P22_KSCAN_I2            0x00040004UL  /*!< Io p22 function act as kscan input ch2 */
#define SYS_MFP_P22_I2SM_CLK            0x00040400UL  /*!< Io p22 function act as i2s master clk */
#define SYS_MFP_P22_LL_DBG05            0x00040404UL  /*!< Io p22 function act as linklayer debug */
#define SYS_MFP_P22_Msk                 0x00040404UL  /*!< Io p22 function mask */

#define SYS_MFP_P23_GPIO                0x00000000UL  /*!< Io p23 function act as gpio */
#define SYS_MFP_P23_QDEC_Y_IDX          0x00000008UL  /*!< Io p23 function act as qdec ch y index */
#define SYS_MFP_P23_PWM1_CH5            0x00000800UL  /*!< Io p23 function act as pwm1 ch5 */
#define SYS_MFP_P23_I2SS_WS             0x00000808UL  /*!< Io p23 function act as i2s slave ws */
#define SYS_MFP_P23_SPI1_MOSI           0x00080000UL  /*!< Io p23 function act as pwm ch0 */
#define SYS_MFP_P23_KSCAN_I3            0x00080008UL  /*!< Io p23 function act as kscan input ch3 */
#define SYS_MFP_P23_I2SM_WS             0x00080800UL  /*!< Io p23 function act as i2s master ws */
#define SYS_MFP_P23_LL_DBG06            0x00080808UL  /*!< Io p23 function act as linklayer debug */
#define SYS_MFP_P23_Msk                 0x00080808UL  /*!< Io p23 function mask */

#define SYS_MFP_P24_GPIO                0x00000000UL  /*!< Io p24 function act as gpio */
#define SYS_MFP_P24_UART1_TX            0x00000010UL  /*!< Io p24 function act as uart1 tx */
#define SYS_MFP_P24_SPI1_CLK            0x00001000UL  /*!< Io p24 function act as spi1 clk */
#define SYS_MFP_P24_ANT_SW0             0x00001010UL  /*!< Io p24 function act as ant sw0 */
#define SYS_MFP_P24_PWM1_CH0            0x00100000UL  /*!< Io p24 function act as pwm1 ch0 */
#define SYS_MFP_P24_KSCAN_O8            0x00100010UL  /*!< Io p24 function act as kscan out ch8 */
#define SYS_MFP_P24_QDEC_Z0             0x00101000UL  /*!< Io p24 function act as qdec ch z0 */
#define SYS_MFP_P24_MDM_DBG07           0x00101010UL  /*!< Io p24 function act as modem debug */
#define SYS_MFP_P24_Msk                 0x00101010UL  /*!< Io p24 function mask */

#define SYS_MFP_P25_GPIO                0x00000000UL  /*!< Io p25 function act as gpio */
#define SYS_MFP_P25_UART1_RX            0x00000020UL  /*!< Io p25 function act as uart1 rx */
#define SYS_MFP_P25_SPI1_CS             0x00002000UL  /*!< Io p25 function act as spi1 cs */
#define SYS_MFP_P25_ANT_SW1             0x00002020UL  /*!< Io p25 function act as ant sw1 */
#define SYS_MFP_P25_QDEC_Z_IDX          0x00200000UL  /*!< Io p25 function act as qdec ch z index */
#define SYS_MFP_P25_KSCAN_O9            0x00200020UL  /*!< Io p25 function act as kscan output ch9 */
#define SYS_MFP_P25_PWM1_CH1            0x00202000UL  /*!< Io p25 function act as pwm1 ch1 */
#define SYS_MFP_P25_MDM_DBG08           0x00202020UL  /*!< Io p25 function act as modem debug */
#define SYS_MFP_P25_Msk                 0x00202020UL  /*!< Io p25 function mask */

#define SYS_MFP_P26_GPIO                0x00000000UL  /*!< Io p26 function act as gpio */
#define SYS_MFP_P26_UART1_CTS           0x00000040UL  /*!< Io p26 function act as uart1 cts */
#define SYS_MFP_P26_UART0_TX            0x00004000UL  /*!< Io p26 function act as uart0 tx */
#define SYS_MFP_P26_SPI1_MISO           0x00004040UL  /*!< Io p26 function act as spi1 miso */
#define SYS_MFP_P26_MDM_DBG17           0x00400000UL  /*!< Io p26 function act as modem debug */
#define SYS_MFP_P26_KSCAN_O14           0x00400040UL  /*!< Io p26 function act as kscan output ch14 */
#define SYS_MFP_P26_PWM1_CH2            0x00404000UL  /*!< Io p26 function act as pwm1 ch2 */
#define SYS_MFP_P26_LL_DBG12            0x00404040UL  /*!< Io p26 function act as linklayer debug */
#define SYS_MFP_P26_Msk                 0x00404040UL  /*!< Io p26 function mask */

#define SYS_MFP_P27_GPIO                0x00000000UL  /*!< Io p27 function act as gpio */
#define SYS_MFP_P27_UART1_RTS           0x00000080UL  /*!< Io p27 function act as uart1 rts */
#define SYS_MFP_P27_UART0_RX            0x00008000UL  /*!< Io p27 function act as uart0 rx */
#define SYS_MFP_P27_SPI1_MOSI           0x00008080UL  /*!< Io p27 function act as spi1 mosi */
#define SYS_MFP_P27_ADC_CH2             0x00800000UL  /*!< Io p27 function act as adc ch2 */
#define SYS_MFP_P27_KSCAN_O15           0x00800080UL  /*!< Io p27 function act as kscan output ch15 */
#define SYS_MFP_P27_QDEC_Z1             0x00808000UL  /*!< Io p27 function act as qdec ch z1 */
#define SYS_MFP_P27_MDM_DBG10           0x00808080UL  /*!< Io p27 function act as modem debug */
#define SYS_MFP_P27_Msk                 0x00808080UL  /*!< Io p27 function mask */

#define SYS_MFP_P30_GPIO                0x00000000UL  /*!< Io p30 function act as gpio */
#define SYS_MFP_P30_UART0_TX            0x00000001UL  /*!< Io p30 function act as uart0 tx */
#define SYS_MFP_P30_PWM1_CH6            0x00000100UL  /*!< Io p30 function act as pwm1 ch6 */
#define SYS_MFP_P30_SPI0_MOSI           0x00000101UL  /*!< Io p30 function act as spi0 mosi */
#define SYS_MFP_P30_ADC_CH1             0x00010000UL  /*!< Io p30 function act as adc ch1 */
#define SYS_MFP_P30_KSCAN_I4            0x00010001UL  /*!< Io p30 function act as kscan input ch4 */
#define SYS_MFP_P30_MDM_DBG11           0x00010100UL  /*!< Io p30 function act as modem debug */
#define SYS_MFP_P30_MDM_SPI_CLK         0x00010101UL  /*!< Io p30 function act as modem spi clk */
#define SYS_MFP_P30_Msk                 0x00010101UL  /*!< Io p30 function mask */

#define SYS_MFP_P31_GPIO                0x00000000UL  /*!< Io p31 function act as gpio */
#define SYS_MFP_P31_UART0_RX            0x00000002UL  /*!< Io p31 function act as uart0 rx */
#define SYS_MFP_P31_PWM1_CH7            0x00000200UL  /*!< Io p31 function act as pwm ch7 */
#define SYS_MFP_P31_SPI0_MISO           0x00000202UL  /*!< Io p31 function act as spi0 miso */
#define SYS_MFP_P31_ADC_CH0             0x00020000UL  /*!< Io p31 function act as adc ch0 */
#define SYS_MFP_P31_KSCAN_I5            0x00020002UL  /*!< Io p31 function act as kscan input ch5 */
#define SYS_MFP_P31_MDM_DBG12           0x00020200UL  /*!< Io p31 function act as modem debug */
#define SYS_MFP_P31_MDM_SPI_CS          0x00020202UL  /*!< Io p31 function act as modem spi cs */
#define SYS_MFP_P31_Msk                 0x00020202UL  /*!< Io p31 function mask */

#define SYS_MFP_P32_GPIO                0x00000000UL  /*!< Io p32 function act as gpio */
#define SYS_MFP_P32_SPI0_CS             0x00000004UL  /*!< Io p32 function act as spi0 cs */
#define SYS_MFP_P32_BOD_OUT             0x00000400UL  /*!< Io p32 function act as bod output */
#define SYS_MFP_P32_INT0                0x00000404UL  /*!< Io p32 function act as int0 */
#define SYS_MFP_P32_PWM0_CH2            0x00040000UL  /*!< Io p32 function act as pwm2 ch2 */
#define SYS_MFP_P32_KSCAN_O16           0x00040004UL  /*!< Io p32 function act as kscan output ch16 */
#define SYS_MFP_P32_MDM_DBG14           0x00040400UL  /*!< Io p32 function act as modem debug */
#define SYS_MFP_P32_MDM_SPI_MOSI        0x00040404UL  /*!< Io p32 function act as modem spi mosi */
#define SYS_MFP_P32_Msk                 0x00040404UL  /*!< Io p32 function mask */

#define SYS_MFP_P33_GPIO                0x00000000UL  /*!< Io p33 function act as gpio */
#define SYS_MFP_P33_SPI0_CLK            0x00000008UL  /*!< Io p33 function act as spi0 clk */
#define SYS_MFP_P33_LVR_OUT             0x00000800UL  /*!< Io p33 function act as lvr output */
#define SYS_MFP_P33_UART0_RTS           0x00000808UL  /*!< Io p33 function act as uart0 rts */
#define SYS_MFP_P33_PWM0_CH3            0x00080000UL  /*!< Io p33 function act as pwm0 ch3 */
#define SYS_MFP_P33_KSCAN_O17           0x00080008UL  /*!< Io p33 function act as kscan output ch17 */
#define SYS_MFP_P33_LL_DBG03            0x00080800UL  /*!< Io p33 function act as linklayer debug */
#define SYS_MFP_P33_MDM_SPI_MISO        0x00080808UL  /*!< Io p33 function act as modem spi miso */
#define SYS_MFP_P33_Msk                 0x00080808UL  /*!< Io p33 function mask */

#define SYS_MFP_P34_GPIO                0x00000000UL  /*!< Io p34 function act as gpio */
#define SYS_MFP_P34_PWM2_CH4            0x00000010UL  /*!< Io p34 function act as pwm2 ch4 */
#define SYS_MFP_P34_KSCAN_O18           0x00001000UL  /*!< Io p34 function act as kscan output ch18 */
#define SYS_MFP_P34_MDM_DBG02           0x00001010UL  /*!< Io p34 function act as modem debug */
#define SYS_MFP_P34_MDM_TX_DATA         0x00100000UL  /*!< Io p34 function act as modem tx data */
#define SYS_MFP_P34_TMR1_EXT            0x00100010UL  /*!< Io p34 function act as timer1 externel input */
#define SYS_MFP_P34_USB_DIP             0x00101000UL  /*!< Io p34 function act as usb phy dip */
#define SYS_MFP_P34_RESERVED            0x00101010UL  /*!< Reversed function*/
#define SYS_MFP_P34_Msk                 0x00101010UL  /*!< Io p34 function mask */

#define SYS_MFP_P35_GPIO                0x00000000UL  /*!< Io p35 function act as gpio */
#define SYS_MFP_P35_PWM2_CH5            0x00000020UL  /*!< Io p35 function act as pwm2 ch5 */
#define SYS_MFP_P35_KSCAN_O19           0x00002000UL  /*!< Io p35 function act as kscan output ch19 */
#define SYS_MFP_P35_MDM_DBG05           0x00002020UL  /*!< Io p35 function act as modem debug */
#define SYS_MFP_P35_MDM_TX_ON           0x00200000UL  /*!< Io p35 function act as modem tx on */
#define SYS_MFP_P35_UART0_CTS           0x00200020UL  /*!< Io p35 function act as uart0 cts */
#define SYS_MFP_P35_USB_DIM             0x00202000UL  /*!< Io p35 function act as usb phy dim */
#define SYS_MFP_P35_RESERVED            0x00202020UL  /*!< Reversed function */
#define SYS_MFP_P35_Msk                 0x00202020UL  /*!< Io p35 function mask */

#define SYS_MFP_P36_GPIO                0x00000000UL  /*!< Io p36 function act as gpio */
#define SYS_MFP_P36_PWM2_CH6            0x00000040UL  /*!< Io p36 function act as pwm2 ch6 */
#define SYS_MFP_P36_KSCAN_O20           0x00004000UL  /*!< Io p36 function act as kscan output ch20 */
#define SYS_MFP_P36_MDM_DBG06           0x00004040UL  /*!< Io p36 function act as modem debug */
#define SYS_MFP_P36_MDM_RX_ON           0x00400000UL  /*!< Io p36 function act as modem rx on */
#define SYS_MFP_P36_CALIB_CLK           0x00400040UL  /*!< Io p36 function act as calibrate clk */
#define SYS_MFP_P36_USB_DIDIF           0x00404000UL  /*!< Io p36 function act as usb phy didif */
#define SYS_MFP_P36_RESERVED            0x00404040UL  /*!< Reversed function */
#define SYS_MFP_P36_Msk                 0x00404040UL  /*!< Io p36 function mask */

#define SYS_MFP_P37_GPIO                0x00000000UL  /*!< Io p37 function act as gpio */
#define SYS_MFP_P37_PWM2_CH7            0x00000080UL  /*!< Io p37 function act as pwm2 ch7 */
#define SYS_MFP_P37_KSCAN_O21           0x00008000UL  /*!< Io p37 function act as kscan output ch21 */
#define SYS_MFP_P37_KSCAN_I2            0x00008080UL  /*!< Io p37 function act as kscan input ch2 */
#define SYS_MFP_P37_MDM_ACC_MATCH       0x00800000UL  /*!< Io p37 function act as modem acc match */
#define SYS_MFP_P37_MDM_DBG15           0x00800080UL  /*!< Io p37 function act as modem debug */
#define SYS_MFP_P37_USB_DOP             0x00808000UL  /*!< Io p37 function act as usb phy dop */
#define SYS_MFP_P37_RESERVED            0x00808080UL  /*!< Reversed function */
#define SYS_MFP_P37_Msk                 0x00808080UL  /*!< Io p37 function mask */

#define SYS_MFP_P40_GPIO                0x00000000UL  /*!< Io p40 function act as gpio */
#define SYS_MFP_P40_SPI0_CLK            0x00000001UL  /*!< Io p40 function act as spi0 clk */
#define SYS_MFP_P40_I2C0_SDA            0x00000100UL  /*!< Io p40 function act as i2c sda */
#define SYS_MFP_P40_I2SS_CLK            0x00000101UL  /*!< Io p40 function act as i2s slave clk */
#define SYS_MFP_P40_I2SM_CLK            0x00010000UL  /*!< Io p40 function act as i2s master clk */
#define SYS_MFP_P40_MDM_RX_DATA         0x00010001UL  /*!< Io p40 function act as i2s modem rx data */
#define SYS_MFP_P40_PWM2_CH0            0x00010100UL  /*!< Io p40 function act as pwm2 ch0 */
#define SYS_MFP_P40_KSCAN_O2            0x00010101UL  /*!< Io p40 function act as kscan output ch2 */
#define SYS_MFP_P40_Msk                 0x00010101UL  /*!< Io p40 function mask */

#define SYS_MFP_P41_GPIO                0x00000000UL  /*!< Io p41 function act as gpio */
#define SYS_MFP_P41_SPI0_CS             0x00000002UL  /*!< Io p41 function act as spi0 cs */
#define SYS_MFP_P41_I2SS_DI             0x00000200UL  /*!< Io p41 function act as i2s slave di */
#define SYS_MFP_P41_PWM2_CH1            0x00000202UL  /*!< Io p41 function act as pwm2 ch1 */
#define SYS_MFP_P41_I2SM_DI             0x00020000UL  /*!< Io p41 function act as i2s master di */
#define SYS_MFP_P41_MDM_RX_CLK          0x00020002UL  /*!< Io p41 function act as modem rx clk */
#define SYS_MFP_P41_MDM_DBG13           0x00020200UL  /*!< Io p41 function act as modem debug */
#define SYS_MFP_P41_I2C0_SCL            0x00020202UL  /*!< Io p41 function act as i2c scl */
#define SYS_MFP_P41_Msk                 0x00020202UL  /*!< Io p41 function mask */

#define SYS_MFP_P42_GPIO                0x00000000UL  /*!< Io p42 function act as gpio */
#define SYS_MFP_P42_SPI0_MISO           0x00000004UL  /*!< Io p42 function act as spi0 miso */
#define SYS_MFP_P42_I2SS_DO             0x00000400UL  /*!< Io p42 function act as i2s slvae do */
#define SYS_MFP_P42_PWM2_CH2            0x00000404UL  /*!< Io p42 function act as pwm2 ch2 */
#define SYS_MFP_P42_I2SM_DO             0x00040000UL  /*!< Io p42 function act as i2s master do */
#define SYS_MFP_P42_MDM_RX_DAT0         0x00040004UL  /*!< Io p42 function act as modem rx dat0 */
#define SYS_MFP_P42_LL_DBG02            0x00040400UL  /*!< Io p42 function act as linklayer debug */
#define SYS_MFP_P42_MDM_DBG20           0x00040404UL  /*!< Io p42 function act as modem debug */
#define SYS_MFP_P42_Msk                 0x00040404UL  /*!< Io p42 function mask */

#define SYS_MFP_P43_GPIO                0x00000000UL  /*!< Io p43 function act as gpio */
#define SYS_MFP_P43_SPI0_MOSI           0x00000008UL  /*!< Io p43 function act as spi0 mosi */
#define SYS_MFP_P43_UART0_RX            0x00000800UL  /*!< Io p43 function act as uart0 rx */
#define SYS_MFP_P43_PWM2_CH3            0x00000808UL  /*!< Io p43 function act as pwm2 ch3 */
#define SYS_MFP_P43_I2SMCLK             0x00080000UL  /*!< Io p43 function act as i2s mclk */
#define SYS_MFP_P43_MDM_RX_DAT1         0x00080008UL  /*!< Io p43 function act as i2s master data in */
#define SYS_MFP_P43_UART0_TX            0x00080008UL  /*!< Io p43 function act as uart0 tx,if uart0_tx_en=1 */
#define SYS_MFP_P43_MDM_DBG00           0x00080800UL  /*!< Io p43 function act as modem debug */
#define SYS_MFP_P43_KSCAN_O15           0x00080808UL  /*!< Io p43 function act as kscan output ch15 */
#define SYS_MFP_P43_Msk                 0x00080808UL  /*!< Io p43 function mask */

#define SYS_MFP_P44_GPIO                0x00000000UL  /*!< Io p44 function act as gpio */
#define SYS_MFP_P44_UART0_TX            0x00000010UL  /*!< Io p44 function act as uart0 tx */
#define SYS_MFP_P44_I2SS_WS             0x00001000UL  /*!< Io p44 function act as i2s slave ws */
#define SYS_MFP_P44_PWM2_CH4            0x00001010UL  /*!< Io p44 function act as pwm2 ch4 */
#define SYS_MFP_P44_I2SM_WS             0x00100000UL  /*!< Io p44 function act as i2s master ws */
#define SYS_MFP_P44_MDM_IO_RSTN         0x00100010UL  /*!< Io p44 function act as modem io reset */
#define SYS_MFP_P44_MDM_DBG01           0x00101000UL  /*!< Io p44 function act as modem debug */
#define SYS_MFP_P44_USB_DOM             0x00101010UL  /*!< Io p44 function act as usb phy dom */
#define SYS_MFP_P44_Msk                 0x00101010UL  /*!< Io p44 function mask */

#define SYS_MFP_P45_GPIO                0x00000000UL  /*!< Io p45 function act as gpio */
#define SYS_MFP_P45_PWM2_CH0            0x00000020UL  /*!< Io p45 function act as pwm2 ch0 */
#define SYS_MFP_P45_KSCAN_O22           0x00002000UL  /*!< Io p45 function act as kscan output ch22 */
#define SYS_MFP_P45_MDM_EST_DONE        0x00002020UL  /*!< Io p45 function act as modem est done */
#define SYS_MFP_P45_USB_NDOE            0x00200000UL  /*!< Io p45 function act as usb phy ndoe */
#define SYS_MFP_P45_RESERVED            0x00200020UL  /*!< Reversed function */
#define SYS_MFP_P45_RESERVED1           0x00202000UL  /*!< Reversed function */
#define SYS_MFP_P45_RESERVED2           0x00202020UL  /*!< Reversed function */
#define SYS_MFP_P45_Msk                 0x00202020UL  /*!< Io p45 function mask */

#define SYS_MFP_P46_GPIO                0x00000000UL  /*!< Io p46 function act as gpio */
#define SYS_MFP_P46_SWD_CLK             0x00000040UL  /*!< Io p46 function act as swd clk */
#define SYS_MFP_P46_UART1_RX            0x00004000UL  /*!< Io p46 function act as uart1 rx */
#define SYS_MFP_P46_I2C0_SCL            0x00004040UL  /*!< Io p46 function act as i2c scl */
#define SYS_MFP_P46_SPI0_CLK            0x00400000UL  /*!< Io p46 function act as spi0 clk */
#define SYS_MFP_P46_ANT_SW5             0x00400040UL  /*!< Io p46 function act as ant sw5 */
#define SYS_MFP_P46_KSCAN_O0            0x00404000UL  /*!< Io p46 function act as kscan output ch0 */
#define SYS_MFP_P46_MDM_DBG09           0x00404040UL  /*!< Io p46 function act as modem debug */
#define SYS_MFP_P46_Msk                 0x00404040UL  /*!< Io p46 function mask */

#define SYS_MFP_P47_GPIO                0x00000000UL  /*!< Io p47 function act as gpio */
#define SYS_MFP_P47_SWD_DAT             0x00000080UL  /*!< Io p47 function act as swd data */
#define SYS_MFP_P47_UART1_TX            0x00008000UL  /*!< Io p47 function act as uart1 tx */
#define SYS_MFP_P47_I2C0_SDA            0x00008080UL  /*!< Io p47 function act as i2c sda */
#define SYS_MFP_P47_SPI0_CS             0x00800000UL  /*!< Io p47 function act as spi0 cs */
#define SYS_MFP_P47_ANT_SW6             0x00800080UL  /*!< Io p47 function act as ant sw6 */
#define SYS_MFP_P47_KSCAN_O1            0x00808000UL  /*!< Io p47 function act as kscan output ch1 */
#define SYS_MFP_P47_MDM_DBG16           0x00808080UL  /*!< Io p47 function act as modem debug */
#define SYS_MFP_P47_Msk                 0x00808080UL  /*!< Io p47 function mask */

#define SYS_MFP_P50_GPIO                0x00000000UL  /*!< Io p50 function act as gpio */
#define SYS_MFP_P50_PWM2_CH1            0x00000001UL  /*!< Io p50 function act as pwm2 ch1 */
#define SYS_MFP_P50_KSCAN_O0            0x00000100UL  /*!< Io p50 function act as kscan output ch0 */
#define SYS_MFP_P50_MDM_STDBY_EN        0x00000101UL  /*!< Io p50 function act as modem standby en */
#define SYS_MFP_P50_FSYNSD_D            0x00010000UL  /*!< Io p50 function act as fsynsd */
#define SYS_MFP_P50_RESERVED1           0x00010001UL  /*!< Reversed function */
#define SYS_MFP_P50_RESERVED2           0x00010100UL  /*!< Reversed function */
#define SYS_MFP_P50_RCO_32M             0x00010101UL  /*!< Io p50 function act as rc 32M clk */
#define SYS_MFP_P50_Msk                 0x00010101UL  /*!< Io p50 function mask */

#define SYS_MFP_P51_GPIO                0x00000000UL  /*!< Io p51 function act as gpio */
#define SYS_MFP_P51_PWM1_CH7            0x00000002UL  /*!< Io p51 function act as pwm1 ch7 */
#define SYS_MFP_P51_KSCAN_O1            0x00000200UL  /*!< Io p51 function act as kscan output ch1 */
#define SYS_MFP_P51_QDEC_X_IDX          0x00000202UL  /*!< Io p51 function act as qdec ch x index */
#define SYS_MFP_P51_FSYNSD_DIV2         0x00020000UL  /*!< Io p51 function act as fsynsd divider */
#define SYS_MFP_P51_RESERVED1           0x00020002UL  /*!< Reversed function */
#define SYS_MFP_P51_RESERVED2           0x00020200UL  /*!< Reversed function */
#define SYS_MFP_P51_XTH_32M             0x00020202UL  /*!< Io p51 function act as xth 32M clk */
#define SYS_MFP_P51_Msk                 0x00020202UL  /*!< Io p51 function mask */

#define SYS_MFP_P52_GPIO                0x00000000UL  /*!< Io p52 function act as gpio */
#define SYS_MFP_P52_EXT_STADC           0x00000004UL  /*!< Io p52 function act as externel adc input */
#define SYS_MFP_P52_SPI1_CS             0x00000400UL  /*!< Io p52 function act as spi1 cs */
#define SYS_MFP_P52_INT1                0x00000404UL  /*!< Io p52 function act as int1 */
#define SYS_MFP_P52_ADC_CH3             0x00040000UL  /*!< Io p52 function act as adc ch3 */
#define SYS_MFP_P52_KSCAN_O2            0x00040004UL  /*!< Io p52 function act as kscan output ch2 */
#define SYS_MFP_P52_ANT_SW7             0x00040400UL  /*!< Io p52 function act as ant sw7 */
#define SYS_MFP_P52_LL_DBG00            0x00040404UL  /*!< Io p52 function act as linklayer debug */
#define SYS_MFP_P52_Msk                 0x00040404UL  /*!< Io p52 function mask */

#define SYS_MFP_P53_GPIO                0x00000000UL  /*!< Io p53 function act as gpio */
#define SYS_MFP_P53_PWM1_CH3            0x00000008UL  /*!< Io p53 function act as pwm1 ch3 */
#define SYS_MFP_P53_KSCAN_O3            0x00000800UL  /*!< Io p53 function act as kscan output ch3 */
#define SYS_MFP_P53_QDEC_Y_IDX          0x00000808UL  /*!< Io p53 function act as qdec ch y index */
#define SYS_MFP_P53_RESERVED            0x00080000UL  /*!< Reversed function */
#define SYS_MFP_P53_RESERVED1           0x00080008UL  /*!< Reversed function */
#define SYS_MFP_P53_RESERVED2           0x00080800UL  /*!< Reversed function */
#define SYS_MFP_P53_RESERVED3           0x00080808UL  /*!< Reversed function */
#define SYS_MFP_P53_Msk                 0x00080808UL  /*!< Io p53 function mask */

#define SYS_MFP_P54_GPIO                0x00000000UL  /*!< Io p54 function act as gpio */
#define SYS_MFP_P54_PWM2_CH2            0x00000010UL  /*!< Io p54 function act as pwm2 ch2 */
#define SYS_MFP_P54_KSCAN_O23           0x00001000UL  /*!< Io p54 function act as kscan output ch23 */
#define SYS_MFP_P54_QDEC_Z_IDX          0x00001010UL  /*!< Io p54 function act as qdec ch z index */
#define SYS_MFP_P54_RESERVED            0x00100000UL  /*!< Reversed function */
#define SYS_MFP_P54_RESERVED1           0x00100010UL  /*!< Reversed function */
#define SYS_MFP_P54_RESERVED2           0x00101000UL  /*!< Reversed function */
#define SYS_MFP_P54_RESERVED3           0x00101010UL  /*!< Reversed function */
#define SYS_MFP_P54_Msk                 0x00101010UL  /*!< Io p54 function mask */

#define SYS_MFP_P55_GPIO                0x00000000UL  /*!< Io p55 function act as gpio */
#define SYS_MFP_P55_PWM2_CH3            0x00000020UL  /*!< Io p55 function act as pwm2 ch3 */
#define SYS_MFP_P55_KSCAN_O4            0x00002000UL  /*!< Io p55 function act as kscan output ch4 */
#define SYS_MFP_P55_QDEC_X0             0x00002020UL  /*!< Io p55 function act as qdec ch x0 */
#define SYS_MFP_P55_RESERVED            0x00200000UL  /*!< Reversed function */
#define SYS_MFP_P55_RESERVED1           0x00200020UL  /*!< Reversed function */
#define SYS_MFP_P55_RESERVED2           0x00202000UL  /*!< Reversed function */
#define SYS_MFP_P55_RESERVED3           0x00202020UL  /*!< Reversed function */
#define SYS_MFP_P55_Msk                 0x00202020UL  /*!< Io p55 function mask */

#define SYS_MFP_P56_GPIO                0x00000000UL  /*!< Io p56 function act as gpio */
#define SYS_MFP_P56_PWM1_CH6            0x00000040UL  /*!< Io p56 function act as pwm1 ch6 */
#define SYS_MFP_P56_UART0_CTS           0x00004000UL  /*!< Io p56 function act as uart0 cts */
#define SYS_MFP_P56_UART1_TX            0x00004040UL  /*!< Io p56 function act as uart1 tx */
#define SYS_MFP_P56_QDEC_X0             0x00400000UL  /*!< Io p56 function act as qdec ch x0 */
#define SYS_MFP_P56_KSCAN_O11           0x00400040UL  /*!< Io p56 function act as kscan output ch11 */
#define SYS_MFP_P56_MDM_DBG19           0x00404000UL  /*!< Io p56 function act as modem debug */
#define SYS_MFP_P56_LL_DBG11            0x00404040UL  /*!< Io p56 function act as linklayer debug */
#define SYS_MFP_P56_Msk                 0x00404040UL  /*!< Io p56 function mask */

#define SYS_MFP_P57_GPIO                0x00000000UL  /*!< Io p57 function act as gpio */
#define SYS_MFP_P57_PWM2_CH5            0x00000080UL  /*!< Io p57 function act as pwm2 ch5 */
#define SYS_MFP_P57_KSCAN_O5            0x00008000UL  /*!< Io p57 function act as kscan output ch5 */
#define SYS_MFP_P57_QDEC_X1             0x00008080UL  /*!< Io p57 function act as qdec ch x1 */
#define SYS_MFP_P57_RESERVED            0x00800000UL  /*!< Reversed function */
#define SYS_MFP_P57_RESERVED1           0x00800080UL  /*!< Reversed function */
#define SYS_MFP_P57_RESERVED2           0x00808000UL  /*!< Reversed function */
#define SYS_MFP_P57_RESERVED3           0x00808080UL  /*!< Reversed function */
#define SYS_MFP_P57_Msk                 0x00808080UL  /*!< Io p57 function mask */
/**@} */

/**
  * @brief      Set pin function
  * @param[in]   Port GPIO port number.
  * @param[in]   Bit  The single or multiple pins of specified gpio port.
  * @param[in]   Func  pin function
  * @return     None
  */
#define SYS_SET_MFP(Port, Bit, Func)     \
            (SYS->Port##_MFP = (SYS->Port##_MFP & ~SYS_MFP_##Port##Bit##_Msk) | SYS_MFP_##Port##Bit##_##Func)

/**@defgroup BOD_LVR_DEBOUNCE_FLAG Bod and lvr glitch time control
 * @brief       Bod and lvr glitch time control constant definitions
 * @{ */
#define SYS_BLDBCTL_BODDBSEL_2POW4      0x00000001UL  /*!< Bod debounce time 2^4 sys clock */
#define SYS_BLDBCTL_BODDBSEL_2POW7      0x00000002UL  /*!< Bod debounce time 2^7 sys clock */
#define SYS_BLDBCTL_BODDBSEL_2POW9      0x00000004UL  /*!< Bod debounce time 2^9 sys clock */
#define SYS_BLDBCTL_BODDBSEL_2POW11     0x00000008UL  /*!< Bod debounce time 2^11 sys clock */
#define SYS_BLDBCTL_BODDBSEL_2POW13     0x00000010UL  /*!< Bod debounce time 2^13 sys clock */
#define SYS_BLDBCTL_BODDBSEL_2POW15     0x00000020UL  /*!< Bod debounce time 2^15 sys clock */

#define SYS_BLDBCTL_LVRDBSEL_2POW4      0x00000100UL  /*!< Lvr debounce time 2^4 sys clock */
#define SYS_BLDBCTL_LVRDBSEL_2POW7      0x00000200UL  /*!< Lvr debounce time 2^7 sys clock */
#define SYS_BLDBCTL_LVRDBSEL_2POW9      0x00000400UL  /*!< Lvr debounce time 2^9 sys clock */
#define SYS_BLDBCTL_LVRDBSEL_2POW11     0x00000800UL  /*!< Lvr debounce time 2^11 sys clock */
#define SYS_BLDBCTL_LVRDBSEL_2POW13     0x00001000UL  /*!< Lvr debounce time 2^13 sys clock */
#define SYS_BLDBCTL_LVRDBSEL_2POW15     0x00002000UL  /*!< Lvr debounce time 2^15 sys clock */
/**@} */


/**
  * @brief      Enable Brown-out detector reset function
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Disable register write-protection function
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
     {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
    while(SYS->REGLCTL == 0);
}

/**
  * @brief      Enable register write-protection function
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    // (Workaround) To insure write-protected reg
    // is succefully wrote before lock
#if 1
    __NOP();__NOP();__NOP();__NOP();__NOP();
#else
    __ISB();
#endif
    SYS->REGLCTL = 0;
}

/**
  * @brief  This function check register write-protection bit setting
  * @retval 0: Write-protection function is disabled.
  * @retval 1: Write-protection function is enabled.
  */
uint32_t SYS_IsRegLocked(void);

/**
  * @brief  This function disable BOD function.
  * @return None
  */
void SYS_DisableBOD(void);

/**
  * @brief       Set delay time
  * @param[in]   u32NopCnt nop count.
  * @return     None
  */
void SYS_delay_10nop(uint32_t u32NopCnt);

/*@}*/

#ifdef __cplusplus
}
#endif

#endif /* __PAN_SYS_H__ */
