#ifndef __PANSERIES_H__
#define __PANSERIES_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>


typedef enum IRQn
{
  NonMaskableInt_IRQn         = -14,    /*!<  2 Cortex-M0 Non Maskable Interrupt                  */
  HardFault_IRQn              = -13,    /*!<  3 Cortex-M0 Hard Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */
  I2C0_IRQn                   = 0,      /*!< i2c Interrupt */
  SPI0_IRQn                   = 1,      /*!< spi0 Interrupt */
  I2S_IRQn                    = 2,      /*!< i2s Interrupt */
  UART0_IRQn                  = 3,      /*!< uart0 Interrupt */
  PWM0_IRQn                   = 4,      /*!< pwm0 Interrupt */
  ADC_IRQn                    = 5,      /*!< adc Interrupt */
  WDT_IRQn                    = 6,      /*!< watch dog Interrupt */
  WWDT_IRQn                   = 7,      /*!< window watch dog Interrupt */
  TMR0_IRQn                   = 8,      /*!< timer0 Interrupt */
  ACC_IRQn                    = 9,      /*!< acc Interrupt */
  SPI1_IRQn                   = 10,     /*!< spi1 Interrupt */
  LL_IRQn                     = 11,     /*!< linlayer Interrupt */
  UART1_IRQn                  = 12,     /*!< uart1 Interrupt */
  TMR1_IRQn                   = 13,     /*!< timer1 Interrupt */
  TMR2_IRQn                   = 14,     /*!< timer2 Interrupt */
  TRIM_IRQn                   = 15,     /*!< clk trim Interrupt */
  KSCAN_IRQn                  = 16,     /*!< keyscan Interrupt */
  QDEC_IRQn                   = 17,     /*!< qdec Interrupt */
  GPIO_IRQn                   = 18,     /*!< gpio Interrupt */
  PWM1_IRQn                   = 19,     /*!< pwm1 Interrupt */
  PWM2_IRQn                   = 20,     /*!< pwm2 Interrupt */
  USBDMA_IRQn                 = 21,     /*!< usb dma Interrupt */
  USB_IRQn                    = 22,     /*!< usb Interrupt */
  RSVD0_IRQn                  = 23,		/*!< reserved 0 Interrupt */
  RSVD1_IRQn                  = 24,		/*!< reserved 1 Interrupt */
  RSVD2_IRQn                  = 25,		/*!< reserved 2 Interrupt */
  RSVD3_IRQn                  = 26,		/*!< reserved 3 Interrupt */
  DMA_IRQn                    = 27,     /*!< dma Interrupt */
  BOD_IRQn                    = 28,     /*!< bod Interrupt */
  SLEEP_IRQn                  = 29,     /*!< sleep Interrupt */
  STANDBY_IRQn                = 30,     /*!< standby Interrupt */
  MODEM_IRQn                  = 31,     /*!< Modem Interrupt */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                 0x0000    /*!< Core revision r0p0                               */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __FPU_PRESENT             0         /*!< FPU present or not                               */

#include <core_cm0.h>                       /* Cortex-M0 processor and core peripherals           */
#include "system_PanSeries.h"             /* Panchip Series System include file                   */


/** @addtogroup Exported_types
  * @{
  */

#include <stdbool.h>

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;


/*---------------------- Timer Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TIMER Controller
@{ */

typedef struct {


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Timer0 Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock or event source is divided by (PSC+1) before it is fed to the timer up counter.
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |[19]    |CAPSRC    |Capture Pin Source Select Bit
     * |        |          |0 = Capture Function source is from TMx_EXT (x= 0~1)
     * |        |          |pin.
     * |        |          |1 = Capture Function source is from internal ACMP output signal.
     * |        |          |User can set CAPSRCMP (TIMERx_EXTCTL[9]) to decide which ACMP output signal as timer capture source.
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while the timer interrupt flag TIF (TIMERx_INTSTS[0]) is 1 and INTEN (TIMERx_CTL[29]) is enabled, the timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[24]    |EXTCNTEN  |Event Counter Mode Enable Bit
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |0 = Event counter mode Disabled.
     * |        |          |1 = Event counter mode Enabled.
     * |        |          |Note: When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source
     * |[25]    |ACTSTS    |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |RSTCNT    |Timer Counter Reset
     * |        |          |Setting this bit will reset the 24-bit up counter value CNT (TIMERx_CNT[23:0]) and also force CNTEN (TIMERx_CTL[30]) to 0 if ACTSTS (TIMERx_CTL[25]) is 1.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset internal 8-bit prescale counter, 24-bit up counter value and CNTEN bit.
     * |[28:27] |OPMODE    |Timer Counting Mode Selection
     * |        |          |00 = The Timer controller is operated in one-shot mode.
     * |        |          |01 = The Timer controller is operated in periodic mode.
     * |        |          |10 = The Timer controller is operated in toggle-output mode.
     * |        |          |11 = The Timer controller is operated in continuous counting mode.
     * |[29]    |INTEN     |Timer Interrupt Enable Bit
     * |        |          |0 = Timer Interrupt Disabled.
     * |        |          |1 = Timer Interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the timer interrupt flag TIF is set to 1, the timer interrupt signal will be generated and inform CPU.
     * |[30]    |CNTEN     |Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then setting CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMERx_CTL[28:27] = 2'b00) when the timer interrupt flag TIF (TIMERx_INTSTS[0]) is generated.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
    */
    __IO uint32_t CTL;

    /**
     * CMP
     * ===================================================================================================
     * Offset: 0x04  Timer0 Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Compared Value
     * |        |          |CMPDAT is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TIMERx_INTSTS[0] Timer Interrupt Flag) will set to .
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the timer compared value while user writes a new value into the CMPDAT fiel.
    */
    __IO uint32_t CMP;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  Timer0 Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value, and interrupt occur.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[2]     |TF      	 |Timer Flag
     * |        |          |This bit indicates the flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = Timer No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
    */
    __IO uint32_t INTSTS;

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  Timer0 Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |This field can be reflected the internal 24-bit timer counter value or external event input counter value from TMx (x=0~1) pin.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 0, user can read CNT value for getting current 24- bit counter value.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 1, user can read CNT value for getting current 24- bit event input counter value.
    */
    __I  uint32_t CNT;

    /**
     * CAP
     * ===================================================================================================
     * Offset: 0x10  Timer0 Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, CAPIF (TIMERx_EINTSTS[0]) will set to 1 and the current timer counter value CNT (TIMERx_CNT[23:0]) will be auto-loaded into this CAPDAT field.
    */
    __I  uint32_t CAP;

    /**
     * EXTCTL
     * ===================================================================================================
     * Offset: 0x14  Timer0 External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Phase
     * |        |          |This bit indicates the detection phase of external counting pin TMx (x= 0~1).
     * |        |          |0 = A falling edge of external counting pin will be counted.
     * |        |          |1 = A rising edge of external counting pin will be counted.
     * |[2:1]   |CAPEDGE   |Timer External Capture Pin Edge Detection
     * |        |          |00 = A falling edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |01 = A rising edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |10 = Either rising or falling edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |CAPEN     |Timer External Capture Pin Enable Bit
     * |        |          |This bit enables the TMx_EXT pin.
     * |        |          |0 = TMx_EXT (x= 0~1) pin Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin Enabled.
     * |[4]     |CAPFUNCS  |Capture Function Select Bit
     * |        |          |0 = External Capture Mode Enabled.
     * |        |          |1 = External Reset Mode Enabled.
     * |        |          |Note1: When CAPFUNCS is 0, transition on TMx_EXT (x= 0~1) pin is using to save the 24-bit timer counter value to CAPDAT register.
     * |        |          |Note2: When CAPFUNCS is 1, transition on TMx_EXT (x= 0~1) pin is using to reset the 24-bit timer counter value.
     * |[5]     |CAPIEN    |Timer External Capture Interrupt Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~1) pin detection Interrupt Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin detection Interrupt Enabled.
     * |        |          |Note: CAPIEN is used to enable timer external interrupt.
     * |        |          |If CAPIEN enabled, timer will generate an interrupt when CAPIF (TIMERx_EINTSTS[0]) is .
     * |        |          |For example, while CAPIEN = 1, CAPEN = 1, and CAPEDGE = 00, an 1 to 0 transition on the TMx_EXT pin will cause the CAPIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[6]     |CAPDBEN   |Timer External Capture Pin De-bounce Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~1) pin de-bounce Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin de-bounce Enabled.
     * |        |          |Note1: If this bit is enabled, the edge detection of TMx_EXT pin is detected with de-bounce circuit.
     * |        |          |Note2: The de-bounce circuit doesn't support ACMP output.
     * |[7]     |CNTDBEN   |Timer Counter Pin De-bounce Enable Bit
     * |        |          |0 = TMx (x= 0~1) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~1) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * |[8]     |CAPSEL    |Capture Mode Select Bit
     * |        |          |0 = Timer counter reset function or free-counting mode of timer capture function.
     * |        |          |1 = Trigger-counting mode of timer capture function.
     * |[9]     |ACMPSSEL  |ACMP Source Selection to Trigger Capture Function
     * |        |          |For Timer 0:
     * |        |          |0 = Capture Function source is from ACMP0 output signal for TIMER.
     * |        |          |1 = Capture Function source is from ACMP1 output signal for TIMER.
    */
    __IO uint32_t EXTCTL;

    /**
     * EINTSTS
     * ===================================================================================================
     * Offset: 0x18  Timer0 External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer External Capture Interrupt Flag
     * |        |          |This bit indicates the timer external capture interrupt flag status.
     * |        |          |0 = TMx_EXT (x= 0~1) pin interrupt did not occur.
     * |        |          |1 = TMx_EXT (x= 0~1) pin interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~1) pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPIF status.
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
     * |[1]     |CAPF      |Timer External Capture Flag
     * |        |          |This bit indicates the timer external capture flag status.
     * |        |          |0 = TMx_EXT (x= 0~1) pin did not occur.
     * |        |          |1 = TMx_EXT (x= 0~1) pin occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~1) pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPF status.
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
*/
    __IO uint32_t EINTSTS;

} TIMER_T;

/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */

#define TIMER_CTL_PSC_Pos                  (0)                                               /*!< TIMER_T::CTL: PSC Position                  */
#define TIMER_CTL_PSC_Msk                  (0xfful << TIMER_CTL_PSC_Pos)                     /*!< TIMER_T::CTL: PSC Mask                      */

#define TIMER_CTL_CAPSRC_Pos               (19)                                              /*!< TIMER_T::CTL: CAPSRC Position               */
#define TIMER_CTL_CAPSRC_Msk               (0x1ul << TIMER_CTL_CAPSRC_Pos)                   /*!< TIMER_T::CTL: CAPSRC Mask                   */

#define TIMER_CTL_WKEN_Pos                 (23)                                              /*!< TIMER_T::CTL: WKEN Position                 */
#define TIMER_CTL_WKEN_Msk                 (0x1ul << TIMER_CTL_WKEN_Pos)                     /*!< TIMER_T::CTL: WKEN Mask                     */

#define TIMER_CTL_EXTCNTEN_Pos             (24)                                              /*!< TIMER_T::CTL: EXTCNTEN Position             */
#define TIMER_CTL_EXTCNTEN_Msk             (0x1ul << TIMER_CTL_EXTCNTEN_Pos)                 /*!< TIMER_T::CTL: EXTCNTEN Mask                 */

#define TIMER_CTL_ACTSTS_Pos               (25)                                              /*!< TIMER_T::CTL: ACTSTS Position               */
#define TIMER_CTL_ACTSTS_Msk               (0x1ul << TIMER_CTL_ACTSTS_Pos)                   /*!< TIMER_T::CTL: ACTSTS Mask                   */

#define TIMER_CTL_RSTCNT_Pos               (26)                                              /*!< TIMER_T::CTL: RSTCNT Position               */
#define TIMER_CTL_RSTCNT_Msk               (0x1ul << TIMER_CTL_RSTCNT_Pos)                   /*!< TIMER_T::CTL: RSTCNT Mask                   */

#define TIMER_CTL_OPMODE_Pos               (27)                                              /*!< TIMER_T::CTL: OPMODE Position               */
#define TIMER_CTL_OPMODE_Msk               (0x3ul << TIMER_CTL_OPMODE_Pos)                   /*!< TIMER_T::CTL: OPMODE Mask                   */

#define TIMER_CTL_INTEN_Pos                (29)                                              /*!< TIMER_T::CTL: INTEN Position                */
#define TIMER_CTL_INTEN_Msk                (0x1ul << TIMER_CTL_INTEN_Pos)                    /*!< TIMER_T::CTL: INTEN Mask                    */

#define TIMER_CTL_CNTEN_Pos                (30)                                              /*!< TIMER_T::CTL: CNTEN Position                */
#define TIMER_CTL_CNTEN_Msk                (0x1ul << TIMER_CTL_CNTEN_Pos)                    /*!< TIMER_T::CTL: CNTEN Mask                    */

#define TIMER_CTL_ICEDEBUG_Pos             (31)                                              /*!< TIMER_T::CTL: ICEDEBUG Position             */
#define TIMER_CTL_ICEDEBUG_Msk             (0x1ul << TIMER_CTL_ICEDEBUG_Pos)                 /*!< TIMER_T::CTL: ICEDEBUG Mask                 */

#define TIMER_CMP_CMPDAT_Pos               (0)                                               /*!< TIMER_T::CMP: CMPDAT Position               */
#define TIMER_CMP_CMPDAT_Msk               (0xfffffful << TIMER_CMP_CMPDAT_Pos)              /*!< TIMER_T::CMP: CMPDAT Mask                   */

#define TIMER_INTSTS_TIF_Pos               (0)                                               /*!< TIMER_T::INTSTS: TIF Position               */
#define TIMER_INTSTS_TIF_Msk               (0x1ul << TIMER_INTSTS_TIF_Pos)                   /*!< TIMER_T::INTSTS: TIF Mask                   */

#define TIMER_INTSTS_TWKF_Pos              (1)                                               /*!< TIMER_T::INTSTS: TWKF Position              */
#define TIMER_INTSTS_TWKF_Msk              (0x1ul << TIMER_INTSTS_TWKF_Pos)                  /*!< TIMER_T::INTSTS: TWKF Mask                  */

#define TIMER_INTSTS_TF_Pos              	 (2)                                               /*!< TIMER_T::INTSTS: TF Position              */
#define TIMER_INTSTS_TF_Msk              	 (0x1ul << TIMER_INTSTS_TF_Pos)                  	 /*!< TIMER_T::INTSTS: TF Mask  								*/

#define TIMER_CNT_CNT_Pos                  (0)                                               /*!< TIMER_T::CNT: CNT Position                  */
#define TIMER_CNT_CNT_Msk                  (0xfffffful << TIMER_CNT_CNT_Pos)                 /*!< TIMER_T::CNT: CNT Mask                      */

#define TIMER_CAP_CAPDAT_Pos               (0)                                               /*!< TIMER_T::CAP: CAPDAT Position               */
#define TIMER_CAP_CAPDAT_Msk               (0xfffffful << TIMER_CAP_CAPDAT_Pos)              /*!< TIMER_T::CAP: CAPDAT Mask                   */

#define TIMER_EXTCTL_CNTPHASE_Pos          (0)                                               /*!< TIMER_T::EXTCTL: CNTPHASE Position          */
#define TIMER_EXTCTL_CNTPHASE_Msk          (0x1ul << TIMER_EXTCTL_CNTPHASE_Pos)              /*!< TIMER_T::EXTCTL: CNTPHASE Mask              */

#define TIMER_EXTCTL_CAPEDGE_Pos           (1)                                               /*!< TIMER_T::EXTCTL: CAPEDGE Position           */
#define TIMER_EXTCTL_CAPEDGE_Msk           (0x3ul << TIMER_EXTCTL_CAPEDGE_Pos)               /*!< TIMER_T::EXTCTL: CAPEDGE Mask               */

#define TIMER_EXTCTL_CAPEN_Pos             (3)                                               /*!< TIMER_T::EXTCTL: CAPEN Position             */
#define TIMER_EXTCTL_CAPEN_Msk             (0x1ul << TIMER_EXTCTL_CAPEN_Pos)                 /*!< TIMER_T::EXTCTL: CAPEN Mask                 */

#define TIMER_EXTCTL_CAPFUNCS_Pos          (4)                                               /*!< TIMER_T::EXTCTL: CAPFUNCS Position          */
#define TIMER_EXTCTL_CAPFUNCS_Msk          (0x1ul << TIMER_EXTCTL_CAPFUNCS_Pos)              /*!< TIMER_T::EXTCTL: CAPFUNCS Mask              */

#define TIMER_EXTCTL_CAPIEN_Pos            (5)                                               /*!< TIMER_T::EXTCTL: CAPIEN Position            */
#define TIMER_EXTCTL_CAPIEN_Msk            (0x1ul << TIMER_EXTCTL_CAPIEN_Pos)                /*!< TIMER_T::EXTCTL: CAPIEN Mask                */

#define TIMER_EXTCTL_CAPDBEN_Pos           (6)                                               /*!< TIMER_T::EXTCTL: CAPDBEN Position           */
#define TIMER_EXTCTL_CAPDBEN_Msk           (0x1ul << TIMER_EXTCTL_CAPDBEN_Pos)               /*!< TIMER_T::EXTCTL: CAPDBEN Mask               */

#define TIMER_EXTCTL_CNTDBEN_Pos           (7)                                               /*!< TIMER_T::EXTCTL: CNTDBEN Position           */
#define TIMER_EXTCTL_CNTDBEN_Msk           (0x1ul << TIMER_EXTCTL_CNTDBEN_Pos)               /*!< TIMER_T::EXTCTL: CNTDBEN Mask               */

#define TIMER_EXTCTL_CAPSEL_Pos            (8)                                               /*!< TIMER_T::EXTCTL: CAPSEL Position            */
#define TIMER_EXTCTL_CAPSEL_Msk            (0x1ul << TIMER_EXTCTL_CAPSEL_Pos)                /*!< TIMER_T::EXTCTL: CAPSEL Mask                */

#define TIMER_EXTCTL_ACMPSSEL_Pos          (9)                                               /*!< TIMER_T::EXTCTL: ACMPSSEL Position          */
#define TIMER_EXTCTL_ACMPSSEL_Msk          (0x1ul << TIMER_EXTCTL_ACMPSSEL_Pos)              /*!< TIMER_T::EXTCTL: ACMPSSEL Mask              */

#define TIMER_EINTSTS_CAPIF_Pos            (0)                                               /*!< TIMER_T::EINTSTS: CAPIF Position            */
#define TIMER_EINTSTS_CAPIF_Msk            (0x1ul << TIMER_EINTSTS_CAPIF_Pos)                /*!< TIMER_T::EINTSTS: CAPIF Mask                */

#define TIMER_EINTSTS_CAPF_Pos             (1)                                               /*!< TIMER_T::EINTSTS: CAPF Position            */
#define TIMER_EINTSTS_CAPF_Msk             (0x1ul << TIMER_EINTSTS_CAPF_Pos)                 /*!< TIMER_T::EINTSTS: CAPF Mask                */


/**@}*/ /* TIMER_CONST */
/**@}*/ /* end of TIMER register group */









/*---------------------- General Purpose Input/Output Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup GP General Purpose Input/Output Controller(GP)
    Memory Mapped Structure for GP Controller
@{ */

typedef struct {


    /**
     * MODE
     * ===================================================================================================
     * Offset: 0x00  P0 I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MODE0     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3:2]   |MODE1     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[5:4]   |MODE2     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[7:6]   |MODE3     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[9:8]   |MODE4     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[11:10] |MODE5     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[13:12] |MODE6     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[15:14] |MODE7     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5..
    */
    __IO uint32_t MODE;

    /**
     * DINOFF
     * ===================================================================================================
     * Offset: 0x04  P0 Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |DINOFF0   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[17]    |DINOFF1   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[18]    |DINOFF2   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[19]    |DINOFF3   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[20]    |DINOFF4   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[21]    |DINOFF5   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[22]    |DINOFF6   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[23]    |DINOFF7   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DINOFF;

    /**
     * DOUT
     * ===================================================================================================
     * Offset: 0x08  P0 Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DOUT0     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DOUT1     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DOUT2     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DOUT3     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DOUT4     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DOUT5     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DOUT6     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DOUT7     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DOUT;

    /**
     * DATMSK
     * ===================================================================================================
     * Offset: 0x0C  P0 Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DATMSK0   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DATMSK1   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DATMSK2   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DATMSK3   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DATMSK4   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DATMSK5   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DATMSK6   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DATMSK7   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DATMSK;

    /**
     * PIN
     * ===================================================================================================
     * Offset: 0x10  P0 Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIN0      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |PIN1      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |PIN2      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |PIN3      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |PIN4      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |PIN5      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |PIN6      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |PIN7      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO  uint32_t PIN;

    /**
     * DBEN
     * ===================================================================================================
     * Offset: 0x14  P0 De-bounce Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBEN0     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DBEN1     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DBEN2     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DBEN3     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DBEN4     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DBEN5     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DBEN6     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DBEN7     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DBEN;

    /**
     * INTTYPE
     * ===================================================================================================
     * Offset: 0x18  P0 Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TYPE0     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |TYPE1     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |TYPE2     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |TYPE3     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |TYPE4     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |TYPE5     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |TYPE6     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |TYPE7     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTTYPE;

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x1C  P0 Interrupt Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN0    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |FLIEN1    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |FLIEN2    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |FLIEN3    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |FLIEN4    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |FLIEN5    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |FLIEN6    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |FLIEN7    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1function
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[16]    |RHIEN0    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[17]    |RHIEN1    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[18]    |RHIEN2    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[19]    |RHIEN3    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[20]    |RHIEN4    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[21]    |RHIEN5    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[22]    |RHIEN6    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[23]    |RHIEN7    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTEN;

    /**
     * INTSRC
     * ===================================================================================================
     * Offset: 0x20  P0 Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTSRC0   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |INTSRC1   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |INTSRC2   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |INTSRC3   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |INTSRC4   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |INTSRC5   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |INTSRC6   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |INTSRC7   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[8]     |INTSRC8   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[9]     |INTSRC9   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[10]    |INTSRC10  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[11]    |INTSRC11  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[12]    |INTSRC12  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[13]    |INTSRC13  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[14]    |INTSRC14  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[15]    |INTSRC15  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTSRC;

} GPIO_T;

typedef struct {

    /**
     * DBCTL
     * ===================================================================================================
     * Offset: 0x180  De-bounce Cycle Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clock.
     * |        |          |0001 = Sample interrupt input once per 2 clocks.
     * |        |          |0010 = Sample interrupt input once per 4 clocks.
     * |        |          |0011 = Sample interrupt input once per 8 clocks.
     * |        |          |0100 = Sample interrupt input once per 16 clocks.
     * |        |          |0101 = Sample interrupt input once per 32 clocks.
     * |        |          |0110 = Sample interrupt input once per 64 clocks.
     * |        |          |0111 = Sample interrupt input once per 128 clocks.
     * |        |          |1000 = Sample interrupt input once per 256 clocks.
     * |        |          |1001 = Sample interrupt input once per 2*256 clocks.
     * |        |          |1010 = Sample interrupt input once per 4*256 clocks.
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks.
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks.
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks.
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks.
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks.
     * |[4]     |DBCLKSRC  |De-bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is HCLK.
     * |        |          |1 = De-bounce counter clock source is 10 kHz internal low speed RC oscillator (LIRC).
    */
    __IO uint32_t DBCTL;

} GPIO_DB_T;


/**
    @addtogroup GP_CONST GP Bit Field Definition
    Constant Definitions for GP Controller
@{ */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_DBCTL_DBCLKSEL_Pos            (0)                                               /*!< GP_T::DBCTL: DBCLKSEL Position            */
#define GP_DBCTL_DBCLKSEL_Msk            (0xful << GP_DBCTL_DBCLKSEL_Pos)                  /*!< GP_T::DBCTL: DBCLKSEL Mask                */

#define GP_DBCTL_DBCLKSRC_Pos            (4)                                               /*!< GP_T::DBCTL: DBCLKSRC Position            */
#define GP_DBCTL_DBCLKSRC_Msk            (0x1ul << GP_DBCTL_DBCLKSRC_Pos)                  /*!< GP_T::DBCTL: DBCLKSRC Mask                */

/**@}*/ /* GP_CONST */
/**@}*/ /* end of GP register group */







/*---------------------- Analog to Digital Converter -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */

typedef struct {


    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x00  A/D Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]   |RESULT    |A/D Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_DAT register is rea.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwrote.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_DAT register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t DAT;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[7];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x20  A/D Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |A/D Converter Enable Bit
     * |        |          |0 = A/D Converter Disabled.
     * |        |          |1 = A/D Converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADCIEN    |A/D Interrupt Enable Bit
     * |        |          |A/D conversion end interrupt request is generated if ADCIEN bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[5:4]   |HWTRGSEL  |Hardware Trigger Source Select Bit
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |11 = A/D conversion is started by PWM trigger.
     * |        |          |Others = Reserved.
     * |        |          |Note: Software should disable TRGEN and SWTRG before change TRGS.
     * |[6]     |HWTRGCOND |Hardware External Trigger Condition
     * |        |          |This bit decides whether the external pin STADC trigger event is falling or raising edge.
     * |        |          |The signal must be kept at stable state at least 4 PCLKs at high and low state for edge trigge.
     * |        |          |0 = Falling edge.
     * |        |          |1 = Raising edge.
     * |[8]     |HWTRGEN   |Hardware External Trigger Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by external STADC pin.
     * |        |          |If external trigger is enabled, the SWTRG bit can be set to 1 by the selected hardware trigger sourc.
     * |        |          |0= External trigger Disabled.
     * |        |          |1= External trigger Enabled.
     * |[11]    |SWTRG     |Software Trigger A/D Conversion Start
     * |        |          |SWTRG bit can be set to 1 from two sources: software and external pin STADC.
     * |        |          |SWTRG will be cleared to 0 by hardware automatically after conversion complet.
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
    */
    __IO uint32_t CTL;

    /**
     * CHEN
     * ===================================================================================================
     * Offset: 0x24  A/D Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |Analog Input Channel 0 Enable Bit
     * |        |          |0 = Channel 0 Disabled.
     * |        |          |1 = Channel 0 Enabled.
     * |        |          |Note: If software enables more than one channel, the channel with the smallest number will be selected and the other enabled channels will be ignored.
     * |[1]     |CHEN1     |Analog Input Channel 1 Enable Bit
     * |        |          |0 = Channel 1 Disabled.
     * |        |          |1 = Channel 1 Enabled.
     * |[2]     |CHEN2     |Analog Input Channel 2 Enable Bit
     * |        |          |0 = Channel 2 Disabled.
     * |        |          |1 = Channel 2 Enabled.
     * |[3]     |CHEN3     |Analog Input Channel 3 Enable Bit
     * |        |          |0 = Channel 3 Disabled.
     * |        |          |1 = Channel 3 Enabled.
     * |[4]     |CHEN4     |Analog Input Channel 4 Enable Bit
     * |        |          |0 = Channel 4 Disabled.
     * |        |          |1 = Channel 4 Enabled.
     * |[5]     |CHEN5     |Analog Input Channel 5 Enable Bit
     * |        |          |0 = Channel 5 Disabled.
     * |        |          |1 = Channel 5 Enabled.
     * |[6]     |CHEN6     |Analog Input Channel 6 Enable Bit
     * |        |          |0 = Channel 6 Disabled.
     * |        |          |1 = Channel 6 Enabled.
     * |[7]     |CHEN7     |Analog Input Channel 7 Enable Bit
     * |        |          |0 = Channel 7 Disabled.
     * |        |          |1 = Channel 7 Enabled.
     * |[8]     |CHEN8     |Analog Input Channel 8 Enable Bit
     * |        |          |0 = Channel 8 Disabled.
     * |        |          |1 = Channel 8 Enabled.
     * |[9]     |CH8SEL    |Analog Input Channel 8 Selection
     * |        |          |0 = External analog input.
     * |        |          |1 = Internal band-gap voltage (VBG).
     * |        |          |Note: When software selects the band-gap voltage as the analog input source of ADC channel 8, the ADC clock rate needs to be limited to lower than 300 kHz.
    */
    __IO uint32_t CHEN;

    /**
     * CMP0
     * ===================================================================================================
     * Offset: 0x28  A/D Compare Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |A/D Compare Enable Bit
     * |        |          |Set 1 to this bit to enable comparing CMPDAT (ADC_CMPx[25:16]) with specified channel conversion results when converted data is loaded into the ADC_DAT register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |ADCMPIE   |A/D Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPIE bit will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |Set this field to select which channel's result to be compared.
     * |        |          |Note: Valid setting of this field is channel 0~7.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[27:16] |CMPDAT    |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
    */
    __IO uint32_t CMP0;

    /**
     * CMP1
     * ===================================================================================================
     * Offset: 0x2C  A/D Compare Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |A/D Compare Enable Bit
     * |        |          |Set 1 to this bit to enable comparing CMPDAT (ADC_CMPx[25:16]) with specified channel conversion results when converted data is loaded into the ADC_DAT register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |ADCMPIE   |A/D Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPIE bit will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |Set this field to select which channel's result to be compared.
     * |        |          |Note: Valid setting of this field is channel 0~7.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[25:16] |CMPDAT    |Comparison Data
     * |        |          |The 10-bit data is used to compare with conversion result of specified channel.
    */
    __IO uint32_t CMP1;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x30  A/D Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADIF      |A/D Conversion End Interrupt Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADIF is set to 1 When A/D conversion ends.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[1]     |ADCMPIF0   |A/D Compare interrupt Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP0, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP0 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP0 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[2]     |ADCMPIF1   |A/D Compare interrupt Flag 1
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP1, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP1 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP1 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[3]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is mirror of as SWTRG bit in ADC_CTL
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[6:4]   |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |This filed reflects the current conversion channel when BUSY=1.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * |[8]     |VALID     |Data Valid Flag (Read Only)
     * |        |          |It is a mirror of VALID bit in ADC_DAT register.
     * |[16]    |OV        |Overrun Flag (Read Only)
     * |        |          |It is a mirror to OV bit in ADC_DAT register.
     * |[24]    |ADCF      |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADIF is set to 1 When A/D conversion ends.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[25]     |ADCMPF0   |A/D Compare Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP0, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP0 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP0 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[26]     |ADCMPF1   |A/D Compare Flag 1
     * |        |          |When the selecte d channel A/D conversion result meets the setting condition in ADC_CMP1, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP1 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP1 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.

    */
    __IO uint32_t STATUS;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[4];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * TRGDLY
     * ===================================================================================================
     * Offset: 0x44  A/D Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DELAY     |PWM Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after PWM trigger.
     * |        |          |PWM trigger delay time is (4 * DELAY) * system clock.
    */
    __IO uint32_t TRGDLY;

    /**
     * EXTSMPT
     * ===================================================================================================
     * Offset: 0x48  A/D Sampling Time Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |EXTSMPT   |Additional ADC Sample Clock
     * |        |          |If the ADC input is unstable, user can set this register to increase the sampling time to get a stable ADC input signal.
     * |        |          |The default sampling time is 2 ADC clocks.
     * |        |          |The additional clock number will be inserted to lengthen the sampling clock.
     * |        |          |0 = disable adc.
     * |        |          |1 = Number of additional clock cycles is 1.
     * |        |          |2 = Number of additional clock cycles is 2.
     * |        |          |3 = Number of additional clock cycles is 3.
     * |        |          |4 = Number of additional clock cycles is 4.
     * |        |          |5 = Number of additional clock cycles is 5.
     * |        |          |6 = Number of additional clock cycles is 6.
     * |        |          |7 = Number of additional clock cycles is 7.
     * |        |          |8 = Number of additional clock cycles is 8.
     * |        |          |9 = Number of additional clock cycles is 9.
     * |        |          |... = Number of additional clock cycles is...
     * |        |          |1023 = Number of additional clock cycles is1023
    */
    __IO uint32_t EXTSMPT;

    /**
     * SEQCTL
     * ===================================================================================================
     * Offset: 0x4C  A/D PWM Sequential Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]    |SEQEN      |ADC Sequential Mode Enable Bit
     * |        |               |When ADC sequential mode is enabled, two of three ADC channels from 0 to 2 will automatically convert analog data in the sequence of channel [0, 1] or channel[1, 2] or channel[0, 2] defined by MODESEL (ADC_SEQCTL[3:2]).
     * |        |               |0 = ADC sequential mode Disabled.
     * |        |               |1 = ADC sequential mode Enabled.
     * |[1]    |SEQTYPE   |ADC Sequential Mode Type
     * |        |               |0 = ADC delay time is only inserted before the first conversion.
     * |        |               |The second conversion starts immediately after the first conversion is completed.
     * |        |               |(for 2/3-shunt type).
     * |        |               |1 = ADC delay time is inserted before each conversion. (for 1-shunt type)
     * |[3:2]   |MODESEL |ADC Sequential Mode Selection
     * |        |               |00 = Issue ADC_INT after Channel 0 then Channel 1 conversion finishes when SEQEN =1.
     * |        |               |01 = Issue ADC_INT after Channel 1 then Channel 2 conversion finishes when SEQEN =1.
     * |        |               |10 = Issue ADC_INT after Channel 0 then Channel 2 conversion finishes when SEQEN =1.
     * |        |               |11 = Reserved.
     * |[4]    |DELAY_EN  |ADC delay time inserted before 2nd conversion
     * |        |               |0 = ADC delay time only inserted before the 1st conversion;
     * |        |               |1 = ADC delay time inserted before each conversion.
     * |[5]    |TRG_SEL   |TRG1CTL or TRG2CTL select for 1-shunt sequential mode.
     * |        |               |0 = using TRG1CTL to trigger sequential conversion;
     * |        |               |1 = using TRG2CTL to trigger sequential conversion.
     * |[11:8]  |TRG1CTL   |PWM Trigger Source Selection For TRG1CTL[3:2]
     * |        |          |00 = PWM Trigger source is PWM0.
     * |        |          |01 = PWM Trigger source is PWM2.
     * |        |          |10 = PWM Trigger source is PWM4.
     * |        |          |11 = PWM Trigger source is PWM6..
     * |        |          |PWM Trigger Type Selection for TRG1CTL[1:0]
     * |        |          |00 = Rising of the selected PWM.
     * |        |          |01 = Center of the selected PWM.
     * |        |          |10 = Falling of the selected PWM.
     * |        |          |11 = Period of the selected PWM.
     * |        |          |Note: PWM trigger source is valid for 1-shunt and 2/3-shunt type.
     * |[19:16] |TRG2CTL   |PWM Trigger Source Selection For TRG2CTL[3:2]
     * |        |          |00 = PWM Trigger source is PWM0.
     * |        |          |01 = PWM Trigger source is PWM2.
     * |        |          |10 = PWM Trigger source is PWM4.
     * |        |          |11 = PWM Trigger source is PWM6..
     * |        |          |PWM Trigger Type Selection for TRG2CTL[1:0]
     * |        |          |00 = Rising of the selected PWM.
     * |        |          |01 = Center of the selected PWM.
     * |        |          |10 = Falling of the selected PWM.
     * |        |          |11 = Period of the selected PWM.
     * |        |          |Note: PWM trigger source is valid for 1-shunt type and 2/3-shunt type.
    */
    __IO uint32_t SEQCTL;

    /**
     * SEQDAT1
     * ===================================================================================================
     * Offset: 0x50  A/D PWM Sequential Mode First Result Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwritten.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t SEQDAT1;

    /**
     * SEQDAT2
     * ===================================================================================================
     * Offset: 0x54  A/D PWM Sequential Mode Second Result Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwritten.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the  ADC_SEQDATx; register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t SEQDAT2;

    /**
     * SEQDAT2
     * ===================================================================================================
     * Offset: 0x58  A/D analog parameters register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TESTMODE |A/D adc test mode enable
     * |         |          |0= test mode
     * |         |          |1= normal mode
     * |[1]     |SH_SEL | Default : 1
     * |         |          |0= External Sample Hold (SH) width with HOLD2(Internal Signal)
     * |         |          |1= SH is SH.
     * |[19:16] |CLKDIV    |ADC Clock Divider  Could not be set as 0.
     * |[25:24] |CMPCTL   |ADC comparator current control
     * |[27:26] |DRV_CTL  |ADC VCM buffer current control
    */
    __IO  uint32_t CTL2;		
    __IO  uint32_t LS_CTL;      /*OFFSET:0X5C*/
	__IO  uint32_t SUB_CTL;     /*OFFSET:0X60*/
	__I   uint32_t DATA_SUB;    /*OFFSET:0X64*/
	__IO  uint32_t FIFO_CTL;    /*OFFSET:0X68*/
	__IO  uint32_t BV_CTL;      /*OFFSET:0X6C*/
	__I   uint32_t POP_DATA;    /*OFFSET:0X70*/
}ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */
/*ADC_DAT*/
#define ADC_DAT_RESULT_Pos               (0)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_DAT_RESULT_Msk               (0xffful << ADC_DAT_RESULT_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_DAT_OV_Pos                   (16)                                              /*!< ADC_T::DAT: OV Position                   */
#define ADC_DAT_OV_Msk                   (0x1ul << ADC_DAT_OV_Pos)                         /*!< ADC_T::DAT: OV Mask                       */
#define ADC_DAT_VALID_Pos                (17)                                              /*!< ADC_T::DAT: VALID Position                */
#define ADC_DAT_VALID_Msk                (0x1ul << ADC_DAT_VALID_Pos)                      /*!< ADC_T::DAT: VALID Mask                    */
/*ADC_CTL*/
#define ADC_CTL_ADCEN_Pos                (0)                                               /*!< ADC_T::CTL: ADCEN Position                */
#define ADC_CTL_ADCEN_Msk                (0x1ul << ADC_CTL_ADCEN_Pos)                      /*!< ADC_T::CTL: ADCEN Mask                    */
#define ADC_CTL_ADCIEN_Pos               (1)                                               /*!< ADC_T::CTL: ADCIEN Position               */
#define ADC_CTL_ADCIEN_Msk               (0x1ul << ADC_CTL_ADCIEN_Pos)                     /*!< ADC_T::CTL: ADCIEN Mask                   */
#define ADC_CTL_HWTRGSEL_Pos             (4)                                               /*!< ADC_T::CTL: HWTRGSEL Position             */
#define ADC_CTL_HWTRGSEL_Msk             (0x3ul << ADC_CTL_HWTRGSEL_Pos)                   /*!< ADC_T::CTL: HWTRGSEL Mask                 */
#define ADC_CTL_HWTRGCOND_Pos            (6)                                               /*!< ADC_T::CTL: HWTRGCOND Position            */
#define ADC_CTL_HWTRGCOND_Msk            (0x1ul << ADC_CTL_HWTRGCOND_Pos)                  /*!< ADC_T::CTL: HWTRGCOND Mask                */
#define ADC_CTL_HWTRGEN_Pos              (8)                                               /*!< ADC_T::CTL: HWTRGEN Position              */
#define ADC_CTL_HWTRGEN_Msk              (0x1ul << ADC_CTL_HWTRGEN_Pos)                    /*!< ADC_T::CTL: HWTRGEN Mask                  */
#define ADC_CTL_SWTRG_Pos                (11)                                              /*!< ADC_T::CTL: SWTRG Position                */
#define ADC_CTL_SWTRG_Msk                (0x1ul << ADC_CTL_SWTRG_Pos)                      /*!< ADC_T::CTL: SWTRG Mask                    */
/*ADC_CHEN*/
#define ADC_CHEN_CHEN0_Pos               (0)                                               /*!< ADC_T::CHEN: CHEN0 Position               */
#define ADC_CHEN_CHEN0_Msk               (0x1ul << ADC_CHEN_CHEN0_Pos)                     /*!< ADC_T::CHEN: CHEN0 Mask                   */
#define ADC_CHEN_CHEN1_Pos               (1)                                               /*!< ADC_T::CHEN: CHEN1 Position               */
#define ADC_CHEN_CHEN1_Msk               (0x1ul << ADC_CHEN_CHEN1_Pos)                     /*!< ADC_T::CHEN: CHEN1 Mask                   */
#define ADC_CHEN_CHEN2_Pos               (2)                                               /*!< ADC_T::CHEN: CHEN2 Position               */
#define ADC_CHEN_CHEN2_Msk               (0x1ul << ADC_CHEN_CHEN2_Pos)                     /*!< ADC_T::CHEN: CHEN2 Mask                   */
#define ADC_CHEN_CHEN3_Pos               (3)                                               /*!< ADC_T::CHEN: CHEN3 Position               */
#define ADC_CHEN_CHEN3_Msk               (0x1ul << ADC_CHEN_CHEN3_Pos)                     /*!< ADC_T::CHEN: CHEN3 Mask                   */
#define ADC_CHEN_CHEN4_Pos               (4)                                               /*!< ADC_T::CHEN: CHEN4 Position               */
#define ADC_CHEN_CHEN4_Msk               (0x1ul << ADC_CHEN_CHEN4_Pos)                     /*!< ADC_T::CHEN: CHEN4 Mask                   */
#define ADC_CHEN_CHEN5_Pos               (5)                                               /*!< ADC_T::CHEN: CHEN5 Position               */
#define ADC_CHEN_CHEN5_Msk               (0x1ul << ADC_CHEN_CHEN5_Pos)                     /*!< ADC_T::CHEN: CHEN5 Mask                   */
#define ADC_CHEN_CHEN6_Pos               (6)                                               /*!< ADC_T::CHEN: CHEN6 Position               */
#define ADC_CHEN_CHEN6_Msk               (0x1ul << ADC_CHEN_CHEN6_Pos)                     /*!< ADC_T::CHEN: CHEN6 Mask                   */
#define ADC_CHEN_CHEN7_Pos               (7)                                               /*!< ADC_T::CHEN: CHEN7 Position               */
#define ADC_CHEN_CHEN7_Msk               (0x1ul << ADC_CHEN_CHEN7_Pos)                     /*!< ADC_T::CHEN: CHEN7 Mask                   */
#define ADC_CHEN_CH8_VBG_Pos           	 (8)                                               /*!< ADC_T::CHEN: BANDGAP CHANNEL Position               */
#define ADC_CHEN_CH8_VBG_Msk           	 (0x1ul << ADC_CHEN_CH8_VBG_Pos)                   /*!< ADC_T::CHEN: BANDGAP CHANNEL Mask                   */
#define ADC_CHEN_CH9_TMP_Pos             (9)                                               /*!< ADC_T::CHEN: TEMPERATURE Position              */
#define ADC_CHEN_CH9_TMP_Msk             (0x1ul << ADC_CHEN_CH9_TMP_Pos)                   /*!< ADC_T::CHEN: TEMPERATURE Mask                  */
#define ADC_CHEN_CH10_VDD_4_Pos          (10)                                              /*!< ADC_T::CHEN: CHANNEL VDD/4 Position              */
#define ADC_CHEN_CH10_VDD_4_Msk          (0x1ul << ADC_CHEN_CH10_VDD_4_Pos)                /*!< ADC_T::CHEN: CHANNEL VDD/4 Mask                  */
#define ADC_CHEN_CH11_FLOAT_Pos          (11)                                              /*!< ADC_T::CHEN: CHANNEL GND Position              */
#define ADC_CHEN_CH11_FLOAT_Msk          (0x1ul << ADC_CHEN_CH11_FLOAT_Pos)                /*!< ADC_T::CHEN: CHANNEL GND Mask                  */
#define ADC_CHEN_ALL_Msk              	 (0xFFFul)
/*ADC_CMP0*/
#define ADC_CMP0_ADCMPEN_Pos             (0)                                               /*!< ADC_T::CMP0: ADCMPEN Position             */
#define ADC_CMP0_ADCMPEN_Msk             (0x1ul << ADC_CMP0_ADCMPEN_Pos)                   /*!< ADC_T::CMP0: ADCMPEN Mask                 */
#define ADC_CMP0_ADCMPIE_Pos             (1)                                               /*!< ADC_T::CMP0: ADCMPIE Position             */
#define ADC_CMP0_ADCMPIE_Msk             (0x1ul << ADC_CMP0_ADCMPIE_Pos)                   /*!< ADC_T::CMP0: ADCMPIE Mask                 */
#define ADC_CMP0_CMPCOND_Pos             (2)                                               /*!< ADC_T::CMP0: CMPCOND Position             */
#define ADC_CMP0_CMPCOND_Msk             (0x1ul << ADC_CMP0_CMPCOND_Pos)                   /*!< ADC_T::CMP0: CMPCOND Mask                 */
#define ADC_CMP0_CMPCH_Pos               (3)                                               /*!< ADC_T::CMP0: CMPCH Position               */
#define ADC_CMP0_CMPCH_Msk               (0x7ul << ADC_CMP0_CMPCH_Pos)                     /*!< ADC_T::CMP0: CMPCH Mask                   */
#define ADC_CMP0_CMPMCNT_Pos             (8)                                               /*!< ADC_T::CMP0: CMPMCNT Position             */
#define ADC_CMP0_CMPMCNT_Msk             (0xful << ADC_CMP0_CMPMCNT_Pos)                   /*!< ADC_T::CMP0: CMPMCNT Mask                 */
#define ADC_CMP0_CMPDAT_Pos              (16)                                              /*!< ADC_T::CMP0: CMPDAT Position              */
#define ADC_CMP0_CMPDAT_Msk              (0xffful << ADC_CMP0_CMPDAT_Pos)                  /*!< ADC_T::CMP0: CMPDAT Mask                  */
/*ADC_CMP1*/
#define ADC_CMP1_ADCMPEN_Pos             (0)                                               /*!< ADC_T::CMP1: ADCMPEN Position             */
#define ADC_CMP1_ADCMPEN_Msk             (0x1ul << ADC_CMP1_ADCMPEN_Pos)                   /*!< ADC_T::CMP1: ADCMPEN Mask                 */
#define ADC_CMP1_ADCMPIE_Pos             (1)                                               /*!< ADC_T::CMP1: ADCMPIE Position             */
#define ADC_CMP1_ADCMPIE_Msk             (0x1ul << ADC_CMP1_ADCMPIE_Pos)                   /*!< ADC_T::CMP1: ADCMPIE Mask                 */
#define ADC_CMP1_CMPCOND_Pos             (2)                                               /*!< ADC_T::CMP1: CMPCOND Position             */
#define ADC_CMP1_CMPCOND_Msk             (0x1ul << ADC_CMP1_CMPCOND_Pos)                   /*!< ADC_T::CMP1: CMPCOND Mask                 */
#define ADC_CMP1_CMPCH_Pos               (3)                                               /*!< ADC_T::CMP1: CMPCH Position               */
#define ADC_CMP1_CMPCH_Msk               (0x7ul << ADC_CMP1_CMPCH_Pos)                     /*!< ADC_T::CMP1: CMPCH Mask                   */
#define ADC_CMP1_CMPMCNT_Pos             (8)                                               /*!< ADC_T::CMP1: CMPMCNT Position             */
#define ADC_CMP1_CMPMCNT_Msk             (0xful << ADC_CMP1_CMPMCNT_Pos)                   /*!< ADC_T::CMP1: CMPMCNT Mask                 */
#define ADC_CMP1_CMPDAT_Pos              (16)                                              /*!< ADC_T::CMP1: CMPDAT Position              */
#define ADC_CMP1_CMPDAT_Msk              (0xffful << ADC_CMP1_CMPDAT_Pos)                  /*!< ADC_T::CMP1: CMPDAT Mask                  */
/*ADC_STATUS*/
#define ADC_STATUS_ADIF_Pos              (0)                                               /*!< ADC_T::STATUS: ADIF Position              */
#define ADC_STATUS_ADIF_Msk              (0x1ul << ADC_STATUS_ADIF_Pos)                    /*!< ADC_T::STATUS: ADIF Mask                  */
#define ADC_STATUS_ADCMPIF0_Pos          (1)                                               /*!< ADC_T::STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPIF0_Msk          (0x1ul << ADC_STATUS_ADCMPIF0_Pos)                 /*!< ADC_T::STATUS: ADCMPF0 Mask               */
#define ADC_STATUS_ADCMPIF1_Pos          (2)                                               /*!< ADC_T::STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPIF1_Msk          (0x1ul << ADC_STATUS_ADCMPIF1_Pos)                 /*!< ADC_T::STATUS: ADCMPF1 Mask               */
#define ADC_STATUS_BUSY_Pos              (3)                                               /*!< ADC_T::STATUS: BUSY Position              */
#define ADC_STATUS_BUSY_Msk              (0x1ul << ADC_STATUS_BUSY_Pos)                    /*!< ADC_T::STATUS: BUSY Mask                  */
#define ADC_STATUS_CHANNEL_Pos           (4)                                               /*!< ADC_T::STATUS: CHANNEL Position           */
#define ADC_STATUS_CHANNEL_Msk           (0x7ul << ADC_STATUS_CHANNEL_Pos)                 /*!< ADC_T::STATUS: CHANNEL Mask               */
#define ADC_STATUS_VALID_Pos             (8)                                               /*!< ADC_T::STATUS: VALID Position             */
#define ADC_STATUS_VALID_Msk             (0x1ul << ADC_STATUS_VALID_Pos)                   /*!< ADC_T::STATUS: VALID Mask                 */
#define ADC_STATUS_INTMSK_FULL_Pos  	 (9)                                               /*!< ADC_T::STATUS: FIFO FULL INTERRUPT Position             */
#define ADC_STATUS_INTMSK_FULL_Msk  	 (0x1ul << ADC_STATUS_INTMSK_FULL_Pos)             /*!< ADC_T::STATUS: FIFO FULL INTERRUPT Mask                 */
#define ADC_STATUS_INTMSK_EMPTY_Pos 	 (10)                                              /*!< ADC_T::STATUS: FIFO EMPTY INTERRUPT Position             */
#define ADC_STATUS_INTMSK_EMPTY_Msk 	 (0x1ul << ADC_STATUS_INTMSK_EMPTY_Pos)            /*!< ADC_T::STATUS: FIFO EMPTY INTERRUPT Mask                 */
#define ADC_STATUS_INTMSK_OVER_Pos  	 (11)                                              /*!< ADC_T::STATUS: FIFO OVER INTERRUPT Position             */
#define ADC_STATUS_INTMSK_OVER_Msk  	 (0x1ul << ADC_STATUS_INTMSK_OVER_Pos)             /*!< ADC_T::STATUS: FIFO OVER INTERRUPT Mask                 */
#define ADC_STATUS_INTMSK_HALF_Pos  	 (12)                                              /*!< ADC_T::STATUS: FIFO HALF INTERRUPT Position             */
#define ADC_STATUS_INTMSK_HALF_Msk  	 (0x1ul << ADC_STATUS_INTMSK_HALF_Pos)             /*!< ADC_T::STATUS: FIFO HALF INTERRUPT Mask                 */
#define ADC_STATUS_INTMSK_AD_Pos  	 	 (13)                                              /*!< ADC_T::STATUS: A/D INTERRUPT MASK Position             */
#define ADC_STATUS_INTMSK_AD_Msk  	 	 (0x1ul << ADC_STATUS_INTMSK_AD_Pos)               /*!< ADC_T::STATUS: A/D INTERRUPT MASK Mask                 */
#define ADC_STATUS_INTMSK_CMP0_Pos  	 (14)                                              /*!< ADC_T::STATUS: A/D INTERRUPT MASK Position             */
#define ADC_STATUS_INTMSK_CMP0_Msk  	 (0x1ul << ADC_STATUS_INTMSK_CMP0_Pos)             
#define ADC_STATUS_INTMSK_CMP1_Pos  	 (15)                                              
#define ADC_STATUS_INTMSK_CMP1_Msk  	 (0x1ul << ADC_STATUS_INTMSK_CMP1_Pos)             
#define ADC_STATUS_OV_Pos                (16)                                              /*!< ADC_T::STATUS: OV Position                */
#define ADC_STATUS_OV_Msk                (0x1ul << ADC_STATUS_OV_Pos)                      /*!< ADC_T::STATUS: OV Mask                    */
#define ADC_STATUS_ONE_CH_CLR_SEL_Pos  	 (17)                                              /*!< ADC_T::STATUS: ADC ONE CHANNAL FLAG CLEAR SELECT Position             */
#define ADC_STATUS_ONE_CH_CLR_SEL_Msk  	 (0x1ul << ADC_STATUS_ONE_CH_CLR_SEL_Pos)          /*!< ADC_T::STATUS: ADC ONE CHANNAL FLAG CLEAR SELECT Mask                 */
#define ADC_STATUS_ONE_CH_FLAG_Pos 	 	 (18)                                              /*!< ADC_T::STATUS: ADC ONE CHANNAL FLAG Position             */
#define ADC_STATUS_ONE_CH_FLAG_Msk 	 	 (0x1ul << ADC_STATUS_ONE_CH_FLAG_Pos)             /*!< ADC_T::STATUS: ADC ONE CHANNAL FLAG Mask                 */
#define ADC_STATUS_INTFLG_FULL_Pos  	 (19)                                              /*!< ADC_T::STATUS: FIFO FULL INTERRUPT  Position             */
#define ADC_STATUS_INTFLG_FULL_Msk  	 (0x1ul << ADC_STATUS_INTFLG_FULL_Pos)             /*!< ADC_T::STATUS: FIFO FULL INTERRUPT  FLAG Mask                 */
#define ADC_STATUS_INTFLG_EMPTY_Pos 	 (20)                                              /*!< ADC_T::STATUS: FIFO EMPTY INTERRUPT FLAG Position             */
#define ADC_STATUS_INTFLG_EMPTY_Msk 	 (0x1ul << ADC_STATUS_INTFLG_EMPTY_Pos)            /*!< ADC_T::STATUS: FIFO EMPTY INTERRUPT FLAG Mask                 */
#define ADC_STATUS_INTFLG_OVER_Pos  	 (21)                                              /*!< ADC_T::STATUS: FIFO OVER INTERRUPT  FLAG Position             */
#define ADC_STATUS_INTFLG_OVER_Msk  	 (0x1ul << ADC_STATUS_INTFLG_OVER_Pos)             /*!< ADC_T::STATUS: FIFO OVER INTERRUPT  FLAG Mask                 */
#define ADC_STATUS_INTFLG_HALF_Pos  	 (22)                                              /*!< ADC_T::STATUS: FIFO HALF INTERRUPT  FLAG Position             */
#define ADC_STATUS_INTFLG_HALF_Msk  	 (0x1ul << ADC_STATUS_INTFLG_HALF_Pos)             /*!< ADC_T::STATUS: FIFO HALF INTERRUPT  FLAG Mask                 */
#define ADC_STATUS_ADCF_Pos              (24)                                               /*!< ADC_T::STATUS: ADIF Position              */
#define ADC_STATUS_ADCF_Msk              (0x1ul << ADC_STATUS_ADCF_Pos)                    /*!< ADC_T::STATUS: ADIF Mask                  */
#define ADC_STATUS_ADCMPF0_Pos           (25)                                               /*!< ADC_T::STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPF0_Msk           (0x1ul << ADC_STATUS_ADCMPF0_Pos)                 /*!< ADC_T::STATUS: ADCMPF0 Mask               */
#define ADC_STATUS_ADCMPF1_Pos           (26)                                               /*!< ADC_T::STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPF1_Msk           (0x1ul << ADC_STATUS_ADCMPF1_Pos)                 /*!< ADC_T::STATUS: ADCMPF1 Mask               */
#define ADC_STATUS_FLAG_FULL_Pos  	 	 (27)                                              /*!< ADC_T::STATUS: FIFO FULL   FLAG Position             */
#define ADC_STATUS_FLAG_FULL_Msk  	 	 (0x1ul << ADC_STATUS_FLAG_FULL_Pos)               /*!< ADC_T::STATUS: FIFO FULL   FLAG Mask                 */
#define ADC_STATUS_FLAG_EMPTY_Pos 	 	 (28)                                              /*!< ADC_T::STATUS: FIFO EMPTY  FLAG Position             */
#define ADC_STATUS_FLAG_EMPTY_Msk 	 	 (0x1ul << ADC_STATUS_FLAG_EMPTY_Pos)              /*!< ADC_T::STATUS: FIFO EMPTY  FLAG Mask                 */
#define ADC_STATUS_FLAG_OVER_Pos  	 	 (29)                                              /*!< ADC_T::STATUS: FIFO OVER   FLAG Position             */
#define ADC_STATUS_FLAG_OVER_Msk  	 	 (0x1ul << ADC_STATUS_FLAG_OVER_Pos)               /*!< ADC_T::STATUS: FIFO OVER   FLAG Mask                 */
#define ADC_STATUS_FLAG_HALF_Pos  	 	 (30)                                              /*!< ADC_T::STATUS: FIFO HALF   FLAG Position             */
#define ADC_STATUS_FLAG_HALF_Msk  	 	 (0x1ul << ADC_STATUS_FLAG_HALF_Pos)               /*!< ADC_T::STATUS: FIFO HALF   FLAG Mask                 */
/*ADC_TRGDLY*/
#define ADC_TRGDLY_DELAY_Pos             (0)                                               /*!< ADC_T::TRGDLY: DELAY Position             */
#define ADC_TRGDLY_DELAY_Msk             (0xfful << ADC_TRGDLY_DELAY_Pos)                  /*!< ADC_T::TRGDLY: DELAY Mask                 */
/*ADC_EXTSMPT*/
#define ADC_EXTSMPT_EXTSMPT_Pos          (0)                                               /*!< ADC_T::EXTSMPT: EXTSMPT Position          */
#define ADC_EXTSMPT_EXTSMPT_Msk          (0x3fful << ADC_EXTSMPT_EXTSMPT_Pos)                /*!< ADC_T::EXTSMPT: EXTSMPT Mask              */
/*ADC_SEQCTL*/
#define ADC_SEQCTL_SEQEN_Pos             (0)                                               /*!< ADC_T::SEQCTL: SEQEN Position             */
#define ADC_SEQCTL_SEQEN_Msk             (0x1ul << ADC_SEQCTL_SEQEN_Pos)                   /*!< ADC_T::SEQCTL: SEQEN Mask                 */
#define ADC_SEQCTL_SEQTYPE_Pos           (1)                                               /*!< ADC_T::SEQCTL: SEQTYPE Position           */
#define ADC_SEQCTL_SEQTYPE_Msk           (0x1ul << ADC_SEQCTL_SEQTYPE_Pos)                 /*!< ADC_T::SEQCTL: SEQTYPE Mask               */
#define ADC_SEQCTL_MODESEL_Pos           (2)                                               /*!< ADC_T::SEQCTL: MODESEL Position           */
#define ADC_SEQCTL_MODESEL_Msk           (0x3ul << ADC_SEQCTL_MODESEL_Pos)                 /*!< ADC_T::SEQCTL: MODESEL Mask               */
#define ADC_SEQCTL_DELAY_EN_Pos          (4)                                               /*!< ADC_T::SEQCTL: DELAY_EN Position           */
#define ADC_SEQCTL_DELAY_EN_Msk          (0x1ul << ADC_SEQCTL_DELAY_EN_Pos)                /*!< ADC_T::SEQCTL: DELAY_EN Mask               */
#define ADC_SEQCTL_TRG_SEL_Pos           (5)                                               /*!< ADC_T::SEQCTL: TRG_SEL Position           */
#define ADC_SEQCTL_TRG_SEL_Msk           (0x1ul << ADC_SEQCTL_TRG_SEL_Pos)                 /*!< ADC_T::SEQCTL: TRG_SEL Mask               */
#define ADC_SEQCTL_ONE_CH_EN_Pos         (6)                                               /*!< ADC_T::SEQCTL: PWM SEQUENCE IN ONE CHANNEL Position           */
#define ADC_SEQCTL_ONE_CH_EN_Msk         (0x1ul << ADC_SEQCTL_ONE_CH_EN_Pos)               /*!< ADC_T::SEQCTL: PWM SEQUENCE IN ONE CHANNEL Mask               */
#define ADC_SEQCTL_TRG1CTL_Pos           (8)                                               /*!< ADC_T::SEQCTL: TRG1CTL Position           */
#define ADC_SEQCTL_TRG1CTL_Msk           (0xful << ADC_SEQCTL_TRG1CTL_Pos)                 /*!< ADC_T::SEQCTL: TRG1CTL Mask               */
#define ADC_SEQCTL_ONE_CH_SEL_Pos        (12)                                              /*!< ADC_T::SEQCTL: PWM SEQUENCE CHANNEL SELECT Position           */
#define ADC_SEQCTL_ONE_CH_SEL_Msk        (0x7ul << ADC_SEQCTL_ONE_CH_SEL_Pos)              /*!< ADC_T::SEQCTL: PWM SEQUENCE CHANNEL SELECT Mask               */
#define ADC_SEQCTL_TRG2CTL_Pos           (16)                                              /*!< ADC_T::SEQCTL: TRG2CTL Position           */
#define ADC_SEQCTL_TRG2CTL_Msk           (0xful << ADC_SEQCTL_TRG2CTL_Pos)                 /*!< ADC_T::SEQCTL: TRG2CTL Mask               */
/*ADC_SEQDAT1*/
#define ADC_SEQDAT1_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT1: RESULT Position           */
#define ADC_SEQDAT1_RESULT_Msk           (0x3fful << ADC_SEQDAT1_RESULT_Pos)               /*!< ADC_T::SEQDAT1: RESULT Mask               */
#define ADC_SEQDAT1_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT1: OV Position               */
#define ADC_SEQDAT1_OV_Msk               (0x1ul << ADC_SEQDAT1_OV_Pos)                     /*!< ADC_T::SEQDAT1: OV Mask                   */
#define ADC_SEQDAT1_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT1: VALID Position            */
#define ADC_SEQDAT1_VALID_Msk            (0x1ul << ADC_SEQDAT1_VALID_Pos)                  /*!< ADC_T::SEQDAT1: VALID Mask                */
/*ADC_SEQDAT2*/
#define ADC_SEQDAT2_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT2: RESULT Position           */
#define ADC_SEQDAT2_RESULT_Msk           (0x3fful << ADC_SEQDAT2_RESULT_Pos)               /*!< ADC_T::SEQDAT2: RESULT Mask               */
#define ADC_SEQDAT2_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT2: OV Position               */
#define ADC_SEQDAT2_OV_Msk               (0x1ul << ADC_SEQDAT2_OV_Pos)                     /*!< ADC_T::SEQDAT2: OV Mask                   */
#define ADC_SEQDAT2_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT2: VALID Position            */
#define ADC_SEQDAT2_VALID_Msk            (0x1ul << ADC_SEQDAT2_VALID_Pos)                  /*!< ADC_T::SEQDAT2: VALID Mask                */
/*ADC_CTL2*/
#define ADC_CTL2_TESTMODE_Pos            (0)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_TESTMODE_Msk            (0x1ul << ADC_CTL2_TESTMODE_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_CTL2_SHSEL_Pos            	 (1)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_SHSEL_Msk            	 (0x1ul << ADC_CTL2_SHSEL_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_CTL2_DMA_EN_Pos            	 (2)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_DMA_EN_Msk            	 (0x1ul << ADC_CTL2_DMA_EN_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_CTL2_CLKDIV_Pos              (8)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_CLKDIV_Msk              (0x7ul << ADC_CTL2_CLKDIV_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_SEL_VREF_Pos                 (17)                                              /*!< ADC_T::SEL_VREF Position          */
#define ADC_SEL_VREF_Msk                 (0x1ul << ADC_SEL_VREF_Pos)                       /*!< ADC_T::SEL_VREF Mask              */
#define ADC_CTL2_CMPCTL_Pos              (24)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_CMPCTL_Msk              (0x3ul << ADC_CTL2_CMPCTL_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */
#define ADC_CTL2_DRVCTL_Pos              (26)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_DRVCTL_Msk              (0x3ul << ADC_CTL2_DRVCTL_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */
/*LS_CTL*/
#define ADC_LS_CTL_LS_EN_Pos             (0)                                               /*!< ADC_T::LEFT_SHIFT EN Position               */
#define ADC_LS_CTL_LS_EN_Msk             (0x1ul << ADC_LS_CTL_LS_EN_Pos)                   /*!< ADC_T::LEFT_SHIFT EN Mask                   */
#define ADC_LS_CTL_LS_DAT_Pos            (16)                                              /*!< ADC_T::LEFT_SHIFT DATA Position               */
#define ADC_LS_CTL_LS_DAT_Msk            (0xFFFFul << ADC_LS_CTL_LS_DAT_Pos)               /*!< ADC_T::LEFT_SHIFT DATA Mask                   */
/*SUB_CTL*/
#define ADC_SUB_CTL_BIAS_EN_Pos          (0)                                               /*!< ADC_T::BIAS EN Position               */
#define ADC_SUB_CTL_BIAS_EN_Msk          (0x1ul << ADC_SUB_CTL_BIAS_EN_Pos)                /*!< ADC_T::BIAS EN Mask                   */
#define ADC_SUB_CTL_BIAS_DAT_Pos         (16)                                              /*!< ADC_T::BIAS DATA Position               */
#define ADC_SUB_CTL_BIAS_DAT_Msk         (0xFFFFul << ADC_SUB_CTL_BIAS_DAT_Pos)            /*!< ADC_T::BIAS DATA Mask                   */
/*DATA_SUB*/
#define ADC_DAT_SUB_LEFT_BIAS_Pos        (0)                                               /*!< ADC_T::DATA LEFT_BIAS Position               */
#define ADC_DAT_SUB_LEFT_BIAS_Msk        (0x1FFFFul << ADC_DAT_SUB_LEFT_BIAS_Pos)           /*!< ADC_T::DATA LEFT_BIAS Mask                   */
/*FIFO_CTL*/
#define ADC_FIFO_CTL_EN_Pos          	 (0)                                               /*!< ADC_T::FIFO EN Position               */
#define ADC_FIFO_CTL_EN_Msk          	 (0x1ul << ADC_FIFO_CTL_EN_Pos)                    /*!< ADC_T::FIFO EN Mask                   */
#define ADC_FIFO_CTL_HALF_OR_FULL_Pos    (1)                                               /*!< ADC_T::FIFO HALF OR FULL REQ Position               */
#define ADC_FIFO_CTL_HALF_OR_FULL_Msk    (0x1ul << ADC_FIFO_CTL_HALF_OR_FULL_Pos)          /*!< ADC_T::FIFO HALF OR FULL REQ Mask                   */
/*BV_CTL*/

/*POP_DATA*/
#define ADC_FIFO_POP_DAT_Pos    		 (0)                                               /*!< ADC_T::FIFO POP DATA Position               */
#define ADC_FIFO_POP_DAT_Msk    		 (0x3FFul << ADC_FIFO_POP_DAT_Pos)                 /*!< ADC_T::FIFO POP DATA Mask                   */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */






/*---------------------- System Manger Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */

typedef struct {
    __IO uint32_t P0_MFP;           /*!< P0 multi function and input type control register */
    __IO uint32_t P1_MFP;           /*!< P1 multi function and input type control register */
    __IO uint32_t P2_MFP;           /*!< P2 multi function and input type control register */
    __IO uint32_t P3_MFP;           /*!< P3 multi function and input type control register */
    __IO uint32_t P4_MFP;           /*!< P4 multi function and input type control register */
    __IO uint32_t P5_MFP;           /*!< P5 multi function and input type control register */
    __IO  uint32_t RESV[10];        /*!< Reversed register */
    __O  uint32_t REGLCTL;          /*!< Register write protection control */
    __IO  uint32_t STATUS;          /*!< Rom status register */
    __IO  uint32_t CTRL0;           /*!< Dma channel select register */
} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */


#define SYS_P0_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P0_MFP: MFP Position               */
#define SYS_P0_MFP_MFP_Msk               (0xfful << SYS_P0_MFP_MFP_Pos)                    /*!< SYS_T::P0_MFP: MFP Mask                   */

#define SYS_P0_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P0_MFP: ALT0 Position              */
#define SYS_P0_MFP_ALT0_Msk              (0x1ul << SYS_P0_MFP_ALT0_Pos)                    /*!< SYS_T::P0_MFP: ALT0 Mask                  */

#define SYS_P0_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P0_MFP: ALT1 Position              */
#define SYS_P0_MFP_ALT1_Msk              (0x1ul << SYS_P0_MFP_ALT1_Pos)                    /*!< SYS_T::P0_MFP: ALT1 Mask                  */

#define SYS_P0_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P0_MFP: ALT2 Position              */
#define SYS_P0_MFP_ALT2_Msk              (0x1ul << SYS_P0_MFP_ALT2_Pos)                    /*!< SYS_T::P0_MFP: ALT2 Mask                  */

#define SYS_P0_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P0_MFP: ALT3 Position              */
#define SYS_P0_MFP_ALT3_Msk              (0x1ul << SYS_P0_MFP_ALT3_Pos)                    /*!< SYS_T::P0_MFP: ALT3 Mask                  */

#define SYS_P0_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P0_MFP: ALT4 Position              */
#define SYS_P0_MFP_ALT4_Msk              (0x1ul << SYS_P0_MFP_ALT4_Pos)                    /*!< SYS_T::P0_MFP: ALT4 Mask                  */

#define SYS_P0_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P0_MFP: ALT5 Position              */
#define SYS_P0_MFP_ALT5_Msk              (0x1ul << SYS_P0_MFP_ALT5_Pos)                    /*!< SYS_T::P0_MFP: ALT5 Mask                  */

#define SYS_P0_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P0_MFP: ALT6 Position              */
#define SYS_P0_MFP_ALT6_Msk              (0x1ul << SYS_P0_MFP_ALT6_Pos)                    /*!< SYS_T::P0_MFP: ALT6 Mask                  */

#define SYS_P0_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P0_MFP: ALT7 Position              */
#define SYS_P0_MFP_ALT7_Msk              (0x1ul << SYS_P0_MFP_ALT7_Pos)                    /*!< SYS_T::P0_MFP: ALT7 Mask                  */

#define SYS_P1_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P1_MFP: MFP Position               */
#define SYS_P1_MFP_MFP_Msk               (0xfful << SYS_P1_MFP_MFP_Pos)                    /*!< SYS_T::P1_MFP: MFP Mask                   */

#define SYS_P1_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P1_MFP: ALT0 Position              */
#define SYS_P1_MFP_ALT0_Msk              (0x1ul << SYS_P1_MFP_ALT0_Pos)                    /*!< SYS_T::P1_MFP: ALT0 Mask                  */

#define SYS_P1_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P1_MFP: ALT1 Position              */
#define SYS_P1_MFP_ALT1_Msk              (0x1ul << SYS_P1_MFP_ALT1_Pos)                    /*!< SYS_T::P1_MFP: ALT1 Mask                  */

#define SYS_P1_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P1_MFP: ALT2 Position              */
#define SYS_P1_MFP_ALT2_Msk              (0x1ul << SYS_P1_MFP_ALT2_Pos)                    /*!< SYS_T::P1_MFP: ALT2 Mask                  */

#define SYS_P1_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P1_MFP: ALT3 Position              */
#define SYS_P1_MFP_ALT3_Msk              (0x1ul << SYS_P1_MFP_ALT3_Pos)                    /*!< SYS_T::P1_MFP: ALT3 Mask                  */

#define SYS_P1_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P1_MFP: ALT4 Position              */
#define SYS_P1_MFP_ALT4_Msk              (0x1ul << SYS_P1_MFP_ALT4_Pos)                    /*!< SYS_T::P1_MFP: ALT4 Mask                  */

#define SYS_P1_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P1_MFP: ALT5 Position              */
#define SYS_P1_MFP_ALT5_Msk              (0x1ul << SYS_P1_MFP_ALT5_Pos)                    /*!< SYS_T::P1_MFP: ALT5 Mask                  */

#define SYS_P1_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P1_MFP: ALT6 Position              */
#define SYS_P1_MFP_ALT6_Msk              (0x1ul << SYS_P1_MFP_ALT6_Pos)                    /*!< SYS_T::P1_MFP: ALT6 Mask                  */

#define SYS_P1_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P1_MFP: ALT7 Position              */
#define SYS_P1_MFP_ALT7_Msk              (0x1ul << SYS_P1_MFP_ALT7_Pos)                    /*!< SYS_T::P1_MFP: ALT7 Mask                  */

#define SYS_P1_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P1_MFP: TYPE Position              */
#define SYS_P1_MFP_TYPE_Msk              (0xfful << SYS_P1_MFP_TYPE_Pos)                   /*!< SYS_T::P1_MFP: TYPE Mask                  */

#define SYS_P1_MFP_P12EXT_Pos            (26)                                              /*!< SYS_T::P1_MFP: P12EXT Position            */
#define SYS_P1_MFP_P12EXT_Msk            (0x1ul << SYS_P1_MFP_P12EXT_Pos)                  /*!< SYS_T::P1_MFP: P12EXT Mask                */

#define SYS_P1_MFP_P13EXT_Pos            (27)                                              /*!< SYS_T::P1_MFP: P13EXT Position            */
#define SYS_P1_MFP_P13EXT_Msk            (0x1ul << SYS_P1_MFP_P13EXT_Pos)                  /*!< SYS_T::P1_MFP: P13EXT Mask                */

#define SYS_P1_MFP_P14EXT_Pos            (28)                                              /*!< SYS_T::P1_MFP: P14EXT Position            */
#define SYS_P1_MFP_P14EXT_Msk            (0x1ul << SYS_P1_MFP_P14EXT_Pos)                  /*!< SYS_T::P1_MFP: P14EXT Mask                */


#define SYS_P2_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P2_MFP: ALT0 Position              */
#define SYS_P2_MFP_ALT0_Msk              (0x1ul << SYS_P2_MFP_ALT0_Pos)                    /*!< SYS_T::P2_MFP: ALT0 Mask                  */

#define SYS_P2_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P2_MFP: ALT1 Position              */
#define SYS_P2_MFP_ALT1_Msk              (0x1ul << SYS_P2_MFP_ALT1_Pos)                    /*!< SYS_T::P2_MFP: ALT1 Mask                  */


#define SYS_P2_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P2_MFP: ALT2 Position              */
#define SYS_P2_MFP_ALT2_Msk              (0x1ul << SYS_P2_MFP_ALT2_Pos)                    /*!< SYS_T::P2_MFP: ALT2 Mask                  */

#define SYS_P2_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P2_MFP: ALT3 Position              */
#define SYS_P2_MFP_ALT3_Msk              (0x1ul << SYS_P2_MFP_ALT3_Pos)                    /*!< SYS_T::P2_MFP: ALT3 Mask                  */

#define SYS_P2_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P2_MFP: ALT4 Position              */
#define SYS_P2_MFP_ALT4_Msk              (0x1ul << SYS_P2_MFP_ALT4_Pos)                    /*!< SYS_T::P2_MFP: ALT4 Mask                  */

#define SYS_P2_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P2_MFP: ALT5 Position              */
#define SYS_P2_MFP_ALT5_Msk              (0x1ul << SYS_P2_MFP_ALT5_Pos)                    /*!< SYS_T::P2_MFP: ALT5 Mask                  */

#define SYS_P2_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P2_MFP: ALT6 Position              */
#define SYS_P2_MFP_ALT6_Msk              (0x1ul << SYS_P2_MFP_ALT6_Pos)                    /*!< SYS_T::P2_MFP: ALT6 Mask                  */


#define SYS_P2_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P2_MFP: ALT7 Position              */
#define SYS_P2_MFP_ALT7_Msk              (0x1ul << SYS_P2_MFP_ALT7_Pos)                    /*!< SYS_T::P2_MFP: ALT7 Mask                  */


#define SYS_P3_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P3_MFP: MFP Position               */
#define SYS_P3_MFP_MFP_Msk               (0xfful << SYS_P3_MFP_MFP_Pos)                    /*!< SYS_T::P3_MFP: MFP Mask                   */

#define SYS_P3_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P3_MFP: ALT0 Position              */
#define SYS_P3_MFP_ALT0_Msk              (0x1ul << SYS_P3_MFP_ALT0_Pos)                    /*!< SYS_T::P3_MFP: ALT0 Mask                  */

#define SYS_P3_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P3_MFP: ALT1 Position              */
#define SYS_P3_MFP_ALT1_Msk              (0x1ul << SYS_P3_MFP_ALT1_Pos)                    /*!< SYS_T::P3_MFP: ALT1 Mask                  */

#define SYS_P3_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P3_MFP: ALT2 Position              */
#define SYS_P3_MFP_ALT2_Msk              (0x1ul << SYS_P3_MFP_ALT2_Pos)                    /*!< SYS_T::P3_MFP: ALT2 Mask                  */

#define SYS_P3_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P3_MFP: ALT4 Position              */
#define SYS_P3_MFP_ALT4_Msk              (0x1ul << SYS_P3_MFP_ALT4_Pos)                    /*!< SYS_T::P3_MFP: ALT4 Mask                  */

#define SYS_P3_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P3_MFP: ALT5 Position              */
#define SYS_P3_MFP_ALT5_Msk              (0x1ul << SYS_P3_MFP_ALT5_Pos)                    /*!< SYS_T::P3_MFP: ALT5 Mask                  */

#define SYS_P3_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P3_MFP: ALT6 Position              */
#define SYS_P3_MFP_ALT6_Msk              (0x1ul << SYS_P3_MFP_ALT6_Pos)                    /*!< SYS_T::P3_MFP: ALT6 Mask                  */

#define SYS_P3_MFP_P32EXT_Pos            (26)                                              /*!< SYS_T::P3_MFP: P32EXT Position            */
#define SYS_P3_MFP_P32EXT_Msk            (0x1ul << SYS_P3_MFP_P32EXT_Pos)                  /*!< SYS_T::P3_MFP: P32EXT Mask                */

#define SYS_P4_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P4_MFP: MFP Position               */
#define SYS_P4_MFP_MFP_Msk               (0xfful << SYS_P4_MFP_MFP_Pos)                    /*!< SYS_T::P4_MFP: MFP Mask                   */

#define SYS_P4_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P4_MFP: ALT6 Position              */
#define SYS_P4_MFP_ALT6_Msk              (0x1ul << SYS_P4_MFP_ALT6_Pos)                    /*!< SYS_T::P4_MFP: ALT6 Mask                  */

#define SYS_P4_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P4_MFP: ALT7 Position              */
#define SYS_P4_MFP_ALT7_Msk              (0x1ul << SYS_P4_MFP_ALT7_Pos)                    /*!< SYS_T::P4_MFP: ALT7 Mask                  */

#define SYS_P5_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P5_MFP: MFP Position               */
#define SYS_P5_MFP_MFP_Msk               (0xfful << SYS_P5_MFP_MFP_Pos)                    /*!< SYS_T::P5_MFP: MFP Mask                   */

#define SYS_P5_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P5_MFP: ALT0 Position              */
#define SYS_P5_MFP_ALT0_Msk              (0x1ul << SYS_P5_MFP_ALT0_Pos)                    /*!< SYS_T::P5_MFP: ALT0 Mask                  */

#define SYS_P5_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P5_MFP: ALT1 Position              */
#define SYS_P5_MFP_ALT1_Msk              (0x1ul << SYS_P5_MFP_ALT1_Pos)                    /*!< SYS_T::P5_MFP: ALT1 Mask                  */

#define SYS_P5_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P5_MFP: ALT2 Position              */
#define SYS_P5_MFP_ALT2_Msk              (0x1ul << SYS_P5_MFP_ALT2_Pos)                    /*!< SYS_T::P5_MFP: ALT2 Mask                  */

#define SYS_P5_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P5_MFP: ALT3 Position              */
#define SYS_P5_MFP_ALT3_Msk              (0x1ul << SYS_P5_MFP_ALT3_Pos)                    /*!< SYS_T::P5_MFP: ALT3 Mask                  */

#define SYS_P5_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P5_MFP: ALT4 Position              */
#define SYS_P5_MFP_ALT4_Msk              (0x1ul << SYS_P5_MFP_ALT4_Pos)                    /*!< SYS_T::P5_MFP: ALT4 Mask                  */

#define SYS_P5_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P5_MFP: ALT5 Position              */
#define SYS_P5_MFP_ALT5_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT5 Mask                  */

#define SYS_P5_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P5_MFP: ALT6 Position              */
#define SYS_P5_MFP_ALT6_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT6 Mask                  */

#define SYS_P5_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P5_MFP: ALT7 Position              */
#define SYS_P5_MFP_ALT7_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT7 Mask                  */

#define SYS_P5_MFP_P50EXT_Pos              (24)                                              /*!< SYS_T::P5_MFP: P50EXT Position              */
#define SYS_P5_MFP_P50EXT_Msk              (0x1ul << SYS_P5_MFP_P50EXT_Pos)                    /*!< SYS_T::P5_MFP: P50EXT Mask                  */

#define SYS_P5_MFP_P51EXT_Pos              (25)                                              /*!< SYS_T::P5_MFP: P51EXT Position              */
#define SYS_P5_MFP_P51EXT_Msk              (0x1ul << SYS_P5_MFP_P51EXT_Pos)                    /*!< SYS_T::P5_MFP: P51EXT Mask                  */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position          */
#define SYS_REGLCTL_REGLCTL_Msk          (0xfful << SYS_REGLCTL_REGLCTL_Pos)               /*!< SYS_T::REGLCTL: REGLCTL Mask              */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */








/**
  * @}
  */
/*---------------------- System Clock Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct {
    __IO uint32_t RSTSTS;               //0x40040000
    __IO uint32_t IPRST0;               //0x40040004
	__IO uint32_t IPRST1;               //0x40040008
	__IO uint32_t BODCTL;               //0x4004000C
	__IO uint32_t BLDBCTL;              //0x40040010
    __IO uint32_t RSV[3];
	__IO uint32_t CLK_TOP_CTRL;         //0x40040020
	__IO uint32_t RCL_CTRL;             //0x40040024
	__IO uint32_t RCH_CTRL;             //0x40040028
	__IO uint32_t XTL_CTRL;             //0x4004002C
	__IO uint32_t XTH_CTRL;             //0x40040030  
	__IO uint32_t DPLL_CTRL;            //0x40040034
	__IO uint32_t AHB_CLK_CTRL;         //0x40040038
	__IO uint32_t APB1_CLK_CTRL;        //0x4004003C
	__IO uint32_t APB1_CLK_CTRL1;       //0x40040040
	__IO uint32_t APB2_CLK_CTRL;        //0x40040044
    __IO uint32_t APB2_CLK_CTRL1;       //0x40040048
    __IO uint32_t ACT_32K_CTRL;         //0x4004004C
    __IO uint32_t ACT_32K_BASECORR;     //0x40040050
} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

/*RSTSTS*/
#define CLK_RSTSTS_CHIPRF_Pos            (0)                                          
#define CLK_RSTSTS_CHIPRF_Msk            (0x1ul << CLK_RSTSTS_CHIPRF_Pos)             
#define CLK_RSTSTS_PINRF_Pos             (1)                                           
#define CLK_RSTSTS_PINRF_Msk             (0x1ul << CLK_RSTSTS_PINRF_Pos)               
#define CLK_RSTSTS_WDTRF_Pos             (2)                                           
#define CLK_RSTSTS_WDTRF_Msk             (0x1ul << CLK_RSTSTS_WDTRF_Pos)               
#define CLK_RSTSTS_LVRRF_Pos             (3)                                           
#define CLK_RSTSTS_LVRRF_Msk             (0x1ul << CLK_RSTSTS_LVRRF_Pos)               
#define CLK_RSTSTS_BODRF_Pos             (4)                                           
#define CLK_RSTSTS_BODRF_Msk             (0x1ul << CLK_RSTSTS_BODRF_Pos)               
#define CLK_RSTSTS_SYSRF_Pos             (5)                                           
#define CLK_RSTSTS_SYSRF_Msk             (0x1ul << CLK_RSTSTS_SYSRF_Pos)               
#define CLK_RSTSTS_CPURF_Pos             (7)                                           
#define CLK_RSTSTS_CPURF_Msk             (0x1ul << CLK_RSTSTS_CPURF_Pos)               

/*IPRST0*/
#define CLK_IPRST0_CHIPRST_Pos           (0)                                           
#define CLK_IPRST0_CHIPRST_Msk           (0x1ul << CLK_IPRST0_CHIPRST_Pos)             
#define CLK_IPRST0_CPURST_Pos            (1)                                           
#define CLK_IPRST0_CPURST_Msk            (0x1ul << CLK_IPRST0_CPURST_Pos)              
#define CLK_IPRST0_DMARST_Pos            (2)                                           
#define CLK_IPRST0_DMARST_Msk            (0x1ul << CLK_IPRST0_DMARST_Pos)              
#define CLK_IPRST0_RFRST_Pos             (3)                                           
#define CLK_IPRST0_RFRST_Msk             (0x1ul << CLK_IPRST0_RFRST_Pos)              
#define CLK_IPRST0_EFUSERST_Pos          (4)                                           
#define CLK_IPRST0_EFUSERST_Msk          (0x1ul << CLK_IPRST0_EFUSERST_Pos) 
#define CLK_IPRST0_ECCRST_Pos            (5)                                           
#define CLK_IPRST0_ECCRST_Msk            (0x1ul << CLK_IPRST0_ECCRST_Pos) 
#define CLK_IPRST0_USBRST_Pos            (6)                                           
#define CLK_IPRST0_USBRST_Msk            (0x1ul << CLK_IPRST0_USBRST_Pos) 
#define CLK_IPRST0_MDMSTDBYRST_Pos       (7)                                           
#define CLK_IPRST0_MDMSTDBYRST_Msk       (0x1ul << CLK_IPRST0_MDMSTDBYRST_Pos) 
#define CLK_IPRST0_MDMRST_Pos            (8)                                           
#define CLK_IPRST0_MDMRST_Msk            (0x1ul << CLK_IPRST0_MDMRST_Pos) 

/*IPRST1*/
#define CLK_IPRST1_I2C0RST_Pos           (0)                                          
#define CLK_IPRST1_I2C0RST_Msk           (0x1ul << CLK_IPRST1_I2C0RST_Pos)            
#define CLK_IPRST1_SPI0RST_Pos           (2)                                          
#define CLK_IPRST1_SPI0RST_Msk           (0x1ul << CLK_IPRST1_SPI0RST_Pos)            
#define CLK_IPRST1_SPI1RST_Pos           (3)                                          
#define CLK_IPRST1_SPI1RST_Msk           (0x1ul << CLK_IPRST1_SPI1RST_Pos)            
#define CLK_IPRST1_UART0RST_Pos          (6)                                          
#define CLK_IPRST1_UART0RST_Msk          (0x1ul << CLK_IPRST1_UART0RST_Pos)           
#define CLK_IPRST1_UART1RST_Pos          (7)                                          
#define CLK_IPRST1_UART1RST_Msk          (0x1ul << CLK_IPRST1_UART1RST_Pos)           
#define CLK_IPRST1_PWM0RST_Pos           (8)                                          
#define CLK_IPRST1_PWM0RST_Msk           (0x1ul << CLK_IPRST1_PWM0RST_Pos)            
#define CLK_IPRST1_ADCRST_Pos            (9)                                          
#define CLK_IPRST1_ADCRST_Msk            (0x1ul << CLK_IPRST1_ADCRST_Pos)             
#define CLK_IPRST1_WDTRST_Pos            (10)                                         
#define CLK_IPRST1_WDTRST_Msk            (0x1ul << CLK_IPRST1_WDTRST_Pos)             
#define CLK_IPRST1_WWDTRST_Pos           (11)                                         
#define CLK_IPRST1_WWDTRST_Msk           (0x1ul << CLK_IPRST1_WWDTRST_Pos)            
#define CLK_IPRST1_TMR0RST_Pos           (12)                                         
#define CLK_IPRST1_TMR0RST_Msk           (0x1ul << CLK_IPRST1_TMR0RST_Pos)            
#define CLK_IPRST1_TMR1RST_Pos           (13)                                         
#define CLK_IPRST1_TMR1RST_Msk           (0x1ul << CLK_IPRST1_TMR1RST_Pos)            
#define CLK_IPRST1_TMR2RST_Pos           (14)                                         
#define CLK_IPRST1_TMR2RST_Msk           (0x1ul << CLK_IPRST1_TMR2RST_Pos)            
#define CLK_IPRST1_GPIORST_Pos           (15)                                         
#define CLK_IPRST1_GPIORST_Msk           (0x1ul << CLK_IPRST1_GPIORST_Pos)            
#define CLK_IPRST1_I2SMRST_Pos           (18)                                         
#define CLK_IPRST1_I2SMRST_Msk           (0x1ul << CLK_IPRST1_I2SMRST_Pos)            
#define CLK_IPRST1_I2SSRST_Pos           (19)                                         
#define CLK_IPRST1_I2SSRST_Msk           (0x1ul << CLK_IPRST1_I2SSRST_Pos)            
#define CLK_IPRST1_KEYSCAN_RST_Pos       (20)                                         
#define CLK_IPRST1_KEYSCAN_RST_Msk       (0x1ul << CLK_IPRST1_KEYSCAN_RST_Pos)            
#define CLK_IPRST1_QDECRST_Pos           (21)                                         
#define CLK_IPRST1_QDECRST_Msk           (0x1ul << CLK_IPRST1_QDECRST_Pos)            
#define CLK_IPRST1_TRIMRST_Pos           (22)                                         
#define CLK_IPRST1_TRIMRST_Msk           (0x1ul << CLK_IPRST1_TRIMRST_Pos)            
 
/*BODCTL*/                                                                            
#define CLK_BODCTL_BODRSTEN_Pos          (3)                                          
#define CLK_BODCTL_BODRSTEN_Msk          (0x1ul << CLK_BODCTL_BODRSTEN_Pos)           
#define CLK_BODCTL_BODIF_Pos             (4)                                          
#define CLK_BODCTL_BODIF_Msk             (0x1ul << CLK_BODCTL_BODIF_Pos)              
#define CLK_BODCTL_BODOUT_Pos            (6)                                          
#define CLK_BODCTL_BODOUT_Msk            (0x1ul << CLK_BODCTL_BODOUT_Pos) 
#define CLK_BODCTL_LVREN_Pos             (16)                                          
#define CLK_BODCTL_LVREN_Msk             (0x1ul << CLK_BODCTL_LVREN_Pos) 
#define CLK_BODCTL_BODEN_Pos             (17)                                          
#define CLK_BODCTL_BODEN_Msk             (0x1ul << CLK_BODCTL_BODEN_Pos)   
#define CLK_BODCTL_BODSEL_Pos            (18)                                          
#define CLK_BODCTL_BODSEL_Msk            (0x7ul << CLK_BODCTL_BODSEL_Pos)  

/*BOD LVR DEBOUCE CTL*/                                                               
#define CLK_BLDBCTL_LVRDB_SEL_Pos        (8)                                          
#define CLK_BLDBCTL_LVRDB_SEL_Msk        (0x3Ful << CLK_BLDBCTL_LVRDB_SEL_Pos)        
#define CLK_BLDBCTL_BODDB_SEL_Pos        (0)                                          
#define CLK_BLDBCTL_BODDB_SEL_Msk        (0x3Ful << CLK_BLDBCTL_BODDB_SEL_Pos)        
                                                                                      
/*CLK_TOP_CTRL*/
#define CLK_TOPCTL_RCL_EN_Pos            (0)                                         
#define CLK_TOPCTL_RCL_EN_Msk            (0x1ul << CLK_TOPCTL_RCL_EN_Pos)            
#define CLK_TOPCTL_RCH_EN_Pos            (1)                                         
#define CLK_TOPCTL_RCH_EN_Msk            (0x1ul << CLK_TOPCTL_RCH_EN_Pos)            
#define CLK_TOPCTL_XTL_EN_Pos            (2)                                         
#define CLK_TOPCTL_XTL_EN_Msk            (0x1ul << CLK_TOPCTL_XTL_EN_Pos)            
#define CLK_TOPCTL_XTH_EN_Pos            (3)                                         
#define CLK_TOPCTL_XTH_EN_Msk            (0x1ul << CLK_TOPCTL_XTH_EN_Pos)            
#define CLK_TOPCTL_DPLL_EN_Pos           (4)                                         
#define CLK_TOPCTL_DPLL_EN_Msk           (0x1ul << CLK_TOPCTL_DPLL_EN_Pos)           
#define CLK_TOPCTL_SYS_CLK_SEL_Pos       (8)                                         
#define CLK_TOPCTL_SYS_CLK_SEL_Msk       (0x3ul << CLK_TOPCTL_SYS_CLK_SEL_Pos)           
#define CLK_TOPCTL_32K_CLK_SEL_Pos       (10)                                        
#define CLK_TOPCTL_32K_CLK_SEL_Msk       (0x1ul << CLK_TOPCTL_32K_CLK_SEL_Pos)
#define CLK_TOPCTL_AHB_DIV_Pos       	 (12)                                        
#define CLK_TOPCTL_AHB_DIV_Msk       	 (0xFul << CLK_TOPCTL_AHB_DIV_Pos)
#define CLK_TOPCTL_APB1_DIV_Pos          (16)                                        
#define CLK_TOPCTL_APB1_DIV_Msk          (0xFul << CLK_TOPCTL_APB1_DIV_Pos)          
#define CLK_TOPCTL_APB2_DIV_Pos          (20)                                        
#define CLK_TOPCTL_APB2_DIV_Msk          (0xFul << CLK_TOPCTL_APB2_DIV_Pos)          
                                                                                     
/*RCL_CTRL*/                                                                         
#define CLK_RCLCTL_RC32K_T_Pos           (0)                                         
#define CLK_RCLCTL_RC32K_T_Msk           (0x7ul << CLK_RCLCTL_RC32K_T_Pos)           
#define CLK_RCLCTL_RC32K_I_Pos           (3)                                         
#define CLK_RCLCTL_RC32K_I_Msk           (0x3ul << CLK_RCLCTL_RC32K_I_Pos)           
#define CLK_RCLCTL_RC32K_C_Pos           (5)                                         
#define CLK_RCLCTL_RC32K_C_Msk           (0x1Ful << CLK_RCLCTL_RC32K_C_Pos)           
#define CLK_RCLCTL_LP_LDOVR_Pos          (10)                                         
#define CLK_RCLCTL_LP_LDOVR_Msk          (0x3ul << CLK_RCLCTL_LP_LDOVR_Pos)           
#define CLK_RCLCTL_LP_RC32K_FC_Pos       (12)                                         
#define CLK_RCLCTL_LP_RC32K_FC_Msk       (0xFul << CLK_RCLCTL_LP_RC32K_FC_Pos)           
#define CLK_RCLCTL_STARTUP_Pos           (16)                                         
#define CLK_RCLCTL_STARTUP_Msk           (0x3Ful << CLK_RCLCTL_STARTUP_Pos)           

/*XTH_CTRL*/                                                                         
#define CLK_XTHCTL_FSYN_EN_Pos           (0)                                         
#define CLK_XTHCTL_FSYN_EN_Msk           (0x1ul << CLK_XTHCTL_FSYN_EN_Pos)           
#define CLK_XTHCTL_START_COUNTER_Pos     (1)                                         
#define CLK_XTHCTL_START_COUNTER_Msk     (0x1ul << CLK_XTHCTL_START_COUNTER_Pos)           
#define CLK_XTHCTL_START_FAST_Pos        (2)                                         
#define CLK_XTHCTL_START_FAST_Msk        (0x1ul << CLK_XTHCTL_START_FAST_Pos)                     
       

/*DPLL CTL*/
#define CLK_DPLLCTL_FREQ_OUT_Pos         (0)                                         
#define CLK_DPLLCTL_FREQ_OUT_Msk         (0x1ul << CLK_DPLLCTL_FREQ_OUT_Pos)
#define CLK_DPLLCTL_RCLK_SEL_Pos         (1)                                         
#define CLK_DPLLCTL_RCLK_SEL_Msk         (0x1ul << CLK_DPLLCTL_RCLK_SEL_Pos)
#define CLK_DPLLCTL_PLL_TEST_Pos         (10)                                         
#define CLK_DPLLCTL_PLL_TEST_Msk         (0x1ul << CLK_DPLLCTL_PLL_TEST_Pos)
#define CLK_DPLLCTL_STARTUP_Pos          (16)                                         
#define CLK_DPLLCTL_STARTUP_Msk          (0x3ul << CLK_DPLLCTL_STARTUP_Pos)           

/*AHB CLK CTRL*/
#define CLK_AHBCLK_DMAEN_Pos             (0)                                         
#define CLK_AHBCLK_DMAEN_Msk             (0x1ul << CLK_AHBCLK_DMAEN_Pos)           
#define CLK_AHBCLK_GPIOEN_Pos            (1)                                         
#define CLK_AHBCLK_GPIOEN_Msk            (0x1ul << CLK_AHBCLK_GPIOEN_Pos)           
#define CLK_AHBCLK_SYSTICK_EN_Pos        (2)                                         
#define CLK_AHBCLK_SYSTICK_EN_Msk        (0x1ul << CLK_AHBCLK_SYSTICK_EN_Pos)           
#define CLK_AHBCLK_APB1EN_Pos            (3)                                         
#define CLK_AHBCLK_APB1EN_Msk            (0x1ul << CLK_AHBCLK_APB1EN_Pos)           
#define CLK_AHBCLK_APB2EN_Pos            (4)                                         
#define CLK_AHBCLK_APB2EN_Msk            (0x1ul << CLK_AHBCLK_APB2EN_Pos)           
#define CLK_AHBCLK_AHBEN_Pos             (5)                                         
#define CLK_AHBCLK_AHBEN_Msk             (0x1ul << CLK_AHBCLK_AHBEN_Pos)           
#define CLK_AHBCLK_BLE32M_EN_Pos         (6)                                         
#define CLK_AHBCLK_BLE32M_EN_Msk         (0x1ul << CLK_AHBCLK_BLE32M_EN_Pos)           
#define CLK_AHBCLK_BLE32K_EN_Pos         (7)                                         
#define CLK_AHBCLK_BLE32K_EN_Msk         (0x1ul << CLK_AHBCLK_BLE32K_EN_Pos) 
#define CLK_AHBCLK_BLE32M_SEL_Pos        (8)                                         
#define CLK_AHBCLK_BLE32M_SEL_Msk        (0x1ul << CLK_AHBCLK_BLE32M_SEL_Pos) 
#define CLK_AHBCLK_BLE32K_SEL_Pos        (9)                                         
#define CLK_AHBCLK_BLE32K_SEL_Msk        (0x1ul << CLK_AHBCLK_BLE32K_SEL_Pos) 
#define CLK_AHBCLK_SPI_FLASH_DIV_Pos     (16)                                         
#define CLK_AHBCLK_SPI_FLASH_DIV_Msk     (0xFul << CLK_AHBCLK_SPI_FLASH_DIV_Pos)      

/*APB1 CLK CTRL*/
#define CLK_APB1CLK_WDTSRC_SEL_Pos       (16)                                         
#define CLK_APB1CLK_WDTSRC_SEL_Msk       (0x1ul << CLK_APB1CLK_WDTSRC_SEL_Pos)           
#define CLK_APB1CLK_WWDTSRC_SEL_Pos      (17)                                         
#define CLK_APB1CLK_WWDTSRC_SEL_Msk      (0x1ul << CLK_APB1CLK_WWDTSRC_SEL_Pos)           
#define CLK_APB1CLK_TMR0SRC_SEL_Pos      (18)                                         
#define CLK_APB1CLK_TMR0SRC_SEL_Msk      (0x3ul << CLK_APB1CLK_TMR0SRC_SEL_Pos)           

/*APB2 CLK CTRL*/
#define CLK_APB2CLK_TMR1SRC_SEL_Pos      (8)                                         
#define CLK_APB2CLK_TMR1SRC_SEL_Msk      (0x3ul << CLK_APB2CLK_TMR1SRC_SEL_Pos)           
#define CLK_APB2CLK_TMR2SRC_SEL_Pos      (10)                                         
#define CLK_APB2CLK_TMR2SRC_SEL_Msk      (0x3ul << CLK_APB2CLK_TMR2SRC_SEL_Pos)   
#define CLK_APB2CLK_KEYSCAN_DIV_Pos      (13)                                         
#define CLK_APB2CLK_KEYSCAN_DIV_Msk      (0x1fful << CLK_APB2CLK_KEYSCAN_DIV_Pos)           
#define CLK_APB2CLK_QDEC_DIV_Pos         (23)                                         
#define CLK_APB2CLK_QDEC_DIV_Msk         (0x1fful << CLK_APB2CLK_QDEC_DIV_Pos)           

/*APB2 CLK CTRL1*/
#define CLK_APB2CLK_TRIM_EN_Pos          (0)                                         
#define CLK_APB2CLK_TRIM_EN_Msk          (0x1ul << CLK_APB2CLK_TRIM_EN_Pos)           
#define CLK_APB2CLK_TRIM_CALC_EN_Pos     (1)                                         
#define CLK_APB2CLK_TRIM_CALC_EN_Msk     (0x1ul << CLK_APB2CLK_TRIM_CALC_EN_Pos)   
#define CLK_APB2CLK_CALC_CLK_SEL_Pos     (2)                                         
#define CLK_APB2CLK_CALC_CLK_SEL_Msk     (0x3ul << CLK_APB2CLK_CALC_CLK_SEL_Pos)           
#define CLK_APB2CLK_CALC_CLK_DIV_Pos     (4)                                         
#define CLK_APB2CLK_CALC_CLK_DIV_Msk     (0x1fful << CLK_APB2CLK_CALC_CLK_DIV_Pos)           
#define CLK_APB2CLK_QDEC_CLK_SEL_Pos     (13)                                         
#define CLK_APB2CLK_QDEC_CLK_SEL_Msk     (0x1ul << CLK_APB2CLK_QDEC_CLK_SEL_Pos)           
#define CLK_APB2CLK_KSCAN_CLK_DIV_Pos    (14)                                         
#define CLK_APB2CLK_KSCAN_CLK_DIV_Msk    (0x1ul << CLK_APB2CLK_KSCAN_CLK_DIV_Pos) 

/*ACT_32K_CTRL*/
#define CLK_ACT_32K_EN_Pos               (0)
#define CLK_ACT_32K_EN_Msk               (0x1ul << CLK_ACT_32K_EN_Pos)
#define CLK_ACT_32K_LL_CLK_32K_SEL_Pos   (1)
#define CLK_ACT_32K_LL_CLK_32K_SEL_Msk   (0x1ul << CLK_ACT_32K_LL_CLK_32K_SEL_Pos)
#define CLK_ACT_32K_SLP_CNT_UP_Pos       (2)
#define CLK_ACT_32K_SLP_CNT_UP_Msk       (0x1ul << CLK_ACT_32K_SLP_CNT_UP_Pos)
#define CLK_ACT_32K_ACT_CLK_UP_DONE_Pos       (3)
#define CLK_ACT_32K_ACT_CLK_UP_DONE_Msk       (0x1ul << CLK_ACT_32K_ACT_CLK_UP_DONE_Pos)
#define CLK_ACT_32K_FINEOFFSET_Pos       (12)
#define CLK_ACT_32K_FINEOFFSET_Msk       (0x3FFul << CLK_ACT_32K_FINEOFFSET_Pos)
#define CLK_ACT_32K_FINECORR_Pos         (22)
#define CLK_ACT_32K_FINECORR_Msk         (0x3FFul << CLK_ACT_32K_FINECORR_Pos)

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Watch Dog Timer Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */

typedef struct {


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  WDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTCNT    |Reset WDT Up Counter (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal 18-bit WDT up counter value.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This bit will be automatically cleared by hardware.
     * |[1]     |RSTEN     |WDT Time-out Reset Enable Bit (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset function If the WDT up counter value has not been cleared after the specific WDT reset delay period expires.
     * |        |          |0 = WDT time-out reset function Disabled.
     * |        |          |1 = WDT time-out reset function Enabled.
     * |        |          |Note: This bit is write-protected. Refer to the SYS_REGLCTL register.
     * |[2]     |RSTF      |WDT Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
     * |        |          |0 = WDT time-out reset did not occur.
     * |        |          |1 = WDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |IF        |WDT Time-out Interrupt Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WKEN      |WDT Time-out Wake-up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while WDT time-out interrupt flag IF (WDT_CTL[3]) is generated to 1 and interrupt enable bit INTEN (WDT_CTL[6]) is enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to LIRC or LXT.
     * |[5]     |WKF       |WDT Time-out Wake-up Flag (Write Protect)
     * |        |          |This bit indicates the interrupt wake-up flag status of WDT
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if WDT time-out interrupt signal generated.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * |[6]     |INTEN     |WDT Time-out Interrupt Enable Bit (Write Protect)
     * |        |          |If this bit is enabled, the WDT time-out interrupt signal is generated and inform to CPU.
     * |        |          |0 = WDT time-out interrupt Disabled.
     * |        |          |1 = WDT time-out interrupt Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |WDTEN     |WDT Enable Bit (Write Protect)
     * |        |          |0 = WDT Disabled (This action will reset the internal up counter value).
     * |        |          |1 = WDT Enabled.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: If CWDTEN[2:0] (combined by Config0[31] and Config0[4:3]) bits is not configure to 111, this bit is forced as 1 and user cannot change this bit to 0.
     * |[10:8]  |TOUTSEL   |WDT Time-out Interval Selection (Write Protect)
     * |        |          |These three bits select the time-out interval period for the WDT.
     * |        |          |000 = 2^4 * WDT_CLK.
     * |        |          |001 = 2^6 * WDT_CLK.
     * |        |          |010 = 2^8 * WDT_CLK.
     * |        |          |011 = 2^10 * WDT_CLK.
     * |        |          |100 = 2^12 * WDT_CLK.
     * |        |          |101 = 2^14 * WDT_CLK.
     * |        |          |110 = 2^16 * WDT_CLK.
     * |        |          |111 = 2^18 * WDT_CLK.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]     |TOF      |WDT Time-out Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval
     * |        |          |0 = WDT time-out did not occur.
     * |        |          |1 = WDT time-out occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement affects WDT counting.
     * |        |          |WDT up counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
    */
    __IO uint32_t CTL;

    /**
     * ALTCTL
     * ===================================================================================================
     * Offset: 0x04  WDT Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |RSTDSEL   |WDT Reset Delay Selection (Write Protect)
     * |        |          |When WDT time-out happened, user has a time named WDT Reset Delay Period to clear WDT counter by setting RSTCNT (WDT_CTL[0]) to prevent WDT time-out reset happened.
     * |        |          |User can select a suitable setting of RSTDSEL for different WDT Reset Delay Period.
     * |        |          |00 = WDT Reset Delay Period is 1025 * WDT_CLK.
     * |        |          |01 = WDT Reset Delay Period is 129 * WDT_CLK.
     * |        |          |10 = WDT Reset Delay Period is 17 * WDT_CLK.
     * |        |          |11 = WDT Reset Delay Period is 2 * WDT_CLK.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This register will be reset to 0 if WDT time-out reset happened.
    */
    __IO uint32_t ALTCTL;

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT_T::CTL: RSTCNT Position               */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT_T::CTL: RSTCNT Mask                   */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position                */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                    */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position                 */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                     */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                   */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                       */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position                 */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                     */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position                  */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                      */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position                */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                    */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position                */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                    */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position              */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask                  */

#define WDT_CTL_TOF_Pos                  (16)                                               /*!< WDT_T::CTL: TOF Position                   */
#define WDT_CTL_TOF_Msk                  (0x1ul << WDT_CTL_TOF_Pos)                         /*!< WDT_T::CTL: TOF Mask                       */

#define WDT_CTL_ICEDEBUG_Pos             (31)                                              /*!< WDT_T::CTL: ICEDEBUG Position             */
#define WDT_CTL_ICEDEBUG_Msk             (0x1ul << WDT_CTL_ICEDEBUG_Pos)                   /*!< WDT_T::CTL: ICEDEBUG Mask                 */

#define WDT_ALTCTL_RSTDSEL_Pos           (0)                                               /*!< WDT_T::ALTCTL: RSTDSEL Position           */
#define WDT_ALTCTL_RSTDSEL_Msk           (0x3ul << WDT_ALTCTL_RSTDSEL_Pos)                 /*!< WDT_T::ALTCTL: RSTDSEL Mask               */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */








/*---------------------- Window Watchdog Timer -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup WWDT Window Watchdog Timer(WWDT)
    Memory Mapped Structure for WWDT Controller
@{ */

typedef struct {


    /**
     * RLDCNT
     * ===================================================================================================
     * Offset: 0x00  WWDT Reload Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RLDCNT    |WWDT Reload Counter Register
     * |        |          |Writing 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
     * |        |          |Note: User can only write WWDT_RLDCNT register to reload WWDT counter value when current WWDT counter value between 0 and CMPDAT (WWDT_CTL[21:16]).
     * |        |          |If user writes WWDT_RLDCNT when current WWDT counter value is larger than CMPDAT, WWDT reset signal will generate immediately.
    */
    __O  uint32_t RLDCNT;

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x04  WWDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTEN    |WWDT Enable Bit
     * |        |          |Set this bit to enable WWDT counter counting.
     * |        |          |0 = WWDT counter is stopped.
     * |        |          |1 = WWDT counter is starting counting.
     * |[1]     |INTEN     |WWDT Interrupt Enable Bit
     * |        |          |If this bit is enabled, the WWDT counter compare match interrupt signal is generated and inform to CPU.
     * |        |          |0 = WWDT counter compare match interrupt Disabled.
     * |        |          |1 = WWDT counter compare match interrupt Enabled.
     * |[11:8]  |PSCSEL    |WWDT Counter Prescale Period Select Bits
     * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * WWDT_CLK.
     * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * WWDT_CLK.
     * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * WWDT_CLK.
     * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * WWDT_CLK.
     * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * WWDT_CLK.
     * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * WWDT_CLK.
     * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * WWDT_CLK.
     * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * WWDT_CLK.
     * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * WWDT_CLK.
     * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * WWDT_CLK.
     * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * WWDT_CLK.
     * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * WWDT_CLK.
     * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * WWDT_CLK.
     * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * WWDT_CLK.
     * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * WWDT_CLK.
     * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * WWDT_CLK.
     * |[21:16] |CMPDAT    |WWDT Window Compare Bits
     * |        |          |Set this register to adjust the valid reload window.
     * |        |          |Note: User can only write WWDT_RLDCNT register to reload WWDT counter value when current WWDT counter value between 0 and CMPDAT.
     * |        |          |If user writes WWDT_RLDCNT register when current WWDT counter value larger than CMPDAT, WWDT reset signal will generate immediately.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit
     * |        |          |0 = ICE debug mode acknowledgement effects WWDT counting.
     * |        |          |WWDT down counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WWDT down counter will keep going no matter CPU is held by ICE or not.
    */
    __IO uint32_t CTL;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x08  WWDT Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of WWDT while WWDT counter value matches CMPDAT (WWDT_CTL[21:16]).
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches CMPDAT.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |WWDTRF    |WWDT Timer-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WWDT time-out reset or not.
     * |        |          |0 = WWDT time-out reset did not occur.
     * |        |          |1 = WWDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[2]     |WWDTF     |WWDT Compare Match Flag
     * |        |          |This bit indicates the flag status of WWDT while WWDT counter value matches CMPDAT (WWDT_CTL[21:16]).
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches CMPDAT.
     * |        |          |Note: This bit is cleared by writing 1 to it.

    */
    __IO uint32_t STATUS;

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  WWDT Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CNTDAT    |WWDT Counter Value
     * |        |          |CNTDAT will be updated continuously to monitor 6-bit WWDT down counter value.
    */
    __I  uint32_t CNT;

} WWDT_T;

/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
@{ */

#define WWDT_RLDCNT_RLDCNT_Pos           (0)                                               /*!< WWDT_T::RLDCNT: RLDCNT Position           */
#define WWDT_RLDCNT_RLDCNT_Msk           (0xfffffffful << WWDT_RLDCNT_RLDCNT_Pos)          /*!< WWDT_T::RLDCNT: RLDCNT Mask               */

#define WWDT_CTL_WWDTEN_Pos              (0)                                               /*!< WWDT_T::CTL: WWDTEN Position              */
#define WWDT_CTL_WWDTEN_Msk              (0x1ul << WWDT_CTL_WWDTEN_Pos)                    /*!< WWDT_T::CTL: WWDTEN Mask                  */

#define WWDT_CTL_INTEN_Pos               (1)                                               /*!< WWDT_T::CTL: INTEN Position               */
#define WWDT_CTL_INTEN_Msk               (0x1ul << WWDT_CTL_INTEN_Pos)                     /*!< WWDT_T::CTL: INTEN Mask                   */

#define WWDT_CTL_PSCSEL_Pos              (8)                                               /*!< WWDT_T::CTL: PSCSEL Position              */
#define WWDT_CTL_PSCSEL_Msk              (0xful << WWDT_CTL_PSCSEL_Pos)                    /*!< WWDT_T::CTL: PSCSEL Mask                  */

#define WWDT_CTL_CMPDAT_Pos              (16)                                              /*!< WWDT_T::CTL: CMPDAT Position              */
#define WWDT_CTL_CMPDAT_Msk              (0x3ful << WWDT_CTL_CMPDAT_Pos)                   /*!< WWDT_T::CTL: CMPDAT Mask                  */

#define WWDT_CTL_ICEDEBUG_Pos            (31)                                              /*!< WWDT_T::CTL: ICEDEBUG Position            */
#define WWDT_CTL_ICEDEBUG_Msk            (0x1ul << WWDT_CTL_ICEDEBUG_Pos)                  /*!< WWDT_T::CTL: ICEDEBUG Mask                */

#define WWDT_STATUS_WWDTIF_Pos           (0)                                               /*!< WWDT_T::STATUS: WWDTIF Position           */
#define WWDT_STATUS_WWDTIF_Msk           (0x1ul << WWDT_STATUS_WWDTIF_Pos)                 /*!< WWDT_T::STATUS: WWDTIF Mask               */

#define WWDT_STATUS_WWDTRF_Pos           (1)                                               /*!< WWDT_T::STATUS: WWDTRF Position           */
#define WWDT_STATUS_WWDTRF_Msk           (0x1ul << WWDT_STATUS_WWDTRF_Pos)                 /*!< WWDT_T::STATUS: WWDTRF Mask               */

#define WWDT_STATUS_WWDTF_Pos            (2)                                               /*!< WWDT_T::STATUS: WWDTF Position           */
#define WWDT_STATUS_WWDTF_Msk            (0x1ul << WWDT_STATUS_WWDTF_Pos)                 /*!< WWDT_T::STATUS: WWDTF Mask               */

#define WWDT_CNT_CNTDAT_Pos              (0)                                               /*!< WWDT_T::CNT: CNTDAT Position              */
#define WWDT_CNT_CNTDAT_Msk              (0x3ful << WWDT_CNT_CNTDAT_Pos)                   /*!< WWDT_T::CNT: CNTDAT Mask                  */

/**@}*/ /* WWDT_CONST */
/**@}*/ /* end of WWDT register group */








/*---------------------- Pulse Width Modulation Controller -------------------------*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */

typedef struct {


    /**
     * CLKPSC
     * ===================================================================================================
     * Offset: 0x00  PWM Clock Pre-scale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CLKPSC01  |Clock Prescaler 0 For PWM Counter 0 And 1
     * |        |          |Clock input is divided by (CLKPSC01 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC01 = 0, the clock prescaler 0 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[15:8]  |CLKPSC23  |Clock Prescaler 2 For PWM Counter 2 And 3
     * |        |          |Clock input is divided by (CLKPSC23 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC23 = 0, the clock prescaler 2 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[23:16] |CLKPSC45  |Clock Prescaler 4 For PWM Counter 4 And 5
     * |        |          |Clock input is divided by (CLKPSC45 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC45 = 0, the clock prescaler 4 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
    * |[31:24] |CLKPSC67  |Clock Prescaler 6 For PWM Counter 6 And 7
    * |        |          |Clock input is divided by (CLKPSC67 + 1) before it is fed to the corresponding PWM counter.
    * |        |          |If CLKPSC67 = 0, the clock prescaler 6 output clock will be stopped.
    * |        |          |So the corresponding PWM counter will also be stopped.
    */
    __IO uint32_t CLKPSC;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CLKDIV0   |Counter 0 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC01*2).
     * |        |          |001 = Clock input / (CLKPSC01*4).
     * |        |          |010 = Clock input / (CLKPSC01*8).
     * |        |          |011 = Clock input / (CLKPSC01*16).
     * |        |          |100 = Clock input / CLKPSC01.
     * |        |          |Others = Clock input.
     * |[6:4]   |CLKDIV1   |Counter 1 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC01*2).
     * |        |          |001 = Clock input / (CLKPSC01*4).
     * |        |          |010 = Clock input / (CLKPSC01*8).
     * |        |          |011 = Clock input / (CLKPSC01*16).
     * |        |          |100 = Clock input / CLKPSC01.
     * |        |          |Others = Clock input.
     * |[10:8]  |CLKDIV2   |Counter 2 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC23*2).
     * |        |          |001 = Clock input / (CLKPSC23*4).
     * |        |          |010 = Clock input / (CLKPSC23*8).
     * |        |          |011 = Clock input / (CLKPSC23*16).
     * |        |          |100 = Clock input / CLKPSC23.
     * |        |          |Others = Clock input.
     * |[14:12] |CLKDIV3   |Counter 3 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC23*2).
     * |        |          |001 = Clock input / (CLKPSC23*4).
     * |        |          |010 = Clock input / (CLKPSC23*8).
     * |        |          |011 = Clock input / (CLKPSC23*16).
     * |        |          |100 = Clock input / CLKPSC23.
     * |        |          |Others = Clock input.
     * |[18:16] |CLKDIV4   |Counter 4 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC45*2).
     * |        |          |001 = Clock input / (CLKPSC45*4).
     * |        |          |010 = Clock input / (CLKPSC45*8).
     * |        |          |011 = Clock input / (CLKPSC45*16).
     * |        |          |100 = Clock input / CLKPSC45.
     * |        |          |Others = Clock input.
     * |[22:20] |CLKDIV5   |Counter 5 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC45*2).
     * |        |          |001 = Clock input / (CLKPSC45*4).
     * |        |          |010 = Clock input / (CLKPSC45*8).
     * |        |          |011 = Clock input / (CLKPSC45*16).
     * |        |          |100 = Clock input / CLKPSC45.
     * |        |          |Others = Clock input.
     *|[26:24] |CLKDIV4   |Counter 6 Clock Divider Selection
     *|        |          |Select clock input for PWM counter.
     *|        |          |000 = Clock input / (CLKPSC67*2).
     *|        |          |001 = Clock input / (CLKPSC67*4).
     *|        |          |010 = Clock input / (CLKPSC67*8).
     *|        |          |011 = Clock input / (CLKPSC67*16).
     *|        |          |100 = Clock input / CLKPSC67.
     *|        |          |Others = Clock input.
     *|[30:28] |CLKDIV5   |Counter 7 Clock Divider Selection
     *|        |          |Select clock input for PWM counter.
     *|        |          |000 = Clock input / (CLKPSC67*2).
     *|        |          |001 = Clock input / (CLKPSC67*4).
     *|        |          |010 = Clock input / (CLKPSC67*8).
     *|        |          |011 = Clock input / (CLKPSC67*16).
     *|        |          |100 = Clock input / CLKPSC67.
     *|        |          |Others = Clock input.
     */
    __IO uint32_t CLKDIV;

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |PWM Counter 0 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[2]     |PINV0     |PWM0_CH0 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH0 output inverter Disabled.
     * |        |          |1 = PWM0_CH0 output inverter Enabled.
     * |[3]     |CNTMODE0  |PWM Counter 0 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD0 and CMP0 cleared.
     * |[4]     |CNTEN1    |PWM Counter 1 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[5]     |HCUPDT    |Half Cycle Update Enable for Center-aligned Type
     * |        |          |0 = Disable half cycle update PERIOD & CMP.
     * |        |          |1 = Enable half cycle update PERIOD & CMP.
     * |[6]     |PINV1     |PWM0_CH1 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH1 output inverter Disable.
     * |        |          |1 = PWM0_CH1 output inverter Enable.
     * |[7]     |CNTMODE1  |PWM Counter 1 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD1 and CMP1 cleared.
     * |[8]     |CNTEN2    |PWM Counter 2 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[10]    |PINV2     |PWM0_CH2 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH2 output inverter Disabled.
     * |        |          |1 = PWM0_CH2 output inverter Enabled.
     * |[11]    |CNTMODE2  |PWM Counter 2 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD2 and CMP2 cleared.
     * |[12]    |CNTEN3    |PWM Counter 3 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[14]    |PINV3     |PWM0_CH 3 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH3 output inverter Disabled.
     * |        |          |1 = PWM0_CH3 output inverter Enabled.
     * |[15]    |CNTMODE3  |PWM Counter 3 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD3 and CMP3 cleared.
     * |[16]    |CNTEN4    |PWM Counter 4 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[18]    |PINV4     |PWM0_CH4 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH4 output inverter Disabled.
     * |        |          |1 = PWM0_CH4 output inverter Enabled.
     * |[19]    |CNTMODE4  |PWM Counter 4 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD4 and CMP4 cleared.
     * |[20]    |CNTEN5    |PWM Counter 5 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[21]    |ASYMEN    |Asymmetric Mode In Center-aligned Type
     * |        |          |0 = Symmetric mode in center-aligned type.
     * |        |          |1 = Asymmetric mode in center-aligned type.
     * |[22]    |PINV5     |PWM0_CH5 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH5 output inverter Disabled.
     * |        |          |1 = PWM0_CH5 output inverter Enabled.
     * |[23]    |CNTMODE5  |PWM Counter 5 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD5 and CMP5 cleared.
     *|[24]    |CNTEN6    |PWM Counter 6 Enable Start Run
     *|        |          |0 = Corresponding PWM counter running Stopped.
     *|        |          |1 = Corresponding PWM counter start run Enabled.
     *|[26]    |PINV6     |PWM0_CH6 Output Inverter Enable Bit
     *|        |          |0 = PWM0_CH6 output inverter Disabled.
     *|        |          |1 = PWM0_CH6 output inverter Enabled.
     *|[27]    |CNTMODE6  |PWM Counter 6 Auto-reload/One-shot Mode
     *|        |          |0 = One-shot mode.
     *|        |          |1 = Auto-reload mode.
     *|        |          |Note: If there is a rising transition at this bit, it will cause PERIOD6 and CMP6 cleared.
     *|[28]    |CNTEN7    |PWM Counter 7 Enable Start Run
     *|        |          |0 = Corresponding PWM counter running Stopped.
     *|        |          |1 = Corresponding PWM counter start run Enabled.
     *|[30]    |PINV7     |PWM0_CH7 Output Inverter Enable Bit
     *|        |          |0 = PWM0_CH7 output inverter Disabled.
     *|        |          |1 = PWM0_CH7 output inverter Enabled.
     *|[31]    |CNTMODE7  |PWM Counter 7 Auto-reload/One-shot Mode
     *|        |          |0 = One-shot mode.
     *|        |          |1 = Auto-reload mode.
     *|        |          |Note: If there is a rising transition at this bit, it will cause PERIOD7 and CMP7 cleared.
    */
    __IO uint32_t CTL;

    /**
     * PERIOD0
     * ===================================================================================================
     * Offset: 0x0C  PWM Counter Period Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD0   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn:
     * |	 low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
     */
    __IO uint32_t PERIOD0;

    /**
     * PERIOD1
     * ===================================================================================================
     * Offset: 0x10  PWM Counter Period Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD1   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD1;

    /**
     * PERIOD2
     * ===================================================================================================
     * Offset: 0x14  PWM Counter Period Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD2   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD2;

    /**
     * PERIOD3
     * ===================================================================================================
     * Offset: 0x18  PWM Counter Period Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD3   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD3;

    /**
     * PERIOD4
     * ===================================================================================================
     * Offset: 0x1C  PWM Counter Period Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD4   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD4;

    /**
     * PERIOD5
     * ===================================================================================================
     * Offset: 0x20  PWM Counter Period Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD5   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD5;

    /**
    * PERIOD6
    * ===================================================================================================
    * Offset: 0x24  PWM Counter Period Register 6
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |PERIOD6   |PWM Counter Period Value
    * |        |          |PERIODn determines the PWM counter period.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD6;

    /**
    * PERIOD7
    * ===================================================================================================
    * Offset: 0x28  PWM Counter Period Register 7
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |PERIOD7   |PWM Counter Period Value
    * |        |          |PERIODn determines the PWM counter period.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD7;

    /**
     * CMPDAT0
     * ===================================================================================================
* Offset: 0x2C  PWM Comparator Register 0		// 0x24 - > 0x2C
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP0      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD0     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT0;

    /**
     * CMPDAT1
     * ===================================================================================================
     * Offset: 0x30  PWM Comparator Register 1		// 0x28 -> 0x30
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP1      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD1     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT1;

    /**
     * CMPDAT2
     * ===================================================================================================
     * Offset: 0x34  PWM Comparator Register 2		// 0x2C -> 0x34
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP2      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD2     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT2;

    /**
     * CMPDAT3
     * ===================================================================================================
     * Offset: 0x38  PWM Comparator Register 3		// 0x30 -> 0x38
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP3      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD3     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT3;

    /**
     * CMPDAT4
     * ===================================================================================================
     * Offset: 0x3C  PWM Comparator Register 4		// 0x34 -> 0x3C
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP4      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD4     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT4;

    /**
     * CMPDAT5
     * ===================================================================================================
    * Offset: 0x40  PWM Comparator Register 5		// 0x38 -> 0x40
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP5      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD5     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT5;

    /**
    * CMPDAT6
    * ===================================================================================================
    * Offset: 0x44  PWM Comparator Register 6
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |CMP6      |PWM Comparator Register
    * |        |          |CMP determines the PWM duty.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
    * |[31:16] |CMPD6     |PWM Comparator Register For Down Counter In Asymmetric Mode
    * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
    * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
    * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT6;

    /**
    * CMPDAT7
    * ===================================================================================================
    * Offset: 0x48  PWM Comparator Register 7
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |CMP7      |PWM Comparator Register
    * |        |          |CMP determines the PWM duty.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
    * |[31:16] |CMPD7     |PWM Comparator Register For Down Counter In Asymmetric Mode
    * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
    * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
    * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT7;

    /**
    * CTL2
    * ===================================================================================================
    * Offset: 0x4C  PWM Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[17]    |PINTTYPE  |PWM Interrupt Type Selection
    * |        |          |0 = ZIFn will be set if PWM counter underflows.
    * |        |          |1 = ZIFn will be set if PWM counter matches PERIODn register.
    * |        |          |Note: This bit is effective when PWM is in center-aligned type only.
    * |[24]    |DTCNT01   |Dead-time 0 Counter Enable Bit (PWM0_CH0 And PWM0_CH1 Pair For PWMA Group)						// removed from CTL
    * |        |          |0 = Dead-time 0 generator Disabled.
    * |        |          |1 = Dead-time 0 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH0 and PWM0_CH1 becomes a complementary pair for PWMA group.
    * |[25]    |DTCNT23   |Dead-time 2 Counter Enable Bit (PWM0_CH2 And PWM0_CH3 Pair For PWMB Group)
    * |        |          |0 = Dead-time 2 generator Disabled.
    * |        |          |1 = Dead-time 2 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH2 and PWM0_CH3 becomes a complementary pair for PWMB group.
    * |[26]    |DTCNT45   |Dead-time 4 Counter Enable Bit (PWM0_CH4 And PWM0_CH5 Pair For PWMC Group)
    * |        |          |0 = Dead-time 4 generator Disabled.
    * |        |          |1 = Dead-time 4 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH4 and PWM0_CH5 becomes a complementary pair for PWMC group.
    * |[27]    |DTCNT67   |Dead-time 6 Counter Enable Bit (PWM0_CH6 And PWM0_CH7 Pair For PWMC Group)
    * |        |          |0 = Dead-time 6 generator Disabled.
    * |        |          |1 = Dead-time 6 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH6 and PWM0_CH7 becomes a complementary pair for PWMC group.
    * |[27]    |CNTCLR    |Clear PWM Counter Control Bit
    * |        |          |0 = Do not clear PWM counter.
    * |        |          |1 = All 16-bit PWM counters cleared to 0x0000.
    * |        |          |Note: It is automatically cleared by hardware.
    * |[29:28] |MODE      |PWM Operating Mode Select Bit
    * |        |          |00 = Independent mode.
    * |        |          |01 = Complementary mode.
    * |        |          |10 = Synchronized mode.
    * |        |          |11 = Reserved.
    * |[30]    |GROUPEN   |Group Function Enable Bit
    * |        |          |0 = The signals timing of all PWM channels are independent.
    * |        |          |1 = Unify the signals timing of PWM0_CH0, PWM0_CH2 and PWM0_CH4 in the same phase which is controlled by PWM0_CH0 and also unify the signals timing of PWM0_CH1, PWM0_CH3 and PWM0_CH5 in the same phase which is controlled by PWM0_CH1.
    * |[31]    |CNTTYPE   |PWM Counter-aligned Type Select Bit
    * |        |          |0 = Edge-aligned type.
    * |        |          |1 = Center-aligned type.
    */
    __IO uint32_t CTL2;


    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x58  PWM Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZFn      |PWM Zero Point Flag
     * |        |          |Flag is set by hardware when PWMn counter down count reaches zero point.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[15:8]  |CMPDFn   |PWM Compare Down Flag
     * |        |          |Flag is set by hardware when PWMn counter down count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[23:18] |PFn      |PWM Period Flag
     * |        |          |Flag is set by hardware when PWM0_CHn counter reaches PERIODn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[31:24] |CMPUFn   |PWM Compare Up Flag
     * |        |          |Flag is set by hardware when PWM0_CHn counter up count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO  uint32_t FLAG;


    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x54  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZIENn     |PWM Zero Point Interrupt Enable Bit
     * |        |          |0 = PWM0_CHn zero point interrupt Disabled.
     * |        |          |1 = PWM0_CHn zero point interrupt Enabled.
     * |[15:8]  |CMPDIENn  |PWM Compare Down Interrupt Enable Bit
     * |        |          |0 = PWM0_CHn compare down interrupt Disabled.
     * |        |          |1 = PWM0_CHn compare down interrupt Enabled.
     * |[23:16] |PIENn     |PWM Period Interrupt Enable Bit
     * |        |          |0 = PWM0_CHn period interrupt Disabled.
     * |        |          |1 = PWM0_CHn period interrupt Enabled.
     * |[31:24] |CMPUIENn  |PWM Compare Up Interrupt Enable Bit
     * |        |          |0 = PWM0_CHn compare up interrupt Disabled.
     * |        |          |1 = PWM0_CHn compare up interrupt Enabled.
    */
    __IO uint32_t INTEN;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x58  PWM Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZIFn      |PWM Zero Point Interrupt Flag
     * |        |          |Flag is set by hardware when PWMn counter down count reaches zero point.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[15:8]  |CMPDIFn   |PWM Compare Down Interrupt Flag
     * |        |          |Flag is set by hardware when PWMn counter down count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[23:18] |PIFn      |PWM Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM0_CHn counter reaches PERIODn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[31:24] |CMPUIFn   |PWM Compare Up Interrupt Flag
     * |        |          |Flag is set by hardware when PWM0_CHn counter up count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t INTSTS;

    /**
     * POEN
     * ===================================================================================================
     * Offset: 0x5C  PWM Output Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |POENn     |PWM Output Enable Bits
     * |        |          |0 = PWM channel n output to pin Disabled.
     * |        |          |1 = PWM channel n output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
    */
    __IO uint32_t POEN;

       /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * DTCTL
     * ===================================================================================================
     * Offset: 0x64  PWM Dead-time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DTI01     |Dead-time Interval Register For Pair Of Channel0 And Channel1 (PWM0_CH0 And PWM0_CH1 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[15:8]  |DTI23     |Dead-time Interval Register For Pair Of Channel2 And Channel3 (PWM0_CH2 And PWM0_CH3 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[23:16] |DTI45     |Dead-time Interval Register For Pair Of Channel4 And Channel5 (PWM0_CH4 And PWM0_CH5 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[31:24] |DTI67     |Dead-time Interval Register For Pair Of Channel6 And Channel7 (PWM0_CH6 And PWM0_CH7 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
    */
    __IO uint32_t DTCTL;

    /**
     * ADCTCTL0
     * ===================================================================================================
     * Offset: 0x68  PWM Trigger Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN0  |Channel 0 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching CMP0 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN0  |Channel 0 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM Trigger ADC Function While channel0's Counter Matching PERIOD0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN0  |Channel 0 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching CMP0 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[3]     |ZPTRGEN0  |Channel 0 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[8]     |CUTRGEN1  |Channel 1 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching CMP1 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN1  |Channel 1 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching PERIOD1
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN1  |Channel 1 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching CMP1 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[11]    |ZPTRGEN1  |Channel 1 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function While channel1's Counter Matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[16]    |CUTRGEN2  |Channel 2 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching CMP2 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[17]    |CPTRGEN2  |Channel 2 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching PERIOD2
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[18]    |CDTRGEN2  |Channel 2 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching CMP2 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[19]    |ZPTRGEN2  |Channel 2 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[24]    |CUTRGEN3  |Channel 3 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching CMP3 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
     * |[25]    |CPTRGEN3  |Channel 3 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching PERIOD3
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
     * |[26]    |CDTRGEN3  |Channel 3 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching CMP3 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
     * |[27]    |ZPTRGEN3  |Channel 3 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    */
    __IO uint32_t ADCTCTL0;

    /**
     * ADCTCTL1
     * ===================================================================================================
     * Offset: 0x6C  PWM Trigger Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN4  |Channel 4 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching CMP4 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN4  |Channel 4 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching PERIOD4
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN4  |Channel 4 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching CMP4 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[3]     |ZPTRGEN4  |Channel 4 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[8]     |CUTRGEN5  |Channel 5 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching CMP5 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN5  |Channel 5 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching PERIOD5
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN5  |Channel 5 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching CMP5 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[11]    |ZPTRGEN5  |Channel 5 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[16]    |CUTRGEN6  |Channel 6 Compare Up Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching CMP6 in up-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
    * |[17]    |CPTRGEN6  |Channel 6 Center Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching PERIOD6
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
    * |[18]    |CDTRGEN6  |Channel 6 Compare Down Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching CMP6 in down-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[19]    |ZPTRGEN6  |Channel 6 Zero Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching 0
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[24]    |CUTRGEN7  |Channel 7 Compare Up Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching CMP7 in up-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
    * |[25]    |CPTRGEN7  |Channel 7 Center Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching PERIOD7
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
    * |[26]    |CDTRGEN7  |Channel 7 Compare Down Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching CMP7 in down-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    * |[27]    |ZPTRGEN7  |Channel 7 Zero Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching 0
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    */
    __IO uint32_t ADCTCTL1;

    /**
     * ADCTSTS0
     * ===================================================================================================
     * Offset: 0x70  PWM Trigger Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF0   |Channel 0 Compare Up Trigger ADC Flag
     * |        |          |When the channel0's counter is counting up to CMP0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[1]     |CPTRGF0   |Channel 0 Center Point Trigger ADC Flag
     * |        |          |When the channel0's counter is counting to PERIOD0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[2]     |CDTRGF0   |Channel 0 Compare Down Trigger ADC Flag
     * |        |          |When the channel0's counter is counting down to CMP0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[3]     |ZPTRGF0   |Channel 0 Zero Point Trigger ADC Flag
     * |        |          |When the channel0's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[8]     |CUTRGF1   |Channel 1 Compare Up Trigger ADC Flag
     * |        |          |When the channel1's counter is counting up to CMP1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[9]     |CPTRGF1   |Channel 1 Center Point Trigger ADC Flag
     * |        |          |When the channel1's counter is counting to PERIOD1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[10]    |CDTRGF1   |Channel 1 Compare Down Trigger ADC Flag
     * |        |          |When the channel1's counter is counting down to CMP1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[11]    |ZPTRGF1   |Channel 1 Zero Point Trigger ADC Flag
     * |        |          |When the channel1's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[16]    |CUTRGF2   |Channel 2 Compare Up Trigger ADC Flag
     * |        |          |When the channel2's counter is counting up to CMP2, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[17]    |CPTRGF2   |Channel 2 Center Point Trigger ADC Flag
     * |        |          |When the channel2's counter is counting to PERIOD2, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[18]    |CDTRGF2   |Channel 2 Compare Down Trigger ADC Flag
     * |        |          |When the channel2's counter is counting down to CMP2, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[19]    |ZPTRGF2   |Channel 2 Zero Point Trigger ADC Enable Bit
     * |        |          |When the channel2's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[24]    |CUTRGF3   |Channel 3 Compare Up Trigger ADC Flag
     * |        |          |When the channel3's counter is counting up to CMP3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[25]    |CPTRGF3   |Channel 3 Center Point Trigger ADC Flag
     * |        |          |When the channel3's counter is counting to PERIOD3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[26]    |CDTRGF3   |Channel 3 Compare Down Trigger ADC Flag
     * |        |          |When the channel3's counter is counting down to CMP3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[27]    |ZPTRGF3   |Channel 3 Zero Point Trigger ADC Flag
     * |        |          |When the channel3's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t ADCTSTS0;

    /**
     * ADCTSTS1
     * ===================================================================================================
     * Offset: 0x74  PWM Trigger Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF4   |Channel 4 Compare Up Trigger ADC Flag
     * |        |          |When the channel4's counter is counting up to CMP4, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[1]     |CPTRGF4   |Channel 4 Center Point Trigger ADC Flag
     * |        |          |When the channel4's counter is counting to PERIOD4, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[2]     |CDTRGF4   |Channel 4 Compare Down Trigger ADC Flag
     * |        |          |When the channel4's counter is counting down to CMP4, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[3]     |ZPTRGF4   |Channel 4 Zero Point Trigger ADC Flag
     * |        |          |When the channel4's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[8]     |CUTRGF5   |Channel 5 Compare Up Trigger ADC Flag
     * |        |          |When the channel5's counter is counting up to CMP5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[9]     |CPTRGF5   |Channel 5 Center Point Trigger ADC Flag
     * |        |          |When the channel5's counter is counting to PERIOD5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[10]    |CDTRGF5   |Channel 5 Compare Down Trigger ADC Flag
     * |        |          |When the channel5's counter is counting down to CMP5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[11]    |ZPTRGF5   |Channel 5 Zero Point Trigger ADC Flag
     * |        |          |When the channel5's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[16]    |CUTRGF6   |Channel 6 Compare Up Trigger ADC Flag
     * |        |          |When the channel6's counter is counting up to CMP6, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[17]    |CPTRGF6   |Channel 6 Center Point Trigger ADC Flag
     * |        |          |When the channel6's counter is counting to PERIOD6, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[18]    |CDTRGF6   |Channel 6 Compare Down Trigger ADC Flag
     * |        |          |When the channel6's counter is counting down to CMP6, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[19]    |ZPTRGF6   |Channel 6 Zero Point Trigger ADC Enable Bit
     * |        |          |When the channel6's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[24]    |CUTRGF7   |Channel 7 Compare Up Trigger ADC Flag
     * |        |          |When the channel7's counter is counting up to CMP7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[25]    |CPTRGF7   |Channel 7 Center Point Trigger ADC Flag
     * |        |          |When the channel7's counter is counting to PERIOD7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[26]    |CDTRGF7   |Channel 7 Compare Down Trigger ADC Flag
     * |        |          |When the channel7's counter is counting down to CMP7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[27]    |ZPTRGF7   |Channel 7 Zero Point Trigger ADC Flag
     * |        |          |When the channel7's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t ADCTSTS1;


    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[4];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * PCACTL
     * ===================================================================================================
     * Offset: 0x88  PWM Precise Center-Aligned Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PCAEN     |PWM Precise Center-aligned Type Enable Bit
     * |        |          |0 = Precise center-aligned type Disabled.
     * |        |          |1 = Precise center-aligned type Enabled.
    */
    __IO uint32_t PCACTL;


} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC01_Pos          (0)                                               /*!< PWM_T::CLKPSC: CLKPSC01 Position          */
#define PWM_CLKPSC_CLKPSC01_Msk          (0xfful << PWM_CLKPSC_CLKPSC01_Pos)               /*!< PWM_T::CLKPSC: CLKPSC01 Mask              */

#define PWM_CLKPSC_CLKPSC23_Pos          (8)                                               /*!< PWM_T::CLKPSC: CLKPSC23 Position          */
#define PWM_CLKPSC_CLKPSC23_Msk          (0xfful << PWM_CLKPSC_CLKPSC23_Pos)               /*!< PWM_T::CLKPSC: CLKPSC23 Mask              */

#define PWM_CLKPSC_CLKPSC45_Pos          (16)                                              /*!< PWM_T::CLKPSC: CLKPSC45 Position          */
#define PWM_CLKPSC_CLKPSC45_Msk          (0xfful << PWM_CLKPSC_CLKPSC45_Pos)               /*!< PWM_T::CLKPSC: CLKPSC45 Mask              */

#define PWM_CLKPSC_CLKPSC67_Pos          (24)                                              /*!< PWM_T::CLKPSC: CLKPSC67 Position          */
#define PWM_CLKPSC_CLKPSC67_Msk          (0xfful << PWM_CLKPSC_CLKPSC67_Pos)               /*!< PWM_T::CLKPSC: CLKPSC67 Mask              */

#define PWM_CLKDIV_CLKDIV0_Pos           (0)                                               /*!< PWM_T::CLKDIV: CLKDIV0 Position           */
#define PWM_CLKDIV_CLKDIV0_Msk           (0x7ul << PWM_CLKDIV_CLKDIV0_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV0 Mask               */

#define PWM_CLKDIV_CLKDIV1_Pos           (4)                                               /*!< PWM_T::CLKDIV: CLKDIV1 Position           */
#define PWM_CLKDIV_CLKDIV1_Msk           (0x7ul << PWM_CLKDIV_CLKDIV1_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV1 Mask               */

#define PWM_CLKDIV_CLKDIV2_Pos           (8)                                               /*!< PWM_T::CLKDIV: CLKDIV2 Position           */
#define PWM_CLKDIV_CLKDIV2_Msk           (0x7ul << PWM_CLKDIV_CLKDIV2_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV2 Mask               */

#define PWM_CLKDIV_CLKDIV3_Pos           (12)                                              /*!< PWM_T::CLKDIV: CLKDIV3 Position           */
#define PWM_CLKDIV_CLKDIV3_Msk           (0x7ul << PWM_CLKDIV_CLKDIV3_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV3 Mask               */

#define PWM_CLKDIV_CLKDIV4_Pos           (16)                                              /*!< PWM_T::CLKDIV: CLKDIV4 Position           */
#define PWM_CLKDIV_CLKDIV4_Msk           (0x7ul << PWM_CLKDIV_CLKDIV4_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV4 Mask               */

#define PWM_CLKDIV_CLKDIV5_Pos           (20)                                              /*!< PWM_T::CLKDIV: CLKDIV5 Position           */
#define PWM_CLKDIV_CLKDIV5_Msk           (0x7ul << PWM_CLKDIV_CLKDIV5_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV5 Mask               */


#define PWM_CLKDIV_CLKDIV6_Pos           (24)                                              /*!< PWM_T::CLKDIV: CLKDIV6 Position           */
#define PWM_CLKDIV_CLKDIV6_Msk           (0x7ul << PWM_CLKDIV_CLKDIV6_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV6 Mask               */

#define PWM_CLKDIV_CLKDIV7_Pos           (28)                                              /*!< PWM_T::CLKDIV: CLKDIV7 Position           */
#define PWM_CLKDIV_CLKDIV7_Msk           (0x7ul << PWM_CLKDIV_CLKDIV7_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV7 Mask               */

#define PWM_CTL_CNTEN0_Pos               (0)                                               /*!< PWM_T::CTL: CNTEN0 Position               */
#define PWM_CTL_CNTEN0_Msk               (0x1ul << PWM_CTL_CNTEN0_Pos)                     /*!< PWM_T::CTL: CNTEN0 Mask                   */

#define PWM_CTL_PINV0_Pos                (2)                                               /*!< PWM_T::CTL: PINV0 Position                */
#define PWM_CTL_PINV0_Msk                (0x1ul << PWM_CTL_PINV0_Pos)                      /*!< PWM_T::CTL: PINV0 Mask                    */

#define PWM_CTL_CNTEN1_Pos               (4)                                               /*!< PWM_T::CTL: CNTEN1 Position               */
#define PWM_CTL_CNTEN1_Msk               (0x1ul << PWM_CTL_CNTEN1_Pos)                     /*!< PWM_T::CTL: CNTEN1 Mask                   */

#define PWM_CTL_HCUPDT_Pos               (5)                                               /*!< PWM_T::CTL: HCUPDT Position               */
#define PWM_CTL_HCUPDT_Msk               (0x1ul << PWM_CTL_HCUPDT_Pos)                     /*!< PWM_T::CTL: HCUPDT Mask                   */

#define PWM_CTL_PINV1_Pos                (6)                                               /*!< PWM_T::CTL: PINV1 Position                */
#define PWM_CTL_PINV1_Msk                (0x1ul << PWM_CTL_PINV1_Pos)                      /*!< PWM_T::CTL: PINV1 Mask                    */

#define PWM_CTL_CNTMODE1_Pos             (7)                                               /*!< PWM_T::CTL: CNTMODE1 Position             */
#define PWM_CTL_CNTMODE1_Msk             (0x1ul << PWM_CTL_CNTMODE1_Pos)                   /*!< PWM_T::CTL: CNTMODE1 Mask                 */

#define PWM_CTL_CNTEN2_Pos               (8)                                               /*!< PWM_T::CTL: CNTEN2 Position               */
#define PWM_CTL_CNTEN2_Msk               (0x1ul << PWM_CTL_CNTEN2_Pos)                     /*!< PWM_T::CTL: CNTEN2 Mask                   */

#define PWM_CTL_PINV2_Pos                (10)                                              /*!< PWM_T::CTL: PINV2 Position                */
#define PWM_CTL_PINV2_Msk                (0x1ul << PWM_CTL_PINV2_Pos)                      /*!< PWM_T::CTL: PINV2 Mask                    */

#define PWM_CTL_CNTMODE2_Pos             (11)                                              /*!< PWM_T::CTL: CNTMODE2 Position             */
#define PWM_CTL_CNTMODE2_Msk             (0x1ul << PWM_CTL_CNTMODE2_Pos)                   /*!< PWM_T::CTL: CNTMODE2 Mask                 */

#define PWM_CTL_CNTEN3_Pos               (12)                                              /*!< PWM_T::CTL: CNTEN3 Position               */
#define PWM_CTL_CNTEN3_Msk               (0x1ul << PWM_CTL_CNTEN3_Pos)                     /*!< PWM_T::CTL: CNTEN3 Mask                   */

#define PWM_CTL_PINV3_Pos                (14)                                              /*!< PWM_T::CTL: PINV3 Position                */
#define PWM_CTL_PINV3_Msk                (0x1ul << PWM_CTL_PINV3_Pos)                      /*!< PWM_T::CTL: PINV3 Mask                    */

#define PWM_CTL_CNTMODE3_Pos             (15)                                              /*!< PWM_T::CTL: CNTMODE3 Position             */
#define PWM_CTL_CNTMODE3_Msk             (0x1ul << PWM_CTL_CNTMODE3_Pos)                   /*!< PWM_T::CTL: CNTMODE3 Mask                 */

#define PWM_CTL_CNTEN4_Pos               (16)                                              /*!< PWM_T::CTL: CNTEN4 Position               */
#define PWM_CTL_CNTEN4_Msk               (0x1ul << PWM_CTL_CNTEN4_Pos)                     /*!< PWM_T::CTL: CNTEN4 Mask                   */

#define PWM_CTL_PINV4_Pos                (18)                                              /*!< PWM_T::CTL: PINV4 Position                */
#define PWM_CTL_PINV4_Msk                (0x1ul << PWM_CTL_PINV4_Pos)                      /*!< PWM_T::CTL: PINV4 Mask                    */

#define PWM_CTL_CNTMODE4_Pos             (19)                                              /*!< PWM_T::CTL: CNTMODE4 Position             */
#define PWM_CTL_CNTMODE4_Msk             (0x1ul << PWM_CTL_CNTMODE4_Pos)                   /*!< PWM_T::CTL: CNTMODE4 Mask                 */

#define PWM_CTL_CNTEN5_Pos               (20)                                              /*!< PWM_T::CTL: CNTEN5 Position               */
#define PWM_CTL_CNTEN5_Msk               (0x1ul << PWM_CTL_CNTEN5_Pos)                     /*!< PWM_T::CTL: CNTEN5 Mask                   */

#define PWM_CTL_ASYMEN_Pos               (21)                                              /*!< PWM_T::CTL: ASYMEN Position               */
#define PWM_CTL_ASYMEN_Msk               (0x1ul << PWM_CTL_ASYMEN_Pos)                     /*!< PWM_T::CTL: ASYMEN Mask                   */

#define PWM_CTL_PINV5_Pos                (22)                                              /*!< PWM_T::CTL: PINV5 Position                */
#define PWM_CTL_PINV5_Msk                (0x1ul << PWM_CTL_PINV5_Pos)                      /*!< PWM_T::CTL: PINV5 Mask                    */

#define PWM_CTL_CNTMODE5_Pos             (23)                                              /*!< PWM_T::CTL: CNTMODE5 Position             */
#define PWM_CTL_CNTMODE5_Msk             (0x1ul << PWM_CTL_CNTMODE5_Pos)                   /*!< PWM_T::CTL: CNTMODE5 Mask                 */

#define PWM_CTL_CNTEN6_Pos               (24)                                              /*!< PWM_T::CTL: CNTEN6 Position               */
#define PWM_CTL_CNTEN6_Msk               (0x1ul << PWM_CTL_CNTEN6_Pos)                     /*!< PWM_T::CTL: CNTEN6 Mask                   */

#define PWM_CTL_PINV6_Pos                (26)                                              /*!< PWM_T::CTL: PINV6 Position                */
#define PWM_CTL_PINV6_Msk                (0x1ul << PWM_CTL_PINV6_Pos)                      /*!< PWM_T::CTL: PINV6 Mask                    */

#define PWM_CTL_CNTMODE6_Pos             (27)                                              /*!< PWM_T::CTL: CNTMODE6 Position             */
#define PWM_CTL_CNTMODE6_Msk             (0x1ul << PWM_CTL_CNTMODE6_Pos)                   /*!< PWM_T::CTL: CNTMODE6 Mask                 */

#define PWM_CTL_CNTEN7_Pos               (28)                                              /*!< PWM_T::CTL: CNTEN7 Position               */
#define PWM_CTL_CNTEN7_Msk               (0x1ul << PWM_CTL_CNTEN7_Pos)                     /*!< PWM_T::CTL: CNTEN7 Mask                   */

#define PWM_CTL_PINV7_Pos                (30)                                              /*!< PWM_T::CTL: PINV7 Position                */
#define PWM_CTL_PINV7_Msk                (0x1ul << PWM_CTL_PINV7_Pos)                      /*!< PWM_T::CTL: PINV7 Mask                    */

#define PWM_CTL_CNTMODE7_Pos             (31)                                              /*!< PWM_T::CTL: CNTMODE7 Position             */
#define PWM_CTL_CNTMODE7_Msk             (0x1ul << PWM_CTL_CNTMODE7_Pos)                   /*!< PWM_T::CTL: CNTMODE7 Mask                 */

#define PWM_PERIOD0_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD0 Position          */
#define PWM_PERIOD0_PERIOD0_Msk          (0xfffful << PWM_PERIOD0_PERIOD0_Pos)             /*!< PWM_T::PERIOD0: PERIOD0 Mask              */

#define PWM_PERIOD0_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD1 Position          */
#define PWM_PERIOD0_PERIOD1_Msk          (0xfffful << PWM_PERIOD0_PERIOD1_Pos)             /*!< PWM_T::PERIOD0: PERIOD1 Mask              */

#define PWM_PERIOD0_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD2 Position          */
#define PWM_PERIOD0_PERIOD2_Msk          (0xfffful << PWM_PERIOD0_PERIOD2_Pos)             /*!< PWM_T::PERIOD0: PERIOD2 Mask              */

#define PWM_PERIOD0_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD3 Position          */
#define PWM_PERIOD0_PERIOD3_Msk          (0xfffful << PWM_PERIOD0_PERIOD3_Pos)             /*!< PWM_T::PERIOD0: PERIOD3 Mask              */

#define PWM_PERIOD0_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD4 Position          */
#define PWM_PERIOD0_PERIOD4_Msk          (0xfffful << PWM_PERIOD0_PERIOD4_Pos)             /*!< PWM_T::PERIOD0: PERIOD4 Mask              */

#define PWM_PERIOD0_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD5 Position          */
#define PWM_PERIOD0_PERIOD5_Msk          (0xfffful << PWM_PERIOD0_PERIOD5_Pos)             /*!< PWM_T::PERIOD0: PERIOD5 Mask              */

#define PWM_PERIOD0_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD6 Position          */
#define PWM_PERIOD0_PERIOD6_Msk          (0xfffful << PWM_PERIOD0_PERIOD6_Pos)             /*!< PWM_T::PERIOD0: PERIOD6 Mask              */

#define PWM_PERIOD0_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD7 Position          */
#define PWM_PERIOD0_PERIOD7_Msk          (0xfffful << PWM_PERIOD0_PERIOD7_Pos)             /*!< PWM_T::PERIOD0: PERIOD7 Mask              */

#define PWM_PERIOD1_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD0 Position          */
#define PWM_PERIOD1_PERIOD0_Msk          (0xfffful << PWM_PERIOD1_PERIOD0_Pos)             /*!< PWM_T::PERIOD1: PERIOD0 Mask              */

#define PWM_PERIOD1_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD1 Position          */
#define PWM_PERIOD1_PERIOD1_Msk          (0xfffful << PWM_PERIOD1_PERIOD1_Pos)             /*!< PWM_T::PERIOD1: PERIOD1 Mask              */

#define PWM_PERIOD1_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD2 Position          */
#define PWM_PERIOD1_PERIOD2_Msk          (0xfffful << PWM_PERIOD1_PERIOD2_Pos)             /*!< PWM_T::PERIOD1: PERIOD2 Mask              */

#define PWM_PERIOD1_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD3 Position          */
#define PWM_PERIOD1_PERIOD3_Msk          (0xfffful << PWM_PERIOD1_PERIOD3_Pos)             /*!< PWM_T::PERIOD1: PERIOD3 Mask              */

#define PWM_PERIOD1_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD4 Position          */
#define PWM_PERIOD1_PERIOD4_Msk          (0xfffful << PWM_PERIOD1_PERIOD4_Pos)             /*!< PWM_T::PERIOD1: PERIOD4 Mask              */

#define PWM_PERIOD1_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD5 Position          */
#define PWM_PERIOD1_PERIOD5_Msk          (0xfffful << PWM_PERIOD1_PERIOD5_Pos)             /*!< PWM_T::PERIOD1: PERIOD5 Mask              */

#define PWM_PERIOD1_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD6 Position          */
#define PWM_PERIOD1_PERIOD6_Msk          (0xfffful << PWM_PERIOD1_PERIOD6_Pos)             /*!< PWM_T::PERIOD1: PERIOD6 Mask              */

#define PWM_PERIOD1_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD7 Position          */
#define PWM_PERIOD1_PERIOD7_Msk          (0xfffful << PWM_PERIOD1_PERIOD7_Pos)             /*!< PWM_T::PERIOD1: PERIOD7 Mask              */

#define PWM_PERIOD2_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD0 Position          */
#define PWM_PERIOD2_PERIOD0_Msk          (0xfffful << PWM_PERIOD2_PERIOD0_Pos)             /*!< PWM_T::PERIOD2: PERIOD0 Mask              */

#define PWM_PERIOD2_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD1 Position          */
#define PWM_PERIOD2_PERIOD1_Msk          (0xfffful << PWM_PERIOD2_PERIOD1_Pos)             /*!< PWM_T::PERIOD2: PERIOD1 Mask              */

#define PWM_PERIOD2_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD2 Position          */
#define PWM_PERIOD2_PERIOD2_Msk          (0xfffful << PWM_PERIOD2_PERIOD2_Pos)             /*!< PWM_T::PERIOD2: PERIOD2 Mask              */

#define PWM_PERIOD2_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD3 Position          */
#define PWM_PERIOD2_PERIOD3_Msk          (0xfffful << PWM_PERIOD2_PERIOD3_Pos)             /*!< PWM_T::PERIOD2: PERIOD3 Mask              */

#define PWM_PERIOD2_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD4 Position          */
#define PWM_PERIOD2_PERIOD4_Msk          (0xfffful << PWM_PERIOD2_PERIOD4_Pos)             /*!< PWM_T::PERIOD2: PERIOD4 Mask              */

#define PWM_PERIOD2_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD5 Position          */
#define PWM_PERIOD2_PERIOD5_Msk          (0xfffful << PWM_PERIOD2_PERIOD5_Pos)             /*!< PWM_T::PERIOD2: PERIOD5 Mask              */

#define PWM_PERIOD2_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD6 Position          */
#define PWM_PERIOD2_PERIOD6_Msk          (0xfffful << PWM_PERIOD2_PERIOD6_Pos)             /*!< PWM_T::PERIOD2: PERIOD6 Mask              */

#define PWM_PERIOD2_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD7 Position          */
#define PWM_PERIOD2_PERIOD7_Msk          (0xfffful << PWM_PERIOD2_PERIOD7_Pos)             /*!< PWM_T::PERIOD2: PERIOD7 Mask              */

#define PWM_PERIOD3_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD0 Position          */
#define PWM_PERIOD3_PERIOD0_Msk          (0xfffful << PWM_PERIOD3_PERIOD0_Pos)             /*!< PWM_T::PERIOD3: PERIOD0 Mask              */

#define PWM_PERIOD3_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD1 Position          */
#define PWM_PERIOD3_PERIOD1_Msk          (0xfffful << PWM_PERIOD3_PERIOD1_Pos)             /*!< PWM_T::PERIOD3: PERIOD1 Mask              */

#define PWM_PERIOD3_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD2 Position          */
#define PWM_PERIOD3_PERIOD2_Msk          (0xfffful << PWM_PERIOD3_PERIOD2_Pos)             /*!< PWM_T::PERIOD3: PERIOD2 Mask              */

#define PWM_PERIOD3_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD3 Position          */
#define PWM_PERIOD3_PERIOD3_Msk          (0xfffful << PWM_PERIOD3_PERIOD3_Pos)             /*!< PWM_T::PERIOD3: PERIOD3 Mask              */

#define PWM_PERIOD3_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD4 Position          */
#define PWM_PERIOD3_PERIOD4_Msk          (0xfffful << PWM_PERIOD3_PERIOD4_Pos)             /*!< PWM_T::PERIOD3: PERIOD4 Mask              */

#define PWM_PERIOD3_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD5 Position          */
#define PWM_PERIOD3_PERIOD5_Msk          (0xfffful << PWM_PERIOD3_PERIOD5_Pos)             /*!< PWM_T::PERIOD3: PERIOD5 Mask              */

#define PWM_PERIOD3_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD6 Position          */
#define PWM_PERIOD3_PERIOD6_Msk          (0xfffful << PWM_PERIOD3_PERIOD6_Pos)             /*!< PWM_T::PERIOD3: PERIOD6 Mask              */

#define PWM_PERIOD3_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD7 Position          */
#define PWM_PERIOD3_PERIOD7_Msk          (0xfffful << PWM_PERIOD3_PERIOD7_Pos)             /*!< PWM_T::PERIOD3: PERIOD7 Mask              */

#define PWM_PERIOD4_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD0 Position          */
#define PWM_PERIOD4_PERIOD0_Msk          (0xfffful << PWM_PERIOD4_PERIOD0_Pos)             /*!< PWM_T::PERIOD4: PERIOD0 Mask              */

#define PWM_PERIOD4_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD1 Position          */
#define PWM_PERIOD4_PERIOD1_Msk          (0xfffful << PWM_PERIOD4_PERIOD1_Pos)             /*!< PWM_T::PERIOD4: PERIOD1 Mask              */

#define PWM_PERIOD4_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD2 Position          */
#define PWM_PERIOD4_PERIOD2_Msk          (0xfffful << PWM_PERIOD4_PERIOD2_Pos)             /*!< PWM_T::PERIOD4: PERIOD2 Mask              */

#define PWM_PERIOD4_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD3 Position          */
#define PWM_PERIOD4_PERIOD3_Msk          (0xfffful << PWM_PERIOD4_PERIOD3_Pos)             /*!< PWM_T::PERIOD4: PERIOD3 Mask              */

#define PWM_PERIOD4_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD4 Position          */
#define PWM_PERIOD4_PERIOD4_Msk          (0xfffful << PWM_PERIOD4_PERIOD4_Pos)             /*!< PWM_T::PERIOD4: PERIOD4 Mask              */

#define PWM_PERIOD4_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD5 Position          */
#define PWM_PERIOD4_PERIOD5_Msk          (0xfffful << PWM_PERIOD4_PERIOD5_Pos)             /*!< PWM_T::PERIOD4: PERIOD5 Mask              */

#define PWM_PERIOD4_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD6 Position          */
#define PWM_PERIOD4_PERIOD6_Msk          (0xfffful << PWM_PERIOD4_PERIOD6_Pos)             /*!< PWM_T::PERIOD4: PERIOD6 Mask              */

#define PWM_PERIOD4_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD7 Position          */
#define PWM_PERIOD4_PERIOD7_Msk          (0xfffful << PWM_PERIOD4_PERIOD7_Pos)             /*!< PWM_T::PERIOD4: PERIOD7 Mask              */

#define PWM_PERIOD5_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD0 Position          */
#define PWM_PERIOD5_PERIOD0_Msk          (0xfffful << PWM_PERIOD5_PERIOD0_Pos)             /*!< PWM_T::PERIOD5: PERIOD0 Mask              */

#define PWM_PERIOD5_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD1 Position          */
#define PWM_PERIOD5_PERIOD1_Msk          (0xfffful << PWM_PERIOD5_PERIOD1_Pos)             /*!< PWM_T::PERIOD5: PERIOD1 Mask              */

#define PWM_PERIOD5_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD2 Position          */
#define PWM_PERIOD5_PERIOD2_Msk          (0xfffful << PWM_PERIOD5_PERIOD2_Pos)             /*!< PWM_T::PERIOD5: PERIOD2 Mask              */

#define PWM_PERIOD5_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD3 Position          */
#define PWM_PERIOD5_PERIOD3_Msk          (0xfffful << PWM_PERIOD5_PERIOD3_Pos)             /*!< PWM_T::PERIOD5: PERIOD3 Mask              */

#define PWM_PERIOD5_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD4 Position          */
#define PWM_PERIOD5_PERIOD4_Msk          (0xfffful << PWM_PERIOD5_PERIOD4_Pos)             /*!< PWM_T::PERIOD5: PERIOD4 Mask              */

#define PWM_PERIOD5_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD5 Position          */
#define PWM_PERIOD5_PERIOD5_Msk          (0xfffful << PWM_PERIOD5_PERIOD5_Pos)             /*!< PWM_T::PERIOD5: PERIOD5 Mask              */

#define PWM_PERIOD5_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD6 Position          */
#define PWM_PERIOD5_PERIOD6_Msk          (0xfffful << PWM_PERIOD6_PERIOD6_Pos)             /*!< PWM_T::PERIOD5: PERIOD6 Mask              */

#define PWM_PERIOD5_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD7 Position          */
#define PWM_PERIOD5_PERIOD7_Msk          (0xfffful << PWM_PERIOD6_PERIOD7_Pos)             /*!< PWM_T::PERIOD5: PERIOD7 Mask              */

#define PWM_PERIOD6_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD0 Position          */
#define PWM_PERIOD6_PERIOD0_Msk          (0xfffful << PWM_PERIOD6_PERIOD0_Pos)             /*!< PWM_T::PERIOD6: PERIOD0 Mask              */

#define PWM_PERIOD6_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD1 Position          */
#define PWM_PERIOD6_PERIOD1_Msk          (0xfffful << PWM_PERIOD6_PERIOD1_Pos)             /*!< PWM_T::PERIOD6: PERIOD1 Mask              */

#define PWM_PERIOD6_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD2 Position          */
#define PWM_PERIOD6_PERIOD2_Msk          (0xfffful << PWM_PERIOD6_PERIOD2_Pos)             /*!< PWM_T::PERIOD6: PERIOD2 Mask              */

#define PWM_PERIOD6_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD3 Position          */
#define PWM_PERIOD6_PERIOD3_Msk          (0xfffful << PWM_PERIOD6_PERIOD3_Pos)             /*!< PWM_T::PERIOD6: PERIOD3 Mask              */

#define PWM_PERIOD6_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD4 Position          */
#define PWM_PERIOD6_PERIOD4_Msk          (0xfffful << PWM_PERIOD6_PERIOD4_Pos)             /*!< PWM_T::PERIOD6: PERIOD4 Mask              */

#define PWM_PERIOD6_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD5 Position          */
#define PWM_PERIOD6_PERIOD5_Msk          (0xfffful << PWM_PERIOD6_PERIOD5_Pos)             /*!< PWM_T::PERIOD6: PERIOD5 Mask              */

#define PWM_PERIOD6_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD6 Position          */
#define PWM_PERIOD6_PERIOD6_Msk          (0xfffful << PWM_PERIOD6_PERIOD6_Pos)             /*!< PWM_T::PERIOD6: PERIOD6 Mask              */

#define PWM_PERIOD6_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD7 Position          */
#define PWM_PERIOD6_PERIOD7_Msk          (0xfffful << PWM_PERIOD6_PERIOD7_Pos)             /*!< PWM_T::PERIOD6: PERIOD7 Mask              */

#define PWM_PERIOD7_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD0 Position          */
#define PWM_PERIOD7_PERIOD0_Msk          (0xfffful << PWM_PERIOD7_PERIOD0_Pos)             /*!< PWM_T::PERIOD7: PERIOD0 Mask              */

#define PWM_PERIOD7_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD1 Position          */
#define PWM_PERIOD7_PERIOD1_Msk          (0xfffful << PWM_PERIOD7_PERIOD1_Pos)             /*!< PWM_T::PERIOD7: PERIOD1 Mask              */

#define PWM_PERIOD7_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD2 Position          */
#define PWM_PERIOD7_PERIOD2_Msk          (0xfffful << PWM_PERIOD7_PERIOD2_Pos)             /*!< PWM_T::PERIOD7: PERIOD2 Mask              */

#define PWM_PERIOD7_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD3 Position          */
#define PWM_PERIOD7_PERIOD3_Msk          (0xfffful << PWM_PERIOD7_PERIOD3_Pos)             /*!< PWM_T::PERIOD7: PERIOD3 Mask              */

#define PWM_PERIOD7_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD4 Position          */
#define PWM_PERIOD7_PERIOD4_Msk          (0xfffful << PWM_PERIOD7_PERIOD4_Pos)             /*!< PWM_T::PERIOD7: PERIOD4 Mask              */

#define PWM_PERIOD7_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD5 Position          */
#define PWM_PERIOD7_PERIOD5_Msk          (0xfffful << PWM_PERIOD7_PERIOD5_Pos)             /*!< PWM_T::PERIOD7: PERIOD5 Mask              */

#define PWM_PERIOD7_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD6 Position          */
#define PWM_PERIOD7_PERIOD6_Msk          (0xfffful << PWM_PERIOD7_PERIOD6_Pos)             /*!< PWM_T::PERIOD7: PERIOD6 Mask              */

#define PWM_PERIOD7_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD7 Position          */
#define PWM_PERIOD7_PERIOD7_Msk          (0xfffful << PWM_PERIOD7_PERIOD7_Pos)             /*!< PWM_T::PERIOD7: PERIOD7 Mask              */

#define PWM_CMPDAT0_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP0 Position             */
#define PWM_CMPDAT0_CMP0_Msk             (0xfffful << PWM_CMPDAT0_CMP0_Pos)                /*!< PWM_T::CMPDAT0: CMP0 Mask                 */

#define PWM_CMPDAT0_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP1 Position             */
#define PWM_CMPDAT0_CMP1_Msk             (0xfffful << PWM_CMPDAT0_CMP1_Pos)                /*!< PWM_T::CMPDAT0: CMP1 Mask                 */

#define PWM_CMPDAT0_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP2 Position             */
#define PWM_CMPDAT0_CMP2_Msk             (0xfffful << PWM_CMPDAT0_CMP2_Pos)                /*!< PWM_T::CMPDAT0: CMP2 Mask                 */

#define PWM_CMPDAT0_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP3 Position             */
#define PWM_CMPDAT0_CMP3_Msk             (0xfffful << PWM_CMPDAT0_CMP3_Pos)                /*!< PWM_T::CMPDAT0: CMP3 Mask                 */

#define PWM_CMPDAT0_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP4 Position             */
#define PWM_CMPDAT0_CMP4_Msk             (0xfffful << PWM_CMPDAT0_CMP4_Pos)                /*!< PWM_T::CMPDAT0: CMP4 Mask                 */

#define PWM_CMPDAT0_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP5 Position             */
#define PWM_CMPDAT0_CMP5_Msk             (0xfffful << PWM_CMPDAT0_CMP5_Pos)                /*!< PWM_T::CMPDAT0: CMP5 Mask                 */

#define PWM_CMPDAT0_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP6 Position             */
#define PWM_CMPDAT0_CMP6_Msk             (0xfffful << PWM_CMPDAT0_CMP6_Pos)                /*!< PWM_T::CMPDAT0: CMP6 Mask                 */

#define PWM_CMPDAT0_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP7 Position             */
#define PWM_CMPDAT0_CMP7_Msk             (0xfffful << PWM_CMPDAT0_CMP7_Pos)                /*!< PWM_T::CMPDAT0: CMP7 Mask                 */

#define PWM_CMPDAT0_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD0 Position            */
#define PWM_CMPDAT0_CMPD0_Msk            (0xfffful << PWM_CMPDAT0_CMPD0_Pos)               /*!< PWM_T::CMPDAT0: CMPD0 Mask                */

#define PWM_CMPDAT0_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD1 Position            */
#define PWM_CMPDAT0_CMPD1_Msk            (0xfffful << PWM_CMPDAT0_CMPD1_Pos)               /*!< PWM_T::CMPDAT0: CMPD1 Mask                */

#define PWM_CMPDAT0_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD2 Position            */
#define PWM_CMPDAT0_CMPD2_Msk            (0xfffful << PWM_CMPDAT0_CMPD2_Pos)               /*!< PWM_T::CMPDAT0: CMPD2 Mask                */

#define PWM_CMPDAT0_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD3 Position            */
#define PWM_CMPDAT0_CMPD3_Msk            (0xfffful << PWM_CMPDAT0_CMPD3_Pos)               /*!< PWM_T::CMPDAT0: CMPD3 Mask                */

#define PWM_CMPDAT0_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD4 Position            */
#define PWM_CMPDAT0_CMPD4_Msk            (0xfffful << PWM_CMPDAT0_CMPD4_Pos)               /*!< PWM_T::CMPDAT0: CMPD4 Mask                */

#define PWM_CMPDAT0_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD5 Position            */
#define PWM_CMPDAT0_CMPD5_Msk            (0xfffful << PWM_CMPDAT0_CMPD5_Pos)               /*!< PWM_T::CMPDAT0: CMPD5 Mask                */

#define PWM_CMPDAT0_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD6 Position            */
#define PWM_CMPDAT0_CMPD6_Msk            (0xfffful << PWM_CMPDAT0_CMPD6_Pos)               /*!< PWM_T::CMPDAT0: CMPD6 Mask                */

#define PWM_CMPDAT0_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD7 Position            */
#define PWM_CMPDAT0_CMPD7_Msk            (0xfffful << PWM_CMPDAT0_CMPD7_Pos)               /*!< PWM_T::CMPDAT0: CMPD7 Mask                */

#define PWM_CMPDAT1_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP0 Position             */
#define PWM_CMPDAT1_CMP0_Msk             (0xfffful << PWM_CMPDAT1_CMP0_Pos)                /*!< PWM_T::CMPDAT1: CMP0 Mask                 */

#define PWM_CMPDAT1_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP1 Position             */
#define PWM_CMPDAT1_CMP1_Msk             (0xfffful << PWM_CMPDAT1_CMP1_Pos)                /*!< PWM_T::CMPDAT1: CMP1 Mask                 */

#define PWM_CMPDAT1_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP2 Position             */
#define PWM_CMPDAT1_CMP2_Msk             (0xfffful << PWM_CMPDAT1_CMP2_Pos)                /*!< PWM_T::CMPDAT1: CMP2 Mask                 */

#define PWM_CMPDAT1_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP3 Position             */
#define PWM_CMPDAT1_CMP3_Msk             (0xfffful << PWM_CMPDAT1_CMP3_Pos)                /*!< PWM_T::CMPDAT1: CMP3 Mask                 */

#define PWM_CMPDAT1_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP4 Position             */
#define PWM_CMPDAT1_CMP4_Msk             (0xfffful << PWM_CMPDAT1_CMP4_Pos)                /*!< PWM_T::CMPDAT1: CMP4 Mask                 */

#define PWM_CMPDAT1_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP5 Position             */
#define PWM_CMPDAT1_CMP5_Msk             (0xfffful << PWM_CMPDAT1_CMP5_Pos)                /*!< PWM_T::CMPDAT1: CMP5 Mask                 */

#define PWM_CMPDAT1_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP6 Position             */
#define PWM_CMPDAT1_CMP6_Msk             (0xfffful << PWM_CMPDAT1_CMP6_Pos)                /*!< PWM_T::CMPDAT1: CMP6 Mask                 */

#define PWM_CMPDAT1_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP7 Position             */
#define PWM_CMPDAT1_CMP7_Msk             (0xfffful << PWM_CMPDAT1_CMP7_Pos)                /*!< PWM_T::CMPDAT1: CMP7 Mask                 */

#define PWM_CMPDAT1_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD0 Position            */
#define PWM_CMPDAT1_CMPD0_Msk            (0xfffful << PWM_CMPDAT1_CMPD0_Pos)               /*!< PWM_T::CMPDAT1: CMPD0 Mask                */

#define PWM_CMPDAT1_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD1 Position            */
#define PWM_CMPDAT1_CMPD1_Msk            (0xfffful << PWM_CMPDAT1_CMPD1_Pos)               /*!< PWM_T::CMPDAT1: CMPD1 Mask                */

#define PWM_CMPDAT1_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD2 Position            */
#define PWM_CMPDAT1_CMPD2_Msk            (0xfffful << PWM_CMPDAT1_CMPD2_Pos)               /*!< PWM_T::CMPDAT1: CMPD2 Mask                */

#define PWM_CMPDAT1_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD3 Position            */
#define PWM_CMPDAT1_CMPD3_Msk            (0xfffful << PWM_CMPDAT1_CMPD3_Pos)               /*!< PWM_T::CMPDAT1: CMPD3 Mask                */

#define PWM_CMPDAT1_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD4 Position            */
#define PWM_CMPDAT1_CMPD4_Msk            (0xfffful << PWM_CMPDAT1_CMPD4_Pos)               /*!< PWM_T::CMPDAT1: CMPD4 Mask                */

#define PWM_CMPDAT1_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD5 Position            */
#define PWM_CMPDAT1_CMPD5_Msk            (0xfffful << PWM_CMPDAT1_CMPD5_Pos)               /*!< PWM_T::CMPDAT1: CMPD5 Mask                */

#define PWM_CMPDAT1_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD6 Position            */
#define PWM_CMPDAT1_CMPD6_Msk            (0xfffful << PWM_CMPDAT1_CMPD6_Pos)               /*!< PWM_T::CMPDAT1: CMPD6 Mask                */

#define PWM_CMPDAT1_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD7 Position            */
#define PWM_CMPDAT1_CMPD7_Msk            (0xfffful << PWM_CMPDAT1_CMPD7_Pos)               /*!< PWM_T::CMPDAT1: CMPD7 Mask                */

#define PWM_CMPDAT2_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP0 Position             */
#define PWM_CMPDAT2_CMP0_Msk             (0xfffful << PWM_CMPDAT2_CMP0_Pos)                /*!< PWM_T::CMPDAT2: CMP0 Mask                 */

#define PWM_CMPDAT2_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP1 Position             */
#define PWM_CMPDAT2_CMP1_Msk             (0xfffful << PWM_CMPDAT2_CMP1_Pos)                /*!< PWM_T::CMPDAT2: CMP1 Mask                 */

#define PWM_CMPDAT2_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP2 Position             */
#define PWM_CMPDAT2_CMP2_Msk             (0xfffful << PWM_CMPDAT2_CMP2_Pos)                /*!< PWM_T::CMPDAT2: CMP2 Mask                 */

#define PWM_CMPDAT2_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP3 Position             */
#define PWM_CMPDAT2_CMP3_Msk             (0xfffful << PWM_CMPDAT2_CMP3_Pos)                /*!< PWM_T::CMPDAT2: CMP3 Mask                 */

#define PWM_CMPDAT2_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP4 Position             */
#define PWM_CMPDAT2_CMP4_Msk             (0xfffful << PWM_CMPDAT2_CMP4_Pos)                /*!< PWM_T::CMPDAT2: CMP4 Mask                 */

#define PWM_CMPDAT2_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP5 Position             */
#define PWM_CMPDAT2_CMP5_Msk             (0xfffful << PWM_CMPDAT2_CMP5_Pos)                /*!< PWM_T::CMPDAT2: CMP5 Mask                 */

#define PWM_CMPDAT2_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP6 Position             */
#define PWM_CMPDAT2_CMP6_Msk             (0xfffful << PWM_CMPDAT2_CMP6_Pos)                /*!< PWM_T::CMPDAT2: CMP6 Mask                 */

#define PWM_CMPDAT2_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP7 Position             */
#define PWM_CMPDAT2_CMP7_Msk             (0xfffful << PWM_CMPDAT2_CMP7_Pos)                /*!< PWM_T::CMPDAT2: CMP7 Mask                 */

#define PWM_CMPDAT2_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD0 Position            */
#define PWM_CMPDAT2_CMPD0_Msk            (0xfffful << PWM_CMPDAT2_CMPD0_Pos)               /*!< PWM_T::CMPDAT2: CMPD0 Mask                */

#define PWM_CMPDAT2_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD1 Position            */
#define PWM_CMPDAT2_CMPD1_Msk            (0xfffful << PWM_CMPDAT2_CMPD1_Pos)               /*!< PWM_T::CMPDAT2: CMPD1 Mask                */

#define PWM_CMPDAT2_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD2 Position            */
#define PWM_CMPDAT2_CMPD2_Msk            (0xfffful << PWM_CMPDAT2_CMPD2_Pos)               /*!< PWM_T::CMPDAT2: CMPD2 Mask                */

#define PWM_CMPDAT2_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD3 Position            */
#define PWM_CMPDAT2_CMPD3_Msk            (0xfffful << PWM_CMPDAT2_CMPD3_Pos)               /*!< PWM_T::CMPDAT2: CMPD3 Mask                */

#define PWM_CMPDAT2_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD4 Position            */
#define PWM_CMPDAT2_CMPD4_Msk            (0xfffful << PWM_CMPDAT2_CMPD4_Pos)               /*!< PWM_T::CMPDAT2: CMPD4 Mask                */

#define PWM_CMPDAT2_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD5 Position            */
#define PWM_CMPDAT2_CMPD5_Msk            (0xfffful << PWM_CMPDAT2_CMPD5_Pos)               /*!< PWM_T::CMPDAT2: CMPD5 Mask                */

#define PWM_CMPDAT2_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD6 Position            */
#define PWM_CMPDAT2_CMPD6_Msk            (0xfffful << PWM_CMPDAT2_CMPD6_Pos)               /*!< PWM_T::CMPDAT2: CMPD6 Mask                */

#define PWM_CMPDAT2_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD7 Position            */
#define PWM_CMPDAT2_CMPD7_Msk            (0xfffful << PWM_CMPDAT2_CMPD7_Pos)               /*!< PWM_T::CMPDAT2: CMPD7 Mask                */

#define PWM_CMPDAT3_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP0 Position             */
#define PWM_CMPDAT3_CMP0_Msk             (0xfffful << PWM_CMPDAT3_CMP0_Pos)                /*!< PWM_T::CMPDAT3: CMP0 Mask                 */

#define PWM_CMPDAT3_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP1 Position             */
#define PWM_CMPDAT3_CMP1_Msk             (0xfffful << PWM_CMPDAT3_CMP1_Pos)                /*!< PWM_T::CMPDAT3: CMP1 Mask                 */

#define PWM_CMPDAT3_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP2 Position             */
#define PWM_CMPDAT3_CMP2_Msk             (0xfffful << PWM_CMPDAT3_CMP2_Pos)                /*!< PWM_T::CMPDAT3: CMP2 Mask                 */

#define PWM_CMPDAT3_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP3 Position             */
#define PWM_CMPDAT3_CMP3_Msk             (0xfffful << PWM_CMPDAT3_CMP3_Pos)                /*!< PWM_T::CMPDAT3: CMP3 Mask                 */

#define PWM_CMPDAT3_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP4 Position             */
#define PWM_CMPDAT3_CMP4_Msk             (0xfffful << PWM_CMPDAT3_CMP4_Pos)                /*!< PWM_T::CMPDAT3: CMP4 Mask                 */

#define PWM_CMPDAT3_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP5 Position             */
#define PWM_CMPDAT3_CMP5_Msk             (0xfffful << PWM_CMPDAT3_CMP5_Pos)                /*!< PWM_T::CMPDAT3: CMP5 Mask                 */

#define PWM_CMPDAT3_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP6 Position             */
#define PWM_CMPDAT3_CMP6_Msk             (0xfffful << PWM_CMPDAT3_CMP6_Pos)                /*!< PWM_T::CMPDAT3: CMP6 Mask                 */

#define PWM_CMPDAT3_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP7 Position             */
#define PWM_CMPDAT3_CMP7_Msk             (0xfffful << PWM_CMPDAT3_CMP7_Pos)                /*!< PWM_T::CMPDAT3: CMP7 Mask                 */

#define PWM_CMPDAT3_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD0 Position            */
#define PWM_CMPDAT3_CMPD0_Msk            (0xfffful << PWM_CMPDAT3_CMPD0_Pos)               /*!< PWM_T::CMPDAT3: CMPD0 Mask                */

#define PWM_CMPDAT3_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD1 Position            */
#define PWM_CMPDAT3_CMPD1_Msk            (0xfffful << PWM_CMPDAT3_CMPD1_Pos)               /*!< PWM_T::CMPDAT3: CMPD1 Mask                */

#define PWM_CMPDAT3_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD2 Position            */
#define PWM_CMPDAT3_CMPD2_Msk            (0xfffful << PWM_CMPDAT3_CMPD2_Pos)               /*!< PWM_T::CMPDAT3: CMPD2 Mask                */

#define PWM_CMPDAT3_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD3 Position            */
#define PWM_CMPDAT3_CMPD3_Msk            (0xfffful << PWM_CMPDAT3_CMPD3_Pos)               /*!< PWM_T::CMPDAT3: CMPD3 Mask                */

#define PWM_CMPDAT3_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD4 Position            */
#define PWM_CMPDAT3_CMPD4_Msk            (0xfffful << PWM_CMPDAT3_CMPD4_Pos)               /*!< PWM_T::CMPDAT3: CMPD4 Mask                */

#define PWM_CMPDAT3_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD5 Position            */
#define PWM_CMPDAT3_CMPD5_Msk            (0xfffful << PWM_CMPDAT3_CMPD5_Pos)               /*!< PWM_T::CMPDAT3: CMPD5 Mask                */

#define PWM_CMPDAT3_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD6 Position            */
#define PWM_CMPDAT3_CMPD6_Msk            (0xfffful << PWM_CMPDAT3_CMPD4_Pos)               /*!< PWM_T::CMPDAT3: CMPD6 Mask                */

#define PWM_CMPDAT3_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD7 Position            */
#define PWM_CMPDAT3_CMPD7_Msk            (0xfffful << PWM_CMPDAT3_CMPD7_Pos)               /*!< PWM_T::CMPDAT3: CMPD7 Mask                */

#define PWM_CMPDAT4_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP0 Position             */
#define PWM_CMPDAT4_CMP0_Msk             (0xfffful << PWM_CMPDAT4_CMP0_Pos)                /*!< PWM_T::CMPDAT4: CMP0 Mask                 */

#define PWM_CMPDAT4_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP1 Position             */
#define PWM_CMPDAT4_CMP1_Msk             (0xfffful << PWM_CMPDAT4_CMP1_Pos)                /*!< PWM_T::CMPDAT4: CMP1 Mask                 */

#define PWM_CMPDAT4_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP2 Position             */
#define PWM_CMPDAT4_CMP2_Msk             (0xfffful << PWM_CMPDAT4_CMP2_Pos)                /*!< PWM_T::CMPDAT4: CMP2 Mask                 */

#define PWM_CMPDAT4_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP3 Position             */
#define PWM_CMPDAT4_CMP3_Msk             (0xfffful << PWM_CMPDAT4_CMP3_Pos)                /*!< PWM_T::CMPDAT4: CMP3 Mask                 */

#define PWM_CMPDAT4_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP4 Position             */
#define PWM_CMPDAT4_CMP4_Msk             (0xfffful << PWM_CMPDAT4_CMP4_Pos)                /*!< PWM_T::CMPDAT4: CMP4 Mask                 */

#define PWM_CMPDAT4_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP5 Position             */
#define PWM_CMPDAT4_CMP5_Msk             (0xfffful << PWM_CMPDAT4_CMP5_Pos)                /*!< PWM_T::CMPDAT4: CMP5 Mask                 */

#define PWM_CMPDAT4_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP6 Position             */
#define PWM_CMPDAT4_CMP6_Msk             (0xfffful << PWM_CMPDAT4_CMP6_Pos)                /*!< PWM_T::CMPDAT4: CMP6 Mask                 */

#define PWM_CMPDAT4_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP7 Position             */
#define PWM_CMPDAT4_CMP7_Msk             (0xfffful << PWM_CMPDAT4_CMP7_Pos)                /*!< PWM_T::CMPDAT4: CMP7 Mask                 */

#define PWM_CMPDAT4_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD0 Position            */
#define PWM_CMPDAT4_CMPD0_Msk            (0xfffful << PWM_CMPDAT4_CMPD0_Pos)               /*!< PWM_T::CMPDAT4: CMPD0 Mask                */

#define PWM_CMPDAT4_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD1 Position            */
#define PWM_CMPDAT4_CMPD1_Msk            (0xfffful << PWM_CMPDAT4_CMPD1_Pos)               /*!< PWM_T::CMPDAT4: CMPD1 Mask                */

#define PWM_CMPDAT4_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD2 Position            */
#define PWM_CMPDAT4_CMPD2_Msk            (0xfffful << PWM_CMPDAT4_CMPD2_Pos)               /*!< PWM_T::CMPDAT4: CMPD2 Mask                */

#define PWM_CMPDAT4_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD3 Position            */
#define PWM_CMPDAT4_CMPD3_Msk            (0xfffful << PWM_CMPDAT4_CMPD3_Pos)               /*!< PWM_T::CMPDAT4: CMPD3 Mask                */

#define PWM_CMPDAT4_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD4 Position            */
#define PWM_CMPDAT4_CMPD4_Msk            (0xfffful << PWM_CMPDAT4_CMPD4_Pos)               /*!< PWM_T::CMPDAT4: CMPD4 Mask                */

#define PWM_CMPDAT4_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD5 Position            */
#define PWM_CMPDAT4_CMPD5_Msk            (0xfffful << PWM_CMPDAT4_CMPD5_Pos)               /*!< PWM_T::CMPDAT4: CMPD5 Mask                */

#define PWM_CMPDAT4_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD6 Position            */
#define PWM_CMPDAT4_CMPD6_Msk            (0xfffful << PWM_CMPDAT4_CMPD6_Pos)               /*!< PWM_T::CMPDAT4: CMPD6 Mask                */

#define PWM_CMPDAT4_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD7 Position            */
#define PWM_CMPDAT4_CMPD7_Msk            (0xfffful << PWM_CMPDAT4_CMPD7_Pos)               /*!< PWM_T::CMPDAT4: CMPD7 Mask                */

#define PWM_CMPDAT5_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP0 Position             */
#define PWM_CMPDAT5_CMP0_Msk             (0xfffful << PWM_CMPDAT5_CMP0_Pos)                /*!< PWM_T::CMPDAT5: CMP0 Mask                 */

#define PWM_CMPDAT5_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP1 Position             */
#define PWM_CMPDAT5_CMP1_Msk             (0xfffful << PWM_CMPDAT5_CMP1_Pos)                /*!< PWM_T::CMPDAT5: CMP1 Mask                 */

#define PWM_CMPDAT5_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP2 Position             */
#define PWM_CMPDAT5_CMP2_Msk             (0xfffful << PWM_CMPDAT5_CMP2_Pos)                /*!< PWM_T::CMPDAT5: CMP2 Mask                 */

#define PWM_CMPDAT5_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP3 Position             */
#define PWM_CMPDAT5_CMP3_Msk             (0xfffful << PWM_CMPDAT5_CMP3_Pos)                /*!< PWM_T::CMPDAT5: CMP3 Mask                 */

#define PWM_CMPDAT5_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP4 Position             */
#define PWM_CMPDAT5_CMP4_Msk             (0xfffful << PWM_CMPDAT5_CMP4_Pos)                /*!< PWM_T::CMPDAT5: CMP4 Mask                 */

#define PWM_CMPDAT5_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP5 Position             */
#define PWM_CMPDAT5_CMP5_Msk             (0xfffful << PWM_CMPDAT5_CMP5_Pos)                /*!< PWM_T::CMPDAT5: CMP5 Mask                 */

#define PWM_CMPDAT5_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP6 Position             */
#define PWM_CMPDAT5_CMP6_Msk             (0xfffful << PWM_CMPDAT5_CMP6_Pos)                /*!< PWM_T::CMPDAT5: CMP6 Mask                 */

#define PWM_CMPDAT5_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP7 Position             */
#define PWM_CMPDAT5_CMP7_Msk             (0xfffful << PWM_CMPDAT5_CMP7_Pos)                /*!< PWM_T::CMPDAT5: CMP7 Mask                 */

#define PWM_CMPDAT5_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD0 Position            */
#define PWM_CMPDAT5_CMPD0_Msk            (0xfffful << PWM_CMPDAT5_CMPD0_Pos)               /*!< PWM_T::CMPDAT5: CMPD0 Mask                */

#define PWM_CMPDAT5_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD1 Position            */
#define PWM_CMPDAT5_CMPD1_Msk            (0xfffful << PWM_CMPDAT5_CMPD1_Pos)               /*!< PWM_T::CMPDAT5: CMPD1 Mask                */

#define PWM_CMPDAT5_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD2 Position            */
#define PWM_CMPDAT5_CMPD2_Msk            (0xfffful << PWM_CMPDAT5_CMPD2_Pos)               /*!< PWM_T::CMPDAT5: CMPD2 Mask                */

#define PWM_CMPDAT5_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD3 Position            */
#define PWM_CMPDAT5_CMPD3_Msk            (0xfffful << PWM_CMPDAT5_CMPD3_Pos)               /*!< PWM_T::CMPDAT5: CMPD3 Mask                */

#define PWM_CMPDAT5_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD4 Position            */
#define PWM_CMPDAT5_CMPD4_Msk            (0xfffful << PWM_CMPDAT5_CMPD4_Pos)               /*!< PWM_T::CMPDAT5: CMPD4 Mask                */

#define PWM_CMPDAT5_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD5 Position            */
#define PWM_CMPDAT5_CMPD5_Msk            (0xfffful << PWM_CMPDAT5_CMPD5_Pos)               /*!< PWM_T::CMPDAT5: CMPD5 Mask                */

#define PWM_CMPDAT5_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD6 Position            */
#define PWM_CMPDAT5_CMPD6_Msk            (0xfffful << PWM_CMPDAT5_CMPD6_Pos)               /*!< PWM_T::CMPDAT5: CMPD6 Mask                */

#define PWM_CMPDAT5_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD7 Position            */
#define PWM_CMPDAT5_CMPD7_Msk            (0xfffful << PWM_CMPDAT5_CMPD7_Pos)               /*!< PWM_T::CMPDAT5: CMPD7 Mask                */

#define PWM_CMPDAT6_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP0 Position             */
#define PWM_CMPDAT6_CMP0_Msk             (0xfffful << PWM_CMPDAT6_CMP0_Pos)                /*!< PWM_T::CMPDAT6: CMP0 Mask                 */

#define PWM_CMPDAT6_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP1 Position             */
#define PWM_CMPDAT6_CMP1_Msk             (0xfffful << PWM_CMPDAT6_CMP1_Pos)                /*!< PWM_T::CMPDAT6: CMP1 Mask                 */

#define PWM_CMPDAT6_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP2 Position             */
#define PWM_CMPDAT6_CMP2_Msk             (0xfffful << PWM_CMPDAT6_CMP2_Pos)                /*!< PWM_T::CMPDAT6: CMP2 Mask                 */

#define PWM_CMPDAT6_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP3 Position             */
#define PWM_CMPDAT6_CMP3_Msk             (0xfffful << PWM_CMPDAT6_CMP3_Pos)                /*!< PWM_T::CMPDAT6: CMP3 Mask                 */

#define PWM_CMPDAT6_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP4 Position             */
#define PWM_CMPDAT6_CMP4_Msk             (0xfffful << PWM_CMPDAT6_CMP4_Pos)                /*!< PWM_T::CMPDAT6: CMP4 Mask                 */

#define PWM_CMPDAT6_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP5 Position             */
#define PWM_CMPDAT6_CMP5_Msk             (0xfffful << PWM_CMPDAT6_CMP5_Pos)                /*!< PWM_T::CMPDAT6: CMP5 Mask                 */

#define PWM_CMPDAT6_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP6 Position             */
#define PWM_CMPDAT6_CMP6_Msk             (0xfffful << PWM_CMPDAT6_CMP6_Pos)                /*!< PWM_T::CMPDAT6: CMP6 Mask                 */

#define PWM_CMPDAT6_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP7 Position             */
#define PWM_CMPDAT6_CMP7_Msk             (0xfffful << PWM_CMPDAT6_CMP7_Pos)                /*!< PWM_T::CMPDAT6: CMP7 Mask                 */

#define PWM_CMPDAT6_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD0 Position            */
#define PWM_CMPDAT6_CMPD0_Msk            (0xfffful << PWM_CMPDAT6_CMPD0_Pos)               /*!< PWM_T::CMPDAT6: CMPD0 Mask                */

#define PWM_CMPDAT6_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD1 Position            */
#define PWM_CMPDAT6_CMPD1_Msk            (0xfffful << PWM_CMPDAT6_CMPD1_Pos)               /*!< PWM_T::CMPDAT6: CMPD1 Mask                */

#define PWM_CMPDAT6_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD2 Position            */
#define PWM_CMPDAT6_CMPD2_Msk            (0xfffful << PWM_CMPDAT6_CMPD2_Pos)               /*!< PWM_T::CMPDAT6: CMPD2 Mask                */

#define PWM_CMPDAT6_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD3 Position            */
#define PWM_CMPDAT6_CMPD3_Msk            (0xfffful << PWM_CMPDAT6_CMPD3_Pos)               /*!< PWM_T::CMPDAT6: CMPD3 Mask                */

#define PWM_CMPDAT6_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD4 Position            */
#define PWM_CMPDAT6_CMPD4_Msk            (0xfffful << PWM_CMPDAT6_CMPD4_Pos)               /*!< PWM_T::CMPDAT6: CMPD4 Mask                */

#define PWM_CMPDAT6_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD5 Position            */
#define PWM_CMPDAT6_CMPD5_Msk            (0xfffful << PWM_CMPDAT6_CMPD5_Pos)               /*!< PWM_T::CMPDAT6: CMPD5 Mask                */

#define PWM_CMPDAT6_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD6 Position            */
#define PWM_CMPDAT6_CMPD6_Msk            (0xfffful << PWM_CMPDAT6_CMPD6_Pos)               /*!< PWM_T::CMPDAT6: CMPD6 Mask                */

#define PWM_CMPDAT6_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD7 Position            */
#define PWM_CMPDAT6_CMPD7_Msk            (0xfffful << PWM_CMPDAT6_CMPD7_Pos)               /*!< PWM_T::CMPDAT6: CMPD7 Mask                */

#define PWM_CMPDAT7_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP0 Position             */
#define PWM_CMPDAT7_CMP0_Msk             (0xfffful << PWM_CMPDAT7_CMP0_Pos)                /*!< PWM_T::CMPDAT7: CMP0 Mask                 */

#define PWM_CMPDAT7_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP1 Position             */
#define PWM_CMPDAT7_CMP1_Msk             (0xfffful << PWM_CMPDAT7_CMP1_Pos)                /*!< PWM_T::CMPDAT7: CMP1 Mask                 */

#define PWM_CMPDAT7_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP2 Position             */
#define PWM_CMPDAT7_CMP2_Msk             (0xfffful << PWM_CMPDAT7_CMP2_Pos)                /*!< PWM_T::CMPDAT7: CMP2 Mask                 */

#define PWM_CMPDAT7_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP3 Position             */
#define PWM_CMPDAT7_CMP3_Msk             (0xfffful << PWM_CMPDAT7_CMP3_Pos)                /*!< PWM_T::CMPDAT7: CMP3 Mask                 */

#define PWM_CMPDAT7_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP4 Position             */
#define PWM_CMPDAT7_CMP4_Msk             (0xfffful << PWM_CMPDAT7_CMP4_Pos)                /*!< PWM_T::CMPDAT7: CMP4 Mask                 */

#define PWM_CMPDAT7_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP5 Position             */
#define PWM_CMPDAT7_CMP5_Msk             (0xfffful << PWM_CMPDAT7_CMP5_Pos)                /*!< PWM_T::CMPDAT7: CMP5 Mask                 */

#define PWM_CMPDAT7_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP6 Position             */
#define PWM_CMPDAT7_CMP6_Msk             (0xfffful << PWM_CMPDAT7_CMP6_Pos)                /*!< PWM_T::CMPDAT7: CMP6 Mask                 */

#define PWM_CMPDAT7_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP7 Position             */
#define PWM_CMPDAT7_CMP7_Msk             (0xfffful << PWM_CMPDAT7_CMP7_Pos)                /*!< PWM_T::CMPDAT7: CMP7 Mask                 */

#define PWM_CMPDAT7_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD0 Position            */
#define PWM_CMPDAT7_CMPD0_Msk            (0xfffful << PWM_CMPDAT7_CMPD0_Pos)               /*!< PWM_T::CMPDAT7: CMPD0 Mask                */

#define PWM_CMPDAT7_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD1 Position            */
#define PWM_CMPDAT7_CMPD1_Msk            (0xfffful << PWM_CMPDAT7_CMPD1_Pos)               /*!< PWM_T::CMPDAT7: CMPD1 Mask                */

#define PWM_CMPDAT7_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD2 Position            */
#define PWM_CMPDAT7_CMPD2_Msk            (0xfffful << PWM_CMPDAT7_CMPD2_Pos)               /*!< PWM_T::CMPDAT7: CMPD2 Mask                */

#define PWM_CMPDAT7_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD3 Position            */
#define PWM_CMPDAT7_CMPD3_Msk            (0xfffful << PWM_CMPDAT7_CMPD3_Pos)               /*!< PWM_T::CMPDAT7: CMPD3 Mask                */

#define PWM_CMPDAT7_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD4 Position            */
#define PWM_CMPDAT7_CMPD4_Msk            (0xfffful << PWM_CMPDAT7_CMPD4_Pos)               /*!< PWM_T::CMPDAT7: CMPD4 Mask                */

#define PWM_CMPDAT7_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD5 Position            */
#define PWM_CMPDAT7_CMPD5_Msk            (0xfffful << PWM_CMPDAT7_CMPD5_Pos)               /*!< PWM_T::CMPDAT7: CMPD5 Mask                */

#define PWM_CMPDAT7_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD6 Position            */
#define PWM_CMPDAT7_CMPD6_Msk            (0xfffful << PWM_CMPDAT7_CMPD6_Pos)               /*!< PWM_T::CMPDAT7: CMPD6 Mask                */

#define PWM_CMPDAT7_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD7 Position            */
#define PWM_CMPDAT7_CMPD7_Msk            (0xfffful << PWM_CMPDAT7_CMPD7_Pos)               /*!< PWM_T::CMPDAT7: CMPD7 Mask                */

#define PWM_CTL2_BRKIEN_Pos              (16)                                              /*!< PWM_T::CTL2: BRKIEN Position              */
#define PWM_CTL2_BRKIEN_Msk              (0x1ul << PWM_CTL2_BRKIEN_Pos)                    /*!< PWM_T::CTL2: BRKIEN Mask                  */

#define PWM_CTL2_PINTTYPE_Pos            (17)                                              /*!< PWM_T::CTL2: PINTTYPE Position            */
#define PWM_CTL2_PINTTYPE_Msk            (0x1ul << PWM_CTL2_PINTTYPE_Pos)                  /*!< PWM_T::CTL2: PINTTYPE Mask                */

#define PWM_CTL2_DTCNT01_Pos              (24)                                             /*!< PWM_T::CTL2: DTCNT01 Position             */
#define PWM_CTL2_DTCNT01_Msk              (0x1ul << PWM_CTL2_DTCNT01_Pos)                  /*!< PWM_T::CTL2: DTCNT01 Mask                 */

#define PWM_CTL2_DTCNT23_Pos              (25)                                             /*!< PWM_T::CTL2: DTCNT23 Position             */
#define PWM_CTL2_DTCNT23_Msk              (0x1ul << PWM_CTL2_DTCNT23_Pos)                  /*!< PWM_T::CTL2: DTCNT23 Mask                 */

#define PWM_CTL2_DTCNT45_Pos              (26)                                             /*!< PWM_T::CTL2: DTCNT45 Position             */
#define PWM_CTL2_DTCNT45_Msk              (0x1ul << PWM_CTL2_DTCNT45_Pos)                  /*!< PWM_T::CTL2: DTCNT45 Mask                 */

#define PWM_CTL2_DTCNT67_Pos              (27)                                             /*!< PWM_T::CTL2: DTCNT67 Position             */
#define PWM_CTL2_DTCNT67_Msk              (0x1ul << PWM_CTL2_DTCNT67_Pos)                  /*!< PWM_T::CTL2: DTCNT67 Mask                 */

#define PWM_CTL2_CNTCLR_Pos               (27)                                             /*!< PWM_T::CTL2: CNTCLR Position              */
#define PWM_CTL2_CNTCLR_Msk               (0x1ul << PWM_CTL2_CNTCLR_Pos)                   /*!< PWM_T::CTL2: CNTCLR Mask                  */

#define PWM_CTL2_MODE_Pos                 (28)                                             /*!< PWM_T::CTL2: MODE Position                */
#define PWM_CTL2_MODE_Msk                 (0x3ul << PWM_CTL2_MODE_Pos)                     /*!< PWM_T::CTL2: MODE Mask                    */

#define PWM_CTL2_GROUPEN_Pos              (30)                                             /*!< PWM_T::CTL2: GROUPEN Position             */
#define PWM_CTL2_GROUPEN_Msk              (0x1ul << PWM_CTL2_GROUPEN_Pos)                  /*!< PWM_T::CTL2: GROUPEN Mask                 */

#define PWM_CTL2_CNTTYPE_Pos              (31)                                             /*!< PWM_T::CTL2: CNTTYPE Position             */
#define PWM_CTL2_CNTTYPE_Msk              (0x1ul << PWM_CTL2_CNTTYPE_Pos)                  /*!< PWM_T::CTL2: CNTTYPE Mask                 */

#define PWM_FLAG_ZF0_Pos              	 (0)                                               /*!< PWM_T::INTSTS: ZIF0 Position              */
#define PWM_FLAG_ZF0_Msk              	 (0x1ul << PWM_FLAG_ZF0_Pos)                    /*!< PWM_T::INTSTS: ZIF0 Mask                  */

#define PWM_FLAG_ZF1_Pos              	 (1)                                               /*!< PWM_T::INTSTS: ZIF1 Position              */
#define PWM_FLAG_ZF1_Msk              	 (0x1ul << PWM_FLAG_ZF1_Pos)                    /*!< PWM_T::INTSTS: ZIF1 Mask                  */

#define PWM_FLAG_ZF2_Pos              	 (2)                                               /*!< PWM_T::INTSTS: ZIF2 Position              */
#define PWM_FLAG_ZF2_Msk              	 (0x1ul << PWM_FLAG_ZF2_Pos)                    /*!< PWM_T::INTSTS: ZIF2 Mask                  */

#define PWM_FLAG_ZF3_Pos              	 (3)                                               /*!< PWM_T::INTSTS: ZIF3 Position              */
#define PWM_FLAG_ZF3_Msk              	 (0x1ul << PWM_FLAG_ZF3_Pos)                    /*!< PWM_T::INTSTS: ZIF3 Mask                  */

#define PWM_FLAG_ZF4_Pos              	 (4)                                               /*!< PWM_T::INTSTS: ZIF4 Position              */
#define PWM_FLAG_ZF4_Msk              	 (0x1ul << PWM_FLAG_ZF4_Pos)                    /*!< PWM_T::INTSTS: ZIF4 Mask                  */

#define PWM_FLAG_ZF5_Pos              	 (5)                                               /*!< PWM_T::INTSTS: ZIF5 Position              */
#define PWM_FLAG_ZF5_Msk              	 (0x1ul << PWM_FLAG_ZF5_Pos)                    /*!< PWM_T::INTSTS: ZIF5 Mask                  */

#define PWM_FLAG_ZF6_Pos              	 (6)                                               /*!< PWM_T::INTSTS: ZIF6 Position              */
#define PWM_FLAG_ZF6_Msk              	 (0x1ul << PWM_FLAG_ZF6_Pos)                    /*!< PWM_T::INTSTS: ZIF6 Mask                  */

#define PWM_FLAG_ZF7_Pos              	 (7)                                               /*!< PWM_T::INTSTS: ZIF7 Position              */
#define PWM_FLAG_ZF7_Msk              	 (0x1ul << PWM_FLAG_ZF7_Pos)                    /*!< PWM_T::INTSTS: ZIF7 Mask                  */

#define PWM_FLAG_CMPDF0_Pos           	 (8)                                               /*!< PWM_T::INTSTS: CMPDIF0 Position           */
#define PWM_FLAG_CMPDF0_Msk           	 (0x1ul << PWM_FLAG_CMPDF0_Pos)                 /*!< PWM_T::INTSTS: CMPDIF0 Mask               */

#define PWM_FLAG_CMPDF1_Pos           	 (9)                                               /*!< PWM_T::INTSTS: CMPDIF1 Position           */
#define PWM_FLAG_CMPDF1_Msk           	 (0x1ul << PWM_FLAG_CMPDF1_Pos)                 /*!< PWM_T::INTSTS: CMPDIF1 Mask               */

#define PWM_FLAG_CMPDF2_Pos           	 (10)                                              /*!< PWM_T::INTSTS: CMPDIF2 Position           */
#define PWM_FLAG_CMPDF2_Msk           	 (0x1ul << PWM_FLAG_CMPDF2_Pos)                 /*!< PWM_T::INTSTS: CMPDIF2 Mask               */

#define PWM_FLAG_CMPDF3_Pos           	 (11)                                              /*!< PWM_T::INTSTS: CMPDIF3 Position           */
#define PWM_FLAG_CMPDF3_Msk           	 (0x1ul << PWM_FLAG_CMPDF3_Pos)                 /*!< PWM_T::INTSTS: CMPDIF3 Mask               */

#define PWM_FLAG_CMPDF4_Pos           	 (12)                                              /*!< PWM_T::INTSTS: CMPDIF4 Position           */
#define PWM_FLAG_CMPDF4_Msk           	 (0x1ul << PWM_FLAG_CMPDF4_Pos)                 /*!< PWM_T::INTSTS: CMPDIF4 Mask               */

#define PWM_FLAG_CMPDF5_Pos           	 (13)                                              /*!< PWM_T::INTSTS: CMPDIF5 Position           */
#define PWM_FLAG_CMPDF5_Msk           	 (0x1ul << PWM_FLAG_CMPDF5_Pos)                 /*!< PWM_T::INTSTS: CMPDIF5 Mask               */

#define PWM_FLAG_CMPDF6_Pos           	 (14)                                              /*!< PWM_T::INTSTS: CMPDIF6 Position           */
#define PWM_FLAG_CMPDF6_Msk           	 (0x1ul << PWM_FLAG_CMPDF6_Pos)                 /*!< PWM_T::INTSTS: CMPDIF6 Mask               */

#define PWM_FLAG_CMPDF7_Pos           	 (15)                                              /*!< PWM_T::INTSTS: CMPDIF7 Position           */
#define PWM_FLAG_CMPDF7_Msk           	 (0x1ul << PWM_FLAG_CMPDF7_Pos)                 /*!< PWM_T::INTSTS: CMPDIF7 Mask               */

#define PWM_FLAG_PF0_Pos              	 (16)                                              /*!< PWM_T::INTSTS: PIF0 Position              */
#define PWM_FLAG_PF0_Msk              	 (0x1ul << PWM_FLAG_PF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask                  */

#define PWM_FLAG_PF1_Pos              	 (17)                                              /*!< PWM_T::INTSTS: PIF1 Position              */
#define PWM_FLAG_PF1_Msk              	 (0x1ul << PWM_FLAG_PF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask                  */

#define PWM_FLAG_PF2_Pos              	 (18)                                              /*!< PWM_T::INTSTS: PIF2 Position              */
#define PWM_FLAG_PF2_Msk              	 (0x1ul << PWM_FLAG_PF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask                  */

#define PWM_FLAG_PF3_Pos              	 (19)                                              /*!< PWM_T::INTSTS: PIF3 Position              */
#define PWM_FLAG_PF3_Msk              	 (0x1ul << PWM_FLAG_PF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask                  */

#define PWM_FLAG_PF4_Pos              	 (20)                                              /*!< PWM_T::INTSTS: PIF4 Position              */
#define PWM_FLAG_PF4_Msk              	 (0x1ul << PWM_FLAG_PF4_Pos)                    /*!< PWM_T::INTSTS: PIF4 Mask                  */

#define PWM_FLAG_PF5_Pos              	 (21)                                              /*!< PWM_T::INTSTS: PIF5 Position              */
#define PWM_FLAG_PF5_Msk              	 (0x1ul << PWM_FLAG_PF5_Pos)                    /*!< PWM_T::INTSTS: PIF5 Mask                  */

#define PWM_FLAG_PF6_Pos              	 (22)                                              /*!< PWM_T::INTSTS: PIF6 Position              */
#define PWM_FLAG_PF6_Msk              	 (0x1ul << PWM_FLAG_PF6_Pos)                    /*!< PWM_T::INTSTS: PIF6 Mask                  */

#define PWM_FLAG_PF7_Pos              	 (23)                                              /*!< PWM_T::INTSTS: PIF7 Position              */
#define PWM_FLAG_PF7_Msk              	 (0x1ul << PWM_FLAG_PF7_Pos)                    /*!< PWM_T::INTSTS: PIF7 Mask                  */

#define PWM_FLAG_CMPUF0_Pos           	 (24)                                              /*!< PWM_T::INTSTS: CMPUIF0 Position           */
#define PWM_FLAG_CMPUF0_Msk           	 (0x1ul << PWM_FLAG_CMPUF0_Pos)                 /*!< PWM_T::INTSTS: CMPUIF0 Mask               */

#define PWM_FLAG_CMPUF1_Pos           	 (25)                                              /*!< PWM_T::INTSTS: CMPUIF1 Position           */
#define PWM_FLAG_CMPUF1_Msk           	 (0x1ul << PWM_FLAG_CMPUF1_Pos)                 /*!< PWM_T::INTSTS: CMPUIF1 Mask               */

#define PWM_FLAG_CMPUF2_Pos           	 (26)                                              /*!< PWM_T::INTSTS: CMPUIF2 Position           */
#define PWM_FLAG_CMPUF2_Msk           	 (0x1ul << PWM_FLAG_CMPUF2_Pos)                 /*!< PWM_T::INTSTS: CMPUIF2 Mask               */

#define PWM_FLAG_CMPUF3_Pos           	 (27)                                              /*!< PWM_T::INTSTS: CMPUIF3 Position           */
#define PWM_FLAG_CMPUF3_Msk           	 (0x1ul << PWM_FLAG_CMPUF3_Pos)                 /*!< PWM_T::INTSTS: CMPUIF3 Mask               */

#define PWM_FLAG_CMPUF4_Pos           	 (28)                                              /*!< PWM_T::INTSTS: CMPUIF4 Position           */
#define PWM_FLAG_CMPUF4_Msk           	 (0x1ul << PWM_FLAG_CMPUF4_Pos)                 /*!< PWM_T::INTSTS: CMPUIF4 Mask               */

#define PWM_FLAG_CMPUF5_Pos           	 (29)                                              /*!< PWM_T::INTSTS: CMPUIF5 Position           */
#define PWM_FLAG_CMPUF5_Msk           	 (0x1ul << PWM_FLAG_CMPUF5_Pos)                 /*!< PWM_T::INTSTS: CMPUIF5 Mask               */

#define PWM_FLAG_CMPUF6_Pos           	 (30)                                              /*!< PWM_T::INTSTS: CMPUIF6 Position           */
#define PWM_FLAG_CMPUF6_Msk           	 (0x1ul << PWM_FLAG_CMPUF6_Pos)                 /*!< PWM_T::INTSTS: CMPUIF6 Mask               */

#define PWM_FLAG_CMPUF7_Pos           	 (31)                                              /*!< PWM_T::INTSTS: CMPUIF7 Position           */
#define PWM_FLAG_CMPUF7_Msk           	 (0x1ul << PWM_FLAG_CMPUF7_Pos)                 /*!< PWM_T::INTSTS: CMPUIF7 Mask               */

#define PWM_INTEN_ZIEN0_Pos              (0)                                               /*!< PWM_T::INTEN: ZIEN0 Position              */
#define PWM_INTEN_ZIEN0_Msk              (0x1ul << PWM_INTEN_ZIEN0_Pos)                    /*!< PWM_T::INTEN: ZIEN0 Mask                  */

#define PWM_INTEN_ZIEN1_Pos              (1)                                               /*!< PWM_T::INTEN: ZIEN1 Position              */
#define PWM_INTEN_ZIEN1_Msk              (0x1ul << PWM_INTEN_ZIEN1_Pos)                    /*!< PWM_T::INTEN: ZIEN1 Mask                  */

#define PWM_INTEN_ZIEN2_Pos              (2)                                               /*!< PWM_T::INTEN: ZIEN2 Position              */
#define PWM_INTEN_ZIEN2_Msk              (0x1ul << PWM_INTEN_ZIEN2_Pos)                    /*!< PWM_T::INTEN: ZIEN2 Mask                  */

#define PWM_INTEN_ZIEN3_Pos              (3)                                               /*!< PWM_T::INTEN: ZIEN3 Position              */
#define PWM_INTEN_ZIEN3_Msk              (0x1ul << PWM_INTEN_ZIEN3_Pos)                    /*!< PWM_T::INTEN: ZIEN3 Mask                  */

#define PWM_INTEN_ZIEN4_Pos              (4)                                               /*!< PWM_T::INTEN: ZIEN4 Position              */
#define PWM_INTEN_ZIEN4_Msk              (0x1ul << PWM_INTEN_ZIEN4_Pos)                    /*!< PWM_T::INTEN: ZIEN4 Mask                  */

#define PWM_INTEN_ZIEN5_Pos              (5)                                               /*!< PWM_T::INTEN: ZIEN5 Position              */
#define PWM_INTEN_ZIEN5_Msk              (0x1ul << PWM_INTEN_ZIEN5_Pos)                    /*!< PWM_T::INTEN: ZIEN5 Mask                  */


#define PWM_INTEN_ZIEN6_Pos              (6)                                               /*!< PWM_T::INTEN: ZIEN6 Position              */
#define PWM_INTEN_ZIEN6_Msk              (0x1ul << PWM_INTEN_ZIEN6_Pos)                    /*!< PWM_T::INTEN: ZIEN6 Mask                  */


#define PWM_INTEN_ZIEN7_Pos              (7)                                               /*!< PWM_T::INTEN: ZIEN7 Position              */
#define PWM_INTEN_ZIEN7_Msk              (0x1ul << PWM_INTEN_ZIEN7_Pos)                    /*!< PWM_T::INTEN: ZIEN7 Mask                  */

#define PWM_INTEN_CMPDIEN0_Pos           (8)                                               /*!< PWM_T::INTEN: CMPDIEN0 Position           */
#define PWM_INTEN_CMPDIEN0_Msk           (0x1ul << PWM_INTEN_CMPDIEN0_Pos)                 /*!< PWM_T::INTEN: CMPDIEN0 Mask               */

#define PWM_INTEN_CMPDIEN1_Pos           (9)                                               /*!< PWM_T::INTEN: CMPDIEN1 Position           */
#define PWM_INTEN_CMPDIEN1_Msk           (0x1ul << PWM_INTEN_CMPDIEN1_Pos)                 /*!< PWM_T::INTEN: CMPDIEN1 Mask               */

#define PWM_INTEN_CMPDIEN2_Pos           (10)                                              /*!< PWM_T::INTEN: CMPDIEN2 Position           */
#define PWM_INTEN_CMPDIEN2_Msk           (0x1ul << PWM_INTEN_CMPDIEN2_Pos)                 /*!< PWM_T::INTEN: CMPDIEN2 Mask               */

#define PWM_INTEN_CMPDIEN3_Pos           (11)                                              /*!< PWM_T::INTEN: CMPDIEN3 Position           */
#define PWM_INTEN_CMPDIEN3_Msk           (0x1ul << PWM_INTEN_CMPDIEN3_Pos)                 /*!< PWM_T::INTEN: CMPDIEN3 Mask               */

#define PWM_INTEN_CMPDIEN4_Pos           (12)                                              /*!< PWM_T::INTEN: CMPDIEN4 Position           */
#define PWM_INTEN_CMPDIEN4_Msk           (0x1ul << PWM_INTEN_CMPDIEN4_Pos)                 /*!< PWM_T::INTEN: CMPDIEN4 Mask               */

#define PWM_INTEN_CMPDIEN5_Pos           (13)                                              /*!< PWM_T::INTEN: CMPDIEN5 Position           */
#define PWM_INTEN_CMPDIEN5_Msk           (0x1ul << PWM_INTEN_CMPDIEN5_Pos)                 /*!< PWM_T::INTEN: CMPDIEN5 Mask               */


#define PWM_INTEN_CMPDIEN6_Pos           (14)                                              /*!< PWM_T::INTEN: CMPDIEN7 Position           */
#define PWM_INTEN_CMPDIEN6_Msk           (0x1ul << PWM_INTEN_CMPDIEN6_Pos)                 /*!< PWM_T::INTEN: CMPDIEN7 Mask               */

#define PWM_INTEN_CMPDIEN7_Pos           (15)                                              /*!< PWM_T::INTEN: CMPDIEN7 Position           */
#define PWM_INTEN_CMPDIEN7_Msk           (0x1ul << PWM_INTEN_CMPDIEN7_Pos)                 /*!< PWM_T::INTEN: CMPDIEN7 Mask               */

#define PWM_INTEN_PIEN0_Pos              (16)                                              /*!< PWM_T::INTEN: PIEN0 Position              */
#define PWM_INTEN_PIEN0_Msk              (0x1ul << PWM_INTEN_PIEN0_Pos)                    /*!< PWM_T::INTEN: PIEN0 Mask                  */

#define PWM_INTEN_PIEN1_Pos              (17)                                              /*!< PWM_T::INTEN: PIEN1 Position              */
#define PWM_INTEN_PIEN1_Msk              (0x1ul << PWM_INTEN_PIEN1_Pos)                    /*!< PWM_T::INTEN: PIEN1 Mask                  */

#define PWM_INTEN_PIEN2_Pos              (18)                                              /*!< PWM_T::INTEN: PIEN2 Position              */
#define PWM_INTEN_PIEN2_Msk              (0x1ul << PWM_INTEN_PIEN2_Pos)                    /*!< PWM_T::INTEN: PIEN2 Mask                  */

#define PWM_INTEN_PIEN3_Pos              (19)                                              /*!< PWM_T::INTEN: PIEN3 Position              */
#define PWM_INTEN_PIEN3_Msk              (0x1ul << PWM_INTEN_PIEN3_Pos)                    /*!< PWM_T::INTEN: PIEN3 Mask                  */

#define PWM_INTEN_PIEN4_Pos              (20)                                              /*!< PWM_T::INTEN: PIEN4 Position              */
#define PWM_INTEN_PIEN4_Msk              (0x1ul << PWM_INTEN_PIEN4_Pos)                    /*!< PWM_T::INTEN: PIEN4 Mask                  */

#define PWM_INTEN_PIEN5_Pos              (21)                                              /*!< PWM_T::INTEN: PIEN5 Position              */
#define PWM_INTEN_PIEN5_Msk              (0x1ul << PWM_INTEN_PIEN5_Pos)                    /*!< PWM_T::INTEN: PIEN5 Mask                  */


#define PWM_INTEN_PIEN6_Pos              (22)                                              /*!< PWM_T::INTEN: PIEN6 Position              */
#define PWM_INTEN_PIEN6_Msk              (0x1ul << PWM_INTEN_PIEN6_Pos)                    /*!< PWM_T::INTEN: PIEN6 Mask                  */


#define PWM_INTEN_PIEN7_Pos              (23)                                              /*!< PWM_T::INTEN: PIEN7 Position              */
#define PWM_INTEN_PIEN7_Msk              (0x1ul << PWM_INTEN_PIEN7_Pos)                    /*!< PWM_T::INTEN: PIEN7 Mask                  */

#define PWM_INTEN_CMPUIEN0_Pos           (24)                                              /*!< PWM_T::INTEN: CMPUIEN0 Position           */
#define PWM_INTEN_CMPUIEN0_Msk           (0x1ul << PWM_INTEN_CMPUIEN0_Pos)                 /*!< PWM_T::INTEN: CMPUIEN0 Mask               */

#define PWM_INTEN_CMPUIEN1_Pos           (25)                                              /*!< PWM_T::INTEN: CMPUIEN1 Position           */
#define PWM_INTEN_CMPUIEN1_Msk           (0x1ul << PWM_INTEN_CMPUIEN1_Pos)                 /*!< PWM_T::INTEN: CMPUIEN1 Mask               */

#define PWM_INTEN_CMPUIEN2_Pos           (26)                                              /*!< PWM_T::INTEN: CMPUIEN2 Position           */
#define PWM_INTEN_CMPUIEN2_Msk           (0x1ul << PWM_INTEN_CMPUIEN2_Pos)                 /*!< PWM_T::INTEN: CMPUIEN2 Mask               */

#define PWM_INTEN_CMPUIEN3_Pos           (27)                                              /*!< PWM_T::INTEN: CMPUIEN3 Position           */
#define PWM_INTEN_CMPUIEN3_Msk           (0x1ul << PWM_INTEN_CMPUIEN3_Pos)                 /*!< PWM_T::INTEN: CMPUIEN3 Mask               */

#define PWM_INTEN_CMPUIEN4_Pos           (28)                                              /*!< PWM_T::INTEN: CMPUIEN4 Position           */
#define PWM_INTEN_CMPUIEN4_Msk           (0x1ul << PWM_INTEN_CMPUIEN4_Pos)                 /*!< PWM_T::INTEN: CMPUIEN4 Mask               */

#define PWM_INTEN_CMPUIEN5_Pos           (29)                                              /*!< PWM_T::INTEN: CMPUIEN5 Position           */
#define PWM_INTEN_CMPUIEN5_Msk           (0x1ul << PWM_INTEN_CMPUIEN5_Pos)                 /*!< PWM_T::INTEN: CMPUIEN5 Mask               */

#define PWM_INTEN_CMPUIEN6_Pos           (30)                                              /*!< PWM_T::INTEN: CMPUIEN6 Position           */
#define PWM_INTEN_CMPUIEN6_Msk           (0x1ul << PWM_INTEN_CMPUIEN6_Pos)                 /*!< PWM_T::INTEN: CMPUIEN6 Mask               */

#define PWM_INTEN_CMPUIEN7_Pos           (31)                                              /*!< PWM_T::INTEN: CMPUIEN7 Position           */
#define PWM_INTEN_CMPUIEN7_Msk           (0x1ul << PWM_INTEN_CMPUIEN7_Pos)                 /*!< PWM_T::INTEN: CMPUIEN7 Mask               */

#define PWM_INTSTS_ZIF0_Pos              (0)                                               /*!< PWM_T::INTSTS: ZIF0 Position              */
#define PWM_INTSTS_ZIF0_Msk              (0x1ul << PWM_INTSTS_ZIF0_Pos)                    /*!< PWM_T::INTSTS: ZIF0 Mask                  */

#define PWM_INTSTS_ZIF1_Pos              (1)                                               /*!< PWM_T::INTSTS: ZIF1 Position              */
#define PWM_INTSTS_ZIF1_Msk              (0x1ul << PWM_INTSTS_ZIF1_Pos)                    /*!< PWM_T::INTSTS: ZIF1 Mask                  */

#define PWM_INTSTS_ZIF2_Pos              (2)                                               /*!< PWM_T::INTSTS: ZIF2 Position              */
#define PWM_INTSTS_ZIF2_Msk              (0x1ul << PWM_INTSTS_ZIF2_Pos)                    /*!< PWM_T::INTSTS: ZIF2 Mask                  */

#define PWM_INTSTS_ZIF3_Pos              (3)                                               /*!< PWM_T::INTSTS: ZIF3 Position              */
#define PWM_INTSTS_ZIF3_Msk              (0x1ul << PWM_INTSTS_ZIF3_Pos)                    /*!< PWM_T::INTSTS: ZIF3 Mask                  */

#define PWM_INTSTS_ZIF4_Pos              (4)                                               /*!< PWM_T::INTSTS: ZIF4 Position              */
#define PWM_INTSTS_ZIF4_Msk              (0x1ul << PWM_INTSTS_ZIF4_Pos)                    /*!< PWM_T::INTSTS: ZIF4 Mask                  */

#define PWM_INTSTS_ZIF5_Pos              (5)                                               /*!< PWM_T::INTSTS: ZIF5 Position              */
#define PWM_INTSTS_ZIF5_Msk              (0x1ul << PWM_INTSTS_ZIF5_Pos)                    /*!< PWM_T::INTSTS: ZIF5 Mask                  */

#define PWM_INTSTS_ZIF6_Pos              (6)                                               /*!< PWM_T::INTSTS: ZIF6 Position              */
#define PWM_INTSTS_ZIF6_Msk              (0x1ul << PWM_INTSTS_ZIF6_Pos)                    /*!< PWM_T::INTSTS: ZIF6 Mask                  */

#define PWM_INTSTS_ZIF7_Pos              (7)                                               /*!< PWM_T::INTSTS: ZIF7 Position              */
#define PWM_INTSTS_ZIF7_Msk              (0x1ul << PWM_INTSTS_ZIF7_Pos)                    /*!< PWM_T::INTSTS: ZIF7 Mask                  */

#define PWM_INTSTS_CMPDIF0_Pos           (8)                                               /*!< PWM_T::INTSTS: CMPDIF0 Position           */
#define PWM_INTSTS_CMPDIF0_Msk           (0x1ul << PWM_INTSTS_CMPDIF0_Pos)                 /*!< PWM_T::INTSTS: CMPDIF0 Mask               */

#define PWM_INTSTS_CMPDIF1_Pos           (9)                                               /*!< PWM_T::INTSTS: CMPDIF1 Position           */
#define PWM_INTSTS_CMPDIF1_Msk           (0x1ul << PWM_INTSTS_CMPDIF1_Pos)                 /*!< PWM_T::INTSTS: CMPDIF1 Mask               */

#define PWM_INTSTS_CMPDIF2_Pos           (10)                                              /*!< PWM_T::INTSTS: CMPDIF2 Position           */
#define PWM_INTSTS_CMPDIF2_Msk           (0x1ul << PWM_INTSTS_CMPDIF2_Pos)                 /*!< PWM_T::INTSTS: CMPDIF2 Mask               */

#define PWM_INTSTS_CMPDIF3_Pos           (11)                                              /*!< PWM_T::INTSTS: CMPDIF3 Position           */
#define PWM_INTSTS_CMPDIF3_Msk           (0x1ul << PWM_INTSTS_CMPDIF3_Pos)                 /*!< PWM_T::INTSTS: CMPDIF3 Mask               */

#define PWM_INTSTS_CMPDIF4_Pos           (12)                                              /*!< PWM_T::INTSTS: CMPDIF4 Position           */
#define PWM_INTSTS_CMPDIF4_Msk           (0x1ul << PWM_INTSTS_CMPDIF4_Pos)                 /*!< PWM_T::INTSTS: CMPDIF4 Mask               */

#define PWM_INTSTS_CMPDIF5_Pos           (13)                                              /*!< PWM_T::INTSTS: CMPDIF5 Position           */
#define PWM_INTSTS_CMPDIF5_Msk           (0x1ul << PWM_INTSTS_CMPDIF5_Pos)                 /*!< PWM_T::INTSTS: CMPDIF5 Mask               */

#define PWM_INTSTS_CMPDIF6_Pos           (14)                                              /*!< PWM_T::INTSTS: CMPDIF6 Position           */
#define PWM_INTSTS_CMPDIF6_Msk           (0x1ul << PWM_INTSTS_CMPDIF6_Pos)                 /*!< PWM_T::INTSTS: CMPDIF6 Mask               */

#define PWM_INTSTS_CMPDIF7_Pos           (15)                                              /*!< PWM_T::INTSTS: CMPDIF7 Position           */
#define PWM_INTSTS_CMPDIF7_Msk           (0x1ul << PWM_INTSTS_CMPDIF7_Pos)                 /*!< PWM_T::INTSTS: CMPDIF7 Mask               */

#define PWM_INTSTS_PIF0_Pos              (16)                                              /*!< PWM_T::INTSTS: PIF0 Position              */
#define PWM_INTSTS_PIF0_Msk              (0x1ul << PWM_INTSTS_PIF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask                  */

#define PWM_INTSTS_PIF1_Pos              (17)                                              /*!< PWM_T::INTSTS: PIF1 Position              */
#define PWM_INTSTS_PIF1_Msk              (0x1ul << PWM_INTSTS_PIF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask                  */

#define PWM_INTSTS_PIF2_Pos              (18)                                              /*!< PWM_T::INTSTS: PIF2 Position              */
#define PWM_INTSTS_PIF2_Msk              (0x1ul << PWM_INTSTS_PIF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask                  */

#define PWM_INTSTS_PIF3_Pos              (19)                                              /*!< PWM_T::INTSTS: PIF3 Position              */
#define PWM_INTSTS_PIF3_Msk              (0x1ul << PWM_INTSTS_PIF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask                  */

#define PWM_INTSTS_PIF4_Pos              (20)                                              /*!< PWM_T::INTSTS: PIF4 Position              */
#define PWM_INTSTS_PIF4_Msk              (0x1ul << PWM_INTSTS_PIF4_Pos)                    /*!< PWM_T::INTSTS: PIF4 Mask                  */

#define PWM_INTSTS_PIF5_Pos              (21)                                              /*!< PWM_T::INTSTS: PIF5 Position              */
#define PWM_INTSTS_PIF5_Msk              (0x1ul << PWM_INTSTS_PIF5_Pos)                    /*!< PWM_T::INTSTS: PIF5 Mask                  */

#define PWM_INTSTS_PIF6_Pos              (22)                                              /*!< PWM_T::INTSTS: PIF6 Position              */
#define PWM_INTSTS_PIF6_Msk              (0x1ul << PWM_INTSTS_PIF6_Pos)                    /*!< PWM_T::INTSTS: PIF6 Mask                  */

#define PWM_INTSTS_PIF7_Pos              (23)                                              /*!< PWM_T::INTSTS: PIF7 Position              */
#define PWM_INTSTS_PIF7_Msk              (0x1ul << PWM_INTSTS_PIF7_Pos)                    /*!< PWM_T::INTSTS: PIF7 Mask                  */

#define PWM_INTSTS_CMPUIF0_Pos           (24)                                              /*!< PWM_T::INTSTS: CMPUIF0 Position           */
#define PWM_INTSTS_CMPUIF0_Msk           (0x1ul << PWM_INTSTS_CMPUIF0_Pos)                 /*!< PWM_T::INTSTS: CMPUIF0 Mask               */

#define PWM_INTSTS_CMPUIF1_Pos           (25)                                              /*!< PWM_T::INTSTS: CMPUIF1 Position           */
#define PWM_INTSTS_CMPUIF1_Msk           (0x1ul << PWM_INTSTS_CMPUIF1_Pos)                 /*!< PWM_T::INTSTS: CMPUIF1 Mask               */

#define PWM_INTSTS_CMPUIF2_Pos           (26)                                              /*!< PWM_T::INTSTS: CMPUIF2 Position           */
#define PWM_INTSTS_CMPUIF2_Msk           (0x1ul << PWM_INTSTS_CMPUIF2_Pos)                 /*!< PWM_T::INTSTS: CMPUIF2 Mask               */

#define PWM_INTSTS_CMPUIF3_Pos           (27)                                              /*!< PWM_T::INTSTS: CMPUIF3 Position           */
#define PWM_INTSTS_CMPUIF3_Msk           (0x1ul << PWM_INTSTS_CMPUIF3_Pos)                 /*!< PWM_T::INTSTS: CMPUIF3 Mask               */

#define PWM_INTSTS_CMPUIF4_Pos           (28)                                              /*!< PWM_T::INTSTS: CMPUIF4 Position           */
#define PWM_INTSTS_CMPUIF4_Msk           (0x1ul << PWM_INTSTS_CMPUIF4_Pos)                 /*!< PWM_T::INTSTS: CMPUIF4 Mask               */

#define PWM_INTSTS_CMPUIF5_Pos           (29)                                              /*!< PWM_T::INTSTS: CMPUIF5 Position           */
#define PWM_INTSTS_CMPUIF5_Msk           (0x1ul << PWM_INTSTS_CMPUIF5_Pos)                 /*!< PWM_T::INTSTS: CMPUIF5 Mask               */

#define PWM_INTSTS_CMPUIF6_Pos           (30)                                              /*!< PWM_T::INTSTS: CMPUIF6 Position           */
#define PWM_INTSTS_CMPUIF6_Msk           (0x1ul << PWM_INTSTS_CMPUIF6_Pos)                 /*!< PWM_T::INTSTS: CMPUIF6 Mask               */

#define PWM_INTSTS_CMPUIF7_Pos           (31)                                              /*!< PWM_T::INTSTS: CMPUIF7 Position           */
#define PWM_INTSTS_CMPUIF7_Msk           (0x1ul << PWM_INTSTS_CMPUIF7_Pos)                 /*!< PWM_T::INTSTS: CMPUIF7 Mask               */

#define PWM_POEN_POEN0_Pos               (0)                                               /*!< PWM_T::POEN: POEN0 Position               */
#define PWM_POEN_POEN0_Msk               (0x1ul << PWM_POEN_POEN0_Pos)                     /*!< PWM_T::POEN: POEN0 Mask                   */

#define PWM_POEN_POEN1_Pos               (1)                                               /*!< PWM_T::POEN: POEN1 Position               */
#define PWM_POEN_POEN1_Msk               (0x1ul << PWM_POEN_POEN1_Pos)                     /*!< PWM_T::POEN: POEN1 Mask                   */

#define PWM_POEN_POEN2_Pos               (2)                                               /*!< PWM_T::POEN: POEN2 Position               */
#define PWM_POEN_POEN2_Msk               (0x1ul << PWM_POEN_POEN2_Pos)                     /*!< PWM_T::POEN: POEN2 Mask                   */

#define PWM_POEN_POEN3_Pos               (3)                                               /*!< PWM_T::POEN: POEN3 Position               */
#define PWM_POEN_POEN3_Msk               (0x1ul << PWM_POEN_POEN3_Pos)                     /*!< PWM_T::POEN: POEN3 Mask                   */

#define PWM_POEN_POEN4_Pos               (4)                                               /*!< PWM_T::POEN: POEN4 Position               */
#define PWM_POEN_POEN4_Msk               (0x1ul << PWM_POEN_POEN4_Pos)                     /*!< PWM_T::POEN: POEN4 Mask                   */

#define PWM_POEN_POEN5_Pos               (5)                                               /*!< PWM_T::POEN: POEN5 Position               */
#define PWM_POEN_POEN5_Msk               (0x1ul << PWM_POEN_POEN5_Pos)                     /*!< PWM_T::POEN: POEN5 Mask                   */

#define PWM_POEN_POEN6_Pos               (6)                                               /*!< PWM_T::POEN: POEN6 Position               */
#define PWM_POEN_POEN6_Msk               (0x1ul << PWM_POEN_POEN4_Pos)                     /*!< PWM_T::POEN: POEN6 Mask                   */

#define PWM_POEN_POEN7_Pos               (7)                                               /*!< PWM_T::POEN: POEN7 Position               */
#define PWM_POEN_POEN7_Msk               (0x1ul << PWM_POEN_POEN7_Pos)                     /*!< PWM_T::POEN: POEN7 Mask                   */

#define PWM_BRKCTL_BRK0EN_Pos            (0)                                               /*!< PWM_T::BRKCTL: BRK0EN Position            */
#define PWM_BRKCTL_BRK0EN_Msk            (0x1ul << PWM_BRKCTL_BRK0EN_Pos)                  /*!< PWM_T::BRKCTL: BRK0EN Mask                */

#define PWM_BRKCTL_BRK1EN_Pos            (1)                                               /*!< PWM_T::BRKCTL: BRK1EN Position            */
#define PWM_BRKCTL_BRK1EN_Msk            (0x1ul << PWM_BRKCTL_BRK1EN_Pos)                  /*!< PWM_T::BRKCTL: BRK1EN Mask                */

#define PWM_BRKCTL_BRK0SEL_Pos           (2)                                               /*!< PWM_T::BRKCTL: BRK0SEL Position           */
#define PWM_BRKCTL_BRK0SEL_Msk           (0x1ul << PWM_BRKCTL_BRK0SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK0SEL Mask               */

#define PWM_BRKCTL_BRK1SEL_Pos           (3)                                               /*!< PWM_T::BRKCTL: BRK1SEL Position           */
#define PWM_BRKCTL_BRK1SEL_Msk           (0x1ul << PWM_BRKCTL_BRK1SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK1SEL Mask               */

#define PWM_BRKCTL_BRKSTS_Pos            (7)                                               /*!< PWM_T::BRKCTL: BRKSTS Position            */
#define PWM_BRKCTL_BRKSTS_Msk            (0x1ul << PWM_BRKCTL_BRKSTS_Pos)                  /*!< PWM_T::BRKCTL: BRKSTS Mask                */

#define PWM_BRKCTL_BRKACT_Pos            (8)                                               /*!< PWM_T::BRKCTL: BRKACT Position            */
#define PWM_BRKCTL_BRKACT_Msk            (0x1ul << PWM_BRKCTL_BRKACT_Pos)                  /*!< PWM_T::BRKCTL: BRKACT Mask                */

#define PWM_BRKCTL_SWBRK_Pos             (9)                                               /*!< PWM_T::BRKCTL: SWBRK Position             */
#define PWM_BRKCTL_SWBRK_Msk             (0x1ul << PWM_BRKCTL_SWBRK_Pos)                   /*!< PWM_T::BRKCTL: SWBRK Mask                 */

#define PWM_BRKCTL_BRKIF0_Pos            (16)                                              /*!< PWM_T::BRKCTL: BRKIF0 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKIF0_Msk            (0x1ul << PWM_BRKCTL_BRKIF0_Pos)                  /*!< PWM_T::BRKSTS: BRKIF0 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKIF1_Pos            (17)                                              /*!< PWM_T::BRKSTS: BRKIF1 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKIF1_Msk            (0x1ul << PWM_BRKCTL_BRKIF1_Pos)                  /*!< PWM_T::BRKSTS: BRKIF1 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKF0_Pos             (18)                                              /*!< PWM_T::BRKCTL: BRKIF0 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKF0_Msk             (0x1ul << PWM_BRKCTL_BRKF0_Pos)                  /*!< PWM_T::BRKSTS: BRKIF0 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKF1_Pos             (19)                                              /*!< PWM_T::BRKSTS: BRKIF1 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKF1_Msk             (0x1ul << PWM_BRKCTL_BRKF1_Pos)                  /*!< PWM_T::BRKSTS: BRKIF1 Mask 								*/

#define PWM_BRKCTL_BKOD0_Pos             (24)                                              /*!< PWM_T::BRKCTL: BKOD0 Position             */
#define PWM_BRKCTL_BKOD0_Msk             (0x1ul << PWM_BRKCTL_BKOD0_Pos)                   /*!< PWM_T::BRKCTL: BKOD0 Mask                 */

#define PWM_BRKCTL_BKOD1_Pos             (25)                                              /*!< PWM_T::BRKCTL: BKOD1 Position             */
#define PWM_BRKCTL_BKOD1_Msk             (0x1ul << PWM_BRKCTL_BKOD1_Pos)                   /*!< PWM_T::BRKCTL: BKOD1 Mask                 */

#define PWM_BRKCTL_BKOD2_Pos             (26)                                              /*!< PWM_T::BRKCTL: BKOD2 Position             */
#define PWM_BRKCTL_BKOD2_Msk             (0x1ul << PWM_BRKCTL_BKOD2_Pos)                   /*!< PWM_T::BRKCTL: BKOD2 Mask                 */

#define PWM_BRKCTL_BKOD3_Pos             (27)                                              /*!< PWM_T::BRKCTL: BKOD3 Position             */
#define PWM_BRKCTL_BKOD3_Msk             (0x1ul << PWM_BRKCTL_BKOD3_Pos)                   /*!< PWM_T::BRKCTL: BKOD3 Mask                 */

#define PWM_BRKCTL_BKOD4_Pos             (28)                                              /*!< PWM_T::BRKCTL: BKOD4 Position             */
#define PWM_BRKCTL_BKOD4_Msk             (0x1ul << PWM_BRKCTL_BKOD4_Pos)                   /*!< PWM_T::BRKCTL: BKOD4 Mask                 */

#define PWM_BRKCTL_BKOD5_Pos             (29)                                              /*!< PWM_T::BRKCTL: BKOD5 Position             */
#define PWM_BRKCTL_BKOD5_Msk             (0x1ul << PWM_BRKCTL_BKOD5_Pos)                   /*!< PWM_T::BRKCTL: BKOD5 Mask                 */

#define PWM_BRKCTL_D6BKOD_Pos            (30)                                              /*!< PWM_T::BRKCTL: D6BKOD Position            */
#define PWM_BRKCTL_D6BKOD_Msk            (0x1ul << PWM_BRKCTL_D6BKOD_Pos)                  /*!< PWM_T::BRKCTL: D6BKOD Mask                */

#define PWM_BRKCTL_D7BKOD_Pos            (31)                                              /*!< PWM_T::BRKCTL: D7BKOD Position            */
#define PWM_BRKCTL_D7BKOD_Msk            (0x1ul << PWM_BRKCTL_D7BKOD_Pos)                  /*!< PWM_T::BRKCTL: D7BKOD Mask                */

#define PWM_DTCTL_DTI01_Pos              (0)                                               /*!< PWM_T::DTCTL: DTI01 Position              */
#define PWM_DTCTL_DTI01_Msk              (0xfful << PWM_DTCTL_DTI01_Pos)                   /*!< PWM_T::DTCTL: DTI01 Mask                  */

#define PWM_DTCTL_DTI23_Pos              (8)                                               /*!< PWM_T::DTCTL: DTI23 Position              */
#define PWM_DTCTL_DTI23_Msk              (0xfful << PWM_DTCTL_DTI23_Pos)                   /*!< PWM_T::DTCTL: DTI23 Mask                  */

#define PWM_DTCTL_DTI45_Pos              (16)                                              /*!< PWM_T::DTCTL: DTI45 Position              */
#define PWM_DTCTL_DTI45_Msk              (0xfful << PWM_DTCTL_DTI45_Pos)                   /*!< PWM_T::DTCTL: DTI45 Mask                  */

#define PWM_DTCTL_DTI67_Pos              (24)                                              /*!< PWM_T::DTCTL: DTI67 Position              */
#define PWM_DTCTL_DTI67_Msk              (0xfful << PWM_DTCTL_DTI67_Pos)                   /*!< PWM_T::DTCTL: DTI67 Mask                  */

#define PWM_ADCTCTL0_CUTRGEN0_Pos        (0)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN0 Position        */
#define PWM_ADCTCTL0_CUTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN0 Mask            */

#define PWM_ADCTCTL0_CPTRGEN0_Pos        (1)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN0 Position        */
#define PWM_ADCTCTL0_CPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CDTRGEN0_Pos        (2)                                               /*!< PWM_T::ADCTCTL0: CDTRGEN0 Position        */
#define PWM_ADCTCTL0_CDTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN0 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN0_Pos        (3)                                               /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Position        */
#define PWM_ADCTCTL0_ZPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CUTRGEN1_Pos        (8)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN1 Position        */
#define PWM_ADCTCTL0_CUTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN1 Mask            */

#define PWM_ADCTCTL0_CPTRGEN1_Pos        (9)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN1 Position        */
#define PWM_ADCTCTL0_CPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CDTRGEN1_Pos        (10)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Position        */
#define PWM_ADCTCTL0_CDTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN1_Pos        (11)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Position        */
#define PWM_ADCTCTL0_ZPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CUTRGEN2_Pos        (16)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Position        */
#define PWM_ADCTCTL0_CUTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Mask            */

#define PWM_ADCTCTL0_CPTRGEN2_Pos        (17)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Position        */
#define PWM_ADCTCTL0_CPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CDTRGEN2_Pos        (18)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Position        */
#define PWM_ADCTCTL0_CDTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN2_Pos        (19)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Position        */
#define PWM_ADCTCTL0_ZPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CUTRGEN3_Pos        (24)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Position        */
#define PWM_ADCTCTL0_CUTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Mask            */

#define PWM_ADCTCTL0_CPTRGEN3_Pos        (25)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Position        */
#define PWM_ADCTCTL0_CPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Mask            */

#define PWM_ADCTCTL0_CDTRGEN3_Pos        (26)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Position        */
#define PWM_ADCTCTL0_CDTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN3_Pos        (27)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Position        */
#define PWM_ADCTCTL0_ZPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Mask            */

#define PWM_ADCTCTL1_CUTRGEN4_Pos        (0)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN4 Position        */
#define PWM_ADCTCTL1_CUTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN4 Mask            */

#define PWM_ADCTCTL1_CPTRGEN4_Pos        (1)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN4 Position        */
#define PWM_ADCTCTL1_CPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CDTRGEN4_Pos        (2)                                               /*!< PWM_T::ADCTCTL1: CDTRGEN4 Position        */
#define PWM_ADCTCTL1_CDTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN4 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN4_Pos        (3)                                               /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Position        */
#define PWM_ADCTCTL1_ZPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CUTRGEN5_Pos        (8)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN5 Position        */
#define PWM_ADCTCTL1_CUTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN5 Mask            */

#define PWM_ADCTCTL1_CPTRGEN5_Pos        (9)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN5 Position        */
#define PWM_ADCTCTL1_CPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN5 Mask            */

#define PWM_ADCTCTL1_CDTRGEN5_Pos        (10)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Position        */
#define PWM_ADCTCTL1_CDTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN5_Pos        (11)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Position        */
#define PWM_ADCTCTL1_ZPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Mask            */

#define PWM_ADCTCTL1_CUTRGEN6_Pos        (16)                                              /*!< PWM_T::ADCTCTL1: CUTRGEN6 Position        */
#define PWM_ADCTCTL1_CUTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN6 Mask            */

#define PWM_ADCTCTL1_CPTRGEN6_Pos        (17)                                              /*!< PWM_T::ADCTCTL1: CPTRGEN6 Position        */
#define PWM_ADCTCTL1_CPTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN6 Mask            */

#define PWM_ADCTCTL1_CDTRGEN6_Pos        (18)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN6 Position        */
#define PWM_ADCTCTL1_CDTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN6 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN6_Pos        (19)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN6 Position        */
#define PWM_ADCTCTL1_ZPTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN6 Mask            */

#define PWM_ADCTCTL1_CUTRGEN7_Pos        (24)                                              /*!< PWM_T::ADCTCTL1: CUTRGEN7 Position        */
#define PWM_ADCTCTL1_CUTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN7 Mask            */

#define PWM_ADCTCTL1_CPTRGEN7_Pos        (25)                                              /*!< PWM_T::ADCTCTL1: CPTRGEN7 Position        */
#define PWM_ADCTCTL1_CPTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN7 Mask            */

#define PWM_ADCTCTL1_CDTRGEN7_Pos        (26)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN7 Position        */
#define PWM_ADCTCTL1_CDTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN7 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN7_Pos        (27)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN7 Position        */
#define PWM_ADCTCTL1_ZPTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN7 Mask            */

#define PWM_ADCTSTS0_CUTRGF0_Pos         (0)                                               /*!< PWM_T::ADCTSTS0: CUTRGF0 Position         */
#define PWM_ADCTSTS0_CUTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF0 Mask             */

#define PWM_ADCTSTS0_CPTRGF0_Pos         (1)                                               /*!< PWM_T::ADCTSTS0: CPTRGF0 Position         */
#define PWM_ADCTSTS0_CPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF0 Mask             */

#define PWM_ADCTSTS0_CDTRGF0_Pos         (2)                                               /*!< PWM_T::ADCTSTS0: CDTRGF0 Position         */
#define PWM_ADCTSTS0_CDTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF0 Mask             */

#define PWM_ADCTSTS0_ZPTRGF0_Pos         (3)                                               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Position         */
#define PWM_ADCTSTS0_ZPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Mask             */

#define PWM_ADCTSTS0_CUTRGF1_Pos         (8)                                               /*!< PWM_T::ADCTSTS0: CUTRGF1 Position         */
#define PWM_ADCTSTS0_CUTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF1 Mask             */

#define PWM_ADCTSTS0_CPTRGF1_Pos         (9)                                               /*!< PWM_T::ADCTSTS0: CPTRGF1 Position         */
#define PWM_ADCTSTS0_CPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF1 Mask             */

#define PWM_ADCTSTS0_CDTRGF1_Pos         (10)                                              /*!< PWM_T::ADCTSTS0: CDTRGF1 Position         */
#define PWM_ADCTSTS0_CDTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF1 Mask             */

#define PWM_ADCTSTS0_ZPTRGF1_Pos         (11)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF1 Position         */
#define PWM_ADCTSTS0_ZPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF1 Mask             */

#define PWM_ADCTSTS0_CUTRGF2_Pos         (16)                                              /*!< PWM_T::ADCTSTS0: CUTRGF2 Position         */
#define PWM_ADCTSTS0_CUTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF2 Mask             */

#define PWM_ADCTSTS0_CPTRGF2_Pos         (17)                                              /*!< PWM_T::ADCTSTS0: CPTRGF2 Position         */
#define PWM_ADCTSTS0_CPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF2 Mask             */

#define PWM_ADCTSTS0_CDTRGF2_Pos         (18)                                              /*!< PWM_T::ADCTSTS0: CDTRGF2 Position         */
#define PWM_ADCTSTS0_CDTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF2 Mask             */

#define PWM_ADCTSTS0_ZPTRGF2_Pos         (19)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF2 Position         */
#define PWM_ADCTSTS0_ZPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF2 Mask             */

#define PWM_ADCTSTS0_CUTRGF3_Pos         (24)                                              /*!< PWM_T::ADCTSTS0: CUTRGF3 Position         */
#define PWM_ADCTSTS0_CUTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF3 Mask             */

#define PWM_ADCTSTS0_CPTRGF3_Pos         (25)                                              /*!< PWM_T::ADCTSTS0: CPTRGF3 Position         */
#define PWM_ADCTSTS0_CPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF3 Mask             */

#define PWM_ADCTSTS0_CDTRGF3_Pos         (26)                                              /*!< PWM_T::ADCTSTS0: CDTRGF3 Position         */
#define PWM_ADCTSTS0_CDTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF3 Mask             */

#define PWM_ADCTSTS0_ZPTRGF3_Pos         (27)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF3 Position         */
#define PWM_ADCTSTS0_ZPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF3 Mask             */

#define PWM_ADCTSTS1_CUTRGF4_Pos         (0)                                               /*!< PWM_T::ADCTSTS1: CUTRGF4 Position         */
#define PWM_ADCTSTS1_CUTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF4 Mask             */

#define PWM_ADCTSTS1_CPTRGF4_Pos         (1)                                               /*!< PWM_T::ADCTSTS1: CPTRGF4 Position         */
#define PWM_ADCTSTS1_CPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF4 Mask             */

#define PWM_ADCTSTS1_CDTRGF4_Pos         (2)                                               /*!< PWM_T::ADCTSTS1: CDTRGF4 Position         */
#define PWM_ADCTSTS1_CDTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF4 Mask             */

#define PWM_ADCTSTS1_ZPTRGF4_Pos         (3)                                               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Position         */
#define PWM_ADCTSTS1_ZPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Mask             */

#define PWM_ADCTSTS1_CUTRGF5_Pos         (8)                                               /*!< PWM_T::ADCTSTS1: CUTRGF5 Position         */
#define PWM_ADCTSTS1_CUTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF5 Mask             */

#define PWM_ADCTSTS1_CPTRGF5_Pos         (9)                                               /*!< PWM_T::ADCTSTS1: CPTRGF5 Position         */
#define PWM_ADCTSTS1_CPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF5 Mask             */

#define PWM_ADCTSTS1_CDTRGF5_Pos         (10)                                              /*!< PWM_T::ADCTSTS1: CDTRGF5 Position         */
#define PWM_ADCTSTS1_CDTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF5 Mask             */

#define PWM_ADCTSTS1_ZPTRGF5_Pos         (11)                                              /*!< PWM_T::ADCTSTS1: ZPTRGF5 Position         */
#define PWM_ADCTSTS1_ZPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF5 Mask             */


#define PWM_ADCTSTS1_CUTRGF6_Pos         (16)                                              /*!< PWM_T::ADCTSTS0: CUTRGF6 Position         */
#define PWM_ADCTSTS1_CUTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CUTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF6 Mask             */

#define PWM_ADCTSTS1_CPTRGF6_Pos         (17)                                              /*!< PWM_T::ADCTSTS0: CPTRGF6 Position         */
#define PWM_ADCTSTS1_CPTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CPTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF6 Mask             */

#define PWM_ADCTSTS1_CDTRGF6_Pos         (18)                                              /*!< PWM_T::ADCTSTS0: CDTRGF6 Position         */
#define PWM_ADCTSTS1_CDTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CDTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF6 Mask             */

#define PWM_ADCTSTS1_ZPTRGF6_Pos         (19)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF6 Position         */
#define PWM_ADCTSTS1_ZPTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_ZPTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF6 Mask             */

#define PWM_ADCTSTS1_CUTRGF7_Pos         (24)                                              /*!< PWM_T::ADCTSTS0: CUTRGF7 Position         */
#define PWM_ADCTSTS1_CUTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CUTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF7 Mask             */

#define PWM_ADCTSTS1_CPTRGF7_Pos         (25)                                              /*!< PWM_T::ADCTSTS0: CPTRGF7 Position         */
#define PWM_ADCTSTS1_CPTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CPTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF7 Mask             */

#define PWM_ADCTSTS1_CDTRGF7_Pos         (26)                                              /*!< PWM_T::ADCTSTS0: CDTRGF7 Position         */
#define PWM_ADCTSTS1_CDTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CDTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF7 Mask             */

#define PWM_ADCTSTS1_ZPTRGF7_Pos         (27)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF7 Position         */
#define PWM_ADCTSTS1_ZPTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_ZPTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF7 Mask             */

#define PWM_PCACTL_PCAEN_Pos             (0)                                               /*!< PWM_T::PCACTL: PCAEN Position             */
#define PWM_PCACTL_PCAEN_Msk             (0x1ul << PWM_PCACTL_PCAEN_Pos)

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */




/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct 
{
    __IO uint32_t X_FL_CTL;         //0X0
    __IO uint8_t X_FL_TRIG;         //0X4
    __IO uint8_t X_FL_CONFIG;       //0X5
    __IO uint8_t X_FL_WD[6];        //0X6
    __IO uint32_t X_FL_X_MODE;      //0XC
    __IO uint8_t X_FL_X2_CMD;       //0X10
    __IO uint8_t X_FL_X4_CMD;       //0X11
    __IO uint8_t X_FL_DP_CMD;       //0X12
    __IO uint8_t X_FL_RDP_CMD;      //0X13
    __IO uint32_t X_FL_REMAP_ADDR;  //0X14
    __IO uint32_t X_FL_DP_CTL;      //0X18
}FLCTL_T;

typedef struct 
{
    __IO uint8_t X_FL_BUFFER[256];
}FLCTL_W_BUFF_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for Flash Memory Controller
@{ */
//X_FL_CTL
#define FLCTL_CTL_BYTES_NUM_R_Pos           (8)
#define FLCTL_CTL_BYTES_NUM_R_Msk           (0xFFFFFFul << FLCTL_CTL_BYTES_NUM_R_Pos)
#define FLCTL_CTL_BYTES_NUM_W_Pos           (0)
#define FLCTL_CTL_BYTES_NUM_W_Msk           (0x7ul << FLCTL_CTL_BYTES_NUM_W_Pos)

//X_FL_TRIG
#define FLCTL_TRIG_TRG_Pos                  (0)
#define FLCTL_TRIG_TRG_Msk                  (0x1ul << FLCTL_TRIG_TRG_Pos)

//X_FL_CONFIG
#define FLCTL_CONFIG_TX_ADDR_TRANS_Pos      (2)
#define FLCTL_CONFIG_TX_ADDR_TRANS_Msk      (0x1ul << FLCTL_CONFIG_TX_ADDR_TRANS_Pos)
#define FLCTL_CONFIG_INFO_EN_Pos            (1)
#define FLCTL_CONFIG_INFO_EN_Msk            (0x1ul << FLCTL_CONFIG_INFO_EN_Pos)
#define FLCTL_CONFIG_PP_ACTIVE_Pos          (0)
#define FLCTL_CONFIG_PP_ACTIVE_Msk          (0x1ul << FLCTL_CONFIG_PP_ACTIVE_Pos)

//X_FL_WD[6]
#define FLCTL_W_DATA_Pos                    (0)
#define FLCTL_W_DATA_Msk                    (0xFFul << FLCTL_W_DATA_Pos)

//X_FL_X_MODE
#define FLCTL_X_MODE_CLK_DLY_Pos            (21)
#define FLCTL_X_MODE_CLK_DLY_Msk            (0x7ul << FLCTL_X_MODE_CLK_DLY_Pos)
#define FLCTL_X_MODE_RX_NEG_Pos             (20)
#define FLCTL_X_MODE_RX_NEG_Msk             (0x1ul << FLCTL_X_MODE_RX_NEG_Pos)
#define FLCTL_X_MODE_CRC32_EN_Pos           (19)
#define FLCTL_X_MODE_CRC32_EN_Msk           (0x1ul << FLCTL_X_MODE_CRC32_EN_Pos)
#define FLCTL_X_MODE_ENHANCE_MODE_Pos       (18)
#define FLCTL_X_MODE_ENHANCE_MODE_Msk       (0x1ul << FLCTL_X_MODE_ENHANCE_MODE_Pos)
#define FLCTL_X_MODE_LONG_TIME_OP_Pos       (17)
#define FLCTL_X_MODE_LONG_TIME_OP_Msk       (0x1ul << FLCTL_X_MODE_LONG_TIME_OP_Pos)
#define FLCTL_X_MODE_EN_BURST_WRAP_Pos      (16)
#define FLCTL_X_MODE_EN_BURST_WRAP_Msk      (0x1ul << FLCTL_X_MODE_EN_BURST_WRAP_Pos)
#define FLCTL_X_MODE_CS_IDLE_CYCLE_Pos      (8)
#define FLCTL_X_MODE_CS_IDLE_CYCLE_Msk      (0xFFul << FLCTL_X_MODE_CS_IDLE_CYCLE_Pos)
#define FLCTL_X_MODE_X_MODE_Pos             (0)
#define FLCTL_X_MODE_X_MODE_Msk             (0x3ul << FLCTL_X_MODE_X_MODE_Pos)

//X_FL_X2_CMD
#define FLCTL_X2_CMD_Pos                    (0)
#define FLCTL_X2_CMD_Msk                    (0xFFul << FLCTL_X2_CMD_Pos)

//X_FL_X4_CMD
#define FLCTL_X4_CMD_Pos                    (0)
#define FLCTL_X4_CMD_Msk                    (0xFFul << FLCTL_X4_CMD_Pos)

//X_FL_DP_CMD
#define FLCTL_DP_CMD_Pos                    (0)
#define FLCTL_DP_CMD_Msk                    (0xFFul << FLCTL_DP_CMD_Pos)

//X_FL_RDP_CMD
#define FLCTL_RDP_CMD_Pos                   (0)
#define FLCTL_RDP_CMD_Msk                   (0xFFul << FLCTL_RDP_CMD_Pos)

//X_FL_REMAP_ADDR
#define FLCTL_REMAP_ADDR_Pos                (0)
#define FLCTL_REMAP_ADDR_Msk                (0xFFul << FLCTL_REMAP_ADDR_Pos)

//X_FL_DP_CTL
#define FLCTL_DP_CTL_RDP_WT_CNT_Pos         (8)
#define FLCTL_DP_CTL_RDP_WT_CNT_Msk         (0xFFFFFFul << FLCTL_DP_CTL_RDP_WT_CNT_Pos)
#define FLCTL_DP_CTL_DP_EN_Pos              (0)
#define FLCTL_DP_CTL_DP_EN_Msk              (0x1ul << FLCTL_DP_CTL_DP_EN_Pos)

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


typedef struct 
{
    __IO uint32_t X_CACHE_EN;
	__IO uint32_t X_RESVR;
	__IO uint32_t X_CACHE_INI;
}CR_T;

/**@}*/ /* end of FMC register group */

/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */

typedef struct {
    __IO uint32_t CR0; 			//0x00
    __IO uint32_t CR1;			//0x04
    __IO uint32_t DR;			  //0x08
    __IO uint32_t SR; 	    //0x0C
    __IO uint32_t CPSR; 	  //0x10
    __IO uint32_t IMSC;			//0x14
    __IO uint32_t RIS;		  //0x18
    __IO uint32_t MIS;		  //0x1C
  	__IO uint32_t ICR;			//0x20
    __IO uint32_t DMACR;		//0x24
} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */
//CTRL0
#define SPI_CR0_SCR_Pos                   (8)
#define SPI_CR0_SCR_Msk                   (0xFFul << SPI_CR0_SCR_Pos)

#define SPI_CR0_SPH_Pos                   (7)
#define SPI_CR0_SPH_Msk                   (0x1ul << SPI_CR0_SPH_Pos)

#define SPI_CR0_SPO_Pos                   (6)
#define SPI_CR0_SPO_Msk                   (0x1ul << SPI_CR0_SPO_Pos)

#define SPI_CR0_FRF_Pos                   (4)
#define SPI_CR0_FRF_Msk                   (0x3ul << SPI_CR0_FRF_Pos)

#define SPI_CR0_DSS_Pos                   (0)
#define SPI_CR0_DSS_Msk                   (0xFul << SPI_CR0_DSS_Pos)
//CTRL1
#define SPI_CR1_RX_LSB_EN_Pos             (5)
#define SPI_CR1_RX_LSB_EN_Msk             (0x1ul << SPI_CR1_RX_LSB_EN_Pos)

#define SPI_CR1_TX_LSB_EN_Pos             (4)
#define SPI_CR1_TX_LSB_EN_Msk             (0x1ul << SPI_CR1_TX_LSB_EN_Pos)

#define SPI_CR1_SOD_Pos                   (3)
#define SPI_CR1_SOD_Msk                   (0x1ul << SPI_CR1_SOD_Pos)

#define SPI_CR1_MS_Pos              	  (2)
#define SPI_CR1_MS_Msk                    (0x1ul << SPI_CR1_MS_Pos)

#define SPI_CR1_SSE_Pos              	  (1)
#define SPI_CR1_SSE_Msk                   (0x1ul << SPI_CR1_SSE_Pos)

#define SPI_CR1_LBM_Pos                   (0)
#define SPI_CR1_LBM_Msk                   (0x1ul << SPI_CR1_LBM_Pos)
//DR
#define SPI_DR_Pos                        (0)
#define SPI_DR_Msk                        (0xFFul << SPI_DR_Pos)

//SR
#define SPI_SR_BUSY_Pos			      	  (4)
#define SPI_SR_BUSY_Msk                   (0x1ul << SPI_SR_BUSY_Pos)

#define SPI_SR_RFF_Pos			      	  (3)
#define SPI_SR_RFF_Msk                    (0x1ul << SPI_SR_RFF_Pos)

#define SPI_SR_RFNE_Pos			      	  (2)
#define SPI_SR_RFNE_Msk                   (0x1ul << SPI_SR_RFNE_Pos)

#define SPI_SR_TFNF_Pos			      	  (1)
#define SPI_SR_TFNF_Msk                   (0x1ul << SPI_SR_TFNF_Pos)

#define SPI_SR_TFE_Pos			      	  (0)
#define SPI_SR_TFE_Msk                    (0x1ul << SPI_SR_TFE_Pos)

//Prescale
#define SPI_CPSR_DIV_Pos			      (0)
#define SPI_CPSR_DIV_Msk                  (0xFul << SPI_CPSR_DIV_Pos)

//IMSC
#define SPI_IMSC_TXIM_Pos			      (3)
#define SPI_IMSC_TXIM_Msk                 (0x1ul << SPI_IMSC_TXIM_Pos)

#define SPI_IMSC_RXIM_Pos		      	  (2)
#define SPI_IMSC_RXIM_Msk                 (0x1ul << SPI_IMSC_RXIM_Pos)

#define SPI_IMSC_RTIM_Pos   	      	  (1)
#define SPI_IMSC_RTIM_Msk                 (0x1ul << SPI_IMSC_RTIM_Pos)

//RIS
#define SPI_RIS_TXIS_Pos			      (3)
#define SPI_RIS_TXIS_Msk                  (0x1ul << SPI_RIS_TXIS_Pos)

#define SPI_RIS_RXIS_Pos		      	  (2)
#define SPI_RIS_RXIS_Msk                  (0x1ul << SPI_RIS_RXIS_Pos)

#define SPI_RIS_RTIS_Pos   	      	      (1)
#define SPI_RIS_RTIS_Msk                  (0x1ul << SPI_RIS_RTIS_Pos)

#define SPI_RIS_RORIS_Pos   	      	  (0)
#define SPI_RIS_RORIS_Msk                 (0x1ul << SPI_RIS_RORIS_Pos)
//MIS
#define SPI_MIS_TXMIS_Pos 			      (3)
#define SPI_MIS_TXMIS_Msk                 (0x1ul << SPI_MIS_TXMIS_Pos)

#define SPI_MIS_RXMIS_Pos   	      	  (2)
#define SPI_MIS_RXMIS_Msk                 (0x1ul << SPI_MIS_RXMIS_Pos)

#define SPI_MIS_RTMIS_Pos    	      	  (1)
#define SPI_MIS_RTMIS_Msk                 (0x1ul << SPI_MIS_RTMIS_Pos)

#define SPI_MIS_RORMIS_Pos			      (0)
#define SPI_MIS_RORMIS_Msk                (0x1ul << SPI_MIS_RORMIS_Pos)

//ICR
#define SPI_ICR_RTICR_Pos    	      	  (1)
#define SPI_ICR_RTICR_Msk                 (0x1ul << SPI_ICR_RTICR_Pos)

#define SPI_ICR_RORICR_Pos			      (0)
#define SPI_ICR_RORICR_Msk                (0x1ul << SPI_ICR_RORICR_Pos)

//DMACR
#define SPI_DMACR_TDMAE_Pos			      (1)
#define SPI_DMACR_TDMAE_Msk               (0x1ul << SPI_DMACR_TDMAE_Pos)

#define SPI_DMACR_RDMAE_Pos			      (0)
#define SPI_DMACR_RDMAE_Msk               (0x1ul << SPI_DMACR_RDMAE_Pos)

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */







/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
    Memory Mapped Structure for UART Controller
@{ */

typedef struct {
     /**
     * RBR
     * ===================================================================================================
     * Offset: 0x00  UART Receive Buffer Register. This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Data byte received on the serial input port (sin) in UART mode,
     * |        |          |or the serial infrared input (sir_in) in infrared mode.
     * |        |          |
     * |[8]     |MSB9      |Data byte received on the serial input port (sin) in UART mode for the MSB 9th bit.
     * |        |          |It is applicable only when UART_9BIT_DATA_EN=1.
     */
    /**
     * THR
     * ===================================================================================================
     * Offset: 0x00  UART Transmit Holding Register. This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Transmit Holding Register
     * |        |          |Write :
     * |        |          |Data to be transmitted on the serial output port in UART mode or the serial infrared output in infrared mode.
     * |        |          |
     * |[8]     |MSB9      |Data to be transmitted on the serial output port in UART mode for the MSB 9th bit.
     * |        |          |It is applicable only when UART_9BIT_DATA_EN=1.
     */
    /**
     * DLL
     * ===================================================================================================
     * Offset: 0x00  Divisor Latch (Low) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLL      |Lower 8 bits  of a 16-bit, read/write,
     * |        |          |Divisor Latch register that contains the baud rate divisor for the UART.
     * |        |          |
     */
    __IO uint32_t RBR_THR_DLL;  // receive buffer register
                                    // receive buffer register
                                    // divisor latch low          (0x00)
    /**
     * DLH
     * ===================================================================================================
     * Offset: 0x04  Divisor Latch (High) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLH      |Upper 8-bits of a 16-bit, read/write,
     * |        |          |Divisor Latch register that contains the baud rate divisor for the UART.
     * |        |          |
     */
    /**
     * IER
     * ===================================================================================================
     * Offset: 0x04  Interrupt Enable Register  This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     | EPTI     |Enable Programmable THRE Interrupt . This is used to enable/disable the generation of THRE Interrupt.
     * |        |          |
     * |        |          |
     * |[6:4]   | Reserved |
     * |        |          |
     * |[3]     | EMSI     |Enable Modem Status Interrupt. This is used to enable/disable the generation of Modem Status Interrupt.
     * |        |          |This is the fourth highest priority interrupt.
     * |[2]     | ERLSI    |Enable Receiver Line Status Interrupt. This is used to enable/disable the generation of Receiver Line Status Interrupt.
     * |        |          |This is the highest priority interrupt.
     * |[1]     | ETHREI   |Enable Transmit Holding Register Empty Interrupt. This is used to enable/disable the generation of Transmitter Holding Register Empty Interrupt.
     * |        |          |This is the third highest priority interrupt.
     * |[0]     | ERDAI    |Enable Received Data Available Interrupt. This is used to enable/disable the generation of Received Data Available Interrupt and the Character Timeout Interrupt.
     * |        |          |These are the second highest priority interrupts.
     */
    __IO uint32_t IER_DLH;      // interrupt enable register
                                    // divisor latch high         (0x04)
    /**
     * IIR
     * ===================================================================================================
     * Offset: 0x08  Interrupt Identity Register -- read-only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:6]   | FIFOSE   |FIFOs Enabled. This is used to indicate whether the FIFOs are enabled or disabled..
     * |        |          |
     * |[5:4]   | Reserved |
     * |        |          |
     * |[3:0]   | IID      |Interrupt ID. This indicates the highest priority pending interrupt which can be one of the following types:
     * |        |          |0000 - modem status
     * |        |          |0001 - no interrupt pending
     * |        |          |0010 - THR empty
     * |        |          |0100 - received data available
     * |        |          |0110 - receiver line status
     * |        |          |0111 - reserved
     * |        |          |1100 - character timeout
     * |        |          |Bit 3 indicates an interrupt can only occur when the FIFOs are enabled and used to distinguish a Character Timeout condition interrupt.
     */

    /**
     * FCR
     * ===================================================================================================
     * Offset: 0x08  FIFO Control Register --write-only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:6]   | RT       |RCVR Trigger:
     * |        |          |00 - 1 character in the FIFO
     * |        |          |01 - FIFO 1/4 full
     * |        |          |10 - FIFO 1/2 full
     * |        |          |11 - FIFO 2 less than full
     * |        |          |
     * |[5:4]   | TET      |TX Empty Trigger
     * |        |          |00 - FIFO empty
     * |        |          |01 - 2 Characters in the FIFO
     * |        |          |10 - FIFO 1/4 full
     * |        |          |11 - FIFO 1/2 full
     * |        |          |
     * |[3]     |Reserved  |
     * |[2]     | XFIFOR   |XMIT FIFO Reset. This resets the control portion of the transmit FIFO and treats the FIFO as empty.
     * |        |          |
     * |[1]     | RFIFOR   |RCVR FIFO Reset. This resets the control portion of the receive FIFO and treats the FIFO as empty.
     * |        |          |
     * |[0]     | FIFOE    |FIFO Enable. This enables/disables the transmit (XMIT) and receive (RCVR) FIFOs.
     * |        |          |Whenever the value of this bit is changed both the XMIT and RCVR controller portion of FIFOs is reset.
     */

    __IO uint32_t IIR_FCR;      // interrupt identity register

    /**
     * LCR
     * ===================================================================================================
     * Offset: 0x0C  UART Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |DLAB      |Divisor Latch Access Bit.  Always writable, always readable.
     * |        |          |This bit is used to enable reading and writing of the Divisor Latch register (DLL and DLH/LPDLL and LPDLH) to set the baud rate of the UART.
     * |        |          |This bit must be cleared after initial baud rate setup in order to access other registers.
     * |        |          |
     * |[6]     |BC        |Break Control Bit. This is used to cause a break condition to be transmitted to the receiving device.
     * |        |          |
     * |[5]     |SP        |Stick Parity. Always writable, always readable. This bit is used to force parity value.
     * |        |          |
     * |[4]     |EPS       |Even Parity Select. Always writable, always readable.
     * |        |          |This is used to select between even and odd parity, when parity is enabled (PEN set to 1)
     * |        |          |
     * |        |          |
     * |[3]     |PEN       |Parity Enable. Always writable, always readable.
     * |        |          |This bit is used to enable and disable parity generation and detection in transmitted and received serial character respectively.
     * |        |          |
     * |        |          |
     * |[2]     |STOP      |Number of stop bits.  Always writable, always readable.
     * |        |          |This is used to select the number of stop bits per character that the peripheral transmits and receives.
     * |[1:0]   |DLS       |
     * |        |          |Data Length Select.  Always writable and readable.
     * |        |          |When DLS_E in LCR_EXT is set to 0, this register is used to select the number of data bits per character
     * |        |          |that the peripheral transmits and receives. The number
     * |        |          |of bits that may be selected are as follows:
     * |        |          |00 - 5 bits
     * |        |          |01 - 6 bits
     * |        |          |10 - 7 bits
     * |        |          |11 - 8 bits
    */
    __IO uint32_t LCR;
    /**
     * MCR
     * ===================================================================================================
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |SIRE      |SIR Mode Enable.
     * |        |          |
     * |[5]     |AFCE      |Auto Flow Control Enable.
     * |        |          |
     * |[4]     |LB        |LoopBack Bit. This is used to put the UART into a diagnostic mode for test purposes..
     * |[3]     |OUT2      |
     * |[2]     |OUT1      |
     * |[1]     |RTS       |Request to Send. This is used to directly control the Request to Send (rts_n) output.
     * |        |          |The Request To Send (rts_n) output is used to inform the modem or data set that the UART is ready to exchange data.
     * |[0]     |DTR       |Data Terminal Ready. This is used to directly control the Data Terminal Ready (dtr_n) output.
     */
    __IO uint32_t MCR;
   /**
     * LSR
     * ===================================================================================================
     * Offset: 0x14  UART Line Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |ADDR_RCVD |Address Received bit.
     * |[7]     |RFE       |Receiver FIFO Error bit. .
     * |[6]     |TEMT      |Transmitter Empty bit.
     * |        |          |
     * |[5]     |THRE      |Transmit Holding Register Empty bit.
     * |        |          |
     * |[4]     |BI        |Break Interrupt bit.
     * |[3]     |FE        |Framing Error bit
     * |[2]     |PE        |Parity Error bit
     * |[1]     |OE        |Overrun error bit.
     * |        |          |
     * |[0]     |DR        |Data Ready bit. This is used to indicate that the receiver contains at least one character in the RBR or the receiver FIFO
     */
    __IO uint32_t LSR;
   /**
     * LSR
     * ===================================================================================================
     * Offset: 0x18  UART Line Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |DCD       |Data Carrier Detect.
     * |[6]     |RI        |Ring Indicator.
     * |        |          |
     * |[5]     |DSR       |Data Set Ready
     * |        |          |
     * |[4]     |CTS       |Clear to Send. This is used to indicate the current state of the modem control line cts_n.
     * |        |          |This bit is the complement of cts_n. When the Clear to Send input (cts_n) is asserted it is an indication that the modem or data set is ready to exchange data with the uart.

     * |[3]     |DDCD      |Delta Data Carrier Detect
     * |[2]     |TERI      |Trailing Edge of Ring Indicator
     * |[1]     |DDSR      |Delta Data Set Ready
     * |        |          |
     * |[0]     |DCTS      |Delta Clear to Send
     */

    __IO uint32_t MSR;
   /**
     * SCR
     * ===================================================================================================
     * Offset: 0x1C  Scratchpad Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  | SCR      |This register is for programmers to use as a temporary storage space. It has no defined purpose in the uart
     */

    __IO uint32_t SCR;
   /**
     * LPDLL
     * ===================================================================================================
     * Offset: 0x20  Divisor Latch (Low) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | LPDLL    |Lower 8 bits  of a 16-bit, read/write,
     * |        |          |Low Power Divisor Latch register that contains the baud rate divisor for the UART,
     * |        |          |which must give a baud rate of 115.2K. This is required for SIR Low Power (minimum pulse width) detection at the receiver.
     */
    __IO uint32_t LPDLL;
   /**
     * LPDLH
     * ===================================================================================================
     * Offset: 0x24  Divisor Latch (High) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | LPDLH    |Upper 8-bits of a 16-bit, read/write,
     * |        |          |Low Power Divisor Latch register that contains the baud rate divisor for the UART,
     * |        |          |which must give a baud rate of 115.2K. This is required for SIR Low Power (minimum pulse width) detection at the receiver.
     */
    __IO uint32_t LPDLH;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[21];
    /// @endcond //HIDDEN_SYMBOLS
    /**
     * USR
     * ===================================================================================================
     * Offset: 0x7C  UART Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |RFF       |Receive FIFO Full. This is used to indicate that the receive FIFO is completely full.
     * |[3]     |RFNE      |Receive FIFO Not Empty. This is used to indicate that the receive FIFO contains one or more entries
     * |[2]     |TFE       |Transmit FIFO Empty. This is used to indicate that the transmit FIFO is completely empty
     * |[1]     |TFNF      |Transmit FIFO Not Full. This is used to indicate that the transmit FIFO in not full.
     * |        |          |
     */
    __IO uint32_t USR;
   /**
     * TFL
     * ===================================================================================================
     * Offset: 0x80  Transmit FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   | TFL      |Transmit FIFO Level. This indicates the number of data entries in the transmit FIFO.
     * |        |          |
     */

    __IO uint32_t TFL;
   /**
     * RFL
     * ===================================================================================================
     * Offset: 0x84  Receive  FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   | RFL      |Receive FIFO Level. This indicates the number of data entries in the receive FIFO.
     * |        |          |
     */
    __IO uint32_t RFL;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[7];
    /// @endcond //HIDDEN_SYMBOLS
		
   /**
     * HTX
     * ===================================================================================================
     * Offset: 0xA4  halt TX
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     | htx      |This register is use to halt transmissions for testing, so that the transmit FIFO can be
		 *										 |filled by the master when FIFOs are implemented and enabled.
     * |        |          |
     */
    __IO uint32_t HTX;		
		
   /**
     * DMASA
     * ===================================================================================================
     * Offset: 0xA8  DMA Software Acknowledge
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     | RFL      |This register is use to perform a DMA software acknowledge if a transfer needs to be terminated due to an error condition.
     * |        |          |
     */
    __IO uint32_t DMASA;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED2[5];
    /// @endcond //HIDDEN_SYMBOLS
   /**
     * DLF
     * ===================================================================================================
     * Offset: 0xC0  Receive  FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   | DLF      |Fractional part of divisor.
     * |        |          |
     */
    __IO uint32_t DLF;
   /**
     * RAR
     * ===================================================================================================
     * Offset: 0xC4  Receive Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLF      |This is an address matching register during receive mode.
     * |        |          |If the 9-th bit is set in the incoming character then the remaining 8-bits will be checked against this register value.
     */
    __IO uint32_t RAR;
   /**
     * TAR
     * ===================================================================================================
     * Offset: 0xC8  Transmit Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLF      |This is an address matching register during transmit mode. If DLS_E
     * |        |          |(LCR_EXT[0]) bit is enabled, then uart sends the 9-bit character with 9-th bit set to 1 and
     * |        |          |remaining 8-bit address will be sent from this register provided 'SEND_ADDR' (LCR_EXT[2]) bit is set to 1.
     */
    __IO uint32_t TAR;
   /**
     * TAR
     * ===================================================================================================
     * Offset: 0xC8  Transmit Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3]     |TRANSMIT_ |Transmit mode control bit. This bit is used to control the type of transmit mode during 9-bit data transfers.
     * |        |MODE      |1 - In this mode of operation, Transmit Holding Register (THR) is 9-bit wide. You must ensure that the THR register is written correctly for address/data.
     * |        |          |Address: 9th bit is set to 1,Data: 9th bit is set to 0.
     * |        |          |NOTE: Transmit address register (TAR) is not applicable in this mode of operation
     * |        |          |0 - In this mode of operation, Transmit Holding Register (THR) is 8-bit wide.
     * |        |          |The user needs to program the address into Transmit Address Register (TAR) and data into the THR register.
     * |        |          |SEND_ADDR bit is used as a control knob to indicate the uart on when to send the address.
     * |        |          |
     * |[2]     |SEND_ADDR |Send address control bit. This bit is used as a control knob for the user to
     * |        |          |determine when to send the address during transmit mode.
     * |        |          |1 - 9-bit character will be transmitted with 9-th bit set to 1
     * |        |          |and the remaining 8-bits will match to what is being programmed in "Transmit Address Register".
     * |        |          |0 - 9-bit character will be transmitted with 9-th bit set to 0
     * |        |          |and the remaining 8- bits will be taken from the TxFIFO which is programmed through 8-bit wide THR register.
     * |        |          |NOTE:
     * |        |          |1. This bit is auto-cleared by the hardware, after sending out the address character. User is not expected to program this bit to 0.
     * |        |          |2. This field is applicable only when DLS_E bit is set to 1 and TRANSMIT_MODE is set to 0.
     * |        |          |
     * |[1]     |ADDR_MATCH|Address Match Mode. This bit is used to enable the address match feature during receive
     * |        |          |1 - Address match mode; uart will wait until the incoming character with 9-th bit set to 1.
     * |        |          |And, further checks to see if the address matches with what is programmed in "Receive Address Match Register".
     * |        |          |If match is found, then sub-sequent characters will be treated as valid data and uart starts receiving data.
     * |        |          |0 - Normal mode; uart will start to receive the data and 9-bit character will be formed
     * |        |          |and written into the receive RxFIFO. User is responsible to read the data and differentiate b/n address and data.
     * |        |          |NOTE: This field is applicable only when DLS_E is set to 1.
     * |        |          |
     * |[0]     | DLS_E    |Extension for DLS. This bit is used to enable 9-bit data for transmit and receive transfers.
     * |        |          |1 - 9 bits per character
     * |        |          |0 - Number of data bits selected by DLS
     */

    __IO uint32_t LCR_EXT;

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_RBR_MSB9_Pos                (8)
#define UART_RBR_MSB9_Msk                (0x1ul << UART_RBR_MSB9_Pos)

#define UART_RBR_DAT_Pos                 (0)
#define UART_RBR_DAT_Msk                 (0xfful << UART_RBR_DAT_Pos)

#define UART_THR_MSB9_Pos                (8)
#define UART_THR_MSB9_Msk                (0x1ul << UART_THR_MSB9_Pos)

#define UART_THR_DAT_Pos                 (0)
#define UART_THR_DAT_Msk                 (0xfful << UART_THR_DAT_Pos)

#define UART_DLH_Pos                     (0)
#define UART_DLH_Msk                     (0xfful << UART_DLH_Pos)

#define UART_DLL_Pos                     (0)
#define UART_DLL_Msk                     (0xfful << UART_DLL_Pos)
//Interrupt enable
#define UART_IER_EPTI_Pos                (7)
#define UART_IER_EPTI_Msk                (0x1ul << UART_IER_EPTI_Pos)

#define UART_IER_EMSI_Pos                (3)
#define UART_IER_EMSI_Msk                (0x1ul << UART_IER_EMSI_Pos)
#define UART_IER_ERLSI_Pos               (2)
#define UART_IER_ERLSI_Msk               (0x1ul << UART_IER_ERLSI_Pos)
#define UART_IER_ETHREI_Pos              (1)
#define UART_IER_ETHREI_Msk              (0x1ul << UART_IER_ETHREI_Pos)
#define UART_IER_ERDAI_Pos               (0)
#define UART_IER_ERDAI_Msk               (0x1ul << UART_IER_ERDAI_Pos)

#define UART_IER_ALL_IRQ_Pos             (0)
#define UART_IER_ALL_IRQ_Msk             (0xful << UART_IER_ALL_IRQ_Pos)
//Interrput ID
#define UART_IIR_FIFOSE_Pos              (6)
#define UART_IIR_FIFOSE_Msk              (0x3ul << UART_IIR_FIFOSE_Pos)

#define UART_IIR_IID_Pos                 (0)
#define UART_IIR_IID_Msk                 (0xful << UART_IIR_IID_Pos)

//FIFO Control Register
#define UART_FCR_RT_Pos                 (6)
#define UART_FCR_RT_Msk                 (0x3ul << UART_FCR_RT_Pos)
#define UART_FCR_TET_Pos                (4)
#define UART_FCR_TET_Msk                (0x3ul << UART_FCR_TET_Pos)
#define UART_FCR_DMAM_Pos             	(3)
#define UART_FCR_DMAM_Msk             	(0x1ul << UART_FCR_DMAM_Pos)
#define UART_FCR_XFIFOR_Pos             (2)
#define UART_FCR_XFIFOR_Msk             (0x1ul << UART_FCR_XFIFOR_Pos)
#define UART_FCR_RFIFOR_Pos             (1)
#define UART_FCR_RFIFOR_Msk             (0x1ul << UART_FCR_RFIFOR_Pos)
#define UART_FCR_FIFOE_Pos              (0)
#define UART_FCR_FIFOE_Msk              (0x1ul << UART_FCR_FIFOE_Pos)

//Line Control Register
#define UART_LCR_DLAB_Pos              (7)
#define UART_LCR_DLAB_Msk              (0x1ul << UART_LCR_DLAB_Pos)
#define UART_LCR_BC_Pos                (6)
#define UART_LCR_BC_Msk                (0x1ul << UART_LCR_BC_Pos)
#define UART_LCR_SP_Pos                (5)
#define UART_LCR_SP_Msk                (0x1ul << UART_LCR_SP_Pos)
#define UART_LCR_EPS_Pos               (4)
#define UART_LCR_EPS_Msk               (0x1ul << UART_LCR_EPS_Pos)
#define UART_LCR_PEN_Pos               (3)
#define UART_LCR_PEN_Msk               (0x1ul << UART_LCR_PEN_Pos)
#define UART_LCR_STOP_Pos              (2)
#define UART_LCR_STOP_Msk              (0x1ul << UART_LCR_STOP_Pos)
#define UART_LCR_DLS_Pos               (0)
#define UART_LCR_DLS_Msk               (0x3ul << UART_LCR_DLS_Pos)

//Modem Control Register
#define UART_MCR_SIRE_Pos               (6)
#define UART_MCR_SIRE_Msk               (0x1ul << UART_MCR_SIRE_Pos)
#define UART_MCR_AFCE_Pos               (5)
#define UART_MCR_AFCE_Msk               (0x1ul << UART_MCR_AFCE_Pos)
#define UART_MCR_LB_Pos                 (4)
#define UART_MCR_LB_Msk                 (0x1ul << UART_MCR_LB_Pos)
#define UART_MCR_OUT2_Pos               (3)
#define UART_MCR_OUT2_Msk               (0x1ul << UART_MCR_OUT2_Pos)
#define UART_MCR_OUT1_Pos               (2)
#define UART_MCR_OUT1_Msk               (0x1ul << UART_MCR_OUT1_Pos)
#define UART_MCR_RTS_Pos                (1)
#define UART_MCR_RTS_Msk                (0x1ul << UART_MCR_RTS_Pos)
#define UART_MCR_DTR_Pos                (0)
#define UART_MCR_DTR_Msk                (0x1ul << UART_MCR_DTR_Pos)

//Line Status Register
#define UART_LSR_ADDR_RCVD_Pos          (8)
#define UART_LSR_ADDR_RCVD_Msk          (0x1ul << UART_LSR_ADDR_RCVD_Pos)
#define UART_LSR_RFE_Pos                (7)
#define UART_LSR_RFE_Msk                (0x1ul << UART_LSR_RFE_Pos)
#define UART_LSR_TEMT_Pos               (6)
#define UART_LSR_TEMT_Msk               (0x1ul << UART_LSR_TEMT_Pos)
#define UART_LSR_THRE_Pos               (5)
#define UART_LSR_THRE_Msk               (0x1ul << UART_LSR_THRE_Pos)
#define UART_LSR_BI_Pos                 (4)
#define UART_LSR_BI_Msk                 (0x1ul << UART_LSR_BI_Pos)
#define UART_LSR_FE_Pos                 (3)
#define UART_LSR_FE_Msk                 (0x1ul << UART_LSR_FE_Pos)
#define UART_LSR_PE_Pos                 (2)
#define UART_LSR_PE_Msk                 (0x1ul << UART_LSR_PE_Pos)
#define UART_LSR_OE_Pos                 (1)
#define UART_LSR_OE_Msk                 (0x1ul << UART_LSR_OE_Pos)
#define UART_LSR_DR_Pos                 (0)
#define UART_LSR_DR_Msk                 (0x1ul << UART_LSR_DR_Pos)

#define UART_LSR_LINE_STATUS_Pos        (0)
#define UART_LSR_LINE_STATUS_Msk        (0x1FFul << UART_LSR_LINE_STATUS_Pos)

//Modem Status Register
#define UART_MSR_DCD_Pos                 (7)
#define UART_MSR_DCD_Msk                 (0x1ul << UART_MSR_DCD_Pos)
#define UART_MSR_RI_Pos                  (6)
#define UART_MSR_RI_Msk                  (0x1ul << UART_MSR_RI_Pos)
#define UART_MSR_DSR_Pos                 (5)
#define UART_MSR_DSR_Msk                 (0x1ul << UART_MSR_DSR_Pos)
#define UART_MSR_CTS_Pos                 (4)
#define UART_MSR_CTS_Msk                 (0x1ul << UART_MSR_CTS_Pos)
#define UART_MSR_DDCD_Pos                (3)
#define UART_MSR_DDCD_Msk                (0x1ul << UART_MSR_DDCD_Pos)
#define UART_MSR_TERI_Pos                (2)
#define UART_MSR_TERI_Msk                (0x1ul << UART_MSR_TERI_Pos)
#define UART_MSR_DDSR_Pos                (1)
#define UART_MSR_DDSR_Msk                (0x1ul << UART_MSR_DDSR_Pos)
#define UART_MSR_DCTS_Pos                (0)
#define UART_MSR_DCTS_Msk                (0x1ul << UART_MSR_DCTS_Pos)

#define UART_MSR_MODEM_STATUS_Pos        (0)
#define UART_MSR_MODEM_STATUS_Msk        (0xFFul << UART_MSR_MODEM_STATUS_Pos)

//Scratchpad register
#define UART_SCR_Pos                     (0)
#define UART_SCR_Msk                     (0xFFFFFFFFul << UART_SCR_Pos)

//Low Power Divisor Latch Low Register
#define UART_LPDLL_Pos                   (0)
#define UART_LPDLL_Msk                   (0xFFul << UART_LPDLL_Pos)

//Low Power Divisor Latch High Register
#define UART_LPDLH_Pos                   (0)
#define UART_LPDLH_Msk                   (0xFFul << UART_LPDLH_Pos)

//Uart Status Register
#define UART_USR_RFF_Pos                 (4)
#define UART_USR_RFF_Msk                 (0x1ul << UART_USR_RFF_Pos)
#define UART_USR_RFNE_Pos                (3)
#define UART_USR_RFNE_Msk                (0x1ul << UART_USR_RFNE_Pos)
#define UART_USR_TFE_Pos                 (2)
#define UART_USR_TFE_Msk                 (0x1ul << UART_USR_TFE_Pos)
#define UART_USR_TFNF_Pos                (1)
#define UART_USR_TFNF_Msk                (0x1ul << UART_USR_TFNF_Pos)

//Transmit FIFO Level
#define UART_TFL_Pos                     (0)
#define UART_TFL_Msk                     (0x1Ful << UART_TFL_Pos)

//Receive FIFO Level
#define UART_RFL_Pos                     (0)
#define UART_RFL_Msk                     (0x1Ful << UART_RFL_Pos)

//DMA software Acknowledge
#define UART_DMASA_Pos                   (0)
#define UART_DMASA_Msk                   (0x1ul << UART_DMASA_Pos)

//Divisor Latch Fraction Register
#define UART_DLF_Pos                     (0)
#define UART_DLF_Msk                     (0xFul << UART_DLF_Pos)

//Receive Address Register
#define UART_RAR_Pos                     (0)
#define UART_RAR_Msk                     (0xFFul << UART_RAR_Pos)

//Transmit Address Register
#define UART_TAR_Pos                     (0)
#define UART_TAR_Msk                     (0xFFul << UART_TAR_Pos)


// Line Extented Control Register

#define UART_LCR_EXT_TRANSMIT_MODE_Pos      (3)
#define UART_LCR_EXT_TRANSMIT_MODE_Msk      (0x1ul << UART_LCR_EXT_TRANSMIT_MODE_Pos)
#define UART_LCR_EXT_SEND_ADDR_Pos          (2)
#define UART_LCR_EXT_SEND_ADDR_Msk          (0x1ul << UART_LCR_EXT_SEND_ADDR_Pos)
#define UART_LCR_EXT_ADDR_MATCH_Pos         (1)
#define UART_LCR_EXT_ADDR_MATCH_Msk         (0x1ul << UART_LCR_EXT_ADDR_MATCH_Pos)
#define UART_LCR_EXT_DLS_E_Pos              (0)
#define UART_LCR_EXT_DLS_E_Msk              (0x1ul << UART_LCR_EXT_DLS_E_Pos)


/******************************************************************************/
/*                                                                            */
/*                      UART Interface                                        */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for Data Register ********************/

#define UART_DR_OE                                              ((uint32_t)0x00000800)
#define UART_DR_BE                                              ((uint32_t)0x00000400)
#define UART_DR_PE                                              ((uint32_t)0x00000200)
#define UART_DR_FE                                              ((uint32_t)0x00000100)
#define UART_DR_DATA                                            ((uint32_t)0x000000FF)

/*******************  Bit definition for UARTRSR/UARTECR Register ********************/

#define UART_RSCR_OE                                             ((uint32_t)0x00000008)
#define UART_RSCR_BE                                             ((uint32_t)0x00000004)
#define UART_RSCR_PE                                             ((uint32_t)0x00000002)
#define UART_RSCR_FE                                             ((uint32_t)0x00000001)
//A write to these bits clears the framing, parity, break, and overrun errors.
#define UART_RSCR_CLR_ERR                                        ((uint32_t)0x000000ff)


/*******************  Bit definition for UARTFR Register ********************/

#define UART_STATUS_RI                                             ((uint32_t)0x00000100)
#define UART_STATUS_TXFE                                           ((uint32_t)0x00000080)
#define UART_STATUS_RXFF                                           ((uint32_t)0x00000040)
#define UART_STATUS_TXFF                                           ((uint32_t)0x00000020)
#define UART_STATUS_RXFE                                           ((uint32_t)0x00000010)
#define UART_STATUS_BUSY                                           ((uint32_t)0x00000008)
#define UART_STATUS_DCD                                            ((uint32_t)0x00000004)
#define UART_STATUS_DSR                                            ((uint32_t)0x00000002)
#define UART_STATUS_CTS                                            ((uint32_t)0x00000001)

/*******************  Bit definition for UARTILPR Register ********************/
#define UART_IRDA_LP_DIV                                         ((uint32_t)0x000000FF)

/*******************  Bit definition for UARTIBRD Register ********************/
#define UART_BRD_IBRD                                                ((uint32_t)0x0000FFFF)

/*******************  Bit definition for UARTFBRD Register ********************/
#define UART_BRD_FBRD                                                ((uint32_t)0x0000003F)

/*******************  Bit definition for UARTLCR_H Register ********************/
#define UART_LC_CLEAR_MSK                                        ((uint32_t)0x000000FF)

#define UART_LC_SPS                                              ((uint32_t)0x00000080)
#define UART_LC_WLEN                                             ((uint32_t)0x00000060)
#define UART_LC_FEN                                              ((uint32_t)0x00000010)
#define UART_LC_STP2                                             ((uint32_t)0x00000008)
#define UART_LC_EPS                                              ((uint32_t)0x00000004)
#define UART_LC_PEN                                              ((uint32_t)0x00000002)
#define UART_LC_BRK                                              ((uint32_t)0x00000001)

/*******************  Bit definition for UARTCR Register ********************/
#define UART_CR_CLEAR_MSK                                        ((uint32_t)0x0000FF87)

#define UART_CR_CTSEN                                            ((uint32_t)0x00008000)
#define UART_CR_RTSEN                                            ((uint32_t)0x00004000)
#define UART_CR_OUT2                                             ((uint32_t)0x00002000)
#define UART_CR_OUT1                                             ((uint32_t)0x00001000)
#define UART_CR_RTS                                              ((uint32_t)0x00000800)
#define UART_CR_DTR                                              ((uint32_t)0x00000400)
#define UART_CR_RXE                                              ((uint32_t)0x00000200)
#define UART_CR_TXE                                              ((uint32_t)0x00000100)
#define UART_CR_LBE                                              ((uint32_t)0x00000080)
#define UART_CR_SIRLP                                            ((uint32_t)0x00000004)
#define UART_CR_SIREN                                            ((uint32_t)0x00000002)
#define UART_CR_UARTEN                                           ((uint32_t)0x00000001)

/*******************  Bit definition for UARTIFLS Register ********************/
#define UART_INT_FIFO_RXIFLSEL                                   ((uint32_t)0x00000038)
#define UART_INT_FIFO_TXIFLSEL                                   ((uint32_t)0x00000007)

/*******************  Bit definition for UARTIMSC Register ********************/
#define UART_INT_MSK_OEIM                                        ((uint32_t)0x00000400)
#define UART_INT_MSK_BEIM                                        ((uint32_t)0x00000200)
#define UART_INT_MSK_PEIM                                        ((uint32_t)0x00000100)
#define UART_INT_MSK_FEIM                                        ((uint32_t)0x00000080)
#define UART_INT_MSK_RTIM                                        ((uint32_t)0x00000040)
#define UART_INT_MSK_TXIM                                        ((uint32_t)0x00000020)
#define UART_INT_MSK_RXIM                                        ((uint32_t)0x00000010)
#define UART_INT_MSK_DSRMIM                                      ((uint32_t)0x00000008)
#define UART_INT_MSK_DCDMIM                                      ((uint32_t)0x00000004)
#define UART_INT_MSK_CTSMIM                                      ((uint32_t)0x00000002)
#define UART_INT_MSK_RIMIM                                       ((uint32_t)0x00000001)

/*******************  Bit definition for UARTRIS Register ********************/
#define UART_RAW_INT_MSK_OEIM                                        ((uint32_t)0x00000400)
#define UART_RAW_INT_MSK_BEIM                                        ((uint32_t)0x00000200)
#define UART_RAW_INT_MSK_PEIM                                        ((uint32_t)0x00000100)
#define UART_RAW_INT_MSK_FEIM                                        ((uint32_t)0x00000080)
#define UART_RAW_INT_MSK_RTIM                                        ((uint32_t)0x00000040)
#define UART_RAW_INT_MSK_TXIM                                        ((uint32_t)0x00000020)
#define UART_RAW_INT_MSK_RXIM                                        ((uint32_t)0x00000010)
#define UART_RAW_INT_MSK_DSRMIM                                      ((uint32_t)0x00000008)
#define UART_RAW_INT_MSK_DCDMIM                                      ((uint32_t)0x00000004)
#define UART_RAW_INT_MSK_CTSMIM                                      ((uint32_t)0x00000002)
#define UART_RAW_INT_MSK_RIMIM                                       ((uint32_t)0x00000001)

/*******************  Bit definition for UARTMIS Register ********************/
#define UART_MASKED_INT_STAT_OEIM                                        ((uint32_t)0x00000400)
#define UART_MASKED_INT_STAT_BEIM                                        ((uint32_t)0x00000200)
#define UART_MASKED_INT_STAT_PEIM                                        ((uint32_t)0x00000100)
#define UART_MASKED_INT_STAT_FEIM                                        ((uint32_t)0x00000080)
#define UART_MASKED_INT_STAT_RTIM                                        ((uint32_t)0x00000040)
#define UART_MASKED_INT_STAT_TXIM                                        ((uint32_t)0x00000020)
#define UART_MASKED_INT_STAT_RXIM                                        ((uint32_t)0x00000010)
#define UART_MASKED_INT_STAT_DSRMIM                                      ((uint32_t)0x00000008)
#define UART_MASKED_INT_STAT_DCDMIM                                      ((uint32_t)0x00000004)
#define UART_MASKED_INT_STAT_CTSMIM                                      ((uint32_t)0x00000002)
#define UART_MASKED_INT_STAT_RIMIM                                       ((uint32_t)0x00000001)

/*******************  Bit definition for UARTICR Register ********************/
#define UART_CLR_INT_OEIM                                        ((uint32_t)0x00000400)
#define UART_CLR_INT_BEIM                                        ((uint32_t)0x00000200)
#define UART_CLR_INT_PEIM                                        ((uint32_t)0x00000100)
#define UART_CLR_INT_FEIM                                        ((uint32_t)0x00000080)
#define UART_CLR_INT_RTIM                                        ((uint32_t)0x00000040)
#define UART_CLR_INT_TXIM                                        ((uint32_t)0x00000020)
#define UART_CLR_INT_RXIM                                        ((uint32_t)0x00000010)
#define UART_CLR_INT_DSRMIM                                      ((uint32_t)0x00000008)
#define UART_CLR_INT_DCDMIM                                      ((uint32_t)0x00000004)
#define UART_CLR_INT_CTSMIM                                      ((uint32_t)0x00000002)
#define UART_CLR_INT_RIMIM                                       ((uint32_t)0x00000001)

/*******************  Bit definition for UARTDMACR Register ********************/
#define UART_DMA_CR_DMAONERR                                      ((uint32_t)0x00000004)
#define UART_DMA_CR_TXDMAE                                        ((uint32_t)0x00000002)
#define UART_DMA_CR_RXDMAE                                        ((uint32_t)0x00000001)

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */





/**
    @addtogroup DMA Controller
    Memory Mapped Structure for DMA Controller
@{ */

/**
  * @brief Dma Interface
  */
typedef struct {

  /* Channel registers                                    */
  /* The offset address for each of the channel registers */
  /*  is shown for channel 0. For other channel numbers   */
  /*  use the following equation.                         */
  /*                                                      */
  /*    offset = (channel_num * 0x058) + channel_0 offset */
  /*                                                      */
  struct {
      __IO uint32_t SAR_L;     /* Source Address Reg      (0x000) */
      __IO uint32_t SAR_H;
      __IO uint32_t DAR_L;     /* Destination Address Reg (0x008) */
      __IO uint32_t DAR_H;
      #if 0
      __IO uint32_t LLP_L;     /* Linked List Pointer Reg (0x010) */
      __IO uint32_t LLP_H;
      #else
      __IO uint32_t RESERVED0[2];

      #endif
      __IO uint32_t CTL_L;     /* Control Reg             (0x018) */
      __IO uint32_t CTL_H;
      #if 0
      __IO uint32_t sstat_l;   /* Source Status Reg       (0x020) */
      __IO uint32_t sstat_h;
      __IO uint32_t dstat_l;   /* Destination Status Reg  (0x028) */
      __IO uint32_t dstat_h;
      __IO uint32_t sstatar_l; /* Source Status Addr Reg  (0x030) */
      __IO uint32_t sstatar_h;
      __IO uint32_t dstatar_l; /* Dest Status Addr Reg    (0x038) */
      __IO uint32_t dstatar_h;
      #else
      __IO uint32_t RESERVED1[8];
      #endif
      __IO uint32_t CFG_L;     /* Configuration Reg       (0x040) */
      __IO uint32_t CFG_H;
      #if 1
      __IO uint32_t SGR_L;     /* Source Gather Reg       (0x048) */
      __IO uint32_t SGR_H;
      __IO uint32_t DSR_L;     /* Destination Scatter Reg (0x050) */
      __IO uint32_t DSR_H;
      #else
      __IO uint32_t RESERVED2[4];
      #endif
  } CH[8];

  /* Interrupt Raw Status Registers */
  __IO uint32_t RAW_TFR_L;     /* Raw Status for IntTfr   (0x2c0) */
  __IO uint32_t RAW_TFR_H;
  __IO uint32_t RAW_BLOCK_L;   /* Raw Status for IntBlock (0x2c8) */
  __IO uint32_t RAW_BLOCK_H;
  __IO uint32_t RAW_SRCTRAN_L; /* Raw Status IntSrcTran   (0x2d0) */
  __IO uint32_t RAW_SRCTRAN_H;
  __IO uint32_t RAW_DSTTRAN_L; /* Raw Status IntDstTran   (0x2d8) */
  __IO uint32_t RAW_DSTTRAN_H;
  __IO uint32_t RAW_ERR_L;     /* Raw Status for IntErr   (0x2e0) */
  __IO uint32_t RAW_ERR_H;

  /* Interrupt Status Registers */
  __IO uint32_t STATUS_TFR_L;    /* Status for IntTfr     (0x2e8) */
  __IO uint32_t STATUS_TFR_H;
  __IO uint32_t STATUS_BLOCK_L;  /* Status for IntBlock   (0x2f0) */
  __IO uint32_t STATUS_BLOCK_H;
  __IO uint32_t STATUS_SRCTRAN_L;/* Status for IntSrcTran (0x2f8) */
  __IO uint32_t STATUS_SRCTRAN_H;
  __IO uint32_t STATUS_DSTTRAN_L;/* Status for IntDstTran (0x300) */
  __IO uint32_t STATUS_DSTTRAN_H;
  __IO uint32_t STATUS_ERR_L;    /* Status for IntErr     (0x308) */
  __IO uint32_t STATUS_ERR_H;

  /* Interrupt Mask Registers */
  __IO uint32_t MSK_TFR_L;      /* Mask for IntTfr       (0x310) */
  __IO uint32_t MSK_TFR_H;
  __IO uint32_t MSK_BLOCK_L;    /* Mask for IntBlock     (0x318) */
  __IO uint32_t MSK_BLOCK_H;
  __IO uint32_t MSK_SRCTRAN_L;  /* Mask for IntSrcTran   (0x320) */
  __IO uint32_t MSK_SRCTRAN_H;
  __IO uint32_t MSK_DSTTRAN_L;  /* Mask for IntDstTran   (0x328) */
  __IO uint32_t MSK_DSTTRAN_H;
  __IO uint32_t MSK_ERR_L;      /* Mask for IntErr       (0x330) */
  __IO uint32_t MSK_ERR_H;

  /* Interrupt Clear Registers */
  __IO uint32_t CLEAR_TFR_L;     /* Clear for IntTfr      (0x338) */
  __IO uint32_t CLEAR_TFR_H;
  __IO uint32_t CLEAR_BLOCK_L;   /* Clear for IntBlock    (0x340) */
  __IO uint32_t CLEAR_BLOCK_H;
  __IO uint32_t CLEAR_SRCTRAN_L; /* Clear for IntSrcTran  (0x348) */
  __IO uint32_t CLEAR_SRCTRAN_H;
  __IO uint32_t CLEAR_DSTTRAN_L; /* Clear for IntDstTran  (0x350) */
  __IO uint32_t CLEAR_DSTTRAN_H;
  __IO uint32_t CLEAR_ERR_L;     /* Clear for IntErr      (0x358) */
  __IO uint32_t CLEAR_ERR_H;
  __IO uint32_t STATUS_INT_L;    /* Combined Intr Status  (0x360) */
  __IO uint32_t STATUS_INT_H;

  __IO uint32_t RESERVED3[12];
  
  __IO uint32_t DMA_CFG_REG_L; /* Configuration Register  (0x398) */
  __IO uint32_t DMA_CFG_REG_H;

  __IO uint32_t CH_EN_REG_L;   /* Channel Enable Register (0x3a0) */
  __IO uint32_t CH_EN_REG_H;

}DMA_T;
/*****/

#define DMAC_DMACFGREG_L_DMA_EN_Pos         (0)
#define DMAC_DMACFGREG_L_DMA_EN_Msk         (0x1ul << DMAC_DMACFGREG_L_DMA_EN_Pos)
//channel
#define DMAC_CHENREG_L_CH_EN_Pos(ch)        (ch)
#define DMAC_CHENREG_L_CH_EN_Msk(ch)        (0x1ul << DMAC_CHENREG_L_CH_EN_Pos(ch))

#define DMAC_CHENREG_L_CH_EN_ALL_Pos        (0)
#define DMAC_CHENREG_L_CH_EN_ALL_Msk        (0xfful << DMAC_CHENREG_L_CH_EN_ALL_Pos)

#define DMAC_CHENREG_L_CH_EN_WE_Pos(ch)     ((ch)+8)
#define DMAC_CHENREG_L_CH_EN_WE_Msk(ch)     (0x1ul << DMAC_CHENREG_L_CH_EN_WE_Pos(ch))

#define DMAC_CHENREG_L_CH_EN_WE_ALL_Pos     (8)
#define DMAC_CHENREG_L_CH_EN_WE_ALL_Msk     (0xfful << DMAC_CHENREG_L_CH_EN_WE_ALL_Pos)

#define DMAC_SAR_L_SAR_Pos              (0)
#define DMAC_SAR_L_SAR_Msk              (0xfffffffful << DMAC_SAR_L_SAR_Pos)

#define DMAC_DAR_L_DAR_Pos              (0)
#define DMAC_DAR_L_DAR_Msk              (0xfffffffful << DMAC_DAR_L_DAR_Pos)

#define DMAC_CTL_L_INT_EN_Pos           (0)
#define DMAC_CTL_L_INT_EN_Msk           (0x1ul << DMAC_CTL_L_INT_EN_Pos)

#define DMAC_CTL_L_DST_TR_WIDTH_Pos     (1)
#define DMAC_CTL_L_DST_TR_WIDTH_Msk     (0x3ul << DMAC_CTL_L_DST_TR_WIDTH_Pos)

#define DMAC_CTL_L_SRC_TR_WIDTH_Pos     (4)
#define DMAC_CTL_L_SRC_TR_WIDTH_Msk     (0x3ul << DMAC_CTL_L_SRC_TR_WIDTH_Pos)

#define DMAC_CTL_L_DINC_Pos             (7)
#define DMAC_CTL_L_DINC_Msk             (0x3ul << DMAC_CTL_L_DINC_Pos)

#define DMAC_CTL_L_SINC_Pos             (9)
#define DMAC_CTL_L_SINC_Msk             (0x3ul << DMAC_CTL_L_SINC_Pos)

#define DMAC_CTL_L_DEST_MSIZE_Pos       (11)
#define DMAC_CTL_L_DEST_MSIZE_Msk       (0x7ul << DMAC_CTL_L_DEST_MSIZE_Pos)

#define DMAC_CTL_L_SRC_MSIZE_Pos        (14)
#define DMAC_CTL_L_SRC_MSIZE_Msk        (0x7ul << DMAC_CTL_L_SRC_MSIZE_Pos)

#define DMAC_CTL_L_TT_FC_Pos            (20)
#define DMAC_CTL_L_TT_FC_Msk            (0x7ul << DMAC_CTL_L_TT_FC_Pos)

#define DMAC_CTL_H_BLOCK_TS_Pos         (0)
#define DMAC_CTL_H_BLOCK_TS_Msk         (0xffful << DMAC_CTL_H_BLOCK_TS_Pos)

#define DMAC_CTL_H_DONE_Pos             (12)
#define DMAC_CTL_H_DONE_Msk             (0x1ul << DMAC_CTL_H_DONE_Pos)

#define DMAC_CFG_L_CH_PRIOR_Pos         (5)
#define DMAC_CFG_L_CH_PRIOR_Msk         (0x7 << DMAC_CFG_L_CH_PRIOR_Pos)

#define DMAC_CFG_L_CH_SUSP_Pos          (8)
#define DMAC_CFG_L_CH_SUSP_Msk          (0x1ul << DMAC_CFG_L_CH_SUSP_Pos)

#define DMAC_CFG_L_FIFO_EMPTY_Pos       (9)
#define DMAC_CFG_L_FIFO_EMPTY_Msk       (0x1ul << DMAC_CFG_L_FIFO_EMPTY_Pos)

#define DMAC_CFG_L_HS_SEL_DST_Pos       (10)
#define DMAC_CFG_L_HS_SEL_DST_Msk       (0x1ul << DMAC_CFG_L_HS_SEL_DST_Pos)

#define DMAC_CFG_L_HS_SEL_SRC_Pos       (11)
#define DMAC_CFG_L_HS_SEL_SRC_Msk       (0x1ul << DMAC_CFG_L_HS_SEL_SRC_Pos)


#define DMAC_CFG_L_DST_HS_POL_Pos       (18)
#define DMAC_CFG_L_DST_HS_POL_Msk       (0x1ul << DMAC_CFG_L_DST_HS_POL_Pos)

#define DMAC_CFG_L_SRC_HS_POL_Pos       (19)
#define DMAC_CFG_L_SRC_HS_POL_Msk       (0x1ul << DMAC_CFG_L_SRC_HS_POL_Pos)



#define DMAC_CFG_H_FCMODE_Pos           (0)
#define DMAC_CFG_H_FCMODE_Msk           (0x1ul << DMAC_CFG_H_FCMODE_Pos)

#define DMAC_CFG_H_FIFO_MODE_Pos        (1)
#define DMAC_CFG_H_FIFO_MODE_Msk        (0x1ul << DMAC_CFG_H_FIFO_MODE_Pos)

#define DMAC_CFG_H_PROTCTL_Pos          (2)
#define DMAC_CFG_H_PROTCTL_Msk          (0x7ul << DMAC_CFG_H_PROTCTL_Pos)

#define DMAC_CFG_H_SRC_PER_Pos          (7)
#define DMAC_CFG_H_SRC_PER_Msk          (0xful << DMAC_CFG_H_SRC_PER_Pos)

#define DMAC_CFG_H_DEST_PER_Pos         (11)
#define DMAC_CFG_H_DEST_PER_Msk         (0xful << DMAC_CFG_H_DEST_PER_Pos)

#define DMAC_INT_RAW_STAT_CLR_Pos(ch)   (ch)
#define DMAC_INT_RAW_STAT_CLR_Msk(ch)   (0x1ul << DMAC_INT_RAW_STAT_CLR_Pos(ch))

#define DMAC_INT_RAW_STAT_CLR_ALL_Pos   (0)
#define DMAC_INT_RAW_STAT_CLR_ALL_Msk   (0xfful << DMAC_INT_RAW_STAT_CLR_ALL_Pos)

#define DMAC_INT_MASK_L_Pos(ch)         (ch)
#define DMAC_INT_MASK_L_Msk(ch)         (0x1ul << DMAC_INT_MASK_L_Pos(ch))

#define DMAC_INT_MASK_L_ALL_Pos         (0)
#define DMAC_INT_MASK_L_ALL_Msk         (0xfful << DMAC_INT_MASK_L_ALL_Pos)

#define DMAC_INT_MASK_L_WE_Pos(ch)      ((ch)+8)
#define DMAC_INT_MASK_L_WE_Msk(ch)      (0x1ul << DMAC_INT_MASK_L_WE_Pos(ch))

#define DMAC_INT_MASK_L_WE_ALL_Pos      (8)
#define DMAC_INT_MASK_L_WE_ALL_Msk      (0xfful << DMAC_INT_MASK_L_WE_ALL_Pos)

#define DMAC_STATUSINT_L_TFR_Pos        (0)
#define DMAC_STATUSINT_L_TFR_Msk        (0x1ul << DMAC_STATUSINT_L_TFR_Pos)

#define DMAC_STATUSINT_L_BLOCK_Pos      (1)
#define DMAC_STATUSINT_L_BLOCK_Msk      (0x1ul << DMAC_STATUSINT_L_BLOCK_Pos)

#define DMAC_STATUSINT_L_SRCTRAN_Pos    (2)
#define DMAC_STATUSINT_L_SRCTRAN_Msk    (0x1ul << DMAC_STATUSINT_L_SRCTRAN_Pos)

#define DMAC_STATUSINT_L_DSTTRAN_Pos    (3)
#define DMAC_STATUSINT_L_DSTTRAN_Msk    (0x1ul << DMAC_STATUSINT_L_DSTTRAN_Pos)

#define DMAC_STATUSINT_L_ERR_Pos        (4)
#define DMAC_STATUSINT_L_ERR_Msk        (0x1ul << DMAC_STATUSINT_L_ERR_Pos)
/**@}*/ /* end of dma register group */

/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
    __IO uint32_t CON               ;//(I2C_BASE + 0x000)
    __IO uint32_t TAR               ;//(I2C_BASE + 0x004)
    __IO uint32_t SAR               ;//(I2C_BASE + 0x008)
    __IO uint32_t HS_MADDR          ;//(I2C_BASE + 0x00C)
    __IO uint32_t DATACMD           ;//(I2C_BASE + 0x010)
    __IO uint32_t SS_SCL_HCNT       ;//(I2C_BASE + 0x014)
    __IO uint32_t SS_SCL_LCNT       ;//(I2C_BASE + 0x018)
    __IO uint32_t FS_SCL_HCNT       ;//(I2C_BASE + 0x01C)
    __IO uint32_t FS_SCL_LCNT       ;//(I2C_BASE + 0x020)
    __IO uint32_t HS_SCL_HCNT       ;//(I2C_BASE + 0x024)
    __IO uint32_t HS_SCL_LCNT       ;//(I2C_BASE + 0x028)
    __IO uint32_t INTR_STAT         ;//(I2C_BASE + 0x02C)
    __IO uint32_t INTR_MASK         ;//(I2C_BASE + 0x030)
    __IO uint32_t RAW_INTR_STAT     ;//(I2C_BASE + 0x034)
    __IO uint32_t RX_TL             ;//(I2C_BASE + 0x038)
    __IO uint32_t TX_TL             ;//(I2C_BASE + 0x03C)
    __IO uint32_t CLR_INTR          ;//(I2C_BASE + 0x040)
    __IO uint32_t CLR_RX_UND        ;//(I2C_BASE + 0x044)
    __IO uint32_t CLR_RX_OVR        ;//(I2C_BASE + 0x048)
    __IO uint32_t CLR_TX_OVR        ;//(I2C_BASE + 0x04C)
    __IO uint32_t CLR_RD_REQ        ;//(I2C_BASE + 0x050)
    __IO uint32_t CLR_TX_ABRT       ;//(I2C_BASE + 0x054)
    __IO uint32_t CLR_RX_DONE       ;//(I2C_BASE + 0x058)
    __IO uint32_t CLR_ACTIVITY      ;//(I2C_BASE + 0x05C)
    __IO uint32_t CLR_STOP_DET      ;//(I2C_BASE + 0x060)
    __IO uint32_t CLR_START_DET     ;//(I2C_BASE + 0x064)
    __IO uint32_t CLR_GEN_CALL      ;//(I2C_BASE + 0x068)
    __IO uint32_t IC_ENABLE            ;//(I2C_BASE + 0x06C)
    __IO uint32_t STATUS            ;//(I2C_BASE + 0x070)
    __IO uint32_t TXFLR             ;//(I2C_BASE + 0x074)
    __IO uint32_t RXFLR             ;//(I2C_BASE + 0x078)
    __IO uint32_t SDA_HOLD          ;//(I2C_BASE + 0x07C)
    __IO uint32_t TX_ABRT_SRC       ;//(I2C_BASE + 0x080)
    __IO uint32_t SLV_DATA_NACK     ;//(I2C_BASE + 0x084)
    __IO uint32_t DMA_CR            ;//(I2C_BASE + 0x088)
    __IO uint32_t DMA_TDLR          ;//(I2C_BASE + 0x08C)
    __IO uint32_t DMA_RDLR          ;//(I2C_BASE + 0x090)
    __IO uint32_t SDA_SETUP         ;//(I2C_BASE + 0x094)
    __IO uint32_t ACK_GENERAL_CALL  ;//(I2C_BASE + 0x098)
    __IO uint32_t ENABLE_STATUS     ;//(I2C_BASE + 0x09C)
    __IO uint32_t FS_SPKLEN         ;//(I2C_BASE + 0x0A0)
    __IO uint32_t HS_SPKLEN         ;//(I2C_BASE + 0x0A4)
	__IO uint32_t CLR_RESTART_DET         ;//(I2C_BASE + 0x0A8)

}I2C_T;

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CON register  ********************/
#define  I2C_CON_MASTER_MODE                            ((uint16_t)0x0001)            /*!<Master mode                             */

#define  I2C_CON_SPEED                       		    ((uint16_t)0x0006)            /*!<Speed                                   */
#define  I2C_CON_SPEED_0								((uint16_t)0x0002)
#define  I2C_CON_SPEED_1								((uint16_t)0x0004)

#define  I2C_CON_IC_10BITADDR_SLAVE                     ((uint16_t)0x0008)
#define  I2C_CON_IC_10BITADDR_MASTER                    ((uint16_t)0x0010)
#define  I2C_CON_IC_RESTART_EN                          ((uint16_t)0x0020)
#define  I2C_CON_IC_SLAVE_DISABLE                       ((uint16_t)0x0040)
#define  I2C_CON_STOP_DET_IFADDRESSED                   ((uint16_t)0x0080)
#define  I2C_CON_TX_EMPTY_CTRL                          ((uint16_t)0x0100)

/*******************  Bit definition for I2C_TAR register  ********************/
#define I2C_TAR_TAR                                    ((uint16_t)0x03ff)
#define I2C_TAR_IC_TAR_0                                  ((uint16_t)0x0001)
#define I2C_TAR_IC_TAR_1                                  ((uint16_t)0x0002)
#define I2C_TAR_IC_TAR_2                                  ((uint16_t)0x0004)
#define I2C_TAR_IC_TAR_3                                  ((uint16_t)0x0008)
#define I2C_TAR_IC_TAR_4                                  ((uint16_t)0x0010)
#define I2C_TAR_IC_TAR_5                                  ((uint16_t)0x0020)
#define I2C_TAR_IC_TAR_6                                  ((uint16_t)0x0040)
#define I2C_TAR_IC_TAR_7                                  ((uint16_t)0x0080)
#define I2C_TAR_IC_TAR_8                                  ((uint16_t)0x0100)
#define I2C_TAR_IC_TAR_9                                  ((uint16_t)0x0200)

#define I2C_TAR_GC_OR_START                               ((uint16_t)0x0400)
#define I2C_TAR_SPECIAL                                   ((uint16_t)0x0800)
#define I2C_TAR_IC_10BITADDR_MASTER                       ((uint16_t)0x1000)

/*******************  Bit definition for I2C_SAR register  ********************/
#define I2C_SAR_IC_SAR                                    ((uint16_t)0x03ff)
/*******************  Bit definition for I2C_HS_MADDR register  ********************/
#define I2C_HS_MADDR_IC_HS_MAR                            ((uint16_t)0x0007)
/*******************  Bit definition for I2C_DATA_CMD register  ********************/
#define I2C_DATA_CMD_DAT                                  ((uint16_t)0x00ff)
#define I2C_DATA_CMD_CMD_R                                  ((uint16_t)0x0100)
#define I2C_DATA_CMD_STOP                                 ((uint16_t)0x0200)
#define I2C_DATA_CMD_RESTART                              ((uint16_t)0x0400)

/*******************  Bit definition for I2C_SS_SCL_HCNT register  ********************/
#define I2C_SS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_SS_SCL_LCNT register  ********************/
#define I2C_SS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_FS_SCL_HCNT register  ********************/
#define I2C_FS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_FS_SCL_LCNT register  ********************/
#define I2C_FS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_HS_SCL_HCNT register  ********************/
#define I2C_HS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_HS_SCL_LCNT register  ********************/
#define I2C_HS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_INTR_STAT register  ********************/
#define I2C_INTR_STAT_R_RX_UNDER                                  ((uint16_t)0x0001)
#define I2C_INTR_STAT_R_RX_OVER                                   ((uint16_t)0x0002)
#define I2C_INTR_STAT_R_RX_FULL                                   ((uint16_t)0x0004)
#define I2C_INTR_STAT_R_TX_OVER                                   ((uint16_t)0x0008)
#define I2C_INTR_STAT_R_TX_EMPTY                                  ((uint16_t)0x0010)
#define I2C_INTR_STAT_R_RD_REQ                                    ((uint16_t)0x0020)
#define I2C_INTR_STAT_R_TX_ABRT                                   ((uint16_t)0x0040)
#define I2C_INTR_STAT_R_RX_DONE                                   ((uint16_t)0x0080)
#define I2C_INTR_STAT_R_ACTIVITY                                  ((uint16_t)0x0100)
#define I2C_INTR_STAT_R_STOP_DET                                  ((uint16_t)0x0200)
#define I2C_INTR_STAT_R_START_DET                                 ((uint16_t)0x0400)
#define I2C_INTR_STAT_R_GEN_CALL                                  ((uint16_t)0x0800)
#define I2C_INTR_STAT_R_RESTART_DET                               ((uint16_t)0x1000)
#define I2C_INTR_STAT_R_MST_ON_HOLD                               ((uint16_t)0x2000)


/*******************  Bit definition for I2C_INTR_MASK register  ********************/
#define I2C_INTR_MASK_M_RX_UNDER                                  ((uint16_t)0x0001)
#define I2C_INTR_MASK_M_RX_OVER                                   ((uint16_t)0x0002)
#define I2C_INTR_MASK_M_RX_FULL                                   ((uint16_t)0x0004)
#define I2C_INTR_MASK_M_TX_OVER                                   ((uint16_t)0x0008)
#define I2C_INTR_MASK_M_TX_EMPTY                                  ((uint16_t)0x0010)
#define I2C_INTR_MASK_M_RD_REQ                                    ((uint16_t)0x0020)
#define I2C_INTR_MASK_M_TX_ABRT                                   ((uint16_t)0x0040)
#define I2C_INTR_MASK_M_RX_DONE                                   ((uint16_t)0x0080)
#define I2C_INTR_MASK_M_ACTIVITY                                  ((uint16_t)0x0100)
#define I2C_INTR_MASK_M_STOP_DET                                  ((uint16_t)0x0200)
#define I2C_INTR_MASK_M_START_DET                                 ((uint16_t)0x0400)
#define I2C_INTR_MASK_M_GEN_CALL                                  ((uint16_t)0x0800)
#define I2C_INTR_MASK_M_RESTART_DET                               ((uint16_t)0x1000)
#define I2C_INTR_MASK_M_MST_ON_HOLD                               ((uint16_t)0x2000)

/*******************  Bit definition for I2C_RAW_INTR_STAT register  ********************/
#define I2C_RAW_INTR_STAT_RX_UNDER                              ((uint16_t)0x0001)
#define I2C_RAW_INTR_STAT_RX_OVER                               ((uint16_t)0x0002)
#define I2C_RAW_INTR_STAT_RX_FULL                               ((uint16_t)0x0004)
#define I2C_RAW_INTR_STAT_TX_OVER                               ((uint16_t)0x0008)
#define I2C_RAW_INTR_STAT_TX_EMPTY                              ((uint16_t)0x0010)
#define I2C_RAW_INTR_STAT_RD_REQ                                ((uint16_t)0x0020)
#define I2C_RAW_INTR_STAT_TX_ABRT                               ((uint16_t)0x0040)
#define I2C_RAW_INTR_STAT_RX_DONE                               ((uint16_t)0x0080)
#define I2C_RAW_INTR_STAT_ACTIVITY                              ((uint16_t)0x0100)
#define I2C_RAW_INTR_STAT_STOP_DET                              ((uint16_t)0x0200)
#define I2C_RAW_INTR_STAT_START_DET                             ((uint16_t)0x0400)
#define I2C_RAW_INTR_STAT_GEN_CALL                              ((uint16_t)0x0800)
#define I2C_RAW_INTR_STAT_RESTART_DET                           ((uint16_t)0x1000)
#define I2C_RAW_INTR_STAT_MST_ON_HOLD                           ((uint16_t)0x2000)

/*******************  Bit definition for I2C_RX_TL register  ********************/
#define I2C_RX_TL                                                 ((uint16_t)0x00ff)

/*******************  Bit definition for I2C_TX_TL register  ********************/
#define I2C_TX_TL                                                 ((uint16_t)0x00ff)

/*******************  Bit definition for I2C_CLR_INTR register  ********************/
#define I2C_CLR_INTR                                              ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_UNDER register  ********************/
#define I2C_CLR_RX_UNDER                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_OVER register  ********************/
#define I2C_CLR_RX_UNDER                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_TX_OVER register  ********************/
#define I2C_CLR_TX_OVER                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RD_REQ register  ********************/
#define I2C_CLR_RD_REQ                                            ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_TX_ABORT register  ********************/
#define I2C_CLR_TX_ABORT                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_DONE register  ********************/
#define I2C_CLR_RX_DONE                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_ACTIVITY register  ********************/
#define I2C_CLR_ACTIVITY                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_STOP_DET register  ********************/
#define I2C_CLR_STOP_DET                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_START_DET register  ********************/
#define I2C_CLR_START_DET                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_GEN_CALL register  ********************/
#define I2C_CLR_GEN_CALL                                           ((uint16_t)0x0001)
/*******************  Bit definition for I2C_CLR_RESTART_DET register  ********************/
#define I2C_CLR_RESTART_DET                                        ((uint16_t)0x0001)
/*******************  Bit definition for I2C_CLR_MST_ON_HOLD register  ********************/
#define I2C_CLR_MST_ON_HOLD                                        ((uint16_t)0x0001)


/*******************  Bit definition for I2C_ENABLE register  ********************/
#define I2C_ENABLE_ENABLE                                          ((uint16_t)0x0001)
#define I2C_ENABLE_ABORT                                           ((uint16_t)0x0002)
/*******************  Bit definition for I2C_STATUS register  ********************/
#define I2C_STATUS_ACTIVITY                                        ((uint16_t)0x0001)
#define I2C_STATUS_TFNF                                            ((uint16_t)0x0002)
#define I2C_STATUS_TFE                                             ((uint16_t)0x0004)
#define I2C_STATUS_RFNE                                            ((uint16_t)0x0008)
#define I2C_STATUS_RFE                                             ((uint16_t)0x0010)
#define I2C_STATUS_MST_ACTIVITY                                    ((uint16_t)0x0020)
#define I2C_STATUS_SLV_ACTIVITY                                    ((uint16_t)0x0040)

/*******************  Bit definition for I2C_TXFLR register  ********************/
#define I2C_TXFLR                                                  ((uint32_t)0x0000000f)

/*******************  Bit definition for I2C_RXFLR register  ********************/
#define I2C_RXFLR                                                  ((uint32_t)0x0000000f)

/*******************  Bit definition for I2C_SDA_HOLD register  ********************/
#define I2C_SDA_HOLD                                               ((uint32_t)0x0000ffff)

/*******************  Bit definition for I2C_TX_ABRT_SOURCE register  ********************/
#define I2C_TX_ABRT_SOURCE_7B_ADDR_NOACK                       ((uint32_t)0x00000001)
#define I2C_TX_ABRT_SOURCE_10ADDR1_NOACK                      ((uint32_t)0x00000002)
#define I2C_TX_ABRT_SOURCE_10ADDR2_NOACK                      ((uint32_t)0x00000004)
#define I2C_TX_ABRT_SOURCE_TXDATA_NOACK                       ((uint32_t)0x00000008)
#define I2C_TX_ABRT_SOURCE_GCALL_NOACK                        ((uint32_t)0x00000010)
#define I2C_TX_ABRT_SOURCE_GCALL_READ                         ((uint32_t)0x00000020)
#define I2C_TX_ABRT_SOURCE_HS_ACKDET                          ((uint32_t)0x00000040)
#define I2C_TX_ABRT_SOURCE_SBYTE_ACKDET                       ((uint32_t)0x00000080)
#define I2C_TX_ABRT_SOURCE_HS_NORSTRT                         ((uint32_t)0x00000100)
#define I2C_TX_ABRT_SOURCE_SBYTE_NORSTRT                      ((uint32_t)0x00000200)
#define I2C_TX_ABRT_SOURCE_10B_RD_NORSTRT                     ((uint32_t)0x00000400)
#define I2C_TX_ABRT_SOURCE_MASTER_DIS                         ((uint32_t)0x00000800)
#define I2C_TX_ABRT_SOURCE_ARB_LOST                           ((uint32_t)0x00001000)
#define I2C_TX_ABRT_SOURCE_SLVFLUSH_TXFIFO                    ((uint32_t)0x00002000)
#define I2C_TX_ABRT_SOURCE_SLV_ARBLOST                        ((uint32_t)0x00004000)
#define I2C_TX_ABRT_SOURCE_SLVRD_INTX                         ((uint32_t)0x00008000)
#define I2C_TX_ABRT_SOURCE_USER_ABRT                          ((uint32_t)0x00010000)

#define I2C_TX_ABRT_SOURCE_TX_FLUSH_CNT                            ((uint32_t)0xff000000)

/*******************  Bit definition for I2C_SLV_DATA_NACK_ONLY register  ********************/
#define I2C_SLV_DATA_NACK_ONLY                                     ((uint32_t)0x00000001)

/*******************  Bit definition for I2C_DMA_CR register  ********************/
#define I2C_DMA_CR_RDMAE                                           ((uint32_t)0x00000001)
#define I2C_DMA_CR_TDMAE                                           ((uint32_t)0x00000002)

/*******************  Bit definition for I2C_DMA_TDLR register  ********************/
#define I2C_DMA_TDLR                                               ((uint32_t)0x00000007)

/*******************  Bit definition for I2C_DMA_RDLR register  ********************/
#define I2C_DMA_RDLR                                               ((uint32_t)0x00000007)

/*******************  Bit definition for I2C_SDA_SETUP register  ********************/
#define I2C_SDA_SETUP                                              ((uint32_t)0x000000ff)

/*******************  Bit definition for I2C_ACK_GENERAL_CALL register  ********************/
#define I2C_ACK_GENERAL_CALL                                       ((uint32_t)0x00000001)

/*******************  Bit definition for I2C_ENABLE_STATUS register  ********************/
#define I2C_ENABLE_STATUS_IC_EN                                    ((uint32_t)0x00000001)
#define I2C_ENABLE_STATUS_SLV_RX_ABORTED                           ((uint32_t)0x00000002)
#define I2C_ENABLE_STATUS_SLV_FIFO_FILLED_AND_FLUSHED              ((uint32_t)0x00000004)

/*******************  Bit definition for I2C_FS_SPKLEN register  ********************/
#define I2C_FS_SPKLEN                                              ((uint32_t)0x0000000ff)

/*******************  Bit definition for I2C_HS_SPKLEN register  ********************/
#define I2C_HS_SPKLEN                                              ((uint32_t)0x0000000ff)

/*******************  Bit definition for I2C_COMP_PARAM1 register  ********************/
#define I2C_COMP_PARAM1_APB_DATA_WIDTH                             ((uint32_t)0x000000003)
#define I2C_COMP_PARAM1_MAX_SPEED_MODE                             ((uint32_t)0x000000006)
#define I2C_COMP_PARAM1_HC_COUNT_VALUES                            ((uint32_t)0x000000010)
#define I2C_COMP_PARAM1_INTR_IO                                    ((uint32_t)0x000000020)
#define I2C_COMP_PARAM1_HAS_DMA                                    ((uint32_t)0x000000040)
#define I2C_COMP_PARAM1_ADD_ENCODED_PARAMS                         ((uint32_t)0x000000080)
#define I2C_COMP_PARAM1_RX_BUFFER_DEPTH                            ((uint32_t)0x00000ff00)
#define I2C_COMP_PARAM1_TX_BUFFER_DEPTH                            ((uint32_t)0x000ff0000)

/*******************  Bit definition for I2C_COMP_VERSION register  ********************/
#define I2C_COMP_VERSION                                           ((uint32_t)0xffffffffff)

/*******************  Bit definition for I2C_COMP_TYPE register  ********************/
#define I2C_COMP_TYPE                                              ((uint32_t)0xffffffffff)

/**
  * @}
  */

/*---------------------- I2S Bus Controller -------------------------*/
/**
    @addtogroup I2S Inter-IC Sound (I2S)
    Memory Mapped Structure for I2S Controller
@{ */

/**
  * @brief Inter-IC Sound Interface
  */
typedef struct
{
    __IO uint32_t IER                       ; //0x00        I2S enable Register
    __IO uint32_t IRER                      ; //0x04        Receiver block Register
    __IO uint32_t ITER                      ; //0x08        Transmitter block Register
    __IO uint32_t CER                       ; //0x0c        Clock enable Register
    __IO uint32_t CCR                       ; //0x10        Clock configuration Register
    __IO uint32_t RXFFR                     ; //0x14        Receiver block FIFO reset Resister
    __IO uint32_t TXFFR                     ; //0x18        Transmitter block FIFO reset Register

    __IO uint32_t RSV0                      ; //0x1c        reserve

    struct {
        __IO uint32_t LTHR_LRBR             ; //0x20        Left receiver buffer; Left transmit holding Register
        __IO uint32_t RTHR_RRBR             ; //0x24        Right receiver buffer; Right transmit holding Register
        __IO uint32_t RER                   ; //0x28        Receiver channel Register
        __IO uint32_t TER                   ; //0x2c        Transmitter channel Register
        __IO uint32_t RCR                   ; //0x30        Receive configuration Register
        __IO uint32_t TCR                   ; //0x34        Transmit configuration Register
        __IO uint32_t ISR                   ; //0x38        Interrupt status Register
        __IO uint32_t IMR                   ; //0x3c        Interrupt mask Register
        __IO uint32_t ROR                   ; //0x40        Receive overrun Register
        __IO uint32_t TOR                   ; //0x44        Transmit overrun Register
        __IO uint32_t RFCR                  ; //0x48        Receive FIFO configuration
        __IO uint32_t TFCR                  ; //0x4c        Transmit FIFO configuration
        __IO uint32_t RFF                   ; //0x50        Receive FIFO flush
        __IO uint32_t TFF                   ; //0x54        Transmit FIFO flush

        __IO uint32_t RSV1[2]               ; //0x58, 0x5C        reserve
    } CH[4];        //14 regs, 2 rsvs

    __IO uint32_t RSV2[40]                  ; //0x120-0x1BC,  reserve

    __IO uint32_t RXDMA                     ; //0x1C0    Receive block register
    __IO uint32_t RRXDMA    /*RSV3*/        ; //0x1C4    Reset receive block register
    __IO uint32_t TXDMA                     ; //0x1C8    Transmit block register
    __IO uint32_t RTXDMA    /*RSV4*/        ; //0x1CC    Reset Transmit block register

  __IO uint32_t RSV5[8]                     ; //0x1D0, 1D4, 1D8, 1DC, 0x1E0, 1E4, 1E8, 1EC, reserve

    __IO uint32_t I2S_COMP_PARAM_2          ;/*0x1F0  Receive word size configuration
                                                [12:10] I2S_RX_WORDSIZE_3, [9:7] I2S_RX_WORDSIZE_2,
                                                [5:3]   I2S_RX_WORDSIZE_1, [2:0] I2S_RX_WORDSIZE_0
                                                0x0 = 12 bit resolution,   0x1 = 16 bit resolution
                                                0x2 = 20 bit resolution,   0x3 = 24 bit resolution
                                                0x4 = 32 bit resolution,   0x5 - 0x7 = reserved        */
    __IO uint32_t I2S_COMP_PARAM_1          ;/*0x1F4    I2S_TX_WORDSIZE_x,      I2S_TX_CHANNELS,        I2S_RX_CHANNELS
                                                I2S_RECEIVER_BLOCK,     I2S_TRANSMITTER_BLOCK,  I2S_MODE_EN
                                                I2S_FIFO_DEPTH_GLOBAL,  APB_DATA_WIDTH       */
    __IO uint32_t I2S_COMP_VERSION          ;//0x1F8    I2S Component Version Register
    __IO uint32_t I2S_COMP_TYPE             ;//0x1FC    I2S Component Type Register
}I2S_T;
/**@}*/ /* end of i2s register group */


/**
    @addtogroup KSCAN Bus Controller
    Memory Mapped Structure for KSCAN Controller
@{ */

/**
  * @brief Key scan Interface
  */
typedef struct
{
    __IO uint32_t KS_EN                     ; //0x00
    __IO uint32_t KS_IO_CFG                 ; //0x04
    __IO uint32_t KS_INT_CFG                ; //0X08
    __IO uint32_t KS_CFG                    ; //0X0C
    __I  uint32_t KS_INFO0                  ; //0X10
    __I  uint32_t KS_INFO1                  ; //0X14
    __I  uint32_t KS_INFO2                  ; //0X18
    __I  uint32_t KS_INFO3                  ; //0X1C
    __I  uint32_t KS_INFO4                  ; //0X20
    __I  uint32_t KS_INFO5                  ; //0X24
}KSCAN_T;

//KS_EN	
#define KS_DLY_DISABLE_Pos                  (2)
#define KS_DLY_DISABLE_Msk                  (0x1ul << KS_DLY_DISABLE_Pos)
#define KS_WK_ENABLE_Pos                    (1)
#define KS_WK_ENABLE_Msk                    (0x1ul << KS_WK_ENABLE_Pos)
#define KS_ENABLE_Pos                       (0)
#define KS_ENABLE_Msk                       (0x1ul << KS_ENABLE_Pos)
//KS_IO_CFG	
#define KS_OUTPUT_ENABLE_Pos                (8)
#define KS_OUTPUT_ENABLE_Msk                (0x3FFul << KS_OUTPUT_ENABLE_Pos)
#define KS_INPUT_ENABLE_Pos                 (0)
#define KS_INPUT_ENABLE_Msk                 (0x1ul << KS_INPUT_ENABLE_Pos)
//KS_INT_CFG  
#define KS_WK_FLAG_Pos                      (3)
#define KS_WK_FLAG_Msk                      (0x1ul << KS_WK_FLAG_Pos)
#define KS_INT_FLAG_Pos                     (2)
#define KS_INT_FLAG_Msk                     (0x1ul << KS_INT_FLAG_Pos)
#define KS_FLAG_Pos                         (1)
#define KS_FLAG_Msk                         (0x1ul << KS_FLAG_Pos)
#define KS_INT_ENABLE_Pos                   (0)
#define KS_INT_ENABLE_Msk                   (0x1ul << KS_INT_ENABLE_Pos)
//KS_CFG        
#define KS_POLARITY_VALID_HIGH_Pos          (13)
#define KS_POLARITY_VALID_HIGH_Msk          (0x1ul << KS_POLARITY_VALID_HIGH_Pos)
#define KS_STATUS_CLEAR_Pos                 (12)
#define KS_STATUS_CLEAR_Msk                 (0x1ul << KS_STATUS_CLEAR_Pos)
#define KS_SCAN_INTERVAL_Pos                (8)
#define KS_SCAN_INTERVAL_Msk                (0xFul << KS_SCAN_INTERVAL_Pos)
#define KS_ROW_INTERVAL_Pos                 (4)
#define KS_ROW_INTERVAL_Msk                 (0xFul << KS_ROW_INTERVAL_Pos)
#define KS_DEBOUNCE_TIME_Pos                (0)
#define KS_DEBOUNCE_TIME_Msk                (0xFul << KS_DEBOUNCE_TIME_Pos)
/**@}*/ /* end of KSCAN register group */


/**
    @addtogroup QDEC Controller
    Memory Mapped Structure for QDEC Controller
@{ */

/**
  * @brief Qdec Interface
  */
typedef struct
{
    __IO uint32_t QDEC_EN                   ; //0X00
    __IO uint32_t QDEC_INT_FLAG             ; //0X04
    __IO uint32_t QDEC_INT_RAW              ; //0X08
    __IO uint32_t QDEC_CTL                  ; //0X0C
    __IO uint32_t QDEC_X_CNT                ; //0X10
    __IO uint32_t QDEC_Y_CNT                ; //0X14
    __IO uint32_t QDEC_Z_CNT                ; //0X18
}QDEC_T;

//QDEC_EN
#define QDEC_EVENT_EN_Pos                   (7)
#define QDEC_EVENT_EN_Msk                   (0x1ul << QDEC_EVENT_EN_Pos)
#define QDEC_WAKEUP_EN_Pos                  (6)
#define QDEC_WAKEUP_EN_Msk                  (0x1ul << QDEC_WAKEUP_EN_Pos)
#define QDEC_INDEX_EN_Pos                   (5)
#define QDEC_INDEX_EN_Msk                   (0x1ul << QDEC_INDEX_EN_Pos)
#define QDEC_CHANNEL_Z_EN_Pos               (4)
#define QDEC_CHANNEL_Z_EN_Msk               (0x1ul << QDEC_CHANNEL_Z_EN_Pos)
#define QDEC_CHANNEL_Y_EN_Pos               (3)
#define QDEC_CHANNEL_Y_EN_Msk               (0x1ul << QDEC_CHANNEL_Y_EN_Pos)
#define QDEC_CHANNEL_X_EN_Pos               (2)
#define QDEC_CHANNEL_X_EN_Msk               (0x1ul << QDEC_CHANNEL_X_EN_Pos)
#define QDEC_FILTER_EN_Pos                  (1)
#define QDEC_FILTER_EN_Msk                  (0x1ul << QDEC_FILTER_EN_Pos)
#define QDEC_EN_Pos                         (0)
#define QDEC_EN_Msk                         (0x1ul << QDEC_EN_Pos)
//QDEC_INT_FLAG
//QDEC_INT_RAW
#define QDEC_INT_WAKEUP_Pos                 (6)
#define QDEC_INT_WAKEUP_Msk                 (0x1ul << QDEC_INT_WAKEUP_Pos)
#define QDEC_INT_INDEX_Pos                  (5)
#define QDEC_INT_INDEX_Msk                  (0x1ul << QDEC_INT_INDEX_Pos)
#define QDEC_INT_FIFO_UNDERFLOW_Pos         (4)
#define QDEC_INT_FIFO_UNDERFLOW_Msk         (0x1ul << QDEC_INT_FIFO_UNDERFLOW_Pos)
#define QDEC_INT_FIFO_OVERFLOW_Pos          (3)
#define QDEC_INT_FIFO_OVERFLOW_Msk          (0x1ul << QDEC_INT_FIFO_OVERFLOW_Pos)
#define QDEC_INT_INVALID_Pos                (2)
#define QDEC_INT_INVALID_Msk                (0x1ul << QDEC_INT_INVALID_Pos)
#define QDEC_INT_CNT_UNDERFLOW_Pos          (1)
#define QDEC_INT_CNT_UNDERFLOW_Msk          (0x1ul << QDEC_INT_CNT_UNDERFLOW_Pos)
#define QDEC_INT_CNT_OVERFLOW_Pos           (0)
#define QDEC_INT_CNT_OVERFLOW_Msk           (0x1ul << QDEC_INT_CNT_OVERFLOW_Pos)
//QDEC_CTL     
#define QDEC_EVENT_THRESHOLD_Pos            (16)
#define QDEC_EVENT_THRESHOLD_Msk            (0xFFul << QDEC_EVENT_THRESHOLD_Pos)
#define QDEC_INDEX_POLARITY_Pos             (12)
#define QDEC_INDEX_POLARITY_Msk             (0x1ul << QDEC_INDEX_POLARITY_Pos)
#define QDEC_FILTER_THRESHOLD_Pos           (9)
#define QDEC_FILTER_THRESHOLD_Msk           (0x7ul << QDEC_FILTER_THRESHOLD_Pos)
#define QDEC_CNT_RESOLUTION_Pos             (7)
#define QDEC_CNT_RESOLUTION_Msk             (0x3ul << QDEC_CNT_RESOLUTION_Pos)
//QDEC_X_CNT   
//QDEC_Y_CNT   
//QDEC_Z_CNT   
#define QDEC_CNT_CLEAR_Pos                  (16)
#define QDEC_CNT_CLEAR_Msk                  (0x1ul << QDEC_CNT_CLEAR_Pos)
#define QDEC_CNT_VALUE_Pos                  (0)
#define QDEC_CNT_VALUE_Msk                  (0xfffful << QDEC_CNT_VALUE_Pos)
/**@}*/ /* end of qdec register group */

/**
    @addtogroup ANALOG Controller
    Memory Mapped Structure for ANALOG Controller
@{ */

/**
  * @brief Analog Interface
  */
typedef struct{
		__IO uint32_t LP_REG_SYNC;      //(0x00)
		__IO uint32_t LP_FL_CTRL;       //(0x04)
		__IO uint32_t LP_TMR_CTRL;      //(0x08)
		__IO uint32_t LP_INT_CTRL;      //(0x0C)
		__IO uint32_t LP_RST_CTRL;      //(0x10)
		__IO uint32_t LP_WDT_CNT;       //(0x14)
        __IO uint32_t LP_WDT_CTRL;      //(0x18)
		__IO uint32_t LP_PTAT_POLY;     //(0x1C)
		__IO uint32_t LP_HP_LDO;      	//(0x20)
		__IO uint32_t LP_LP_LDO;      	//(0x24)
		__IO uint32_t LP_ANA_LDO;     	//(0x28)
		__IO uint32_t LP_FSYN_LDO;      //(0x2C)
		__IO uint32_t LP_SW;      		//(0x30)
		__IO uint32_t LP_BUCK;      	//(0x34)
		__IO uint32_t LP_MISC;      	//(0x38)
		__IO uint32_t RSV0;      		//(0x3C)
		__IO uint32_t ANA_ADC_LDO;      //(0x40)
		__IO uint32_t ANA_RFFE_LDO;     //(0x44)
		__IO uint32_t ANA_VCO_LDO;      //(0x48)
		__IO uint32_t ANA_DFT;          //(0x4C)
		__IO uint32_t ANA_TEMP;     	//(0x50)
		__IO uint32_t ANA_MISC1;        //(0x54)
		__IO uint32_t ANA_MISC2;        //(0x58)
		__IO uint32_t RSV1;		        //(0x5C)
}ANA_T;

/*LP_REG_SYNC*/
#define ANAC_LP_REG_SYNC_3V_Pos             (0)
#define ANAC_LP_REG_SYNC_3V_Msk             (0x1ul << ANAC_LP_REG_SYNC_3V_Pos)
#define ANAC_LP_REG_SYNC_STS_Pos            (1)
#define ANAC_LP_REG_SYNC_STS_Msk            (0x1ul << ANAC_LP_REG_SYNC_STS_Pos)
#define ANAC_LP_REG_AUTOSYNC_Pos            (2)
#define ANAC_LP_REG_AUTOSYNC_Msk            (0x1ul << ANAC_LP_REG_AUTOSYNC_Pos)
/*LP_FL_CTRL*/
#define ANAC_FL_EXT_WAKEUP_EN_Pos           (0)
#define ANAC_FL_EXT_WAKEUP_EN_Msk           (0x1ul << ANAC_FL_EXT_WAKEUP_EN_Pos)
#define ANAC_FL_EXT_WAKEUP_SEL_Pos          (1)            
#define ANAC_FL_EXT_WAKEUP_SEL_Msk          (0x1ul << ANAC_FL_EXT_WAKEUP_SEL_Pos)
#define ANAC_FL_SLEEP_EN_Pos                (2)            
#define ANAC_FL_SLEEP_EN_Msk                (0x1ul << ANAC_FL_SLEEP_EN_Pos)
#define ANAC_FL_SLEEP_CNT_EN_Pos            (3)            
#define ANAC_FL_SLEEP_CNT_EN_Msk            (0x1ul << ANAC_FL_SLEEP_CNT_EN_Pos)
#define ANAC_FL_STANDBY_EN_Pos              (4)            
#define ANAC_FL_STANDBY_EN_Msk              (0x1ul << ANAC_FL_STANDBY_EN_Pos)
//#define ANAC_FL_LDO_SEL_Pos                 (5)            
//#define ANAC_FL_LDO_SEL_Msk                 (0x1ul << ANAC_FL_LDO_SEL_Pos)
#define ANAC_FL_RCL_GATE_EN_Pos             (6)            
#define ANAC_FL_RCL_GATE_EN_Msk             (0x1ul << ANAC_FL_RCL_GATE_EN_Pos)
#define ANAC_FL_PWR_CTL_EN_Pos              (7)            
#define ANAC_FL_PWR_CTL_EN_Msk              (0x1ul << ANAC_FL_PWR_CTL_EN_Pos)
#define ANAC_FL_DEEPSLEEP_REQUEST_Pos       (8)            
#define ANAC_FL_DEEPSLEEP_REQUEST_Msk       (0x1ul << ANAC_FL_DEEPSLEEP_REQUEST_Pos)
#define ANAC_FL_SLEEP_REQUEST_Pos           (9)            
#define ANAC_FL_SLEEP_REQUEST_Msk           (0x1ul << ANAC_FL_SLEEP_REQUEST_Pos)
#define ANAC_FL_LEVEL_SHIFT_EN_Pos          (10)            
#define ANAC_FL_LEVEL_SHIFT_EN_Msk          (0x1ul << ANAC_FL_LEVEL_SHIFT_EN_Pos)

#define ANAC_FL_FLASH_EN_Pos                (15)           
#define ANAC_FL_FLASH_EN_Msk                (0x1ul << ANAC_FL_FLASH_EN_Pos)
#define ANAC_FL_DPLL_EN_Pos                 (16)           
#define ANAC_FL_DPLL_EN_Msk                 (0x1ul << ANAC_FL_DPLL_EN_Pos)
#define ANAC_FL_OSCH_EN_Pos                 (17)           
#define ANAC_FL_OSCH_EN_Msk                 (0x1ul << ANAC_FL_OSCH_EN_Pos)
#define ANAC_FL_XTAL_EN_Pos                 (18)           
#define ANAC_FL_XTAL_EN_Msk                 (0x1ul << ANAC_FL_XTAL_EN_Pos)
#define ANAC_FL_XTAL32K_EN_Pos              (19)           
#define ANAC_FL_XTAL32K_EN_Msk              (0x1ul << ANAC_FL_XTAL32K_EN_Pos)
#define ANAC_FL_RC32K_EN_Pos                (20)           
#define ANAC_FL_RC32K_EN_Msk                (0x1ul << ANAC_FL_RC32K_EN_Pos)
#define ANAC_FL_LDO_LP_EN_Pos               (21)           
#define ANAC_FL_LDO_LP_EN_Msk               (0x1ul << ANAC_FL_LDO_LP_EN_Pos)
#define ANAC_FL_LDO_HP_EN_Pos               (22)           
#define ANAC_FL_LDO_HP_EN_Msk               (0x1ul << ANAC_FL_LDO_HP_EN_Pos)
#define ANAC_FL_PMU_EN_Pos                  (23)           
#define ANAC_FL_PMU_EN_Msk                  (0x1ul << ANAC_FL_PMU_EN_Pos)
#define ANAC_SRAM0_POWER_CTL_Pos            (24)           
#define ANAC_SRAM0_POWER_CTL_Msk            (0x1ul << ANAC_SRAM0_POWER_CTL_Pos)
#define ANAC_SRAM1_POWER_CTL_Pos            (25)           
#define ANAC_SRAM1_POWER_CTL_Msk            (0x1ul << ANAC_SRAM1_POWER_CTL_Pos)
#define ANAC_SRAM2_POWER_CTL_Pos            (26)           
#define ANAC_SRAM2_POWER_CTL_Msk            (0x1ul << ANAC_SRAM2_POWER_CTL_Pos)
#define ANAC_SRAM3_POWER_CTL_Pos            (27)           
#define ANAC_SRAM3_POWER_CTL_Msk            (0x1ul << ANAC_SRAM3_POWER_CTL_Pos)
#define ANAC_LL_SRAM_POWER_CTL_Pos         	(28)           
#define ANAC_LL_SRAM_POWER_CTL_Msk         	(0x1ul << ANAC_LL_SRAM_POWER_CTL_Pos)
#define ANAC_DECRYPT_SRAM_CTL_Pos         	(29)           
#define ANAC_DECRYPT_SRAM_CTL_Msk         	(0x1ul << ANAC_DECRYPT_SRAM_CTL_Pos)
#define ANAC_LDO_POWER_CTL_Pos         		(30)           
#define ANAC_LDO_POWER_CTL_Msk         		(0x1ul << ANAC_LDO_POWER_CTL_Pos)
#define ANAC_GPIO_POWER_CTL_Pos         	(31)           
#define ANAC_GPIO_POWER_CTL_Msk         	(0x1ul << ANAC_GPIO_POWER_CTL_Pos)


/*LP_INT_CTL*/
#define ANAC_INT_STANDBY_INT_EN_Pos         (0)
#define ANAC_INT_STANDBY_INT_EN_Msk         (0x1ul << ANAC_INT_STANDBY_INT_EN_Pos)
#define ANAC_INT_SLEEP_INT_EN_Pos           (1)
#define ANAC_INT_SLEEP_INT_EN_Msk           (0x1ul << ANAC_INT_SLEEP_INT_EN_Pos)
#define ANAC_INT_STANDBY_INT_FLG_Pos        (8)
#define ANAC_INT_STANDBY_INT_FLG_Msk        (0x1ul << ANAC_INT_STANDBY_INT_FLG_Pos)
#define ANAC_INT_SLEEP_INT_FLG_Pos          (9)
#define ANAC_INT_SLEEP_INT_FLG_Msk          (0x1ul << ANAC_INT_SLEEP_INT_FLG_Pos)
#define ANAC_INT_P56_WKUP_FLG_Pos           (16)
#define ANAC_INT_P56_WKUP_FLG_Msk           (0x1ul << ANAC_INT_P56_WKUP_FLG_Pos)
#define ANAC_INT_TMR32K_WKUP_FLG_Pos        (17)
#define ANAC_INT_TMR32K_WKUP_FLG_Msk        (0x1ul << ANAC_INT_TMR32K_WKUP_FLG_Pos)
#define ANAC_SRAM_RETENTION_FLG_Pos        	(18)
#define ANAC_SRAM_RETENTION_FLG_Msk        	(0x1ul << ANAC_SRAM_RETENTION_FLG_Pos)


/*LP_RST_CTRL*/
#define ANAC_DIG_DLY_TIME_Pos       		(0)
#define ANAC_DIG_DLY_TIME_Msk       		(0x3FFul << ANAC_DIG_DLY_TIME_Pos)
#define ANAC_DIG_RST_EN_Pos         		(10)
#define ANAC_DIG_RST_EN_Msk         		(0x1ul << ANAC_DIG_RST_EN_Pos)
#define ANAC_CLK_MUX_RST_CTRL_Pos         	(11)
#define ANAC_CLK_MUX_RST_CTRL_Msk         	(0x1ul << ANAC_CLK_MUX_RST_CTRL_Pos)
/*LP_WDT_CTRL*/
#define ANAC_LP_WDT_EN_Pos       			(0)
#define ANAC_LP_WDT_EN_Msk       			(0x1ul << ANAC_LP_WDT_EN_Pos)
#define ANAC_LP_WDT_CLEAR_Pos         		(1)
#define ANAC_LP_WDT_CLEAR_Msk         		(0x1ul << ANAC_LP_WDT_CLEAR_Pos)
/*ANA_BUNK*/

/*ANA_TEMP*/
#define ANAC_TEMP_ENABLE_Pos            	(0)
#define ANAC_TEMP_ENABLE_Msk            	(0x1ul << ANAC_TEMP_ENABLE_Pos)
#define ANAC_TEMP_GAIN_CTL_Pos            	(1)
#define ANAC_TEMP_GAIN_CTL_Msk            	(0x3ul << ANAC_TEMP_GAIN_CTL_Pos)

/*ANA_MISC*/
#define ANAC_MISC_SCAN_MODE_Pos             (0)
#define ANAC_MISC_SCAN_MODE_Msk             (0x1ul << ANAC_MISC_SCAN_MODE_Pos)
#define ANAC_MISC_MBIST_MODE_Pos            (1)
#define ANAC_MISC_MBIST_MODE_Msk            (0x1ul << ANAC_MISC_MBIST_MODE_Pos)
#define ANAC_MISC_EXT_RCL_EN_Pos            (3)
#define ANAC_MISC_EXT_RCL_EN_Msk            (0x1ul << ANAC_MISC_EXT_RCL_EN_Pos)
#define ANAC_MISC_EXT_RCH_EN_Pos            (4)
#define ANAC_MISC_EXT_RCH_EN_Msk            (0x1ul << ANAC_MISC_EXT_RCH_EN_Pos)
/**@}*/ /* end of analog register group */


/**
    @addtogroup EFUSE Controller
    Memory Mapped Structure for EFUSE Controller
@{ */

/**
  * @brief Efuse Interface
  */
typedef struct {                                //offset
    __IO uint32_t  EF_CTL                   ;   //0x00
    __IO uint32_t  EF_ADDR                  ;   //0x04
    __IO uint32_t  EF_DAT                   ;   //0x08
    __IO uint32_t  EF_RSV0                  ;   //0x0C
    __IO uint32_t  EF_VDD                   ;   //0x10
    __IO uint32_t  EF_RSV1                  ;   //0x14
    __IO uint32_t  EF_CMD                   ;   //0x18
    __IO uint32_t  EF_TRG                   ;   //0x1c
    __IO uint32_t  EF_PROG_TIMING1          ;   //0x20
    __IO uint32_t  EF_PROG_TIMING2          ;   //0x24
    __IO uint32_t  EF_PROG_TIMING3          ;   //0x28
    __IO uint32_t  EF_READ_TIMING4          ;   //0x2c
    __IO uint32_t  EF_READ_TIMING5          ;   //0x30
    __IO uint32_t  EF_RSV2[12]              ;   //0x34 ~ 0x60
    __IO uint32_t  EF_OP_ERROR              ;   //0x64
} EFUSE_T;


//EFUSE_CTL
#define EFUSE_CTL_EFUSE_EN_Pos                          (0)                                               
#define EFUSE_CTL_EFUSE_EN_Msk                          (0x1ul << EFUSE_CTL_EFUSE_EN_Pos)                          

#define EFUSE_CTL_EFUSECTL_BANK1_Pos                    (1)                                               
#define EFUSE_CTL_EFUSECTL_BANK1_Msk                    (0x1ul << EFUSE_CTL_EFUSECTL_BANK1_Pos )          

#define EFUSE_CTL_EFUSECTL_BANK2_Pos                    (2)                                               
#define EFUSE_CTL_EFUSECTL_BANK2_Msk                    (0x1ul << EFUSE_CTL_EFUSECTL_BANK2_Pos )          

#define EFUSE_CTL_EFUSECTL_BANK3_Pos                    (3)                                               
#define EFUSE_CTL_EFUSECTL_BANK3_Msk                    (0x1ul << EFUSE_CTL_EFUSECTL_BANK3_Pos )          

#define EFUSE_CTL_EFUSECTL_BANK4_Pos                    (4)                                               
#define EFUSE_CTL_EFUSECTL_BANK4_Msk                    (0x1ul << EFUSE_CTL_EFUSECTL_BANK4_Pos )          

#define EFUSE_CTL_THR_DVDD_Pos                          (5)
#define EFUSE_CTL_THR_DVDD_Msk                          (0x1ul << EFUSE_CTL_THR_DVDD_Pos)

#define EFUSE_CTL_TAEN_RD_CTL_Pos                       (6)
#define EFUSE_CTL_TAEN_RD_CTL_Msk                       (0x1ul << EFUSE_CTL_TAEN_RD_CTL_Pos)

#define EFUSE_CTL_THR_RD_CTL_Pos                        (7)
#define EFUSE_CTL_THR_RD_CTL_Msk                        (0x1ul << EFUSE_CTL_THR_RD_CTL_Pos)

#define EFUSE_CTL_TRD_CTL_Pos                           (8)
#define EFUSE_CTL_TRD_CTL_Msk                           (0x1ul << EFUSE_CTL_TRD_CTL_Pos)

#define EFUSE_CTL_TSR_RD_CTL_Pos                        (9)
#define EFUSE_CTL_TSR_RD_CTL_Msk                        (0x1ul << EFUSE_CTL_TSR_RD_CTL_Pos)

#define EFUSE_CTL_TSR_DVDD_CTL_Pos                      (10)
#define EFUSE_CTL_TSR_DVDD_CTL_Msk                      (0x1ul << EFUSE_CTL_TSR_DVDD_CTL_Pos)

#define EFUSE_CTL_THP_RD_CTL_Pos                        (11)
#define EFUSE_CTL_THP_RD_CTL_Msk                        (0x1ul << EFUSE_CTL_THP_RD_CTL_Pos)
 
#define EFUSE_CTL_THP_PG_AVDD_CTL_Pos                   (12)
#define EFUSE_CTL_THP_PG_AVDD_CTL_Msk                   (0x1ul << EFUSE_CTL_THP_PG_AVDD_CTL_Pos)

#define EFUSE_CTL_TAEN_PGM_CTL_Pos                      (13)
#define EFUSE_CTL_TAEN_PGM_CTL_Msk                      (0x1ul << EFUSE_CTL_TAEN_PGM_CTL_Pos)

#define EFUSE_CTL_THP_PGM_CTL_Pos                       (14)
#define EFUSE_CTL_THP_PGM_CTL_Msk                       (0x1ul << EFUSE_CTL_THP_PGM_CTL_Pos)

#define EFUSE_CTL_TPGM_CTL_Pos                          (15)
#define EFUSE_CTL_TPGM_CTL_Msk                          (0x1ul << EFUSE_CTL_TPGM_CTL_Pos)
 
#define EFUSE_CTL_TSP_PGM_CTL_Pos                       (16)
#define EFUSE_CTL_TSP_PGM_CTL_Msk                       (0x1ul << EFUSE_CTL_TSP_PGM_CTL_Pos)

#define EFUSE_CTL_TSP_PG_AVDD_CTL_Pos                   (17)
#define EFUSE_CTL_TSP_PG_AVDD_CTL_Msk                   (0x1ul << EFUSE_CTL_TSP_PG_AVDD_CTL_Pos)

#define EFUSE_CTL_TSP_RD_CTL_Pos                        (18)
#define EFUSE_CTL_TSP_TSP_RD_CTL_Msk                    (0x1ul << EFUSE_CTL_TSP_RD_CTL_Pos)

//EFUSE_ADDR
#define EFUSE_ADDR_EFUSEADDR_ADR_Pos                    (0)    
#define EFUSE_ADDR_EFUSEADDR_ADR_Msk                    (0xfful << EFUSE_ADDR_EFUSEADDR_ADR_Pos)

//EFUSE_DAT
#define EFUSE_DAT_EFUSEDAT_DAT_Pos                      (0)          
#define EFUSE_DAT_EFUSEDAT_DAT_Msk                      (0xfful << EFUSE_DAT_EFUSEDAT_DAT_Pos)

//EFUSE_VDD
#define EFUSE_VDD_DVDD_REG_Pos                          (0)                 
#define EFUSE_VDD_DVDD_REG_Msk                          (0x1ul << EFUSE_VDD_DVDD_REG_Pos)

#define EFUSE_VDD_AVDD_REG_Pos                          (1)                 
#define EFUSE_VDD_AVDD_REG_Msk                          (0x1ul << EFUSE_VDD_AVDD_REG_Pos)

//EFUSE_CMD
#define EFUSE_CMD_EFUSECMD_CMD_Pos                      (0)                              
#define EFUSE_CMD_EFUSECMD_CMD_Msk                      (0x3ul << EFUSE_CMD_EFUSECMD_CMD_Pos)

//EFUSE_TRG
#define EFUSE_TRG_EFUSETRG_GO_Pos                       (0)                                                
#define EFUSE_TRG_EFUSETRG_GO_Msk                       (0x1ul << EFUSE_TRG_EFUSETRG_GO_Pos)

//EFUSE_PROG_TIMING1
#define EFUSE_PROG_TIMING1_TSP_RD_SET_Pos               (0)       
#define EFUSE_PROG_TIMING1_TSP_RD_SET_Msk               (0x1fful << EFUSE_PROG_TIMING1_TSP_RD_SET_Pos) 

#define EFUSE_PROG_TIMING1_TSP_PG_AVDD_SET_Pos          (9)       
#define EFUSE_PROG_TIMING1_TSP_PG_AVDD_SET_Msk          (0x1fful << EFUSE_PROG_TIMING1_TSP_PG_AVDD_SET_Pos) 

#define EFUSE_PROG_TIMING1_TSP_PGM_SET_Pos              (18)       
#define EFUSE_PROG_TIMING1_TSP_PGM_SET_Msk              (0x1fful << EFUSE_PROG_TIMING1_TSP_PGM_SET_Pos) 

//EFUSE_PROG_TIMING2
#define EFUSE_PROG_TIMING2_TPGM_SET_Pos                 (0)       
#define EFUSE_PROG_TIMING2_TPGM_SET_Msk                 (0x1fful << EFUSE_PROG_TIMING2_TPGM_SET_Pos) 

#define EFUSE_PROG_TIMING2_THP_PGM_SET_Pos              (9)       
#define EFUSE_PROG_TIMING2_THP_PGM_SET_Msk              (0x1fful << EFUSE_PROG_TIMING2_THP_PGM_SET_Pos) 

#define EFUSE_PROG_TIMING2_TAEN_PGM_SET_Pos             (18)       
#define EFUSE_PROG_TIMING2_TAEN_PGM_SET_Msk             (0x1fful << EFUSE_PROG_TIMING2_TAEN_PGM_SET_Pos) 

//EFUSE_PROG_TIMING3
#define EFUSE_PROG_TIMING3_THP_PG_AVDD_SET_Pos          (0)       
#define EFUSE_PROG_TIMING3_THP_PG_AVDD_SET_Msk          (0x1fful << EFUSE_PROG_TIMING3_THP_PG_AVDD_SET_Pos) 

#define EFUSE_PROG_TIMING3_THP_RD_SET_Pos               (9)       
#define EFUSE_PROG_TIMING3_THP_RD_SET_Msk               (0x1fful << EFUSE_PROG_TIMING3_THP_RD_SET_Pos) 

//EFUSE_READ_TIMING4
#define EFUSE_READ_TIMING4_TSR_DVDD_SET_Pos             (0)   
#define EFUSE_READ_TIMING4_TSR_DVDD_SET_Msk             (0x1fful << EFUSE_READ_TIMING4_TSR_DVDD_SET_Pos)

#define EFUSE_READ_TIMING4_TSR_RD_SET_Pos               (9)   
#define EFUSE_READ_TIMING4_TSR_RD_SET_Msk               (0x1fful << EFUSE_READ_TIMING4_TSR_RD_SET_Pos)

#define EFUSE_READ_TIMING4_TRD_SET_Pos                  (18)   
#define EFUSE_READ_TIMING4_TRD_SET_Msk                  (0x1fful << EFUSE_READ_TIMING4_TRD_SET_Pos)


//EFUSE_READ_TIMING5
#define EFUSE_READ_TIMING5_THR_RD_SET_Pos               (0)
#define EFUSE_READ_TIMING5_THR_RD_SET_Msk               (0x1fful << EFUSE_READ_TIMING5_THR_RD_SET_Pos)

#define EFUSE_READ_TIMING5_TAEN_RD_SET_Pos              (9)
#define EFUSE_READ_TIMING5_TAEN_RD_SET_Msk              (0x1fful << EFUSE_READ_TIMING5_TAEN_RD_SET_Pos)

#define EFUSE_READ_TIMING5_THR_DVDD_SET_Pos             (18)
#define EFUSE_READ_TIMING5_THR_DVDD_SET_Msk             (0x1fful << EFUSE_READ_TIMING5_THR_DVDD_SET_Pos)
/**@}*/ /* end of efuse register group */

/**
    @addtogroup CLKTRIM Controller
    Memory Mapped Structure for CLKTRIM Controller
@{ */

/**
  * @brief Clktrim Interface
  */
typedef struct {                                  //offset
    __IO uint32_t  TRIM_EN                    ;   //0x00
    __IO uint32_t  TRIM_CODE                  ;   //0x04
    __IO uint32_t  TRIM_CTRL             	  ;   //0x08
    __IO uint32_t  TRIM_INT               	  ;   //0x0C
	__IO uint32_t  TRIM_CAL_CNT               ;   //0x10
	__IO uint32_t  TRIM_IDEAL_CNT             ;   //0x14
	__I  uint32_t  TRIM_REAL_CNT              ;   //0x18
} TRIM_T;

/*TRIM_EN*/
#define TRIM_MEASURE_TUNING_EN_Pos                      (0)           
#define TRIM_MEASURE_TUNING_EN_Msk                      (0x1ul << TRIM_MEASURE_TUNING_EN_Pos)
#define TRIM_COARSE_TUNING_EN_Pos                       (1)           
#define TRIM_COARSE_TUNING_EN_Msk                       (0x1ul << TRIM_COARSE_TUNING_EN_Pos)
#define TRIM_FINE_TUNING_EN_Pos                         (2)           
#define TRIM_FINE_TUNING_EN_Msk                         (0x1ul << TRIM_FINE_TUNING_EN_Pos)
#define TRIM_PRECISON_TUNING_EN_Pos                     (3)           
#define TRIM_PRECISON_TUNING_EN_Msk                     (0x1ul << TRIM_PRECISON_TUNING_EN_Pos)
/*TRIM_CODE*/
#define TRIM_COARSE_CODE_Pos                            (0)           
#define TRIM_COARSE_CODE_Msk                            (0x7ul << TRIM_COARSE_CODE_Pos)
#define TRIM_FINE_CODE_Pos                          	(8)           
#define TRIM_FINE_CODE_Msk                          	(0x3ful << TRIM_FINE_CODE_Pos)
#define TRIM_PRECISION_CODE_Pos                         (16)           
#define TRIM_PRECISION_CODE_Msk                         (0xfful << TRIM_PRECISION_CODE_Pos)
#define TRIM_BIT_WIDTH_Pos                          	(24)           
#define TRIM_BIT_WIDTH_Msk                          	(0x7ul << TRIM_BIT_WIDTH_Pos)
/*TRIM_CTRL*/
#define TRIM_CLRL_RELATION_Pos                          (0)           
#define TRIM_CLRL_RELATION_Msk                          (0x1ul << TRIM_CLRL_RELATION_Pos)
#define TRIM_CLRL_EARLY_TERM_EN_Pos                     (8)           
#define TRIM_CLRL_EARLY_TERM_EN_Msk                     (0x1ul << TRIM_CLRL_EARLY_TERM_EN_Pos)
#define TRIM_CLRL_ERR_RANGE_Pos                         (16)           
#define TRIM_CLRL_ERR_RANGE_Msk                         (0xfffful << TRIM_CLRL_ERR_RANGE_Pos)

/*TRIM_INT*/
#define TRIM_INT_EN_Pos                          		(0)           
#define TRIM_INT_EN_Msk                          		(0x1ul << TRIM_INT_EN_Pos)
#define TRIM_INT_MEASURE_STOP_Pos                       (1)           
#define TRIM_INT_MEASURE_STOP_Msk                       (0x1ul << TRIM_INT_MEASURE_STOP_Pos)
#define TRIM_FLAG_MEASURE_STOP_Pos                      (2)           
#define TRIM_FLAG_MEASURE_STOP_Msk                      (0x1ul << TRIM_FLAG_MEASURE_STOP_Pos)
#define TRIM_INT_CTUNE_STOP_Pos                         (3)           
#define TRIM_INT_CTUNE_STOP_Msk                         (0x1ul << TRIM_INT_CTUNE_STOP_Pos)
#define TRIM_FLAG_CTUNE_STOP_Pos                        (4)           
#define TRIM_FLAG_CTUNE_STOP_Msk                        (0x1ul << TRIM_FLAG_CTUNE_STOP_Pos)
#define TRIM_INT_FTUNE_STOP_Pos                         (5)           
#define TRIM_INT_FTUNE_STOP_Msk                         (0x1ul << TRIM_INT_FTUNE_STOP_Pos)
#define TRIM_FLAG_FTUNE_STOP_Pos                        (6)           
#define TRIM_FLAG_FTUNE_STOP_Msk                        (0x1ul << TRIM_FLAG_FTUNE_STOP_Pos)
#define TRIM_INT_PTUNE_STOP_Pos                         (7)           
#define TRIM_INT_PTUNE_STOP_Msk                         (0x1ul << TRIM_INT_PTUNE_STOP_Pos)
#define TRIM_FLAG_PTUNE_STOP_Pos                        (8)           
#define TRIM_FLAG_PTUNE_STOP_Msk                        (0x1ul << TRIM_FLAG_PTUNE_STOP_Pos)
#define TRIM_INT_OVERFLOW_Pos                          	(9)           
#define TRIM_INT_OVERFLOW_Msk                          	(0x1ul << TRIM_INT_OVERFLOW_Pos)
#define TRIM_FLAG_OVERFLOW_Pos                          (10)           
#define TRIM_FLAG_OVERFLOW_Msk                          (0x1ul << TRIM_FLAG_OVERFLOW_Pos)
#define TRIM_INT_HW_TRIG_Pos                          	(11)           
#define TRIM_INT_HW_TRIG_Msk                          	(0x1ul << TRIM_INT_HW_TRIG_Pos)
#define TRIM_FLAG_HW_TRIG_Pos                        	(12)           
#define TRIM_FLAG_HW_TRIG_Msk                        	(0x1ul << TRIM_FLAG_HW_TRIG_Pos)
/*TRIM_CAL_CNT  */
#define TRIM_CALC_CNT_Pos                        		(0)           
#define TRIM_CALC_CNT_Msk                        		(0xFFFFFFul << TRIM_INT_EN_Pos)
#define TRIM_CALC_WATI_CNT_Pos                        	(24)           
#define TRIM_CALC_WATI_CNT_Msk                        	(0xFFul << TRIM_CALC_WATI_CNT_Pos)
/**@}*/ /* end of clktrim register group */

/**
    @addtogroup ACC Controller
    Memory Mapped Structure for ACC Controller
@{ */

/**
  * @brief Acc Interface
  */
typedef struct {
    __IO uint32_t ACC_R0;           		// 0x40090000
    __IO uint32_t ACC_R1;           		// 0x40090004
    __IO uint32_t ACC_R2;           		// 0x40090008
    __IO uint32_t ACC_MULT_TRG;          	// 0x4009000C, WP
    __IO uint32_t ACC_INT;          		// 0x40090010, WP
    __IO uint32_t ACC_DIV_TRG;         		// 0x40090014
    __IO uint32_t ACC_LOGIC_SELECT;       	// 0x40090018, WP
    __IO uint32_t ACC_NUM_WORDS;    		// 0x4009001C
    __IO uint32_t ACC_DIVISOR;         		// 0x40090020, WP
    __IO uint32_t ACC_DIVIDEND;        		// 0x40090024, WP
    __IO uint32_t ACC_MULT_LEFT[8];      	// 0x40090028
    __IO uint32_t ACC_MULT_RIGHT[8];     	// 0x40090048
    __I  uint32_t ACC_MULT_RESULT[16];   	// 0x40090068
    __IO uint32_t ACC_MULT_DIV_SEL;       	// 0x400900A8, WP
    __I  uint32_t ACC_DIV_RESULT;       	// 0x400900AC	
}ACC_T;

//ACC_INT
#define ACC_INT_INT_EN_Pos                       		(0)           
#define ACC_INT_INT_EN_Msk                       		(0x1ul << ACC_INT_INT_EN_Pos)
#define ACC_INT_STATUS_FLAG_Pos                    		(1)           
#define ACC_INT_STATUS_FLAG_Msk                    		(0x1ul << ACC_INT_STATUS_FLAG_Pos)
#define ACC_INT_INT_FLAG_Pos                    		(2)           
#define ACC_INT_INT_FLAG_Msk                    		(0x1ul << ACC_INT_INT_FLAG_Pos)
//ACC_LOGIC_SELECT
#define ACC_LOGIC_SEL_MULT_EN_Pos                       (0)           
#define ACC_LOGIC_SEL_MULT_EN_Msk                       (0x1ul << ACC_LOGIC_SEL_MULT_EN_Pos)
#define ACC_LOGIC_SEL_CALC_CYCLE_Pos                    (8)           
#define ACC_LOGIC_SEL_CALC_CYCLE_Msk                    (0x7ul << ACC_LOGIC_SEL_CALC_CYCLE_Pos)
//ACC_MULT_DIV_SEL
#define ACC_MULT_DIV_SEL_DIV_EN_Pos                     (0)           
#define ACC_MULT_DIV_SEL_DIV_EN_Msk                     (0x1ul << ACC_MULT_DIV_SEL_DIV_EN_Pos)
#define ACC_MULT_DIV_SEL_CALC_SIGNED_Pos                (1)           
#define ACC_MULT_DIV_SEL_CALC_SIGNED_Msk                (0x1ul << ACC_MULT_DIV_SEL_CALC_SIGNED_Pos)
#define ACC_MULT_DIV_SEL_CALC_CYCLE_Pos                	(2)           
#define ACC_MULT_DIV_SEL_CALC_CYCLE_Msk                	(0x3Ful << ACC_MULT_DIV_SEL_CALC_CYCLE_Pos)
/**@}*/


/**
    @addtogroup USB Controller
    Memory Mapped Structure for USB Controller
@{ */

/**
  * @brief Usb Interface
  */
typedef struct {
	__IO uint8_t FADDR    			;//0X00;
	__IO uint8_t POWER    			;//0X01;
	__IO uint8_t INT_IN1  			;//0X02;
	__IO uint8_t INT_IN2  			;//0X03;
	__IO uint8_t INT_OUT1 			;//0X04;
	__IO uint8_t INT_OUT2 			;//0X05;
	__IO uint8_t INT_USB  			;//0X06;
	__IO uint8_t INT_IN1E 			;//0X07;
	__IO uint8_t INT_IN2E 			;//0X08;
	__IO uint8_t INT_OUT1E			;//0X09;
	__IO uint8_t INT_OUT2E			;//0X0A;
	__IO uint8_t INT_USBE 			;//0X0B;
	__IO uint8_t FRAME1   			;//0X0C;
	__IO uint8_t FRAME2   			;//0X0D;
	__IO uint8_t INDEX    			;//0X0E;
	__IO uint8_t REV     			;//0X0F;
	__IO uint8_t MAX_PKT_IN   		;//0X10;
	__IO uint8_t CSR0_INCSR1   		;//0X11;
	__IO uint8_t IN_CSR2   			;//0X12;
	__IO uint8_t MAX_PKT_OUT  		;//0X13;
	__IO uint8_t OUT_CSR1  			;//0X14;
	__IO uint8_t OUT_CSR2  			;//0X15;
	__IO uint8_t OUT_COUNT1			;//0X16;
	__IO uint8_t OUT_COUNT2			;//0X17;
	__IO uint8_t REV0[8]			;//0X18~1F;
	__IO uint8_t FIFO_EP0 			;//0X20;
	__IO uint8_t REV1[3]			;//0X21~23;
	__IO uint8_t FIFO_EP1 			;//0X24; 
	__IO uint8_t REV2[3]			;//0X25~27;
	__IO uint8_t FIFO_EP2 			;//0X28;
	__IO uint8_t REV3[3]			;//0X29~2B;
	__IO uint8_t FIFO_EP3 			;//0X2C;    
}USB_T;

typedef struct {
	__IO uint32_t DMA_INTR 			;//0X00;
	__IO uint32_t CNTL1    			;//0X04;
	__IO uint32_t ADDR1    			;//0X08;
	__IO uint32_t COUNT1   			;//0X0C;
//	__IO uint32_t REV   			;//0X10;
//	__IO uint32_t CNTL2    			;//0X14;
//	__IO uint32_t ADDR2    			;//0X18;
//	__IO uint32_t COUNT2  			;//0X1C;
}USBDMA_T;
/**@}*/


/**
    @addtogroup PRIVATE RF Controller
    Memory Mapped Structure for PRIVATE RF Controller
@{ */

/**
  * @brief Private rf Interface
  */

typedef struct {
    __IO uint32_t revs0;            // 0x50020400
    __IO uint32_t FSM5;             // 0x50020404
    __IO uint32_t FSM6;             // 0x50020408
    __IO uint32_t FSM13;            // 0x5002040c
    __IO uint32_t FSM14;            // 0x50020410
    __IO uint32_t FSM15;            // 0x50020414
    __IO uint32_t FSM16;            // 0x50020418
    __IO uint32_t FSM17;            // 0x5002041C
    __IO uint32_t FSM18;            // 0x50020420
    __IO uint32_t FSM19;            // 0x50020424
    __IO uint32_t PHY1;             // 0x50020428
    __IO uint32_t PHY2;             // 0x5002042c
    __IO uint32_t PHY3;             // 0x50020430
    __IO uint32_t PHY4;             // 0x50020434
    __IO uint32_t PHY5;             // 0x50020438
    __IO uint32_t revs1[2];         // 0x5002043c~0x50020440
    __IO uint32_t INTR1;            // 0x50020444
    __IO uint32_t INTCLR;           // 0x50020448
    __IO uint32_t INTMSK;           // 0x5002044C
    __IO uint32_t INTR4;            // 0x50020450
    __IO uint32_t revs2;            // 0x50020454
    __IO uint32_t ACTTMR1;          // 0x50020458
    __IO uint32_t ACTTMR2;          // 0x5002045c
    __IO uint32_t ACTTMR6;          // 0x50020460
    __IO uint32_t ACTTMR7;          // 0x50020464
    __IO uint32_t revs3;            // 0x50020468
    __IO uint32_t SECURE1;          // 0x5002046c
    __IO uint32_t SECURE2;          // 0x50020470
    __IO uint32_t SECURE3;          // 0x50020474
    __IO uint32_t SECURE4;          // 0x50020478
    __IO uint32_t SECURE5;          // 0x5002047c
    __IO uint32_t SECURE6;          // 0x50020480
    __IO uint32_t SECURE7;          // 0x50020484
    __IO uint32_t SECURE8;          // 0x50020488
    __IO uint32_t SECURE9;          // 0x5002048c
    __IO uint32_t SECURE10;         // 0x50020490
    __IO uint32_t revs4;            // 0x50020494
    __IO uint32_t TEST_MUX00;       // 0x50020498
    __IO uint32_t RNG1;             // 0x5002049c
    __IO uint32_t RNG2;             // 0x500204a0
    __IO uint32_t revs5;            // 0x500204a4
    __IO uint32_t LL_MAC_CTRL;      // 0x500204a8
    __IO uint32_t TEST_MUX01;       // 0x500204ac
    __IO uint32_t TEST_MUX02;       // 0x500204b0
    __IO uint32_t TEST_MUX03;       // 0x500204b4
    __IO uint32_t revs6[2];         // 0x500204b8~0x500204bc
    __IO uint32_t R00_CTL; 				  // 0x500204C0
    __IO uint32_t R01_INT_CTL; 			// 0x500204C4
    __IO uint32_t R02_TMR_CTL; 			// 0x500204C8
    __IO uint32_t R03_RX_ADDR_L;    // 0x500204CC
    __IO uint32_t R04_RX_ADDR_M; 		// 0x500204D0
    __IO uint32_t R05_TX_ADDR_L;    // 0x500204D4
    __IO uint32_t R06_TX_ADDR_M;    // 0x500204D8
    __IO uint32_t R07_SRAM_CTL;     // 0x500204DC
    __IO uint32_t R08_REVS;         // 0x500204E0
    __IO uint32_t R09_REVS;         // 0x500204E4
}PRI_RF_T;

/*fsm5*/
#define FSM5_SM_STP_Pos                                  6
#define FSM5_SM_STP_Msk                                  (0x1ul<<FSM5_SM_STP_Pos)            /* Description:   - Stop LL scanning or initiating immediately - Stop LL  DTM transmission and reception - Stop MAC FSM RX operation */
#define FSM5_SM_TX_LATENCY_Pos                           0
#define FSM5_SM_TX_LATENCY_Msk                           (0x3ful<<FSM5_SM_TX_LATENCY_Pos)            /* This field sets the required time for the TX path to flush the last bit on the air after the packet processor generates the TX Done flag. */
/*fsm6*/
#define FSM6_SM_RX_LATENCY_Pos                           0
#define FSM6_SM_RX_LATENCY_Msk                           (0xful<<FSM6_SM_RX_LATENCY_Pos)             /* This field sets the time it takes to receive the last bit on the air until the Packet processor generates the RX Done flag. */
/*fsm13*/
#define FSM13_SM_RX_PACKET_ADDR_1_Pos                    16
#define FSM13_SM_RX_PACKET_ADDR_1_Msk                    (0xfffful<<FSM13_SM_RX_PACKET_ADDR_1_Pos)      /* This field specifies the start address of the first RX packet in DATA RAM. */
#define FSM13_SM_TX_PACKET_ADDR_1_Pos                    0
#define FSM13_SM_TX_PACKET_ADDR_1_Msk                    (0xfffful<<FSM13_SM_TX_PACKET_ADDR_1_Pos)          /* This field specifies the start address of the first TX packet in DATA RAM. */
/*fsm14*/
#define FSM14_SM_RX_PACKET_ADDR_2_Pos                    16
#define FSM14_SM_RX_PACKET_ADDR_2_Msk                    (0xfffful<<FSM14_SM_RX_PACKET_ADDR_2_Pos)      /* This field specifies the start address of the second RX packet in DATA RAM. */
#define FSM14_SM_TX_PACKET_ADDR_2_Pos                    0
#define FSM14_SM_TX_PACKET_ADDR_2_Msk                    (0xfffful<<FSM14_SM_TX_PACKET_ADDR_2_Pos)          /* This field specifies the start address of the second TX packet in DATA RAM */
/*fsm15*/
#define FSM15_SM_RX_PACKET_ADDR_3_Pos                    16
#define FSM15_SM_RX_PACKET_ADDR_3_Msk                    (0xfffful<<FSM15_SM_RX_PACKET_ADDR_3_Pos)      /* This field specifies the start address of the third RX packet in DATA RAM. */
#define FSM15_SM_TX_PACKET_ADDR_3_Pos                    0
#define FSM15_SM_TX_PACKET_ADDR_3_Msk                    (0xfffful<<FSM15_SM_TX_PACKET_ADDR_3_Pos)          /* This field specifies the start address of the third TX packet in DATA RAM */
/*fsm16*/
#define FSM16_SM_RX_PACKET_ADDR_4_Pos                    16
#define FSM16_SM_RX_PACKET_ADDR_4_Msk                    (0xfffful<<FSM16_SM_RX_PACKET_ADDR_4_Pos)      /* This field specifies the start address of the fourth RX packet in DATA RAM. */
#define FSM16_SM_TX_PACKET_ADDR_4_Pos                    0
#define FSM16_SM_TX_PACKET_ADDR_4_Msk                    (0xfffful<<FSM16_SM_TX_PACKET_ADDR_4_Pos)          /* This field specifies the start address of the fourth TX packet in DATA RAM */
/*fsm17*/
#define FSM17_SM_RX_PACKET_ADDR_5_Pos                    16
#define FSM17_SM_RX_PACKET_ADDR_5_Msk                    (0xfffful<<FSM17_SM_RX_PACKET_ADDR_5_Pos)      /* This field specifies the start address of the fifth RX packet in DATA RAM. */
#define FSM17_SM_TX_PACKET_ADDR_5_Pos                    0
#define FSM17_SM_TX_PACKET_ADDR_5_Msk                    (0xfffful<<FSM17_SM_TX_PACKET_ADDR_5_Pos)          /* This field specifies the start address of the fifth TX packet in DATA RAM */
/*fsm18*/
#define FSM18_SM_RX_PACKET_ADDR_6_Pos                    16
#define FSM18_SM_RX_PACKET_ADDR_6_Msk                    (0xfffful<<FSM18_SM_RX_PACKET_ADDR_6_Pos)      /* This field specifies the start address of the sixth RX packet in DATA RAM. */
#define FSM18_SM_TX_PACKET_ADDR_6_Pos                    0
#define FSM18_SM_TX_PACKET_ADDR_6_Msk                    (0xfffful<<FSM18_SM_TX_PACKET_ADDR_6_Pos)          /* This field specifies the start address of the sixth TX packet in DATA RAM */
/*fsm19*/
#define FSM19_SM_RX_PACKET_ADDR_7_Pos                    16
#define FSM19_SM_RX_PACKET_ADDR_7_Msk                    (0xfffful<<FSM19_SM_RX_PACKET_ADDR_7_Pos)      /* This field specifies the start address of the seventh RX packet in DATA RAM. */
#define FSM19_SM_TX_PACKET_ADDR_7_Pos                    0
#define FSM19_SM_TX_PACKET_ADDR_7_Msk                    (0xfffful<<FSM19_SM_TX_PACKET_ADDR_7_Pos)          /* This field specifies the start address of the seventh TX packet in DATA RAM */
/*phy1*/
#define PHY1_PHY_DRV_RSP_BUF_FULL_Pos                    22
#define PHY1_PHY_DRV_RSP_BUF_FULL_Msk                    (0x1ul<<PHY1_PHY_DRV_RSP_BUF_FULL_Pos)        /* This field sets the Response buffer full flag. */
#define PHY1_PHY_DRV_RSP_BUF_EMPTY_Pos                   21
#define PHY1_PHY_DRV_RSP_BUF_EMPTY_Msk                   (0x1ul<<PHY1_PHY_DRV_RSP_BUF_EMPTY_Pos)        /* This field sets the Response buffer empty flag. */
#define PHY1_PHY_DRV_CFG_BUF_ERROR_Pos                   20
#define PHY1_PHY_DRV_CFG_BUF_ERROR_Msk                   (0x1ul<<PHY1_PHY_DRV_CFG_BUF_ERROR_Pos)        /* This field sets the Configuration buffer error flag (underflow or overflow). */
#define PHY1_PHY_DRV_CFG_BUF_FULL_Pos                    19
#define PHY1_PHY_DRV_CFG_BUF_FULL_Msk                    (0x1ul<<PHY1_PHY_DRV_CFG_BUF_FULL_Pos)         /* This field sets the Configuration buffer full flag. */
#define PHY1_PHY_DRV_CFG_BUF_EMPTY_Pos                   18
#define PHY1_PHY_DRV_CFG_BUF_EMPTY_Msk                   (0x1ul<<PHY1_PHY_DRV_CFG_BUF_EMPTY_Pos)         /* This field sets the Configuration buffer empty flag. */
#define PHY1_IF_CLK_PRESCALE_Pos                         13
#define PHY1_IF_CLK_PRESCALE_Msk                         (0x1ful<<PHY1_IF_CLK_PRESCALE_Pos)         /* This field specifies the prescaler value to be used for configuration interface, should be even number larger than 1 */
#define PHY1_PHY_DRV_RSSI_REG_ADDR_Pos                   5
#define PHY1_PHY_DRV_RSSI_REG_ADDR_Msk                   (0xfful<<PHY1_PHY_DRV_RSSI_REG_ADDR_Pos)          /* The address of RSSI in phy register file */
#define PHY1_PHY_DRV_CFG_TRG_Pos                         4
#define PHY1_PHY_DRV_CFG_TRG_Msk                         (0x1ul<<PHY1_PHY_DRV_CFG_TRG_Pos)            /* This field triggers the execution of the commands loaded in the configuration buffer. */
#define PHY1_PHY_DRV_CFG_REG_Pos                         2
#define PHY1_PHY_DRV_CFG_REG_Msk                         (0x3ul<<PHY1_PHY_DRV_CFG_REG_Pos)             /* This field specifies the General purpose configuration register. - phy_drv_cfg_reg [0] :  Define  PHY scheme  - phy_drv_cfg_reg [1] : Define writing modes in response buffer */
#define PHY1_PHY_DRV_RSP_BUF_FLUSH_Pos                   1
#define PHY1_PHY_DRV_RSP_BUF_FLUSH_Msk                   (0x1ul<<PHY1_PHY_DRV_RSP_BUF_FLUSH_Pos)             /* This field specifies the Flush of the response buffer. */
#define PHY1_PHY_DRV_CFG_BUF_FLUSH_Pos                   0
#define PHY1_PHY_DRV_CFG_BUF_FLUSH_Msk                   (0x1ul<<PHY1_PHY_DRV_CFG_BUF_FLUSH_Pos)             /* This field specifies the Flush of the configuration buffer. */
/*phy2*/
#define PHY2_PHY_DRV_RSP_BUF_ERROR_Pos                   28
#define PHY2_PHY_DRV_RSP_BUF_ERROR_Msk                   (0x1ul<<PHY2_PHY_DRV_RSP_BUF_ERROR_Pos)      /* This field sets the Response buffer error flag. */
#define PHY2_PHY_DRV_SEQ_RX_END_ADDR_Pos                 21
#define PHY2_PHY_DRV_SEQ_RX_END_ADDR_Msk                 (0x7ful<<PHY2_PHY_DRV_SEQ_RX_END_ADDR_Pos)       /* This field specifies the Sequencer start address for RX end operation sequence. */
#define PHY2_PHY_DRV_SEQ_TX_END_ADDR_Pos                 14
#define PHY2_PHY_DRV_SEQ_TX_END_ADDR_Msk                 (0x7ful<<PHY2_PHY_DRV_SEQ_TX_END_ADDR_Pos)        /* This field specifies the Sequencer start address for TX end operation sequence. */
#define PHY2_PHY_DRV_SEQ_RX_STRT_ADDR_Pos                7
#define PHY2_PHY_DRV_SEQ_RX_STRT_ADDR_Msk                (0x7ful<<PHY2_PHY_DRV_SEQ_RX_STRT_ADDR_Pos)          /* This field specifies the Sequencer start address for RX start operation sequence. */
#define PHY2_PHY_DRV_SEQ_TX_STRT_ADDR_Pos                0
#define PHY2_PHY_DRV_SEQ_TX_STRT_ADDR_Msk                (0x7ful<<PHY2_PHY_DRV_SEQ_TX_STRT_ADDR_Pos)            /* This field specifies the Sequencer start address for TX start operation sequence. */
/*phy3*/
#define PHY3_PHY_DRV_CFG_BUF_DIN_Pos                     0
#define PHY3_PHY_DRV_CFG_BUF_DIN_Msk                     (0xfffffffful<<PHY3_PHY_DRV_CFG_BUF_DIN_Pos)      /* This field specifies the Configuration buffer input data. */
/*phy4*/
#define PHY4_PHY_DRV_RSP_BUF_DOUT_Pos                    0
#define PHY4_PHY_DRV_RSP_BUF_DOUT_Msk                    (0xfffffffful<<PHY4_PHY_DRV_RSP_BUF_DOUT_Pos)      /* This field specifies the Response buffer output data. */
/*phy5*/
#define PHY5_PHY_DRV_SET_CHAN_REG_ADDR_Pos               0
#define PHY5_PHY_DRV_SET_CHAN_REG_ADDR_Msk               (0xfful<<PHY5_PHY_DRV_SET_CHAN_REG_ADDR_Pos)            /* This field Contains the address of the register in PHY register file that change the RF channel */
/*intr1*/
#define INTR1_IC_RNG_DONE_Pos                            3
#define INTR1_IC_RNG_DONE_Msk                            (0x1ul<<INTR1_IC_RNG_DONE_Pos)             /* This field specifies the output status done signal of the TRNG. */
#define INTR1_IC_SEC_DONE_Pos                            2
#define INTR1_IC_SEC_DONE_Msk                            (0x1ul<<INTR1_IC_SEC_DONE_Pos)             /* This field specifies the output status of done signal of the security module. */
#define INTR1_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_Pos          1
#define INTR1_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_Msk          (0x1ul<<INTR1_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_Pos)             /* This field specifies the output status of Active timer start event compare IRQ. */
#define INTR1_IC_PHY_DRV_CFG_DONE_Pos                    0
#define INTR1_IC_PHY_DRV_CFG_DONE_Msk                    (0x1ul<<INTR1_IC_PHY_DRV_CFG_DONE_Pos)             /* This field specifies the output status of the PHY drv configuration done. */
/*intclr*/
#define INTCLR_IC_RNG_DONE_CLR_Pos                       3
#define INTCLR_IC_RNG_DONE_CLR_Msk                       (0x1ul<<INTCLR_IC_RNG_DONE_CLR_Pos)             /* This field specifies the active-high clear signal of TRNG done output status */
#define INTCLR_IC_SEC_DONE_CLR_Pos                       2
#define INTCLR_IC_SEC_DONE_CLR_Msk                       (0x1ul<<INTCLR_IC_SEC_DONE_CLR_Pos)             /* This field specifies the active-high clear signal of security done output status */
#define INTCLR_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_CLR_Pos     1
#define INTCLR_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_CLR_Msk     (0x1ul<<INTCLR_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_CLR_Pos)             /* This field specifies the active-high clear signal of active timer start event compare  irq output status */
#define INTCLR_IC_PHY_DRV_CFG_DONE_CLR_Pos               0
#define INTCLR_IC_PHY_DRV_CFG_DONE_CLR_Msk               (0x1ul<<INTCLR_IC_PHY_DRV_CFG_DONE_CLR_Pos)             /* This field specifies the active-high clear signal of PHY driver configuration output status */
/*intmsk*/
#define INTMSK_IC_RNG_DONE_MASK_Pos                      3
#define INTMSK_IC_RNG_DONE_MASK_Msk                      (0x1ul<<INTMSK_IC_RNG_DONE_MASK_Pos)             /* This field specifies the mask of the done signal of TRNG done interrupt */
#define INTMSK_IC_SEC_DONE_MASK_Pos                      2
#define INTMSK_IC_SEC_DONE_MASK_Msk                      (0x1ul<<INTMSK_IC_SEC_DONE_MASK_Pos)             /* This field specifies the mask signal for the security done interrupt */
#define INTMSK_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_MASK_Pos    1
#define INTMSK_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_MASK_Msk    (0x1ul<<INTMSK_IC_ACTTMR_CMPR_STRT_EVENT_IRQ_MASK_Pos)             /* This field specifies the mask signal for the active timer start event compare unit interrupt. */
#define INTMSK_IC_PHY_DRV_CFG_DONE_MASK_Pos              0
#define INTMSK_IC_PHY_DRV_CFG_DONE_MASK_Msk              (0x1ul<<INTMSK_IC_PHY_DRV_CFG_DONE_MASK_Pos)             /* This field specifies the mask signal for the PHY configuration done interrupt. */
/*intr4*/
#define INTR4_IC_CLR_IRQ_Pos                             6
#define INTR4_IC_CLR_IRQ_Msk                             (0x1ul<<INTR4_IC_CLR_IRQ_Pos)            /* This field specifies active high clear signal of the IRQ. */
#define INTR4_IC_ONE_PULSE_FLAG_IRQ_Pos                  5
#define INTR4_IC_ONE_PULSE_FLAG_IRQ_Msk                  (0x1ul<<INTR4_IC_ONE_PULSE_FLAG_IRQ_Pos)            /* This field defines the IRQ in self clear mode.   - 1'b0: Define IRQ output to be number of clock pulses of i_pulse_cnt  - 1'b1: Define IRQ output to be one pulse of its input clock */
#define INTR4_IC_MODE_IRQ_Pos                            4
#define INTR4_IC_MODE_IRQ_Msk                            (0x1ul<<INTR4_IC_MODE_IRQ_Pos)            /* This field applies the following mode for the IRQ:  - 1'b0: Self clear of IRQ  - 1'b1: Host clear through the i_clr_irq */
#define INTR4_IC_PULSE_CNT_Pos                           0
#define INTR4_IC_PULSE_CNT_Msk                           (0xful<<INTR4_IC_PULSE_CNT_Pos)             /* This field specifies the number of pulses when the output IRQ is high before being cleared. */
/*acttmr1*/
#define ACTTMR1_ACTTMR_CMPR_MAX_EVENT_DIN_Pos            0
#define ACTTMR1_ACTTMR_CMPR_MAX_EVENT_DIN_Msk            (0xfffffffful<<ACTTMR1_ACTTMR_CMPR_MAX_EVENT_DIN_Pos)      /* This field specifies the max event compare unit input compare value */
/*acttmr2*/
#define ACTTMR2_ACTTMR_DISABLE_FLG_Pos                   2
#define ACTTMR2_ACTTMR_DISABLE_FLG_Msk                   (0x1ul<<ACTTMR2_ACTTMR_DISABLE_FLG_Pos)             /* Flag used to force disable accttmr to cancel the multiple event scheduling */
#define ACTTMR2_ACT_CLK_ACCUR_FLG_Pos                    1
#define ACTTMR2_ACT_CLK_ACCUR_FLG_Msk                    (0x1ul<<ACTTMR2_ACT_CLK_ACCUR_FLG_Pos)             /* Flag used to schedule multiple events with active clock accuracy */
#define ACTTMR2_ACTTMR_CMPR_MAX_EVENT_EN_Pos             0
#define ACTTMR2_ACTTMR_CMPR_MAX_EVENT_EN_Msk             (0x1ul<<ACTTMR2_ACTTMR_CMPR_MAX_EVENT_EN_Pos)             /* This field specifies the max event compare unit enable. */
/*acttmr6*/
#define ACTTMR6_ACTTMR_CMPR_STRT_EVENT_DIN_Pos           0
#define ACTTMR6_ACTTMR_CMPR_STRT_EVENT_DIN_Msk           (0xfffffffful<<ACTTMR6_ACTTMR_CMPR_STRT_EVENT_DIN_Pos)      /* This field specifies the start event compare unit input compare value */
/*acttmr7*/
#define ACTTMR7_ACTTMR_CPTUR_BUS_DOUT_Pos                0
#define ACTTMR7_ACTTMR_CPTUR_BUS_DOUT_Msk                (0xfffffffful<<ACTTMR7_ACTTMR_CPTUR_BUS_DOUT_Pos)      /* This field specifies the capture units match output (snapshot of base timer on match). */
/*secure1*/
#define SECURE1_SEC_DECRYPT_EN_Pos                       26
#define SECURE1_SEC_DECRYPT_EN_Msk                       (0x1ul<<SECURE1_SEC_DECRYPT_EN_Pos)       /* Length of AD Data */
#define SECURE1_SEC_AD_LEN_Pos                           19
#define SECURE1_SEC_AD_LEN_Msk                           (0x7ful<<SECURE1_SEC_AD_LEN_Pos)       /* Length of AD Data */
#define SECURE1_SEC_MIC_LEN_Pos                          17
#define SECURE1_SEC_MIC_LEN_Msk                          (0x3ul<<SECURE1_SEC_MIC_LEN_Pos)         /* MIC Length 0 => 0Bytes, 1=> 4Bytes, 2 =>  8Bytes, 3 => 16Bytes */
#define SECURE1_SEC_VLD_MAC_Pos                          10
#define SECURE1_SEC_VLD_MAC_Msk                          (0x7ful<<SECURE1_SEC_VLD_MAC_Pos)         /* This field specifies the number of packets that have valid MIC. */
#define SECURE1_SEC_PKT_ENABLES_Pos                      3
#define SECURE1_SEC_PKT_ENABLES_Msk                      (0x7ful<<SECURE1_SEC_PKT_ENABLES_Pos)           /* This field indicates the packets to be processed. */
#define SECURE1_SEC_MODE_Pos                             1
#define SECURE1_SEC_MODE_Msk                             (0x3ul<<SECURE1_SEC_MODE_Pos)             /* Security mode  0 => ECB_Decrypt, 1=> CCM_Decrypt, 2 =>  ECB_Encrypt, 3 => CCM_Encrypt */
#define SECURE1_SEC_ENABLE_Pos                           0
#define SECURE1_SEC_ENABLE_Msk                           (0x1ul<<SECURE1_SEC_ENABLE_Pos)             /* This field specifies the active-high enable for AES.  - 1'b0: Enable secure controller   - 1'b1: Disable secure controller */
/*secure2*/
#define SECURE2_SEC_IV3_Pos                              24
#define SECURE2_SEC_IV3_Msk                              (0xfful<<SECURE2_SEC_IV3_Pos)      /* This field specifies Nonce 8. */
#define SECURE2_SEC_IV2_Pos                              16
#define SECURE2_SEC_IV2_Msk                              (0xfful<<SECURE2_SEC_IV2_Pos)        /* This field specifies Nonce 7. */
#define SECURE2_SEC_IV1_Pos                              8
#define SECURE2_SEC_IV1_Msk                              (0xfful<<SECURE2_SEC_IV1_Pos)          /* This field specifies Nonce 6. */
#define SECURE2_SEC_IV0_Pos                              0
#define SECURE2_SEC_IV0_Msk                              (0xfful<<SECURE2_SEC_IV0_Pos)            /* This field specifies Nonce 5. */
/*secure3*/
#define SECURE3_SEC_IV7_Pos                              24
#define SECURE3_SEC_IV7_Msk                              (0xfful<<SECURE3_SEC_IV7_Pos)      /* This field specifies Nonce 12. */
#define SECURE3_SEC_IV6_Pos                              16
#define SECURE3_SEC_IV6_Msk                              (0xfful<<SECURE3_SEC_IV6_Pos)        /* This field specifies Nonce 11. */
#define SECURE3_SEC_IV5_Pos                              8
#define SECURE3_SEC_IV5_Msk                              (0xfful<<SECURE3_SEC_IV5_Pos)          /* This field specifies Nonce 10. */
#define SECURE3_SEC_IV4_Pos                              0
#define SECURE3_SEC_IV4_Msk                              (0xfful<<SECURE3_SEC_IV4_Pos)            /* This field specifies Nonce 9. */
/*secure4*/
#define SECURE4_SEC_KEY2_RX_ADDR_Pos                     16
#define SECURE4_SEC_KEY2_RX_ADDR_Msk                     (0xfffful<<SECURE4_SEC_KEY2_RX_ADDR_Pos)      /* This field specifies the key pointer for packet 1. */
#define SECURE4_SEC_KEY1_RX_ADDR_Pos                     0
#define SECURE4_SEC_KEY1_RX_ADDR_Msk                     (0xfffful<<SECURE4_SEC_KEY1_RX_ADDR_Pos)          /* This field specifies the key pointer for packet 0. */
/*secure5*/
#define SECURE5_SEC_KEY4_RX_ADDR_Pos                     16
#define SECURE5_SEC_KEY4_RX_ADDR_Msk                     (0xfffful<<SECURE5_SEC_KEY4_RX_ADDR_Pos)      /* This field specifies the key pointer for packet 3. */
#define SECURE5_SEC_KEY3_RX_ADDR_Pos                     0
#define SECURE5_SEC_KEY3_RX_ADDR_Msk                     (0xfffful<<SECURE5_SEC_KEY3_RX_ADDR_Pos)          /* This field specifies the key pointer for packet 2. */
/*secure6*/
#define SECURE6_SEC_KEY6_RX_ADDR_Pos                     16
#define SECURE6_SEC_KEY6_RX_ADDR_Msk                     (0xfffful<<SECURE6_SEC_KEY6_RX_ADDR_Pos)      /* This field specifies the key pointer for packet 5. */
#define SECURE6_SEC_KEY5_RX_ADDR_Pos                     0
#define SECURE6_SEC_KEY5_RX_ADDR_Msk                     (0xfffful<<SECURE6_SEC_KEY5_RX_ADDR_Pos)          /* This field specifies the key pointer for packet 4. */
/*secure7*/
#define SECURE7_SEC_MD_LEN_Pos                           24
#define SECURE7_SEC_MD_LEN_Msk                           (0x7ful<<SECURE7_SEC_MD_LEN_Pos)      /* Length of m Data */
#define SECURE7_SEC_KEY7_RX_ADDR_Pos                     8
#define SECURE7_SEC_KEY7_RX_ADDR_Msk                     (0xfffful<<SECURE7_SEC_KEY7_RX_ADDR_Pos)        /* This field specifies the key pointer for packet 6. */
#define SECURE7_SEC_PKT_CNT_TX_MSB_Pos                   0
#define SECURE7_SEC_PKT_CNT_TX_MSB_Msk                   (0xfful<<SECURE7_SEC_PKT_CNT_TX_MSB_Pos)            /* Counter for the transmitted packets to be processed (Nonce 0:4) (MSB). */
/*secure8*/
#define SECURE8_SEC_PKT_CNT_TX_LSB_Pos                   0
#define SECURE8_SEC_PKT_CNT_TX_LSB_Msk                   (0xfffffffful<<SECURE8_SEC_PKT_CNT_TX_LSB_Pos)      /* Counter for the transmitted packets to be processed (Nonce 0:4) (LSB). */
/*secure9*/
#define SECURE9_SEC_PKT_CNT_RX_MSB_Pos                   0
#define SECURE9_SEC_PKT_CNT_RX_MSB_Msk                   (0xfful<<SECURE9_SEC_PKT_CNT_RX_MSB_Pos)            /* Counter for the received packets to be processed (Nonce 0:4)(MSB) */
/*secure10*/
#define SECURE10_SEC_PKT_CNT_RX_LSB_Pos                  0
#define SECURE10_SEC_PKT_CNT_RX_LSB_Msk                  (0xfffffffful<<SECURE10_SEC_PKT_CNT_RX_LSB_Pos)      /* Counter for the received packets to be processed (Nonce 0:4)(LSB) */
/*text_mux00*/
#define TEST_MUX00_TST_MUX_SELECT_03_Pos                 18
#define TEST_MUX00_TST_MUX_SELECT_03_Msk                 (0x3ful<<TEST_MUX00_TST_MUX_SELECT_03_Pos)        /* This field specifies the Test MUX 3 Selection lines. */
#define TEST_MUX00_TST_MUX_SELECT_02_Pos                 12
#define TEST_MUX00_TST_MUX_SELECT_02_Msk                 (0x3ful<<TEST_MUX00_TST_MUX_SELECT_02_Pos)           /* This field specifies the Test MUX 2 Selection lines. */
#define TEST_MUX00_TST_MUX_SELECT_01_Pos                 6
#define TEST_MUX00_TST_MUX_SELECT_01_Msk                 (0x3ful<<TEST_MUX00_TST_MUX_SELECT_01_Pos)           /* This field specifies the Test MUX 1 Selection lines. */
#define TEST_MUX00_TST_MUX_SELECT_00_Pos                 0
#define TEST_MUX00_TST_MUX_SELECT_00_Msk                 (0x3ful<<TEST_MUX00_TST_MUX_SELECT_00_Pos)            /* This field specifies the Test MUX 0 Selection lines. */
/*rng1*/
#define RNG1_RNG_RING_SCRMB_SEL_Pos                      1
#define RNG1_RNG_RING_SCRMB_SEL_Msk                      (0x1ul<<RNG1_RNG_RING_SCRMB_SEL_Pos)             /* This field enables the RING output only. */
#define RNG1_RNG_EN_Pos                                  0
#define RNG1_RNG_EN_Msk                                  (0x1ul<<RNG1_RNG_EN_Pos)             /* This field specifies the enable signal for the RNG module. */
/*rng2*/
#define RNG2_RNG_DATA_Pos                                0
#define RNG2_RNG_DATA_Msk                                (0xfffffffful<<RNG2_RNG_DATA_Pos)      /* This field specifies the 32-bit stream data output of the TRNG module. */
/*ll_mac_ctrl*/
#define LL_MAC_CTRL_CTRL_LL_EN_Pos                       0
#define LL_MAC_CTRL_CTRL_LL_EN_Msk                       (0x1ul<<LL_MAC_CTRL_CTRL_LL_EN_Pos)             /* 0 => MAC enabled 1 => LL enabled */
/*test_mux01*/
#define TEST_MUX01_TST_MUX_SELECT_07_Pos                 18
#define TEST_MUX01_TST_MUX_SELECT_07_Msk                 (0x3ful<<TEST_MUX01_TST_MUX_SELECT_07_Pos)       /* This field specifies the Test MUX 3 Selection lines. */
#define TEST_MUX01_TST_MUX_SELECT_06_Pos                 12
#define TEST_MUX01_TST_MUX_SELECT_06_Msk                 (0x3ful<<TEST_MUX01_TST_MUX_SELECT_06_Pos)          /* This field specifies the Test MUX 2 Selection lines. */
#define TEST_MUX01_TST_MUX_SELECT_05_Pos                 6
#define TEST_MUX01_TST_MUX_SELECT_05_Msk                 (0x3ful<<TEST_MUX01_TST_MUX_SELECT_05_Pos)           /* This field specifies the Test MUX 1 Selection lines. */
#define TEST_MUX01_TST_MUX_SELECT_04_Pos                 0
#define TEST_MUX01_TST_MUX_SELECT_04_Msk                 (0x3ful<<TEST_MUX01_TST_MUX_SELECT_04_Pos)            /* This field specifies the Test MUX 0 Selection lines. */
/*test_mux02*/
#define TEST_MUX02_TST_MUX_SELECT_11_Pos                 18
#define TEST_MUX02_TST_MUX_SELECT_11_Msk                 (0x3ful<<TEST_MUX02_TST_MUX_SELECT_11_Pos)        /* This field specifies the Test MUX 3 Selection lines. */
#define TEST_MUX02_TST_MUX_SELECT_10_Pos                 12
#define TEST_MUX02_TST_MUX_SELECT_10_Msk                 (0x3ful<<TEST_MUX02_TST_MUX_SELECT_10_Pos)           /* This field specifies the Test MUX 2 Selection lines. */
#define TEST_MUX02_TST_MUX_SELECT_09_Pos                 6
#define TEST_MUX02_TST_MUX_SELECT_09_Msk                 (0x3ful<<TEST_MUX02_TST_MUX_SELECT_09_Pos)           /* This field specifies the Test MUX 1 Selection lines. */
#define TEST_MUX02_TST_MUX_SELECT_08_Pos                 0
#define TEST_MUX02_TST_MUX_SELECT_08_Msk                 (0x3ful<<TEST_MUX02_TST_MUX_SELECT_08_Pos)            /* This field specifies the Test MUX 0 Selection lines. */
/*test_mux03*/
#define TEST_MUX03_TST_MUX_SELECT_15_Pos                 18
#define TEST_MUX03_TST_MUX_SELECT_15_Msk                 (0x3ful<<TEST_MUX03_TST_MUX_SELECT_15_Pos)        /* This field specifies the Test MUX 3 Selection lines. */
#define TEST_MUX03_TST_MUX_SELECT_14_Pos                 12
#define TEST_MUX03_TST_MUX_SELECT_14_Msk                 (0x3ful<<TEST_MUX03_TST_MUX_SELECT_14_Pos)           /* This field specifies the Test MUX 2 Selection lines. */
#define TEST_MUX03_TST_MUX_SELECT_13_Pos                 6
#define TEST_MUX03_TST_MUX_SELECT_13_Msk                 (0x3ful<<TEST_MUX03_TST_MUX_SELECT_13_Pos)           /* This field specifies the Test MUX 1 Selection lines. */
#define TEST_MUX03_TST_MUX_SELECT_12_Pos                 0
#define TEST_MUX03_TST_MUX_SELECT_12_Msk                 (0x3ful<<TEST_MUX03_TST_MUX_SELECT_12_Pos)            /* This field specifies the Test MUX 0 Selection lines. */

/*R00_CTL*/
#define R00_CTL_TX_PAYLOAD_LEN_Pos                       	24
#define R00_CTL_TX_PAYLOAD_LEN_Msk                          (0xFFul<<R00_CTL_TX_PAYLOAD_LEN_Pos)    /* [31:24] 8bits */
#define R00_CTL_RX_PAYLOAD_LEN_Pos                         	16
#define R00_CTL_RX_PAYLOAD_LEN_Msk                          (0xFFul<<R00_CTL_RX_PAYLOAD_LEN_Pos)    /* [23:16] 8bits */
#define R00_CTL_ACCADDR_CRCSRC_DIS_Pos                      15
#define R00_CTL_ACCADDR_CRCSRC_DIS_Msk                      (0x1ul<<R00_CTL_ACCADDR_CRCSRC_DIS_Pos) /* [15] 1bit */
#define R00_CTL_ADDR_BYTE_LEN_Pos                          	12
#define R00_CTL_ADDR_BYTE_LEN_Msk                           (0x3ul<<R00_CTL_ADDR_BYTE_LEN_Pos)      /* [13:12] 2bits*/
#define R00_CTL_DPY_EN_Pos                                 	11
#define R00_CTL_DPY_EN_Msk                                  (0x1ul<<R00_CTL_DPY_EN_Pos)             /* [11] 1bit*/
#define R00_CTL_CRC_EN_Pos                                 	10
#define R00_CTL_CRC_EN_Msk                                  (0x1ul<<R00_CTL_CRC_EN_Pos)             /* [10] 1bit*/
#define R00_CTL_CRC_SEL16_Pos                              	9
#define R00_CTL_CRC_SEL16_Msk                               (0x1ul<<R00_CTL_CRC_SEL16_Pos)          /* [9] 1bit*/
#define R00_CTL_SCR_EN_Pos                                 	8
#define R00_CTL_SCR_EN_Msk                                  (0x1ul<<R00_CTL_SCR_EN_Pos)             /* [8] 1bit*/
#define R00_CTL_NORDIC_ENHANCE_Pos                         	7
#define R00_CTL_NORDIC_ENHANCE_Msk                          (0x1ul<<R00_CTL_NORDIC_ENHANCE_Pos)     /* [7] 1bit*/
#define R00_CTL_ENHANCE_Pos                                	6
#define R00_CTL_ENHANCE_Msk                                 (0x1ul<<R00_CTL_ENHANCE_Pos)            /* [6] 1bit*/
#define R00_CTL_BW_MODE_Pos                                	5
#define R00_CTL_BW_MODE_Msk                                 (0x1ul<<R00_CTL_BW_MODE_Pos)            /* [5] 1bit*/
#define R00_CTL_CHIP_MODE_Pos                              	3
#define R00_CTL_CHIP_MODE_Msk                               (0x3ul<<R00_CTL_CHIP_MODE_Pos)          /* [4:3] 2bits*/
#define R00_CTL_RX_ACK_PAYLOAD_EN_Pos                      	2
#define R00_CTL_RX_ACK_PAYLOAD_EN_Msk                       (0x1ul<<R00_CTL_RX_ACK_PAYLOAD_EN_Pos)  /* [2] 1bit*/
#define R00_CTL_TX_NOACK_EN_Pos                            	1
#define R00_CTL_TX_NOACK_EN_Msk                             (0x1ul<<R00_CTL_TX_NOACK_EN_Pos)        /* [1] 1bit*/
#define R00_CTL_PRI_RX_Pos                                 	0
#define R00_CTL_PRI_RX_Msk                                  (0x1ul<<R00_CTL_PRI_RX_Pos)             /* [0] 1bit*/
/*R01_INT_CTL*/
#define R01_INT_PRI_RX_PID_Pos                             29                                      
#define R01_INT_PRI_RX_PID_Msk                              (0x3ul<<R01_INT_PRI_RX_PID_Pos)
#define R01_INT_PRI_TX_PID_Pos                             27
#define R01_INT_PRI_TX_PID_Msk                              (0x3ul<<R01_INT_PRI_TX_PID_Pos)
#define R01_INT_RX_PID_MANUAL_Pos                          25
#define R01_INT_RX_PID_MANUAL_Msk                           (0x3ul<<R01_INT_RX_PID_MANUAL_Pos)
#define R01_INT_PRI_ENDIAN_Pos                             21
#define R01_INT_PRI_ENDIAN_Msk                              (0x1ul<<R01_INT_PRI_ENDIAN_Pos)
#define R01_INT_PRI_TX_DONE_IRQ_EN_Pos                     20
#define R01_INT_PRI_TX_DONE_IRQ_EN_Msk                      (0x1ul<<R01_INT_PRI_TX_DONE_IRQ_EN_Pos)
#define R01_INT_PRI_RX_DONE_IRQ_EN_Pos                     19
#define R01_INT_PRI_RX_DONE_IRQ_EN_Msk                      (0x1ul<<R01_INT_PRI_RX_DONE_IRQ_EN_Pos)
#define R01_INT_PRI_RX_GOON_Pos                            18
#define R01_INT_PRI_RX_GOON_Msk                             (0x1ul<<R01_INT_PRI_RX_GOON_Pos)
#define R01_INT_PRI_RST_Pos                                17
#define R01_INT_PRI_RST_Msk                                 (0x1ul<<R01_INT_PRI_RST_Pos)
#define R01_INT_CTL_EXIT_RX_Pos                             16
#define R01_INT_CTL_EXIT_RX_Msk                             (0x1ul<<R01_INT_CTL_EXIT_RX_Pos)
#define R01_INT_TX_DONE_IRQ_FLAG_Pos                       15
#define R01_INT_TX_DONE_IRQ_FLAG_Msk                        (0x1ul<<R01_INT_TX_DONE_IRQ_FLAG_Pos)
#define R01_INT_RX_DONE_IRQ_FLAG_Pos                       14
#define R01_INT_RX_DONE_IRQ_FLAG_Msk                        (0x1ul<<R01_INT_RX_DONE_IRQ_FLAG_Pos)
#define R01_INT_RX_PID_ERR_IRQ_FLAG_Pos                    13
#define R01_INT_RX_PID_ERR_IRQ_FLAG_Msk                     (0x1ul<<R01_INT_RX_PID_ERR_IRQ_FLAG_Pos)
#define R01_INT_CTL_RX_CRC_ERR_Pos                    		  12		
#define R01_INT_CTL_RX_CRC_ERR_Msk                     		  (0x1ul<<R01_INT_CTL_RX_CRC_ERR_Pos)     		/* [12] 1bit*/
#define R01_INT_CTL_TX_TIMEOUT_IRQ_Pos                      11		
#define R01_INT_CTL_TX_TIMEOUT_IRQ_Msk                      (0x1ul<<R01_INT_CTL_TX_TIMEOUT_IRQ_Pos) 		/* [11] 1bit*/
#define R01_INT_CTL_TX_IRQ_Pos                            	10		
#define R01_INT_CTL_TX_IRQ_Msk                             	(0x1ul<<R01_INT_CTL_TX_IRQ_Pos)         		/* [10] 1bit*/
#define R01_INT_CTL_RX_IRQ_Pos                            	9		
#define R01_INT_CTL_RX_IRQ_Msk                              (0x1ul<<R01_INT_CTL_RX_IRQ_Pos)         		/* [9] 1bit*/
#define R01_INT_CTL_IRQ_CLR_EN_Pos                          8		
#define R01_INT_CTL_IRQ_CLR_EN_Msk                          (0x1ul<<R01_INT_CTL_IRQ_CLR_EN_Pos)     		/* [8] 1bit*/
#define R01_INT_CTL_RX_CRC_ERR_MASK_Pos                     7	
#define R01_INT_CTL_RX_CRC_ERR_MASK_Msk                     (0x1ul<<R01_INT_CTL_RX_CRC_ERR_MASK_Pos)  	/* [7] 1bit*/
#define R01_INT_CTL_TX_TIMEOUT_IRQ_MASK_Pos              	5
#define R01_INT_CTL_TX_TIMEOUT_IRQ_MASK_Msk               	(0x1ul<<R01_INT_CTL_TX_TIMEOUT_IRQ_MASK_Pos)  /* [5] 1bit*/
#define R01_INT_CTL_TX_IRQ_MASK_Pos                         4
#define R01_INT_CTL_TX_IRQ_MASK_Msk                         (0x1ul<<R01_INT_CTL_TX_IRQ_MASK_Pos)          /* [4] 1bit*/
#define R01_INT_CTL_RX_IRQ_MASK_Pos                         3
#define R01_INT_CTL_RX_IRQ_MASK_Msk                         (0x1ul<<R01_INT_CTL_RX_IRQ_MASK_Pos)          /* [3] 1bit*/
#define R01_INT_CTL_TX_PID_MANUAL_EN_Pos                    2
#define R01_INT_CTL_TX_PID_MANUAL_EN_Msk                    (0x1ul<<R01_INT_CTL_TX_PID_MANUAL_EN_Pos)     /* [2] 1bit*/
#define R01_INT_CTL_TX_PID_MANUAL_Pos                       0
#define R01_INT_CTL_TX_PID_MANUAL_Msk                       (0x3ul<<R01_INT_CTL_TX_PID_MANUAL_Pos)        /* [1:0] 2bits*/
/*R02_TMR_CTL*/
#define R02_TMR_CTL_RX_WAIT_TIME_Pos                        16
#define R02_TMR_CTL_RX_WAIT_TIME_Msk                        (0xFFFFul<<R02_TMR_CTL_RX_WAIT_TIME_Pos)     	   /* [31:16] 16bits*/
#define R02_TMR_CTL_RX_WAIT_TIME_EN_Pos                     15
#define R02_TMR_CTL_RX_WAIT_TIME_EN_Msk                     (0X1ul<<R02_TMR_CTL_RX_WAIT_TIME_EN_Pos)           /* [15] 1bit*/
#define R02_TMR_CTL_TRX_TRANS_WAIT_TIME_Pos                 0
#define R02_TMR_CTL_TRX_TRANS_WAIT_TIME_Msk                 (0x7FFFul<<R02_TMR_CTL_TRX_TRANS_WAIT_TIME_Pos)  /* [14:0] 15bits*/
/*R03_RX_ADDR_L*/
#define R03_RX_ADDR_L_L32B_Pos                            	0
#define R03_RX_ADDR_L_L32B_Msk                             	(0xFFFFFFFFul<<R03_RX_ADDR_L_L32B_Pos)       	   /* [31:0] 32bits*/
/*R04_RX_ADDR_M*/

#define R04_RX_ADDR_M_RX_HEADER_Pos                         16
#define R04_RX_ADDR_M_RX_HEADER_Msk                          (0xFFFFul<<R04_RX_ADDR_M_RX_HEADER_Pos)            /* [16:31] 1bits*/
#define R04_RX_ADDR_M_NORMAL_M1_Pos                         11
#define R04_RX_ADDR_M_NORMAL_M1_Msk                          (0x1ul<<R04_RX_ADDR_M_NORMAL_M1_Pos)            /* [11] 1bits*/
#define R04_RX_ADDR_M_HDR_LEN_NUMB_Pos                      9
#define R04_RX_ADDR_M_HDR_LEN_NUMB_Msk                       (0x3ul<<R04_RX_ADDR_M_HDR_LEN_NUMB_Pos)            /* [9:10] 2bits*/
#define R04_RX_ADDR_M_HDR_LEN_EXIST_Pos                     8
#define R04_RX_ADDR_M_HDR_LEN_EXIST_Msk                      (0x1ul<<R04_RX_ADDR_M_HDR_LEN_EXIST_Pos)            /* [8] 1bits*/
#define R04_RX_ADDR_M_M32B_Pos                            	0
#define R04_RX_ADDR_M_M32B_Msk                             	(0xFFul<<R04_RX_ADDR_M_M32B_Pos)            		   /* [7:0] 8bits*/
/*R05_TX_ADDR_L*/																									   
#define R05_TX_ADDR_L_L32B_Pos                            	0		                                       
#define R05_TX_ADDR_L_L32B_Msk                             	(0xFFFFFFFFul<<R05_TX_ADDR_L_L32B_Pos)      		   /* [31:0] 32bits*/
/*R06_TX_ADDR_M*/																										   
#define R06_TX_ADDR_M_M32B_Pos                           	0		                                       
#define R06_TX_ADDR_M_M32B_Msk                             	(0xFFul<<R06_TX_ADDR_M_M32B_Pos)            		   /* [7:0] 8bits*/
/*R07_SRAM_CTL*/
#define R07_SRAM_CTL_RX_START_ADDR_Pos                      17
#define R07_SRAM_CTL_RX_START_ADDR_Msk                      (0x7FFul<<R07_SRAM_CTL_RX_START_ADDR_Pos)        /* [7:0] 8bits*/
#define R07_SRAM_CTL_RX_READY_Pos                           16
#define R07_SRAM_CTL_RX_READY_Msk                           (0x1ul<<R07_SRAM_CTL_RX_READY_Pos)               /* [7:0] 8bits*/
#define R07_SRAM_CTL_TX_START_ADDR_Pos                      1
#define R07_SRAM_CTL_TX_START_ADDR_Msk                      (0x7FFul<<R07_SRAM_CTL_TX_START_ADDR_Pos)        /* [7:0] 8bits*/
#define R07_SRAM_CTL_TX_READY_Pos                           0
#define R07_SRAM_CTL_TX_READY_Msk                           (0x1<<R07_SRAM_CTL_TX_READY_Pos)               /* [7:0] 8bits*/
/**@}*/


/** @addtogroup Peripheral_memory_map
  * @{
  */

/******************************************************************************/
/*                            Peripheral memory map                           */
/******************************************************************************/

#define CR_BASE 		  	   (0x00400000Ul)
#define SRAM_BASE              (0x20000000UL)

#define AHB1_PERIPH_BASE       (0x40000000UL)
//#define AHB2_PERIPH_BASE       (0x50000000UL)

#define APB1_PERIPH_BASE       (0x40000000UL)
#define APB2_PERIPH_BASE       (0x40010000UL)

#define BLE_PERIPH_BASE        (0x50020000UL)

/******************************************************************************/
/*                             AHB1--APB1                                     */
/******************************************************************************/
#define I2C0_BASE              (APB1_PERIPH_BASE + 0x00000000UL) //0x40000000-0x40000FFF
#define SPI0_BASE              (APB1_PERIPH_BASE + 0x00001000UL) //0x40001000-0x40001FFF
#define UART0_BASE             (APB1_PERIPH_BASE + 0x00003000UL) //0x40003000-0x40003FFF
#define PWM0_BASE              (APB1_PERIPH_BASE + 0x00004000UL) //0x40004000-0x40004FFF
#define ADC_BASE               (APB1_PERIPH_BASE + 0x00005000UL) //0x40005000-0x40005FFF
#define WDT_BASE               (APB1_PERIPH_BASE + 0x00006000UL) //0x40006000-0x40006FFF
#define WWDT_BASE              (APB1_PERIPH_BASE + 0x00007000UL) //0x40007000-0x40007FFF
#define TIMER0_BASE            (APB1_PERIPH_BASE + 0x00008000UL) //0x40008000-0x40008FFF
#define I2S_MST_BASE           (APB1_PERIPH_BASE + 0x00009000UL) //0x40009000-0x40009FFF
#define I2S_SLV_BASE           (APB1_PERIPH_BASE + 0x0000A000UL) //0x4000A000-0x4000AFFF
#define PWM1_BASE              (APB1_PERIPH_BASE + 0x0000b000UL) //0x4000b000-0x4000AFFF
#define PWM2_BASE              (APB1_PERIPH_BASE + 0x0000c000UL) //0x4000c000-0x4000AFFF
/******************************************************************************/
/*                             AHB1--APB2                                     */
/******************************************************************************/
#define SPI1_BASE              (APB2_PERIPH_BASE + 0x00001000UL) //0x40011000-0x40011FFF
#define UART1_BASE             (APB2_PERIPH_BASE + 0x00003000UL) //0x40013000-0x40013FFF
#define TIMER1_BASE            (APB2_PERIPH_BASE + 0x00004000UL) //0x40014000-0x40014FFF
#define TIMER2_BASE            (APB2_PERIPH_BASE + 0x00005000UL) //0x40015000-0x40015FFF
#define TRIM_BASE              (APB2_PERIPH_BASE + 0x00006000UL) //0x40016000-0x40016FFF
#define KSCAN_BASE             (APB2_PERIPH_BASE + 0x00009000UL) //0x40019000-0x40019FFF
#define QDEC_BASE              (APB2_PERIPH_BASE + 0x0000A000UL) //0x4001A000-0x4001AFFF


/******************************************************************************/
/*                             AHB2                                           */
/******************************************************************************/
#define GPIO_BASE              (AHB1_PERIPH_BASE + 0x00020000UL) //0x40020000-0x4002FFFF
#define P0_BASE                (GPIO_BASE        + 0x00000000UL) //0x40020000-0x4002000F
#define P1_BASE                (GPIO_BASE        + 0x00000040UL) //0x40020010-0x4002001F
#define P2_BASE                (GPIO_BASE        + 0x00000080UL) //0x40020020-0x4002002F
#define P3_BASE                (GPIO_BASE        + 0x000000C0UL) //0x40020030-0x4002003F
#define P4_BASE                (GPIO_BASE        + 0x00000100UL) //0x40020040-0x4002004F
#define P5_BASE                (GPIO_BASE        + 0x00000140UL) //0x40020050-0x4002005F
#define GPIO_DBNCECON_BASE     (GPIO_BASE        + 0x00000180UL) //0x40020060-0x4002006F
#define GPIOBIT0_BASE          (GPIO_BASE        + 0x00000200UL) //0x40020080-0x40020087
#define GPIOBIT1_BASE          (GPIO_BASE        + 0x00000220UL) //0x40020088-0x4002008F
#define GPIOBIT2_BASE          (GPIO_BASE        + 0x00000240UL) //0x40020090-0x40020097
#define GPIOBIT3_BASE          (GPIO_BASE        + 0x00000260UL) //0x40020098-0x4002009F
#define GPIOBIT4_BASE          (GPIO_BASE        + 0x00000280UL) //0x400200A0-0x400200A7
#define GPIOBIT5_BASE          (GPIO_BASE        + 0x000002A0UL) //0x400200A8-0x400200AF

#define SYS_BASE               (AHB1_PERIPH_BASE + 0x00030000UL) //0x40030000-0x4003FFFF
#define CLK_BASE               (AHB1_PERIPH_BASE + 0x00040000UL) //0x40040000-0x4004FFFF
#define FLCTL_BASE             (AHB1_PERIPH_BASE + 0x00050000UL) //0x40050000-0x4005FFFF
#define FLCTL_BASE1            (AHB1_PERIPH_BASE + 0x00050400UL) //0x40050400-0x4005FFFF
#define DMA_BASE               (AHB1_PERIPH_BASE + 0x00060000UL) //0x40060000-0x4006FFFF
#define ANA_BASE               (AHB1_PERIPH_BASE + 0x00070000UL) //0x40070000-0x4007FFFF
#define EFUSE_BASE             (AHB1_PERIPH_BASE + 0x00080000UL) //0x40080000-0x4008FFFF
#define ACC_BASE               (AHB1_PERIPH_BASE + 0x00090000UL) //0x40090000-0x4008FFFF
#define USB_BASE               (AHB1_PERIPH_BASE + 0x000A0000UL) //0x400a0000-0x400a01FF
#define USBDMA_BASE            (AHB1_PERIPH_BASE + 0x000A0200UL) //0x400a0200-0x400aFFFF


#define PRI_RF_BASE            (BLE_PERIPH_BASE  + 0X00000400UL) //0x50020400-0x500204e4
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define CR                	   ((CR_T *) CR_BASE)

/*APB1*/
#define I2C0                   ((I2C_T *)         I2C0_BASE     )
#define SPI0                   ((SPI_T *)         SPI0_BASE     )
#define UART0                  ((UART_T *)        UART0_BASE    )
#define PWM0                   ((PWM_T *)         PWM0_BASE     )
#define ADC                    ((ADC_T *)         ADC_BASE      )
#define WDT                    ((WDT_T *)         WDT_BASE      )
#define WWDT                   ((WWDT_T *)        WWDT_BASE     )
#define TIMER0                 ((TIMER_T *)       TIMER0_BASE   )
#define I2S_M                  ((I2S_T *)         I2S_MST_BASE  )
#define I2S_S                  ((I2S_T *)         I2S_SLV_BASE  )
#define PWM1                   ((PWM_T *)         PWM1_BASE     )
#define PWM2                   ((PWM_T *)         PWM2_BASE     )


/*APB2*/
#define SPI1                   ((SPI_T *)         SPI1_BASE     )
#define UART1                  ((UART_T *)        UART1_BASE    )
#define TIMER1                 ((TIMER_T *)       TIMER1_BASE   )
#define TIMER2                 ((TIMER_T *)       TIMER2_BASE   )
#define TRIM                   ((TRIM_T *)        TRIM_BASE     )
#define KSCAN                  ((KSCAN_T *)       KSCAN_BASE    )
#define QDEC                   ((QDEC_T *)        QDEC_BASE     )
/*AHB2*/
#define GPIO                   ((GPIO_T *)        GPIO_BASE     )
#define P0                     ((GPIO_T *)        P0_BASE       )
#define P1                     ((GPIO_T *)        P1_BASE       )
#define P2                     ((GPIO_T *)        P2_BASE       )
#define P3                     ((GPIO_T *)        P3_BASE       )
#define P4                     ((GPIO_T *)        P4_BASE       )
#define P5                     ((GPIO_T *)        P5_BASE       )
#define GPIO_DB                ((GPIO_DB_T *) GPIO_DBNCECON_BASE)  ///< Pointer to GPIO de-bounce register structure

#define SYS                    ((SYS_T  *)        SYS_BASE      )
#define CLK                    ((CLK_T  *)        CLK_BASE      )
#define FLCTL              	   ((FLCTL_T *)       FLCTL_BASE    )
#define FLCTL_BUFF             ((FLCTL_W_BUFF_T *)FLCTL_BASE1   )
#define DMA                    ((DMA_T *)         DMA_BASE      )
#define ANA                    ((ANA_T *)         ANA_BASE      )
#define EFUSE                  ((EFUSE_T *)       EFUSE_BASE    )
#define ACC                    ((ACC_T *)         ACC_BASE	    )
#define USB                    ((USB_T *)         USB_BASE      )
#define USBDMA                 ((USBDMA_T  *)     USBDMA_BASE   )


#define PRI_RF                 ((PRI_RF_T *)      PRI_RF_BASE   )

/**
  * @}
  */


/** @addtogroup Exported_macro
  * @{
  */
#ifndef NULL
#define NULL           (0)      ///< NULL pointer
#endif


/*
#define ENABLE         (1)      ///< Enable, define to use in API parameters
#define DISABLE        (0)      ///< Disable, define to use in API parameters
*/
/* Define one bit mask */
#define BIT0     (0x00000001UL)       ///< Bit 0 mask of an 32 bit integer
#define BIT1     (0x00000002UL)       ///< Bit 1 mask of an 32 bit integer
#define BIT2     (0x00000004UL)       ///< Bit 2 mask of an 32 bit integer
#define BIT3     (0x00000008UL)       ///< Bit 3 mask of an 32 bit integer
#define BIT4     (0x00000010UL)       ///< Bit 4 mask of an 32 bit integer
#define BIT5     (0x00000020UL)       ///< Bit 5 mask of an 32 bit integer
#define BIT6     (0x00000040UL)       ///< Bit 6 mask of an 32 bit integer
#define BIT7     (0x00000080UL)       ///< Bit 7 mask of an 32 bit integer
#define BIT8     (0x00000100UL)       ///< Bit 8 mask of an 32 bit integer
#define BIT9     (0x00000200UL)       ///< Bit 9 mask of an 32 bit integer
#define BIT10    (0x00000400UL)       ///< Bit 10 mask of an 32 bit integer
#define BIT11    (0x00000800UL)       ///< Bit 11 mask of an 32 bit integer
#define BIT12    (0x00001000UL)       ///< Bit 12 mask of an 32 bit integer
#define BIT13    (0x00002000UL)       ///< Bit 13 mask of an 32 bit integer
#define BIT14    (0x00004000UL)       ///< Bit 14 mask of an 32 bit integer
#define BIT15    (0x00008000UL)       ///< Bit 15 mask of an 32 bit integer
#define BIT16    (0x00010000UL)       ///< Bit 16 mask of an 32 bit integer
#define BIT17    (0x00020000UL)       ///< Bit 17 mask of an 32 bit integer
#define BIT18    (0x00040000UL)       ///< Bit 18 mask of an 32 bit integer
#define BIT19    (0x00080000UL)       ///< Bit 19 mask of an 32 bit integer
#define BIT20    (0x00100000UL)       ///< Bit 20 mask of an 32 bit integer
#define BIT21    (0x00200000UL)       ///< Bit 21 mask of an 32 bit integer
#define BIT22    (0x00400000UL)       ///< Bit 22 mask of an 32 bit integer
#define BIT23    (0x00800000UL)       ///< Bit 23 mask of an 32 bit integer
#define BIT24    (0x01000000UL)       ///< Bit 24 mask of an 32 bit integer
#define BIT25    (0x02000000UL)       ///< Bit 25 mask of an 32 bit integer
#define BIT26    (0x04000000UL)       ///< Bit 26 mask of an 32 bit integer
#define BIT27    (0x08000000UL)       ///< Bit 27 mask of an 32 bit integer
#define BIT28    (0x10000000UL)       ///< Bit 28 mask of an 32 bit integer
#define BIT29    (0x20000000UL)       ///< Bit 29 mask of an 32 bit integer
#define BIT30    (0x40000000UL)       ///< Bit 30 mask of an 32 bit integer
#define BIT31    (0x80000000UL)       ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk              (0x000000FF)         ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk              (0x0000FF00)         ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk              (0x00FF0000)         ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk              (0xFF000000)         ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/**
  * @}
  */


#ifdef  USE_FULL_ASSERT

  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);

#else

  #define assert_param(expr) ((void)0)

#endif


#ifndef CONFIG_HAS_PANCHIP_PLATFORM
#include "pan_acc.h"
#include "pan_adc.h"
#include "pan_clk.h"
#include "pan_clktrim.h"
#include "pan_dmac.h"
#include "pan_efuse.h"
#include "pan_fmc.h"
#include "pan_gpio.h"
#include "pan_i2c.h"
#include "pan_i2s.h"
#include "pan_kscan.h"
#include "pan_lp.h"
#include "pan_pwm.h"
#include "pan_qdec.h"
#include "pan_spi.h"
#include "pan_sys.h"
#include "pan_sys_log.h"
#include "pan_timer.h"
#include "pan_uart.h"
#include "pan_wdt.h"
#include "pan_wwdt.h"
#endif /* CONFIG_HAS_PANCHIP_PLATFORM */

#endif /* __PANSERIES_H__ */
