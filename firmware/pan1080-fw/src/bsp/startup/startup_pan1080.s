
  .syntax unified
	.cpu cortex-m0
	.fpu softvfp
	.thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
  movs r3, #0
  str  r3, [r2]
  adds r2, r2, #4


LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* Call the clock system initialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  0
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */  
  .word  I2C0_IRQHandler                        /* 16+ 0: SSP 0 Handler                   */
  .word  SPI0_IRQHandler                        /* 16+ 1: SSP 1 Handler                   */
  .word  I2S_IRQHandler                         /* 16+ 2: UART 0 Handler                  */
  .word  UART0_IRQHandler                       /* 16+ 3: UART 1 Handler                  */
  .word  PWM0_IRQHandler                        /* 16+ 4: UART 2 Handler                  */
  .word  ADC_IRQHandler                         /* 16+ 5: I2C 0 Handler                   */
  .word  WDT_IRQHandler                         /* 16+ 6: I2C 1 Handler                   */
  .word  WWDT_IRQHandler                        /* 16+ 6: I2C 1 Handler                   */
  .word  TMR0_IRQHandler                        /* 16+ 7: GPIO Port 0 Combined Handler    */
  .word  ACC_IRQHandler                         /* 16+ 8: GPIO Port 1 Combined Handler    */
  .word  SPI1_IRQHandler                        /* 16+ 9: GPIO Port 2 Combined Handler    */
  .word  LL_IRQHandler                          /* 16+10: GPIO Port 3 Combined Handler    */
  .word  UART1_IRQHandler		                    /* 16+11: DMA Combined Handler            */
  .word  TMR1_IRQHandler                        /* 16+12: Dual timer 0 handler            */ 
  .word  TMR2_IRQHandler		                    /* 16+ 13: Dual timer 1 Handler	          */
  .word  TRIM_IRQHandler		                    /* 16+ 14: PWM0 Handler		                */
  .word  KSCAN_IRQHandler		                    /* 16+ 15: PWM1 Handler		                */
  .word  QDEC_IRQHandler		                    /* 16+ 16: PWM2 Handler		                */
  .word  GPIO_IRQHandler		                    /* 16+ 17: PWM3 Handler		                */
  .word  PWM1_IRQHandler		                    /* 16+ 18: PWM4 Handler		                */
  .word  PWM2_IRQHandler		                    /* 16+ 19: PWM5 Handler		                */
  .word  USBDMA_IRQHandler		                  /* 16+ 20: PWM6 Handler		                */
  .word  USB_IRQHandler		                      /* 16+ 21: PWM7 Handler		                */
  .word  RSVD_IRQHandler		                    /* 16+ 22: RTC Handler			              */
  .word  EXT0_IRQHandler		                    /* 16+ 23: ADC Handler			              */
  .word  EXT1_IRQHandler                        /* 16+ 24: WZTOE Handler		              */
  .word  EXT2_IRQHandler                        /* 16+ 25: EXTI Handler                   */
  .word  DMA_IRQHandler                         /* 16+ 25: EXTI Handler                   */
  .word  BOD_IRQHandler                         /* 16+ 25: EXTI Handler                   */
  .word  SLEEP_IRQHandler                       /* 16+ 25: EXTI Handler                   */
  .word  STANDBY_IRQHandler                     /* 16+ 25: EXTI Handler                   */
  .word  MODEM_IRQHandler                       /* 16+ 25: EXTI Handler                   */


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  /* External Interrupts */  
  .weak      I2C0_IRQHandler
  .thumb_set I2C0_IRQHandler,Default_Handler

  .weak      SPI0_IRQHandler
  .thumb_set SPI0_IRQHandler,Default_Handler

  .weak      I2S_IRQHandler
  .thumb_set I2S_IRQHandler,Default_Handler

  .weak      UART0_IRQHandler
  .thumb_set UART0_IRQHandler,Default_Handler

  .weak      PWM0_IRQHandler
  .thumb_set PWM0_IRQHandler,Default_Handler

  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak      WDT_IRQHandler
  .thumb_set WDT_IRQHandler,Default_Handler

  .weak      WWDT_IRQHandler
  .thumb_set WWDT_IRQHandler,Default_Handler

  .weak      TMR0_IRQHandler
  .thumb_set TMR0_IRQHandler,Default_Handler

  .weak      ACC_IRQHandler
  .thumb_set ACC_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      LL_IRQHandler
  .thumb_set LL_IRQHandler,Default_Handler

  .weak      UART1_IRQHandler
  .thumb_set UART1_IRQHandler,Default_Handler

  .weak      TMR1_IRQHandler
  .thumb_set TMR1_IRQHandler,Default_Handler

  .weak      TMR2_IRQHandler
  .thumb_set TMR2_IRQHandler,Default_Handler

  .weak      TRIM_IRQHandler
  .thumb_set TRIM_IRQHandler,Default_Handler

  .weak      KSCAN_IRQHandler
  .thumb_set KSCAN_IRQHandler,Default_Handler

  .weak      GPIO_IRQHandler
  .thumb_set GPIO_IRQHandler,Default_Handler

  .weak      PWM1_IRQHandler
  .thumb_set PWM1_IRQHandler,Default_Handler

  .weak      PWM2_IRQHandler
  .thumb_set PWM2_IRQHandler,Default_Handler

  .weak      USBDMA_IRQHandler
  .thumb_set USBDMA_IRQHandler,Default_Handler

  .weak      USB_IRQHandler
  .thumb_set USB_IRQHandler,Default_Handler  

  .weak      RSVD_IRQHandler
  .thumb_set RSVD_IRQHandler,Default_Handler  

  .weak      EXT0_IRQHandler
  .thumb_set EXT0_IRQHandler,Default_Handler    

  .weak      EXT1_IRQHandler
  .thumb_set EXT1_IRQHandler,Default_Handler    

  .weak      EXT2_IRQHandler
  .thumb_set EXT2_IRQHandler,Default_Handler    

  .weak      DMA_IRQHandler
  .thumb_set DMA_IRQHandler,Default_Handler    

  .weak      BOD_IRQHandler
  .thumb_set BOD_IRQHandler,Default_Handler    

  .weak      SLEEP_IRQHandler
  .thumb_set SLEEP_IRQHandler,Default_Handler    

  .weak      STANDBY_IRQHandler
  .thumb_set STANDBY_IRQHandler,Default_Handler    

  .weak      MODEM_IRQHandler
  .thumb_set MODEM_IRQHandler,Default_Handler    
