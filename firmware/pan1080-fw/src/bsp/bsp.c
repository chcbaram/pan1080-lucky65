#include "bsp.h"


extern void swtimerISR(void);

static volatile uint32_t systick_counter = 0;

void SysTick_Handler(void)
{
  systick_counter++;
  // swtimerISR();
}


bool bspInit(void)
{
  volatile uint32_t system_core_clock;

  CLK_RefClkSrcConfig(CLK_SYS_SRCSEL_XTH);
  CLK_PCLK1Config(1);
  CLK_PCLK2Config(1);

  CLK_SYSCLKConfig(CLK_DPLL_REF_CLKSEL_XTH, CLK_DPLL_OUT_64M);
  CLK_RefClkSrcConfig(CLK_SYS_SRCSEL_DPLL);

  // Peripheral Clock Enable
  CLK_AHBPeriphClockCmd(CLK_AHBPeriph_All, ENABLE);
  CLK_APB1PeriphClockCmd(CLK_APB1Periph_All, ENABLE);
  CLK_APB2PeriphClockCmd(CLK_APB2Periph_All, ENABLE);

  system_core_clock = CLK_GetCPUFreq();

  SysTick_Config(system_core_clock/1000);  
  NVIC_SetPriority(SysTick_IRQn, 0);

  return true;
}

void delay(uint32_t time_ms)
{
  uint32_t pre_time = systick_counter;

  while(systick_counter-pre_time < time_ms);  
}

uint32_t millis(void)
{
  return systick_counter;
}



