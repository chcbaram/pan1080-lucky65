
/**************************************************************************//**
* @file     pan_hal_gpio.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series GPIO (General Purpose Input/Output) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#include "pan_hal.h"

void HAL_GPIO_Init(GPIO_Pin_Id PinID, GPIO_Init_Opt *GPIO_InitObj)
{
  uint32_t position = 0x00u;

  uint8_t port = PinID >> 4;
  // uint8_t bit = PinID - (PinID >> 4);
  uint8_t bit = PinID % 0x10;

  GPIO_T *GPIOx = (GPIO_T *)(P0_BASE + 0x40 * port);
  position = 1UL << bit;
  /*--------------------- GPIO Mode Configuration ------------------------*/
  GPIOx->MODE = (GPIOx->MODE & ~(0x3 << (bit << 1))) | (GPIO_InitObj->Mode << (bit << 1));

  /*--------------------- GPIO Pull Configuration ------------------------*/
  if ((GPIO_InitObj->Mode == HAL_GPIO_MODE_INPUT_DIGITAL) || (GPIO_InitObj->Mode == HAL_GPIO_MODE_QUASI))
    GPIOx->DINOFF &= ~(position << 16);
  else if (GPIO_InitObj->Mode == HAL_GPIO_MODE_INPUT_ANALOG)
    GPIOx->DINOFF |= (position << 16);

  switch (GPIO_InitObj->Pull)
  {
  case HAL_PULL_UP:
    GPIOx->DINOFF &= ~(position << 8);
    GPIOx->DINOFF |= position;
    break;
  case HAL_PULL_DOWN:
    GPIOx->DINOFF &= ~position;
    GPIOx->DINOFF |= (position << 8);
    break;
  case HAL_PULL_DISABLE:
    GPIOx->DINOFF &= ~position;
    GPIOx->DINOFF &= ~(position << 8);
    break;
  }

  /*--------------------- GPIO Debounce Configuration ------------------------*/
  if (GPIO_InitObj->Debounce == ENABLE)
    GPIOx->DBEN |= position;
  else if (GPIO_InitObj->Debounce == DISABLE)
    GPIOx->DBEN &= ~position;

  /*--------------------- Support auto_3V sync ------------------------*/
  if (GPIO_InitObj->Mode == HAL_GPIO_MODE_INPUT_DIGITAL || GPIO_InitObj->Mode == HAL_GPIO_MODE_QUASI || GPIO_InitObj->Pull == HAL_PULL_UP || GPIO_InitObj->Pull == HAL_PULL_DOWN)
  {
    if (((GPIOx == P5) && (position & (1 << 6))) || ((GPIOx == P4) && (position & (1 << 6))) || ((GPIOx == P4) && (position & (1 << 7))))
    {
      // These Pins' DINOFF bits should be synced to 3v area
      CLK_Wait3vSyncReady();
    }
  }
}

void HAL_GPIO_WritePin(GPIO_Pin_Id PinID, GPIO_Pinstate_Opt PinState)
{
    uint8_t port = PinID >> 4;
    // uint8_t bit = PinID - (PinID >> 4);
    uint8_t bit = PinID % 0x10;
    uint32_t Data;

    Data = GPIO_GetOutData((GPIO_T *)(P0_BASE + 0x40 * port));
    Data = (Data & ~(BIT0 << bit)) | PinState << bit;

    GPIO_SetOutData((GPIO_T *)(P0_BASE + 0x40 * port), Data);
}

GPIO_Pinstate_Opt HAL_GPIO_ReadPin(GPIO_Pin_Id PinID)
{
    GPIO_Pinstate_Opt bitstatus;
    uint8_t port = PinID >> 4;
    // uint8_t bit = PinID - (PinID >> 4);
    uint8_t bit = PinID % 0x10;

    if ((GPIO_GetInData((GPIO_T *)(P0_BASE + 0x40 * port)) & (BIT0 << bit)))
    {
        bitstatus = HAL_GPIO_PIN_SET;
    }
    else
    {
        bitstatus = HAL_GPIO_PIN_RESET;
    }
    return bitstatus;
}

void HAL_GPIO_DeInit(GPIO_Pin_Id PinID)
{
    GPIO_Init_Opt GPIO_InitObj;
    GPIO_InitObj.Mode = HAL_GPIO_MODE_INPUT_ANALOG;
    GPIO_InitObj.Pull = HAL_PULL_DISABLE;
    HAL_GPIO_Init(PinID, &GPIO_InitObj);
}

uint8_t HAL_GPIO_SetDebounceTime(Debounce_Time_Opt time)
{
	SystemCoreClock = FREQ_64MHZ;
  return 0;
}

GPIO_CallbackFunc GPIO_CallbackFunc_Array[48];

void HAL_GPIO_Init_INT(GPIO_Pin_Id PinID, GPIO_Interrupt_Opt *GPIO_InterruptObj)
{
    uint8_t port = PinID >> 4;
    // uint8_t bit = PinID - (PinID >> 4);
    uint8_t bit = PinID % 0x10;
    GPIO_T *GPIOx = (GPIO_T *)(P0_BASE + 0x40 * port);

    GPIO_CallbackFunc_Array[port * GPIO_PIN_MAX + bit] = GPIO_InterruptObj->CallbackFunc;

    /*--------------------- GPIO Interrupt Configuration ------------------------*/
    if (GPIO_InterruptObj->Mode == HAL_GPIO_INT_DISABLE)
    {
      GPIOx->INTTYPE &= ~(1UL << bit);
      GPIOx->INTEN &= ~((0x00010001UL) << bit);
    }
    else
    {
      GPIOx->INTTYPE |= (((GPIO_InterruptObj->Mode >> 24) & 0xFFUL) << bit);
      GPIOx->INTEN |= ((GPIO_InterruptObj->Mode & 0xFFFFFFUL) << bit);
    }
}

void GPIO_IRQHandler(void)
{
    /* To see which interrupt occurred in GPIO Port0 ~ Port5 */
    for (size_t port = 0; port < 6; port++)
    {
      GPIO_T *gpio = (GPIO_T *)(P0_BASE + 0x40 * port);

      if (GPIO_GetIntFlag(gpio, 0xFF) != 0)
      {
        for (size_t i = 0; i < 8; i++)
        {
          if (GPIO_GetIntFlag(gpio, BIT(i)))
          {
            GPIO_ClrIntFlag(gpio, BIT(i));
            uint32_t Attr = ((gpio->INTEN & GPIO_INTEN_MASK_BASE << i) >> i) |
                            ((gpio->INTTYPE & GPIO_INTTYPE_MASK_BASE << i) >> i << 24);
            if (Attr == GPIO_INT_HIGH || Attr == GPIO_INT_LOW)
            {
              GPIO_DisableInt(gpio, i); // Disable INT of curr pin to avoid endless Level Trigger
            }
            if (GPIO_CallbackFunc_Array[port * GPIO_PIN_MAX + i] != NULL)
            {
              GPIO_CallbackFunc_Array[port * GPIO_PIN_MAX + i]();
            }
          }
        }
        GPIO_ClrAllIntFlag(gpio);
      }
    }
}
