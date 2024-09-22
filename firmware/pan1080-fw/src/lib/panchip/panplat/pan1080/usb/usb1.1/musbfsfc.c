/**************************************************************************/ /**
                                                                              * @file     musbfsfc.c
                                                                              * @version  V1.00
                                                                              * $Revision: 2 $
                                                                              * $Date: 20/08/04 14:25 $
                                                                              * @brief    Interrupt handler for MUSBFSFC firmware
                                                                              *
                                                                              * @note
                                                                              * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
                                                                              *****************************************************************************/
#include "musbfsfc.h"
#include "PanSeries.h"
#include "descript.h"
#include "endpoint.h"
#include "endpoint0.h"
#include "usb_panchip.h"

uint8_t bulkin_ep_need_data = DATA_NONEED;

// #pragma arm section code = "RAMCODE"

void USB_PersonalEpInit(void)
{
  WRITE_REG(USB->INDEX, 0);
  WRITE_REG(USB->MAX_PKT_IN, 8);
  WRITE_REG(USB->MAX_PKT_OUT, 8);
  /* Initialise endpoint status structures with endpoint numbers */
  gepbin1.byEP = 1;
  EndpointBulkIn(&gepbin1, M_EP_RESET);
  WRITE_REG(USB->INDEX, 1);
  WRITE_REG(USB->MAX_PKT_IN, 8);

  gepbout2.byEP = 2;
  gepbin2.byEP  = 2;
  EndpointBulkOut(&gepbout2, M_EP_RESET);
  EndpointBulkIn(&gepbin2, M_EP_RESET);

  WRITE_REG(USB->INDEX, 2);
  WRITE_REG(USB->MAX_PKT_OUT, 8);
  WRITE_REG(USB->MAX_PKT_IN, 8);

  gepbout3.byEP = 3;
  gepbin3.byEP  = 3;
  EndpointBulkOut(&gepbout3, M_EP_RESET);
  EndpointBulkIn(&gepbin3, M_EP_RESET);

  WRITE_REG(USB->INDEX, 3);
  WRITE_REG(USB->MAX_PKT_OUT, 8);
  WRITE_REG(USB->MAX_PKT_IN, 8);

  WRITE_REG(USB->INDEX, 0);
}

/****************************************
   USB Reset
****************************************/
void USB_Reset(void)
{
  /* Set device into default state */
  gnDevState = DEVSTATE_DEFAULT;
  cfg.pcfg1  = &cfg1;
  cfg.pcfg2  = &cfg2;
  /* Initialise descriptors */
  // InitialiseDesc();
  /* Reset any state machines for each endpoint */
  /* This example assumes 3 bulk IN endpoints with endpoint numbers 1,2, 3*/
  /* and 3 bulk out endpoints with endpoint numbers 1,2 */
  Endpoint0(M_EP_RESET);

  USB_PersonalEpInit();
  bulkin_ep_need_data = DATA_NEED;
}

/****************************************
   Top level interrupt handler
****************************************/
void musbfsfc_intr(void)
{
  uint8_t  IntrUSB;
  uint16_t IntrIn;
  uint16_t IntrOut;
  uint32_t IntrFlag;

  /* Read interrupt registers */
  /* Mote if less than 8 IN endpoints are configured then */
  /* only M_REG_INTRIN1 need be read. */
  /* Similarly if less than 8 OUT endpoints are configured then */
  /* only M_REG_INTROUT1 need be read. */
  IntrUSB   = READ_REG(USB->INT_USB);
  IntrIn    = (uint16_t)READ_REG(USB->INT_IN2);
  IntrIn  <<= 8;
  IntrIn   |= (uint16_t)READ_REG(USB->INT_IN1);
  IntrOut   = (uint16_t)READ_REG(USB->INT_OUT2);
  IntrOut <<= 8;
  IntrOut  |= (uint16_t)READ_REG(USB->INT_OUT1);

  IntrFlag   = IntrIn;
  IntrFlag <<= 16;
  IntrFlag  |= IntrOut;

  if (IntrUSB & M_INTR_PLUG)
  {
    if ((IntrUSB & M_INTR_PLUG_OUT) == M_INTR_PLUG_OUT)
    {
      SYS_TEST("Plug out\n");
    }
    else if (IntrUSB & M_INTR_PLUG_OUT)
    {
      SYS_TEST("Plug in\n");
    }
  }
  if (IntrUSB & M_INTR_RESUME)
  {
    SYS_TEST("Resume\n");
  }

  if (IntrUSB & M_INTR_SUSPEND)
  {
    SYS_TEST("Suspend\n");
  }

  if (IntrUSB & M_INTR_SOF)
  {
    SYS_TEST("Sof\n");
  }

  /* Check for system interrupts */
  if (IntrUSB & M_INTR_RESET)
  {
    USB->POWER    |= 0x1;
    USB->INT_USBE |= 0x17;
    SYS_TEST("RST\r\n");
    USB_Reset();
  }
  /* Check for endpoint 0 interrupt */
  if (IntrIn & M_INTR_EP0)
  {
    Endpoint0(M_EP_NORMAL);
  }

  /* Check for each configured endpoint interrupt */
  /* This example assumes 2 bulk IN endpoints with endpoint numbers 1,2, */
  /* and 2 bulk out endpoints with endpoint numbers 1,2 */

  // if(IntrFlag&0xfffefffe)
  //      USB_TestExample(IntrFlag);

  return;
}

// #pragma arm section
