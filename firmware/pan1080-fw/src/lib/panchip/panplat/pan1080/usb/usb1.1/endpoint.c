/**************************************************************************//**
 * @file     endpoint.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/08/04 14:25 $
 * @brief    Endpoint interrupt routine for MUSBFSFC firmware
 *
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#include "endpoint.h"
#include "PanSeries.h"
#include "usb_panchip.h"

USB_BulkInStatus gepbin1 = { 0, }, gepbin2 = { 0, }, gepbin3 = { 0, };
USB_BulkOutStatus gepbout1 = { 0, }, gepbout2 = { 0, }, gepbout3 = { 0, };
/****************************************
   EndpointBulkIn service routine
   This routine provides simple handling for bulk in endpoints.
   Endpoint MaxP is defined in endpoint.h.
   While the nBytesLeft field of the pbostate structure is >= 0
   the endpoint will send packets of MaxP, with the residue in the last packet.
   If the data to send is an exact multiple of MaxP then a null data packet
   will be sent.
****************************************/
void EndpointBulkIn(USB_BulkInStatus *pbistate, uint32_t nCallState)
{
  uint32_t nBytes;
  uint8_t  byInCSR;

  /* Reset endpoint */
  if (nCallState == M_EP_RESET)
  {
    pbistate->nBytesLeft = M_EP_NODATA;
    WRITE_REG(USB->INDEX, pbistate->byEP);

    byInCSR = M_INCSR_CDT | M_INCSR_FF;
    WRITE_REG(USB->CSR0_INCSR1, byInCSR);
    return;
  }

  /* Check whether there is any data to send */
  if (pbistate->nBytesLeft == M_EP_NODATA)
  {
    return;
  }

  /* Determine number of bytes to send */
  if (pbistate->nBytesLeft < M_EP_MAXP)
  {
    nBytes               = pbistate->nBytesLeft;
    pbistate->nBytesLeft = M_EP_NODATA;
  }
  else
  {
    nBytes                = M_EP_MAXP;
    pbistate->nBytesLeft -= M_EP_MAXP;
  }

  ///* Load FIFO */
  WRITE_REG(USB->INDEX, pbistate->byEP);
  USB_Write((uint32_t)pbistate->byEP, nBytes, pbistate->pData);
  pbistate->pData = (uint8_t *)pbistate->pData + nBytes;

  byInCSR = M_INCSR_IPR;
  WRITE_REG(USB->CSR0_INCSR1, byInCSR);
}

/****************************************
   EndpointBulkOut service routine
   This routine provides simple handling for bulk out endpoints.
   Endpoint MaxP is defined in endpoint.h.
   It assumes a transfer is complete when a packet of less than MaxP is received.
****************************************/
void EndpointBulkOut(USB_BulkOutStatus *pbostate, uint32_t nCallState)
{
  uint32_t nBytes;
  uint8_t  byOutCSR;

  /* Reset endpoint */
  if (nCallState == M_EP_RESET)
  {
    pbostate->nBytesRecv = 0;
    pbostate->nBuffLen   = 0;
    WRITE_REG(USB->INDEX, pbostate->byEP);
    byOutCSR = M_OUTCSR_CDT | M_OUTCSR_FF;
    WRITE_REG(USB->OUT_CSR1, byOutCSR);
    return;
  }

  /* Set index register */
  WRITE_REG(USB->INDEX, pbostate->byEP);
  /* Read OUT CSR register */
  byOutCSR = READ_REG(USB->OUT_CSR1);

  /* Get packet, */
  /* may need to unload more than one packet if double buffering enabled */
  while (byOutCSR & M_OUTCSR_OPR)
  {
    /* Get packet size */
    nBytes               = (int)READ_REG(USB->OUT_COUNT2);
    nBytes             <<= 8;
    nBytes              |= (int)READ_REG(USB->OUT_COUNT1);
    pbostate->nBuffLen   = READ_REG(USB->MAX_PKT_OUT) << 3;

    /* Check there is room in the buffer */
    //		printk("AA :%x,%x,%x \r\n",pbostate->nBytesRecv,nBytes,pbostate->nBuffLen);
    if (pbostate->nBytesRecv + nBytes > pbostate->nBuffLen)
    {
      /* Call to function to handle buffer over run */
      WRITE_REG(USB->OUT_CSR1, 0);
      return;
    }

    /* Unload FIFO */
    USB_Read((int)pbostate->byEP, nBytes, pbostate->pData);
    /* Update status */
    pbostate->nBytesRecv += nBytes;
    pbostate->pData       = (uint8_t *)pbostate->pData + nBytes;

    /* Check for end of transfer */
    if (nBytes < M_EP_MAXP)
    {
      /* Call function to handle received data */
    }

    /* Clear OutPktRdy */
    WRITE_REG(USB->OUT_CSR1, 0);
    /* Check for another packet */
    byOutCSR = READ_REG(USB->OUT_CSR1);
    //		WRITE_REG(USB->OUT_CSR1, M_OUTCSR_CDT);
  }
}
