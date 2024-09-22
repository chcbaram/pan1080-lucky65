/**************************************************************************//**
* @file     uart.c
* @version  V1.10
* $Revision: 2 $
* $Date:    19/10/08 09:25 $
* @brief    Panchip series UART driver source file
*
* @note
* Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_uart.h"
#include "pan_clk.h"

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct .
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @retval UART init success (true) or fail (false)
  */

bool UART_Init(UART_T* UARTx, UART_InitTypeDef* UART_InitStruct)
{
    uint32_t tmpreg = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    uint64_t apbclock = 0x00;
    /*---------------------------- UART BRR Configuration -----------------------*/
    /* Configure the UART Baud Rate */
    apbclock = CLK_GetPeripheralFreq((void*)UARTx);

    /*unlock to enable write & read divisor register*/
    UARTx->LCR |= UART_LCR_DLAB_Msk;
    /* Determine the integer part baud_rate_divisor =  PCLK*100 / (16*required_baud_rate)*/
    integerdivider = ((25 * apbclock) / (4 * (UART_InitStruct->UART_BaudRate)));

    //Too high baudrate (too small divider) would cause DLL/DLH be all 0 which means UART disabled,
    //thus return false if this happens.
    if (integerdivider < 100)
        return false;

    tmpreg = (integerdivider / 100);
    UARTx->RBR_THR_DLL = tmpreg & 0xFF;
    UARTx->IER_DLH = (tmpreg & 0xFF00 ) >> 8;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * tmpreg);

    /* Implement the fractional part in the register */
    UARTx->DLF = ((((fractionaldivider * 16) + 50) / 100)) ;
    UARTx->LCR &=  ~UART_LCR_DLAB_Msk;

    /*---------------------------- UART Line Configuration -----------------------*/
    tmpreg = UARTx->LCR;
    tmpreg &= ~(UART_LCR_SP_Msk | UART_LCR_EPS_Msk | UART_LCR_PEN_Msk | UART_LCR_STOP_Msk | UART_LCR_DLS_Msk);
    tmpreg |= (UART_InitStruct->UART_LineCtrl);
    UARTx->LCR = tmpreg;

    return true;
}

void UART_DeInit(UART_T* UARTx)
{

}

bool UART_SendMultipleData(UART_T* UARTx, uint8_t* Buf, size_t BufSize, size_t SendSize)
{
    if (BufSize < SendSize)
        return false;

    while (SendSize--)
    {
        UART_SendData(UARTx, *Buf++);
        while(!(UART_GetLineStatus(UARTx) & UART_LSR_TEMT_Msk));    // Wait until THR is empty to avoid data lost
    }

    return true;
}

bool UART_ReceiveMultipleData(UART_T* UARTx, uint8_t* Buf, size_t BufSize, size_t ExpectSize)
{
    if (BufSize < ExpectSize)
        return false;

    while ((UART_GetLineStatus(UARTx) & UART_LSR_DR_Msk) && ExpectSize)
    {
        *Buf++ = UART_ReceiveData(UARTx);
        ExpectSize--;
    }

    return true;
}


/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
