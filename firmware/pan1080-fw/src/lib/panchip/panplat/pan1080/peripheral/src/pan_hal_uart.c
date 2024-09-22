/**************************************************************************//**
* @file     pan_hal_uart.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series UART (Universal Asynchronous Receiver-Transmitter) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#include "pan_hal.h"

volatile bool uartRxIntFlag = false;

volatile bool uartTxDone = false;

UART_HandleTypeDef UART_Handle_Array[2] =
    {
        {.UARTx = UART0,
         .InitObj = {0},
         .InterruptObj = {0},
         .pTxBuffPtr = NULL,
         .TxXferSize = 0,
         .TxXferCount = 0,
         .pRxBuffPtr = NULL,
         .RxXferSize = 0,
         .RxXferCount = 0,
         .IRQn = UART0_IRQn,
         .RxIntCallback = NULL,
         .TxIntCallback = NULL,
         .DmaSrc = DMAC_Peripheral_UART0_Rx,
         .DmaDst = DMAC_Peripheral_UART0_Tx,
         .ErrorCode = 0},
        {.UARTx = UART1,
         .InitObj = {0},
         .InterruptObj = {0},
         .pTxBuffPtr = NULL,
         .TxXferSize = 0,
         .TxXferCount = 0,
         .pRxBuffPtr = NULL,
         .RxXferSize = 0,
         .RxXferCount = 0,
         .IRQn = UART1_IRQn,
         .RxIntCallback = NULL,
         .TxIntCallback = NULL,
         .DmaSrc = DMAC_Peripheral_UART1_Rx,
         .DmaDst = DMAC_Peripheral_UART1_Tx,
         .ErrorCode = 0}};

void HAL_DelayMs(uint32_t t)
{
    for (uint32_t i = 0; i < t; i++)
    {
        SYS_delay_10nop(0x1080);
    }
}



bool HAL_UART_Init(UART_HandleTypeDef *uart)
// bool HAL_UART_Init(UART_T *UARTx, UART_Init_Opt *UART_InitObj)
{
    uint32_t tmpreg = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    uint64_t apbclock = 0x00;

    /*---------------------------- UART BRR Configuration -----------------------*/
    /* Configure the UART Baud Rate */
    apbclock = CLK_GetPeripheralFreq((void *)uart->UARTx);
    /*unlock to enable write & read divisor register*/
    uart->UARTx->LCR |= UART_LCR_DLAB_Msk;
    /* Determine the integer part baud_rate_divisor =  PCLK*100 / (16*required_baud_rate)*/
    integerdivider = ((25 * apbclock) / (4 * (uart->InitObj.BaudRate)));

    // Too high baudrate (too small divider) would cause DLL/DLH be all 0 which means UART disabled,
    // thus return false if this happens.
    if (integerdivider < 100)
        return false;

    tmpreg = (integerdivider / 100);
    uart->UARTx->RBR_THR_DLL = tmpreg & 0xFF;
    uart->UARTx->IER_DLH = (tmpreg & 0xFF00) >> 8;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * tmpreg);

    /* Implement the fractional part in the register */
    uart->UARTx->DLF = ((((fractionaldivider * 16) + 50) / 100));
    uart->UARTx->LCR &= ~UART_LCR_DLAB_Msk;

    /*---------------------------- UART Line Configuration -----------------------*/
    tmpreg = uart->UARTx->LCR;
    tmpreg &= ~(UART_LCR_SP_Msk | UART_LCR_EPS_Msk | UART_LCR_PEN_Msk | UART_LCR_STOP_Msk | UART_LCR_DLS_Msk);
    tmpreg |= (uart->InitObj.Format);
    uart->UARTx->LCR = tmpreg;

    /*-------------------------------enable fifo----------------------------------*/
    // SCR(r/w) stores a copy of FCR(wo)

    uart->UARTx->SCR |= UART_FCR_FIFOE_Msk;
    uart->UARTx->IIR_FCR = uart->UARTx->SCR;
    UART_EnableFifo(uart->UARTx);

    return true;
}

void HAL_UART_SendData(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size)
{
    uart->pTxBuffPtr = Buf;
    uart->TxXferSize = Size;
    uart->TxXferCount = 0;
    while (uart->TxXferCount < uart->TxXferSize)
    {
        UART_SendData(uart->UARTx, uart->pTxBuffPtr[uart->TxXferCount++]);
        while (!(UART_GetLineStatus(uart->UARTx) & UART_LSR_TEMT_Msk))
            ;
    }
}

void HAL_UART_ReceiveData(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, uint32_t Timeout)
{
    UART_SetRxTrigger(uart->UARTx, UART_RX_FIFO_HALF_FULL);

    uart->pRxBuffPtr = Buf;
    uart->RxXferSize = Size;
    uart->RxXferCount = 0;
    while (uart->RxXferCount < uart->RxXferSize)
    {
        while ((UART_GetLineStatus(uart->UARTx) & UART_LSR_DR_Msk))
        {
            uart->pRxBuffPtr[uart->RxXferCount++] = UART_ReceiveData(uart->UARTx);
        }
    }
}

void HAL_UART_SendData_INT(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, UART_CallbackFunc Callback)
{
    uart->pTxBuffPtr = Buf;
    uart->TxXferSize = Size;
    uart->TxXferCount = 0;
    uart->TxIntCallback = Callback;
    UART_SetTxTrigger(uart->UARTx, UART_TX_FIFO_QUARTER_FULL);

    uart->InterruptObj.Switch = ENABLE;
    uart->InterruptObj.Mode = HAL_UART_INT_THR_EMPTY;
    HAL_UART_Init_INT(uart);
    NVIC_EnableIRQ(uart->IRQn);
}

void HAL_UART_ReceiveData_INT(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size,uint32_t Timeout,UART_CallbackFunc Callback)
{
    uart->pRxBuffPtr = Buf;
    uart->RxXferSize = Size;
    uart->RxXferCount = 0;
    uart->RxIntCallback = Callback;


    uart->InterruptObj.Switch = ENABLE;
    uart->InterruptObj.Mode = HAL_UART_INT_RECV_DATA_AVL;
    HAL_UART_Init_INT(uart);
    uart->InterruptObj.Mode = HAL_UART_INT_LINE_STATUS;
    HAL_UART_Init_INT(uart);
    uart->InterruptObj.Mode = HAL_UART_INT_ALL;
    HAL_UART_Init_INT(uart);
    NVIC_EnableIRQ(uart->IRQn);
}



void HAL_UART_SendData_DMA(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, UART_CallbackFunc Callback)
{
    uart->InterruptObj.Switch = ENABLE;
    uart->InterruptObj.Mode = HAL_UART_INT_THR_EMPTY;
    HAL_UART_Init_INT(uart);

    uart->pTxBuffPtr = Buf;
    uart->TxXferSize = Size;
    uart->TxXferCount = 0;

    

    UART_ResetTxFifo(uart->UARTx);
    UART_SetTxTrigger(uart->UARTx, UART_TX_FIFO_HALF_FULL);
    UART_EnablePtime(uart->UARTx); //Enable Programmable THRE Interrupt Mode
    // uart->InterruptObj.Mode = HAL_UART_INT_LINE_STATUS;
    // HAL_UART_Init_INT(uart);
    // NVIC_EnableIRQ(uart->IRQn);
    uint32_t TxChNum = 0xFF;
    // Initialize the DMA controller
    DMAC_Init(DMA);
    NVIC_EnableIRQ(DMA_IRQn);

    /* Get free DMA channel */
    TxChNum = DMAC_AcquireChannel(DMA);
    /* Enable DMA transfer interrupt */
    DMAC_ClrIntFlagMsk(DMA, TxChNum, DMAC_FLAG_INDEX_TFR);
    DMAC_ClrIntFlagMsk(DMA, TxChNum, DMAC_FLAG_INDEX_DSTTFR);
    DMAC_ClrIntFlagMsk(DMA, TxChNum, DMAC_FLAG_INDEX_BLK);
    DMAC_ClrIntFlagMsk(DMA, TxChNum, DMAC_FLAG_INDEX_ERR);
    dma_mem2uart_config.PeripheralDst = uart->DmaDst;
    DMAC_Channel_Array[TxChNum].ConfigTmp = dma_mem2uart_config;
    DMAC_Channel_Array[TxChNum].CallbackUart = Callback;
    DMAC_Channel_Array[TxChNum].PeriMode = DMAC_Peri_UART;
    DMAC_Channel_Array[TxChNum].pBuffPtr = (uint32_t*)uart->pTxBuffPtr;
    DMAC_Channel_Array[TxChNum].XferSize = uart->TxXferSize;
    DMAC_SetChannelConfig(DMA, TxChNum, &DMAC_Channel_Array[TxChNum].ConfigTmp);

    /* Condition check */
    uint32_t DataWidthInByteSrc = 1 << DMAC_Channel_Array[TxChNum].ConfigTmp.DataWidthSrc;
    // BlockSize = DataLen / DataWidthInByteSrc    uint32_t IsNotDivisible = Size % DataWidthInByteSrc;
    uint32_t BlockSize = Size / DataWidthInByteSrc; // BlockSize = DataLen / DataWidthInByteSrc

    /* Start DMA Tx channel */
    DMAC_StartChannel(DMA, TxChNum, Buf, (void *)&(uart->UARTx->RBR_THR_DLL), BlockSize);
}

void HAL_UART_ReceiveData_DMA(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, uint32_t Timeout,UART_CallbackFunc Callback)
{
    uart->pRxBuffPtr = Buf;
    uart->RxXferSize = Size;
    uart->RxXferCount = 0;
    UART_ResetRxFifo(uart->UARTx);

    UART_SetRxTrigger(uart->UARTx, UART_RX_FIFO_HALF_FULL);
    // UART_SetRxTrigger(uart->UARTx, UART_RX_FIFO_HALF_FULL);
    uint32_t RxChNum = 0xFF;
    // Initialize the DMA controller
    DMAC_Init(DMA);
    NVIC_EnableIRQ(DMA_IRQn);
    RxChNum = DMAC_AcquireChannel(DMA);
    /* Enable DMA transfer interrupt */
    DMAC_ClrIntFlagMsk(DMA, RxChNum, DMAC_FLAG_INDEX_TFR);
    DMAC_ClrIntFlagMsk(DMA, RxChNum, DMAC_FLAG_INDEX_SRCTFR);
    DMAC_ClrIntFlagMsk(DMA, RxChNum, DMAC_FLAG_INDEX_BLK);
    DMAC_ClrIntFlagMsk(DMA, RxChNum, DMAC_FLAG_INDEX_ERR);

    // dma_uart2mem_config.PeripheralDst = uart->DmaSrc;
    DMAC_Channel_Array[RxChNum].ConfigTmp = dma_uart2mem_config;
    DMAC_Channel_Array[RxChNum].ConfigTmp.PeripheralSrc = uart->DmaSrc;
    DMAC_Channel_Array[RxChNum].CallbackUart = Callback;
    DMAC_Channel_Array[RxChNum].PeriMode = DMAC_Peri_UART;
    DMAC_Channel_Array[RxChNum].pBuffPtr = (uint32_t*)uart->pRxBuffPtr;
    DMAC_Channel_Array[RxChNum].XferSize = uart->RxXferSize;
    DMAC_SetChannelConfig(DMA, RxChNum, &DMAC_Channel_Array[RxChNum].ConfigTmp);

    /* Condition check */
    uint32_t DataWidthInByteSrc = 1 <<DMAC_Channel_Array[RxChNum].ConfigTmp.DataWidthSrc;
    uint32_t IsNotDivisible = Size % DataWidthInByteSrc;
    uint32_t BlockSize = Size / DataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc
    /* Start DMA Rx channel */

    (void)IsNotDivisible;
    
    DMAC_StartChannel(DMA, RxChNum, (void*)&(uart->UARTx->RBR_THR_DLL), uart->pRxBuffPtr, BlockSize);
}


void HAL_UART_Init_INT(UART_HandleTypeDef *uart)
{
   
    if (uart->InterruptObj.Switch == ENABLE)
    {
        uart->UARTx->IER_DLH |= ((uart->InterruptObj.Mode << UART_IER_ALL_IRQ_Pos) & UART_IER_ALL_IRQ_Msk);
    }
    else if (uart->InterruptObj.Switch == DISABLE)
    {
        uart->UARTx->IER_DLH &= ~((uart->InterruptObj.Mode << UART_IER_ALL_IRQ_Pos) & UART_IER_ALL_IRQ_Msk);
    }
}

static void UART_HandleLineStatus(UART_HandleTypeDef *uart)
{
    uint32_t lineStatus = UART_GetLineStatus(uart->UARTx);

    // Filter line error
    if ((lineStatus & (UART_LINE_OVERRUN_ERR | UART_LINE_PARITY_ERR | UART_LINE_FRAME_ERR | UART_LINE_RECV_FIFO_EMPTY)) != 0x0)
    {
        // SYS_TEST("An Uart line error occured, error type: %s\n", ConvLsrErrFlagToStr(lineStatus, str, sizeof(str)));
        //SYS_TEST("An Uart line error occured, error type:\n");
    }

    // Handle THRE event
    if (lineStatus & UART_LINE_THRE)
    {
        if ((uart->UARTx->IER_DLH & UART_IER_EPTI_Msk) == 0)
        {

        }

        else if ((uart->UARTx->IIR_FCR & UART_IIR_FIFOSE_Msk) != 0)
        {

        }
    }
}

static void UART_HandleModemStatus(UART_HandleTypeDef *uart)
{
    uint32_t modemStatus = UART_GetModemStatus(uart->UARTx);

    SYS_TEST("An Uart modem event occurs, modemStatus=0x%lx\n", modemStatus);
}

static void UART_HandleReceivedData(UART_HandleTypeDef *uart,UART_Cb_Flag_Opt Flag)
{
    while (!UART_IsRxFifoEmpty(uart->UARTx))
    {
        uart->pRxBuffPtr[uart->RxXferCount++] = UART_ReceiveData(uart->UARTx);
    }
    
    if ((Flag == UART_CB_FLAG_RX_TIMEOUT) || (uart->RxXferCount >= uart->RxXferSize))
    {
        if (uart->RxIntCallback != NULL)
        {
            uart->RxIntCallback(Flag,uart->pRxBuffPtr,uart->RxXferSize);
        }
        // uart->RxXferCount = 0;
    }
}

static void UART_HandleTransmittingData(UART_HandleTypeDef *uart)
{
    while (!UART_IsTxFifoFull(uart->UARTx))
    {
        if (uart->TxXferCount < uart->TxXferSize)
        {
            UART_SendData(uart->UARTx, uart->pTxBuffPtr[uart->TxXferCount++]);
        }
        else
        {
            UART_DisableIrq(uart->UARTx, UART_IRQ_THR_EMPTY); // Disable THRE Interrupt after transmitting done
            if (uart->TxIntCallback != NULL)
            {
                uart->TxIntCallback(UART_CB_FLAG_TX_FINISH, uart->pTxBuffPtr,uart->TxXferCount);
            }
            // uart->TxXferCount = 0;    
            break;
        }
    }   
}

static void UART_HandleProc(UART_HandleTypeDef *uart)
{
    UART_EventDef event = UART_GetActiveEvent(uart->UARTx);
    //SYS_TEST("UART (0x%x) Hdl IID: 0x%x\n", (uint32_t)UARTx, event);

    switch (event)
    {
    case UART_EVENT_LINE:
        UART_HandleLineStatus(uart);
        break;
    case UART_EVENT_DATA:
        // SYS_TEST("Rx thres trig!\n");
        UART_HandleReceivedData(uart,UART_CB_FLAG_RX_FINISH);
        break;
    case UART_EVENT_TIMEOUT:
        SYS_TEST("Rx tout trig!\n");
        UART_HandleReceivedData(uart,UART_CB_FLAG_RX_TIMEOUT);
        break;
    case UART_EVENT_THR_EMPTY:
        // SYS_TEST("Tx thres trig!\n");
        UART_HandleTransmittingData(uart);
        break;
    case UART_EVENT_MODEM:
        UART_HandleModemStatus(uart);
        break;
    case UART_EVENT_NONE:
        /* Just ignore this event. */
        break;
    default:
        // SYS_TEST("UART Handler Error, is not expected running to here! IID: 0x%x\n", event);
        break;
    }
}

void UART0_IRQHandler(void)
{
    UART_HandleProc(&UART_Handle_Array[0]);
}

void UART1_IRQHandler(void)
{
    UART_HandleProc(&UART_Handle_Array[1]);
}


/*** (C) COPYRIGHT 2023 Panchip Technology Corp. ***/
