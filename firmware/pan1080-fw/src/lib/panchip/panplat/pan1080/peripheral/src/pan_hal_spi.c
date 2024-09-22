/**************************************************************************//**
* @file     pan_hal_spi.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series SPI (Serial Peripheral Interface) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#include "pan_hal.h"

SPI_HandleTypeDef SPI_Handle_Array[2] = 
{
    {
        .SPIx = SPI0,  
        .InitObj = {0},
        .InterruptObj = {0},
        .Xfer_Stat = SPI_STAT_NULL,
        .pTxBuffPtr = NULL,
        .TxXferSize = 0,
        .TxXferCount = 0,
        .pRxBuffPtr = NULL,
        .RxXferSize = 0,
        .RxXferCount = 0,
        .IRQn = SPI0_IRQn,  // Adjust this to appropriate IRQn_Type value
        .RxIntCallback = NULL,
        .TxIntCallback = NULL,
        .DmaSrc = DMAC_Peripheral_SPI0_Rx,
        .DmaDst = DMAC_Peripheral_SPI0_Tx,
        .ErrorCode = 0
    },
    {
        .SPIx = SPI1,  
        .InitObj = {0},
        .InterruptObj = {0},
        .Xfer_Stat = SPI_STAT_NULL,
        .pTxBuffPtr = NULL,
        .TxXferSize = 0,
        .TxXferCount = 0,
        .pRxBuffPtr = NULL,
        .RxXferSize = 0,
        .RxXferCount = 0,
        .IRQn = SPI1_IRQn,  // Adjust this to appropriate IRQn_Type value
        .RxIntCallback = NULL,
        .TxIntCallback = NULL,
        .DmaSrc = DMAC_Peripheral_SPI1_Rx,
        .DmaDst = DMAC_Peripheral_SPI1_Tx,
        .ErrorCode = 0
    }
};



void HAL_SPI_Init(SPI_HandleTypeDef *spi)
{
    uint32_t tmpreg = 0;
    //uint8_t baud_div = CalcDivisorByBaudrate(spi);

    if(SPI_IsSpiEnabled(spi->SPIx))
        SPI_DisableSpi(spi->SPIx);

    tmpreg = spi->SPIx->CR0;

    tmpreg &= ~(SPI_CR0_SPH_Msk | SPI_CR0_SPO_Msk  | SPI_CR0_DSS_Msk);
    tmpreg |= (
               ((spi->InitObj.ClkPhase           << SPI_CR0_SPH_Pos) & SPI_CR0_SPH_Msk )  |   \
                ((spi->InitObj.ClkPolarity          << SPI_CR0_SPO_Pos) & SPI_CR0_SPO_Msk )  |   \
                ((spi->InitObj.DataSize << SPI_CR0_DSS_Pos) & SPI_CR0_DSS_Msk )      \
              );
    tmpreg &= ~(SPI_CR0_FRF_Msk);
    tmpreg |= (spi->InitObj.Format << SPI_CR0_FRF_Pos);
    tmpreg &= ~(SPI_CR0_SCR_Msk);
    // tmpreg |= ((spi->SPI_InitStruct->SPI_baudRateDiv-1) << SPI_CR0_SCR_Pos);   //Fixed SCR to 0, only use CPSR to config baudrate
    //tmpreg |= ((baud_div-1) << SPI_CR0_SCR_Pos);   //Fixed SCR to 0, only use CPSR to config baudrate
    tmpreg |= ((spi->InitObj.BaudrateDiv-1) << SPI_CR0_SCR_Pos);   //Fixed SCR to 0, only use CPSR to config baudrate    
    spi->SPIx->CR0 = tmpreg;

    //Baudrate Config, expect_baudrate = apb_clk / ((1 + SCR) * CPSR)
    spi->SPIx->CPSR = 2;//SPI_InitStruct->SPI_baudRateDiv;

    //Role Select
    if(spi->InitObj.Role == SPI_RoleSlave)
    {
        spi->SPIx->CR1 |= SPI_CR1_MS_Msk;
    }
    else
    {
        spi->SPIx->CR1 &= ~SPI_CR1_MS_Msk;
    }

    SPI_EnableSpi(spi->SPIx);
}

void HAL_SPI_DeInit(SPI_HandleTypeDef *spi){
    SPI_DisableSpi(spi->SPIx);
}

void HAL_SPI_Init_INT(SPI_HandleTypeDef *spi){
    SPI_EnableIrq(spi->SPIx, spi->InterruptObj.Mode);
}

void HAL_SPI_DeInit_INT(SPI_HandleTypeDef *spi){
    SPI_DisableIrq(spi->SPIx, spi->InterruptObj.Mode);
}


// Receive multiple data from specified SPI, and return the size of actual data received
static size_t HAL_SPI_RecvMultiDataAsync(SPI_T* SPIx, uint16_t* data, size_t expect_recv_size)
{
    size_t i = 0;
    for (; i < expect_recv_size; i++)
    {
        // Fetch data from Rx FIFO until it's empty
        if (!SPI_IsRxFifoEmpty(SPIx))
        {
            data[i] = SPI_ReceiveData(SPIx);
        }
        else
        {
            break;
        }
    }
    return i;
}

// Send multiple data to specified SPI, and return the size of actual data sent
static size_t HAL_SPI_SendMultiDataAsync(SPI_T* SPIx, const uint16_t* data, size_t expect_send_size)
{
    size_t i = 0;
    for (; i < expect_send_size; i++)
    {
        // Fill data into Tx FIFO until it's full
        if (!SPI_IsTxFifoFull(SPIx))
        {
            SPI_SendData(SPIx, data[i]);
        }
        else
        {
            break;
        }
    }
    return i;
}

void HAL_SPI_SendData(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, uint32_t Timeout)
{    
	spi->TxXferSize = Size;
    spi->TxXferCount =0;
    spi->pTxBuffPtr = Buf;

    while (spi->TxXferCount < spi->TxXferSize)
    {
        if (!SPI_IsTxFifoFull(spi->SPIx))
        {
            SPI_SendData(spi->SPIx, spi->pTxBuffPtr[spi->TxXferCount++]);
        }
    }
}

void HAL_SPI_ReceiveData(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, uint32_t Timeout)
{
    spi->RxXferCount = 0;
    spi->RxXferSize =Size;
    spi->pRxBuffPtr = Buf;

    if(spi->InitObj.Role == SPI_RoleMaster)
    {
        while (spi->RxXferCount < spi->RxXferSize)
        {
            if (SPI_IsTxFifoEmpty(spi->SPIx) && SPI_IsRxFifoEmpty(spi->SPIx))
            {
                SPI_SendData(spi->SPIx, '\0'); // Send dummy data to generate CLK for receving
                while (SPI_IsBusy(spi->SPIx))
                    ; // Make sure enough clock pulses have been sent out
                spi->pRxBuffPtr[spi->RxXferCount++] = SPI_ReceiveData(spi->SPIx);
            }
            else
            {
                SYS_TEST("Warning, is not expected running to here!\n");
            }
        }
    }
    else if (spi->InitObj.Role == SPI_RoleSlave)
    {
        while (spi->RxXferCount < spi->RxXferSize)
        {
            if (!SPI_IsRxFifoEmpty(spi->SPIx))
            {
                spi->pRxBuffPtr[spi->RxXferCount++] = SPI_ReceiveData(spi->SPIx);
            }
        }
    }
}

void HAL_SPI_SendReceiveData (SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize, uint16_t* RecvBuf, size_t RecvBufSize)
{
    size_t actual_sent_size = 0;
    size_t actual_recv_size = 0;
    while (!(actual_sent_size == SendBufSize && actual_recv_size == RecvBufSize))
    {
        if (actual_sent_size < SendBufSize)
        {
            actual_sent_size += HAL_SPI_SendMultiDataAsync(spi->SPIx, SendBuf + actual_sent_size, SendBufSize - actual_sent_size);
        }
        if (actual_recv_size < RecvBufSize)
        {
            actual_recv_size += HAL_SPI_RecvMultiDataAsync(spi->SPIx, RecvBuf + actual_recv_size, RecvBufSize - actual_recv_size);
        }
    }
}


void HAL_SPI_SendData_INT(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, SPI_CallbackFunc Callback) 
{
    spi->Xfer_Stat = SPI_STAT_TX;
    /* Enable Interrupt of target SPI */
    // spi->InterruptObj.Mode = SPI_IRQ_ALL;
    spi->TxXferSize = Size;
    spi->TxXferCount =0;
    spi->pTxBuffPtr = Buf;
    spi->TxIntCallback = Callback;
    // HAL_SPI_Init_INT(spi);
    SPI_EnableIrq(spi->SPIx, SPI_IRQ_TX_HALF_EMPTY);
    NVIC_EnableIRQ(spi->IRQn);        
}


void HAL_SPI_SendReceiveData_INT(SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize,SPI_CallbackFunc SendCallback, uint16_t* RecvBuf, size_t RecvBufSize,SPI_CallbackFunc RecvCallback)
{
    spi->Xfer_Stat = SPI_STAT_RX;

    spi->TxXferSize = SendBufSize;
    spi->TxXferCount =0;
    spi->pTxBuffPtr = SendBuf;
    spi->TxIntCallback = SendCallback;

    spi->RxXferSize = RecvBufSize;
    spi->RxXferCount =0;
    spi->pRxBuffPtr = RecvBuf;
    spi->RxIntCallback = RecvCallback;
    SPI_EnableIrq(spi->SPIx, SPI_IRQ_ALL);
    NVIC_EnableIRQ(spi->IRQn);
}


uint32_t HAL_SPI_SendData_DMA(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size,SPI_CallbackFunc Callback) 
{
    DMAC_Init(DMA);
    NVIC_EnableIRQ(DMA_IRQn);               
    SPI_EnableDmaTx(spi->SPIx);

    spi->TxXferCount =0;
    spi->TxXferSize = Size;
    spi->pTxBuffPtr =Buf;
    uint32_t DmaChNum = 0xFF;
    /* Get free DMA channel */
    DmaChNum = DMAC_AcquireChannel(DMA);
    /* Enable DMA transfer interrupt */
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_DSTTFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_ERR);

    dma_mem2spi_config.PeripheralDst = spi->DmaDst;
    DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2spi_config;
    DMAC_Channel_Array[DmaChNum].CallbackSpi = Callback;
    DMAC_Channel_Array[DmaChNum].StopFlag = false;

    DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)spi->pTxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = spi->TxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_SPI;    
    DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);

    /* Condition check */
    uint32_t DataWidthInByteSrc =  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    // uint32_t DataWidthInByteSrc = 1 <<  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    uint32_t IsNotDivisible = spi->TxXferSize % DataWidthInByteSrc;
    uint32_t BlockSize = spi->TxXferSize / DataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc

    /* Start DMA Tx channel */
    DMAC_StartChannel(DMA, DmaChNum, spi->pTxBuffPtr, (void*)&(spi->SPIx->DR), BlockSize);
    return DmaChNum;
}

static uint32_t HAL_SPI_SendData_DMA_ForSR(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size,SPI_CallbackFunc Callback) 
{

    // DMAC_Init(DMA);
    // NVIC_EnableIRQ(DMA_IRQn);               
    SPI_EnableDmaTx(spi->SPIx);

    spi->TxXferCount =0;
    spi->TxXferSize = Size;
    spi->pTxBuffPtr =Buf;
    uint32_t DmaChNum = 0xFF;
    /* Get free DMA channel */
    DmaChNum = DMAC_AcquireChannel(DMA);
    /* Enable DMA transfer interrupt */
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_DSTTFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_ERR);

    dma_mem2spi_config.PeripheralDst = spi->DmaDst;
    DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2spi_config;
    DMAC_Channel_Array[DmaChNum].CallbackSpi = Callback;
    DMAC_Channel_Array[DmaChNum].StopFlag = false;

    DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)spi->pTxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = spi->TxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_SPI;    
    DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);

    // /* Condition check */
    // uint32_t DataWidthInByteSrc =  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    // // uint32_t DataWidthInByteSrc = 1 <<  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    // uint32_t IsNotDivisible = spi->TxXferSize % DataWidthInByteSrc;
    // uint32_t BlockSize = spi->TxXferSize / DataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc

    // /* Start DMA Tx channel */
    // DMAC_StartChannel(DMA, DmaChNum, spi->pTxBuffPtr, (void*)&(spi->SPIx->DR), BlockSize);
    return DmaChNum;
}


static uint32_t HAL_SPI_ReceiveData_DMA_ForSR(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, SPI_CallbackFunc Callback) 
{

    // DMAC_Init(DMA);
    // NVIC_EnableIRQ(DMA_IRQn);
    SPI_EnableDmaRx(spi->SPIx);
    
    spi->RxXferCount =0;
    spi->RxXferSize = Size;
    spi->pRxBuffPtr =Buf;
    // Initialize the DMA controller
    uint32_t DmaChNum = 0xff;
    /* Get free DMA channel */
    DmaChNum = DMAC_AcquireChannel(DMA);
    /* Enable DMA transfer interrupt */
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_SRCTFR);
    DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_ERR);


    dma_spi2mem_config.PeripheralSrc = spi->DmaSrc;
    DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_spi2mem_config;
    DMAC_Channel_Array[DmaChNum].CallbackSpi = Callback;
    DMAC_Channel_Array[DmaChNum].StopFlag = false;

    DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)spi->pRxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = spi->RxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_SPI;
    DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);

    // /* Condition check */
    // uint32_t DataWidthInByteSrc =  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    // uint32_t IsNotDivisible = spi->RxXferSize % DataWidthInByteSrc;
    // uint32_t BlockSize = spi->RxXferSize / DataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc

    // /* Start DMA Rx channel */
    // DMAC_StartChannel(DMA, DmaChNum, (void*)&(spi->SPIx->DR), spi->pRxBuffPtr, BlockSize);
    return DmaChNum;
}

uint32_t HAL_SPI_SendReceiveData_DMA(SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize,SPI_CallbackFunc SendCallback, uint16_t* RecvBuf, size_t RecvBufSize,SPI_CallbackFunc RecvCallback){
    uint32_t RxChNum = 0xFF;
    uint32_t TxChNum = 0xFF;

    DMAC_Init(DMA);
    NVIC_EnableIRQ(DMA_IRQn);               
    TxChNum = HAL_SPI_SendData_DMA_ForSR(spi,SendBuf,SendBufSize,SendCallback);
    RxChNum = HAL_SPI_ReceiveData_DMA_ForSR(spi,RecvBuf,RecvBufSize,RecvCallback);

    /* Condition check */
    uint32_t RxDataWidthInByteSrc =  DMAC_Channel_Array[RxChNum].ConfigTmp.DataWidthSrc;
    uint32_t RxIsNotDivisible = spi->RxXferSize % RxDataWidthInByteSrc;
    uint32_t RxBlockSize = spi->RxXferSize / RxDataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc


    /* Condition check */
    uint32_t TxDataWidthInByteSrc =  DMAC_Channel_Array[TxChNum].ConfigTmp.DataWidthSrc;
    // uint32_t DataWidthInByteSrc = 1 <<  DMAC_Channel_Array[DmaChNum].ConfigTmp.DataWidthSrc;
    uint32_t TxIsNotDivisible = spi->TxXferSize % TxDataWidthInByteSrc;
    uint32_t TxBlockSize = spi->TxXferSize / TxDataWidthInByteSrc;    //BlockSize = DataLen / DataWidthInByteSrc
    
    /* Start DMA Rx channel */
    DMAC_StartChannel(DMA, RxChNum, (void*)&(spi->SPIx->DR), spi->pRxBuffPtr, RxBlockSize);
    /* Start DMA Tx channel */
    DMAC_StartChannel(DMA, TxChNum, spi->pTxBuffPtr, (void*)&(spi->SPIx->DR), TxBlockSize);

    return 0;
}

static void SPI_HandleReceivedData(SPI_HandleTypeDef *spi)
{
    while (!SPI_IsRxFifoEmpty(spi->SPIx))
    {
        spi->pRxBuffPtr[spi->RxXferCount++] = SPI_ReceiveData(spi->SPIx);
        // if (spi->RxXferCount > spi->RxXferSize) // Rx buffer full                                                // if (spi->RxXferCount >= spi->RxXferSize)   //Rx buffer full
        // {
        //     SYS_TEST("Error, too much data received, Rx buffer full!\n");
        //     spi->RxXferCount = 0; // Reset Rx buf index
        //     break;
        // }
        if (spi->RxXferCount >= spi->RxXferSize) // Rx buffer full                                                 // if (spi->RxXferCount >= spi->RxXferSize)   //Rx buffer full
        {
            if (spi->RxIntCallback != NULL)
            {
                spi->RxIntCallback(SPI_CB_FLAG_INT, spi->pRxBuffPtr, spi->RxXferCount);
            }
            SPI_DisableIrq(spi->SPIx, SPI_IRQ_RX_HALF_FULL|SPI_IRQ_RX_TIMEOUT);
            break;
        }
    }
}

static void SPI_HandleTransmittingData(SPI_HandleTypeDef *spi)
{
    while (!SPI_IsTxFifoFull(spi->SPIx))
    {
        if (spi->TxXferCount < spi->TxXferSize)
        {
            SPI_SendData(spi->SPIx, spi->pTxBuffPtr[spi->TxXferCount++]);
        }
        if (spi->TxXferCount >= spi->TxXferSize)
        {
            SPI_DisableIrq(spi->SPIx, SPI_IRQ_TX_HALF_EMPTY);
            if (spi->TxIntCallback != NULL)
            {
                spi->TxIntCallback(SPI_CB_FLAG_INT, spi->pTxBuffPtr, spi->TxXferCount);
            }
            break;
        }

    }
}
void SPI_HandleProc(SPI_HandleTypeDef *spi)
{
   if (SPI_IsIrqActive(spi->SPIx, SPI_IRQ_RX_OVERRUN))
   {
//       SYS_TEST("r\n");
       SPI_ClearIrq(spi->SPIx, SPI_IRQ_RX_OVERRUN); // Clear overrun INT flag manually
//       SYS_TEST("SPI Rx Overrun!\n");
       return;
   }

   if (SPI_IsIrqActive(spi->SPIx, SPI_IRQ_RX_HALF_FULL))
   {
       SPI_HandleReceivedData(spi);
   }
   else if (SPI_IsIrqActive(spi->SPIx, SPI_IRQ_RX_TIMEOUT))
   {
//       SYS_TEST("R\n");
       SPI_HandleReceivedData(spi);
   }

   if (SPI_IsIrqActive(spi->SPIx, SPI_IRQ_TX_HALF_EMPTY))
   {
        // SYS_TEST("T\n");
       SPI_HandleTransmittingData(spi);
   }
}

void SPI0_IRQHandler(void)
{
    SPI_HandleProc(&SPI_Handle_Array[0]);
}

void SPI1_IRQHandler(void)
{
    SPI_HandleProc(&SPI_Handle_Array[1]);
}



