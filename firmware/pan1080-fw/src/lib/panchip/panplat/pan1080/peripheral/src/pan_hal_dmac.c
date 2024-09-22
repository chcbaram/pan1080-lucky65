/**************************************************************************//**
* @file     pan_hal_dmac.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series DMAC (Direct Memory Access Controller) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#include "pan_hal.h"

DMAC_HandleTypeDef DMAC_Channel_Array[DMAC_CHANNEL_NUMS] = {{0,}, {0,}, {0,}};

void HAL_DMA_PostIRQHandler(DMAC_HandleTypeDef *hdma);


//uart ReceiveDma
DMAC_ChannelConfigTypeDef dma_uart2mem_config = {
    .TransferType    = DMAC_TransferType_Per2Mem,
    // Destination config (Memory)
    .AddrChangeDst   = DMAC_AddrChange_Increment,
    .BurstLenDst     = DMAC_BurstLen_4,
    .DataWidthDst    = DMAC_DataWidth_32,
    .HandshakeDst    = DMAC_Handshake_Default,
    // Source config (Peripheral)
    .AddrChangeSrc   = DMAC_AddrChange_NoChange,
    .BurstLenSrc     = DMAC_BurstLen_8, //According to peripheral FIFO threshold
    .DataWidthSrc    = DMAC_DataWidth_8, //According to peripheral FIFO width
    .HandshakeSrc    = DMAC_Handshake_Hardware,
    .PeripheralSrc   = NULL, //Config to corresponding peripheral

    .FlowControl     = DMAC_FlowControl_DMA,
    .IntEnable       = ENABLE,
};


//uart sendDma
DMAC_ChannelConfigTypeDef dma_mem2uart_config = {
    .TransferType    = DMAC_TransferType_Mem2Per,
    .AddrChangeDst   = DMAC_AddrChange_NoChange,
    .BurstLenDst     = DMAC_BurstLen_8,
    .DataWidthDst    = DMAC_DataWidth_8,
    .HandshakeDst    = DMAC_Handshake_Hardware,
    //.PeripheralDst   = TGT_UART_DMA_DST,
    .PeripheralDst   = NULL,
    .AddrChangeSrc   = DMAC_AddrChange_Increment,
//    .BurstLenSrc     = DMAC_BurstLen_4,
	  .BurstLenSrc     = DMAC_BurstLen_8,

    //.DataWidthSrc    = DMAC_DataWidth_32,
	  .DataWidthSrc    = DMAC_DataWidth_8,

    .HandshakeSrc    = DMAC_Handshake_Default,
    .FlowControl     = DMAC_FlowControl_DMA,
    .IntEnable       = ENABLE
};

DMAC_ChannelConfigTypeDef dma_spi2mem_config = {
    /* Set dma channel control & config register */
    .TransferType = DMAC_TransferType_Per2Mem,
    // Destination config (Memory)
    .AddrChangeDst = DMAC_AddrChange_Increment,
    .BurstLenDst = DMAC_BurstLen_4,
    // .DataWidthDst = DMAC_DataWidth_32,
    .DataWidthDst = DMAC_DataWidth_16,
    .HandshakeDst = DMAC_Handshake_Default,
    // Source config (Peripheral)
    .AddrChangeSrc = DMAC_AddrChange_NoChange,
    .BurstLenSrc = DMAC_BurstLen_4,    // According to peripheral FIFO threshold
    .DataWidthSrc = DMAC_DataWidth_16, // According to peripheral FIFO width
    .HandshakeSrc = DMAC_Handshake_Hardware,
    //.PeripheralSrc = TGT_SPI_DMA_SRC, // Config to corresponding peripheral
    .PeripheralSrc = NULL, // Config to corresponding peripheral
    // General config
    .FlowControl = DMAC_FlowControl_DMA,
    .IntEnable = ENABLE,
    /* Items in RxConfigTmp which are not influenced here is initialized as 0 by compiler. */
};

DMAC_ChannelConfigTypeDef dma_mem2spi_config = {
    /* Set dma channel control & config register */
    .TransferType = DMAC_TransferType_Mem2Per,
    // Destination config (Peripheral)
    .AddrChangeDst = DMAC_AddrChange_NoChange,
    .BurstLenDst = DMAC_BurstLen_4,    // According to peripheral FIFO threshold
    .DataWidthDst = DMAC_DataWidth_16, // According to peripheral FIFO width
    .HandshakeDst = DMAC_Handshake_Hardware,
    //.PeripheralDst = TGT_SPI_DMA_DST, // Config to corresponding peripheral
    .PeripheralDst = NULL, // Config to corresponding peripheral
    // Source config (Memory)
    .AddrChangeSrc = DMAC_AddrChange_Increment,
    .BurstLenSrc = DMAC_BurstLen_4,
    // .DataWidthSrc = DMAC_DataWidth_32,
	.DataWidthSrc = DMAC_DataWidth_16,
    .HandshakeSrc = DMAC_Handshake_Default,
    // General config
    .FlowControl = DMAC_FlowControl_DMA,
    .IntEnable = ENABLE,
    /* Items in TxConfigTmp which are not influenced here is initialized as 0 by compiler. */
};

DMAC_ChannelConfigTypeDef dma_mem2i2c_config = {
    /*set tx dma channel control&config register*/
    .AddrChangeDst   = DMAC_AddrChange_NoChange,
    .AddrChangeSrc   = DMAC_AddrChange_Increment,
    .BurstLenDst     = DMAC_BurstLen_4,
    .BurstLenSrc     = DMAC_BurstLen_4,
    .DataWidthDst    = DMAC_DataWidth_8,
    // .DataWidthDst    = DMAC_DataWidth_16,
    .DataWidthSrc    = DMAC_DataWidth_32,
    .TransferType    = DMAC_TransferType_Mem2Per,
    .FlowControl     = DMAC_FlowControl_DMA,
    .HandshakeDst    = DMAC_Handshake_Hardware,
    .HandshakeSrc    = DMAC_Handshake_Hardware,
    // .PeripheralDst   = DMAC_Peripheral_I2C0_Tx,
    .PeripheralDst   = NULL,
    .IntEnable       = ENABLE,
};

DMAC_ChannelConfigTypeDef dma_mem2i2c_u16_config = {
    /*set tx dma channel control&config register*/
    .AddrChangeDst   = DMAC_AddrChange_NoChange,
    .AddrChangeSrc   = DMAC_AddrChange_Increment,
    .BurstLenDst     = DMAC_BurstLen_4,
    .BurstLenSrc     = DMAC_BurstLen_4,
    //.DataWidthDst    = DMAC_DataWidth_8,
    .DataWidthDst    = DMAC_DataWidth_16,
    .DataWidthSrc    = DMAC_DataWidth_32,
    .TransferType    = DMAC_TransferType_Mem2Per,
    .FlowControl     = DMAC_FlowControl_DMA,
    .HandshakeDst    = DMAC_Handshake_Hardware,
    .HandshakeSrc    = DMAC_Handshake_Hardware,
    // .PeripheralDst   = DMAC_Peripheral_I2C0_Tx,
    .PeripheralDst   = NULL,
    .IntEnable       = ENABLE,
};


DMAC_ChannelConfigTypeDef dma_i2c2mem_config = {

    /*set tx dma channel control&config register*/
    .AddrChangeDst   = DMAC_AddrChange_Increment,
    .AddrChangeSrc   = DMAC_AddrChange_NoChange,
    .BurstLenDst     = DMAC_BurstLen_4,
    .BurstLenSrc     = DMAC_BurstLen_4,
    //.DataWidthDst    = DMAC_DataWidth_8,
    .DataWidthDst    = DMAC_DataWidth_32,
    .DataWidthSrc    = DMAC_DataWidth_8,
    .TransferType    = DMAC_TransferType_Per2Mem,
    .FlowControl     = DMAC_FlowControl_DMA,
    .HandshakeDst    = DMAC_Handshake_Hardware,
    .HandshakeSrc    = DMAC_Handshake_Hardware,
    // .PeripheralSrc   = DMAC_Peripheral_I2C0_Rx,
    .PeripheralSrc   = NULL,
    .IntEnable       = ENABLE,

};

DMAC_ChannelConfigTypeDef dma_adc2mem_config = {
    .AddrChangeDst   = DMAC_AddrChange_Increment,
    .AddrChangeSrc   = DMAC_AddrChange_NoChange,
    .BurstLenDst     = DMAC_BurstLen_8,
    .BurstLenSrc     = DMAC_BurstLen_8,
    .DataWidthDst    = DMAC_DataWidth_16,
    .DataWidthSrc    = DMAC_DataWidth_32,
    .TransferType    = DMAC_TransferType_Per2Mem,
    .FlowControl     = DMAC_FlowControl_DMA,
    .HandshakeDst    = DMAC_Handshake_Hardware,
    .HandshakeSrc    = DMAC_Handshake_Hardware,
    .PeripheralSrc   = DMAC_Peripheral_ADC,
    .IntEnable       = ENABLE
};

void DMA_IRQHandler(void)
{
    /* Handle DMA Transfer Complete Interrupt */
    if (DMAC_CombinedIntStatus(DMA, DMAC_FLAG_MASK_TFR))
    {
//        SYS_TEST("Handle DMA Transfer Complete Interrupt ");
        for (int i = 0; i < 3; i++)
        {
            if (DMAC_IntFlag(DMA, i, DMAC_FLAG_INDEX_TFR))
            {
                DMAC_Channel_Array[i].XferFlag = true;
                DMAC_ClrIntFlag(DMA, i, DMAC_FLAG_INDEX_TFR);

                
                DMAC_Channel_Array[i].StopFlag =true;
                /* Release DMA Rx channel */
                DMAC_ReleaseChannel(DMA, i);
                /* Disable DMA */
                if(DMAC_IsAllChannelReleased(DMA)){
                     DMAC_DeInit(DMA);
                }
                switch (DMAC_Channel_Array[i].PeriMode)
                {
                case DMAC_Peri_UART:
                    if (DMAC_Channel_Array[i].CallbackUart != NULL)
                    {
                        DMAC_Channel_Array[i].CallbackUart(UART_CB_FLAG_DMA, (uint8_t *)DMAC_Channel_Array[i].pBuffPtr,
                                                           DMAC_Channel_Array[i].XferSize);
                    }
                    break;
                case DMAC_Peri_I2C:
                    if (DMAC_Channel_Array[i].CallbackI2c != NULL)
                    {
                        DMAC_Channel_Array[i].CallbackI2c(I2C_CB_FLAG_DMA, (uint8_t *)DMAC_Channel_Array[i].pBuffPtr,
                                                          DMAC_Channel_Array[i].XferSize);
                    }
                case DMAC_Peri_ADC:
                    if (DMAC_Channel_Array[i].CallbackAdc!= NULL)
                    {
                        DMAC_Channel_Array[i].CallbackAdc(ADC_CB_FLAG_DMA, DMAC_Channel_Array[i].pBuffPtr,
                                                          DMAC_Channel_Array[i].XferSize);
                    }
                case DMAC_Peri_SPI:
                    if (DMAC_Channel_Array[i].CallbackSpi!= NULL)
                    {
                        DMAC_Channel_Array[i].CallbackSpi(SPI_CB_FLAG_DMA, (uint16_t *)DMAC_Channel_Array[i].pBuffPtr,
                                                          DMAC_Channel_Array[i].XferSize);
                    }
                default:
                    break;
                }

//                if(DMAC_Channel_Array[i].ConfigTmp.PeripheralDst=)
//                {
//                        SPI_DisableDmaRx(TGT_SPI);
//                        SPI_DisableDmaTx(TGT_SPI);
//                }
                //  DMA_DefaultValueCheckCase0();

            }
        }
    }
    /* Handle Block Transfer Complete Interrupt */
    if(DMAC_CombinedIntStatus(DMA, DMAC_FLAG_MASK_BLK))
    {
        //idx_blk++;
        if(DMAC_IntFlag(DMA, 0, DMAC_FLAG_INDEX_BLK)){
            DMAC_ClrIntFlag(DMA, 0, DMAC_FLAG_INDEX_BLK);
        }
        if(DMAC_IntFlag(DMA, 1, DMAC_FLAG_INDEX_BLK)){
            DMAC_ClrIntFlag(DMA, 1, DMAC_FLAG_INDEX_BLK);
        }
        if(DMAC_IntFlag(DMA, 2, DMAC_FLAG_INDEX_BLK)){
            DMAC_ClrIntFlag(DMA, 2, DMAC_FLAG_INDEX_BLK);
        }
    }
    /* Handle Source Transation Complete Interrupt */
    if(DMAC_CombinedIntStatus(DMA, DMAC_FLAG_MASK_SRCTFR))
    {
        //idx_srctfr++;
        if(DMAC_IntFlag(DMA, 0, DMAC_FLAG_INDEX_SRCTFR)){
            DMAC_ClrIntFlag(DMA, 0, DMAC_FLAG_INDEX_SRCTFR);
        }
        if(DMAC_IntFlag(DMA, 1, DMAC_FLAG_INDEX_SRCTFR)){
            DMAC_ClrIntFlag(DMA, 1, DMAC_FLAG_INDEX_SRCTFR);
        }
        if(DMAC_IntFlag(DMA, 2, DMAC_FLAG_INDEX_SRCTFR)){
            DMAC_ClrIntFlag(DMA, 2, DMAC_FLAG_INDEX_SRCTFR);
        }
    }
    /* Handle Destination Transaction Complete Interrupt */
    if(DMAC_CombinedIntStatus(DMA, DMAC_FLAG_MASK_DSTTFR))
    {
        //idx_dsttfr++;
        if(DMAC_IntFlag(DMA, 0, DMAC_FLAG_INDEX_DSTTFR)){
            DMAC_ClrIntFlag(DMA, 0, DMAC_FLAG_INDEX_DSTTFR);
        }
        if(DMAC_IntFlag(DMA, 1, DMAC_FLAG_INDEX_DSTTFR)){
            DMAC_ClrIntFlag(DMA, 1, DMAC_FLAG_INDEX_DSTTFR);
        }
        if(DMAC_IntFlag(DMA, 2, DMAC_FLAG_INDEX_DSTTFR)){
            DMAC_ClrIntFlag(DMA, 2, DMAC_FLAG_INDEX_DSTTFR);
        }
    }
    /* Handle Error Interrupt */
    if(DMAC_CombinedIntStatus(DMA, DMAC_FLAG_MASK_ERR))
    {
        //idx_err++;
        if(DMAC_IntFlag(DMA, 0, DMAC_FLAG_INDEX_ERR)){
            DMAC_ClrIntFlag(DMA, 0, DMAC_FLAG_INDEX_ERR);
        }
        if(DMAC_IntFlag(DMA, 1, DMAC_FLAG_INDEX_ERR)){
            DMAC_ClrIntFlag(DMA, 1, DMAC_FLAG_INDEX_ERR);
        }
        if(DMAC_IntFlag(DMA, 2, DMAC_FLAG_INDEX_ERR)){
            DMAC_ClrIntFlag(DMA, 2, DMAC_FLAG_INDEX_ERR);
        }
    }
}

