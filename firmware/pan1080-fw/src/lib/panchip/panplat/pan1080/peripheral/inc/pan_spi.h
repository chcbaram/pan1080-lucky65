/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series spi driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef __PAN_SPI_H__
#define __PAN_SPI_H__

/**
 * @brief Spi Interface
 * @defgroup spi_interface Spi Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**@defgroup SPI_IRQ_FLAG Spi interrupt event type
 * @brief       Spi interrupt event type constants definitions
 * @{ */ 
typedef enum SPI_Irq
{
    SPI_IRQ_TX_HALF_EMPTY   = 0x08,     /*!< Transmit FIFO Half Empty Interrupt */
    SPI_IRQ_RX_HALF_FULL    = 0x04,     /*!< Receive FIFO Half Full Interrupt */
    SPI_IRQ_RX_TIMEOUT      = 0x02,     /*!< Receive FIFO Not Empty and No Read prior to Timeout Interrupt */
    SPI_IRQ_RX_OVERRUN      = 0x01,     /*!< Receive FIFO Overflow Interrupt */
    SPI_IRQ_ALL             = 0x0f      /*!< All interrupts */
} SPI_IrqDef;
/**@} */

/**@defgroup SPI_WORK_MODE_FLAG Spi operating mode
 * @brief       Spi operating mode constants definitions
 * @{ */ 
typedef enum SPI_Role
{
    SPI_RoleMaster  = 0x0,		/*!< Spi work as a master */	
    SPI_RoleSlave   = 0x1			/*!< Spi work as a slave */
} SPI_RoleDef;  
/**@} */


/**@defgroup SPI_DATA_SIZE_FLAG Spi data frame size
 * @brief       Spi data frame size constants definitions
 * @{ */ 
typedef enum SPI_DataFrameSize
{
    SPI_DataFrame_4b    = 0x03,
    SPI_DataFrame_5b    = 0x04,
    SPI_DataFrame_6b    = 0x05,
    SPI_DataFrame_7b    = 0x06,
    SPI_DataFrame_8b    = 0x07,
    SPI_DataFrame_9b    = 0x08,
    SPI_DataFrame_10b   = 0x09,
    SPI_DataFrame_11b   = 0x0a,
    SPI_DataFrame_12b   = 0x0b,
    SPI_DataFrame_13b   = 0x0c,
    SPI_DataFrame_14b   = 0x0d,
    SPI_DataFrame_15b   = 0x0e,
    SPI_DataFrame_16b   = 0x0f
} SPI_DataFrameSizeDef;
/**@} */

/**@defgroup SPI_BAUD_DIV_FLAG Spi baudrate divider
 * @brief       Spi baudrate divider constants definitions
 * @{ */
typedef enum SPI_BaudRateDiv
{
    SPI_BaudRateDiv_2   = 0x02,			/*!< Baud rate 2 frequency division */
    SPI_BaudRateDiv_4   = 0x04,			/*!< Baud rate 4 frequency division */
    SPI_BaudRateDiv_6   = 0x06,			/*!< Baud rate 6 frequency division */
    SPI_BaudRateDiv_8   = 0x08,			/*!< Baud rate 8 frequency division */
    SPI_BaudRateDiv_16  = 0x10,			/*!< Baud rate 16 frequency division */
    SPI_BaudRateDiv_32  = 0x20,			/*!< Baud rate 32 frequency division */
    SPI_BaudRateDiv_48  = 0x30,			/*!< Baud rate 48 frequency division */
    SPI_BaudRateDiv_64  = 0x40,			/*!< Baud rate 64 frequency division */
    SPI_BaudRateDiv_96  = 0x60,			/*!< Baud rate 96 frequency division */
    SPI_BaudRateDiv_128 = 0x80,			/*!< Baud rate 128 frequency division */
    SPI_BaudRateDiv_160 = 0xA0,			/*!< Baud rate 160 frequency division */
    SPI_BaudRateDiv_192 = 0xC0,			/*!< Baud rate 192 frequency division */
    SPI_BaudRateDiv_224 = 0xE0,			/*!< Baud rate 224 frequency division */
    SPI_BaudRateDiv_240 = 0xF0,			/*!< Baud rate 240 frequency division */
    SPI_BaudRateDiv_250 = 0xFA,			/*!< Baud rate 250 frequency division */
} SPI_BaudRateDivDef;
/**@} */

#define SPI_BAUD_RATE_DIV(x)	((x)?(x):(256))	/*!< Baud rate x frequency division */

/**@defgroup SPI_POLARITY_FLAG Spi clock polarity
 * @brief       Spi clock polarity constants definitions,Serial Clock Polarity, Used to 
 *							select the polarity of the inactive serial clock, which is held inactive when
 *							the DW_apb_ssi master is not actively transferring data on the serial bus
 * @{ */
typedef enum SPI_ClockPol
{
    SPI_ClockPolarityLow    = 0x0,		/*!< Polarity low */
    SPI_ClockPolarityHigh   = 0x1			/*!< Polarity high */
} SPI_ClockPolDef;
/**@} */

/**@defgroup SPI_PHASE_FLAG Spi clock phase
 * @brief       Spi clock phase constants definitions
 * @{ */
typedef enum SPI_ClockPhase
{
    SPI_ClockPhaseFirstEdge     = 0x0,		/*!< First edge sampling */
    SPI_ClockPhaseSecondEdge    = 0x1			/*!< Second edge sampling */
} SPI_ClockPhaseDef;
/**@} */

/**@defgroup SPI_FORMAT_FLAG Spi frame format
 * @brief       Spi frame format constants definitions
 * @{ */
typedef enum SPI_FrameFormat
{
    SPI_FormatMotorola     = 0x00,		/*!< Spi frame format select motorola */	
    SPI_FormatTi           = 0x01,		/*!< Spi frame format select ti */	
    SPI_FormatMicrowire    = 0x02,		/*!< Spi frame format select microwire */	
} SPI_FrameFormatDef;
/**@} */


/**
  * @brief  SPI Init Structure definition
  */
  
/** 
 * @brief Structure with spi init feature.
 *
 * @param SPI_role 						Specifies the SPI operating mode.\ref SPI_WORK_MODE_FLAG
 * @param SPI_dataFrameSize		Specifies the SPI data size	\ref SPI_DATA_SIZE_FLAG
 * @param SPI_CPOL						Specifies the SPI data size \ref SPI_POLARITY_FLAG
 * @param SPI_CPHA						Specifies the clock active edge for the bit capture \ref SPI_PHASE_FLAG
 * @param SPI_baudRateDiv			Specifies the Baud Rate Divisor used to configure the transmit and receive
 *																 SCK clock \ref SPI_BAUD_DIV_FLAG
 * @param SPI_format					Specifies the SPI format \ref SPI_FORMAT_FLAG
 */
typedef struct
{
    SPI_RoleDef SPI_role;
    SPI_DataFrameSizeDef SPI_dataFrameSize;
    SPI_ClockPolDef SPI_CPOL;
    SPI_ClockPhaseDef SPI_CPHA;
    uint16_t SPI_baudRateDiv;
    SPI_FrameFormatDef SPI_format;
} SPI_InitTypeDef;
/**@} */


/**
  * @brief  Synchronous serial port enable
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_EnableSpi(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->CR1;
    tmpreg |= SPI_CR1_SSE_Msk;
    SPIx->CR1 = tmpreg;
}

/**
  * @brief  Synchronous serial port disable
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_DisableSpi(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->CR1;
    tmpreg &= ~SPI_CR1_SSE_Msk;
    SPIx->CR1 = tmpreg;
}

/**
  * @brief  get synchronous serial port status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsSpiEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->CR1;
    if(tmpreg & SPI_CR1_SSE_Msk)
        return true;
    else
        return false;
}

/**
  * @brief  DMA for the transmit FIFO is enabled
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_EnableDmaTx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    tmpreg |= SPI_DMACR_TDMAE_Msk;
    SPIx->DMACR = tmpreg;
}

/**
  * @brief  DMA for the transmit FIFO is disabled
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_DisableDmaTx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    tmpreg &= ~SPI_DMACR_TDMAE_Msk;
    SPIx->DMACR = tmpreg;
}

/**
  * @brief  get DMA for the transmit FIFO status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsDmaTxEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    if(tmpreg & SPI_DMACR_TDMAE_Msk)
        return true;
    else
        return false;
}

/**
  * @brief  DMA for the receive FIFO is enabled
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_EnableDmaRx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    tmpreg |= SPI_DMACR_RDMAE_Msk;
    SPIx->DMACR = tmpreg;
}

/**
  * @brief  DMA for the receive FIFO is disabled
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_DisableDmaRx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    tmpreg &= ~SPI_DMACR_RDMAE_Msk;
    SPIx->DMACR = tmpreg;
}

/**
  * @brief  get DMA for the receive FIFO status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsDmaRxEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;
    if(tmpreg & SPI_DMACR_RDMAE_Msk)
        return true;
    else
        return false;
}

/**
  * @brief  Send data,DR is the data register and is 16-bits wide
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE void SPI_SendData(SPI_T* SPIx, uint32_t Data)
{
    SPIx->DR = Data;
}

/**
  * @brief  Receive data,DR is the data register and is 16-bits wide
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return None
  */
__STATIC_INLINE uint32_t SPI_ReceiveData(SPI_T* SPIx)
{
    return SPIx->DR;
}

/**
  * @brief  Interruput mask Enable
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: where x can be 
  *          SPI_IRQ_TX_HALF_EMPTY 
  *          SPI_IRQ_RX_HALF_FULL 
  *          SPI_IRQ_RX_TIMEOUT
  *          SPI_IRQ_RX_OVERRUN
  *          SPI_IRQ_ALL 
  * @return None
  */
__STATIC_INLINE void SPI_EnableIrq(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->IMSC;
    tmpreg |= irq;
    SPIx->IMSC = tmpreg;
}

/**
  * @brief  Interruput mask Disable
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: where x can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  * @return None
  */
__STATIC_INLINE void SPI_DisableIrq(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->IMSC;
    tmpreg &= ~irq;
    SPIx->IMSC = tmpreg;
}

/**
  * @brief  Get Interruput mask status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: where x can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  * @return None
  */
__STATIC_INLINE bool SPI_IsIrqEnabled(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg;

    tmpreg = SPIx->IMSC;
    if(tmpreg & irq )
        return true;
    else
        return false;
}

/**
  * @brief  Get Masked Interruput status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral. 
  * @return SPI_IrqDef: where return value can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  */
__STATIC_INLINE SPI_IrqDef SPI_GetMskedActiveIrq(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->MIS;
    tmpreg &= SPI_IRQ_ALL;

    return (SPI_IrqDef)tmpreg;
}

/**
  * @brief  Get Masked Interruput status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: where x can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsIrqActive(SPI_T * SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->MIS;
    if(tmpreg & irq )
        return true;
    else
        return false;
}

/**
  * @brief  Get raw Interruput status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: where x can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsRawIrqActive(SPI_T * SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->RIS;
    if(tmpreg & irq )
        return true;
    else
        return false;
}

/**
  * @brief  clear Interruput status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  irq: can be 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  * @return None
  */
__STATIC_INLINE void SPI_ClearIrq(SPI_T * SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->ICR;
    tmpreg |= irq ;
    SPIx->ICR = tmpreg;
}
/**
  * @brief  Get Raw Interruput status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral. 
  * @return SPI_IrqDef: where return value can be 
  *          \ref SPI_IRQ_TX_HALF_EMPTY 
  *          \ref SPI_IRQ_RX_HALF_FULL 
  *          \ref SPI_IRQ_RX_TIMEOUT
  *          \ref SPI_IRQ_RX_OVERRUN
  *          \ref SPI_IRQ_ALL 
  */
__STATIC_INLINE SPI_IrqDef SPI_GetRawActiveIrq(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->RIS;
    tmpreg &= SPI_IRQ_ALL;

    return (SPI_IrqDef)tmpreg;
}

/**
  * @brief  Get Tx FIFO empty status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsTxFifoEmpty(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;
    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_TFE_Msk)
        return true;
    else
        return false;
}
/**
  * @brief  Get Tx FIFO full status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsTxFifoFull(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_TFNF_Msk)
        return false;
    else
        return true;
}
/**
  * @brief  Get Tx FIFO empty status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsRxFifoEmpty(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_RFNE_Msk)
        return false;
    else
        return true;

}
/**
  * @brief  Get Tx FIFO full status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsRxFifoFull(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_RFF_Msk)
        return true;
    else
        return false;

}
/**
  * @brief  Get Synchronous serial port busy status
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @return true/false
  */
__STATIC_INLINE bool SPI_IsBusy(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;
    if(tmpreg & SPI_SR_BUSY_Msk)
        return true;
    else
        return false;
}

/**
  * @brief  Tx Lsb Send function control
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @retval none
  */
__STATIC_INLINE void SPI_TxLsbEnable(SPI_T* SPIx,bool enable)
{
	(enable) ? (SPIx->CR1 |= SPI_CR1_TX_LSB_EN_Msk) : (SPIx->CR1 &= ~SPI_CR1_TX_LSB_EN_Msk);
}

/**
  * @brief  Rx Lsb Send function control
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @retval none
  */
__STATIC_INLINE void SPI_RxLsbEnable(SPI_T* SPIx,bool enable)
{
	(enable) ? (SPIx->CR1 |= SPI_CR1_RX_LSB_EN_Msk) : (SPIx->CR1 &= ~SPI_CR1_RX_LSB_EN_Msk);
}
/**
  * @brief  Initializes the SPIx peripheral according to the specified
  *         parameters in the SPI_InitStruct .
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that contains
  *         the configuration information for the specified SPI peripheral.
  * @return None
  */
void SPI_Init(SPI_T* SPIx, SPI_InitTypeDef* SPI_InitStruct);

/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_SPI_H__ */

