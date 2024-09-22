/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series i2s driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef _PAN_I2S_H__
#define _PAN_I2S_H__

/**
 * @brief I2s Interface
 * @defgroup i2s_interface I2s Interface
 * @{
 */
 
#ifdef __cplusplus
extern "C"
{
#endif

/**@defgroup I2S_WORD_SEL_FLAG I2s word select 
 * @brief       I2s word select cycles in the left or right sample mode
 * @{ */ 
#define I2S_WORD_SEL_16_CYCLE               (0)			/*!< Word select line (ws) stays in the left or right sample mode in 16 cycle */
#define I2S_WORD_SEL_24_CYCLE               (1)			/*!< Word select line (ws) stays in the left or right sample mode in 24 cycle */
#define I2S_WORD_SEL_32_CYCLE               (2)			/*!< Word select line (ws) stays in the left or right sample mode in 32 cycle */
/**@} */

/**@defgroup I2S_CLK_GATING_FLAG I2s clk gating 
 * @brief       I2s clk gating definition
 * @{ */ 
#define I2S_CLK_GATING_SEL_0_CYCLE          (0)			/*!< No clock gating */
#define I2S_CLK_GATING_SEL_12_CYCLE         (1)			/*!< Gate after 12 clock cycles */
#define I2S_CLK_GATING_SEL_16_CYCLE         (2)			/*!< Gate after 16 clock cycles */
#define I2S_CLK_GATING_SEL_24_CYCLE         (3)			/*!< Gate after 24 clock cycles */	
#define I2S_CLK_GATING_SEL_32_CYCLE         (4)			/*!< Gate after 32 clock cycles */
/**@} */

/**@defgroup I2S_WORD_LENTH_FLAG I2s data resolution select 
 * @brief       I2s data resolution select definition
 * @{ */ 
#define I2S_WORD_LEN_IGNORE                 (0)			/*!< Ignore word length */
#define I2S_WORD_LEN_12_BIT_RESOLUTION      (1)			/*!< 12 bit resolution */
#define I2S_WORD_LEN_16_BIT_RESOLUTION      (2)			/*!< 16 bit resolution */
#define I2S_WORD_LEN_20_BIT_RESOLUTION      (3)			/*!< 20 bit resolution */
#define I2S_WORD_LEN_24_BIT_RESOLUTION      (4)			/*!< 24 bit resolution */
#define I2S_WORD_LEN_32_BIT_RESOLUTION      (5)			/*!< 32 bit resolution */
/**@} */

/**@defgroup I2S_INT_FLAG I2s interrupt flag 
 * @brief       I2s interrupt flag definition
 * @{ */ 
#define I2S_INT_TX_OVER                     (1<<5)	/*!< Status of data overrun interrupt for the TX channel */
#define I2S_INT_TX_EMPTY                    (1<<4)	/*!< Status of transmit empty trigger interrupt. TX FIFO is empty */
#define I2S_INT_RX_OVER                     (1<<1)	/*!< Status of data overrun interrupt for the RX channel */
#define I2S_INT_RX_VALID                    (1<<0)	/*!< Status of Receive data available interrupt */
#define I2S_INT_NONE                        (0)			/*!< no interrupt */
/**@} */


#define I2S_ENABLE_COMMON_BIT0              (1)			/*!< General common bit value */

#define I2S_SCLK_GATING_Pos                 (0)			/*!< Position of I2s sclk gating control bit  */
#define I2S_SCLK_GATING_Msk                 (0x7ul << I2S_SCLK_GATING_Pos)		/*!< I2s sclk gating control bit mask value  */
#define I2S_WORD_SELECT_LEN_Pos             (3)			/*!< Position of I2s word lenth select control bit */
#define I2S_WORD_SELECT_LEN_Msk             (0x3ul << I2S_WORD_SELECT_LEN_Pos)	/*!< I2s word lenth select control bit mask value  */


#define I2S_WORD_LENGTH_SET_Pos             (0)			/*!< Position of I2s data resolution control bit */
#define I2S_WORD_LENGTH_SET_Msk             (0x7ul << I2S_WORD_LENGTH_SET_Pos)		/*!< I2s data resolution control bit mask value  */


#define I2S_SINGLE_STEREO_BUFSEL_L			(0)	/*!< I2s left buffer selected */
#define I2S_SINGLE_STEREO_BUFSEL_R			(1)	/*!< I2s right buffer selected */

#define I2S_SINGLE_STEREO_TX				(1<<16)	/*!< I2s singal stereo transmit enable bit */
#define I2S_SINGLE_TX_BUFSEL_Pos			(17)	/*!< Position of I2s singal stereo transmit buffer select control bit */
#define I2S_SINGLE_TX_BUFSEL_Msk			(1<<I2S_SINGLE_TX_BUFSEL_Pos)	/*!< I2s singal stereo transmit buffer select control bit mask value  */

#define I2S_SINGLE_STEREO_RX				(1<<18)	/*!< I2s singal stereo receive enable bit */
#define I2S_SINGLE_RX_BUFSEL_Pos			(19)	/*!< Position of I2s singal stereo receive buffer select control bit */
#define I2S_SINGLE_RX_BUFSEL_Msk			(1<<I2S_SINGLE_RX_BUFSEL_Pos)	/*!< I2s singal stereo receive buffer select control bit mask value  */

/**
  * @brief  Set I2s module enable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_EnableI2s(I2S_T *I2Sx)
{
    I2Sx->IER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s module disable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_DisableI2s(I2S_T *I2Sx)
{
    I2Sx->IER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s received block enable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_EnableRecBlock(I2S_T *I2Sx)
{
    I2Sx->IRER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s received block disable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_DisableRecBlock(I2S_T *I2Sx)
{
    I2Sx->IRER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter block enable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_EnableTransmitBlock(I2S_T *I2Sx)
{
    I2Sx->ITER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter block disable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_DisableTransmitBlock(I2S_T *I2Sx)
{
    I2Sx->ITER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s CLK generation enable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_EnableClk(I2S_T *I2Sx)
{
    I2Sx->CER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s CLK generation disable
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_DisableClk(I2S_T *I2Sx)
{
    I2Sx->CER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s receive fifo reset
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_SetRecFifoRst(I2S_T *I2Sx)
{
    I2Sx->RXFFR |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter fifo reset
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE void I2S_SetTransmitFifoRst(I2S_T *I2Sx)
{
    I2Sx->TXFFR |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s receive channel enable
  * @param  I2Sx The base address of i2s module
 	* @param  Ch: I2s channel select
  * @return None
  */
__STATIC_INLINE void I2S_EnRecChannel(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].RER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s receive channel disable
  * @param  I2Sx The base address of i2s module
 	* @param  Ch: I2s channel select
  * @return None
  */
__STATIC_INLINE void I2S_DisableRecChannel(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].RER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter channel enable
  * @param  I2Sx The base address of i2s module
  * @param  Ch  I2s channel number  
  * @return None
  */
__STATIC_INLINE void I2S_EnableTransmitCh(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].TER |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter channel disable
  * @param  I2Sx The base address of i2s module
 	* @param  Ch: I2s channel select
  * @return None
  */    
__STATIC_INLINE void I2S_DisableTransmitCh(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].TER &= ~I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s interrupt mask
  * @param  I2Sx The base address of i2s module
  * @param  Ch: I2s channel select
  * @param  Msk: interrupt status:
                             \ref I2S_INT_TX_OVER
                             \ref I2S_INT_TX_EMPTY
                             \ref I2S_INT_RX_OVER
                             \ref I2S_INT_RX_VALID
  * @return None
  */
__STATIC_INLINE void I2S_SetIntMsk(I2S_T *I2Sx,uint8_t Ch,uint8_t Msk)
{
    I2Sx->CH[Ch].IMR |= Msk;
}

/**
  * @brief  Clear I2s interrupt mask
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Msk: interrupt status:
                                \ref I2S_INT_TX_OVER
                                \ref I2S_INT_TX_EMPTY
                                \ref I2S_INT_RX_OVER
                                \ref I2S_INT_RX_VALID
  * @return None
  */
__STATIC_INLINE void I2S_ClearIntMsk(I2S_T *I2Sx,uint8_t Ch,uint8_t Msk)
{
    I2Sx->CH[Ch].IMR &= ~Msk;
}

/**
  * @brief  Set I2s interrupt mask
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Msk: interrupt status:
                               \ref I2S_INT_TX_OVER
                               \ref I2S_INT_TX_EMPTY
                               \ref I2S_INT_RX_OVER
                               \ref I2S_INT_RX_VALID
  * @return true or false
  */
__STATIC_INLINE bool I2S_IsIntMsk(I2S_T *I2Sx,uint8_t Ch,uint8_t Msk)
{
    return (I2Sx->CH[Ch].IMR & Msk)?(true):(false);
}

/**
  * @brief  Set I2s rx trigger level
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Level: trigger level
  * @return none
  */
__STATIC_INLINE void I2S_SetRxTrigLevel(I2S_T *I2Sx,uint8_t Ch,uint8_t Level)
{
    I2Sx->CH[Ch].RFCR = Level;
}
/**
  * @brief  Set I2s tx trigger level
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Level: trigger level
  * @return None
  */
__STATIC_INLINE void I2S_SetTxTrigLevel(I2S_T *I2Sx,uint8_t Ch,uint8_t Level)
{
    I2Sx->CH[Ch].TFCR = Level;
}
/**
  * @brief  Set I2s receive channel fifo reset
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @return None
  */
__STATIC_INLINE void I2S_SetRecChFifoRst(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].RFF |= I2S_ENABLE_COMMON_BIT0;
}
/**
  * @brief  Set I2s transmitter channel fifo reset
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @return None
  */
__STATIC_INLINE void I2S_SetTransmitChFifoRst(I2S_T *I2Sx,uint8_t Ch)
{
    I2Sx->CH[Ch].TFF |= I2S_ENABLE_COMMON_BIT0;
}

/**
  * @brief  Write I2s tx dma data
  * @param  I2Sx The base address of i2s module
  * @param  Data: write data    
  * @return None
  */
__STATIC_INLINE void I2S_WriteTxDmaDat(I2S_T *I2Sx,uint32_t Data)
{
    I2Sx->TXDMA = Data;
}
/**
  * @brief  Read I2s rx dma data
  * @param  I2Sx The base address of i2s module
  * @return None
  */
__STATIC_INLINE uint32_t I2S_ReadRxDmaDat(I2S_T *I2Sx)
{
    return I2Sx->RXDMA;
}
/**
  * @brief  Get I2s interrupt status
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @return Interrupt status
  */
__STATIC_INLINE uint8_t I2S_GetIntStatus(I2S_T *I2Sx,uint8_t Ch)
{
    return I2Sx->CH[Ch].ISR;
}
/**
  * @brief  Adjust which i2s interrupt occured 
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Msk: interrupt status:
                               \ref I2S_INT_TX_OVER
                               \ref I2S_INT_TX_EMPTY
                               \ref I2S_INT_RX_OVER
                               \ref I2S_INT_RX_VALID
  * @return None
  */
__STATIC_INLINE bool I2S_IsIntOccured(I2S_T *I2Sx,uint8_t Ch,uint8_t Msk)
{
    return (I2Sx->CH[Ch].ISR & Msk)?(true):(false);
}
/**
  * @brief  Write I2s left transmitter data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Data: write data,16bit data aligned
  * @return None
  */
__STATIC_INLINE void I2S_WriteLeftTransmitData(I2S_T *I2Sx,uint8_t Ch,uint16_t Data)
{
    I2Sx->CH[Ch].LTHR_LRBR = (uint16_t)Data;
}
/**
  * @brief  Read I2s left receive data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @return Left receive data
  */
__STATIC_INLINE uint16_t I2S_ReadLeftRecData(I2S_T *I2Sx,uint8_t Ch)
{
    return (uint16_t)I2Sx->CH[Ch].LTHR_LRBR;
}

/**
  * @brief  Write I2s right transmitter data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  Data: write data,16bit data aligned
  * @return None
  */
__STATIC_INLINE void I2S_WriteRightTransmitData(I2S_T *I2Sx,uint8_t Ch,uint16_t Data)
{
    I2Sx->CH[Ch].RTHR_RRBR = (uint16_t)Data;
}
/**
  * @brief  Read I2s left receive data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @return Right receive data
  */
__STATIC_INLINE uint16_t I2S_ReadRightRecData(I2S_T *I2Sx,uint8_t Ch)
{
    return (uint16_t)I2Sx->CH[Ch].RTHR_RRBR;
}

/**
  * @brief  Enable I2s single stereo TX function
  * @param  i2s The base address of i2s module
  * @param  enable: TX single stereo enable or not
  * @retval none
  */
__STATIC_INLINE void I2S_TxSingleStereoEnable(I2S_T *I2Sx,bool enable)
{
    (enable) ? (I2Sx->IER |= I2S_SINGLE_STEREO_TX): (I2Sx->IER &= ~I2S_SINGLE_STEREO_TX);
}

/**
  * @brief  buffer select in single stereo TX mode
  * @param  i2s The base address of i2s module
  * @param  bufSel: left or right stereo select
  * @retval none
  */
__STATIC_INLINE void I2S_TxSingleStereoBufSel(I2S_T *I2Sx,uint32_t bufSel)
{
    (bufSel == I2S_SINGLE_STEREO_BUFSEL_R) ? (I2Sx->IER &= ~I2S_SINGLE_TX_BUFSEL_Msk): (I2Sx->IER |= I2S_SINGLE_TX_BUFSEL_Msk);
}

/**
  * @brief  Enable I2s single stereo RX function
  * @param  i2s The base address of i2s module
  * @param  enable: RX single stereo enable or not
  * @retval none
  */
__STATIC_INLINE void I2S_RxSingleStereoEnable(I2S_T *I2Sx,bool enable)
{
    (enable) ? (I2Sx->IER |= I2S_SINGLE_STEREO_RX): (I2Sx->IER &= ~I2S_SINGLE_STEREO_RX);
}

/**
  * @brief  buffer select in single stereo TX mode
  * @param  i2s The base address of i2s module
  * @param  bufSel: left or right stereo select
  * @retval none
  */
__STATIC_INLINE void I2S_RxSingleStereoBufSel(I2S_T *I2Sx,uint32_t bufSel)
{
    (bufSel == I2S_SINGLE_STEREO_BUFSEL_R) ? (I2Sx->IER &= ~I2S_SINGLE_RX_BUFSEL_Msk): (I2Sx->IER |= I2S_SINGLE_RX_BUFSEL_Msk);
}
/**
  * @brief  set I2s word select lenth,default 16 clk cycle
  * @param  i2s The base address of i2s module
  * @param  u8WordSel: sclk cycle select
  *                     \ref I2S_WORD_SEL_16_CYCLE
  *                     \ref I2S_WORD_SEL_24_CYCLE
  *                     \ref I2S_WORD_SEL_32_CYCLE
  * @return None
  */
void I2S_SetWordSelectLen(I2S_T *i2s,uint8_t u8WordSel);
/**
  * @brief  set I2s clk gating cycle,default:no gating
  * @param  i2s The base address of i2s module
  * @param  u8SclkSel: gating select:
  *                     \ref I2S_CLK_GATING_SEL_0_CYCLE
  *                     \ref I2S_CLK_GATING_SEL_12_CYCLE
  *                     \ref I2S_CLK_GATING_SEL_16_CYCLE
  *                     \ref I2S_CLK_GATING_SEL_24_CYCLE
  *                     \ref I2S_CLK_GATING_SEL_32_CYCLE
  * @return None
  */
void I2S_SetSclkGate(I2S_T *i2s,uint8_t u8SclkSel);
/**
  * @brief  set I2s receive word length resolution,default:no word length
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  u8WordLen: word length resolution select:
  *                                         \ref  I2S_WORD_LEN_IGNORE
  *                                         \ref  I2S_WORD_LEN_12_BIT_RESOLUTION
  *                                         \ref  I2S_WORD_LEN_16_BIT_RESOLUTION
  *                                         \ref  I2S_WORD_LEN_20_BIT_RESOLUTION
  *                                         \ref  I2S_WORD_LEN_24_BIT_RESOLUTION
  *                                         \ref  I2S_WORD_LEN_32_BIT_RESOLUTION
  * @return None
  */
void I2S_SetRecWordLen(I2S_T *i2s,uint8_t ch,uint8_t u8WordLen);
/**
  * @brief  set I2s transmitter word length resolution,default:no word length
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  u8WordLen: word length resolution select:
  *                                        \ref   I2S_WORD_LEN_IGNORE
  *                                        \ref   I2S_WORD_LEN_12_BIT_RESOLUTION
  *                                        \ref   I2S_WORD_LEN_16_BIT_RESOLUTION
  *                                        \ref   I2S_WORD_LEN_20_BIT_RESOLUTION
  *                                        \ref   I2S_WORD_LEN_24_BIT_RESOLUTION
  *                                        \ref   I2S_WORD_LEN_32_BIT_RESOLUTION
  * @retval None
  */
void I2S_SetTrmWordLen(I2S_T *i2s,uint8_t ch,uint8_t u8WordLen);

/**
  * @brief  Clear rx fifo over interrupt status
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @retval NONE
  */
uint32_t I2S_ClearRxOverInt(I2S_T *i2s,uint8_t ch);

/**
  * @brief  Clear tx fifo over interrupt status
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @retval NONE
  */
uint32_t I2S_ClearTxOverInt(I2S_T *i2s,uint8_t ch);

/**
  * @brief  I2S send data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  LeftBuf: left channel data buffer
  * @param  RightBuf: right channel data buffer
  * @param  Size: send data number
  * @retval NONE
  */
void I2S_Send(I2S_T *I2Sx,uint8_t Ch,uint32_t *LeftBuf,uint32_t *RightBuf,uint32_t Size);

/**
  * @brief  I2S send data
  * @param  I2Sx The base address of i2s module
  * @param  Ch: i2s channel select
  * @param  LeftBuf: left channel data buffer
  * @param  RightBuf: right channel data buffer
  * @param  Size: send data number
  * @retval NONE
  */
void I2S_Receive(I2S_T *I2Sx,uint8_t Ch,uint32_t *LeftBuf,uint32_t *RightBuf,uint32_t Size);

/**@} */


#ifdef __cplusplus
}
#endif

#endif /* _PAN_I2S_H__ */
