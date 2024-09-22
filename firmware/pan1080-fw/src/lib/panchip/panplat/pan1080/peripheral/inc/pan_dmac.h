/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series dma driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */

#ifndef __PAN_DMAC_H__
#define __PAN_DMAC_H__

/**
 * @brief Dma Interface
 * @defgroup dma_interface dma Interface
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#define DMAC_CHANNEL_NUMBER         (3)			/*!< Number of available dma channels */
#define DMA_INVLID_CHANNEL          (~0ul)	/*!< return value after acquire no dma channel */
    
/*CTL register*/

/**@defgroup DMA_TRANSFER_TYPE_FLAG Dma transfer type 
 * @brief       Dma transfer type definition
 * @{ */

#define DMAC_TransferType_Mem2Mem    (0)		/*!< Transfer from memory to memory */
#define DMAC_TransferType_Mem2Per    (1)		/*!< Transfer from memory to peripheral */
#define DMAC_TransferType_Per2Mem    (2)		/*!< Transfer from peripheral to memory */
#define DMAC_TransferType_Per2Per    (3)		/*!< Transfer from peripheral to peripheral */
/**@} */


#define DMAC_FlowControl_DMA         (0)		/*!< Flow controlled by dma */
#define DMAC_FlowControl_Peripheral  (1)		/*!< Flow controlled by peripheral */

/**@defgroup DMA_BURST_LENTH_FLAG Dma burst lenth 
 * @brief       Dma burst lenth definition
 * @{ */

#define DMAC_BurstLen_1              (0)		/*!< Burst lenth set value 1,unit is data width */
#define DMAC_BurstLen_4              (1)		/*!< Burst lenth set value 4,unit is data width */
#define DMAC_BurstLen_8              (2)		/*!< Burst lenth set value 8,unit is data width */
#define DMAC_BurstLen_16             (3)		/*!< Burst lenth set value 16,unit is data width */
#define DMAC_BurstLen_32             (4)		/*!< Burst lenth set value 32,unit is data width */
#define DMAC_BurstLen_64             (5)		/*!< Burst lenth set value 64,unit is data width */
#define DMAC_BurstLen_128            (6)		/*!< Burst lenth set value 128,unit is data width */
#define DMAC_BurstLen_256            (7)		/*!< Burst lenth set value 256,unit is data width */
/**@} */


/**@defgroup DMA_ADR_CHANGE_FLAG Dma memory address state 
 * @brief       Dma memory address state definition
 * @{ */
#define DMAC_AddrChange_Increment   (0)		/*!< Memory address increment */
#define DMAC_AddrChange_Decrement   (1)		/*!< Memory address decrement */
#define DMAC_AddrChange_NoChange    (2)		/*!< Memory address has no change */
/**@} */


/**@defgroup DMA_DATA_WIDTH_FLAG Dma data width select 
 * @brief       Dma data width select definition
 * @{ */
#define DMAC_DataWidth_8            (0)		/*!< Data width select 8 bit */
#define DMAC_DataWidth_16           (1)		/*!< Data width select 16 bit */
#define DMAC_DataWidth_32           (2)		/*!< Data width select 32 bit */
#define DMAC_DataWidth_64           (3)		/*!< Data width select 64 bit */
#define DMAC_DataWidth_128          (4)		/*!< Data width select 128 bit */
#define DMAC_DataWidth_256          (5)		/*!< Data width select 256 bit */
/**@} */


/**@defgroup DMA_CH_NUM_FUNC_FLAG Dma channel function 
 * @brief       Dma channel function definition
 * @{ */
    
#define DMAC_Peripheral_I2C0_Tx      (0)		/*!< I2c tx dma channel */
#define DMAC_Peripheral_I2C0_Rx      (1)		/*!< I2c rx dma channel */
#define DMAC_Peripheral_SPI0_Tx      (2)		/*!< Spi0 tx dma channel */
#define DMAC_Peripheral_SPI0_Rx      (3)		/*!< Spi0 rx dma channel */
#define DMAC_Peripheral_UART0_Tx     (4)		/*!< Uart0 tx dma channel */
#define DMAC_Peripheral_UART0_Rx     (5)		/*!< Uart0 rx dma channel */
#define DMAC_Peripheral_IIS_Tx     	 (6)		/*!< I2s tx dma channel */
#define DMAC_Peripheral_IIS_Rx     	 (7)		/*!< I2s rx dma channel */
#define DMAC_Peripheral_SPI1_Tx      (8)		/*!< Spi1 tx dma channel */
#define DMAC_Peripheral_SPI1_Rx      (9)		/*!< Spi1 rx dma channel */
#define DMAC_Peripheral_UART1_Tx     (10)		/*!< Uart1 tx dma channel */
#define DMAC_Peripheral_UART1_Rx     (11)		/*!< Uart1 rx dma channel */
#define DMAC_Peripheral_ADC					 (12)		/*!< Adc dma channel */
/**@} */


/**@defgroup DMA_HANDSHAKE_FLAG Dma handshake polarity 
 * @brief       Dma handshake polarity definition
 * @{ */
#define DMAC_HandshakePolarity_Low   (1)		/*!< Handshake polarity low */		
#define DMAC_HandshakePolarity_High  (0)		/*!< Handshake polarity high */
/**@} */


/**@defgroup DMA_HANDSHAKE_SEL_FLAG Dma handshake control
 * @brief       Dma handshake control definition
 * @{ */
#define DMAC_Handshake_Default       (1)		/*!< Handshake control default value,default software */
#define DMAC_Handshake_Software      (1)		/*!< Handshake controlled by software */
#define DMAC_Handshake_Hardware      (0)		/*!< Handshake controlled by hardware */
/**@} */

#define DMAC_LockLevel_Tfr_Complete             (0)		/*!< Over complete DMA transfer */
#define DMAC_LockLevel_Block_Complete           (1)		/*!< Over complete DMA block transfer */
#define DMAC_LockLevel_Transaction_Complete     (2)		/*!< Over complete DMA transaction */

/**@defgroup DMA_CH_PRIORITY_FLAG Dma channel priority
 * @brief       Dma channel priority definition
 * @{ */
#define DMAC_ChannelPriority_0                  (0)		/*!< channel priority select 0,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_1                  (1)		/*!< channel priority select 1,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_2                  (2)		/*!< channel priority select 2,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_3                  (3)		/*!< channel priority select 3,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_4                  (4)		/*!< channel priority select 4,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_5                  (5)		/*!< channel priority select 5,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_6                  (6)		/*!< channel priority select 6,A priority of 7 is the highest priority */
#define DMAC_ChannelPriority_7                  (7)		/*!< channel priority select 7,A priority of 7 is the highest priority */
/**@} */

#define DMAC_CHANNELALL_MASK        (~((~0ul)<<DMAC_CHANNEL_NUMBER))		/*!< Set interrupt mask of all dma channel */

/**@defgroup DMA_INTERRUPT_FLAG Dma interrupt status
 * @brief       Dma interrupt status definition
 * @{ */
#define DMAC_FLAG_INDEX_TFR         (0<<1)		/*!< Interrupt Status transfer complete */
#define DMAC_FLAG_INDEX_BLK         (1<<1)		/*!< Interrupt Status block transfer complete */
#define DMAC_FLAG_INDEX_SRCTFR      (2<<1)		/*!< Interrupt Status source transfer complete */
#define DMAC_FLAG_INDEX_DSTTFR      (3<<1)		/*!< Interrupt Status destination transfer complete */
#define DMAC_FLAG_INDEX_ERR         (4<<1)		/*!< Interrupt Status transfer error */
/**@} */


/**@defgroup DMA_INTERRUPT_MASK_FLAG Dma interrupt mask
 * @brief       Dma interrupt mask definition
 * @{ */

#define DMAC_FLAG_MASK_TFR          (1ul<<(DMAC_FLAG_INDEX_TFR   >>1))		/*!< Write 1 to unmask transfer complete interrupt */	
#define DMAC_FLAG_MASK_BLK          (1ul<<(DMAC_FLAG_INDEX_BLK   >>1))		/*!< Write 1 to unmask block transfer complete interrupt */
#define DMAC_FLAG_MASK_SRCTFR       (1ul<<(DMAC_FLAG_INDEX_SRCTFR>>1))		/*!< Write 1 to unmask source transfer complete interrupt */
#define DMAC_FLAG_MASK_DSTTFR       (1ul<<(DMAC_FLAG_INDEX_DSTTFR>>1))		/*!< Write 1 to unmask destination transfer complete interrupt */
#define DMAC_FLAG_MASK_ERR          (1ul<<(DMAC_FLAG_INDEX_ERR   >>1))		/*!< Write 1 to unmask transfer error interrupt */
#define DMAC_FLAG_MASK_ALL          (0x1Ful)
/**@} */


/** 
 	* @brief Structure with dma channel config feature.
 	*
 	* @param CTL_L 	Low 32 bit of dma channel control register.
 	* @param CTL_H		High 32 bit of dma channel control register.
 	* @param CFG_L		Low 32 bit of dma channel config register.
 	* @param CFG_H		High 32 bit of dma channel config register.
 	*/
typedef struct __DMAC_ChannelConfigTypeDef
{
    uint32_t CTL_L;
    uint32_t CTL_H;
    uint32_t CFG_L;
    uint32_t CFG_H;
}__DMAC_ChannelConfigTypeDef;

/** 
 	* @brief Structure with dma channel detail config.
 	*
 	* @param IntEnable 							Interrupt enable bit.
 	* @param DataWidthDst						Destination data width.	\ref DMA_DATA_WIDTH_FLAG
 	* @param DataWidthSrc						Source data width. \ref DMA_DATA_WIDTH_FLAG
 	* @param AddrChangeDst					Destination address change. \ref DMA_ADR_CHANGE_FLAG
 	* @param AddrChangeSrc 					Source address change. \ref DMA_ADR_CHANGE_FLAG
 	* @param BurstLenDst						Destination burst transaction length. \ref DMA_BURST_LENTH_FLAG
 	* @param BurstLenSrc						Source burst transaction length. \ref DMA_BURST_LENTH_FLAG
 	* @param GatherEnSrc						Source gather enable bit.
 	* @param ScatterEnDst 					Destination scatter enable bit.
 	* @param __Rev0									reversed bit.
 	* @param TransferType						Transfer Type. \ref DMA_TRANSFER_TYPE_FLAG
 	* @param FlowControl						Flow Control \ref DMAC_FlowControl_Peripheral, \ref DMAC_FlowControl_DMA
 	* @param __Rev1 								reversed bit.
 	* @param BlockSize							Block size number ,block_size = data_len / DataWidth.
 	* @param Done										Done flag.
 	* @param __Rev2									reversed bit.
 	* @param __Rev3 								reversed bit.
 	* @param ChannelPriority				Channel priority,it can be 0~7,the priority 7 is the highest.
 	* @param ChannelSuspend					Channel Suspend,transfer is not complete ,set this bit can stop transmit.
 	* @param FifoEmpty							Indicates if there is data left in the channel FIFO
 	* @param HandshakeDst 					Destination Handshake select:software or hardware
 	* @param HandshakeSrc						Source Handshake select:software or hardware
 	* @param LockBus								When active, the AHB bus master signal hlock is
	*																	asserted for the duration specified in CFGx.LOCK_B_L.
 	* @param LockChannel						When the channel is granted control of the master bus interface and if 
 	*																	the CFGx.LOCK_CH bit is asserted,then no other channels are granted 
 	*																	control of the master bus interface for the duration specified in CFGx.LOCK_CH_L
 	* @param LockBusLevel						Indicates the duration over which CFGx.LOCK_B bit applies.
 	* @param LockChannelLevel 			Indicates the duration over which CFGx.LOCK_CH bit applies.
 	* @param HandshakePolarityDst		Destination Handshaking Interface Polarity:0-high, 1-low.
 	* @param HandshakePolaritySrc		Source Handshaking Interface Polarity:0-high, 1-low.
 	* @param __Rev4									reversed bit.
 	* @param AutoReloadSrc 					Automatic Source Reload.
 	* @param AutoReloadDst 					Automatic Destination Reload.
 	* @param FlowControlMode				Determines when source transaction requests are serviced when
 	*																	the Destination Peripheral is the flow controller.
 	* @param FifoMode								Determines how much space or data needs to be available in the FIFO.
 	* @param ProtectControl					Used to drive the AHB HPROT[3:1] bus.
 	* @param __Rev5 								reversed bit.
 	* @param PeripheralSrc					Assigns a hardware handshaking interface to the source of channel x if the
	*																	CFGx.HS_SEL_SRC field is 0
 	* @param PeripheralDst					Assigns a hardware handshaking interface to the destination of channel x 
 	*																	if the CFGx.HS_SEL_DST field is 0
 	* @param __Rev6									reversed bit.
 	*/
typedef struct DMAC_ChannelConfigTypeDef
{
    uint32_t IntEnable : 1;
    uint32_t DataWidthDst : 3;
    uint32_t DataWidthSrc : 3;
    uint32_t AddrChangeDst : 2;
    uint32_t AddrChangeSrc : 2; 
    uint32_t BurstLenDst : 3;
    uint32_t BurstLenSrc : 3;
    uint32_t GatherEnSrc : 1;
    uint32_t ScatterEnDst : 1;
    uint32_t __Rev0 : 1;
    uint32_t TransferType : 2;
    uint32_t FlowControl : 1;
    uint32_t __Rev1 : 9;

    uint32_t BlockSize : 12;
    uint32_t Done : 1; 
    uint32_t __Rev2 : 19;

    uint32_t __Rev3 : 5;
    uint32_t ChannelPriority : 3;
    uint32_t ChannelSuspend : 1;
    uint32_t FifoEmpty : 1;
    uint32_t HandshakeDst : 1;
    uint32_t HandshakeSrc : 1;
    uint32_t LockBus : 1;
    uint32_t LockChannel : 1;
    uint32_t LockBusLevel : 2;
    uint32_t LockChannelLevel : 2;
    uint32_t HandshakePolarityDst : 1;
    uint32_t HandshakePolaritySrc : 1;
    uint32_t __Rev4 : 10;
    uint32_t AutoReloadSrc : 1;
    uint32_t AutoReloadDst : 1;

    uint32_t FlowControlMode : 1;
    uint32_t FifoMode : 1;
    uint32_t ProtectControl : 3;
    uint32_t __Rev5 : 2;
    uint32_t PeripheralSrc : 4;
    uint32_t PeripheralDst : 4;
    uint32_t __Rev6 : 17;
}DMAC_ChannelConfigTypeDef;

/**
  * @brief	  Adjust dma channel is busy or not
  * @param  	dma	Where dma is the base address of dma peripheral
  * @param  	ChIdx	Where chidx is the dma channel number.It could be 0,1,2  
  * @retval 	0 dma channel invalid
  * @retval 	1 dma channel valid
	*/ 
__STATIC_INLINE uint32_t DMAC_IsChannelValid(DMA_T* dma,uint32_t ChIdx)
{
    return (ChIdx < DMAC_CHANNEL_NUMBER ? 1 : 0);
}
/**
  * @brief		:Set interrupt mask of one dma channel 
  * @param		:Where idx is the dma channel number.It could be 0,1,2  
  * @return		:None
*/ 
#define DMAC_ChannelMask(idx)       (1ul<<(idx))
/**
  * @brief	Adjust interrupt occured or not 
  * @param  dma	Where dma is the base address of dma peripheral
  * @param  FlgMsk	Where flgMsk is combined interrupt status ,it could be:
  *                 \ref DMAC_FLAG_MASK_TFR   
  *                 \ref DMAC_FLAG_MASK_BLK   
  *                 \ref DMAC_FLAG_MASK_SRCTFR
  *                 \ref DMAC_FLAG_MASK_DSTTFR
  *                 \ref DMAC_FLAG_MASK_ERR   
  *                 \ref DMAC_FLAG_MASK_ALL   
  * @retval 	0 dma interrupt did not occurred
  * @retval 	1 dma interrupt occurred
	*/ 
__STATIC_INLINE uint32_t DMAC_CombinedIntStatus(DMA_T* dma,uint32_t FlgMsk)
{
    return (dma->STATUS_INT_L & FlgMsk ? 1 : 0);
}
/**
  * @brief	Adjust interrupt raw flag set or not
  * @param  dma	where dma is the base address of dma peripheral
  * @param  ChIdx	where ChIdx is the dma channel number.It could be 0,1,2
  * @param  FlgIdx	where FlgIdx is interrupt type,It could be:\ref DMA_INTERRUPT_FLAG     
  * @return	None
	*/
__STATIC_INLINE uint32_t DMAC_StatusFlag(DMA_T* dma,uint32_t ChIdx,uint32_t FlgIdx)
{
    return ((__IO uint32_t*)(&dma->RAW_TFR_L))[FlgIdx]&(0x1<<ChIdx) ? 1 : 0;
}

/**
  * @brief	Adjust interrupt event occured or not after masking
  * @param  dma	where dma is the base address of dma peripheral
  * @param  ChIdx	where ChIdx is the dma channel number.It could be 0,1,2
  * @param  FlgIdx	where FlgIdx is interrupt type,It could be:\ref DMA_INTERRUPT_FLAG     
  * @return	None
	*/
__STATIC_INLINE uint32_t DMAC_IntFlag(DMA_T* dma,uint32_t ChIdx,uint32_t FlgIdx)
{
    return ((__IO uint32_t*)(&dma->STATUS_TFR_L))[FlgIdx]&(0x1<<ChIdx) ? 1 : 0;
}

/**
  * @brief	Adjust interrupt mask set or not
  * @param  dma	where dma is the base address of dma peripheral
  * @param  ChIdx	where ChIdx is the dma channel number.It could be 0,1,2
  * @param  FlgIdx	where FlgIdx is interrupt type,It could be:\ref DMA_INTERRUPT_FLAG     
  * @return	None
	*/
__STATIC_INLINE uint32_t DMAC_IntFlagMsk(DMA_T* dma,uint32_t ChIdx,uint32_t FlgIdx)
{
    return ((__IO uint32_t*)(&dma->MSK_TFR_L))[FlgIdx]&(0x1<<ChIdx) ? 1 : 0;
}

/**
  * @brief	Adjust interrupt mask cleared or not
  * @param  dma	where dma is the base address of dma peripheral
  * @param  ChIdx	where ChIdx is the dma channel number.It could be 0,1,2
  * @param  FlgIdx	where FlgIdx is interrupt type,It could be:\ref DMA_INTERRUPT_FLAG     
  * @return	None
	*/
__STATIC_INLINE void DMAC_ClrIntFlag(DMA_T* dma,uint32_t ChIdx,uint32_t FlgIdx)
{
    ((__IO uint32_t*)(&dma->CLEAR_TFR_L))[FlgIdx]= (0x1<<ChIdx);
}

/**
  * @brief	  Enable dma
  * @param  	dma	Where dma is the base address of dma peripheral
  * @return   None
	*/
void DMAC_Init(DMA_T * dma);

/**
  * @brief	  Disable dma & mask all dma interrupt
  * @param  	dma	Where dma is the base address of dma peripheral
  * @return   None
	*/
void DMAC_DeInit(DMA_T * dma);

/**
  * @brief		Clear interrupt mask to make sure intterupt enable
  * @param  	dma	Where dma is the base address of dma peripheral
  * @param		ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2
  * @param		FlgIdx	Where FlgIdx is interrupt type,It could be:
  *                  \ref DMAC_FLAG_INDEX_TFR    
  *                  \ref DMAC_FLAG_INDEX_BLK    
  *                  \ref DMAC_FLAG_INDEX_SRCTFR 
  *                  \ref DMAC_FLAG_INDEX_DSTTFR 
  *                  \ref DMAC_FLAG_INDEX_ERR    
  * @return		None
	*/
void DMAC_ClrIntFlagMsk(DMA_T* dma, uint32_t ChIdx, uint32_t FlgIdx);

/**
  * @brief	Mask interrupt,interrupt is useless
  * @param  dma	Where dma is the base address of dma peripheral
  * @param  ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2
  * @param  FlgIdx	Where FlgIdx is interrupt type,It could be:
  *                  \ref DMAC_FLAG_INDEX_TFR    
  *                  \ref DMAC_FLAG_INDEX_BLK    
  *                  \ref DMAC_FLAG_INDEX_SRCTFR 
  *                  \ref DMAC_FLAG_INDEX_DSTTFR 
  *                  \ref DMAC_FLAG_INDEX_ERR    
  * @return None
	*/
void DMAC_SetIntFlagMsk(DMA_T* dma, uint32_t ChIdx, uint32_t FlgIdx);

/**
  * @brief	Set dma channel config,include control register and config register
  * @param  dma	Where dma is the base address of dma peripheral
  * @param  ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2
  * @param  Config	Where Config is the config of dma channel. \ref DMAC_ChannelConfigTypeDef      
  * @return None
	*/
void DMAC_SetChannelConfig(DMA_T * dma,uint32_t ChIdx,DMAC_ChannelConfigTypeDef *Config);

/**
  * @brief	Get dma channel config,include control register and config register
  * @param  dma	Where dma is the base address of dma peripheral
  * @param  ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2
  * @param  Config	Where Config is the config of dma channel. \ref DMAC_ChannelConfigTypeDef      
  * @return None
	*/
void DMAC_GetChannelConfig(DMA_T * dma,uint32_t ChIdx,DMAC_ChannelConfigTypeDef *Config);

/**
  * @brief	Start dma channel to transmit
  * @param  dma	Where dma is the base address of dma peripheral
  * @param  ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2
  * @param  Src	Where Src is the address of source data:    
  * @param  Dst	Where Dst is the address of destination data:      
  * @param  Len	Where Len is the block size number:      
  * @return None
	*/
void DMAC_StartChannel(DMA_T * dma,uint32_t ChIdx,void* Src,void* Dst,uint32_t Len);

/**
  * @brief		Stop dma channel
  * @param		dma	Where dma is the base address of dma peripheral
  * @param		ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2  
  * @return		None
	*/
void DMAC_StopChannel(DMA_T * dma,uint32_t ChIdx);

/**
  * @brief		Acquire dma free channel
  * @param		dma	Where dma is the base address of dma peripheral      
  * @return		None
	*/
uint32_t DMAC_AcquireChannel(DMA_T * dma);

/**
  * @brief		Release dma channel
  * @param		dma	Where dma is the base address of dma peripheral   
  * @param		ChIdx	Where ChIdx is the dma channel number.It could be 0,1,2  
  * @return 	None
	*/
void DMAC_ReleaseChannel(DMA_T * dma, uint32_t ChIdx);
bool DMAC_IsAllChannelReleased(DMA_T *dma);


/**@} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PAN_DMAC_H__ */

