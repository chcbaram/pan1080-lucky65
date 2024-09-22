/********************************************************* * 
@file		: dmac.c 
@brief  	:dmac driver source file in IP library 
@author 	:zhongfeng
@version	:v1.0 
@date 		:19/08/15
Copyright (C) 2019 Panchip Technology Corp. All rights reserved. 
***********************************************************/
#include "PanSeries.h"
#include "pan_dmac.h"

static uint8_t g_DmacChannels = 0;

/*
  * @brief	   :enable dma
  * @param in  :where dma is the base address of dma peripheral
  * @retval    :none
*/
void DMAC_Init(DMA_T * dma)
{
    DMAC_DeInit(dma);
    /*dma enable*/
    dma->DMA_CFG_REG_L = 0x1;
}
/*
  * @brief	   :disable dma & mask all dma interrupt
  * @param in  :where dma is the base address of dma peripheral
  * @retval    :none
*/
void DMAC_DeInit(DMA_T * dma)
{
    /*disable dma*/
    dma->DMA_CFG_REG_L = 0x0;
    /*mask interrupt*/
    dma->MSK_TFR_L = 0xff00ul;
    dma->MSK_BLOCK_L = 0xff00ul;
    dma->MSK_DSTTRAN_L = 0xff00ul;
    dma->MSK_ERR_L = 0xff00ul;
    dma->MSK_SRCTRAN_L = 0xff00ul;
}
/*
  * @brief	   :clear interrupt mask to make sure intterupt enable
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2
  * @param in  :where FlgIdx is interrupt type,It could be:
  *                  DMAC_FLAG_INDEX_TFR    
  *                  DMAC_FLAG_INDEX_BLK    
  *                  DMAC_FLAG_INDEX_SRCTFR 
  *                  DMAC_FLAG_INDEX_DSTTFR 
  *                  DMAC_FLAG_INDEX_ERR    
  * @retval    :none
*/
void DMAC_ClrIntFlagMsk(DMA_T* dma, uint32_t ChIdx, uint32_t FlgIdx)
{
    /*choose the interrupt mask sequence*/
    uint32_t mask = DMAC_ChannelMask(ChIdx);
    /*write enable*/
    mask |= mask << 8;
    /*Clear specified interrupt mask*/
    ((__IO uint32_t*)(&(dma)->MSK_TFR_L))[FlgIdx] |= mask;
}
/*
  * @brief	   :mask interrupt,interrupt is useless
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2
  * @param in  :where FlgIdx is interrupt type,It could be:
  *                  DMAC_FLAG_INDEX_TFR    
  *                  DMAC_FLAG_INDEX_BLK    
  *                  DMAC_FLAG_INDEX_SRCTFR 
  *                  DMAC_FLAG_INDEX_DSTTFR 
  *                  DMAC_FLAG_INDEX_ERR    
  * @retval    :none
*/
void DMAC_SetIntFlagMsk(DMA_T* dma, uint32_t ChIdx, uint32_t FlgIdx)
{
    /*choose the interrupt mask sequence*/
    uint32_t msk = DMAC_ChannelMask(ChIdx);
    uint32_t reg = ((__IO uint32_t*)(&(dma)->MSK_TFR_L))[FlgIdx] & (~msk);
    /*set specified interrupt mask,Operable when the write enable operation*/
    ((__IO uint32_t*)(&(dma)->MSK_TFR_L))[FlgIdx] = reg | (msk << 8);
}
/*
  * @brief	   :set dma channel config,include control register and config register
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2
  * @param in  :where Config is the config of dma channel:    
  * @retval    :none
*/
void DMAC_SetChannelConfig(DMA_T * dma,uint32_t ChIdx,DMAC_ChannelConfigTypeDef *Config)
{
    __DMAC_ChannelConfigTypeDef* cfg = (__DMAC_ChannelConfigTypeDef*)Config;
    dma->CH[ChIdx].CTL_L = cfg->CTL_L;
    dma->CH[ChIdx].CTL_H = cfg->CTL_H;
    dma->CH[ChIdx].CFG_L = cfg->CFG_L;
    dma->CH[ChIdx].CFG_H = cfg->CFG_H;
}
/*
  * @brief	   :get dma channel config,include control register and config register
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2
  * @param in  :where Config is the config of dma channel:    
  * @retval    :none
*/
void DMAC_GetChannelConfig(DMA_T * dma,uint32_t ChIdx,DMAC_ChannelConfigTypeDef *Config)
{
    __DMAC_ChannelConfigTypeDef* cfg = (__DMAC_ChannelConfigTypeDef*)Config;
    cfg->CTL_L = dma->CH[ChIdx].CTL_L;
    cfg->CTL_H = dma->CH[ChIdx].CTL_H;
    cfg->CFG_L = dma->CH[ChIdx].CFG_L;
    cfg->CFG_H = dma->CH[ChIdx].CFG_H;    
}
/*
  * @brief	   :start dma channel to transmit
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2
  * @param in  :where Src is the address of source data:    
  * @param in  :where Dst is the address of destination data:      
  * @param in  :where Len is the block size number:      
  * @retval    :none
*/
void DMAC_StartChannel(DMA_T * dma,uint32_t ChIdx,void* Src,void* Dst,uint32_t Len)
{
    /*set the source address of dma transfer*/
    if(Src) dma->CH[ChIdx].SAR_L = (uint32_t)Src;
    /*set the destination address of dma transfer*/
    if(Dst) dma->CH[ChIdx].DAR_L = (uint32_t)Dst;
    /*set the number of block size, block_size = data_len / DataWidth*/
    if(Len) dma->CH[ChIdx].CTL_H = (dma->CH[ChIdx].CTL_H & (~0xffful)) | Len;
    /*DMA channel enable,start to transfer*/
    dma->CH_EN_REG_L |= (0x101ul << ChIdx);
}
/*
  * @brief	   :stop dma channel
  * @param in  :where dma is the base address of dma peripheral
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2  
  * @retval    :none
*/
void DMAC_StopChannel(DMA_T * dma,uint32_t ChIdx)
{
    dma->CH_EN_REG_L = (dma->CH_EN_REG_L & (~(0x1ul << ChIdx))) | (0x100ul << ChIdx);
}
/*
  * @brief	   :Acquire dma free channel
  * @param in  :where dma is the base address of dma peripheral      
  * @retval    :none
*/
uint32_t DMAC_AcquireChannel(DMA_T * dma)
{
    uint32_t idx;
    uint8_t msk;
    for(idx = 0, msk = 0x1; idx < DMAC_CHANNEL_NUMBER; idx++, msk <<= 1){
        if(!(g_DmacChannels & msk)){
            g_DmacChannels |= msk;
            return idx;
        }
    }
    return DMA_INVLID_CHANNEL;
}
/*
  * @brief	   :release dma channel
  * @param in  :where dma is the base address of dma peripheral   
  * @param in  :where ChIdx is the dma channel number.It could be 0,1,2  
  * @retval    :none
*/
void DMAC_ReleaseChannel(DMA_T * dma, uint32_t ChIdx)
{
    g_DmacChannels &= ~(1ul << ChIdx);
}

bool DMAC_IsAllChannelReleased(DMA_T *dma)
{
    return (g_DmacChannels == 0);
}

/*-------------------------------------------------------*/

