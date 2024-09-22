/**************************************************************************//**
* @file     pan_hal_dmac.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series DMAC (Direct Memory Access Controller) HAL (Hardware Abstraction Layer) header file.
*
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __PAN_HAL_DMAC_H__
#define __PAN_HAL_DMAC_H__


#include "pan_hal_def.h"

/**
 * @defgroup pan_hal_dmac DMAC driver
 * @{
 * @ingroup pan_dmac
 * @brief   Direct Memory Access Controller (DMAC) peripheral driver.
 */
typedef enum
{
  DMAC_Peri_UART = 0x01,
  DMAC_Peri_I2C = 0x02, 
  DMAC_Peri_SPI = 0x03,
  DMAC_Peri_ADC = 0x04,
}DMAC_Peri_Mode_Opt;


// typedef void (*DMAC_CallbackFunc)(uint8_t);
/**
 * @brief  DMA handle Structure definition
 */
typedef struct
{
  DMA_T *dma;
  DMAC_ChannelConfigTypeDef ConfigTmp;

  DMAC_CallbackFunc Callback;
  // uint8_t Status;
  bool XferFlag;
  SPI_T *SPIx;

  UART_CallbackFunc CallbackUart;
  I2C_CallbackFunc CallbackI2c;
  SPI_CallbackFunc CallbackSpi;
  ADC_CallbackFunc CallbackAdc;
  DMAC_Peri_Mode_Opt PeriMode;

  bool StopFlag;
  uint32_t *pBuffPtr;
  uint16_t XferSize;
} DMAC_HandleTypeDef;

#define DMAC_CHANNEL_NUMS 3
extern DMAC_HandleTypeDef DMAC_Channel_Array[DMAC_CHANNEL_NUMS];
#define DMAC_Channel_1 DMAC_Channel_Array[0]
#define DMAC_Channel_2 DMAC_Channel_Array[1]
#define DMAC_Channel_3 DMAC_Channel_Array[2]
extern DMAC_ChannelConfigTypeDef dma_uart2mem_config;
extern DMAC_ChannelConfigTypeDef dma_mem2uart_config; 
extern DMAC_ChannelConfigTypeDef dma_spi2mem_config;
extern DMAC_ChannelConfigTypeDef dma_mem2spi_config;
extern DMAC_ChannelConfigTypeDef dma_mem2i2c_config;
extern DMAC_ChannelConfigTypeDef dma_i2c2mem_config; 
extern DMAC_ChannelConfigTypeDef dma_adc2mem_config;
extern DMAC_ChannelConfigTypeDef dma_mem2i2c_u16_config;

void DMA_IRQHandler(void);

#endif


