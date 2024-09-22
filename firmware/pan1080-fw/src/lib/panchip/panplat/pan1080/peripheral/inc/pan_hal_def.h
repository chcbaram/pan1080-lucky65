/**************************************************************************
* @file     pan_hal_def.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series HAL (Hardware Abstraction Layer) common definitions header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __PAN_HAL_DEF_H__
#define __PAN_HAL_DEF_H__

#include "PanSeries.h"
typedef void (*DMAC_CallbackFunc)(uint8_t);


typedef enum{
  UART_CB_FLAG_TX_FINISH,
  UART_CB_FLAG_RX_FINISH = 0x10,
  UART_CB_FLAG_RX_TIMEOUT = 0x11,
  UART_CB_FLAG_RX_BUFFFULL = 0x12,
  UART_CB_FLAG_DMA = 0x13,
}UART_Cb_Flag_Opt;

typedef enum{
  I2C_CB_FLAG_STOP = 0X10,
  I2C_CB_FLAG_DMA = 0x23,
}I2C_Cb_Flag_Opt;

typedef enum{
  ADC_CB_FLAG_DMA = 0x42,
}ADC_Cb_Flag_Opt;

typedef enum{
  SPI_CB_FLAG_INT = 0X10,
  SPI_CB_FLAG_DMA = 0x23,
}SPI_Cb_Flag_Opt;

typedef void (*UART_CallbackFunc)(UART_Cb_Flag_Opt,uint8_t*,uint16_t);
typedef void (*I2C_CallbackFunc)(I2C_Cb_Flag_Opt,uint8_t*,uint16_t);
typedef void (*SPI_CallbackFunc)(SPI_Cb_Flag_Opt,uint16_t*,uint16_t);
typedef void (*ADC_CallbackFunc)(ADC_Cb_Flag_Opt,uint32_t*,uint16_t);


#define I2C_SLAVE_OWN_ADDRESS7          (0x50)
#define I2C_SLAVE_OWN_ADDRESS10         (0x350)
#define I2C_MASTER_TAR_ADDRESS7         (0xA0)//0xA8

#endif
