/**************************************************************************//**
* @file     pan_hal_spi.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series SPI HAL (Hardware Abstraction Layer) header file.
*
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef _PAN_HAL_SPI_H_
#define _PAN_HAL_SPI_H_

#include "pan_hal_def.h"

/**
 * @brief SPI HAL Interface
 * @defgroup spi_hal_interface Spi HAL Interface
 * @{
 */

// Config SPI software buffer size
#define SPI_RX_BUF_SIZE         256
#define SPI_TX_BUF_SIZE         256

/**
 * @brief  Structure to configure SPI interrupt mode.
 */
typedef struct
{
    SPI_IrqDef Mode; /*!< SPI interrupt mode. */
} SPI_Interrupt_Opt;

/**
 * @brief  Enumeration defining SPI baud rates.
 */
typedef enum {
    BAUDRATE_100K   = 100000,   /*!< Baud rate: 100,000 bps. */
    BAUDRATE_500K   = 500000,   /*!< Baud rate: 500,000 bps. */
    BAUDRATE_1M     = 1000000,  /*!< Baud rate: 1,000,000 bps. */
    BAUDRATE_2M     = 2000000,  /*!< Baud rate: 2,000,000 bps. */
    BAUDRATE_5M     = 5000000,  /*!< Baud rate: 5,000,000 bps. */
    BAUDRATE_10M    = 10000000  /*!< Baud rate: 10,000,000 bps. */
} SPI_BaudRate;

/**
 * @brief  SPI Init Structure definition
 */
typedef struct
{
  SPI_RoleDef Role;              /*!< Specifies the SPI operating mode */
  SPI_DataFrameSizeDef DataSize; /*!< Specifies the SPI data size */
  SPI_ClockPolDef ClkPolarity;   /*!< Specifies the serial clock steady state */
  SPI_ClockPhaseDef ClkPhase;    /*!< Specifies the clock active edge for the bit capture */
  // SPI_BaudRate Baudrate;      /*!< Specifies the Baud Rate Prescaler used to configure the transmit and receive SCK clock. */
  SPI_BaudRateDivDef BaudrateDiv; /*!< Specifies the Baud Rate Prescaler used to configure the transmit and receive SCK clock. */
  SPI_FrameFormatDef Format;      /*!< Specifies the SPI format */
} SPI_Init_Opt;


typedef enum{
    SPI_STAT_NULL,
    SPI_STAT_RX= 0x10,
    SPI_STAT_TX = 0x01,
    SPI_STAT_RXTX = 0x11,
}SPI_Xfer_Stat_Opt;

/**
 * @brief  SPI handle Structure definition
 */
typedef struct
{
  SPI_T *SPIx; /*!< SPI registers base address        */

  SPI_Init_Opt InitObj;           /*!< SPI communication parameters      */
  SPI_Xfer_Stat_Opt Xfer_Stat;    /*!< SPI transfer status               */
  SPI_Interrupt_Opt InterruptObj; /*!< SPI interrupt options           */

  uint16_t *pTxBuffPtr; /*!< Pointer to SPI Tx transfer buffer */
  uint16_t TxXferSize; /*!< SPI Tx transfer size              */
  __IO uint16_t TxXferCount; /*!< SPI Tx transfer counter           */

  uint16_t *pRxBuffPtr; /*!< Pointer to SPI Rx transfer buffer */
  uint16_t RxXferSize; /*!< SPI Rx transfer size              */
  __IO uint16_t RxXferCount; /*!< SPI Rx transfer counter           */

  IRQn_Type IRQn; /*!< SPI IRQ number                    */
  SPI_CallbackFunc RxIntCallback; /*!< Callback function for Rx interrupt */
  SPI_CallbackFunc TxIntCallback; /*!< Callback function for Tx interrupt */

  uint32_t DmaSrc;         /*!< DMA source address for SPI data transfer */
  uint32_t DmaDst;         /*!< DMA destination address for SPI data transfer */

  __IO uint32_t ErrorCode; /*!< SPI Error code                    */

} SPI_HandleTypeDef;

/**
 * @brief Array of SPI handle structures for managing multiple SPI interfaces.
 *
 * You can access specific SPI handles using SPI0_OBJ and SPI1_OBJ aliases.
 */
extern SPI_HandleTypeDef SPI_Handle_Array[2];

/**
 * @brief SPI handle for the first SPI interface (SPI0).
 *
 * You can use this handle to configure and manage SPI0.
 */
#define SPI0_OBJ SPI_Handle_Array[0]

/**
 * @brief SPI handle for the second SPI interface (SPI1).
 *
 * You can use this handle to configure and manage SPI1.
 */
#define SPI1_OBJ SPI_Handle_Array[1]

/**
 * @brief Initialize the SPI module.
 * @param spi Pointer to the SPI handle structure.
 */
void HAL_SPI_Init(SPI_HandleTypeDef *spi);

/**
 * @brief DeInitialize the SPI module.
 * @param spi Pointer to the SPI handle structure.
 */
void HAL_SPI_DeInit(SPI_HandleTypeDef *spi);

/**
 * @brief Initialize SPI interrupts.
 * @param spi Pointer to the SPI handle structure.
 */
void HAL_SPI_Init_INT(SPI_HandleTypeDef *spi);

/**
 * @brief Initialize SPI interrupts.
 * @param spi Pointer to the SPI handle structure.
 */
void HAL_SPI_DeInit_INT(SPI_HandleTypeDef *spi);

/**
 * @brief Send data over SPI.
 * @param spi Pointer to the SPI handle structure.
 * @param Buf Pointer to the data buffer to send.
 * @param Size Size of the data to send.
 * @param Timeout Timeout for the send operation.
 */
void HAL_SPI_SendData(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t size,uint32_t Timeout); 

/**
 * @brief Receive data from SPI.
 * @param spi Pointer to the SPI handle structure.
 * @param Buf Pointer to the buffer to store received data.
 * @param Size Size of the data to receive.
 * @param Timeout Timeout for the receive operation.
 */
void HAL_SPI_ReceiveData(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t size,uint32_t Timeout); 

// void SPI_MultiDataSendReceiveAsync(SPI_T* SPIx, uint16_t* data_to_send, size_t expect_send_size, uint16_t* buf_to_recv, size_t expect_recv_size);
/**
 * @brief Send and receive data over SPI.
 * @param spi Pointer to the SPI handle structure.
 * @param SendBuf Pointer to the data buffer to send.
 * @param SendBufSize Size of the data to send.
 * @param RecvBuf Pointer to the buffer to store received data.
 * @param RecvBufSize Size of the data to receive.
 */
void HAL_SPI_SendReceiveData (SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize, uint16_t* RecvBuf, size_t RecvBufSize);

/**
 * @brief Send and receive data over SPI.
 * @param spi Pointer to the SPI handle structure.
 * @param SendBuf Pointer to the data buffer to send.
 * @param SendBufSize Size of the data to send.
 * @param RecvBuf Pointer to the buffer to store received data.
 * @param RecvBufSize Size of the data to receive.
 */
void HAL_SPI_SendData_INT(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, SPI_CallbackFunc Callback);
// void HAL_SPI_ReceiveData_INT(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, SPI_CallbackFunc Callback);

/**
 * @brief Send and receive data over SPI with interrupt support.
 * @param spi Pointer to the SPI handle structure.
 * @param SendBuf Pointer to the data buffer to send.
 * @param SendBufSize Size of the data to send.
 * @param SendCallback Callback function to handle the transmit interrupt.
 * @param RecvBuf Pointer to the buffer to store received data.
 * @param RecvBufSize Size of the data to receive.
 * @param RecvCallback Callback function to handle the receive interrupt.
 */
void HAL_SPI_SendReceiveData_INT(SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize,SPI_CallbackFunc SendCallback, uint16_t* RecvBuf, size_t RecvBufSize,SPI_CallbackFunc RecvCallback);

/**
 * @brief Send data over SPI using DMA for improved performance.
 * @param spi Pointer to the SPI handle structure.
 * @param Buf Pointer to the data buffer to send.
 * @param Size Size of the data to send.
 * @param Callback Callback function to handle the DMA transfer completion.
 * @return DMA channel number used for the transfer.
 */
uint32_t HAL_SPI_SendData_DMA(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size,SPI_CallbackFunc Callback);
// uint32_t HAL_SPI_ReceiveData_DMA(SPI_HandleTypeDef *spi, uint16_t *Buf, size_t Size, SPI_CallbackFunc Callback);

/**
 * @brief Send and receive data over SPI using DMA for improved performance.
 * @param spi Pointer to the SPI handle structure.
 * @param SendBuf Pointer to the data buffer to send.
 * @param SendBufSize Size of the data to send.
 * @param SendCallback Callback function to handle the DMA transmit completion.
 * @param RecvBuf Pointer to the buffer to store received data.
 * @param RecvBufSize Size of the data to receive.
 * @param RecvCallback Callback function to handle the DMA receive completion.
 */
uint32_t HAL_SPI_SendReceiveData_DMA(SPI_HandleTypeDef *spi, uint16_t* SendBuf, size_t SendBufSize,SPI_CallbackFunc SendCallback, uint16_t* RecvBuf, size_t RecvBufSize,SPI_CallbackFunc RecvCallback);
/** @} */ // end of group
#endif 
