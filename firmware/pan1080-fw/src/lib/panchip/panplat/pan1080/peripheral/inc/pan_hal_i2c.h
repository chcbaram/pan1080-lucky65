/**************************************************************************//**
* @file     pan_hal_i2c.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series I2C (Inter-Integrated Circuit) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PAN_HAL_I2C_H
#define __PAN_HAL_I2C_H
#include "pan_hal_def.h"

/**
 * @defgroup pan_hal_i2c I2C driver
 * @{
 * @ingroup pan_i2c
 * @brief   Inter-Integrated Circuit (I2C) peripheral driver.
 */
/**
 * @brief Enumeration of I2C Address Modes.
 *
 * This enumeration defines the possible address modes for I2C communication, such as 7-bit or 10-bit addressing.
 */
typedef enum  {
  I2C_ADDR_7BIT,  /*!< 7-bit I2C address mode */
  I2C_ADDR_10BIT  /*!< 10-bit I2C address mode */
} I2C_AddressMode_Opt;

/**
 * @brief Enumeration of I2C Roles.
 *
 * This enumeration defines the possible roles for an I2C device, either a master or a slave.
 */
typedef enum
{
    I2C_ROLE_MASTER = ((uint16_t)0x0021), /*!< I2C master role */
    I2C_ROLE_SLAVE  = ((uint16_t)0x0000)  /*!< I2C slave role */
} I2C_Role_Opt;

/**
 * @brief Enumeration of I2C Duty Cycles.
 *
 * This enumeration defines the possible duty cycles for I2C fast mode, such as 16/9 or 2.
 */
typedef enum
{
    I2C_DUTYCYCLE_16_9 = I2C_DutyCycle_16_9, /*!< I2C fast mode duty cycle 16/9 */
    I2C_DUTYCYCLE_2    = I2C_DutyCycle_2     /*!< I2C fast mode duty cycle 2 */
} I2C_DutyCycle_Opt;

/**
 * @brief Enumeration of I2C Directions.
 *
 * This enumeration defines the possible data transfer directions for I2C, either transmitter or receiver.
 */
typedef enum
{
    I2C_DIR_TX = ((uint16_t)0x0000), /*!< I2C transmit direction */
    I2C_DIR_RX = ((uint16_t)0x0100)  /*!< I2C receive direction */
} I2C_Direction_Opt;

/**
 * @brief Enumeration of I2C Acknowledged Address Modes.
 *
 * This enumeration defines the possible address acknowledgment modes for I2C, such as 7-bit or 10-bit.
 */
typedef enum
{
    I2C_ACK_ADDR_7BIT  = I2C_AcknowledgedAddress_7bit,  /*!< I2C acknowledged address mode 7-bit */
    I2C_ACK_ADDR_10BIT = I2C_AcknowledgedAddress_10bit  /*!< I2C acknowledged address mode 10-bit */
} I2C_AckAddr_Opt;

/**
 * @brief Enumeration of I2C Own Address Modes.
 *
 * This enumeration defines the possible own address modes for I2C, both for slave and master devices.
 */
typedef enum {
  I2C_SLAVE_OWN_ADDR_7BIT  = I2C_SLAVE_OWN_ADDRESS7,     /*!< 7-bit own address for I2C slave */
  I2C_SLAVE_OWN_ADDR_10BIT = I2C_SLAVE_OWN_ADDRESS10,    /*!< 10-bit own address for I2C slave */
  I2C_MASTER_OWN_ADDR_7BIT = I2C_MASTER_TAR_ADDRESS7    /*!< 7-bit own address for I2C master */
} I2C_OwnAddress_Opt;

/**
 * @brief Enumeration of I2C Clock Speeds.
 *
 * This enumeration defines the possible clock speeds for I2C communication in terms of bits per second (bps).
 */
typedef enum
{
  I2C_SPEED_100K = (100000), /*!< 100 Kbps I2C clock speed */
  I2C_SPEED_200K = (200000), /*!< 200 Kbps I2C clock speed */
  I2C_SPEED_400K = (400000), /*!< 400 Kbps I2C clock speed */
  I2C_SPEED_500K = (500000), /*!< 500 Kbps I2C clock speed */
  I2C_SPEED_1M   = (1000000),/*!< 1 Mbps I2C clock speed */
  I2C_SPEED_2M   = (2000000) /*!< 2 Mbps I2C clock speed */
} I2C_ClockSpeed_Opt;

// typedef void (*I2C_CallbackFunc)(uint8_t);
// typedef void (*I2C_CallbackFunc)(void);
/**
  * @brief  I2C Init structure definition
  */

typedef struct
{
  I2C_ClockSpeed_Opt ClockSpeed;          /*!< Specifies the clock frequency.
                                         This parameter must be set to a value lower than 400kHz */

  I2C_Role_Opt Role;                /*!< Specifies the I2C mode.
                                         This parameter can be a value of @ref I2C_mode */

  I2C_DutyCycle_Opt DutyCycle;           /*!< Specifies the I2C fast mode duty cycle.
                                         This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

  I2C_OwnAddress_Opt OwnAddress;         /*!< Specifies the first device own address.
                                         This parameter can be a 7-bit or 10-bit address. */

  I2C_AckAddr_Opt AckAddress; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */

  I2C_AddressMode_Opt AddressMode;
}I2C_Init_Opt;

typedef enum{
  I2C_Tx_Dir,
  I2C_Rx_Dir
}I2C_Xfer_Dir_Opt;

typedef struct
{
	uint8_t pass;
}I2C_Interrupt_Opt;

/**
 * @brief I2C Handle Structure Definition
 *
 * This structure defines the configuration and management parameters for I2C communication.
 */
typedef struct
{
    I2C_T* I2Cx; /*!< Base address of the I2C peripheral */

    I2C_Init_Opt I2C_InitObj; /*!< I2C communication parameters */
    I2C_Interrupt_Opt InterruptObj; /*!< I2C interrupt configuration */

    uint8_t *pTxBuffPtr; /*!< Pointer to the transmit buffer */
    uint16_t TxXferSize; /*!< Transmit data size */
    __IO uint16_t TxXferCount; /*!< Transmit data counter */

    uint8_t *pRxBuffPtr; /*!< Pointer to the receive buffer */
    uint16_t RxXferSize; /*!< Receive data size */
    __IO uint16_t RxXferCount; /*!< Receive data counter */

    IRQn_Type IRQn; /*!< I2C interrupt request number */

    I2C_CallbackFunc RxIntCallback; /*!< Callback function for receive interrupts */
    I2C_CallbackFunc TxIntCallback; /*!< Callback function for transmit interrupts */

    uint32_t DMA_SRC; /*!< DMA source configuration for I2C */
    uint32_t DMA_DST; /*!< DMA destination configuration for I2C */

    uint8_t StopFlag; /*!< Flag indicating whether to generate a stop condition */
    I2C_Xfer_Dir_Opt Xfer_Dir; /*!< Direction of the I2C data transfer (transmit or receive) */

    __IO uint32_t ErrorCode; /*!< I2C error code */

} I2C_HandleTypeDef;

/**
 * @brief External array of I2C handle structures.
 *
 * This external array holds I2C handle structures, which can be used to configure and manage I2C communication.
 */
extern I2C_HandleTypeDef I2C_Handle_Array[1];

/**
 * @brief Macro to access the first I2C handle object in the array.
 *
 * You can use this macro, I2C0_OBJ, to conveniently access the first I2C handle in the array for configuration and management.
 */
#define I2C0_OBJ I2C_Handle_Array[0]


/**
 * @brief Initialize the I2C peripheral.
 *
 * This function initializes the I2C peripheral based on the configuration provided in the I2C_HandleTypeDef structure.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @return true if initialization is successful, false otherwise.
 */
bool HAL_I2C_Init(I2C_HandleTypeDef *i2c);

void HAL_I2C_DeInit(I2C_HandleTypeDef *i2c);
void HAL_I2C_Init_INT(I2C_HandleTypeDef *i2c);
void HAL_I2C_DeInit_INT(I2C_HandleTypeDef *i2c);


/**
 * @brief Transmit data as a master via I2C.
 *
 * This function sends data to a device with the specified address as a master using I2C communication.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param DevAddress  device address.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be transmitted.
 * @param Timeout Timeout duration for the operation.
 */
void HAL_I2C_Master_SendData(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,uint32_t Timeout);

/**
 * @brief Receive data as a master via I2C.
 *
 * This function receives data from a device with the specified address as a master using I2C communication.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param DevAddress device address.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Timeout Timeout duration for the operation.
 */
void HAL_I2C_Master_ReceiveData(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,uint32_t Timeout);

/**
 * @brief Transmit data as a master via I2C using interrupts.
 *
 * This function sends data to a device with the specified address as a master using I2C communication with interrupts.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param DevAddress device address.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be transmitted.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Master_SendData_INT(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);


/**
 * @brief Receive data as a master via I2C using interrupts.
 *
 * This function receives data from a device with the specified address as a master using I2C communication with interrupts.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param DevAddress device address.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Master_ReceiveData_INT(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
  * @brief  Sends data over I2C using DMA for the master mode.
  * @param  i2c: Pointer to the I2C handle structure.
  * @param  DevAddress: device address to communicate with.
  * @param  Buf: Pointer to the data buffer containing the data to be sent.
  * @param  Size: Number of bytes to send.
  * @param  Callback: Callback function to be executed when the DMA transfer is complete.
  * @retval None
  */
void HAL_I2C_Master_SendData_DMA(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
 * @brief Receive data as a master via I2C using DMA.
 *
 * This function receives data from a device with the specified address as a master using I2C communication with DMA.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param DevAddress device address.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Master_ReceiveData_DMA(I2C_HandleTypeDef *i2c,uint16_t DevAddress,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
 * @brief Send data as a slave via I2C.
 *
 * This function sends data as a slave in response to read requests from a master device.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be sent.
 * @param Timeout Timeout duration for the operation.
 */
void HAL_I2C_Slave_SendData(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,uint32_t Timeout);

/**
 * @brief Receive data as a slave via I2C.
 *
 * This function receives data as a slave in response to read commands from a master device.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Timeout Timeout duration for the operation.
 */
void HAL_I2C_Slave_ReceiveData(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,uint32_t Timeout);

/**
 * @brief Send data as a slave via I2C using interrupts.
 *
 * This function sends data as a slave in response to read requests from a master device using interrupts.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be sent.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Slave_SendData_INT(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
 * @brief Receive data as a slave via I2C using interrupts.
 *
 * This function receives data as a slave in response to read commands from a master device using interrupts.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Slave_ReceiveData_INT(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);
// uint32_t HAL_I2C_Slave_SendData_DMA(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
 * @brief Send data as a slave via I2C using DMA.
 *
 * This function sends data as a slave in response to read requests from a master device using DMA.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be sent.
 * @param Callback Callback function to be called upon completion.
 * @return The DMA channel number used for the operation.
 */
void HAL_I2C_Slave_SendData_DMA(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);

/**
 * @brief Receive data as a slave via I2C using DMA.
 *
 * This function receives data as a slave in response to read commands from a master device using DMA.
 *
 * @param i2c Pointer to the I2C_HandleTypeDef structure.
 * @param Buf Pointer to the data buffer.
 * @param Size Size of the data to be received.
 * @param Callback Callback function to be called upon completion.
 */
void HAL_I2C_Slave_ReceiveData_DMA(I2C_HandleTypeDef *i2c,uint8_t *Buf,uint32_t Size,I2C_CallbackFunc Callback);


// void HAL_I2C_Master_SendData_DMA_U16(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint16_t *Buf, uint32_t Size, I2C_CallbackFunc Callback);


/** @} */ // end of group
#endif /* __PAN_HAL_I2C_H */
