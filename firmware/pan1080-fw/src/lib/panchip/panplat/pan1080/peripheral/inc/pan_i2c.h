/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series i2c driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */

#ifndef __PAN_I2C_H__
#define __PAN_I2C_H__

/**
 * @brief I2c Interface
 * @defgroup i2c_interface I2c Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif


/** 
 * @brief Structure with i2c init feature.
 *
 * @param I2C_ClockSpeed 	Specifies the clock frequency.
 * @param I2C_Mode				Specifies the I2C mode.This parameter can be a value of
 *													  \ref PAN_I2C_MODE_MASTER,\ref PAN_I2C_MODE_SLAVE
 * @param I2C_DutyCycle		Specifies the I2C fast mode duty cycle.This parameter
 *													 can be a value of \ref I2C_DutyCycle_16_9, \ref I2C_DutyCycle_2
 * @param I2C_OwnAddress1	Specifies the first device own address,
 *													 This parameter can be a 7-bit or 10-bit address.
 * @param I2C_AcknowledgedAddress		Specifies if 7-bit or 10-bit address is acknowledged.
 *													 This parameter can be a value of 
 *														\ref I2C_AcknowledgedAddress_7bit, \ref I2C_AcknowledgedAddress_10bit
 */
typedef struct
{
  uint32_t I2C_ClockSpeed;
  uint16_t I2C_Mode;                
  uint16_t I2C_DutyCycle;           
  uint16_t I2C_OwnAddress1;         
  uint16_t I2C_AcknowledgedAddress;
}I2C_InitTypeDef;




#define PAN_I2C_MODE_MASTER                         ((uint16_t)0x0021)	/*!< I2C master mode */
#define PAN_I2C_MODE_SLAVE                          ((uint16_t)0x0000)	/*!< I2C slave mode */
#define IS_I2C_MODE(MODE)                       (((MODE) == PAN_I2C_MODE_MASTER) || \
                                                 ((MODE) == PAN_I2C_MODE_SLAVE))	/*!< I2C mode check */

                                                 
#define I2C_SPEED_STANDARD_MODE                 (uint16_t)(0x0002)	/*!< I2C standard speed mode, speed less than 100KHz */
#define I2C_SPEED_FAST_MODE                     (uint16_t)(0x0004)	/*!< I2C fast speed mode,speed less than 1MHz */
#define I2C_SPEED_HIGH_MODE                     (uint16_t)(0x0006)	/*!< I2C high speed mode,speed larger than 1MHz */


#define I2C_DutyCycle_16_9                      ((uint16_t)0x4000) /*!< I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2                         ((uint16_t)0xBFFF) /*!< I2C fast mode Tlow/Thigh = 2 */
#define IS_I2C_DUTY_CYCLE(CYCLE)                (((CYCLE) == I2C_DutyCycle_16_9) || \
                                                 ((CYCLE) == I2C_DutyCycle_2))	/*!< I2C fast mode duty cycle check */


#define I2C_AcknowledgedAddress_7bit            ((uint16_t)0x4000)	/*!< I2C 7-bit acknowledged address enable */
#define I2C_AcknowledgedAddress_10bit           ((uint16_t)0xC000)	/*!< I2C 10-bit acknowledged address enable */

#define I2C_CMD_WR                              ((uint8_t)0x00)	/*!< I2C write cmd */
#define I2C_CMD_RD                              ((uint8_t)0x01)	/*!< I2C read cmd */
#define I2C_CMD_STOP                            ((uint8_t)0x02)	/*!< I2C stop cmd */
#define I2C_CMD_RESTART                         ((uint8_t)0x04)	/*!< I2C restart cmd */

/**@defgroup I2C_REC_TRIG_LEV I2c rx threshold level
 * @brief       I2c receive fifo threshold level
 * @{ */
#define I2C_RX_TL_0                             ((uint8_t)0x00)	/*!< I2C receive fifo threshold value 0,same as 1 */
#define I2C_RX_TL_1                             ((uint8_t)0x01)	/*!< I2C receive fifo threshold value 1 */
#define I2C_RX_TL_2                             ((uint8_t)0x02)	/*!< I2C receive fifo threshold value 2 */
#define I2C_RX_TL_3                             ((uint8_t)0x03)	/*!< I2C receive fifo threshold value 3 */
#define I2C_RX_TL_4                             ((uint8_t)0x04)	/*!< I2C receive fifo threshold value 4 */
#define I2C_RX_TL_5                             ((uint8_t)0x05)	/*!< I2C receive fifo threshold value 5 */
#define I2C_RX_TL_6                             ((uint8_t)0x06)	/*!< I2C receive fifo threshold value 6 */
#define I2C_RX_TL_7                             ((uint8_t)0x07)	/*!< I2C receive fifo threshold value 7 */
#define I2C_RX_TL_8                             ((uint8_t)0x08)	/*!< I2C receive fifo threshold value 8 */
/**@} */

/**@defgroup I2C_TRANS_TRIG_LEV I2c tx transmit level
 * @brief       I2c transmit fifo threshold level
 * @{ */
#define I2C_TX_TL_0                             ((uint8_t)0x00)	/*!< I2C transmit fifo threshold value 0 */
#define I2C_TX_TL_1                             ((uint8_t)0x01)	/*!< I2C transmit fifo threshold value 1 */
#define I2C_TX_TL_2                             ((uint8_t)0x02)	/*!< I2C transmit fifo threshold value 2 */
#define I2C_TX_TL_3                             ((uint8_t)0x03)	/*!< I2C transmit fifo threshold value 3 */
#define I2C_TX_TL_4                             ((uint8_t)0x04)	/*!< I2C transmit fifo threshold value 4 */
#define I2C_TX_TL_5                             ((uint8_t)0x05)	/*!< I2C transmit fifo threshold value 5 */
#define I2C_TX_TL_6                             ((uint8_t)0x06)	/*!< I2C transmit fifo threshold value 6 */
#define I2C_TX_TL_7                             ((uint8_t)0x07)	/*!< I2C transmit fifo threshold value 7 */
#define I2C_TX_TL_8                             ((uint8_t)0x08)	/*!< I2C transmit fifo threshold value 8 */
/**@} */

/**@defgroup I2C_INT_SRC_FLAG I2c interrupts
 * @brief       I2c raw interrupts flag 
 * @{ */
#define I2C_IT_RX_UNDER                         ((uint16_t)0x0001)	/*!< I2C receive underflow interrupts flag */
#define I2C_IT_RX_OVER                          ((uint16_t)0x0002)	/*!< I2C receive overflow interrupts flag */
#define I2C_IT_RX_FULL                          ((uint16_t)0x0004)	/*!< I2C receive fifo full interrupts flag */
#define I2C_IT_TX_OVER                          ((uint16_t)0x0008)	/*!< I2C transmit overflow interrupts flag */
#define I2C_IT_TX_EMPTY                         ((uint16_t)0x0010)	/*!< I2C transmit fifo empty interrupts flag */
#define I2C_IT_RD_REQ                           ((uint16_t)0x0020)	/*!< I2C read request interrupts flag */
#define I2C_IT_TX_ABORT                         ((uint16_t)0x0040)	/*!< I2C transmit abort interrupts flag */
#define I2C_IT_RX_DONE                          ((uint16_t)0x0080)	/*!< I2C read finish interrupts flag */
#define I2C_IT_ACTIVITY                         ((uint16_t)0x0100)	/*!< I2C activity flag */
#define I2C_IT_STOP_DET                         ((uint16_t)0x0200)	/*!< I2C stop interrupts flag */
#define I2C_IT_START_DET                        ((uint16_t)0x0400)	/*!< I2C start interrupts flag */
#define I2C_IT_GEN_CALL                         ((uint16_t)0x0800)	/*!< I2C general call interrupts flag */
#define I2C_IT_MST_ON_HOLD                      ((uint16_t)0x2000)	/*!< I2C master holding interrupts flag */
#define I2C_IT_ALL                              ((uint16_t)0x2FFF)	/*!< I2C all interrupts flag */
/**@} */

/**@defgroup I2C_STATE_FLAG I2c state
 * @brief       I2c device and fifo state flag 
 * @{ */
#define I2C_FLAG_SLV_ACTIVITY                   ((uint32_t)0x00000040)	/*!< I2C salve activity flag */
#define I2C_FLAG_MST_ACTIVITY                   ((uint32_t)0x00000020)	/*!< I2C master activity flag */
#define I2C_FLAG_RFF                  					((uint32_t)0x00000010)	/*!< I2C receive fifo full flag */
#define I2C_FLAG_RFNE                  					((uint32_t)0x00000008)	/*!< I2C receive fifo not empty flag */
#define I2C_FLAG_TFE                    				((uint32_t)0x00000004)	/*!< I2C transmit fifo empty flag */
#define I2C_FLAG_TFNF                   				((uint32_t)0x00000002)	/*!< I2C transmit fifo not full flag */
#define I2C_FLAG_ACTIVITY                     	((uint32_t)0x00000001)	/*!< I2C activity flag */
/**@} */

#define FLAG_MASK                               ((uint32_t)0x007F0FFF)  /*!< I2C flag mask */
#define ITEN_MASK                               ((uint16_t)0x2FFF)  		/*!< I2C interrupt enable mask */
#define IT_FLAG_MASK                            ((uint16_t)0x2FFF)  		/*!< I2C interrupt flag mask */


/**
  * @brief  Set I2C mode .
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Mode: role of the I2Cx peripheral.
  *          This parameter can be: \ref PAN_I2C_MODE_MASTER or \ref PAN_I2C_MODE_SLAVE.
  * @note   This function should be called before enabling
            the I2C Peripheral.
  * @return None
  */
__STATIC_INLINE void I2C_SetMode(I2C_T* I2Cx, uint16_t Mode)
{
    (Mode == PAN_I2C_MODE_MASTER)?(I2Cx->CON |= (I2C_CON_IC_SLAVE_DISABLE | I2C_CON_MASTER_MODE)):(I2Cx->CON &= ~(I2C_CON_IC_SLAVE_DISABLE | I2C_CON_MASTER_MODE));
}
/**
  * @brief  Set I2C Tx fifo threshold value .
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  ThresholdValue:	Transmit FIFO Threshold Level
  *          This parameter can be \ref I2C_TRANS_TRIG_LEV.
  * @note   Controls the level of entries (or below) that trigger the TX_EMPTY interrupt (bit 4 in
  *         IC_RAW_INTR_STAT register). The valid range is 0-8, with the additional
  *         restriction that it may not be set to value larger than the depth of the buffer. If an
  *         attempt is made to do that, the actual value set will be the maximum depth of the
  *         buffer.A value of 0 sets the threshold for 0 entries, and a value of 8 sets the threshold for
  *         8 entries.
  * @return None
  */
__STATIC_INLINE void I2C_SetTxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue)
{
    I2Cx->TX_TL = (I2Cx->TX_TL & ~I2C_TX_TL) | (ThresholdValue & 0xF);
}

/**
  * @brief  Set I2C Rx fifo threshold value .
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  ThresholdValue:	Receive FIFO Threshold Level
  *          This parameter can be \ref I2C_REC_TRIG_LEV.
  * @note   Controls the level of entries (or below) that trigger the RX_FULL interrupt (bit 2 in
  *         IC_RAW_INTR_STAT register). The valid range is 0-8, with the additional
  *         restriction that it may not be set to value larger than the depth of the buffer. If an
  *         attempt is made to do that, the actual value set will be the maximum depth of the
  *         buffer.A value of 0 sets the threshold for 0 entries, and a value of 8 sets the threshold for
  *         8 entries.
  * @return None
  */
__STATIC_INLINE void I2C_SetRxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue)
{
    I2Cx->RX_TL = (I2Cx->RX_TL & ~I2C_RX_TL) | (ThresholdValue & 0xF);
}
/**
  * @brief  Enables or disables the specified I2C peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  NewState: new state of the I2Cx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @return None
  */
__STATIC_INLINE void I2C_Cmd(I2C_T* I2Cx, FunctionalState NewState)
{
    (NewState != DISABLE)?(I2Cx->IC_ENABLE |= I2C_ENABLE_ENABLE):(I2Cx->IC_ENABLE &= (uint32_t)~((uint32_t)I2C_ENABLE_ENABLE));
}
/**
  * @brief  Enables or disables the specified I2C general call feature.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  NewState: new state of the I2C General call.
  *          This parameter can be: ENABLE or DISABLE.
  * @return None
  */
__STATIC_INLINE void I2C_GeneralCallCmd(I2C_T* I2Cx, FunctionalState NewState)
{
    (NewState != DISABLE)?(I2Cx->TAR = (I2Cx->TAR | I2C_TAR_SPECIAL) & (~I2C_TAR_GC_OR_START)):(I2Cx->TAR = I2Cx->TAR);
}

/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Data: Byte to be transmitted..
  * @param	Cmd : I2c operate command \ref I2C_CMD_RD,\ref I2C_CMD_WR,\ref I2C_CMD_RESTART,\ref I2C_CMD_STOP
  * @return None
  */
__STATIC_INLINE void I2C_SendDataCmd(I2C_T* I2Cx, uint8_t Data, uint8_t Cmd)
{
    I2Cx->DATACMD = ((Cmd << 8 ) | (Data & 0xFF));
}
/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param	Cmd : I2c operate command \ref I2C_CMD_RD,\ref I2C_CMD_WR,\ref I2C_CMD_RESTART,\ref I2C_CMD_STOP
  * @return None
  */
__STATIC_INLINE void I2C_SendCmd(I2C_T* I2Cx, uint8_t Cmd)
{
    I2Cx->DATACMD |= (Cmd << 8);
}
/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @return The value of the received data.
  */
__STATIC_INLINE uint8_t I2C_ReceiveData(I2C_T* I2Cx)
{
    return (uint8_t)I2Cx->DATACMD;
}
/**
  * @brief  Aborting I2C Transfers.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @return None
  */
__STATIC_INLINE void I2C_AbortTransfer(I2C_T* I2Cx)
{
    I2Cx->IC_ENABLE |= I2C_ENABLE_ABORT;
}
/**
  * @brief  Enables or disables the specified I2C DMA requests.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  NewState: new state of the I2C DMA transfer.
  *          This parameter can be: ENABLE or DISABLE.
  * @return None
  */
__STATIC_INLINE void I2C_DMACmd(I2C_T* I2Cx, FunctionalState NewState)
{
    (NewState != DISABLE)?(I2Cx->DMA_CR |= (I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE)):(I2Cx->DMA_CR &= (uint32_t)~((uint32_t)(I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE)));
}
/**
  * @brief  DMA Transmit Data Level Register.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Level: This parameter controls the level at which a DMA request is made by the transmit logic
  * @return None
  */
__STATIC_INLINE void I2C_DMATransferDataLevel(I2C_T* I2Cx, uint8_t Level)
{
    I2Cx->DMA_TDLR |= (Level & 0x07);
}
/**
  * @brief  DMA Receive Data Level Register.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Level: This parameter controls the level at which a DMA request is made by the receive logic.
  * @return None
  */
__STATIC_INLINE void I2C_DMAReceiveDataLevel(I2C_T* I2Cx, uint8_t Level)
{
    I2Cx->DMA_RDLR |= (Level & 0x07);
}
/**
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_IT: specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *          @arg I2C_IT_RX_UNDER
  *          @arg I2C_IT_RX_OVER
  *          @arg I2C_IT_RX_FULL
  *          @arg I2C_IT_TX_OVER
  *          @arg I2C_IT_TX_EMPTY
  *          @arg I2C_IT_RD_REQ
  *          @arg I2C_IT_TX_ABRT
  *          @arg I2C_IT_RX_DONE
  *          @arg I2C_IT_ACTIVITY
  *          @arg I2C_IT_STOP_DET
  *          @arg I2C_IT_START_DET
  *          @arg I2C_IT_GEN_CALL
  *          @arg I2C_IT_MST_ON_HOLD
  * @return The new state of I2C_IT (SET or RESET).
  */
__STATIC_INLINE ITStatus I2C_GetITStatus(I2C_T* I2Cx, uint16_t I2C_IT)
{
    return ((I2Cx->INTR_STAT & (I2C_IT & IT_FLAG_MASK)) != (uint32_t)RESET)?(SET):(RESET);
}
/**
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_IT: specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  *             @arg I2C_IT_MST_ON_HOLD
  * @return The new state of I2C_IT (SET or RESET).
  */
__STATIC_INLINE ITStatus I2C_GetRawITStatus(I2C_T* I2Cx, uint16_t I2C_IT)
{
    return ((I2Cx->RAW_INTR_STAT & (I2C_IT & IT_FLAG_MASK)) != (uint32_t)RESET)?(SET):(RESET);
}
/**
  * @brief  Enables or disables the specified I2C interrupts.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_IT: specifies the I2C interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  * @param  NewState: new state of the specified I2C interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @return None
  */
__STATIC_INLINE void I2C_ITConfig(I2C_T* I2Cx, uint16_t I2C_IT, FunctionalState NewState)
{
    (NewState != DISABLE)?(I2Cx->INTR_MASK |= I2C_IT):(I2Cx->INTR_MASK &= (uint16_t)~I2C_IT);
}

/**
  * @brief  Enables or disables all the specified I2C interrupts.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @return None
  */
__STATIC_INLINE void I2C_DisableAllIT(I2C_T* I2Cx)
{
    I2Cx->INTR_MASK = 0x0;
}
/**
  * @brief  Clears all the I2Cx's interrupt pending bits.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @return None
  */
__STATIC_INLINE void I2C_ClearAllITPendingBit(I2C_T* I2Cx)
{
    (void)(I2Cx->CLR_INTR);
}

/**
  * @brief  Get the source of tx_abort.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @return true or false
  */
__STATIC_INLINE bool I2C_AbortSrcCheck(I2C_T* I2Cx,uint32_t IC_MSK)
{
    return (I2Cx->TX_ABRT_SRC & IC_MSK)?(true):(false);
}


/**
  * @brief  Set slave address.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  addr: slave address.
  * @return true or false
  */
__STATIC_INLINE void I2C_SetSlaveAddr(I2C_T* I2Cx,uint32_t addr)
{
    I2Cx->SAR = (I2Cx->SAR & ~I2C_TAR_TAR) | addr;
}


/**
  * @brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  *
  * @note   To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency
  *         (I2C peripheral input clock) must be a multiple of 10 MHz.
  *
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that contains
  *         the configuration information for the specified I2C peripheral.
  * @note   This function should be called before enabling
            the I2C Peripheral.
  * @retval false	init fail
  * @retval true	init succeed
  *
  *	@code:
	* I2C_InitTypeDef  I2C_InitStructure;
  * 
	* I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	* I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_OWN_ADDRESS7;
	* I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	* I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED_100K;
	* I2C_Cmd(I2Cx, DISABLE);
	* bool ret = I2C_Init(I2Cx, &I2C_InitStructure);
  *
  * @endcode
  */
bool I2C_Init(I2C_T* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Address: specifies the slave 7-bit address which will be transmitted
  * @return None.
  */
void I2C_Set7bitAddress(I2C_T* I2Cx, uint8_t Address);
/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  Address: specifies the slave 10-bit address which will be transmitted
  * @return None.
  */
void I2C_Set10bitAddress(I2C_T* I2Cx, uint16_t Address);

/**
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *          @arg I2C_FLAG_SLV_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  *          @arg I2C_FLAG_RFF
  *          @arg I2C_FLAG_TFE
  *          @arg I2C_FLAG_TFNF
  *          @arg I2C_FLAG_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  * @return The new state of I2C_FLAG (SET or RESET).
  */
FlagStatus I2C_GetFlagStatus(I2C_T* I2Cx, uint32_t I2C_FLAG);
/**
  * @brief  Clears the I2Cx's interrupt pending bits.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  I2C_IT: specifies the interrupt pending bit to clear.
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  *             @arg I2C_IT_MST_ON_HOLD
  * @return None
  */
void I2C_ClearITPendingBit(I2C_T* I2Cx, uint16_t I2C_IT);

/**
  * @brief  read data from the I2Cx peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  WriteBuf: cache of stored readings.
  * @param  Size: number of data read from I2C peripheral.
  * @return none.
  */
void I2C_Write(I2C_T* I2Cx,uint8_t *WriteBuf,uint32_t Size);

/**
  * @brief  write data to the I2Cx peripheral.
  * @param  I2Cx: where I2Cx is a i2c peripheral base address.
  * @param  ReadBuf: cache of stored writings.
  * @param  Size: number of data read from I2C peripheral.
  * @return None.
  */
void I2C_Read(I2C_T* I2Cx,uint8_t *ReadBuf,uint32_t Size);

/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_I2C_H__ */
