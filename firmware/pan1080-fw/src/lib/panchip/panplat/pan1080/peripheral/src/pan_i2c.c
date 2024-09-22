/**************************************************************************//**
 * @file     i2c.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/2216:24 $
 * @brief    Panchip series I2C driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_i2c.h"
#include "pan_clk.h"

/**
  * @brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  *
  * @note   To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency
  *         (I2C peripheral input clock) must be a multiple of 10 MHz.
  *
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that contains
  *         the configuration information for the specified I2C peripheral.
  * @note   This function should be called before enabling
            the I2C Peripheral.

  * @retval None
  */
bool I2C_Init(I2C_T* I2Cx, I2C_InitTypeDef* I2C_InitStruct)
{
	uint32_t tmpreg = 0;
	uint32_t result = 0x04;
	uint32_t pclk;

	/*get system clock frequency*/
    pclk = CLK_GetPeripheralFreq(I2Cx);

	/*---------------------------- I2Cx CON Configuration ------------------------*/
	/*Disable I2C */

	tmpreg = I2Cx->CON;

	/*restart enable*/
	tmpreg |= (I2C_CON_IC_RESTART_EN);
	/* slave address mode*/
	if(I2C_AcknowledgedAddress_7bit == I2C_InitStruct->I2C_AcknowledgedAddress)
	{
		tmpreg &= ~I2C_CON_IC_10BITADDR_SLAVE;
	}
	else//10-bit addr
	{
		tmpreg |= I2C_CON_IC_10BITADDR_SLAVE;
	}

    tmpreg &= ~(I2C_CON_SPEED);
	/*---------------------------- I2Cx HCNT/LCNT Configuration ------------------------*/
	/* Configure speed in standard mode */
	result = pclk / I2C_InitStruct->I2C_ClockSpeed;
	if (I2C_InitStruct->I2C_ClockSpeed <= 100000)
	{
		/* Standart mode speed calculate: Tlow/Thigh = 1 */
		result /= 2;
		if(result < I2Cx->FS_SPKLEN +7)
			return false;
		/*HCNT equals to LCNT*/
		I2Cx->SS_SCL_HCNT = result - I2Cx->FS_SPKLEN - 7;
		I2Cx->SS_SCL_LCNT = result - 1;

		tmpreg |= I2C_SPEED_STANDARD_MODE;
	}
	/* Configure speed in fast mode */
	else  if (I2C_InitStruct->I2C_ClockSpeed <= 1000000)
	{
		if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
		{
			/* Fast mode speed calculate: Tlow/Thigh = 2 */
			result /= 3;
			if(result < I2Cx->FS_SPKLEN +7)
				return false;
			I2Cx->FS_SCL_HCNT = result - I2Cx->FS_SPKLEN - 7;
			I2Cx->FS_SCL_LCNT = (result << 1) - 1;            
		}
		else
		{
			/* Fast mode speed calculate: Tlow/Thigh = 16/9 */
			if(result*9/25 < I2Cx->FS_SPKLEN +7)
				return false;			
			/* Set DUTY bit */
			I2Cx->FS_SCL_HCNT = result * 9 / 25 - I2Cx->FS_SPKLEN - 7;
			I2Cx->FS_SCL_LCNT = (result * 16 / 25) - 1;
		}

		tmpreg |= I2C_SPEED_FAST_MODE;

	}
    /* To use the I2C at 1 MHz (in fast mode plus ) */
	else
	{
		result /= 3;
		if(result < I2Cx->HS_SPKLEN +7)
			return false;
        /* Set DUTY bit */
        I2Cx->HS_SCL_HCNT = result - I2Cx->HS_SPKLEN - 7;
        I2Cx->HS_SCL_LCNT = (result << 1) - 1;
        tmpreg |= I2C_SPEED_HIGH_MODE;
	}

	I2Cx->CON = tmpreg;

	/*data setup time: 250ns*/
	I2Cx->SDA_SETUP = (pclk) / 4000000 + 1;

	/* Set I2Cx Own Address1 and acknowledged address */
	I2Cx->SAR &= ~( I2C_TAR_TAR);
	/* Get 7-bit address from I2C_OwnAddress1*/
	I2Cx->SAR |=  ((I2C_InitStruct ->I2C_OwnAddress1 )) ;
	/*fifo threshold */
	I2Cx->TX_TL = I2C_TX_TL_4;
	I2Cx->RX_TL = I2C_RX_TL_4;
	return true;
}


/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave 7-bit address which will be transmitted
  * @retval None.
  */
void I2C_Set7bitAddress(I2C_T* I2Cx, uint8_t Address)
{
	uint32_t tmpreg;
	 /* Check the parameters */

	tmpreg = I2Cx->TAR;

    /*7-bits address mode*/
	tmpreg &= ~(I2C_TAR_IC_10BITADDR_MASTER | I2C_TAR_SPECIAL | I2C_TAR_GC_OR_START | I2C_TAR_TAR);

	tmpreg |= (Address & 0x07F);

	/* Send the address */
	I2Cx->TAR = tmpreg;

}

/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave 10-bit address which will be transmitted
  * @retval None.
  */
void I2C_Set10bitAddress(I2C_T* I2Cx, uint16_t Address)
{
	uint32_t tmpreg;

	tmpreg = I2Cx->TAR;

    /*7-bits address mode*/
	tmpreg &= ~(I2C_TAR_IC_10BITADDR_MASTER | I2C_TAR_SPECIAL | I2C_TAR_GC_OR_START | I2C_TAR_TAR);

	tmpreg |= (I2C_TAR_IC_10BITADDR_MASTER | (Address & 0x03ff));

	/* Send the address */
	I2Cx->TAR = tmpreg;

}

/**
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *          @arg I2C_FLAG_SLV_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  *          @arg I2C_FLAG_RFF
  *          @arg I2C_FLAG_TFE
  *          @arg I2C_FLAG_TFNF
  *          @arg I2C_FLAG_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
FlagStatus I2C_GetFlagStatus(I2C_T* I2Cx, uint32_t I2C_FLAG)
{
	FlagStatus bitstatus = RESET;
	__IO uint32_t i2cxbase = 0;

	/* Get the I2Cx peripheral base address */
	i2cxbase = (uint32_t)I2Cx;

	/* Get bit[23:0] of the flag */
	I2C_FLAG &= FLAG_MASK;
	i2cxbase += 0x70;

	if(((*(__IO uint32_t *)i2cxbase) & I2C_FLAG) != (uint32_t)RESET)
	{
		/* I2C_FLAG is set */
		bitstatus = SET;
	}
	else
	{
		/* I2C_FLAG is reset */
		bitstatus = RESET;
	}

	/* Return the I2C_FLAG status */
	return  bitstatus;
}


/**
  * @brief  Clears the I2Cx's interrupt pending bits.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
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
  * @retval None
  */
void I2C_ClearITPendingBit(I2C_T* I2Cx, uint16_t I2C_IT)
{
    uint16_t flagpos = 0;

    /* Get the I2C flag position */
    flagpos = (I2C_IT & IT_FLAG_MASK);

    /* Clear the selected I2C flag */
    if(flagpos & I2C_IT_RX_UNDER)
	    (void)(I2Cx->CLR_RX_UND);

    if(flagpos & I2C_IT_RX_OVER)
	    (void)(I2Cx->CLR_RX_OVR);

    if(flagpos & I2C_IT_TX_OVER)
	    (void)(I2Cx->CLR_TX_OVR);

    if(flagpos & I2C_IT_RD_REQ)
	    (void)(I2Cx->CLR_RD_REQ);

    if(flagpos & I2C_IT_TX_ABORT)
	    (void)(I2Cx->CLR_TX_ABRT);

    if(flagpos & I2C_IT_RX_DONE)
	    (void)(I2Cx->CLR_RX_DONE);

    if(flagpos & I2C_IT_ACTIVITY)
	    (void)(I2Cx->CLR_ACTIVITY);

    if(flagpos & I2C_IT_STOP_DET)
	    (void)(I2Cx->CLR_STOP_DET);

    if(flagpos & I2C_IT_START_DET)
		(void)(I2Cx->CLR_START_DET);

    if(flagpos & I2C_IT_GEN_CALL)
		(void)(I2Cx->CLR_GEN_CALL);

	if(flagpos & I2C_IT_MST_ON_HOLD)
		(void)(I2Cx->CLR_RESTART_DET);
	
	if(flagpos & I2C_IT_ACTIVITY)
		(void)(I2Cx->CLR_ACTIVITY);
}

/**
  * @brief  read data from the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  ReadBuf: cache of stored readings.
  * @param  Size: number of data read from I2C peripheral.
  * @retval none.
  */
void I2C_Read(I2C_T* I2Cx,uint8_t *ReadBuf,uint32_t Size)
{
    while(Size){
        if(I2C_CON_MASTER_MODE & I2Cx->CON){
            while(!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TFNF)){}
			if(Size > 1){
				I2C_SendCmd(I2Cx,I2C_CMD_RD);
			}else{
				I2C_SendCmd(I2Cx,I2C_CMD_RD|I2C_CMD_STOP);
			}
        }
        while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RFNE)){}
        *ReadBuf = (uint8_t)I2Cx->DATACMD;
        ReadBuf++;
        Size--;
    }
}

/**
  * @brief  write data to the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  WriteBuf: cache of stored writings.
  * @param  Size: number of data read from I2C peripheral.
  * @retval none.
  */
void I2C_Write(I2C_T* I2Cx,uint8_t *WriteBuf,uint32_t Size)
{
    while(Size){
        while(!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TFNF));
        if(Size > 1)
            I2Cx->DATACMD = ((I2C_CMD_WR << 8 ) | (*WriteBuf & 0xFF));
        else{
            if(I2C_CON_MASTER_MODE & I2Cx->CON)
                I2Cx->DATACMD = ((I2C_CMD_STOP << 8 ) | (*WriteBuf & 0xFF));
            else
                I2Cx->DATACMD = ((I2C_CMD_WR << 8 ) | (*WriteBuf & 0xFF));
        }
        WriteBuf++;
        Size--;
    }
}

#ifndef __OS_SYSTEM_MODE__
void I2C_OsRead(I2C_T* I2Cx,uint8_t *ReadBuf,uint32_t Size)
{
}
#endif
