/**************************************************************************//**
* @file     pan_hal_i2c.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series I2C (Inter-Integrated Circuit) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/


#include "pan_hal.h"

I2C_HandleTypeDef I2C_Handle_Array[1] = {
	{.I2Cx = I2C0,
	 .I2C_InitObj = {0},  // Assuming I2C_Init_Opt has appropriate zeroed initial state
	 .InterruptObj = {0}, // Assuming I2C_Interrupt_Opt has appropriate zeroed initial state
	 .pTxBuffPtr = NULL,
	 .TxXferSize = 0,
	 .TxXferCount = 0,
	 .pRxBuffPtr = NULL,
	 .RxXferSize = 0,
	 .RxXferCount = 0,
	 .IRQn = I2C0_IRQn, // You might want to replace with an appropriate value
	 .RxIntCallback = NULL,
	 .TxIntCallback = NULL,
	 .DMA_SRC = DMAC_Peripheral_I2C0_Rx,
	 .DMA_DST = DMAC_Peripheral_I2C0_Tx,
	 .StopFlag = 0,
	 .Xfer_Dir = 0,
	 .ErrorCode = 0}};

static void I2C_SendData(I2C_T* I2Cx, uint8_t Data)
{
    I2Cx->DATACMD =  Data & 0xFF;
}

bool HAL_I2C_Init(I2C_HandleTypeDef *i2c)
{
	// I2C_Init() begin
	uint32_t tmpreg = 0;
	uint32_t result = 0x04;
	uint32_t pclk;
	// I2C_Init() end

	// I2C_InitRole() begin
	// The IC_SAR holds the slave address when the I2C is operating as a slave
	if (i2c->I2C_InitObj.AddressMode == I2C_ADDR_10BIT)
	{
		i2c->I2C_InitObj.OwnAddress = I2C_SLAVE_OWN_ADDR_10BIT;
		i2c->I2C_InitObj.AckAddress = I2C_ACK_ADDR_10BIT;
	}
	else
	{
		i2c->I2C_InitObj.OwnAddress = I2C_SLAVE_OWN_ADDR_7BIT;
		i2c->I2C_InitObj.AckAddress = I2C_ACK_ADDR_7BIT;
	}

	I2C_Cmd(i2c->I2Cx, DISABLE);
// I2C_InitRole() end

// I2C_Init() begin
/*get system clock frequency*/
#ifndef FPGA_MODE
	pclk = CLK_GetPeripheralFreq(i2c->I2Cx);
#else
	pclk = CLK_GetHCLKFreq();
#endif

	/*---------------------------- I2Cx CON Configuration ------------------------*/
	/*Disable I2C */

	tmpreg = i2c->I2Cx->CON;

	/*restart enable*/
	tmpreg |= (I2C_CON_IC_RESTART_EN);
	/* slave address mode*/
	// if(I2C_AcknowledgedAddress_7bit == I2C_InitStruct->I2C_AcknowledgedAddress)
	if (I2C_ACK_ADDR_7BIT == i2c->I2C_InitObj.AckAddress)
	{
		tmpreg &= ~I2C_CON_IC_10BITADDR_SLAVE;
	}
	else // 10-bit addr
	{
		tmpreg |= I2C_CON_IC_10BITADDR_SLAVE;
	}

	tmpreg &= ~(I2C_CON_SPEED);
	/*---------------------------- I2Cx HCNT/LCNT Configuration ------------------------*/
	/* Configure speed in standard mode */
	result = pclk / i2c->I2C_InitObj.ClockSpeed;
	if (i2c->I2C_InitObj.ClockSpeed <= 100000)
	{
		/* Standart mode speed calculate: Tlow/Thigh = 1 */
		result /= 2;
		if (result < i2c->I2Cx->FS_SPKLEN + 7)
			return false;
		/*HCNT equals to LCNT*/
		i2c->I2Cx->SS_SCL_HCNT = result - i2c->I2Cx->FS_SPKLEN - 7;
		i2c->I2Cx->SS_SCL_LCNT = result - 1;

		tmpreg |= I2C_SPEED_STANDARD_MODE;
	}
	/* Configure speed in fast mode */
	else if (i2c->I2C_InitObj.ClockSpeed <= 1000000)
	{
		// if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
		if (i2c->I2C_InitObj.DutyCycle == I2C_DutyCycle_2)
		{
			/* Fast mode speed calculate: Tlow/Thigh = 2 */
			result /= 3;
			if (result < i2c->I2Cx->FS_SPKLEN + 7)
				return false;
			i2c->I2Cx->FS_SCL_HCNT = result - i2c->I2Cx->FS_SPKLEN - 7;
			i2c->I2Cx->FS_SCL_LCNT = (result << 1) - 1;
		}
		else
		{
			/* Fast mode speed calculate: Tlow/Thigh = 16/9 */
			if (result * 9 / 25 < i2c->I2Cx->FS_SPKLEN + 7)
				return false;
			/* Set DUTY bit */
			i2c->I2Cx->FS_SCL_HCNT = result * 9 / 25 - i2c->I2Cx->FS_SPKLEN - 7;
			i2c->I2Cx->FS_SCL_LCNT = (result * 16 / 25) - 1;
		}

		tmpreg |= I2C_SPEED_FAST_MODE;
	}
	/* To use the I2C at 1 MHz (in fast mode plus ) */
	else
	{
		result /= 3;
		if (result < i2c->I2Cx->HS_SPKLEN + 7)
			return false;
		/* Set DUTY bit */
		i2c->I2Cx->HS_SCL_HCNT = result - i2c->I2Cx->HS_SPKLEN - 7;
		i2c->I2Cx->HS_SCL_LCNT = (result << 1) - 1;
		tmpreg |= I2C_SPEED_HIGH_MODE;
	}

	i2c->I2Cx->CON = tmpreg;

	/*data setup time: 250ns*/
	i2c->I2Cx->SDA_SETUP = (pclk) / 4000000 + 1;

	/* Set I2Cx Own Address1 and acknowledged address */
	i2c->I2Cx->SAR &= ~(I2C_TAR_TAR);
	/* Get 7-bit address from I2C_OwnAddress1*/
	// i2c->I2Cx->SAR |=  ((I2C_InitStruct ->I2C_OwnAddress1 )) ;
	i2c->I2Cx->SAR |= ((i2c->I2C_InitObj.OwnAddress));
	/*fifo threshold */
	i2c->I2Cx->TX_TL = I2C_TX_TL_4;
	i2c->I2Cx->RX_TL = I2C_RX_TL_4;

	// I2C_InitRole() begin
	//  if(!ret){
	//  	SYS_TEST("apb clk can not support the iic speed %d \n",i2c->I2C_InitObj.I2C_ClockSpeed);
	//   }
	//   if(role == I2C_MODE_MASTER)
	if (i2c->I2C_InitObj.Role == I2C_ROLE_MASTER)
	{
		// 1. set master mode
		I2C_SetMode(i2c->I2Cx, I2C_ROLE_MASTER);
		// 2. set remote slave addr
		if (i2c->I2C_InitObj.AddressMode == I2C_ADDR_10BIT)
		{
			// I2C_Set10bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDRESS10);
			I2C_Set10bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDR_10BIT);
		}
		else
			// I2C_Set7bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDRESS7);
			I2C_Set7bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDR_7BIT);
		i2c->I2Cx->SDA_HOLD = ((0x3 << 16) | 0x3);
	}
	else
	{
		// I2C_SetMode(i2c->I2Cx, I2C_MODE_SLAVE);
		I2C_SetMode(i2c->I2Cx, I2C_ROLE_SLAVE);
	}
	I2C_DisableAllIT(i2c->I2Cx);
	//	 I2C_ITConfig(I2Cx, I2C_IT_RX_FULL|I2C_IT_RD_REQ|I2C_IT_STOP_DET, ENABLE);

	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);
	// I2C_InitRole() end
	return 0;
}

static void HAL_I2C_SetAddress(I2C_HandleTypeDef *i2c, uint16_t DevAddress)
{
	if (i2c->I2C_InitObj.AddressMode == I2C_ADDR_10BIT)
	{
		// I2C_Set10bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDRESS10);
		I2C_Set10bitAddress(i2c->I2Cx, DevAddress);
	}
	else
	{
		// I2C_Set7bitAddress(i2c->I2Cx, I2C_SLAVE_OWN_ADDRESS7);
		I2C_Set7bitAddress(i2c->I2Cx, (uint8_t)DevAddress);
	}

	i2c->I2Cx->SDA_HOLD = ((0x3 << 16) | 0x3);
}

void HAL_I2C_DeInit(I2C_HandleTypeDef *i2c) {}
void HAL_I2C_Init_INT(I2C_HandleTypeDef *i2c)
{
	I2C_Cmd(i2c->I2Cx, DISABLE);
	I2C_DisableAllIT(i2c->I2Cx);
	I2C_ITConfig(i2c->I2Cx, I2C_IT_START_DET | I2C_IT_RX_FULL | I2C_IT_RD_REQ | I2C_IT_STOP_DET, ENABLE);
	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);

	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_0);
	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_0);
}
void HAL_I2C_DeInit_INT(I2C_HandleTypeDef *i2c)
{
}

void HAL_I2C_Master_SendData(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, uint32_t Timeout)
{

	HAL_I2C_SetAddress(i2c, DevAddress);
	i2c->Xfer_Dir = I2C_Tx_Dir;
	while (Size)
	{
		while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF))
			;
		if (Size > 1)
			i2c->I2Cx->DATACMD = ((I2C_CMD_WR << 8) | (*Buf & 0xFF));
		else
		{
			if (I2C_CON_MASTER_MODE & i2c->I2Cx->CON)
				i2c->I2Cx->DATACMD = ((I2C_CMD_STOP << 8) | (*Buf & 0xFF));
			else
				i2c->I2Cx->DATACMD = ((I2C_CMD_WR << 8) | (*Buf & 0xFF));
		}
		Buf++;
		Size--;
	}
}

void HAL_I2C_Master_ReceiveData(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, uint32_t Timeout)
{
	HAL_I2C_SetAddress(i2c, DevAddress);
	i2c->Xfer_Dir = I2C_Rx_Dir;

	// void I2C_Read()
	while (Size)
	{
		if (I2C_CON_MASTER_MODE & i2c->I2Cx->CON)
		{
			while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF))
			{
			}
			if (Size > 1)
			{
				I2C_SendCmd(i2c->I2Cx, I2C_CMD_RD);
			}
			else
			{
				I2C_SendCmd(i2c->I2Cx, I2C_CMD_RD | I2C_CMD_STOP);
			}
		}
		while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_RFNE))
		{
		}
		*Buf = (uint8_t)i2c->I2Cx->DATACMD;
		Buf++;
		Size--;
	}
}

void HAL_I2C_Master_SendData_INT(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	i2c->pTxBuffPtr = Buf;
	i2c->TxXferSize = Size;
	i2c->TxXferCount = 0;
	i2c->TxIntCallback = Callback;
	// i2c->Xfer_Dir = I2C_Tx_Dir;
	i2c->StopFlag = 0;
	I2C_Cmd(i2c->I2Cx, DISABLE);
	HAL_I2C_SetAddress(i2c, DevAddress);
	I2C_DisableAllIT(i2c->I2Cx);
	I2C_ITConfig(i2c->I2Cx, I2C_IT_TX_EMPTY | I2C_IT_START_DET | I2C_IT_RX_FULL | I2C_IT_RD_REQ | I2C_IT_STOP_DET, ENABLE);
	//I2C_ITConfig(i2c->I2Cx, I2C_IT_TX_EMPTY | I2C_IT_START_DET | I2C_IT_STOP_DET, ENABLE);

	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);
	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_4);
	NVIC_EnableIRQ(i2c->IRQn);

	while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF));
	I2C_SendDataCmd(I2C0, *i2c->pTxBuffPtr, I2C_CMD_WR);

	// i2c->I2Cx->DATACMD = ((I2C_CMD_WR << 8) | (*i2c->pTxBuffPtr & 0xFF));
	i2c->TxXferCount++;
}

void HAL_I2C_Master_ReceiveData_INT(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	i2c->pRxBuffPtr = Buf;
	i2c->RxXferSize = Size;
	i2c->RxXferCount = 0;
	i2c->RxIntCallback = Callback;
	// i2c->Xfer_Dir = I2C_Rx_Dir;
	i2c->StopFlag = 0;
	I2C_Cmd(i2c->I2Cx, DISABLE);
	HAL_I2C_SetAddress(i2c, DevAddress);
	I2C_DisableAllIT(i2c->I2Cx);
	I2C_ITConfig(i2c->I2Cx, I2C_IT_START_DET | I2C_IT_RX_FULL | I2C_IT_RD_REQ | I2C_IT_STOP_DET, ENABLE);
	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);
	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_0);

	NVIC_EnableIRQ(i2c->IRQn);

	while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF));
	I2C_SendCmd(i2c->I2Cx, I2C_CMD_RD);

}

void HAL_I2C_Master_SendData_DMA(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	HAL_I2C_SetAddress(i2c, DevAddress);
	i2c->Xfer_Dir = I2C_Tx_Dir;
	i2c->pTxBuffPtr = Buf;
	i2c->TxXferSize = Size;
	i2c->TxXferCount = 0;

	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_0);
	I2C_DMATransferDataLevel(i2c->I2Cx, I2C_TX_TL_0);

	I2C_DMACmd(i2c->I2Cx, ENABLE);
	// Initialize the DMA controller
	DMAC_Init(DMA);
	NVIC_EnableIRQ(DMA_IRQn);

	uint32_t DmaChNum = 0xFF;
	DmaChNum = DMAC_AcquireChannel(DMA);

	dma_mem2i2c_config.PeripheralDst = i2c->DMA_DST;
	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2i2c_config;
	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
	DMAC_Channel_Array[DmaChNum].XferFlag = false;
	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)i2c->pTxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = i2c->TxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
	/*enable dma transfer interrupt*/
	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
	
	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
	/*start dma tx channel*/

	DMAC_StartChannel(DMA, DmaChNum, i2c->pTxBuffPtr, (void *)&(i2c->I2Cx->DATACMD), i2c->TxXferSize);


}

// void HAL_I2C_Master_SendData_DMA_U16(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint16_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
// {
// 	HAL_I2C_SetAddress(i2c, DevAddress);
// 	i2c->Xfer_Dir = I2C_Tx_Dir;
// //	i2c->pTxBuffPtr = Buf;
// 	i2c->TxXferSize = Size;
// 	i2c->TxXferCount = 0;

// 	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_0);
// 	I2C_DMATransferDataLevel(i2c->I2Cx, I2C_TX_TL_0);

// 	I2C_DMACmd(i2c->I2Cx, ENABLE);
// 	// Initialize the DMA controller
// 	DMAC_Init(DMA);
// 	NVIC_EnableIRQ(DMA_IRQn);

// 	uint32_t DmaChNum = 0xFF;
// 	DmaChNum = DMAC_AcquireChannel(DMA);

// 	dma_mem2i2c_config.PeripheralDst = i2c->DMA_DST;
// 	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2i2c_u16_config;
// 	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
// 	DMAC_Channel_Array[DmaChNum].XferFlag = false;
// 	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)Buf;
// 	DMAC_Channel_Array[DmaChNum].XferSize = i2c->TxXferSize;
// 	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
// 	/*enable dma transfer interrupt*/
// 	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
	
// 	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
// 	/*start dma tx channel*/

// 	DMAC_StartChannel(DMA, DmaChNum, Buf, (void *)&(i2c->I2Cx->DATACMD), i2c->TxXferSize);

// }

static void HAL_I2C_Master_SendData_DMA_U16_STC(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint16_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	HAL_I2C_SetAddress(i2c, DevAddress);
	i2c->Xfer_Dir = I2C_Tx_Dir;
//	i2c->pTxBuffPtr = Buf;
	i2c->TxXferSize = Size;
	i2c->TxXferCount = 0;

	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_0);
	I2C_DMATransferDataLevel(i2c->I2Cx, I2C_TX_TL_0);

	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_3);
	I2C_DMAReceiveDataLevel(i2c->I2Cx, I2C_RX_TL_3);

	// I2C_DMACmd(i2c->I2Cx, ENABLE);
	// Initialize the DMA controller
	DMAC_Init(DMA);
	NVIC_EnableIRQ(DMA_IRQn);

	uint32_t DmaChNum = 0xFF;
	DmaChNum = DMAC_AcquireChannel(DMA);

	dma_mem2i2c_config.PeripheralDst = i2c->DMA_DST;
	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2i2c_u16_config;
	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
	DMAC_Channel_Array[DmaChNum].XferFlag = false;
	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)Buf;
	DMAC_Channel_Array[DmaChNum].XferSize = i2c->TxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
	/*enable dma transfer interrupt*/
	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);
	
	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
	/*start dma tx channel*/

	DMAC_StartChannel(DMA, DmaChNum, Buf, (void *)&(i2c->I2Cx->DATACMD), i2c->TxXferSize);

}

static uint16_t cmd_buf[256]={0};

void HAL_I2C_Master_ReceiveData_DMA(I2C_HandleTypeDef *i2c, uint16_t DevAddress, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
    uint16_t i;
	for(i=0;i<Size-1;i++){
        cmd_buf[i] = (I2C_CMD_RD << 8);
    }
    cmd_buf[i] = ((I2C_CMD_RD|I2C_CMD_STOP) << 8);

	HAL_I2C_Master_SendData_DMA_U16_STC(i2c,DevAddress,cmd_buf,Size,NULL);

	// HAL_I2C_SetAddress(i2c, DevAddress);
	i2c->Xfer_Dir = I2C_Rx_Dir;
	i2c->pRxBuffPtr = Buf;
	i2c->RxXferSize = Size;
	i2c->RxXferCount = 0;
	// // I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_3);
	// I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_3);
	// // I2C_DMATransferDataLevel(i2c->I2Cx, I2C_TX_TL_3);
	// I2C_DMAReceiveDataLevel(i2c->I2Cx, I2C_RX_TL_3);
	// I2C_DMACmd(i2c->I2Cx, ENABLE);
	// // Initialize the DMA controller
	// DMAC_Init(DMA);
	// NVIC_EnableIRQ(DMA_IRQn);

	uint32_t DmaChNum = 0xFF;
	/*get free dma channel;*/
	DmaChNum = DMAC_AcquireChannel(DMA);
	/*enable dma transfer interrupt*/
	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);

	dma_i2c2mem_config.PeripheralSrc = i2c->DMA_SRC;
	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_i2c2mem_config;
	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
	DMAC_Channel_Array[DmaChNum].XferFlag = false;
	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t*)i2c->pRxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = i2c->RxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
	/*start dma tx channel*/
	DMAC_StartChannel(DMA, DmaChNum, (void *)&(i2c->I2Cx->DATACMD), i2c->pRxBuffPtr, i2c->RxXferSize);
	I2C_DMACmd(i2c->I2Cx, ENABLE);
	
	// SYS_TEST("blocksize:%d\r\n ch:%d\r\n",i2c->RxXferSize,DmaChNum);
}

void HAL_I2C_Slave_SendData(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, uint32_t Timeout)
{
	// uint8_t i = 0;
	i2c->Xfer_Dir = I2C_Tx_Dir;

	while (1)
	{
		if (I2C_GetRawITStatus(i2c->I2Cx, I2C_IT_RD_REQ))
		{
			// I2C_Write(I2C0, TmpBuf + i, 1);
			// while(Size){
			while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF))
				;
			// if(Size > 1)
			// 	i2c->I2Cx->DATACMD = ((I2C_CMD_WR << 8 ) | (*Buf & 0xFF));
			// else{
			if (I2C_CON_MASTER_MODE & i2c->I2Cx->CON)
				i2c->I2Cx->DATACMD = ((I2C_CMD_STOP << 8) | (*Buf & 0xFF));
			else
				i2c->I2Cx->DATACMD = ((I2C_CMD_WR << 8) | (*Buf & 0xFF));
			// }
			Buf++;
			// Size--;
			// }

			// i++;
			I2C_ClearITPendingBit(i2c->I2Cx, I2C_IT_RD_REQ);
		}
		if (I2C_GetRawITStatus(i2c->I2Cx, I2C_IT_STOP_DET))
		{
			// i = 0;
			I2C_ClearITPendingBit(i2c->I2Cx, I2C_IT_STOP_DET);
			// return 0;
			break;
		}
	}
}

void HAL_I2C_Slave_ReceiveData(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, uint32_t Timeout)
{
	i2c->Xfer_Dir = I2C_Rx_Dir;

	// void I2C_Read()
	while (Size)
	{
		if (I2C_CON_MASTER_MODE & i2c->I2Cx->CON)
		{
			while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_TFNF))
			{
			}
			if (Size > 1)
			{
				I2C_SendCmd(i2c->I2Cx, I2C_CMD_RD);
			}
			else
			{
				I2C_SendCmd(i2c->I2Cx, I2C_CMD_RD | I2C_CMD_STOP);
			}
		}
		while (!I2C_GetFlagStatus(i2c->I2Cx, I2C_FLAG_RFNE))
		{
		}
		*Buf = (uint8_t)i2c->I2Cx->DATACMD;
		Buf++;
		Size--;
	}
}

void HAL_I2C_Slave_SendData_INT(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	// i2c->Xfer_Dir = I2C_Tx_Dir;
	i2c->pTxBuffPtr = Buf;
	i2c->TxXferSize = Size;
	i2c->TxXferCount = 0;
	i2c->TxIntCallback = Callback;
	i2c->StopFlag = 0;
	I2C_Cmd(i2c->I2Cx, DISABLE);
	I2C_DisableAllIT(i2c->I2Cx);
	I2C_ITConfig(i2c->I2Cx, I2C_IT_START_DET | I2C_IT_RX_FULL | I2C_IT_RD_REQ | I2C_IT_STOP_DET, ENABLE);
	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);

	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_0);
	NVIC_EnableIRQ(i2c->IRQn);
}
void HAL_I2C_Slave_ReceiveData_INT(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	// i2c->Xfer_Dir = I2C_Rx_Dir;
	i2c->pRxBuffPtr = Buf;
	i2c->RxXferSize = Size;
	i2c->RxXferCount = 0;
	i2c->RxIntCallback = Callback;
	i2c->StopFlag = 0;

	I2C_Cmd(i2c->I2Cx, DISABLE);
	I2C_DisableAllIT(i2c->I2Cx);
	I2C_ITConfig(i2c->I2Cx, I2C_IT_START_DET | I2C_IT_RX_FULL | I2C_IT_RD_REQ | I2C_IT_STOP_DET, ENABLE);
	/* I2C Peripheral Enable */
	I2C_Cmd(i2c->I2Cx, ENABLE);

	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_8);
	NVIC_EnableIRQ(i2c->IRQn);
}

void HAL_I2C_Slave_SendData_DMA(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
// uint32_t HAL_I2C_Slave_SendData_DMA(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	i2c->Xfer_Dir = I2C_Tx_Dir;
	i2c->pTxBuffPtr = Buf;
	i2c->TxXferSize = Size;
	i2c->TxXferCount = 0;
	i2c->StopFlag = false;

	I2C_SetTxTirggerLevel(i2c->I2Cx, I2C_TX_TL_3);
	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_3);
	I2C_DMATransferDataLevel(i2c->I2Cx, I2C_TX_TL_3);
	I2C_DMAReceiveDataLevel(i2c->I2Cx, I2C_RX_TL_3);
	// I2C_DMACmd(i2c->I2Cx, ENABLE);
	// Initialize the DMA controller
	DMAC_Init(DMA);
	NVIC_EnableIRQ(DMA_IRQn);

	uint32_t DmaChNum = 0xFF;
	/*get free dma channel;*/
	DmaChNum = DMAC_AcquireChannel(DMA);
	/*enable dma transfer interrupt*/
	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);

	dma_mem2i2c_config.PeripheralDst = i2c->DMA_DST;
	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_mem2i2c_config;
	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
	DMAC_Channel_Array[DmaChNum].XferFlag = false;
	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)i2c->pTxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = i2c->TxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
	/*start dma tx channel*/
	DMAC_StartChannel(DMA, DmaChNum, i2c->pTxBuffPtr, (void *)&(i2c->I2Cx->DATACMD), i2c->TxXferSize/4);
	while(!I2C_GetRawITStatus(i2c->I2Cx, I2C_IT_RD_REQ)){}
	I2C_DMACmd(i2c->I2Cx, ENABLE);
	I2C_ClearITPendingBit(i2c->I2Cx, I2C_IT_RD_REQ);
}
void HAL_I2C_Slave_ReceiveData_DMA(I2C_HandleTypeDef *i2c, uint8_t *Buf, uint32_t Size, I2C_CallbackFunc Callback)
{
	i2c->Xfer_Dir = I2C_Rx_Dir;
	i2c->pRxBuffPtr = Buf;
	i2c->RxXferSize = Size;
	i2c->RxXferCount = 0;

	I2C_SetRxTirggerLevel(i2c->I2Cx, I2C_RX_TL_3);
	I2C_DMAReceiveDataLevel(i2c->I2Cx, I2C_RX_TL_3);

	I2C_DMACmd(i2c->I2Cx, ENABLE);
	// Initialize the DMA controller
	DMAC_Init(DMA);
	NVIC_EnableIRQ(DMA_IRQn);

	uint32_t DmaChNum = 0xFF;
	DmaChNum = DMAC_AcquireChannel(DMA);
	DMAC_ClrIntFlagMsk(DMA, DmaChNum, DMAC_FLAG_INDEX_TFR);

	dma_i2c2mem_config.PeripheralSrc = i2c->DMA_SRC;
	DMAC_Channel_Array[DmaChNum].ConfigTmp = dma_i2c2mem_config;
	DMAC_Channel_Array[DmaChNum].CallbackI2c = Callback;
	DMAC_Channel_Array[DmaChNum].XferFlag = false;
	DMAC_Channel_Array[DmaChNum].pBuffPtr = (uint32_t *)i2c->pRxBuffPtr;
	DMAC_Channel_Array[DmaChNum].XferSize = i2c->RxXferSize;
	DMAC_Channel_Array[DmaChNum].PeriMode = DMAC_Peri_I2C;
	DMAC_SetChannelConfig(DMA, DmaChNum, &DMAC_Channel_Array[DmaChNum].ConfigTmp);
	/*start dma tx channel*/
	DMAC_StartChannel(DMA, DmaChNum, (void *)&(i2c->I2Cx->DATACMD), i2c->pRxBuffPtr, i2c->RxXferSize);

}

void I2C0_IRQHandler()
{
	// I2C_IT_TX_EMPTY  不能软件清零
	if (I2C_GetITStatus(I2C0, I2C_IT_TX_EMPTY))
	{
		// SYS_TEST("TE\r\n");
		while (I2C_GetFlagStatus(I2C0, I2C_FLAG_TFNF) == 0);
		// SYS_TEST("TE1\r\n");
		if (I2C0_OBJ.I2C_InitObj.Role == I2C_ROLE_MASTER)
		{
			// if (I2C0_OBJ.Xfer_Dir == I2C_Tx_Dir)
			// {
				I2C0_OBJ.I2Cx->DATACMD = ((I2C_CMD_WR << 8) | (I2C0_OBJ.pTxBuffPtr[I2C0_OBJ.TxXferCount] & 0xFF));
				I2C0_OBJ.TxXferCount++;
				if (I2C0_OBJ.TxXferCount >= I2C0_OBJ.TxXferSize)
				{
					I2C_SendCmd(I2C0_OBJ.I2Cx, I2C_CMD_STOP);
					I2C0_OBJ.Xfer_Dir = I2C_Tx_Dir;
					I2C_ITConfig(I2C0_OBJ.I2Cx, I2C_IT_TX_EMPTY, DISABLE);
					if (I2C0_OBJ.TxIntCallback != NULL)
					{
						I2C0_OBJ.TxIntCallback(I2C_CB_FLAG_STOP, I2C0_OBJ.pTxBuffPtr, I2C0_OBJ.TxXferCount);
					}
				}
			// }
		}

		// I2C_ClearITPendingBit(I2C0, I2C_IT_TX_EMPTY);
	}
	if (I2C_GetITStatus(I2C0_OBJ.I2Cx, I2C_IT_START_DET))
	{
		//		SYS_TEST("I2C_IT_START_DET \r\n");
		I2C_ClearITPendingBit(I2C0_OBJ.I2Cx, I2C_IT_START_DET);
	}
	if (I2C_GetITStatus(I2C0_OBJ.I2Cx, I2C_IT_TX_ABORT))
	{
		//		SYS_TEST("I2C_IT_TX_ABORT \r\n");
		I2C_ClearITPendingBit(I2C0_OBJ.I2Cx, I2C_IT_TX_ABORT);
	}
	if (I2C_GetITStatus(I2C0_OBJ.I2Cx, I2C_IT_RX_FULL))
	{
//		SYS_TEST("I2C_IT_RX_FULL \r\n");
					// SYS_TEST("RF");

		while (I2C_GetFlagStatus(I2C0_OBJ.I2Cx, I2C_FLAG_RFNE))
		{
			if(I2C0_OBJ.RxXferCount < (I2C0_OBJ.RxXferSize)){

				I2C0_OBJ.pRxBuffPtr[I2C0_OBJ.RxXferCount] = I2C_ReceiveData(I2C0_OBJ.I2Cx);
				if (I2C0_OBJ.I2C_InitObj.Role == I2C_ROLE_MASTER)
				{
					I2C_SendCmd(I2C0_OBJ.I2Cx, I2C_CMD_RD);
				}
				I2C0_OBJ.RxXferCount++;
			}
			if (I2C0_OBJ.RxXferCount == I2C0_OBJ.RxXferSize)
			{
				// SYS_TEST("RF1\r\n");
				if (I2C0_OBJ.I2C_InitObj.Role == I2C_ROLE_MASTER)
				{
					I2C_SendCmd(I2C0_OBJ.I2Cx, I2C_CMD_STOP);
				}
				if (I2C0_OBJ.RxIntCallback != NULL)
				{
					I2C0_OBJ.RxIntCallback(I2C_CB_FLAG_STOP, I2C0_OBJ.pRxBuffPtr, I2C0_OBJ.RxXferCount);
					I2C0_OBJ.RxIntCallback=NULL;
				}
				I2C0_OBJ.StopFlag = 1;
			}
		}

		I2C_ClearITPendingBit(I2C0_OBJ.I2Cx, I2C_IT_RX_FULL);
//				SYS_TEST("t%d\r\n",I2C0_OBJ.RxXferCount);

	}

	if (I2C_GetITStatus(I2C0_OBJ.I2Cx, I2C_IT_STOP_DET))
	{

//		SYS_TEST("I2C_IT_STOP_DET \r\n");
		// I2C0_OBJ.TxXferCount = 0;
		while (I2C_GetFlagStatus(I2C0_OBJ.I2Cx, I2C_FLAG_RFNE))
		{
//			I2C0_OBJ.pRxBuffPtr[I2C0_OBJ.RxXferCount] = I2C_ReceiveData(I2C0_OBJ.I2Cx);
//			I2C0_OBJ.RxXferCount++;
		}
		I2C0_OBJ.StopFlag = 1;
		if (I2C0_OBJ.Xfer_Dir == I2C_Rx_Dir)
		{

		}
		else if (I2C0_OBJ.Xfer_Dir == I2C_Tx_Dir)
		{

		}

		I2C_ClearITPendingBit(I2C0_OBJ.I2Cx, I2C_IT_STOP_DET);
	}

	if (I2C_GetITStatus(I2C0, I2C_IT_RD_REQ)) // 只有作为Slave，Master	想读时触发
	{
		// SYS_TEST("RD \r\n");
		while (I2C_GetFlagStatus(I2C0, I2C_FLAG_TFNF) == 0);// wait tx fifo no full
		// SYS_TEST("RD1 \r\n");
		if (I2C0_OBJ.I2C_InitObj.Role == I2C_ROLE_SLAVE)
		{
			I2C_SendData(I2C0, I2C0_OBJ.pTxBuffPtr[I2C0_OBJ.TxXferCount]);
			// I2C_SendDataCmd(I2C0, I2C0_OBJ.pTxBuffPtr[I2C0_OBJ.TxXferCount], I2C_CMD_WR);
			I2C0_OBJ.TxXferCount++;
//			SYS_TEST("RD %d\r\n",I2C0_OBJ.TxXferCount);
			if (I2C0_OBJ.TxXferCount >= I2C0_OBJ.TxXferSize)
			{
				// I2C_SendCmd(I2C0, I2C_CMD_STOP);
				if (I2C0_OBJ.TxIntCallback != NULL)
				{
					I2C0_OBJ.TxIntCallback(I2C_CB_FLAG_STOP, I2C0_OBJ.pTxBuffPtr, I2C0_OBJ.TxXferCount);
				}
			}
		}
	
		I2C_ClearITPendingBit(I2C0, I2C_IT_RD_REQ);
	}
}

