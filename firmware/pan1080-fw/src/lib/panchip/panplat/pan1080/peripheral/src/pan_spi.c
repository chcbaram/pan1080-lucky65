/**************************************************************************//**
 * @file     spi.c
 * @version  V1.10
 * $Revision: 2 $
 * $Date:    19/10/22  17:18 $
 * @brief    PAN series SPI driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PanSeries.h"
#include "pan_spi.h"

/** @addtogroup PAN_Device_Driver PAN Device Driver
  @{
*/

/** @addtogroup PAN_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup PAN_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  Initializes the SPIx peripheral according to the specified
  *         parameters in the SPI_InitStruct .
  * @param  SPIx: where x can be 1, 2 to select the
  *         SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that contains
  *         the configuration information for the specified SPI peripheral.
  * @retval None
  */
void SPI_Init(SPI_T* SPIx, SPI_InitTypeDef* SPI_InitStruct)
{
    uint32_t tmpreg = 0;

    if(SPI_IsSpiEnabled(SPIx))
        SPI_DisableSpi(SPIx);

    tmpreg = SPIx->CR0;

    tmpreg &= ~(SPI_CR0_SPH_Msk | SPI_CR0_SPO_Msk  | SPI_CR0_DSS_Msk);
    tmpreg |= (
               ((SPI_InitStruct->SPI_CPHA           << SPI_CR0_SPH_Pos) & SPI_CR0_SPH_Msk )  |   \
                ((SPI_InitStruct->SPI_CPOL          << SPI_CR0_SPO_Pos) & SPI_CR0_SPO_Msk )  |   \
                ((SPI_InitStruct->SPI_dataFrameSize << SPI_CR0_DSS_Pos) & SPI_CR0_DSS_Msk )      \
              );
    tmpreg &= ~(SPI_CR0_FRF_Msk);
    tmpreg |= (SPI_InitStruct->SPI_format << SPI_CR0_FRF_Pos);
    tmpreg &= ~(SPI_CR0_SCR_Msk);
    tmpreg |= ((SPI_InitStruct->SPI_baudRateDiv-1) << SPI_CR0_SCR_Pos);   //Fixed SCR to 0, only use CPSR to config baudrate

    SPIx->CR0 = tmpreg;

    //Baudrate Config, expect_baudrate = apb_clk / ((1 + SCR) * CPSR)
    SPIx->CPSR = 2;//SPI_InitStruct->SPI_baudRateDiv;

    //Role Select
    if(SPI_InitStruct->SPI_role == SPI_RoleSlave)
    {
        SPIx->CR1 |= SPI_CR1_MS_Msk;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_MS_Msk;
    }
}


/*@}*/ /* end of group PAN_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PAN_SPI_Driver */

/*@}*/ /* end of group PAN_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
