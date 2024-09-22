/**************************************************************************//**
 * @file     efuse.c
 * @version  V1.00
 * $Revision: 0$
 * $Date: 20/10/14 15:26 $ 
 * @brief    Panchip SoC eFuse driver source file
 *
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#include "PanSeries.h"
#include "pan_efuse.h"


/**
  * @brief  Read specified one byte from eFuse.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to read data from.
  * @retval Data read from eFuse.
  *
  * @note Unlock register before use this API.
  */
uint8_t EFUSE_ReadByte(EFUSE_T* efuse, uint32_t addr)
{
    efuse->EF_CMD   = EFUSE_CMD_READ;
    efuse->EF_ADDR  = addr;
    efuse->EF_TRG   = EFUSE_TRG_EFUSETRG_GO_Msk;
    while(efuse->EF_TRG){}

    return efuse->EF_DAT;
}

/**
  * @brief  Read multiple data from eFuse.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to read data from.
  * @param[in]  data    Buffer to store data read from eFuse.
  * @param[in]  len     Number of bytes to read.
  * @retval true    Read success.
  * @retval false   Error occurs while reading.
  *
  * @note Unlock register before use this API.
  */
bool EFUSE_Read(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        efuse->EF_CMD   = EFUSE_CMD_READ;
        efuse->EF_ADDR  = addr + i;
        efuse->EF_TRG   = EFUSE_TRG_EFUSETRG_GO_Msk;
        while(efuse->EF_TRG){}

        if (efuse->EF_OP_ERROR)
        {
            efuse->EF_OP_ERROR = 1u;    // Clear error status reg
            return false;
        }

        data[i] = efuse->EF_DAT;
    }

    return true;
}

/**
  * @brief  Write one byte data to specified eFuse address.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to write data to.
  * @param[in]  data    Data to write to eFuse.
  * @return None.
  *
  * @note Unlock register before use this API.
  */
void EFUSE_WriteByte(EFUSE_T* efuse, uint32_t addr, uint8_t data)
{
    efuse->EF_VDD = (EFUSE_VDD_AVDD_REG_Msk | EFUSE_VDD_DVDD_REG_Msk);

    efuse->EF_CMD    = EFUSE_CMD_PROGRAM;
    efuse->EF_ADDR   = addr;
    efuse->EF_DAT    = data;
    efuse->EF_TRG   |= EFUSE_TRG_EFUSETRG_GO_Msk;
    while(efuse->EF_TRG){}

    efuse->EF_VDD &= ~(EFUSE_VDD_AVDD_REG_Msk);
}

/**
  * @brief  Write multiple data to eFuse.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to write data to.
  * @param[in]  data    Buffer to store data to be write to eFuse.
  * @param[in]  len     Number of bytes to write.
  * @retval true    Write success.
  * @retval false   Error occurs while writing.
  *
  * @note Unlock register before use this API.
  */
bool EFUSE_Write(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len)
{
    efuse->EF_VDD = (EFUSE_VDD_AVDD_REG_Msk | EFUSE_VDD_DVDD_REG_Msk); //open avdd

    for (size_t i = 0; i < len; i++)
    {
        efuse->EF_CMD    = EFUSE_CMD_PROGRAM;
        efuse->EF_ADDR   = addr + i;
        efuse->EF_DAT    = data[i];
        efuse->EF_TRG   |= EFUSE_TRG_EFUSETRG_GO_Msk;
        while(efuse->EF_TRG){}

        if (efuse->EF_OP_ERROR)
        {
            efuse->EF_OP_ERROR = 1u;    // Clear error status reg
            efuse->EF_VDD &= ~(EFUSE_VDD_AVDD_REG_Msk); // Disconnect AVDD before return
            return false;
        }
    }

    efuse->EF_VDD &= ~(EFUSE_VDD_AVDD_REG_Msk);

    return true;
}

/**
  * @brief  Read multiple data from User Area of eFuse (Address 0xE9 ~ 0xFF).
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to read data from, should be in range of 0xE9 ~ 0xFF.
  * @param[in]  data    Buffer to store data read from eFuse.
  * @param[in]  len     Number of bytes to read.
  * @retval true    Read success.
  * @retval false   Error, illegal parameter (addr or len).
  *
  * @note Unlock register before use this API.
  */
bool EFUSE_UserRead(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len)
{
    if ((addr < 0xE9) || (addr > 0xFF) || (addr + len > 0x100))
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        efuse->EF_CMD   = EFUSE_CMD_READ;
        efuse->EF_ADDR  = addr + i;
        efuse->EF_TRG   = EFUSE_TRG_EFUSETRG_GO_Msk;
        while(efuse->EF_TRG){}
        data[i] = efuse->EF_DAT;
    }

    return true;
}

/**
  * @brief  Write multiple data to User Area of eFuse (Address 0xE9 ~ 0xFF).
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to write data to, should be in range of 0xE9 ~ 0xFF.
  * @param[in]  data    Buffer to store data to be write to eFuse.
  * @param[in]  len     Number of bytes to write.
  * @retval true    Write success.
  * @retval false   Error, illegal parameter (addr or len).
  *
  * @note Unlock register before use this API.
  */
bool EFUSE_UserWrite(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len)
{
    if ((addr < 0xE9) || (addr > 0xFF) || (addr + len > 0x100))
    {
        return false;
    }

    efuse->EF_VDD = (EFUSE_VDD_AVDD_REG_Msk | EFUSE_VDD_DVDD_REG_Msk); //open avdd

    for (size_t i = 0; i < len; i++)
    {
        efuse->EF_CMD    = EFUSE_CMD_PROGRAM;
        efuse->EF_ADDR   = addr + i;
        efuse->EF_DAT    = data[i];
        efuse->EF_TRG   |= EFUSE_TRG_EFUSETRG_GO_Msk;
        while(efuse->EF_TRG){}
    }

    efuse->EF_VDD &= ~(EFUSE_VDD_AVDD_REG_Msk);

    return true;
}
