/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series efuse driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */
#ifndef __PAN_EFUSE_H__
#define __PAN_EFUSE_H__

/**
 * @brief Efuse Interface
 * @defgroup efuse_interface Efuse Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**@defgroup EFUSE_OPERATE_FLAG Efuse operate command 
 * @brief       Efuse operate command definitions
 * @{ */
#define EFUSE_CMD_READ      (0X0)   /*!< Command to read efuse momery */
#define EFUSE_CMD_PROGRAM   (0X1)   /*!< Command to program data to efuse memory */
#define EFUSE_CMD_INACTIVE  (0X2)   /*!< Command to disable efuse operate */
/**@} */

/**@defgroup EFUSE_STATUS_FLAG Efuse status 
 * @brief       Efuse status definitions
 * @{ */
#define EFUSE_STATUS_OK     (0x0)   /*!< Status of efuse operate,status ok */
#define EFUSE_STATUS_FAIL   (0x1)   /*!< Status of efuse operate,status fail */
/**@} */

/**
  * @brief  Init eFuse module.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @return None.
  *
  * @note Unlock register before use this API.
  */
__STATIC_INLINE void EFUSE_Init(EFUSE_T* efuse)
{
    efuse->EF_VDD = EFUSE_VDD_DVDD_REG_Msk;
    efuse->EF_CTL = EFUSE_CTL_EFUSE_EN_Msk |
                    EFUSE_CTL_EFUSECTL_BANK1_Msk |
                    EFUSE_CTL_EFUSECTL_BANK2_Msk |
                    EFUSE_CTL_EFUSECTL_BANK3_Msk |
                    EFUSE_CTL_EFUSECTL_BANK4_Msk;
}

/**
  * @brief  Un-Init eFuse module.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @return None.
  *
  * @note Unlock register before use this API.
  */
__STATIC_INLINE void EFUSE_UnInit(EFUSE_T* efuse)
{
    efuse->EF_CTL = 0;
    efuse->EF_VDD = 0;  // Disable eFuse VDD to save power
}

/**
  * @brief  Get eFuse error status flag.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @retval EFUSE_STATUS_OK    The previous eFuse operation success.
  * @retval EFUSE_STATUS_FAIL  The previous eFuse operation is illegal.
  */
__STATIC_INLINE uint32_t EFUSE_GetErrorStatus(EFUSE_T* efuse)
{
    return efuse->EF_OP_ERROR;
}

/**
  * @brief  Clear eFuse error status flag.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @return None
  */
__STATIC_INLINE void EFUSE_ClrErrorStatus(EFUSE_T* efuse)
{
    // Write 0x1 to clear this register
    efuse->EF_OP_ERROR = 1u;
}

/**
  * @brief  Read specified one byte from eFuse.
  *
  * @param[in]  efuse   Module name of eFuse, typically EFUSE.
  * @param[in]  addr    eFuse address to read data from.
  * @return Data read from eFuse.
  *
  * @note Unlock register before use this API.
  */
extern uint8_t EFUSE_ReadByte(EFUSE_T* efuse, uint32_t addr);

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
extern void EFUSE_WriteByte(EFUSE_T* efuse, uint32_t addr, uint8_t data);

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
extern bool EFUSE_Read(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len);

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
extern bool EFUSE_Write(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len);

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
extern bool EFUSE_UserRead(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len);

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
extern bool EFUSE_UserWrite(EFUSE_T* efuse, uint32_t addr, uint8_t* data, uint16_t len);


/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_EFUSE_H__ */
