/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief    Panchip series kscan driver header file
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $
 */
#ifndef __PAN_KSCAN_H__
#define __PAN_KSCAN_H__

/**
 * @brief Kscan Interface
 * @defgroup kscan_interface Kscan Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif

/**@defgroup KSCAN_ENABLE_FLAG Kscan enable bit
 * @brief       Kscan enable bit definitions
 * @{ */
#define KS_PERIPHRAL_ENABLE                 (0X00000001)  /*!< Enable kscan peripheral */
#define KS_WAKEUP_ENABLE                    (0X00000002)  /*!< Enable kscan wakeup function */
#define KS_WAKEUP_AND_PERIPH_ENABLE         (0X00000003)  /*!< Enable kscan peripheral and wakeup function */
/**@} */

/**@defgroup KSCAN_INTERVAL_CLK_FLAG Kscan interval clk cycle
 * @brief       Kscan interval clk cycle definitions
 * @{ */
#define KS_INTERVAL_TIME_CLK_1              (0X0)   /*!< 1 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_2              (0X1)   /*!< 2 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_4              (0X2)   /*!< 4 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_8              (0X3)   /*!< 8 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_16             (0X4)   /*!< 16 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_32             (0X5)   /*!< 32 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_64             (0X6)   /*!< 64 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_128            (0X7)   /*!< 128 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_256            (0X8)   /*!< 256 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_512            (0X9)   /*!< 512 sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_1024           (0XA)   /*!< 1K sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_2048           (0XB)   /*!< 2k sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_4K             (0XC)   /*!< 4k sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_8K             (0XD)   /*!< 8k sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_16K            (0XE)   /*!< 16k sclk between the current row and the next row */
#define KS_INTERVAL_TIME_CLK_32K            (0XF)   /*!< 32k sclk between the current row and the next row */
/**@} */


/**
  * @brief  This function is set keyscan enable bit
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of the keyscan peripheral
  * @param  func: function select
                        \ref KS_PERIPHRAL_ENABLE
                        \ref KS_WAKEUP_ENABLE
                        \ref KS_WAKEUP_AND_PERIPH_ENABLE
  * @return none
  */
__STATIC_INLINE void KS_Enable(KSCAN_T *ks,FunctionalState NewState,uint32_t func)
{
    (NewState != DISABLE)?((ks)->KS_EN |= func):((ks)->KS_EN &= ~func);
}


/**
  * @brief  This function is set keyscan input enable function
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of the keyscan peripheral
  * @param  func: function select,input range[7~0]
  * @return none
  */
__STATIC_INLINE void KS_InputEnable(KSCAN_T *ks,FunctionalState NewState,uint32_t func)
{
    (NewState != DISABLE)?((ks)->KS_IO_CFG |= func):((ks)->KS_IO_CFG &= ~func);
}

/**
  * @brief  This function is set keyscan output enable function
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of the keyscan peripheral
  * @param  func: function select,input range[23~0]
  * @return none
  */
__STATIC_INLINE void KS_OutputEnable(KSCAN_T *ks,FunctionalState NewState,uint32_t func)
{
    (NewState != DISABLE)?((ks)->KS_IO_CFG |= (func << 8)):((ks)->KS_IO_CFG &= ~(func << 8));
}

/**
  * @brief  This function is used to check keyscan status
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of the keyscan peripheral
  * @return none
  */
__STATIC_INLINE void KS_IntEnable(KSCAN_T *ks,FunctionalState NewState)
{
    (NewState != DISABLE)?((ks)->KS_INT_CFG |= KS_INT_ENABLE_Msk):((ks)->KS_INT_CFG &= ~KS_INT_ENABLE_Msk);
}

/**
  * @brief  This function is used to check keyscan status
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  func: status select
  *                             \ref KS_INT_FLAG_Msk
  *                             \ref KS_FLAG_Msk
  * @return none
  */
__STATIC_INLINE bool KS_StatusCheck(KSCAN_T *ks,uint32_t func)
{
    return ((ks)->KS_INT_CFG & func)?(true):(false);
}

/**
  * @brief  This function is used to clear keyscan status
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  func: status select
  *                             \ref KS_INT_FLAG_Msk
  *                             \ref KS_FLAG_Msk
  * @return none
  */
__STATIC_INLINE void KS_ClearFlag(KSCAN_T *ks,uint32_t func)
{
    (ks)->KS_INT_CFG |= func;
}

/**
  * @brief  This function is used to set keyscan polarity as high
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of the keyscan polarity
  * @return none
  */
__STATIC_INLINE void KS_SetKeyscanPolarity(KSCAN_T *ks,FunctionalState NewState)
{
    (NewState != DISABLE)?((ks)->KS_CFG |= KS_POLARITY_VALID_HIGH_Msk):((ks)->KS_CFG &= ~KS_POLARITY_VALID_HIGH_Msk);
}

/**
  * @brief  This function is used to clear all key status enable
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  NewState: new state of all status clear
  * @return none
  */
__STATIC_INLINE void KS_ClearKeyscanStatus(KSCAN_T *ks,FunctionalState NewState)
{
    (NewState != DISABLE)?((ks)->KS_CFG |= KS_STATUS_CLEAR_Msk):((ks)->KS_CFG &= ~KS_STATUS_CLEAR_Msk);
}

/**
  * @brief  This function is used to set keyscan interval time between current frame to next
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  func: interval time select
  *                             \ref KSCAN_INTERVAL_CLK_FLAG
  * @return none
  */
__STATIC_INLINE void KS_SetFrameInterval(KSCAN_T *ks,uint32_t func)
{
    (ks)->KS_CFG = ((ks)->KS_CFG & ~KS_SCAN_INTERVAL_Msk) | (func << KS_SCAN_INTERVAL_Pos);
}

/**
  * @brief  This function is used to get keyscan interval time between current frame to next
  * @param  ks: where ks is a keyscan peripheral base address
  * @return none
  */
__STATIC_INLINE uint32_t KS_GetFrameInterval(KSCAN_T *ks)
{
    return (((ks)->KS_CFG & KS_SCAN_INTERVAL_Msk) >> KS_SCAN_INTERVAL_Pos);
}

/**
  * @brief  This function is used to set keyscan interval time between current row to next
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  func: interval time select
  *                             \ref KSCAN_INTERVAL_CLK_FLAG
  * @return none
  */
__STATIC_INLINE void KS_SetRowInterval(KSCAN_T *ks,uint32_t func)
{
    (ks)->KS_CFG = ((ks)->KS_CFG & ~KS_ROW_INTERVAL_Msk) | (func << KS_ROW_INTERVAL_Pos);
}
/**
  * @brief  This function is used to get keyscan interval time between current row to next
  * @param  ks: where ks is a keyscan peripheral base address
  * @return none
  */
__STATIC_INLINE uint32_t KS_GetRowInterval(KSCAN_T *ks)
{
    return (((ks)->KS_CFG & KS_ROW_INTERVAL_Msk) >> KS_ROW_INTERVAL_Pos);
}

/**
  * @brief  This function is used to set keyscan debounce time
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  func: interval time select
  *                             \ref KSCAN_INTERVAL_CLK_FLAG
  * @return none
  */
__STATIC_INLINE void KS_SetDebounceTime(KSCAN_T *ks,uint32_t func)
{
    (ks)->KS_CFG = ((ks)->KS_CFG & ~KS_DEBOUNCE_TIME_Msk) | (func << KS_DEBOUNCE_TIME_Pos);
}

/**
  * @brief  This function is used to get keyscan debounce time
  * @param  ks: where ks is a keyscan peripheral base address
  * @return none
  */
__STATIC_INLINE uint32_t KS_GetDebounceTime(KSCAN_T *ks)
{
    return (((ks)->KS_CFG & KS_DEBOUNCE_TIME_Msk) >> KS_DEBOUNCE_TIME_Pos);
}
/**
  * @brief  This function is used to enable keyscan delay function,if enabled
  *          isr will reported after scan finish.
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  enable: new state of the keyscan scan status
  * @return none
  */
__STATIC_INLINE void KS_WaitScanFinish(KSCAN_T *ks,bool enable)
{
    (enable) ? (ks->KS_EN |= KS_DLY_DISABLE_Msk) : (ks->KS_EN &= ~KS_DLY_DISABLE_Msk);
}
/**
  * @brief  This function is used to read key info
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  rowIdx: row number select (0~23)
  * @return row key infomation
  */
uint32_t KS_ReadKeyRowInfo(KSCAN_T *ks,uint8_t rowIdx);

/**
  * @brief  This function is used to read key info
  * @param  ks: where ks is a keyscan peripheral base address
  * @param  rowIdx: row number select (0~23)
  * @param  colIdx: column number select (0~7)
  * @return one key state
  */
bool KS_IsKeyPressed(KSCAN_T *ks,uint8_t rowIdx,uint8_t colIdx);

/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_KSCAN_H__ */
