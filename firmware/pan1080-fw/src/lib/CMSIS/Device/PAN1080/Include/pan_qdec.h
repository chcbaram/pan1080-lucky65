/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series qdec driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/19 18:33 $ 
 */

#ifndef __PAN_QDEC_H__
#define __PAN_QDEC_H__
/**
 * @brief Qdec Interface
 * @defgroup qdec_interface Qdec Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif


#define QDEC_FUNC_ALL              (0XFF)   /*!< Enable all qdec function */
#define QDEC_FUNC_ALL_EXCEPT_WK    (0XBF)   /*!< Enable all qdec function except wake up function */



/**@defgroup QDEC_POLARITY_FLAG Qdec polarity type 
 * @brief       Qdec polarity type definitions
 * @{ */
#define QDEC_POLARITY_HIGH            (1)   /*!< Polarity high */
#define QDEC_POLARITY_LOW             (0)   /*!< Polarity low */
/**@} */

/**@defgroup QDEC_CHANNEL_FLAG Qdec polarity type 
 * @brief       Qdec channel counter definitions
 * @{ */
#define QDEC_CNT_IDX_X                (0)   /*!< Channel x counter */
#define QDEC_CNT_IDX_Y                (1)   /*!< Channel y counter */
#define QDEC_CNT_IDX_Z                (2)   /*!< Channel z counter */
/**@} */

/**@defgroup QDEC_CHANNEL_FLAG Qdec polarity type 
 * @brief       Qdec channel counter definitions
 * @{ */
#define QDEC_MAX_CNT_RESOLUTION       (3)   /*!< Max counter resolution */
#define QDEC_CNT_RESOLUTION_1X        (0)   /*!< b'00->1x resolution */
#define QDEC_CNT_RESOLUTION_2X        (1)   /*!< b'01->2x resolution */
#define QDEC_CNT_RESOLUTION_4X        (2)   /*!< b'02->4x resolution */
/**@} */

/**@defgroup KSCAN_ENABLE_FLAG Kscan enable bit 
 * @brief       Kscan enable bit definitions
 * @{ */
#define QDEC_MAX_FILTER_NUM           (7)   /*!< Max filter count,unit is qdec sclk */
#define QDEC_FILTER_PERIOD_1          (0)   /*!< 1 clk period */
#define QDEC_FILTER_PERIOD_2          (1)   /*!< 2 clk period */
#define QDEC_FILTER_PERIOD_3          (2)   /*!< 3 clk period */
#define QDEC_FILTER_PERIOD_4          (3)   /*!< 4 clk period */
#define QDEC_FILTER_PERIOD_5          (4)   /*!< 5 clk period */
#define QDEC_FILTER_PERIOD_6          (5)   /*!< 6 clk period */
#define QDEC_FILTER_PERIOD_7          (6)   /*!< 7 clk period */
#define QDEC_FILTER_PERIOD_8          (7)   /*!< 8 clk period */
/**@} */

/**
  * @brief  This function is set qdec enable bit
  * @param  NewState: new state of the qdec peripheral
  * @param  func: function select
  *                        \ref QDEC_EVENT_EN_Msk
  *                        \ref QDEC_WAKEUP_EN_Msk   
  *                        \ref QDEC_INDEX_EN_Msk    
  *                        \ref QDEC_CHANNEL_Z_EN_Msk
  *                        \ref QDEC_CHANNEL_Y_EN_Msk
  *                        \ref QDEC_CHANNEL_X_EN_Msk
  *                        \ref QDEC_FILTER_EN_Msk   
  *                        \ref QDEC_EN_Msk            
  * @return none
  */
__STATIC_INLINE void QDEC_Enable(FunctionalState NewState,uint32_t func)
{
    (NewState != DISABLE)?(QDEC->QDEC_EN |= func):(QDEC->QDEC_EN &= ~func);
}

/**
  * @brief  This function adjust qdec interrupt occured or not
  * @param  intMsk: interrupt select 
  *                        \ref QDEC_INT_INDEX_Msk         
  *                        \ref QDEC_INT_FIFO_UNDERFLOW_Msk
  *                        \ref QDEC_INT_FIFO_OVERFLOW_Msk      
  *                        \ref QDEC_INT_INVALID_Msk       
  *                        \ref QDEC_INT_CNT_UNDERFLOW_Msk 
  *                        \ref QDEC_INT_CNT_OVERFLOW_Msk  
  * @return false or true
  */
__STATIC_INLINE bool QDEC_IsIntOccured(uint32_t intMsk)
{
    return ((QDEC->QDEC_INT_FLAG & intMsk) == intMsk)?(true):(false);
}

/**
  * @brief  This function clear qdec interrupt flag
  * @param  intMsk: interrupt select 
  *                        \ref QDEC_INT_INDEX_Msk         
  *                        \ref QDEC_INT_FIFO_UNDERFLOW_Msk
  *                        \ref QDEC_INT_FIFO_OVERFLOW_Msk      
  *                        \ref QDEC_INT_INVALID_Msk       
  *                        \ref QDEC_INT_CNT_UNDERFLOW_Msk 
  *                        \ref QDEC_INT_CNT_OVERFLOW_Msk  
  * @return none
  */
__STATIC_INLINE void QDEC_ClearIntFlag(uint32_t intMsk)
{
    QDEC->QDEC_INT_FLAG |= intMsk;
}

/**
  * @brief  This function adjust qdec raw interrupt occured or not
  * @param  intMsk: interrupt select 
  *                        \ref QDEC_INT_INDEX_Msk         
  *                        \ref QDEC_INT_FIFO_UNDERFLOW_Msk
  *                        \ref QDEC_INT_FIFO_OVERFLOW_Msk      
  *                        \ref QDEC_INT_INVALID_Msk       
  *                        \ref QDEC_INT_CNT_UNDERFLOW_Msk 
  *                        \ref QDEC_INT_CNT_OVERFLOW_Msk  
  * @return false or true
  */
__STATIC_INLINE bool QDEC_IsRawIntOccured(uint32_t intMsk)
{
    return ((QDEC->QDEC_INT_RAW & intMsk) == intMsk)?(true):(false);
}

/**
  * @brief  This function clear qdec raw interrupt flag
  * @param  intMsk: interrupt select 
  *                        \ref QDEC_INT_INDEX_Msk         
  *                        \ref QDEC_INT_FIFO_UNDERFLOW_Msk
  *                        \ref QDEC_INT_FIFO_OVERFLOW_Msk      
  *                        \ref QDEC_INT_INVALID_Msk       
  *                        \ref QDEC_INT_CNT_UNDERFLOW_Msk 
  *                        \ref QDEC_INT_CNT_OVERFLOW_Msk  
  * @return false or true
  */
__STATIC_INLINE void QDEC_ClearRawIntFlag(uint32_t intMsk)
{
    QDEC->QDEC_INT_RAW |= intMsk;
}

/**
  * @brief  This function used to set interrupt mask,
  *         if masked,qdec int flag will not be placed.
  * @param  intMsk: interrupt select 
  *                         \ref QDEC_INT_INDEX_Msk         
  *                         \ref QDEC_INT_FIFO_UNDERFLOW_Msk
  *                         \ref QDEC_INT_FIFO_OVERFLOW_Msk      
  *                         \ref QDEC_INT_INVALID_Msk       
  *                         \ref QDEC_INT_CNT_UNDERFLOW_Msk 
  *                         \ref QDEC_INT_CNT_OVERFLOW_Msk  
  * @param[in] NewState function enable or not. Including: ENABLE and DISABLE
  * @return none
  */
__STATIC_INLINE void QDEC_ClearIntMsk(uint32_t intMsk,FunctionalState NewState)
{
    (NewState)?(QDEC->QDEC_CTL |= intMsk):(QDEC->QDEC_CTL &= ~intMsk);
}


/**
  * @brief  This function used to set index polarity
  * @param[in] polarity: polarity select,including:
  *                         \ref QDEC_POLARITY_HIGH
  *                         \ref QDEC_POLARITY_LOW       
  * @return none
  */
__STATIC_INLINE void QDEC_SetPolarity(uint8_t polarity)
{
    (QDEC_POLARITY_HIGH == polarity)?(QDEC->QDEC_CTL |= QDEC_INDEX_POLARITY_Msk):(QDEC->QDEC_CTL &= ~QDEC_INDEX_POLARITY_Msk);
}


/**
  * @brief  This function used to get event count value
  * @param[in] idx: x,y,z count idx select,including:
  *                             \ref QDEC_CNT_IDX_X
  *                             \ref QDEC_CNT_IDX_Y
  *                             \ref QDEC_CNT_IDX_Z
  *                      
  * @return none
  */
__STATIC_INLINE uint32_t QDEC_GetEventCnt(uint8_t idx)
{
    return ((uint32_t*)(&QDEC->QDEC_X_CNT))[idx] & QDEC_CNT_VALUE_Msk;
}

/**
  * @brief  This function used to clear event count value
  * @param[in] idx: x,y,z count idx select,including:
  *                             \ref QDEC_CNT_IDX_X
  *                             \ref QDEC_CNT_IDX_Y
  *                             \ref QDEC_CNT_IDX_Z
  *                      
  * @return none
  */
__STATIC_INLINE uint32_t QDEC_ClearEventCnt(uint8_t idx)
{
    return ((uint32_t*)(&QDEC->QDEC_X_CNT))[idx] | QDEC_CNT_CLEAR_Msk;
}

/**
  * @brief  This function used to set count resolution
  * @param[in] resolution: count resolution value,including:
  *                         \ref QDEC_CNT_RESOLUTION_1X
  *                         \ref QDEC_CNT_RESOLUTION_2X
  *                         \ref QDEC_CNT_RESOLUTION_4X
  * @return none
  */
void QDEC_SetCntResolution(uint8_t resolution);

/**
  * @brief  This function used to set filter threshold
  * @param[in] threshold: filter threshold value,including:
  *                    \ref QDEC_FILTER_PERIOD_1
  *                    \ref QDEC_FILTER_PERIOD_2
  *                    \ref QDEC_FILTER_PERIOD_3
  *                    \ref QDEC_FILTER_PERIOD_4
  *                    \ref QDEC_FILTER_PERIOD_5
  *                    \ref QDEC_FILTER_PERIOD_6
  *                    \ref QDEC_FILTER_PERIOD_7
  *                    \ref QDEC_FILTER_PERIOD_8
  * @return none
  */
void QDEC_SetFilterThreshold(uint8_t threshold);

/**
  * @brief  This function used to set event threshold
  * @param[in] threshold: event threshold value     
  * @return none
  */
void QDEC_SetEventThreshold(uint16_t threshold);


/**@} */
#ifdef __cplusplus
}
#endif

#endif
