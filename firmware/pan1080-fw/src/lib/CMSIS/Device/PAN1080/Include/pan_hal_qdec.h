/******************************************************************************
* @file     pan_hal_qdec.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series Quadrature Decoder (QDEC) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PAN_HAL_QDEC_H__
#define __PAN_HAL_QDEC_H__
#include "pan_hal_def.h"

/**
 * @brief QDEC HAL Interface
 * @defgroup qdec_hal_interface Qdec HAL Interface
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Enumeration of QDEC resolutions.
 */
typedef enum
{
    QDEC_RESOLUTION_1X = QDEC_CNT_RESOLUTION_1X, /**< 1X resolution. */
    QDEC_RESOLUTION_2X = QDEC_CNT_RESOLUTION_2X, /**< 2X resolution. */
    QDEC_RESOLUTION_4X = QDEC_CNT_RESOLUTION_4X, /**< 4X resolution. */
    QDEC_MAX_RESOLUTION = QDEC_MAX_CNT_RESOLUTION   /**< Maximum resolution. */
} QDEC_Resolution_Opt;

/**
 * @brief Enumeration of QDEC index options.
 */
typedef enum
{
    QDEC_IDX_X = QDEC_CNT_IDX_X, /**< X index option. */
    QDEC_IDX_Y = QDEC_CNT_IDX_Y, /**< Y index option. */
    QDEC_IDX_Z = QDEC_CNT_IDX_Z  /**< Z index option. */
} QDEC_Index_Opt;

/**
 * @brief Enumeration of QDEC interrupt modes.
 */
typedef enum
{
    QDEC_INT_WAKEUP = (0x1ul << QDEC_INT_WAKEUP_Pos),           /**< Wakeup interrupt mode. */
    QDEC_INT_INDEX = (0x1ul << QDEC_INT_INDEX_Pos),             /**< Index interrupt mode. */
    QDEC_INT_FIFO_UNDERFLOW = (0x1ul << QDEC_INT_FIFO_UNDERFLOW_Pos), /**< FIFO underflow interrupt mode. */
    QDEC_INT_FIFO_OVERFLOW = (0x1ul << QDEC_INT_FIFO_OVERFLOW_Pos), /**< FIFO overflow interrupt mode. */
    QDEC_INT_INVALID = (0x1ul << QDEC_INT_INVALID_Pos),         /**< Invalid interrupt mode. */
    QDEC_INT_CNT_UNDERFLOW = (0x1ul << QDEC_INT_CNT_UNDERFLOW_Pos), /**< Counter underflow interrupt mode. */
    QDEC_INT_CNT_OVERFLOW = (0x1ul << QDEC_INT_CNT_OVERFLOW_Pos)  /**< Counter overflow interrupt mode. */
} QDEC_INT_Mode_Opt;

/**
 * @brief Enumeration of QDEC callback types.
 */
typedef enum
{
    QDEC_CB_INDEX = 0X01,     /**< Index callback type. */
    QDEC_CB_OVERFLOW = 0X02,  /**< Overflow callback type. */
    QDEC_CB_UNDERFLOW = 0X03  /**< Underflow callback type. */
} QDEC_Callback_Type_Opt;

/**
 * @brief Function pointer type for QDEC callbacks.
 */
typedef void (*QDEC_CallbackFunc)(QDEC_Callback_Type_Opt, int16_t);

/**
 * @brief Enumeration of QDEC filter periods.
 */
typedef enum
{
    QDEC_FILTR_PER_1 = QDEC_FILTER_PERIOD_1, /**< Filter period 1. */
    QDEC_FILTR_PER_2 = QDEC_FILTER_PERIOD_2, /**< Filter period 2. */
    QDEC_FILTR_PER_3 = QDEC_FILTER_PERIOD_3, /**< Filter period 3. */
    QDEC_FILTR_PER_4 = QDEC_FILTER_PERIOD_4, /**< Filter period 4. */
    QDEC_FILTR_PER_5 = QDEC_FILTER_PERIOD_5, /**< Filter period 5. */
    QDEC_FILTR_PER_6 = QDEC_FILTER_PERIOD_6, /**< Filter period 6. */
    QDEC_FILTR_PER_7 = QDEC_FILTER_PERIOD_7, /**< Filter period 7. */
    QDEC_FILTR_PER_8 = QDEC_FILTER_PERIOD_8  /**< Filter period 8. */
} QDEC_FilterPeriod_Opt;

/**
 * @brief Enumeration of QDEC operating modes.
 */
typedef enum
{
    QDEC_MODE_ALL = QDEC_FUNC_ALL,                     /**< All QDEC functions enabled. */
    QDEC_MODE_EVENT = (0x1ul << QDEC_EVENT_EN_Pos),   /**< Event mode enabled. */
    QDEC_MODE_WK = (0x1ul << QDEC_WAKEUP_EN_Pos),     /**< Wakeup mode enabled. */
    QDEC_MODE_INDEX = (0x1ul << QDEC_INDEX_EN_Pos)    /**< Index mode enabled. */
} QDEC_Mode_Opt;

/**
 * @brief Enumeration of QDEC polarity options.
 */
typedef enum
{
    QDEC_POL_LOW = QDEC_POLARITY_LOW,   /**< Low polarity. */
    QDEC_POL_HIGH = QDEC_POLARITY_HIGH  /**< High polarity. */
} QDEC_Polarity_Opt;

/**
 * @brief Structure for QDEC interrupt configuration.
 */
typedef struct
{
    QDEC_INT_Mode_Opt Mode; /**< QDEC interrupt mode. */
} QDEC_Interrupt_Opt;

/**
 * @brief Structure for QDEC initialization parameters.
 */
typedef struct
{
    QDEC_Polarity_Opt Polarity;     /**< QDEC polarity option. */
    QDEC_Resolution_Opt Resolution; /**< QDEC resolution option. */
    QDEC_FilterPeriod_Opt FilterPeriod; /**< QDEC filter period option. */
    uint16_t EventThrs;           /**< Event threshold value. */
    QDEC_Mode_Opt Mode;           /**< QDEC operating mode. */
    QDEC_Index_Opt CH;            /**< QDEC index option. */
} QDEC_Init_Opt;

/**
 * @brief Structure for QDEC handle.
 */
typedef struct
{
    QDEC_Init_Opt InitObj;        /**< QDEC initialization parameters. */
    QDEC_Interrupt_Opt InterruptObj; /**< QDEC interrupt configuration. */
    QDEC_CallbackFunc Callback;   /**< QDEC callback function. */
    // int16_t Cnt;   
    // int16_t xCnt;
    // int16_t yCnt;
    // int16_t zCnt;
} QDEC_HandleTypeDef;

/**
 * @brief Array of QDEC handles.
 */
extern QDEC_HandleTypeDef QDEC_Handle_Array[1];

/**
 * @brief QDEC object.
 *
 * Global QDEC object for simplified usage. Typically, you can access the QDEC
 * functionality through this object.
 */
#define QDEC_OBJ QDEC_Handle_Array[0]

/**
  * @brief  Initializes the QDEC peripheral with the specified configuration.
  * @param  qdec: Pointer to a QDEC_HandleTypeDef structure that contains the configuration information for the QDEC peripheral.
  * @retval None
  */
void HAL_QDEC_Init(QDEC_HandleTypeDef *qdec);

/**
  * @brief  Initializes the QDEC peripheral with interrupt support.
  * @param  qdec: Pointer to a QDEC_HandleTypeDef structure that contains the configuration information for the QDEC peripheral.
  * @retval None
  */
void HAL_QDEC_Init_INT(QDEC_HandleTypeDef *qdec);
#ifdef __cplusplus
}
#endif

/** @} */ // end of group

#endif // __PAN_HAL_QDEC_H__
