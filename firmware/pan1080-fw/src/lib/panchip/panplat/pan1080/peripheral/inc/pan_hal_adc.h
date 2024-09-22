/*********************************************************
 * @file     pan_hal_adc.h
 * @version  V0.0.0
 * $Revision: 1 $
 * $Date:    23/09/10 $
 * @brief    Panchip series ADC (Analog-to-Digital Converter) HAL (Hardware Abstraction Layer) header file.
 *
 * @note
 * Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __PAN_HAL_ADC_H
#define __PAN_HAL_ADC_H

#include "pan_hal_def.h"

/**
 * @brief ADC HAL Interface
 * @defgroup adc_hal_interface Adc HAL Interface
 * @{
 */

/**
 * @brief ADC operation mode options.
 *
 * This enumeration defines the possible operation modes for the ADC.
 */
typedef enum
{
    ADC_MODE_CONV,       /*!< Regular ADC conversion mode. */
    ADC_MODE_EXTRIG,     /*!< External trigger mode for ADC. */
    ADC_MODE_COMPARE,    /*!< ADC comparison mode. */
    ADC_MODE_PWMSEQ,     /*!< ADC PWM sequencing mode. */
    ADC_MODE_SGLCONVSEQ, /*!< Single ADC conversion in a sequence. */
} ADC_Mode_Opt;

/**
 * @brief ADC conversion mode options.
 *
 * This enumeration defines the possible conversion modes for the ADC.
 */
typedef enum
{
    ADC_CONV_MODE_BASE, /*!< Basic ADC conversion mode. */
    ADC_CONV_MODE_TEMP, /*!< Temperature sensor conversion mode. */
    ADC_CONV_MODE_VBG,  /*!< Voltage BandGap conversion mode. */
    ADC_CONV_MODE_VDD4, /*!< Quarter VDD conversion mode. */
} ADC_Convert_Mode_Opt;

/**
 * @brief ADC channel identifiers.
 *
 * This enumeration defines the identifiers for ADC channels.
 */
typedef enum
{
    ADC_CH0 = 0x0, /*!< ADC channel 0. */
    ADC_CH1 = 0x1, /*!< ADC channel 1. */
    ADC_CH2 = 0x2, /*!< ADC channel 2. */
    ADC_CH3 = 0x3, /*!< ADC channel 3. */
    ADC_CH4 = 0x4, /*!< ADC channel 4. */
    ADC_CH5 = 0x5, /*!< ADC channel 5. */
    ADC_CH6 = 0x6, /*!< ADC channel 6. */
    ADC_CH7 = 0x7, /*!< ADC channel 7. */
    // ADC_CH8     = 0x8,  /*!< ADC channel 8. */
    // ADC_CH9     = 0x9,  /*!< ADC channel 9. */
    // ADC_CH10    = 0xA,  /*!< ADC channel 10. */
    ADC_CH11 = 0xB, /*!< ADC channel 11. */

    ADC_CH_VBG = 0x8,   /*!< Voltage BandGap channel. Note: Duplicates the channel number of ADC_CH8. */
    ADC_CH_TEMP = 0x9,  /*!< Temperature sensor channel. Note: Duplicates the channel number of ADC_CH9. */
    ADC_CH_VDD_4 = 0xA, /*!< Quarter VDD channel. Note: Duplicates the channel number of ADC_CH10. */
} ADC_Ch_Id;

/**
 * @brief ADC sampling clock options.
 *
 * This enumeration defines the various sampling time options for ADC conversion,
 * represented in terms of ADC clock cycles.
 */
typedef enum
{
    ADC_SAMPLE_CLK_0 = 0UL,    /*!< ADC sample time is 0 ADC clock */
    ADC_SAMPLE_CLK_1 = 1UL,    /*!< ADC sample time is 1 ADC clock */
    ADC_SAMPLE_CLK_2 = 2UL,    /*!< ADC sample time is 2 ADC clock */
    ADC_SAMPLE_CLK_4 = 3UL,    /*!< ADC sample time is 4 ADC clock */
    ADC_SAMPLE_CLK_8 = 4UL,    /*!< ADC sample time is 8 ADC clock */
    ADC_SAMPLE_CLK_16 = 5UL,   /*!< ADC sample time is 16 ADC clock */
    ADC_SAMPLE_CLK_32 = 6UL,   /*!< ADC sample time is 32 ADC clock */
    ADC_SAMPLE_CLK_64 = 7UL,   /*!< ADC sample time is 64 ADC clock */
    ADC_SAMPLE_CLK_128 = 8UL,  /*!< ADC sample time is 128 ADC clock */
    ADC_SAMPLE_CLK_256 = 9UL,  /*!< ADC sample time is 256 ADC clock */
    ADC_SAMPLE_CLK_512 = 10UL, /*!< ADC sample time is 512 ADC clock */
    ADC_SAMPLE_CLK_1024 = 11UL /*!< ADC sample time is 1024 ADC clock */
} ADC_SampleClock_Opt;

/**
 * @brief ADC input range options.
 *
 * This enumeration defines the input voltage range options available for ADC
 * conversion.
 */
typedef enum
{
    ADC_INPUT_RANGE_LOW = 0UL, /*!< ADC input range 0.4V~1.4V. */
    ADC_INPUT_RANGE_HIGH = 1UL /*!< ADC input range 0.4V~2.4V. */
} ADC_InputRange_Opt;

/**
 * @brief ADC initialization options structure.
 *
 * This structure defines the parameters required for the initialization of
 * the ADC. It includes configuration options like sample clock, input range,
 * mode of operation, and hardware trigger source.
 */
typedef struct
{
    ADC_SampleClock_Opt SampleClock; /*!< Defines the sample clock duration for the ADC. */
    ADC_InputRange_Opt InputRange;   /*!< Defines the input voltage range for ADC conversion. */
    ADC_Mode_Opt Mode;               /*!< Defines the ADC mode of operation. */
} ADC_Init_Opt;

/**
 * @brief ADC Handle Structure definition.
 *
 * This structure holds the configuration and runtime information for an ADC (Analog-to-Digital Converter) instance.
 */
typedef struct
{
    uint32_t *pRxBuffPtr;      /**< Pointer to ADC Rx transfer buffer. */
    uint16_t RxXferSize;       /**< Size of the ADC Rx transfer buffer. */
    __IO uint16_t RxXferCount; /**< Counter for tracking ADC Rx transfers. */
} ADC_HandleTypeDef;

extern ADC_HandleTypeDef ADC_Handle_Array[1]; /**< Array of ADC handle structures. */
#define ADC_OBJ ADC_Handle_Array[0]           /**< Reference to the ADC handle object for single ADC. */

/**
 * @brief Initialize the ADC for a specific channel with the given configuration.
 *
 * @param ChID     The ADC channel ID to initialize.
 * @param InitObj  ADC initialization options.
 */
void HAL_ADC_Init(ADC_Ch_Id ChID, ADC_Init_Opt InitObj);

/**
 * @brief DeInitialize the ADC.
 *
 * @note To be implemented: Handle deinitialization if necessary.
 *
 * @note This function powers down the ADC.
 */
void HAL_ADC_DeInit(void);

/**
 * @brief Start the conversion process for a specific ADC channel.
 *
 * @param ChID  The ADC channel ID to start the conversion for.
 */
void HAL_ADC_Convert(ADC_Ch_Id ChID);

/**
 * @brief Start DMA-based ADC data transfer to a memory buffer.
 *
 * @param Buf       Pointer to the destination memory buffer.
 * @param Size      The size of the data transfer.
 * @param Callback  Callback function to be executed upon DMA completion.
 *
 * @return The DMA channel number used for the transfer.
 *
 * @note This function initializes and starts DMA transfer for ADC data (Ref: DMA_Init).
 */
uint32_t HAL_ADC_GetValue(uint16_t Cnt);

/**
 * @brief Start DMA-based ADC data transfer to a memory buffer.
 *
 * @param Buf       Pointer to the destination memory buffer.
 * @param Size      The size of the data transfer.
 * @param Callback  Callback function to be executed upon DMA completion.
 *
 * @return The DMA channel number used for the transfer.
 *
 * @note This function initializes and starts DMA transfer for ADC data (Ref: DMA_Init).
 */
uint32_t HAL_ADC_Start_DMA(uint32_t *Buf, uint16_t Size, ADC_CallbackFunc Callback);
/** @} */ // end of group
#endif    /* __PAN_HAL_ADC_H */
