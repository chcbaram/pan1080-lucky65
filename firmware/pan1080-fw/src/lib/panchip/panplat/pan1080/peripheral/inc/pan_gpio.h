/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series gpio driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
 
#ifndef __PAN_GPIO_H__
#define __PAN_GPIO_H__

/**
 * @brief Gpio Interface
 * @defgroup gpio_interface Gpio Interface
 * @{
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


#define GPIO_PIN_MAX    8   /*!< Specify Maximum Pins of Each GPIO Port */

/**@defgroup GPIO_MODE_FLAG Gpio mode
 * @brief       Gpio mode definitions
 * @{ */
 
typedef enum _GPIO_ModeDef
{
    GPIO_MODE_INPUT         = 0x0,  /*!< Input Mode */
    GPIO_MODE_OUTPUT        = 0x1,  /*!< Output Mode */
    GPIO_MODE_OPEN_DRAIN    = 0x2,  /*!< Open-Drain Mode */
    GPIO_MODE_QUASI         = 0x3   /*!< Quasi-bidirectional Mode */
} GPIO_ModeDef;
/**@} */

/**@defgroup GPIO_INT_FLAG Gpio interrupt type
 * @brief       Gpio interrupt type definitions
 * @{ */

typedef enum _GPIO_IntAttrDef
{
    GPIO_INT_RISING         = 0x00010000UL, /*!< Interrupt enable by Input Rising Edge */
    GPIO_INT_FALLING        = 0x00000001UL, /*!< Interrupt enable by Input Falling Edge */
    GPIO_INT_BOTH_EDGE      = 0x00010001UL, /*!< Interrupt enable by both Rising Edge and Falling Edge */
    GPIO_INT_HIGH           = 0x01010000UL, /*!< Interrupt enable by Level-High */    
    GPIO_INT_LOW            = 0x01000001UL  /*!< Interrupt enable by Level-Low */
} GPIO_IntAttrDef;

/**@} */

/**@defgroup GPIO_DB_SRC_FLAG Gpio debounce src
 * @brief       Gpio debouce src definitions
 * @{ */

typedef enum _GPIO_ClkSrcDef
{
    GPIO_DBCTL_DBCLKSRC_RCL     = 0x00000010UL, /*!< DBCTL setting for de-bounce counter clock source is the internal 32 kHz */ 
    GPIO_DBCTL_DBCLKSRC_HCLK    = 0x00000000UL  /*!< DBCTL setting for de-bounce counter clock source is the internal HCLK */ 
} GPIO_ClkSrcDef;
/**@} */


/**@defgroup GPIO_DB_CNT_FLAG Gpio debounce count
 * @brief       Gpio debouce count definitions
 * @{ */

typedef enum _GPIO_ClkSelDef
{
    GPIO_DBCTL_DBCLKSEL_1       = 0x00000000UL, /*!< DBCTL setting for sampling cycle = 1 clocks */
    GPIO_DBCTL_DBCLKSEL_2       = 0x00000001UL, /*!< DBCTL setting for sampling cycle = 2 clocks */
    GPIO_DBCTL_DBCLKSEL_4       = 0x00000002UL, /*!< DBCTL setting for sampling cycle = 4 clocks */
    GPIO_DBCTL_DBCLKSEL_8       = 0x00000003UL, /*!< DBCTL setting for sampling cycle = 8 clocks */
    GPIO_DBCTL_DBCLKSEL_16      = 0x00000004UL, /*!< DBCTL setting for sampling cycle = 16 clocks */
    GPIO_DBCTL_DBCLKSEL_32      = 0x00000005UL, /*!< DBCTL setting for sampling cycle = 32 clocks */
    GPIO_DBCTL_DBCLKSEL_64      = 0x00000006UL, /*!< DBCTL setting for sampling cycle = 64 clocks */
    GPIO_DBCTL_DBCLKSEL_128     = 0x00000007UL, /*!< DBCTL setting for sampling cycle = 128 clocks */
    GPIO_DBCTL_DBCLKSEL_256     = 0x00000008UL, /*!< DBCTL setting for sampling cycle = 256 clocks */
    GPIO_DBCTL_DBCLKSEL_512     = 0x00000009UL, /*!< DBCTL setting for sampling cycle = 512 clocks */
    GPIO_DBCTL_DBCLKSEL_1024    = 0x0000000AUL, /*!< DBCTL setting for sampling cycle = 1024 clocks */
    GPIO_DBCTL_DBCLKSEL_2048    = 0x0000000BUL, /*!< DBCTL setting for sampling cycle = 2048 clocks */
    GPIO_DBCTL_DBCLKSEL_4096    = 0x0000000CUL, /*!< DBCTL setting for sampling cycle = 4096 clocks */
    GPIO_DBCTL_DBCLKSEL_8192    = 0x0000000DUL, /*!< DBCTL setting for sampling cycle = 8192 clocks */
    GPIO_DBCTL_DBCLKSEL_16384   = 0x0000000EUL, /*!< DBCTL setting for sampling cycle = 16384 clocks */
    GPIO_DBCTL_DBCLKSEL_32768   = 0x0000000FUL  /*!< DBCTL setting for sampling cycle = 32768 clocks */
} GPIO_ClkSelDef;
/**@} */


/**@defgroup GPIO_ADDR_FLAG Gpio address map
 * @{ */

/** 
 * @brief       Define gpio pin Data input/output. It could be used to control each \n
 *							- I/O pin by pin address mapping.
 *
 * @param[in]   Port GPIO port number. It could be \ref 0, \ref 1, \ref 2, \ref 3, \ref 4 or \ref 5.
 * @param[in]   Pin  The single or multiple pins of specified gpio port. \ref 0, \ref 1, \ref 2,.. \ref 7
 *
 * @code:
 *      P00 = 1; 
 *  
 * @endcode
 * @details		It is used to set P0.0 to high;
 *
 * @code:
 *  
 *      if (P00)
 *          P00 = 0;
 * @endcode
 * @details		If P0.0 pin status is high, then set P0.0 data output to low.
 */
#define GPIO_PIN_ADDR(Port, Pin)   (*((volatile uint32_t *)((GPIOBIT0_BASE+(0x20*(Port))) + ((Pin)<<2)))) 
#define P00             GPIO_PIN_ADDR(0, 0) /*!< Specify P00 Pin Data Input/Output */
#define P01             GPIO_PIN_ADDR(0, 1) /*!< Specify P01 Pin Data Input/Output */
#define P02             GPIO_PIN_ADDR(0, 2) /*!< Specify P02 Pin Data Input/Output */
#define P03             GPIO_PIN_ADDR(0, 3) /*!< Specify P03 Pin Data Input/Output */
#define P04             GPIO_PIN_ADDR(0, 4) /*!< Specify P04 Pin Data Input/Output */
#define P05             GPIO_PIN_ADDR(0, 5) /*!< Specify P05 Pin Data Input/Output */
#define P06             GPIO_PIN_ADDR(0, 6) /*!< Specify P06 Pin Data Input/Output */
#define P07             GPIO_PIN_ADDR(0, 7) /*!< Specify P07 Pin Data Input/Output */
#define P10             GPIO_PIN_ADDR(1, 0) /*!< Specify P10 Pin Data Input/Output */
#define P11             GPIO_PIN_ADDR(1, 1) /*!< Specify P11 Pin Data Input/Output */
#define P12             GPIO_PIN_ADDR(1, 2) /*!< Specify P12 Pin Data Input/Output */
#define P13             GPIO_PIN_ADDR(1, 3) /*!< Specify P13 Pin Data Input/Output */
#define P14             GPIO_PIN_ADDR(1, 4) /*!< Specify P14 Pin Data Input/Output */
#define P15             GPIO_PIN_ADDR(1, 5) /*!< Specify P15 Pin Data Input/Output */
#define P16             GPIO_PIN_ADDR(1, 6) /*!< Specify P16 Pin Data Input/Output */
#define P17             GPIO_PIN_ADDR(1, 7) /*!< Specify P17 Pin Data Input/Output */
#define P20             GPIO_PIN_ADDR(2, 0) /*!< Specify P20 Pin Data Input/Output */
#define P21             GPIO_PIN_ADDR(2, 1) /*!< Specify P21 Pin Data Input/Output */
#define P22             GPIO_PIN_ADDR(2, 2) /*!< Specify P22 Pin Data Input/Output */
#define P23             GPIO_PIN_ADDR(2, 3) /*!< Specify P23 Pin Data Input/Output */
#define P24             GPIO_PIN_ADDR(2, 4) /*!< Specify P24 Pin Data Input/Output */
#define P25             GPIO_PIN_ADDR(2, 5) /*!< Specify P25 Pin Data Input/Output */
#define P26             GPIO_PIN_ADDR(2, 6) /*!< Specify P26 Pin Data Input/Output */
#define P27             GPIO_PIN_ADDR(2, 7) /*!< Specify P27 Pin Data Input/Output */
#define P30             GPIO_PIN_ADDR(3, 0) /*!< Specify P30 Pin Data Input/Output */
#define P31             GPIO_PIN_ADDR(3, 1) /*!< Specify P31 Pin Data Input/Output */
#define P32             GPIO_PIN_ADDR(3, 2) /*!< Specify P32 Pin Data Input/Output */
#define P33             GPIO_PIN_ADDR(3, 3) /*!< Specify P33 Pin Data Input/Output */
#define P34             GPIO_PIN_ADDR(3, 4) /*!< Specify P34 Pin Data Input/Output */
#define P35             GPIO_PIN_ADDR(3, 5) /*!< Specify P35 Pin Data Input/Output */
#define P36             GPIO_PIN_ADDR(3, 6) /*!< Specify P36 Pin Data Input/Output */
#define P37             GPIO_PIN_ADDR(3, 7) /*!< Specify P37 Pin Data Input/Output */
#define P40             GPIO_PIN_ADDR(4, 0) /*!< Specify P40 Pin Data Input/Output */
#define P41             GPIO_PIN_ADDR(4, 1) /*!< Specify P41 Pin Data Input/Output */
#define P42             GPIO_PIN_ADDR(4, 2) /*!< Specify P42 Pin Data Input/Output */
#define P43             GPIO_PIN_ADDR(4, 3) /*!< Specify P43 Pin Data Input/Output */
#define P44             GPIO_PIN_ADDR(4, 4) /*!< Specify P44 Pin Data Input/Output */
#define P45             GPIO_PIN_ADDR(4, 5) /*!< Specify P45 Pin Data Input/Output */
#define P46             GPIO_PIN_ADDR(4, 6) /*!< Specify P46 Pin Data Input/Output */
#define P47             GPIO_PIN_ADDR(4, 7) /*!< Specify P47 Pin Data Input/Output */
#define P50             GPIO_PIN_ADDR(5, 0) /*!< Specify P50 Pin Data Input/Output */
#define P51             GPIO_PIN_ADDR(5, 1) /*!< Specify P51 Pin Data Input/Output */
#define P52             GPIO_PIN_ADDR(5, 2) /*!< Specify P52 Pin Data Input/Output */
#define P53             GPIO_PIN_ADDR(5, 3) /*!< Specify P53 Pin Data Input/Output */
#define P54             GPIO_PIN_ADDR(5, 4) /*!< Specify P54 Pin Data Input/Output */
#define P55             GPIO_PIN_ADDR(5, 5) /*!< Specify P55 Pin Data Input/Output */
#define P56             GPIO_PIN_ADDR(5, 6) /*!< Specify P56 Pin Data Input/Output */
#define P57             GPIO_PIN_ADDR(5, 7) /*!< Specify P57 Pin Data Input/Output */
/**@} */


/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_ClrIntFlag(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->INTSRC = PinMask;
}

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio	GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 *
 * @return      None
 *
 * @details     Clear the interrupt status of All GPIO pin.
 */
 
__STATIC_INLINE void GPIO_ClrAllIntFlag(GPIO_T *gpio)
{
    gpio->INTSRC = gpio->INTSRC;
}

/**
 * @brief       Disable Pin De-bounce Function
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_DisableDebounce(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DBEN &= ~PinMask;
}

/**
 * @brief       Enable Pin De-bounce Function
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_EnableDebounce(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DBEN |= PinMask;
}

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_DisableDigitalPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF |= (PinMask << 16);
}

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_EnableDigitalPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF &= ~(PinMask << 16);
}

/**
 * @brief       Disable I/O Digital pull up Path
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O Digital pull up Path of specified GPIO pin.
 */
 __STATIC_INLINE void GPIO_DisablePullupPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF &= ~PinMask;
}

 /**
 * @brief       Enable I/O Digital pull up Path
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O Digital pull up Path of specified GPIO pin.
 */
 __STATIC_INLINE void GPIO_EnablePullupPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF |= PinMask;
}

/**
 * @brief       Disable I/O Digital pull down Path
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O Digital pull down Path of specified GPIO pin.
 */
 __STATIC_INLINE void GPIO_DisablePulldownPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF &= ~(PinMask << 8);
}

 /**
 * @brief       Enable I/O Digital pull down Path
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O Digital pull down Path of specified GPIO pin.
 */
 __STATIC_INLINE void GPIO_EnablePulldownPath(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DINOFF |= (PinMask << 8);
}

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_DisableDoutMask(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DATMSK &= ~PinMask;
}

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 */
__STATIC_INLINE void GPIO_EnableDoutMask(GPIO_T *gpio, uint32_t PinMask)
{
    gpio->DATMSK |= PinMask;
}

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio			GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  	The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @retval      false			No interrupt at specified GPIO pin
 * @retval      true			The specified GPIO pin generate an interrupt
 *
 * @details     Get the interrupt status of specified GPIO pin.
 */
__STATIC_INLINE bool GPIO_GetIntFlag(GPIO_T *gpio, uint32_t PinMask)
{
    return (bool)(gpio->INTSRC & PinMask);
}

/**
 * @brief       Set De-bounce Sampling Cycle Time
 *
 * @param[in]   clksrc      The de-bounce counter clock source. It could be GPIO_DBCTL_DBCLKSRC_HCLK \n
 *													- or GPIO_DBCTL_DBCLKSRC_RCL.
 * @param[in]   clksel      The de-bounce sampling cycle selection. It could be \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_1, \ref GPIO_DBCTL_DBCLKSEL_2, \ref GPIO_DBCTL_DBCLKSEL_4, \ref GPIO_DBCTL_DBCLKSEL_8, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_16, \ref GPIO_DBCTL_DBCLKSEL_32, \ref GPIO_DBCTL_DBCLKSEL_64, \ref GPIO_DBCTL_DBCLKSEL_128, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_256, \ref GPIO_DBCTL_DBCLKSEL_512, \ref GPIO_DBCTL_DBCLKSEL_1024, \ref GPIO_DBCTL_DBCLKSEL_2048, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_4096, \ref GPIO_DBCTL_DBCLKSEL_8192, \ref GPIO_DBCTL_DBCLKSEL_16384, \ref GPIO_DBCTL_DBCLKSEL_32768.
 *
 * @return      None
 *
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_RCL, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 10 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (2^4)*(1/(10*1000)) s = 16*0.0001 s = 1600 us,
 *              and system will sampling interrupt input once per 1600 us.
 */
__STATIC_INLINE void GPIO_SetDebounceTime(GPIO_ClkSrcDef ClkSrc, GPIO_ClkSelDef ClkSel)
{
    GPIO_DB->DBCTL = (GPIO_DB->DBCTL & BIT31) | ClkSrc | ClkSel;
}

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   gpio		GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 *
 * @return      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 */
__STATIC_INLINE uint32_t GPIO_GetInData(GPIO_T *gpio)
{
    return gpio->PIN;
}

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   gpio	GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   data	GPIO port data.
 *
 * @retval      None
 *
 * @details     Set the Data into specified GPIO port.
 */
__STATIC_INLINE void GPIO_SetOutData(GPIO_T *gpio, uint32_t data)
{
    gpio->DOUT = data;
}

/**
 * @brief       Get GPIO Port OUT Data
 *
 * @param[in]   gpio		GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 *
 * @return      The specified port data
 *
 * @details     Get the DOUT register of specified GPIO port.
 */
__STATIC_INLINE uint32_t GPIO_GetOutData(GPIO_T *gpio)
{
    return gpio->DOUT;
}

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @retval      None
 *
 * @details     Toggle the specified GPIO pin output.
 */
__STATIC_INLINE void GPIO_Toggle(GPIO_T *gpio, uint32_t u32PinMask)
{
    GPIO_SetOutData(gpio, GPIO_GetOutData(gpio) ^ u32PinMask);
}

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   u32Pin       Pxy
 *
 * @note This API only takes effect when the Digital Input Path
 *       of current pin is enabled.
 *
 * @retval      None
 *
 * @details     Toggle the specified GPIO pin output.
 */
__STATIC_INLINE void GPIO_TogglePin(uint32_t u32Pin)
{
    u32Pin ^= 1;
}

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   gpio         GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin          The pin of specified GPIO port. It could be 0 ~ 7.
 * @param[in]   IntAttribs      The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING,
 *                              - \ref GPIO_INT_FALLING,
 *                              - \ref GPIO_INT_BOTH_EDGE,
 *                              - \ref GPIO_INT_HIGH,
 *                              - \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
__STATIC_INLINE void GPIO_EnableInt(GPIO_T *gpio, uint32_t Pin, GPIO_IntAttrDef IntAttribs)
{
    gpio->INTTYPE |= (((IntAttribs >> 24) & 0xFFUL) << Pin);
    gpio->INTEN = (gpio->INTEN & ~(0x00010001UL << Pin)) | ((IntAttribs & 0xFFFFFFUL) << Pin);
}

/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   gpio     GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
__STATIC_INLINE void GPIO_DisableInt(GPIO_T *gpio, uint32_t Pin)
{
    gpio->INTTYPE &= ~(1UL << Pin);
    gpio->INTEN &= ~((0x00010001UL) << Pin);
}

/**
 * @brief       Enable External GPIO interrupt 0
 *
 * @param[in]   gpio         GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin          The pin of specified GPIO port.
 * @param[in]   IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING, \ref GPIO_INT_FALLING, \ref GPIO_INT_BOTH_EDGE, \ref GPIO_INT_HIGH, \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT0    GPIO_EnableInt


/**
 * @brief       Disable External GPIO interrupt 0
 *
 * @param[in]   gpio     GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT0   GPIO_DisableInt


/**
 * @brief       Enable External GPIO interrupt 1
 *
 * @param[in]   gpio            GPIO port. It could \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin          The pin of specified GPIO port.
 * @param[in]   IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING, \ref GPIO_INT_FALLING, \ref GPIO_INT_BOTH_EDGE, \ref GPIO_INT_HIGH, \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT1    GPIO_EnableInt


/**
 * @brief       Disable External GPIO interrupt 1
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   Pin      		The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT1   GPIO_DisableInt


/**
 * @brief       Set GPIO Work Mode
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   PinMask  		The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7.
 * @param[in]   Mode        Gpio mode definitions (see @ref GPIO_ModeDef).
 *
 * @retval      None
 *
 * @details     Set the GPIO pin work mode.
 */
extern void GPIO_SetMode(GPIO_T *gpio, uint32_t PinMask, GPIO_ModeDef Mode);

/**@} */


#ifdef __cplusplus
}
#endif

#endif /* __PAN_GPIO_H__ */

