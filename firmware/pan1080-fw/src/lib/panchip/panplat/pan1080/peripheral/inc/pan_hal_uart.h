/**************************************************************************//**
* @file     pan_hal_uart.h
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series UART (Universal Asynchronous Receiver-Transmitter) HAL header file.
* @note     Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __PAN_HAL_UART_H__
#define __PAN_HAL_UART_H__

#include "pan_hal_def.h"

/**
 * @brief UART HAL Interface
 * @defgroup uart_hal_interface Uart HAL Interface
 * @{
 */

typedef enum 
{
    HAL_UART_CB_TX_EMPTY       = 0UL, /**< @brief Callback when the transmit buffer is empty. */
    HAL_UART_CB_TX_FULL,              /**< @brief Callback when the transmit buffer is full. */
    HAL_UART_CB_LINE_ERROR,           /**< @brief Callback for receiver line errors. */
    HAL_UART_CB_TX_FINISH,            /**< @brief Callback when the transmit operation is finished. */
    HAL_UART_CB_MODEM,                /**< @brief Callback for modem status. */
    HAL_UART_CB_MAX                   /**< @brief Sentinel value. */
} UART_Callback_Type_Opt;

typedef enum
{
    HAL_UART_INT_RECV_DATA_AVL  = 0x01, /**< @brief Receive data available interrupt. */
    HAL_UART_INT_THR_EMPTY      = 0x02, /**< @brief Transmitter holding register empty interrupt. */
    HAL_UART_INT_LINE_STATUS    = 0x04, /**< @brief Receiver line status interrupt. */
    HAL_UART_INT_MODEM_STATUS   = 0x08, /**< @brief Modem status interrupt. */
    HAL_UART_INT_ALL            = 0x0f, /**< @brief All interrupts. */
    HAL_UART_INT_DISABLE = 0x00 /**< @brief Disable interrupts. */
} UART_Interrupt_Mode_Opt;

// Configure UART software buffer size
#define UART_RX_BUF_SIZE        256
#define UART_TX_BUF_SIZE        256

extern uint8_t uartRxBuffer[UART_RX_BUF_SIZE];
extern uint8_t uartTxBuffer[UART_TX_BUF_SIZE];
extern volatile uint32_t uartRxBufIdx;
extern uint32_t uartTxBufIdx;
extern volatile bool uartTxDone;
extern volatile bool uartAddrRcvd;
extern volatile bool uartRxIntFlag;

/**
 * @brief UART Format Options Enum Definition.
 *
 * This enumeration lists the possible combinations of data bits, parity bits,
 * and stop bits for UART communication.
 */
typedef enum 
{
    HAL_UART_FMT_5_N_1       = 0x00,     // 5 data bits, no parity, 1 stop bit
    HAL_UART_FMT_5_N_1_5     = 0x04,     // 5 data bits, no parity, 1.5 stop bits
    HAL_UART_FMT_5_E_1       = 0x18,     // 5 data bits, even parity, 1 stop bit
    HAL_UART_FMT_5_E_1_5     = 0x1c,     // 5 data bits, even parity, 1.5 stop bits
    HAL_UART_FMT_5_O_1       = 0x08,     // 5 data bits, odd parity, 1 stop bit
    HAL_UART_FMT_5_O_1_5     = 0x0c,     // 5 data bits, odd parity, 1.5 stop bits
    HAL_UART_FMT_5_S_1       = 0x38,     // 5 data bits, space parity, 1 stop bit
    HAL_UART_FMT_5_S_1_5     = 0x3c,     // 5 data bits, space parity, 1.5 stop bits
    HAL_UART_FMT_5_M_1       = 0x28,     // 5 data bits, mark parity, 1 stop bit
    HAL_UART_FMT_5_M_1_5     = 0x2c,     // 5 data bits, mark parity, 1.5 stop bits
    HAL_UART_FMT_6_N_1       = 0x01,     // 6 data bits, no parity, 1 stop bit
    HAL_UART_FMT_6_N_2       = 0x05,     // 6 data bits, no parity, 2 stop bits
    HAL_UART_FMT_6_E_1       = 0x19,     // 6 data bits, even parity, 1 stop bit
    HAL_UART_FMT_6_E_2       = 0x1d,     // 6 data bits, even parity, 2 stop bits
    HAL_UART_FMT_6_O_1       = 0x09,     // 6 data bits, odd parity, 1 stop bit
    HAL_UART_FMT_6_O_2       = 0x0d,     // 6 data bits, odd parity, 2 stop bits
    HAL_UART_FMT_6_S_1       = 0x39,     // 6 data bits, space parity, 1 stop bit
    HAL_UART_FMT_6_S_2       = 0x3d,     // 6 data bits, space parity, 2 stop bits
    HAL_UART_FMT_6_M_1       = 0x29,     // 6 data bits, mark parity, 1 stop bit
    HAL_UART_FMT_6_M_2       = 0x2d,     // 6 data bits, mark parity, 2 stop bits
    HAL_UART_FMT_7_N_1       = 0x02,     // 7 data bits, no parity, 1 stop bit
    HAL_UART_FMT_7_N_2       = 0x06,     // 7 data bits, no parity, 2 stop bits
    HAL_UART_FMT_7_E_1       = 0x1a,     // 7 data bits, even parity, 1 stop bit
    HAL_UART_FMT_7_E_2       = 0x1e,     // 7 data bits, even parity, 2 stop bits
    HAL_UART_FMT_7_O_1       = 0x0a,     // 7 data bits, odd parity, 1 stop bit
    HAL_UART_FMT_7_O_2       = 0x0e,     // 7 data bits, odd parity, 2 stop bits
    HAL_UART_FMT_7_S_1       = 0x3a,     // 7 data bits, space parity, 1 stop bit
    HAL_UART_FMT_7_S_2       = 0x3e,     // 7 data bits, space parity, 2 stop bits
    HAL_UART_FMT_7_M_1       = 0x2a,     // 7 data bits, mark parity, 1 stop bit
    HAL_UART_FMT_7_M_2       = 0x2e,     // 7 data bits, mark parity, 2 stop bits
    HAL_UART_FMT_8_N_1       = 0x03,     // 8 data bits, no parity, 1 stop bit
    HAL_UART_FMT_8_N_2       = 0x07,     // 8 data bits, no parity, 2 stop bits
    HAL_UART_FMT_8_E_1       = 0x1b,     // 8 data bits, even parity, 1 stop bit
    HAL_UART_FMT_8_E_2       = 0x1f,     // 8 data bits, even parity, 2 stop bits
    HAL_UART_FMT_8_O_1       = 0x0b,     // 8 data bits, odd parity, 1 stop bit
    HAL_UART_FMT_8_O_2       = 0x0f,     // 8 data bits, odd parity, 2 stop bits
    HAL_UART_FMT_8_S_1       = 0x3b,     // 8 data bits, space parity, 1 stop bit
    HAL_UART_FMT_8_S_2       = 0x3f,     // 8 data bits, space parity, 2 stop bits
    HAL_UART_FMT_8_M_1       = 0x2b,     // 8 data bits, mark parity, 1 stop bit
    HAL_UART_FMT_8_M_2       = 0x2f,     // 8 data bits, mark parity, 2 stop bits
} UART_Format_Opt;

/**
 * @brief  UART Init Structure definition
 */
typedef struct
{
  uint32_t BaudRate;      /**< @brief Set the initial baud rate. */
  UART_Format_Opt Format; /**< @brief Set the initial line control parameters. */
} UART_Init_Opt;

/**
 * @brief UART Interrupt Options Structure Definition.
 *
 * This structure contains configuration and control information for
 * managing the interrupt behavior of a UART peripheral.
 */
typedef struct
{
    bool Switch;                      /**< @brief Enables or disables the interrupt. True to enable, false to disable. */
    UART_Interrupt_Mode_Opt Mode;     /**< @brief Specifies the interrupt mode or triggering condition. */
    UART_Callback_Type_Opt CallbackTyp; /**< @brief Specifies the type or category of the callback function. */
    UART_CallbackFunc CallbackFunc;   /**< @brief Pointer to the callback function to be executed when the specified interrupt event occurs. */
} UART_Interrupt_Opt;

/**
 * @brief  UART handle Structure definition
 */
typedef struct
{
    UART_T *UARTx; /*!< UART registers base address. */

    UART_Init_Opt InitObj; /*!< UART communication parameters. */

    UART_Interrupt_Opt InterruptObj;

    uint8_t *pTxBuffPtr; /*!< Pointer to UART Tx transfer Buffer. */
    uint16_t TxXferSize; /*!< UART Tx Transfer size. */
    __IO uint16_t TxXferCount; /*!< UART Tx Transfer Counter. */

    uint8_t *pRxBuffPtr; /*!< Pointer to UART Rx transfer Buffer. */
    uint16_t RxXferSize; /*!< UART Rx Transfer size. */
    __IO uint16_t RxXferCount; /*!< UART Rx Transfer Counter. */

    IRQn_Type IRQn;                  /**< IRQ number associated with the UART. */

    UART_CallbackFunc RxIntCallback; /**< Callback function for UART receive interrupts. */
    UART_CallbackFunc TxIntCallback; /**< Callback function for UART transmit interrupts. */

    uint32_t DmaSrc;                 /**< DMA source channel for UART data transfer. */
    uint32_t DmaDst;                 /**< DMA destination channel for UART data transfer. */
    
    __IO uint32_t ErrorCode;         /**< UART error code, indicating any encountered errors. */
} UART_HandleTypeDef;

#define UART0_OBJ UART_Handle_Array[0]    /**< UART handle for UART0. */
#define UART1_OBJ UART_Handle_Array[1]    /**< UART handle for UART1. */

extern UART_HandleTypeDef UART_Handle_Array[2];  /**< Array of UART handles for multiple UART instances. */

/**
 * @brief Initialize the UART peripheral.
 *
 * This function initializes the UART peripheral with the specified baud rate and format.
 * It configures the UART's line control parameters, including data bits, stop bits, and parity.
 *
 * @param[in] uart Pointer to the UART handle structure.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool HAL_UART_Init(UART_HandleTypeDef *uart);

/**
 * @brief Send data using UART.
 *
 * This function sends data over UART without interrupts. It waits until the UART's
 * transmitter holding register is empty before sending each byte.
 *
 * @param[in] uart Pointer to the UART handle structure.
 * @param[in] Buf  Pointer to the data buffer to be transmitted.
 * @param[in] Size Size of the data to be transmitted.
 */
void HAL_UART_SendData(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size);

/**
 * @brief Receive data using UART.
 *
 * This function receives data over UART without interrupts. It continuously polls
 * the UART's data register and stores received bytes in the specified buffer.
 *
 * @param[in] uart     Pointer to the UART handle structure.
 * @param[in] Buf      Pointer to the data buffer to store received data.
 * @param[in] Size     Size of the data to be received.
 * @param[in] Timeout  Timeout value for receiving data.
 */
void HAL_UART_ReceiveData(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, uint32_t Timeout);

/**
 * @brief Transmit data using UART with interrupt.
 *
 * This function initializes the UART transmission with interrupt and sets up the required parameters.
 *
 * @param[in] uart     Pointer to the UART handle structure.
 * @param[in] Buf      Pointer to the data buffer to be transmitted.
 * @param[in] Size     Size of the data to be transmitted.
 * @param[in] Callback Callback function to be called when the transmission is complete.
 */
void HAL_UART_SendData_INT(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, UART_CallbackFunc Callback);

/**
 * @brief Receive data using UART with interrupt.
 *
 * This function initializes the UART reception with interrupt and sets up the required parameters.
 *
 * @param[in] uart     Pointer to the UART handle structure.
 * @param[in] Buf      Pointer to the data buffer to store received data.
 * @param[in] Size     Size of the data to be received.
 * @param[in] Timeout  Timeout value for receiving data.
 * @param[in] Callback Callback function to be called when data is received or a timeout occurs.
 */
void HAL_UART_ReceiveData_INT(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size, uint32_t Timeout,UART_CallbackFunc Callback);

/**
 * @brief Transmit data using UART with DMA.
 *
 * This function initializes UART transmission with DMA and sets up the required parameters.
 *
 * @param[in] uart     Pointer to the UART handle structure.
 * @param[in] Buf      Pointer to the data buffer to be transmitted.
 * @param[in] Size     Size of the data to be transmitted.
 * @param[in] Callback Callback function to be called when the transmission is complete.
 */
void HAL_UART_SendData_DMA(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size,UART_CallbackFunc Callback);

/**
 * @brief Receive data using UART with DMA.
 *
 * This function initializes UART reception with DMA and sets up the required parameters.
 *
 * @param[in] uart     Pointer to the UART handle structure.
 * @param[in] Buf      Pointer to the data buffer to store received data.
 * @param[in] Size     Size of the data to be received.
 * @param[in] Timeout  Timeout value for receiving data.
 * @param[in] Callback Callback function to be called when data is received.
 */
void HAL_UART_ReceiveData_DMA(UART_HandleTypeDef *uart, uint8_t *Buf, size_t Size,  uint32_t Timeout,UART_CallbackFunc Callback);

/**
 * @brief Initialize UART interrupt settings.
 *
 * This function configures the UART interrupt settings based on the provided
 * UART handle and the settings specified in the InterruptObj structure.
 *
 * @param[in] uart Pointer to the UART handle structure.
 */
void HAL_UART_Init_INT(UART_HandleTypeDef *uart);

/**
 * @brief Interrupt handler for UART0.
 *
 * This function is called when an interrupt occurs for UART0. It processes
 * the interrupt by calling the UART_HandleProc function for UART0.
 */
void UART0_IRQHandler(void);

/**
 * @brief Interrupt handler for UART1.
 *
 * This function is called when an interrupt occurs for UART1. It processes
 * the interrupt by calling the UART_HandleProc function for UART1.
 */
void UART1_IRQHandler(void);

extern void HAL_DelayMs(uint32_t t);

/** @} */ // end of group 
#endif
