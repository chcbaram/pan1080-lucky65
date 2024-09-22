/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip uart adc driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */
#ifndef __PAN_UART_H__
#define __PAN_UART_H__

/**
 * @brief Uart Interface
 * @defgroup uart_interface Uart Interface
 * @{
 */
#ifdef __cplusplus
extern "C"
{
#endif



/**@defgroup UART_TX_TRIG_FLAG Uart transmit fifo threshold 
 * @brief       Uart transmit fifo threshold definition
 * @{ */
typedef enum UART_TxTrigger
{
    UART_TX_FIFO_EMPTY              = 0x0,
    UART_TX_FIFO_TWO_CHARS          = 0x1,
    UART_TX_FIFO_QUARTER_FULL       = 0x2,
    UART_TX_FIFO_HALF_FULL          = 0x3
} UART_TxTriggerDef;
/**@} */

/**@defgroup UART_RX_TRIG_FLAG Uart receive fifo threshold 
 * @brief       Uart receive fifo threshold definition
 * @{ */
typedef enum UART_RxTrigger
{
    UART_RX_FIFO_ONE_CHAR           = 0x0,
    UART_RX_FIFO_QUARTER_FULL       = 0x1,
    UART_RX_FIFO_HALF_FULL          = 0x2,
    UART_RX_FIFO_TWO_LESS_THAN_FULL = 0x3
} UART_RxTriggerDef;
/**@} */


/**@defgroup UART_LINE_CONTROL_FLAG Uart line control 
 * @brief       This is the data type used for manipulation of the UART line control settings
 * @{ */

typedef enum UART_LineCtrl
{
    Uart_Line_5n1       = 0x00,     /*!<  5 data bits, no parity, 1 stop bit */
    Uart_Line_5n1_5     = 0x04,     /*!<  5 data bits, no parity, 1.5 stop bits */
    Uart_Line_5e1       = 0x18,     /*!<  5 data bits, even parity, 1 stop bit */
    Uart_Line_5e1_5     = 0x1c,     /*!<  5 data bits, even parity, 1.5 stop bits */
    Uart_Line_5o1       = 0x08,     /*!<  5 data bits, odd parity, 1 stop bit */
    Uart_Line_5o1_5     = 0x0c,     /*!<  5 data bits, odd parity, 1.5 stop bits */
    Uart_Line_5s1       = 0x38,     /*!<  5 data bits, space parity, 1 stop bit */
    Uart_Line_5s1_5     = 0x3c,     /*!<  5 data bits, space parity, 1.5 stop bits */
    Uart_Line_5m1       = 0x28,     /*!<  5 data bits, mark parity, 1 stop bit */
    Uart_Line_5m1_5     = 0x2c,     /*!<  5 data bits, mark parity, 1.5 stop bits */
    Uart_Line_6n1       = 0x01,     /*!<  6 data bits, no parity, 1 stop bit */
    Uart_Line_6n2       = 0x05,     /*!<  6 data bits, no parity, 2 stop bits */
    Uart_Line_6e1       = 0x19,     /*!<  6 data bits, even parity, 1 stop bit */
    Uart_Line_6e2       = 0x1d,     /*!<  6 data bits, even parity, 2 stop bits */
    Uart_Line_6o1       = 0x09,     /*!<  6 data bits, odd parity, 1 stop bit */
    Uart_Line_6o2       = 0x0d,     /*!<  6 data bits, odd parity, 2 stop bits */
    Uart_Line_6s1       = 0x39,     /*!<  6 data bits, space parity, 1 stop bit */
    Uart_Line_6s2       = 0x3d,     /*!<  6 data bits, space parity, 2 stop bits */
    Uart_Line_6m1       = 0x29,     /*!<  6 data bits, mark parity, 1 stop bit */
    Uart_Line_6m2       = 0x2d,     /*!<  6 data bits, mark parity, 2 stop bits */
    Uart_Line_7n1       = 0x02,     /*!<  7 data bits, no parity, 1 stop bit */
    Uart_Line_7n2       = 0x06,     /*!<  7 data bits, no parity, 2 stop bits */
    Uart_Line_7e1       = 0x1a,     /*!<  7 data bits, even parity, 1 stop bit */
    Uart_Line_7e2       = 0x1e,     /*!<  7 data bits, even parity, 2 stop bits */
    Uart_Line_7o1       = 0x0a,     /*!<  7 data bits, odd parity, 1 stop bit */
    Uart_Line_7o2       = 0x0e,     /*!<  7 data bits, odd parity, 2 stop bits */
    Uart_Line_7s1       = 0x3a,     /*!<  7 data bits, space parity, 1 stop bit */
    Uart_Line_7s2       = 0x3e,     /*!<  7 data bits, space parity, 2 stop bits */
    Uart_Line_7m1       = 0x2a,     /*!<  7 data bits, mark parity, 1 stop bit */
    Uart_Line_7m2       = 0x2e,     /*!<  7 data bits, mark parity, 2 stop bits */
    Uart_Line_8n1       = 0x03,     /*!<  8 data bits, no parity, 1 stop bit */
    Uart_Line_8n2       = 0x07,     /*!<  8 data bits, no parity, 2 stop bits */
    Uart_Line_8e1       = 0x1b,     /*!<  8 data bits, even parity, 1 stop bit */
    Uart_Line_8e2       = 0x1f,     /*!<  8 data bits, even parity, 2 stop bits */
    Uart_Line_8o1       = 0x0b,     /*!<  8 data bits, odd parity, 1 stop bit */
    Uart_Line_8o2       = 0x0f,     /*!<  8 data bits, odd parity, 2 stop bits */
    Uart_Line_8s1       = 0x3b,     /*!<  8 data bits, space parity, 1 stop bit */
    Uart_Line_8s2       = 0x3f,     /*!<  8 data bits, space parity, 2 stop bits */
    Uart_Line_8m1       = 0x2b,     /*!<  8 data bits, mark parity, 1 stop bit */
    Uart_Line_8m2       = 0x2f,     /*!<  8 data bits, mark parity, 2 stop bits */
} UART_LineCtrlDef;
/**@} */

/**@defgroup UART_INT_FLAG Uart interrupts status 
 * @brief    These are the bit definitions used for managing UART interrupts.
 *  The functionality of ETBEI and ERBFI alters when programmable THRE
 *  interrupt mode is active (dw_uart_enablePtime()).  See the
 *  DW_apb_uart databook for a detailed description.
 * @{ */

typedef enum UART_Irq
{
    UART_IRQ_RECV_DATA_AVL  = 0x01,     /*!< receive data available */
    UART_IRQ_THR_EMPTY      = 0x02,     /*!< transmitter holding register empty */
    UART_IRQ_LINE_STATUS    = 0x04,     /*!< receiver line status */
    UART_IRQ_MODEM_STATUS   = 0x08,     /*!< modem status */
    UART_IRQ_ALL            = 0x0f      /*!< all interrupts */
} UART_IrqDef;
/**@} */


/**@defgroup UART_EVENT_FLAG Uart event mode
 * @brief    This is the data type used for specifying UART events.  An event is
 *  the occurrence of an interrupt which must be handled appropriately.
 *  One of these events is passed at a time to the user listener
 *  function to be processed.  The exception to this are the
 *  UART_EVENT_THR_EMPTY and UART_EVENT_TIMEOUT interrupts which are handled
 *  internally by the interrupt handler.
 * @{ */

typedef enum UART_Event
{
    UART_EVENT_MODEM        = 0x0,  /*!< CTR, DSR, RI or DCD status changed. */
    UART_EVENT_NONE         = 0x1,  /*!< No event/interrupt pending. */
    UART_EVENT_THR_EMPTY    = 0x2,  /*!< Transmit holding register empty or TX FIFO at or below trigger threshold. */
    UART_EVENT_DATA         = 0x4,  /*!< Receive buffer register data available (non-FIFO mode) or RX FIFO trigger level reached. */
    UART_EVENT_LINE         = 0x6,  /*!< Overrun/parity/framing error or break interrupt occurred. */
#if 0
    UART_EVENT_BUSY         = 0x7,  /*!< Attempt to write to the LCR[7] while DW_apb_uart was busy (DLAB). */
#endif
    UART_EVENT_TIMEOUT      = 0xc   /*!< No characters in or out of the receiver FIFO during the last four character times. */
} UART_EventDef;
/**@} */


/**@defgroup TIMER_LINE_STATUS_FLAG Uart line status
 * @brief    These are the definitions used for reporting the line status
 *  including errors, if any.  Note that they represent bits of an 8-bit
 *  register and more than one can be active at any time.
 * @{ */

typedef enum UART_LineStatus
{
    UART_LINE_DATA_RDY          = 0x01,     /*!< data ready */
    UART_LINE_OVERRUN_ERR       = 0x02,     /*!< overrun error */
    UART_LINE_PARITY_ERR        = 0x04,     /*!< parity error */
    UART_LINE_FRAME_ERR         = 0x08,     /*!< framing error */
    UART_LINE_BREAK_INT         = 0x10,     /*!< break interrupt */
    UART_LINE_THRE              = 0x20,     /*!< transmit holding register empty (or tx fifo full) */
    UART_LINE_TXSR_EMPTY        = 0x40,     /*!< transmitter shift register empty */
    UART_LINE_RECV_FIFO_EMPTY   = 0x80,     /*!< receiver FIFO error */
    UART_LINE_ADDR_RCVD         = 0x100     /*!< address received (9-bit mode) */
} UART_LineStatusDef;
/**@} */

/**@defgroup TIMER_MODEM_STATUS_FLAG Uart modem status
 * @brief    Uart modem status definitions
 * @{ */
typedef enum UART_ModemStatus
{
    UART_MODEM_DCTS     = 0x01,     /*!< delta clear to send */
    UART_MODEM_DDSR     = 0x02,     /*!< delta data set ready */
    UART_MODEM_TERI     = 0x04,     /*!< trailing edge of ring indicator */
    UART_MODEM_DDCD     = 0x08,     /*!< delta data carrier detect */
    UART_MODEM_CTS      = 0x10,     /*!< clear to send */
    UART_MODEM_DSR      = 0x20,     /*!< data set ready */
    UART_MODEM_RI       = 0x40,     /*!< ring indicator */
    UART_MODEM_DCD      = 0x80      /*!< data carrier detect */
} UART_ModemStatusDef;
/**@} */


/** 
 * @brief Structure with uart init feature.
 *
 * @param UART_BaudRate 	set initial baudrate.
 * @param UART_LineCtrl		set initial line control parameter. \ref UART_LineCtrlDef
 */  
typedef struct
{
    uint32_t UART_BaudRate;             // set initial baudrate
    UART_LineCtrlDef UART_LineCtrl;     // set initial line control parameter
} UART_InitTypeDef;




/**
  * @brief  This function is used to enable stick parity
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_EnableStickParity(UART_T* UARTx)
{
    UARTx->LCR |= UART_LCR_SP_Msk;
}

/**
  * @brief  This function is used to disable stick parity
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_DisableStickParity(UART_T* UARTx)
{
    UARTx->LCR &= ~UART_LCR_SP_Msk;
}

/**
  * @brief  This function is used to get line status
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE UART_LineStatusDef UART_GetLineStatus(UART_T* UARTx)
{
    return (UART_LineStatusDef)(UARTx->LSR & UART_LSR_LINE_STATUS_Msk);
}

__STATIC_INLINE UART_ModemStatusDef UART_GetModemStatus(UART_T* UARTx)
{
    return (UART_ModemStatusDef)(UARTx->MSR & UART_MSR_MODEM_STATUS_Msk);
}

/**
 * @brief	 This function enables receive and transmit FIFOs, if they are
 *  available.
 * @param  UARTx: where x can be 1, 2 to select the
 *         UART peripheral.
 * @return None
 */
__STATIC_INLINE void UART_EnableFifo(UART_T* UARTx)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR |= UART_FCR_FIFOE_Msk;

    UARTx->IIR_FCR = UARTx->SCR;
}

/**
 * @brief	 This function disables receive and transmit FIFOs.
 * @param  UARTx: where x can be 1, 2 to select the
 *         UART peripheral.
 * @return None
 */

__STATIC_INLINE void UART_DisableFifo(UART_T* UARTx)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR &= ~UART_FCR_FIFOE_Msk;

    UARTx->IIR_FCR = UARTx->SCR;
}

/**
 * @brief	 Adjust whether the FIFOs enabled or not.
 * @param  UARTx: where x can be 1, 2 to select the
 *         UART peripheral.
 * @return None
 */
__STATIC_INLINE bool UART_IsFifoEnabled(UART_T* UARTx)
{
    return (UARTx->IIR_FCR & UART_IIR_FIFOSE_Msk) ? true : false;
}

/**
	* @brief  This function is used to reset rx fifo
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @return None
	*/
__STATIC_INLINE void UART_ResetRxFifo(UART_T* UARTx)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR |= UART_FCR_RFIFOR_Msk;

    UARTx->IIR_FCR = UARTx->SCR;
    UARTx->SCR &= ~UART_FCR_RFIFOR_Msk; //Clear RFIFOR bit in SCR after use

    // Read Rx FIFO once again to de-assert rx interrupt if there is
    volatile uint32_t val = UARTx->RBR_THR_DLL;
    val = val;
}

/**
	* @brief  This function is used to reset tx fifo
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @return None
	*/
__STATIC_INLINE void UART_ResetTxFifo(UART_T* UARTx)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR |= UART_FCR_XFIFOR_Msk;

    UARTx->IIR_FCR = UARTx->SCR;
    UARTx->SCR &= ~UART_FCR_XFIFOR_Msk; //Clear XFIFOR bit in SCR after use
}

/**
	* @brief	Set the trigger level of the transmitter FIFO empty interrupt.
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @param  level: Uart transmit fifo threshold \ref UART_TxTriggerDef
	* @return None
	*/
__STATIC_INLINE void UART_SetTxTrigger(UART_T* UARTx, UART_TxTriggerDef level)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR &= ~UART_FCR_TET_Msk;
    UARTx->SCR |= (level << UART_FCR_TET_Pos);

    UARTx->IIR_FCR = UARTx->SCR;
}


/**
	* @brief	Sets the trigger level of the receiver FIFO full interrupt.
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @param  level: Uart receive fifo threshold \ref UART_RxTriggerDef
	* @return None
	*/
__STATIC_INLINE void UART_SetRxTrigger(UART_T* UARTx, UART_RxTriggerDef level)
{
    // SCR(r/w) stores a copy of FCR(wo)
    UARTx->SCR &= ~UART_FCR_RT_Msk;
    UARTx->SCR |= (level << UART_FCR_RT_Pos);

    UARTx->IIR_FCR = UARTx->SCR;
}

/**
	* @brief	Get the transmit fifo level.
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @return how many bytes deep the transmitter and receiver FIFOs are
	*/
__STATIC_INLINE uint8_t UART_GetTxFifoLevel(UART_T* UARTx)
{
    return (uint8_t)((UARTx->TFL & UART_TFL_Msk) >> UART_TFL_Pos);
}

/**
	* @brief	Get the receive fifo level.
	* @param  UARTx: where x can be 1, 2 to select the
	*         UART peripheral.
	* @return the number of characters currently present in the Rx fifo.
	*/
__STATIC_INLINE uint8_t UART_GetRxFifoLevel(UART_T* UARTx)
{
     return (uint8_t)((UARTx->RFL & UART_RFL_Msk) >> UART_RFL_Pos);
}

/**
  * @brief  Enable auto flow control mode
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_EnableAfc(UART_T *UARTx)
{
    UARTx->MCR |= UART_MCR_AFCE_Msk;
}

/**
  * @brief  Disable auto flow control mode
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_DisableAfc(UART_T *UARTx)
{
    UARTx->MCR &= ~UART_MCR_AFCE_Msk;
}

/**
  * @brief  Check auto flow control mode enable or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE bool UART_IsAfcEnabled(UART_T *UARTx)
{
    return (UARTx->MCR & UART_MCR_AFCE_Msk) ? true : false;
}

/**
  * @brief  Uart send data function
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  Data: data need to seed
  * @return None
  */
__STATIC_INLINE void UART_SendData(UART_T* UARTx, uint8_t Data)
{
    UARTx->RBR_THR_DLL = Data;
}

/**
  * @brief  Uart receive data function
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return received data
  */
__STATIC_INLINE uint8_t UART_ReceiveData(UART_T* UARTx)
{
    return (uint8_t)UARTx->RBR_THR_DLL;
}

/**
  * @brief  Programmable THRE interrupt mode enable
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return none
  */
__STATIC_INLINE void UART_EnablePtime(UART_T* UARTx)
{
    UARTx->IER_DLH |= UART_IER_EPTI_Msk;
}

/**
  * @brief  Programmable THRE interrupt mode disable
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return none
  */
__STATIC_INLINE void UART_DisablePtime(UART_T* UARTx)
{
    UARTx->IER_DLH &= ~UART_IER_EPTI_Msk;
}

/**
  * @brief  Check programmable THRE interrupt mode enable or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return none
  */
__STATIC_INLINE bool UART_IsPtimeEnabled(UART_T* UARTx)
{
    return (UARTx->IER_DLH & UART_IER_EPTI_Msk) ? true : false;
}

/**
  * @brief  Enable interrupt,include Modem status linestatus,THRE,receive data
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  irq: interrupt type,see \ref UART_IrqDef
  * @return none
  */
__STATIC_INLINE void UART_EnableIrq(UART_T* UARTx, UART_IrqDef irq)
{
    UARTx->IER_DLH |= ((irq << UART_IER_ALL_IRQ_Pos) & UART_IER_ALL_IRQ_Msk);
}

/**
  * @brief  Disable interrupt
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  irq: interrupt type,see \ref UART_IrqDef
  * @return none
  */
__STATIC_INLINE void UART_DisableIrq(UART_T* UARTx, UART_IrqDef irq)
{
    UARTx->IER_DLH &= ~((irq << UART_IER_ALL_IRQ_Pos) & UART_IER_ALL_IRQ_Msk);
}

/**
  * @brief  Check interrupt enable or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  irq: interrupt type,see \ref UART_IrqDef
  * @return none
  */
__STATIC_INLINE bool UART_IsIrqEnabled(UART_T* UARTx, UART_IrqDef irq)
{
    return (UARTx->IER_DLH & ((irq << UART_IER_ALL_IRQ_Pos) & UART_IER_ALL_IRQ_Msk)) ? true : false;
}

/**
  * @brief  Get which interrupt is actived 
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return interrupt type
  */
__STATIC_INLINE uint8_t UART_GetIrqMasked(UART_T* UARTx)
{
    return (uint8_t)((UARTx->IER_DLH & UART_IER_ALL_IRQ_Msk) >> UART_IER_ALL_IRQ_Pos);
}

/**
  * @brief  Get active interrupt ID 
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return interrupt ID
  */
__STATIC_INLINE UART_EventDef UART_GetActiveEvent(UART_T * UARTx)
{
    return (UART_EventDef)((UARTx->IIR_FCR & UART_IIR_IID_Msk) >> UART_IIR_IID_Pos);
}

/**
  * @brief  Check tx fifo empty or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE bool UART_IsTxFifoEmpty(UART_T* UARTx)
{
    return (UARTx->USR & UART_USR_TFE_Msk) ? true : false;
}

/**
  * @brief  Check tx fifo full or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE bool UART_IsTxFifoFull(UART_T* UARTx)
{
    return (UARTx->USR & UART_USR_TFNF_Msk) ? false : true;
}

/**
  * @brief  Check rx fifo empty or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE bool UART_IsRxFifoEmpty(UART_T* UARTx)
{
    return (UARTx->USR & UART_USR_RFNE_Msk) ? false : true;
}

/**
  * @brief  Check rx fifo full or not
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE bool UART_IsRxFifoFull(UART_T* UARTx)
{
    return (UARTx->USR & UART_USR_RFF_Msk) ? true : false;
}

/**
  * @brief  Enable halt transmit,this function is used to halt transmit for test
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE void UART_EnableHaltTX(UART_T* UARTx)
{
    UARTx->HTX = 0x1;
}

/**
  * @brief  Disable halt transmit
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return true or false
  */
__STATIC_INLINE void UART_DisableHaltTX(UART_T* UARTx)
{
    UARTx->HTX = 0x0;
}

/**
  * @brief  Enable request to send in flow control mode
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_EnableRts(UART_T* UARTx)
{
    UARTx->MCR |= UART_MCR_RTS_Msk;
}

/**
  * @brief  Disable request to send in flow control mode
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_DisableRts(UART_T* UARTx)
{
    UARTx->MCR &= ~UART_MCR_RTS_Msk;
}

/**
  * @brief  Enable 9-bit data transmit and receive transfer
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_Enable9BitData( UART_T* UARTx)
{
    UARTx->LCR_EXT |= UART_LCR_EXT_DLS_E_Msk;
}

/**
  * @brief  Disable 9-bit data transmit and receive transfer
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_Disable9BitData( UART_T* UARTx)
{
    UARTx->LCR_EXT &= ~UART_LCR_EXT_DLS_E_Msk;
}

/**
  * @brief  Enable address match feature during receive
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_EnableAddrMatchMode(UART_T* UARTx)
{
   UARTx->LCR_EXT |= UART_LCR_EXT_ADDR_MATCH_Msk;
}

/**
  * @brief  Disable address match feature during receive
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_DisableAddrMatchMode(UART_T* UARTx)
{
   UARTx->LCR_EXT &= ~UART_LCR_EXT_ADDR_MATCH_Msk;
}

/**
  * @brief  UART Send Address,9-bit character will be transmitted with 9-th bit set to 1 
  * and the remaining 8-bits will match to what is being programmed in "Transmit Address Register"
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
__STATIC_INLINE void UART_SendAddr(UART_T* UARTx)
{
   UARTx->LCR_EXT |= UART_LCR_EXT_SEND_ADDR_Msk;
}

/**
  * @brief  This is an address matching register during receive mode,Set Receive Address
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  addr: Receive Address
  * @return None
  */
__STATIC_INLINE void UART_SetReceiveAddress( UART_T* UARTx,uint32_t addr)
{
    UARTx->RAR = addr;
}

/**
  * @brief  This is an address matching register during transmit mode,Set transmit address
  * @param  UARTx: where x can be 1, 2 to select the UART peripheral.
  * @param  addr: Transmit address
  * @return None
  */
__STATIC_INLINE void UART_SetTransmitAddress( UART_T* UARTx,uint32_t addr)
{
    UARTx->TAR = addr;
}

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct .
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @return UART init success (true) or fail (false)
  * @code
  *
  *	UART_InitTypeDef Init_Struct;
	*
	*	Init_Struct.UART_BaudRate = 115200;
  *	Init_Struct.UART_LineCtrl = Uart_Line_8n1;
	*
  *	UART_Init(UART1, &Init_Struct);
  *	UART_EnableFifo(UART1);
  *
  * @endcode
  */
extern bool UART_Init(UART_T* UARTx, UART_InitTypeDef* UART_InitStruct);
/**
  * @brief  Deinit the UARTx peripheral
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @return None
  */
extern void UART_DeInit(UART_T* UARTx );

/**
  * @brief  This function is used to send multiple data
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  Buf: pointer to send data storage location. 
  * @param  BufSize: used to indicate the offset of buf.
  * @param  SendSize: used to indicate the size need to send.     
  * @return Send success (true) or fail (false)
  */
extern bool UART_SendMultipleData(UART_T* UARTx, uint8_t* Buf, size_t BufSize, size_t SendSize);

/**
  * @brief  This function is used to receive multiple data
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  Buf: pointer to receive data storage location 
  * @param  BufSize: used to indicate the offset of buf
  * @param  ExpectSize: used to indicate the size expect to receive  
  * @return Receive success (true) or fail (false)
  */
extern bool UART_ReceiveMultipleData(UART_T* UARTx, uint8_t* Buf, size_t BufSize, size_t ExpectSize);


/**@} */

#ifdef __cplusplus
}
#endif

#endif /* __PAN_UART_H__ */









