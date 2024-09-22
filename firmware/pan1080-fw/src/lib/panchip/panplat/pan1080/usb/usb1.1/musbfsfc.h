/**************************************************************************//**
 * @file     musbfsfc.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/08/04 14:25 $
 * @brief    Header file for MUSBFSFC firmware
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef _MUSBFSFC_H_
#define _MUSBFSFC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
// #include "PANSeries.h"

/* Interrupt register bit masks */
#define M_INTR_SUSPEND                          0x01
#define M_INTR_RESUME                           0x02
#define M_INTR_RESET                            0x04
#define M_INTR_SOF                              0x08
#define M_INTR_PLUG                 0x10    /* Plug and Unplug intr */
#define M_INTR_PLUG_OUT             0xC0
#define M_INTR_EP0                              0x0001

/*power*/
#define M_POWER_RESUME                          0x04
#define M_POWER_SUSPENDENB              0x01

#define M_INTR_SUSPEND_Enab             0x00000001


/* Register read/write macros */
// #define READ_REG(reg)                (reg)
// #define WRITE_REG(reg,data)  ((reg) = (data))


#define USB_INT_BULKOUT_EP1             (0x2)
#define USB_INT_BULKOUT_EP2             (0x4)
#define USB_INT_BULKOUT_EP3             (0x8)
#define USB_INT_BULKIN_EP1              (0x20000)
#define USB_INT_BULKIN_EP2              (0x40000)
#define USB_INT_BULKIN_EP3              (0x80000)


#define DATA_NEED                               (0)
#define DATA_NONEED                             (1)

#ifndef NULL
#define NULL 0
#endif

extern uint32_t gnDevState;
extern void *gpCurCfg;

/* Function prototypes */
void USB_Reset(void);
void Endpoint0(int32_t nCallState);
void musbfsfc_intr(void);
void USB_DmaReceivePre(uint8_t *pBuffer, uint8_t u8Ep, uint32_t data_len);
void USB_DmaTransfer(uint8_t *pBuffer, uint8_t u8Ep, uint32_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* _MUSBFSFC_H_ */

