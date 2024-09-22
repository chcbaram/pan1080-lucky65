/****************************************

   endpoint.h
   Endpoint header file for MUSBFSFC firmware

****************************************/

#ifndef _ENDPOINT_H_
#define _ENDPOINT_H_

#include <stdint.h>

#define M_EP_NODATA          -1
/* Define maximum packet size for bulk endpoints */
#define M_EP_MAXP            64

/* Define endpoint call status values */
#define M_EP_NORMAL          0
#define M_EP_RESET           1

/* Endpoint CSR register bits */
#define M_INCSR_IPR          0x01
#define M_INCSR_FifoNotEmpty 0x02
#define M_INCSR_UnderRun     0x04
#define M_INCSR_FF           0x08
#define M_INCSR_SendStall    0x10
#define M_INCSR_SentStall    0x20
#define M_INCSR_CDT          0x40

#define M_OUTCSR_OPR         0x01
#define M_OUTCSR_FifoFull    0x02
#define M_OUTCSR_OverRun     0x04
#define M_OUTCSR_DataError   0x08
#define M_OUTCSR_FF          0x10
#define M_OUTCSR_SendStall   0x20
#define M_OUTCSR_SentStall   0x40
#define M_OUTCSR_CDT         0x80

#define EnableDMA            0x00000001
#define DIR_IN               0x00000002
#define DIR_OUT              0x00000000
#define DMAMode0             0x00000000
#define DMAMode1             0x00000004
#define IntEnab              0x00000008
#define EndPoint1            0x00000010
#define EndPoint2            0x00000020
#define EndPoint3            0x00000030
#define DmaMode1Max1         0x00000100
#define DmaMode1Max2         0x00000200
#define DmaMode1Max3         0x00000300
#define DmaMode1Max4         0x00000400
#define DmaMode1Max5         0x00000500
#define DmaMode1Max6         0x00000600
#define DmaMode1Max7         0x00000700
#define DmaMode1Max8         0x00000800
#define DmaMode1Max16        0x00001000

#pragma pack(1)

/* Bulk In Endpoint status structure */
typedef struct
{
  void   *pData;      /* Pointer to data to transmit */
  int32_t nBytesLeft; /* Number of bytes left to send */
  uint8_t byEP;       /* Endpoint number */
} USB_BulkInStatus;

// typedef USB_BulkInStatus *PM_EPBIN_STATUS;

/* Bulk Out Endpoint status structure */
typedef struct
{
  void    *pData;      /* Pointer to data buffer */
  uint32_t nBuffLen;   /* Length of data buffer */
  uint32_t nBytesRecv; /* Number of bytes received */
  uint8_t  byEP;       /* Endpoint number */
} USB_BulkOutStatus;

#pragma pack()
// typedef USB_BulkOutStatus * PM_EPBOUT_STATUS;

extern USB_BulkInStatus  gepbin1, gepbin2, gepbin3;
extern USB_BulkOutStatus gepbout1, gepbout2, gepbout3;
extern uint8_t           usb_driver_init_finish;

/* Function prototypes */
void EndpointBulkIn(USB_BulkInStatus *pbistate, uint32_t nCallState);
void EndpointBulkOut(USB_BulkOutStatus *pbostate, uint32_t nCallState);

#endif /* _ENDPOINT_H_ */
