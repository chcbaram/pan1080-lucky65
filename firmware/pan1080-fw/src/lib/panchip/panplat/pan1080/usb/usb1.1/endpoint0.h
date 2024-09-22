/****************************************

   endpoint0.h
   Endpoint 0 header file for MUSBFSFC firmware

****************************************/

#ifndef _ENDPOINT0_H_
#define _ENDPOINT0_H_

#include <stdint.h>

/* Endpoint 0 states */
#define M_EP0_IDLE          0
#define M_EP0_RX            1
#define M_EP0_TX            2

/* Define device states */
#define DEVSTATE_DEFAULT    0
#define DEVSTATE_ADDRESS    1
#define DEVSTATE_CONFIG     2

/* CSR0 bit masks */
#define M_CSR0_OUTPKTRDY    0x01
#define M_CSR0_INPKTRDY     0x02
#define M_CSR0_SENTSTALL    0x04
#define M_CSR0_DATAEND      0x08
#define M_CSR0_SETUPEND     0x10
#define M_CSR0_SENDSTALL    0x20
#define M_CSR0_SVDOUTPKTRDY 0x40
#define M_CSR0_SVDSETUPEND  0x80

#define M_INCSR2_FrcDataTog 0x08
#define M_INCSR2_DMAEnab    0x10
#define M_INCSR2_MODE       0x20
#define M_INCSR2_ISO        0x40
#define M_INCSR2_AUTOSET    0x80

#define M_OUTCSR2_DMAMode   0x10
#define M_OUTCSR2_DMAEnab   0x20
#define M_OUTCSR2_ISO       0x40
#define M_OUTCSR2_AutoClear 0x80

/* Define maximum packet size for endpoint 0 */
#define M_EP0_MAXP          64
/* Define maximum number of interfaces in a configuration */
#define M_MAXIFS            16
/* Define maximum number of reportsize in a configuration */


#pragma pack(1)

/* Endpoint 0 status structure */
typedef struct
{
  int32_t nState;     /* IDLE/RX/TX */
  int32_t nBytesLeft; /* Number of bytes left to send in TX mode */
  void   *pData;      /* Pointer to data to transmit/receive */
  int32_t nBytesRecv; /* Number of bytes received in RX mode */
  uint8_t byFAddr;    /* New function address */
} USB_EndPointStatusDef;

// typedef USB_EndPointStatusDef *PM_EP0_STATUS;
#pragma pack()
extern uint32_t gnDevState;
/* Local functions */
void     Endpoint0(int32_t nCallState);
void     Endpoint0_Tx(USB_EndPointStatusDef *pep0state);
void     Endpoint0_Rx(void);
uint32_t ConfigureIfs(void);

#endif /* _ENDPOINT0_H_ */
