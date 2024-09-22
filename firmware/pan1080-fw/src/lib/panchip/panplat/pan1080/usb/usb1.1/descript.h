/****************************************

   descript.h
   Endpoint 0 descriptor definitions for MUSBFSFC firmware

   Note the defined structures assume WORDS are stored little-endian.

****************************************/

#ifndef _DESCRIPT_H_
#define _DESCRIPT_H_

/* Command bit fields */
#define M_CMD_DATADIR   0x80

/* Request Type Field */
#define M_CMD_TYPEMASK  0x60
#define M_CMD_STDREQ    0x00
#define M_CMD_CLASSREQ  0x20
#define M_CMD_VENDREQ   0x40
#define M_CMD_STDDEVIN  0x80
#define M_CMD_STDDEVOUT 0x00
#define M_CMD_STDIFIN   0x81
#define M_CMD_STDIFOUT  0x01
#define M_CMD_STDEPIN   0x82
#define M_CMD_STDEPOUT  0x02

/* Standard Request Codes */
#define GET_STATUS              0x00
#define CLEAR_FEATURE           0x01
#define SET_FEATURE             0x03
#define SET_ADDRESS             0x05
#define GET_DESCRIPTOR          0x06
#define SET_DESCRIPTOR          0x07
#define GET_CONFIGURATION       0x08
#define SET_CONFIGURATION       0x09
#define GET_INTERFACE           0x0A
#define SET_INTERFACE           0x0B
#define SYNCH_FRAME             0x0C

/* Standard Descriptor Types */
#define M_DST_DEVICE     0x01
#define M_DST_CONFIG     0x02
#define M_DST_STRING     0x03
#define M_DST_INTERFACE  0x04
#define M_DST_ENDPOINT   0x05
#define M_DST_DEVQUAL    0x06
#define M_DST_OTHERSPEED 0x07
#define M_DST_POWER      0x08
#define M_CMD_DESCMASK   0xFF00
#define M_CMD_DEVICE     (M_DST_DEVICE << 8)
#define M_CMD_CONFIG     (M_DST_CONFIG << 8)
#define M_CMD_STRING     (M_DST_STRING << 8)
#define M_CMD_DEVQUAL    (M_DST_DEVQUAL << 8)
#define M_CMD_OTHERSPEED (M_DST_OTHERSPEED << 8)
#define M_CMD_HID_DSC    (0X2100)
#define M_CMD_REPORT     (0X2200)

/* Endpoint transfer types */
#define M_EP_TFMASK     0x03
#define M_EP_CONTROL    0x00
#define M_EP_ISO        0x01
#define M_EP_BULK       0x02
#define M_EP_INTR       0x03

/* Standard Device Feature Selectors */
#define M_FTR_DEVREMWAKE 0x0001



#pragma pack(1)
/* Command structure */
typedef struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} USB_CommandDef;
// typedef USB_CommandDef *PCOMMAND;

/* Standard Device Descriptor */
typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} USB_DeviceDscrDef;
// typedef USB_DeviceDscrDef *PSTD_DEV_DSCR;

/* Standard Configuration Descriptor */
typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} USB_CfgDscrDef;
// typedef USB_CfgDscrDef *PSTD_CFG_DSCR;

/* Standard Interface Descriptor */
typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} USB_InterfaceDscrDef;
// typedef USB_InterfaceDscrDef *PSTD_IF_DSCR;

/* Standard Endpoint Descriptor */
typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} USB_EndPointDscrDef;
typedef USB_EndPointDscrDef *PSTD_EP_DSCR;

/* Standard hid Descriptor */
typedef struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	uint8_t bDescriptorTypeClass;
	uint16_t wDescriptorLength;
} USB_HidDscrDef;

/* hid report Descriptor */

typedef struct {
	uint16_t bUsagePage;
	uint16_t bUsageMin;
	uint16_t bUsageMax;
	uint16_t bLogicMin;
	uint16_t bLogicMax;
	uint16_t bReportCount;
	uint16_t bReportSize;
	uint16_t bInput;
	uint16_t bReportCount1;
	uint16_t bReportSize1;
	uint16_t bInput1;
} USB_ButtonDscrDef;

typedef struct {
	uint16_t bUsagePage;
	uint16_t bUsageX;
	uint16_t bUsageY;
	uint16_t bLogicMin;
	uint16_t bLogicMax;
	uint16_t bReportCount;
	uint16_t bReportSize;
	uint16_t bInput;
} USB_CoordinateDscrDef;
typedef struct {
	uint16_t bPointer;
	uint16_t bCollection;
	USB_ButtonDscrDef button;
	USB_CoordinateDscrDef coord;
	uint16_t bEndCollection;
} USB_UsrDscrDef;

typedef struct {
	uint16_t bUsagePage;
	uint16_t bUsageType;
	uint16_t bCollection;
	USB_UsrDscrDef usr;
	uint16_t bEndCollection;
} USB_ReportDscrDef;
#pragma pack()

/* Configuration 1 */
typedef struct {
	USB_InterfaceDscrDef stdif;             /* Interface 1 Alternate 1 Descriptor */
	USB_HidDscrDef hid0;
	USB_EndPointDscrDef stdep_0;            /* 1st Endpoint Descriptor for Interface 1 */
//	USB_EndPointDscrDef   stdep_1;      /* 2nd Endpoint Descriptor for Interface 1 */
} USB_CfgDef;

/* Configuration 1 */
typedef struct {
	USB_CfgDscrDef stdcfg;          /* Required Standard Configuration Descriptor */
	USB_CfgDef cfg_00;              /* Interface 0 Descriptor &Endpoint Descriptor*/
//	USB_CfgDef	  cfg_10;		  /* Interface 1 Alternate 0 Descriptor & Endpoint Descriptor */
//	USB_CfgDef	  cfg_11;		  /* Interface 1 Alternate 1 Descriptor & Endpoint Descriptor */
//	USB_CfgDef	  cfg_20;	     /* Interface 2 Descriptor & Endpoint Descriptor*/
} __attribute ((aligned(4))) USB_ConfigTypeDef;

/* Configuration 2 */
typedef struct {
	USB_CfgDscrDef stdcfg;          /* Required Standard Configuration Descriptor */
	USB_CfgDef cfg_00;              /* Interface 0 Descriptor &Endpoint Descriptor*/
	USB_CfgDef cfg_10;              /* Interface 1 Alternate 0 Descriptor & Endpoint Descriptor */
	USB_CfgDef cfg_20;              /* Interface 2 Descriptor & Endpoint Descriptor*/
} __attribute ((aligned(4))) USB_ConfigTypeDef_2;

/* Top level structure containing pointers to each configuration */
typedef struct {
	USB_ConfigTypeDef               *pcfg1;                 /* Pointer to M_CFG_1 */
	USB_ConfigTypeDef_2             *pcfg2;                 /* Pointer to M_CFG_2 */
} __attribute ((aligned(4))) USB_ConfigPointTypeDef;

extern USB_ConfigTypeDef cfg1;
extern USB_ConfigTypeDef_2 cfg2;
extern USB_ConfigPointTypeDef cfg;
extern USB_DeviceDscrDef stddevdsc;
extern USB_HidDscrDef gHidDsc;

#endif /* _DESCRIPT_H_ */

