/*
 * Copyright (C) 2021 Panchip Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0 
 */
 
/**
 * @file
 * @brief    Panchip series fmc driver header file 
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 21/11/12 18:33 $ 
 */


#ifndef __PAN_FMC_H__
#define __PAN_FMC_H__

/**
 * @brief Fmc Interface
 * @defgroup fmc_interface Fmc Interface
 * @{
 */
 
#ifdef __cplusplus
extern "C"
{
#endif


/**@defgroup FMC_ERASE_FLAG Fmc erase command 
 * @brief       Fmc erase command definition
 * @{ */

#define CMD_ERASE_PAGE		(0X81)		/*!< Command to earse one page */
#define CMD_ERASE_SECTOR	(0X20)		/*!< Command to earse one sector,4K byte size */
#define CMD_ERASE_32K		(0X52)			/*!< Command to earse 64K byte size memory one time */
#define CMD_ERASE_64K		(0Xd8)			/*!< Command to earse 64K byte size memory one time */
#define CMD_ERASE_CHIP		(0xc7)		/*!< Command to earse all flash memory */
/**@} */

/**@defgroup FMC_STATUS_WR_FLAG Fmc status reg w/r command
 * @brief       Fmc status register write and read command definition
 * @{ */
#define CMD_READ_STATUS_L	(0x05)	/*!< Command to read flash status register low 8 bit */
#define CMD_READ_STATUS_H	(0x35)	/*!< Command to read flash status register high 8 bit */
#define CMD_WRITE_STATUS	(0x01)	/*!< Command to write flash status register */
/**@} */

/**@defgroup FMC_STATUS_OPERATION_FLAG Fmc status reg operation command
 * @brief       Fmc status register operation command definition
 * @{ */
#define OPS_WR_STU_REG_NOR	(0x00)	/*!< Command to control flash status register write mode */
#define OPS_WR_STU_REG_ALL	(0x01)	/*!< Command to control flash status register write mode */
/**@} */

/**@defgroup FMC_READ_FLAG Fmc read command
 * @brief       Fmc read command definition
 * @{ */
#define CMD_FAST_READ		(0x0b)				/*!< Command to control read mode as fast mode */
#define CMD_NORM_READ		(0x03)				/*!< Command to control read mode as normal mode */
#define CMD_DREAD           (0x3B)		/*!< Command to control read mode as dual mode */
#define CMD_2READ           (0xBB)		/*!< Command to control read mode as 2 io read mode */
#define CMD_QREAD           (0x6B)		/*!< Command to control read mode as quad mode */
#define CMD_4READ           (0xEB)		/*!< Command to control read mode as 4 io read mode */
/**@} */

/**@defgroup FMC_BURST_READ_FLAG Fmc burst read command
 * @brief       Fmc burst read command definition
 * @{ */
#define CMD_BURST_READ		(0x77)			/*!< Command to control read mode as burst read mode */
#define BURST_READ_MODE_8	(0x00)			/*!< Burst read a fixed length of 8 byte within a 256-byte page */
#define BURST_READ_MODE_16	(0x01)		/*!< Burst read a fixed length of 16 byte within a 256-byte page */
#define BURST_READ_MODE_32	(0x02)		/*!< Burst read a fixed length of 32 byte within a 256-byte page */
#define BURST_READ_MODE_64	(0x03)		/*!< Burst read a fixed length of 64 byte within a 256-byte page */
/**@} */


#define CMD_TRIG			(0x01)					/*!< Command to triger start operate */
#define CMD_WRITE_ENABLE	(0x06)			/*!< Command to write enable operate */

/**@defgroup FMC_MODE_FLAG Fmc work type select
 * @brief       Fmc work type select definition
 * @{ */
#define FLASH_X1_MODE       0				/*!< Flash work type select 1 line mode */
#define FLASH_X2_MODE       1				/*!< Flash work type select 2 line mode */
#define FLASH_X4_MODE       2				/*!< Flash work type select 4 line mode */
/**@} */

#define  QUAD_ENABLE_Pos                               (9)			/*!< Flash status reg quad enable bit position */
#define  QUAD_ENABLE_Msk                               (0x1ul << QUAD_ENABLE_Pos)	/*!< Flash status reg quad enable value */
#define  Write_Enable_Latch_Pos                        (1)			/*!< Flash write enable bit position */
#define  Write_Enable_Latch_Msk                        (0x1ul << Write_Enable_Latch_Pos)	/*!< Flash write enable value */
#define  Write_In_Process_Pos                          (0)			/*!< Flash status reg WIP bit position */
#define  Write_In_Process_Msk                          (0x1ul << Write_In_Process_Pos)	/*!< Flash status reg WIP bit value,this bit indicates whether the memory is busy */
#define  Long_Time_Op_Pos                              (17)			/*!< Flash mode reg long time operate bit position */

#define PAGE_SIZE			(256)					/*!< Flash one page size */	
#define SECTOR_SIZE         (4096)	/*!< Flash one sector size */	

/** 
 * @brief Structure with flash id.
 *
 * @param manufacturer_id 		Specifies the flash manufacturer id
 * @param memory_type_id			Specifies the flash memory type id
 * @param memory_density_id		Specifies the flash memory density id  
 * @param uid									Specifies the flash unique id
 */
/* #pragma anon_unions */
typedef struct _flash_ids {
    uint8_t manufacturer_id;
    union {
        uint8_t d[2];
        struct {
            uint8_t memory_type_id;
            uint8_t memory_density_id;
        };
    };
    uint8_t uid[16];
} FLASH_IDS_T;

extern FLASH_IDS_T flash_ids;

/**
  * @brief		Enter flash deep power down mode
  * @param  	fmc Where fmc is the base address of fmc peripheral
  * @return 	None
 	* @note  		This function should be put in RAM in case of use
 	*			Make sure Flash is out of enhance mode before use this API  
	*/
__STATIC_FORCEINLINE void FMC_EnterDeepPowerDownMode(FLCTL_T *fmc)
{
    // enter flash dp mode
    fmc->X_FL_CTL = (0<<8) | (0x01<<0);
    fmc->X_FL_WD[0] = 0xB9;
    fmc->X_FL_TRIG = CMD_TRIG;
    while(fmc->X_FL_TRIG){};
}

/**
  * @brief		Exit flash deep power down mode
  * @param  	fmc Where fmc is the base address of fmc peripheral
  * @return 	None
 	* @note  		This function should be put in RAM in case of use
 	*			Flash may need some time (0.1us ~ 30us for different flash vendor)
 	*			before it reliably wakes up from deep power down mode.  
	*/
__STATIC_FORCEINLINE void FMC_ExitDeepPowerDownMode(FLCTL_T *fmc)
{
    // exit flash dp mode
    fmc->X_FL_CTL = (0<<8) | (0x01<<0);
    fmc->X_FL_WD[0] = 0xAB;
    fmc->X_FL_TRIG = CMD_TRIG;
    while(fmc->X_FL_TRIG){};
}

/**
  * @brief  This function is used to get flash unique id,
  * @param  fmc: where fmc is a flash peripheral.
  * @return Flash unique id
  */
extern void FMC_GetFlashUniqueId(FLCTL_T *fmc);

/**
  * @brief  This function is used to get flash jedec id,
  * @param  fmc: where fmc is a flash peripheral.
  * @return Flash jedec id
  */
extern void FMC_GetFlashJedecId(FLCTL_T *fmc);

/**
  * @brief Get Size of Flash in Bytes
  * @param fmc Flash memory controller base
  * @return Size in bytes
  */
extern uint32_t FMC_GetFlashCodeAreaSize(FLCTL_T *fmc);

/**
  * @brief Erase a 4KB Sector in Flash Memory.
  *
  * This function is used to erase a 4KB flash sector
  * in flash code area.
  *
  * @param fmc: where fmc is a flash peripheral.
  * @param adr: where addr is a erase start address.
  * @retval 0:  Success.
  * @retval -1: Fail.
  */
extern int FMC_EraseSector(FLCTL_T *fmc,unsigned long adr);
/**
  * @brief Erase a 32KB Block in Flash Memory.
  *
  * This function is used to erase a 32KB flash block
  * in flash code area.
  *
  * @note The 1st 28KB flash block is not allowed to be erased
  * as the Info Area resides in this physical 32KB flash block.
  * @note The 32KB flash block start address is 0x7000, 0xF000,
  * 0x17000, 0x1F000, 0x27000, 0x2F000, ...
  *
  * @param fmc: where fmc is a flash peripheral.
  * @param adr: where addr is a erase start address.
  * @retval 0: Success.
  * @retval -1: Fail.
  */
extern int FMC_EraseBlock32k(FLCTL_T *fmc,unsigned long addr);
/**
  * @brief Erase a 64KB Block in Flash Memory.
  *
  * This function is used to erase a 64KB flash block
  * in flash code area.
  *
  * @note The 1st 60KB flash block is not allowed to be erased
  * as the Info Area resides in this physical 64KB flash block.
  * @note The 64KB flash block start address is 0xF000, 0x1F000,
  * 0x2F000, 0x3F000, 0x4F000, 0x5F000, ...
  *
  * @param fmc: where fmc is a flash peripheral.
  * @param adr: where addr is a erase start address.
  * @retval 0: Success.
  * @retval -1: Fail.
  */
extern int FMC_EraseBlock64k(FLCTL_T *fmc,unsigned long addr);
/**
  * @brief Erase the whole flash memory.
  *
  * This function is used to erase all data in flash, include
  * Code Area and Info Area.
  *
  * @note This API should only be used when you really know what
  * you are doing.
  *
  * @param fmc: where fmc is a flash peripheral.
  * @retval 0: Success.
  * @retval -1: Fail.
  */
extern int FMC_EraseChip(FLCTL_T *fmc);
/**
  * @brief Erase specific data in flash code area.
  *
  * This function is used to erase specific data in flash Code
  * Area.
  *
  * @note This API is the most recommended function used to erase
  * data in flash code area.
  *
  * @param fmc: where fmc is a flash peripheral.
  * @param addr: start address to erase, should be 4KB aligned.
  * @param len: length to erase in bytes.
  * @retval 0: Success.
  * @retval -1: Fail.
  */
extern int FMC_EraseCodeArea(FLCTL_T *fmc, uint32_t addr, uint32_t len);

/**
  * @brief  This function is used to init Icache,
  * @param  fmc: Where fmc is a flash peripheral.
  * @param  mode: Where mode can be \ref FLASH_X1_MODE
  *                                 \ref FLASH_X2_MODE
  *                                 \ref FLASH_X4_MODE
  * @return None
  */
extern void InitIcache(FLCTL_T *fmc,uint32_t mode);

/**
  * @brief        Calculate and read the CRC32 checksum of a specified flash area.
  * @param[in]  	fmc: Where fmc is a flash peripheral.
  * @param[in]    addr     Start address of the flash area to be executed CRC32 checksum calculation.
  * @param[in]    count    Number of bytes to be calculated.
  * @param[out]   chksum   If success, it will contain the result of CRC32 checksum calculation.
  * @retval   0   Success
  * @retval   -1  Invalid parameter.
  */
extern int32_t FMC_GetCrc32Sum(FLCTL_T *fmc,uint32_t addr, uint32_t count, uint32_t *chksum);

/**  
  * @brief	Blank check checks if memory is blank
  * @param 	fmc: 	Where fmc is a flash peripheral.
  * @param	adr:	Block start address
  * @param	sz:		Block Size (in bytes)
  * @retval 0   	Success
  * @retval 1  		Fail
  */
extern int BlankCheck (FLCTL_T *fmc,unsigned long adr, unsigned long sz);

/**  
  * @brief	Verify in flash memory
  * @param 	fmc: 	Where fmc is a flash peripheral.
  * @param	adr:	Block start address
  * @param	sz:		Data lenth need to verify
  * @param	buf:	Programed data
  * @note		The buf address should be 4-bytes-alingned.
  * @return   		Final verify address
  */
extern unsigned long Verify(FLCTL_T *fmc, unsigned long adr, unsigned long sz, unsigned char *buf);

/**  
  * @brief	Program page in flash memory
  * @param 	fmc: 	Where fmc is a flash peripheral.
  * @param	adr:	Block start address
  * @param	sz:		Page size
  * @param	buf:	Page data 
  * @retval 0   	Success
  * @retval 1  		Fail
  */
extern int ProgramPage (FLCTL_T *fmc, unsigned long adr, unsigned long sz, unsigned char *buf);

/**
  * @brief  This function is used to read status,
	* The Read Status Register can be read at any time
  * @param  fmc: where fmc is a flash peripheral.
	* @param  cmd: where cmd can be \ref CMD_READ_STATUS_L(return register bit[7:0])
	*																\ref CMD_READ_STATUS_H(return register bit[15:8])
  * @return status value
  */
extern unsigned char FMC_ReadStatusReg(FLCTL_T *fmc,unsigned char cmd);

/**
  * @brief  This function is used to enable write function,
  * @param  fmc: where fmc is a flash peripheral.
  * @return Enable or not
  */
extern unsigned char FMC_WriteEnable(FLCTL_T *fmc);

/**
  * @brief  This function is used to write status,
  * The Read Status Register can be read at any time
  * @param  fmc: where fmc is a flash peripheral.
  * @param  ops: where ops can be \ref OPS_WR_STU_REG_ALL
  *                               \ref OPS_WR_STU_REG_NOR
  * @param  buf: where buf is write value
  * @return none
  */
extern void FMC_WriteStatusReg(FLCTL_T *fmc, unsigned char ops, unsigned int buf);

/**
	* @brief  This function is used to read flash,
	* @param  fmc: where fmc is a flash peripheral.
	* @param  Addr: where Addr is start address to read
	* @param  cmd: where cmd can be \ref CMD_FAST_READ or \ref CMD_NORM_READ
	* @return 1byte data
	*/
extern unsigned char FMC_ReadByte(FLCTL_T *fmc, unsigned int Addr, unsigned char cmd);

/**
	* @brief  This function is used to write one byte to flash memory
	* @param  fmc: Where fmc is a flash peripheral.
	* @param  addr: Where Addr is start address to write
	* @param  data: Where data is data need to program
	* @return None
	*/
extern void FMC_WriteByte(FLCTL_T *fmc, unsigned int addr, unsigned char data);

/**
	* @brief  This function is used to write halfword to flash memory
	* @param  fmc: Where fmc is a flash peripheral.
	* @param  addr: Where Addr is start address to write
	* @param  data: Where data is data need to program
	* @return None
	*/
extern void FMC_WriteHalfWord(FLCTL_T *fmc, unsigned int addr, unsigned short data);

/**
	* @brief  This function is used to read flash
	* @param  fmc: where fmc is a flash peripheral.
	* @param  Addr: where Addr is start address to read
	* @param  cmd: where cmd can be CMD_FAST_READ or CMD_NOR_READ
	* @return 4byte data
	*/
extern unsigned int FMC_ReadWord(FLCTL_T *fmc,unsigned int Addr, unsigned char cmd);

/**
  * @brief  This function is used to read a page size (256 bytes) of data from flash
  * @param  fmc     where fmc is a flash peripheral.
  * @param  Addr    where Addr is start address to read
  * @param  cmd     where cmd can be CMD_FAST_READ or CMD_NORM_READ
  * @return Internal Buffer address
  */
extern unsigned char *FMC_ReadPage(FLCTL_T *fmc,unsigned int Addr,unsigned char cmd);

/**
  * @brief  This function is used to read data stream from flash
  * @param  fmc     where fmc is a flash peripheral.
  * @param  Addr    where Addr is start address to read
  * @param  cmd     where cmd can be CMD_FAST_READ or CMD_NORM_READ
  * @param  buf     where buf is a buffer to store read data
  * @param  len     where len is data length of bytes to read
  * @return None
  */
extern int FMC_ReadStream(FLCTL_T *fmc, unsigned int Addr, unsigned char cmd, unsigned char *buf, unsigned int len);

/**
  * @brief  This function is used to write data stream to flash
  * @param  fmc     where fmc is a flash peripheral
  * @param  Addr    where Addr is start address to write, can be any valid address
  * @param  buf     where buf is a buffer to store data to write
  * @param  len     where len is data length of bytes to write
  * @return None
  */
extern int FMC_WriteStream(FLCTL_T *fmc, unsigned int Addr, unsigned char *buf, unsigned int len);
/**
  * @brief  This function is used to read data from the flash 4KB INFO area.
  * @param  fmc     where fmc is a flash peripheral.
  * @param  Addr    where Addr is start address to read, can be 0 ~ 4095
  * @param  cmd     where cmd can be CMD_FAST_READ or CMD_NORM_READ
  * @param  buf     where buf is a buffer to store read data
  * @param  len     where len is data length of bytes to read
  * @retval 0   read success
  * @retval -1  read fail
  */
extern int FMC_ReadInfoArea(FLCTL_T *fmc, unsigned int Addr, unsigned char cmd, unsigned char *buf, unsigned int len);
/**
  * @brief  This function is used to write data to the flash 4KB INFO area.
  * @param  fmc     where fmc is a flash peripheral
  * @param  Addr    where Addr is start address to write, can be 0 ~ 4095
  * @param  buf     where buf is a buffer to store data to write
  * @param  len     where len is data length of bytes to write
  * @retval 0   write success
  * @retval -1  write fail
  */
extern int FMC_WriteInfoArea(FLCTL_T *fmc, unsigned int Addr, unsigned char *buf, unsigned int len);
/**
  * @brief  This function is used to erase the flash 4KB INFO area.
  * @param  fmc where fmc is a flash peripheral
  * @retval 0   erase success
  * @retval -1  erase fail
  */
extern int FMC_EraseInfoArea(FLCTL_T *fmc);

/**
	* @brief  This function is used to set flash work mode
	* @param  fmc: Where fmc is a flash peripheral.
	* @param  mode: Where mode is line mode select. \ref FMC_MODE_FLAG
	* @param  enhance: Whether enhanced mode is supported
	* @return None
	*/
extern void FMC_SetFlashMode(FLCTL_T *fmc, uint32_t mode, bool enhance);

/**
	* @brief  This function is used to exit flash enhanced read mode
	* @param  fmc: Where fmc is a flash peripheral.
	* @return None
	*/
extern void FMC_ExitEnhanceMode(FLCTL_T *fmc);

/**
	* @brief  This function is used to enter flash enhanced read mode
	* @param  fmc: Where fmc is a flash peripheral.
	* @param  mode: Where mode is line mode select. \ref FMC_MODE_FLAG
	* @note 	Some Flash does not support 2-line enhance mode
	* @return None
	*/
extern void FMC_EnterEnhanceMode(FLCTL_T *fmc, uint32_t mode);

/**
	* @brief  This function is used to enable auto enter deep power down function
	* @param  fmc: Where fmc is a flash peripheral.
	* @return None
	*/
extern void FMC_EnableAutoDp(FLCTL_T *fmc);

/**
	* @brief  This function is used to disable auto enter deep power down function
	* @param  fmc: Where fmc is a flash peripheral.
	* @return None
	*/
extern void FMC_DisableAutoDp(FLCTL_T *fmc);

/**
	* @brief  This function is used to set wait count for release from Deep Power-down mode
	* @param  fmc: Where fmc is a flash peripheral.
	* @param  cnt: Where cnt is a flash clk count value.
	* @return None
	*/
extern void FMC_SetRdpWaitCount(FLCTL_T *fmc, uint32_t cnt);

/**@} */


#ifdef __cplusplus
}
#endif

#endif /* __PAN_FMC_H__ */

