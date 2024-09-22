/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 19/10/28 16:08 $ 
 * @brief    Panchip series fmc driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/ 
#include "PanSeries.h"
#include "pan_fmc.h"

#define BIT(x)   (0x1UL << (x))

FLASH_IDS_T flash_ids = {0};

void FMC_GetFlashUniqueId(FLCTL_T *fmc)
{
	fmc->X_FL_CTL = (16<<8) | (5<<0);
	fmc->X_FL_WD[0] = 0x4B;
    fmc->X_FL_WD[1] = 0x00;
    fmc->X_FL_WD[2] = 0x00;
    fmc->X_FL_WD[3] = 0x00;
    fmc->X_FL_WD[4] = 0x00;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

    memcpy(flash_ids.uid, (void*)FLCTL_BUFF->X_FL_BUFFER, 16);
}

void FMC_GetFlashJedecId(FLCTL_T *fmc)
{
	fmc->X_FL_CTL = (3<<8) | (1<<0);
	fmc->X_FL_WD[0] = 0x9F;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

	flash_ids.manufacturer_id = FLCTL_BUFF->X_FL_BUFFER[0];
	flash_ids.memory_type_id = FLCTL_BUFF->X_FL_BUFFER[1];
	flash_ids.memory_density_id = FLCTL_BUFF->X_FL_BUFFER[2];
}

/**
 * @brief Get Size of Flash Code Area in Bytes
 * @param fmc Flash memory controller base
 * @note There are 2 hw defined areas for pan1080 flash: Code Area and Info Area.
 *       The Info Area is fixed to 4K Bytes, and all remained flash belongs to
 *       the Code Area. Commonly and as default we can only access the Code Area
 *       from address 0 to CodeAreaSize-1, and we can access the Info Area (from
 *       address 0 to 4095) using special access API introduces later.
 * @return Size in bytes
 */
uint32_t FMC_GetFlashCodeAreaSize(FLCTL_T *fmc)
{
    if (flash_ids.memory_density_id == 0)
    {
        FMC_GetFlashJedecId(fmc);
    }
    return BIT(flash_ids.memory_density_id) - SECTOR_SIZE;
}

/**
  * @brief  This function is used to read status,
	* The Read Status Register can be read at any time
  * @param  fmc: where fmc is a flash peripheral.
	* @param  cmd: where cmd can be CMD_READ_STATUS_L(return register bit[7:0])
																	CMD_READ_STATUS_H(return register bit[15:8])
  * @retval status value
  */
unsigned char FMC_ReadStatusReg(FLCTL_T *fmc,unsigned char cmd)
{
	fmc->X_FL_CTL = (1<<8) | (1<<0);
	fmc->X_FL_WD[0] = cmd;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

	return FLCTL_BUFF->X_FL_BUFFER[0];
}

/**
  * @brief  This function is used to enable write function,
  * @param  fmc: where fmc is a flash peripheral.
  * @retval enable or not
  */
unsigned char FMC_WriteEnable(FLCTL_T *fmc)
{
	fmc->X_FL_CTL = (0<<8) | (1<<0);
	fmc->X_FL_WD[0] = CMD_WRITE_ENABLE;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

    return 1;
}
/**
  * @brief  This is a internal function used to erase flash,
  * @param  fmc: where fmc is a flash peripheral.
  * @param  Addr: where addr is a erase start address.  
  * @param  cmd: where cmd can be CMD_ERASE_PAGE
	*							  CMD_ERASE_SECTOR
	*							  CMD_ERASE_32K
	*							  CMD_ERASE_64K
	*							  CMD_ERASE_CHIP
  * @retval 0: Success
  * @retval -1: Fail
  */
static int FMC_Erase(FLCTL_T *fmc,unsigned int Addr,unsigned char cmd)
{
	unsigned char offset = 0;
	unsigned char bytes_num_w;

	FMC_WriteEnable(fmc);

	if(cmd != CMD_ERASE_CHIP)
	// - it's page/sector/block erase
		bytes_num_w = 0x04;
	else
	// - it's chip erase
		bytes_num_w = 0x01;
			
	fmc->X_FL_CTL = (0<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[offset++] = cmd;

	fmc->X_FL_WD[offset++] = (Addr & 0x00ff0000)>>16;
	fmc->X_FL_WD[offset++] = (Addr & 0x0000ff00)>>8;
	fmc->X_FL_WD[offset++] = Addr & 0xff;
	fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){}

	while(FMC_ReadStatusReg(fmc,CMD_READ_STATUS_L) & Write_In_Process_Msk) {}
	return 0;
}

/**
  * @brief Erase a 4KB Sector in Flash Memory.
  *
  * This function is used to erase a 4KB flash sector
  * in flash code area.
  *
  * @param fmc: where fmc is a flash peripheral.
  * @param adr: where addr is a erase start address.
  * @retval 0: Success.
  * @retval -1: Fail.
  */
int FMC_EraseSector(FLCTL_T *fmc,unsigned long addr)
{
    if (addr >= FMC_GetFlashCodeAreaSize(fmc)) {
        return -1;
    }
    return FMC_Erase(fmc,addr,CMD_ERASE_SECTOR);
}

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
int FMC_EraseBlock32k(FLCTL_T *fmc,unsigned long addr)
{
    if ((addr >= FMC_GetFlashCodeAreaSize(fmc))
        || (addr < 0x7000)) {
        return -1;
    }
    return FMC_Erase(fmc,addr,CMD_ERASE_32K);
}

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
int FMC_EraseBlock64k(FLCTL_T *fmc,unsigned long addr)
{
    if ((addr >= FMC_GetFlashCodeAreaSize(fmc))
        || (addr < 0xF000)) {
        return -1;
    }
    return FMC_Erase(fmc,addr,CMD_ERASE_64K);
}

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
int FMC_EraseChip(FLCTL_T *fmc)
{
    return FMC_Erase(fmc,0x0,CMD_ERASE_CHIP);
}

static void find_special_chunk_in_range(size_t chunk_pattern, uint32_t range_start_sector_idx, uint16_t range_sector_num,
                                        uint32_t *chunk_start_sector_idx, uint16_t *chunk_sector_num)
{
    size_t remainder = range_start_sector_idx % chunk_pattern;

    for (size_t sector_idx = remainder ? (range_start_sector_idx - remainder + chunk_pattern) : range_start_sector_idx;
            sector_idx <= range_start_sector_idx + range_sector_num;
            sector_idx += chunk_pattern) {
        if (*chunk_start_sector_idx == 0) {
            *chunk_start_sector_idx = (sector_idx == 0) ? chunk_pattern : sector_idx;    //Found chunk start position (and should not be 1st block)
        } else {
            *chunk_sector_num += chunk_pattern; //Found multiple expected chunk
        }
    }
}

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
int FMC_EraseCodeArea(FLCTL_T *fmc, uint32_t addr, uint32_t len)
{
    uint32_t block_64k_start_sector_idx = 0;
    uint16_t block_64k_sector_num = 0;
    uint32_t block_32k_start_sector_idx = 0;
    uint16_t block_32k_sector_num = 0;

    uint32_t start_phy_sector_idx = addr / SECTOR_SIZE + 1;
    uint16_t sector_num = len / SECTOR_SIZE + (len % SECTOR_SIZE ? 1 : 0);
    uint16_t total_phy_sector_num = FMC_GetFlashCodeAreaSize(fmc) / SECTOR_SIZE + 1;

    // Check if erase start address and len are 4KB aligned and size not out of flash range
    if ((addr % SECTOR_SIZE != 0) || ((start_phy_sector_idx + sector_num) > total_phy_sector_num)) {
        return -1;
    }

    // Fine 32K blocks from the initial data chunk
    find_special_chunk_in_range(8, start_phy_sector_idx, sector_num, &block_32k_start_sector_idx, &block_32k_sector_num);

    if (block_32k_sector_num == 0) { // No 32k-block found
        // Do only sector erase
        for (size_t i = 0; i < sector_num; i++) {
            FMC_EraseSector(FLCTL, (start_phy_sector_idx + i - 1) * SECTOR_SIZE);
        }
    } else { // Found at least 1 32k-block
        // Do sector erase for leading sectors
        for (size_t i = 0; i < block_32k_start_sector_idx - start_phy_sector_idx; i++) {
            FMC_EraseSector(FLCTL, (start_phy_sector_idx + i - 1) * SECTOR_SIZE);
        }
        // Do sector erase for trailing sectors
        for (size_t i = 0; i < start_phy_sector_idx + sector_num - block_32k_start_sector_idx - block_32k_sector_num; i++) {
            FMC_EraseSector(FLCTL, (block_32k_start_sector_idx + block_32k_sector_num + i - 1) * SECTOR_SIZE);
        }

        // Find 64K blocks from the 32K-block chunk if there are
        find_special_chunk_in_range(16, block_32k_start_sector_idx, block_32k_sector_num, &block_64k_start_sector_idx, &block_64k_sector_num);

        if (block_64k_sector_num == 0) { // No 64k-block found
            // Do only block-32k erase
            for (size_t i = 0; i < block_32k_sector_num; i += 8) {
                FMC_EraseBlock32k(FLCTL, (block_32k_start_sector_idx + i - 1) * SECTOR_SIZE);
            }
        } else { // Found at least 1 64k-block
            // Do block-32k erase for leading sectors if there are
            if (block_32k_start_sector_idx % 16 != 0) {
                FMC_EraseBlock32k(FLCTL, (block_32k_start_sector_idx - 1) * SECTOR_SIZE);
            }
            // Do block-32k erase for trailing sectors if there are
            if ((block_32k_start_sector_idx + block_32k_sector_num) % 16 != 0) {
                FMC_EraseBlock32k(FLCTL, (block_64k_start_sector_idx + block_64k_sector_num - 1) * SECTOR_SIZE);
            }
            // Do block_64k erase
            for (size_t i = 0; i < block_64k_sector_num; i += 16) {
                FMC_EraseBlock64k(FLCTL, (block_64k_start_sector_idx + i - 1) * SECTOR_SIZE);
            }
        }
    }

    return 0;
}

/**
  * @brief  This function is used to write status,
  * The Read Status Register can be read at any time
  * @param  fmc: where fmc is a flash peripheral.
  * @param  ops: where ops can be OPS_WR_STU_REG_ALL
  *                               OPS_WR_STU_REG_NOR
  * @param  buf: where buf is write value
  * @retval none
  */
void FMC_WriteStatusReg(FLCTL_T *fmc,unsigned char ops,unsigned int buf)
{
	unsigned char offset =0;
	unsigned char bytes_num_w;

	if(FMC_WriteEnable(fmc) == 0)
		return;	

	if(ops == OPS_WR_STU_REG_ALL)
	// - write staus [15:0]
		bytes_num_w = 0x03; 
	else
	// - write staus [ 7:0]
		bytes_num_w = 0x02;

    fmc->X_FL_CONFIG &= ~BIT2;  //clear tx_address_transaction bit
	
	fmc->X_FL_CTL = (0<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[offset++] = CMD_WRITE_STATUS;
	fmc->X_FL_WD[offset++] = buf&0xff;
	fmc->X_FL_WD[offset++] = (buf>>8)&0xff;
	fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

    fmc->X_FL_CONFIG |= BIT2;  //reset tx_address_transaction bit

//	while(FMC_ReadStatusReg(fmc,CMD_READ_STATUS_L) & Write_In_Process_Msk);
}
/**
  * @brief  This function is used to read flash,
  * @param  fmc: where fmc is a flash peripheral.
	* @param  Addr: where Addr is start address to read
	* @param  cmd: where cmd can be CMD_FAST_READ or CMD_NOR_READ
  * @retval 4byte data
  */
unsigned int FMC_ReadWord(FLCTL_T *fmc,unsigned int Addr, unsigned char cmd)
{
//	unsigned char offset =0;
	unsigned char bytes_num_w;

	if(cmd == CMD_FAST_READ || cmd == CMD_QREAD || cmd == CMD_2READ || cmd == CMD_DREAD)
		bytes_num_w = 0x05; 
	else if (cmd == CMD_4READ)
        bytes_num_w = 0x07;
    else if (cmd == CMD_NORM_READ)
		bytes_num_w = 0x04;
    else
        bytes_num_w = 0x04;

    fmc->X_FL_CTL = (4<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[0] = cmd;
	fmc->X_FL_WD[1] = (Addr & 0x00ff0000)>>16;
	fmc->X_FL_WD[2] = (Addr & 0x0000ff00)>>8;
	fmc->X_FL_WD[3] = Addr & 0xff;
    fmc->X_FL_WD[4] = 0xff;
    fmc->X_FL_WD[5] = 0xff;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};
	
	return *((__I uint32_t*)FLCTL_BUFF->X_FL_BUFFER);
}
/**
  * @brief  This function is used to read flash,
  * @param  fmc: where fmc is a flash peripheral.
	* @param  Addr: where Addr is start address to read
	* @param  cmd: where cmd can be CMD_FAST_READ or CMD_NOR_READ
  * @retval 1byte data
  */
unsigned char FMC_ReadByte(FLCTL_T *fmc,unsigned int Addr,unsigned char cmd)
{
	unsigned char bytes_num_w;

	if(cmd == CMD_FAST_READ || cmd == CMD_QREAD || cmd == CMD_2READ || cmd == CMD_DREAD)
		bytes_num_w = 0x05; 
	else if (cmd == CMD_4READ)
        bytes_num_w = 0x07;
    else if (cmd == CMD_NORM_READ)
		bytes_num_w = 0x04;
    else
		bytes_num_w = 0x04;

	fmc->X_FL_CTL = (1<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[0] = cmd;
	fmc->X_FL_WD[1] = (Addr & 0x00ff0000)>>16;
	fmc->X_FL_WD[2] = (Addr & 0x0000ff00)>>8;
	fmc->X_FL_WD[3] = Addr & 0xff;
    fmc->X_FL_WD[4] = 0xff;
    fmc->X_FL_WD[5] = 0xff;
	
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

	return FLCTL_BUFF->X_FL_BUFFER[0];
	 
}
/**
  * @brief  This function is used to read a page size (256 bytes) of data from flash
  * @param  fmc     where fmc is a flash peripheral.
  * @param  Addr    where Addr is start address to read
  * @param  cmd     where cmd can be CMD_FAST_READ or CMD_NORM_READ
  * @retval Internal Buffer address
  */
unsigned char *FMC_ReadPage(FLCTL_T *fmc,unsigned int Addr,unsigned char cmd)
{
	unsigned char bytes_num_w;

	if(cmd == CMD_FAST_READ || cmd == CMD_QREAD || cmd == CMD_2READ || cmd == CMD_DREAD)
		bytes_num_w = 0x05; 
	else if (cmd == CMD_4READ)
        bytes_num_w = 0x07;
    else if (cmd == CMD_NORM_READ)
		bytes_num_w = 0x04;
    else
        bytes_num_w = 0x04;

    fmc->X_FL_CTL = (256<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[0] = cmd;
	fmc->X_FL_WD[1] = (Addr & 0x00ff0000)>>16;
	fmc->X_FL_WD[2] = (Addr & 0x0000ff00)>>8;
	fmc->X_FL_WD[3] = Addr & 0xff;
    fmc->X_FL_WD[4] = 0xff;
    fmc->X_FL_WD[5] = 0xff;
	
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

	return (unsigned char *)&(FLCTL_BUFF->X_FL_BUFFER[0]);
}

/**
  * Read small data (less than 256 bytes) from flash (for internal use).
  */
static unsigned char *FMC_ReadInternal(FLCTL_T *fmc,unsigned int Addr,unsigned char cmd, unsigned int len)
{
	unsigned char bytes_num_w;

	if(cmd == CMD_FAST_READ || cmd == CMD_QREAD || cmd == CMD_2READ || cmd == CMD_DREAD)
		bytes_num_w = 0x05; 
	else if (cmd == CMD_4READ)
        bytes_num_w = 0x07;
    else if (cmd == CMD_NORM_READ)
		bytes_num_w = 0x04;
    else
        bytes_num_w = 0x04;

    fmc->X_FL_CTL = (len<<8) | (bytes_num_w<<0);
	fmc->X_FL_WD[0] = cmd;
	fmc->X_FL_WD[1] = (Addr & 0x00ff0000)>>16;
	fmc->X_FL_WD[2] = (Addr & 0x0000ff00)>>8;
	fmc->X_FL_WD[3] = Addr & 0xff;
    fmc->X_FL_WD[4] = 0xff;
    fmc->X_FL_WD[5] = 0xff;
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};

	return (unsigned char *)&(FLCTL_BUFF->X_FL_BUFFER[0]);
}

/**
  * @brief  This function is used to read data stream from flash
  * @param  fmc     where fmc is a flash peripheral.
  * @param  Addr    where Addr is start address to read
  * @param  cmd     where cmd can be CMD_FAST_READ or CMD_NORM_READ
  * @param  buf     where buf is a buffer to store read data
  * @param  len     where len is data length of bytes to read
  * @retval None
  */
int FMC_ReadStream(FLCTL_T *fmc, unsigned int Addr, unsigned char cmd, unsigned char *buf, unsigned int len)
{
    unsigned int tmp_addr = Addr;
    unsigned int tmp_size = len;
    unsigned int code_area_size = FMC_GetFlashCodeAreaSize(fmc);
    
    if ((Addr >= code_area_size) || (Addr + len > code_area_size))
    {
        return -1;
    }

    while (tmp_size >= 256)
    {
        memcpy(&buf[tmp_addr - Addr], FMC_ReadPage(fmc, tmp_addr, cmd), 256);
        tmp_addr += 256;
        tmp_size -= 256;
    }

    if (tmp_size)
    {
        memcpy(&buf[tmp_addr - Addr], FMC_ReadInternal(fmc, tmp_addr, cmd, tmp_size), tmp_size);
    }

    return 0;
}
/**
  * @brief  This function is used to write data to buffer,
  * @param  fmc_w_buff: where fmc_w_buff is a cache to store write data.
	* @param  size: where size is write data size
	* @param  pData: write data
  * @retval none
  */
static inline uint32_t FMC_PrepBuf(unsigned int size,unsigned char *pData)
{
	uint32_t i;
    uint32_t len = size > 256 ? 256 : size;
	for(i = 0; i < len; i++)
	{
		FLCTL_BUFF->X_FL_BUFFER[i] = *pData;
		pData ++;
	}
    while(i < 256){
        FLCTL_BUFF->X_FL_BUFFER[i] = 0xFF;
        i++;
    }
    return len;
}

void FMC_SetFlashMode(FLCTL_T *fmc, uint32_t mode, bool enhance)
{
    FMC_ExitEnhanceMode(fmc);

    if (mode == FLASH_X1_MODE)
    {
        // set cpu to x1_mode
        fmc->X_FL_X_MODE = (fmc->X_FL_X_MODE & ~0x3);
    }
    else if (mode == FLASH_X2_MODE)
    {
        // set cpu to x2_mode
        fmc->X_FL_X_MODE = ((fmc->X_FL_X_MODE & ~0x3) | FLASH_X2_MODE);

        if (enhance)
        {
            FMC_EnterEnhanceMode(fmc, FLASH_X2_MODE);
        }
    }
    else if (mode == FLASH_X4_MODE)
    {
        uint16_t status;
        // Ensure QE bit is set
        status = FMC_ReadStatusReg(fmc, CMD_READ_STATUS_H);
        while ((status&0x02) != 0x02)
        {
            status = (status << 8) | FMC_ReadStatusReg(fmc, CMD_READ_STATUS_L) | QUAD_ENABLE_Msk;
            FMC_WriteStatusReg(fmc, OPS_WR_STU_REG_ALL, status);
            status = FMC_ReadStatusReg(fmc, CMD_READ_STATUS_H);
        }

        // set cpu to x4_mode
        fmc->X_FL_X_MODE = ((fmc->X_FL_X_MODE & ~0x3) | FLASH_X4_MODE);

        // en_burst_wrap
        fmc->X_FL_X_MODE |= (0x1 << 16);  // 32bit reg

        // issue burst_wrap command to spi flash
        fmc->X_FL_CTL = (0<<8) | (0x05<<0);
        fmc->X_FL_WD[0] = CMD_BURST_READ;
        fmc->X_FL_WD[4] = BURST_READ_MODE_32<<5;
        fmc->X_FL_TRIG = CMD_TRIG;
        while(fmc->X_FL_TRIG){};

        if (enhance)
        {
            FMC_EnterEnhanceMode(fmc, FLASH_X4_MODE);
        }
    }

    /* Disable FMC Auto DP to ensure sleep mode works properly */
    FMC_DisableAutoDp(fmc);
}

/**
  * Write one byte data to flash.
  */
void FMC_WriteByte(FLCTL_T *fmc, unsigned int addr, unsigned char data)
{
    FMC_WriteEnable(fmc);

    fmc->X_FL_CTL = (0x0<<8) | (0x5<<0);
    fmc->X_FL_CONFIG &= ~0x1; //pp inactive
    fmc->X_FL_WD[0] = 0x02; //program one page
    fmc->X_FL_WD[1] = (addr & 0x00ff0000)>>16;
    fmc->X_FL_WD[2] = (addr & 0x0000ff00)>>8;
    fmc->X_FL_WD[3] =  addr & 0xff;
    fmc->X_FL_WD[4] =  data;
    fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
    fmc->X_FL_TRIG = CMD_TRIG;
    while(fmc->X_FL_TRIG){};

    return;
}

/**
  * Write one half-word data to flash.
  */
void FMC_WriteHalfWord(FLCTL_T *fmc, unsigned int addr, unsigned short data)
{
    FMC_WriteEnable(fmc);

    fmc->X_FL_CTL = (0x0<<8) | (0x6<<0);
    fmc->X_FL_CONFIG &= ~0x1; //pp inactive
    fmc->X_FL_WD[0] = 0x02; //program one page
    fmc->X_FL_WD[1] = (addr & 0x00ff0000)>>16;
    fmc->X_FL_WD[2] = (addr & 0x0000ff00)>>8;
    fmc->X_FL_WD[3] =  addr & 0xff;
    fmc->X_FL_WD[4] = data & 0xff;
    fmc->X_FL_WD[5] = (data & 0xff00)>>8;
    fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
    fmc->X_FL_TRIG = CMD_TRIG;
    while(fmc->X_FL_TRIG){};

    return;
}

/**
  * Write small data (equal or less than 256 bytes) to flash one single page (for internal use).
  */
static int FMC_WritePageInternal(FLCTL_T *fmc, unsigned int addr, unsigned int size, unsigned char *buf)
{
    unsigned int OffsetOfCurrPage = addr % 256;
    unsigned int PageStartAddr = addr - OffsetOfCurrPage;

    if (OffsetOfCurrPage + size > 256)
        return -1;

    if(FMC_WriteEnable(fmc) == 0)
        return -1;

    // Fill the 256-Bytes FMC Write Buffer
    for(size_t i = 0; i < 256; i++)
    {
        if ((i >= OffsetOfCurrPage) && (i < OffsetOfCurrPage + size))
            FLCTL_BUFF->X_FL_BUFFER[i] = *(buf++);
        else
            FLCTL_BUFF->X_FL_BUFFER[i] = 0xFF;
    }

    fmc->X_FL_CTL = (0<<8) | (0x04<<0);
    fmc->X_FL_CONFIG |= 0x1; //pp active
    fmc->X_FL_WD[0] = 0x02; //program one page
    fmc->X_FL_WD[1] = (PageStartAddr & 0x00ff0000)>>16;
    fmc->X_FL_WD[2] = (PageStartAddr & 0x0000ff00)>>8;
    fmc->X_FL_WD[3] =  PageStartAddr & 0xff;
    fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
    fmc->X_FL_TRIG = CMD_TRIG;
    while(fmc->X_FL_TRIG){};

    return 0;
}

/**
  * @brief  This function is used to write data stream to flash
  * @param  fmc     where fmc is a flash peripheral
  * @param  Addr    where Addr is start address to write, can be any valid address
  * @param  buf     where buf is a buffer to store data to write
  * @param  len     where len is data length of bytes to write
  * @retval None
  */
int FMC_WriteStream(FLCTL_T *fmc, unsigned int Addr, unsigned char *buf, unsigned int len)
{
    unsigned int OffsetOfFirstPage = Addr % 256;
    unsigned int WriteSizeInFirstPage = (len < (256 - OffsetOfFirstPage)) ? len : (256 - OffsetOfFirstPage);
    unsigned int FirstPageStartAddr = Addr - OffsetOfFirstPage;

    unsigned int OffsetOfLastPage = (Addr + len) % 256;
    unsigned int WriteSizeInLastPage = (len > WriteSizeInFirstPage) ? OffsetOfLastPage : 0;
    unsigned int LastPageStartAddr = (Addr + len) - OffsetOfLastPage;

    unsigned int code_area_size = FMC_GetFlashCodeAreaSize(fmc);

    if ((Addr >= code_area_size) || (Addr + len > code_area_size))
    {
        return -1;
    }

    size_t MiddlePageNum = (LastPageStartAddr > FirstPageStartAddr) ? ((LastPageStartAddr - FirstPageStartAddr) / 256 - 1) : 0;

    uint8_t *tmp_buf = buf;
    uint32_t tmp_addr = Addr;

    // Write first page
    FMC_WritePageInternal(fmc, tmp_addr, WriteSizeInFirstPage, tmp_buf);
    tmp_addr += WriteSizeInFirstPage;
    tmp_buf += WriteSizeInFirstPage;

    // Write middle pages
    while (MiddlePageNum--)
    {
        FMC_WritePageInternal(fmc, tmp_addr, 256, tmp_buf);
        tmp_addr += 256;
        tmp_buf += 256;
    }

    // Write last page
    if (WriteSizeInLastPage)
    {
        FMC_WritePageInternal(fmc, tmp_addr, WriteSizeInLastPage, tmp_buf);
    }

    return 0;
}

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
int FMC_ReadInfoArea(FLCTL_T *fmc, unsigned int Addr, unsigned char cmd, unsigned char *buf, unsigned int len)
{
    if (Addr >= 4096 || (Addr + len) > 4096)
    {
        return -1;
    }

    fmc->X_FL_CONFIG |= BIT1;   //set info_en bit

    FMC_ReadStream(fmc, Addr, cmd, buf, len);

    fmc->X_FL_CONFIG &= ~BIT1;   //clear info_en bit

    return 0;
}

/**
  * @brief  This function is used to write data to the flash 4KB INFO area.
  * @param  fmc     where fmc is a flash peripheral
  * @param  Addr    where Addr is start address to write, can be 0 ~ 4095
  * @param  buf     where buf is a buffer to store data to write
  * @param  len     where len is data length of bytes to write
  * @retval 0   write success
  * @retval -1  write fail
  */
int FMC_WriteInfoArea(FLCTL_T *fmc, unsigned int Addr, unsigned char *buf, unsigned int len)
{
    if (Addr >= 4096 || (Addr + len) > 4096)
    {
        return -1;
    }

    fmc->X_FL_CONFIG |= BIT1;   //set info_en bit

    FMC_WriteStream(fmc, Addr, buf, len);

    fmc->X_FL_CONFIG &= ~BIT1;   //clear info_en bit

    return 0;
}

/**
  * @brief  This function is used to erase the flash 4KB INFO area.
  * @param  fmc where fmc is a flash peripheral
  * @retval 0   erase success
  * @retval -1  erase fail
  */
int FMC_EraseInfoArea(FLCTL_T *fmc)
{
    int ret;

    fmc->X_FL_CONFIG |= BIT1;   //set info_en bit

    ret = FMC_EraseSector(fmc, 0x0);

    fmc->X_FL_CONFIG &= ~BIT1;   //clear info_en bit

    return ret;
}

/**
  * @brief  This function is used to init Icache,
  * @param  fmc: where fmc is a flash peripheral.
  * @param  mode: where mode can be FLASH_X1_MODE
  *                                 FLASH_X2_MODE
  *                                 FLASH_X4_MODE
  * @retval none
  */
void InitIcache(FLCTL_T *fmc, uint32_t mode)
{
	// Disable cache
	CR->X_CACHE_EN = 0x00;

    if ((mode == FLASH_X1_MODE) || (mode == FLASH_X2_MODE))
    {
        CR->X_CACHE_INI |= 0x02;    // flash_has_no_wrap = 1
    }
    else if (mode == FLASH_X4_MODE)
    {
        CR->X_CACHE_INI &= ~0x02;   // flash_has_no_wrap = 0
    }

	// Flush cache, ini_trg = 1
	CR->X_CACHE_INI |= 0x01; //inv_all

	while(CR->X_CACHE_INI & 0x1){};

	// Enable cache
	CR->X_CACHE_EN = 0x01;
}


/*  
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *    Return Value:   0 - OK,  1 - Failed
 */


int BlankCheck (FLCTL_T *fmc,unsigned long adr, unsigned long sz) 
{
    while(sz >= 4){
        if(FMC_ReadWord(fmc,adr, CMD_DREAD) != 0xFFFFFFFF){
            return 1;
        }
        adr += 4;
        sz -= 4;
    }
    while(sz){
        if(FMC_ReadByte(fmc,adr, CMD_DREAD) != 0xFF){
            return adr;
        }
        adr++;
        sz--;
    }
    return 0;    
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
int ProgramPage (FLCTL_T *fmc,unsigned long adr, unsigned long sz, unsigned char *buf) 
{
	unsigned char offset = 0;
    unsigned int WriteSize = 0;
	WriteSize = FMC_PrepBuf(sz,buf);
    do{	
        offset = 0;
        if(FMC_WriteEnable(fmc) == 0)
            return -1;
        fmc->X_FL_CTL = (0<<8) | (0x04<<0);
        fmc->X_FL_CONFIG |= 0x1; //pp active
        fmc->X_FL_WD[offset++] = 0x02;		//program one page
        fmc->X_FL_WD[offset++] = (adr & 0x00ff0000)>>16;
        fmc->X_FL_WD[offset++] = (adr & 0x0000ff00)>>8;
        fmc->X_FL_WD[offset++] =  adr & 0xff;
        fmc->X_FL_X_MODE |= (0x01<<Long_Time_Op_Pos);
        fmc->X_FL_TRIG = CMD_TRIG;
        while(fmc->X_FL_TRIG){};

        sz -= WriteSize;
        buf += WriteSize;
        adr += 256;
        if(sz){
            /* Fill data for next program */
            WriteSize = FMC_PrepBuf(sz, buf);
        }
//        while(FMC_ReadStatusReg(fmc,CMD_READ_STATUS_L) & Write_In_Process_Msk);
    }while(sz);

  return 0;                                           // Done
}

/*
 *  Verify in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data, note the buf address should be 4-bytes-alingned
 *    Return Value:   final verify address
 */
unsigned long Verify (FLCTL_T *fmc,unsigned long adr, unsigned long sz, unsigned char *buf) 
{	    
    while(sz >= 4){
        if(FMC_ReadWord(fmc,adr, CMD_DREAD) != *((__I uint32_t*)buf)){
            return adr;
        }
        adr += 4;
        sz -= 4;
        buf += 4;
    }
    while(sz){
        if(FMC_ReadByte(fmc,adr, CMD_DREAD) != *buf){
            return adr;
        }
        adr++;
        sz--;
        buf++;
    }
    return adr;	
}
/**
  * @brief        Calculate and read the CRC32 checksum of a specified flash area.
  * @param[in]    addr     Start address of the flash area to be executed CRC32 checksum calculation.
  * @param[in]    count    Number of bytes to be calculated.
  * @param[out]   chksum   If success, it will contain the result of CRC32 checksum calculation.
  * @retval   0   Success
  * @retval   -1  Invalid parameter.
  */
int32_t FMC_GetCrc32Sum(FLCTL_T *fmc,uint32_t addr, uint32_t count, uint32_t *chksum)
{
	unsigned char offset =0;

    // TODO:
    // There is limitation for flash READ cmd (0x03) used here:
    // This command could only use when flash clock < 50MHz (for GD flash)
	fmc->X_FL_WD[offset++] = 0x03;
	fmc->X_FL_CTL = (0x4<<0) | (count<<8);
	//set addr
	fmc->X_FL_WD[offset++] = (addr&0xff0000)>>16;
	fmc->X_FL_WD[offset++] = (addr&0xff00)>>8;
	fmc->X_FL_WD[offset++] = addr&0xff;
	//set crc32 en
	fmc->X_FL_X_MODE |= (0x1<<19);
	
	//set trig
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};
	
	//get check sum
    *chksum = *((uint32_t*)FLCTL_BUFF->X_FL_BUFFER);
	//crc32 disable
	fmc->X_FL_X_MODE &= ~(0x1<<19);	
	return 0;
}

// Exit enhance read mode
void FMC_ExitEnhanceMode(FLCTL_T *fmc)
{
	fmc->X_FL_WD[0] = 0xff;
	fmc->X_FL_WD[1] = 0xff;
	fmc->X_FL_WD[2] = 0xff;
	fmc->X_FL_WD[3] = 0xff;
    fmc->X_FL_CTL = (0<<8) | (4<<0);
    fmc->X_FL_X_MODE &= ~(1 << 18);
	fmc->X_FL_TRIG = 0X01;
	while(fmc->X_FL_TRIG){};
}

// Enter enhance read mode
// Note: Some Flash does not support 2-line enhance mode
void FMC_EnterEnhanceMode(FLCTL_T *fmc, uint32_t mode)
{
    if (mode == FLASH_X2_MODE)
    {
        fmc->X_FL_WD[0]   = 0xBB;
        fmc->X_FL_CTL = (1<<8) | (5<<0);
    }
    else if (mode == FLASH_X4_MODE)
    {
        fmc->X_FL_WD[0]   = 0xEB;
        fmc->X_FL_CTL = (1<<8) | (7<<0);
    }

    fmc->X_FL_WD[4]   = 0xAF;  //8'b10_10_1111 for both GD & PUYA flash
	fmc->X_FL_X_MODE |= (0x1 << 18);
	fmc->X_FL_TRIG = CMD_TRIG;
	while(fmc->X_FL_TRIG){};
}

void FMC_EnableAutoDp(FLCTL_T *fmc)
{
    fmc->X_FL_DP_CTL |= 1u;
}

void FMC_DisableAutoDp(FLCTL_T *fmc)
{
    fmc->X_FL_DP_CTL &= ~1u;
}

void FMC_SetRdpWaitCount(FLCTL_T *fmc, uint32_t cnt)
{
    fmc->X_FL_DP_CTL &= ~(0xFFFFFFu << 8);
    fmc->X_FL_DP_CTL |= cnt << 8;
}
