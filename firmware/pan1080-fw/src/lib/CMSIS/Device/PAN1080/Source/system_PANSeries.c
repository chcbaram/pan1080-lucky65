/**************************************************************************//**
 * @file     system_PanSeries.c
 * @version  V1.00
 * @brief    Panchip series SoC system clock init code and assert handler
 * @note
 * Copyright (C) 2020 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#include "PanSeries.h"
#include "pan_clk.h"
#include "pan_fmc.h"
#include "pan_sys.h"
#include "pan_efuse.h"

/*
 * Global variables defined here should be used after RAM RW/ZI data init.
 */
uint32_t SystemCoreClock;
OTP_STRUCT_T otp;
float SocTemperature = 22; /* Treat default SoC temperature as 22℃ */

/*
 * Global variables here to be used in SystemInit() function should reside
 * in flash, since RAM RW/ZI data are not initialized at that time.
 */
const uint32_t PanFlashLineMode = CONFIG_FLASH_LINE_MODE;
#if (CONFIG_PM)
const bool PanFlashEnhanceEnable = false;
#else
const bool PanFlashEnhanceEnable = false; // Disable flash enhance mode to save power
#endif

void SystemInit(void)
{
    // (Workaround) Maximum HLDO (DVDD) electrical level to enhance system stability
    // ANA->LP_HP_LDO = (ANA->LP_HP_LDO & ~0x2) | (0xf << 3);

    // Use external DVDD capacitor
    ANA->LP_HP_LDO |= BIT9;

    // Flash io pullup enable
    ANA->ANA_MISC1 |= (0x3b << 14);

    // Clear Retention Ctrl Flag (BIT31 of GPIO_DBCTL) in GPIO module to make sure
    // IOMUX is valid to use when soc wakes up from standby mode 1.
    GPIO_DB->DBCTL &= ~BIT31;

    // fix bug 227: j-link debug failed when XTL selected
    if ((CLK->XTL_CTRL & (1 << 24)))
    {
        // XTL_EN
        CLK->CLK_TOP_CTRL |= CLK_TOPCTL_XTL_EN_Msk;
    }

    // Set rx_neg to 1, which can support Flash Clock up to 80MHz
    // (We may need to do some tuning if Flash Clock is greater than 80MHz)
    FLCTL->X_FL_X_MODE |= (1 << 20);    //rx_neg
    // Switch flash to 4-Line mode
    FMC_SetFlashMode(FLCTL, PanFlashLineMode, PanFlashEnhanceEnable);

    // Speed up flash clock from default 4MHz to 32MHz
    CLK_SetFlashClkDiv(CLK_FLASH_CLKDIV_1);
    // Init I-Cache
    InitIcache(FLCTL, PanFlashLineMode);

#if defined(CONFIG_MCUBOOT)

    uint32_t *ptr_dcdc_flag = (uint32_t *)0x2000fffc;

    if(*ptr_dcdc_flag == 0x12345678){
        ANA->RSV1 |= BIT17; // To avoid potential buck enabling fail right after POR
        /* Configure DCDC params in normal mode */
        uint32_t tmp_reg_dcdc = ANA->LP_BUCK;

        tmp_reg_dcdc &= ~0X1;
        tmp_reg_dcdc = (tmp_reg_dcdc & ~(0X7<<5)) | (0X5<<5);
        ANA->LP_BUCK = tmp_reg_dcdc;
    } else {
        /* do nothing default ldo mode */
    }
#endif

#if CONFIG_PM
    /*
     * Enable standby IRQ as early as we can in order to execute corresponding
     * isr earlier to resume system from waking up from hw standby mode.
     * NOTE:
     *   1. We enable standby IRQ after cache init, thus we can execute
     *      standby ISR faster.
     *   2. We enable standby IRQ before the auto 3v-register-sync feature enabled,
     *      thus we can load the backup-to-ram aon register values back to related
     *      registers, so these values can be correctly load to 3v area after the
     *      auto 3v-reg-sync mechanism enabled.
     */
	// NVIC_EnableIRQ(STANDBY_IRQn);
    /* Insure we are going to enter standby isr before the Auto-3v-sync feature enabled */
    // SYS_delay_10nop(1);
#endif

#ifndef SYNC_3V_REG_MANUALLY
    /* Here we enable the auto 3v-register-sync mechanism to simplify
     * operations for registers which have copies in the 3v always-on
     * area so that content in these registers could retain even in
     * low power mode.
     * NOTE: The auto 3v-reg-sync mechanism works for all related
     *       registers except Px_DINOFF regs for P46/P47/P56 in GPIO
     *       module.
     */
    CLK_Set3vSyncAuto();
#endif

    // Enable DCDC in normal mode when CONFIG_SOC_DCDC_PAN1080 enabled
#if (CONFIG_SOC_DCDC_PAN1080)
    ANA->RSV1 |= BIT17; // To avoid potential buck enabling fail right after POR
    /* Configure DCDC params in normal mode */
    uint32_t tmp_reg = ANA->LP_BUCK;
    tmp_reg &= ~0X1;
    tmp_reg = (tmp_reg & ~(0X7<<5)) | (0X5<<5);
    ANA->LP_BUCK = tmp_reg;
#endif

    // Enable DCDC in lowpower mode only if:
    //  - CONFIG_SOC_DCDC_PAN1080 enabled
    //  - and CONFIG_PM enabled
#if (CONFIG_SOC_DCDC_PAN1080 && CONFIG_PM)
	/* Configure DCDC params in lowpower mode */
	uint32_t tmp_reg1 = ANA->LP_FL_CTRL;
	tmp_reg1 &= ~(0x6 << 12);
	ANA->LP_FL_CTRL = tmp_reg1;
#endif

    /*
     * Set this bit to make sure we can later switch pinmux of P43 to the function
     * PAN1080_PIN_FUNC_P43_UART0_TX to use for UART0 Tx.
     */
    SYS->CTRL0 |= BIT2;

    /*
     * Disable eFuse VDD for power saving purpose
     */
    EFUSE->EF_VDD = 0;
}


void SystemCoreClockUpdate(void)
{
    uint32_t div = 0;
    uint32_t freq_out = 0;
    div = (CLK->CLK_TOP_CTRL & CLK_TOPCTL_AHB_DIV_Msk) >> CLK_TOPCTL_AHB_DIV_Pos;
    if(div == 0)
        div = 1;
    else
        div += 1;

    if(CLK->CLK_TOP_CTRL & CLK_SYS_SRCSEL_DPLL){
        freq_out = (CLK->DPLL_CTRL & CLK_DPLLCTL_FREQ_OUT_Msk);
        if(freq_out == CLK_DPLL_OUT_64M){
            SystemCoreClock = FREQ_64MHZ  / div;
        }
        else{
            SystemCoreClock = FREQ_48MHZ / div;
        }
    }
    else{
        SystemCoreClock = FREQ_32MHZ / div;
    }
}

/*============================ CRC8 Algorithm ============================*/
#define CRC8_INIT  0xff
#define CRC8_XOROT 0xff

static uint8_t CRC_Bitrev(uint8_t input, uint8_t len)
{
    uint8_t i;
    uint8_t var;
    var = 0;
    for(i=0;i<len;i++)
    {
        if(input & 0x01)
        {
            var |= 1<<(len-1-i);
        }
        input>>=1;
    }
    return var;
}

static void CRC_CalcCrc8Table(uint8_t formula, uint8_t *CRC8_Table)
{
    uint32_t i,j;
    uint32_t crc8;

    formula=CRC_Bitrev(formula,8);
    for(i=0; i<256; i++)
    {
        crc8 = i;
        for (j=0; j<8; j++)
        {
            if(crc8&1)
            {
                crc8=formula^(crc8>>1);
            }
            else
            {
                crc8=crc8>>1;
            }
        }
        CRC8_Table[i] = crc8;
    }
}

static uint8_t CRC_CalcCrc8(uint8_t *pStart, uint32_t uSize, uint8_t *CRC8_Table)
{
  uint32_t uCRCValue;
  uint8_t *pData;

  /* init the start value */
  uCRCValue = CRC8_INIT;
  pData = pStart;

  /* calculate CRC */
  while (uSize --)
  {
    uCRCValue = CRC8_Table[(uCRCValue ^ *pData++)];
  }
  /* XOR the output value */
  return uCRCValue ^ CRC8_XOROT;
}
/*========================= END of CRC8 Algorithm =========================*/

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

bool SystemHwParamLoader(void)
{
    OTP_STRUCT_T otp_tmp;
    bool clk_efuse = true;

    if (!(CLK->AHB_CLK_CTRL & CLK_AHBPeriph_EFUSE))
    {
        clk_efuse = false;
        CLK->AHB_CLK_CTRL |= CLK_AHBPeriph_EFUSE;
    }
    SYS_UnlockReg();
    EFUSE_Init(EFUSE);
    for (size_t i = 0; i < sizeof(otp_tmp); i++)
    {
        EFUSE->EF_CMD   = EFUSE_CMD_READ;
        EFUSE->EF_ADDR  = i;
        EFUSE->EF_TRG   = EFUSE_TRG_EFUSETRG_GO_Msk;
        while(EFUSE->EF_TRG){}
        if (EFUSE->EF_OP_ERROR)
        {
            EFUSE->EF_OP_ERROR = 1u;
            otp_tmp.d8[i] = 0x0;
            continue;
        }
        otp_tmp.d8[i] = EFUSE->EF_DAT;
    }
    EFUSE_UnInit(EFUSE);
    SYS_UnlockReg();
    if (!clk_efuse)
    {
        clk_efuse = true;
        CLK->AHB_CLK_CTRL &= ~CLK_AHBPeriph_EFUSE;
    }
    memcpy(&otp, &otp_tmp, sizeof(otp_tmp));

    /*
     * NOTE: We should make sure flash enhance mode is disabled before read
     * data from flash.
     */
    FMC_ReadInfoArea(FLCTL, 0x0, CMD_FAST_READ, (uint8_t *)&otp_tmp, sizeof(otp_tmp));
    FMC_GetFlashUniqueId(FLCTL);
    FMC_GetFlashJedecId(FLCTL);

    uint8_t* crc8_table = (uint8_t*)FLCTL_BASE1; /* Temporarily use FMC Buffer as crc8 table array */
    uint8_t crc8_info, crc8_calc;
    /* Valid ft_version: 0x1 ~ 0xFE */
    if (otp_tmp.m.ft_version != 0xFF)	// Check ft_version in INFO
    {
        /* Do INFO crc8 check if ft_version >= 7 */
        if (otp_tmp.m.ft_version >= 0x7)
        {
            crc8_info = otp_tmp.m.ft_checksum; /* Read ft checksum out */
            CRC_CalcCrc8Table(0x31, crc8_table); /* Init CRC8 table */
            otp_tmp.m.ft_checksum = 0xFF; /* Ignore checksum itself */
            crc8_calc = CRC_CalcCrc8(otp_tmp.d8, 256, crc8_table); /* Calc checksum */
            otp_tmp.m.ft_checksum = crc8_info; /* Restore checksum val */
            if (crc8_info != crc8_calc)
            {
                return false;
            }
        }
        /* Prefer to use data in INFO area if there is */
        memcpy(&otp.d8[0x03], &otp_tmp.d8[0x03], 0x5E - 0x03);  // offset 0x03 ~ 0x5D
        memcpy(&otp.d8[0x60], &otp_tmp.d8[0x60], 0x81 - 0x60);  // offset 0x60 ~ 0x80
    }
    else if (otp.m.ft_version != 0x00)	// Check ft_version in eFuse
    {
        crc8_info = otp.m.ft_checksum; /* Read ft checksum out */
        CRC_CalcCrc8Table(0x31, crc8_table); /* Init CRC8 table */
        otp.m.ft_checksum = 0xFF; /* Ignore checksum itself */
        crc8_calc = CRC_CalcCrc8(otp.d8, 256, crc8_table); /* Calc checksum */
        otp.m.ft_checksum = crc8_info; /* Restore checksum val */
        if (crc8_info != crc8_calc)
        {
            return false;
        }
    }
    else
    {
        /* Exit if we cannot find valid ft_verison neither in eFuse nor in INFO area */
        return false;
    }

    ANA->ANA_RFFE_LDO = (ANA->ANA_RFFE_LDO & ~(0xFu << 3)) | (otp.m.rffe_ldo << 3);
    ANA->LP_ANA_LDO = (ANA->LP_ANA_LDO & ~(0xFu << 3)) | (otp.m.ana_ldo << 3);
    ANA->ANA_ADC_LDO = (ANA->ANA_ADC_LDO & ~(0xFu << 3)) | (otp.m.rxadc_ldo << 3);
    ANA->LP_FSYN_LDO = (ANA->LP_FSYN_LDO & ~(0xFu << 3)) | (otp.m.fsync_ldo << 3);
    ANA->ANA_VCO_LDO = (ANA->ANA_VCO_LDO & ~(0xFu << 3)) | (otp.m.vco_ldo << 3);
    CLK->RCH_CTRL = (CLK->RCH_CTRL & ~0xFFu) | otp.m.c_rch;
    if (otp.m.ft_version != 0x1)
    {
        CLK->RCL_CTRL = (CLK->RCL_CTRL & ~0xFFFu & ~(0x7u << 14)) | (otp.m.rcl_rsvd_1 << 14) | (otp.m.rcl_fine << 6) | otp.m.rcl_coarse;
    }
#if 1
    ANA->LP_LP_LDO = (ANA->LP_LP_LDO & ~(0x7u << 1)) | ((MAX(otp.m.lldo_p56, otp.m.lldo_kscan) & 0x7u) << 1);
#else
    ANA->LP_LP_LDO = (ANA->LP_LP_LDO & ~(0x7u << 1)) | (otp.m.c_lldo << 1);
#endif
    ANA->LP_HP_LDO = (ANA->LP_HP_LDO & ~(0xFu << 3)) | (otp.m.c_hldo << 3);
    ANA->LP_PTAT_POLY = (ANA->LP_PTAT_POLY & ~(0x7u << 3)) | (otp.m.c_vbg << 3);
    ANA->LP_MISC = (ANA->LP_MISC & ~(0x7u << 1)) | (otp.m.c_bodref << 1);
    ANA->LP_BUCK = (ANA->LP_BUCK & ~(0x7u << 5) & ~(0x7u << 11) & ~(0x1Fu << 17) & ~(0x1Fu << 27) & ~(0x1Fu << 22))
            | (otp.m.id_buck_vc << 5) | ((otp.m.id_buck_freq >> 5) << 11) | ((otp.m.id_buck_freq & 0x1Fu) << 17) | (otp.m.id_buck_i << 27) | (otp.m.id_buck_refcal << 22);

#ifdef SYNC_3V_REG_MANUALLY
    CLK_Wait3vSyncReady();
#endif

    return true;
}

#ifdef ASSERT_ON
/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 * @return     None
 *
 * @details    The function prints the source file name and line number where
 *             the SYS_ASSERT() error occurs, and then stops in an infinite loop.
 */
void SYS_AssertError(uint8_t* file, uint32_t line)
{
    printf("System Assert Error! file: %s, line %ld\n", file, line);

    while(1);
}
#endif
