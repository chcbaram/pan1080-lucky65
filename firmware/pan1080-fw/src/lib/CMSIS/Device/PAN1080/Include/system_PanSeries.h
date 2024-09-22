/**************************************************************************//**
 * @file     system_PanSeries.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/26 10:19a $
 * @brief    Panchip series system clock definition file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYSTEM_PANSERIES_H__
#define __SYSTEM_PANSERIES_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union _adc_data
{
    uint8_t d8[3];
    struct
    {
        uint8_t c_adc_low_8;        // d8[0]
        uint8_t v_adc_low_4 : 4;    // d8[1] (bit 0 ~ 3)
        uint8_t c_adc_high_4 : 4;   // d8[1] (bit 4 ~ 7)
        uint8_t v_adc_high_8;       // d8[2]
    } __attribute__((packed)) m;
} ADC_DATA_T;

typedef union _otp_struct
{
    uint8_t d8[0x100];
    struct
    {
        uint8_t rsvd0[2];           // offset: 0x0, 0x1
        uint8_t chip_ver;           // offset: 0x2
        uint8_t cp_version;         // offset: 0x3
        uint8_t uid[8];             // offset: 0x4 ~ 0xB
        uint8_t wafer_number;       // offset: 0xC
        uint8_t ana_ldo : 4;        // offset: 0xD (bit 0 ~ 3)
        uint8_t rffe_ldo : 4;       // offset: 0xD (bit 4 ~ 7)
        uint8_t fsync_ldo : 4;      // offset: 0xE (bit 0 ~ 3)
        uint8_t rxadc_ldo : 4;      // offset: 0xE (bit 4 ~ 7)
        uint8_t vco_ldo : 4;        // offset: 0xF (bit 0 ~ 3)
        uint8_t temp_low : 4;       // offset: 0xF (bit 4 ~ 7)
        uint8_t temp_high;          // offset: 0x10
        uint8_t c_rch;              // offset: 0x11
        uint32_t f_rch;             // offset: 0x12, 0x13, 0x14, 0x15
        uint8_t rcl_coarse : 6;     // offset: 0x16 (bit 0 ~ 5)
        uint8_t rsvd1 : 2;          // offset: 0x16 (bit 6 ~ 7)
        uint8_t rcl_fine : 6;       // offset: 0x17 (bit 0 ~ 5)
        uint8_t rsvd2 : 2;          // offset: 0x17 (bit 6 ~ 7)
        uint8_t rsvd3 : 4;          // offset: 0x18 (bit 0 ~ 3)
        uint8_t rcl_rsvd_1 : 3;     // offset: 0x18 (bit 4 ~ 6)
        uint8_t rsvd3_1 : 1;        // offset: 0x18 (bit 7)
        uint16_t f_rcl;             // offset: 0x19, 0x1A
        uint8_t mac_addr[6];        // offset: 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20
        uint8_t c_lldo : 3;         // offset: 0x21 (bit 0 ~ 2)
        uint8_t rsvd4: 1;           // offset: 0x21 (bit 3)
        uint8_t v_lldo_low : 4;     // offset: 0x21 (bit 4 ~ 7)
        uint8_t v_lldo_high;        // offset: 0x22
        uint8_t c_hldo : 4;         // offset: 0x23 (bit 0 ~ 3)
        uint8_t v_hldo_low : 4;     // offset: 0x23 (bit 4 ~ 7)
        uint8_t v_hldo_high;        // offset: 0x24
        uint8_t c_vbg : 3;          // offset: 0x25 (bit 0 ~ 2)
        uint8_t rsvd5: 1;           // offset: 0x25 (bit 3)
        uint8_t v_vbg_low : 4;      // offset: 0x25 (bit 4 ~ 7)
        uint8_t v_vbg_high;         // offset: 0x26
        uint8_t c_adcvbg : 6;       // offset: 0x27 (bit 0 ~ 5)
        uint8_t rsvd6: 2;           // offset: 0x27 (bit 6 ~ 7)
        uint16_t v_adcvbg;          // offset: 0x28, 0x29
        uint8_t rsvd7 : 4;          // offset: 0x2A (bit 0 ~ 3)
        uint8_t c_temp_low : 4;     // offset: 0x2A (bit 4 ~ 7)
        uint8_t c_temp_high;        // offset: 0x2B
        uint8_t rsvd8[4];           // offset: 0x2C ~ 0x2F
        uint8_t adc_data_vbgcode_00[3]; // offset: 0x30 ~ 0x32
        uint8_t adc_data_vbgcode_3f[3]; // offset: 0x33 ~ 0x35
        uint8_t c_vdd_4_high;       // offset: 0x36
        uint8_t v_bat_low : 4;      // offset: 0x37 (bit 0 ~ 3)
        uint8_t c_vdd_4_low: 4;     // offset: 0x37 (bit 4 ~ 7)
        uint8_t v_bat_high;         // offset: 0x38
        uint16_t adc_vdd_k;         // offset: 0x39, 0x3A
        uint16_t adc_vdd_b;         // offset: 0x3B, 0x3C
        uint8_t c_bodref : 3;       // offset: 0x3D (bit 0 ~ 2)
        uint8_t rsvd9: 1;           // offset: 0x3D (bit 3)
        uint8_t v_bodref_low : 4;   // offset: 0x3D (bit 4 ~ 7)
        uint8_t v_bodref_high;      // offset: 0x3E
        uint16_t adc_vbg_k;         // offset: 0x3F, 0x40
        uint16_t adc_vbg_b;         // offset: 0x41, 0x42
        uint8_t lldo_p56;           // offset: 0x43
        uint8_t lldo_kscan;         // offset: 0x44
        uint8_t lldo_timer;         // offset: 0x45
        uint8_t lldo_m0;            // offset: 0x46
        uint8_t id_buck_vc : 3;     // offset: 0x47 (bit 0 ~ 2)
        uint8_t rsvd10: 5;          // offset: 0x47 (bit 3 ~ 7)
        uint8_t id_buck_i : 5;      // offset: 0x48 (bit 0 ~ 4)
        uint8_t rsvd11: 3;          // offset: 0x48 (bit 5 ~ 7)
        uint8_t id_buck_freq;       // offset: 0x49
        uint8_t id_buck_refcal : 5; // offset: 0x4A (bit 0 ~ 4)
        uint8_t rsvd12 : 3;         // offset: 0x4A (bit 5 ~ 7)
        uint8_t c_adcvbg1 : 6;      // offset: 0x4B (bit 0 ~ 5)
        uint8_t rsvd13: 2;          // offset: 0x4B (bit 6 ~ 7)
        uint16_t v_adcvbg1;         // offset: 0x4C, 0x4D
        uint8_t sram_test_flag;     // offset: 0x4E
        uint8_t rsvd14 : 4;         // offset: 0x4F (bit 0 ~ 3)
        uint8_t c_temp_low_1 : 4;   // offset: 0x4F (bit 4 ~ 7)
        uint8_t c_temp_high_1;      // offset: 0x50
        uint8_t rf1;                // offset: 0x51
        uint8_t ft_version;         // offset: 0x52
        uint8_t cp_checksum;        // offset: 0x53
        uint8_t ft_checksum;        // offset: 0x54
        uint8_t rsvd15[11];         // offset: 0x55 ~ 0x5F
        uint8_t adc_data_refvbg[33]; // offset: 0x60 ~ 0x80
        uint8_t rsvd16[0x3E];       // offset: 0x81 ~ 0xBE
        uint8_t feature_ctrl;       // offset: 0xBF
        uint8_t aes_key[16];        // offset: 0xC0 ~ 0xCF
        uint8_t secure_enabled : 1; // offset: 0xD0 (bit 0)
        uint8_t anti_inject_en : 1; // offset: 0xD0 (bit 1)
        uint8_t rsvd17 : 6;         // offset: 0xD0 (bit 2 ~ 7)
        uint8_t flash_enc_ofst[3];  // offset: 0xD1, 0xD2, 0xD3
        uint8_t user_ro_data[20];   // offset: 0xD4 ~ 0xE7
        uint8_t misoperate_flag;    // offset: 0xE8
        uint8_t user_rw_data[23];   // offset: 0xE9 ~ 0xFF
    } __attribute__((packed)) m;
} OTP_STRUCT_T;

extern OTP_STRUCT_T otp;
extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern float SocTemperature;

/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */

extern void SystemCoreClockUpdate (void);
extern void SystemInit (void);
extern bool SystemHwParamLoader(void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_PANSERIES_H__


/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
