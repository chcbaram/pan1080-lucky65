/**************************************************************************//**
* @file     pan_hal_wdt.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series WDT (Watchdog Timer) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "pan_hal.h"

WDT_HandleTypeDef WDT_Handle_Array[1];
void HAL_WDT_Init(WDT_HandleTypeDef *wdt)
{
    // Unlock Regs before Enable WDT as several WDT Regs are write-protected
    SYS_UnlockReg();

    // Select Clock Source
    CLK_SetWdtClkSrc(wdt->InitObj.ClockSource);

    // Enable WDT, ENABLE Reset Mode, disable Wakeup Signal
    WDT_Open(wdt->InitObj.Timeout, wdt->InitObj.ResetDelay, wdt->InitObj.ResetSwitch, wdt->InitObj.WakeupSwitch);

    // Re-lock Regs
    SYS_LockReg();
}
void HAL_WDT_DeInit(WDT_HandleTypeDef *wdt)
{
    SYS_UnlockReg();
    // Disable WDT, need to ensure Regs are unlocked
    WDT_Close();
    // Re-lock Regs
    SYS_LockReg();
}
void HAL_WDT_Feed(WDT_HandleTypeDef *wdt)
{
    SYS_UnlockReg();
    WDT_ResetCounter();
    SYS_LockReg();
}
void HAL_WDT_Init_INT(WDT_HandleTypeDef *wdt)
{
    // Unlock Regs
    SYS_UnlockReg();
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);
    // Re-Lock Regs
    SYS_LockReg();
}

void HAL_WDT_DeInit_INT(WDT_HandleTypeDef *wdt)
{
    // Unlock Regs
    SYS_UnlockReg();
    wdt->Callback=NULL;
    WDT_DisableInt();
    // Re-Lock Regs
    SYS_LockReg();
}

void WDT_IRQHandler(void)
{
    // Clear Timeout Int Flag
    WDT_ClearTimeoutIntFlag();

    // Pulse LA_INPUT_PIN to indicate WDT Int occurred

    // SYS_TEST("--WDT INT\n");
    if (WDT_OBJ.Callback != NULL)
    {
        WDT_OBJ.Callback();
    }


}
