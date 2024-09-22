/**************************************************************************//**
* @file     pan_hal_wwdt.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series WWDT (Windowed Watchdog Timer) HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "pan_hal.h"


WWDT_HandleTypeDef WWDT_Handle_Array[1];

void HAL_WWDT_Init(WWDT_HandleTypeDef *wwdt){

    // Unlock Regs
    SYS_UnlockReg();
    // Select Clock Source
    if(wwdt->InitObj.ClockSource!=0){
        CLK_SetWwdtClkSrc(wwdt->InitObj.ClockSource);
    }

    // Delay some nops before Re-lock Regs
    SYS_delay_10nop(1);
    // Re-Lock Regs
    SYS_LockReg();
    // Enable WWDT, disable Int
    WWDT_Open(wwdt->InitObj.Prescaler, wwdt->InitObj.CmpValue, false);
    // WWDT->CTL = wwdt->WWDT_InitObj.Prescaler |
    //         (wwdt->WWDT_InitObj.CmpValue << WWDT_CTL_CMPDAT_Pos)|
    //         WWDT_CTL_WWDTEN_Msk |
    //         (false ? WWDT_CTL_INTEN_Msk : 0);
}
void HAL_WWDT_DeInit(WWDT_HandleTypeDef *wwdt){
    
}
void HAL_WWDT_Feed(WWDT_HandleTypeDef *wwdt){
    WWDT_ReloadCounter();
}
void HAL_WWDT_Init_INT(WWDT_HandleTypeDef *wwdt){
    
    WWDT_Open(wwdt->InitObj.Prescaler, wwdt->InitObj.CmpValue, true);
    // Enable NVIC WWDT IRQ
    NVIC_EnableIRQ(WWDT_IRQn);
}
void HAL_WWDT_DeInit_INT(WWDT_HandleTypeDef *wwdt){
    wwdt->Callback=NULL;
    WWDT_Open(wwdt->InitObj.Prescaler, wwdt->InitObj.CmpValue, false);
    // Disable NVIC WWDT IRQ, thus WWDT will stop reloading counter
    NVIC_DisableIRQ(WWDT_IRQn);
}

void WWDT_IRQHandler(void)
{
    // Clear Timeout Int Flag
    WWDT_ClearIntFlag();

    // Pulse LA_INPUT_PIN to indicate WWDT Int occurred
    // Reload Counter

    WWDT_ReloadCounter();
    if (WWDT_OBJ.Callback != NULL)
        WWDT_OBJ.Callback();
}
