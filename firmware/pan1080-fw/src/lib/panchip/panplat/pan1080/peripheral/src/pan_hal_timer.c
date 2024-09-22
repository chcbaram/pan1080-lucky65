/**************************************************************************//**
* @file     pan_hal_timer.c
* @version  V0.0.0
* $Revision: 1 $
* $Date:    23/09/10 $
* @brief    Panchip series Timer HAL source file.
* @note
* Copyright (C) 2023 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

#include "pan_hal.h"



TIMER_HandleTypeDef TIMER_Handle_Array[3] =
{
    // TIMER0 Configuration
    {
        .TIMERx = TIMER0,  // Assuming TIMER0 is defined somewhere in your code
        .InitObj = {0},  // Initialize with default values or specify them
        .InterruptObj = {0},  // Initialize with default values or specify them
        .IRQn = TMR0_IRQn,  // Assuming TIMER0_IRQn is defined as an IRQn_Type for TIMER0
        .Callback = NULL,  // Using the dummy callback
    },
    // TIMER1 Configuration
    {
        .TIMERx = TIMER1,  // Assuming TIMER1 is defined somewhere in your code
        .InitObj = {0},  // Initialize with default values or specify them
        .InterruptObj = {0},  // Initialize with default values or specify them
        .IRQn = TMR1_IRQn,  // Assuming TIMER1_IRQn is defined as an IRQn_Type for TIMER1
        .Callback = NULL,  // Using the dummy callback
    },
    // TIMER2 Configuration
    {
        .TIMERx = TIMER2,  // Assuming TIMER2 is defined somewhere in your code
        .InitObj = {0},  // Initialize with default values or specify them
        .InterruptObj = {0},  // Initialize with default values or specify them
        .IRQn = TMR2_IRQn,  // Assuming TIMER2_IRQn is defined as an IRQn_Type for TIMER2
        .Callback = NULL,  // Using the dummy callback
    }
};

void HAL_TIMER_Init(TIMER_HandleTypeDef *timer)
{
    TIMER_SetCmpValue(timer->TIMERx, timer->InitObj.CmpValue); 
    if(timer->InitObj.Mode == TIMER_MODE_BASECNT)
    {
        TIMER_Open(timer->TIMERx, timer->InitObj.CntMode, timer->InitObj.Freq);
    }  
    else if(timer->InitObj.Mode == TIMER_MODE_EVENTCNT){
        // Counter increase on falling edge
        // Set Timer Mode
        TIMER_SetCountingMode(timer->TIMERx, timer->InitObj.CntMode);
        // Set prescale
        TIMER_SetPrescaleValue(timer->TIMERx, timer->InitObj.Prescale);
        TIMER_EnableEventCounter(timer->TIMERx, timer->InitObj.EvtCntEdge);
    }
    else if(timer->InitObj.Mode == TIMER_MODE_INCAP){
        TIMER_Open(timer->TIMERx, timer->InitObj.CntMode, timer->InitObj.Freq);
        // Set capture source to ext PIN
        TIMER_SetCaptureSource(timer->TIMERx, timer->InitObj.CapSrc);
        // Configure Timer capture mode and capture edge
        TIMER_EnableCapture(timer->TIMERx, timer->InitObj.CapMode, timer->InitObj.CapEdge);
    }
    else if(timer->InitObj.Mode == TIMER_MODE_WAKEUP){
        // Set Timer Mode to One Shot Mode
        TIMER_SetCountingMode(timer->TIMERx, TIMER_ONESHOT_MODE);
        // Set prescale
        TIMER_SetPrescaleValue(timer->TIMERx, timer->InitObj.Prescale);
        // Enable wakeup function
        TIMER_EnableWakeup(timer->TIMERx);
    }
    //TIMER_SetCmpValue(timer->TIMERx, timer->InitObj.CmpValue); 

}

void HAL_TIMER_DeInit(TIMER_HandleTypeDef *timer){
    if (timer->InitObj.Mode == TIMER_MODE_EVENTCNT)
    {
        // Disable event counter mode
        TIMER_DisableEventCounter(timer->TIMERx);
    }
    // else if(timer->InitObj.Mode == TIMER_MODE_INCAP){
    //         // Enable Timer capture interrupt
    //     TIMER_DisableCaptureInt(timer->TIMERx);
    // }
    else if(timer->InitObj.Mode == TIMER_MODE_WAKEUP){
        // Disable wakeup function
        TIMER_DisableWakeup(timer->TIMERx);    
    }
    // Reset Timer counter/prescale and stop counting
    TIMER_Reset(timer->TIMERx);
}

void HAL_TIMER_Init_INT(TIMER_HandleTypeDef *timer)
{

    // if ((timer->InitObj.Mode == TIMER_MODE_BASECNT) || (timer->InitObj.Mode == TIMER_MODE_EVENTCNT))
    // {
    //     // Enable interrupt
    //     TIMER_EnableInt(timer->TIMERx);
    // }
    // else if (timer->InitObj.Mode == TIMER_MODE_INCAP)
    // {
    //     // Enable Timer capture interrupt
    //     TIMER_EnableCaptureInt(timer->TIMERx);
    // }
    if (timer->InitObj.Mode == TIMER_MODE_INCAP)
    {
        // Enable Timer capture interrupt
        TIMER_EnableCaptureInt(timer->TIMERx);
    }
    else 
    {
        // Enable interrupt
        TIMER_EnableInt(timer->TIMERx);
    }

}

void HAL_TIMER_DeInit_INT(TIMER_HandleTypeDef *timer)
{
    if (timer->InitObj.Mode == TIMER_MODE_INCAP)
    {
        // Disable Timer capture interrupt
        TIMER_DisableCaptureInt(timer->TIMERx);
    }
    else
    {
        // Disable interrupt
        TIMER_DisableInt(timer->TIMERx);
    }
}

void HAL_TIMER_Start(TIMER_HandleTypeDef *timer)
{
    TIMER_Start(timer->TIMERx);
}

uint32_t HAL_TIMER_GetRealFreq(TIMER_HandleTypeDef *timer){
    uint32_t u32Clk = CLK_GetPeripheralFreq(timer->TIMERx);
    uint32_t u32Prescale = 0;
    uint32_t u32RealFreq = timer->InitObj.Freq;

    if (u32RealFreq > u32Clk)
        u32RealFreq = u32Clk;
    else if (u32RealFreq == 0)
        u32RealFreq = 1;

    u32Prescale = u32Clk / u32RealFreq - 1;

    if (u32Prescale > 0xFF)
        u32Prescale = 0xFF; //Prescale is 8bit in reg

    // Calc real frequency and return
    u32RealFreq = u32Clk / (u32Prescale + 1);

    return u32RealFreq;    
}

static void TIMER_HandleProc(TIMER_HandleTypeDef *timer)
{
    if (TIMER_GetIntFlag(timer->TIMERx))
    {
        // Clear Timer interrupt flag
        TIMER_ClearIntFlag(timer->TIMERx);
//        tmr_int_trigger_cnt++;
//        SYS_TEST("CNT%d\n", tmr_int_trigger_cnt);
        timer->Callback(TIMER_CB_FLAG_CNT);

    }

    if (TIMER_GetCaptureIntFlag(timer->TIMERx))
    {
        // Clear Timer capture interrupt flag
        TIMER_ClearCaptureIntFlag(timer->TIMERx);
        timer->Callback(TIMER_CB_FLAG_CAP);
//        tmr_cap_int_cnt++;
//        SYS_TEST("CAP%02d CntVal:%6d, CapVal:%6d\n",
//            tmr_cap_int_cnt, TIMER_GetCounter(TIMERx), TIMER_GetCaptureData(TIMERx));
    }

    if (TIMER_GetWakeupFlag(timer->TIMERx))
    {
        // Clear Timer interrupt flag
		TIMER_ClearWakeupFlag(timer->TIMERx);
		TIMER_ClearTFFlag(timer->TIMERx);
		TIMER_ClearIntFlag(timer->TIMERx);		
        timer->Callback(TIMER_CB_FLAG_WK);
//        tmr_wakup_int = true;
//        SYS_TEST("WAKEUP\n");
    }
}

void TMR0_IRQHandler(void)
{
    TIMER_HandleProc(&TIMER_Handle_Array[0]);
}

void TMR1_IRQHandler(void)
{
    TIMER_HandleProc(&TIMER_Handle_Array[1]);
}

void TMR2_IRQHandler(void)
{
    TIMER_HandleProc(&TIMER_Handle_Array[2]);
}

