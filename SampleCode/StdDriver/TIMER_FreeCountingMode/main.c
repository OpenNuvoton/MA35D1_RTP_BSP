/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Use the timer pin PJ.13 to demonstrate timer free counting mode
 *           function. And displays the measured input frequency to
 *           UART console
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TMR2_IRQHandler(void)
{
    static int cnt = 0;
    static uint32_t t0, t1;

    TIMER_ClearCaptureIntFlag(TIMER2);

    if(cnt == 0)
    {
        t0 = TIMER_GetCaptureData(TIMER2);
        cnt++;
    }
    else if(cnt == 1)
    {
        t1 = TIMER_GetCaptureData(TIMER2);
        cnt++;
        if(t0 >= t1)
        {
            // over run, drop this data and do nothing
        }
        else
        {
            printf("Input frequency is %dHz\n", 180000000 / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set Timer 2 capture pin */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ13MFP_TM2_EXT;

    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with Timer 2 capture pin PJ.13, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);

    // Configure Timer 2 free counting mode, capture TDR value on rising edge
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_RISING);

    // Start Timer 2
    TIMER_Start(TIMER2);

    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);

    while(1);

}




