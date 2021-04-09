/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Use pin PJ.12 to demonstrates timer event counter function
 *
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void TMR2_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete\n");
    TIMER_ClearIntFlag(TIMER2);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set timer event counting pin */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ12MFP_TM2;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    printf("\nThis sample code use TM2_CNT_OUT(PJ.12) to count PC.4 input event\n");
    printf("Please connect PJ.12 to PC.4, press any key to continue\n");
    getchar();

    /* Configure PC.4 as Output mode */
    GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);

    // Give a dummy target frequency here. Will over write prescale and compare value with macro
    TIMER_Open(TIMER2, TIMER_ONESHOT_MODE, 100);

    // Update prescale and compare value to what we need in event counter mode.
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 1000);
    // Counter increase on falling edge
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    // Start Timer 2
    TIMER_Start(TIMER2);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);


    for(i = 0; i < 1000; i++)
    {
        PC4 = 0; // low
        CLK_SysTickDelay(1);
        PC4 = 1;  // high
        CLK_SysTickDelay(1);
    }

    while(1);

}




