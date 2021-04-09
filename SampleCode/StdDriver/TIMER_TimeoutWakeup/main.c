/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Use timer to wake up system from Power-down mode periodically.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void TMR2_IRQHandler(void)
{
    // Clear wake up flag
    TIMER_ClearWakeupFlag(TIMER2);
    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER2);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable UART clock */
    CLK_EnableModuleClock(UART16_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Enable Timer clock */
    CLK_EnableModuleClock(TMR2_MODULE);
    /* Select Timer clock source from LXT */
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i = 0;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("Timer power down/wake up sample code\n");
    while(!UART_IS_TX_EMPTY(UART16));

    /* Initial Timer2 to periodic mode with 1Hz, since system is fast (180MHz)
       and timer is slow (32kHz), and following function calls all modified timer's
       CTL register, so add extra delay between each function call and make sure the
       setting take effect */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 1);
    CLK_SysTickDelay(50);
    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER2);
    CLK_SysTickDelay(50);
    /* Enable Timer2 interrupt */
    TIMER_EnableInt(TIMER2);
    CLK_SysTickDelay(50);
    NVIC_EnableIRQ(TMR2_IRQn);
    /* Start Timer2 counting */
    TIMER_Start(TIMER2);
    CLK_SysTickDelay(50);
    /* Unlock protected registers */
    SYS_UnlockReg();
    while(1)
    {
        CLK_PowerDown();
        printf("Wake %d\n", i++);
        while(!UART_IS_TX_EMPTY(UART16));
    }

}




