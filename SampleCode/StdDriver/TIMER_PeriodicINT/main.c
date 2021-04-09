 /**************************************************************************//**
 * @file     main.c
 *
 * @brief    Implement timer counting in periodic mode.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        g_au32TMRINTCount[0]++;
    }
}


void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        g_au32TMRINTCount[1]++;
    }
}


void TMR4_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER4) == 1)
    {
        /* Clear Timer4 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER4);

        g_au32TMRINTCount[2]++;
    }
}


void TMR5_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER5) == 1)
    {
        /* Clear Timer5 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER5);

        g_au32TMRINTCount[3]++;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL1_TMR4SEL_HIRC, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL1_TMR5SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitCount, au32Counts[4];

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    Timer Periodic Interrupt Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HXT       \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer4 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 4 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer5 Settings:\n");
    printf("    - Clock source is HXT       \n");
    printf("    - Time-out frequency is 8 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check Timer2 ~ Timer5 interrupt counts are reasonable or not.\n\n");

    /* Open Timer2 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER2);

    /* Open Timer3 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 2);
    TIMER_EnableInt(TIMER3);

    /* Open Timer4 in periodic mode, enable interrupt and 4 interrupt ticks per second */
    TIMER_Open(TIMER4, TIMER_PERIODIC_MODE, 4);
    TIMER_EnableInt(TIMER4);

    /* Open Timer5 in periodic mode, enable interrupt and 8 interrupt ticks per second */
    TIMER_Open(TIMER5, TIMER_PERIODIC_MODE, 8);
    TIMER_EnableInt(TIMER5);

    /* Enable Timer2 ~ Timer5 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(TMR3_IRQn);
    NVIC_EnableIRQ(TMR4_IRQn);
    NVIC_EnableIRQ(TMR5_IRQn);

    /* Clear Timer2 ~ Timer5 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start Timer2 ~ Timer5 counting */
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER4);
    TIMER_Start(TIMER5);

    /* Check Timer2 ~ Timer5 interrupt counts */
    printf("# Timer interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TMRINTCount[0];
            au32Counts[1] = g_au32TMRINTCount[1];
            au32Counts[2] = g_au32TMRINTCount[2];
            au32Counts[3] = g_au32TMRINTCount[3];
            printf("    TMR2:%3d    TMR3:%3d    TMR4:%3d    TMR5:%3d\n",
                   au32Counts[0], au32Counts[1], au32Counts[2], au32Counts[3]);
            u32InitCount = g_au32TMRINTCount[0];

            if((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)) ||
                    (au32Counts[2] > (au32Counts[0] * 4 + 1)) || (au32Counts[2] < (au32Counts[0] * 4 - 1)) ||
                    (au32Counts[3] > (au32Counts[0] * 8 + 1)) || (au32Counts[3] < (au32Counts[0] * 8 - 1)))
            {
                printf("*** FAIL ***\n");
                while(1) {}
            }
        }
    }

    printf("*** PASS ***\n");

    while(1);
}


