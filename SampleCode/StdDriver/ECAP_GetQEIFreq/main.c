/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Show how to use ECAP interface to get QEIA frequency
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t u32Status;
uint32_t u32IC0Hold;

void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        /*PA.0 gpio toggle */
        PA0 ^= 1;
    }
}

void ECAP0_IRQHandler(void)
{
    /* Get input Capture status */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);

    /* Check input capture channel 0 flag */
    if((u32Status & ECAP_STATUS_CAPTF0_Msk) == ECAP_STATUS_CAPTF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk);

        /* Get input capture counter hold value */
        u32IC0Hold = ECAP0->HLD0;
    }

    /* Check input capture channel 1 flag */
    if((u32Status & ECAP_STATUS_CAPTF1_Msk) == ECAP_STATUS_CAPTF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF1_Msk);
    }

    /* Check input capture channel 2 flag */
    if((u32Status & ECAP_STATUS_CAPTF2_Msk) == ECAP_STATUS_CAPTF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF2_Msk);
    }

    /* Check input capture compare-match flag */
    if((u32Status & ECAP_STATUS_CAPCMPF_Msk) == ECAP_STATUS_CAPCMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0,ECAP_STATUS_CAPCMPF_Msk);
    }

    /* Check input capture overflow flag */
    if((u32Status & ECAP_STATUS_CAPOVF_Msk) == ECAP_STATUS_CAPOVF_Msk)
    {
        /* Clear input capture overflow flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0,ECAP_STATUS_CAPOVF_Msk);
    }
}


void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(ECAP0_MODULE);
    CLK_EnableModuleClock(QEI0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select TMR2 module clock source */
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, 0);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set multi-function pins */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG5MFP_Msk | SYS_GPG_MFPL_PG6MFP_Msk | SYS_GPG_MFPL_PG7MFP_Msk);
    SYS->GPG_MFPL |= (SYS_GPG_MFPL_PG5MFP_ECAP0_IC0 | SYS_GPG_MFPL_PG6MFP_ECAP0_IC1 | SYS_GPG_MFPL_PG7MFP_ECAP0_IC2);
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG0MFP_Msk | SYS_GPG_MFPL_PG1MFP_Msk | SYS_GPG_MFPL_PG2MFP_Msk);
    SYS->GPG_MFPL |= (SYS_GPG_MFPL_PG0MFP_QEI0_INDEX | SYS_GPG_MFPL_PG1MFP_QEI0_B | SYS_GPG_MFPL_PG2MFP_QEI0_A);
    /* Lock protected registers */
    SYS_LockReg();
}

void UART16_Init()
{
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
}

void ECAP0_Init(void)
{
    /* Enable ECAP0*/
    ECAP_Open(ECAP0, ECAP_DISABLE_COMPARE);

    /* Select Reload function */
    ECAP_SEL_RELOAD_TRIG_SRC(ECAP0, (ECAP_CTL1_CAP0RLDEN_Msk|ECAP_CTL1_CAP1RLDEN_Msk));

    /* Enable ECAP0 Input Channel 0*/
    ECAP_ENABLE_INPUT_CHANNEL(ECAP0, ECAP_CTL0_IC0EN_Msk);

    /* Enable ECAP0 source from IC0 */
    ECAP_SEL_INPUT_SRC(ECAP0, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_CH);

    /* Select IC0 detect rising edge */
    ECAP_SEL_CAPTURE_EDGE(ECAP0, ECAP_IC0, ECAP_RISING_EDGE);

    /* Input Channel 0 interrupt enabled */
    ECAP_EnableINT(ECAP0, ECAP_CTL0_CAPIEN0_Msk);
}

void QEI0_Init(void)
{
    QEI_Open(QEI0,QEI_CTL_X4_FREE_COUNTING_MODE,0);
}

void Timer2_Init(void)
{
    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER2,TIMER_PERIODIC_MODE,10000);
    TIMER_EnableInt(TIMER2);

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

}

int32_t main(void)
{
    uint32_t u32Hz=0, u32Hz_DET=0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART16_Init();

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|          ECAP with QEI Sample Code           |\n");
    printf("+----------------------------------------------+\n");
    printf("\n");
    printf("  !! GPIO PA.0 toggle periodically    !!\n");
    printf("  !! Connect PA.0 --> PG.2(QEI0_A) !!\n\n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Initial ECAP0 function */
    ECAP0_Init();

    /* Initial QEI0 function */
    QEI0_Init();

    /* Initial Timer2 function */
    Timer2_Init();

    /* Configure PA.0 as output mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    /* Start Timer2 counting */
    TIMER_Start(TIMER2);

    /* Delay 200ms */
    CLK_SysTickDelay(200000);

    /* Init & clear ECAP interrupt status flags */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);
    ECAP0->STATUS = u32Status;

    /* ECAP_CNT starts up-counting */
    ECAP_CNT_START(ECAP0);

    while(1)
    {
        if(u32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            u32Status = 0;

            /* Calculate the IC0 input frequency */
            u32Hz_DET = (SystemCoreClock/2) / (u32IC0Hold + 1);

            if(u32Hz != u32Hz_DET)
            {
                /* If IC0 input frequency is changed, Update frequency */
                u32Hz = u32Hz_DET;
            }
            else
            {
                printf("\nECAP0_IC0 input frequency is %d (Hz),u32IC0Hold=0x%08x\n", u32Hz,u32IC0Hold);
                TIMER_Stop(TIMER2); //Disable timer Counting.
                break;
            }
        }

    }
    /* Disable External Interrupt */
    NVIC_DisableIRQ(ECAP0_IRQn);
    NVIC_DisableIRQ(TMR2_IRQn);

    /* Disable ECAP function */
    ECAP_Close(ECAP0);

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR2_MODULE);

    /* Disable ECAP IP clock */
    CLK_DisableModuleClock(ECAP0_MODULE);

    printf("\nExit ECAP sample code\n");

    while(1);
}



