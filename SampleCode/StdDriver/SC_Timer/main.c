/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate how to use SC embedded timer
 *
  * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void SC0_IRQHandler(void)
{
    static uint32_t sec = 1;

    if (SC0->INTSTS & SC_INTSTS_TMR0IF_Msk)
    {
        /* Clear interrupt flag */
        SC0->INTSTS = SC_INTSTS_TMR0IF_Msk;
        printf("%d sec\n", sec++);
    }

    return;
}

void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL4_SC0SEL_HXT, CLK_CLKDIV1_SC0(1));
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

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
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("\nThis sample code demo how to use SC embedded timer. \n");

    /* Open smartcard interface */
    SC_Open(SC0, SC_PIN_STATE_IGNORE, SC_PIN_STATE_HIGH);

    /* Enable SC TIMER0 interrupt */
    SC_ENABLE_INT(SC0, SC_INTEN_TMR0IEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Real ETU divider value is "23+1=24", and the duration of each ETU is 24/SC_CLK */
    SC0->ETUCTL = 23;

    /* each 1,000,000 ETU will generate TIMER0 timeout event */
    SC_StartTimer(SC0, 0, SC_TMR_MODE_4, 1000000);  // timer counter will be reloaded.

    while (1);
}


