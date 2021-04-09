/**************************************************************************//**
 * @file     main.c
 *
 * @brief
 *           Show the usage of QEI compare function.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define QEI0A   PA0
#define QEI0B   PA1

void QEI0_IRQHandler(void)
{
    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk))     /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk);
    }

    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk))    /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk);
    }
}

void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(QEI0_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set multi-function pins */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
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

int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART16 for printf */
    UART16_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|          QEI Driver Sample Code      |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PA.0 and PG.2 << \n");
    printf("  >> Please connect PA.1 and PG.1 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Configure PA.0 and PA.1 as output mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);

    QEI0A = 0;
    QEI0B = 0;

    /* Set QEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    QEI_Open(QEI0, QEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */
    QEI_SET_CNT_CMP(QEI0, 0x10000);

    /* Enable compare function */
    QEI_ENABLE_CNT_CMP(QEI0);

    /* Enable QEI interrupt */
    QEI_EnableInt(QEI0, QEI_CTL_CMPIEN_Msk | QEI_CTL_OVUNIEN_Msk);

    /* Start QEI function */
    QEI_Start(QEI0);

    /* Wait compare-match and overflow interrupt happened */
    while(1)
    {
        QEI0A = 1;
        CLK_SysTickDelay(16);
        QEI0B = 1;
        CLK_SysTickDelay(16);
        QEI0A = 0;
        CLK_SysTickDelay(16);
        QEI0B = 0;
        CLK_SysTickDelay(16);
    }

}
