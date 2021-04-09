/**************************************************************************//**
 * @file     main.c
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0 default IRQ.
 */
void EINT0_IRQHandler(void)
{
    printf("EINT0_IRQHandler..........\n");
    /* To check if PA.6 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PG, BIT0))
    {
        GPIO_CLR_INT_FLAG(PG, BIT0);
        printf("PG.0 EINT0 occurred.\n");
    }

    /* To check if PI.4 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PI, BIT4))
    {
        GPIO_CLR_INT_FLAG(PI, BIT4);
        printf("PI.4 EINT0 occurred.\n");
    }

}

/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1 default IRQ.
 */
void EINT1_IRQHandler(void)
{

    /* To check if PB.10 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT10))
    {
        GPIO_CLR_INT_FLAG(PB, BIT10);
        printf("PB.10 EINT1 occurred.\n");
    }

}

void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);
    CLK_EnableModuleClock(GPI_MODULE);
    CLK_EnableModuleClock(GPJ_MODULE);
    CLK_EnableModuleClock(GPK_MODULE);
    CLK_EnableModuleClock(GPL_MODULE);
    CLK_EnableModuleClock(GPM_MODULE);
    CLK_EnableModuleClock(GPN_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    
    /* Set multi-function pins for EINT0/EINT1 */
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG0MFP_Msk);
    SYS->GPI_MFPL &= ~(SYS_GPI_MFPL_PI4MFP_Msk);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB10MFP_Msk);
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG0MFP_INT0;
    SYS->GPI_MFPL |= SYS_GPI_MFPL_PI4MFP_INT0;
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB10MFP_INT1;
    
    /* Lock protected registers */
    SYS_LockReg();
}

void UART16_Init()
{

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART16 for printf */
    UART16_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("|    GPIO EINT0/EINT1 Interrupt and De-bounce Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0(PG.0 and PI.4) and EINT1(PB.10) are used to test interrupt\n");

    /* Configure PG.0 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PG, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PG, 0, GPIO_INT_FALLING);

    /* Configure PI.4 as EINT0 pin and enable interrupt by rising edge trigger */
    GPIO_SetMode(PI, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PI, 4, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PB.10 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 10, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of HXT clock */
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_HXT, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_SET_DEBOUNCE_TIME(PG, GPIO_DBCTL_DBCLKSRC_HXT, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_SET_DEBOUNCE_TIME(PI, GPIO_DBCTL_DBCLKSRC_HXT, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PG, BIT0);
    GPIO_ENABLE_DEBOUNCE(PI, BIT4);
    GPIO_ENABLE_DEBOUNCE(PB, BIT10);

    /* Waiting for interrupts */
    while(1);
}
