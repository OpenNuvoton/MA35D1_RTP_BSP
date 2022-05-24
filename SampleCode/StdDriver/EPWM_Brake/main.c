/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate how to use EPWM brake function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       192000000


/**
 * @brief       EPWM0 Brake0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 Brake0 interrupt event
 */
void BRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (EPWM0 channel 0 output will toggle again)\n");
    getchar();

    /* Unlock protected registers */
    SYS_UnlockReg();
    // Clear brake interrupt flag
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
    /* Lock protected registers */
    SYS_LockReg();
}

void SYS_Init(void)
{

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable IP module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable GPIOD module clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set PI multi-function pins for EPWM0 Channel 0,1,2,3 */
    SYS->GPI_MFPL &= ~(SYS_GPI_MFPL_PI0MFP_Msk | SYS_GPI_MFPL_PI1MFP_Msk | SYS_GPI_MFPL_PI2MFP_Msk | SYS_GPI_MFPL_PI3MFP_Msk);
    SYS->GPI_MFPL |= (SYS_GPI_MFPL_PI0MFP_EPWM0_CH0 | SYS_GPI_MFPL_PI1MFP_EPWM0_CH1 | SYS_GPI_MFPL_PI2MFP_EPWM0_CH2 | SYS_GPI_MFPL_PI3MFP_EPWM0_CH3);

    /* Set J0 multi-function pin for EPWM1 brake pin 0 */
    SYS->GPJ_MFPL =  (SYS->GPJ_MFPL & ~SYS_GPJ_MFPL_PJ0MFP_Msk) | SYS_GPJ_MFPL_PJ0MFP_EPWM1_BRAKE0;
}

void UART0_Init()
{

    /* Configure UART0 and set UART0 baud rate */
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

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nConnect PJ.0 (EPWM1 brake pin 0) to PD.0.\n");
    printf("It will generate brake interrupt and EPWM0 channel 0 output stop toggling.\n");

    GPIO_SetMode(PD, BIT0, GPIO_MODE_OUTPUT);
    PD0 = 0;

    // EPWM0 frequency is 100Hz, duty 30%,
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    // Enable output of all EPWM channels
    EPWM_EnableOutput(EPWM0, BIT5|BIT4|BIT3|BIT2|BIT1|BIT0);

    /* Unlock protected registers */
    SYS_UnlockReg();
    // Enable brake and interrupt
    EPWM_EnableFaultBrake (EPWM0, EPWM_CH_0_MASK, 1, EPWM_FB_EDGE_BKP0);
    // Set Brake pin to EPWM1_BRAKE0
    EPWM_SetBrakePinSource(EPWM0, 0, 1);
    EPWM_EnableFaultBrakeInt (EPWM0, 0);
    // Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128
    EPWM_EnableBrakeNoiseFilter(EPWM0, 0, 7, EPWM_NF_CLK_DIV_128);
    // Clear brake interrupt flag
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
    /* Lock protected registers */
    SYS_LockReg();

    NVIC_ClearPendingIRQ(BRAKE0_IRQn);
    NVIC_EnableIRQ(BRAKE0_IRQn);

    // Start
    EPWM_Start(EPWM0, 1);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD0 = 1;

    while(1);
}


