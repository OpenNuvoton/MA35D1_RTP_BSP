/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate how to use EPWM counter output waveform.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       192000000


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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set PI multi-function pins for EPWM0 Channel 0,1,2,3,4,5 */
    SYS->GPI_MFPL &= ~(SYS_GPI_MFPL_PI0MFP_Msk | SYS_GPI_MFPL_PI1MFP_Msk | SYS_GPI_MFPL_PI2MFP_Msk 
                        | SYS_GPI_MFPL_PI3MFP_Msk | SYS_GPI_MFPL_PI4MFP_Msk | SYS_GPI_MFPL_PI5MFP_Msk);
    SYS->GPI_MFPL |= (SYS_GPI_MFPL_PI0MFP_EPWM0_CH0 | SYS_GPI_MFPL_PI1MFP_EPWM0_CH1 | SYS_GPI_MFPL_PI2MFP_EPWM0_CH2 
                    | SYS_GPI_MFPL_PI3MFP_EPWM0_CH3 | SYS_GPI_MFPL_PI4MFP_EPWM0_CH4 | SYS_GPI_MFPL_PI5MFP_EPWM0_CH5);
}

void UART0_Init()
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART16, 115200);
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with EPWM0 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  EPWM0 channel 0: 360000 Hz, duty 90%%.\n");
    printf("  EPWM0 channel 1: 320000 Hz, duty 80%%.\n");
    printf("  EPWM0 channel 2: 250000 Hz, duty 75%%.\n");
    printf("  EPWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  EPWM0 channel 4: 160000 Hz, duty 60%%.\n");
    printf("  EPWM0 channel 5: 150000 Hz, duty 50%%.\n");
    printf("    waveform output pin: EPWM0_CH0(PI.0), EPWM0_CH1(PI.1), EPWM0_CH2(PI.2), EPWM0_CH3(PI.3), EPWM0_CH4(PI.4), EPWM0_CH5(PI.5)\n");

    /* EPWM0 channel 0~5 frequency and duty configuration are as follows */
    EPWM_ConfigOutputChannel(EPWM0, 0, 360000, 90);
    EPWM_ConfigOutputChannel(EPWM0, 1, 320000, 80);
    EPWM_ConfigOutputChannel(EPWM0, 2, 250000, 75);
    EPWM_ConfigOutputChannel(EPWM0, 3, 180000, 70);
    EPWM_ConfigOutputChannel(EPWM0, 4, 160000, 60);
    EPWM_ConfigOutputChannel(EPWM0, 5, 150000, 50);

    /* Enable output of EPWM0 channel 0~5 */
    EPWM_EnableOutput(EPWM0, 0x3F);

    /* Start EPWM0 counter */
    EPWM_Start(EPWM0, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Start EPWM0 counter */
    EPWM_ForceStop(EPWM0, 0x3F);

    printf("Done.");
    while(1);

}


