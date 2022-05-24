/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Capture the EPWM1 Channel 0 waveform by EPWM1 Channel 2.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       192000000

/**
 * @brief       EPWM1 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM1 interrupt event
 */
void EPWM1P1_IRQHandler(void)
{
    if(EPWM_GetCaptureIntFlag(EPWM1, 2) > 1)
    {
        EPWM_ClearCaptureIntFlag(EPWM1, 2, EPWM_CAPTURE_INT_FALLING_LATCH);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(EPWM_T *EPWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    /* Clear Capture Falling Indicator (Time A) */
    EPWM_ClearCaptureIntFlag(EPWM, u32Ch, EPWM_CAPTURE_INT_FALLING_LATCH);

    /* Wait for Capture Falling Indicator  */
    while((EPWM1->CAPIF & (EPWM_CAPIF_CFLIF0_Msk << u32Ch)) == 0);

    /* Clear Capture Falling Indicator (Time B)*/
    EPWM_ClearCaptureIntFlag(EPWM, u32Ch, EPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(EPWM_GetCaptureIntFlag(EPWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        EPWM_ClearCaptureIntFlag(EPWM, u32Ch, EPWM_CAPTURE_INT_FALLING_LATCH | EPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = EPWM_GET_CAPTURE_FALLING_DATA(EPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while(EPWM_GetCaptureIntFlag(EPWM, u32Ch) < 2);

        /* Clear Capture Rising Indicator */
        EPWM_ClearCaptureIntFlag(EPWM, u32Ch, EPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = EPWM_GET_CAPTURE_RISING_DATA(EPWM, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeriod = u32Count[1] - u32Count[2];

    u16LowPeriod = 0x10000 - u32Count[1];

    u16TotalPeriod = 0x10000 - u32Count[2];

    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
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
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set PL multi-function pins for EPWM1 Channel 0 and 2 */
    SYS->GPL_MFPL &= ~(SYS_GPL_MFPL_PL0MFP_Msk | SYS_GPL_MFPL_PL2MFP_Msk);
    SYS->GPL_MFPL |= (SYS_GPL_MFPL_PL0MFP_EPWM1_CH0 | SYS_GPL_MFPL_PL2MFP_EPWM1_CH2);

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
    printf("  This sample code will use EPWM1 channel 2 to capture\n  the signal from EPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    EPWM1 channel 2(PL.2) <--> EPWM1 channel 0(PL.0)\n\n");
    printf("Use EPWM1 Channel 2(PL.2) to capture the EPWM1 Channel 0(PL.0) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start EPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the EPWM1 Channel 0 as EPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* set EPWM1 channel 0 output configuration */
        EPWM_ConfigOutputChannel(EPWM1, 0, 300, 30);

        /* Enable EPWM Output path for EPWM1 channel 0 */
        EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

        /* Enable Timer for EPWM1 channel 0 */
        EPWM_Start(EPWM1, EPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the EPWM1 channel 2 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* set EPWM1 channel 2 capture configuration */
        EPWM_ConfigCaptureChannel(EPWM1, 2, 52, 0);

        /* Enable capture falling edge interrupt for EPWM1 channel 2 */
        //EPWM_EnableCaptureInt(EPWM1, 2, EPWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable EPWM1 NVIC interrupt */
        //NVIC_EnableIRQ(EPWM1P1_IRQn);

        /* Enable Timer for EPWM1 channel 2 */
        EPWM_Start(EPWM1, EPWM_CH_2_MASK);

        /* Enable Capture Function for EPWM1 channel 2 */
        EPWM_EnableCapture(EPWM1, EPWM_CH_2_MASK);

        /* Enable falling capture reload */
        EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN2_Msk;

        /* Wait until EPWM1 channel 2 Timer start to count */
        while((EPWM1->CNT[2]) == 0);

        /* Capture the Input Waveform Data */
        CalPeriodTime(EPWM1, 2);
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop EPWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set EPWM Timer loaded value(Period) as 0. When EPWM internal counter(CNT) reaches to 0, disable EPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set EPWM1 channel 0 loaded value as 0 */
        EPWM_Stop(EPWM1, EPWM_CH_0_MASK);

        /* Wait until EPWM1 channel 0 Timer Stop */
        while((EPWM1->CNT[0] & EPWM_CNT0_CNT_Msk) != 0);

        /* Disable Timer for EPWM1 channel 0 */
        EPWM_ForceStop(EPWM1, EPWM_CH_0_MASK);

        /* Disable EPWM Output path for EPWM1 channel 0 */
        EPWM_DisableOutput(EPWM1, EPWM_CH_0_MASK);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop EPWM1 channel 2 (Recommended procedure method 1)                                                    */
        /* Set EPWM Timer loaded value(Period) as 0. When EPWM internal counter(CNT) reaches to 0, disable EPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable EPWM1 NVIC */
        //NVIC_DisableIRQ(EPWM1P1_IRQn);

        /* Set loaded value as 0 for EPWM1 channel 2 */
        EPWM_Stop(EPWM1, EPWM_CH_2_MASK);

        /* Wait until EPWM1 channel 2 current counter reach to 0 */
        while((EPWM1->CNT[2] & EPWM_CNT0_CNT_Msk) != 0);

        /* Disable Timer for EPWM1 channel 2 */
        EPWM_ForceStop(EPWM1, EPWM_CH_2_MASK);

        /* Disable Capture Function and Capture Input path for  EPWM1 channel 2*/
        EPWM_DisableCapture(EPWM1, EPWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for EPWM1 channel 2 */
        EPWM_ClearCaptureIntFlag(EPWM1, 2, EPWM_CAPTURE_INT_FALLING_LATCH);
    }
}


