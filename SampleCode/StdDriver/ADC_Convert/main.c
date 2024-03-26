/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate ADC function by repeatedly convert the input of ADC
 *           channel 0 and shows the result on UART console.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t volatile u32Busy = 0;

void ADC0_IRQHandler(void)
{
    // Clear interrupt flag
    ADC_CLR_INT_FLAG(ADC0, ADC_ISR_MF_Msk);
    
    // Get ADC convert result
    printf("Convert result is %x\n", (uint32_t)ADC_GET_CONVERSION_DATA(ADC0, 0));

    u32Busy = 0;
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(ADC_MODULE, 0, CLK_CLKDIV4_ADC(900));  // Set ADC clock rate to 200kHz
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB.8 to input mode */
    PB->MODE &= ~GPIO_MODE_MODE8_Msk;
    /* Set multi-function pin ADC channel 0 input*/
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB8MFP_Msk) | SYS_GPB_MFPH_PB8MFP_ADC0_CH0;
    /* Disable digital input path to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT8);

    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("\nThis sample code demonstrate ADC channel 0 conversion and prints the result on UART\n");

    // Enable channel 0
    ADC_Open(ADC0, ADC_INPUT_MODE_NORMAL_CONV, ADC_HIGH_SPEED_MODE, ADC_CH_0_MASK);

    // Power on ADC
    ADC0->CONF |= ADC_REFSEL_AVDD;
    ADC_POWER_ON(ADC0);

    // Enable ADC convert complete interrupt
    ADC_EnableInt(ADC0, ADC_IER_MIEN_Msk);
    NVIC_EnableIRQ(ADC0_IRQn);

    while(1)
    {
        // Trigger ADC conversion if it is idle
        if(!u32Busy)
        {
            u32Busy = 1;
            ADC_START_CONV(ADC0);
        }
    }

}

