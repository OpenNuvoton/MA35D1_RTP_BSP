/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to call ARM CMSIS DSP library to calculate FFT.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "arm_math.h"

#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES / 2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which maximum energy of bin occur */
uint32_t refIndex = 213, testIndex = 0;


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable UART clock */
    CLK_EnableModuleClock(UART16_MODULE);

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


int main()
{
    arm_cfft_radix4_instance_f32 S;
    float32_t maxValue;

    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    /*
        This sample code is used to show how to use ARM Cortex-M4 DSP library to calculate FFT.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|             DSP FFT Sample Code        |\n");
    printf("+----------------------------------------+\n");

    /* Initialize the CFFT/CIFFT module */
    arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix4_f32(&S, testInput_f32_10khz);

    /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
    arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

    /* Calculates maxValue and returns corresponding BIN value */
    arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

    if(testIndex !=  refIndex)
    {
        printf("ERROR: FFT calculation result fail!\n");
    }
    else
    {
        printf("FFT calculation test ok!\n");
    }

    while(1);
}

