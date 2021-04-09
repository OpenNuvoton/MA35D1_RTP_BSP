/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate output different duty waveform in Timer2~Timer5 PWM.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL1_TMR4SEL_PCLK2, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL1_TMR5SEL_PCLK2, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set Timer2~5 PWM output pins */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ12MFP_TM2 | SYS_GPJ_MFPH_PJ14MFP_TM3;
    SYS->GPL_MFPH |= SYS_GPL_MFPH_PL8MFP_TM4;
    SYS->GPL_MFPL |= SYS_GPL_MFPL_PL0MFP_TM5;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART16 and set UART16 Baudrate */
    UART_Open(UART16, 115200);

    printf("+-------------------------------------------------+\n");
    printf("|    Timer2~Timer5 PWM Output Duty Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer2 PWM_CH0 output frequency is 18000 Hz and duty is 50%%.\n");
    printf("# Timer3 PWM_CH0 output frequency is 10000 Hz and duty is 10%%.\n");
    printf("# Timer4 PWM_CH0 output frequency is  9000 Hz and duty is 75%%.\n");
    printf("# Timer5 PWM_CH0 output frequency is  4000 Hz and duty is 20%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer2 PWM_CH0 on PJ.12\n");
    printf("    - Timer3 PWM_CH0 on PJ.14\n");
    printf("    - Timer4 PWM_CH0 on PL.8\n");
    printf("    - Timer5 PWM_CH0 on PL.0\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER2);
    TPWM_ENABLE_PWM_MODE(TIMER3);
    TPWM_ENABLE_PWM_MODE(TIMER4);
    TPWM_ENABLE_PWM_MODE(TIMER5);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER3);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER4);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER5);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER4, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER5, TPWM_CH0);

    /* Set Timer2 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 18000, 50);

    /* Set Timer3 PWM output frequency is 10000 Hz, duty 10% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER3, 10000, 10);

    /* Set Timer4 PWM output frequency is 9000 Hz, duty 75% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER4, 9000, 75);

    /* Set Timer5 PWM output frequency is 4000 Hz, duty 20% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER5, 4000, 20);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER3, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER4, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER5, TPWM_UP_COUNT);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER2);
    TPWM_START_COUNTER(TIMER3);
    TPWM_START_COUNTER(TIMER4);
    TPWM_START_COUNTER(TIMER5);

    printf("*** Check Timer2~Timer5 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1);
}


