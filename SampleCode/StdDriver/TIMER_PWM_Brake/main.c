/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate how to use Timer2 PWM brake function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void TMR2_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (TIMER2 PWM output will toggle again)\n");
    getchar();

    // Clear brake interrupt flag
    SYS_UnlockReg();
    TIMER2->PWMINTSTS1 = TIMER_PWMINTSTS1_BRKEIF0_Msk | TIMER_PWMINTSTS1_BRKEIF1_Msk | TIMER_PWMINTSTS1_BRKESTS0_Msk | TIMER_PWMINTSTS1_BRKESTS1_Msk;
    SYS_LockReg();
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set Timer2 PWM output pins and EPWM1 brake pin 0 (TPWM_TM_BRAKE2),
       Timers share the same brake pins with EPWM */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ12MFP_TM2;
    SYS->GPJ_MFPL |= SYS_GPJ_MFPL_PJ0MFP_EPWM1_BRAKE0;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART16 and set UART16 Baudrate */
    UART_Open(UART16, 115200);

    printf("\nL->H state change on PJ.0 will generate brake interrupt,\n");
    printf("and Timer2 PWM output will stop until brake state cleared.\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER2);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_UP_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);

    /* Set Timer2 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 18000, 50);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER2);

    // Enable brake and interrupt, PWM output stays at low after brake event
    SYS_UnlockReg();
    TPWM_SET_BRAKE_PIN_SOURCE(TIMER2, TPWM_TM_BRAKE2);
    TPWM_EnableFaultBrake(TIMER2, TPWM_OUTPUT_LOW, TPWM_OUTPUT_LOW, TPWM_BRAKE_SOURCE_EDGE_BKPIN);
    TPWM_EnableFaultBrakeInt(TIMER2, TPWM_BRAKE_EDGE);
    SYS_LockReg();

    NVIC_EnableIRQ(TMR2_IRQn);

    while(1);
}


