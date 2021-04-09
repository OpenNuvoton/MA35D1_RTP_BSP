/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Change duty cycle and period of output waveform in PWM down count type.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

volatile uint32_t gu32Period;

void TMR2_IRQHandler(void)
{
    static uint32_t u32Toggle = 0;

    if(TPWM_GET_PERIOD_INT_FLAG(TIMER2))
    {
        if(u32Toggle == 0)
        {
            /* Set PWM period to generate output frequency 36000 Hz */
            TPWM_SET_PERIOD(TIMER2, ((gu32Period/2)-1));

            /* Set PWM duty, 40% */
            TPWM_SET_CMPDAT(TIMER2, (((gu32Period/2)*4) / 10));
        }
        else
        {
            /* Set PWM period to generate output frequency 18000 Hz */
            TPWM_SET_PERIOD(TIMER2, (gu32Period-1));

            /* Set PWM duty, 50% */
            TPWM_SET_CMPDAT(TIMER2, (gu32Period/2));
        }
        u32Toggle ^= 1;
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER2);
    }
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

    /* Set Timer2 PWM CH0(T2) pin */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ12MFP_TM2;

    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART16 and set UART16 Baudrate */
    UART_Open(UART16, 115200);

    printf("+-----------------------------------------------+\n");
    printf("|    Timer PWM Change Duty Cycle Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    printf("# Timer2 PWM_CH0 frequency of first period is 18000 Hz and duty is 50%%.\n");
    printf("# Timer2 PWM_CH0 frequency of second period is 36000 Hz and duty is 40%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer2 PWM_CH0 on PJ.12\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER2);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);

    /* Set Timer2 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 18000, 50);

    /* Get initial period and comparator value */
    gu32Period = TPWM_GET_PERIOD(TIMER2) + 1;

    /* Set PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_DOWN_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);

    /* Enable period event interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER2);

    printf("*** Check Timer2 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1);
}


