/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate Timer PWM Complementary mode and Dead-Time function.
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

    /* Peripheral clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPK multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Set Timer2 PWM CH0(TM2) and Timer3 PWM CH0(TM3) */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ12MFP_TM2 | SYS_GPJ_MFPH_PJ14MFP_TM3;
    /* Set Timer2 PWM CH1(T2_EXT) and Timer3 PWM CH1(T3_EXT) */
    SYS->GPJ_MFPH |= SYS_GPJ_MFPH_PJ13MFP_TM2_EXT | SYS_GPJ_MFPH_PJ15MFP_TM3_EXT;

    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{
    uint32_t u32Period, u32CMP, u32Prescaler, u32DeadTime;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART16 and set UART16 Baudrate */
    UART_Open(UART16, 115200);

    printf("+--------------------------------------------------------------+\n");
    printf("|    Timer PWM Complementary mode and Dead-Time Sample Code    |\n");
    printf("+--------------------------------------------------------------+\n\n");

    /* Configure Timer2 PWM */
    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER2);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER2);

    /* Set Timer2 PWM output frequency is 6000 Hz, duty 40% */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 6000, 40);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER2, (TPWM_CH1|TPWM_CH0));

    /* Get u32Prescaler, u32Period and u32CMP after called TPWM_ConfigOutputFreqAndDuty() API */
    u32Prescaler = (TIMER2->PWMCLKPSC + 1);
    u32Period = (TIMER2->PWMPERIOD + 1);
    u32CMP = TIMER2->PWMCMPDAT;
    u32DeadTime = u32CMP/2;

    printf("# Timer2 PWM output frequency is 6000 Hz and duty 40%%.\n");
    printf("    - Counter clock source:    PCLK1 \n");
    printf("    - Counter clock prescaler: %d \n", u32Prescaler);
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("# I/O configuration:\n");
    printf("    - Timer2 PWM_CH0 on PJ.12, PWM_CH1 on PJ.13\n\n");


    /* Configure Timer3 PWM */
    printf("# Timer3 PWM output frequency is 6000 Hz and duty 40%% with dead-time insertion.\n");
    printf("    - Counter clock source:    PCLK1 \n");
    printf("    - Counter clock prescaler: %d \n", u32Prescaler);
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("    - Dead-Time interval:      %d \n", u32DeadTime);
    printf("# I/O configuration:\n");
    printf("    - Timer3 PWM_CH0 on PJ.14, PWM_CH1 on PJ.15\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER3);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER3);

    /* Set Timer3 PWM output frequency is 6000 Hz, duty 40% */
    TPWM_ConfigOutputFreqAndDuty(TIMER3, 6000, 40);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER3, (TPWM_CH1|TPWM_CH0));

    /* Enable and configure dead-time interval is (u32DeadTime * TMR3_PWMCLK * prescaler) */
    SYS_UnlockReg(); // Unlock protected registers
    TPWM_EnableDeadTimeWithPrescale(TIMER3, (u32DeadTime-1));
    SYS_LockReg(); // Lock protected registers

    printf("*** Check Timer2 and Timer3 PWM output waveform by oscilloscope ***\n");

    /* Start Timer2 and Timer3 PWM counter by trigger Timer2 sync. start */
    TPWM_SET_COUNTER_SYNC_MODE(TIMER2, TPWM_CNTR_SYNC_START_BY_TIMER2);
    TPWM_SET_COUNTER_SYNC_MODE(TIMER3, TPWM_CNTR_SYNC_START_BY_TIMER2);
    TPWM_TRIGGER_COUNTER_SYNC(TIMER2);

    while(1);
}


