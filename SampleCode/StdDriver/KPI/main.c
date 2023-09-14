/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate KPI Controller.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void KPI_IRQHandler(void)
{
    uint32_t u32Column_Num;
    uint32_t u32Mask_Col = 1;
    uint32_t u32Key_Event[4];
    uint32_t i, j;

    u32Column_Num = ((KPI->KPICONF & KPI_KPICONF_KCOL_Msk) >> KPI_KPICONF_KCOL_Pos) + 1;

    for (i = 0; i < u32Column_Num; i++)
    {
        u32Mask_Col = u32Mask_Col * 2;
    }

    u32Mask_Col = u32Mask_Col - 1;
    u32Mask_Col = (u32Mask_Col) | (u32Mask_Col << 8) | (u32Mask_Col << 16) |
                  (u32Mask_Col << 24);

    u32Key_Event[0] = KPI->KPIKPE0;
    u32Key_Event[1] = KPI->KPIKPE1;
    u32Key_Event[2] = KPI->KPIKRE0;
    u32Key_Event[3] = KPI->KPIKRE1;

    // Clear interrupt
    KPI->KPIKPE0 = u32Key_Event[0];
    KPI->KPIKPE1 = u32Key_Event[1];
    KPI->KPIKRE0 = u32Key_Event[2];
    KPI->KPIKRE1 = u32Key_Event[3];

    // Mask unused column
    u32Key_Event[0] = u32Key_Event[0] & u32Mask_Col;
    u32Key_Event[1] = u32Key_Event[1] & u32Mask_Col;
    u32Key_Event[2] = u32Key_Event[2] & u32Mask_Col;
    u32Key_Event[3] = u32Key_Event[3] & u32Mask_Col;

    for (j = 0; j < 4; j++)
    {
        if (u32Key_Event[j] != 0)
        {
            for (i = 0; i < 32; i++)
            {
                if (u32Key_Event [j] & (1<<i))
                {
                    if (j < 2)
                        printf("\n Press Key (Row%d, Column%d) \n", i/8, i%8);
                    else
                        printf("\n Release Key (Row%d, Column%d) \n", i/8, i%8);
                }
            }
        }
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable IP clock */
    CLK_SetModuleClock(KPI_MODULE, CLK_CLKSEL4_KPISEL_LXT, CLK_CLKDIV4_KPI(1));
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;

    /* Set KPI pin is pull high */
    PF->PUSEL &= ~(GPIO_PUSEL_PUSEL0_Msk | GPIO_PUSEL_PUSEL1_Msk | GPIO_PUSEL_PUSEL2_Msk |
                   GPIO_PUSEL_PUSEL3_Msk | GPIO_PUSEL_PUSEL4_Msk | GPIO_PUSEL_PUSEL5_Msk |
                   GPIO_PUSEL_PUSEL6_Msk | GPIO_PUSEL_PUSEL7_Msk);
    PF->PUSEL |= (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL0_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL1_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL2_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL3_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL4_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL5_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL6_Pos) |
                 (GPIO_PUSEL_PULL_UP << GPIO_PUSEL_PUSEL7_Pos);

    /* Lock protected registers */
    SYS_LockReg();;
}

int32_t main (void)
{
    uint32_t u32RowNum = 3, u32ColNum = 3;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    /* KPI setting */
    KPI_Open(KPI, u32RowNum, u32ColNum);
    KPI_ConfigKeyScanTiming(KPI, 50, 3, KPI_ROW_SCAN_DELAY4CLK);

    // Clear interrupt
    KPI->KPIKPE0 = KPI->KPIKPE0;
    KPI->KPIKPE1 = KPI->KPIKPE1;
    KPI->KPIKRE0 = KPI->KPIKRE0;
    KPI->KPIKRE1 = KPI->KPIKRE1;

    NVIC_EnableIRQ(KPI_IRQn);
    KPI_ENABLE_INT(KPI, KPI_KPICONF_PKINTEN_Msk|KPI_KPICONF_RKINTEN_Msk|KPI_KPICONF_INTEN_Msk);

    printf("\nThis sample code demonstrate KPI function\n");
    printf("\nPlease Press KPI key.");

    while(1);
}

