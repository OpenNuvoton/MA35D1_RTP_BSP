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
    uint32_t i;
    uint32_t u32KPI_Status;
    u32KPI_Status = KPI->KPISTATUS;

    if(u32KPI_Status & KPI_KPISTATUS_PKEYINT_Msk) // Press Key
    {
        uint32_t u32KPE0 = KPI->KPIKPE0;
        uint32_t u32KPE1 = KPI->KPIKPE1;

        while(u32KPE0 != 0x0)
        {
            for(i = 0; i < 32; i++)
            {
                if((0x1 << i) & u32KPE0)
                {
                    KPI->KPIKPE0 = (0x1 << i); // clear interrupt
                    printf("\n Press Key (Row%d, Column%d) \n", i/8, i%8);
                    u32KPE0 = KPI->KPIKPE0;
                }
            }
        }

        while(u32KPE1 != 0x0)
        {
            for(i = 0; i < 32; i++)
            {
                if((0x1 << i) & u32KPE1)
                {
                    KPI->KPIKPE1 = (0x1 << i); // clear interrupt
                    printf("\n Press Key (Row%d, Column%d) \n", (i/8 + 4), i%8);
                    u32KPE1 = KPI->KPIKPE1;
                }
            }
        }
    }

    if(u32KPI_Status & KPI_KPISTATUS_RKEYINT_Msk) // Release key
    {
        uint32_t u32KRE0 = KPI->KPIKRE0;
        uint32_t u32KRE1 = KPI->KPIKRE1;

        while(u32KRE0 != 0x0)
        {
            for(i = 0; i < 32; i++)
            {
                if((0x1 << i) & u32KRE0)
                {
                    KPI->KPIKRE0 = (0x1 << i); // clear interrupt
                    printf("\n Release Key (Row%d, Column%d) \n", i/8, i%8);
                    u32KRE0 = KPI->KPIKPE0;
                }
            }
        }

        while(u32KRE1 != 0x0)
        {
            for(i = 0; i < 32; i++)
            {
                if((0x1 << i) & u32KRE1)
                {
                    KPI->KPIKRE1 = (0x1 << i); // clear interrupt
                    printf("\n Release Key (Row%d, Column%d) \n", (i/8 + 4), i%8);
                    u32KRE1 = KPI->KPIKRE1;
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
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV1_UART16(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;

    /* Set KPI multi-function pin */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF0MFP_Msk | SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF2MFP_Msk |
                       SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF6MFP_Msk);
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF0MFP_KPI_COL0 | SYS_GPF_MFPL_PF1MFP_KPI_COL1 | SYS_GPF_MFPL_PF2MFP_KPI_COL2 |
                     SYS_GPF_MFPL_PF4MFP_KPI_ROW0 | SYS_GPF_MFPL_PF5MFP_KPI_ROW1 | SYS_GPF_MFPL_PF6MFP_KPI_ROW2;

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

