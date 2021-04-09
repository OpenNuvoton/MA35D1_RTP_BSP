/**************************************************************************//**
 * @file     main.c
 * @brief    Configure EBI interface to access IS61WV204816BLL(SRAM) on EBI interface.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

extern void SRAM_IS61WV204816BLL(uint32_t u32MaxSize);
void AccessEBIWithPDMA(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI CS0 pin on PJ.0 */
    SYS->GPJ_MFPL |=SYS_GPJ_MFPL_PJ0MFP_EBI_nCS0;
    /* EBI RD and WR pins on PL.4 and PL.5 */
    SYS->GPL_MFPL |=SYS_GPL_MFPL_PL4MFP_EBI_nRD;
    SYS->GPL_MFPL |=SYS_GPL_MFPL_PL5MFP_EBI_nWR;
    /* EBI ALE pin on PB.11 */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB11MFP_EBI_ALE;
    /* EBI MCLK pin on PG.13 */
    SYS->GPG_MFPH |=SYS_GPG_MFPH_PG13MFP_EBI_MCLK;
    /* EBI WRL and WRH pins on PL.11 and PL.10 */
    SYS->GPL_MFPH |=SYS_GPL_MFPH_PL11MFP_EBI_nWRL;
    SYS->GPL_MFPH |=SYS_GPL_MFPH_PL10MFP_EBI_nWRH;

    /* EBI AD0~2 pins on PK.9~11 */
    SYS->GPK_MFPH |= SYS_GPK_MFPH_PK9MFP_EBI_AD0;
    SYS->GPK_MFPH |= SYS_GPK_MFPH_PK10MFP_EBI_AD1;
    SYS->GPK_MFPH |= SYS_GPK_MFPH_PK11MFP_EBI_AD2;
    /* EBI AD3~5 pins on PM.0~2 */
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM0MFP_EBI_AD3;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM1MFP_EBI_AD4;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM2MFP_EBI_AD5;
    /* EBI AD6~8 pins on PL.7~9 */
    SYS->GPL_MFPL |=  SYS_GPL_MFPL_PL7MFP_EBI_AD6;
    SYS->GPL_MFPH |=  SYS_GPL_MFPH_PL8MFP_EBI_AD7;
    SYS->GPL_MFPH |=  SYS_GPL_MFPH_PL9MFP_EBI_AD8;
    /* EBI AD9~13 pins on PA.9~13 */
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA9MFP_EBI_AD9;
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA10MFP_EBI_AD10;
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA11MFP_EBI_AD11;
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA12MFP_EBI_AD12;
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA13MFP_EBI_AD13;
    /* EBI AD14 pin on PL.15 */
    SYS->GPL_MFPH |=  SYS_GPL_MFPH_PL15MFP_EBI_AD14;
    /* EBI AD15 pin on PK.8 */
    SYS->GPK_MFPH |= SYS_GPK_MFPH_PK8MFP_EBI_AD15;

    /* EBI ADR16~19 pins on PB.12~15 */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB12MFP_EBI_ADR16;
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB13MFP_EBI_ADR17;
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EBI_ADR18;
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB15MFP_EBI_ADR19;

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(EBI_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Set GPB multi-function pins for UART16 RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART16 for printf */
    UART_Open(UART16, 115200);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("********************************************************************\n");
    printf("* Please connect IS61WV204816BLL to EBI bank0 before accessing !!  *\n");
    printf("* EBI pins settings:                                               *\n");
    printf("*   - AD0  ~  AD2   on PK.9 ~ PK.11                                *\n");
    printf("*   - AD3  ~  AD5   on PM.0 ~ PM.2                                 *\n");
    printf("*   - AD6  ~  AD8   on PL.7 ~ PL.9                                 *\n");
    printf("*   - AD9  ~  AD13  on PA.9 ~ PA.13                                *\n");
    printf("*   - AD14          on PL.15                                       *\n");
    printf("*   - AD15          on PK.8                                        *\n");
    printf("*   - ADR16 ~ ADR19 on PB.12 ~ PB.15                               *\n");
    printf("*   - nWR  on PL.5                                                 *\n");
    printf("*   - nRD  on PL.4                                                 *\n");
    printf("*   - nWRL on PL.11                                                *\n");
    printf("*   - nWRH on PL.10                                                *\n");
    printf("*   - nCS0 on PJ.0                                                 *\n");
    printf("*   - ALE  on PB.11                                                *\n");
    printf("*   - MCLK on PG.13                                                *\n");
    printf("********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOWEST, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    SRAM_IS61WV204816BLL( 512 * 1024);

    /* EBI SRAM with PDMA test */
    AccessEBIWithPDMA();

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    printf("*** SRAM Test OK ***\n");

    while(1);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
uint32_t SrcArray[64];
uint32_t DestArray[64];
uint32_t volatile u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ
 */
void PDMA2_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA2);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA2) & PDMA_ABTSTS_ABTIF2_Msk)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA2, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA2) & PDMA_TDSTS_TDIF2_Msk)
            u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA2, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt 0x%08x!!\n",status);
}

void AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA2_MODULE);

    for(i=0; i<64; i++)
    {
        SrcArray[i] = 0x76570000 + i;
        u32Result0 += SrcArray[i];
    }

    /* Open Channel 2 */
    PDMA_Open(PDMA2, (1<<2));

    //burst size is 4
    PDMA_SetBurstType(PDMA2, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA2, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(PDMA2, 2, (uint32_t)SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA2, 2, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(PDMA2, 2, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA2_IRQn);

    u32IsTestOver = 0;
    PDMA_Trigger(PDMA2, 2);
    while(u32IsTestOver == 0);
    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for(i=0; i<64; i++)
    {
        SrcArray[i] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA2, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(PDMA2, 2, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA2, 2, PDMA_MEM, FALSE, 0);

    u32IsTestOver = 0;
    PDMA_Trigger(PDMA2, 2);
    while(u32IsTestOver == 0);
    /* Transfer EBI SRAM to internal SRAM done */
    for(i=0; i<64; i++)
    {
        u32Result1 += SrcArray[i];
    }

    if(u32IsTestOver == 1)
    {
        if((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            while(1);
        }
    }
    else
    {
        printf("        PDMA fail\n\n");
        while(1);
    }

    PDMA_Close(PDMA2);
}

