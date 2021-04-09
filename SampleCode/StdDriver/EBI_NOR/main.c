/**************************************************************************//**
 * @file     main.c
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void NOR_MX29LV320T_RESET(uint32_t u32Bank);
extern int32_t NOR_MX29LV320T_CheckStatus(uint32_t u32DstAddr, uint16_t u16Data, uint32_t u32TimeoutMs);
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI CS1 pin on PJ.1 */
    SYS->GPJ_MFPL |=SYS_GPJ_MFPL_PJ1MFP_EBI_nCS1;
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

    /* EBI ADR0~1 pins on PA.0~1 */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_EBI_ADR0;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA1MFP_EBI_ADR1;
    /* EBI ADR2 pin on PI.2 */
    SYS->GPI_MFPL |= SYS_GPI_MFPL_PI2MFP_EBI_ADR2;
    /* EBI ADR3~5 pins on PA.3~5 */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA3MFP_EBI_ADR3;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_EBI_ADR4;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA5MFP_EBI_ADR5;
    /* EBI ADR6~14 pins on PM.3~11 */
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM3MFP_EBI_ADR6;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM4MFP_EBI_ADR7;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM5MFP_EBI_ADR8;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM6MFP_EBI_ADR9;
    SYS->GPM_MFPL |= SYS_GPM_MFPL_PM7MFP_EBI_ADR10;
    SYS->GPM_MFPH |= SYS_GPM_MFPH_PM8MFP_EBI_ADR11;
    SYS->GPM_MFPH |= SYS_GPM_MFPH_PM9MFP_EBI_ADR12;
    SYS->GPM_MFPH |= SYS_GPM_MFPH_PM10MFP_EBI_ADR13;
    SYS->GPM_MFPH |= SYS_GPM_MFPH_PM11MFP_EBI_ADR14;
    /* EBI ADR15 pin on PB.10 */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB10MFP_EBI_ADR15;
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
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART16 for printf */
    UART_Open(UART16, 115200);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0  ~  AD2   on PK.9 ~ PK.11                                    *\n");
    printf("*   - AD3  ~  AD5   on PM.0 ~ PM.2                                     *\n");
    printf("*   - AD6  ~  AD8   on PL.7 ~ PL.9                                     *\n");
    printf("*   - AD9  ~  AD13  on PA.9 ~ PA.13                                    *\n");
    printf("*   - AD14          on PL.15                                           *\n");
    printf("*   - AD15          on PK.8                                            *\n");
    printf("*   - ADR0 ~ ADR1   on PA.0 ~ PA.1                                     *\n");
    printf("*   - ADR2          on PI.2                                            *\n");
    printf("*   - ADR3 ~ ADR5   on PA.3 ~ PA.5                                     *\n");
    printf("*   - ADR6 ~ ADR14  on PM.3 ~ PM.11                                    *\n");
    printf("*   - ADR15         on PB.10                                           *\n");
    printf("*   - ADR16 ~ ADR19 on PB.12 ~ PB.15                                   *\n");
    printf("*   - nWR  on PL.5                                                     *\n");
    printf("*   - nRD  on PL.4                                                     *\n");
    printf("*   - nWRL on PL.11                                                    *\n");
    printf("*   - nWRH on PL.10                                                    *\n");
    printf("*   - nCS1 on PJ.1                                                     *\n");
    printf("*   - ALE  on PB.11                                                    *\n");
    printf("*   - MCLK on PG.13                                                    *\n");
    printf("************************************************************************\n\n");


    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOWEST, EBI_OPMODE_ADSEPARATE, EBI_CS_ACTIVE_LOW);


    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);
    if((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        while(1);
    }
    else
    {
        printf("PASS !!!\n\n");
    }


    /* Step 2, erase chip */
    if(NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        while(1);


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;
    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        if(NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData);
            while(1);
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData);
        }
    }

    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);
        if(u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData, u16RData);
            while(1);
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16RData);
        }
    }
    printf(">> Program flash OK !!!                             \n\n");

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    while(1);
}

