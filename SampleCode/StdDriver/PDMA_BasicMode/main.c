/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Use PDMA2 channel 2 to transfer data from memory to memory.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8SrcArray[256];
uint8_t au8DestArray[256];
#else
__attribute__((aligned(4))) uint8_t au8SrcArray[256];
__attribute__((aligned(4))) uint8_t au8DestArray[256];
#endif

uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ.
 */
void PDMA2_IRQHandler(void)
{
    volatile uint32_t status = PDMA_GET_INT_STATUS(PDMA2);
    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA2) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA2,PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA2) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA2,PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(PDMA2_MODULE);
    CLK_EnableModuleClock(PDMA3_MODULE);
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

void UART16_Init()
{
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART16_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA2 Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");


    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    PDMA_Open(PDMA2,1 << 2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA2,2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA2,2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA2,2, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA2,2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA2,2, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA2_IRQn);
    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA_Trigger(PDMA2,2);

    /* Waiting for transfer done */
    while(g_u32IsTestOver == 0);

    /* Check transfer result */
    if(g_u32IsTestOver == 1)
        printf("test done...\n");
    else if(g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    PDMA_Close(PDMA2);

    while(1);
}
