/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate PDMA2 channel 1 get/clear timeout flag with UART1.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_TEST_LENGTH 100
#define PDMA_TIME 0x5555

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=4
static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];
#else
__attribute__((aligned(4))) static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
__attribute__((aligned(4))) static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];
#endif

volatile uint32_t u32IsTxTestOver = 0;
volatile uint32_t u32IsRxTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA2_IRQHandler(void);
void UART_PDMATest(void);


void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(PDMA2_MODULE);
    CLK_EnableModuleClock(PDMA3_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set multi-function pins */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    SYS->GPL_MFPL &= ~(SYS_GPL_MFPL_PL4MFP_Msk | SYS_GPL_MFPL_PL5MFP_Msk);
    SYS->GPL_MFPL |= (SYS_GPL_MFPL_PL4MFP_UART1_RXD | SYS_GPL_MFPL_PL5MFP_UART1_TXD);
    
    /* Lock protected registers */
    SYS_LockReg();
}

void UART16_Init()
{
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
}

void UART1_Init()
{
    UART_Open(UART1, 115200);
}

void PDMA2_Init(void)
{
    /* Open PDMA2 Channel */
    PDMA_Open(PDMA2,1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA2,1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA2,0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA2,1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA2,0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA2,1, PDMA_WIDTH_8, PDMA_TEST_LENGTH+1);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA2,0, ((uint32_t) (&g_u8Tx_Buffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA2,1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA2,0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA2,1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    //PDMA_SetTimeOut(0, 0, 0x5555);
    PDMA_SetTimeOut(PDMA2,1, 1, PDMA_TIME);

    PDMA_EnableInt(PDMA2,0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA2,1, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA2,1, PDMA_INT_TIMEOUT);
    NVIC_EnableIRQ(PDMA2_IRQn);
    u32IsRxTestOver = 0;
    u32IsTxTestOver = 0;
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART for printf */
    UART16_Init();

    /* Init UART1 */
    UART1_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    UART_PDMATest();

    while(1);
}

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

    if (status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");
        if (PDMA_GET_ABORT_STS(PDMA2) & 0x1){
            u32IsTxTestOver = 2;
        }
        if (PDMA_GET_ABORT_STS(PDMA2) & 0x2){
            u32IsRxTestOver = 2;
        }
        PDMA_CLR_ABORT_FLAG(PDMA2,PDMA_GET_ABORT_STS(PDMA2));
    }
    else if (status & 0x2)     /* done */
    {
        if ( (PDMA_GET_TD_STS(PDMA2) & (1 << 0)))
        {
            u32IsTxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA2,(1 << 0));
        }
        
        if ( (PDMA_GET_TD_STS(PDMA2) & (1 << 1)))
        {
            u32IsRxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA2,(1 << 1));
        }
    }
    else if (status & 0x100)     /* channel 0 timeout */
    {
        u32IsTxTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA2,0);
    }
    else if (status & 0x200)     /* channel 1 timeout */
    {
        u32IsRxTestOver = 3;
        PDMA_SetTimeOut(PDMA2,1, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA2,1);
        PDMA_SetTimeOut(PDMA2,1, 0, PDMA_TIME);
    }
    else
        printf("unknown interrupt !!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART PDMA Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest()
{
    uint32_t i;

    printf("+-----------------------------------------------------------+\n");
    printf("|  PDMA2 timeout Test                                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo PDMA timeout function.       |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    /*
        Using UART1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
        UART1_TX :  Total transfer length =  (PDMA_TEST_LENGTH  ) * 8 bits
        UART1_RX :  Total transfer length =  (PDMA_TEST_LENGTH+1) * 8 bits
    */

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    PDMA2_Init();

    UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk;

    while(u32IsTxTestOver == 0);

    if (u32IsTxTestOver == 1)
        printf("UART1 TX transfer done...\n");
    else if (u32IsTxTestOver == 2)
        printf("UART1 TX transfer abort...\n");
    else if (u32IsTxTestOver == 3)
        printf("UART1 TX timeout...\n");
        
    while(u32IsRxTestOver == 0);

    if (u32IsRxTestOver == 1)
        printf("UART1 RX transfer done...\n");
    else if (u32IsRxTestOver == 2)
        printf("UART1 RX transfer abort...\n");
    else if (u32IsRxTestOver == 3){
        printf("UART1 RX timeout...\n");
    }

    UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
    UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

    printf("PDMA2 timeout test Pass \n\n");
    while(1);

}





