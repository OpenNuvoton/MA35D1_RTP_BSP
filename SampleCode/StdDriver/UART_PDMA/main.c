/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate UART transmit and receive function with PDMA
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA                   ((PDMA_T *)  PDMA2_BASE)

#define ENABLE_PDMA_INTERRUPT 1
#define PDMA_TEST_LENGTH 128
#define PDMA_TIME 0x5555

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH] __attribute__((aligned(512)));
static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH] __attribute__((aligned(512)));

volatile uint32_t u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void);
void UART_PDMATest(void);


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable IP clock */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART1SEL_HXT, CLK_CLKDIV1_UART1(1));
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(PDMA2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;
    /* Set multi-function pins for UART1 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_UART1_nCTS | SYS_GPA_MFPL_PA1MFP_UART1_nRTS |
                     SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init()
{
    UART_Open(UART16, 115200);
}

void UART1_Init()
{
    UART_Open(UART1, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA,1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA,1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA,0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA,1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA,0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA,1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA,0, ((uint32_t) (&g_u8Tx_Buffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA,1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA,0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA,1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    //PDMA_SetTimeOut(0, 0, 0x5555);
    //PDMA_SetTimeOut(1, 0, 0x5555);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(PDMA,0, 0);
    PDMA_EnableInt(PDMA,1, 0);
    NVIC_EnableIRQ(PDMA2_IRQn);
    u32IsTestOver = 0;
#endif
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART_Init();

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
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");
        if (PDMA_GET_ABORT_STS(PDMA) & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA,PDMA_GET_ABORT_STS(PDMA));
    }
    else if (status & 0x2)     /* done */
    {
        if ( (PDMA_GET_TD_STS(PDMA) & (1 << 0)) && (PDMA_GET_TD_STS(PDMA) & (1 << 1)) )
        {
            u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA,PDMA_GET_TD_STS(PDMA));
        }
    }
    else if (status & 0x300)     /* channel 2 timeout */
    {
        printf("timeout interrupt !!\n");
        u32IsTestOver = 3;

        PDMA_SetTimeOut(PDMA,0, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA,0);
        PDMA_SetTimeOut(PDMA,0, 1, PDMA_TIME);

        PDMA_SetTimeOut(PDMA,1, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA,1);
        PDMA_SetTimeOut(PDMA,1, 1, PDMA_TIME);
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
    printf("|  UART PDMA Test                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 PDMA function.         |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    /*
        Using UAR1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
    */

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    while(1)
    {
        PDMA_Init();

        UART1->FIFO |= (0x3 << 1);
        while(UART1->FIFO & (0x3 << 1));

        UART1->INTEN = UART_INTEN_RLSIEN_Msk; // Enable Receive Line interrupt
        NVIC_EnableIRQ(UART1_IRQn);

        UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk;

#ifdef ENABLE_PDMA_INTERRUPT
        while(u32IsTestOver == 0);

        if (u32IsTestOver == 1)
            printf("test done...\n");
        else if (u32IsTestOver == 2)
            printf("target abort...\n");
        else if (u32IsTestOver == 3)
            printf("timeout...\n");
#else
        while( (!(PDMA_GET_TD_STS() & (1 << 0))) || (!(PDMA_GET_TD_STS() & (1 << 1))) );

        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF0_Msk|PDMA_TDSTS_TDIF1_Msk);
#endif

        UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

        for (i=0; i<PDMA_TEST_LENGTH; i++)
        {
            if(g_u8Rx_Buffer[i] != i)
            {
                printf("\n Receive Data Compare Error !!");
                while(1);
            }

            g_u8Rx_Buffer[i] = 0xff;
        }

        printf("\nUART PDMA test Pass.\n");
    }

}

void UART1_IRQHandler(void)
{
    uint32_t u32DAT;
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_PRLSIF_Msk)
    {
        if(UART1->FIFOSTS & UART_FIFOSTS_BIF_Msk)
            printf("\n BIF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_FEF_Msk)
            printf("\n FEF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_PEF_Msk)
            printf("\n PEF \n");

        u32DAT = UART1->DAT; // read out data
        printf("\n Error Data is '0x%x' \n", u32DAT);
        UART1->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
    }
}





