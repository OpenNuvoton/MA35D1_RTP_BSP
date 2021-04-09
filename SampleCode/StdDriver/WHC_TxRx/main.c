/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate Wormhole Controller (WHC) transmit and receive function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t au32RxBuf[WHC_BUFFER_LEN];
uint32_t au32TxBuf[WHC_BUFFER_LEN];

void WHC0_IRQHandler(void)
{
    // Clear interrupt flag
    WHC_CLR_INT_FLAG(WHC0, WHC_INTSTS_RX0IF_Msk);

    WHC_Recv(WHC0, 0, au32RxBuf);

    printf("Received message: %08x, %08x, %08x, %08x\n", au32RxBuf[0], au32RxBuf[1], au32RxBuf[2], au32RxBuf[3]);

}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main (void)
{
    uint32_t i = 0;
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("\nThis sample code demonstrate Wormhole Controller(WHC) transmit and receive function\n");

    WHC_ENABLE_INT(WHC0, WHC_INTEN_RX0IEN_Msk);
    NVIC_EnableIRQ(WHC0_IRQn);

    while(1)
    {
        if(WHC_IS_TX_READY(WHC0, 0))
        {
            au32TxBuf[0] = i++;
            au32TxBuf[1] = i++;
            au32TxBuf[2] = i++;
            au32TxBuf[3] = i++;

            WHC_Send(WHC0, 0, au32TxBuf);
        }
    }
}

