/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate smartcard UART mode by connecting PA.0 and PA.1 pins.
 *
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// This is the string we used in loopback demo
uint8_t au8TxBuf[] = "Hello World!";


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    while(!SCUART_GET_RX_EMPTY(SC0)) {
        putchar(SCUART_READ(SC0));
        fflush(stdout);
    }
    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.

    return;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_EnableModuleClock(UART16_MODULE);
    /* Select IP clock source */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL4_SC0SEL_HXT, CLK_CLKDIV1_SC0(1));
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set PF.10 and PF.11 pin for SC UART mode */
    /* Smartcard CLK pin is used for TX, and DAT pin is used for Rx */
    SYS->GPF_MFPH |= (SYS_GPF_MFPH_PF10MFP_SC0_CLK | SYS_GPF_MFPH_PF11MFP_SC0_DAT);
    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("This sample code demos smartcard interface UART mode\n");
    printf("Please connect SC0 CLK pin(PF.10) with SC0 DAT pin(PF.11)\n");
    printf("Hit any key to continue\n");
    getchar();

    // Open smartcard interface 0 in UART mode. The line config will be 115200-8n1
    // Can call SCUART_SetLineConfig() later if necessary
    SCUART_Open(SC0, 115200);

    // Enable receive interrupt, no need to use other interrupts in this demo
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    // Send the demo string out from SC0_CLK pin
    // Received data from SC0_DAT pin will be print out to UART console
    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    // Loop forever. There's no where to go without an operating system.
    while(1);
}




