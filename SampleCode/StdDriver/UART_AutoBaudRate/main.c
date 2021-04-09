/**************************************************************************//**
 * @file     main.c
 *
 *           Show how to use auto baud rate detection function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void);
void AutoBaudRate_TxTest(void);
void AutoBaudRate_RxTest(void);


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable IP clock */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV1_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART1SEL_HXT, CLK_CLKDIV1_UART1(1));
    CLK_EnableModuleClock(UART1_MODULE);

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
    /* Reset UART0 */
    SYS_ResetModule(UART16_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART16, 115200);
}

void UART1_Init()
{
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART_Init();

    /* Init UART1 */
    UART1_Init();


    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test()
{

    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD    <=========>     UART1_RXD--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = getchar();

    if(u32Item == '0')
        AutoBaudRate_TxTest();
    else
        AutoBaudRate_RxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest()
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item)
        {
        case '1':
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 38400);
            break;
        case '2':
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 57600);
            break;
        default:
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
            break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART1, 0x1);

    }
    while(u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T* uart)
{
    uint8_t u8UartClkSrcSel = 0, u8UartClkDivNum = 0;
    uint32_t u32ClkTbl[4] = {__HXT, 0ul, __LXT, __HIRC};
    uint32_t u32Baud_Div;

    /* Get UART clock source selection and UART clock divider number */
    u8UartClkSrcSel = (CLK->CLKSEL2 & CLK_CLKSEL2_UART1SEL_Msk) >> CLK_CLKSEL2_UART1SEL_Pos;
    u8UartClkDivNum = (CLK->CLKDIV1 & CLK_CLKDIV1_UART1DIV_Msk) >> CLK_CLKDIV1_UART1DIV_Pos;

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetSYSCLK1Freq();

    /* Get UART baud rate divider */
    u32Baud_Div = (uart->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2)) / (((uart->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1);

    /* Unsupported baud rate setting */
    else
        return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_RxTest()
{
    /* Enable auto baud rate detect function */
    UART1->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while((UART1->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0);

    if(UART1->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetUartBaudrate(UART1));
    }
    else if(UART1->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}
