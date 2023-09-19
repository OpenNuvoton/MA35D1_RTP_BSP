/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN bus communication.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFifO0CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len);
void CAN_ShowRecvMessage(void);
void CAN_RxTest(void);
void CAN_TxTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
    /*Recieve the Rx Fifo0 message */
    CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
    g_u8RxFifO0CompleteFlag = 1;

    if(CANFD0->IR == CANFD_IR_BO_Msk)
    {
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        CANFD0->CCCR &= ~CANFD_CCCR_INIT_Msk;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable IP clock */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL4_CANFD0SEL_APLL, 0);
    CLK_EnableModuleClock(CANFD0_MODULE);
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PK multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;
    /* Set PK multi-function pins for CANFD0 RXD and TXD */
    SYS->GPI_MFPL &= ~(SYS_GPI_MFPL_PI2MFP_Msk | SYS_GPI_MFPL_PI3MFP_Msk);
    SYS->GPI_MFPL |= SYS_GPI_MFPL_PI2MFP_CAN0_RXD | SYS_GPI_MFPL_PI3MFP_CAN0_TXD;
}

/*---------------------------------------------------------------------------------------------------------*/
/*              CAN Function Test Menu(Master)                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TestItem(void)
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|              CAN Tx Function Test (Master)                |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111    (Date Lenght 8bytes)           |\n");
    printf("| [2] Standard ID = 0x22F    (Date Lenght 8bytes)           |\n");
    printf("| [3] Standard ID = 0x333    (Date Lenght 8bytes)           |\n");
    printf("| [4] Extended ID = 0x221    (Date Lenght 8bytes)           |\n");
    printf("| [5] Extended ID = 0x3333   (Date Lenght 8bytes)           |\n");
    printf("| [6] Extended ID = 0x44444  (Date Lenght 8bytes)           |\n");
    printf("| Select ID number and master will send message to slave ...|\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Function Tx Test                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CAN_TestItem();
        u8Item = getchar();

        switch (u8Item)
        {
            case '1':
                /*Standard ID =0x111,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
                break;

            case '2':
                /*Standard ID =0x22F,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
                break;

            case '3':
                /*Standard ID =0x333,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
                break;

            case '4':
                /*Extend ID =0x111,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
                break;

            case '5':
                /*Extend ID =0x3333,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
                break;

            case '6':
                /*Extend ID =0x44444,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
                break;

            default:
                break;
        }

    } while (u8Item != 27);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Send Message Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;

    psTxMsg->u32Id = u32Id;
    psTxMsg->eIdType = eIdType;
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC = u8Len;

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFifO0CompleteFlag = 0;

    /* use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != 1)
    {
        printf("Failed to transmit message\n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN CAN Function Rx Test                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_RxTest(void)
{
    uint8_t u8Cnt = 0;
    printf("Start CAN bus reception :\n");

    do
    {
        while (!g_u8RxFifO0CompleteFlag) {}

        CAN_ShowRecvMessage();
        g_u8RxFifO0CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    } while (u8Cnt < 6);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN Receive Message Function                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02d bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                    Init CAN                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PA5)                         CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PA4)                         CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CAN     |  CAN_H  |    CAN    |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /*Get the CAN configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;

    /* Reset CAN IP */
    SYS_UnlockReg();
    SYS_ResetModule(CANFD0_RST);
    SYS_LockReg();

    /*Open the CAN feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110~0x11F in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* receive 0x220 ~ 0x22F in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* receive 0x220 (29-bit id) in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x4444 (29-bit id) in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Enable Standard ID and  Extended ID Filter as RX FOFI0*/
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Tx Rx Interrupt Function Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN interface initialization*/
    CAN_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                      CAN Function Test                                   |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and  |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with    |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if      |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                  |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        CAN_TxTest();
    }
    else
    {
        CAN_RxTest();
    }

    printf("CANFD Sample Code End.\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    /* print a note to terminal */
    printf("\nCAN bus communication example\r\n");

    CAN_TxRxINTTest();

    while (1) {}
}
