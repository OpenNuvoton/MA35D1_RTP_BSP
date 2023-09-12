/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate OpenAMP share memory control via SDRAM.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "rpmsg.h"
#include "openamp.h"
#include "mbox_whc.h"


/* Definition of Test State */
#define A35_SEND_TO_M4    1
#define M4_SEND_TO_A35    2

/* Definition of Test Tx/Rx Size */
#define TX_RX_SIZE    SHM_TX_RX_SIZE

static uint32_t g_u32TestState = 0;
static uint32_t g_u32RxStatus  = 0;

uint8_t g_au8ReceivedRPMsg[TX_RX_SIZE];
uint8_t g_au8TransmitRPMsg[TX_RX_SIZE];

static int Rx_Callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IP clock */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main (void)
{
    struct rpmsg_endpoint resmgr_ept;
    uint32_t i;
    int32_t ret;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    printf("\nThis sample code demonstrate OpenAMP share memory function (RTP M4)\n\n");
    printf("Share Memory Address (SDRAM): 0x%08x\n", SHM_START_ADDRESS);
    printf("TX/RX Buffer Size (Byte)    : %d\n\n", SHM_TX_RX_SIZE);

    /* Init M4 Tx data */
    for (i = 0; i < TX_RX_SIZE; i++)
    {
        g_au8TransmitRPMsg[i] = i;
    }

    MA35D1_OpenAMP_Init(RPMSG_REMOTE, NULL);

    OPENAMP_create_endpoint(&resmgr_ept, "rpmsg-sample", RPMSG_ADDR_ANY, Rx_Callback, NULL);

    while (1)
    {
        OPENAMP_check_for_message(&resmgr_ept);

        if (g_u32TestState == A35_SEND_TO_M4)
        {
            if (g_u32RxStatus)
            {
                g_u32RxStatus = 0;

#ifdef CHECK_RX_DATA
                /* !! Note: comparing will reduce performance. */
                for (i = 0; i < TX_RX_SIZE; i++)
                {
                    if (g_au8ReceivedRPMsg[i] != (i % 256))
                    {
                        printf(" The data received is wrong\n");
                        printf(" Stop test ...\n");
                        printf("i = %d, value = %d\n", i, g_au8ReceivedRPMsg[i]);
                        while (1);
                    }
                }
#endif
            }
        }

        if (g_u32TestState == M4_SEND_TO_A35)
        {
            ret = OPENAMP_send_data(&resmgr_ept, g_au8TransmitRPMsg, TX_RX_SIZE);
            if (ret < 0)
            {
                printf("Failed to send message\r\n");
                printf("Stop test ...\n");
                while (1);
            }

            while (1)
            {
                if (OPENAMP_check_TxAck(&resmgr_ept) == 1)
                break;
            }
        }
    }

}

static int Rx_Callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv)
{
    uint32_t *u32Command = (uint32_t *)data;
#ifdef CHECK_RX_DATA
    uint32_t i;
#endif

    if (*u32Command == COMMAND_RECEIVE_A35_MSG)
    {
        g_u32RxStatus = 1;
        memcpy((void *)g_au8ReceivedRPMsg, (const void *)src, len > sizeof(g_au8ReceivedRPMsg) ? sizeof(g_au8ReceivedRPMsg) : len);

#ifdef CHECK_RX_DATA
        printf("\n Receive %d bytes data from A35: \n", len);
        /* !! Note: printf will reduce performance. */
        for (i = 0; i < len; i++)
        {
            if (i % 10 == 0)
                printf("\n");
            printf("%3d ", g_au8ReceivedRPMsg[i]);
        }
        printf("\n\n");
#endif
    }
    else if (*u32Command == COMMAND_RECEIVE_A35_TEST_START)
    {
        if (len == COMMAND_RECEIVE_A35_TEST_TX)
        {
            g_u32TestState = A35_SEND_TO_M4;
        }
        else if (len == COMMAND_RECEIVE_A35_TEST_RX)
        {
            g_u32TestState = M4_SEND_TO_A35;
        }
    }
    else
    {
        printf("\n Unknown command!! \n");
    }

    return 0;
}


