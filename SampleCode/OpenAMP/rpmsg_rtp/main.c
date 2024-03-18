/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate OpenAMP share memory control.
 *
 *           This project supports rpmsg v1 or v2 by utilizing SRAM or DRAM as
 *           shared memory. Users are able to configure it in openamp_conf.h.
 *           Default settings is rpmsg v1 with shared memory in SRAM
 *
 *           Note:
 *                1. This project is compatible with MA35D1 Linux driver and
 *                   is "NOT" designed for non-OS.
 *                2. If recursive read/write is necessary in your application,
 *                   it is highly recommended to use the v2 architecture.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "rpmsg.h"
#include "openamp.h"
#include "mbox_whc.h"

#define RPMSG_DEBUG_LEVEL 1

#define M4_COMMAND_ACK 0x81
/**
 * @brief CMD frame
 * ---Driver Level---
 * CMD(4 bytes) + LEN(4 bytes)
 *     M4 Tx : COMMAND_SEND_MSG_TO_A35 = A35 Rx
 *     M4 Rx : COMMAND_RECEIVE_A35_MSG = A35 Tx
 *
 * ---Appl Level---
 * SubCMD(4 bytes) + ServerSEQ(4 bytes) + ClientSEQ(4 bytes) + reserved(4 bytes) + payload(0-112 bytes)
 * As server :
 *     Tx : Send ServerSEQ, Rx : Check for ClientSEQ
 * As client :
 *     Rx : Receive ServerSEQ, Tx : Reply ClientSEQ
 */
#define SUBCMD_LEN     16
#define SUBCMD_DIGITS  0xFFFF
#define SUBCMD_START   0x01
#define SUBCMD_SUSPEND 0x02
#define SUBCMD_RESUME  0x01
#define SUBCMD_EXIT    0x08
#define SUBCMD_SEQ     0x10
#define SUBCMD_SEQACK  0x20
#define SUBCMD_ERROR   0x8000

#define SEQ_DIGITS        0xFFFFFFFF
#define SERVER_SEQ_OFFSET 4
#define CLIENT_SEQ_OFFSET 8

uint32_t tx_server_seq = 0, rx_client_seq = 0; // M4 as server
uint32_t rx_server_seq = 0, tx_client_seq = 0; // M4 as client
volatile uint32_t status_flag = 0;
volatile uint32_t rpmsg_wnd = 1;

#define tx_rx_size     Share_Memory_Size/2
#ifndef RPMSG_DDR_BUF
  #if (tx_rx_size > 0x80)
    #error Maximum memory size for SRAM is 0x80 bytes
  #endif
#else
  #if (tx_rx_size < 0x80) || (tx_rx_size > 0x4000)
    #error Memory size for DRAM should be between 0x80 and 0x4000 bytes
  #endif
#endif

uint8_t received_rpmsg[tx_rx_size];
uint8_t transmit_rpmsg[tx_rx_size];

static int rx_callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

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

void prepare_txdata(uint8_t *data, int *len)
{
    int i;
    *len = tx_rx_size - SUBCMD_LEN;
    for(i = 0; i < *len; i++)
    {
        data[i] = i;
    }
}

void receive_rxdata(uint8_t *data, int len)
{
#if (RPMSG_DEBUG_LEVEL > 1)
    printf("\n Receive %d bytes data from A35: \n", len);
    for(int i = 0; i < len; i++)
    {
        printf(" 0x%x ", data[i]);
    }
    printf("\n");
#endif
}

int32_t main (void)
{
    struct rpmsg_endpoint resmgr_ept;
    int ret;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);

    printf("\nThis sample code demonstrate OpenAMP share memory function\n");
    printf("RPMSG version : v%d\n", RPMSG_VERSION);
    printf("Shared memory starts at %s 0x%08x, size 0x%04x\n",
        (SHM_START_ADDRESS > 0x80000000ul) ? "DRAM" : "SRAM", SHM_START_ADDRESS, Share_Memory_Size);

    MA35D1_OpenAMP_Init(RPMSG_REMOTE, NULL);
    OPENAMP_create_endpoint(&resmgr_ept, "rpmsg-sample", RPMSG_ADDR_ANY, rx_callback, NULL);

#ifdef RPMSG_V2_ARCH
    int len = 112;
    uint32_t subCmd[4];

    while(1)
    {
        while(1)
        {
            OPENAMP_check_for_message(&resmgr_ept);

            if(status_flag & SUBCMD_SEQACK)
                status_flag &= ~SUBCMD_SEQACK;
            if(status_flag & SUBCMD_START)
                break;
        }

        memset(subCmd, 0, sizeof(subCmd));

        // Update ack seq from client
        int seq_diff = abs((int)(tx_server_seq - rx_client_seq));

        // Check for A35 keeping up before send something
        if(seq_diff < rpmsg_wnd)
        {
            // CMD set
            subCmd[0] |= SUBCMD_SEQ;
            subCmd[1] |= ++tx_server_seq & SEQ_DIGITS;
            prepare_txdata(transmit_rpmsg + SUBCMD_LEN, &len);
            len += SUBCMD_LEN;
        }

        // Received data from A35, attach an ack
        if(status_flag & SUBCMD_SEQ)
        {
            subCmd[0] |= SUBCMD_SEQACK;
            subCmd[2] |= tx_client_seq & SEQ_DIGITS;

            status_flag &= ~SUBCMD_SEQ;
        }

        if(subCmd[0] & SUBCMD_DIGITS) // SEQ or ACK
        {
            memcpy((void *)transmit_rpmsg, (void *)subCmd, SUBCMD_LEN);

            while(!WHC_IS_TX_READY(WHC0, mbox_ch));
            ret = OPENAMP_send_data(&resmgr_ept, transmit_rpmsg, SUBCMD_LEN + len);
            if (ret < 0)
            {
                printf("Failed to send message\r\n");
            }
#if (RPMSG_DEBUG_LEVEL > 0)
            if(tx_server_seq%1000 == 0)
                printf("Send #%d\n", subCmd[1]);
#endif
            while(1)
            {
                if(OPENAMP_check_TxAck(&resmgr_ept) == 1)
                    break;
            }
        }

        if(status_flag & SUBCMD_EXIT)
			break;
    }

#else

    do {
			OPENAMP_check_for_message(&resmgr_ept);
		} while (status_flag != SUBCMD_START);

    for (int i = 0; i < tx_rx_size; i++)
        transmit_rpmsg[i] = i;

    ret = OPENAMP_send_data(&resmgr_ept, transmit_rpmsg, 5);
    if (ret < 0)
        printf("Failed to send message\r\n");
    else
        printf("\n Transfer %d bytes data to A35 \n", ret);

    while (1)
    {
        if (OPENAMP_check_TxAck(&resmgr_ept) == 1)
            break;
    }
#endif /* End of RPMSG_V2_ARCH */

    printf("\n Test END !!\n");

    while(1);
}

static int rx_callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv)
{
    if(*(uint32_t *)data == COMMAND_RECEIVE_A35_MSG)
    {
#ifdef RPMSG_V2_ARCH
        memcpy((void *)received_rpmsg, (const void *)src, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : (len < SUBCMD_LEN ? SUBCMD_LEN : len));
        uint32_t subcmd = *(uint32_t *)received_rpmsg & SUBCMD_DIGITS;

        if(subcmd & SUBCMD_START)
        {
            status_flag |= SUBCMD_START;
            if(rpmsg_wnd != *(uint32_t *)(received_rpmsg + SUBCMD_LEN))
            {
                rpmsg_wnd = *(uint32_t *)(received_rpmsg + SUBCMD_LEN);
                printf("Window size of rpmsg changed to %d\n", rpmsg_wnd);
            }
        }
        else if(subcmd & SUBCMD_SUSPEND)
            status_flag &= ~SUBCMD_START;
        else if(subcmd & SUBCMD_EXIT)
            status_flag |= SUBCMD_EXIT;

        if(subcmd & SUBCMD_SEQ)
        {
            rx_server_seq = *(uint32_t *)(received_rpmsg + SERVER_SEQ_OFFSET) & SEQ_DIGITS;
            tx_client_seq = rx_server_seq;
            status_flag |= SUBCMD_SEQ;
#if (RPMSG_DEBUG_LEVEL > 0)
            if(tx_client_seq%1000 == 0)
            {
                printf("seq #%d\n", tx_client_seq);
                receive_rxdata(received_rpmsg + SUBCMD_LEN, len - SUBCMD_LEN);
            }
#endif
            // send ack later
        }

        if(subcmd & SUBCMD_SEQACK)
        {
            status_flag |= SUBCMD_SEQACK;
            rx_client_seq = *(uint32_t *)(received_rpmsg + CLIENT_SEQ_OFFSET) & SEQ_DIGITS;
#if (RPMSG_DEBUG_LEVEL > 0)
            if(rx_client_seq%1000 == 0)
                printf("seqack #%d\n", rx_client_seq);
#endif
            //status_flag |= SUBCMD_SEQACK;
            // compare with tx_server_seq later
        }

#else

        memcpy((void *)received_rpmsg, (const void *)src, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : (len < SUBCMD_LEN ? SUBCMD_LEN : len));

		printf("\n Receive %d bytes data from A35: \n", len);
		for(int i = 0; i < len; i++)
		{
			printf(" 0x%x \n", received_rpmsg[i]);
		}
        status_flag = SUBCMD_START;
#endif /* End of RPMSG_V2_ARCH */
    }
    else
    {
        printf("\n unknow command!! \n");
    }

    return 0;
}


