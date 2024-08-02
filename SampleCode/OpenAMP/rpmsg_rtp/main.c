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
 *                3. The sample provides two modes: (a) Both cores send data
 *                   through a periodic timer, and (b) Free run mode, which
 *                   can be switched through "TXRX_FREE_RUN".
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

typedef int (*rpmsg_tx_cb_t)(unsigned char *, int *);
typedef void (*rpmsg_rx_cb_t)(unsigned char *, int);

volatile uint32_t rpmsg_wnd = 1;

struct rtp_rpmsg {
	unsigned int ack_ready;
	// A35 as server
	unsigned int tx_server_seq;
	unsigned int rx_client_seq;
	// A35 as client
	unsigned int rx_server_seq;
	unsigned int tx_client_seq;
	volatile unsigned int status_flag;

	unsigned int subCmd[4];
	int tx_en;
    int tx_trigger;
    rpmsg_tx_cb_t tx_cb;
	rpmsg_rx_cb_t rx_cb;
};

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

struct rtp_rpmsg rpmsg;
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

#if (TXRX_FREE_RUN == 0)
    /* set systick */
    CLK_EnableSysTick(CLK_CLKSEL0_RTPSTSEL_HXT, 24000000/ACK_TIMER_HZ);
    NVIC_EnableIRQ(SysTick_IRQn);
#endif

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * @brief Send data RTP -> A35
 *
 * @param data
 * @param len payload only
 * @return int
 */
int rpmsg_tx_cb(uint8_t *data, int *len)
{
    int i;
    *len = tx_rx_size - SUBCMD_LEN;
    for(i = 0; i < *len; i++)
    {
        data[i] = i;
    }

    return *len;
}

/**
 * @brief Receive data A35 -> RTP
 *
 * @param data
 * @param len payload only
 */
void rpmsg_rx_cb(uint8_t *data, int len)
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

int ma35_rpmsg_open(struct rtp_rpmsg *rpmsg, struct rpmsg_endpoint *resmgr_ept,
                    rpmsg_rx_cb_t rxcb, rpmsg_tx_cb_t txcb)
{
    MA35D1_OpenAMP_Init(RPMSG_REMOTE, NULL);
    OPENAMP_create_endpoint(resmgr_ept, "rpmsg-sample", RPMSG_ADDR_ANY, rx_callback, NULL);

    memset(rpmsg, 0, sizeof(rpmsg));
    rpmsg->rx_cb = rxcb;
    rpmsg->tx_cb = txcb;

    return 0;
}

int ma35_rpmsg_prepare(struct rpmsg_endpoint *resmgr_ept, struct rtp_rpmsg *rpmsg)
{
    do {
        OPENAMP_check_for_message(resmgr_ept);
        if(rpmsg->status_flag & SUBCMD_EXIT) {
            SYS_UnlockReg();
            CLK_DisableSysTick();
            SYS_LockReg();
		    return -1;
        }
    } while(!(rpmsg->status_flag & SUBCMD_START));

    return 0;
}

int rpmsg_ack_process(struct rpmsg_endpoint *resmgr_ept, struct rtp_rpmsg *rpmsg, int len)
{
    int ret;

    if(rpmsg->status_flag & SUBCMD_SEQ)
    {
        rpmsg->subCmd[0] |= SUBCMD_SEQACK;
        rpmsg->subCmd[2] = rpmsg->tx_client_seq & SEQ_DIGITS;

        rpmsg->status_flag &= ~SUBCMD_SEQ;
    }

    if(rpmsg->subCmd[0] & SUBCMD_DIGITS) // SEQ or ACK
    {
        memcpy((void *)transmit_rpmsg, (void *)rpmsg->subCmd, SUBCMD_LEN);

        while(!WHC_IS_TX_READY(WHC0, mbox_ch));
        ret = OPENAMP_send_data(resmgr_ept, transmit_rpmsg, len);
        if (ret < 0)
        {
            printf("Failed to send message\r\n");
        }

        while(1)
        {
            if(OPENAMP_check_TxAck(resmgr_ept) == 1)
                break;
        }
    }
    return 0;
}

/**
 * @return 0 : start tx, else : busy
 */
int rpmsg_tx_trigger()
{
    if(rpmsg.tx_trigger)
        return -1;
    rpmsg.tx_trigger = 1;
    return 0;
}

int rpmsg_reset()
{
    rpmsg.tx_server_seq = rpmsg.tx_client_seq = 0;
    rpmsg.rx_server_seq = rpmsg.rx_client_seq = 0;
    rpmsg.status_flag = 0;
    memset(rpmsg.subCmd, 0, sizeof(rpmsg.subCmd));
    CLK_EnableSysTick(CLK_CLKSEL0_RTPSTSEL_HXT, 24000000/ACK_TIMER_HZ);

    return 0;
}

/**
 * @return 1 : send finished, else : busy
 */
int rpmsg_tx_acked()
{
    return (rpmsg.tx_server_seq == rpmsg.rx_client_seq) && (rpmsg.tx_trigger == 0);
}

void SysTick_Handler()
{
    rpmsg.tx_trigger = 1;
}

int ma35_rpmsg_send(struct rpmsg_endpoint *resmgr_ept, struct rtp_rpmsg *rpmsg, int *len)
{
    int txlen;

    if(!(rpmsg->status_flag & SUBCMD_START))
        return 0;

    if(!rpmsg->tx_trigger)
    {
        if(rpmsg->status_flag & SUBCMD_SEQ)
        {
            if(rpmsg->tx_server_seq != rpmsg->rx_client_seq)
                txlen = tx_rx_size;
            else
                txlen = SUBCMD_LEN;
            rpmsg_ack_process(resmgr_ept, rpmsg, txlen);
        }

        return 0;
    }

    // CMD set
    if(rpmsg->tx_server_seq != rpmsg->rx_client_seq)
    {
        *len = SUBCMD_LEN;
    }
    else
    {
        rpmsg->subCmd[0] |= SUBCMD_SEQ;
        rpmsg->subCmd[1] = ++rpmsg->tx_server_seq & SEQ_DIGITS;
        rpmsg->tx_cb(transmit_rpmsg + SUBCMD_LEN, len);
        *len += SUBCMD_LEN;
    }

    rpmsg_ack_process(resmgr_ept, rpmsg, *len);

    rpmsg->tx_en = 1;
    rpmsg->tx_trigger = 0;

#if (RPMSG_DEBUG_LEVEL > 0)
        if(rpmsg->tx_server_seq%(ACK_TIMER_HZ*10) == 0 && rpmsg->tx_server_seq)
            printf("Send #%d\n", rpmsg->subCmd[1]);
#endif

    return 0;
}

int ma35_rpmsg_cmd_handler(struct rpmsg_endpoint *resmgr_ept, struct rtp_rpmsg *rpmsg)
{
    if(rpmsg->tx_en)
        rpmsg->tx_en = 0;
    else
        rpmsg_ack_process(resmgr_ept, rpmsg, SUBCMD_LEN);

    if(rpmsg->status_flag & SUBCMD_EXIT)
		return -1;

    return 0;
}

int32_t main (void)
{
    struct rpmsg_endpoint resmgr_ept;

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

    ma35_rpmsg_open(&rpmsg, &resmgr_ept, rpmsg_rx_cb, rpmsg_tx_cb);

#ifdef RPMSG_V2_ARCH
    int len;

    while(1)
    {
        ma35_rpmsg_prepare(&resmgr_ept, &rpmsg);
#if (TXRX_FREE_RUN == 1)
        rpmsg.tx_trigger = 1;
#endif
        ma35_rpmsg_send(&resmgr_ept, &rpmsg, &len);
    }

#else
    int ret;

    do {
			OPENAMP_check_for_message(&resmgr_ept);
		} while (rpmsg.status_flag != SUBCMD_START);

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

    printf("\n Test END !!\n");

    while(1);
#endif /* End of RPMSG_V2_ARCH */
}

static int rx_callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv)
{
    static int lastseq = -1;

    if(*(uint32_t *)data == COMMAND_RECEIVE_A35_MSG)
    {
#ifdef RPMSG_V2_ARCH
        memcpy((void *)received_rpmsg, (const void *)src, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : (len < SUBCMD_LEN ? SUBCMD_LEN : len));
        uint32_t subcmd = *(uint32_t *)received_rpmsg & SUBCMD_DIGITS;

        if(subcmd & SUBCMD_START)
        {
            rpmsg_reset();
            rpmsg.status_flag |= SUBCMD_START;
            if(rpmsg_wnd != *(uint32_t *)(received_rpmsg + SUBCMD_LEN))
            {
                rpmsg_wnd = *(uint32_t *)(received_rpmsg + SUBCMD_LEN);
                printf("Window size of rpmsg changed to %d\n", rpmsg_wnd);
            }
        }
        else if(subcmd & SUBCMD_SUSPEND)
            rpmsg.status_flag &= ~SUBCMD_START;
        else if(subcmd & SUBCMD_EXIT)
            rpmsg.status_flag |= SUBCMD_EXIT;

        if(subcmd & SUBCMD_SEQ)
        {
            rpmsg.rx_server_seq = *(uint32_t *)(received_rpmsg + SERVER_SEQ_OFFSET) & SEQ_DIGITS;
            rpmsg.tx_client_seq = rpmsg.rx_server_seq;
            if(lastseq != rpmsg.rx_server_seq)
            {
                rpmsg.status_flag |= SUBCMD_SEQ;
#if (RPMSG_DEBUG_LEVEL > 0)
                if(rpmsg.tx_client_seq%(ACK_TIMER_HZ*10) == 0 && rpmsg.tx_client_seq)
                {
                    printf("ack #%d\n", rpmsg.tx_client_seq);
                }
#endif
                rpmsg.rx_cb(received_rpmsg + SUBCMD_LEN, len - SUBCMD_LEN);
                lastseq = rpmsg.rx_server_seq;
            }
            // send ack later
        }

        if(subcmd & SUBCMD_SEQACK)
        {
            rpmsg.status_flag |= SUBCMD_SEQACK;
            rpmsg.rx_client_seq = *(uint32_t *)(received_rpmsg + CLIENT_SEQ_OFFSET) & SEQ_DIGITS;
        }

#else
        (void)lastseq;
        memcpy((void *)received_rpmsg, (const void *)src, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : (len < SUBCMD_LEN ? SUBCMD_LEN : len));

		printf("\n Receive %d bytes data from A35: \n", len);
		for(int i = 0; i < len; i++)
		{
			printf(" 0x%x \n", received_rpmsg[i]);
		}
        rpmsg.status_flag = SUBCMD_START;
#endif /* End of RPMSG_V2_ARCH */
    }
    else
    {
        printf("\n unknow command!! \n");
    }

    return 0;
}


