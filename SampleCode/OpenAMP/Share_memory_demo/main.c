/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate OpenAMP share memory control.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "rpmsg.h"
#include "openamp.h"
#include "mbox_whc.h"

#define M4_COMMAND_ACK 0x81

static uint32_t rx_status = 0;

#define tx_rx_size 128
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
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV1_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);

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

    MA35D1_OpenAMP_Init(RPMSG_REMOTE, NULL);
    OPENAMP_create_endpoint(&resmgr_ept, "rpmsg-sample", RPMSG_ADDR_ANY, rx_callback, NULL);

    while(1)
    {
        OPENAMP_check_for_message(&resmgr_ept);

        if(rx_status)
        {
            rx_status = 0;
            break;
        }
    }

    for(i = 0; i < tx_rx_size; i++)
    {
        transmit_rpmsg[i] = i;
    }

    ret = OPENAMP_send_data(&resmgr_ept, transmit_rpmsg, 5);
    if (ret < 0)
    {
        printf("Failed to send message\r\n");
    }

    printf("\n Transfer %d bytes data to A35 \n", ret);

    while(1)
    {
        if(OPENAMP_check_TxAck(&resmgr_ept) == 1)
        break;
    }

    printf("\n Test END !!\n");

    while(1);
}

static int rx_callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv)
{
    uint32_t *u32Command = (uint32_t *)data;
    uint32_t i;

    if(*u32Command == COMMAND_RECEIVE_A35_MSG)
    {
        memcpy((void *)received_rpmsg, (const void *)src, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : len);

        printf("\n Receive %d bytes data from A35: \n", len);
        for(i = 0; i < len; i++)
        {
            printf(" 0x%x \n", received_rpmsg[i]);
        }

        rx_status = 1;
    }
    else
    {
        printf("\n unknow command!! \n");
    }

    return 0;
}


