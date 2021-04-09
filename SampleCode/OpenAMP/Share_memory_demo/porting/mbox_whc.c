/**************************************************************************//**
 * @file     mbox_whc.c
 *
 * @brief    OpenAMP mailbox example source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "openamp/open_amp.h"
#include "NuMicro.h"
#include "openamp_conf.h"

#define RX_NO_MSG           1
#define RX_NEW_MSG          2
#define RX_BUF_FREE         3

uint32_t WHC0_RX0_Flag = RX_NO_MSG;


//Initialize WHC
int Mbox_Init(void)
{
    WHC_ENABLE_INT(WHC0, WHC_INTEN_RX0IEN_Msk);
    WHC_ENABLE_INT(WHC0, WHC_INTEN_TX0IEN_Msk);

    WHC_ENABLE_INT(WHC0, WHC_INTEN_RX1IEN_Msk);
    WHC_ENABLE_INT(WHC0, WHC_INTEN_TX1IEN_Msk);

    NVIC_EnableIRQ(WHC0_IRQn);

    return 0;
}

// Poll mailbox
int Mbox_Poll(struct virtio_device *vdev)
{
    static uint32_t test_count = 0;

    test_count++;

    if(test_count == 0x1000)
    {
        WHC0_RX0_Flag = RX_NEW_MSG;
    }

    if (WHC0_RX0_Flag == RX_NEW_MSG)
    {
        printf("\n Rx: RX_NEW_MSG \n");

        rproc_virtio_notified(vdev, VRING1_ID);
        WHC0_RX0_Flag = RX_NO_MSG;

        /* The OpenAMP framework does not notify for free buf: do it here */
        rproc_virtio_notified(NULL, VRING1_ID);
        return 0;
    }

  return -1;
}

// OpenAMP callback function
int Mbox_Notify(void *priv, uint32_t id)
{
    return 0;
}

// WHC IRQ handler
void WHC0_IRQHandler(void)
{
    uint32_t au32RxBuf;
    uint32_t au32TxBuf;

    if(WHC_IS_RX_READY(WHC0, 0))
    {
        printf("\n WHC_INTSTS_RX0 \n");

        WHC0_RX0_Flag = RX_NEW_MSG;

        WHC_Recv(WHC0, 0, &au32RxBuf);

        // Clear interrupt flag
        WHC_CLR_INT_FLAG(WHC0, WHC_INTSTS_RX0IF_Msk);
        au32TxBuf = 0x55aa5a5a; // response ack
        WHC_Send(WHC0, 0, &au32TxBuf);
    }
}


