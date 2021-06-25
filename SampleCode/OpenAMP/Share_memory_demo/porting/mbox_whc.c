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

uint32_t WHC0_RX_Flag = RX_NO_MSG;
uint32_t WHC0_TX_Flag = TX_NO_ACK;


//Initialize WHC
int Mbox_Init(void)
{
    if(mbox_ch == 0)
    {
        WHC_ENABLE_INT(WHC0, WHC_INTEN_RX0IEN_Msk);
        WHC_ENABLE_INT(WHC0, WHC_INTEN_TX0IEN_Msk);
    }
    else if(mbox_ch == 1)
    {
        WHC_ENABLE_INT(WHC0, WHC_INTEN_RX1IEN_Msk);
        WHC_ENABLE_INT(WHC0, WHC_INTEN_TX1IEN_Msk);
    }
    else if(mbox_ch == 2)
    {
        WHC_ENABLE_INT(WHC0, WHC_INTEN_RX2IEN_Msk);
        WHC_ENABLE_INT(WHC0, WHC_INTEN_TX2IEN_Msk);
    }

    NVIC_EnableIRQ(WHC0_IRQn);

    return 0;
}

uint32_t au32RxCom[4];
uint32_t au32TxCom[4];

uint32_t au32RxBuf[4];
uint32_t au32TxBuf[4];

// Poll mailbox
int Mbox_Poll(struct rpmsg_endpoint *ept)
{
    if (WHC0_RX_Flag == RX_NEW_MSG)
    {
        ept->cb(ept, &au32RxBuf, (size_t)au32RxBuf[1], SHM_START_ADDRESS, NULL);

        WHC0_RX_Flag = RX_NO_MSG;

        au32TxBuf[0] = COMMAND_RETURN_ACK; // response ack

        WHC_Send(WHC0, mbox_ch, au32TxBuf);

        while((WHC0->TXSTS & 0xf) != 0xf); // wait Tx

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
    uint32_t i;

    if(WHC_IS_RX_READY(WHC0, mbox_ch))
    {
        WHC_Recv(WHC0, mbox_ch, &au32RxCom[0]);

        if(au32RxCom[0] == COMMAND_RECEIVE_A35_MSG)
        {
            WHC0_RX_Flag = RX_NEW_MSG;

            for(i = 0; i < 4; i++)
                au32RxBuf[i] = au32RxCom[i];
        }
        else if(au32RxCom[0] == COMMAND_RECEIVE_A35_ACK)
        {
            WHC0_TX_Flag = TX_ACK;
        }

        // Clear interrupt flag
        WHC_CLR_INT_FLAG(WHC0, WHC_INTSTS_RX2IF_Msk);
    }

    WHC_CLR_INT_FLAG(WHC0, WHC0->TXSTS << WHC_INTSTS_TX0IF_Pos);
}


