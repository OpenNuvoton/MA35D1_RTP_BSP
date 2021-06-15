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

#define M4_COMMAND_ACK 0x81

static uint32_t rx_status = 0;

uint32_t received_rpmsg[128];

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
    uint32_t u32msg;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);

    printf("\nThis sample code demonstrate OpenAMP share memory function\n");

    MA35D1_OpenAMP_Init(RPMSG_REMOTE, NULL);
    OPENAMP_create_endpoint(&resmgr_ept, "rpmsg-sample", RPMSG_ADDR_ANY, rx_callback, NULL);

    while(1)
    {
        OPENAMP_check_for_message();

        if(rx_status)
        {
            break;
        }
    }

    u32msg = M4_COMMAND_ACK;
    if (OPENAMP_send(&resmgr_ept, &u32msg, 1))
    {
        printf("Failed to send message\r\n");
    }

    while(1);
}

static int rx_callback(struct rpmsg_endpoint *rp_chnl, void *data, size_t len, uint32_t src, void *priv)
{
    /* copy received msg, and raise a flag  */
    memcpy(received_rpmsg, data, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : len);
    rx_status = 1;
    return 0;
}


