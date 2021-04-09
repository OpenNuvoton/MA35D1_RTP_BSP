/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Read the smartcard ATR from smartcard 1 interface.
 *
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF 0  // Smartcard interface 0

/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
    // Card insert/remove event occurred, no need to check other event...
    if(SCLIB_CheckCDEvent(SC_INTF))
        return;
    // Check if there's any timeout event occurs. If so, it usually indicates an error
    SCLIB_CheckTimeOutEvent(SC_INTF);
    // Check transmit and receive interrupt, all data transmission take place in this function
    SCLIB_CheckTxRxEvent(SC_INTF);
    /*
        Check if there's any transmission error occurred (e.g. parity error, frame error...)
        These errors will induce SCLIB to deactivation smartcard eventually.
    */
    SCLIB_CheckErrorEvent(SC_INTF);

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
    // Valid smartcard clock rate is from 1MHz to 4MHz, we set it to 4MHz
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL4_SC0SEL_HXT, CLK_CLKDIV1_SC0(6));
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set MFP for SC0 interface */
    SYS->GPF_MFPH |= SYS_GPF_MFPH_PF10MFP_SC0_CLK |
                     SYS_GPF_MFPH_PF11MFP_SC0_DAT |
                     SYS_GPF_MFPH_PF12MFP_SC0_RST |
                     SYS_GPF_MFPH_PF13MFP_SC0_PWR;
    SYS->GPI_MFPL |= SYS_GPI_MFPL_PI0MFP_SC0_nCD;

    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    SCLIB_CARD_INFO_T s_info;
    int retval, i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("\nThis sample code reads ATR from smartcard\n");

    /*
        Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
        The second and third parameter needs to be set according to the board design
    */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    // Wait 'til card insert
    while(SC_IsCardInserted(SC0) == FALSE);

    /*
        Activate slot 0, and disable EMV2000 check during card activation
        EMV is a technical standard for smart payment cards and for payment terminals and automated teller
        machines that can accept them. It has a stricter checking rule than ISO 7816-3. If the second
        parameter set as TRUE, SCLIB will report activation failure for cards comply with ISO 7816 but not EMV2000
    */
    retval = SCLIB_Activate(SC_INTF, FALSE);

    if(retval == SCLIB_SUCCESS)
    {
        /*
            Use SCLIB_GetCardInfo to get information about the card, which includes ATR.

            An Answer To Reset (ATR) is a message output by a contact Smart Card conforming to
            ISO/IEC 7816 standards, following electrical reset of the card's chip by a card reader.
            The ATR conveys information about the communication parameters proposed by the card,
            and the card's nature and state.                                --Wikipedia
        */
        SCLIB_GetCardInfo(SC_INTF, &s_info);
        printf("ATR: ");
        for(i = 0; i < s_info.ATR_Len; i++)
            printf("%02x ", s_info.ATR_Buf[i]);
        printf("\n");
    }
    else
        printf("Smartcard activate failed\n");

    // No operating system, so we have no where to go, just loop forever.
    while(1);
}




