/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate hardware semaphore (HWSEM) lock/unlock function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define KEY     0x55

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();
    /* Enable UART clock */
    CLK_EnableModuleClock(UART16_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
    printf("\nThis sample code demonstrate Hardware Semaphore(HWSEM) lock and unlock function\n");


    while(1)
    {
        // HWSEM_Try_Lock() may failed, so need to check return value
        if(HWSEM_Try_Lock(HWSEM0, 0, KEY) == 0)
        {
            // Lock by us, access critical section, and then unlock
            HWSEM_UNLOCK(HWSEM0, 0, KEY);
        }
        else
        {
            // Lock by A35, can't access critical section
        }

        // HWSEM_Spin_Lock() returns after acquired the lock
        HWSEM_Spin_Lock(HWSEM0, 0, KEY);
        // Lock by us, access critical section, and then unlock
        HWSEM_UNLOCK(HWSEM0, 0, KEY);
    }

}

