/**************************************************************************//**
 * @file     main.c
 *
 * @brief    Demonstrate hardware semaphore (HWSEM) Signal function.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define HWSEM_CH_A35     0
#define HWSEM_CH_CM4     4
#define CM4_KEY          0x55

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
}

static uint32_t flag = 0;
void HWSEM0_IRQHandler(void)
{
	uint32_t reg;

	reg = HWSEM0->INTSTSM4;

	flag = HWSEM_GET_INT_FLAG(HWSEM0, HWSEM_CH_A35);

	// Clear flags
	HWSEM0->INTSTSM4 = reg;
}

/* Signal to A35 and check status */
void Signal_Task(uint32_t ch, uint8_t key)
{
	/* Try lock */
	HWSEM_Spin_Lock(HWSEM0, ch, key);

	/* Unlock to issue INT */
	HWSEM_UNLOCK(HWSEM0, ch, key);

	// Check lock & INT status, A35 should clear this
	if(HWSEM_CHK_INT_FLAG(HWSEM0, ch))
	{
		printf("---Reply signal to A35---\n");
	}
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
    printf("\nThis sample code demonstrate Hardware Semaphore(HWSEM) Signal function\n");

	// Enable HWSEM INT
	HWSEM_ENABLE_INT(HWSEM0, HWSEM_CH_A35);
	NVIC_EnableIRQ(HWSEM0_IRQn);

    while(1)
    {
        // If a signal from A35 is received, reply a signal
		if(flag)
		{
			flag = 0;

			printf("---Get signal from A35---\n");

			Signal_Task(HWSEM_CH_CM4, CM4_KEY);
		}
    }

}

