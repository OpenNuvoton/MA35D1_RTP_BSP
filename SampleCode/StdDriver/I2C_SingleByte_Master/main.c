/**************************************************************************//**
 * @file     main.c
 *
 * @brief
 *           Show how to use I2C Signle byte API Read and Write data to Slave
 *           Needs to work with I2C_Slave sample code.
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for Debug UART RXD and TXD */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD;
    /* Set multi-function pins for I2C1 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB10MFP_I2C1_SDA | SYS_GPB_MFPH_PB11MFP_I2C1_SCL;

    /* I2C pin enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN10_Msk | GPIO_SMTEN_SMTEN11_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C1_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C1, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C1, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C1, 3, 0x75, 0);   /* Slave Address : 0x75 */
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);

}


int32_t main(void)
{
    uint32_t i;
    uint8_t u8data, u8tmp, err;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART16, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for Single Byte Read/Write Test |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C1) <---> I2C Slave(I2C1)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");

    /* Init I2C1 */
    I2C1_Init();

    /* Slave Address */
    g_u8DeviceAddr = 0x15;

    err = 0;

    for(i = 0; i < 256; i++)
    {
        u8tmp = (uint8_t)i + 3;

        /* Single Byte Write (Two Registers) */
        while(I2C_WriteByteTwoRegs(I2C1, g_u8DeviceAddr, i, u8tmp));

        /* Single Byte Read (Two Registers) */
        u8data = I2C_ReadByteTwoRegs(I2C1, g_u8DeviceAddr, i);
        if(u8data != u8tmp)
        {
            err = 1;
            printf("%03d: Single byte write data fail,  W(0x%X)/R(0x%X) \n", i, u8tmp, u8data);
        }
    }

    printf("\n");

    if(err)
        printf("Single byte Read/Write access Fail.....\n");
    else
        printf("Single byte Read/Write access Pass.....\n");

    while(1);
}



