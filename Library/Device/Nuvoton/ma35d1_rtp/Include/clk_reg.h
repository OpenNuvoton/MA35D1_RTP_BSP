/**************************************************************************//**
 * @file     clk_reg.h
 * @brief    CLK register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct
{


    /**
     * @var CLK_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTEN     |HXT Enable Bit (Write Protect)
     * |        |          |The bit default value is set by flash controller user configuration register CONFIG0 [26]
     * |        |          |When the default clock source is from HXT, this bit is set to 1 automatically.
     * |        |          |0 = 4~24 MHz external high speed crystal (HXT) Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal (HXT) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |LXTEN     |LXT Enable Bit (Write Protect)
     * |        |          |0 = 32.768 kHz external low speed crystal (LXT) Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal (LXT) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 12 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 12 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |PDWKDLY   |Enable the Wake-up Delay Counter (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip works at 4~24 MHz external high speed crystal oscillator (HXT), and 256 clock cycles when chip works at 12 MHz internal high speed RC oscillator (HIRC).
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note1: The interrupt will occur when both PDWKIF and PDWKIEN are high.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
     * |        |          |The flag is set if any wake-up source is occurred. Refer Power Modes and Wake-up Sources chapter.
     * |        |          |Note1: Write 1 to clear the bit to 0.
     * |        |          |Note2: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1.
     * |[7]     |PDEN      |System Power-down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |In Power-down mode, HXT and the HIRC will be disabled in this mode, but LXT and LIRC are not controlled by Power-down mode.
     * |        |          |In Power-down mode, the PLL and system clock are disabled, and ignored the clock source selection
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from LXT or LIRC.
     * |        |          |0 = Chip will not enter Power-down mode after CPU sleep command WFI.
     * |        |          |1 = Chip enters Power-down mode after CPU sleep command WFI.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11:10] |HXTGAIN   |HXT Gain Control Bit (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally
     * |        |          |If gain control is enabled, crystal will consume more power than gain control off.
     * |        |          |00 = HXT frequency is lower than from 8 MHz.
     * |        |          |01 = HXT frequency is from 8 MHz to 12 MHz.
     * |        |          |10 = HXT frequency is from 12 MHz to 16 MHz.
     * |        |          |11 = HXT frequency is higher than 16 MHz.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |HXTSELTYP |HXT Crystal Type Select Bit (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = Select INV type.
     * |        |          |1 = Select GM type.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |HXTTBEN   |HXT Crystal TURBO Mode (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = HXT Crystal TURBO mode disabled.
     * |        |          |1 = HXT Crystal TURBO mode enabled.
     * |[17:16] |HIRCSTBS  |HIRC Stable Count Select (Write Protect)
     * |        |          |00 = HIRC stable count is 64 clocks.
     * |        |          |01 = HIRC stable count is 24 clocks.
     * |        |          |others = Reserved.
     * |[18]    |HIRCEN    |HIRC48M Enable Bit (Write Protect)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) Enabled.
     * @var CLK_T::AHBCLK
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |PDMACKEN  |PDMA Controller Clock Enable Bit
     * |        |          |0 = PDMA peripheral clock Disabled.
     * |        |          |1 = PDMA peripheral clock Enabled.
     * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     * |[3]     |EBICKEN   |EBI Controller Clock Enable Bit
     * |        |          |0 = EBI peripheral clock Disabled.
     * |        |          |1 = EBI peripheral clock Enabled.
     * |[5]     |EMACCKEN  |Ethernet Controller Clock Enable Bit
     * |        |          |0 = Ethernet Controller engine clock Disabled.
     * |        |          |1 = Ethernet Controller engine clock Enabled.
     * |[6]     |SDH0CKEN  |SD0 Controller Clock Enable Bit
     * |        |          |0 = SD0 engine clock Disabled.
     * |        |          |1 = SD0 engine clock Enabled.
     * |[7]     |CRCCKEN   |CRC Generator Controller Clock Enable Bit
     * |        |          |0 = CRC peripheral clock Disabled.
     * |        |          |1 = CRC peripheral clock Enabled.
     * |[10]    |HSUSBDCKEN|HSUSB Device Clock Enable Bit
     * |        |          |0 = HSUSB device controller's clock Disabled.
     * |        |          |1 = HSUSB device controller's clock Enabled.
     * |[12]    |CRPTCKEN  |Cryptographic Accelerator Clock Enable Bit
     * |        |          |0 = Cryptographic Accelerator clock Disabled.
     * |        |          |1 = Cryptographic Accelerator clock Enabled.
     * |[14]    |SPIMCKEN  |SPIM Controller Clock Enable Bit
     * |        |          |0 = SPIM controller clock Disabled.
     * |        |          |1 = SPIM controller clock Enabled.
     * |[15]    |FMCIDLE   |Flash Memory Controller Clock Enable Bit in IDLE Mode
     * |        |          |0 = FMC clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMC clock Enabled when chip is under IDLE mode.
     * |[16]    |USBHCKEN  |USB HOST Controller Clock Enable Bit
     * |        |          |0 = USB HOST peripheral clock Disabled.
     * |        |          |1 = USB HOST peripheral clock Enabled.
     * |[17]    |SDH1CKEN  |SD1 Controller Clock Enable Bit
     * |        |          |0 = SD1 engine clock Disabled.
     * |        |          |1 = SD1 engine clock Enabled.
     * @var CLK_T::APBCLK0
     * Offset: 0x08  APB Devices Clock Enable Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Watchdog timer clock Disabled.
     * |        |          |1 = Watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |RTCCKEN   |Real-time-clock APB Interface Clock Enable Bit
     * |        |          |This bit is used to control the RTC APB clock only
     * |        |          |The RTC peripheral clock source is selected from RTCSEL(CLK_CLKSEL3[8])
     * |        |          |It can be selected to 32.768 kHz external low speed crystal or 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = RTC clock Disabled.
     * |        |          |1 = RTC clock Enabled.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2CKEN  |Timer2 Clock Enable Bit
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3CKEN  |Timer3 Clock Enable Bit
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |CLKOCKEN  |CLKO Clock Enable Bit
     * |        |          |0 = CLKO clock Disabled.
     * |        |          |1 = CLKO clock Enabled.
     * |[7]     |ACMP01CKEN|Analog Comparator 0/1 Clock Enable Bit
     * |        |          |0 = Analog comparator 0/1 clock Disabled.
     * |        |          |1 = Analog comparator 0/1 clock Enabled.
     * |[8]     |I2C0CKEN  |I2C0 Clock Enable Bit
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[9]     |I2C1CKEN  |I2C1 Clock Enable Bit
     * |        |          |0 = I2C1 clock Disabled.
     * |        |          |1 = I2C1 clock Enabled.
     * |[10]    |I2C2CKEN  |I2C2 Clock Enable Bit
     * |        |          |0 = I2C2 clock Disabled.
     * |        |          |1 = I2C2 clock Enabled.
     * |[12]    |QSPI0CKEN  |QSPI0 Clock Enable Bit
     * |        |          |0 = QSPI0 clock Disabled.
     * |        |          |1 = QSPI0 clock Enabled.
     * |[13]    |SPI0CKEN  |SPI0 Clock Enable Bit
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[14]    |SPI1CKEN  |SPI1 Clock Enable Bit
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[15]    |SPI2CKEN  |SPI2 Clock Enable Bit
     * |        |          |0 = SPI2 clock Disabled.
     * |        |          |1 = SPI2 clock Enabled.
     * |[16]    |UART0CKEN |UART0 Clock Enable Bit
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Bit
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[18]    |UART2CKEN |UART2 Clock Enable Bit
     * |        |          |0 = UART2 clock Disabled.
     * |        |          |1 = UART2 clock Enabled.
     * |[19]    |UART3CKEN |UART3 Clock Enable Bit
     * |        |          |0 = UART3 clock Disabled.
     * |        |          |1 = UART3 clock Enabled.
     * |[20]    |UART4CKEN |UART4 Clock Enable Bit
     * |        |          |0 = UART4 clock Disabled.
     * |        |          |1 = UART4 clock Enabled.
     * |[21]    |UART5CKEN |UART5 Clock Enable Bit
     * |        |          |0 = UART5 clock Disabled.
     * |        |          |1 = UART5 clock Enabled.
     * |[24]    |CAN0CKEN  |CAN0 Clock Enable Bit
     * |        |          |0 = CAN0 clock Disabled.
     * |        |          |1 = CAN0 clock Enabled.
     * |[25]    |CAN1CKEN  |CAN1 Clock Enable Bit
     * |        |          |0 = CAN1 clock Disabled.
     * |        |          |1 = CAN1 clock Enabled.
     * |[26]    |OTGCKEN   |USB OTG Clock Enable Bit
     * |        |          |0 = USB OTG clock Disabled.
     * |        |          |1 = USB OTG clock Enabled.
     * |[27]    |USBDCKEN  |USB Device Clock Enable Bit
     * |        |          |0 = USB Device clock Disabled.
     * |        |          |1 = USB Device clock Enabled.
     * |[28]    |EADCCKEN  |Enhanced Analog-digital-converter (EADC) Clock Enable Bit
     * |        |          |0 = EADC clock Disabled.
     * |        |          |1 = EADC clock Enabled.
     * |[29]    |I2S0CKEN  |I2S0 Clock Enable Bit
     * |        |          |0 = I2S0 Clock Disabled.
     * |        |          |1 = I2S0 Clock Enabled.
     * |[30]    |HSOTGCKEN |HSUSB OTG Clock Enable Bit
     * |        |          |0 = HSUSB OTG clock Disabled.
     * |        |          |1 = HSUSB OTG clock Enabled.
     * @var CLK_T::APBCLK1
     * Offset: 0x0C  APB Devices Clock Enable Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SC0CKEN   |SC0 Clock Enable Bit
     * |        |          |0 = SC0 clock Disabled.
     * |        |          |1 = SC0 clock Enabled.
     * |[1]     |SC1CKEN   |SC1 Clock Enable Bit
     * |        |          |0 = SC1 clock Disabled.
     * |        |          |1 = SC1 clock Enabled.
     * |[2]     |SC2CKEN   |SC2 Clock Enable Bit
     * |        |          |0 = SC2 clock Disabled.
     * |        |          |1 = SC2 clock Enabled.
     * |[6]     |SPI3CKEN  |SPI3 Clock Enable Bit
     * |        |          |0 = SPI3 clock Disabled.
     * |        |          |1 = SPI3 clock Enabled.
     * |[8]     |USCI0CKEN |USCI0 Clock Enable Bit
     * |        |          |0 = USCI0 clock Disabled.
     * |        |          |1 = USCI0 clock Enabled.
     * |[9]     |USCI1CKEN |USCI1 Clock Enable Bit
     * |        |          |0 = USCI1 clock Disabled.
     * |        |          |1 = USCI1 clock Enabled.
     * |[12]    |DACCKEN   |DAC Clock Enable Bit
     * |        |          |0 = DAC clock Disabled.
     * |        |          |1 = DAC clock Enabled.
     * |[16]    |EPWM0CKEN |EPWM0 Clock Enable Bit
     * |        |          |0 = EPWM0 clock Disabled.
     * |        |          |1 = EPWM0 clock Enabled.
     * |[17]    |EPWM1CKEN |EPWM1 Clock Enable Bit
     * |        |          |0 = EPWM1 clock Disabled.
     * |        |          |1 = EPWM1 clock Enabled.
     * |[18]    |BPWM0CKEN |BPWM0 Clock Enable Bit
     * |        |          |0 = BPWM0 clock Disabled.
     * |        |          |1 = BPWM0 clock Enabled.
     * |[19]    |BPWM1CKEN |BPWM1 Clock Enable Bit
     * |        |          |0 = BPWM1 clock Disabled.
     * |        |          |1 = BPWM1 clock Enabled.
     * |[22]    |QEI0CKEN  |QEI0 Clock Enable Bit
     * |        |          |0 = QEI0 clock Disabled.
     * |        |          |1 = QEI0 clock Enabled.
     * |[23]    |QEI1CKEN  |QEI1 Clock Enable Bit
     * |        |          |0 = QEI1 clock Disabled.
     * |        |          |1 = QEI1 clock Enabled.
     * |[26]    |ECAP0CKEN |ECAP0 Clock Enable Bit
     * |        |          |0 = ECAP0 clock Disabled.
     * |        |          |1 = ECAP0 clock Enabled.
     * |[27]    |ECAP1CKEN |ECAP1 Clock Enable Bit
     * |        |          |0 = ECAP1 clock Disabled.
     * |        |          |1 = ECAP1 clock Enabled.
     * |[30]    |OPACKEN   |OP Amplifier (OPA) Clock Enable Bit
     * |        |          |0 = OPA clock Disabled.
     * |        |          |1 = OPA clock Enabled.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |Before clock switching, the related clock sources (both pre-select and new-select) must be turned on.
     * |        |          |The default value is reloaded from the value of CFOSC (CONFIG0[26]) in user configuration register of Flash controller by any reset
     * |        |          |Therefore the default value is either 000b or 111b.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PLL.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |111 = Clock source from HIRC.
     * |        |          |Other = Reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5:3]   |STCLKSEL  |Cortex-M4 SysTick Clock Source Selection (Write Protect)
     * |        |          |If SYST_CTRL[2]=0, SysTick uses listed clock source below.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from HXT/2.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |111 = Clock source from HIRC/2.
     * |        |          |Note: if SysTick clock source is not from HCLK (i.e
     * |        |          |SYST_CTRL[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |USBSEL    |USB Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from RC48M.
     * |        |          |1 = Clock source from PLL.
     * |[21:20] |SDH0SEL   |SD0 Engine Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source from HXT clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from HIRC clock.
     * |[23:22] |SDH1SEL   |SD1 Engine Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source from HXT clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from HIRC clock.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |00 = Reserved.
     * |        |          |01 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM0 pin.
     * |        |          |101 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM1 pin.
     * |        |          |101 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[18:16] |TMR2SEL   |TIMER2 Clock Source Selection
     * |        |          |000 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM2 pin.
     * |        |          |101 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[22:20] |TMR3SEL   |TIMER3 Clock Source Selection
     * |        |          |000 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM3 pin.
     * |        |          |101 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[25:24] |UART0SEL  |UART0 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[27:26] |UART1SEL  |UART1 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[29:28] |CLKOSEL   |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[31:30] |WWDTSEL   |Window Watchdog Timer Clock Source Selection
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Others = Reserved.
     * @var CLK_T::CLKSEL2
     * Offset: 0x18  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EPWM0SEL  |EPWM0 Clock Source Selection
     * |        |          |The peripheral clock source of EPWM0 is defined by EPWM0SEL.
     * |        |          |0 = Clock source from PLL.
     * |        |          |1 = Clock source from PCLK0.
     * |[1]     |EPWM1SEL  |EPWM1 Clock Source Selection
     * |        |          |The peripheral clock source of EPWM1 is defined by EPWM1SEL.
     * |        |          |0 = Clock source from PLL.
     * |        |          |1 = Clock source from PCLK1.
     * |[3:2]   |QSPI0SEL   |QSPI0 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[5:4]   |SPI0SEL   |SPI0 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK1.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[7:6]   |SPI1SEL   |SPI1 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[8]     |BPWM0SEL  |BPWM0 Clock Source Selection
     * |        |          |The peripheral clock source of BPWM0 is defined by BPWM0SEL.
     * |        |          |0 = Clock source from PLL.
     * |        |          |1 = Clock source from PCLK0.
     * |[9]     |BPWM1SEL  |BPWM1 Clock Source Selection
     * |        |          |The peripheral clock source of BPWM1 is defined by BPWM1SEL.
     * |        |          |0 = Clock source from PLL.
     * |        |          |1 = Clock source from PCLK1.
     * |[11:10] |SPI2SEL   |SPI2 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK1.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[13:12] |SPI3SEL   |SPI3 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * @var CLK_T::CLKSEL3
     * Offset: 0x1C  Clock Source Select Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SC0SEL    |SC0 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[3:2]   |SC1SEL    |SC0 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK1.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[5:4]   |SC2SEL    |SC2 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[8]     |RTCSEL    |RTC Clock Source Selection
     * |        |          |0 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |1 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |[17:16] |I2S0SEL   |I2S0 Clock Source Selection
     * |        |          |00 = Clock source from HXT clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |10 = Clock source from PCLK.
     * |        |          |11 = Clock source from HIRC clock.
     * |[25:24] |UART2SEL  |UART2 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[27:26] |UART3SEL  |UART3 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[29:28] |UART4SEL  |UART4 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * |[31:30] |UART5SEL  |UART5 Clock Source Selection
     * |        |          |00 = Clock source from 4~24 MHz external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from 32.768 kHz external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from 12 MHz internal high speed RC oscillator (HIRC).
     * @var CLK_T::CLKDIV0
     * Offset: 0x20  Clock Divider Number Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[7:4]   |USBDIV    |USB Clock Divide Number From PLL Clock
     * |        |          |USB clock frequency = (PLL frequency) / (USBDIV + 1).
     * |[11:8]  |UART0DIV  |UART0 Clock Divide Number From UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |[15:12] |UART1DIV  |UART1 Clock Divide Number From UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |[23:16] |EADCDIV   |EADC Clock Divide Number From EADC Clock Source
     * |        |          |EADC clock frequency = (EADC clock source frequency) / (EADCDIV + 1).
     * |[31:24] |SDH0DIV   |SD0 Clock Divide Number From SD0 Clock Source
     * |        |          |SD0 clock frequency = (SD0 clock source frequency) / (SDH0DIV + 1).
     * @var CLK_T::CLKDIV1
     * Offset: 0x24  Clock Divider Number Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SC0DIV    |SC0 Clock Divide Number From SC0 Clock Source
     * |        |          |SC0 clock frequency = (SC0 clock source frequency ) / (SC0DIV + 1).
     * |[15:8]  |SC1DIV    |SC1 Clock Divide Number From SC1 Clock Source
     * |        |          |SC1 clock frequency = (SC1 clock source frequency ) / (SC1DIV + 1).
     * |[23:16] |SC2DIV    |SC2 Clock Divide Number From SC2 Clock Source
     * |        |          |SC2 clock frequency = (SC2 clock source frequency ) / (SC2DIV + 1).
     * @var CLK_T::CLKDIV3
     * Offset: 0x2C  Clock Divider Number Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:16] |EMACDIV   |Ethernet Clock Divide Number Form HCLK
     * |        |          |EMAC MDCLK clock frequency = (HCLK) / (EMACDIV + 1).
     * |[31:24] |SDH1DIV   |SD1 Clock Divide Number From SD1 Clock Source
     * |        |          |SD1 clock frequency = (SD1 clock source frequency) / (SDH1DIV + 1).
     * @var CLK_T::CLKDIV4
     * Offset: 0x30  Clock Divider Number Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART2DIV  |UART2 Clock Divide Number From UART2 Clock Source
     * |        |          |UART2 clock frequency = (UART2 clock source frequency) / (UART2DIV + 1).
     * |[7:4]   |UART3DIV  |UART3 Clock Divide Number From UART3 Clock Source
     * |        |          |UART3 clock frequency = (UART3 clock source frequency) / (UART3DIV + 1).
     * |[11:8]  |UART4DIV  |UART4 Clock Divide Number From UART4 Clock Source
     * |        |          |UART4 clock frequency = (UART4 clock source frequency) / (UART4DIV + 1).
     * |[15:12] |UART5DIV  |UART5 Clock Divide Number From UART5 Clock Source
     * |        |          |UART5 clock frequency = (UART5 clock source frequency) / (UART5DIV + 1).
     * @var CLK_T::PCLKDIV
     * Offset: 0x34  APB Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |APB0DIV   |APB0 Clock Divider
     * |        |          |APB0 clock can be divided from HCLK
     * |        |          |000: PCLK0 = HCLK.
     * |        |          |001: PCLK0 = 1/2 HCLK.
     * |        |          |010: PCLK0 = 1/4 HCLK.
     * |        |          |011: PCLK0 = 1/8 HCLK.
     * |        |          |100: PCLK0 = 1/16 HCLK.
     * |        |          |Others: Reserved.
     * |[6:4]   |APB1DIV   |APB1 Clock Divider
     * |        |          |APB1 clock can be divided from HCLK
     * |        |          |000: PCLK1 = HCLK.
     * |        |          |001: PCLK1 = 1/2 HCLK.
     * |        |          |010: PCLK1 = 1/4 HCLK.
     * |        |          |011: PCLK1 = 1/8 HCLK.
     * |        |          |100: PCLK1 = 1/16 HCLK.
     * |        |          |Others: Reserved.
     * @var CLK_T::PLLCTL
     * Offset: 0x40  PLL Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FBDIV     |PLL Feedback Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13:9]  |INDIV     |PLL Input Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[15:14] |OUTDIV    |PLL Output Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |PD        |Power-down Mode (Write Protect)
     * |        |          |If set the PDEN bit to 1 in CLK_PWRCTL register, the PLL will enter Power-down mode, too.
     * |        |          |0 = PLL is in normal mode.
     * |        |          |1 = PLL is in Power-down mode (default).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17]    |BP        |PLL Bypass Control (Write Protect)
     * |        |          |0 = PLL is in normal mode (default).
     * |        |          |1 = PLL clock output is same as PLL input clock FIN.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[18]    |OE        |PLL OE (FOUT Enable) Pin Control (Write Protect)
     * |        |          |0 = PLL FOUT Enabled.
     * |        |          |1 = PLL FOUT is fixed low.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[19]    |PLLSRC    |PLL Source Clock Selection (Write Protect)
     * |        |          |0 = PLL source clock from 4~24 MHz external high-speed crystal oscillator (HXT).
     * |        |          |1 = PLL source clock from 12 MHz internal high-speed oscillator (HIRC).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[23]    |STBSEL    |PLL Stable Counter Selection (Write Protect)
     * |        |          |0 = PLL stable time is 6144 PLL source clock (suitable for source clock is equal to or less than 12 MHz).
     * |        |          |1 = PLL stable time is 12288 PLL source clock (suitable for source clock is larger than 12 MHz).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::STATUS
     * Offset: 0x50  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTSTB    |HXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock is not stable or disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock is stable and enabled.
     * |[1]     |LXTSTB    |LXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is not stable or disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock is stabled and enabled.
     * |[2]     |PLLSTB    |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable and enabled.
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 12 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 12 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |        |          |Note: This bit is read only.
     * |[6]     |HIRC48MSTB|HIRC 48MHz Clock Source Stable Flag (Read Only)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |        |          |Note: This bit is read only.
     * |[7]     |CLKSFAIL  |Clock Switching Fail Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 0
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * @var CLK_T::CLKOCTL
     * Offset: 0x60  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     * |[6]     |CLK1HZEN  |Clock Output 1Hz Enable Bit
     * |        |          |0 = 1 Hz clock output for 32.768 kHz frequency compensation Disabled.
     * |        |          |1 = 1 Hz clock output for 32.768 kHz frequency compensation Enabled.
     * @var CLK_T::CLKDCTL
     * Offset: 0x70  Clock Fail Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDEN   |HXT Clock Fail Detector Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail detector Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail detector Enabled.
     * |[5]     |HXTFIEN   |HXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail interrupt Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail interrupt Enabled.
     * |[12]    |LXTFDEN   |LXT Clock Fail Detector Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Enabled.
     * |[13]    |LXTFIEN   |LXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Enabled.
     * |[16]    |HXTFQDEN  |HXT Clock Frequency Range Detector Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector Enabled.
     * |[17]    |HXTFQIEN  |HXT Clock Frequency Range Detector Interrupt Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Enabled.
     * @var CLK_T::CLKDSTS
     * Offset: 0x74  Clock Fail Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTFIF    |HXT Clock Fail Interrupt Flag
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock is normal.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock stops.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * |[1]     |LXTFIF    |LXT Clock Fail Interrupt Flag
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is normal.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) stops.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * |[8]     |HXTFQIF   |HXT Clock Frequency Range Detector Interrupt Flag
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency is normal.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency is abnormal.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * @var CLK_T::CDUPB
     * Offset: 0x78  Clock Frequency Range Detector Upper Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |UPERBD    |HXT Clock Frequency Range Detector Upper Boundary Value
     * |        |          |The bits define the maximum value of frequency range detector window.
     * |        |          |When HXT frequency higher than this maximum frequency value, the HXT Clock Frequency Range Detector Interrupt Flag will set to 1.
     * @var CLK_T::CDLOWB
     * Offset: 0x7C  Clock Frequency Range Detector Lower Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |LOWERBD   |HXT Clock Frequency Range Detector Lower Boundary Value
     * |        |          |The bits define the minimum value of frequency range detector window.
     * |        |          |When HXT frequency lower than this minimum frequency value, the HXT Clock Frequency Range Detector Interrupt Flag will set to 1.
     * @var CLK_T::PMUCTL
     * Offset: 0x90  Power Manager Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PDMSEL    |Power-down Mode Selection (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |These bits control chip power-down mode grade selection when CPU execute WFI/WFE instruction.
     * |        |          |000 = Power-down mode is selected. (PD)
     * |        |          |001 = Low leakage Power-down mode is selected (LLPD).
     * |        |          |010 =Fast wake-up Power-down mode is selected (FWPD).
     * |        |          |011 = Reserved.
     * |        |          |100 = Standby Power-down mode 0 is selected (SPD0) (SRAM retention).
     * |        |          |101 = Standby Power-down mode 1 is selected (SPD1).
     * |        |          |110 = Deep Power-down mode is selected (DPD).
     * |        |          |111 = Reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |DPDHOLDEN |Deep-Power-Down Mode GPIO Hold Enable
     * |        |          |0 = When GPIO enters deep power-down mode, all I/O status are tri-state.
     * |        |          |1 = When GPIO enters deep power-down mode, all I/O status are hold to keep normal operating status.
     * |        |          |    After chip was waked up from deep power-down mode, the I/O are still keep hold status until user set CLK_IOPDCTL[0]
     * |        |          |    to release I/O hold status.
     * |[8]     |WKTMREN   |Wake-up Timer Enable (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = Wake-up timer disable at DPD/SPD mode.
     * |        |          |1 = Wake-up timer enabled at DPD/SPD mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11:9]  |WKTMRIS   |Wake-up Timer Time-out Interval Select (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |These bits control wake-up timer time-out interval when chip at DPD/SPD mode.
     * |        |          |000 = Time-out interval is 128 OSC10K clocks (12.8 ms).
     * |        |          |001 = Time-out interval is 256 OSC10K clocks (25.6 ms).
     * |        |          |010 = Time-out interval is 512 OSC10K clocks (51.2 ms).
     * |        |          |011 = Time-out interval is 1024 OSC10K clocks (102.4ms).
     * |        |          |100 = Time-out interval is 4096 OSC10K clocks (409.6ms).
     * |        |          |101 = Time-out interval is 8192 OSC10K clocks (819.2ms).
     * |        |          |110 = Time-out interval is 16384 OSC10K clocks (1638.4ms).
     * |        |          |111 = Time-out interval is 65536 OSC10K clocks (6553.6ms).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17:16] |WKPINEN   |Wake-up Pin Enable (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |00 = Wake-up pin disable at Deep Power-down mode.
     * |        |          |01 = Wake-up pin rising edge enabled at Deep Power-down mode.
     * |        |          |10 = Wake-up pin falling edge enabled at Deep Power-down mode.
     * |        |          |11 = Wake-up pin both edge enabled at Deep Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[18]    |ACMPSPWK  |ACMP Standby Power-down Mode Wake-up Enable (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = ACMP wake-up disable at Standby Power-down mode.
     * |        |          |1 = ACMP wake-up enabled at Standby Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[23]    |RTCWKEN   |RTC Wake-up Enable (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = RTC wake-up disable at Deep Power-down mode or Standby Power-down mode.
     * |        |          |1 = RTC wake-up enabled at Deep Power-down mode or Standby Power-down mode.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::PMUSTS
     * Offset: 0x94  Power Manager Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PINWK     |Pin Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Deep Power-down mode was requested by a transition of the WAKEUP pin (GPC.0)
     * |        |          |This flag is cleared when DPD mode is entered.
     * |[1]     |TMRWK     |Timer Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Deep Power-down mode (DPD) or Standby Power-down (SPD) mode was requested by wakeup timer time-out
     * |        |          |This flag is cleared when DPD or SPD mode is entered.
     * |[2]     |RTCWK     |RTC Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Deep Power-down mode (DPD) or Standby Power-down (SPD) mode was requested with a RTC alarm, tick time or tamper happened
     * |        |          |This flag is cleared when DPD or SPD mode is entered.
     * |[8]     |GPAWK     |GPA Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Standby Power-down mode was requested by a transition of selected one GPA group pins
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[9]     |GPBWK     |GPB Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Standby Power-down mode was requested by a transition of selected one GPB group pins
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[10]    |GPCWK     |GPC Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Standby Power-down mode was requested by a transition of selected one GPC group pins
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[11]    |GPDWK     |GPD Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wake-up of chip from Standby Power-down mode was requested by a transition of selected one GPD group pins
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[12]    |LVRWK     |LVR Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Standby Power-down mode was requested with a LVR happened
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[13]    |BODWK     |BOD Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Standby Power-down mode (SPD) was requested with a BOD happened
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[14]    |ACMPWK    |ACMP Wake-up Flag (Read Only)
     * |        |          |This flag indicates that wakeup of device from Standby Power-down mode (SPD) was requested with a ACMP transition
     * |        |          |This flag is cleared when SPD mode is entered.
     * |[31]    |CLRWK     |Clear Wake-up Flag
     * |        |          |0 = No clear.
     * |        |          |1 = Clear all wake-up flag.
     * @var CLK_T::LDOCTL
     * Offset: 0x98  LDO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18]    |PDBIASEN  |Power-down Bias Enable Bit
     * |        |          |0 = Reserved.
     * |        |          |1 = Power-down bias enabled.
     * |        |          |Note: This bit should set to 1 before chip enter power-down mode.
     * @var CLK_T::SWKDBCTL
     * Offset: 0x9C  Standby Power-down Wake-up De-bounce Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SWKDBCLKSEL|Standby Power-down Wake-up De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample wake-up input once per 1 clocks.
     * |        |          |0001 = Sample wake-up input once per 2 clocks.
     * |        |          |0010 = Sample wake-up input once per 4 clocks.
     * |        |          |0011 = Sample wake-up input once per 8 clocks.
     * |        |          |0100 = Sample wake-up input once per 16 clocks.
     * |        |          |0101 = Sample wake-up input once per 32 clocks.
     * |        |          |0110 = Sample wake-up input once per 64 clocks.
     * |        |          |0111 = Sample wake-up input once per 128 clocks.
     * |        |          |1000 = Sample wake-up input once per 256 clocks.
     * |        |          |1001 = Sample wake-up input once per 2*256 clocks.
     * |        |          |1010 = Sample wake-up input once per 4*256 clocks.
     * |        |          |1011 = Sample wake-up input once per 8*256 clocks.
     * |        |          |1100 = Sample wake-up input once per 16*256 clocks.
     * |        |          |1101 = Sample wake-up input once per 32*256 clocks.
     * |        |          |1110 = Sample wake-up input once per 64*256 clocks.
     * |        |          |1111 = Sample wake-up input once per 128*256 clocks.
     * |        |          |Note: De-bounce counter clock source is the 10 kHz internal low speed RC oscillator (LIRC).
     * @var CLK_T::PASWKCTL
     * Offset: 0xA0  GPA Standby Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Standby Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPA group pin wake-up function disabled.
     * |        |          |1 = GPA group pin wake-up function enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPA group pin rising edge wake-up function disabled.
     * |        |          |1 = GPA group pin rising edge wake-up function enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPA group pin falling edge wake-up function disabled.
     * |        |          |1 = GPA group pin falling edge wake-up function enabled.
     * |[7:4]   |WKPSEL    |GPA Standby Power-down Wake-up Pin Select
     * |        |          |0000 = GPA.0 wake-up function enabled.
     * |        |          |0001 = GPA.1 wake-up function enabled.
     * |        |          |0010 = GPA.2 wake-up function enabled.
     * |        |          |0011 = GPA.3 wake-up function enabled.
     * |        |          |0100 = GPA.4 wake-up function enabled.
     * |        |          |0101 = GPA.5 wake-up function enabled.
     * |        |          |0110 = GPA.6 wake-up function enabled.
     * |        |          |0111 = GPA.7 wake-up function enabled.
     * |        |          |1000 = GPA.8 wake-up function enabled.
     * |        |          |1001 = GPA.9 wake-up function enabled.
     * |        |          |1010 = GPA.10 wake-up function enabled.
     * |        |          |1011 = GPA.11 wake-up function enabled.
     * |        |          |1100 = GPA.12 wake-up function enabled.
     * |        |          |1101 = GPA.13 wake-up function enabled.
     * |        |          |1110 = GPA.14 wake-up function enabled.
     * |        |          |1111 = GPA.15 wake-up function enabled.
     * |[8]     |DBEN      |GPA Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding IO
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator.
     * |        |          |0 = Standby power-down wake-up pin De-bounce function disable.
     * |        |          |1 = Standby power-down wake-up pin De-bounce function enable.
     * |        |          |The de-bounce function is valid only for edge triggered.
     * @var CLK_T::PBSWKCTL
     * Offset: 0xA4  GPB Standby Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Standby Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPB group pin wake-up function disabled.
     * |        |          |1 = GPB group pin wake-up function enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPB group pin rising edge wake-up function disabled.
     * |        |          |1 = GPB group pin rising edge wake-up function enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPB group pin falling edge wake-up function disabled.
     * |        |          |1 = GPB group pin falling edge wake-up function enabled.
     * |[7:4]   |WKPSEL    |GPB Standby Power-down Wake-up Pin Select
     * |        |          |0000 = GPB.0 wake-up function enabled.
     * |        |          |0001 = GPB.1 wake-up function enabled.
     * |        |          |0010 = GPB.2 wake-up function enabled.
     * |        |          |0011 = GPB.3 wake-up function enabled.
     * |        |          |0100 = GPB.4 wake-up function enabled.
     * |        |          |0101 = GPB.5 wake-up function enabled.
     * |        |          |0110 = GPB.6 wake-up function enabled.
     * |        |          |0111 = GPB.7 wake-up function enabled.
     * |        |          |1000 = GPB.8 wake-up function enabled.
     * |        |          |1001 = GPB.9 wake-up function enabled.
     * |        |          |1010 = GPB.10 wake-up function enabled.
     * |        |          |1011 = GPB.11 wake-up function enabled.
     * |        |          |1100 = GPB.12 wake-up function enabled.
     * |        |          |1101 = GPB.13 wake-up function enabled.
     * |        |          |1110 = GPB.14 wake-up function enabled.
     * |        |          |1111 = GPB.15 wake-up function enabled.
     * |[8]     |DBEN      |GPB Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding IO
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator.
     * |        |          |0 = Standby power-down wake-up pin De-bounce function disable.
     * |        |          |1 = Standby power-down wake-up pin De-bounce function enable.
     * |        |          |The de-bounce function is valid only for edge triggered.
     * @var CLK_T::PCSWKCTL
     * Offset: 0xA8  GPC Standby Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Standby Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPC group pin wake-up function disabled.
     * |        |          |1 = GPC group pin wake-up function enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPC group pin rising edge wake-up function disabled.
     * |        |          |1 = GPC group pin rising edge wake-up function enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPC group pin falling edge wake-up function disabled.
     * |        |          |1 = GPC group pin falling edge wake-up function enabled.
     * |[7:4]   |WKPSEL    |GPC Standby Power-down Wake-up Pin Select
     * |        |          |0000 = GPC.0 wake-up function enabled.
     * |        |          |0001 = GPC.1 wake-up function enabled.
     * |        |          |0010 = GPC.2 wake-up function enabled.
     * |        |          |0011 = GPC.3 wake-up function enabled.
     * |        |          |0100 = GPC.4 wake-up function enabled.
     * |        |          |0101 = GPC.5 wake-up function enabled.
     * |        |          |0110 = GPC.6 wake-up function enabled.
     * |        |          |0111 = GPC.7 wake-up function enabled.
     * |        |          |1000 = GPC.8 wake-up function enabled.
     * |        |          |1001 = GPC.9 wake-up function enabled.
     * |        |          |1010 = GPC.10 wake-up function enabled.
     * |        |          |1011 = GPC.11 wake-up function enabled.
     * |        |          |1100 = GPC.12 wake-up function enabled.
     * |        |          |1101 = GPC.13 wake-up function enabled.
     * |        |          |1110 = GPC.14 wake-up function enabled.
     * |        |          |1111 = GPC.15 wake-up function enabled.
     * |[8]     |DBEN      |GPC Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding IO
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator.
     * |        |          |0 = Standby power-down wake-up pin De-bounce function disable.
     * |        |          |1 = Standby power-down wake-up pin De-bounce function enable.
     * |        |          |The de-bounce function is valid only for edge triggered.
     * @var CLK_T::PDSWKCTL
     * Offset: 0xAC  GPD Standby Power-down Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Standby Power-down Pin Wake-up Enable Bit
     * |        |          |0 = GPD group pin wake-up function disabled.
     * |        |          |1 = GPD group pin wake-up function enabled.
     * |[1]     |PRWKEN    |Pin Rising Edge Wake-up Enable Bit
     * |        |          |0 = GPD group pin rising edge wake-up function disabled.
     * |        |          |1 = GPD group pin rising edge wake-up function enabled.
     * |[2]     |PFWKEN    |Pin Falling Edge Wake-up Enable Bit
     * |        |          |0 = GPD group pin falling edge wake-up function disabled.
     * |        |          |1 = GPD group pin falling edge wake-up function enabled.
     * |[7:4]   |WKPSEL    |GPD Standby Power-down Wake-up Pin Select
     * |        |          |0000 = GPD.0 wake-up function enabled.
     * |        |          |0001 = GPD.1 wake-up function enabled.
     * |        |          |0010 = GPD.2 wake-up function enabled.
     * |        |          |0011 = GPD.3 wake-up function enabled.
     * |        |          |0100 = GPD.4 wake-up function enabled.
     * |        |          |0101 = GPD.5 wake-up function enabled.
     * |        |          |0110 = GPD.6 wake-up function enabled.
     * |        |          |0111 = GPD.7 wake-up function enabled.
     * |        |          |1000 = GPD.8 wake-up function enabled.
     * |        |          |1001 = GPD.9 wake-up function enabled.
     * |        |          |1010 = GPD.10 wake-up function enabled.
     * |        |          |1011 = GPD.11 wake-up function enabled.
     * |        |          |1100 = GPD.12 wake-up function enabled.
     * |        |          |1101 = GPD.13 wake-up function enabled.
     * |        |          |1110 = GPD.14 wake-up function enabled.
     * |        |          |1111 = GPD.15 wake-up function enabled.
     * |[8]     |DBEN      |GPD Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding IO
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the wake-up
     * |        |          |The de-bounce clock source is the 10 kHz internal low speed RC oscillator.
     * |        |          |0 = Standby power-down wake-up pin De-bounce function disable.
     * |        |          |1 = Standby power-down wake-up pin De-bounce function enable.
     * |        |          |The de-bounce function is valid only for edge triggered.
     * @var CLK_T::IOPDCTL
     * Offset: 0xB0  GPIO Standby Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IOHR      |GPIO Hold Release
     * |        |          |When GPIO enter standby power-down mode, all I/O status are hold to keep normal operating status
     * |        |          |After chip was waked up from standby power-down mode, the I/O are still keep hold status until user set this bit to release I/O hold status.
     * |        |          |This bit is auto cleared by hardware.
     */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power-down Control Register                               */
    __IO uint32_t SYSCLK0;               /*!< [0x0004] AXI and AHB Device Clock Enable Control Register 0               */
    __IO uint32_t SYSCLK1;               /*!< [0x0008] AXI and AHB Device Clock Enable Control Register 1               */
    __IO uint32_t APBCLK0;               /*!< [0x000c] APB Devices Clock Enable Control Register 0                      */
    __IO uint32_t APBCLK1;               /*!< [0x0010] APB Devices Clock Enable Control Register 1                      */
    __IO uint32_t APBCLK2;               /*!< [0x0014] APB Devices Clock Enable Control Register 2                      */
    __IO uint32_t CLKSEL0;               /*!< [0x0018] Clock Source Select Control Register 0                           */
    __IO uint32_t CLKSEL1;               /*!< [0x001c] Clock Source Select Control Register 1                           */
    __IO uint32_t CLKSEL2;               /*!< [0x0020] Clock Source Select Control Register 2                           */
    __IO uint32_t CLKSEL3;               /*!< [0x0024] Clock Source Select Control Register 3                           */
    __IO uint32_t CLKSEL4;               /*!< [0x0028] Clock Source Select Control Register 4                           */
    __IO uint32_t CLKDIV0;               /*!< [0x002c] Clock Divider Number Register 0                                  */
    __IO uint32_t CLKDIV1;               /*!< [0x0030] Clock Divider Number Register 1                                  */
    __IO uint32_t CLKDIV2;               /*!< [0x0034] Clock Divider Number Register 2                                  */
    __IO uint32_t CLKDIV3;               /*!< [0x0038] Clock Divider Number Register 3                                  */
    __IO uint32_t CLKDIV4;               /*!< [0x003c] Clock Divider Number Register 4                                  */
    __IO uint32_t CLKOCTL;               /*!< [0x0040] Clock Output Control Register                                    */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE1[3];
    /** @endcond */
    __I  uint32_t STATUS;                /*!< [0x0050] Clock Status Monitor Register                                    */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE2[3];
    /** @endcond */
    __IO uint32_t PLL0CTL0;              /*!< [0x0060] CA-PLL Control Register 0                                        */
    __IO uint32_t PLL0CTL1;              /*!< [0x0064] CA-PLL Control Register 1                                        */
    __IO uint32_t PLL0CTL2;              /*!< [0x0068] CA-PLL Control Register 2                                        */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE3;
    /** @endcond */
    __IO uint32_t PLL1CTL0;              /*!< [0x0070] SYS-PLL Control Register 0                                       */
    __IO uint32_t PLL1CTL1;              /*!< [0x0074] SYS-PLL Control Register 1                                       */
    __IO uint32_t PLL1CTL2;              /*!< [0x0078] SYS-PLL Control Register 2                                       */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE4;
    /** @endcond */
    __IO uint32_t PLL2CTL0;              /*!< [0x0080] DDR-PLL Control Register 0                                       */
    __IO uint32_t PLL2CTL1;              /*!< [0x0084] DDR-PLL Control Register 1                                       */
    __IO uint32_t PLL2CTL2;              /*!< [0x0088] DDR-PLL Control Register 2                                       */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE5;
    /** @endcond */
    __IO uint32_t PLL3CTL0;              /*!< [0x0090] APLL Control Register 0              	                        */
    __IO uint32_t PLL3CTL1;              /*!< [0x0094] APLL Control Register 1              	                        */
    __IO uint32_t PLL3CTL2;              /*!< [0x0098] APLL Control Register 2              	                        */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE6;
    /** @endcond */
    __IO uint32_t PLL4CTL0;              /*!< [0x00A0] EPLL Control Register 0                                          */
    __IO uint32_t PLL4CTL1;              /*!< [0x00A4] EPLL Control Register 1                                          */
    __IO uint32_t PLL4CTL2;              /*!< [0x00A8] EPLL Control Register 2                                          */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE7;
    /** @endcond */
    __IO uint32_t PLL5CTL0;              /*!< [0x00B0] VPLL Control Register 0                                          */
    __IO uint32_t PLL5CTL1;              /*!< [0x00B4] VPLL Control Register 1                                          */
    __IO uint32_t PLL5CTL2;              /*!< [0x00B8] VPLL Control Register 2                                          */
    /** @cond HIDDEN_SYMBOLS */
    __I  uint32_t RESERVE8;
    /** @endcond */
    __IO uint32_t CLKDCTL;               /*!< [0x00C0] Clock Fail Detector Control Register                             */
    __IO uint32_t CLKDSTS;               /*!< [0x00C4] Clock Fail Detector Status Register                              */
    __IO uint32_t CDUPB;                 /*!< [0x00C8] Clock Frequency Range Detector Upper Boundary Register           */
    __IO uint32_t CDLOWB;                /*!< [0x00CC] Clock Frequency Range Detector Lower Boundary Register           */
    __IO uint32_t HXTFSEL;               /*!< [0x00D0] HXT Filter Select Control Register                               */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_HXTEN_Pos             (0)                                               /*!< CLK_T::PWRCTL: HXTEN Position          */
#define CLK_PWRCTL_HXTEN_Msk             (0x1ul << CLK_PWRCTL_HXTEN_Pos)                   /*!< CLK_T::PWRCTL: HXTEN Mask              */

#define CLK_PWRCTL_LXTEN_Pos             (1)                                               /*!< CLK_T::PWRCTL: LXTEN Position          */
#define CLK_PWRCTL_LXTEN_Msk             (0x1ul << CLK_PWRCTL_LXTEN_Pos)                   /*!< CLK_T::PWRCTL: LXTEN Mask              */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_PDWKDLY_Pos           (4)                                               /*!< CLK_T::PWRCTL: PDWKDLY Position        */
#define CLK_PWRCTL_PDWKDLY_Msk           (0x1ul << CLK_PWRCTL_PDWKDLY_Pos)                 /*!< CLK_T::PWRCTL: PDWKDLY Mask            */

#define CLK_PWRCTL_PDWKIEN_Pos           (5)                                               /*!< CLK_T::PWRCTL: PDWKIEN Position        */
#define CLK_PWRCTL_PDWKIEN_Msk           (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)                 /*!< CLK_T::PWRCTL: PDWKIEN Mask            */

#define CLK_PWRCTL_PDWKIF_Pos            (6)                                               /*!< CLK_T::PWRCTL: PDWKIF Position         */
#define CLK_PWRCTL_PDWKIF_Msk            (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                  /*!< CLK_T::PWRCTL: PDWKIF Mask             */

#define CLK_PWRCTL_PDEN_Pos              (7)                                               /*!< CLK_T::PWRCTL: PDEN Position           */
#define CLK_PWRCTL_PDEN_Msk              (0x1ul << CLK_PWRCTL_PDEN_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask               */

#define CLK_PWRCTL_HXTDS_Pos             (10)                                              /*!< CLK_T::PWRCTL: HXTDS Position          */
#define CLK_PWRCTL_HXTDS_Msk             (0x3ul << CLK_PWRCTL_HXTDS_Pos)                   /*!< CLK_T::PWRCTL: HXTDS Mask              */

#define CLK_PWRCTL_HIRCSTBS_Pos          (16)                                              /*!< CLK_T::PWRCTL: HIRCSTBS Position       */
#define CLK_PWRCTL_HIRCSTBS_Msk          (0x3ul << CLK_PWRCTL_HIRCSTBS_Pos)                /*!< CLK_T::PWRCTL: HIRCSTBS Mask           */

#define CLK_SYSCLK0_CA35CKEN_Pos         (0)                                               /*!< CLK_T::SYSCLK0: CA35CKEN Position      */
#define CLK_SYSCLK0_CA35CKEN_Msk         (0x1ul << CLK_SYSCLK0_CA35CKEN_Pos)               /*!< CLK_T::SYSCLK0: CA35CKEN Mask          */

#define CLK_SYSCLK0_CM4CKEN_Pos          (1)                                               /*!< CLK_T::SYSCLK0: CM4CKEN Position       */
#define CLK_SYSCLK0_CM4CKEN_Msk          (0x1ul << CLK_SYSCLK0_CM4CKEN_Pos)                /*!< CLK_T::SYSCLK0: CM4CKEN Mask           */

#define CLK_SYSCLK0_TAHBCKEN_Pos         (2)                                               /*!< CLK_T::SYSCLK0: TAHBCKEN Position      */
#define CLK_SYSCLK0_TAHBCKEN_Msk         (0x1ul << CLK_SYSCLK0_TAHBCKEN_Pos)               /*!< CLK_T::SYSCLK0: TAHBCKEN Mask          */

#define CLK_SYSCLK0_LVRDBEN_Pos          (3)                                               /*!< CLK_T::SYSCLK0: LVRDBEN Position       */
#define CLK_SYSCLK0_LVRDBEN_Msk          (0x1ul << CLK_SYSCLK1_LVRDBEN_Pos)                /*!< CLK_T::SYSCLK0: LVRDBEN Mask           */

#define CLK_SYSCLK0_DDR0CKEN_Pos         (4)                                               /*!< CLK_T::SYSCLK0: DDR0CKEN Position      */
#define CLK_SYSCLK0_DDR0CKEN_Msk         (0x1ul << CLK_SYSCLK0_DDR0CKEN_Pos)               /*!< CLK_T::SYSCLK0: DDR0CKEN Mask          */

#define CLK_SYSCLK0_DDR6CKEN_Pos         (5)                                               /*!< CLK_T::SYSCLK0: DDR6CKEN Position      */
#define CLK_SYSCLK0_DDR6CKEN_Msk         (0x1ul << CLK_SYSCLK0_DDR6CKEN_Pos)               /*!< CLK_T::SYSCLK0: DDR6CKEN Mask          */

#define CLK_SYSCLK0_CAN0CKEN_Pos         (8)                                               /*!< CLK_T::SYSCLK0: CAN0CKEN Position      */
#define CLK_SYSCLK0_CAN0CKEN_Msk         (0x1ul << CLK_SYSCLK0_CAN0CKEN_Pos)               /*!< CLK_T::SYSCLK0: CAN0CKEN Mask          */

#define CLK_SYSCLK0_CAN1CKEN_Pos         (9)                                               /*!< CLK_T::SYSCLK0: CAN1CKEN Position      */
#define CLK_SYSCLK0_CAN1CKEN_Msk         (0x1ul << CLK_SYSCLK0_CAN1CKEN_Pos)               /*!< CLK_T::SYSCLK0: CAN1CKEN Mask          */

#define CLK_SYSCLK0_CAN2CKEN_Pos         (10)                                              /*!< CLK_T::SYSCLK0: CAN2CKEN Position      */
#define CLK_SYSCLK0_CAN2CKEN_Msk         (0x1ul << CLK_SYSCLK0_CAN2CKEN_Pos)               /*!< CLK_T::SYSCLK0: CAN2CKEN Mask          */

#define CLK_SYSCLK0_CAN3CKEN_Pos         (11)                                              /*!< CLK_T::SYSCLK0: CAN3CKEN Position      */
#define CLK_SYSCLK0_CAN3CKEN_Msk         (0x1ul << CLK_SYSCLK0_CAN3CKEN_Pos)               /*!< CLK_T::SYSCLK0: CAN3CKEN Mask          */

#define CLK_SYSCLK0_SDH0HCKEN_Pos        (16)                                              /*!< CLK_T::SYSCLK0: SDH0HCKEN Position     */
#define CLK_SYSCLK0_SDH0HCKEN_Msk        (0x1ul << CLK_SYSCLK0_SDH0HCKEN_Pos)              /*!< CLK_T::SYSCLK0: SDH0HCKEN Mask         */

#define CLK_SYSCLK0_SDH1HCKEN_Pos        (17)                                              /*!< CLK_T::SYSCLK0: SDH1HCKEN Position     */
#define CLK_SYSCLK0_SDH1HCKEN_Msk        (0x1ul << CLK_SYSCLK0_SDH1HCKEN_Pos)              /*!< CLK_T::SYSCLK0: SDH1HCKEN Mask         */

#define CLK_SYSCLK0_NANDCKEN_Pos         (18)                                              /*!< CLK_T::SYSCLK0: NANDCKEN Position      */
#define CLK_SYSCLK0_NANDCKEN_Msk         (0x1ul << CLK_SYSCLK0_NANDCKEN_Pos)               /*!< CLK_T::SYSCLK0: NANDCKEN Mask          */

#define CLK_SYSCLK0_USBDCKEN_Pos         (19)                                              /*!< CLK_T::SYSCLK0: USBDCKEN Position      */
#define CLK_SYSCLK0_USBDCKEN_Msk         (0x1ul << CLK_SYSCLK0_USBDCKEN_Pos)               /*!< CLK_T::SYSCLK0: USBDCKEN Mask          */

#define CLK_SYSCLK0_USBHCKEN_Pos         (20)                                              /*!< CLK_T::SYSCLK0: USBHCKEN Position      */
#define CLK_SYSCLK0_USBHCKEN_Msk         (0x1ul << CLK_SYSCLK0_USBHCKEN_Pos)               /*!< CLK_T::SYSCLK0: USBHCKEN Mask          */

#define CLK_SYSCLK0_HUSBH0EN_Pos         (21)                                              /*!< CLK_T::SYSCLK0: HUSBH0EN Position      */
#define CLK_SYSCLK0_HUSBH0EN_Msk         (0x1ul << CLK_SYSCLK0_HUSBH0EN_Pos)               /*!< CLK_T::SYSCLK0: HUSBH0EN Mask          */

#define CLK_SYSCLK0_HUSBH1EN_Pos         (22)                                              /*!< CLK_T::SYSCLK0: HUSBH1EN Position      */
#define CLK_SYSCLK0_HUSBH1EN_Msk         (0x1ul << CLK_SYSCLK0_HUSBH1EN_Pos)               /*!< CLK_T::SYSCLK0: HUSBH1EN Mask          */

#define CLK_SYSCLK0_GFXCKEN_Pos          (24)                                              /*!< CLK_T::SYSCLK0: GFXCKEN Position       */
#define CLK_SYSCLK0_GFXCKEN_Msk          (0x1ul << CLK_SYSCLK0_GFXCKEN_Pos)                /*!< CLK_T::SYSCLK0: GFXCKEN Mask           */

#define CLK_SYSCLK0_VC8KCKEN_Pos         (25)                                              /*!< CLK_T::SYSCLK0: VC8KCKEN Position      */
#define CLK_SYSCLK0_VC8KCKEN_Msk         (0x1ul << CLK_SYSCLK0_VC8KCKEN_Pos)               /*!< CLK_T::SYSCLK0: VC8KCKEN Mask          */

#define CLK_SYSCLK0_DCUCKEN_Pos          (26)                                              /*!< CLK_T::SYSCLK0: DCUCKEN Position       */
#define CLK_SYSCLK0_DCUCKEN_Msk          (0x1ul << CLK_SYSCLK0_DCUCKEN_Pos)                /*!< CLK_T::SYSCLK0: DCUCKEN Mask           */

#define CLK_SYSCLK0_GMAC0CKEN_Pos        (27)                                              /*!< CLK_T::SYSCLK0: GMAC0CKEN Position     */
#define CLK_SYSCLK0_GMAC0CKEN_Msk        (0x1ul << CLK_SYSCLK0_GMAC0CKEN_Pos)              /*!< CLK_T::SYSCLK0: GMAC0CKEN Mask         */

#define CLK_SYSCLK0_GMAC1CKEN_Pos        (28)                                              /*!< CLK_T::SYSCLK0: GMAC1CKEN Position     */
#define CLK_SYSCLK0_GMAC1CKEN_Msk        (0x1ul << CLK_SYSCLK0_GMAC1CKEN_Pos)              /*!< CLK_T::SYSCLK0: GMAC1CKEN Mask         */

#define CLK_SYSCLK0_CAP0HCKEN_Pos        (29)                                              /*!< CLK_T::SYSCLK0: CAP0HCKEN Position     */
#define CLK_SYSCLK0_CAP0HCKEN_Msk        (0x1ul << CLK_SYSCLK0_CAP0HCKEN_Pos)              /*!< CLK_T::SYSCLK0: CAP0HCKEN Mask         */

#define CLK_SYSCLK0_CAP1HCKEN_Pos        (30)                                              /*!< CLK_T::SYSCLK0: CAP1HCKEN Position     */
#define CLK_SYSCLK0_CAP1HCKEN_Msk        (0x1ul << CLK_SYSCLK0_CAP1HCKEN_Pos)              /*!< CLK_T::SYSCLK0: CAP1HCKEN Mask         */

#define CLK_SYSCLK1_PDMA0CKEN_Pos        (0)                                               /*!< CLK_T::SYSCLK1: PDMA0CKEN Position     */
#define CLK_SYSCLK1_PDMA0CKEN_Msk        (0x1ul << CLK_SYSCLK1_PDMA0CKEN_Pos)              /*!< CLK_T::SYSCLK1: PDMA0CKEN Mask         */

#define CLK_SYSCLK1_PDMA1CKEN_Pos        (1)                                               /*!< CLK_T::SYSCLK1: PDMA1CKEN Position     */
#define CLK_SYSCLK1_PDMA1CKEN_Msk        (0x1ul << CLK_SYSCLK1_PDMA1CKEN_Pos)              /*!< CLK_T::SYSCLK1: PDMA1CKEN Mask         */

#define CLK_SYSCLK1_PDMA2CKEN_Pos        (2)                                               /*!< CLK_T::SYSCLK1: PDMA2CKEN Position     */
#define CLK_SYSCLK1_PDMA2CKEN_Msk        (0x1ul << CLK_SYSCLK1_PDMA2CKEN_Pos)              /*!< CLK_T::SYSCLK1: PDMA2CKEN Mask         */

#define CLK_SYSCLK1_PDMA3CKEN_Pos        (3)                                               /*!< CLK_T::SYSCLK1: PDMA3CKEN Position     */
#define CLK_SYSCLK1_PDMA3CKEN_Msk        (0x1ul << CLK_SYSCLK1_PDMA3CKEN_Pos)              /*!< CLK_T::SYSCLK1: PDMA3CKEN Mask         */

#define CLK_SYSCLK1_WH0CKEN_Pos          (4)                                               /*!< CLK_T::SYSCLK1: WH0CKEN Position       */
#define CLK_SYSCLK1_WH0CKEN_Msk          (0x1ul << CLK_SYSCLK1_WH0CKEN_Pos)                /*!< CLK_T::SYSCLK1: WH0CKEN Mask           */

#define CLK_SYSCLK1_WH1CKEN_Pos          (5)                                               /*!< CLK_T::SYSCLK1: WH1CKEN Position       */
#define CLK_SYSCLK1_WH1CKEN_Msk          (0x1ul << CLK_SYSCLK1_WH1CKEN_Pos)                /*!< CLK_T::SYSCLK1: WH1CKEN Mask           */

#define CLK_SYSCLK1_HWSCKEN_Pos          (6)                                               /*!< CLK_T::SYSCLK1: HWSCKEN Position       */
#define CLK_SYSCLK1_HWSCKEN_Msk          (0x1ul << CLK_SYSCLK1_HWSCKEN_Pos)                /*!< CLK_T::SYSCLK1: HWSCKEN Mask           */

#define CLK_SYSCLK1_EBICKEN_Pos          (7)                                               /*!< CLK_T::SYSCLK1: EBICKEN Position       */
#define CLK_SYSCLK1_EBICKEN_Msk          (0x1ul << CLK_SYSCLK1_EBICKEN_Pos)                /*!< CLK_T::SYSCLK1: EBICKEN Mask           */

#define CLK_SYSCLK1_SRAM0CKEN_Pos        (8)                                               /*!< CLK_T::SYSCLK1: SRAM0CKEN Position     */
#define CLK_SYSCLK1_SRAM0CKEN_Msk        (0x1ul << CLK_SYSCLK1_SRAM0CKEN_Pos)              /*!< CLK_T::SYSCLK1: SRAM0CKEN Mask         */

#define CLK_SYSCLK1_SRAM1CKEN_Pos        (9)                                               /*!< CLK_T::SYSCLK1: SRAM1CKEN Position     */
#define CLK_SYSCLK1_SRAM1CKEN_Msk        (0x1ul << CLK_SYSCLK1_SRAM1CKEN_Pos)              /*!< CLK_T::SYSCLK1: SRAM1CKEN Mask         */

#define CLK_SYSCLK1_ROMCKEN_Pos          (10)                                              /*!< CLK_T::SYSCLK1: ROMCKEN Position       */
#define CLK_SYSCLK1_ROMCKEN_Msk          (0x1ul << CLK_SYSCLK1_ROMCKEN_Pos)                /*!< CLK_T::SYSCLK1: ROMCKEN Mask           */

#define CLK_SYSCLK1_TRACKEN_Pos          (11)                                              /*!< CLK_T::SYSCLK1: TRACKEN Position       */
#define CLK_SYSCLK1_TRACKEN_Msk          (0x1ul << CLK_SYSCLK1_TRACKEN_Pos)                /*!< CLK_T::SYSCLK1: TRACKEN Mask           */

#define CLK_SYSCLK1_DBGCKEN_Pos          (12)                                              /*!< CLK_T::SYSCLK1: DBGCKEN Position       */
#define CLK_SYSCLK1_DBGCKEN_Msk          (0x1ul << CLK_SYSCLK1_DBGCKEN_Pos)                /*!< CLK_T::SYSCLK1: DBGCKEN Mask           */

#define CLK_SYSCLK1_CLKOCKEN_Pos         (13)                                              /*!< CLK_T::SYSCLK1: CLKOCKEN Position      */
#define CLK_SYSCLK1_CLKOCKEN_Msk         (0x1ul << CLK_SYSCLK1_CLKOCKEN_Pos)               /*!< CLK_T::SYSCLK1: CLKOCKEN Mask          */

#define CLK_SYSCLK1_GTMRCKEN_Pos         (14)                                              /*!< CLK_T::SYSCLK1: GTMRCKEN Position      */
#define CLK_SYSCLK1_GTMRCKEN_Msk         (0x1ul << CLK_SYSCLK1_GTMRCKEN_Pos)               /*!< CLK_T::SYSCLK1: GTMRCKEN Mask          */

#define CLK_SYSCLK1_GPACKEN_Pos          (16)                                              /*!< CLK_T::SYSCLK1: GPACKEN Position       */
#define CLK_SYSCLK1_GPACKEN_Msk          (0x1ul << CLK_SYSCLK1_GPACKEN_Pos)                /*!< CLK_T::SYSCLK1: GPACKEN Mask           */

#define CLK_SYSCLK1_GPBCKEN_Pos          (17)                                              /*!< CLK_T::SYSCLK1: GPBCKEN Position       */
#define CLK_SYSCLK1_GPBCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPBCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPBCKEN Mask           */

#define CLK_SYSCLK1_GPCCKEN_Pos          (18)                                              /*!< CLK_T::SYSCLK1: GPCCKEN Position       */
#define CLK_SYSCLK1_GPCCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPCCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPCCKEN Mask           */

#define CLK_SYSCLK1_GPDCKEN_Pos          (19)                                              /*!< CLK_T::SYSCLK1: GPDCKEN Position       */
#define CLK_SYSCLK1_GPDCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPDCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPDCKEN Mask           */

#define CLK_SYSCLK1_GPECKEN_Pos          (20)                                              /*!< CLK_T::SYSCLK1: GPECKEN Position       */
#define CLK_SYSCLK1_GPECKEN_Msk          (0x1ul << CLK_SYSCLK1_GPECKEN_Pos)                /*!< CLK_T::SYSCLK1: GPECKEN Mask           */

#define CLK_SYSCLK1_GPFCKEN_Pos          (21)                                              /*!< CLK_T::SYSCLK1: GPFCKEN Position       */
#define CLK_SYSCLK1_GPFCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPFCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPFCKEN Mask           */

#define CLK_SYSCLK1_GPGCKEN_Pos          (22)                                              /*!< CLK_T::SYSCLK1: GPGCKEN Position       */
#define CLK_SYSCLK1_GPGCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPGCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPGCKEN Mask           */

#define CLK_SYSCLK1_GPHCKEN_Pos          (23)                                              /*!< CLK_T::SYSCLK1: GPHCKEN Position       */
#define CLK_SYSCLK1_GPHCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPHCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPHCKEN Mask           */

#define CLK_SYSCLK1_GPICKEN_Pos          (24)                                              /*!< CLK_T::SYSCLK1: GPICKEN Position       */
#define CLK_SYSCLK1_GPICKEN_Msk          (0x1ul << CLK_SYSCLK1_GPICKEN_Pos)                /*!< CLK_T::SYSCLK1: GPICKEN Mask           */

#define CLK_SYSCLK1_GPJCKEN_Pos          (25)                                              /*!< CLK_T::SYSCLK1: GPJCKEN Position       */
#define CLK_SYSCLK1_GPJCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPJCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPJCKEN Mask           */

#define CLK_SYSCLK1_GPKCKEN_Pos          (26)                                              /*!< CLK_T::SYSCLK1: GPKCKEN Position       */
#define CLK_SYSCLK1_GPKCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPKCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPKCKEN Mask           */

#define CLK_SYSCLK1_GPLCKEN_Pos          (27)                                              /*!< CLK_T::SYSCLK1: GPLCKEN Position       */
#define CLK_SYSCLK1_GPLCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPLCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPLCKEN Mask           */

#define CLK_SYSCLK1_GPMCKEN_Pos          (28)                                              /*!< CLK_T::SYSCLK1: GPMCKEN Position       */
#define CLK_SYSCLK1_GPMCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPMCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPMCKEN Mask           */

#define CLK_SYSCLK1_GPNCKEN_Pos          (29)                                              /*!< CLK_T::SYSCLK1: GPNCKEN Position       */
#define CLK_SYSCLK1_GPNCKEN_Msk          (0x1ul << CLK_SYSCLK1_GPNCKEN_Pos)                /*!< CLK_T::SYSCLK1: GPNCKEN Mask           */

#define CLK_APBCLK0_TMR0CKEN_Pos         (0)                                               /*!< CLK_T::APBCLK0: TMR0CKEN Position      */
#define CLK_APBCLK0_TMR0CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR0CKEN Mask          */

#define CLK_APBCLK0_TMR1CKEN_Pos         (1)                                               /*!< CLK_T::APBCLK0: TMR1CKEN Position      */
#define CLK_APBCLK0_TMR1CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR1CKEN Mask          */

#define CLK_APBCLK0_TMR2CKEN_Pos         (2)                                               /*!< CLK_T::APBCLK0: TMR2CKEN Position      */
#define CLK_APBCLK0_TMR2CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR2CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR2CKEN Mask          */

#define CLK_APBCLK0_TMR3CKEN_Pos         (3)                                               /*!< CLK_T::APBCLK0: TMR3CKEN Position      */
#define CLK_APBCLK0_TMR3CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR3CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR3CKEN Mask          */

#define CLK_APBCLK0_TMR4CKEN_Pos         (4)                                               /*!< CLK_T::APBCLK0: TMR4CKEN Position      */
#define CLK_APBCLK0_TMR4CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR4CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR4CKEN Mask          */

#define CLK_APBCLK0_TMR5CKEN_Pos         (5)                                               /*!< CLK_T::APBCLK0: TMR5CKEN Position      */
#define CLK_APBCLK0_TMR5CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR5CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR5CKEN Mask          */

#define CLK_APBCLK0_TMR6CKEN_Pos         (6)                                               /*!< CLK_T::APBCLK0: TMR6CKEN Position      */
#define CLK_APBCLK0_TMR6CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR6CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR6CKEN Mask          */

#define CLK_APBCLK0_TMR7CKEN_Pos         (7)                                               /*!< CLK_T::APBCLK0: TMR7CKEN Position      */
#define CLK_APBCLK0_TMR7CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR7CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR7CKEN Mask          */

#define CLK_APBCLK0_TMR8CKEN_Pos         (8)                                               /*!< CLK_T::APBCLK0: TMR8CKEN Position      */
#define CLK_APBCLK0_TMR8CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR8CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR8CKEN Mask          */

#define CLK_APBCLK0_TMR9CKEN_Pos         (9)                                               /*!< CLK_T::APBCLK0: TMR9CKEN Position      */
#define CLK_APBCLK0_TMR9CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR9CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR9CKEN Mask          */

#define CLK_APBCLK0_TMR10CKEN_Pos        (10)                                              /*!< CLK_T::APBCLK0: TMR10CKEN Position     */
#define CLK_APBCLK0_TMR10CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR10CKEN_Pos)              /*!< CLK_T::APBCLK0: TMR10CKEN Mask         */

#define CLK_APBCLK0_TMR11CKEN_Pos        (11)                                              /*!< CLK_T::APBCLK0: TMR11CKEN Position     */
#define CLK_APBCLK0_TMR11CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR11CKEN_Pos)              /*!< CLK_T::APBCLK0: TMR11CKEN Mask         */

#define CLK_APBCLK0_UART0CKEN_Pos        (12)                                              /*!< CLK_T::APBCLK0: UART0CKEN Position     */
#define CLK_APBCLK0_UART0CKEN_Msk        (0x1ul << CLK_APBCLK0_UART0CKEN_Pos)              /*!< CLK_T::APBCLK0: UART0CKEN Mask         */

#define CLK_APBCLK0_UART1CKEN_Pos        (13)                                              /*!< CLK_T::APBCLK0: UART1CKEN Position     */
#define CLK_APBCLK0_UART1CKEN_Msk        (0x1ul << CLK_APBCLK0_UART1CKEN_Pos)              /*!< CLK_T::APBCLK0: UART1CKEN Mask         */

#define CLK_APBCLK0_UART2CKEN_Pos        (14)                                              /*!< CLK_T::APBCLK0: UART2CKEN Position     */
#define CLK_APBCLK0_UART2CKEN_Msk        (0x1ul << CLK_APBCLK0_UART2CKEN_Pos)              /*!< CLK_T::APBCLK0: UART2CKEN Mask         */

#define CLK_APBCLK0_UART3CKEN_Pos        (15)                                              /*!< CLK_T::APBCLK0: UART3CKEN Position     */
#define CLK_APBCLK0_UART3CKEN_Msk        (0x1ul << CLK_APBCLK0_UART3CKEN_Pos)              /*!< CLK_T::APBCLK0: UART3CKEN Mask         */

#define CLK_APBCLK0_UART4CKEN_Pos        (16)                                              /*!< CLK_T::APBCLK0: UART4CKEN Position     */
#define CLK_APBCLK0_UART4CKEN_Msk        (0x1ul << CLK_APBCLK0_UART4CKEN_Pos)              /*!< CLK_T::APBCLK0: UART4CKEN Mask         */

#define CLK_APBCLK0_UART5CKEN_Pos        (17)                                              /*!< CLK_T::APBCLK0: UART5CKEN Position     */
#define CLK_APBCLK0_UART5CKEN_Msk        (0x1ul << CLK_APBCLK0_UART5CKEN_Pos)              /*!< CLK_T::APBCLK0: UART5CKEN Mask         */

#define CLK_APBCLK0_UART6CKEN_Pos        (18)                                              /*!< CLK_T::APBCLK0: UART6CKEN Position     */
#define CLK_APBCLK0_UART6CKEN_Msk        (0x1ul << CLK_APBCLK0_UART6CKEN_Pos)              /*!< CLK_T::APBCLK0: UART6CKEN Mask         */

#define CLK_APBCLK0_UART7CKEN_Pos        (19)                                              /*!< CLK_T::APBCLK0: UART7CKEN Position     */
#define CLK_APBCLK0_UART7CKEN_Msk        (0x1ul << CLK_APBCLK0_UART7CKEN_Pos)              /*!< CLK_T::APBCLK0: UART7CKEN Mask         */

#define CLK_APBCLK0_UART8CKEN_Pos        (20)                                              /*!< CLK_T::APBCLK0: UART8CKEN Position     */
#define CLK_APBCLK0_UART8CKEN_Msk        (0x1ul << CLK_APBCLK0_UART8CKEN_Pos)              /*!< CLK_T::APBCLK0: UART8CKEN Mask         */

#define CLK_APBCLK0_UART9CKEN_Pos        (21)                                              /*!< CLK_T::APBCLK0: UART9CKEN Position     */
#define CLK_APBCLK0_UART9CKEN_Msk        (0x1ul << CLK_APBCLK0_UART9CKEN_Pos)              /*!< CLK_T::APBCLK0: UART9CKEN Mask         */

#define CLK_APBCLK0_UART10CKEN_Pos       (22)                                              /*!< CLK_T::APBCLK0: UART10CKEN Position    */
#define CLK_APBCLK0_UART10CKEN_Msk       (0x1ul << CLK_APBCLK0_UART10CKEN_Pos)             /*!< CLK_T::APBCLK0: UART10CKEN Mask        */

#define CLK_APBCLK0_UART11CKEN_Pos       (23)                                              /*!< CLK_T::APBCLK0: UART11CKEN Position    */
#define CLK_APBCLK0_UART11CKEN_Msk       (0x1ul << CLK_APBCLK0_UART11CKEN_Pos)             /*!< CLK_T::APBCLK0: UART11CKEN Mask        */

#define CLK_APBCLK0_UART12CKEN_Pos       (24)                                              /*!< CLK_T::APBCLK0: UART12CKEN Position    */
#define CLK_APBCLK0_UART12CKEN_Msk       (0x1ul << CLK_APBCLK0_UART12CKEN_Pos)             /*!< CLK_T::APBCLK0: UART12CKEN Mask        */

#define CLK_APBCLK0_UART13CKEN_Pos       (25)                                              /*!< CLK_T::APBCLK0: UART13CKEN Position    */
#define CLK_APBCLK0_UART13CKEN_Msk       (0x1ul << CLK_APBCLK0_UART13CKEN_Pos)             /*!< CLK_T::APBCLK0: UART13CKEN Mask        */

#define CLK_APBCLK0_UART14CKEN_Pos       (26)                                              /*!< CLK_T::APBCLK0: UART14CKEN Position    */
#define CLK_APBCLK0_UART14CKEN_Msk       (0x1ul << CLK_APBCLK0_UART14CKEN_Pos)             /*!< CLK_T::APBCLK0: UART14CKEN Mask        */

#define CLK_APBCLK0_UART15CKEN_Pos       (27)                                              /*!< CLK_T::APBCLK0: UART15CKEN Position    */
#define CLK_APBCLK0_UART15CKEN_Msk       (0x1ul << CLK_APBCLK0_UART15CKEN_Pos)             /*!< CLK_T::APBCLK0: UART15CKEN Mask        */

#define CLK_APBCLK0_UART16CKEN_Pos       (28)                                              /*!< CLK_T::APBCLK0: UART16CKEN Position    */
#define CLK_APBCLK0_UART16CKEN_Msk       (0x1ul << CLK_APBCLK0_UART16CKEN_Pos)             /*!< CLK_T::APBCLK0: UART16CKEN Mask        */

#define CLK_APBCLK0_RTCCKEN_Pos          (29)                                              /*!< CLK_T::APBCLK0: RTCCKEN Position       */
#define CLK_APBCLK0_RTCCKEN_Msk          (0x1ul << CLK_APBCLK0_RTCCKEN_Pos)                /*!< CLK_T::APBCLK0: RTCCKEN Mask           */

#define CLK_APBCLK0_DDRPCKEN_Pos         (30)                                              /*!< CLK_T::APBCLK0: DDRPCKEN Position      */
#define CLK_APBCLK0_DDRPCKEN_Msk         (0x1ul << CLK_APBCLK0_DDRPCKEN_Pos)               /*!< CLK_T::APBCLK0: DDRPCKEN Mask          */

#define CLK_APBCLK0_KPICKEN_Pos          (31)                                              /*!< CLK_T::APBCLK0: KPICKEN Position       */
#define CLK_APBCLK0_KPICKEN_Msk          (0x1ul << CLK_APBCLK0_KPICKEN_Pos)                /*!< CLK_T::APBCLK0: KPICKEN Mask           */

#define CLK_APBCLK1_I2C0CKEN_Pos         (0)                                               /*!< CLK_T::APBCLK1: I2C0CKEN Position      */
#define CLK_APBCLK1_I2C0CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C0CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C0CKEN Mask          */

#define CLK_APBCLK1_I2C1CKEN_Pos         (1)                                               /*!< CLK_T::APBCLK1: I2C1CKEN Position      */
#define CLK_APBCLK1_I2C1CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C1CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C1CKEN Mask          */

#define CLK_APBCLK1_I2C2CKEN_Pos         (2)                                               /*!< CLK_T::APBCLK1: I2C2CKEN Position      */
#define CLK_APBCLK1_I2C2CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C2CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C2CKEN Mask          */

#define CLK_APBCLK1_I2C3CKEN_Pos         (3)                                               /*!< CLK_T::APBCLK1: I2C3CKEN Position      */
#define CLK_APBCLK1_I2C3CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C3CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C3CKEN Mask          */

#define CLK_APBCLK1_I2C4CKEN_Pos         (4)                                               /*!< CLK_T::APBCLK1: I2C4CKEN Position      */
#define CLK_APBCLK1_I2C4CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C4CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C4CKEN Mask          */

#define CLK_APBCLK1_I2C5CKEN_Pos         (5)                                               /*!< CLK_T::APBCLK1: I2C5CKEN Position      */
#define CLK_APBCLK1_I2C5CKEN_Msk         (0x1ul << CLK_APBCLK1_I2C5CKEN_Pos)               /*!< CLK_T::APBCLK1: I2C5CKEN Mask          */

#define CLK_APBCLK1_QSPI0CKEN_Pos        (6)                                               /*!< CLK_T::APBCLK1: QSPI0CKEN Position     */
#define CLK_APBCLK1_QSPI0CKEN_Msk        (0x1ul << CLK_APBCLK1_QSPI0CKEN_Pos)              /*!< CLK_T::APBCLK1: QSPI0CKEN Mask         */

#define CLK_APBCLK1_QSPI1CKEN_Pos        (7)                                               /*!< CLK_T::APBCLK1: QSPI1CKEN Position     */
#define CLK_APBCLK1_QSPI1CKEN_Msk        (0x1ul << CLK_APBCLK1_QSPI1CKEN_Pos)              /*!< CLK_T::APBCLK1: QSPI1CKEN Mask         */

#define CLK_APBCLK1_CAN0CKEN_Pos         (8)                                               /*!< CLK_T::APBCLK1: CAN0CKEN Position      */
#define CLK_APBCLK1_CAN0CKEN_Msk         (0x1ul << CLK_APBCLK1_CAN0CKEN_Pos)               /*!< CLK_T::APBCLK1: CAN0CKEN Mask          */

#define CLK_APBCLK1_CAN1CKEN_Pos         (9)                                               /*!< CLK_T::APBCLK1: CAN1CKEN Position      */
#define CLK_APBCLK1_CAN1CKEN_Msk         (0x1ul << CLK_APBCLK1_CAN1CKEN_Pos)               /*!< CLK_T::APBCLK1: CAN1CKEN Mask          */

#define CLK_APBCLK1_CAN2CKEN_Pos         (10)                                              /*!< CLK_T::APBCLK1: CAN2CKEN Position      */
#define CLK_APBCLK1_CAN2CKEN_Msk         (0x1ul << CLK_APBCLK1_CAN2CKEN_Pos)               /*!< CLK_T::APBCLK1: CAN2CKEN Mask          */

#define CLK_APBCLK1_CAN3CKEN_Pos         (11)                                              /*!< CLK_T::APBCLK1: CAN3CKEN Position      */
#define CLK_APBCLK1_CAN3CKEN_Msk         (0x1ul << CLK_APBCLK1_CAN3CKEN_Pos)               /*!< CLK_T::APBCLK1: CAN3CKEN Mask          */

#define CLK_APBCLK1_SMC0CKEN_Pos         (12)                                              /*!< CLK_T::APBCLK1: SMC0CKEN Position      */
#define CLK_APBCLK1_SMC0CKEN_Msk         (0x1ul << CLK_APBCLK1_SMC0CKEN_Pos)               /*!< CLK_T::APBCLK1: SMC0CKEN Mask          */

#define CLK_APBCLK1_SMC1CKEN_Pos         (13)                                              /*!< CLK_T::APBCLK1: SMC1CKEN Position      */
#define CLK_APBCLK1_SMC1CKEN_Msk         (0x1ul << CLK_APBCLK1_SMC1CKEN_Pos)               /*!< CLK_T::APBCLK1: SMC1CKEN Mask          */

#define CLK_APBCLK1_WDT0CKEN_Pos         (16)                                              /*!< CLK_T::APBCLK1: WDT0CKEN Position      */
#define CLK_APBCLK1_WDT0CKEN_Msk         (0x1ul << CLK_APBCLK1_WDT0CKEN_Pos)               /*!< CLK_T::APBCLK1: WDT0CKEN Mask          */

#define CLK_APBCLK1_WDT1CKEN_Pos         (17)                                              /*!< CLK_T::APBCLK1: WDT1CKEN Position      */
#define CLK_APBCLK1_WDT1CKEN_Msk         (0x1ul << CLK_APBCLK1_WDT1CKEN_Pos)               /*!< CLK_T::APBCLK1: WDT1CKEN Mask          */

#define CLK_APBCLK1_WDT2CKEN_Pos         (18)                                              /*!< CLK_T::APBCLK1: WDT2CKEN Position      */
#define CLK_APBCLK1_WDT2CKEN_Msk         (0x1ul << CLK_APBCLK1_WDT2CKEN_Pos)               /*!< CLK_T::APBCLK1: WDT2CKEN Mask          */

#define CLK_APBCLK1_EPWM0CKEN_Pos        (24)                                              /*!< CLK_T::APBCLK1: EPWM0CKEN Position     */
#define CLK_APBCLK1_EPWM0CKEN_Msk        (0x1ul << CLK_APBCLK1_EPWM0CKEN_Pos)              /*!< CLK_T::APBCLK1: EPWM0CKEN Mask         */

#define CLK_APBCLK1_EPWM1CKEN_Pos        (25)                                              /*!< CLK_T::APBCLK1: EPWM1CKEN Position     */
#define CLK_APBCLK1_EPWM1CKEN_Msk        (0x1ul << CLK_APBCLK1_EPWM1CKEN_Pos)              /*!< CLK_T::APBCLK1: EPWM1CKEN Mask         */

#define CLK_APBCLK1_EPWM2CKEN_Pos        (26)                                              /*!< CLK_T::APBCLK1: EPWM2CKEN Position     */
#define CLK_APBCLK1_EPWM2CKEN_Msk        (0x1ul << CLK_APBCLK1_EPWM2CKEN_Pos)              /*!< CLK_T::APBCLK1: EPWM2CKEN Mask         */

#define CLK_APBCLK2_I2S0CKEN_Pos         (0)                                               /*!< CLK_T::APBCLK2: I2S0CKEN Position      */
#define CLK_APBCLK2_I2S0CKEN_Msk         (0x1ul << CLK_APBCLK2_I2S0CKEN_Pos)               /*!< CLK_T::APBCLK2: I2S0CKEN Mask          */

#define CLK_APBCLK2_I2S1CKEN_Pos         (1)                                               /*!< CLK_T::APBCLK2: I2S1CKEN Position      */
#define CLK_APBCLK2_I2S1CKEN_Msk         (0x1ul << CLK_APBCLK2_I2S1CKEN_Pos)               /*!< CLK_T::APBCLK2: I2S1CKEN Mask          */

#define CLK_APBCLK2_SSMCCEN_Pos          (2)                                               /*!< CLK_T::APBCLK2: SSMCCEN Position       */
#define CLK_APBCLK2_SSMCCEN_Msk          (0x1ul << CLK_APBCLK2_SSMCCEN_Pos)                /*!< CLK_T::APBCLK2: SSMCCEN Mask           */

#define CLK_APBCLK2_SSPCCEN_Pos          (3)                                               /*!< CLK_T::APBCLK2: SSPCCEN Position       */
#define CLK_APBCLK2_SSPCCEN_Msk          (0x1ul << CLK_APBCLK2_SSPCCEN_Pos)                /*!< CLK_T::APBCLK2: SSPCCEN Mask           */

#define CLK_APBCLK2_SPI0CKEN_Pos         (4)                                               /*!< CLK_T::APBCLK2: SPI0CKEN Position      */
#define CLK_APBCLK2_SPI0CKEN_Msk         (0x1ul << CLK_APBCLK2_SPI0CKEN_Pos)               /*!< CLK_T::APBCLK2: SPI0CKEN Mask          */

#define CLK_APBCLK2_SPI1CKEN_Pos         (5)                                               /*!< CLK_T::APBCLK2: SPI1CKEN Position      */
#define CLK_APBCLK2_SPI1CKEN_Msk         (0x1ul << CLK_APBCLK2_SPI1CKEN_Pos)               /*!< CLK_T::APBCLK2: SPI1CKEN Mask          */

#define CLK_APBCLK2_SPI2CKEN_Pos         (6)                                               /*!< CLK_T::APBCLK2: SPI0CKEN Position      */
#define CLK_APBCLK2_SPI2CKEN_Msk         (0x1ul << CLK_APBCLK2_SPI2CKEN_Pos)               /*!< CLK_T::APBCLK2: SPI0CKEN Mask          */

#define CLK_APBCLK2_SPI3CKEN_Pos         (7)                                               /*!< CLK_T::APBCLK2: SPI1CKEN Position      */
#define CLK_APBCLK2_SPI3CKEN_Msk         (0x1ul << CLK_APBCLK2_SPI3CKEN_Pos)               /*!< CLK_T::APBCLK2: SPI1CKEN Mask          */

#define CLK_APBCLK2_ECAP0CKEN_Pos        (8)                                               /*!< CLK_T::APBCLK2: ECAP0CKEN Position     */
#define CLK_APBCLK2_ECAP0CKEN_Msk        (0x1ul << CLK_APBCLK2_ECAP0CKEN_Pos)              /*!< CLK_T::APBCLK2: ECAP0CKEN Mask         */

#define CLK_APBCLK2_ECAP1CKEN_Pos        (9)                                               /*!< CLK_T::APBCLK2: ECAP1CKEN Position     */
#define CLK_APBCLK2_ECAP1CKEN_Msk        (0x1ul << CLK_APBCLK2_ECAP1CKEN_Pos)              /*!< CLK_T::APBCLK2: ECAP1CKEN Mask         */

#define CLK_APBCLK2_ECAP2CKEN_Pos        (10)                                              /*!< CLK_T::APBCLK2: ECAP2CKEN Position     */
#define CLK_APBCLK2_ECAP2CKEN_Msk        (0x1ul << CLK_APBCLK2_ECAP2CKEN_Pos)              /*!< CLK_T::APBCLK2: ECAP2CKEN Mask         */

#define CLK_APBCLK2_QEI0CKEN_Pos         (12)                                              /*!< CLK_T::APBCLK2: QEI0CKEN Position      */
#define CLK_APBCLK2_QEI0CKEN_Msk         (0x1ul << CLK_APBCLK2_QEI0CKEN_Pos)               /*!< CLK_T::APBCLK2: QEI0CKEN Mask          */

#define CLK_APBCLK2_QEI1CKEN_Pos         (13)                                              /*!< CLK_T::APBCLK2: QEI1CKEN Position      */
#define CLK_APBCLK2_QEI1CKEN_Msk         (0x1ul << CLK_APBCLK2_QEI1CKEN_Pos)               /*!< CLK_T::APBCLK2: QEI1CKEN Mask          */

#define CLK_APBCLK2_QEI2CKEN_Pos         (14)                                              /*!< CLK_T::APBCLK2: QEI2CKEN Position      */
#define CLK_APBCLK2_QEI2CKEN_Msk         (0x1ul << CLK_APBCLK2_QEI2CKEN_Pos)               /*!< CLK_T::APBCLK2: QEI2CKEN Mask          */

#define CLK_APBCLK2_ADCCKEN_Pos          (24)                                              /*!< CLK_T::APBCLK2: ADCCKEN Position       */
#define CLK_APBCLK2_ADCCKEN_Msk          (0x1ul << CLK_APBCLK2_ADCCKEN_Pos)                /*!< CLK_T::APBCLK2: ADCCKEN Mask           */

#define CLK_APBCLK2_EADCCKEN_Pos         (25)                                              /*!< CLK_T::APBCLK2: EADCCKEN Position      */
#define CLK_APBCLK2_EADCCKEN_Msk         (0x1ul << CLK_APBCLK2_EADCCKEN_Pos)               /*!< CLK_T::APBCLK2: EADCCKEN Mask          */

#define CLK_CLKSEL0_CA35CKSEL_Pos        (0)                                               /*!< CLK_T::CLKSEL0: CA35CKSEL Position     */
#define CLK_CLKSEL0_CA35CKSEL_Msk        (0x3ul << CLK_CLKSEL0_CA35CKSEL_Pos)              /*!< CLK_T::CLKSEL0: CA35CKSEL Mask         */

#define CLK_CLKSEL0_SYSCK0SEL_Pos        (2)                                               /*!< CLK_T::CLKSEL0: SYSCK0SEL Position     */
#define CLK_CLKSEL0_SYSCK0SEL_Msk        (0x1ul << CLK_CLKSEL0_SYSCK0SEL_Pos)              /*!< CLK_T::CLKSEL0: SYSCK0SEL Mask         */

#define CLK_CLKSEL0_LVRDBSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: LVRDBSEL Position      */
#define CLK_CLKSEL0_LVRDBSEL_Msk         (0x1ul << CLK_CLKSEL0_LVRDBSEL_Pos)               /*!< CLK_T::CLKSEL0: LVRDBSEL Mask          */

#define CLK_CLKSEL0_SYSCK1SEL_Pos        (4)                                               /*!< CLK_T::CLKSEL0: SYSCK1SEL Position     */
#define CLK_CLKSEL0_SYSCK1SEL_Msk        (0x3ul << CLK_CLKSEL0_SYSCK1SEL_Pos)              /*!< CLK_T::CLKSEL0: SYSCK1SEL Mask         */

#define CLK_CLKSEL0_RTPSTSEL_Pos         (8)                                               /*!< CLK_T::CLKSEL0: RTPSTSEL Position       */
#define CLK_CLKSEL0_RTPSTSEL_Msk         (0x7ul << CLK_CLKSEL0_RTPSTSEL_Pos)               /*!< CLK_T::CLKSEL0: RTPSTSEL Mask           */

#define CLK_CLKSEL0_CCAP0SEL_Pos         (12)                                              /*!< CLK_T::CLKSEL0: CCAP0SEL Position      */
#define CLK_CLKSEL0_CCAP0SEL_Msk         (0x3ul << CLK_CLKSEL0_CCAP0SEL_Pos)               /*!< CLK_T::CLKSEL0: CCAP0SEL Mask          */

#define CLK_CLKSEL0_CCAP1SEL_Pos         (14)                                              /*!< CLK_T::CLKSEL0: CCAP1SEL Position      */
#define CLK_CLKSEL0_CCAP1SEL_Msk         (0x3ul << CLK_CLKSEL0_CCAP1SEL_Pos)               /*!< CLK_T::CLKSEL0: CCAP1SEL Mask          */

#define CLK_CLKSEL0_SD0SEL_Pos           (16)                                              /*!< CLK_T::CLKSEL0: SD0SEL Position        */
#define CLK_CLKSEL0_SD0SEL_Msk           (0x3ul << CLK_CLKSEL0_SD0SEL_Pos)                 /*!< CLK_T::CLKSEL0: SD0SEL Mask            */

#define CLK_CLKSEL0_SD1SEL_Pos           (18)                                              /*!< CLK_T::CLKSEL0: SD1SEL Position        */
#define CLK_CLKSEL0_SD1SEL_Msk           (0x3ul << CLK_CLKSEL0_SD1SEL_Pos)                 /*!< CLK_T::CLKSEL0: SD1SEL Mask            */

#define CLK_CLKSEL0_DCUSEL_Pos           (24)                                              /*!< CLK_T::CLKSEL0: DCUSEL Position        */
#define CLK_CLKSEL0_DCUSEL_Msk           (0x1ul << CLK_CLKSEL0_DCUSEL_Pos)                 /*!< CLK_T::CLKSEL0: DCUSEL Mask            */

#define CLK_CLKSEL0_DCUPSEL_Pos          (25)                                              /*!< CLK_T::CLKSEL0: DCUPSEL Position       */
#define CLK_CLKSEL0_DCUPSEL_Msk          (0x1ul << CLK_CLKSEL0_DCUPSEL_Pos)                /*!< CLK_T::CLKSEL0: DCUPSEL Mask           */

#define CLK_CLKSEL0_GFXSEL_Pos           (26)                                              /*!< CLK_T::CLKSEL0: GFXSEL Position        */
#define CLK_CLKSEL0_GFXSEL_Msk           (0x1ul << CLK_CLKSEL0_GFXSEL_Pos)                 /*!< CLK_T::CLKSEL0: GFXSEL Mask            */

#define CLK_CLKSEL1_TMR0SEL_Pos          (0)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos          (4)                                               /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_TMR2SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR2SEL Position       */
#define CLK_CLKSEL1_TMR2SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR2SEL Mask           */

#define CLK_CLKSEL1_TMR3SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR3SEL Position       */
#define CLK_CLKSEL1_TMR3SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR3SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR3SEL Mask           */

#define CLK_CLKSEL1_TMR4SEL_Pos          (16)                                              /*!< CLK_T::CLKSEL1: TMR4SEL Position       */
#define CLK_CLKSEL1_TMR4SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR4SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR4SEL Mask           */

#define CLK_CLKSEL1_TMR5SEL_Pos          (20)                                              /*!< CLK_T::CLKSEL1: TMR5SEL Position       */
#define CLK_CLKSEL1_TMR5SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR5SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR5SEL Mask           */

#define CLK_CLKSEL1_TMR6SEL_Pos          (24)                                              /*!< CLK_T::CLKSEL1: TMR6SEL Position       */
#define CLK_CLKSEL1_TMR6SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR6SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR6SEL Mask           */

#define CLK_CLKSEL1_TMR7SEL_Pos          (28)                                              /*!< CLK_T::CLKSEL1: TMR7SEL Position       */
#define CLK_CLKSEL1_TMR7SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR7SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR7SEL Mask           */

#define CLK_CLKSEL2_TMR8SEL_Pos          (0)                                               /*!< CLK_T::CLKSEL2: TMR8SEL Position       */
#define CLK_CLKSEL2_TMR8SEL_Msk          (0x7ul << CLK_CLKSEL2_TMR8SEL_Pos)                /*!< CLK_T::CLKSEL2: TMR8SEL Mask           */

#define CLK_CLKSEL2_TMR9SEL_Pos          (4)                                               /*!< CLK_T::CLKSEL2: TMR9SEL Position       */
#define CLK_CLKSEL2_TMR9SEL_Msk          (0x7ul << CLK_CLKSEL2_TMR9SEL_Pos)                /*!< CLK_T::CLKSEL2: TMR9SEL Mask           */

#define CLK_CLKSEL2_TMR10SEL_Pos         (8)                                               /*!< CLK_T::CLKSEL2: TMR10SEL Position      */
#define CLK_CLKSEL2_TMR10SEL_Msk         (0x7ul << CLK_CLKSEL2_TMR10SEL_Pos)               /*!< CLK_T::CLKSEL2: TMR10SEL Mask          */

#define CLK_CLKSEL2_TMR11SEL_Pos         (12)                                              /*!< CLK_T::CLKSEL2: TMR11SEL Position      */
#define CLK_CLKSEL2_TMR11SEL_Msk         (0x7ul << CLK_CLKSEL2_TMR11SEL_Pos)               /*!< CLK_T::CLKSEL2: TMR11SEL Mask          */

#define CLK_CLKSEL2_UART0SEL_Pos         (16)                                              /*!< CLK_T::CLKSEL2: UART0SEL Position      */
#define CLK_CLKSEL2_UART0SEL_Msk         (0x3ul << CLK_CLKSEL2_UART0SEL_Pos)               /*!< CLK_T::CLKSEL2: UART0SEL Mask          */

#define CLK_CLKSEL2_UART1SEL_Pos         (18)                                              /*!< CLK_T::CLKSEL2: UART1SEL Position      */
#define CLK_CLKSEL2_UART1SEL_Msk         (0x3ul << CLK_CLKSEL2_UART1SEL_Pos)               /*!< CLK_T::CLKSEL2: UART1SEL Mask          */

#define CLK_CLKSEL2_UART2SEL_Pos         (20)                                              /*!< CLK_T::CLKSEL2: UART2SEL Position      */
#define CLK_CLKSEL2_UART2SEL_Msk         (0x3ul << CLK_CLKSEL2_UART2SEL_Pos)               /*!< CLK_T::CLKSEL2: UART2SEL Mask          */

#define CLK_CLKSEL2_UART3SEL_Pos         (22)                                              /*!< CLK_T::CLKSEL2: UART3SEL Position      */
#define CLK_CLKSEL2_UART3SEL_Msk         (0x3ul << CLK_CLKSEL2_UART3SEL_Pos)               /*!< CLK_T::CLKSEL2: UART3SEL Mask          */

#define CLK_CLKSEL2_UART4SEL_Pos         (24)                                              /*!< CLK_T::CLKSEL2: UART4SEL Position      */
#define CLK_CLKSEL2_UART4SEL_Msk         (0x3ul << CLK_CLKSEL2_UART4SEL_Pos)               /*!< CLK_T::CLKSEL2: UART4SEL Mask          */

#define CLK_CLKSEL2_UART5SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL2: UART5SEL Position      */
#define CLK_CLKSEL2_UART5SEL_Msk         (0x3ul << CLK_CLKSEL2_UART5SEL_Pos)               /*!< CLK_T::CLKSEL2: UART5SEL Mask          */

#define CLK_CLKSEL2_UART6SEL_Pos         (28)                                              /*!< CLK_T::CLKSEL2: UART6SEL Position      */
#define CLK_CLKSEL2_UART6SEL_Msk         (0x3ul << CLK_CLKSEL2_UART6SEL_Pos)               /*!< CLK_T::CLKSEL2: UART6SEL Mask          */

#define CLK_CLKSEL2_UART7SEL_Pos         (30)                                              /*!< CLK_T::CLKSEL2: UART7SEL Position      */
#define CLK_CLKSEL2_UART7SEL_Msk         (0x3ul << CLK_CLKSEL2_UART7SEL_Pos)               /*!< CLK_T::CLKSEL2: UART7SEL Mask          */

#define CLK_CLKSEL3_UART8SEL_Pos         (0)                                               /*!< CLK_T::CLKSEL3: UART8SEL Position      */
#define CLK_CLKSEL3_UART8SEL_Msk         (0x3ul << CLK_CLKSEL3_UART8SEL_Pos)               /*!< CLK_T::CLKSEL3: UART8SEL Mask          */

#define CLK_CLKSEL3_UART9SEL_Pos         (2)                                               /*!< CLK_T::CLKSEL3: UART9SEL Position      */
#define CLK_CLKSEL3_UART9SEL_Msk         (0x3ul << CLK_CLKSEL3_UART9SEL_Pos)               /*!< CLK_T::CLKSEL3: UART9SEL Mask          */

#define CLK_CLKSEL3_UART10SEL_Pos        (4)                                               /*!< CLK_T::CLKSEL3: UART10SEL Position     */
#define CLK_CLKSEL3_UART10SEL_Msk        (0x3ul << CLK_CLKSEL3_UART10SEL_Pos)              /*!< CLK_T::CLKSEL3: UART10SEL Mask         */

#define CLK_CLKSEL3_UART11SEL_Pos        (6)                                               /*!< CLK_T::CLKSEL3: UART11SEL Position     */
#define CLK_CLKSEL3_UART11SEL_Msk        (0x3ul << CLK_CLKSEL3_UART11SEL_Pos)              /*!< CLK_T::CLKSEL3: UART11SEL Mask         */

#define CLK_CLKSEL3_UART12SEL_Pos        (8)                                               /*!< CLK_T::CLKSEL3: UART12SEL Position     */
#define CLK_CLKSEL3_UART12SEL_Msk        (0x3ul << CLK_CLKSEL3_UART12SEL_Pos)              /*!< CLK_T::CLKSEL3: UART12SEL Mask         */

#define CLK_CLKSEL3_UART13SEL_Pos        (10)                                              /*!< CLK_T::CLKSEL3: UART13SEL Position     */
#define CLK_CLKSEL3_UART13SEL_Msk        (0x3ul << CLK_CLKSEL3_UART13SEL_Pos)              /*!< CLK_T::CLKSEL3: UART13SEL Mask         */

#define CLK_CLKSEL3_UART14SEL_Pos        (12)                                              /*!< CLK_T::CLKSEL3: UART14SEL Position     */
#define CLK_CLKSEL3_UART14SEL_Msk        (0x3ul << CLK_CLKSEL3_UART14SEL_Pos)              /*!< CLK_T::CLKSEL3: UART14SEL Mask         */

#define CLK_CLKSEL3_UART15SEL_Pos        (14)                                              /*!< CLK_T::CLKSEL3: UART15SEL Position     */
#define CLK_CLKSEL3_UART15SEL_Msk        (0x3ul << CLK_CLKSEL3_UART15SEL_Pos)              /*!< CLK_T::CLKSEL3: UART15SEL Mask         */

#define CLK_CLKSEL3_UART16SEL_Pos        (16)                                              /*!< CLK_T::CLKSEL3: UART16SEL Position     */
#define CLK_CLKSEL3_UART16SEL_Msk        (0x3ul << CLK_CLKSEL3_UART16SEL_Pos)              /*!< CLK_T::CLKSEL3: UART16SEL Mask         */

#define CLK_CLKSEL3_WDT0SEL_Pos          (20)                                              /*!< CLK_T::CLKSEL3: WDT0SEL Position       */
#define CLK_CLKSEL3_WDT0SEL_Msk          (0x3ul << CLK_CLKSEL3_WDT0SEL_Pos)                /*!< CLK_T::CLKSEL3: WDT0SEL Mask           */

#define CLK_CLKSEL3_WWDT0SEL_Pos         (22)                                              /*!< CLK_T::CLKSEL3: WWDT0SEL Position      */
#define CLK_CLKSEL3_WWDT0SEL_Msk         (0x3ul << CLK_CLKSEL3_WWDT0SEL_Pos)               /*!< CLK_T::CLKSEL3: WWDT0SEL Mask          */

#define CLK_CLKSEL3_WDT1SEL_Pos          (24)                                              /*!< CLK_T::CLKSEL3: WDT1SEL Position       */
#define CLK_CLKSEL3_WDT1SEL_Msk          (0x3ul << CLK_CLKSEL3_WDT1SEL_Pos)                /*!< CLK_T::CLKSEL3: WDT1SEL Mask           */

#define CLK_CLKSEL3_WWDT1SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL3: WWDT1SEL Position      */
#define CLK_CLKSEL3_WWDT1SEL_Msk         (0x3ul << CLK_CLKSEL3_WWDT1SEL_Pos)               /*!< CLK_T::CLKSEL3: WWDT1SEL Mask          */

#define CLK_CLKSEL3_WDT2SEL_Pos          (28)                                              /*!< CLK_T::CLKSEL3: WDT2SEL Position       */
#define CLK_CLKSEL3_WDT2SEL_Msk          (0x3ul << CLK_CLKSEL3_WDT2SEL_Pos)                /*!< CLK_T::CLKSEL3: WDT2SEL Mask           */

#define CLK_CLKSEL3_WWDT2SEL_Pos         (30)                                              /*!< CLK_T::CLKSEL3: WWDT2SEL Position      */
#define CLK_CLKSEL3_WWDT2SEL_Msk         (0x3ul << CLK_CLKSEL3_WWDT2SEL_Pos)               /*!< CLK_T::CLKSEL3: WWDT2SEL Mask          */

#define CLK_CLKSEL4_SPI0SEL_Pos          (0)                                               /*!< CLK_T::CLKSEL4: SPI0SEL Position       */
#define CLK_CLKSEL4_SPI0SEL_Msk          (0x3ul << CLK_CLKSEL4_SPI0SEL_Pos)                /*!< CLK_T::CLKSEL4: SPI0SEL Mask           */

#define CLK_CLKSEL4_SPI1SEL_Pos          (2)                                               /*!< CLK_T::CLKSEL4: SPI1SEL Position       */
#define CLK_CLKSEL4_SPI1SEL_Msk          (0x3ul << CLK_CLKSEL4_SPI1SEL_Pos)                /*!< CLK_T::CLKSEL4: SPI1SEL Mask           */

#define CLK_CLKSEL4_SPI2SEL_Pos          (4)                                               /*!< CLK_T::CLKSEL4: SPI2SEL Position       */
#define CLK_CLKSEL4_SPI2SEL_Msk          (0x3ul << CLK_CLKSEL4_SPI2SEL_Pos)                /*!< CLK_T::CLKSEL4: SPI2SEL Mask           */

#define CLK_CLKSEL4_SPI3SEL_Pos          (6)                                               /*!< CLK_T::CLKSEL4: SPI3SEL Position       */
#define CLK_CLKSEL4_SPI3SEL_Msk          (0x3ul << CLK_CLKSEL4_SPI3SEL_Pos)                /*!< CLK_T::CLKSEL4: SPI3SEL Mask           */

#define CLK_CLKSEL4_QSPI0SEL_Pos         (8)                                               /*!< CLK_T::CLKSEL4: QSPI0SEL Position      */
#define CLK_CLKSEL4_QSPI0SEL_Msk         (0x3ul << CLK_CLKSEL4_QSPI0SEL_Pos)               /*!< CLK_T::CLKSEL4: QSPI0SEL Mask          */

#define CLK_CLKSEL4_QSPI1SEL_Pos         (10)                                              /*!< CLK_T::CLKSEL4: QSPI1SEL Position      */
#define CLK_CLKSEL4_QSPI1SEL_Msk         (0x3ul << CLK_CLKSEL4_QSPI1SEL_Pos)               /*!< CLK_T::CLKSEL4: QSPI1SEL Mask          */

#define CLK_CLKSEL4_I2S0SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL4: I2S0SEL Position       */
#define CLK_CLKSEL4_I2S0SEL_Msk          (0x3ul << CLK_CLKSEL4_I2S0SEL_Pos)                /*!< CLK_T::CLKSEL4: I2S0SEL Mask           */

#define CLK_CLKSEL4_I2S1SEL_Pos          (14)                                              /*!< CLK_T::CLKSEL4: I2S1SEL Position       */
#define CLK_CLKSEL4_I2S1SEL_Msk          (0x3ul << CLK_CLKSEL4_I2S1SEL_Pos)                /*!< CLK_T::CLKSEL4: I2S1SEL Mask           */

#define CLK_CLKSEL4_CAN0SEL_Pos          (16)                                              /*!< CLK_T::CLKSEL4: CAN0SEL Position       */
#define CLK_CLKSEL4_CAN0SEL_Msk          (0x1ul << CLK_CLKSEL4_CAN0SEL_Pos)                /*!< CLK_T::CLKSEL4: CAN0SEL Mask           */

#define CLK_CLKSEL4_CAN1SEL_Pos          (17)                                              /*!< CLK_T::CLKSEL4: CAN1SEL Position       */
#define CLK_CLKSEL4_CAN1SEL_Msk          (0x1ul << CLK_CLKSEL4_CAN1SEL_Pos)                /*!< CLK_T::CLKSEL4: CAN1SEL Mask           */

#define CLK_CLKSEL4_CAN2SEL_Pos          (18)                                              /*!< CLK_T::CLKSEL4: CAN2SEL Position       */
#define CLK_CLKSEL4_CAN2SEL_Msk          (0x1ul << CLK_CLKSEL4_CAN2SEL_Pos)                /*!< CLK_T::CLKSEL4: CAN2SEL Mask           */

#define CLK_CLKSEL4_CAN3SEL_Pos          (19)                                              /*!< CLK_T::CLKSEL4: CAN3SEL Position       */
#define CLK_CLKSEL4_CAN3SEL_Msk          (0x1ul << CLK_CLKSEL4_CAN3SEL_Pos)                /*!< CLK_T::CLKSEL4: CAN3SEL Mask           */

#define CLK_CLKSEL4_CLKOSEL_Pos          (24)                                              /*!< CLK_T::CLKSEL4: CLKOSEL Position       */
#define CLK_CLKSEL4_CLKOSEL_Msk          (0xFul << CLK_CLKSEL4_CLKOSEL_Pos)                /*!< CLK_T::CLKSEL4: CLKOSEL Mask           */

#define CLK_CLKSEL4_SC0SEL_Pos           (28)                                              /*!< CLK_T::CLKSEL4: SC0SEL Position        */
#define CLK_CLKSEL4_SC0SEL_Msk           (0x1ul << CLK_CLKSEL4_SC0SEL_Pos)                 /*!< CLK_T::CLKSEL4: SC0SEL Mask            */

#define CLK_CLKSEL4_SC1SEL_Pos           (29)                                              /*!< CLK_T::CLKSEL4: SC1SEL Position        */
#define CLK_CLKSEL4_SC1SEL_Msk           (0x1ul << CLK_CLKSEL4_SC1SEL_Pos)                 /*!< CLK_T::CLKSEL4: SC1SEL Mask            */

#define CLK_CLKSEL4_KPISEL_Pos           (30)                                              /*!< CLK_T::CLKSEL4: KPISEL Position        */
#define CLK_CLKSEL4_KPISEL_Msk           (0x1ul << CLK_CLKSEL4_KPISEL_Pos)                 /*!< CLK_T::CLKSEL4: KPISEL Mask            */

#define CLK_CLKDIV0_SDH0DIV_Pos          (0)                                               /*!< CLK_T::CLKDIV0: SDH0DIV Position       */
#define CLK_CLKDIV0_SDH0DIV_Msk          (0x7ul << CLK_CLKDIV0_SDH0DIV_Pos)                /*!< CLK_T::CLKDIV0: SDH0DIV Mask           */

#define CLK_CLKDIV0_SDH1DIV_Pos          (8)                                               /*!< CLK_T::CLKDIV0: SDH1DIV Position       */
#define CLK_CLKDIV0_SDH1DIV_Msk          (0x7ul << CLK_CLKDIV0_SDH1DIV_Pos)                /*!< CLK_T::CLKDIV0: SDH1DIV Mask           */

#define CLK_CLKDIV0_ACLK0DIV_Pos         (16)                                              /*!< CLK_T::CLKDIV0: ACLK0DIV Position      */
#define CLK_CLKDIV0_ACLK0DIV_Msk         (0x1ul << CLK_CLKDIV0_ACLK0DIV_Pos)               /*!< CLK_T::CLKDIV0: ACLK0DIV Mask          */

#define CLK_CLKDIV0_DCUPDIV_Pos        	 (24)                                              /*!< CLK_T::CLKDIV0: DCUPDIV Position       */
#define CLK_CLKDIV0_DCUPDIV_Msk        	 (0x3ul << CLK_CLKDIV0_DCUPDIV_Pos)                /*!< CLK_T::CLKDIV0: DCUPDIV Mask           */

#define CLK_CLKDIV0_EMAC0DIV_Pos         (20)                                              /*!< CLK_T::CLKDIV0: EMAC0DIV Position      */
#define CLK_CLKDIV0_EMAC0DIV_Msk         (0x3ul << CLK_CLKDIV0_EMAC0DIV_Pos)               /*!< CLK_T::CLKDIV0: EMAC0DIV Mask          */

#define CLK_CLKDIV0_EMAC1DIV_Pos         (22)                                              /*!< CLK_T::CLKDIV0: EMAC1DIV Position      */
#define CLK_CLKDIV0_EMAC1DIV_Msk         (0x3ul << CLK_CLKDIV0_EMAC1DIV_Pos)               /*!< CLK_T::CLKDIV0: EMAC1DIV Mask          */

#define CLK_CLKDIV1_SC0DIV_Pos           (0)                                               /*!< CLK_T::CLKDIV1: SC0DIV Position        */
#define CLK_CLKDIV1_SC0DIV_Msk           (0xful << CLK_CLKDIV1_SC0DIV_Pos)                 /*!< CLK_T::CLKDIV1: SC0DIV Mask            */

#define CLK_CLKDIV1_SC1DIV_Pos           (4)                                               /*!< CLK_T::CLKDIV1: SC1DIV Position        */
#define CLK_CLKDIV1_SC1DIV_Msk           (0xful << CLK_CLKDIV1_SC1DIV_Pos)                 /*!< CLK_T::CLKDIV1: SC1DIV Mask            */

#define CLK_CLKDIV1_CCAP0DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV1: CCAP0DIV Position      */
#define CLK_CLKDIV1_CCAP0DIV_Msk         (0xfful << CLK_CLKDIV1_CCAP0DIV_Pos)              /*!< CLK_T::CLKDIV1: CCAP0DIV Mask          */

#define CLK_CLKDIV1_CCAP1DIV_Pos         (12)                                              /*!< CLK_T::CLKDIV1: CCAP1DIV Position      */
#define CLK_CLKDIV1_CCAP1DIV_Msk         (0xfful << CLK_CLKDIV1_CCAP1DIV_Pos)              /*!< CLK_T::CLKDIV1: CCAP1DIV Mask          */

#define CLK_CLKDIV1_UART0DIV_Pos         (16)                                              /*!< CLK_T::CLKDIV1: UART0DIV Position      */
#define CLK_CLKDIV1_UART0DIV_Msk         (0xful << CLK_CLKDIV1_UART0DIV_Pos)               /*!< CLK_T::CLKDIV1: UART0DIV Mask          */

#define CLK_CLKDIV1_UART1DIV_Pos         (20)                                              /*!< CLK_T::CLKDIV1: UART1DIV Position      */
#define CLK_CLKDIV1_UART1DIV_Msk         (0xful << CLK_CLKDIV1_UART1DIV_Pos)               /*!< CLK_T::CLKDIV1: UART1DIV Mask          */

#define CLK_CLKDIV1_UART2DIV_Pos         (24)                                              /*!< CLK_T::CLKDIV1: UART2DIV Position      */
#define CLK_CLKDIV1_UART2DIV_Msk         (0xful << CLK_CLKDIV1_UART2DIV_Pos)               /*!< CLK_T::CLKDIV1: UART2DIV Mask          */

#define CLK_CLKDIV1_UART3DIV_Pos         (28)                                              /*!< CLK_T::CLKDIV1: UART3DIV Position      */
#define CLK_CLKDIV1_UART3DIV_Msk         (0xful << CLK_CLKDIV1_UART3DIV_Pos)               /*!< CLK_T::CLKDIV1: UART3DIV Mask          */

#define CLK_CLKDIV2_UART4DIV_Pos         (0)                                               /*!< CLK_T::CLKDIV2: UART4DIV Position      */
#define CLK_CLKDIV2_UART4DIV_Msk         (0xful << CLK_CLKDIV2_UART4DIV_Pos)               /*!< CLK_T::CLKDIV2: UART4DIV Mask          */

#define CLK_CLKDIV2_UART5DIV_Pos         (4)                                               /*!< CLK_T::CLKDIV2: UART5DIV Position      */
#define CLK_CLKDIV2_UART5DIV_Msk         (0xful << CLK_CLKDIV2_UART5DIV_Pos)               /*!< CLK_T::CLKDIV2: UART5DIV Mask          */

#define CLK_CLKDIV2_UART6DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV2: UART6DIV Position      */
#define CLK_CLKDIV2_UART6DIV_Msk         (0xful << CLK_CLKDIV2_UART6DIV_Pos)               /*!< CLK_T::CLKDIV2: UART6DIV Mask          */

#define CLK_CLKDIV2_UART7DIV_Pos         (12)                                              /*!< CLK_T::CLKDIV2: UART7DIV Position      */
#define CLK_CLKDIV2_UART7DIV_Msk         (0xful << CLK_CLKDIV2_UART7DIV_Pos)               /*!< CLK_T::CLKDIV2: UART7DIV Mask          */

#define CLK_CLKDIV2_UART8DIV_Pos         (16)                                              /*!< CLK_T::CLKDIV2: UART8DIV Position      */
#define CLK_CLKDIV2_UART8DIV_Msk         (0xful << CLK_CLKDIV2_UART8DIV_Pos)               /*!< CLK_T::CLKDIV2: UART8DIV Mask          */

#define CLK_CLKDIV2_UART9DIV_Pos         (20)                                              /*!< CLK_T::CLKDIV2: UART9DIV Position      */
#define CLK_CLKDIV2_UART9DIV_Msk         (0xful << CLK_CLKDIV2_UART9DIV_Pos)               /*!< CLK_T::CLKDIV2: UART9DIV Mask          */

#define CLK_CLKDIV2_UART10DIV_Pos        (24)                                              /*!< CLK_T::CLKDIV2: UART10DIV Position     */
#define CLK_CLKDIV2_UART10DIV_Msk        (0xful << CLK_CLKDIV2_UART10DIV_Pos)              /*!< CLK_T::CLKDIV2: UART10DIV Mask         */

#define CLK_CLKDIV2_UART11DIV_Pos        (28)                                              /*!< CLK_T::CLKDIV2: UART11DIV Position     */
#define CLK_CLKDIV2_UART11DIV_Msk        (0xful << CLK_CLKDIV2_UART11DIV_Pos)              /*!< CLK_T::CLKDIV2: UART11DIV Mask         */

#define CLK_CLKDIV3_UART12DIV_Pos        (0)                                               /*!< CLK_T::CLKDIV3: UART12DIV Position     */
#define CLK_CLKDIV3_UART12DIV_Msk        (0xful << CLK_CLKDIV3_UART12DIV_Pos)              /*!< CLK_T::CLKDIV3: UART12DIV Mask         */

#define CLK_CLKDIV3_UART13DIV_Pos        (4)                                               /*!< CLK_T::CLKDIV3: UART13DIV Position     */
#define CLK_CLKDIV3_UART13DIV_Msk        (0xful << CLK_CLKDIV3_UART13DIV_Pos)              /*!< CLK_T::CLKDIV3: UART13DIV Mask         */

#define CLK_CLKDIV3_UART14DIV_Pos        (8)                                               /*!< CLK_T::CLKDIV3: UART14DIV Position     */
#define CLK_CLKDIV3_UART14DIV_Msk        (0xful << CLK_CLKDIV3_UART14DIV_Pos)              /*!< CLK_T::CLKDIV3: UART14DIV Mask         */

#define CLK_CLKDIV3_UART15DIV_Pos        (12)                                              /*!< CLK_T::CLKDIV3: UART15DIV Position     */
#define CLK_CLKDIV3_UART15DIV_Msk        (0xful << CLK_CLKDIV3_UART15DIV_Pos)              /*!< CLK_T::CLKDIV3: UART15DIV Mask         */

#define CLK_CLKDIV3_UART16DIV_Pos        (16)                                              /*!< CLK_T::CLKDIV3: UART16DIV Position     */
#define CLK_CLKDIV3_UART16DIV_Msk        (0xful << CLK_CLKDIV3_UART16DIV_Pos)              /*!< CLK_T::CLKDIV3: UART16DIV Mask         */

#define CLK_CLKDIV3_TRACEDIV_Pos         (24)                                              /*!< CLK_T::CLKDIV3: TRACEDIV Position      */
#define CLK_CLKDIV3_TRACEDIV_Msk         (0xful << CLK_CLKDIV3_TRACEDIV_Pos)               /*!< CLK_T::CLKDIV3: TRACEDIV Mask          */

#define CLK_CLKDIV3_DBGDIV_Pos           (28)                                              /*!< CLK_T::CLKDIV3: DBGDIV Position        */
#define CLK_CLKDIV3_DBGDIV_Msk           (0xful << CLK_CLKDIV3_DBGDIV_Pos)                 /*!< CLK_T::CLKDIV3: DBGDIV Mask            */

#define CLK_CLKDIV4_EADCDIV_Pos          (0)                                               /*!< CLK_T::CLKDIV4: EADCDIV Position       */
#define CLK_CLKDIV4_EADCDIV_Msk          (0xful << CLK_CLKDIV4_EADCDIV_Pos)                /*!< CLK_T::CLKDIV4: EADCDIV Mask           */

#define CLK_CLKDIV4_ADCDIV_Pos           (4)                                               /*!< CLK_T::CLKDIV4: ADCDIV Position        */
#define CLK_CLKDIV4_ADCDIV_Msk           (0x1fffful << CLK_CLKDIV4_ADCDIV_Pos)             /*!< CLK_T::CLKDIV4: ADCDIV Mask            */

#define CLK_CLKDIV4_KPIDIV_Pos           (24)                                              /*!< CLK_T::CLKDIV4: KPIDIV Position        */
#define CLK_CLKDIV4_KPIDIV_Msk           (0xfful << CLK_CLKDIV4_KPIDIV_Pos)                /*!< CLK_T::CLKDIV4: KPIDIV Mask            */

#define CLK_CLKOCTL_FREQSEL_Pos          (0)                                               /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk          (0xful << CLK_CLKOCTL_FREQSEL_Pos)                /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

#define CLK_STATUS_HXTSTB_Pos            (0)                                               /*!< CLK_T::STATUS: HXTSTB Position         */
#define CLK_STATUS_HXTSTB_Msk            (0x1ul << CLK_STATUS_HXTSTB_Pos)                  /*!< CLK_T::STATUS: HXTSTB Mask             */

#define CLK_STATUS_LXTSTB_Pos            (1)                                               /*!< CLK_T::STATUS: LXTSTB Position         */
#define CLK_STATUS_LXTSTB_Msk            (0x1ul << CLK_STATUS_LXTSTB_Pos)                  /*!< CLK_T::STATUS: LXTSTB Mask             */

#define CLK_STATUS_SYSPLLSTB_Pos         (2)                                               /*!< CLK_T::STATUS: SYSPLLSTB Position      */
#define CLK_STATUS_SYSPLLSTB_Msk         (0x1ul << CLK_STATUS_SYSPLLSTB_Pos)               /*!< CLK_T::STATUS: SYSPLLSTB Mask          */

#define CLK_STATUS_LIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos           (4)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_CAPLLSTB_Pos          (6)                                               /*!< CLK_T::STATUS: CAPLLSTB Position       */
#define CLK_STATUS_CAPLLSTB_Msk          (0x1ul << CLK_STATUS_CAPLLSTB_Pos)                /*!< CLK_T::STATUS: CAPLLSTB Mask           */

#define CLK_STATUS_DDRPLLSTB_Pos         (8)                                               /*!< CLK_T::STATUS: DDRPLLSTB Position      */
#define CLK_STATUS_DDRPLLSTB_Msk         (0x1ul << CLK_STATUS_DDRPLLSTB_Pos)               /*!< CLK_T::STATUS: DDRPLLSTB Mask          */

#define CLK_STATUS_EPLLSTB_Pos           (9)                                               /*!< CLK_T::STATUS: EPLLSTB Position        */
#define CLK_STATUS_EPLLSTB_Msk           (0x1ul << CLK_STATUS_EPLLSTB_Pos)                 /*!< CLK_T::STATUS: EPLLSTB Mask            */

#define CLK_STATUS_APLLSTB_Pos           (10)                                              /*!< CLK_T::STATUS: APLLSTB Position        */
#define CLK_STATUS_APLLSTB_Msk           (0x1ul << CLK_STATUS_APLLSTB_Pos)                 /*!< CLK_T::STATUS: APLLSTB Mask            */

#define CLK_STATUS_VPLLSTB_Pos           (11)                                              /*!< CLK_T::STATUS: VPLLSTB Position        */
#define CLK_STATUS_VPLLSTB_Msk           (0x1ul << CLK_STATUS_VPLLSTB_Pos)                 /*!< CLK_T::STATUS: VPLLSTB Mask            */

#define CLK_PLL0CTL0_FBDIV_Pos           (0)                                               /*!< CLK_T::PLL0CTL0: FBDIV Position        */
#define CLK_PLL0CTL0_FBDIV_Msk           (0x7fful << CLK_PLL0CTL0_FBDIV_Pos)               /*!< CLK_T::PLL0CTL0: FBDIV Mask            */

#define CLK_PLL0CTL0_INDIV_Pos           (12)                                              /*!< CLK_T::PLL0CTL0: INDIV Position        */
#define CLK_PLL0CTL0_INDIV_Msk           (0x3ful << CLK_PLL0CTL0_INDIV_Pos)                /*!< CLK_T::PLL0CTL0: INDIV Mask            */

#define CLK_PLL0CTL0_MODE_Pos            (18)                                              /*!< CLK_T::PLL0CTL0: MODE Position         */
#define CLK_PLL0CTL0_MODE_Msk            (0x3ul << CLK_PLL0CTL0_MODE_Pos)                  /*!< CLK_T::PLL0CTL0: MODE Mask             */

#define CLK_PLL0CTL0_SSRATE_Pos          (20)                                              /*!< CLK_T::PLL0CTL0: SSRATE Position       */
#define CLK_PLL0CTL0_SSRATE_Msk          (0x7fful << CLK_PLL0CTL0_SSRATE_Pos)              /*!< CLK_T::PLL0CTL0: SSRATE Mask           */

#define CLK_PLL0CTL1_PD_Pos              (0)                                               /*!< CLK_T::PLL0CTL1: PD Position           */
#define CLK_PLL0CTL1_PD_Msk              (0x1ul << CLK_PLL0CTL1_PD_Pos)                    /*!< CLK_T::PLL0CTL1: PD Mask               */

#define CLK_PLL0CTL1_BP_Pos              (1)                                               /*!< CLK_T::PLL0CTL1: BP Position           */
#define CLK_PLL0CTL1_BP_Msk              (0x1ul << CLK_PLL0CTL1_BP_Pos)                    /*!< CLK_T::PLL0CTL1: BP Mask               */

#define CLK_PLL0CTL1_OUTDIV_Pos          (4)                                               /*!< CLK_T::PLL0CTL1: OUTDIV Position       */
#define CLK_PLL0CTL1_OUTDIV_Msk          (0x7ul << CLK_PLL0CTL1_OUTDIV_Pos)                /*!< CLK_T::PLL0CTL1: OUTDIV Mask           */

#define CLK_PLL0CTL1_FRAC_Pos            (8)                                               /*!< CLK_T::PLL0CTL1: FRAC Position         */
#define CLK_PLL0CTL1_FRAC_Msk            (0xfffffful << CLK_PLL0CTL1_FRAC_Pos)             /*!< CLK_T::PLL0CTL1: FRAC Mask             */

#define CLK_PLL0CTL2_SLOPE_Pos           (0)                                               /*!< CLK_T::PLL0CTL2: SLOPE Position        */
#define CLK_PLL0CTL2_SLOPE_Msk           (0xfffffful << CLK_PLL0CTL2_SLOPE_Pos)            /*!< CLK_T::PLL0CTL2: SLOPE Mask            */

#define CLK_CLKDCTL_HXTFDEN_Pos          (4)                                               /*!< CLK_T::CLKDCTL: HXTFDEN Position       */
#define CLK_CLKDCTL_HXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFDEN Mask           */

#define CLK_CLKDCTL_HXTFIEN_Pos          (5)                                               /*!< CLK_T::CLKDCTL: HXTFIEN Position       */
#define CLK_CLKDCTL_HXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFIEN Mask           */

#define CLK_CLKDCTL_LXTFDEN_Pos          (12)                                              /*!< CLK_T::CLKDCTL: LXTFDEN Position       */
#define CLK_CLKDCTL_LXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFDEN Mask           */

#define CLK_CLKDCTL_LXTFIEN_Pos          (13)                                              /*!< CLK_T::CLKDCTL: LXTFIEN Position       */
#define CLK_CLKDCTL_LXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFQDEN_Pos         (16)                                              /*!< CLK_T::CLKDCTL: HXTFQDEN Position      */
#define CLK_CLKDCTL_HXTFQDEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQDEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQDEN Mask          */

#define CLK_CLKDCTL_HXTFQIEN_Pos         (17)                                              /*!< CLK_T::CLKDCTL: HXTFQIEN Position      */
#define CLK_CLKDCTL_HXTFQIEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQIEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQIEN Mask          */

#define CLK_CLKDSTS_HXTFIF_Pos           (0)                                               /*!< CLK_T::CLKDSTS: HXTFIF Position        */
#define CLK_CLKDSTS_HXTFIF_Msk           (0x1ul << CLK_CLKDSTS_HXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: HXTFIF Mask            */

#define CLK_CLKDSTS_LXTFIF_Pos           (1)                                               /*!< CLK_T::CLKDSTS: LXTFIF Position        */
#define CLK_CLKDSTS_LXTFIF_Msk           (0x1ul << CLK_CLKDSTS_LXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: LXTFIF Mask            */

#define CLK_CLKDSTS_HXTFQIF_Pos          (8)                                               /*!< CLK_T::CLKDSTS: HXTFQIF Position       */
#define CLK_CLKDSTS_HXTFQIF_Msk          (0x1ul << CLK_CLKDSTS_HXTFQIF_Pos)                /*!< CLK_T::CLKDSTS: HXTFQIF Mask           */

#define CLK_CDUPB_UPERBD_Pos             (0)                                               /*!< CLK_T::CDUPB: UPERBD Position          */
#define CLK_CDUPB_UPERBD_Msk             (0x3fful << CLK_CDUPB_UPERBD_Pos)                 /*!< CLK_T::CDUPB: UPERBD Mask              */

#define CLK_CDLOWB_LOWERBD_Pos           (0)                                               /*!< CLK_T::CDLOWB: LOWERBD Position        */
#define CLK_CDLOWB_LOWERBD_Msk           (0x3fful << CLK_CDLOWB_LOWERBD_Pos)               /*!< CLK_T::CDLOWB: LOWERBD Mask            */

#define CLK_HXTFSEL_HXTFSEL_Pos          (0)                                               /*!< CLK_T::HXTFSEL: HXTFSEL Position       */
#define CLK_HXTFSEL_HXTFSEL_Msk          (0x1ul << CLK_HXTFSEL_HXTFSEL_Pos)                /*!< CLK_T::HXTFSEL: HXTFSEL Mask           */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __CLK_REG_H__ */
