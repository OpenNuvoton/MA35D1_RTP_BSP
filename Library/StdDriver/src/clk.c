/**************************************************************************//**
 * @file     clk.c
 * @brief    series CLK driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable clock divider output function
  * @param      None
  * @return     None
  * @details    This function disable clock divider output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK_DisableModuleClock(CLKO_MODULE);
}

/**
  * @brief      This function enable clock divider output module clock,
  *             enable clock divider output function and set frequency selection.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL4_CKOSEL_HXT
  *             - \ref CLK_CLKSEL4_CKOSEL_LXT
  *             - \ref CLK_CLKSEL4_CKOSEL_HIRC
  *             - \ref CLK_CLKSEL4_CKOSEL_LIRC
  *             - \ref CLK_CLKSEL4_CKOSEL_SYSPLL
  *             - \ref CLK_CLKSEL4_CKOSEL_DDRPLL
  *             - \ref CLK_CLKSEL4_CKOSEL_EPLL_DIV4
  *             - \ref CLK_CLKSEL4_CKOSEL_APLL
  *             - \ref CLK_CLKSEL4_CKOSEL_VPLL
  *             - \ref CLK_CLKSEL4_CKOSEL_CACLK
  *             - \ref CLK_CLKSEL4_CKOSEL_AXI0ACLK
  *             - \ref CLK_CLKSEL4_CKOSEL_SYSCLK0
  *             - \ref CLK_CLKSEL4_CKOSEL_SYSCLK1
  *             - \ref CLK_CLKSEL4_CKOSEL_PCLK3
  *             - \ref CLK_CLKSEL4_CKOSEL_PCLK4
  * @param[in]  u32ClkDiv is divider output frequency selection. It could be 0~15.
  * @param[in]  u32ClkDivBy1En is clock divided by one enabled.
  * @return     None
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv. \n
  *             The formula is: \n
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1) \n
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves. \n
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (u32ClkDiv) | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK_EnableModuleClock(CLKO_MODULE);

    /* Select CKO clock source */
    CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, 0UL);
}

/**
  * @brief      Enter to Power-down mode
  * @param      None
  * @return     None
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    SYS->PMUCR |= (SYS_PMUCR_RTPPDEN_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

}

/**
  * @brief      Enter to Idle mode
  * @param      None
  * @return     None
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    SYS->PMUCR &= ~(SYS_PMUCR_RTPPDEN_Msk);

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Get external high speed crystal clock frequency
  * @param      None
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    uint32_t u32Freq;

    if ((CLK->PWRCTL & CLK_PWRCTL_HXTEN_Msk) == CLK_PWRCTL_HXTEN_Msk)
    {
        u32Freq = __HXT;
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}


/**
  * @brief      Get external low speed crystal clock frequency
  * @param      None
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    uint32_t u32Freq;

    if ((CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk) == CLK_PWRCTL_LXTEN_Msk)
    {
        u32Freq = __LXT;
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Get SYSCLK0 frequency
  * @param      None
  * @return     SYSCLK0 frequency
  * @details    This function get SYSCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetSYSCLK0Freq(void)
{
    uint32_t u32Freq;

    if ((CLK->CLKSEL0 & CLK_CLKSEL0_SYSCK0SEL_Msk) == CLK_CLKSEL0_SYSCK0SEL_EPLL_DIV2)
    {
        u32Freq = CLK_GetPLLClockFreq(EPLL) / 2;
    }
    else
    {
        u32Freq = CLK_GetPLLClockFreq(SYSPLL);
    }

    return u32Freq;
}

/**
  * @brief      Get SYSCLK1 frequency
  * @param      None
  * @return     SYSCLK1 frequency
  * @details    This function get SYSCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetSYSCLK1Freq(void)
{
    uint32_t u32Freq;

    if ((CLK->CLKSEL0 & CLK_CLKSEL0_SYSCK1SEL_Msk) == CLK_CLKSEL0_SYSCK1SEL_HXT)
    {
        u32Freq = __HXT;
    }
    else if ((CLK->CLKSEL0 & CLK_CLKSEL0_SYSCK1SEL_Msk) == CLK_CLKSEL0_SYSCK1SEL_SYSPLL)
    {
        u32Freq = CLK_GetPLLClockFreq(SYSPLL);
    }
    else
    {
        u32Freq = CLK_GetPLLClockFreq(APLL);
    }

    return u32Freq;
}

/**
  * @brief      Get HCLK0~2 and PCLK0~2 frequency
  * @param      None
  * @return     HCLK0~2 and PCLK0~2 frequency
  * @details    This function get HCLK0~2 and PCLK0~2 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t u32Freq;

    u32Freq = CLK_GetSYSCLK1Freq();

    return u32Freq;
}

/**
  * @brief      Get HCLK3 and PCLK3~4 frequency
  * @param      None
  * @return     HCLK3 and PCLK3~4 frequency
  * @details    This function get HCLK3 and PCLK3~4 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK3Freq(void)
{
    uint32_t u32Freq;

    u32Freq = CLK_GetSYSCLK1Freq() / 2;

    return u32Freq;
}

/**
  * @brief      Get CPU frequency
  * @param      None
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    uint32_t u32Freq;

    u32Freq = CLK_GetSYSCLK1Freq();

    return u32Freq;
}


/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency. The range of u32Hclk is running up to 180MHz.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    /* The range of u32Hclk is running up to 180 MHz */
    if (u32Hclk > FREQ_180MHZ)
    {
        u32Hclk = FREQ_180MHZ;
    }

#if 0
    uint32_t u32HXTSTB;

    /* Read HIRC clock source stable flag */
    u32HXTSTB = CLK->STATUS & CLK_STATUS_HXTSTB_Msk;

    /* The range of u32Hclk is running up to 189 MHz */
    if (u32Hclk > FREQ_189MHZ)
    {
        u32Hclk = FREQ_189MHZ;
    }

    /* Switch HCLK clock source to HXT clock for safe */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_SYSCK1SEL_HXT;

    /* Configure PLL setting if HXT clock is enabled */
    if ((CLK->PWRCTL & CLK_PWRCTL_HXTEN_Msk) == CLK_PWRCTL_HXTEN_Msk)
    {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, u32Hclk);
    }
    /* Configure PLL setting if HXT clock is not enabled */
    else
    {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, u32Hclk);

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;
    }

    /* Select HCLK clock source to PLL,
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1UL));

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if (u32HIRCSTB == 0UL)
    {
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
    }

#endif
    /* Return actually HCLK frequency is PLL frequency divide 1 */
    return u32Hclk;
}

/**
  * @brief      This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_PLL
  *             - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV0_HCLK(x)
  * @return     None
  * @details    This function set HCLK clock source and HCLK clock divider. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
#if 0
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Apply new Divider */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if (u32HIRCSTB == 0UL)
    {
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
    }

#endif
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index          |Clock source                           |Divider                    |
  * | :----------------    | :-----------------------------------  | :----------------------   |
  * |\ref CA35CK_MODULE    |\ref CLK_CLKSEL0_CA35CKSEL_HXT         | x                         |
  * |\ref CA35CK_MODULE    |\ref CLK_CLKSEL0_CA35CKSEL_CA-PLL      | x                         |
  * |\ref CA35CK_MODULE    |\ref CLK_CLKSEL0_CA35CKSEL_APLL        | x                         |
  * |\ref CA35CK_MODULE    |\ref CLK_CLKSEL0_CA35CKSEL_EPLL        | x                         |
  * |\ref SYSCK0_MODULE    |\ref CLK_CLKSEL0_SYSCK0SEL_SYS-PLL     | x                         |
  * |\ref SYSCK0_MODULE    |\ref CLK_CLKSEL0_SYSCK0SEL_EPLL        | x                         |
  * |\ref LVRDB_MODULE     |\ref CLK_CLKSEL0_LVRDBSEL_LIRC         | x                         |
  * |\ref LVRDB_MODULE     |\ref CLK_CLKSEL0_LVRDBSEL_HIRC         | x                         |
  * |\ref SYSCK1_MODULE    |\ref CLK_CLKSEL0_SYSCK1SEL_HXT         | x                         |
  * |\ref SYSCK1_MODULE    |\ref CLK_CLKSEL0_SYSCK1SEL_SYS-PLL     | x                         |
  * |\ref SYSCK1_MODULE    |\ref CLK_CLKSEL0_SYSCK1SEL_APLL        | x                         |
  * |\ref RTPST_MODULE     |\ref CLK_CLKSEL0_RTPSTSEL_HXT          | x                         |
  * |\ref RTPST_MODULE     |\ref CLK_CLKSEL0_RTPSTSEL_LXT          | x                         |
  * |\ref RTPST_MODULE     |\ref CLK_CLKSEL0_RTPSTSEL_SYSCLK1      | x                         |
  * |\ref CCAP0_MODULE     |\ref CLK_CLKSEL0_CCAP0SEL_HXT          |\ref CLK_CLKDIV1_CCAP0(x)  |
  * |\ref CCAP0_MODULE     |\ref CLK_CLKSEL0_CCAP0SEL_SYS-PLL      |\ref CLK_CLKDIV1_CCAP0(x)  |
  * |\ref CCAP0_MODULE     |\ref CLK_CLKSEL0_CCAP0SEL_APLL         |\ref CLK_CLKDIV1_CCAP0(x)  |
  * |\ref CCAP0_MODULE     |\ref CLK_CLKSEL0_CCAP0SEL_VPLL         |\ref CLK_CLKDIV1_CCAP0(x)  |
  * |\ref CCAP1_MODULE     |\ref CLK_CLKSEL0_CCAP1SEL_HXT          |\ref CLK_CLKDIV1_CCAP1(x)  |
  * |\ref CCAP1_MODULE     |\ref CLK_CLKSEL0_CCAP1SEL_SYS-PLL      |\ref CLK_CLKDIV1_CCAP1(x)  |
  * |\ref CCAP1_MODULE     |\ref CLK_CLKSEL0_CCAP1SEL_APLL         |\ref CLK_CLKDIV1_CCAP1(x)  |
  * |\ref CCAP1_MODULE     |\ref CLK_CLKSEL0_CCAP1SEL_VPLL         |\ref CLK_CLKDIV1_CCAP1(x)  |
  * |\ref SD0_MODULE       |\ref CLK_CLKSEL0_SD0SEL_SYS-PLL        | x                         |
  * |\ref SD0_MODULE       |\ref CLK_CLKSEL0_SD0SEL_APLL           | x                         |
  * |\ref SD0_MODULE       |\ref CLK_CLKSEL0_SD0SEL_VPLL           | x                         |
  * |\ref SD1_MODULE       |\ref CLK_CLKSEL0_SD1SEL_SYS-PLL        | x                         |
  * |\ref SD1_MODULE       |\ref CLK_CLKSEL0_SD1SEL_APLL           | x                         |
  * |\ref SD1_MODULE       |\ref CLK_CLKSEL0_SD1SEL_VPLL           | x                         |
  * |\ref DCU_MODULE       |\ref CLK_CLKSEL0_DCUSEL_SYS-PLL        | x                         |
  * |\ref DCU_MODULE       |\ref CLK_CLKSEL0_DCUSEL_EPLL           | x                         |
  * |\ref DCUP_MODULE      |\ref CLK_CLKSEL0_DCUPSEL_APLL          |\ref CLK_CLKDIV0_DCUP(x)   |
  * |\ref DCUP_MODULE      |\ref CLK_CLKSEL0_DCUPSEL_VPLL          |\ref CLK_CLKDIV0_DCUP(x)   |
  * |\ref GFX_MODULE       |\ref CLK_CLKSEL0_GFXSEL_SYS-PLL        | x                         |
  * |\ref GFX_MODULE       |\ref CLK_CLKSEL0_GFXSEL_EPLL           | x                         |
  * |\ref DBG_MODULE       |\ref CLK_CLKSEL0_DBGSEL_HIRC           |\ref CLK_CLKDIV3_DBG(x)    |
  * |\ref DBG_MODULE       |\ref CLK_CLKSEL0_DBGSEL_SYS-PLL        |\ref CLK_CLKDIV3_DBG(x)    |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HXT           | x                         |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_LXT           | x                         |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_LIRC          | x                         |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HIRC          | x                         |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_PCLK1         | x                         |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_EXT           | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_HXT           | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_LXT           | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_LIRC          | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_HIRC          | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_PCLK1         | x                         |
  * |\ref TMR3_MODULE      |\ref CLK_CLKSEL1_TMR3SEL_EXT           | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_HXT           | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_LXT           | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_LIRC          | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_HIRC          | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_EXT           | x                         |
  * |\ref TMR4_MODULE      |\ref CLK_CLKSEL1_TMR4SEL_PCLK2         | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_HXT           | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_LXT           | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_LIRC          | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_HIRC          | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_EXT           | x                         |
  * |\ref TMR5_MODULE      |\ref CLK_CLKSEL1_TMR5SEL_PCLK2         | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_HXT           | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_LXT           | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_LIRC          | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_HIRC          | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_PCLK0         | x                         |
  * |\ref TMR6_MODULE      |\ref CLK_CLKSEL1_TMR6SEL_EXT           | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_HXT           | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_LXT           | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_LIRC          | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_HIRC          | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_PCLK0         | x                         |
  * |\ref TMR7_MODULE      |\ref CLK_CLKSEL1_TMR7SEL_EXT           | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_HXT           | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_LXT           | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_LIRC          | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_HIRC          | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_PCLK1         | x                         |
  * |\ref TMR8_MODULE      |\ref CLK_CLKSEL2_TMR8SEL_EXT           | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_HXT           | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_LXT           | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_LIRC          | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_HIRC          | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_PCLK1         | x                         |
  * |\ref TMR9_MODULE      |\ref CLK_CLKSEL2_TMR9SEL_EXT           | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_HXT          | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_LXT          | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_LIRC         | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_HIRC         | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_EXT          | x                         |
  * |\ref TMR10_MODULE     |\ref CLK_CLKSEL2_TMR10SEL_PCLK2        | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_HXT          | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_LXT          | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_LIRC         | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_HIRC         | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_EXT          | x                         |
  * |\ref TMR11_MODULE     |\ref CLK_CLKSEL2_TMR11SEL_PCLK2        | x                         |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL2_UART0SEL_HXT          |\ref CLK_CLKDIV1_UART0(x)  |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL2_UART0SEL_LXT          |\ref CLK_CLKDIV1_UART0(x)  |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL2_UART0SEL_HIRC         |\ref CLK_CLKDIV1_UART0(x)  |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL2_UART0SEL_SYSCLK1      |\ref CLK_CLKDIV1_UART0(x)  |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL2_UART1SEL_HXT          |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL2_UART1SEL_LXT          |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL2_UART1SEL_HIRC         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL2_UART1SEL_SYSCLK1      |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART2_MODULE     |\ref CLK_CLKSEL2_UART2SEL_HXT          |\ref CLK_CLKDIV1_UART2(x)  |
  * |\ref UART2_MODULE     |\ref CLK_CLKSEL2_UART2SEL_LXT          |\ref CLK_CLKDIV1_UART2(x)  |
  * |\ref UART2_MODULE     |\ref CLK_CLKSEL2_UART2SEL_HIRC         |\ref CLK_CLKDIV1_UART2(x)  |
  * |\ref UART2_MODULE     |\ref CLK_CLKSEL2_UART2SEL_SYSCLK1      |\ref CLK_CLKDIV1_UART2(x)  |
  * |\ref UART3_MODULE     |\ref CLK_CLKSEL2_UART3SEL_HXT          |\ref CLK_CLKDIV1_UART3(x)  |
  * |\ref UART3_MODULE     |\ref CLK_CLKSEL2_UART3SEL_LXT          |\ref CLK_CLKDIV1_UART3(x)  |
  * |\ref UART3_MODULE     |\ref CLK_CLKSEL2_UART3SEL_HIRC         |\ref CLK_CLKDIV1_UART3(x)  |
  * |\ref UART3_MODULE     |\ref CLK_CLKSEL2_UART3SEL_SYSCLK1      |\ref CLK_CLKDIV1_UART3(x)  |
  * |\ref UART4_MODULE     |\ref CLK_CLKSEL2_UART4SEL_HXT          |\ref CLK_CLKDIV2_UART4(x)  |
  * |\ref UART4_MODULE     |\ref CLK_CLKSEL2_UART4SEL_LXT          |\ref CLK_CLKDIV2_UART4(x)  |
  * |\ref UART4_MODULE     |\ref CLK_CLKSEL2_UART4SEL_HIRC         |\ref CLK_CLKDIV2_UART4(x)  |
  * |\ref UART4_MODULE     |\ref CLK_CLKSEL2_UART4SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART4(x)  |
  * |\ref UART5_MODULE     |\ref CLK_CLKSEL2_UART5SEL_HXT          |\ref CLK_CLKDIV2_UART5(x)  |
  * |\ref UART5_MODULE     |\ref CLK_CLKSEL2_UART5SEL_LXT          |\ref CLK_CLKDIV2_UART5(x)  |
  * |\ref UART5_MODULE     |\ref CLK_CLKSEL2_UART5SEL_HIRC         |\ref CLK_CLKDIV2_UART5(x)  |
  * |\ref UART5_MODULE     |\ref CLK_CLKSEL2_UART5SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART5(x)  |
  * |\ref UART6_MODULE     |\ref CLK_CLKSEL2_UART6SEL_HXT          |\ref CLK_CLKDIV2_UART6(x)  |
  * |\ref UART6_MODULE     |\ref CLK_CLKSEL2_UART6SEL_LXT          |\ref CLK_CLKDIV2_UART6(x)  |
  * |\ref UART6_MODULE     |\ref CLK_CLKSEL2_UART6SEL_HIRC         |\ref CLK_CLKDIV2_UART6(x)  |
  * |\ref UART6_MODULE     |\ref CLK_CLKSEL2_UART6SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART6(x)  |
  * |\ref UART7_MODULE     |\ref CLK_CLKSEL2_UART7SEL_HXT          |\ref CLK_CLKDIV2_UART7(x)  |
  * |\ref UART7_MODULE     |\ref CLK_CLKSEL2_UART7SEL_LXT          |\ref CLK_CLKDIV2_UART7(x)  |
  * |\ref UART7_MODULE     |\ref CLK_CLKSEL2_UART7SEL_HIRC         |\ref CLK_CLKDIV2_UART7(x)  |
  * |\ref UART7_MODULE     |\ref CLK_CLKSEL2_UART7SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART7(x)  |
  * |\ref UART8_MODULE     |\ref CLK_CLKSEL3_UART8SEL_HXT          |\ref CLK_CLKDIV2_UART8(x)  |
  * |\ref UART8_MODULE     |\ref CLK_CLKSEL3_UART8SEL_LXT          |\ref CLK_CLKDIV2_UART8(x)  |
  * |\ref UART8_MODULE     |\ref CLK_CLKSEL3_UART8SEL_HIRC         |\ref CLK_CLKDIV2_UART8(x)  |
  * |\ref UART8_MODULE     |\ref CLK_CLKSEL3_UART8SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART8(x)  |
  * |\ref UART9_MODULE     |\ref CLK_CLKSEL3_UART9SEL_HXT          |\ref CLK_CLKDIV2_UART9(x)  |
  * |\ref UART9_MODULE     |\ref CLK_CLKSEL3_UART9SEL_LXT          |\ref CLK_CLKDIV2_UART9(x)  |
  * |\ref UART9_MODULE     |\ref CLK_CLKSEL3_UART9SEL_HIRC         |\ref CLK_CLKDIV2_UART9(x)  |
  * |\ref UART9_MODULE     |\ref CLK_CLKSEL3_UART9SEL_SYSCLK1      |\ref CLK_CLKDIV2_UART9(x)  |
  * |\ref UART10_MODULE    |\ref CLK_CLKSEL3_UART10SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART10_MODULE    |\ref CLK_CLKSEL3_UART10SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART10_MODULE    |\ref CLK_CLKSEL3_UART10SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART10_MODULE    |\ref CLK_CLKSEL3_UART10SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART11_MODULE    |\ref CLK_CLKSEL3_UART11SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART11_MODULE    |\ref CLK_CLKSEL3_UART11SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART11_MODULE    |\ref CLK_CLKSEL3_UART11SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART11_MODULE    |\ref CLK_CLKSEL3_UART11SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART12_MODULE    |\ref CLK_CLKSEL3_UART12SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART12_MODULE    |\ref CLK_CLKSEL3_UART12SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART12_MODULE    |\ref CLK_CLKSEL3_UART12SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART12_MODULE    |\ref CLK_CLKSEL3_UART12SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART13_MODULE    |\ref CLK_CLKSEL3_UART13SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART13_MODULE    |\ref CLK_CLKSEL3_UART13SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART13_MODULE    |\ref CLK_CLKSEL3_UART13SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART13_MODULE    |\ref CLK_CLKSEL3_UART13SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART14_MODULE    |\ref CLK_CLKSEL3_UART14SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART14_MODULE    |\ref CLK_CLKSEL3_UART14SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART14_MODULE    |\ref CLK_CLKSEL3_UART14SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART14_MODULE    |\ref CLK_CLKSEL3_UART14SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART15_MODULE    |\ref CLK_CLKSEL3_UART15SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART15_MODULE    |\ref CLK_CLKSEL3_UART15SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART15_MODULE    |\ref CLK_CLKSEL3_UART15SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART15_MODULE    |\ref CLK_CLKSEL3_UART15SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART16_MODULE    |\ref CLK_CLKSEL3_UART16SEL_HXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART16_MODULE    |\ref CLK_CLKSEL3_UART16SEL_LXT         |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART16_MODULE    |\ref CLK_CLKSEL3_UART16SEL_HIRC        |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref UART16_MODULE    |\ref CLK_CLKSEL3_UART16SEL_SYSCLK1     |\ref CLK_CLKDIV1_UART1(x)  |
  * |\ref WDT0_MODULE      |\ref CLK_CLKSEL3_WDT0SEL_LXT           | x                         |
  * |\ref WDT0_MODULE      |\ref CLK_CLKSEL3_WDT0SEL_LIRC          | x                         |
  * |\ref WDT0_MODULE      |\ref CLK_CLKSEL3_WDT0SEL_PCLK3         | x                         |
  * |\ref WWDT0_MODULE     |\ref CLK_CLKSEL3_WWDT0SEL_LIRC         | x                         |
  * |\ref WWDT0_MODULE     |\ref CLK_CLKSEL3_WWDT0SEL_PCLK3        | x                         |
  * |\ref WDT1_MODULE      |\ref CLK_CLKSEL3_WDT1SEL_LXT           | x                         |
  * |\ref WDT1_MODULE      |\ref CLK_CLKSEL3_WDT1SEL_LIRC          | x                         |
  * |\ref WDT1_MODULE      |\ref CLK_CLKSEL3_WDT1SEL_PCLK3         | x                         |
  * |\ref WWDT1_MODULE     |\ref CLK_CLKSEL3_WWDT1SEL_LIRC         | x                         |
  * |\ref WWDT1_MODULE     |\ref CLK_CLKSEL3_WWDT1SEL_PCLK3        | x                         |
  * |\ref WDT2_MODULE      |\ref CLK_CLKSEL3_WDT2SEL_LXT           | x                         |
  * |\ref WDT2_MODULE      |\ref CLK_CLKSEL3_WDT2SEL_LIRC          | x                         |
  * |\ref WDT2_MODULE      |\ref CLK_CLKSEL3_WDT2SEL_PCLK4         | x                         |
  * |\ref WWDT2_MODULE     |\ref CLK_CLKSEL3_WWDT2SEL_LIRC         | x                         |
  * |\ref WWDT2_MODULE     |\ref CLK_CLKSEL3_WWDT2SEL_PCLK4        | x                         |
  * |\ref SPI0_MODULE      |\ref CLK_CLKSEL4_SPI0SEL_HXT           | x                         |
  * |\ref SPI0_MODULE      |\ref CLK_CLKSEL4_SPI0SEL_HIRC          | x                         |
  * |\ref SPI0_MODULE      |\ref CLK_CLKSEL4_SPI0SEL_PCLK1         | x                         |
  * |\ref SPI0_MODULE      |\ref CLK_CLKSEL4_SPI0SEL_EPLL          | x                         |
  * |\ref SPI1_MODULE      |\ref CLK_CLKSEL4_SPI1SEL_HXT           | x                         |
  * |\ref SPI1_MODULE      |\ref CLK_CLKSEL4_SPI1SEL_HIRC          | x                         |
  * |\ref SPI1_MODULE      |\ref CLK_CLKSEL4_SPI1SEL_EPLL          | x                         |
  * |\ref SPI1_MODULE      |\ref CLK_CLKSEL4_SPI1SEL_PCLK2         | x                         |
  * |\ref SPI2_MODULE      |\ref CLK_CLKSEL4_SPI2SEL_HXT           | x                         |
  * |\ref SPI2_MODULE      |\ref CLK_CLKSEL4_SPI2SEL_HIRC          | x                         |
  * |\ref SPI2_MODULE      |\ref CLK_CLKSEL4_SPI2SEL_PCLK1         | x                         |
  * |\ref SPI2_MODULE      |\ref CLK_CLKSEL4_SPI2SEL_EPLL          | x                         |
  * |\ref SPI3_MODULE      |\ref CLK_CLKSEL4_SPI3SEL_HXT           | x                         |
  * |\ref SPI3_MODULE      |\ref CLK_CLKSEL4_SPI3SEL_HIRC          | x                         |
  * |\ref SPI3_MODULE      |\ref CLK_CLKSEL4_SPI3SEL_EPLL          | x                         |
  * |\ref SPI3_MODULE      |\ref CLK_CLKSEL4_SPI3SEL_PCLK2         | x                         |
  * |\ref QSPI0_MODULE     |\ref CLK_CLKSEL4_QSPI0SEL_HXT          | x                         |
  * |\ref QSPI0_MODULE     |\ref CLK_CLKSEL4_QSPI0SEL_HIRC         | x                         |
  * |\ref QSPI0_MODULE     |\ref CLK_CLKSEL4_QSPI0SEL_PCLK0        | x                         |
  * |\ref QSPI0_MODULE     |\ref CLK_CLKSEL4_QSPI0SEL_EPLL         | x                         |
  * |\ref QSPI1_MODULE     |\ref CLK_CLKSEL4_QSPI1SEL_HXT          | x                         |
  * |\ref QSPI1_MODULE     |\ref CLK_CLKSEL4_QSPI1SEL_HIRC         | x                         |
  * |\ref QSPI1_MODULE     |\ref CLK_CLKSEL4_QSPI1SEL_PCLK0        | x                         |
  * |\ref QSPI1_MODULE     |\ref CLK_CLKSEL4_QSPI1SEL_EPLL         | x                         |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL4_I2S0SEL_HXT           | x                         |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL4_I2S0SEL_HIRC          | x                         |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL4_I2S0SEL_PCLK0         | x                         |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL4_I2S0SEL_APLL          | x                         |
  * |\ref I2S1_MODULE      |\ref CLK_CLKSEL4_I2S1SEL_HXT           | x                         |
  * |\ref I2S1_MODULE      |\ref CLK_CLKSEL4_I2S1SEL_HIRC          | x                         |
  * |\ref I2S1_MODULE      |\ref CLK_CLKSEL4_I2S1SEL_APLL          | x                         |
  * |\ref I2S1_MODULE      |\ref CLK_CLKSEL4_I2S1SEL_PCLK2         | x                         |
  * |\ref CAN0_MODULE      |\ref CLK_CLKSEL4_CAN0SEL_APLL          | x                         |
  * |\ref CAN0_MODULE      |\ref CLK_CLKSEL4_CAN0SEL_VPLL          | x                         |
  * |\ref CAN1_MODULE      |\ref CLK_CLKSEL4_CAN1SEL_APLL          | x                         |
  * |\ref CAN1_MODULE      |\ref CLK_CLKSEL4_CAN1SEL_VPLL          | x                         |
  * |\ref CAN2_MODULE      |\ref CLK_CLKSEL4_CAN2SEL_APLL          | x                         |
  * |\ref CAN2_MODULE      |\ref CLK_CLKSEL4_CAN2SEL_VPLL          | x                         |
  * |\ref CAN3_MODULE      |\ref CLK_CLKSEL4_CAN3SEL_APLL          | x                         |
  * |\ref CAN3_MODULE      |\ref CLK_CLKSEL4_CAN3SEL_VPLL          | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_HXT            | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_LXT            | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_LIRC           | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_HIRC           | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_CAPLL          | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_SYSPLL         | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_APLL           | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_EPLL           | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_VPLL           | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_SYSCLK0        | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_SYSCLK1        | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_PCLK3          | x                         |
  * |\ref CKO_MODULE       |\ref CLK_CLKSEL4_CKOSEL_PCLK4          | x                         |
  * |\ref SC0_MODULE       |\ref CLK_CLKSEL4_SC0SEL_HXT            |\ref CLK_CLKDIV1_SC0(x)    |
  * |\ref SC0_MODULE       |\ref CLK_CLKSEL4_SC0SEL_PCLK4          |\ref CLK_CLKDIV1_SC0(x)    |
  * |\ref SC1_MODULE       |\ref CLK_CLKSEL4_SC1SEL_HXT            |\ref CLK_CLKDIV1_SC1(x)    |
  * |\ref SC1_MODULE       |\ref CLK_CLKSEL4_SC1SEL_PCLK4          |\ref CLK_CLKDIV1_SC1(x)    |
  * |\ref KPI_MODULE       |\ref CLK_CLKSEL4_KPISEL_HXT            |\ref CLK_CLKDIV4_KPI(x)    |
  * |\ref KPI_MODULE       |\ref CLK_CLKSEL4_KPISEL_LXT            |\ref CLK_CLKDIV4_KPI(x)    |
  * |\ref ADC_MODULE       | x                                     |\ref CLK_CLKDIV4_ADC(x)    |
  * |\ref EADC_MODULE      | x                                     |\ref CLK_CLKDIV4_EADC(x)   |
  *
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32sel = 0U, u32div = 0U;

    if (u32ModuleIdx == KPI_MODULE)
    {
        CLK->CLKDIV4 = CLK->CLKDIV4 & ~(CLK_CLKDIV4_KPIDIV_Msk) | u32ClkDiv;
        CLK->CLKSEL4 = CLK->CLKSEL4 & ~(CLK_CLKSEL4_KPISEL_Msk) | u32ClkSrc;
        CLK->APBCLK0 = CLK->APBCLK0 & ~(CLK_APBCLK0_KPICKEN_Msk) | CLK_APBCLK0_KPICKEN_Msk;
    }
    else if (u32ModuleIdx == ADC_MODULE)
    {
        CLK->CLKDIV4 = CLK->CLKDIV4 & ~(CLK_CLKDIV4_ADCDIV_Msk) | u32ClkDiv;
        CLK->APBCLK2 = CLK->APBCLK2 & ~(CLK_APBCLK2_ADCCKEN_Msk) | CLK_APBCLK2_ADCCKEN_Msk;
    }
    else
    {
        if (MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
        {
            /* Get clock divider control register address */
            u32div = (uint32_t)&CLK->CLKDIV0 + ((MODULE_CLKDIV(u32ModuleIdx)) * 4U);
            /* Apply new divider */
            M32(u32div) = (M32(u32div) & (~(MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
        }

        if (MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
        {
            /* Get clock select control register address */
            u32sel = (uint32_t)&CLK->CLKSEL0 + ((MODULE_CLKSEL(u32ModuleIdx)) * 4U);
            /* Set new clock selection setting */
            M32(u32sel) = (M32(u32sel) & (~(MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
        }
    }
}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HXT
  *             - \ref CLK_CLKSEL0_RTPSTSEL_LXT
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_RTPSTSEL_SYSCLK1_DIV2
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HIRC
  * @return     None
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_RTPSTSEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN_Msk
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN_Msk
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief      Enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PDMA2_MODULE
  *             - \ref PDMA3_MODULE
  *             - \ref WH0_MODULE
  *             - \ref WH1_MODULE
  *             - \ref HWS_MODULE
  *             - \ref EBI_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ROM_MODULE
  *             - \ref TRA_MODULE
  *             - \ref DBG_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref GTMR_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref GPI_MODULE
  *             - \ref GPJ_MODULE
  *             - \ref GPK_MODULE
  *             - \ref GPL_MODULE
  *             - \ref GPM_MODULE
  *             - \ref GPN_MODULE
  *             - \ref TAHB_MODULE
  *             - \ref DDR0_MODULE
  *             - \ref DDR6_MODULE
  *             - \ref CAN0_MODULE
  *             - \ref CAN1_MODULE
  *             - \ref CAN2_MODULE
  *             - \ref CAN3_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref TMR9_MODULE
  *             - \ref TMR10_MODULE
  *             - \ref TMR11_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref UART10_MODULE
  *             - \ref UART11_MODULE
  *             - \ref UART12_MODULE
  *             - \ref UART13_MODULE
  *             - \ref UART14_MODULE
  *             - \ref UART15_MODULE
  *             - \ref UART16_MODULE
  *             - \ref RTC_MODULE
  *             - \ref DDRP_MODULE
  *             - \ref KPI_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2C4_MODULE
  *             - \ref I2C5_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WDT2_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EPWM2_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref QEI0_MODULE
  *             - \ref QEI1_MODULE
  *             - \ref QEI2_MODULE
  *             - \ref ADC_MODULE
  *             - \ref EADC_MODULE
  * @return     None
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32tmpVal = 0UL, u32tmpAddr = 0UL;

    u32tmpVal = (1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32tmpAddr = (uint32_t)&CLK->SYSCLK0;
    u32tmpAddr += ((MODULE_APBCLK(u32ModuleIdx) * 4UL));

    *(volatile uint32_t *)u32tmpAddr |= u32tmpVal;
}

/**
  * @brief      Disable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PDMA2_MODULE
  *             - \ref PDMA3_MODULE
  *             - \ref WH0_MODULE
  *             - \ref WH1_MODULE
  *             - \ref HWS_MODULE
  *             - \ref EBI_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ROM_MODULE
  *             - \ref TRA_MODULE
  *             - \ref DBG_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref GTMR_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref GPI_MODULE
  *             - \ref GPJ_MODULE
  *             - \ref GPK_MODULE
  *             - \ref GPL_MODULE
  *             - \ref GPM_MODULE
  *             - \ref GPN_MODULE
  *             - \ref TAHB_MODULE
  *             - \ref DDR0_MODULE
  *             - \ref DDR6_MODULE
  *             - \ref CAN0_MODULE
  *             - \ref CAN1_MODULE
  *             - \ref CAN2_MODULE
  *             - \ref CAN3_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref TMR9_MODULE
  *             - \ref TMR10_MODULE
  *             - \ref TMR11_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref UART10_MODULE
  *             - \ref UART11_MODULE
  *             - \ref UART12_MODULE
  *             - \ref UART13_MODULE
  *             - \ref UART14_MODULE
  *             - \ref UART15_MODULE
  *             - \ref UART16_MODULE
  *             - \ref RTC_MODULE
  *             - \ref DDRP_MODULE
  *             - \ref KPI_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2C4_MODULE
  *             - \ref I2C5_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WDT2_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EPWM2_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref QEI0_MODULE
  *             - \ref QEI1_MODULE
  *             - \ref QEI2_MODULE
  *             - \ref ADC_MODULE
  *             - \ref EADC_MODULE
  * @return     None
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32tmpVal = 0UL, u32tmpAddr = 0UL;

    u32tmpVal = ~(1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32tmpAddr = (uint32_t)&CLK->SYSCLK0;
    u32tmpAddr += ((MODULE_APBCLK(u32ModuleIdx) * 4UL));

    *(uint32_t *)u32tmpAddr &= u32tmpVal;
}

/**
  * @brief      Get PLL Operation Mode
  * @param[in]  u32PllIdx is PLL clock index. Including :
  *             - \ref CAPLL
  *             - \ref SYSPLL
  *             - \ref DDRPLL
  *             - \ref APLL
  *             - \ref EPLL
  *             - \ref VPLL
  * @return     PLL PLL Operation Mode
  * @details    This function get PLL Operation Mode.
  */
uint32_t CLK_GetPLLOpMode(uint32_t u32PllIdx)
{
    uint32_t u32Mode;

    if (u32PllIdx == CAPLL)
        u32Mode = (CLK->PLL0CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;
    else if (u32PllIdx == SYSPLL)
        u32Mode = (CLK->PLL1CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;
    else if (u32PllIdx == DDRPLL)
        u32Mode = (CLK->PLL2CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;
    else if (u32PllIdx == APLL)
        u32Mode = (CLK->PLL3CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;
    else if (u32PllIdx == EPLL)
        u32Mode = (CLK->PLL4CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;
    else
        u32Mode = (CLK->PLL5CTL0 & ~(CLK_PLL0CTL0_MODE_Msk)) >> CLK_PLL0CTL0_MODE_Pos;

    return u32Mode;
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_SYSPLLSTB_Msk
  *             - \ref CLK_STATUS_CAPLLSTB_Msk
  *             - \ref CLK_STATUS_DDRPLLSTB_Msk
  *             - \ref CLK_STATUS_EPLLSTB_Msk
  *             - \ref CLK_STATUS_APLLSTB_Msk
  *             - \ref CLK_STATUS_VPLLSTB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~300ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = 2160000;
    uint32_t u32Ret = 1U;

    while ((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    return u32Ret;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HXT
  *             - \ref CLK_CLKSEL0_RTPSTSEL_LXT
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_RTPSTSEL_SYSCLK1_DIV2
  *             - \ref CLK_CLKSEL0_RTPSTSEL_HIRC
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;

    /* Set System Tick clock source */
    if (u32ClkSrc == CLK_CLKSEL0_RTPSTSEL_HIRC)
    {
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    }
    else
    {
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_RTPSTSEL_Msk) | u32ClkSrc;
    }

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0UL;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;
}

/**
  * @brief      Get PLL clock frequency
  * @param[in]  u32PllIdx is PLL clock index. Including :
  *             - \ref CAPLL
  *             - \ref SYSPLL
  *             - \ref DDRPLL
  *             - \ref APLL
  *             - \ref EPLL
  *             - \ref VPLL
  * @return     PLL frequency
  * @details    This function get PLL frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPLLClockFreq(uint32_t u32PllIdx)
{
    uint32_t u32M, u32N, u32P, u32X, u32Mode;
    uint32_t u32CTLVal0, u32CTLVal1, u32PllClk, u32RefClk;

    u32Mode = CLK_GetPLLOpMode(u32PllIdx);

    u32CTLVal0 = *(volatile uint32_t *)(CLK_BASE + 0x60 + (0x10 * (u32PllIdx)));
    u32CTLVal1 = *(volatile uint32_t *)(CLK_BASE + 0x64 + (0x10 * (u32PllIdx)));

    if (u32Mode == 0)
    {
        u32N = (u32CTLVal0 & CLK_PLL0CTL0_FBDIV_Msk);
        u32M = (u32CTLVal0 & CLK_PLL0CTL0_INDIV_Msk) >> (CLK_PLL0CTL0_INDIV_Pos);
        u32P = (u32CTLVal1 & CLK_PLL0CTL1_OUTDIV_Msk) >> (CLK_PLL0CTL1_OUTDIV_Pos);

        /* u32RefClk is shifted to avoid overflow */
        u32RefClk = __HXT / 100;
        /* Actual PLL output clock frequency */
        u32PllClk = ((u32RefClk * u32N) / (u32P * u32M)) * 100;

    }
    else if (u32Mode == 1)
    {
        u32N = (u32CTLVal0 & CLK_PLL0CTL0_FBDIV_Msk);
        u32M = (u32CTLVal0 & CLK_PLL0CTL0_INDIV_Msk) >> (CLK_PLL0CTL0_INDIV_Pos);
        u32P = (u32CTLVal1 & CLK_PLL0CTL1_OUTDIV_Msk) >> (CLK_PLL0CTL1_OUTDIV_Pos);
        u32X =	(u32CTLVal1 & CLK_PLL0CTL1_FRAC_Msk) >> (CLK_PLL0CTL1_FRAC_Pos);

        /* Actual PLL output clock frequency */
        u32X = (((u32X * 1000) + 500) >> 24);
        u32PllClk = (__HXT * ((u32N * 1000) + u32X)) / 1000 / u32P / u32M;
    }
    else
    {
        u32N = (u32CTLVal0 & CLK_PLL0CTL0_FBDIV_Msk);
        u32M = (u32CTLVal0 & CLK_PLL0CTL0_INDIV_Msk) >> (CLK_PLL0CTL0_INDIV_Pos);
        //u32SR = (u32CTLVal0 & CLK_PLL0CTL0_SSRATE_Msk) >> (CLK_PLL0CTL0_SSRATE_Pos);

        u32P = (u32CTLVal1 & CLK_PLL0CTL1_OUTDIV_Msk) >> (CLK_PLL0CTL1_OUTDIV_Pos);
        u32X =	(u32CTLVal1 & CLK_PLL0CTL1_FRAC_Msk) >> (CLK_PLL0CTL1_FRAC_Pos);

        /* Actual PLL output clock frequency */
        u32X = ((u32X * 1000) >> 24);
        u32PllClk = (__HXT * ((u32N * 1000) + u32X)) / 1000 / u32P / u32M;
    }

    return u32PllClk;
}

/**
  * @brief      Get selected module clock source
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PDMA2_MODULE
  *             - \ref PDMA3_MODULE
  *             - \ref WH0_MODULE
  *             - \ref WH1_MODULE
  *             - \ref HWS_MODULE
  *             - \ref EBI_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ROM_MODULE
  *             - \ref TRA_MODULE
  *             - \ref DBG_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref GTMR_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref GPI_MODULE
  *             - \ref GPJ_MODULE
  *             - \ref GPK_MODULE
  *             - \ref GPL_MODULE
  *             - \ref GPM_MODULE
  *             - \ref GPN_MODULE
  *             - \ref TAHB_MODULE
  *             - \ref DDR0_MODULE
  *             - \ref DDR6_MODULE
  *             - \ref CAN0_MODULE
  *             - \ref CAN1_MODULE
  *             - \ref CAN2_MODULE
  *             - \ref CAN3_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref TMR9_MODULE
  *             - \ref TMR10_MODULE
  *             - \ref TMR11_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref UART10_MODULE
  *             - \ref UART11_MODULE
  *             - \ref UART12_MODULE
  *             - \ref UART13_MODULE
  *             - \ref UART14_MODULE
  *             - \ref UART15_MODULE
  *             - \ref UART16_MODULE
  *             - \ref RTC_MODULE
  *             - \ref DDRP_MODULE
  *             - \ref KPI_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2C4_MODULE
  *             - \ref I2C5_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WDT2_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EPWM2_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref QEI0_MODULE
  *             - \ref QEI1_MODULE
  *             - \ref QEI2_MODULE
  *             - \ref ADC_MODULE
  *             - \ref EADC_MODULE
  * @return     Selected module clock source setting
  * @details    This function get selected module clock source.
  */
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx)
{
    uint32_t u32sel = 0;
    uint32_t u32SelTbl[5] = {0x0, 0x4, 0x8, 0xC, 0x10};

    /* Get clock source selection setting */
    if (MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + (u32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);
        /* Get clock source selection setting */
        return ((M32(u32sel) & (MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx))) >> MODULE_CLKSEL_Pos(u32ModuleIdx));
    }
    else
        return 0;
}

/**
  * @brief      Get selected module clock divider number
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PDMA2_MODULE
  *             - \ref PDMA3_MODULE
  *             - \ref WH0_MODULE
  *             - \ref WH1_MODULE
  *             - \ref HWS_MODULE
  *             - \ref EBI_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ROM_MODULE
  *             - \ref TRA_MODULE
  *             - \ref DBG_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref GTMR_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref GPI_MODULE
  *             - \ref GPJ_MODULE
  *             - \ref GPK_MODULE
  *             - \ref GPL_MODULE
  *             - \ref GPM_MODULE
  *             - \ref GPN_MODULE
  *             - \ref TAHB_MODULE
  *             - \ref DDR0_MODULE
  *             - \ref DDR6_MODULE
  *             - \ref CAN0_MODULE
  *             - \ref CAN1_MODULE
  *             - \ref CAN2_MODULE
  *             - \ref CAN3_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref TMR9_MODULE
  *             - \ref TMR10_MODULE
  *             - \ref TMR11_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref UART10_MODULE
  *             - \ref UART11_MODULE
  *             - \ref UART12_MODULE
  *             - \ref UART13_MODULE
  *             - \ref UART14_MODULE
  *             - \ref UART15_MODULE
  *             - \ref UART16_MODULE
  *             - \ref RTC_MODULE
  *             - \ref DDRP_MODULE
  *             - \ref KPI_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2C4_MODULE
  *             - \ref I2C5_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WDT2_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EPWM2_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref QEI0_MODULE
  *             - \ref QEI1_MODULE
  *             - \ref QEI2_MODULE
  *             - \ref ADC_MODULE
  *             - \ref EADC_MODULE
  * @return     Selected module clock divider number setting
  * @details    This function get selected module clock divider number.
  */
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx)
{
    uint32_t u32div = 0;
    uint32_t u32DivTbl[5] = {0x0, 0x4, 0xc, 0xc, 0x10};

    if (MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        if (u32ModuleIdx == KPI_MODULE)
        {
            u32div = (CLK->CLKDIV4 & CLK_CLKDIV4_KPIDIV_Msk) >> CLK_CLKDIV4_KPIDIV_Pos;
            return u32div;
        }
        else if (u32ModuleIdx == ADC_MODULE)
        {
            u32div = (CLK->CLKDIV4 & CLK_CLKDIV4_ADCDIV_Msk) >> CLK_CLKDIV4_ADCDIV_Pos;
            return u32div;
        }
        else
        {
            /* Get clock divider control register address */
            u32div = (uint32_t)&CLK->CLKDIV0 + (u32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);
            /* Get clock divider number setting */
            return ((M32(u32div) & (MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx))) >> MODULE_CLKDIV_Pos(u32ModuleIdx));
        }
    }
    else
        return 0;
}


/*@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CLK_Driver */

/*@}*/ /* end of group Standard_Driver */

