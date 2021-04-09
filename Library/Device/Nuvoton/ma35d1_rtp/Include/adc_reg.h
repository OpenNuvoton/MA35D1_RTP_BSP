/**************************************************************************//**
 * @file     adc.h
 * @brief    ADC driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __ADC_REG_H__
#define __ADC_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */


typedef struct
{
    __IO uint32_t CTL;          /* 0x000 */
    __IO uint32_t CONF;
    __IO uint32_t IER;
    __IO uint32_t ISR;
    __IO uint32_t WKISR;        /* 0x010 */
    __IO uint32_t RESERVE0[3];
    __IO uint32_t XYDATA;       /* 0x020 */
    __IO uint32_t ZDATA;
    __IO uint32_t DATA;
    __IO uint32_t RESERVE1[114];
    __IO uint32_t XYSORT[4];    /* 0x1F4 */
    __IO uint32_t ZSORT[4];
} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_CTL_ADEN_Pos            (0)
#define ADC_CTL_ADEN_Msk            (0x1ul << ADC_CTL_ADEN_Pos)

#define ADC_CTL_MST_Pos             (8)
#define ADC_CTL_MST_Msk             (0x1ul << ADC_CTL_MST_Pos)

#define ADC_CTL_PEDEEN_Pos          (9)
#define ADC_CTL_PEDEEN_Msk          (0x1ul << ADC_CTL_PEDEEN_Pos)

#define ADC_CTL_WKTEN_Pos           (11)
#define ADC_CTL_WKTEN_Msk           (0x1ul << ADC_CTL_WKTEN_Pos)

#define ADC_CTL_WMSWCH_Pos          (16)
#define ADC_CTL_WMSWCH_Msk          (0x1ul << ADC_CTL_WMSWCH_Pos)

#define ADC_CONF_TEN_Pos            (0)
#define ADC_CONF_TEN_Msk            (0x1ul << ADC_CONF_TEN_Pos)

#define ADC_CONF_ZEN_Pos            (1)
#define ADC_CONF_ZEN_Msk            (0x1ul << ADC_CONF_ZEN_Pos)

#define ADC_CONF_NAEN_Pos           (2)
#define ADC_CONF_NAEN_Msk           (0x1ul << ADC_CONF_NAEN_Pos)

#define ADC_CONF_REFSEL_Pos         (6)
#define ADC_CONF_REFSEL_Msk         (0x3ul << ADC_CONF_REFSEL_Pos)

#define ADC_CONF_CHSEL_Pos          (12)
#define ADC_CONF_CHSEL_Msk          (0x7ul << ADC_CONF_CHSEL_Pos)

#define ADC_CONF_TMAVDIS_Pos        (20)
#define ADC_CONF_TMAVDIS_Msk        (0x1ul << ADC_CONF_TMAVDIS_Pos)

#define ADC_CONF_ZMAVDIS_Pos        (21)
#define ADC_CONF_ZMAVDIS_Msk        (0x1ul << ADC_CONF_ZMAVDIS_Pos)

#define ADC_CONF_SPEED_Pos          (22)
#define ADC_CONF_SPEED_Msk          (0x1ul << ADC_CONF_SPEED_Pos)

#define ADC_IER_MIEN_Pos            (0)
#define ADC_IER_MIEN_Msk            (0x1ul << ADC_IER_MIEN_Pos)

#define ADC_IER_PEDEIEN_Pos         (2)
#define ADC_IER_PEDEIEN_Msk         (0x1ul << ADC_IER_PEDEIEN_Pos)

#define ADC_IER_WKTIEN_Pos          (3)
#define ADC_IER_WKTIEN_Msk          (0x1ul << ADC_IER_WKTIEN_Pos)

#define ADC_ISR_MF_Pos              (0)
#define ADC_ISR_MF_Msk              (0x1ul << ADC_ISR_MF_Pos)

#define ADC_ISR_PEDEF_Pos           (2)
#define ADC_ISR_PEDEF_Msk           (0x1ul << ADC_ISR_PEDEF_Pos)

#define ADC_ISR_TF_Pos              (8)
#define ADC_ISR_TF_Msk              (0x1ul << ADC_ISR_TF_Pos)

#define ADC_ISR_ZF_Pos              (9)
#define ADC_ISR_ZF_Msk              (0x1ul << ADC_ISR_ZF_Pos)

#define ADC_ISR_NACF_Pos            (10)
#define ADC_ISR_NACF_Msk            (0x1ul << ADC_ISR_NACF_Pos)

#define ADC_ISR_INTTC_Pos           (17)
#define ADC_ISR_INTTC_Msk           (0x1ul << ADC_ISR_INTTC_Pos)

#define ADC_WKISR_WPEDEF_Pos        (1)
#define ADC_WKISR_WPEDEF_Msk        (0x1ul << ADC_WKISR_WPEDEF_Pos)

#define ADC_XYDATA_XDATA_Pos        (0)
#define ADC_XYDATA_XDATA_Msk        (0xFFFul << ADC_XYDATA_XDATA_Pos)

#define ADC_XYDATA_YDATA_Pos        (16)
#define ADC_XYDATA_YDATA_Msk        (0xFFFul << ADC_XYDATA_YDATA_Pos)

#define ADC_ZDATA_Z1DATA_Pos        (0)
#define ADC_ZDATA_Z1DATA_Msk        (0xFFFul << ADC_ZDATA_Z1DATA_Pos)

#define ADC_ZDATA_Z2DATA_Pos        (16)
#define ADC_ZDATA_Z2DATA_Msk        (0xFFFul << ADC_ZDATA_Z2DATA_Pos)

#define ADC_DATA_ADCDATA_Pos        (0)
#define ADC_DATA_ADCDATA_Msk        (0xFFFul << ADC_DATA_ADCDATA_Pos)

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif  //__ADC_REG_H__


