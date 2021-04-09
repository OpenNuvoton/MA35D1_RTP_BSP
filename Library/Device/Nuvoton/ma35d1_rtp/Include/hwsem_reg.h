/**************************************************************************//**
 * @file     hwsem_reg.h
 * @brief    HWSEM register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __HWSEM_REG_H__
#define __HWSEM_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup HArdware Semaphore (HWSEM)
    Memory Mapped Structure for HWSEM
@{ */

typedef struct
{
    __IO uint32_t CTL;
    __IO uint32_t INTEN_CA35;
    __IO uint32_t INTEN_CM4;
    __IO uint32_t INTSTS_CA35;
    __IO uint32_t INTSTS_CM4;
    __I  uint32_t reserved[3];
    __IO uint32_t SEM[8];
} HWSEM_T;

/**
    @addtogroup HWSEM_CONST HWSEM Bit Field Definition
    Constant Definitions for HWSEM
@{ */

#define HWSEM_CTL_SEM0RST_Pos                (0)
#define HWSEM_CTL_SEM0RST_Msk                (0x1ul << HWSEM_CTL_SEM0RST_Pos)

#define HWSEM_CTL_SEM1RST_Pos                (1)
#define HWSEM_CTL_SEM1RST_Msk                (0x1ul << HWSEM_CTL_SEM1RST_Pos)

#define HWSEM_CTL_SEM2RST_Pos                (2)
#define HWSEM_CTL_SEM2RST_Msk                (0x1ul << HWSEM_CTL_SEM2RST_Pos)

#define HWSEM_CTL_SEM3RST_Pos                (3)
#define HWSEM_CTL_SEM3RST_Msk                (0x1ul << HWSEM_CTL_SEM3RST_Pos)

#define HWSEM_CTL_SEM4RST_Pos                (4)
#define HWSEM_CTL_SEM4RST_Msk                (0x1ul << HWSEM_CTL_SEM4RST_Pos)

#define HWSEM_CTL_SEM5RST_Pos                (5)
#define HWSEM_CTL_SEM5RST_Msk                (0x1ul << HWSEM_CTL_SEM5RST_Pos)

#define HWSEM_CTL_SEM6RST_Pos                (6)
#define HWSEM_CTL_SEM6RST_Msk                (0x1ul << HWSEM_CTL_SEM6RST_Pos)

#define HWSEM_CTL_SEM7RST_Pos                (7)
#define HWSEM_CTL_SEM7RST_Msk                (0x1ul << HWSEM_CTL_SEM7RST_Pos)

#define HWSEM_INTEN_SEM0IEN_Pos              (0)
#define HWSEM_INTEN_SEM0IEN_Msk              (0x1ul << HWSEM_INTEN_SEM0IEN_Pos)

#define HWSEM_INTEN_SEM1IEN_Pos              (1)
#define HWSEM_INTEN_SEM1IEN_Msk              (0x1ul << HWSEM_INTEN_SEM1IEN_Pos)

#define HWSEM_INTEN_SEM2IEN_Pos              (2)
#define HWSEM_INTEN_SEM2IEN_Msk              (0x1ul << HWSEM_INTEN_SEM2IEN_Pos)

#define HWSEM_INTEN_SEM3IEN_Pos              (3)
#define HWSEM_INTEN_SEM3IEN_Msk              (0x1ul << HWSEM_INTEN_SEM3IEN_Pos)

#define HWSEM_INTEN_SEM4IEN_Pos              (4)
#define HWSEM_INTEN_SEM4IEN_Msk              (0x1ul << HWSEM_INTEN_SEM4IEN_Pos)

#define HWSEM_INTEN_SEM5IEN_Pos              (5)
#define HWSEM_INTEN_SEM5IEN_Msk              (0x1ul << HWSEM_INTEN_SEM5IEN_Pos)

#define HWSEM_INTEN_SEM6IEN_Pos              (6)
#define HWSEM_INTEN_SEM6IEN_Msk              (0x1ul << HWSEM_INTEN_SEM6IEN_Pos)

#define HWSEM_INTEN_SEM7IEN_Pos              (7)
#define HWSEM_INTEN_SEM7IEN_Msk              (0x1ul << HWSEM_INTEN_SEM7IEN_Pos)

#define HWSEM_INTSTS_SEM0IF_Pos              (0)
#define HWSEM_INTSTS_SEM0IF_Msk              (0x1ul << HWSEM_INTSTS_SEM0IF_Pos)

#define HWSEM_INTSTS_SEM1IF_Pos              (1)
#define HWSEM_INTSTS_SEM1IF_Msk              (0x1ul << HWSEM_INTSTS_SEM1IF_Pos)

#define HWSEM_INTSTS_SEM2IF_Pos              (2)
#define HWSEM_INTSTS_SEM2IF_Msk              (0x1ul << HWSEM_INTSTS_SEM2IF_Pos)

#define HWSEM_INTSTS_SEM3IF_Pos              (3)
#define HWSEM_INTSTS_SEM3IF_Msk              (0x1ul << HWSEM_INTSTS_SEM3IF_Pos)

#define HWSEM_INTSTS_SEM4IF_Pos              (4)
#define HWSEM_INTSTS_SEM4IF_Msk              (0x1ul << HWSEM_INTSTS_SEM4IF_Pos)

#define HWSEM_INTSTS_SEM5IF_Pos              (5)
#define HWSEM_INTSTS_SEM5IF_Msk              (0x1ul << HWSEM_INTSTS_SEM5IF_Pos)

#define HWSEM_INTSTS_SEM6IF_Pos              (6)
#define HWSEM_INTSTS_SEM6IF_Msk              (0x1ul << HWSEM_INTSTS_SEM6IF_Pos)

#define HWSEM_INTSTS_SEM7IF_Pos              (7)
#define HWSEM_INTSTS_SEM7IF_Msk              (0x1ul << HWSEM_INTSTS_SEM7IF_Pos)

#define HWSEM_SEM_ID_Pos                     (0)
#define HWSEM_SEM_ID_Msk                     (0xful << HWSEM_SEM_ID_Pos)

#define HWSEM_SEM_KEY_Pos                    (8)
#define HWSEM_SEM_KEY_Msk                    (0xfful << HWSEM_SEM_KEY_Pos)

/**@}*/ /* HWSEM_CONST */
/**@}*/ /* end of HWSEM register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __HWSEM_REG_H__ */
