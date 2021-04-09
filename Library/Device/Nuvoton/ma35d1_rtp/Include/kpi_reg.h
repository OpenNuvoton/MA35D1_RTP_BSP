/**************************************************************************//**
 * @file     kpi_reg.h
 * @version  V1.00
 * @brief    KPI register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 20 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __KPI_REG_H__
#define __KPI_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup Keypad Interface (KPI)
    Memory Mapped Structure for KPI
@{ */



typedef struct
{
    __IO uint32_t KPICONF;               /*!< [0x0000] Keypad Configuration Register                      */
    __IO uint32_t KPI3KCONF;             /*!< [0x0004] Keypad 3-keys Configuration Register               */
    __IO uint32_t KPISTATUS;             /*!< [0x0008] Keypad Status Register                             */
    __IO uint32_t KPIRSTC;               /*!< [0x000c] Keypad Reset Period Control Register               */
    __IO uint32_t KPIKEST0;              /*!< [0x0010] Keypad State Register 0                            */
    __IO uint32_t KPIKEST1;              /*!< [0x0014] Keypad State Register 1                            */
    __IO uint32_t KPIKPE0;               /*!< [0x0018] Lower 32 Key Press Event Indicator                 */
    __IO uint32_t KPIKPE1;               /*!< [0x001c] Upper 32 Key Press Event Indicator                 */
    __IO uint32_t KPIKRE0;               /*!< [0x0020] Lower 32 Key Release Event Indicator               */
    __IO uint32_t KPIKRE1;               /*!< [0x0024] Upper 32 Key Release Event Indicator               */
    __IO uint32_t KPIPRESCALDIV;         /*!< [0x0028] Pre-scale Divider                                  */
} KPI_T;

/**
    @addtogroup KPI_CONST KPI Bit Field Definition
    Constant Definitions for KPI
@{ */

#define KPI_KPICONF_ENKP_Pos                 (0)                                               /*!< KPI_T::KPICONF: ENKP Position              */
#define KPI_KPICONF_ENKP_Msk                 (0x1ul << KPI_KPICONF_ENKP_Pos)                   /*!< KPI_T::KPICONF: ENKP Mask                  */

#define KPI_KPICONF_PKINTEN_Pos              (1)                                               /*!< KPI_T::KPICONF: PKINTEN Position           */
#define KPI_KPICONF_PKINTEN_Msk              (0x1ul << KPI_KPICONF_PKINTEN_Pos)                /*!< KPI_T::KPICONF: PKINTEN Mask               */

#define KPI_KPICONF_RKINTEN_Pos              (2)                                               /*!< KPI_T::KPICONF: RKINTEN Position           */
#define KPI_KPICONF_RKINTEN_Msk              (0x1ul << KPI_KPICONF_RKINTEN_Pos)                /*!< KPI_T::KPICONF: RKINTEN Mask               */

#define KPI_KPICONF_INTEN_Pos                (3)                                               /*!< KPI_T::KPICONF: INTEN Position             */
#define KPI_KPICONF_INTEN_Msk                (0x1ul << KPI_KPICONF_INTEN_Pos)                  /*!< KPI_T::KPICONF: INTEN Mask                 */

#define KPI_KPICONF_WAKEUP_Pos               (5)                                               /*!< KPI_T::KPICONF: WAKEUP Position            */
#define KPI_KPICONF_WAKEUP_Msk               (0x1ul << KPI_KPICONF_WAKEUP_Pos)                 /*!< KPI_T::KPICONF: WAKEUP Mask                */

#define KPI_KPICONF_PRESCALE_Pos             (8)                                               /*!< KPI_T::KPICONF: PRESCALE Position          */
#define KPI_KPICONF_PRESCALE_Msk             (0xfful << KPI_KPICONF_PRESCALE_Pos)              /*!< KPI_T::KPICONF: PRESCALE Mask              */

#define KPI_KPICONF_DBCLKSEL_Pos             (16)                                              /*!< KPI_T::KPICONF: DBCLKSEL Position          */
#define KPI_KPICONF_DBCLKSEL_Msk             (0xful << KPI_KPICONF_DBCLKSEL_Pos)               /*!< KPI_T::KPICONF: DBCLKSEL Mask              */

#define KPI_KPICONF_SCANROWD_Pos             (22)                                              /*!< KPI_T::KPICONF: SCANROWD Position          */
#define KPI_KPICONF_SCANROWD_Msk             (0x3ul << KPI_KPICONF_SCANROWD_Pos)               /*!< KPI_T::KPICONF: SCANROWD Mask              */

#define KPI_KPICONF_KCOL_Pos                 (24)                                              /*!< KPI_T::KPICONF: KCOL Position              */
#define KPI_KPICONF_KCOL_Msk                 (0x7ul << KPI_KPICONF_KCOL_Pos)                   /*!< KPI_T::KPICONF: KCOL Mask                  */

#define KPI_KPICONF_KROW_Pos                 (28)                                              /*!< KPI_T::KPICONF: KROW Position              */
#define KPI_KPICONF_KROW_Msk                 (0x7ul << KPI_KPICONF_KROW_Pos)                   /*!< KPI_T::KPICONF: KROW Mask                  */

#define KPI_KPI3KCONF_K30C_Pos               (0)                                               /*!< KPI_T::KPI3KCONF: K30C Position            */
#define KPI_KPI3KCONF_K30C_Msk               (0x7ul << KPI_KPI3KCONF_K30C_Pos)                 /*!< KPI_T::KPI3KCONF: K30C Mask                */

#define KPI_KPI3KCONF_K30R_Pos               (3)                                               /*!< KPI_T::KPI3KCONF: K30R Position            */
#define KPI_KPI3KCONF_K30R_Msk               (0x7ul << KPI_KPI3KCONF_K30R_Pos)                 /*!< KPI_T::KPI3KCONF: K30R Mask                */

#define KPI_KPI3KCONF_K31C_Pos               (8)                                               /*!< KPI_T::KPI3KCONF: K31C Position            */
#define KPI_KPI3KCONF_K31C_Msk               (0x7ul << KPI_KPI3KCONF_K31C_Pos)                 /*!< KPI_T::KPI3KCONF: K31C Mask                */

#define KPI_KPI3KCONF_K31R_Pos               (11)                                              /*!< KPI_T::KPI3KCONF: K31R Position            */
#define KPI_KPI3KCONF_K31R_Msk               (0x7ul << KPI_KPI3KCONF_K31R_Pos)                 /*!< KPI_T::KPI3KCONF: K31R Mask                */

#define KPI_KPI3KCONF_K32C_Pos               (16)                                              /*!< KPI_T::KPI3KCONF: K32C Position            */
#define KPI_KPI3KCONF_K32C_Msk               (0x7ul << KPI_KPI3KCONF_K32C_Pos)                 /*!< KPI_T::KPI3KCONF: K32C Mask                */

#define KPI_KPI3KCONF_K32R_Pos               (19)                                              /*!< KPI_T::KPI3KCONF: K32R Position            */
#define KPI_KPI3KCONF_K32R_Msk               (0x7ul << KPI_KPI3KCONF_K32R_Pos)                 /*!< KPI_T::KPI3KCONF: K32R Mask                */

#define KPI_KPI3KCONF_EN3KYRST_Pos           (24)                                              /*!< KPI_T::KPI3KCONF: EN3KYRST Position        */
#define KPI_KPI3KCONF_EN3KYRST_Msk           (0x1ul << KPI_KPI3KCONF_EN3KYRST_Pos)             /*!< KPI_T::KPI3KCONF: EN3KYRST Mask            */

#define KPI_KPISTATUS_PDWAKE_Pos             (0)                                               /*!< KPI_T::KPISTATUS: PDWAKE Position          */
#define KPI_KPISTATUS_PDWAKE_Msk             (0x1ul << KPI_KPISTATUS_PDWAKE_Pos)               /*!< KPI_T::KPISTATUS: PDWAKE Mask              */

#define KPI_KPISTATUS_RST3KEY_Pos            (1)                                               /*!< KPI_T::KPISTATUS: RST3KEY Position         */
#define KPI_KPISTATUS_RST3KEY_Msk            (0x1ul << KPI_KPISTATUS_RST3KEY_Pos)              /*!< KPI_T::KPISTATUS: RST3KEY Mask             */

#define KPI_KPISTATUS_KEYINT_Pos             (2)                                               /*!< KPI_T::KPISTATUS: KEYINT Position          */
#define KPI_KPISTATUS_KEYINT_Msk             (0x1ul << KPI_KPISTATUS_KEYINT_Pos)               /*!< KPI_T::KPISTATUS: KEYINT Mask              */

#define KPI_KPISTATUS_RKEYINT_Pos            (3)                                               /*!< KPI_T::KPISTATUS: RKEYINT Position         */
#define KPI_KPISTATUS_RKEYINT_Msk            (0x1ul << KPI_KPISTATUS_RKEYINT_Pos)              /*!< KPI_T::KPISTATUS: RKEYINT Mask             */

#define KPI_KPISTATUS_PKEYINT_Pos            (4)                                               /*!< KPI_T::KPISTATUS: PKEYINT Position         */
#define KPI_KPISTATUS_PKEYINT_Msk            (0x1ul << KPI_KPISTATUS_PKEYINT_Pos)              /*!< KPI_T::KPISTATUS: PKEYINT Mask             */

#define KPI_KPIRSTC_RSTC_Pos                 (0)                                               /*!< KPI_T::KPIRSTC: RSTC Position              */
#define KPI_KPIRSTC_RSTC_Msk                 (0xfful << KPI_KPIRSTC_RSTC_Pos)                  /*!< KPI_T::KPIRSTC: RSTC Mask                  */

#define KPI_KPIKEST0_KEST00_Pos              (0)                                               /*!< KPI_T::KPIKEST0: KEST00 Position           */
#define KPI_KPIKEST0_KEST00_Msk              (0x1ul << KPI_KPIKEST0_KEST00_Pos)                /*!< KPI_T::KPIKEST0: KEST00 Mask               */

#define KPI_KPIKEST0_KEST01_Pos              (1)                                               /*!< KPI_T::KPIKEST0: KEST01 Position           */
#define KPI_KPIKEST0_KEST01_Msk              (0x1ul << KPI_KPIKEST0_KEST01_Pos)                /*!< KPI_T::KPIKEST0: KEST01 Mask               */

#define KPI_KPIKEST0_KEST02_Pos              (2)                                               /*!< KPI_T::KPIKEST0: KEST02 Position           */
#define KPI_KPIKEST0_KEST02_Msk              (0x1ul << KPI_KPIKEST0_KEST02_Pos)                /*!< KPI_T::KPIKEST0: KEST02 Mask               */

#define KPI_KPIKEST0_KEST03_Pos              (3)                                               /*!< KPI_T::KPIKEST0: KEST03 Position           */
#define KPI_KPIKEST0_KEST03_Msk              (0x1ul << KPI_KPIKEST0_KEST03_Pos)                /*!< KPI_T::KPIKEST0: KEST03 Mask               */

#define KPI_KPIKEST0_KEST04_Pos              (4)                                               /*!< KPI_T::KPIKEST0: KEST04 Position           */
#define KPI_KPIKEST0_KEST04_Msk              (0x1ul << KPI_KPIKEST0_KEST04_Pos)                /*!< KPI_T::KPIKEST0: KEST04 Mask               */

#define KPI_KPIKEST0_KEST05_Pos              (5)                                               /*!< KPI_T::KPIKEST0: KEST05 Position           */
#define KPI_KPIKEST0_KEST05_Msk              (0x1ul << KPI_KPIKEST0_KEST05_Pos)                /*!< KPI_T::KPIKEST0: KEST05 Mask               */

#define KPI_KPIKEST0_KEST06_Pos              (6)                                               /*!< KPI_T::KPIKEST0: KEST06 Position           */
#define KPI_KPIKEST0_KEST06_Msk              (0x1ul << KPI_KPIKEST0_KEST06_Pos)                /*!< KPI_T::KPIKEST0: KEST06 Mask               */

#define KPI_KPIKEST0_KEST07_Pos              (7)                                               /*!< KPI_T::KPIKEST0: KEST07 Position           */
#define KPI_KPIKEST0_KEST07_Msk              (0x1ul << KPI_KPIKEST0_KEST07_Pos)                /*!< KPI_T::KPIKEST0: KEST07 Mask               */

#define KPI_KPIKEST0_KEST10_Pos              (8)                                               /*!< KPI_T::KPIKEST0: KEST10 Position           */
#define KPI_KPIKEST0_KEST10_Msk              (0x1ul << KPI_KPIKEST0_KEST10_Pos)                /*!< KPI_T::KPIKEST0: KEST10 Mask               */

#define KPI_KPIKEST0_KEST11_Pos              (9)                                               /*!< KPI_T::KPIKEST0: KEST11 Position           */
#define KPI_KPIKEST0_KEST11_Msk              (0x1ul << KPI_KPIKEST0_KEST11_Pos)                /*!< KPI_T::KPIKEST0: KEST11 Mask               */

#define KPI_KPIKEST0_KEST12_Pos              (10)                                              /*!< KPI_T::KPIKEST0: KEST12 Position           */
#define KPI_KPIKEST0_KEST12_Msk              (0x1ul << KPI_KPIKEST0_KEST12_Pos)                /*!< KPI_T::KPIKEST0: KEST12 Mask               */

#define KPI_KPIKEST0_KEST13_Pos              (11)                                              /*!< KPI_T::KPIKEST0: KEST13 Position           */
#define KPI_KPIKEST0_KEST13_Msk              (0x1ul << KPI_KPIKEST0_KEST13_Pos)                /*!< KPI_T::KPIKEST0: KEST13 Mask               */

#define KPI_KPIKEST0_KEST14_Pos              (12)                                              /*!< KPI_T::KPIKEST0: KEST14 Position           */
#define KPI_KPIKEST0_KEST14_Msk              (0x1ul << KPI_KPIKEST0_KEST14_Pos)                /*!< KPI_T::KPIKEST0: KEST14 Mask               */

#define KPI_KPIKEST0_KEST15_Pos              (13)                                              /*!< KPI_T::KPIKEST0: KEST15 Position           */
#define KPI_KPIKEST0_KEST15_Msk              (0x1ul << KPI_KPIKEST0_KEST15_Pos)                /*!< KPI_T::KPIKEST0: KEST15 Mask               */

#define KPI_KPIKEST0_KEST16_Pos              (14)                                              /*!< KPI_T::KPIKEST0: KEST16 Position           */
#define KPI_KPIKEST0_KEST16_Msk              (0x1ul << KPI_KPIKEST0_KEST16_Pos)                /*!< KPI_T::KPIKEST0: KEST16 Mask               */

#define KPI_KPIKEST0_KEST17_Pos              (15)                                              /*!< KPI_T::KPIKEST0: KEST17 Position           */
#define KPI_KPIKEST0_KEST17_Msk              (0x1ul << KPI_KPIKEST0_KEST17_Pos)                /*!< KPI_T::KPIKEST0: KEST17 Mask               */

#define KPI_KPIKEST0_KEST20_Pos              (16)                                              /*!< KPI_T::KPIKEST0: KEST20 Position           */
#define KPI_KPIKEST0_KEST20_Msk              (0x1ul << KPI_KPIKEST0_KEST20_Pos)                /*!< KPI_T::KPIKEST0: KEST20 Mask               */

#define KPI_KPIKEST0_KEST21_Pos              (17)                                              /*!< KPI_T::KPIKEST0: KEST21 Position           */
#define KPI_KPIKEST0_KEST21_Msk              (0x1ul << KPI_KPIKEST0_KEST21_Pos)                /*!< KPI_T::KPIKEST0: KEST21 Mask               */

#define KPI_KPIKEST0_KEST22_Pos              (18)                                              /*!< KPI_T::KPIKEST0: KEST22 Position           */
#define KPI_KPIKEST0_KEST22_Msk              (0x1ul << KPI_KPIKEST0_KEST22_Pos)                /*!< KPI_T::KPIKEST0: KEST22 Mask               */

#define KPI_KPIKEST0_KEST23_Pos              (19)                                              /*!< KPI_T::KPIKEST0: KEST23 Position           */
#define KPI_KPIKEST0_KEST23_Msk              (0x1ul << KPI_KPIKEST0_KEST23_Pos)                /*!< KPI_T::KPIKEST0: KEST23 Mask               */

#define KPI_KPIKEST0_KEST24_Pos              (20)                                              /*!< KPI_T::KPIKEST0: KEST24 Position           */
#define KPI_KPIKEST0_KEST24_Msk              (0x1ul << KPI_KPIKEST0_KEST24_Pos)                /*!< KPI_T::KPIKEST0: KEST24 Mask               */

#define KPI_KPIKEST0_KEST25_Pos              (21)                                              /*!< KPI_T::KPIKEST0: KEST25 Position           */
#define KPI_KPIKEST0_KEST25_Msk              (0x1ul << KPI_KPIKEST0_KEST25_Pos)                /*!< KPI_T::KPIKEST0: KEST25 Mask               */

#define KPI_KPIKEST0_KEST26_Pos              (22)                                              /*!< KPI_T::KPIKEST0: KEST26 Position           */
#define KPI_KPIKEST0_KEST26_Msk              (0x1ul << KPI_KPIKEST0_KEST26_Pos)                /*!< KPI_T::KPIKEST0: KEST26 Mask               */

#define KPI_KPIKEST0_KEST27_Pos              (23)                                              /*!< KPI_T::KPIKEST0: KEST27 Position           */
#define KPI_KPIKEST0_KEST27_Msk              (0x1ul << KPI_KPIKEST0_KEST27_Pos)                /*!< KPI_T::KPIKEST0: KEST27 Mask               */

#define KPI_KPIKEST0_KEST30_Pos              (24)                                              /*!< KPI_T::KPIKEST0: KEST30 Position           */
#define KPI_KPIKEST0_KEST30_Msk              (0x1ul << KPI_KPIKEST0_KEST30_Pos)                /*!< KPI_T::KPIKEST0: KEST30 Mask               */

#define KPI_KPIKEST0_KEST31_Pos              (25)                                              /*!< KPI_T::KPIKEST0: KEST31 Position           */
#define KPI_KPIKEST0_KEST31_Msk              (0x1ul << KPI_KPIKEST0_KEST31_Pos)                /*!< KPI_T::KPIKEST0: KEST31 Mask               */

#define KPI_KPIKEST0_KEST32_Pos              (26)                                              /*!< KPI_T::KPIKEST0: KEST32 Position           */
#define KPI_KPIKEST0_KEST32_Msk              (0x1ul << KPI_KPIKEST0_KEST32_Pos)                /*!< KPI_T::KPIKEST0: KEST32 Mask               */

#define KPI_KPIKEST0_KEST33_Pos              (27)                                              /*!< KPI_T::KPIKEST0: KEST33 Position           */
#define KPI_KPIKEST0_KEST33_Msk              (0x1ul << KPI_KPIKEST0_KEST33_Pos)                /*!< KPI_T::KPIKEST0: KEST33 Mask               */

#define KPI_KPIKEST0_KEST34_Pos              (28)                                              /*!< KPI_T::KPIKEST0: KEST34 Position           */
#define KPI_KPIKEST0_KEST34_Msk              (0x1ul << KPI_KPIKEST0_KEST34_Pos)                /*!< KPI_T::KPIKEST0: KEST34 Mask               */

#define KPI_KPIKEST0_KEST35_Pos              (29)                                              /*!< KPI_T::KPIKEST0: KEST35 Position           */
#define KPI_KPIKEST0_KEST35_Msk              (0x1ul << KPI_KPIKEST0_KEST35_Pos)                /*!< KPI_T::KPIKEST0: KEST35 Mask               */

#define KPI_KPIKEST0_KEST36_Pos              (30)                                              /*!< KPI_T::KPIKEST0: KEST36 Position           */
#define KPI_KPIKEST0_KEST36_Msk              (0x1ul << KPI_KPIKEST0_KEST36_Pos)                /*!< KPI_T::KPIKEST0: KEST36 Mask               */

#define KPI_KPIKEST1_KEST37_Pos              (31)                                              /*!< KPI_T::KPIKEST0: KEST37 Position           */
#define KPI_KPIKEST1_KEST37_Msk              (0x1ul << KPI_KPIKEST0_KEST37_Pos)                /*!< KPI_T::KPIKEST0: KEST37 Mask               */

#define KPI_KPIKEST1_KEST40_Pos              (0)                                               /*!< KPI_T::KPIKEST0: KEST00 Position           */
#define KPI_KPIKEST1_KEST40_Msk              (0x1ul << KPI_KPIKEST1_KEST40_Pos)                /*!< KPI_T::KPIKEST0: KEST00 Mask               */

#define KPI_KPIKEST1_KEST41_Pos              (1)                                               /*!< KPI_T::KPIKEST1: KEST41 Position           */
#define KPI_KPIKEST1_KEST41_Msk              (0x1ul << KPI_KPIKEST1_KEST41_Pos)                /*!< KPI_T::KPIKEST1: KEST41 Mask               */

#define KPI_KPIKEST1_KEST42_Pos              (2)                                               /*!< KPI_T::KPIKEST1: KEST42 Position           */
#define KPI_KPIKEST1_KEST42_Msk              (0x1ul << KPI_KPIKEST1_KEST42_Pos)                /*!< KPI_T::KPIKEST1: KEST42 Mask               */

#define KPI_KPIKEST1_KEST43_Pos              (3)                                               /*!< KPI_T::KPIKEST1: KEST43 Position           */
#define KPI_KPIKEST1_KEST43_Msk              (0x1ul << KPI_KPIKEST1_KEST43_Pos)                /*!< KPI_T::KPIKEST1: KEST43 Mask               */

#define KPI_KPIKEST1_KEST44_Pos              (4)                                               /*!< KPI_T::KPIKEST1: KEST44 Position           */
#define KPI_KPIKEST1_KEST44_Msk              (0x1ul << KPI_KPIKEST1_KEST44_Pos)                /*!< KPI_T::KPIKEST1: KEST44 Mask               */

#define KPI_KPIKEST1_KEST45_Pos              (5)                                               /*!< KPI_T::KPIKEST1: KEST45 Position           */
#define KPI_KPIKEST1_KEST45_Msk              (0x1ul << KPI_KPIKEST1_KEST45_Pos)                /*!< KPI_T::KPIKEST1: KEST45 Mask               */

#define KPI_KPIKEST1_KEST46_Pos              (6)                                               /*!< KPI_T::KPIKEST1: KEST46 Position           */
#define KPI_KPIKEST1_KEST46_Msk              (0x1ul << KPI_KPIKEST1_KEST46_Pos)                /*!< KPI_T::KPIKEST1: KEST46 Mask               */

#define KPI_KPIKEST1_KEST47_Pos              (7)                                               /*!< KPI_T::KPIKEST1: KEST47 Position           */
#define KPI_KPIKEST1_KEST47_Msk              (0x1ul << KPI_KPIKEST1_KEST47_Pos)                /*!< KPI_T::KPIKEST1: KEST47 Mask               */

#define KPI_KPIKEST1_KEST50_Pos              (8)                                               /*!< KPI_T::KPIKEST1: KEST50 Position           */
#define KPI_KPIKEST1_KEST50_Msk              (0x1ul << KPI_KPIKEST0_KEST50_Pos)                /*!< KPI_T::KPIKEST1: KEST50 Mask               */

#define KPI_KPIKEST1_KEST51_Pos              (9)                                               /*!< KPI_T::KPIKEST1: KEST51 Position           */
#define KPI_KPIKEST1_KEST51_Msk              (0x1ul << KPI_KPIKEST1_KEST51_Pos)                /*!< KPI_T::KPIKEST1: KEST51 Mask               */

#define KPI_KPIKEST1_KEST52_Pos              (10)                                              /*!< KPI_T::KPIKEST1: KEST52 Position           */
#define KPI_KPIKEST1_KEST52_Msk              (0x1ul << KPI_KPIKEST1_KEST52_Pos)                /*!< KPI_T::KPIKEST1: KEST52 Mask               */

#define KPI_KPIKEST1_KEST53_Pos              (11)                                              /*!< KPI_T::KPIKEST1: KEST53 Position           */
#define KPI_KPIKEST1_KEST53_Msk              (0x1ul << KPI_KPIKEST1_KEST53_Pos)                /*!< KPI_T::KPIKEST1: KEST53 Mask               */

#define KPI_KPIKEST1_KEST54_Pos              (12)                                              /*!< KPI_T::KPIKEST1: KEST54 Position           */
#define KPI_KPIKEST1_KEST54_Msk              (0x1ul << KPI_KPIKEST1_KEST54_Pos)                /*!< KPI_T::KPIKEST1: KEST54 Mask               */

#define KPI_KPIKEST1_KEST55_Pos              (13)                                              /*!< KPI_T::KPIKEST1: KEST55 Position           */
#define KPI_KPIKEST1_KEST55_Msk              (0x1ul << KPI_KPIKEST1_KEST55_Pos)                /*!< KPI_T::KPIKEST1: KEST55 Mask               */

#define KPI_KPIKEST1_KEST56_Pos              (14)                                              /*!< KPI_T::KPIKEST1: KEST56 Position           */
#define KPI_KPIKEST1_KEST56_Msk              (0x1ul << KPI_KPIKEST1_KEST56_Pos)                /*!< KPI_T::KPIKEST1: KEST56 Mask               */

#define KPI_KPIKEST1_KEST57_Pos              (15)                                              /*!< KPI_T::KPIKEST1: KEST57 Position           */
#define KPI_KPIKEST1_KEST57_Msk              (0x1ul << KPI_KPIKEST1_KEST57_Pos)                /*!< KPI_T::KPIKEST1: KEST57 Mask               */

#define KPI_KPIKPE0_KPE00_Pos              (0)                                                 /*!< KPI_T::KPIKPE0: KPE00 Position             */
#define KPI_KPIKPE0_KPE00_Msk              (0x1ul << KPI_KPIKPE0_KPE00_Pos)                    /*!< KPI_T::KPIKPE0: KPE00 Mask                 */
#define KPI_KPIKPE0_KPE01_Pos              (1)                                                 /*!< KPI_T::KPIKPE0: KPE01 Position             */
#define KPI_KPIKPE0_KPE01_Msk              (0x1ul << KPI_KPIKPE0_KPE01_Pos)                    /*!< KPI_T::KPIKPE0: KPE01 Mask                 */
#define KPI_KPIKPE0_KPE02_Pos              (2)                                                 /*!< KPI_T::KPIKPE0: KPE02 Position             */
#define KPI_KPIKPE0_KPE02_Msk              (0x1ul << KPI_KPIKPE0_KPE02_Pos)                    /*!< KPI_T::KPIKPE0: KPE02 Mask                 */
#define KPI_KPIKPE0_KPE03_Pos              (3)                                                 /*!< KPI_T::KPIKPE0: KPE03 Position             */
#define KPI_KPIKPE0_KPE03_Msk              (0x1ul << KPI_KPIKPE0_KPE03_Pos)                    /*!< KPI_T::KPIKPE0: KPE02 Mask                 */
#define KPI_KPIKPE0_KPE04_Pos              (4)                                                 /*!< KPI_T::KPIKPE0: KPE04 Position             */
#define KPI_KPIKPE0_KPE04_Msk              (0x1ul << KPI_KPIKPE0_KPE04_Pos)                    /*!< KPI_T::KPIKPE0: KPE04 Mask                 */
#define KPI_KPIKPE0_KPE05_Pos              (5)                                                 /*!< KPI_T::KPIKPE0: KPE05 Position             */
#define KPI_KPIKPE0_KPE05_Msk              (0x1ul << KPI_KPIKPE0_KPE05_Pos)                    /*!< KPI_T::KPIKPE0: KPE05 Mask                 */
#define KPI_KPIKPE0_KPE06_Pos              (6)                                                 /*!< KPI_T::KPIKPE0: KPE06 Position             */
#define KPI_KPIKPE0_KPE06_Msk              (0x1ul << KPI_KPIKPE0_KPE06_Pos)                    /*!< KPI_T::KPIKPE0: KPE06 Mask                 */
#define KPI_KPIKPE0_KPE07_Pos              (7)                                                 /*!< KPI_T::KPIKPE0: KPE07 Position             */
#define KPI_KPIKPE0_KPE07_Msk              (0x1ul << KPI_KPIKPE0_KPE07_Pos)                    /*!< KPI_T::KPIKPE0: KPE07 Mask                 */
#define KPI_KPIKPE0_KPE10_Pos              (8)                                                 /*!< KPI_T::KPIKPE0: KPE10 Position             */
#define KPI_KPIKPE0_KPE10_Msk              (0x1ul << KPI_KPIKPE0_KPE10_Pos)                    /*!< KPI_T::KPIKPE0: KPE10 Mask                 */
#define KPI_KPIKPE0_KPE11_Pos              (9)                                                 /*!< KPI_T::KPIKPE0: KPE11 Position             */
#define KPI_KPIKPE0_KPE11_Msk              (0x1ul << KPI_KPIKPE0_KPE11_Pos)                    /*!< KPI_T::KPIKPE0: KPE11 Mask                 */
#define KPI_KPIKPE0_KPE12_Pos              (10)                                                /*!< KPI_T::KPIKPE0: KPE12 Position             */
#define KPI_KPIKPE0_KPE12_Msk              (0x1ul << KPI_KPIKPE0_KPE12_Pos)                    /*!< KPI_T::KPIKPE0: KPE12 Mask                 */
#define KPI_KPIKPE0_KPE13_Pos              (11)                                                /*!< KPI_T::KPIKPE0: KPE13 Position             */
#define KPI_KPIKPE0_KPE13_Msk              (0x1ul << KPI_KPIKPE0_KPE13_Pos)                    /*!< KPI_T::KPIKPE0: KPE13 Mask                 */
#define KPI_KPIKPE0_KPE14_Pos              (12)                                                /*!< KPI_T::KPIKPE0: KPE14 Position             */
#define KPI_KPIKPE0_KPE14_Msk              (0x1ul << KPI_KPIKPE0_KPE14_Pos)                    /*!< KPI_T::KPIKPE0: KPE14 Mask                 */
#define KPI_KPIKPE0_KPE15_Pos              (13)                                                /*!< KPI_T::KPIKPE0: KPE15 Position             */
#define KPI_KPIKPE0_KPE15_Msk              (0x1ul << KPI_KPIKPE0_KPE15_Pos)                    /*!< KPI_T::KPIKPE0: KPE15 Mask                 */
#define KPI_KPIKPE0_KPE16_Pos              (14)                                                /*!< KPI_T::KPIKPE0: KPE16 Position             */
#define KPI_KPIKPE0_KPE16_Msk              (0x1ul << KPI_KPIKPE0_KPE16_Pos)                    /*!< KPI_T::KPIKPE0: KPE16 Mask                 */
#define KPI_KPIKPE0_KPE17_Pos              (15)                                                /*!< KPI_T::KPIKPE0: KPE17 Position             */
#define KPI_KPIKPE0_KPE17_Msk              (0x1ul << KPI_KPIKPE0_KPE17_Pos)                    /*!< KPI_T::KPIKPE0: KPE17 Mask                 */
#define KPI_KPIKPE0_KPE20_Pos              (16)                                                /*!< KPI_T::KPIKPE0: KPE20 Position             */
#define KPI_KPIKPE0_KPE20_Msk              (0x1ul << KPI_KPIKPE0_KPE20_Pos)                    /*!< KPI_T::KPIKPE0: KPE20 Mask                 */
#define KPI_KPIKPE0_KPE21_Pos              (17)                                                /*!< KPI_T::KPIKPE0: KPE21 Position             */
#define KPI_KPIKPE0_KPE21_Msk              (0x1ul << KPI_KPIKPE0_KPE21_Pos)                    /*!< KPI_T::KPIKPE0: KPE21 Mask                 */
#define KPI_KPIKPE0_KPE22_Pos              (18)                                                /*!< KPI_T::KPIKPE0: KPE22 Position             */
#define KPI_KPIKPE0_KPE22_Msk              (0x1ul << KPI_KPIKPE0_KPE22_Pos)                    /*!< KPI_T::KPIKPE0: KPE22 Mask                 */
#define KPI_KPIKPE0_KPE23_Pos              (19)                                                /*!< KPI_T::KPIKPE0: KPE23 Position             */
#define KPI_KPIKPE0_KPE23_Msk              (0x1ul << KPI_KPIKPE0_KPE23_Pos)                    /*!< KPI_T::KPIKPE0: KPE23 Mask                 */
#define KPI_KPIKPE0_KPE24_Pos              (20)                                                /*!< KPI_T::KPIKPE0: KPE24 Position             */
#define KPI_KPIKPE0_KPE24_Msk              (0x1ul << KPI_KPIKPE0_KPE24_Pos)                    /*!< KPI_T::KPIKPE0: KPE24 Mask                 */
#define KPI_KPIKPE0_KPE25_Pos              (21)                                                /*!< KPI_T::KPIKPE0: KPE25 Position             */
#define KPI_KPIKPE0_KPE25_Msk              (0x1ul << KPI_KPIKPE0_KPE25_Pos)                    /*!< KPI_T::KPIKPE0: KPE25 Mask                 */
#define KPI_KPIKPE0_KPE26_Pos              (22)                                                /*!< KPI_T::KPIKPE0: KPE26 Position             */
#define KPI_KPIKPE0_KPE26_Msk              (0x1ul << KPI_KPIKPE0_KPE26_Pos)                    /*!< KPI_T::KPIKPE0: KPE26 Mask                 */
#define KPI_KPIKPE0_KPE27_Pos              (23)                                                /*!< KPI_T::KPIKPE0: KPE27 Position             */
#define KPI_KPIKPE0_KPE27_Msk              (0x1ul << KPI_KPIKPE0_KPE27_Pos)                    /*!< KPI_T::KPIKPE0: KPE27 Mask                 */
#define KPI_KPIKPE0_KPE30_Pos              (24)                                                /*!< KPI_T::KPIKPE0: KPE30 Position             */
#define KPI_KPIKPE0_KPE30_Msk              (0x1ul << KPI_KPIKPE0_KPE30_Pos)                    /*!< KPI_T::KPIKPE0: KPE30 Mask                 */
#define KPI_KPIKPE0_KPE31_Pos              (25)                                                /*!< KPI_T::KPIKPE0: KPE31 Position             */
#define KPI_KPIKPE0_KPE31_Msk              (0x1ul << KPI_KPIKPE0_KPE31_Pos)                    /*!< KPI_T::KPIKPE0: KPE31 Mask                 */
#define KPI_KPIKPE0_KPE32_Pos              (26)                                                /*!< KPI_T::KPIKPE0: KPE32 Position             */
#define KPI_KPIKPE0_KPE32_Msk              (0x1ul << KPI_KPIKPE0_KPE32_Pos)                    /*!< KPI_T::KPIKPE0: KPE32 Mask                 */
#define KPI_KPIKPE0_KPE33_Pos              (27)                                                /*!< KPI_T::KPIKPE0: KPE33 Position             */
#define KPI_KPIKPE0_KPE33_Msk              (0x1ul << KPI_KPIKPE0_KPE33_Pos)                    /*!< KPI_T::KPIKPE0: KPE33 Mask                 */
#define KPI_KPIKPE0_KPE34_Pos              (28)                                                /*!< KPI_T::KPIKPE0: KPE34 Position             */
#define KPI_KPIKPE0_KPE34_Msk              (0x1ul << KPI_KPIKPE0_KPE34_Pos)                    /*!< KPI_T::KPIKPE0: KPE34 Mask                 */
#define KPI_KPIKPE0_KPE35_Pos              (29)                                                /*!< KPI_T::KPIKPE0: KPE35 Position             */
#define KPI_KPIKPE0_KPE35_Msk              (0x1ul << KPI_KPIKPE0_KPE35_Pos)                    /*!< KPI_T::KPIKPE0: KPE35 Mask                 */
#define KPI_KPIKPE0_KPE36_Pos              (30)                                                /*!< KPI_T::KPIKPE0: KPE36 Position             */
#define KPI_KPIKPE0_KPE36_Msk              (0x1ul << KPI_KPIKPE0_KPE36_Pos)                    /*!< KPI_T::KPIKPE0: KPE36 Mask                 */
#define KPI_KPIKPE0_KPE37_Pos              (31)                                                /*!< KPI_T::KPIKPE0: KPE37 Position             */
#define KPI_KPIKPE0_KPE37_Msk              (0x1ul << KPI_KPIKPE0_KPE37_Pos)                    /*!< KPI_T::KPIKPE0: KPE37 Mask                 */


#define KPI_KPIKPE1_KPE40_Pos              (0)                                                 /*!< KPI_T::KPIKPE1: KPE40 Position             */
#define KPI_KPIKPE1_KPE40_Msk              (0x1ul << KPI_KPIKPE1_KPE40_Pos)                    /*!< KPI_T::KPIKPE1: KPE40 Mask                 */
#define KPI_KPIKPE1_KPE41_Pos              (1)                 1                               /*!< KPI_T::KPIKPE1: KPE41 Position             */
#define KPI_KPIKPE1_KPE41_Msk              (0x1ul << KPI_KPIKPE1_KPE41_Pos)                    /*!< KPI_T::KPIKPE1: KPE41 Mask                 */
#define KPI_KPIKPE1_KPE42_Pos              (2)                 1                               /*!< KPI_T::KPIKPE1: KPE42 Position             */
#define KPI_KPIKPE1_KPE42_Msk              (0x1ul << KPI_KPIKPE1_KPE42_Pos)                    /*!< KPI_T::KPIKPE1: KPE42 Mask                 */
#define KPI_KPIKPE1_KPE43_Pos              (3)                 1                               /*!< KPI_T::KPIKPE1: KPE43 Position             */
#define KPI_KPIKPE1_KPE43_Msk              (0x1ul << KPI_KPIKPE1_KPE43_Pos)                    /*!< KPI_T::KPIKPE1: KPE42 Mask                 */
#define KPI_KPIKPE1_KPE44_Pos              (4)                 1                               /*!< KPI_T::KPIKPE1: KPE44 Position             */
#define KPI_KPIKPE1_KPE44_Msk              (0x1ul << KPI_KPIKPE1_KPE44_Pos)                    /*!< KPI_T::KPIKPE1: KPE44 Mask                 */
#define KPI_KPIKPE1_KPE45_Pos              (5)                 1                               /*!< KPI_T::KPIKPE1: KPE45 Position             */
#define KPI_KPIKPE1_KPE45_Msk              (0x1ul << KPI_KPIKPE1_KPE45_Pos)                    /*!< KPI_T::KPIKPE1: KPE45 Mask                 */
#define KPI_KPIKPE1_KPE46_Pos              (6)                 1                               /*!< KPI_T::KPIKPE1: KPE46 Position             */
#define KPI_KPIKPE1_KPE46_Msk              (0x1ul << KPI_KPIKPE1_KPE46_Pos)                    /*!< KPI_T::KPIKPE1: KPE46 Mask                 */
#define KPI_KPIKPE1_KPE47_Pos              (7)                 1                               /*!< KPI_T::KPIKPE1: KPE47 Position             */
#define KPI_KPIKPE1_KPE47_Msk              (0x1ul << KPI_KPIKPE1_KPE47_Pos)                    /*!< KPI_T::KPIKPE1: KPE47 Mask                 */
#define KPI_KPIKPE1_KPE50_Pos              (8)                 1                               /*!< KPI_T::KPIKPE1: KPE50 Position             */
#define KPI_KPIKPE1_KPE50_Msk              (0x1ul << KPI_KPIKPE1_KPE50_Pos)                    /*!< KPI_T::KPIKPE1: KPE50 Mask                 */
#define KPI_KPIKPE1_KPE51_Pos              (9)                 1                               /*!< KPI_T::KPIKPE1: KPE51 Position             */
#define KPI_KPIKPE1_KPE51_Msk              (0x1ul << KPI_KPIKPE1_KPE51_Pos)                    /*!< KPI_T::KPIKPE1: KPE51 Mask                 */
#define KPI_KPIKPE1_KPE52_Pos              (10)                1                               /*!< KPI_T::KPIKPE1: KPE52 Position             */
#define KPI_KPIKPE1_KPE52_Msk              (0x1ul << KPI_KPIKPE1_KPE52_Pos)                    /*!< KPI_T::KPIKPE1: KPE52 Mask                 */
#define KPI_KPIKPE1_KPE53_Pos              (11)                1                               /*!< KPI_T::KPIKPE1: KPE53 Position             */
#define KPI_KPIKPE1_KPE53_Msk              (0x1ul << KPI_KPIKPE1_KPE53_Pos)                    /*!< KPI_T::KPIKPE1: KPE53 Mask                 */
#define KPI_KPIKPE1_KPE54_Pos              (12)                1                               /*!< KPI_T::KPIKPE1: KPE54 Position             */
#define KPI_KPIKPE1_KPE54_Msk              (0x1ul << KPI_KPIKPE1_KPE54_Pos)                    /*!< KPI_T::KPIKPE1: KPE54 Mask                 */
#define KPI_KPIKPE1_KPE55_Pos              (13)                1                               /*!< KPI_T::KPIKPE1: KPE55 Position             */
#define KPI_KPIKPE1_KPE55_Msk              (0x1ul << KPI_KPIKPE1_KPE55_Pos)                    /*!< KPI_T::KPIKPE1: KPE55 Mask                 */
#define KPI_KPIKPE1_KPE56_Pos              (14)                1                               /*!< KPI_T::KPIKPE1: KPE56 Position             */
#define KPI_KPIKPE1_KPE56_Msk              (0x1ul << KPI_KPIKPE1_KPE56_Pos)                    /*!< KPI_T::KPIKPE1: KPE56 Mask                 */
#define KPI_KPIKPE1_KPE57_Pos              (15)                                                /*!< KPI_T::KPIKPE1: KPE57 Position             */
#define KPI_KPIKPE1_KPE57_Msk              (0x1ul << KPI_KPIKPE1_KPE57_Pos)                    /*!< KPI_T::KPIKPE1: KPE57 Mask                 */

#define KPI_KPIKRE0_KRE00_Pos              (0)                                                 /*!< KPI_T::KPIKRE0: KRE00 Position             */
#define KPI_KPIKRE0_KRE00_Msk              (0x1ul << KPI_KPIKRE0_KRE00_Pos)                    /*!< KPI_T::KPIKRE0: KRE00 Mask                 */
#define KPI_KPIKRE0_KRE01_Pos              (1)                                                 /*!< KPI_T::KPIKRE0: KRE01 Position             */
#define KPI_KPIKRE0_KRE01_Msk              (0x1ul << KPI_KPIKRE0_KRE01_Pos)                    /*!< KPI_T::KPIKRE0: KRE01 Mask                 */
#define KPI_KPIKRE0_KRE02_Pos              (2)                                                 /*!< KPI_T::KPIKRE0: KRE02 Position             */
#define KPI_KPIKRE0_KRE02_Msk              (0x1ul << KPI_KPIKRE0_KRE02_Pos)                    /*!< KPI_T::KPIKRE0: KRE02 Mask                 */
#define KPI_KPIKRE0_KRE03_Pos              (3)                                                 /*!< KPI_T::KPIKRE0: KRE03 Position             */
#define KPI_KPIKRE0_KRE03_Msk              (0x1ul << KPI_KPIKRE0_KRE03_Pos)                    /*!< KPI_T::KPIKRE0: KRE02 Mask                 */
#define KPI_KPIKRE0_KRE04_Pos              (4)                                                 /*!< KPI_T::KPIKRE0: KRE04 Position             */
#define KPI_KPIKRE0_KRE04_Msk              (0x1ul << KPI_KPIKRE0_KRE04_Pos)                    /*!< KPI_T::KPIKRE0: KRE04 Mask                 */
#define KPI_KPIKRE0_KRE05_Pos              (5)                                                 /*!< KPI_T::KPIKRE0: KRE05 Position             */
#define KPI_KPIKRE0_KRE05_Msk              (0x1ul << KPI_KPIKRE0_KRE05_Pos)                    /*!< KPI_T::KPIKRE0: KRE05 Mask                 */
#define KPI_KPIKRE0_KRE06_Pos              (6)                                                 /*!< KPI_T::KPIKRE0: KRE06 Position             */
#define KPI_KPIKRE0_KRE06_Msk              (0x1ul << KPI_KPIKRE0_KRE06_Pos)                    /*!< KPI_T::KPIKRE0: KRE06 Mask                 */
#define KPI_KPIKRE0_KRE07_Pos              (7)                                                 /*!< KPI_T::KPIKRE0: KRE07 Position             */
#define KPI_KPIKRE0_KRE07_Msk              (0x1ul << KPI_KPIKRE0_KRE07_Pos)                    /*!< KPI_T::KPIKRE0: KRE07 Mask                 */
#define KPI_KPIKRE0_KRE10_Pos              (8)                                                 /*!< KPI_T::KPIKRE0: KRE10 Position             */
#define KPI_KPIKRE0_KRE10_Msk              (0x1ul << KPI_KPIKRE0_KRE10_Pos)                    /*!< KPI_T::KPIKRE0: KRE10 Mask                 */
#define KPI_KPIKRE0_KRE11_Pos              (9)                                                 /*!< KPI_T::KPIKRE0: KRE11 Position             */
#define KPI_KPIKRE0_KRE11_Msk              (0x1ul << KPI_KPIKRE0_KRE11_Pos)                    /*!< KPI_T::KPIKRE0: KRE11 Mask                 */
#define KPI_KPIKRE0_KRE12_Pos              (10)                                                /*!< KPI_T::KPIKRE0: KRE12 Position             */
#define KPI_KPIKRE0_KRE12_Msk              (0x1ul << KPI_KPIKRE0_KRE12_Pos)                    /*!< KPI_T::KPIKRE0: KRE12 Mask                 */
#define KPI_KPIKRE0_KRE13_Pos              (11)                                                /*!< KPI_T::KPIKRE0: KRE13 Position             */
#define KPI_KPIKRE0_KRE13_Msk              (0x1ul << KPI_KPIKRE0_KRE13_Pos)                    /*!< KPI_T::KPIKRE0: KRE13 Mask                 */
#define KPI_KPIKRE0_KRE14_Pos              (12)                                                /*!< KPI_T::KPIKRE0: KRE14 Position             */
#define KPI_KPIKRE0_KRE14_Msk              (0x1ul << KPI_KPIKRE0_KRE14_Pos)                    /*!< KPI_T::KPIKRE0: KRE14 Mask                 */
#define KPI_KPIKRE0_KRE15_Pos              (13)                                                /*!< KPI_T::KPIKRE0: KRE15 Position             */
#define KPI_KPIKRE0_KRE15_Msk              (0x1ul << KPI_KPIKRE0_KRE15_Pos)                    /*!< KPI_T::KPIKRE0: KRE15 Mask                 */
#define KPI_KPIKRE0_KRE16_Pos              (14)                                                /*!< KPI_T::KPIKRE0: KRE16 Position             */
#define KPI_KPIKRE0_KRE16_Msk              (0x1ul << KPI_KPIKRE0_KRE16_Pos)                    /*!< KPI_T::KPIKRE0: KRE16 Mask                 */
#define KPI_KPIKRE0_KRE17_Pos              (15)                                                /*!< KPI_T::KPIKRE0: KRE17 Position             */
#define KPI_KPIKRE0_KRE17_Msk              (0x1ul << KPI_KPIKRE0_KRE17_Pos)                    /*!< KPI_T::KPIKRE0: KRE17 Mask                 */
#define KPI_KPIKRE0_KRE20_Pos              (16)                                                /*!< KPI_T::KPIKRE0: KRE20 Position             */
#define KPI_KPIKRE0_KRE20_Msk              (0x1ul << KPI_KPIKRE0_KRE20_Pos)                    /*!< KPI_T::KPIKRE0: KRE20 Mask                 */
#define KPI_KPIKRE0_KRE21_Pos              (17)                                                /*!< KPI_T::KPIKRE0: KRE21 Position             */
#define KPI_KPIKRE0_KRE21_Msk              (0x1ul << KPI_KPIKRE0_KRE21_Pos)                    /*!< KPI_T::KPIKRE0: KRE21 Mask                 */
#define KPI_KPIKRE0_KRE22_Pos              (18)                                                /*!< KPI_T::KPIKRE0: KRE22 Position             */
#define KPI_KPIKRE0_KRE22_Msk              (0x1ul << KPI_KPIKRE0_KRE22_Pos)                    /*!< KPI_T::KPIKRE0: KRE22 Mask                 */
#define KPI_KPIKRE0_KRE23_Pos              (19)                                                /*!< KPI_T::KPIKRE0: KRE23 Position             */
#define KPI_KPIKRE0_KRE23_Msk              (0x1ul << KPI_KPIKRE0_KRE23_Pos)                    /*!< KPI_T::KPIKRE0: KRE23 Mask                 */
#define KPI_KPIKRE0_KRE24_Pos              (20)                                                /*!< KPI_T::KPIKRE0: KRE24 Position             */
#define KPI_KPIKRE0_KRE24_Msk              (0x1ul << KPI_KPIKRE0_KRE24_Pos)                    /*!< KPI_T::KPIKRE0: KRE24 Mask                 */
#define KPI_KPIKRE0_KRE25_Pos              (21)                                                /*!< KPI_T::KPIKRE0: KRE25 Position             */
#define KPI_KPIKRE0_KRE25_Msk              (0x1ul << KPI_KPIKRE0_KRE25_Pos)                    /*!< KPI_T::KPIKRE0: KRE25 Mask                 */
#define KPI_KPIKRE0_KRE26_Pos              (22)                                                /*!< KPI_T::KPIKRE0: KRE26 Position             */
#define KPI_KPIKRE0_KRE26_Msk              (0x1ul << KPI_KPIKRE0_KRE26_Pos)                    /*!< KPI_T::KPIKRE0: KRE26 Mask                 */
#define KPI_KPIKRE0_KRE27_Pos              (23)                                                /*!< KPI_T::KPIKRE0: KRE27 Position             */
#define KPI_KPIKRE0_KRE27_Msk              (0x1ul << KPI_KPIKRE0_KRE27_Pos)                    /*!< KPI_T::KPIKRE0: KRE27 Mask                 */
#define KPI_KPIKRE0_KRE30_Pos              (24)                                                /*!< KPI_T::KPIKRE0: KRE30 Position             */
#define KPI_KPIKRE0_KRE30_Msk              (0x1ul << KPI_KPIKRE0_KRE30_Pos)                    /*!< KPI_T::KPIKRE0: KRE30 Mask                 */
#define KPI_KPIKRE0_KRE31_Pos              (25)                                                /*!< KPI_T::KPIKRE0: KRE31 Position             */
#define KPI_KPIKRE0_KRE31_Msk              (0x1ul << KPI_KPIKRE0_KRE31_Pos)                    /*!< KPI_T::KPIKRE0: KRE31 Mask                 */
#define KPI_KPIKRE0_KRE32_Pos              (26)                                                /*!< KPI_T::KPIKRE0: KRE32 Position             */
#define KPI_KPIKRE0_KRE32_Msk              (0x1ul << KPI_KPIKRE0_KRE32_Pos)                    /*!< KPI_T::KPIKRE0: KRE32 Mask                 */
#define KPI_KPIKRE0_KRE33_Pos              (27)                                                /*!< KPI_T::KPIKRE0: KRE33 Position             */
#define KPI_KPIKRE0_KRE33_Msk              (0x1ul << KPI_KPIKRE0_KRE33_Pos)                    /*!< KPI_T::KPIKRE0: KRE33 Mask                 */
#define KPI_KPIKRE0_KRE34_Pos              (28)                                                /*!< KPI_T::KPIKRE0: KRE34 Position             */
#define KPI_KPIKRE0_KRE34_Msk              (0x1ul << KPI_KPIKRE0_KRE34_Pos)                    /*!< KPI_T::KPIKRE0: KRE34 Mask                 */
#define KPI_KPIKRE0_KRE35_Pos              (29)                                                /*!< KPI_T::KPIKRE0: KRE35 Position             */
#define KPI_KPIKRE0_KRE35_Msk              (0x1ul << KPI_KPIKRE0_KRE35_Pos)                    /*!< KPI_T::KPIKRE0: KRE35 Mask                 */
#define KPI_KPIKRE0_KRE36_Pos              (30)                                                /*!< KPI_T::KPIKRE0: KRE36 Position             */
#define KPI_KPIKRE0_KRE36_Msk              (0x1ul << KPI_KPIKRE0_KRE36_Pos)                    /*!< KPI_T::KPIKRE0: KRE36 Mask                 */
#define KPI_KPIKRE0_KRE37_Pos              (31)                                                /*!< KPI_T::KPIKRE0: KRE37 Position             */
#define KPI_KPIKRE0_KRE37_Msk              (0x1ul << KPI_KPIKRE0_KRE37_Pos)                    /*!< KPI_T::KPIKRE0: KRE37 Mask                 */


#define KPI_KPIKRE1_KRE40_Pos              (0)                                                 /*!< KPI_T::KPIKRE1: KRE40 Position             */
#define KPI_KPIKRE1_KRE40_Msk              (0x1ul << KPI_KPIKRE1_KRE40_Pos)                    /*!< KPI_T::KPIKRE1: KRE40 Mask                 */
#define KPI_KPIKRE1_KRE41_Pos              (1)                 1                               /*!< KPI_T::KPIKRE1: KRE41 Position             */
#define KPI_KPIKRE1_KRE41_Msk              (0x1ul << KPI_KPIKRE1_KRE41_Pos)                    /*!< KPI_T::KPIKRE1: KRE41 Mask                 */
#define KPI_KPIKRE1_KRE42_Pos              (2)                 1                               /*!< KPI_T::KPIKRE1: KRE42 Position             */
#define KPI_KPIKRE1_KRE42_Msk              (0x1ul << KPI_KPIKRE1_KRE42_Pos)                    /*!< KPI_T::KPIKRE1: KRE42 Mask                 */
#define KPI_KPIKRE1_KRE43_Pos              (3)                 1                               /*!< KPI_T::KPIKRE1: KRE43 Position             */
#define KPI_KPIKRE1_KRE43_Msk              (0x1ul << KPI_KPIKRE1_KRE43_Pos)                    /*!< KPI_T::KPIKRE1: KRE42 Mask                 */
#define KPI_KPIKRE1_KRE44_Pos              (4)                 1                               /*!< KPI_T::KPIKRE1: KRE44 Position             */
#define KPI_KPIKRE1_KRE44_Msk              (0x1ul << KPI_KPIKRE1_KRE44_Pos)                    /*!< KPI_T::KPIKRE1: KRE44 Mask                 */
#define KPI_KPIKRE1_KRE45_Pos              (5)                 1                               /*!< KPI_T::KPIKRE1: KRE45 Position             */
#define KPI_KPIKRE1_KRE45_Msk              (0x1ul << KPI_KPIKRE1_KRE45_Pos)                    /*!< KPI_T::KPIKRE1: KRE45 Mask                 */
#define KPI_KPIKRE1_KRE46_Pos              (6)                 1                               /*!< KPI_T::KPIKRE1: KRE46 Position             */
#define KPI_KPIKRE1_KRE46_Msk              (0x1ul << KPI_KPIKRE1_KRE46_Pos)                    /*!< KPI_T::KPIKRE1: KRE46 Mask                 */
#define KPI_KPIKRE1_KRE47_Pos              (7)                 1                               /*!< KPI_T::KPIKRE1: KRE47 Position             */
#define KPI_KPIKRE1_KRE47_Msk              (0x1ul << KPI_KPIKRE1_KRE47_Pos)                    /*!< KPI_T::KPIKRE1: KRE47 Mask                 */
#define KPI_KPIKRE1_KRE50_Pos              (8)                 1                               /*!< KPI_T::KPIKRE1: KRE50 Position             */
#define KPI_KPIKRE1_KRE50_Msk              (0x1ul << KPI_KPIKRE1_KRE50_Pos)                    /*!< KPI_T::KPIKRE1: KRE50 Mask                 */
#define KPI_KPIKRE1_KRE51_Pos              (9)                 1                               /*!< KPI_T::KPIKRE1: KRE51 Position             */
#define KPI_KPIKRE1_KRE51_Msk              (0x1ul << KPI_KPIKRE1_KRE51_Pos)                    /*!< KPI_T::KPIKRE1: KRE51 Mask                 */
#define KPI_KPIKRE1_KRE52_Pos              (10)                1                               /*!< KPI_T::KPIKRE1: KRE52 Position             */
#define KPI_KPIKRE1_KRE52_Msk              (0x1ul << KPI_KPIKRE1_KRE52_Pos)                    /*!< KPI_T::KPIKRE1: KRE52 Mask                 */
#define KPI_KPIKRE1_KRE53_Pos              (11)                1                               /*!< KPI_T::KPIKRE1: KRE53 Position             */
#define KPI_KPIKRE1_KRE53_Msk              (0x1ul << KPI_KPIKRE1_KRE53_Pos)                    /*!< KPI_T::KPIKRE1: KRE53 Mask                 */
#define KPI_KPIKRE1_KRE54_Pos              (12)                1                               /*!< KPI_T::KPIKRE1: KRE54 Position             */
#define KPI_KPIKRE1_KRE54_Msk              (0x1ul << KPI_KPIKRE1_KRE54_Pos)                    /*!< KPI_T::KPIKRE1: KRE54 Mask                 */
#define KPI_KPIKRE1_KRE55_Pos              (13)                1                               /*!< KPI_T::KPIKRE1: KRE55 Position             */
#define KPI_KPIKRE1_KRE55_Msk              (0x1ul << KPI_KPIKRE1_KRE55_Pos)                    /*!< KPI_T::KPIKRE1: KRE55 Mask                 */
#define KPI_KPIKRE1_KRE56_Pos              (14)                1                               /*!< KPI_T::KPIKRE1: KRE56 Position             */
#define KPI_KPIKRE1_KRE56_Msk              (0x1ul << KPI_KPIKRE1_KRE56_Pos)                    /*!< KPI_T::KPIKRE1: KRE56 Mask                 */
#define KPI_KPIKRE1_KRE57_Pos              (15)                                                /*!< KPI_T::KPIKRE1: KRE57 Position             */
#define KPI_KPIKRE1_KRE57_Msk              (0x1ul << KPI_KPIKRE1_KRE57_Pos)                    /*!< KPI_T::KPIKRE1: KRE57 Mask                 */

#define KPI_KPIPRESCALDIV_PRESCALDIV_Pos   (0)                                                 /*!< KPI_T::KPIPRESCALDIV: PRESCALDIV Position             */
#define KPI_KPIPRESCALDIV_PRESCALDIV_Msk   (0xfful << KPI_KPIPRESCALDIV_PRESCALDIV_Pos)        /*!< KPI_T::KPIPRESCALDIV: PRESCALDIV Mask                 */


#endif


