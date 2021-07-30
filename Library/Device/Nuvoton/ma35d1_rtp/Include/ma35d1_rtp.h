/**************************************************************************//**
 * @file     ma35d1_rtp.h
 * @brief    Peripheral access layer header file.
 *           This file contains all the peripheral register's definitions
 *           and bits definitions and memory mapping.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/**
  \mainpage NuMicro MA35D1 RTP Driver Reference Guide
  *
  * <b>Introduction</b>
  *
  * This user manual describes the usage of MA35D1 RTP device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Important Notice</b>
  *
  * Nuvoton Products are neither intended nor warranted for usage in systems or equipment,
  * any malfunction or failure of which may cause loss of human life, bodily injury or severe
  * property damage. Such applications are deemed, "Insecure Usage".
  *
  * Insecure usage includes, but is not limited to: equipment for surgical implementation,
  * atomic energy control instruments, airplane or spaceship instruments, the control or
  * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
  * instruments, all types of safety devices, and other applications intended to support or
  * sustain life.
  *
  * All Insecure Usage shall be made at customer's risk, and in the event that third parties
  * lay claims to Nuvoton as a result of customer's Insecure Usage, customer shall indemnify
  * the damages and liabilities thus incurred by Nuvoton.
  *
  * Please note that all data and specifications are subject to change without notice. All the
  * trademarks of products and companies mentioned in this datasheet belong to their respective
  * owners.
  *
  * <b>Copyright Notice</b>
  *
  * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
  */
#ifndef __MA35D1_RTP_H__
#define __MA35D1_RTP_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup CMSIS_Device Device CMSIS Definitions
  Configuration of the Cortex-M4 Processor and Core Peripherals
  @{
*/

/**
 * @details  Interrupt Number Definition.
 */
typedef enum IRQn
{
    /******  Cortex-M4 Processor Exceptions Numbers *************************************************/
    NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt                        */
    MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt                   */
    BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt                           */
    UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt                         */
    SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                             */
    DebugMonitor_IRQn             = -4,       /*!< 12 Debug Monitor Interrupt                       */
    PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                             */
    SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt                         */

    /******  ma35d_rtp Specific Interrupt Numbers **************************************************/

    LVD_IRQn                      = 0,
    PWRWU_IRQn                    = 2,
    HWSEM0_IRQn                   = 3,
    CKFAIL_IRQn                   = 4,
    WHC0_IRQn                     = 5,
    RTC_IRQn                      = 6,
    TAMPER_IRQn                   = 7,
    WDT2_IRQn                     = 8,
    WWDT2_IRQn                    = 9,
    EINT0_IRQn                    = 10,
    EINT1_IRQn                    = 11,
    EINT2_IRQn                    = 12,
    EINT3_IRQn                    = 13,
    GPA_IRQn                      = 16,
    GPB_IRQn                      = 17,
    GPC_IRQn                      = 18,
    GPD_IRQn                      = 19,
    TMR2_IRQn                     = 22,
    TMR3_IRQn                     = 23,
    BRAKE0_IRQn                   = 24,
    EPWM0P0_IRQn                  = 25,
    EPWM0P1_IRQn                  = 26,
    EPWM0P2_IRQn                  = 27,
    QEI0_IRQn                     = 28,
    ECAP0_IRQn                    = 29,
    QSPI1_IRQn                    = 31,
    UART1_IRQn                    = 35,
    UART2_IRQn                    = 36,
    UART3_IRQn                    = 37,
    UART4_IRQn                    = 38,
    UART5_IRQn                    = 39,
    EADC00_IRQn                   = 40,
    EADC01_IRQn                   = 41,
    EADC02_IRQn                   = 42,
    EADC03_IRQn                   = 43,
    I2C1_IRQn                     = 45,
    I2S0_IRQn                     = 46,
    CANFD00_IRQn                  = 47,
    SC0_IRQn                      = 48,
    GPE_IRQn                      = 49,
    GPF_IRQn                      = 50,
    GPG_IRQn                      = 51,
    GPH_IRQn                      = 52,
    GPI_IRQn                      = 53,
    GPJ_IRQn                      = 54,
    TMR4_IRQn                     = 55,
    TMR5_IRQn                     = 56,
    TMR6_IRQn                     = 57,
    TMR7_IRQn                     = 58,
    BRAKE1_IRQn                   = 59,
    EPWM1P0_IRQn                  = 60,
    EPWM1P1_IRQn                  = 61,
    EPWM1P2_IRQn                  = 62,
    QEI1_IRQn                     = 63,
    ECAP1_IRQn                    = 64,
    SPI0_IRQn                     = 65,
    SPI1_IRQn                     = 66,
    PDMA2_IRQn                    = 67,
    PDMA3_IRQn                    = 68,
    UART6_IRQn                    = 69,
    UART7_IRQn                    = 70,
    UART8_IRQn                    = 71,
    UART9_IRQn                    = 72,
    UART10_IRQn                   = 73,
    UART11_IRQn                   = 74,
    I2C2_IRQn                     = 75,
    I2C3_IRQn                     = 76,
    I2S1_IRQn                     = 77,
    CANFD10_IRQn                  = 78,
    SC1_IRQn                      = 79,
    GPK_IRQn                      = 80,
    GPL_IRQn                      = 81,
    GPM_IRQn                      = 82,
    GPN_IRQn                      = 83,
    TMR8_IRQn                     = 84,
    TMR9_IRQn                     = 85,
    TMR10_IRQn                    = 86,
    TMR11_IRQn                    = 87,
    BRAKE2_IRQn                   = 88,
    EPWM2P0_IRQn                  = 89,
    EPWM2P1_IRQn                  = 90,
    EPWM2P2_IRQn                  = 91,
    QEI2_IRQn                     = 92,
    ECAP2_IRQn                    = 93,
    SPI2_IRQn                     = 94,
    SPI3_IRQn                     = 95,
    UART12_IRQn                   = 96,
    UART13_IRQn                   = 97,
    UART14_IRQn                   = 98,
    UART15_IRQn                   = 99,
    UART16_IRQn                   = 100,
    I2C4_IRQn                     = 101,
    I2C5_IRQn                     = 102,
    CANFD20_IRQn                  = 103,
    CANFD30_IRQn                  = 104,
    KPI_IRQn                      = 105,
    CANFD01_IRQn                  = 106,
    CANFD11_IRQn                  = 107,
    CANFD21_IRQn                  = 108,
    CANFD31_IRQn                  = 109,
    ADC0_IRQn                     = 110,
}
IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M4 Processor and Core Peripherals */
#define __CM4_REV                 0x0201UL    /*!< Core Revision r2p1                               */
#define __NVIC_PRIO_BITS          4UL         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0UL         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             1UL         /*!< MPU present or not                               */
#ifdef __FPU_PRESENT
#undef __FPU_PRESENT
#define __FPU_PRESENT             1UL         /*!< FPU present or not                               */
#else
#define __FPU_PRESENT             1UL         /*!< FPU present or not                               */
#endif

/*@}*/ /* end of group CMSIS_Device */


#include "core_cm4.h"               /* Cortex-M4 processor and core peripherals           */
#include "system_ma35d1_rtp.h"            /* System include file                         */
#include <stdint.h>



#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/******************************************************************************/
/*                            Register definitions                            */
/******************************************************************************/
#include "sys_reg.h"
#include "clk_reg.h"

#include "uart_reg.h"
#include "whc_reg.h"
#include "hwsem_reg.h"
#include "wdt_reg.h"
#include "wwdt_reg.h"
#include "adc_reg.h"
#include "eadc_reg.h"
#include "sc_reg.h"
#include "gpio_reg.h"
#include "ecap_reg.h"
#include "qei_reg.h"
#include "i2c_reg.h"
#include "i2s_reg.h"
#include "pdma_reg.h"
#include "qspi_reg.h"
#include "spi_reg.h"
#include "rtc_reg.h"
#include "timer_reg.h"
#include "ebi_reg.h"
#include "epwm_reg.h"
//#include "kpi_reg.h"
#include "canfd_reg.h"

/** @addtogroup PERIPHERAL_MEM_MAP Peripheral Memory Base
  Memory Mapped Structure for Peripherals
  @{
 */
/* Peripheral and SRAM base address */
#define SRAM_BASE               (0x00000000ul)      /*!< SRAM Base Address       */
#define DDR_BASE                (0x00040000ul)      /*!< DDR Base Address        */

/*!< AHB peripherals */
#define SYS_BASE                (0x40460000UL)
#define CLK_BASE                (0x40460200UL)
#define GPIOA_BASE              (0x40040000UL)
#define GPIOB_BASE              (0x40040040UL)
#define GPIOC_BASE              (0x40040080UL)
#define GPIOD_BASE              (0x400400C0UL)
#define GPIOE_BASE              (0x40040100UL)
#define GPIOF_BASE              (0x40040140UL)
#define GPIOG_BASE              (0x40040180UL)
#define GPIOH_BASE              (0x400401C0UL)
#define GPIOI_BASE              (0x40040200UL)
#define GPIOJ_BASE              (0x40040240UL)
#define GPIOK_BASE              (0x40040280UL)
#define GPIOL_BASE              (0x400402C0UL)
#define GPIOM_BASE              (0x40040300UL)
#define GPION_BASE              (0x40040340UL)
#define GPIO_PIN_DATA_BASE      (0x40040800UL)
#define PDMA2_BASE              (0x400A0000UL)
#define PDMA3_BASE              (0x400B0000UL)
#define EBI_BASE                (0x40100000UL)
#define HWSEM_BASE              (0x40380000UL)
#define WHC0_BASE               (0x403A0000UL)
#define MCAN0_BASE              (0x403C0000UL)
#define MCAN1_BASE              (0x403D0000UL)
#define MCAN2_BASE              (0x403E0000UL)
#define MCAN3_BASE              (0x403F0000UL)
#define RTC_BASE                (0x40410000UL)
#define ADC0_BASE               (0x40420000UL)
#define EADC0_BASE              (0x40430000UL)
#define I2S0_BASE               (0x40480000UL)
#define I2S1_BASE               (0x40490000UL)
#define KPI_BASE                (0x404A0000UL)
#define TIMER2_BASE             (0x40510000UL)
#define TIMER3_BASE             (0x40510100UL)
#define TIMER4_BASE             (0x40520000UL)
#define TIMER5_BASE             (0x40520100UL)
#define TIMER6_BASE             (0x40530000UL)
#define TIMER7_BASE             (0x40530100UL)
#define TIMER8_BASE             (0x40540000UL)
#define TIMER9_BASE             (0x40540100UL)
#define TIMER10_BASE            (0x40550000UL)
#define TIMER11_BASE            (0x40550100UL)
#define EPWM0_BASE              (0x40580000UL)
#define EPWM1_BASE              (0x40590000UL)
#define EPWM2_BASE              (0x405A0000UL)
#define SPI0_BASE               (0x40600000UL)
#define SPI1_BASE               (0x40610000UL)
#define SPI2_BASE               (0x40620000UL)
#define SPI3_BASE               (0x40630000UL)
#define QSPI1_BASE              (0x40690000UL)
#define UART1_BASE              (0x40710000UL)
#define UART2_BASE              (0x40720000UL)
#define UART3_BASE              (0x40730000UL)
#define UART4_BASE              (0x40740000UL)
#define UART5_BASE              (0x40750000UL)
#define UART6_BASE              (0x40760000UL)
#define UART7_BASE              (0x40770000UL)
#define UART8_BASE              (0x40780000UL)
#define UART9_BASE              (0x40790000UL)
#define UART10_BASE             (0x407A0000UL)
#define UART11_BASE             (0x407B0000UL)
#define UART12_BASE             (0x407C0000UL)
#define UART13_BASE             (0x407D0000UL)
#define UART14_BASE             (0x407E0000UL)
#define UART15_BASE             (0x407F0000UL)
#define UART16_BASE             (0x40880000UL)
#define I2C1_BASE               (0x40810000UL)
#define I2C2_BASE               (0x40820000UL)
#define I2C3_BASE               (0x40830000UL)
#define I2C4_BASE               (0x40840000UL)
#define I2C5_BASE               (0x40850000UL)
#define SC0_BASE                (0x40900000UL)
#define SC1_BASE                (0x40910000UL)
#define WDT2_BASE               (0x40980000UL)
#define WWDT2_BASE              (0x40980100UL)
#define QEI0_BASE               (0x40B00000UL)
#define QEI1_BASE               (0x40B10000UL)
#define QEI2_BASE               (0x40B20000UL)
#define ECAP0_BASE              (0x40B40000UL)
#define ECAP1_BASE              (0x40B50000UL)
#define ECAP2_BASE              (0x40B60000UL)
#define CANFD0_BASE             (0x403C0000UL)
#define CANFD1_BASE             (0x403D0000UL)
#define CANFD2_BASE             (0x403E0000UL)
#define CANFD3_BASE             (0x403F0000UL)


/*@}*/ /* end of group PERIPHERAL_MEM_MAP */


/** @addtogroup PERIPHERAL_DECLARATION Peripheral Pointer
  The Declaration of Peripherals
  @{
 */

#define SYS                     ((SYS_T *)   SYS_BASE)
#define CLK                     ((CLK_T *)   CLK_BASE)
#define PA                      ((GPIO_T *)  GPIOA_BASE)
#define PB                      ((GPIO_T *)  GPIOB_BASE)
#define PC                      ((GPIO_T *)  GPIOC_BASE)
#define PD                      ((GPIO_T *)  GPIOD_BASE)
#define PE                      ((GPIO_T *)  GPIOE_BASE)
#define PF                      ((GPIO_T *)  GPIOF_BASE)
#define PG                      ((GPIO_T *)  GPIOG_BASE)
#define PH                      ((GPIO_T *)  GPIOH_BASE)
#define PI                      ((GPIO_T *)  GPIOI_BASE)
#define PJ                      ((GPIO_T *)  GPIOJ_BASE)
#define PK                      ((GPIO_T *)  GPIOK_BASE)
#define PL                      ((GPIO_T *)  GPIOL_BASE)
#define PM                      ((GPIO_T *)  GPIOM_BASE)
#define PN                      ((GPIO_T *)  GPION_BASE)
#define PDMA2                   ((PDMA_T *)  PDMA2_BASE)
#define PDMA3                   ((PDMA_T *)  PDMA3_BASE)
#define EBI                     ((EBI_T *)   EBI_BASE)
#define HWSEM0                  ((HWSEM_T *) HWSEM_BASE)
#define WHC0                    ((WHC_T *)   WHC0_BASE)
#define MCAN0                   ((MCAN_T *)  MCAN0_BASE)
#define MCAN1                   ((MCAN_T *)  MCAN1_BASE)
#define MCAN2                   ((MCAN_T *)  MCAN2_BASE)
#define MCAN3                   ((MCAN_T *)  MCAN3_BASE)
#define RTC                     ((RTC_T *)   RTC_BASE)
#define ADC0                    ((ADC_T *)   ADC0_BASE)
#define EADC0                   ((EADC_T *)  EADC0_BASE)
#define I2S0                    ((I2S_T *)   I2S0_BASE)
#define I2S1                    ((I2S_T *)   I2S1_BASE)
#define KPI                     ((KPI_T *)   KPI_BASE)
#define TIMER2                  ((TIMER_T *) TIMER2_BASE)
#define TIMER3                  ((TIMER_T *) TIMER3_BASE)
#define TIMER4                  ((TIMER_T *) TIMER4_BASE)
#define TIMER5                  ((TIMER_T *) TIMER5_BASE)
#define TIMER6                  ((TIMER_T *) TIMER6_BASE)
#define TIMER7                  ((TIMER_T *) TIMER7_BASE)
#define TIMER8                  ((TIMER_T *) TIMER8_BASE)
#define TIMER9                  ((TIMER_T *) TIMER9_BASE)
#define TIMER10                 ((TIMER_T *) TIMER10_BASE)
#define TIMER11                 ((TIMER_T *) TIMER11_BASE)
#define EPWM0                   ((EPWM_T *)  EPWM0_BASE)
#define EPWM1                   ((EPWM_T *)  EPWM1_BASE)
#define EPWM2                   ((EPWM_T *)  EPWM2_BASE)
#define SPI0                    ((SPI_T *)   SPI0_BASE)
#define SPI1                    ((SPI_T *)   SPI1_BASE)
#define SPI2                    ((SPI_T *)   SPI2_BASE)
#define SPI3                    ((SPI_T *)   SPI3_BASE)
#define QSPI1                   ((QSPI_T *)  QSPI1_BASE)
#define UART1                   ((UART_T *)  UART1_BASE)
#define UART2                   ((UART_T *)  UART2_BASE)
#define UART3                   ((UART_T *)  UART3_BASE)
#define UART4                   ((UART_T *)  UART4_BASE)
#define UART5                   ((UART_T *)  UART5_BASE)
#define UART6                   ((UART_T *)  UART6_BASE)
#define UART7                   ((UART_T *)  UART7_BASE)
#define UART8                   ((UART_T *)  UART8_BASE)
#define UART9                   ((UART_T *)  UART9_BASE)
#define UART10                  ((UART_T *)  UART10_BASE)
#define UART11                  ((UART_T *)  UART11_BASE)
#define UART12                  ((UART_T *)  UART12_BASE)
#define UART13                  ((UART_T *)  UART13_BASE)
#define UART14                  ((UART_T *)  UART14_BASE)
#define UART15                  ((UART_T *)  UART15_BASE)
#define UART16                  ((UART_T *)  UART16_BASE)
#define I2C1                    ((I2C_T *)   I2C1_BASE)
#define I2C2                    ((I2C_T *)   I2C2_BASE)
#define I2C3                    ((I2C_T *)   I2C3_BASE)
#define I2C4                    ((I2C_T *)   I2C4_BASE)
#define I2C5                    ((I2C_T *)   I2C5_BASE)
#define SC0                     ((SC_T *)    SC0_BASE)
#define SC1                     ((SC_T *)    SC1_BASE)
#define WDT2                    ((WDT_T *)   WDT2_BASE)
#define WWDT2                   ((WWDT_T *)  WWDT2_BASE)
#define QEI0                    ((QEI_T *)   QEI0_BASE)
#define QEI1                    ((QEI_T *)   QEI1_BASE)
#define QEI2                    ((QEI_T *)   QEI2_BASE)
#define ECAP0                   ((ECAP_T *)  ECAP0_BASE)
#define ECAP1                   ((ECAP_T *)  ECAP1_BASE)
#define ECAP2                   ((ECAP_T *)  ECAP2_BASE)
#define CANFD0                  ((CANFD_T*)  CANFD0_BASE)
#define CANFD1                  ((CANFD_T*)  CANFD1_BASE)
#define CANFD2                  ((CANFD_T*)  CANFD2_BASE)
#define CANFD3                  ((CANFD_T*)  CANFD3_BASE)

/*@}*/ /* end of group ERIPHERAL_DECLARATION */

/** @addtogroup IO_ROUTINE I/O Routines
  The Declaration of I/O Routines
  @{
 */

typedef volatile uint8_t  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile uint16_t vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile uint32_t vu32;       ///< Define 32-bit unsigned volatile data type
typedef volatile uint64_t vu64;       ///< Define 64-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)     *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)            (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)     *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)            (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)            (*((volatile unsigned char *)(port)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)    *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)           (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)    *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)           (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))


/*@}*/ /* end of group IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup Legacy_Constants Legacy Constants
  Legacy Constants
  @{
*/

#ifndef NULL
#define NULL           (0)      ///< NULL pointer
#endif

#define TRUE           (1UL)      ///< Boolean true, define to use in API parameters or return value
#define FALSE          (0UL)      ///< Boolean false, define to use in API parameters or return value

#define ENABLE         (1UL)      ///< Enable, define to use in API parameters
#define DISABLE        (0UL)      ///< Disable, define to use in API parameters

/* Define one bit mask */
#define BIT0     (0x00000001UL)       ///< Bit 0 mask of an 32 bit integer
#define BIT1     (0x00000002UL)       ///< Bit 1 mask of an 32 bit integer
#define BIT2     (0x00000004UL)       ///< Bit 2 mask of an 32 bit integer
#define BIT3     (0x00000008UL)       ///< Bit 3 mask of an 32 bit integer
#define BIT4     (0x00000010UL)       ///< Bit 4 mask of an 32 bit integer
#define BIT5     (0x00000020UL)       ///< Bit 5 mask of an 32 bit integer
#define BIT6     (0x00000040UL)       ///< Bit 6 mask of an 32 bit integer
#define BIT7     (0x00000080UL)       ///< Bit 7 mask of an 32 bit integer
#define BIT8     (0x00000100UL)       ///< Bit 8 mask of an 32 bit integer
#define BIT9     (0x00000200UL)       ///< Bit 9 mask of an 32 bit integer
#define BIT10    (0x00000400UL)       ///< Bit 10 mask of an 32 bit integer
#define BIT11    (0x00000800UL)       ///< Bit 11 mask of an 32 bit integer
#define BIT12    (0x00001000UL)       ///< Bit 12 mask of an 32 bit integer
#define BIT13    (0x00002000UL)       ///< Bit 13 mask of an 32 bit integer
#define BIT14    (0x00004000UL)       ///< Bit 14 mask of an 32 bit integer
#define BIT15    (0x00008000UL)       ///< Bit 15 mask of an 32 bit integer
#define BIT16    (0x00010000UL)       ///< Bit 16 mask of an 32 bit integer
#define BIT17    (0x00020000UL)       ///< Bit 17 mask of an 32 bit integer
#define BIT18    (0x00040000UL)       ///< Bit 18 mask of an 32 bit integer
#define BIT19    (0x00080000UL)       ///< Bit 19 mask of an 32 bit integer
#define BIT20    (0x00100000UL)       ///< Bit 20 mask of an 32 bit integer
#define BIT21    (0x00200000UL)       ///< Bit 21 mask of an 32 bit integer
#define BIT22    (0x00400000UL)       ///< Bit 22 mask of an 32 bit integer
#define BIT23    (0x00800000UL)       ///< Bit 23 mask of an 32 bit integer
#define BIT24    (0x01000000UL)       ///< Bit 24 mask of an 32 bit integer
#define BIT25    (0x02000000UL)       ///< Bit 25 mask of an 32 bit integer
#define BIT26    (0x04000000UL)       ///< Bit 26 mask of an 32 bit integer
#define BIT27    (0x08000000UL)       ///< Bit 27 mask of an 32 bit integer
#define BIT28    (0x10000000UL)       ///< Bit 28 mask of an 32 bit integer
#define BIT29    (0x20000000UL)       ///< Bit 29 mask of an 32 bit integer
#define BIT30    (0x40000000UL)       ///< Bit 30 mask of an 32 bit integer
#define BIT31    (0x80000000UL)       ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk              (0x000000FFUL)         ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk              (0x0000FF00UL)         ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk              (0x00FF0000UL)         ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk              (0xFF000000UL)         ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group Legacy_Constants */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"

#include "uart.h"
#include "hwsem.h"
#include "whc.h"
#include "gpio.h"
#include "ecap.h"
#include "qei.h"
#include "timer.h"
#include "timer_pwm.h"
#include "pdma.h"
#include "i2c.h"
#include "i2s.h"
#include "epwm.h"
#include "eadc.h"
#include "adc.h"
#include "wdt.h"
#include "wwdt.h"
#include "ebi.h"
#include "scuart.h"
#include "sc.h"
#include "spi.h"
#include "qspi.h"
#include "rtc.h"
//#include "kpi.h"
#include "canfd.h"

#ifdef __cplusplus
}
#endif

#endif  /* __MA35D1_RTP_H__ */

