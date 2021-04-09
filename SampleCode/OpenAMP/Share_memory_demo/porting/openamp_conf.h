/****************************************************************************
 * @file     openamp_conf.h
 *
 * @brief    openamp config file.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __OPENAMP_CONF__H__
#define __OPENAMP_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

#if defined (__LOG_TRACE_IO_) || defined(__LOG_UART_IO_)
#include "log.h"
#endif

#include "NuMicro.h"

#define MAILBOX_WHC_IF_ENABLED


#ifdef MAILBOX_WHC_IF_ENABLED
#include "mbox_whc.h"
#endif /* MAILBOX_WHC_IF_ENABLED */

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))

#define Shere_Memory_Size 1024
static uint32_t Shere_Memory__[Shere_Memory_Size];

#define SHM_START_ADDRESS       (metal_phys_addr_t)(0x24000000)
#define SHM_SIZE                (size_t)Shere_Memory_Size

#else
#define Shere_Memory_Size 1024
static uint32_t Shere_Memory__[Shere_Memory_Size];

#define SHM_START_ADDRESS       (metal_phys_addr_t)(0x24000000)
#define SHM_SIZE                (size_t)Shere_Memory_Size
#endif

#define VRING_RX_STR_ADDR        -1
#define VRING_TX_STR_ADDR        -1
#define VRING_BUF_ADDR           -1
#define VRING_ALIGNMENT          32
#define VRING_NUM_BUF            16

/* Fixed parameter */
#define NUM_RESOURCE_ENTRIES    2
#define VRING_COUNT             2

#define VDEV_ID                 0xFF
#define VRING0_ID               0
#define VRING1_ID               1

/* DEBUG macros */

#if defined (__LOG_TRACE_IO_) || defined(__LOG_UART_IO_)
  #define OPENAMP_log_dbg               log_dbg
  #define OPENAMP_log_info              log_info
  #define OPENAMP_log_warn              log_warn
  #define OPENAMP_log_err               log_err
#else
  #define OPENAMP_log_dbg(...)
  #define OPENAMP_log_info(...)
  #define OPENAMP_log_warn(...)
  #define OPENAMP_log_err(...)
#endif


#ifdef __cplusplus
}
#endif

#endif /* __OPENAMP_CONF__H__ */


