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

//#define RPMSG_DDR_BUF  /* Enable if you want to use DRAM as shared memory */
//#define RPMSG_V2_ARCH  /* Enable if you want to use rpmsg v2 arch */

#ifdef RPMSG_DDR_BUF
  #define Share_Memory_Size 0x4000*2   /* Shared memory in DRAM could be 0x80 ~ 0x4000 bytes */
#else
  #undef Share_Memory_Size
  #define Share_Memory_Size 0x80*2     /* Shared memory in SRAM is fixed to 0x80 bytes */
#endif

#ifdef RPMSG_V2_ARCH
  #define RPMSG_VERSION 2
#else
  #define RPMSG_VERSION 1
#endif

#define MAILBOX_WHC_IF_ENABLED

#ifdef MAILBOX_WHC_IF_ENABLED
#include "mbox_whc.h"

#define mbox_ch 2
#endif /* MAILBOX_WHC_IF_ENABLED */

static uint32_t Shere_Memory__[Share_Memory_Size];

#ifdef RPMSG_DDR_BUF
  #define SHM_START_ADDRESS       (metal_phys_addr_t)(0x80080000)
#else
  #define SHM_START_ADDRESS       (metal_phys_addr_t)(0x2401ff00)
#endif
#define SHM_SIZE                (size_t)Share_Memory_Size
#define SHM_TX_RX_SIZE          (size_t)(Share_Memory_Size/2)
#define SHM_RX_START_ADDRESS    SHM_START_ADDRESS
#define SHM_TX_START_ADDRESS    (SHM_RX_START_ADDRESS+SHM_TX_RX_SIZE)

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


