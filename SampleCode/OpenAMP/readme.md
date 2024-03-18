/*************************************************************************//**
 * @file     readme.md
 * @version  V1.00
 * @brief    OpenAMP read me for MA35D1 MPU RTP core.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

**Share_memory_demo**
This sample code is designed for non-OS & Linux.
The provided sample code is based on rpmsg v1, which utilizes "SRAM" as shared memory.
1. This project involve RTP portion of share memory demo, please also review the sample code in non-OS BSP.
https://github.com/OpenNuvoton/MA35D1_NonOS_BSP/tree/master/SampleCode/OpenAMP/Share_Memory_SRAM
2. This project involve RTP portion of share memory demo, please also review the sample code in Linux applications.
https://github.com/OpenNuvoton/MA35D1_Linux_Applications/tree/master/examples/rpmsg

**Share_Memory_SDRAM**
This sample code is designed for non-OS environments only.
The provided sample code is based on rpmsg v1, which utilizes "DRAM" as shared memory.
This project involve RTP portion of share memory demo, please also review the sample code in non-OS BSP.
https://github.com/OpenNuvoton/MA35D1_NonOS_BSP/tree/master/SampleCode/OpenAMP/Share_Memory_SDRAM

**rpmsg_rtp**
This sample code is designed for Linux only.
The provided sample code can be configured for rpmsg v1 or rpmsg v2, which utilizes SRAM or DRAM as shared memory.
If recursive read/write is necessary in your application, it is highly recommended to use the v2 architecture.
This project involve RTP portion of rpmsg demo, please also review the sample codes in Linux applications.
1. rpmsg-v1
https://github.com/OpenNuvoton/MA35D1_Linux_Applications/tree/master/examples/rpmsg
2. rpmsg-v2
https://github.com/OpenNuvoton/MA35D1_Linux_Applications/tree/master/examples/rpmsg-v2

Note:
a. If "RPMSG_DDR_BUF" is defined, please ensure that the symbols "rpmsg-ddr-buf" and "memory-region" in device tree are also enabled for Linux driver.
b. If "RPMSG_V2_ARCH" is defined, please ensure that the symbol "rpmsg-v2-arch" in device tree is also enabled for Linux driver.
c. If you would like to modify "SHM_START_ADDRESS" and "Share_Memory_Size", please ensure that they match the definition of "rpmsg_buf" in device tree.
d. Please refer to MA35D1 Linux BSP & RTP user manual for more details.