/*
 * Copyright (c) 2018, Linaro Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file	freertos/template/sys.c
 * @brief	machine specific system primitives implementation.
 */

#include <metal/io.h>
#include <metal/sys.h>
#include <metal/utilities.h>
#include <stdint.h>

#include "NuMicro.h"

void sys_irq_restore_enable(unsigned int flags)
{
	metal_unused(flags);
	/* Add implementation here */
}

unsigned int sys_irq_save_disable(void)
{
	return 0;
	/* Add implementation here */
}

void sys_irq_enable(unsigned int vector)
{
	NVIC_EnableIRQ((IRQn_Type)vector);
}

void sys_irq_disable(unsigned int vector)
{
	NVIC_DisableIRQ((IRQn_Type)vector);
}

void metal_machine_cache_flush(void *addr, unsigned int len)
{
	// if (addr && len)
	// 	dcache_clean_by_mva(addr, (size_t)len);
}

void metal_machine_cache_invalidate(void *addr, unsigned int len)
{
	// if (addr && len)
	// 	dcache_clean_invalidate_by_mva(addr, (size_t)len);
}

void metal_generic_default_poll(void)
{
	__WFI();
}

void *metal_machine_io_mem_map(void *va, metal_phys_addr_t pa,
			       size_t size, unsigned int flags)
{
	metal_unused(pa);
	metal_unused(size);
	metal_unused(flags);

	/* Add implementation here */

	return va;
}

#if defined(__CC_ARM)
void __sync_synchronize(void) {
    __DMB();  // Data Memory Barrier intrinsic
}

uint8_t __sync_val_compare_and_swap_1(volatile uint8_t *ptr, uint8_t oldval, uint8_t newval) {
    uint8_t prev;
    __disable_irq();  // Disable interrupts to ensure atomicity
    prev = *ptr;
    if (prev == oldval) {
        *ptr = newval;
    }
    __enable_irq();  // Re-enable interrupts
    return prev;
}
#endif
