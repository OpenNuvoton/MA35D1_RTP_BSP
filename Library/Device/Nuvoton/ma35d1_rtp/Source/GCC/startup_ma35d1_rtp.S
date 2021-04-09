/****************************************************************************//**
 * @file     startup_ma35d1_rtp.S
 * @brief    CMSIS Cortex-M4 Core Device Startup File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/



	.syntax	unified
	.arch	armv7-m

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000800
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000100
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__StackTop            /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long	LVD_IRQHandler        /*   0      */
	.long	0                     /*   1      */
	.long	PWRWU_IRQHandler      /*   2      */
	.long	HWSEM0_IRQHandler     /*   3      */
	.long	CKFAIL_IRQHandler     /*   4      */
	.long	WHC0_IRQHandler       /*   5      */
	.long	RTC_IRQHandler        /*   6      */
	.long	TAMPER_IRQHandler     /*   7      */
	.long	WDT2_IRQHandler       /*   8      */
	.long	WWDT2_IRQHandler      /*   9      */
	.long	EINT0_IRQHandler      /*  10      */
	.long	EINT1_IRQHandler      /*  11      */
	.long	EINT2_IRQHandler      /*  12      */
	.long	EINT3_IRQHandler      /*  13      */
	.long	0                     /*  14      */
	.long	0                     /*  15      */
	.long	GPA_IRQHandler        /*  16      */
	.long	GPB_IRQHandler        /*  17      */
	.long	GPC_IRQHandler        /*  18      */
	.long	GPD_IRQHandler        /*  19      */
	.long	0                     /*  20      */
	.long	0                     /*  21      */
	.long	TMR2_IRQHandler       /*  22      */
	.long	TMR3_IRQHandler       /*  23      */
	.long	BRAKE0_IRQHandler     /*  24      */
	.long	EPWM0P0_IRQHandler    /*  25      */
	.long	EPWM0P1_IRQHandler    /*  26      */
	.long	EPWM0P2_IRQHandler    /*  27      */
	.long	QEI0_IRQHandler       /*  28      */
	.long	ECAP0_IRQHandler      /*  29      */
	.long	0                     /*  30      */
	.long	QSPI1_IRQHandler      /*  31      */
	.long	0                     /*  32      */
	.long	0                     /*  33      */
	.long	0                     /*  34      */
	.long	UART1_IRQHandler      /*  35      */
	.long	UART2_IRQHandler      /*  36      */
	.long	UART3_IRQHandler      /*  37      */
	.long	UART4_IRQHandler      /*  38      */
	.long	UART5_IRQHandler      /*  39      */
	.long	EADC00_IRQHandler     /*  40      */
	.long	EADC01_IRQHandler     /*  41      */
	.long	EADC02_IRQHandler     /*  42      */
	.long	EADC03_IRQHandler     /*  43      */
	.long	0                     /*  44      */
	.long	I2C1_IRQHandler       /*  45      */
	.long	I2S0_IRQHandler       /*  46      */
	.long	MCAN00_IRQHandler     /*  47      */
	.long	SC0_IRQHandler        /*  48      */
	.long	GPE_IRQHandler        /*  49      */
	.long	GPF_IRQHandler        /*  50      */
	.long	GPG_IRQHandler        /*  51      */
	.long	GPH_IRQHandler        /*  52      */
	.long	GPI_IRQHandler        /*  53      */
	.long	GPJ_IRQHandler        /*  54      */
	.long	TMR4_IRQHandler       /*  55      */
	.long	TMR5_IRQHandler       /*  56      */
	.long	TMR6_IRQHandler       /*  57      */
	.long	TMR7_IRQHandler       /*  58      */
	.long	BRAKE1_IRQHandler     /*  59      */
	.long	EPWM1P0_IRQHandler    /*  60      */
	.long	EPWM1P1_IRQHandler    /*  61      */
	.long	EPWM1P2_IRQHandler    /*  62      */
	.long	QEI1_IRQHandler       /*  63      */
	.long	ECAP1_IRQHandler      /*  64      */
	.long	SPI0_IRQHandler       /*  65      */
	.long	SPI1_IRQHandler       /*  66      */
	.long	PDMA2_IRQHandler      /*  67      */
	.long	PDMA3_IRQHandler      /*  68      */
	.long	UART6_IRQHandler      /*  69      */
	.long	UART7_IRQHandler      /*  70      */
	.long	UART8_IRQHandler      /*  71      */
	.long	UART9_IRQHandler      /*  72      */
	.long	UART10_IRQHandler     /*  73      */
	.long	UART11_IRQHandler     /*  74      */
	.long	I2C2_IRQHandler       /*  75      */
	.long	I2C3_IRQHandler       /*  76      */
	.long	I2S1_IRQHandler       /*  77      */
	.long	MACN10_IRQHandler     /*  78      */
	.long	SC1_IRQHandler        /*  79      */
	.long	GPK_IRQHandler        /*  80      */
	.long	GPL_IRQHandler        /*  81      */
	.long	GPM_IRQHandler        /*  82      */
	.long	GPN_IRQHandler        /*  83      */
	.long	TMR8_IRQHandler       /*  84      */
	.long	TMR9_IRQHandler       /*  85      */
	.long	TMR10_IRQHandler      /*  86      */
	.long	TMR11_IRQHandler      /*  87      */
	.long	BRAKE2_IRQHandler     /*  88      */
	.long	EPWM2P0_IRQHandler    /*  89      */
	.long	EPWM2P1_IRQHandler    /*  90      */
	.long	EPWM2P2_IRQHandler    /*  91      */
	.long	QEI2_IRQHandler       /*  92      */
	.long	ECAP2_IRQHandler      /*  93      */
	.long	SPI2_IRQHandler       /*  94      */
	.long	SPI3_IRQHandler       /*  95      */
	.long	UART12_IRQHandler     /*  96      */
	.long	UART13_IRQHandler     /*  97      */
	.long	UART14_IRQHandler     /*  98      */
	.long	UART15_IRQHandler     /*  99      */
	.long	UART16_IRQHandler     /* 100      */
	.long	I2C4_IRQHandler       /* 101      */
	.long	I2C5_IRQHandler       /* 102      */
	.long	MCAN20_IRQHandler     /* 103      */
	.long	MCAN30_IRQHandler     /* 104      */
	.long	KPI_IRQHandler        /* 105      */
	.long	MCAN01_IRQHandler     /* 106      */
	.long	MCAN11_IRQHandler     /* 107      */
	.long	MCAN21_IRQHandler     /* 108      */
	.long	MCAN31_IRQHandler     /* 109      */
	.long	ADC0_IRQHandler       /* 110      */
	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

.L_loop0_0:
	subs	r3, #4
	ittt	ge
	ldrge	r0, [r1, r3]
	strge	r0, [r2, r3]
	bge	.L_loop0_0

	adds	r4, #12
	b	.L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

.L_loop1:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop1
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	movs	r0, 0

.L_loop2_0:
	subs	r2, #4
	itt	ge
	strge	r0, [r1, r2]
	bge	.L_loop2_0

	adds	r3, #8
	b	.L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

	movs	r0, 0
.L_loop3:
	cmp	r1, r2
	itt	lt
	strlt	r0, [r1], #4
	blt	.L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif

#ifndef __START
#define __START _start
#endif
	bl	__START

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

	def_irq_handler	LVD_IRQHandler
	def_irq_handler	PWRWU_IRQHandler
	def_irq_handler	HWSEM0_IRQHandler
	def_irq_handler	CKFAIL_IRQHandler
	def_irq_handler	WHC0_IRQHandler
	def_irq_handler	RTC_IRQHandler
	def_irq_handler	TAMPER_IRQHandler
	def_irq_handler	WDT2_IRQHandler
	def_irq_handler	WWDT2_IRQHandler
	def_irq_handler	EINT0_IRQHandler
	def_irq_handler	EINT1_IRQHandler
	def_irq_handler	EINT2_IRQHandler
	def_irq_handler	EINT3_IRQHandler
	def_irq_handler	GPA_IRQHandler
	def_irq_handler	GPB_IRQHandler
	def_irq_handler	GPC_IRQHandler
	def_irq_handler	GPD_IRQHandler
	def_irq_handler	TMR2_IRQHandler
	def_irq_handler	TMR3_IRQHandler
	def_irq_handler	BRAKE0_IRQHandler
	def_irq_handler	EPWM0P0_IRQHandler
	def_irq_handler	EPWM0P1_IRQHandler
	def_irq_handler	EPWM0P2_IRQHandler
	def_irq_handler	QEI0_IRQHandler
	def_irq_handler	ECAP0_IRQHandler
	def_irq_handler	QSPI1_IRQHandler
	def_irq_handler	UART1_IRQHandler
	def_irq_handler	UART2_IRQHandler
	def_irq_handler	UART3_IRQHandler
	def_irq_handler	UART4_IRQHandler
	def_irq_handler	UART5_IRQHandler
	def_irq_handler	EADC00_IRQHandler
	def_irq_handler	EADC01_IRQHandler
	def_irq_handler	EADC02_IRQHandler
	def_irq_handler	EADC03_IRQHandler
	def_irq_handler	I2C1_IRQHandler
	def_irq_handler	I2S0_IRQHandler
	def_irq_handler	MCAN00_IRQHandler
	def_irq_handler	SC0_IRQHandler
	def_irq_handler	GPE_IRQHandler
	def_irq_handler	GPF_IRQHandler
	def_irq_handler	GPG_IRQHandler
	def_irq_handler	GPH_IRQHandler
	def_irq_handler	GPI_IRQHandler
	def_irq_handler	GPJ_IRQHandler
	def_irq_handler	TMR4_IRQHandler
	def_irq_handler	TMR5_IRQHandler
	def_irq_handler	TMR6_IRQHandler
	def_irq_handler	TMR7_IRQHandler
	def_irq_handler	BRAKE1_IRQHandler
	def_irq_handler	EPWM1P0_IRQHandler
	def_irq_handler	EPWM1P1_IRQHandler
	def_irq_handler	EPWM1P2_IRQHandler
	def_irq_handler	QEI1_IRQHandler
	def_irq_handler	ECAP1_IRQHandler
	def_irq_handler	SPI0_IRQHandler
	def_irq_handler	SPI1_IRQHandler
	def_irq_handler	PDMA2_IRQHandler
	def_irq_handler	PDMA3_IRQHandler
	def_irq_handler	UART6_IRQHandler
	def_irq_handler	UART7_IRQHandler
	def_irq_handler	UART8_IRQHandler
	def_irq_handler	UART9_IRQHandler
	def_irq_handler	UART10_IRQHandler
	def_irq_handler	UART11_IRQHandler
	def_irq_handler	I2C2_IRQHandler
	def_irq_handler	I2C3_IRQHandler
	def_irq_handler	I2S1_IRQHandler
	def_irq_handler	MACN10_IRQHandler
	def_irq_handler	SC1_IRQHandler
	def_irq_handler	GPK_IRQHandler
	def_irq_handler	GPL_IRQHandler
	def_irq_handler	GPM_IRQHandler
	def_irq_handler	GPN_IRQHandler
	def_irq_handler	TMR8_IRQHandler
	def_irq_handler	TMR9_IRQHandler
	def_irq_handler	TMR10_IRQHandler
	def_irq_handler	TMR11_IRQHandler
	def_irq_handler	BRAKE2_IRQHandler
	def_irq_handler	EPWM2P0_IRQHandler
	def_irq_handler	EPWM2P1_IRQHandler
	def_irq_handler	EPWM2P2_IRQHandler
	def_irq_handler	QEI2_IRQHandler
	def_irq_handler	ECAP2_IRQHandler
	def_irq_handler	SPI2_IRQHandler
	def_irq_handler	SPI3_IRQHandler
	def_irq_handler	UART12_IRQHandler
	def_irq_handler	UART13_IRQHandler
	def_irq_handler	UART14_IRQHandler
	def_irq_handler	UART15_IRQHandler
	def_irq_handler	UART16_IRQHandler
	def_irq_handler	I2C4_IRQHandler
	def_irq_handler	I2C5_IRQHandler
	def_irq_handler	MCAN20_IRQHandler
	def_irq_handler	MCAN30_IRQHandler
	def_irq_handler	KPI_IRQHandler
	def_irq_handler	MCAN01_IRQHandler
	def_irq_handler	MCAN11_IRQHandler
	def_irq_handler	MCAN21_IRQHandler
	def_irq_handler	MCAN31_IRQHandler
	def_irq_handler	ADC0_IRQHandler
	.end
