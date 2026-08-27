/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <kernel_internal.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/drivers/pm_cpu_ops.h>
#include <zephyr/platform/hooks.h>
#ifdef CONFIG_RISCV_S_MODE_EXTERNAL_SBI
#include <zephyr/arch/riscv/sbi.h>
#endif
#if defined(CONFIG_RISCV_IMSIC)
#include <zephyr/drivers/interrupt_controller/riscv_imsic.h>
#endif

volatile struct {
	arch_cpustart_t fn;
	void *arg;
} riscv_cpu_init[CONFIG_MP_MAX_NUM_CPUS];

volatile uintptr_t __noinit riscv_cpu_wake_flag;
volatile uintptr_t riscv_cpu_boot_flag;
volatile void *riscv_cpu_sp;

#ifdef CONFIG_RISCV_S_MODE
/* Written by the boot code in reset.S before the kernel reaches S-mode, where
 * mhartid can no longer be read. __noinit because it is set before BSS is
 * cleared.
 */
unsigned long __noinit riscv_boot_hartid;
#endif

extern void __start(void);

#ifdef CONFIG_RISCV_S_MODE_EXTERNAL_SBI
/* Tells the boot code in reset.S that a hart arriving at __initialize is one
 * we asked the SBI to start rather than the hart firmware released into the
 * payload. Set once, before the first hart_start request.
 */
volatile uintptr_t riscv_secondary_boot;
#endif

#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void);
#endif

void arch_cpu_start(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	riscv_cpu_init[cpu_num].fn = fn;
	riscv_cpu_init[cpu_num].arg = arg;

	riscv_cpu_sp = K_KERNEL_STACK_BUFFER(stack) + sz;
	riscv_cpu_boot_flag = 0U;

#ifdef CONFIG_RISCV_S_MODE_EXTERNAL_SBI
	/* Under an external SBI the secondary harts are not running at all:
	 * firmware parked them at reset and only releases them on request, so
	 * there is nobody to hand a wake flag to. Ask the SBI to start this one
	 * at __start, which does the per-hart setup (global pointer, trap
	 * vector) before reset.S routes it to the secondary path.
	 */
	riscv_secondary_boot = 1U;

	struct sbi_ret ret = sbi_ecall(SBI_EXT_HSM, SBI_FUNC_HART_START,
				       _kernel.cpus[cpu_num].arch.hartid,
				       (unsigned long)&__start, 0UL);

	if (ret.error != SBI_SUCCESS) {
		printk("Failed to boot secondary CPU %d: SBI error %ld\n", cpu_num, ret.error);
		return;
	}

	while (riscv_cpu_boot_flag == 0U) {
	}
#else
#ifdef CONFIG_PM_CPU_OPS
	if (pm_cpu_on(cpu_num, (uintptr_t)&__start)) {
		printk("Failed to boot secondary CPU %d\n", cpu_num);
		return;
	}
#endif

	while (riscv_cpu_boot_flag == 0U) {
		riscv_cpu_wake_flag = _kernel.cpus[cpu_num].arch.hartid;
	}
#endif /* CONFIG_RISCV_S_MODE_EXTERNAL_SBI */
}

void arch_secondary_cpu_init(int hartid)
{
	unsigned int i;
	unsigned int cpu_num = 0;

	for (i = 0; i < CONFIG_MP_MAX_NUM_CPUS; i++) {
		if (_kernel.cpus[i].arch.hartid == hartid) {
			cpu_num = i;
		}
	}
#ifdef CONFIG_RISCV_S_MODE
	csr_write(sscratch, &_kernel.cpus[cpu_num]);
#else
	csr_write(mscratch, &_kernel.cpus[cpu_num]);
#endif
#ifdef CONFIG_SMP
	_kernel.cpus[cpu_num].arch.online = true;
#endif
#if defined(CONFIG_MULTITHREADING) && defined(CONFIG_THREAD_LOCAL_STORAGE)
	__asm__("mv tp, %0" : : "r" (z_idle_threads[cpu_num].tls));
#endif
#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
	soc_interrupt_init();
#endif
#ifdef CONFIG_RISCV_PMP
	z_riscv_pmp_init();
#endif
#ifdef CONFIG_CUSTOM_STACK_GUARD
	z_riscv_custom_stack_guard_init();
#endif /* CONFIG_CUSTOM_STACK_GUARD */
#ifdef CONFIG_SMP
	irq_enable(RISCV_IRQ_SOFT);
#endif /* CONFIG_SMP */
#if defined(CONFIG_PLIC_IRQ_AFFINITY) || defined(CONFIG_RISCV_APLIC_DIRECT_IRQ_AFFINITY)
	/* Enable on secondary cores so that they can respond to PLIC */
	irq_enable(RISCV_IRQ_EXT);
#endif /* CONFIG_PLIC_IRQ_AFFINITY || CONFIG_RISCV_APLIC_DIRECT_IRQ_AFFINITY */
#if defined(CONFIG_RISCV_IMSIC) && defined(CONFIG_SMP)
	/* Initialize IMSIC on secondary CPU */
	z_riscv_imsic_secondary_init();
#endif /* CONFIG_RISCV_IMSIC && CONFIG_SMP */
	soc_per_core_init_hook();
	riscv_cpu_init[cpu_num].fn(riscv_cpu_init[cpu_num].arg);
}
