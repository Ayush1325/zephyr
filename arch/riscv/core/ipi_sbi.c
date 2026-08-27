/*
 * Copyright (c) 2021 Intel Corporation
 * Copyright (c) 2026 Ayush Singh <ayushsingh1325@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Scheduler IPIs for a kernel running in S-mode under an external SBI.
 *
 * The CLINT MSIP registers the M-mode implementation pokes are an M-mode
 * resource: firmware leaves them out of the PMP regions granted to S/U, so
 * touching them from here faults. Ask the SBI to raise the supervisor software
 * interrupt on our behalf instead. Unlike MSIP, SSIP *is* writable from S-mode
 * through the sip CSR, so the receiving hart can clear it itself.
 */

#include <zephyr/kernel.h>
#include <kernel_arch_interface.h>
#include <ipi.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/arch/riscv/sbi.h>

static atomic_val_t cpu_pending_ipi[CONFIG_MP_MAX_NUM_CPUS];
#define IPI_SCHED     0
#define IPI_FPU_FLUSH 1

static void send_ipi(unsigned int cpu)
{
	/* One call per target hart, addressed as a single-bit mask based at
	 * that hart id, so hart ids beyond the width of a mask word are fine.
	 */
	sbi_ecall(SBI_EXT_IPI, SBI_FUNC_SEND_IPI, 1UL, _kernel.cpus[cpu].arch.hartid, 0UL);
}

void arch_sched_directed_ipi(uint32_t cpu_bitmap)
{
	unsigned int key = arch_irq_lock();
	unsigned int id = _current_cpu->id;
	unsigned int num_cpus = arch_num_cpus();

	for (unsigned int i = 0; i < num_cpus; i++) {
		if ((i != id) && _kernel.cpus[i].arch.online && ((cpu_bitmap & BIT(i)) != 0)) {
			atomic_set_bit(&cpu_pending_ipi[i], IPI_SCHED);
			send_ipi(i);
		}
	}

	arch_irq_unlock(key);
}

#ifdef CONFIG_FPU_SHARING
void arch_flush_fpu_ipi(unsigned int cpu)
{
	atomic_set_bit(&cpu_pending_ipi[cpu], IPI_FPU_FLUSH);
	send_ipi(cpu);
}
#endif /* CONFIG_FPU_SHARING */

static void sched_ipi_handler(const void *unused)
{
	ARG_UNUSED(unused);

	csr_clear(sip, SIP_SSIP);

	atomic_val_t pending_ipi = atomic_clear(&cpu_pending_ipi[_current_cpu->id]);

	if (pending_ipi & ATOMIC_MASK(IPI_SCHED)) {
		z_sched_ipi();
	}
#ifdef CONFIG_FPU_SHARING
	if (pending_ipi & ATOMIC_MASK(IPI_FPU_FLUSH)) {
		/* disable IRQs */
		csr_clear(sstatus, SSTATUS_SIE);
		/* perform the flush */
		arch_flush_local_fpu();
		/*
		 * No need to re-enable IRQs here as long as
		 * this remains the last case.
		 */
	}
#endif /* CONFIG_FPU_SHARING */
}

#ifdef CONFIG_FPU_SHARING
/*
 * Make sure there is no pending FPU flush request for this CPU while
 * waiting for a contended spinlock to become available. This prevents
 * a deadlock when the lock we need is already taken by another CPU
 * that also wants its FPU content to be reinstated while such content
 * is still live in this CPU's FPU.
 */
void arch_spin_relax(void)
{
	atomic_val_t *pending_ipi = &cpu_pending_ipi[_current_cpu->id];

	if (atomic_test_and_clear_bit(pending_ipi, IPI_FPU_FLUSH)) {
		/*
		 * We may not be in IRQ context here hence cannot use
		 * arch_flush_local_fpu() directly. Use the helper that
		 * flushes the FPU context without altering thread options.
		 */
		z_riscv_fpu_flush_thread(_current_cpu->arch.fpu_owner);
	}
}
#endif /* CONFIG_FPU_SHARING */

int arch_smp_init(void)
{
	IRQ_CONNECT(RISCV_IRQ_SOFT, 0, sched_ipi_handler, NULL, 0);
	irq_enable(RISCV_IRQ_SOFT);

	return 0;
}
