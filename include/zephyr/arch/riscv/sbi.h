/*
 * Copyright (c) 2026 Alexios Lyrakis <alexios.lyrakis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RISC-V Supervisor Binary Interface (SBI) definitions.
 *
 * Defines SBI extension IDs, function IDs, and error codes used by
 * S-mode code to request M-mode firmware services via the @c ecall
 * instruction.  The subset defined here covers the extensions used by
 * Zephyr's in-tree minimal SBI runtime (@c arch/riscv/core/sbi.S) plus
 * the ones Zephyr requests from an external SBI implementation such as
 * OpenSBI (IPI and HSM, used to bring up and signal secondary harts).
 *
 * References: RISC-V SBI Specification v2.0
 * (https://github.com/riscv-non-isa/riscv-sbi-doc)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_SBI_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_SBI_H_

/** @brief SBI extension ID for the Timer extension (TIME) */
#define SBI_EXT_TIME			0x54494D45

/** @brief SBI_EXT_TIME function ID: set the next timer deadline */
#define SBI_FUNC_SET_TIMER		0

/** @brief SBI extension ID for the System Reset extension (SRST) */
#define SBI_EXT_SRST			0x53525354

/** @brief SBI_EXT_SRST function ID: reset or power off the system */
#define SBI_FUNC_SYSTEM_RESET		0

/** @brief SBI_EXT_SRST reset type: clean shutdown (power off) */
#define SBI_SRST_RESET_TYPE_SHUTDOWN	0
/** @brief SBI_EXT_SRST reset type: cold reboot */
#define SBI_SRST_RESET_TYPE_COLD_REBOOT	1
/** @brief SBI_EXT_SRST reset type: warm reboot */
#define SBI_SRST_RESET_TYPE_WARM_REBOOT	2

/** @brief SBI_EXT_SRST reset reason: no specific reason */
#define SBI_SRST_RESET_REASON_NONE	0

/** @brief SBI extension ID for the IPI extension (sPI) */
#define SBI_EXT_IPI			0x735049

/** @brief SBI_EXT_IPI function ID: send an IPI to a set of harts */
#define SBI_FUNC_SEND_IPI		0

/** @brief SBI extension ID for the Hart State Management extension (HSM) */
#define SBI_EXT_HSM			0x48534D

/** @brief SBI_EXT_HSM function ID: start a stopped hart */
#define SBI_FUNC_HART_START		0
/** @brief SBI_EXT_HSM function ID: stop the calling hart */
#define SBI_FUNC_HART_STOP		1
/** @brief SBI_EXT_HSM function ID: query the state of a hart */
#define SBI_FUNC_HART_GET_STATUS	2

/** @brief SBI return code: call completed successfully */
#define SBI_SUCCESS			0
/** @brief SBI return code: requested extension/function is not available */
#define SBI_ERR_NOT_SUPPORTED		-1

#ifndef _ASMLANGUAGE

/** @brief Values returned in a0 (error) and a1 (value) by an SBI call. */
struct sbi_ret {
	long error;
	long value;
};

/**
 * @brief Issue an SBI call to the M-mode firmware.
 *
 * Passes the extension ID in a7 and the function ID in a6 per the SBI v0.2+
 * calling convention, and returns the (error, value) pair the firmware leaves
 * in a0/a1.
 *
 * @param ext  SBI extension ID (@c SBI_EXT_*)
 * @param fid  Function ID within that extension (@c SBI_FUNC_*)
 * @param arg0 First argument, passed in a0
 * @param arg1 Second argument, passed in a1
 * @param arg2 Third argument, passed in a2
 *
 * @return The firmware's error/value pair; @c error is @c SBI_SUCCESS on
 *         success and a negative SBI error code otherwise.
 */
static inline struct sbi_ret sbi_ecall(unsigned long ext, unsigned long fid,
				       unsigned long arg0, unsigned long arg1,
				       unsigned long arg2)
{
	register unsigned long a0 __asm__("a0") = arg0;
	register unsigned long a1 __asm__("a1") = arg1;
	register unsigned long a2 __asm__("a2") = arg2;
	register unsigned long a6 __asm__("a6") = fid;
	register unsigned long a7 __asm__("a7") = ext;

	__asm__ volatile("ecall"
			 : "+r"(a0), "+r"(a1)
			 : "r"(a2), "r"(a6), "r"(a7)
			 : "memory");

	return (struct sbi_ret){ .error = (long)a0, .value = (long)a1 };
}

#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV_SBI_H_ */
