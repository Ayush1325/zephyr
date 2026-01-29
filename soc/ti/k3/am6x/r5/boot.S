/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <offsets_short.h>
#include <cortex_a_r/tcm.h>

_ASM_FILE_PROLOGUE

GTEXT(__boot_spring)
SECTION_SUBSEC_FUNC(boot_section,_start,__boot_spring)
    ldr sp, =(z_arm_sys_stack + CONFIG_ARMV7_SYS_STACK_SIZE)
    bl MpuP_init
    b __start
