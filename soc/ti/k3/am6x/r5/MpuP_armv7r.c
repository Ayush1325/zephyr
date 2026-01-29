/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr/arch/arm/mpu/arm_mpu_mem_cfg.h>
#include <zephyr/arch/arm/cortex_a_r/mpu.h>
#include "../../../../../arch/arm/core/mpu/cortex_a_r/arm_mpu_internal.h"
#include "MpuP_armv7.h"

#define MPU_SECTION __attribute__((section(".boot_section")))

/* Max possible regions in ARMv7-R CPU */
#define MpuP_MAX_REGIONS (16u)

/* APIs defined in MpuP_armv7r_asm.s */
void MpuP_enableAsm(void);
uint32_t MpuP_isEnableAsm(void);
void MpuP_disableBRAsm(void);
void MpuP_enableBRAsm(void);

MpuP_Config gMpuConfig = {1, 1, 1}; // 2 regions, background region on, MPU on
MpuP_RegionConfig gMpuRegionConfig[] = {
	// DDR region
	{
		.baseAddr = 0x80000000,
		.size = MpuP_RegionSize_2G,
		.attrs =
			{
				.isEnable = 1,
				.isCacheable = 0,
				.isBufferable = 0,
				.isSharable = 0,
				.isExecuteNever = 0,
				.tex = 7,
				.accessPerm = MpuP_AP_ALL_RW,
				.subregionDisableMask = 0x0u,
			},
	},
};

static uint32_t MPU_SECTION MpuP_getAttrs(MpuP_RegionAttrs *region)
{
	uint32_t regionAttrs = ((uint32_t)(region->isExecuteNever & 0x1) << 12) |
			       ((uint32_t)(region->accessPerm & 0x7) << 8) |
			       ((uint32_t)(region->tex & 0x7) << 3) |
			       ((uint32_t)(region->isSharable & 0x1) << 2) |
			       ((uint32_t)(region->isCacheable & 0x1) << 1) |
			       ((uint32_t)(region->isBufferable & 0x1) << 0);

	return regionAttrs;
}

void MPU_SECTION MpuP_setRegion(uint32_t regionNum, void *addr, uint32_t size,
				MpuP_RegionAttrs *attrs)
{
	uint32_t baseAddress, sizeAndEnable, regionAttrs;
	uint32_t enabled;

	/* size 5b field */
	size = (size & 0x1F);

	/* If N is the value in size field, the region size is 2N+1 bytes. */
	sizeAndEnable = ((uint32_t)(attrs->subregionDisableMask & 0xFF) << 8) |
			((uint32_t)(size & 0x1F) << 1) | ((uint32_t)(attrs->isEnable & 0x1) << 0);

	/* align base address to region size */
	baseAddress = ((uint32_t)addr & ~((1 << ((uint64_t)size + 1)) - 1));

	/* get region attribute mask */
	regionAttrs = MpuP_getAttrs(attrs);

	enabled = MpuP_isEnableAsm();

	set_region_number(regionNum);
	set_region_base_address(baseAddress);
	set_region_size(sizeAndEnable);
	set_region_attributes(regionAttrs);

	if (enabled) {
		MpuP_enableAsm();
	}
}

void MPU_SECTION MpuP_init()
{
	uint32_t i;
	// uint32_t attr = MpuP_getAttrs(&gMpuRegionConfig[0].attrs);
	// struct arm_mpu_region temp = {
	// 	.name = "DDR",
	// 	.base = 0x80000000,
	// 	.size = REGION_2G,
	// 	.attr = {.rasr = attr},
	// };

	MpuP_disableBRAsm();

	// region_init(0, &temp);

	/*
	 * Initialize MPU regions
	 */
	for (i = 0; i < gMpuConfig.numRegions; i++) {
		MpuP_setRegion(i, (void *)gMpuRegionConfig[i].baseAddr, gMpuRegionConfig[i].size,
			       &gMpuRegionConfig[i].attrs);
	}

	if (gMpuConfig.enableBackgroundRegion) {
		MpuP_enableBRAsm();
	}

	if (gMpuConfig.enableMpu) {
		MpuP_enableAsm();
	}
}
