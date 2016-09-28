/*
 * Copyright (C) 2016 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "a5xx_gpu.h"


extern bool hang_debug;
static void a5xx_dump(struct msm_gpu *gpu);

/*
 * Firmware helpers for PFP and ME (aka PM4).. GPMU is something different.
 */

static int fw_upload(struct drm_gem_object *obj, const struct firmware *fw)
{
	void *ptr;

	ptr = msm_gem_get_vaddr_locked(obj);
	if (IS_ERR(ptr)) {
		dev_err(obj->dev->dev, "map failed: %d\n", (int)PTR_ERR(ptr));
		return PTR_ERR(ptr);
	}

	memcpy(ptr, &fw->data[4], fw->size - 4);
	msm_gem_put_vaddr_locked(obj);

	return 0;
}

static uint32_t fw_version(const struct firmware *fw)
{
	return ((uint32_t *)fw->data)[1];
}

/*
 * hw setup:
 */

static void a5xx_enable_64b(struct msm_gpu *gpu)
{
	/* I guess writing 0x0 would disable (for userptr/HSA type
	 * stuff w/ a 32b userspace?? Though not really sure if we
	 * can change this dynamically or need to reboot GPU..
	 */
	gpu_write(gpu, REG_A5XX_CP_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_VSC_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_GRAS_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_RB_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_PC_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_HLSQ_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_VFD_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_VPC_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_UCHE_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_SP_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_TPL1_ADDR_MODE_CNTL, 0x1);
	gpu_write(gpu, REG_A5XX_RBBM_SECVID_TSB_ADDR_MODE_CNTL, 0x1);
}

static const struct hwcg_regs {
	unsigned offset;
	unsigned val;
} a530_hwcg_regs[] = {
	{REG_A5XX_RBBM_CLOCK_CNTL_SP0, 0x02222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_SP1, 0x02222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_SP2, 0x02222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_SP3, 0x02222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_SP0, 0x02222220},
	{REG_A5XX_RBBM_CLOCK_CNTL2_SP1, 0x02222220},
	{REG_A5XX_RBBM_CLOCK_CNTL2_SP2, 0x02222220},
	{REG_A5XX_RBBM_CLOCK_CNTL2_SP3, 0x02222220},
	{REG_A5XX_RBBM_CLOCK_HYST_SP0, 0x0000F3CF},
	{REG_A5XX_RBBM_CLOCK_HYST_SP1, 0x0000F3CF},
	{REG_A5XX_RBBM_CLOCK_HYST_SP2, 0x0000F3CF},
	{REG_A5XX_RBBM_CLOCK_HYST_SP3, 0x0000F3CF},
	{REG_A5XX_RBBM_CLOCK_DELAY_SP0, 0x00000080},
	{REG_A5XX_RBBM_CLOCK_DELAY_SP1, 0x00000080},
	{REG_A5XX_RBBM_CLOCK_DELAY_SP2, 0x00000080},
	{REG_A5XX_RBBM_CLOCK_DELAY_SP3, 0x00000080},
	{REG_A5XX_RBBM_CLOCK_CNTL_TP0, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_TP1, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_TP2, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_TP3, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_TP0, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_TP1, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_TP2, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_TP3, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL3_TP0, 0x00002222},
	{REG_A5XX_RBBM_CLOCK_CNTL3_TP1, 0x00002222},
	{REG_A5XX_RBBM_CLOCK_CNTL3_TP2, 0x00002222},
	{REG_A5XX_RBBM_CLOCK_CNTL3_TP3, 0x00002222},
	{REG_A5XX_RBBM_CLOCK_HYST_TP0, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST_TP1, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST_TP2, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST_TP3, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST2_TP0, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST2_TP1, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST2_TP2, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST2_TP3, 0x77777777},
	{REG_A5XX_RBBM_CLOCK_HYST3_TP0, 0x00007777},
	{REG_A5XX_RBBM_CLOCK_HYST3_TP1, 0x00007777},
	{REG_A5XX_RBBM_CLOCK_HYST3_TP2, 0x00007777},
	{REG_A5XX_RBBM_CLOCK_HYST3_TP3, 0x00007777},
	{REG_A5XX_RBBM_CLOCK_DELAY_TP0, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY_TP1, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY_TP2, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY_TP3, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY2_TP0, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY2_TP1, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY2_TP2, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY2_TP3, 0x11111111},
	{REG_A5XX_RBBM_CLOCK_DELAY3_TP0, 0x00001111},
	{REG_A5XX_RBBM_CLOCK_DELAY3_TP1, 0x00001111},
	{REG_A5XX_RBBM_CLOCK_DELAY3_TP2, 0x00001111},
	{REG_A5XX_RBBM_CLOCK_DELAY3_TP3, 0x00001111},
	{REG_A5XX_RBBM_CLOCK_CNTL_UCHE, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_UCHE, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL3_UCHE, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL4_UCHE, 0x00222222},
	{REG_A5XX_RBBM_CLOCK_HYST_UCHE, 0x00444444},
	{REG_A5XX_RBBM_CLOCK_DELAY_UCHE, 0x00000002},
	{REG_A5XX_RBBM_CLOCK_CNTL_RB0, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_RB1, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_RB2, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_RB3, 0x22222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_RB0, 0x00222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_RB1, 0x00222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_RB2, 0x00222222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_RB3, 0x00222222},
	{REG_A5XX_RBBM_CLOCK_CNTL_CCU0, 0x00022220},
	{REG_A5XX_RBBM_CLOCK_CNTL_CCU1, 0x00022220},
	{REG_A5XX_RBBM_CLOCK_CNTL_CCU2, 0x00022220},
	{REG_A5XX_RBBM_CLOCK_CNTL_CCU3, 0x00022220},
	{REG_A5XX_RBBM_CLOCK_CNTL_RAC, 0x05522222},
	{REG_A5XX_RBBM_CLOCK_CNTL2_RAC, 0x00505555},
	{REG_A5XX_RBBM_CLOCK_HYST_RB_CCU0, 0x04040404},
	{REG_A5XX_RBBM_CLOCK_HYST_RB_CCU1, 0x04040404},
	{REG_A5XX_RBBM_CLOCK_HYST_RB_CCU2, 0x04040404},
	{REG_A5XX_RBBM_CLOCK_HYST_RB_CCU3, 0x04040404},
	{REG_A5XX_RBBM_CLOCK_HYST_RAC, 0x07444044},
	{REG_A5XX_RBBM_CLOCK_DELAY_RB_CCU_L1_0, 0x00000002},
	{REG_A5XX_RBBM_CLOCK_DELAY_RB_CCU_L1_1, 0x00000002},
	{REG_A5XX_RBBM_CLOCK_DELAY_RB_CCU_L1_2, 0x00000002},
	{REG_A5XX_RBBM_CLOCK_DELAY_RB_CCU_L1_3, 0x00000002},
	{REG_A5XX_RBBM_CLOCK_DELAY_RAC, 0x00010011},
	{REG_A5XX_RBBM_CLOCK_CNTL_TSE_RAS_RBBM, 0x04222222},
	{REG_A5XX_RBBM_CLOCK_MODE_GPC, 0x02222222},
	{REG_A5XX_RBBM_CLOCK_MODE_VFD, 0x00002222},
	{REG_A5XX_RBBM_CLOCK_HYST_TSE_RAS_RBBM, 0x00000000},
	{REG_A5XX_RBBM_CLOCK_HYST_GPC, 0x04104004},
	{REG_A5XX_RBBM_CLOCK_HYST_VFD, 0x00000000},
	{REG_A5XX_RBBM_CLOCK_DELAY_HLSQ, 0x00000000},
	{REG_A5XX_RBBM_CLOCK_DELAY_TSE_RAS_RBBM, 0x00004000},
	{REG_A5XX_RBBM_CLOCK_DELAY_GPC, 0x00000200},
	{REG_A5XX_RBBM_CLOCK_DELAY_VFD, 0x00002222},
	{~0} /* sentinel */
};

/*
 * a5xx_enable_hwcg() - Program the clock control registers
 * @device: The adreno device pointer
 */
static void a5xx_enable_set(struct msm_gpu *gpu, bool enable)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	const struct hwcg_regs *regs;
	int i;

	if (adreno_is_a530(adreno_gpu)) {
		regs = a530_hwcg_regs;
	} else {
		BUG();
	}

	for (i = 0; regs[i].offset != ~0; i++)
		gpu_write(gpu, regs[i].offset, enable ? regs[i].val : 0);

	/* enable top level HWCG */
	gpu_write(gpu, REG_A5XX_RBBM_CLOCK_CNTL, enable ? 0xAAA8AA00 : 0);
	gpu_write(gpu, REG_A5XX_RBBM_ISDB_CNT, enable ? 0x00000182 : 0x00000180);
}

static void a5xx_me_init(struct msm_gpu *gpu)
{
	struct msm_ringbuffer *ring = gpu->rb;

	OUT_PKT7(ring, CP_ME_INIT, 7);
	/* Mask -- look for all ordinals but drawcall range and reset
	 * ucode scratch memory.
	 */
	OUT_RING(ring, 0x0000000f);
	/* Use both contexts for 3D (bit0) 2D (bit1) */
	OUT_RING(ring, 0x00000003);
	/* Enable register protection */
	OUT_RING(ring, 0x20000000);
	/* Header dump address */
	OUT_RING(ring, 0x00000000);
	/* Header dump enable and dump size */
	OUT_RING(ring, 0x00000000);
	/* Below will be ignored by the CP unless bit4 in Mask is set */
	OUT_RING(ring, 0x00000000);
	OUT_RING(ring, 0x00000000);

	gpu->funcs->flush(gpu);
	gpu->funcs->idle(gpu);

	// XXX critical packets??

	/* GPU comes up in secured mode, make it unsecured by default */
	if (to_adreno_gpu(gpu)->info->features & ADRENO_CONTENT_PROTECTION) {
		OUT_PKT7(ring, CP_SET_SECURE_MODE, 1);
		OUT_RING(ring, 0);

		gpu->funcs->flush(gpu);
		gpu->funcs->idle(gpu);
	}
}

static void a5xx_gpmu_init(struct msm_gpu *gpu)
{
	/* TODO */
}

static int a5xx_hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	int ret;

	a5xx_enable_64b(gpu);

	if (adreno_is_a530(adreno_gpu)) {
		gpu_write(gpu, REG_A5XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x00000003);
	} else {
		BUG();
	}

	/* Make all blocks contribute to the GPU BUSY perf counter */
	gpu_write(gpu, REG_A5XX_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffff);

	/*
	 * Enable the RBBM error reporting bits.  This lets us get
	 * useful information on failure
	 */
	gpu_write(gpu, REG_A5XX_RBBM_AHB_CNTL0, 0x00000001);

#if 0  /* TODO .. for now just rely on sw hangcheck */
	/*
	 * Turn on hang detection for a530 v2 and beyond. This spews a
	 * lot of useful information into the RBBM registers on a hang.
	 *
	 * We have 4 RB units, and only RB0 activity signals are working
	 * correctly. Mask out RB1-3 activity signals from the HW hang
	 * detection logic as per recommendation of hardware team.
	 */
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL11, 0xf0000000);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL12, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL13, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL14, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL15, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL16, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL17, 0xffffffff);
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_MASK_CNTL18, 0xffffffff);

//	gpudev->irq->mask |= (1 << A5XX_INT_MISC_HANG_DETECT);

	/*
	 * Set hang detection threshold to 1 million cycles
	 * (0xFFFF*16)
	 */
	gpu_write(gpu, REG_A5XX_RBBM_INTERFACE_HANG_INT_CNTL, (1 << 30) | 0xffff);
#endif

	/* Turn on performance counters */
	gpu_write(gpu, REG_A5XX_RBBM_PERFCTR_CNTL, 0x01);

	/*
	 * This is to increase performance by restricting VFD's cache access,
	 * so that LRZ and other data get evicted less.
	 */
//	gpu_write(gpu, REG_A5XX_UCHE_CACHE_WAYS, 0x02);

	/*
	 * Set UCHE_WRITE_THRU_BASE to the UCHE_TRAP_BASE effectively
	 * disabling L2 bypass
	 */
	gpu_write(gpu, REG_A5XX_UCHE_TRAP_BASE_LO, 0xffff0000);
	gpu_write(gpu, REG_A5XX_UCHE_TRAP_BASE_HI, 0x0001ffff);
	gpu_write(gpu, REG_A5XX_UCHE_WRITE_THRU_BASE_LO, 0xffff0000);
	gpu_write(gpu, REG_A5XX_UCHE_WRITE_THRU_BASE_HI, 0x0001ffff);

#define ADRENO_UCHE_GMEM_BASE	0x100000
	/* Program the GMEM VA range for the UCHE path */
	gpu_write(gpu, REG_A5XX_UCHE_GMEM_RANGE_MIN_LO, ADRENO_UCHE_GMEM_BASE);
	gpu_write(gpu, REG_A5XX_UCHE_GMEM_RANGE_MIN_HI, 0x0);
	gpu_write(gpu, REG_A5XX_UCHE_GMEM_RANGE_MAX_LO,
			ADRENO_UCHE_GMEM_BASE + adreno_gpu->info->gmem - 1);
	gpu_write(gpu, REG_A5XX_UCHE_GMEM_RANGE_MAX_HI, 0x0);

	if (adreno_is_a530(adreno_gpu)) {
		gpu_write(gpu, REG_A5XX_CP_MEQ_THRESHOLDS, 0x40);
		gpu_write(gpu, REG_A5XX_CP_MERCIU_SIZE, 0x40);
		gpu_write(gpu, REG_A5XX_CP_ROQ_THRESHOLDS_2, 0x80000060);
		gpu_write(gpu, REG_A5XX_CP_ROQ_THRESHOLDS_1, 0x40201B16);
		gpu_write(gpu, REG_A5XX_PC_DBG_ECO_CNTL, (0x400 << 11 | 0x300 << 22));
	} else {
		BUG();
	}

	if (adreno_gpu->info->features & ADRENO_QUIRK_TWO_PASS_USE_WFI) {
		/*
		 * Set TWOPASSUSEWFI in A5XX_PC_DBG_ECO_CNTL for
		 * microcodes after v77
		 */
		if (fw_version(adreno_gpu->pfp) >= 0x5ff077) {
			uint32_t val = gpu_read(gpu, REG_A5XX_PC_DBG_ECO_CNTL);
			val |= A5XX_PC_DBG_ECO_CNTL_TWOPASSUSEWFI;
			gpu_write(gpu, REG_A5XX_PC_DBG_ECO_CNTL, val);
		}
	}

	/* Set the USE_RETENTION_FLOPS chicken bit */
	gpu_write(gpu, REG_A5XX_CP_CHICKEN_DBG, 0x02000000);

	/* if not in ISDB mode enable ME/PFP split notification */
	/* NOTE: ISDB == Integrated Shader Debugger */
	gpu_write(gpu, REG_A5XX_RBBM_AHB_CNTL1, 0xA6FFFFFF);

	/* enable HWCG */
	a5xx_enable_set(gpu, true);

	gpu_write(gpu, REG_A5XX_RBBM_AHB_CNTL2, 0x0000003f);

	/* enable access protection to privileged registers */
	gpu_write(gpu, REG_A5XX_CP_PROTECT_CNTL, 0x00000007);

#define set_protect(idx, reg, masklen) do {                        \
		gpu_write(gpu, REG_A5XX_CP_PROTECT_REG(idx),        \
			A5XX_CP_PROTECT_REG_TRAP_WRITE |            \
			A5XX_CP_PROTECT_REG_TRAP_READ |             \
			A5XX_CP_PROTECT_REG_MASK_LEN(masklen) |     \
			A5XX_CP_PROTECT_REG_BASE_ADDR((reg) << 2)); \
	} while (0)

	/* RBBM registers */
	set_protect(0, 0x4, 2);
	set_protect(1, 0x8, 3);
	set_protect(2, 0x10, 4);
	set_protect(3, 0x20, 5);
	set_protect(4, 0x40, 6);
	set_protect(5, 0x80, 6);

	/* Content protection registers */
	set_protect(6, REG_A5XX_RBBM_SECVID_TSB_TRUSTED_BASE_LO, 4);
	set_protect(7, REG_A5XX_RBBM_SECVID_TRUST_CNTL, 1);

	/* CP registers */
	set_protect(8, 0x800, 6);
	set_protect(9, 0x840, 3);
	set_protect(10, 0x880, 5);
	set_protect(11, 0x0AA0, 0);

	/* RB registers */
	set_protect(12, 0xCC0, 0);
	set_protect(13, 0xCF0, 1);

	/* VPC registers */
	set_protect(14, 0xE68, 3);
	set_protect(15, 0xE70, 4);

	/* UCHE registers */
	set_protect(16, 0xE87, 4);

	// XXX are we alive?
	gpu_write(gpu, REG_AXXX_CP_SCRATCH_REG2, 0x123);
	DBG("scratch2: %x", gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG2));

	ret = adreno_hw_init(gpu);
	if (ret)
		return ret;

	/* Load PM4: */
	gpu_write(gpu, REG_A5XX_CP_PM4_INSTR_BASE_LO,
			lower_32_bits(a5xx_gpu->pm4_iova));
	gpu_write(gpu, REG_A5XX_CP_PM4_INSTR_BASE_HI,
			upper_32_bits(a5xx_gpu->pm4_iova));

	/* Load PFP: */
	gpu_write(gpu, REG_A5XX_CP_PFP_INSTR_BASE_LO,
			lower_32_bits(a5xx_gpu->pfp_iova));
	gpu_write(gpu, REG_A5XX_CP_PFP_INSTR_BASE_HI,
			upper_32_bits(a5xx_gpu->pfp_iova));

	/* clear ME_HALT to start micro engine */
	gpu_write(gpu, REG_A5XX_CP_ME_CNTL, 0);

	a5xx_me_init(gpu);
	a5xx_gpmu_init(gpu);

	/*
	 * Send a pipeline stat event whenever the GPU gets powered up
	 * to cause misbehaving perf counters to start ticking
	 */
	if (adreno_is_a530(adreno_gpu)) {
		struct msm_ringbuffer *ring = gpu->rb;

		OUT_PKT7(ring, CP_EVENT_WRITE, 1);
		OUT_RING(ring, STAT_EVENT);

		gpu->funcs->flush(gpu);
		gpu->funcs->idle(gpu);
	}

	return 0;
}

static void a5xx_recover(struct msm_gpu *gpu)
{
	adreno_dump_info(gpu);

	/* dump registers before resetting gpu, if enabled: */
	if (hang_debug)
		a5xx_dump(gpu);

	gpu_write(gpu, REG_A5XX_RBBM_SW_RESET_CMD, 1);
	gpu_read(gpu, REG_A5XX_RBBM_SW_RESET_CMD);
	gpu_write(gpu, REG_A5XX_RBBM_SW_RESET_CMD, 0);

	adreno_recover(gpu);
}

#if 0
/* I suppose this is really a5xx+ */
static void a5xx_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit,
		struct msm_file_private *ctx)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct msm_drm_private *priv = gpu->dev->dev_private;
	struct msm_ringbuffer *ring = gpu->rb;
	unsigned i;

	for (i = 0; i < submit->nr_cmds; i++) {
		switch (submit->cmd[i].type) {
		case MSM_SUBMIT_CMD_IB_TARGET_BUF:
			/* ignore IB-targets */
			break;
		case MSM_SUBMIT_CMD_CTX_RESTORE_BUF:
			/* ignore if there has not been a ctx switch: */
			if (priv->lastctx == ctx)
				break;
		case MSM_SUBMIT_CMD_BUF:
			OUT_PKT7(ring, CP_INDIRECT_BUFFER_PFE, 3);
			OUT_RING(ring, lower_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, upper_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, submit->cmd[i].size);
			break;
		}
	}

	OUT_PKT4(ring, REG_AXXX_CP_SCRATCH_REG2, 1);
	OUT_RING(ring, submit->fence->seqno);

//	if (adreno_is_a3xx(adreno_gpu) || adreno_is_a4xx(adreno_gpu)) {
//		/* Flush HLSQ lazy updates to make sure there is nothing
//		 * pending for indirect loads after the timestamp has
//		 * passed:
//		 */
//		OUT_PKT7(ring, CP_EVENT_WRITE, 1);
//		OUT_RING(ring, HLSQ_FLUSH);
//
//		OUT_PKT7(ring, CP_WAIT_FOR_IDLE, 1);
//		OUT_RING(ring, 0x00000000);
//	}

// XXX probably wants an extra dword for upper 32b??
	OUT_PKT7(ring, CP_EVENT_WRITE, 3);
	OUT_RING(ring, CACHE_FLUSH_TS);
	OUT_RING(ring, rbmemptr(adreno_gpu, fence));
	OUT_RING(ring, submit->fence->seqno);

	/* we could maybe be clever and only CP_COND_EXEC the interrupt: */
	OUT_PKT7(ring, CP_INTERRUPT, 1);
	OUT_RING(ring, 0x80000000);

	gpu->funcs->flush(gpu);
}
#endif

static void a5xx_destroy(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);

	DBG("%s", gpu->name);

	if (a5xx_gpu->pfp_bo) {
		if (a5xx_gpu->pfp_iova)
			msm_gem_put_iova(a5xx_gpu->pfp_bo, gpu->id);
		drm_gem_object_unreference_unlocked(a5xx_gpu->pfp_bo);
	}

	if (a5xx_gpu->pm4_bo) {
		if (a5xx_gpu->pm4_iova)
			msm_gem_put_iova(a5xx_gpu->pm4_bo, gpu->id);
		drm_gem_object_unreference_unlocked(a5xx_gpu->pm4_bo);
	}

	adreno_gpu_cleanup(adreno_gpu);

	kfree(a5xx_gpu);
}

static void a5xx_idle(struct msm_gpu *gpu)
{
	/* wait for ringbuffer to drain: */
	adreno_idle(gpu);

	/* then wait for GPU to finish: */
	if (spin_until(!(gpu_read(gpu, REG_A5XX_RBBM_STATUS) & 0xfffffffe)))
		DRM_ERROR("%s: timeout waiting for GPU to idle!\n", gpu->name);

	/* TODO maybe we need to reset GPU here to recover from hang? */
}

static irqreturn_t a5xx_irq(struct msm_gpu *gpu)
{
	uint32_t status;

	status = gpu_read(gpu, REG_A5XX_RBBM_INT_0_STATUS);
	DBG("%s: Int status %08x", gpu->name, status);

	// XXX
	printk("%s: Int status: %08x\n", gpu->name, status);

	gpu_write(gpu, REG_A5XX_RBBM_INT_CLEAR_CMD, status);

	msm_gpu_retire(gpu);

	return IRQ_HANDLED;
}

static const unsigned int a5xx_registers[] = {
	/* RBBM */
	0x0000, 0x0002, 0x0004, 0x0020, 0x0022, 0x0026, 0x0029, 0x002B,
	0x002E, 0x0035, 0x0038, 0x0042, 0x0044, 0x0044, 0x0047, 0x0095,
	0x0097, 0x00BB, 0x03A0, 0x0464, 0x0469, 0x046F, 0x04D2, 0x04D3,
	0x04E0, 0x0533, 0x0540, 0x0555, 0xF400, 0xF400, 0xF800, 0xF807,
	/* CP */
	0x0800, 0x081A, 0x081F, 0x0841, 0x0860, 0x0860, 0x0880, 0x08A0,
	0x0B00, 0x0B12, 0x0B15, 0x0B28, 0x0B78, 0x0B7F, 0x0BB0, 0x0BBD,
	/* VSC */
	0x0BC0, 0x0BC6, 0x0BD0, 0x0C53, 0x0C60, 0x0C61,
	/* GRAS */
	0x0C80, 0x0C82, 0x0C84, 0x0C85, 0x0C90, 0x0C98, 0x0CA0, 0x0CA0,
	0x0CB0, 0x0CB2, 0x2180, 0x2185, 0x2580, 0x2585,
	/* RB */
	0x0CC1, 0x0CC1, 0x0CC4, 0x0CC7, 0x0CCC, 0x0CCC, 0x0CD0, 0x0CD8,
	0x0CE0, 0x0CE5, 0x0CE8, 0x0CE8, 0x0CEC, 0x0CF1, 0x0CFB, 0x0D0E,
	0x2100, 0x211E, 0x2140, 0x2145, 0x2500, 0x251E, 0x2540, 0x2545,
	/* PC */
	0x0D10, 0x0D17, 0x0D20, 0x0D23, 0x0D30, 0x0D30, 0x20C0, 0x20C0,
	0x24C0, 0x24C0,
	/* VFD */
	0x0E40, 0x0E43, 0x0E4A, 0x0E4A, 0x0E50, 0x0E57,
	/* VPC */
	0x0E60, 0x0E7C,
	/* UCHE */
	0x0E80, 0x0E8E, 0x0E90, 0x0E96, 0xEA0, 0xEA8, 0xEB0, 0xEB2,

	/* RB CTX 0 */
	0xE140, 0xE147, 0xE150, 0xE187, 0xE1A0, 0xE1A9, 0xE1B0, 0xE1B6,
	0xE1C0, 0xE1C7, 0xE1D0, 0xE1D1, 0xE200, 0xE201, 0xE210, 0xE21C,
	0xE240, 0xE268,
	/* GRAS CTX 0 */
	0xE000, 0xE006, 0xE010, 0xE09A, 0xE0A0, 0xE0A4, 0xE0AA, 0xE0EB,
	0xE100, 0xE105,
	/* PC CTX 0 */
	0xE380, 0xE38F, 0xE3B0, 0xE3B0,
	/* VFD CTX 0 */
	0xE400, 0xE405, 0xE408, 0xE4E9, 0xE4F0, 0xE4F0,
	/* VPC CTX 0 */
	0xE280, 0xE280, 0xE282, 0xE2A3, 0xE2A5, 0xE2C2,

	/* RB CTX 1 */
	0xE940, 0xE947, 0xE950, 0xE987, 0xE9A0, 0xE9A9, 0xE9B0, 0xE9B6,
	0xE9C0, 0xE9C7, 0xE9D0, 0xE9D1, 0xEA00, 0xEA01, 0xEA10, 0xEA1C,
	0xEA40, 0xEA68,
	/* GRAS CTX 1 */
	0xE800, 0xE806, 0xE810, 0xE89A, 0xE8A0, 0xE8A4, 0xE8AA, 0xE8EB,
	0xE900, 0xE905,
	/* PC CTX 1 */
	0xEB80, 0xEB8F, 0xEBB0, 0xEBB0,
	/* VFD CTX 1 */
	0xEC00, 0xEC05, 0xEC08, 0xECE9, 0xECF0, 0xECF0,
	/* VPC CTX 1 */
	0xEA80, 0xEA80, 0xEA82, 0xEAA3, 0xEAA5, 0xEAC2,
	/* GPMU */
	0xA800, 0xA8FF, 0xAC60, 0xAC60,
	/* DPM */
	0xB000, 0xB97F, 0xB9A0, 0xB9BF,

	~0 /* sentinel */
};

#ifdef CONFIG_DEBUG_FS
static void a5xx_show(struct msm_gpu *gpu, struct seq_file *m)
{
	gpu->funcs->pm_resume(gpu);
	seq_printf(m, "status:   %08x\n",
			gpu_read(gpu, REG_A5XX_RBBM_STATUS));
	gpu->funcs->pm_suspend(gpu);

	adreno_show(gpu, m);

}
#endif

#define REG(name, reg) REG_ADRENO_DEFINE(REG_ADRENO_ ## name, REG_A5XX_ ## reg)

/* Register offset defines for A5XX, in order of enum REG_ADRENOs */
static const unsigned int a5xx_register_offsets[REG_ADRENO_REGISTER_MAX] = {
	REG(CP_WFI_PEND_CTR, CP_WFI_PEND_CTR),
	REG(CP_RB_BASE, CP_RB_BASE),
	REG(CP_RB_BASE_HI, CP_RB_BASE_HI),
	REG(CP_RB_RPTR, CP_RB_RPTR),
	REG(CP_RB_WPTR, CP_RB_WPTR),
//	REG(CP_CNTL, CP_CNTL),
	REG(CP_ME_CNTL, CP_ME_CNTL),
	REG(CP_RB_CNTL, CP_RB_CNTL),
	REG(CP_IB1_BASE, CP_IB1_BASE),
	REG(CP_IB1_BASE_HI, CP_IB1_BASE_HI),
	REG(CP_IB1_BUFSZ, CP_IB1_BUFSZ),
	REG(CP_IB2_BASE, CP_IB2_BASE),
	REG(CP_IB2_BASE_HI, CP_IB2_BASE_HI),
	REG(CP_IB2_BUFSZ, CP_IB2_BUFSZ),
	REG(CP_ROQ_ADDR, CP_ROQ_DBG_ADDR),
	REG(CP_ROQ_DATA, CP_ROQ_DBG_DATA),
	REG(CP_MERCIU_ADDR, CP_MERCIU_DBG_ADDR),
	REG(CP_MERCIU_DATA, CP_MERCIU_DBG_DATA_1),
	REG(CP_MERCIU_DATA2, CP_MERCIU_DBG_DATA_2),
	REG(CP_MEQ_ADDR, CP_MEQ_DBG_ADDR),
	REG(CP_MEQ_DATA, CP_MEQ_DBG_DATA),
//	REG(CP_PROTECT_REG_0, CP_PROTECT_REG_0),
//	REG(CP_PREEMPT, CP_CONTEXT_SWITCH_CNTL),
//	REG(CP_CONTEXT_SWITCH_SMMU_INFO_LO, CP_CONTEXT_SWITCH_SMMU_INFO_LO),
//	REG(CP_CONTEXT_SWITCH_SMMU_INFO_HI, CP_CONTEXT_SWITCH_SMMU_INFO_HI),
	REG(RBBM_STATUS, RBBM_STATUS),
//	REG(RBBM_STATUS3, RBBM_STATUS3),
	REG(RBBM_PERFCTR_CTL, RBBM_PERFCTR_CNTL),
	REG(RBBM_PERFCTR_LOAD_CMD0, RBBM_PERFCTR_LOAD_CMD0),
	REG(RBBM_PERFCTR_LOAD_CMD1, RBBM_PERFCTR_LOAD_CMD1),
	REG(RBBM_PERFCTR_LOAD_CMD2, RBBM_PERFCTR_LOAD_CMD2),
	REG(RBBM_PERFCTR_LOAD_CMD3, RBBM_PERFCTR_LOAD_CMD3),
	REG(RBBM_INT_0_MASK, RBBM_INT_0_MASK),
	REG(RBBM_INT_0_STATUS, RBBM_INT_0_STATUS),
	REG(RBBM_CLOCK_CTL, RBBM_CLOCK_CNTL),
	REG(RBBM_INT_CLEAR_CMD, RBBM_INT_CLEAR_CMD),
	REG(RBBM_SW_RESET_CMD, RBBM_SW_RESET_CMD),
//	REG(RBBM_BLOCK_SW_RESET_CMD, RBBM_BLOCK_SW_RESET_CMD),
//	REG(RBBM_BLOCK_SW_RESET_CMD2, RBBM_BLOCK_SW_RESET_CMD2),
	REG(UCHE_INVALIDATE0, UCHE_CACHE_INVALIDATE),
	REG(RBBM_PERFCTR_LOAD_VALUE_LO, RBBM_PERFCTR_LOAD_VALUE_LO),
	REG(RBBM_PERFCTR_LOAD_VALUE_HI, RBBM_PERFCTR_LOAD_VALUE_HI),
//	REG(RBBM_SECVID_TRUST_CONTROL, RBBM_SECVID_TRUST_CNTL),
//	REG(RBBM_SECVID_TRUST_CONFIG, RBBM_SECVID_TRUST_CONFIG),
//	REG(RBBM_SECVID_TSB_CONTROL, RBBM_SECVID_TSB_CNTL),
//	REG(RBBM_SECVID_TSB_TRUSTED_BASE, RBBM_SECVID_TSB_TRUSTED_BASE_LO),
//	REG(RBBM_SECVID_TSB_TRUSTED_BASE_HI, RBBM_SECVID_TSB_TRUSTED_BASE_HI),
//	REG(RBBM_SECVID_TSB_TRUSTED_SIZE, RBBM_SECVID_TSB_TRUSTED_SIZE),
//	REG(RBBM_ALWAYSON_COUNTER_LO, RBBM_ALWAYSON_COUNTER_LO),
//	REG(RBBM_ALWAYSON_COUNTER_HI, RBBM_ALWAYSON_COUNTER_HI),
//	REG(VBIF_XIN_HALT_CTRL0, VBIF_XIN_HALT_CTRL0),
//	REG(VBIF_XIN_HALT_CTRL1, VBIF_XIN_HALT_CTRL1),
//	REG(VBIF_VERSION, VBIF_VERSION),
};

static void a5xx_dump(struct msm_gpu *gpu)
{
	printk("status:   %08x\n", gpu_read(gpu, REG_A5XX_RBBM_STATUS));
	adreno_dump(gpu);
}

static int a5xx_pm_resume(struct msm_gpu *gpu)
{
	int ret;

	ret = msm_gpu_pm_resume(gpu);
	if (ret)
		return ret;

	/*
	 * Turn on smaller power domain first to reduce voltage droop.
	 * Set the default register values; set SW_COLLAPSE to 0.
	 */
	gpu_write(gpu, REG_A5XX_GPMU_RBCCU_POWER_CNTL, 0x778000);

	/* Insert a delay between RAC and SPTP GDSC to reduce voltage droop */
	udelay(3);

	ret = spin_until(gpu_read(gpu, REG_A5XX_GPMU_RBCCU_PWR_CLK_STATUS) &
			A5XX_GPMU_RBCCU_PWR_CLK_STATUS_PWR_ON);
	if (ret) {
		dev_err(gpu->dev->dev, "RBCCU GDSC enable failed\n");
		return ret;
	}

	gpu_write(gpu, REG_A5XX_GPMU_SP_POWER_CNTL, 0x778000);

	ret = spin_until(gpu_read(gpu, REG_A5XX_GPMU_SP_PWR_CLK_STATUS) &
			A5XX_GPMU_SP_PWR_CLK_STATUS_PWR_ON);
	if (ret) {
		dev_err(gpu->dev->dev, "SPTP GDSC enable failed\n");
		return ret;
	}

	return 0;
}

static int a5xx_pm_suspend(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	int ret;

	ret = msm_gpu_pm_suspend(gpu);
	if (ret)
		return ret;

	/*
	 * NOTE: if GPMU enabled, then GPMU should be doing this:
	 */

	/* Set the default register values; set SW_COLLAPSE to 1 */
	gpu_write(gpu, REG_A5XX_GPMU_SP_POWER_CNTL, 0x778001);

	/* Insert a delay between RAC and SPTP GDSC to reduce voltage droop */
	udelay(3);

	ret = spin_until(!(gpu_read(gpu, REG_A5XX_GPMU_SP_PWR_CLK_STATUS) &
			A5XX_GPMU_SP_PWR_CLK_STATUS_PWR_ON));
	if (ret)
		dev_err(gpu->dev->dev, "SPTP GDSC disable failed\n");

	gpu_write(gpu, REG_A5XX_GPMU_RBCCU_POWER_CNTL, 0x778001);

	ret = spin_until(!(gpu_read(gpu, REG_A5XX_GPMU_RBCCU_PWR_CLK_STATUS) &
			A5XX_GPMU_RBCCU_PWR_CLK_STATUS_PWR_ON));
	if (ret)
		dev_err(gpu->dev->dev, "RBCCU GDSC disable failed\n");

	if (adreno_is_a530(adreno_gpu)) {
		/* Reset VBIF before PC to avoid popping bogus FIFO entries */
		gpu_write(gpu, REG_A5XX_RBBM_BLOCK_SW_RESET_CMD, 0x003c0000);
		gpu_write(gpu, REG_A5XX_RBBM_BLOCK_SW_RESET_CMD, 0);
	}

	return 0;
}

static int a5xx_get_timestamp(struct msm_gpu *gpu, uint64_t *value)
{
	uint32_t hi, lo, tmp;

	tmp = gpu_read(gpu, REG_A5XX_RBBM_PERFCTR_CP_0_HI);
	do {
		hi = tmp;
		lo = gpu_read(gpu, REG_A5XX_RBBM_PERFCTR_CP_0_LO);
		tmp = gpu_read(gpu, REG_A5XX_RBBM_PERFCTR_CP_0_HI);
	} while (tmp != hi);

	*value = (((uint64_t)hi) << 32) | lo;

	return 0;
}

static const struct adreno_gpu_funcs funcs = {
	.base = {
		.get_param = adreno_get_param,
		.hw_init = a5xx_hw_init,
		.pm_suspend = a5xx_pm_suspend,
		.pm_resume = a5xx_pm_resume,
		.recover = a5xx_recover,
		.last_fence = adreno_last_fence,
		.submit = adreno_submit,
		.flush = adreno_flush,
		.idle = a5xx_idle,
		.irq = a5xx_irq,
		.destroy = a5xx_destroy,
#ifdef CONFIG_DEBUG_FS
		.show = a5xx_show,
#endif
	},
	.get_timestamp = a5xx_get_timestamp,
};

struct msm_gpu *a5xx_gpu_init(struct drm_device *dev)
{
	struct a5xx_gpu *a5xx_gpu = NULL;
	struct adreno_gpu *adreno_gpu;
	struct msm_gpu *gpu;
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = priv->gpu_pdev;
	int ret;

	if (!pdev) {
		dev_err(dev->dev, "no a5xx device\n");
		ret = -ENXIO;
		goto fail;
	}

	a5xx_gpu = kzalloc(sizeof(*a5xx_gpu), GFP_KERNEL);
	if (!a5xx_gpu) {
		ret = -ENOMEM;
		goto fail;
	}

	adreno_gpu = &a5xx_gpu->base;
	gpu = &adreno_gpu->base;

	a5xx_gpu->pdev = pdev;

	gpu->perfcntrs = NULL;
	gpu->num_perfcntrs = 0;

	adreno_gpu->registers = a5xx_registers;
	adreno_gpu->reg_offsets = a5xx_register_offsets;

	ret = adreno_gpu_init(dev, pdev, adreno_gpu, &funcs);
	if (ret)
		goto fail;

	if (!gpu->mmu) {
		dev_err(dev->dev, "No memory protection without IOMMU\n");
		ret = -ENXIO;
		goto fail;
	}

	mutex_lock(&dev->struct_mutex);

	a5xx_gpu->pfp_bo = msm_gem_new(dev, adreno_gpu->pfp->size,
			MSM_BO_UNCACHED);
	if (IS_ERR(a5xx_gpu->pfp_bo)) {
		ret = PTR_ERR(a5xx_gpu->pfp_bo);
		a5xx_gpu->pfp_bo = NULL;
		dev_err(dev->dev, "could not allocate PFP buffer: %d\n", ret);
		goto fail_unlock;
	}

	ret = fw_upload(a5xx_gpu->pfp_bo, adreno_gpu->pfp);
	if (ret)
		goto fail_unlock;

	ret = msm_gem_get_iova_locked(a5xx_gpu->pfp_bo, gpu->id,
			&a5xx_gpu->pfp_iova);
	if (ret) {
		dev_err(dev->dev, "could not map PFP buffer: %d\n", ret);
		goto fail_unlock;
	}

	a5xx_gpu->pm4_bo = msm_gem_new(dev, adreno_gpu->pm4->size,
			MSM_BO_UNCACHED);
	if (IS_ERR(a5xx_gpu->pm4_bo)) {
		ret = PTR_ERR(a5xx_gpu->pm4_bo);
		a5xx_gpu->pm4_bo = NULL;
		dev_err(dev->dev, "could not allocate PM4 buffer: %d\n", ret);
		goto fail_unlock;
	}

	ret = fw_upload(a5xx_gpu->pm4_bo, adreno_gpu->pm4);
	if (ret)
		goto fail_unlock;

	ret = msm_gem_get_iova_locked(a5xx_gpu->pm4_bo, gpu->id,
			&a5xx_gpu->pm4_iova);
	if (ret) {
		dev_err(dev->dev, "could not map PM4 buffer: %d\n", ret);
		goto fail_unlock;
	}

	mutex_unlock(&dev->struct_mutex);

	return gpu;

fail_unlock:
	mutex_unlock(&dev->struct_mutex);
fail:
	if (a5xx_gpu)
		a5xx_destroy(&a5xx_gpu->base.base);

	return ERR_PTR(ret);
}
